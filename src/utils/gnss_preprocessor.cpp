// SPDX-License-Identifier: MIT
#include "gnss_ros_standardization/gnss_preprocessor.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/SVD>  // JacobiSVD: the Doppler solve, and its condition number
#include <Eigen/LU>  // Matrix4d::inverse(); avoids Geometry/Eigenvalues which
                     // clash with RTKLIB's trace() macro (see gnss_preprocessor.hpp)

#include "gnss_ros_standardization/gnss_utils.hpp"

namespace gnss_utils {

namespace {

// Cross-code DD safety: forming a DD across different rover/base tracking codes
// on the same band is carrier-integer-safe when there is no half-cycle ambiguity
// between the codes. Galileo E-band components (pilot Q / data B / combined X)
// are full-cycle coherent, and the L5/E5a band (band index 2) mixes only pilot
// Q vs combined X. GPS/QZS L2 is excluded because 2W (codeless P(Y)) carries a
// possible half cycle relative to L2C. The inter-code bias itself cancels in the
// DD (see Config::cross_code_carrier), so PR and CP are both admissible here.
inline bool crossCodeSafe(int sys, int band) {
  if (sys == SYS_GAL) return true;                    // E1 / E5a / E5b coherent
  if (band == 2 && (sys == SYS_GPS || sys == SYS_QZS)) return true;  // L5 Q vs X
  return false;
}

}  // namespace

// Largest tolerated condition number of the whitened Doppler design matrix.
// Above it the four unknowns (3 velocity + clock drift) are not separable by
// this satellite geometry, and the reported covariance - however finite - is
// describing a direction the data does not constrain. 1e6 on singular values
// corresponds to ~1e12 on the normal matrix the previous code inverted.
constexpr double kDopplerMaxCondition = 1.0e6;

// Undifferenced (zero-difference) Doppler velocity, RTKLIB estvel/resdop-style:
// weighted least squares for receiver velocity (3) + receiver clock drift (1).
// The range-rate model is linear in the unknowns, so ONE solve is exact for a
// given row set; the loop below only re-solves after removing a faulty row.
//   residual = -lambda*D - [ e.(v_sat - v_rcv) + sagnac_rate
//                            + c*drift_rcv - c*drift_sat ]
// with e the rover->satellite unit vector. No differencing is needed: the
// differential error terms double-differencing removes for *position*
// (iono/tropo, clocks) have negligible time-derivatives, so for velocity only
// the receiver clock drift matters and it is estimated as the 4th unknown.
bool estimateDopplerVelocity(const std::vector<SatObs>& sats,
                             const Eigen::Vector3d& rr, double sigma_mps,
                             double max_nsigma, int max_exclude,
                             DopplerVelocitySolution& out) {
  const double omge_c = OMGE / CLIGHT;
  std::vector<int> rows;
  std::set<int> seen;
  for (int i = 0; i < static_cast<int>(sats.size()); ++i) {
    const SatObs& s = sats[i];
    if (s.doppler == 0.0 || s.lam <= 0.0 || s.sat_vel.norm() <= 0.0) continue;
    if (!seen.insert(s.sat).second) continue;  // one Doppler per satellite
    rows.push_back(i);
  }
  // FIVE, not four. Four rows against four unknowns leaves zero redundancy: the
  // fit passes exactly through every observation, so the residuals are
  // identically zero and out.res - the caller's gross-outlier gate - reads a
  // perfect 0 no matter how bad a range rate is. A blunder is then not merely
  // undetectable, it actively looks ideal. Five rows give one degree of freedom,
  // which is enough to notice a fault even though it is not enough to isolate
  // one.
  if (static_cast<int>(rows.size()) < 5) return false;

  // One weighted least-squares pass over the current row set. The range-rate
  // model is linear in (v_rcv, drift), so a single solve is exact.
  //
  // The weights come from an A-PRIORI variance, R_kk = sigma^2 / sin^2(el),
  // rather than a relative elevation weight. That matters twice over:
  //   - N^-1 then carries ABSOLUTE units, so the reported covariance means
  //     something even on a clean epoch (a purely relative weighting leaves the
  //     scale to be recovered from the residuals, which collapses to zero when
  //     the residuals happen to be small);
  //   - the blunder test below can be a Baarda w-test against a fixed sigma
  //     instead of against the estimated one. That distinction is the whole
  //     difference between detecting a single NLOS range rate and not: one
  //     large blunder inflates the a-posteriori sigma_0 enough to normalize
  //     ITSELF back under any threshold (classic masking). solveCodeDdWls in
  //     this codebase makes the same choice for the same reason.
  struct Solve {
    Eigen::Vector4d x{Eigen::Vector4d::Zero()};
    Eigen::Matrix4d Ninv{Eigen::Matrix4d::Zero()};  // [(m/s)^2] absolute
    Eigen::VectorXd r;      // post-fit residuals [m/s]
    Eigen::VectorXd r_var;  // a-priori variance of each residual [(m/s)^2]
    double sigma0_sq{1.0};  // reduced chi-square (dimensionless)
    double cond{0.0};       // condition number of the whitened design matrix
    bool ok{false};
  };
  const double sigma_sq = std::max(sigma_mps, 1e-3) * std::max(sigma_mps, 1e-3);
  auto solve = [&](const std::vector<int>& use) {
    Solve s;
    const int nv = static_cast<int>(use.size());
    Eigen::MatrixXd H(nv, 4);
    Eigen::VectorXd y(nv), w(nv), rvar(nv);
    for (int k = 0; k < nv; ++k) {
      const SatObs& o = sats[use[k]];
      const Eigen::Vector3d e = (o.sat_pos - rr).normalized();
      // Model at v_rcv = 0, drift = 0 (the v_rcv/drift dependence is in H).
      const double sagnac =
          omge_c * (o.sat_vel.y() * rr.x() - o.sat_vel.x() * rr.y());
      const double model0 =
          e.dot(o.sat_vel) + sagnac - CLIGHT * o.sat_clk_drift;
      y(k) = -o.lam * o.doppler - model0;
      H(k, 0) = -e.x() + omge_c * o.sat_pos.y();
      H(k, 1) = -e.y() - omge_c * o.sat_pos.x();
      H(k, 2) = -e.z();
      H(k, 3) = 1.0;
      const double sel = std::sin(std::max(o.el, 0.05));
      rvar(k) = sigma_sq / (sel * sel);  // a-priori [(m/s)^2]
      w(k) = 1.0 / rvar(k);
    }
    // Solve the WHITENED system by SVD rather than forming and inverting the
    // normal equations. N = H'WH squares the condition number of H, and
    // N.inverse() reports nothing when the geometry is nearly rank-deficient -
    // it returns a huge but perfectly finite matrix, so allFinite() waves it
    // through. The singular values of the whitened H are the direct measure of
    // that geometry, and they are what the rank/condition gate below uses.
    const Eigen::VectorXd sw = w.cwiseSqrt();
    const Eigen::MatrixXd Hw = sw.asDiagonal() * H;
    const Eigen::VectorXd yw = sw.asDiagonal() * y;
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
        Hw, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd sv = svd.singularValues();
    if (sv.size() < 4 || !(sv(0) > 0.0)) return s;
    s.cond = sv(0) / std::max(sv(sv.size() - 1), 1e-300);
    if (!std::isfinite(s.cond) || s.cond > kDopplerMaxCondition) return s;
    s.x = svd.solve(yw);
    if (!s.x.allFinite()) return s;
    // (H'WH)^-1 = V diag(1/sv^2) V', from the same decomposition.
    const Eigen::Vector4d inv_sv2 = sv.array().square().inverse();
    s.Ninv = svd.matrixV() * inv_sv2.asDiagonal() * svd.matrixV().transpose();
    if (!s.Ninv.allFinite()) return s;
    s.r = y - H * s.x;
    const double dof = static_cast<double>(nv - 4);
    s.sigma0_sq = dof > 0.0 ? (s.r.dot(w.asDiagonal() * s.r)) / dof : 1.0;
    // Var(residual) = R - H N^-1 H'. Subtractive, and for the same reason the
    // post-fit FIX validation is: the fit has already been pulled towards each
    // observation, so its residual is smaller than its measurement noise.
    // Dividing by R instead would understate every z and miss blunders on
    // exactly the high-leverage rows that do the most damage.
    s.r_var.resize(nv);
    for (int k = 0; k < nv; ++k)
      s.r_var(k) = rvar(k) - H.row(k) * s.Ninv * H.row(k).transpose();
    s.ok = true;
    return s;
  };

  Solve best = solve(rows);
  if (!best.ok) return false;

  // Baarda w-test, one row at a time, RE-TESTED after every removal.
  //
  // The loop below exits in one of three states and the caller must be able to
  // tell them apart, which the previous version could not: it stopped at five
  // rows without re-testing, so "excluded a satellite but the fault is still
  // there" was indistinguishable from "clean" and was reported as a good
  // velocity. The three states are now:
  //   - converged: the worst standardized residual is under the threshold ->
  //     accept.
  //   - out of redundancy (down to 5 rows, one degree of freedom) with a fault
  //     still flagged -> reject the EPOCH. Trimming past five manufactures
  //     confidence rather than earning it, so the answer is not a smaller set,
  //     it is no velocity at all.
  //   - out of budget with a fault still flagged -> reject the EPOCH, same
  //     reasoning.
  int n_dropped = 0;
  bool fault_remains = false;
  while (max_nsigma > 0.0) {
    // Find the worst row under the CURRENT solution, every iteration.
    int w_worst = -1;
    double w_worst_z = 0.0;
    for (int k = 0; k < static_cast<int>(rows.size()); ++k) {
      if (best.r_var(k) <= 1e-12) continue;  // no redundancy: not testable
      const double z = std::fabs(best.r(k)) / std::sqrt(best.r_var(k));
      if (z > w_worst_z) {
        w_worst_z = z;
        w_worst = k;
      }
    }
    if (w_worst < 0 || w_worst_z <= max_nsigma) {
      fault_remains = false;
      break;
    }
    // A fault is flagged. Can we still act on it?
    if (n_dropped >= max_exclude || static_cast<int>(rows.size()) <= 5) {
      fault_remains = true;
      break;
    }
    std::vector<int> trimmed;
    trimmed.reserve(rows.size() - 1);
    for (int k = 0; k < static_cast<int>(rows.size()); ++k)
      if (k != w_worst) trimmed.push_back(rows[k]);
    const Solve retry = solve(trimmed);
    if (!retry.ok) {  // the trimmed geometry is unusable; do not pretend
      fault_remains = true;
      break;
    }
    rows.swap(trimmed);
    best = retry;
    ++n_dropped;
  }
  if (fault_remains) return false;

  const int nv = static_cast<int>(rows.size());
  out.vel = best.x.head<3>();
  // Inflate on bad residuals, never deflate below the a-priori. A variance
  // factor below 1 says the observations agreed better than the noise model
  // predicted, which on one epoch of 4-12 satellites is luck, not evidence -
  // and reporting a covariance scaled by it hands a consumer a confidence the
  // measurement has not earned.
  out.cov = std::max(best.sigma0_sq, 1.0) * best.Ninv.topLeftCorner<3, 3>();
  // Unweighted post-fit residual RMS [m/s]: a gross-outlier gate for the caller
  // (a mismodeled / NLOS Doppler set inflates this well past the ~sub-m/s noise).
  out.res = std::sqrt(best.r.dot(best.r) / nv);
  out.nsat = nv;
  out.excluded = n_dropped;
  return out.cov.allFinite() && out.cov.diagonal().minCoeff() >= 0.0 &&
         std::isfinite(out.res);
}

GnssPreprocessor::GnssPreprocessor(const Config& config) : config_(config) {
  if (config_.bands.empty())
    throw std::invalid_argument("bands must not be empty");
  {
    std::vector<int> sorted = config_.bands;
    std::sort(sorted.begin(), sorted.end());
    if (std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end())
      throw std::invalid_argument("bands must be unique");
  }

  freq_mask_ = FrequencyMask{false, false, false};
  int max_band = 0;
  for (const int b : config_.bands) {
    if (b == 0) freq_mask_.l1 = true;
    if (b == 1) freq_mask_.l2 = true;
    if (b == 2) freq_mask_.l5 = true;
    max_band = std::max(max_band, b);
  }

  spp_opt_ = prcopt_default;
  spp_opt_.mode = PMODE_SINGLE;
  spp_opt_.navsys = config_.navsys | config_.navsys_undifferenced_only;
  spp_opt_.elmin = config_.el_mask_rad;
  spp_opt_.nf = std::min(max_band + 1, NFREQ);
  spp_opt_.ionoopt = IONOOPT_BRDC;
  spp_opt_.tropopt = TROPOPT_SAAS;
  spp_opt_.sateph = EPHOPT_BRDC;

  RoverBaseEpochMatcher::Options mopt;
  // The matcher must not pair beyond what the DD stage can use; see max_age_s.
  mopt.max_tdiff_s = config_.max_age_s;
  mopt.match_tol_s = config_.match_tol_s;
  mopt.duplicate_tol_s = config_.matcher_duplicate_tol_s;
  mopt.reorder_window_s = config_.matcher_reorder_window_s;
  mopt.drop_ahead_s = config_.matcher_decision_horizon_s;
  mopt.queue_limit = config_.matcher_queue_limit;
  // Never erase the rover timeline just because a DD cannot be formed: a
  // base outage still yields a rover-only epoch (has_base=false) so a
  // consumer can dead-reckon / keep its state continuous.
  mopt.emit_unmatched_rover = true;
  matcher_ = RoverBaseEpochMatcher(mopt);  // validates duplicate_tol <= match_tol

  std::memset(&nav_, 0, sizeof(nav_));
}

GnssPreprocessor::~GnssPreprocessor() { freenav(&nav_, 0xFF); }

void GnssPreprocessor::pushEphemerides(const EphMsg& msg) {
  for (const auto& e : msg.gnss_ephemeris) {
    if (!e.satid.empty()) eph_store_.ingestEph(msgToEph(e));
  }
  for (const auto& g : msg.glonass_ephemeris) {
    if (!g.satid.empty()) eph_store_.ingestGeph(msgToGeph(g));
  }
  eph_store_.applyToNav(nav_);
}

void GnssPreprocessor::pushRoverObs(ObsMsg::ConstSharedPtr msg) {
  matcher_.pushRover(std::move(msg));
}

void GnssPreprocessor::pushBaseObs(ObsMsg::ConstSharedPtr msg) {
  matcher_.pushBase(std::move(msg));
}

std::vector<PreprocessedEpoch> GnssPreprocessor::drainEpochs(
    const Eigen::Vector3d* approx_rover) {
  std::vector<PreprocessedEpoch> out;
  if (!hasEphemeris()) return out;
  auto pairs = matcher_.drainMatches();
  if (flush_pending_) {  // an EOF flush arrived before ephemeris; run it now
    flush_pending_ = false;
    auto tail = matcher_.flush();
    pairs.insert(pairs.end(), std::make_move_iterator(tail.begin()),
                 std::make_move_iterator(tail.end()));
  }
  buildEpochs(pairs, approx_rover, out);
  return out;
}

std::vector<PreprocessedEpoch> GnssPreprocessor::flushEpochs(
    const Eigen::Vector3d* approx_rover) {
  std::vector<PreprocessedEpoch> out;
  if (!hasEphemeris()) {  // defer: never erase observations before nav arrives
    flush_pending_ = true;
    return out;
  }
  buildEpochs(matcher_.flush(), approx_rover, out);
  return out;
}

void GnssPreprocessor::buildEpochs(
    const std::vector<RoverBaseEpochMatcher::Pair>& pairs,
    const Eigen::Vector3d* approx_rover,
    std::vector<PreprocessedEpoch>& out) {
  for (const auto& pair : pairs) {
    PreprocessedEpoch ep;
    const ObsMsg* base = pair.hasBase() ? pair.base.get() : nullptr;
    if (buildEpoch(*pair.rover, base, pair.age_s, approx_rover, ep)) {
      out.push_back(std::move(ep));
    }
  }
}

bool GnssPreprocessor::buildEpoch(const ObsMsg& rover, const ObsMsg* base,
                                  double age_s,
                                  const Eigen::Vector3d* approx_rover,
                                  PreprocessedEpoch& out) {
  // One obsd_t per (receiver, satellite): rover (rcv=1) entries first, then
  // base (rcv=2), each stamped with its own receive time. base is null during
  // a base outage (rover-only epoch).
  std::vector<obsd_t> obs = buildObsEpoch(rover, base, freq_mask_);
  if (obs.empty()) { ++epoch_reject_.no_observations; return false; }
  const int n = static_cast<int>(obs.size());
  int n_rover = 0;
  while (n_rover < n && obs[n_rover].rcv == 1) ++n_rover;
  if (n_rover == 0) { ++epoch_reject_.no_rover_entries; return false; }

  // Satellite position/clock at signal transmission time for every entry.
  // satposs() derives the transmission time per entry from its pseudorange
  // and receive time, so rover and base epochs get their own satellite states
  // in this single call. svh is -1 where that was impossible (no pseudorange
  // or no ephemeris).
  std::vector<double> rs(6 * n), dts(2 * n), var(n);
  std::vector<int> svh(n);
  const gtime_t t_rover = gpst2time(static_cast<int>(rover.week), rover.tow);
  satposs(t_rover, obs.data(), n, &nav_, EPHOPT_BRDC,
          rs.data(), dts.data(), var.data(), svh.data());

  // Rover a-priori for elevation masks and as the downstream initial value.
  // Priority: caller-provided approximation (e.g. the optimizer's last
  // estimate) > standalone SPP fix > previous epoch's a-priori > base station
  // position. The base fallback covers the first epoch / sparse sky where SPP
  // cannot yet converge: in RTK the rover lies within the baseline of the
  // base, so it is more than close enough for masks and as a linearization
  // point (the optimizer refines the position regardless).
  //
  // The SPP solve is ALSO published separately as out.rover_ecef_spp. Running
  // it only as an a-priori fallback (the original behaviour) meant a caller
  // that supplied its own estimate never got a position it had not produced
  // itself, so nothing downstream could cross-check that estimate against the
  // observations. Config::compute_spp forces the solve regardless of the hint;
  // which position feeds the masks is unchanged.
  Eigen::Vector3d spp_pos = Eigen::Vector3d::Zero();
  Eigen::Matrix3d spp_cov = Eigen::Matrix3d::Zero();
  bool spp_valid = false;
  int spp_nsat = 0;

  Eigen::Vector3d rr;
  bool have_rr = false;
  if (approx_rover) {
    rr = *approx_rover;
    have_rr = true;
  }
  if (config_.compute_spp || !have_rr) {
    sol_t sol{};
    std::vector<double> azel(2 * n_rover, 0.0);
    std::vector<ssat_t> ssat(MAXSAT);
    char msg[128] = "";
    if (pntpos(obs.data(), n_rover, &nav_, &spp_opt_, &sol, azel.data(),
               ssat.data(), msg)) {
      spp_pos = Eigen::Vector3d(sol.rr[0], sol.rr[1], sol.rr[2]);
      spp_valid = spp_pos.allFinite() && spp_pos.norm() > 1.0;
      // RTKLIB packs the ECEF covariance as {xx, yy, zz, xy, yz, zx}.
      spp_cov(0, 0) = sol.qr[0];
      spp_cov(1, 1) = sol.qr[1];
      spp_cov(2, 2) = sol.qr[2];
      spp_cov(0, 1) = spp_cov(1, 0) = sol.qr[3];
      spp_cov(1, 2) = spp_cov(2, 1) = sol.qr[4];
      spp_cov(2, 0) = spp_cov(0, 2) = sol.qr[5];
      spp_nsat = sol.ns;
      if (!have_rr && spp_valid) {
        rr = spp_pos;
        have_rr = true;
      }
    }
  }
  if (!have_rr && last_apriori_valid_) {
    rr = last_apriori_;
    have_rr = true;
  }
  if (!have_rr && config_.base_ecef.norm() > 0.0) {
    rr = config_.base_ecef;
    have_rr = true;
  }
  if (!have_rr) { ++epoch_reject_.no_apriori; return false; }
  last_apriori_ = rr;
  last_apriori_valid_ = true;

  const double rr_a[3] = {rr.x(), rr.y(), rr.z()};
  double llh[3];
  ecef2pos(rr_a, llh);

  out.week = rover.week;
  out.tow = rover.tow;
  if (base) {
    out.has_base = true;
    out.base_week = base->week;
    out.base_tow = base->tow;
    out.age_s = age_s;
  } else {
    out.has_base = false;
    out.base_tow = std::numeric_limits<double>::quiet_NaN();
    out.age_s = std::numeric_limits<double>::quiet_NaN();
  }
  out.stamp = rover.header.stamp;
  out.base_ecef = config_.base_ecef;
  out.rover_ecef_apriori = rr;
  out.rover_ecef_spp = spp_pos;
  out.rover_spp_valid = spp_valid;
  out.rover_spp_cov = spp_cov;
  out.rover_spp_nsat = spp_nsat;

  // Undifferenced rover satellites passing health/system/elevation/SNR masks.
  for (int i = 0; i < n_rover; ++i) {
    const obsd_t& o = obs[i];
    int prn = 0;
    const int sys = satsys(o.sat, &prn);
    // Undifferenced admission: navsys plus any system allowed for
    // undifferenced use only (see Config::navsys_undifferenced_only).
    if (!(sys & (config_.navsys | config_.navsys_undifferenced_only))) continue;
    if (svh[i] != 0) continue;
    const double* rs_i = &rs[6 * i];
    if (norm(rs_i, 3) <= 0.0) continue;

    double e[3], azel_i[2];
    geodist(rs_i, rr_a, e);
    satazel(llh, e, azel_i);
    if (azel_i[1] < config_.el_mask_rad) continue;

    char id[8] = "";
    satno2id(o.sat, id);

    for (const int b : config_.bands) {
      if (b < 0 || b >= NFREQ) continue;
      if (o.P[b] == 0.0 && o.L[b] == 0.0) continue;
      // SNR mask: elevation-dependent (RTKLIB testsnr) when enabled, otherwise
      // the flat snr_mask_dbhz threshold. An SNR of 0 means UNKNOWN; when a mask
      // is active, exclude it (RTKLIB testsnr treats 0 as below threshold), so a
      // quality-unknown observation is never used only by this node.
      const bool rover_mask =
          config_.snr_mask.ena[0] || config_.snr_mask_dbhz > 0.0;
      if (rover_mask && o.SNR[b] <= 0.0) continue;
      if (o.SNR[b] > 0.0) {
        if (config_.snr_mask.ena[0]) {
          if (testsnr(0, b, azel_i[1], o.SNR[b], &config_.snr_mask)) continue;
        } else if (config_.snr_mask_dbhz > 0.0 &&
                   o.SNR[b] < config_.snr_mask_dbhz) {
          continue;
        }
      }
      const double freq = sat2freq(o.sat, o.code[b], &nav_);
      if (freq <= 0.0) continue;

      SatObs s;
      s.sat = o.sat;
      s.satid = id;
      s.sys = sys;
      s.band = b;
      s.code = o.code[b];
      s.sat_pos = Eigen::Vector3d(rs_i[0], rs_i[1], rs_i[2]);
      s.sat_clk = dts[2 * i];
      s.sat_vel = Eigen::Vector3d(rs_i[3], rs_i[4], rs_i[5]);
      s.sat_clk_drift = dts[2 * i + 1];
      s.lam = CLIGHT / freq;
      s.pr = o.P[b];
      s.cp_m = (o.L[b] != 0.0) ? o.L[b] * s.lam : 0.0;
      s.doppler = o.D[b];
      s.snr = o.SNR[b];
      s.el = azel_i[1];
      s.az = azel_i[0];
      // 0x1 = loss-of-lock / cycle slip (continuity break -> re-key carried amb);
      // 0x2 = half-cycle-ambiguity present (a half-integer -> exclude from integer
      // AR, but DO NOT re-key every epoch). detectSlips adds the rover-base
      // checks and the 0x2 *transition* (which IS a continuity slip).
      s.slip = (o.LLI[b] & 0x1) != 0;
      s.half_cycle = (o.LLI[b] & 0x2) != 0;
      out.rover_sats.push_back(std::move(s));
    }
  }
  // No usable rover satellite after the system / elevation / SNR masks. EMIT
  // the epoch anyway, empty.
  //
  // This used to `return false`, which deleted the epoch from the timeline
  // entirely - and that is a different statement from the one the data
  // supports. What is missing is the GNSS MEASUREMENT; the epoch still has a
  // valid GPST, a valid a-priori and a valid stamp. A consumer with an inertial
  // sensor can carry the state across it, which is the entire reason such a
  // consumer exists, but only if it is told the epoch happened.
  //
  // Measured consequence of the old behaviour, on PPC nagoya_run1 (400 s
  // window): 24 rover epochs vanished with nothing logged anywhere, the
  // tightly-coupled node published no solution across a real sky outage
  // despite being able to preintegrate through it, and the resulting hole
  // then read downstream as a GNSS outage that re-keyed every carried carrier
  // arc.
  //
  // Same principle as the matcher's emit_unmatched_rover (see the constructor):
  // never erase the rover timeline just because this epoch cannot contribute a
  // measurement. Consumers that need observations already test rover_sats /
  // dd for emptiness.
  if (out.rover_sats.empty()) {
    ++epoch_reject_.emitted_without_satellites;
    return true;  // rover-only, measurement-free epoch: dd and rover_sats empty
  }

  // Rover-base cycle-slip detection (sets slip / cp_excluded on out.rover_sats).
  {
    std::map<int, int> rover_idx, base_idx;
    for (int i = 0; i < n_rover; ++i) rover_idx.emplace(obs[i].sat, i);
    for (int i = n_rover; i < n; ++i) base_idx.emplace(obs[i].sat, i);
    // Continuous GPST seconds (not tow alone) so time differences stay positive
    // across the GPS week rollover (tow wraps 604800 -> 0).
    detectSlips(obs, rover_idx, base_idx, out.week * 604800.0 + out.tow, out);
  }

  // Undifferenced Doppler rover velocity (loosely-coupled motion aiding for the
  // FGO; harmless to others - just an extra field).
  {
    DopplerVelocitySolution dv;
    if (estimateDopplerVelocity(out.rover_sats, rr, config_.doppler_sigma_mps,
                                config_.doppler_max_nsigma,
                                config_.doppler_max_exclude, dv)) {
      out.rover_vel_ecef = dv.vel;
      out.rover_vel_cov = dv.cov;
      // .diagonal().sum(), not .trace(): rtklib.h defines a `trace` macro.
      out.rover_vel_var = dv.cov.diagonal().sum() / 3.0;
      out.rover_vel_res = dv.res;
      out.rover_vel_nsat = dv.nsat;
      out.rover_vel_excluded = dv.excluded;
      out.rover_vel_valid = true;
    }
  }

  // Double differences need a usable base epoch and a known base position.
  if (!base) return true;                       // rover-only epoch (base outage)
  if (age_s > config_.max_age_s) return true;
  if (config_.base_ecef.norm() <= 0.0) return true;

  std::map<int, int> base_idx;  // sat -> index into obs[]
  for (int i = n_rover; i < n; ++i) base_idx[obs[i].sat] = i;

  // Pair rover satellites with base entries per (sys, band). Indices refer to
  // out.rover_sats, which is complete (and stable) by now.
  struct PairCand {
    int rov;      // index into out.rover_sats
    int base_i;   // index into obs[] / rs[]
    uint8_t code_base;
    bool pr_ok;
    bool cp_ok;
  };
  // Group by both receivers' observed codes. With cross-code pairing enabled,
  // the rover and base codes may differ from each other, but each must be common
  // to the group's reference and target. Grouping only by the rover code lets a
  // base receiver switch code between satellites, leaving an inter-code bias in
  // the DD and destroying its integer property.
  std::map<std::tuple<int, int, int, int>, std::vector<PairCand>> groups;

  for (int ri = 0; ri < static_cast<int>(out.rover_sats.size()); ++ri) {
    const SatObs& s = out.rover_sats[ri];
    const auto itb = base_idx.find(s.sat);
    if (itb == base_idx.end()) continue;
    const int bi = itb->second;
    const obsd_t& ob = obs[bi];
    if (svh[bi] != 0) continue;
    if (norm(&rs[6 * bi], 3) <= 0.0) continue;
    // Same code on both receivers is required by default (per-receiver code
    // biases would otherwise ride on the DD if a group mixed codes). With
    // cross_code_carrier the code may differ where it is integer-safe: the bias
    // is common to the group's reference and target and cancels in the DD.
    if (ob.code[s.band] != s.code &&
        !(config_.cross_code_carrier && crossCodeSafe(s.sys, s.band)))
      continue;
    // SNR mask on the BASE observation as well. A double difference is only as
    // good as its worst input: masking the rover alone admits a clean rover
    // observation paired with a weak base one, so the DD carries the base's
    // noise while looking (by rover SNR) trustworthy. That asymmetry was
    // measured to produce false fixes - the rover-side mask must have a
    // base-side counterpart. Elevation is the rover's (the base is static and
    // its own elevation differs only by the baseline geometry).
    const bool base_mask =
        config_.snr_mask.ena[1] || config_.snr_mask_dbhz > 0.0;
    if (base_mask && ob.SNR[s.band] <= 0.0) continue;  // unknown base SNR: exclude
    if (ob.SNR[s.band] > 0.0) {
      if (config_.snr_mask.ena[1]) {
        if (testsnr(1, s.band, s.el, ob.SNR[s.band], &config_.snr_mask)) continue;
      } else if (config_.snr_mask_dbhz > 0.0 &&
                 ob.SNR[s.band] < config_.snr_mask_dbhz) {
        continue;
      }
    }

    const bool pr_ok = (s.pr != 0.0) && (ob.P[s.band] != 0.0);
    bool cp_ok = (s.cp_m != 0.0) && (ob.L[s.band] != 0.0);
    // Undifferenced-only systems never enter a double difference at all
    // (neither code nor carrier), which is what keeps them out of AR.
    if (s.sys & config_.navsys_undifferenced_only) continue;
    if (s.sys == SYS_GLO && !config_.glonass_carrier_dd) cp_ok = false;
    if (s.cp_excluded) cp_ok = false;  // CMC gross error: drop carrier this epoch
    if (!pr_ok && !cp_ok) continue;
    const uint8_t code_base = ob.code[s.band];
    groups[{s.sys, s.band, static_cast<int>(s.code),
            static_cast<int>(code_base)}]
        .push_back({ri, bi, code_base, pr_ok, cp_ok});
  }

  for (const auto& kv : groups) {
    const auto& cand = kv.second;
    if (cand.size() < 2) continue;

    // Reference satellite: highest rover elevation, preferring candidates
    // with both pseudorange and carrier phase available.
    const PairCand* ref = nullptr;
    for (const auto& c : cand) {
      if (!(c.pr_ok && c.cp_ok)) continue;
      if (!ref || out.rover_sats[c.rov].el > out.rover_sats[ref->rov].el) ref = &c;
    }
    if (!ref) {
      for (const auto& c : cand) {
        if (!ref || out.rover_sats[c.rov].el > out.rover_sats[ref->rov].el) ref = &c;
      }
    }

    const SatObs& sref = out.rover_sats[ref->rov];
    const obsd_t& ob_ref = obs[ref->base_i];

    for (const auto& c : cand) {
      if (c.rov == ref->rov) continue;
      const SatObs& star = out.rover_sats[c.rov];
      const obsd_t& ob_tar = obs[c.base_i];

      DdSignal d;
      d.sat_ref = sref.sat;
      d.sat_tar = star.sat;
      d.satid_ref = sref.satid;
      d.satid_tar = star.satid;
      d.sys = sref.sys;
      d.band = sref.band;
      d.code_ref = sref.code;
      d.code_tar = star.code;
      d.code_base_ref = ref->code_base;
      d.code_base_tar = c.code_base;
      d.lam = star.lam;

      d.has_pr = ref->pr_ok && c.pr_ok;
      // A single wavelength describes the pair; GLONASS FDMA pairs with
      // different FCN wavelengths cannot form a meaningful carrier DD.
      d.has_cp = ref->cp_ok && c.cp_ok &&
                 std::fabs(sref.lam - star.lam) < 1e-9;
      if (!d.has_pr && !d.has_cp) continue;

      d.sat_ref_rov = sref.sat_pos;
      d.sat_tar_rov = star.sat_pos;
      d.sat_ref_base = Eigen::Vector3d(rs[6 * ref->base_i],
                                       rs[6 * ref->base_i + 1],
                                       rs[6 * ref->base_i + 2]);
      d.sat_tar_base = Eigen::Vector3d(rs[6 * c.base_i],
                                       rs[6 * c.base_i + 1],
                                       rs[6 * c.base_i + 2]);

      const double base_xyz[3] = {config_.base_ecef.x(),
                                  config_.base_ecef.y(),
                                  config_.base_ecef.z()};
      double base_llh[3];
      ecef2pos(base_xyz, base_llh);
      auto base_azel = [&](const Eigen::Vector3d& sat) {
        const double sat_xyz[3] = {sat.x(), sat.y(), sat.z()};
        double e[3], azel[2];
        geodist(sat_xyz, base_xyz, e);
        satazel(base_llh, e, azel);
        return Eigen::Vector2d(azel[0], azel[1]);
      };
      const Eigen::Vector2d azel_base_ref = base_azel(d.sat_ref_base);
      const Eigen::Vector2d azel_base_tar = base_azel(d.sat_tar_base);
      d.el_base_ref = azel_base_ref.y();
      d.el_base_tar = azel_base_tar.y();

      // ionocorr() returns L1 delay/variance. Scale delay by f^-2 and
      // variance by f^-4, then add the troposphere model residual variance.
      struct Atmosphere {
        double ion{0.0}, trop{0.0};
        double ion_var{0.0}, trop_var{0.0};
      };
      auto atmosphere = [&](gtime_t time, int sat, const double* pos,
                            const double* azel, double freq) {
        Atmosphere a;
        double vion = 0.0, vtrop = 0.0;
        ionocorr(time, &nav_, sat, pos, azel, config_.ionoopt,
                 &a.ion, &vion);
        tropcorr(time, &nav_, pos, azel, config_.tropopt,
                 &a.trop, &vtrop);
        const double ratio = (freq > 0.0) ? FREQL1 / freq : 1.0;
        const double scale = ratio * ratio;
        a.ion *= scale;
        a.ion_var = std::max(vion * scale * scale, 0.0);
        a.trop_var = std::max(vtrop, 0.0);
        return a;
      };
      const double rover_azel_ref[2] = {sref.az, sref.el};
      const double rover_azel_tar[2] = {star.az, star.el};
      const double base_azel_ref[2] = {azel_base_ref.x(), azel_base_ref.y()};
      const double base_azel_tar[2] = {azel_base_tar.x(), azel_base_tar.y()};
      const double freq_ref_rov = CLIGHT / sref.lam;
      const double freq_tar_rov = CLIGHT / star.lam;
      const double freq_ref_base =
          sat2freq(sref.sat, ob_ref.code[d.band], &nav_);
      const double freq_tar_base =
          sat2freq(star.sat, ob_tar.code[d.band], &nav_);
      const gtime_t rover_time = gpst2time(out.week, out.tow);
      const Atmosphere ar_ref = atmosphere(
          rover_time, sref.sat, llh, rover_azel_ref, freq_ref_rov);
      const Atmosphere ar_tar = atmosphere(
          rover_time, star.sat, llh, rover_azel_tar, freq_tar_rov);
      const Atmosphere ab_ref = atmosphere(
          ob_ref.time, sref.sat, base_llh, base_azel_ref, freq_ref_base);
      const Atmosphere ab_tar = atmosphere(
          ob_tar.time, star.sat, base_llh, base_azel_tar, freq_tar_base);
      auto differential_model_var = [](const Atmosphere& rover,
                                       const Atmosphere& base) {
        const double d_ion = std::sqrt(rover.ion_var) -
                             std::sqrt(base.ion_var);
        const double d_trop = std::sqrt(rover.trop_var) -
                              std::sqrt(base.trop_var);
        return d_ion * d_ion + d_trop * d_trop;
      };
      d.model_var_ref_sd = differential_model_var(ar_ref, ab_ref);
      d.model_var_tar_sd = differential_model_var(ar_tar, ab_tar);

      // Satellite clock and propagation corrections at each receiver's own
      // epoch. Code has +I+T; carrier has -I+T.
      const double c_dts_ref_rov = CLIGHT * sref.sat_clk;
      const double c_dts_tar_rov = CLIGHT * star.sat_clk;
      const double c_dts_ref_base = CLIGHT * dts[2 * ref->base_i];
      const double c_dts_tar_base = CLIGHT * dts[2 * c.base_i];
      if (d.has_pr) {
        d.pr_rov_ref = sref.pr + c_dts_ref_rov - ar_ref.ion - ar_ref.trop;
        d.pr_base_ref =
            ob_ref.P[d.band] + c_dts_ref_base - ab_ref.ion - ab_ref.trop;
        d.pr_rov_tar = star.pr + c_dts_tar_rov - ar_tar.ion - ar_tar.trop;
        d.pr_base_tar =
            ob_tar.P[d.band] + c_dts_tar_base - ab_tar.ion - ab_tar.trop;
      }
      if (d.has_cp) {
        d.cp_rov_ref = sref.cp_m + c_dts_ref_rov + ar_ref.ion - ar_ref.trop;
        d.cp_base_ref = ob_ref.L[d.band] * sref.lam + c_dts_ref_base +
                        ab_ref.ion - ab_ref.trop;
        d.cp_rov_tar = star.cp_m + c_dts_tar_rov + ar_tar.ion - ar_tar.trop;
        d.cp_base_tar = ob_tar.L[d.band] * star.lam + c_dts_tar_base +
                        ab_tar.ion - ab_tar.trop;
      }

      d.el_ref = sref.el;
      d.el_tar = star.el;
      d.snr_ref = sref.snr;
      d.snr_tar = star.snr;
      // Continuity slip (re-key) and half-cycle (AR-exclude) are computed for
      // both receivers in detectSlips; keep them distinct here.
      d.slip_ref = sref.slip;
      d.slip_tar = star.slip;
      d.half_cycle_ref = sref.half_cycle;
      d.half_cycle_tar = star.half_cycle;

      out.dd.push_back(std::move(d));
    }
  }
  return true;
}

void GnssPreprocessor::detectSlips(const std::vector<obsd_t>& obs,
                                   const std::map<int, int>& rover_idx,
                                   const std::map<int, int>& base_idx,
                                   double tow, PreprocessedEpoch& out) {
  std::set<std::pair<int, int>> slip, cp_excl, half;
  struct Dop {
    int sat, band;
    double r;
  };
  std::vector<Dop> dops;

  for (const auto& kv : rover_idx) {
    const int sat = kv.first;
    const obsd_t& ro = obs[kv.second];
    const auto bit = base_idx.find(sat);
    const obsd_t* bo = (bit != base_idx.end()) ? &obs[bit->second] : nullptr;

    SlipTrack& tr = slip_track_[sat];  // default-constructs (all *val false) if new
    const SlipTrack pr = tr;           // previous-epoch snapshot

    double lam[NFREQ] = {0.0, 0.0, 0.0};
    for (int f = 0; f < NFREQ; ++f) {
      const double freq = sat2freq(sat, ro.code[f], &nav_);
      lam[f] = (freq > 0.0) ? CLIGHT / freq : 0.0;
    }

    // The 1 Hz base is REUSED across ~5 rover epochs. Base-side slip checks must
    // key off the base observation's OWN time, so one base event re-keys once
    // (not up to five times) and a real base outage is not hidden by the rover
    // clock. base_ctow is the base epoch's continuous GPST; new_base is true only
    // on the first rover epoch that consumes a given base observation.
    double base_ctow = -1.0;
    if (bo) {
      int bw = 0;
      const double bt = time2gpst(bo->time, &bw);
      base_ctow = bw * 604800.0 + bt;
    }

    for (int f = 0; f < NFREQ; ++f) {
      if (ro.L[f] == 0.0) continue;  // no rover carrier on this band
      bool band_slip = false;
      const bool base_present = bo && bo->L[f] != 0.0;
      const bool new_base =
          base_present && (!pr.lval_base[f] || base_ctow > pr.base_tow[f] + 1e-6);

      // LLI cycle-slip bit: rover every epoch; base only on a genuinely NEW base
      // epoch (else one flagged base obs re-keys on all ~5 rover reuses).
      if ((ro.LLI[f] & 0x1) || (new_base && (bo->LLI[f] & 0x1))) band_slip = true;
      // Half-cycle-ambiguity bit currently set -> half-integer (exclude from AR).
      if ((ro.LLI[f] & 0x2) || (bo && (bo->LLI[f] & 0x2))) half.insert({sat, f});
      if (pr.codeval[f]) {
        // Half-cycle-ambiguity bit (0x2) transition + observed-code change. The
        // base parts compare against the tracked base state (updated only on a
        // new base epoch below), so they do not fire on reuse.
        if (((ro.LLI[f] ^ pr.lli_rov[f]) & 0x2) ||
            (bo && ((bo->LLI[f] ^ pr.lli_base[f]) & 0x2)))
          band_slip = true;
        if (ro.code[f] != pr.code_rov[f] ||
            (bo && bo->code[f] != pr.code_base[f]))
          band_slip = true;
      }
      // Carrier outage [s]: a long ROVER gap breaks continuity.
      if (pr.lval[f] && (tow - pr.last_seen[f]) > config_.slip_max_gap_s)
        band_slip = true;
      // Base-side carrier outage, measured on the BASE clock: a new base epoch
      // arriving more than slip_max_gap_s after the previous DISTINCT one re-keys
      // (the true base outage the rover-clock version silently bridged).
      if (new_base && pr.lval_base[f] &&
          (base_ctow - pr.base_tow[f]) > config_.slip_max_gap_s)
        band_slip = true;
      if (base_present) {
        tr.lval_base[f] = true;
        tr.base_tow[f] = new_base ? base_ctow : pr.base_tow[f];
      }

      // SD code-minus-carrier gross error (rover-base): drop the corrupt carrier
      // this epoch (cp_excluded) and re-key.
      if (config_.detect_slip_cmc && bo && lam[f] > 0.0 && ro.P[f] != 0.0 &&
          bo->P[f] != 0.0 && bo->L[f] != 0.0) {
        const double cmc =
            (ro.P[f] - bo->P[f]) - (ro.L[f] - bo->L[f]) * lam[f];
        if (pr.cval[f] &&
            std::fabs(cmc - pr.cmc[f]) > config_.slip_cmc_threshold_m) {
          band_slip = true;
          cp_excl.insert({sat, f});
        }
        tr.cmc[f] = cmc;
        tr.cval[f] = true;
      } else {
        tr.cval[f] = false;
      }

      // Doppler-phase residual (common receiver-clock component removed below).
      if (config_.detect_slip_dop && pr.lval[f] && ro.D[f] != 0.0) {
        const double dt = tow - pr.last_seen[f];
        if (dt > 1e-3 && dt <= config_.slip_max_gap_s) {
          dops.push_back({sat, f, (ro.L[f] - pr.lrov[f]) + ro.D[f] * dt});
        }
      }

      if (band_slip) slip.insert({sat, f});

      // Update this band's tracking state.
      tr.lrov[f] = ro.L[f];
      tr.lval[f] = true;
      tr.last_seen[f] = tow;
      tr.code_rov[f] = ro.code[f];
      tr.lli_rov[f] = ro.LLI[f];
      tr.code_base[f] = bo ? bo->code[f] : 0;
      tr.lli_base[f] = bo ? bo->LLI[f] : 0;
      tr.codeval[f] = true;
    }

    // SD geometry-free (dual-frequency, bands 0/1): the sensitive carrier slip
    // detector; a jump flags both bands.
    if (config_.detect_slip_gf && bo && lam[0] > 0.0 && lam[1] > 0.0 &&
        ro.L[0] != 0.0 && ro.L[1] != 0.0 && bo->L[0] != 0.0 && bo->L[1] != 0.0) {
      const double sdgf =
          (ro.L[0] - bo->L[0]) * lam[0] - (ro.L[1] - bo->L[1]) * lam[1];
      if (pr.sdgf_val &&
          std::fabs(sdgf - pr.sdgf) > config_.slip_gf_threshold_m) {
        slip.insert({sat, 0});
        slip.insert({sat, 1});
      }
      tr.sdgf = sdgf;
      tr.sdgf_val = true;
    } else {
      tr.sdgf_val = false;
    }
  }

  // Doppler common-clock (median) rejection PER (system, band): the receiver
  // clock term is frequency-dependent in cycles, so mixing bands/systems would
  // bias the common component and cause false slips. Only per-satellite
  // deviations from the group's common drift are slips; skip groups with < 4.
  if (config_.detect_slip_dop) {
    std::map<std::pair<int, int>, std::vector<std::pair<int, double>>> grp;
    for (const auto& d : dops) {
      int prn = 0;
      grp[{satsys(d.sat, &prn), d.band}].push_back({d.sat, d.r});
    }
    for (auto& kv : grp) {
      auto& v = kv.second;
      if (static_cast<int>(v.size()) < 4) continue;
      std::vector<double> rs;
      rs.reserve(v.size());
      for (const auto& p : v) rs.push_back(p.second);
      std::nth_element(rs.begin(), rs.begin() + rs.size() / 2, rs.end());
      const double common = rs[rs.size() / 2];
      for (const auto& p : v) {
        if (std::fabs(p.second - common) > config_.slip_dop_threshold_cyc)
          slip.insert({p.first, kv.first.second});
      }
    }
  }

  for (auto& s : out.rover_sats) {
    if (slip.count({s.sat, s.band})) s.slip = true;
    if (half.count({s.sat, s.band})) s.half_cycle = true;
    if (cp_excl.count({s.sat, s.band})) s.cp_excluded = true;
  }
}

}  // namespace gnss_utils
