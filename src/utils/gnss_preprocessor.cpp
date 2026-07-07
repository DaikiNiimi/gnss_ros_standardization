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

#include <Eigen/LU>  // Matrix4d::inverse(); avoids Geometry/Eigenvalues which
                     // clash with RTKLIB's trace() macro (see gnss_preprocessor.hpp)

#include "gnss_ros_standardization/gnss_utils.hpp"

namespace gnss_utils {

namespace {

// Undifferenced (zero-difference) Doppler velocity, RTKLIB estvel/resdop-style:
// weighted least squares for receiver velocity (3) + receiver clock drift (1).
// The range-rate model is linear in the unknowns, so one solve is exact.
//   residual = -lambda*D - [ e.(v_sat - v_rcv) + sagnac_rate
//                            + c*drift_rcv - c*drift_sat ]
// with e the rover->satellite unit vector. No differencing is needed: the
// differential error terms double-differencing removes for *position*
// (iono/tropo, clocks) have negligible time-derivatives, so for velocity only
// the receiver clock drift matters and it is estimated as the 4th unknown.
bool estimateDopplerVelocity(const std::vector<SatObs>& sats,
                             const Eigen::Vector3d& rr, Eigen::Vector3d& vel,
                             double& var) {
  const double omge_c = OMGE / CLIGHT;
  std::vector<int> rows;
  std::set<int> seen;
  for (int i = 0; i < static_cast<int>(sats.size()); ++i) {
    const SatObs& s = sats[i];
    if (s.doppler == 0.0 || s.lam <= 0.0 || s.sat_vel.norm() <= 0.0) continue;
    if (!seen.insert(s.sat).second) continue;  // one Doppler per satellite
    rows.push_back(i);
  }
  const int nv = static_cast<int>(rows.size());
  if (nv < 4) return false;

  Eigen::MatrixXd H(nv, 4);
  Eigen::VectorXd y(nv), w(nv);
  for (int k = 0; k < nv; ++k) {
    const SatObs& s = sats[rows[k]];
    const Eigen::Vector3d e = (s.sat_pos - rr).normalized();
    // Model at v_rcv = 0, drift = 0 (the v_rcv/drift dependence is in H).
    const double sagnac =
        omge_c * (s.sat_vel.y() * rr.x() - s.sat_vel.x() * rr.y());
    const double model0 = e.dot(s.sat_vel) + sagnac - CLIGHT * s.sat_clk_drift;
    y(k) = -s.lam * s.doppler - model0;
    H(k, 0) = -e.x() + omge_c * s.sat_pos.y();
    H(k, 1) = -e.y() - omge_c * s.sat_pos.x();
    H(k, 2) = -e.z();
    H(k, 3) = 1.0;
    const double sel = std::sin(std::max(s.el, 0.05));
    w(k) = sel * sel;  // elevation weight (relative)
  }

  const Eigen::MatrixXd HtW = H.transpose() * w.asDiagonal();
  const Eigen::Matrix4d N = HtW * H;
  const Eigen::Matrix4d Ninv = N.inverse();
  if (!Ninv.allFinite()) return false;
  const Eigen::Vector4d x = Ninv * (HtW * y);
  if (!x.allFinite()) return false;

  const Eigen::VectorXd r = y - H * x;
  const double dof = static_cast<double>(nv - 4);
  const double sigma0_sq = dof > 0.0 ? (r.dot(w.asDiagonal() * r)) / dof : 1.0;
  vel = x.head<3>();
  var = sigma0_sq * (Ninv(0, 0) + Ninv(1, 1) + Ninv(2, 2)) / 3.0;
  if (var < 0.0 || !std::isfinite(var)) return false;
  return true;
}

}  // namespace

GnssPreprocessor::GnssPreprocessor(const Config& config) : config_(config) {
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
  spp_opt_.navsys = config_.navsys;
  spp_opt_.elmin = config_.el_mask_rad;
  spp_opt_.nf = std::min(max_band + 1, NFREQ);
  spp_opt_.ionoopt = IONOOPT_BRDC;
  spp_opt_.tropopt = TROPOPT_SAAS;
  spp_opt_.sateph = EPHOPT_BRDC;

  RoverBaseEpochMatcher::Options mopt;
  mopt.max_tdiff_s = config_.max_tdiff_s;
  mopt.match_tol_s = config_.match_tol_s;
  matcher_ = RoverBaseEpochMatcher(mopt);

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
  for (const auto& pair : matcher_.drainMatches()) {
    PreprocessedEpoch ep;
    if (buildEpoch(*pair.rover, *pair.base, pair.age_s, approx_rover, ep)) {
      out.push_back(std::move(ep));
    }
  }
  return out;
}

bool GnssPreprocessor::buildEpoch(const ObsMsg& rover, const ObsMsg& base,
                                  double age_s,
                                  const Eigen::Vector3d* approx_rover,
                                  PreprocessedEpoch& out) {
  // One obsd_t per (receiver, satellite): rover (rcv=1) entries first, then
  // base (rcv=2), each stamped with its own receive time.
  std::vector<obsd_t> obs = buildObsEpoch(rover, &base, freq_mask_);
  if (obs.empty()) return false;
  const int n = static_cast<int>(obs.size());
  int n_rover = 0;
  while (n_rover < n && obs[n_rover].rcv == 1) ++n_rover;
  if (n_rover == 0) return false;

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
  Eigen::Vector3d rr;
  bool have_rr = false;
  if (approx_rover) {
    rr = *approx_rover;
    have_rr = true;
  }
  if (!have_rr) {
    sol_t sol{};
    std::vector<double> azel(2 * n_rover, 0.0);
    std::vector<ssat_t> ssat(MAXSAT);
    char msg[128] = "";
    if (pntpos(obs.data(), n_rover, &nav_, &spp_opt_, &sol, azel.data(),
               ssat.data(), msg)) {
      rr = Eigen::Vector3d(sol.rr[0], sol.rr[1], sol.rr[2]);
      have_rr = true;
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
  if (!have_rr) return false;
  last_apriori_ = rr;
  last_apriori_valid_ = true;

  const double rr_a[3] = {rr.x(), rr.y(), rr.z()};
  double llh[3];
  ecef2pos(rr_a, llh);

  out.week = rover.week;
  out.tow = rover.tow;
  out.age_s = age_s;
  out.stamp = rover.header.stamp;
  out.base_ecef = config_.base_ecef;
  out.rover_ecef_apriori = rr;

  // Undifferenced rover satellites passing health/system/elevation/SNR masks.
  for (int i = 0; i < n_rover; ++i) {
    const obsd_t& o = obs[i];
    int prn = 0;
    const int sys = satsys(o.sat, &prn);
    if (!(sys & config_.navsys)) continue;
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
      // the flat snr_mask_dbhz threshold.
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
  if (out.rover_sats.empty()) return false;

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
    Eigen::Vector3d vel;
    double vvar = 0.0;
    if (estimateDopplerVelocity(out.rover_sats, rr, vel, vvar)) {
      out.rover_vel_ecef = vel;
      out.rover_vel_var = vvar;
      out.rover_vel_valid = true;
    }
  }

  // Double differences need a usable base epoch and a known base position.
  if (age_s > config_.max_age_s) return true;
  if (config_.base_ecef.norm() <= 0.0) return true;

  std::map<int, int> base_idx;  // sat -> index into obs[]
  for (int i = n_rover; i < n; ++i) base_idx[obs[i].sat] = i;

  // Pair rover satellites with base entries per (sys, band). Indices refer to
  // out.rover_sats, which is complete (and stable) by now.
  struct PairCand {
    int rov;      // index into out.rover_sats
    int base_i;   // index into obs[] / rs[]
    bool pr_ok;
    bool cp_ok;
  };
  // Group by (system, band, observed code): the DD reference and target must
  // share the same code, else per-receiver inter-code biases (e.g. 1C vs 1W)
  // do not cancel in the DD. s.code equals the base code here (enforced below).
  std::map<std::tuple<int, int, int>, std::vector<PairCand>> groups;

  for (int ri = 0; ri < static_cast<int>(out.rover_sats.size()); ++ri) {
    const SatObs& s = out.rover_sats[ri];
    const auto itb = base_idx.find(s.sat);
    if (itb == base_idx.end()) continue;
    const int bi = itb->second;
    const obsd_t& ob = obs[bi];
    if (svh[bi] != 0) continue;
    if (norm(&rs[6 * bi], 3) <= 0.0) continue;
    // Same code on both receivers: per-receiver code biases (e.g. 1C vs 1W)
    // do not cancel in DD when the receivers track different codes.
    if (ob.code[s.band] != s.code) continue;

    const bool pr_ok = (s.pr != 0.0) && (ob.P[s.band] != 0.0);
    bool cp_ok = (s.cp_m != 0.0) && (ob.L[s.band] != 0.0);
    if (s.sys == SYS_GLO && !config_.glonass_carrier_dd) cp_ok = false;
    if (s.cp_excluded) cp_ok = false;  // CMC gross error: drop carrier this epoch
    if (!pr_ok && !cp_ok) continue;
    groups[{s.sys, s.band, static_cast<int>(s.code)}].push_back(
        {ri, bi, pr_ok, cp_ok});
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
      d.lam = star.lam;

      d.has_pr = ref->pr_ok && c.pr_ok;
      // A single wavelength describes the pair; GLONASS FDMA pairs with
      // different FCN wavelengths cannot form a meaningful carrier DD.
      d.has_cp = ref->cp_ok && c.cp_ok &&
                 std::fabs(sref.lam - star.lam) < 1e-9;
      if (!d.has_pr && !d.has_cp) continue;

      // Satellite clock correction (+c*dts, at each receiver's OWN reception
      // epoch): removes the satellite clock from the observable so it cancels
      // exactly in the single difference even when rover and base are
      // asynchronous (satposs() gives each entry its own transmission-time
      // dts). Matches RTKLIB zdres(), which applies -CLIGHT*dts to rover and
      // base separately before differencing (rtkpos.c).
      const double c_dts_ref_rov = CLIGHT * sref.sat_clk;
      const double c_dts_tar_rov = CLIGHT * star.sat_clk;
      const double c_dts_ref_base = CLIGHT * dts[2 * ref->base_i];
      const double c_dts_tar_base = CLIGHT * dts[2 * c.base_i];
      if (d.has_pr) {
        d.pr_rov_ref = sref.pr + c_dts_ref_rov;
        d.pr_base_ref = ob_ref.P[d.band] + c_dts_ref_base;
        d.pr_rov_tar = star.pr + c_dts_tar_rov;
        d.pr_base_tar = ob_tar.P[d.band] + c_dts_tar_base;
      }
      if (d.has_cp) {
        d.cp_rov_ref = sref.cp_m + c_dts_ref_rov;
        d.cp_base_ref = ob_ref.L[d.band] * sref.lam + c_dts_ref_base;
        d.cp_rov_tar = star.cp_m + c_dts_tar_rov;
        d.cp_base_tar = ob_tar.L[d.band] * star.lam + c_dts_tar_base;
      }

      d.sat_ref_rov = sref.sat_pos;
      d.sat_tar_rov = star.sat_pos;
      d.sat_ref_base = Eigen::Vector3d(rs[6 * ref->base_i],
                                       rs[6 * ref->base_i + 1],
                                       rs[6 * ref->base_i + 2]);
      d.sat_tar_base = Eigen::Vector3d(rs[6 * c.base_i],
                                       rs[6 * c.base_i + 1],
                                       rs[6 * c.base_i + 2]);

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

    for (int f = 0; f < NFREQ; ++f) {
      if (ro.L[f] == 0.0) continue;  // no rover carrier on this band
      bool band_slip = false;

      // LLI cycle-slip bit (rover / base).
      if ((ro.LLI[f] & 0x1) || (bo && (bo->LLI[f] & 0x1))) band_slip = true;
      // Half-cycle-ambiguity bit currently set -> half-integer (exclude from AR).
      if ((ro.LLI[f] & 0x2) || (bo && (bo->LLI[f] & 0x2))) half.insert({sat, f});
      if (pr.codeval[f]) {
        // Half-cycle-ambiguity bit (0x2) transition (rover / base).
        if (((ro.LLI[f] ^ pr.lli_rov[f]) & 0x2) ||
            (bo && ((bo->LLI[f] ^ pr.lli_base[f]) & 0x2)))
          band_slip = true;
        // Observed-code change (rover / base): the carrier bias changes.
        if (ro.code[f] != pr.code_rov[f] ||
            (bo && bo->code[f] != pr.code_base[f]))
          band_slip = true;
      }
      // Carrier outage [s]: a long ROVER gap breaks continuity.
      if (pr.lval[f] && (tow - pr.last_seen[f]) > config_.slip_max_gap_s)
        band_slip = true;
      // Base-side carrier outage (independent of the rover): the base arc of the
      // rover-base SD ambiguity can break while the rover keeps tracking, so
      // track base presence separately. Base absent -> last_seen_base is left
      // stale (the gap grows); on return, a long gap re-keys.
      if (bo && bo->L[f] != 0.0) {
        if (pr.lval_base[f] &&
            (tow - pr.last_seen_base[f]) > config_.slip_max_gap_s)
          band_slip = true;
        tr.lval_base[f] = true;
        tr.last_seen_base[f] = tow;
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
