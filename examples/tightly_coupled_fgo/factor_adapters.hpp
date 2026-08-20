// SPDX-License-Identifier: MIT
//
// Glue between GnssPreprocessor output and grouped GTSAM DD factors.
// DDs sharing one reference observation map to one vector factor with a full
// covariance. Shared by the gnss_fgo (GNSS-only) and gnss_imu_fgo (GNSS/IMU)
// nodes.
#ifndef EXAMPLES_TIGHTLY_COUPLED_FGO_FACTOR_ADAPTERS_HPP
#define EXAMPLES_TIGHTLY_COUPLED_FGO_FACTOR_ADAPTERS_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Cholesky>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include "gnss_ros_standardization/gnss_preprocessor.hpp"

namespace gnss_fgo {


// Measurement-model constants shared by grouped factor construction, FDE and
// posterior AR. Only the two undifferenced sigmas are data-dependent enough to
// expose in YAML (noise.*); the rest are fixed, well-justified constants (see
// the tightly_coupled_fgo README). One struct keeps a
// single stochastic model.
struct AdapterConfig {
  double pr_sigma_m{0.5};    // UNDIFFERENCED (zenith) pseudorange sigma [m]
  double cp_sigma_m{0.005};  // UNDIFFERENCED (zenith) carrier phase sigma [m]
  // 1/sin(el) elevation weighting of the measurement noise (RTKLIB-style).
  bool elevation_weighting{true};
  // Robust (Huber M-estimator) measurement noise, as in the reference MATLAB.
  bool robust{true};
  double huber_k{1.345};
  // Effective number of rover epochs sharing one base observation. Scaling
  // only the base part prevents a reused 1 Hz sample from contributing the
  // same independent information at every faster rover epoch.
  double base_reuse_factor{1.0};

  // Loose prior on a carried ambiguity at its first appearance / after a re-key
  // (the reference's sig_n0). This soft prior fixes the per-(sys,band) DD gauge.
  double init_sigma_cycles{20.0};
  // Std dev [cycle] of a held integer DD constraint. Added exactly ONCE per arc,
  // so this must equal RTKLIB's SUSTAINED hold sigma (varholdamb accumulates as
  // 0.316/sqrt(N) because RTKLIB re-injects it every epoch), not its per-epoch
  // 0.316. Lock-in risk is bounded by hold_refresh_s instead.
  double hold_sigma_cycles{0.03};
  // Gross-error gate [m] on the DD pseudorange innovation at the motion-predicted
  // position (RTKLIB rejionno). 0 = off, which is the default: gating against a
  // prediction a wrong hold has already moved rejects the good DDs whose
  // redundancy would expose the wrong fix.
  double pr_innov_gate_m{0.0};
};

// What the DD factors of one epoch ACTUALLY contain, recorded as they are built,
// so the post-fit FIX validation can reproduce the robust weight the graph
// applied. A weight is a property of the whole factor, and neither its row set
// nor its R can be re-derived safely at validation time: `pr_innov_gate_m` and
// partial AR drop rows at build time, and R is a function of the cfg, sigma and
// signal order then in force.
struct DdFactorLayout {
  struct Group {
    bool carrier{false};
    std::vector<int> dd_index;    // ep.dd indices, in the factor's ROW order
    Eigen::MatrixXd nominal_cov;  // the R the factor was actually built with
  };
  std::vector<Group> groups;
  bool valid{false};
};

// One carrier DD pair added to the graph this epoch; input to LAMBDA.
struct DdAmbiguityPair {
  gtsam::Key ref;
  gtsam::Key tar;
  double lam;
  double el{0.0};  // target-satellite elevation [rad] (for the AR elevation mask)
  int dd_index{-1};        // index into ep.dd
  bool half_cycle{false};  // either satellite has a half-cycle flag -> AR-exclude
  bool fresh{false};       // ref or tar ambiguity (re)allocated this epoch
                           // (partial-AR drop priority: newest arcs first)
  int lock{0};             // min carried-epoch count of the two ambiguities
                           // (RTKLIB lock count; gates the integer search)
};


inline double receiverMeasurementVar(double sigma, double el,
                                     bool elevation_weighting) {
  const double s = elevation_weighting ? std::max(std::sin(el), 0.1) : 1.0;
  return (sigma / s) * (sigma / s);
}

inline double ddSingleDifferenceVar(const gnss_utils::DdSignal& d, bool ref,
                                    double sigma, const AdapterConfig& cfg) {
  const double el_rov = ref ? d.el_ref : d.el_tar;
  const double el_base = ref ? d.el_base_ref : d.el_base_tar;
  const double model_var = ref ? d.model_var_ref_sd : d.model_var_tar_sd;
  return receiverMeasurementVar(sigma, el_rov, cfg.elevation_weighting) +
         std::max(cfg.base_reuse_factor, 1.0) *
             receiverMeasurementVar(sigma, el_base, cfg.elevation_weighting) +
         std::max(model_var, 0.0);
}

inline Eigen::MatrixXd groupedDdCovariance(
    const std::vector<gnss_utils::DdSignal>& rows, double sigma,
    const AdapterConfig& cfg) {
  const int n = static_cast<int>(rows.size());
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(n, n);
  if (n == 0) return R;
  const double vref = ddSingleDifferenceVar(rows.front(), true, sigma, cfg);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) R(i, j) = vref;
    R(i, i) += ddSingleDifferenceVar(rows[i], false, sigma, cfg);
  }
  return R;
}

// Inverse symmetric square root of a covariance: the decorrelating transform S
// with S R S' = I and S = S'. Symmetric, not Cholesky: a lower-triangular
// factor privileges row 1, so permuting the satellites of a group would change
// the cost, and satellite order is a preprocessor artefact.
//
// Returns false when R is not positive definite or the decomposition fails. A
// covariance that cannot be inverted is REPORTED, never repaired by flooring.
inline bool inverseSymmetricSqrt(const Eigen::MatrixXd& R, Eigen::MatrixXd& S) {
  if (R.rows() != R.cols() || R.rows() == 0 || !R.allFinite()) return false;
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(R);
  if (es.info() != Eigen::Success) return false;
  const Eigen::VectorXd d = es.eigenvalues();
  const double dmax = d.maxCoeff();
  if (!(dmax > 0.0) || !d.allFinite()) return false;
  // Positive definite, not merely non-negative: a zero eigenvalue means a
  // measurement direction with no noise, which is not a model this can whiten.
  if (d.minCoeff() <= 1e-12 * dmax) return false;
  S = es.eigenvectors() * d.cwiseSqrt().cwiseInverse().asDiagonal() *
      es.eigenvectors().transpose();
  return S.allFinite();
}

// Null when R cannot support a noise model. The caller must then NOT build the
// factor, and must NOT fall back to gtsam::noiseModel::Gaussian::Covariance:
// that checks only that the matrix is square, so a singular R yields a model
// whose whitening is NaN, silently poisoning the graph.
//
// R comes from groupedDdCovariance, which sums positive variances, so it is
// positive definite by construction. Reaching the null return is evidence of a
// defect upstream, not of unusual data; the caller logs it.
inline gtsam::SharedNoiseModel groupedDdNoise(const Eigen::MatrixXd& R,
                                              const AdapterConfig& cfg) {
  // Reject an uninvertible covariance before any noise model is built.
  Eigen::MatrixXd S;
  if (!inverseSymmetricSqrt(R, S)) return nullptr;
  if (!cfg.robust) return gtsam::noiseModel::Gaussian::Covariance(R);
  return gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(cfg.huber_k),
      gtsam::noiseModel::Gaussian::Covariance(R));
}

namespace detail {
// Single-difference (rover-base) code-minus-carrier ambiguity estimate [cycles].
// This is the natural initial value for the per-satellite SD ambiguity key: its
// difference across satellites is the DD integer, so initializing both the
// reference and target keys with the SD CMC centres the loose target prior at
// the right place. Falls back to the rover-only estimate when the base
// pseudorange is absent. (cp_* are in metres, = lambda * cycles.)
inline double codeMinusCarrier(double cp_rov, double cp_base, double pr_rov,
                               double pr_base, double lam) {
  if (pr_rov != 0.0 && pr_base != 0.0) {
    return ((cp_rov - cp_base) - (pr_rov - pr_base)) / lam;
  }
  return (pr_rov != 0.0) ? (cp_rov - pr_rov) / lam : 0.0;
}

// Sagnac-corrected geometric range and rcv->sat unit vector (matches
// gtsam::gnss::geodist so residuals are consistent with the float factors).
inline double geodistSagnac(const Eigen::Vector3d& sat,
                            const Eigen::Vector3d& rcv, Eigen::Vector3d* e) {
  const Eigen::Vector3d d = sat - rcv;
  const double r = d.norm();
  if (e) *e = (r > 0.0) ? Eigen::Vector3d(d / r) : Eigen::Vector3d::Zero();
  return r + OMGE * (sat.x() * rcv.y() - sat.y() * rcv.x()) / CLIGHT;
}


// DD carrier-phase residual [m] of one DD at rover position x with integer N.
inline double ddCarrierResidual(const gnss_utils::DdSignal& d,
                                const Eigen::Vector3d& base_ecef,
                                const Eigen::Vector3d& x, double N) {
  Eigen::Vector3d e;
  const double model =
      (geodistSagnac(d.sat_ref_rov, x, &e) -
       geodistSagnac(d.sat_ref_base, base_ecef, &e)) -
      (geodistSagnac(d.sat_tar_rov, x, &e) -
       geodistSagnac(d.sat_tar_base, base_ecef, &e));
  const double obs =
      (d.cp_rov_ref - d.cp_base_ref) - (d.cp_rov_tar - d.cp_base_tar);
  return obs - model - d.lam * N;
}

// DD pseudorange residual (innovation) [m] of one DD at rover position x. The
// satellite and receiver clocks cancel in the double difference, so this is the
// observed DD minus the geometric DD - the quantity RTKLIB's rejionno gates.
inline double ddPseudorangeResidual(const gnss_utils::DdSignal& d,
                                    const Eigen::Vector3d& base_ecef,
                                    const Eigen::Vector3d& x) {
  Eigen::Vector3d e;
  const double model =
      (geodistSagnac(d.sat_ref_rov, x, &e) -
       geodistSagnac(d.sat_ref_base, base_ecef, &e)) -
      (geodistSagnac(d.sat_tar_rov, x, &e) -
       geodistSagnac(d.sat_tar_base, base_ecef, &e));
  const double obs =
      (d.pr_rov_ref - d.pr_base_ref) - (d.pr_rov_tar - d.pr_base_tar);
  return obs - model;
}
}  // namespace detail

// RMS of the code DD residual at `after` minus the same RMS at `before` [m].
// Positive means the move from `before` to `after` fits the pseudoranges WORSE.
//
// Differential on purpose: in an urban canyon the absolute code residual is
// dominated by multipath, metres of it, but that bias is common to both
// positions and cancels here, leaving only the pseudoranges' opinion of the
// increment. Used to validate the correction LAMBDA applies, which is the one
// thing a wrong integer cannot hide - the carrier ambiguity absorbs a state
// error by construction, the pseudorange has no ambiguity to absorb it with.
//
// Returns 0 (no opinion) when fewer than `min_dd` code DDs are usable, so a
// caller can compare against a positive threshold and get a pass-through.
inline double codeResidualGrowth(const gnss_utils::PreprocessedEpoch& ep,
                                 const Eigen::Vector3d& before,
                                 const Eigen::Vector3d& after, int min_dd,
                                 int* n_used = nullptr) {
  double s_before = 0.0, s_after = 0.0;
  int n = 0;
  for (const auto& d : ep.dd) {
    if (!d.has_pr) continue;
    const double r0 = detail::ddPseudorangeResidual(d, ep.base_ecef, before);
    const double r1 = detail::ddPseudorangeResidual(d, ep.base_ecef, after);
    if (!std::isfinite(r0) || !std::isfinite(r1)) continue;
    s_before += r0 * r0;
    s_after += r1 * r1;
    ++n;
  }
  if (n_used) *n_used = n;
  if (n < std::max(min_dd, 1)) return 0.0;
  return std::sqrt(s_after / n) - std::sqrt(s_before / n);
}

// One outage = one re-anchor.
//
// A gap trigger built only on "no usable DD for longer than the threshold" is
// true for EVERY epoch of the outage, because the last-DD timestamp only
// advances when a DD exists. Without this latch the re-anchor re-fires each
// epoch and re-keys every carried carrier arc again.
class GapReanchorLatch {
 public:
  // True when the gap condition should fire NOW. `starved` is the caller's
  // "no usable DD for longer than the threshold, and not a delivery gap" test.
  bool shouldFire(bool starved) const { return starved && !latched_; }

  // Call after acting on shouldFire(). Arms the latch so the same outage
  // cannot fire again.
  void arm() { latched_ = true; }

  // Usable GNSS is back: re-arm for the NEXT outage. A later outage fires
  // again because the DDs that ended this one advance the caller's timestamp.
  void onUsableGnss() { latched_ = false; }

  void reset() { latched_ = false; }
  bool latched() const { return latched_; }

 private:
  bool latched_{false};
};

class CodeGrowthMonitor {
 public:
  // Feed every epoch that produced a fix; returns true when it should be
  // rejected. `threshold` <= 0 disables (and keeps the counter clear).
  bool reject(double growth, double threshold, int persist) {
    if (threshold <= 0.0) {
      run_ = 0;
      return false;
    }
    run_ = (growth > threshold) ? run_ + 1 : 0;
    return run_ >= std::max(persist, 1);
  }
  // Call whenever the ambiguity state is discarded (re-key, graph reset), so a
  // count accumulated against integers that no longer exist cannot carry over.
  void reset() { run_ = 0; }
  int run() const { return run_; }

 private:
  int run_{0};
};

// RMS of the code DD residual AT one position [m] - the absolute companion to
// codeResidualGrowth above, and the one that survives the other's blind spot.
//
// codeResidualGrowth scores the INCREMENT that AR applies, `fix - float`. With a
// confident IMU chain the float is itself dragged to the wrong place by the held
// ambiguities, so `fix ~ float`, the increment vanishes and the differential test
// sees nothing to object to, so tightening it buys nothing. The absolute
// residual has no such hole, because the pseudoranges are not a function of the
// carrier ambiguities at all - whatever the held integers have done to the
// state, the code still reports where the antenna is not.
//
// It works despite multipath because the bar sits well above it: in harsh
// multipath the code DD RMS at a GOOD fix is ~0.75 m median / 1.41 m p90, while
// during a runaway it sits at ~2.0 m median. (Absolute code residuals are easy
// to dismiss as multipath-dominated. That is an assumption, and
// measuring it was what showed the separation is there.)
//
// Returns 0 (no opinion) when fewer than `min_dd` code DDs are usable.
inline double codeResidualRms(const gnss_utils::PreprocessedEpoch& ep,
                              const Eigen::Vector3d& pos, int min_dd,
                              int* n_used = nullptr) {
  double s = 0.0;
  int n = 0;
  for (const auto& d : ep.dd) {
    if (!d.has_pr) continue;
    const double r = detail::ddPseudorangeResidual(d, ep.base_ecef, pos);
    if (!std::isfinite(r)) continue;
    s += r * r;
    ++n;
  }
  if (n_used) *n_used = n;
  if (n < std::max(min_dd, 1)) return 0.0;
  return std::sqrt(s / n);
}

using AmbiguityGroupId = std::tuple<int, int, int, int>;  // sys,band,rov/base code

// Rover position from the CODE double differences alone (weighted least
// squares, RTKLIB DGNSS/RTD equivalent).
//
// WHY THIS EXISTS. An integrity test computed from the graph posterior sits
// INSIDE the loop it is supposed to police:
// a confident IMU drags the float, the float biases the AR prior, the accepted
// integers are held, the holds drag the graph, and the next float is dragged
// further. A statistic derived from that state cannot see the failure, because
// it is made of it. Escaping needs a position that neither the IMU nor ANY
// carrier ambiguity can move - and code DDs are exactly that: no ambiguity term
// appears in them at all.
//
// Note the sense in which this is "independent": NOT statistically (the graph
// consumes the same pseudoranges), but structurally - wrong fixes live in the
// carrier/integer channel, and nothing in that channel can move this solution.
//
// It is preferred over the standalone SPP a-priori for the same job because
// differencing against the base cancels ionosphere, troposphere, orbit and
// satellite-clock error: this lands at ~0.3-0.5 m per epoch on a short
// baseline, against 1-3 m for SPP. That matters because the dangerous
// false-fix band is 0.25-1 m, so the yardstick has to reach into it without
// being averaged over a long window (a long window is a slow escape).
//
// IMPORTANT: pass the RAW `ep`, never the pre-fit-filtered set. The pre-fit
// filter gates DDs against the predicted state, which is the state under
// suspicion; feeding its output back here would re-close the very loop this
// function exists to break.
struct CodeDdSolution {
  Eigen::Vector3d pos{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d cov{Eigen::Matrix3d::Zero()};
  int n_dd{0};        // code DDs used at convergence
  int n_rejected{0};  // dropped by the outlier test
  double resid_rms{0.0};
  int iterations{0};
  bool ok{false};
};

namespace detail {
// d(geodistSagnac)/d(rcv) for a fixed satellite: the receiver-to-satellite unit
// vector enters with a minus sign, plus the (tiny, but free) Sagnac gradient.
inline Eigen::Vector3d geodistSagnacGradient(const Eigen::Vector3d& sat,
                                             const Eigen::Vector3d& rcv) {
  const Eigen::Vector3d d = sat - rcv;
  const double r = d.norm();
  Eigen::Vector3d g = (r > 0.0) ? Eigen::Vector3d(-d / r)
                                : Eigen::Vector3d::Zero();
  g.x() += -OMGE * sat.y() / CLIGHT;
  g.y() += OMGE * sat.x() / CLIGHT;
  return g;
}
}  // namespace detail

// `seed` only speeds up convergence - the iteration is run to convergence, so
// the answer is determined by the observations, not by where it started (a
// property the unit tests assert by perturbing the seed).
inline CodeDdSolution solveCodeDdWls(const gnss_utils::PreprocessedEpoch& ep,
                                     const AdapterConfig& cfg,
                                     const Eigen::Vector3d& seed) {
  constexpr int kMinDd = 4;          // 3 unknowns + 1 for redundancy
  constexpr int kMaxIterations = 10;
  constexpr double kConvergedM = 1e-4;
  constexpr double kOutlierSigma = 4.0;
  constexpr int kMaxRejected = 8;

  CodeDdSolution out;
  if (ep.base_ecef.norm() <= 0.0 || !seed.allFinite()) return out;

  // Group exactly as addGroupedDdFactors does, so each group shares one
  // reference satellite and groupedDdCovariance's shared-reference correlation
  // is the correct one. Rows are then flattened, remembering each group's
  // extent, so the epoch is one block-diagonal system.
  std::map<AmbiguityGroupId, std::vector<gnss_utils::DdSignal>> groups;
  for (const auto& d : ep.dd) {
    if (!d.has_pr) continue;
    groups[AmbiguityGroupId{d.sys, d.band, d.code_ref, d.code_base_ref}]
        .push_back(d);
  }

  Eigen::Vector3d x = seed;
  for (int reject_pass = 0; reject_pass <= kMaxRejected; ++reject_pass) {
    std::vector<const gnss_utils::DdSignal*> rows;
    std::vector<std::pair<AmbiguityGroupId, int>> origin;  // group, index in it
    std::vector<std::pair<int, int>> blocks;               // (start row, size)
    for (const auto& kv : groups) {
      blocks.emplace_back(static_cast<int>(rows.size()),
                          static_cast<int>(kv.second.size()));
      for (int i = 0; i < static_cast<int>(kv.second.size()); ++i) {
        rows.push_back(&kv.second[i]);
        origin.emplace_back(kv.first, i);
      }
    }
    const int n = static_cast<int>(rows.size());
    if (n < kMinDd) return out;

    // Measurement covariance. Built from elevations and modelled atmospheric
    // residuals only, so it does not depend on x and is formed once per pass.
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(n, n);
    {
      auto git = groups.begin();
      for (const auto& blk : blocks) {
        R.block(blk.first, blk.first, blk.second, blk.second) =
            groupedDdCovariance(git->second, cfg.pr_sigma_m, cfg);
        ++git;
      }
    }
    const Eigen::LLT<Eigen::MatrixXd> llt(R);
    if (llt.info() != Eigen::Success) return out;

    // Gauss-Newton on the whitened system.
    Eigen::MatrixXd H(n, 3);
    Eigen::VectorXd r(n);
    Eigen::MatrixXd A;
    bool solved = false;
    for (int it = 0; it < kMaxIterations; ++it) {
      for (int i = 0; i < n; ++i) {
        const auto& d = *rows[i];
        r(i) = detail::ddPseudorangeResidual(d, ep.base_ecef, x);
        // residual = obs - model, so dr/dx = -dmodel/dx.
        H.row(i) = -(detail::geodistSagnacGradient(d.sat_ref_rov, x) -
                     detail::geodistSagnacGradient(d.sat_tar_rov, x))
                        .transpose();
      }
      if (!r.allFinite() || !H.allFinite()) return out;
      A = llt.matrixL().solve(H);
      const Eigen::VectorXd b = llt.matrixL().solve(Eigen::VectorXd(-r));
      const Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(A);
      if (qr.rank() < 3) return out;
      const Eigen::Vector3d dx = qr.solve(b);
      if (!dx.allFinite()) return out;
      x += dx;
      ++out.iterations;
      if (dx.norm() < kConvergedM) { solved = true; break; }
    }
    if (!solved) return out;

    // Outlier screening: Baarda's w-test. This has to be self-contained - the
    // graph's own gating is judged against the state under suspicion, so it is
    // exactly what must not be trusted here.
    //
    // The residual is normalized by its OWN standard deviation, not by the
    // measurement's. Those differ by the redundancy of the row: least squares
    // absorbs part of a blunder into the position, so a large error leaves a
    // deceptively small residual, and dividing by the measurement sigma hides
    // it. Cov(v) = R - H N^-1 H^T supplies the right denominator (it is also
    // why the correlation from the shared reference satellite must be carried
    // through rather than approximated by the diagonal of R).
    const Eigen::Matrix3d N = A.transpose() * A;  // = H^T R^-1 H
    const Eigen::Matrix3d Ninv = N.inverse();
    if (!Ninv.allFinite()) return out;
    const Eigen::MatrixXd Qv = R - H * Ninv * H.transpose();

    int worst_row = -1;
    double worst_z = 0.0;
    for (int i = 0; i < n; ++i) {
      const double var = Qv(i, i);
      if (var <= 1e-9) continue;  // no redundancy on this row: not testable
      const double z = std::fabs(r(i)) / std::sqrt(var);
      if (z > worst_z) { worst_z = z; worst_row = i; }
    }
    if (worst_z > kOutlierSigma && worst_row >= 0 && n > kMinDd &&
        reject_pass < kMaxRejected) {
      // Drop the worst row and re-solve.
      auto& victim_group = groups[origin[worst_row].first];
      victim_group.erase(victim_group.begin() + origin[worst_row].second);
      if (victim_group.empty()) groups.erase(origin[worst_row].first);
      ++out.n_rejected;
      continue;
    }

    const Eigen::Matrix3d cov = Ninv;
    if (!cov.allFinite()) return out;
    out.pos = x;
    out.cov = cov;
    out.n_dd = n;
    out.resid_rms = std::sqrt(r.squaredNorm() / n);
    out.ok = true;
    return out;
  }
  return out;
}

// A rover position this epoch that the ESTIMATOR CANNOT MOVE.
//
// NOT `ep.rover_ecef_apriori`: both nodes pass their own last estimate to
// drainEpochs as the hint, so that field comes back as a copy of the state
// under suspicion. Preference order, both computed from this epoch's
// observations alone:
//   1. code double-difference WLS - best on a short baseline (differential
//      errors cancel), and a converged solve, so the seed is only a
//      linearization point.
//   2. SPP (rover_ecef_spp) - metre-level, needs no base.
// The a-priori is the last resort: a re-anchor that cannot happen is worse than
// one anchored to a stale state. `independent()` reports which case applies.
struct IndependentCodePosition {
  enum class Source { None, DdWls, Spp, Apriori };
  Eigen::Vector3d pos{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d cov{Eigen::Matrix3d::Zero()};
  Source source{Source::None};
  bool ok{false};
  bool independent() const {
    return ok && (source == Source::DdWls || source == Source::Spp);
  }
  const char* sourceName() const {
    switch (source) {
      case Source::DdWls: return "code-DD WLS";
      case Source::Spp: return "SPP";
      case Source::Apriori: return "a-priori (NOT independent)";
      default: return "none";
    }
  }
};

// Sigma floors per source. Neither solve models multipath, so their formal
// covariances are optimistic, and this position decides whether to DISCARD a
// whole graph state - an over-confident anchor is the expensive direction.
inline Eigen::Matrix3d floorCovariance(const Eigen::Matrix3d& cov,
                                       double sigma_floor_m) {
  Eigen::Matrix3d out = cov;
  const double floor_var = sigma_floor_m * sigma_floor_m;
  bool usable = out.allFinite();
  for (int i = 0; i < 3 && usable; ++i) usable = out(i, i) >= 0.0;
  if (!usable) return Eigen::Matrix3d::Identity() * floor_var;
  for (int i = 0; i < 3; ++i) out(i, i) = std::max(out(i, i), floor_var);
  return out;
}

inline IndependentCodePosition independentCodePosition(
    const gnss_utils::PreprocessedEpoch& ep, const AdapterConfig& cfg) {
  constexpr double kDdWlsSigmaFloorM = 1.0;
  constexpr double kSppSigmaFloorM = 3.0;
  constexpr double kAprioriSigmaM = 10.0;
  IndependentCodePosition out;
  const CodeDdSolution dd = solveCodeDdWls(ep, cfg, ep.rover_ecef_apriori);
  if (dd.ok && dd.pos.allFinite() && dd.pos.norm() > 1.0) {
    out.pos = dd.pos;
    out.cov = floorCovariance(dd.cov, kDdWlsSigmaFloorM);
    out.source = IndependentCodePosition::Source::DdWls;
    out.ok = true;
    return out;
  }
  if (ep.rover_spp_valid && ep.rover_ecef_spp.allFinite() &&
      ep.rover_ecef_spp.norm() > 1.0) {
    out.pos = ep.rover_ecef_spp;
    out.cov = floorCovariance(ep.rover_spp_cov, kSppSigmaFloorM);
    out.source = IndependentCodePosition::Source::Spp;
    out.ok = true;
    return out;
  }
  if (ep.rover_ecef_apriori.allFinite() && ep.rover_ecef_apriori.norm() > 1.0) {
    out.pos = ep.rover_ecef_apriori;
    out.cov = Eigen::Matrix3d::Identity() * (kAprioriSigmaM * kAprioriSigmaM);
    out.source = IndependentCodePosition::Source::Apriori;
    out.ok = true;
  }
  return out;
}

// Carried-across-epochs ambiguity manager. One SD carrier ambiguity key per
// (sat,band), reused every epoch; re-keyed on a cycle slip, an outage longer
// than max_outage_s, or the hold-refresh age. The value + a loose init prior
// are inserted only on (re)allocation - an existing key already lives in ISAM2,
// so it must not be re-inserted. Owned by the node, so its keys persist across
// processEpoch calls.
class PersistentAmbiguities {
 public:
  explicit PersistentAmbiguities(std::uint64_t& next_id) : next_id_(next_id) {}

  // Hold-refresh interval [s] (0 disables): re-key a carried ambiguity once its
  // arc reaches this age, dropping the hold on it so it must be re-fixed from a
  // fresh loose prior. This bounds how long an unverified held integer can
  // persist: a subtly-wrong hold otherwise biases the graph posterior, which
  // biases the AR prior, which lets the ratio test keep confirming the same
  // wrong integer (fix-and-hold lock-in). By the refresh age sigma(N) is far
  // below what AR needs, so a correct arc re-locks within an epoch or two. The
  // interval is staggered per (sat,band) (+/-20%, deterministic) so arcs do not
  // all re-key on the same epoch.
  //
  // Applied to ALL arcs, not only held ones: the periodic re-key doubles as a
  // fading memory on the per-pair factors' accumulated optimism and on
  // unmodelled time-correlated errors (multipath).
  void setHoldRefresh(double seconds) { hold_refresh_s_ = seconds; }
  void setRefreshHeldOnly(bool held_only) { refresh_held_only_ = held_only; }

  // (sat,band) pairs currently constrained by a hold factor; the age cap above
  // applies only to these. The caller refreshes this after every applyHolds.
  void setHeld(std::set<std::pair<int, int>> held) { held_ = std::move(held); }

  void resetAll() {
    entries_.clear();
    handled_.clear();
    fresh_.clear();
    anchored_.clear();
    held_.clear();
  }

  // --- Gauge bookkeeping ------------------------------------------------
  // A carrier DD group constrains only DIFFERENCES of its ambiguities: m rows
  // over m+1 keys, so its information matrix has a one-dimensional null space
  // (the common gauge) that exactly one loose prior must pin. `anchored_` is
  // the set of keys currently carrying such a prior.
  //
  // The gauge is enforced at factor construction (addGroupedDdFactorsImpl), not
  // inside obtain(): "does any entry belong to this group?" is evaluated BEFORE
  // a re-key replaces the entry, so a member being re-keyed would see its own
  // stale entry, leave the prior on the abandoned key and leave every live key
  // ungauged.
  bool isAnchored(gtsam::Key k) const { return anchored_.count(k) != 0; }
  void markAnchored(gtsam::Key k) { anchored_.insert(k); }

  // Call once at the start of each epoch's DD loop: the re-key decision for a
  // (sat,band) is then made at most ONCE per epoch. The reference satellite
  // appears in several DDs of its group (all carrying its slip flag), so without
  // this guard a slipped reference would allocate a fresh key on every one of
  // those DDs - breaking the "one (sat,band) = one ambiguity" invariant.
  void beginEpoch() {
    handled_.clear();
    fresh_.clear();
  }

  // Drop ambiguities not seen within max_outage_s of `tow` (their carrier can no
  // longer be assumed continuous), so they re-key with a fresh loose prior.
  void retireStale(double tow, double max_outage_s) {
    for (auto it = entries_.begin(); it != entries_.end();) {
      if (tow - it->second.last_tow > max_outage_s) {
        anchored_.erase(it->second.key);  // its gauge dies with it
        it = entries_.erase(it);
      } else {
        ++it;
      }
    }
  }

  // Force a re-key of (sat,band) on its next obtain (used when a same-generation
  // integer mismatch flags the satellite as suspect).
  void invalidate(int sat, int band) {
    const auto it = entries_.find(std::make_pair(sat, band));
    if (it == entries_.end()) return;
    anchored_.erase(it->second.key);
    entries_.erase(it);
  }

  // Force a re-key of EVERY carried arc on the next obtain. Used when the node
  // re-anchors its state (see the re-anchor path in gnss_imu_fgo): a carried
  // ambiguity absorbed whatever position error the graph had, so after the
  // position moves metres the float values are stale by tens of cycles and
  // would drag the re-anchored solution back. Unlike resetAll() this keeps the
  // held-set bookkeeping, so the next applyHolds() sees the keys change and
  // removes the corresponding hold factors itself.
  void invalidateAll() {
    entries_.clear();
    anchored_.clear();
  }

  // CMC supplies the nonlinear initial value only. The gauge prior is NOT added
  // here - see the gauge bookkeeping above and addGroupedDdFactorsImpl, which
  // owns it because only there is the group's complete key set known.
  // The re-key decision is taken only on the FIRST call for this (sat,band) this
  // epoch (see beginEpoch); later calls return the same key regardless of slip.
  gtsam::Key obtain(int sat, int band, double n0, bool slip, double tow,
                    gtsam::Values& values,
                    const AmbiguityGroupId& group = AmbiguityGroupId{}) {
    const auto id = std::make_pair(sat, band);
    if (handled_.count(id)) return entries_[id].key;
    auto it = entries_.find(id);
    bool aged = false;
    if (it != entries_.end() && hold_refresh_s_ > 0.0 &&
        (!refresh_held_only_ || held_.count(id))) {
      // Staggered age cap (scope per setRefreshHeldOnly).
      const double jitter =
          0.8 + 0.4 * ((static_cast<unsigned>(sat) * 2654435761u + band) %
                       1000u) / 1000.0;
      aged = (tow - it->second.first_tow) > hold_refresh_s_ * jitter;
    }
    gtsam::Key key;
    if (it != entries_.end() && !slip && !aged) {
      it->second.last_tow = tow;
      ++it->second.lock;  // one more epoch of continuous carrier tracking
      key = it->second.key;
    } else {
      if (it != entries_.end()) {
        ++total_rekeys_;  // slip re-key (not first-seen)
        // The abandoned key can no longer gauge anything: nothing will reference
        // it again, and the fixed-lag window will marginalize it out.
        anchored_.erase(it->second.key);
      }
      key = gtsam::symbol_shorthand::N(next_id_++);
      entries_[id] = Entry{key, tow, tow, 0, group};
      values.insert(key, n0);
      // No prior here. The gauge is fixed per DD group at factor construction
      // (addGroupedDdFactorsImpl), where the group's full key set is known.
      fresh_.insert(id);
    }
    handled_.insert(id);
    return key;
  }

  // Current key for (sat,band), if one is live this session.
  bool tryKey(int sat, int band, gtsam::Key& out) const {
    const auto it = entries_.find(std::make_pair(sat, band));
    if (it == entries_.end()) return false;
    out = it->second.key;
    return true;
  }

  // (Re)allocated in the current epoch (between beginEpoch calls)?
  bool isFresh(int sat, int band) const {
    return fresh_.count(std::make_pair(sat, band)) != 0;
  }

  // Consecutive epochs (sat,band) has been carried without a re-key (RTKLIB
  // lock count); 0 for a first-seen / just-re-keyed arc.
  int lockCount(int sat, int band) const {
    const auto it = entries_.find(std::make_pair(sat, band));
    return it == entries_.end() ? 0 : it->second.lock;
  }

  // Every live ambiguity key. The fixed-lag smoother marginalizes a variable
  // once its timestamp ages out of the window, so the node refreshes each live
  // key's timestamp every epoch (even one not observed this epoch but still
  // carried / held) to keep the carried arc - and any hold on it - inside the
  // window until it is genuinely retired.
  std::vector<gtsam::Key> liveKeys() const {
    std::vector<gtsam::Key> out;
    out.reserve(entries_.size());
    for (const auto& kv : entries_) out.push_back(kv.second.key);
    return out;
  }

  // Diagnostics: carried-arc health (the float can only reach carrier quality
  // when arcs live long enough for the ambiguity information to accumulate).
  std::size_t liveCount() const { return entries_.size(); }
  std::size_t freshCount() const { return fresh_.size(); }
  std::uint64_t totalRekeys() const { return total_rekeys_; }
  // Mean age [s] of the live arcs at continuous-GPST time `tow`.
  double meanArcAge(double tow) const {
    if (entries_.empty()) return 0.0;
    double sum = 0.0;
    for (const auto& kv : entries_) sum += tow - kv.second.first_tow;
    return sum / static_cast<double>(entries_.size());
  }

 private:
  struct Entry {
    gtsam::Key key{0};
    double last_tow{0.0};
    double first_tow{0.0};  // arc start (diagnostics)
    int lock{0};            // consecutive carried epochs (RTKLIB lock count)
    AmbiguityGroupId group{};
  };
  std::uint64_t& next_id_;
  std::map<std::pair<int, int>, Entry> entries_;
  std::set<std::pair<int, int>> handled_;  // (sat,band) decided this epoch
  std::set<std::pair<int, int>> fresh_;    // (sat,band) (re)allocated this epoch
  std::set<gtsam::Key> anchored_;          // keys carrying a gauge prior
  std::set<std::pair<int, int>> held_;     // (sat,band) with a live hold factor
  bool refresh_held_only_{false};          // cap scope (see setRefreshHeldOnly)
  std::uint64_t total_rekeys_{0};          // slip/outage re-keys (diagnostics)
  double hold_refresh_s_{0.0};                  // 0 = no age cap (see setHoldRefresh)
};



// Why an epoch did not end in a FIX. resolveAmbiguitiesPosterior has a dozen
// early returns; without recording which one fired, "candidate=0" and
// "candidate=1,fixed=0" are the only observable outcomes and they lump together
// causes with completely different remedies (too few usable carriers vs. an
// unconverged float vs. a numerically bad Qa vs. a rejected post-fix residual).
enum class ArFail {
  None,                 // fixed
  NoPairs,              // nothing reached the resolver
  TooFewEligible,       // half-cycle / lock / missing key left < min_fix
  PosVarGate,           // ambiguity.max_pos_var_m2: float has not converged
  ElevMaskTooFew,       // AR elevation mask left < min_fix
  LambdaNumeric,        // lambda() failed or s[1] <= 0
  QaNotPd,              // Qa or the Schur complement not positive definite
  RatioBelowThreshold,  // the ordinary "not decisive enough" outcome
  ConditioningFailed,   // full-state integer conditioning was not finite
  FdeReject,            // post-fix correlated residual test
  CodeGrowthReject      // ambiguity.max_code_resid_growth_m
};

struct ArResult {
  bool fixed{false};
  // Why not fixed (ArFail::None when fixed). See the enum above.
  ArFail fail{ArFail::NoPairs};
  // Diagnostics recorded next to the decision (0 when not reached).
  int n_eligible{0};        // pairs that survived half-cycle/lock/key filtering
  int n_excl_half{0};       // dropped: half-cycle flag
  int n_excl_lock{0};       // dropped: lock < min_lock
  // Code-DD residual RMS growth from the float to the fixed position [m], and
  // the number of code DDs it was formed from. Recorded on every fix; acting on
  // it is CodeGrowthMonitor's job (a single epoch of disagreement is a false
  // alarm, a sustained one is the runaway signature).
  double code_resid_growth{0.0};
  int n_code_dd{0};
  // Absolute code DD residual RMS at the FLOAT position [m]. Says whether the
  // state the integers were resolved against is itself in the wrong place, which
  // code_resid_growth cannot see. Recorded on every epoch with a valid float.
  double code_resid_rms{0.0};
  int n_excl_nokey{0};      // dropped: ambiguity key absent from the posterior
  int n_partial_dropped{0}; // pairs removed by partial AR before success/give-up
  double pos_var{0.0};      // the value compared against max_pos_var_m2 [m^2]
  // LAMBDA squared residuals of the best and second-best integer candidates.
  // ratio = s1/s0, so these separate the two ways a ratio test can fail:
  // s0 large  -> the float sits far from ANY lattice point (float quality),
  // s1 small  -> a competing candidate is nearly as good (geometry ambiguity).
  double s0{0.0}, s1{0.0};
  // A best integer candidate was found and the state conditioned on it (set even
  // when the ratio test fails, so post-fix residual FDE can vet the candidate).
  bool candidate{false};
  double ratio{0.0};
  // Antenna-space correction to subtract and conditional antenna covariance.
  Eigen::VectorXd state_correction;
  Eigen::MatrixXd state_cov;
  // Correction to ADD to the complete graph state. Point mode is [position(3)];
  // IMU mode is [Pose3 tangent(6), velocity(3), accel/gyro bias(6)]. This comes
  // directly from Qza Qa^-1 and must not be reconstructed from fixed_pos.
  Eigen::VectorXd full_state_delta;
  Eigen::MatrixXd full_state_cov;
  Eigen::VectorXd a_fix;    // fixed DD ambiguities [cycles], in `pairs` order
                            // (NaN for pairs dropped by partial AR)
  // ep.dd index of each a_fix / a_float entry; maps a fixed DD integer back to
  // its (sat_ref,sat_tar,band) for fix-and-hold.
  std::vector<int> cp_dd_index;
  Eigen::VectorXd a_float;  // float DD ambiguities [cycles] (always populated)
  Eigen::MatrixXd Qa;       // float DD ambiguity covariance [cycles^2]
  std::vector<int> fixed_idx;  // indices into the DD list that were fixed
  // Conditioned (fixed) antenna ECEF position [m], set by
  // resolveAmbiguitiesPosterior from the stage-2 joint graph posterior.
  Eigen::Vector3d fixed_pos{Eigen::Vector3d::Zero()};
  // The stage-2 graph FLOAT antenna solution used by the integer search.
  // Paired with a_float/Qa this is the complete pre-LAMBDA state, which is what an
  // offline study needs to judge whether a wrong fix came from a biased float
  // or from an over-confident covariance. Populated whenever the update solved.
  bool float_valid{false};
  Eigen::Vector3d float_ant_pos{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d float_ant_cov{Eigen::Matrix3d::Zero()};
  // Post-fit validation statistic, recorded whether or not it rejected. Dumped
  // per epoch so the test can be scored against truth OFFLINE - the alternative
  // is re-running the whole course for every threshold, which is how earlier
  // candidate statistics were adopted before they were known to work.
  double postfit_chi2{0.0};
  int postfit_dof{0};
  double postfit_max_standard{0.0};
};

// Integer ambiguity resolution: LAMBDA integer least squares (Teunissen) on the
// float DD ambiguities `a` with covariance `Qa`, then the state is conditioned
// through `Qxa`/`Qxx`. `el[i]` is the target elevation [rad] of DD i; pairs
// below ar_el_mask_rad are excluded up front. Canonical LAMBDA + ratio test
// (matches RTKLIB resamb_LAMBDA).
//
// The BEST integer candidate is always sought and the state conditioned on it
// (res.candidate), so the caller's residual-based FDE can vet the candidate even
// when the ratio fails; res.fixed is set only when the ratio test passes. All
// covariances are symmetry- / positive-definiteness-guarded before conditioning,
// so a numerically bad candidate is never produced. Fills
// res.{ratio, candidate, fixed, state_correction, state_cov, a_fix, fixed_idx}.
namespace detail {
inline void lambdaFix(const Eigen::VectorXd& a, const Eigen::MatrixXd& Qa,
                      const Eigen::MatrixXd& Qxa, const Eigen::MatrixXd& Qxx,
                      const std::vector<double>& el, double ratio_threshold,
                      double ar_el_mask_rad, int min_fix, ArResult& res) {
  const int n_dd = static_cast<int>(a.size());
  res.a_fix = Eigen::VectorXd::Constant(
      n_dd, std::numeric_limits<double>::quiet_NaN());
  std::vector<int> active;
  active.reserve(n_dd);
  for (int i = 0; i < n_dd; ++i) {
    if (el[i] >= ar_el_mask_rad) active.push_back(i);
  }
  const int k = static_cast<int>(active.size());
  if (k < std::max(min_fix, 1)) {
    res.fail = ArFail::ElevMaskTooFew;
    return;
  }

  Eigen::VectorXd a_sub(k);
  Eigen::MatrixXd Qa_sub(k, k);
  Eigen::MatrixXd Qxa_sub(Qxa.rows(), k);
  for (int i = 0; i < k; ++i) {
    a_sub(i) = a(active[i]);
    Qxa_sub.col(i) = Qxa.col(active[i]);
    for (int j = 0; j < k; ++j) Qa_sub(i, j) = Qa(active[i], active[j]);
  }

  // Full-set LAMBDA (matrices are column-major, like Eigen's; s[0]/s[1] are the
  // squared residuals of the best/second-best candidates).
  Eigen::MatrixXd F(k, 2);
  double s[2] = {0.0, 0.0};
  if (lambda(k, 2, a_sub.data(), Qa_sub.data(), F.data(), s) != 0 || s[1] <= 0.0) {
    res.fail = ArFail::LambdaNumeric;
    res.s0 = s[0];
    res.s1 = s[1];
    return;
  }
  res.s0 = s[0];
  res.s1 = s[1];
  res.ratio = s[1] / std::max(s[0], 1e-12);

  // Condition the state on the best integer candidate:
  //   dx     = Qxa * Qa^-1 (a - a_fix)
  //   Qstate = Qxx - Qxa * Qa^-1 Qxa^T   (Schur complement)
  // guarding symmetry and positive-definiteness of both Qa and the result.
  const Eigen::VectorXd a_fix_sub = F.col(0);
  const Eigen::LDLT<Eigen::MatrixXd> solver = Qa_sub.ldlt();
  if (solver.info() != Eigen::Success || solver.vectorD().minCoeff() <= 0.0) {
    res.fail = ArFail::QaNotPd;
    return;
  }
  Eigen::MatrixXd S = Qxx - Qxa_sub * solver.solve(Qxa_sub.transpose());
  S = 0.5 * (S + S.transpose());
  const Eigen::LDLT<Eigen::MatrixXd> s_ldlt(S);
  if (!S.allFinite() || s_ldlt.info() != Eigen::Success ||
      s_ldlt.vectorD().minCoeff() <= 0.0) {
    res.fail = ArFail::QaNotPd;
    return;
  }
  const Eigen::VectorXd dc = Qxa_sub * solver.solve(a_sub - a_fix_sub);
  if (!dc.allFinite()) {
    res.fail = ArFail::ConditioningFailed;
    return;
  }
  res.state_correction = dc;
  res.state_cov = S;
  for (int i = 0; i < k; ++i) {
    res.a_fix(active[i]) = a_fix_sub(i);
    res.fixed_idx.push_back(active[i]);
  }
  res.candidate = true;
  res.fixed = (res.ratio >= ratio_threshold);
  res.fail = res.fixed ? ArFail::None : ArFail::RatioBelowThreshold;
}
}  // namespace detail

// ---------------------------------------------------------------------------
// Ambiguity resolution on the graph posterior (RTKLIB resamb_LAMBDA equivalent).
//
// GroupedDdFactor, GraphArPosterior and resolveAmbiguitiesPosterior make graph
// optimization, LAMBDA and full-state conditioning consume one posterior.

// Ambiguity-resolution options shared by both nodes.
struct ArOptions {
  double ratio_threshold{3.0};
  double el_mask_rad{15.0 * D2R};
  int min_fix{5};
  // Fault detection and exclusion: the pre-fit DD innovation test and the
  // post-fit FIX validation. Not a node parameter - measured to cost 15-32
  // points of p05 and up to 4.4x the false fixes when off. Kept as a struct
  // field only so a unit test can isolate the integer-conditioning path.
  bool fde_enable{true};
  // Gate for the correlated pre-fit innovation and post-fix residual tests.
  // Both tests use complete shared-reference/state covariance.
  double fde_nsigma{4.0};
  // Exclusion budget for the PRE-FIT test. When more than this many DDs would
  // have to be removed, the epoch is judged dirty and - unless the a-priori
  // retest rescues it - EVERY DD is discarded.
  //
  // The budget is small relative to the observation count and the cost is
  // large: exhausting it discards the epoch's ENTIRE DD set, which on an urban
  // course happens on several percent of epochs and is the single biggest
  // source of discarded measurement in either node. It applies to both almost
  // equally, so it does not explain a difference between them - but it does
  // bound how well either can do.
  //
  // Exposed as a node parameter so "is this test earning its cost, and is 2
  // the right budget?" can be answered by measurement rather than argument.
  int fde_max_exclude{2};
  // Partial AR: when the full-set ratio test fails, retry the integer search
  // on subsets, dropping the weakest ambiguities (newest arcs, then lowest
  // elevation) one at a time. Dropped pairs stay in the float solution.
  bool partial_ar{true};
  int partial_max_drop{5};
  // Minimum lock count (RTKLIB arlockcnt): an ambiguity must have been carried
  // this many consecutive epochs before it may enter the integer search. A
  // brand-new / just-re-keyed arc carries almost no information, so fixing on it
  // produces confident-but-wrong integers (observed as an early false fix that
  // then poisons the graph via holds). Sub-threshold pairs stay in the float.
  int min_lock{5};
  // Reject the fix when the AR correction makes the CODE double differences
  // measurably worse: RMS of the code DD residual re-evaluated at `fixed_pos`
  // minus the same RMS at the float position, in metres. <= 0 disables.
  //
  // This is a DIFFERENTIAL test and that is the whole point. An absolute code
  // residual is useless in an urban canyon - multipath puts metres of common
  // bias on it - but that bias is shared by both positions and cancels in the
  // difference, leaving only the code's opinion of the increment AR just
  // applied. It is also the only statistic here that a wrong integer cannot
  // hide in: the carrier ambiguity absorbs a state error by construction, the
  // pseudorange has no ambiguity to absorb it with.
  //
  // Three cheaper statistics do NOT work here: |fix - float| instantaneous and
  // windowed (see the gate above), and the post-fit CARRIER residual, whose
  // sense is INVERTED - a residual smaller than the measurement noise marks the
  // over-fit that a wrong integer produces, so small looks good and is not.
  //
  // This statistic is monotone: the false-fix rate of ratio-passing candidates
  // rises steeply once the absolute code DD RMS passes ~0.1 m.
  //
  // Shipped at 0.08 m with code_resid_persist = 5: on a harsh course that
  // removes about a quarter of the false fixes and nearly halves the FIX 3D
  // p95, for a few tenths of a point of fix rate, and it is inert on a benign
  // one. A LARGER threshold trades worse, not better: the state stops drifting
  // the moment a fix is refused, so the statistic never reaches the higher
  // value and the gate simply stops acting.
  double max_code_resid_growth_m{0.08};
  // Minimum code DDs required before the test above can reject. Below this the
  // RMS is too noisy to act on, so the epoch is passed through ungated.
  int code_resid_min_dd{6};
  // Consecutive epochs of code disagreement required before the fix is
  // rejected. 1 = react immediately, and on a benign course almost everything
  // it fires on is a false alarm. Requiring 5 cuts that cost by ~3x while
  // keeping most of the benefit where runaways actually occur, because a
  // runaway disagrees for its whole duration and a false alarm does not.
  int code_resid_persist{5};
  // Absolute code DD residual RMS at the float [m] above which the FLOAT itself
  // is judged to be in the wrong place. <= 0 disables. Acting on it must also
  // drop the carried ambiguities - they are what put the state there, so
  // refusing the fix alone would leave the float exactly as wrong.
  //
  // 2.5 m is measured to be free: across four full-course configurations it
  // costs 0-2 good fixes in total while removing 25-49 % of the false fixes on
  // the harsh course. 2.0 m removes 47-63 % instead but costs 1.2-2.0 % of the
  // good fixes on the benign one.
  double max_code_resid_m{2.5};
  int code_resid_rms_persist{3};
  // Skip AR entirely when the FLOAT position variance exceeds this [m^2],
  // averaged over the three ECEF axes (RTKLIB pos2-arthres1 / thresar[1],
  // default 0.25 m^2 = 0.5 m 1-sigma; see manage_amb_LAMBDA in rtkpos.c, whose
  // own comment reads "will skip AR if too high to avoid false fix").
  // Rationale: LAMBDA conditions the integers on the float. An unconverged
  // float does not merely widen the search - it biases it, and the ratio test
  // then compares two candidates that are both referred to the same wrong
  // place, so a wrong integer set can be self-consistent and pass. No integer
  // decision made from such a float is trustworthy, however good the ratio. 0
  // disables the gate.
  double max_pos_var_m2{0.25};
};

// Upper-tail chi-square critical value at a normal-deviate threshold, via the
// Wilson-Hilferty transform: (X/k)^(1/3) is approximately normal with mean
// 1 - 2/(9k) and variance 2/(9k), so
//     chi2_crit(k, z) = k * (1 - 2/(9k) + z*sqrt(2/(9k)))^3.
// Accurate to better than 1 % of the true quantile for k >= 3 over the range
// of z this code uses, and it needs no special-function library.
//
// Why this replaces `chi2/n > nsigma^2`: that comparison has no distribution
// behind it. A reduced chi-square is ~1 in expectation whatever k is, but its
// SPREAD shrinks as 1/sqrt(k), so a fixed multiple of it is a wildly different
// significance level for 5 rows than for 40 - lenient where there is the most
// evidence, strict where there is the least. Expressing the threshold as a
// quantile keeps one meaning of `fde_nsigma` across every epoch geometry.
inline double chi2CriticalValue(int dof, double z) {
  if (dof <= 0) return 0.0;
  const double k = static_cast<double>(dof);
  const double a = 2.0 / (9.0 * k);
  const double t = 1.0 - a + z * std::sqrt(a);
  return t <= 0.0 ? 0.0 : k * t * t * t;
}

// Outcome of one pre-fit innovation test (see ddPreFitTest).
struct PreFitTestResult {
  bool valid{false};        // the test itself was numerically well-posed
  bool failed{false};       // the global chi^2 / max-standardized test failed
  int worst{-1};            // ep.dd index of the largest standardized residual
  double chi2_per_n{0.0};   // reduced chi^2 of the tested subset
  double max_standard{0.0}; // largest |innovation| / sqrt(S_ii)
};

// One correlated pre-fit innovation test of the code DDs in `active` (indices
// into ep.dd) evaluated at `ref_pos`, with the shared-reference R plus the
// reference position's own covariance.
inline PreFitTestResult ddPreFitTest(const gnss_utils::PreprocessedEpoch& ep,
                                     const std::vector<int>& active,
                                     const Eigen::Vector3d& ref_pos,
                                     const Eigen::Matrix3d& ref_cov,
                                     const AdapterConfig& cfg,
                                     const ArOptions& opt) {
  PreFitTestResult out;
  const int n = static_cast<int>(active.size());
  if (n == 0) return out;

  Eigen::VectorXd innovation(n);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n, 3);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(n, n);
  for (int i = 0; i < n; ++i) {
    const auto& d = ep.dd[active[i]];
    Eigen::Vector3d er, et;
    detail::geodistSagnac(d.sat_ref_rov, ref_pos, &er);
    detail::geodistSagnac(d.sat_tar_rov, ref_pos, &et);
    H.row(i) = (et - er).transpose();
    innovation(i) = detail::ddPseudorangeResidual(d, ep.base_ecef, ref_pos);
    R(i, i) = ddSingleDifferenceVar(d, true, cfg.pr_sigma_m, cfg) +
              ddSingleDifferenceVar(d, false, cfg.pr_sigma_m, cfg);
    for (int j = 0; j < i; ++j) {
      const auto& q = ep.dd[active[j]];
      if (d.sys == q.sys && d.band == q.band && d.code_ref == q.code_ref &&
          d.code_base_ref == q.code_base_ref) {
        R(i, j) = R(j, i) = ddSingleDifferenceVar(d, true, cfg.pr_sigma_m, cfg);
      }
    }
  }
  Eigen::MatrixXd S = R + H * ref_cov * H.transpose();
  S = 0.5 * (S + S.transpose());
  const Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
  if (ldlt.info() != Eigen::Success || ldlt.vectorD().minCoeff() <= 0.0)
    return out;  // do not invent exclusions from a numerically invalid test
  out.valid = true;
  const double chi2 = innovation.dot(ldlt.solve(innovation));
  for (int i = 0; i < n; ++i) {
    const double z =
        std::fabs(innovation(i)) / std::sqrt(std::max(S(i, i), 1e-12));
    if (z > out.max_standard) {
      out.max_standard = z;
      out.worst = active[i];
    }
  }
  out.chi2_per_n = chi2 / std::max(n, 1);
  // Global test against a chi-square QUANTILE at n degrees of freedom, not
  // against chi2/n > nsigma^2. The reduced form does not scale with n, so it
  // grows steadily more permissive as satellites are added: at n = 20 and
  // nsigma = 4 it only fires past chi2 = 320, where the quantile is ~45 - about
  // seven times too loose. That gap is precisely where several MODERATE faults
  // hide, each too small for the per-row w-test above but jointly decisive, and
  // several moderate faults is what an urban multipath epoch actually looks
  // like. chi2_per_n is kept because the AR dump columns report it.
  out.failed = !std::isfinite(chi2) || out.max_standard > opt.fde_nsigma ||
               chi2 > chi2CriticalValue(n, opt.fde_nsigma);
  return out;
}

// Pre-fit global innovation test on code DDs at the motion/IMU prediction.
// Shared-reference R and the full predicted antenna covariance are retained.
// Failed tests exclude the worst row and recompute the complete subset, up to
// fde_max_exclude. The returned epoch is the sole observation set consumed by
// both grouped graph factors and posterior AR.
//
// Predicted-state fault vs. dirty observations. A global failure has two very
// different causes and they MUST be told apart:
//   - a few bad DDs (NLOS/multipath) -> exclude them, keep the rest;
//   - the PREDICTION itself is wrong (the state ran away during a GNSS outage,
//     or its covariance is over-confident) -> every innovation is off by the
//     same state error and no exclusion budget can fix it.
// Rejecting the whole epoch in the second case removes the only measurement
// that could correct the state, so the next epoch predicts from the same wrong
// state and rejects again: a self-locking failure that was measured to kill
// ambiguity tracking for 519 s (40 % of a run) in gnss_imu_fgo. The budget-
// exhausted branch therefore re-runs the SAME test against a reference that is
// independent of the graph - this epoch's code (single-point) a-priori. If the
// DDs are self-consistent there, they are good data seen from a bad state:
// keep them all and report `prediction_fault` so the caller can re-anchor.
inline gnss_utils::PreprocessedEpoch filterDdPreFitInnovation(
    const gnss_utils::PreprocessedEpoch& ep,
    const Eigen::Vector3d& predicted_ant,
    const Eigen::Matrix3d& predicted_cov, const AdapterConfig& cfg,
    const ArOptions& opt, int* excluded_count = nullptr,
    bool* prediction_fault = nullptr) {
  if (prediction_fault) *prediction_fault = false;
  if (!opt.fde_enable || ep.dd.empty()) return ep;
  std::set<int> excluded;
  for (;;) {
    std::vector<int> active;
    for (int i = 0; i < static_cast<int>(ep.dd.size()); ++i)
      if (ep.dd[i].has_pr && !excluded.count(i)) active.push_back(i);
    if (active.empty()) break;

    const PreFitTestResult t =
        ddPreFitTest(ep, active, predicted_ant, predicted_cov, cfg, opt);
    if (!t.valid || !t.failed) break;
    if (t.worst >= 0 &&
        static_cast<int>(excluded.size()) < opt.fde_max_exclude) {
      excluded.insert(t.worst);
      continue;
    }

    // Budget exhausted. Re-test at the code a-priori with a loose
    // (single-point-level) covariance. The whole ORIGINAL set is retested, not
    // just the survivors: if the prediction is the faulty side then the
    // exclusions made above were charged to the wrong account and must be
    // refunded. Passing there means the observations agree with each other and
    // only disagree with the predicted STATE.
    //
    // The reference MUST be one the estimator cannot move; see
    // independentCodePosition, which prefers the code-DD WLS, then SPP, and
    // only falls back to the a-priori hint (a copy of the previous graph float)
    // when neither is available.
    const IndependentCodePosition anchor =
        independentCodePosition(ep, cfg);
    if (anchor.ok) {
      std::vector<int> all;
      for (int i = 0; i < static_cast<int>(ep.dd.size()); ++i)
        if (ep.dd[i].has_pr) all.push_back(i);
      const PreFitTestResult a =
          ddPreFitTest(ep, all, anchor.pos, anchor.cov, cfg, opt);
      if (a.valid && !a.failed) {
        if (prediction_fault) *prediction_fault = true;
        excluded.clear();
        break;  // keep every DD; the state, not the data, is what is wrong
      }
    }
    // Genuinely dirty (or no usable independent reference): reject the epoch
    // rather than feed a known-bad subset into both the graph and AR.
    excluded.clear();
    for (int i = 0; i < static_cast<int>(ep.dd.size()); ++i) excluded.insert(i);
    break;
  }

  gnss_utils::PreprocessedEpoch filtered = ep;
  filtered.dd.clear();
  for (int i = 0; i < static_cast<int>(ep.dd.size()); ++i)
    if (!excluded.count(i)) filtered.dd.push_back(ep.dd[i]);
  if (excluded_count) *excluded_count += static_cast<int>(excluded.size());
  return filtered;
}


// Fix-and-hold: hold each analytically-fixed DD as a TWO-variable constraint
// N_min - N_max = integer on the carried ambiguity keys (a BetweenFactor,
// gauge-free and independent of the absolute ambiguity level). Satellites of a
// pair are normalized to (min,max) so (A,B) and (B,A) map to one hold, and the
// GTSAM key order and integer sign are normalized together (BetweenFactor<double>
// measures N_max - N_min, so the measurement is -integer).

// One accepted fix to hold this epoch (normalized).
struct HoldSpec {
  int band{0};
  int sat_min{0}, sat_max{0};
  gtsam::Key k_min{0}, k_max{0};
  long integer{0};  // N_min - N_max
};

// One currently-held DD (its single BetweenFactor's ISAM2 index).
struct HeldDd {
  gtsam::Key k_min{0}, k_max{0};
  long integer{0};
  std::size_t factor_index{0};
};

// key: (sat_min, sat_max, band)
using HeldDdMap = std::map<std::tuple<int, int, int>, HeldDd>;

struct HoldCandidate {
  gtsam::Key k_min{0}, k_max{0};
  long integer{0};
  int consecutive{0};
};
using HoldCandidateMap =
    std::map<std::tuple<int, int, int>, HoldCandidate>;

// Require the same integer on the same ambiguity generation for each pair on
// consecutive FIX epochs. Missing, changed or re-keyed pairs restart at one;
// a global consecutive-FIX counter cannot certify a newly appeared pair.
inline std::vector<HoldSpec> confirmHoldSpecs(
    HoldCandidateMap& candidates, const std::vector<HoldSpec>& observed,
    int required) {
  using Id = std::tuple<int, int, int>;
  std::set<Id> seen;
  std::vector<HoldSpec> confirmed;
  for (const auto& s : observed) {
    const Id id{s.sat_min, s.sat_max, s.band};
    seen.insert(id);
    auto& c = candidates[id];
    if (c.k_min == s.k_min && c.k_max == s.k_max &&
        c.integer == s.integer) {
      ++c.consecutive;
    } else {
      c = HoldCandidate{s.k_min, s.k_max, s.integer, 1};
    }
    if (c.consecutive >= std::max(required, 1)) confirmed.push_back(s);
  }
  for (auto it = candidates.begin(); it != candidates.end();) {
    if (!seen.count(it->first))
      it = candidates.erase(it);
    else
      ++it;
  }
  return confirmed;
}

// Outcome of applyHolds. On Failure the ISAM2 update threw: GTSAM gives no strong
// rollback guarantee, so its internal graph may be partially modified and no
// longer matches `held` - the caller MUST reset the graph rather than continue.
enum class HoldResult { NoChange, Success, Failure };

// Turn the analytical AR result into normalized hold specs on the current keys.
// A DD whose FLOAT value rounds to a DIFFERENT integer than LAMBDA chose
// (|a_float - a_fix| > 0.5 cycle) is not held: such a candidate rests entirely
// on the decorrelation search, and injecting it as a 0.03-cycle hold is how
// one wrong integer corrupts the graph. (A tighter gate measurably starves
// fix-and-hold - most legitimate holds sit 0.1-0.4 cycles from their float.)
inline std::vector<HoldSpec> collectHoldSpecs(
    const PersistentAmbiguities& mgr, const gnss_utils::PreprocessedEpoch& ep,
    const ArResult& res, double max_frac_cycles = 0.5,
    int min_lock_epochs = 0) {
  std::vector<HoldSpec> out;
  if (!res.fixed) return out;
  for (int i = 0; i < static_cast<int>(res.cp_dd_index.size()); ++i) {
    if (i >= static_cast<int>(res.a_fix.size()) || !std::isfinite(res.a_fix(i)))
      continue;
    if (i < static_cast<int>(res.a_float.size()) &&
        std::fabs(res.a_float(i) - res.a_fix(i)) > max_frac_cycles)
      continue;
    const int di = res.cp_dd_index[i];
    if (di < 0 || di >= static_cast<int>(ep.dd.size())) continue;
    const gnss_utils::DdSignal& d = ep.dd[di];
    // A global consecutive-FIX counter is not sufficient: a newly appeared pair
    // could otherwise be held immediately because older satellites were already
    // fixed for many epochs. Require this pair's own continuous carrier history.
    if (mgr.lockCount(d.sat_ref, d.band) < min_lock_epochs ||
        mgr.lockCount(d.sat_tar, d.band) < min_lock_epochs)
      continue;
    const long n = std::lround(res.a_fix(i));  // N_ref - N_tar
    HoldSpec s;
    s.band = d.band;
    if (d.sat_ref < d.sat_tar) {
      s.sat_min = d.sat_ref;
      s.sat_max = d.sat_tar;
      s.integer = n;  // N_min - N_max = N_ref - N_tar
    } else {
      s.sat_min = d.sat_tar;
      s.sat_max = d.sat_ref;
      s.integer = -n;
    }
    if (!mgr.tryKey(s.sat_min, s.band, s.k_min) ||
        !mgr.tryKey(s.sat_max, s.band, s.k_max))
      continue;
    out.push_back(s);
  }
  return out;
}

// Incrementally reconcile the held set: remove holds whose satellite was
// re-keyed (slip/outage), add a factor for each newly-fixed pair ONCE (survivors
// keep their existing factor - never re-added, so constraints do not stack), and
// reject a same-generation integer mismatch (an anomaly: drop the hold and
// re-key both satellites). All map/index state is committed only after a
// successful smoother update, so a failed update leaves `held` untouched. Runs
// every epoch (even with no new fix) so stale holds are always cleaned up.
// The hold factors touch only live (timestamp-refreshed) ambiguity keys, so the
// smoother never marginalizes them and their factor indices stay valid.
inline HoldResult applyHolds(gtsam::IncrementalFixedLagSmoother& smoother,
                             PersistentAmbiguities& mgr, HeldDdMap& held,
                             const std::vector<HoldSpec>& specs,
                             double hold_sigma) {
  using Id = std::tuple<int, int, int>;
  const auto tight = gtsam::noiseModel::Isotropic::Sigma(1, hold_sigma);
  gtsam::FactorIndices remove;
  std::set<Id> erase_ids;
  std::set<std::pair<int, int>> suspect;  // (sat,band) flagged by a mismatch

  // Step 1: drop holds whose current keys no longer match (re-keyed / gone).
  for (const auto& kv : held) {
    const int smin = std::get<0>(kv.first), smax = std::get<1>(kv.first),
              band = std::get<2>(kv.first);
    gtsam::Key cmin, cmax;
    if (!mgr.tryKey(smin, band, cmin) || !mgr.tryKey(smax, band, cmax) ||
        cmin != kv.second.k_min || cmax != kv.second.k_max) {
      remove.push_back(kv.second.factor_index);
      erase_ids.insert(kv.first);
    }
  }

  // Step 2: classify fixes and validate integer cycle closure before adding
  // any factor. An edge u->v stores N_u-N_v; an already-connected pair must
  // equal the integer implied by the existing path. A mismatch rolls back the
  // whole connected component, not just the newly observed edge.
  using Edge = std::pair<int, long>;  // neighbor, N_this-N_neighbor
  std::map<std::pair<int, int>, std::vector<Edge>> adjacency;  // (sat,band)
  auto add_edge = [&](int u, int v, int band, long integer) {
    adjacency[{u, band}].push_back({v, integer});
    adjacency[{v, band}].push_back({u, -integer});
  };
  for (const auto& kv : held) {
    if (erase_ids.count(kv.first)) continue;
    add_edge(std::get<0>(kv.first), std::get<1>(kv.first),
             std::get<2>(kv.first), kv.second.integer);
  }

  std::vector<HoldSpec> to_add;
  for (const auto& s : specs) {
    const Id id{s.sat_min, s.sat_max, s.band};
    const auto held_it = held.find(id);
    if (held_it != held.end() && erase_ids.count(id) == 0) {
      if (held_it->second.integer == s.integer) continue;
      suspect.insert({s.sat_min, s.band});
      suspect.insert({s.sat_max, s.band});
      continue;
    }

    std::map<int, long> implied;  // N_start-N_sat
    std::vector<int> stack{s.sat_min};
    implied[s.sat_min] = 0;
    while (!stack.empty()) {
      const int u = stack.back();
      stack.pop_back();
      for (const auto& edge : adjacency[{u, s.band}]) {
        if (implied.count(edge.first)) continue;
        implied[edge.first] = implied[u] + edge.second;
        stack.push_back(edge.first);
      }
    }
    const auto path = implied.find(s.sat_max);
    if (path != implied.end()) {
      if (path->second != s.integer) {
        for (const auto& node : implied)
          suspect.insert({node.first, s.band});
      }
      // Consistent cycles are redundant; inconsistent ones are rolled back.
      continue;
    }
    to_add.push_back(s);
    add_edge(s.sat_min, s.sat_max, s.band, s.integer);
  }

  // Step 3: on a mismatch, remove EVERY held DD touching a suspect satellite in
  // this same update (so no suspect constraint survives to the next epoch), and
  // schedule those satellites for re-key (committed only after success).
  auto touches = [&](int smin, int smax, int band) {
    return suspect.count({smin, band}) || suspect.count({smax, band});
  };
  if (!suspect.empty()) {
    for (const auto& kv : held) {
      if (erase_ids.count(kv.first)) continue;
      if (touches(std::get<0>(kv.first), std::get<1>(kv.first),
                  std::get<2>(kv.first))) {
        remove.push_back(kv.second.factor_index);
        erase_ids.insert(kv.first);
      }
    }
  }

  // Step 4: build the add graph, skipping any pair touching a suspect satellite.
  gtsam::NonlinearFactorGraph add_graph;
  std::vector<std::pair<Id, HeldDd>> pending_add;
  for (const auto& s : to_add) {
    if (touches(s.sat_min, s.sat_max, s.band)) continue;
    add_graph.add(gtsam::BetweenFactor<double>(
        s.k_min, s.k_max, static_cast<double>(-s.integer), tight));
    pending_add.push_back(
        {Id{s.sat_min, s.sat_max, s.band}, HeldDd{s.k_min, s.k_max, s.integer, 0}});
  }

  if (add_graph.empty() && remove.empty()) return HoldResult::NoChange;

  try {
    // Hold factors add no new variables, so no timestamps are needed; the keys
    // they touch are kept live by the caller's per-epoch timestamp refresh.
    smoother.update(add_graph, gtsam::Values(),
                    gtsam::FixedLagSmoother::KeyTimestampMap(), remove);
  } catch (const std::exception&) {
    // held map and manager are untouched, but the smoother itself may be
    // inconsistent -> signal the caller to reset the graph.
    return HoldResult::Failure;
  }
  // Commit held map, factor indices, and manager re-keys only after success.
  for (const auto& id : erase_ids) held.erase(id);
  const gtsam::FactorIndices& new_idx = smoother.getISAM2Result().newFactorsIndices;
  for (std::size_t j = 0; j < pending_add.size() && j < new_idx.size(); ++j) {
    pending_add[j].second.factor_index = new_idx[j];
    held[pending_add[j].first] = pending_add[j].second;
  }
  for (const auto& sb : suspect) mgr.invalidate(sb.first, sb.second);
  return HoldResult::Success;
}


// One row in a shared-reference DD factor. Code rows use only dd; carrier rows
// additionally use the two SD ambiguity keys.
struct GroupedDdRow {
  gnss_utils::DdSignal dd;
  gtsam::Key ref{0};
  gtsam::Key tar{0};
  int dd_index{-1};  // index into ep.dd, recorded into DdFactorLayout
};

// A vector-valued DD factor. All rows share the same reference observation and
// are whitened by the full RTKLIB-ddcov covariance, rather than by independent
// scalar models. State is Point3 for gnss_fgo or Pose3 for gnss_imu_fgo.
template <typename State>
class GroupedDdFactor : public gtsam::NoiseModelFactor {
 public:
  using This = GroupedDdFactor<State>;
  using Base = gtsam::NoiseModelFactor;
  using Base::unwhitenedError;

  GroupedDdFactor(gtsam::Key state_key, std::vector<GroupedDdRow> rows,
                  bool carrier, const gtsam::Point3& base,
                  const gtsam::Point3& lever, const gtsam::Pose3& ecef_T_nav,
                  const gtsam::SharedNoiseModel& model)
      : Base(model, makeKeys(state_key, rows, carrier)),
        rows_(std::move(rows)), carrier_(carrier), base_(base), lever_(lever),
        ecef_T_nav_(ecef_T_nav) {
    for (std::size_t i = 0; i < keys_.size(); ++i) key_col_[keys_[i]] = i;
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  gtsam::Vector unwhitenedError(
      const gtsam::Values& values,
      gtsam::OptionalMatrixVecType H = nullptr) const override {
    const int n = static_cast<int>(rows_.size());
    gtsam::Vector error(n);
    constexpr int state_dim = gtsam::traits<State>::dimension;
    Eigen::Matrix<double, 3, state_dim> J_ant;
    Eigen::Vector3d ant;
    if constexpr (std::is_same_v<State, gtsam::Point3>) {
      ant = values.at<gtsam::Point3>(keys_.front());
      J_ant.setIdentity();
    } else {
      const gtsam::Pose3 pose = values.at<gtsam::Pose3>(keys_.front());
      gtsam::Matrix36 H_pose;
      const gtsam::Point3 ant_nav = pose.transformFrom(lever_, H_pose);
      const gtsam::Point3 ant_ecef = ecef_T_nav_.transformFrom(ant_nav);
      ant = Eigen::Vector3d(ant_ecef);
      J_ant = ecef_T_nav_.rotation().matrix() * H_pose;
    }

    if (H) {
      H->resize(keys_.size());
      (*H)[0] = gtsam::Matrix::Zero(n, state_dim);
      for (std::size_t j = 1; j < keys_.size(); ++j)
        (*H)[j] = gtsam::Matrix::Zero(n, 1);
    }

    for (int i = 0; i < n; ++i) {
      const auto& row = rows_[i];
      Eigen::Vector3d e_ref, e_tar, tmp;
      const double model =
          (detail::geodistSagnac(row.dd.sat_ref_rov, ant, &e_ref) -
           detail::geodistSagnac(row.dd.sat_ref_base, base_, &tmp)) -
          (detail::geodistSagnac(row.dd.sat_tar_rov, ant, &e_tar) -
           detail::geodistSagnac(row.dd.sat_tar_base, base_, &tmp));
      const Eigen::Matrix<double, 1, state_dim> Hstate =
          (e_tar - e_ref).transpose() * J_ant;
      if (carrier_) {
        const double nr = values.at<double>(row.ref);
        const double nt = values.at<double>(row.tar);
        const double obs = (row.dd.cp_rov_ref - row.dd.cp_base_ref) -
                           (row.dd.cp_rov_tar - row.dd.cp_base_tar);
        error(i) = model + row.dd.lam * (nr - nt) - obs;
        if (H) {
          (*H)[0].row(i) = Hstate;
          (*H)[key_col_.at(row.ref)](i, 0) = row.dd.lam;
          (*H)[key_col_.at(row.tar)](i, 0) = -row.dd.lam;
        }
      } else {
        const double obs = (row.dd.pr_rov_ref - row.dd.pr_base_ref) -
                           (row.dd.pr_rov_tar - row.dd.pr_base_tar);
        error(i) = model - obs;
        if (H) (*H)[0].row(i) = Hstate;
      }
    }
    return error;
  }

 private:
  static gtsam::KeyVector makeKeys(gtsam::Key state_key,
                                   const std::vector<GroupedDdRow>& rows,
                                   bool carrier) {
    gtsam::KeyVector keys{state_key};
    std::set<gtsam::Key> seen{state_key};
    if (carrier) {
      for (const auto& row : rows) {
        if (seen.insert(row.ref).second) keys.push_back(row.ref);
        if (seen.insert(row.tar).second) keys.push_back(row.tar);
      }
    }
    return keys;
  }

  std::vector<GroupedDdRow> rows_;
  bool carrier_{false};
  gtsam::Point3 base_;
  gtsam::Point3 lever_;
  gtsam::Pose3 ecef_T_nav_;
  std::map<gtsam::Key, std::size_t> key_col_;
};

template <typename State>
inline std::vector<DdAmbiguityPair> addGroupedDdFactorsImpl(
    const gnss_utils::PreprocessedEpoch& ep, gtsam::Key state_key,
    const gtsam::Point3& lever, const gtsam::Pose3& ecef_T_nav,
    const AdapterConfig& cfg, gtsam::NonlinearFactorGraph& graph,
    gtsam::Values& values, PersistentAmbiguities& persistent,
    const Eigen::Vector3d* gate_pos = nullptr, int* n_gated = nullptr,
    DdFactorLayout* layout = nullptr, int* n_bad_cov = nullptr) {
  std::vector<DdAmbiguityPair> pairs;
  persistent.beginEpoch();
  if (layout) {
    layout->groups.clear();
    layout->valid = true;
  }
  using GroupRows = std::pair<std::vector<GroupedDdRow>,
                              std::vector<GroupedDdRow>>;  // code, carrier
  std::map<AmbiguityGroupId, GroupRows> groups;
  const double ctow = ep.week * 604800.0 + ep.tow;

  for (int di = 0; di < static_cast<int>(ep.dd.size()); ++di) {
    const auto& d = ep.dd[di];
    if (gate_pos && cfg.pr_innov_gate_m > 0.0 && d.has_pr &&
        std::fabs(detail::ddPseudorangeResidual(d, ep.base_ecef, *gate_pos)) >
            cfg.pr_innov_gate_m) {
      if (n_gated) ++*n_gated;
      continue;
    }
    const AmbiguityGroupId gid{d.sys, d.band, d.code_ref, d.code_base_ref};
    if (d.has_pr) groups[gid].first.push_back(GroupedDdRow{d, 0, 0, di});
    if (!d.has_cp) continue;

    const double nref = detail::codeMinusCarrier(
        d.cp_rov_ref, d.cp_base_ref, d.pr_rov_ref, d.pr_base_ref, d.lam);
    const double ntar = detail::codeMinusCarrier(
        d.cp_rov_tar, d.cp_base_tar, d.pr_rov_tar, d.pr_base_tar, d.lam);
    const gtsam::Key k_ref =
        persistent.obtain(d.sat_ref, d.band, nref, d.slip_ref, ctow, values, gid);
    const gtsam::Key k_tar =
        persistent.obtain(d.sat_tar, d.band, ntar, d.slip_tar, ctow, values, gid);
    groups[gid].second.push_back(GroupedDdRow{d, k_ref, k_tar, di});

    DdAmbiguityPair p{k_ref, k_tar, d.lam, d.el_tar};
    p.dd_index = di;
    p.half_cycle = d.half_cycle_ref || d.half_cycle_tar;
    p.fresh = persistent.isFresh(d.sat_ref, d.band) ||
              persistent.isFresh(d.sat_tar, d.band);
    p.lock = std::min(persistent.lockCount(d.sat_ref, d.band),
                      persistent.lockCount(d.sat_tar, d.band));
    pairs.push_back(p);
  }

  for (auto& kv : groups) {
    // Gauge the carrier group BEFORE its factor is built. The rows constrain
    // only differences of their ambiguities (m rows, m+1 keys), so the group
    // needs exactly one loose prior on one of ITS OWN keys. Deciding it here -
    // where the complete key set is known - is what makes the invariant hold
    // through re-keys: a group whose members all re-keyed has no anchored key
    // left and is re-gauged, while a group that kept one keeps a single prior.
    // The gauge is per CONNECTED COMPONENT, not per group. A row constrains only
    // the DIFFERENCE of its two ambiguities, so each component of the (ref,tar)
    // graph carries exactly one gauge freedom. A group keyed by
    // (sys, band, code) is NOT guaranteed to be one component - nothing forces
    // its rows to share a reference satellite - and asking only "is ANY key in
    // this group anchored?" then leaves every component but one unpinned, which
    // is a rank deficiency: one such epoch throws
    // IndeterminantLinearSystemException and costs the whole graph. Union-find
    // here is O(rows) and makes the invariant hold whatever the reference
    // structure turns out to be.
    if (!kv.second.second.empty()) {
      std::map<gtsam::Key, gtsam::Key> parent;
      auto find = [&parent](gtsam::Key k) {
        auto it = parent.find(k);
        if (it == parent.end()) {
          parent[k] = k;
          return k;
        }
        while (parent[k] != k) k = parent[k];
        return k;
      };
      auto unite = [&](gtsam::Key a, gtsam::Key b) {
        const gtsam::Key ra = find(a), rb = find(b);
        if (ra != rb) parent[ra] = rb;
      };
      for (const auto& row : kv.second.second) unite(row.ref, row.tar);
      // Components that already carry a gauge from an earlier epoch.
      std::set<gtsam::Key> gauged;
      std::vector<gtsam::Key> keys;
      keys.reserve(parent.size());
      for (const auto& p : parent) keys.push_back(p.first);
      for (const gtsam::Key k : keys)
        if (persistent.isAnchored(k)) gauged.insert(find(k));
      // Anchor every component that has none.
      for (const auto& row : kv.second.second) {
        const gtsam::Key root = find(row.ref);
        if (gauged.count(root)) continue;
        graph.add(gtsam::PriorFactor<double>(
            row.ref, 0.0,
            gtsam::noiseModel::Isotropic::Sigma(1, cfg.init_sigma_cycles)));
        persistent.markAnchored(row.ref);
        gauged.insert(root);
      }
    }
    auto add_phase = [&](std::vector<GroupedDdRow>& rows, bool carrier,
                         double sigma) {
      if (rows.empty()) return;
      std::vector<gnss_utils::DdSignal> signals;
      signals.reserve(rows.size());
      for (const auto& row : rows) signals.push_back(row.dd);
      const Eigen::MatrixXd R = groupedDdCovariance(signals, sigma, cfg);
      const gtsam::SharedNoiseModel noise = groupedDdNoise(R, cfg);
      if (!noise) {
        // An unusable covariance. Drop the group rather than build a factor
        // around a NaN model, and invalidate the layout so the post-fit FIX
        // validation refuses a verdict for this epoch (fail-closed) instead of
        // testing against rows the graph does not actually contain.
        if (n_bad_cov) ++*n_bad_cov;
        if (layout) layout->valid = false;
        return;
      }
      // Record the factor's own rows and its own R before the rows are moved
      // into it - see DdFactorLayout for why neither may be re-derived later.
      if (layout) {
        DdFactorLayout::Group g;
        g.carrier = carrier;
        g.nominal_cov = R;
        g.dd_index.reserve(rows.size());
        for (const auto& row : rows) g.dd_index.push_back(row.dd_index);
        layout->groups.push_back(std::move(g));
      }
      graph.add(std::make_shared<GroupedDdFactor<State>>(
          state_key, std::move(rows), carrier, gtsam::Point3(ep.base_ecef),
          lever, ecef_T_nav, noise));
    };
    add_phase(kv.second.first, false, cfg.pr_sigma_m);
    add_phase(kv.second.second, true, cfg.cp_sigma_m);
  }
  return pairs;
}

inline std::vector<DdAmbiguityPair> addGroupedDdFactors(
    const gnss_utils::PreprocessedEpoch& ep, gtsam::Key position_key,
    const AdapterConfig& cfg, gtsam::NonlinearFactorGraph& graph,
    gtsam::Values& values, PersistentAmbiguities& persistent,
    const Eigen::Vector3d* gate_pos = nullptr, int* n_gated = nullptr,
    DdFactorLayout* layout = nullptr, int* n_bad_cov = nullptr) {
  return addGroupedDdFactorsImpl<gtsam::Point3>(
      ep, position_key, gtsam::Point3(), gtsam::Pose3(), cfg, graph, values,
      persistent, gate_pos, n_gated, layout, n_bad_cov);
}

inline std::vector<DdAmbiguityPair> addGroupedDdFactorsArm(
    const gnss_utils::PreprocessedEpoch& ep, gtsam::Key pose_key,
    const gtsam::Point3& lever_arm, const gtsam::Pose3& ecef_T_nav,
    const AdapterConfig& cfg, gtsam::NonlinearFactorGraph& graph,
    gtsam::Values& values, PersistentAmbiguities& persistent,
    const Eigen::Vector3d* gate_pos = nullptr, int* n_gated = nullptr,
    DdFactorLayout* layout = nullptr, int* n_bad_cov = nullptr) {
  return addGroupedDdFactorsImpl<gtsam::Pose3>(
      ep, pose_key, lever_arm, ecef_T_nav, cfg, graph, values, persistent,
      gate_pos, n_gated, layout, n_bad_cov);
}

// Joint marginal extracted AFTER the grouped DD factors have been optimized.
// There is no second measurement update: a_float, Qa, the antenna state, and
// the complete navigation-state cross covariance all belong to one posterior.
// Fraction of the largest eigenvalue below which a NEGATIVE eigenvalue of
// R_eff - H*P+*H' is treated as a genuine inconsistency rather than numerical
// noise, and the FIX is rejected. Loose enough that ordinary linearization
// mismatch between the window posterior and this epoch's R does not reject.
constexpr double kFdeNegativeEigenvalueFrac = 1e-3;

struct GraphArPosterior {
  Eigen::Vector3d ant{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d ant_cov{Eigen::Matrix3d::Zero()};
  Eigen::VectorXd n;
  Eigen::MatrixXd n_cov;
  Eigen::MatrixXd ant_n_cross;    // 3 x m
  Eigen::MatrixXd full_cov;       // 3x3 Point, 15x15 Pose/Vel/Bias
  Eigen::MatrixXd full_n_cross;   // state_dim x m
  std::map<gtsam::Key, int> col;

  // The values the graph actually LINEARIZED at, which is where the DD factors
  // formed their robust weights. ISAM2 relinearizes lazily (relinearizeSkip), so
  // this is deliberately NOT calculateEstimate(): the post-fit FIX validation
  // has to reconstruct the weights that were applied, not the ones the current
  // estimate would imply. `lin_valid` false means the point was unavailable, and
  // the validation must then decline to produce a verdict rather than guess.
  Eigen::Vector3d lin_ant{Eigen::Vector3d::Zero()};
  Eigen::VectorXd lin_n;  // same column order as `n` / `col`
  bool lin_valid{false};
};

inline std::vector<gtsam::Key> uniqueAmbiguityKeys(
    const std::vector<DdAmbiguityPair>& pairs) {
  std::vector<gtsam::Key> keys;
  std::set<gtsam::Key> seen;
  for (const auto& p : pairs) {
    if (seen.insert(p.ref).second) keys.push_back(p.ref);
    if (seen.insert(p.tar).second) keys.push_back(p.tar);
  }
  return keys;
}

// Read one variable off the graph's LINEARIZATION POINT (not its estimate).
// ISAM2 relinearizes lazily, so the two differ by up to relinearizeSkip epochs'
// worth of correction - and the factors' robust weights were formed at this one.
template <typename T>
inline bool linearizationPoint(const gtsam::ISAM2& isam, gtsam::Key key,
                               T& out) {
  const gtsam::Values& lin = isam.getLinearizationPoint();
  if (!lin.exists(key)) return false;
  out = lin.at<T>(key);
  return true;
}

inline bool linearizationPoint(const gtsam::ISAM2& isam, gtsam::Key key,
                               Eigen::Vector3d& out) {
  gtsam::Point3 p;
  if (!linearizationPoint<gtsam::Point3>(isam, key, p)) return false;
  out = p;
  return true;
}

// The carried ambiguities at the linearization point, in the given key order.
inline bool captureLinearizationPoint(const gtsam::ISAM2& isam,
                                      const std::vector<gtsam::Key>& amb,
                                      Eigen::VectorXd& out) {
  const gtsam::Values& lin = isam.getLinearizationPoint();
  out.resize(static_cast<int>(amb.size()));
  for (int i = 0; i < static_cast<int>(amb.size()); ++i) {
    if (!lin.exists(amb[i])) return false;
    out(i) = lin.at<double>(amb[i]);
  }
  return true;
}

inline bool buildGraphPosteriorPoint(
    const gtsam::ISAM2& isam, gtsam::Key xk,
    const std::vector<DdAmbiguityPair>& pairs, GraphArPosterior& out,
    std::string* err = nullptr) {
  const auto amb = uniqueAmbiguityKeys(pairs);
  if (amb.empty()) return false;
  try {
    gtsam::KeyVector q{xk};
    q.insert(q.end(), amb.begin(), amb.end());
    const gtsam::JointMarginal jm = isam.jointMarginalCovariance(q);
    out.ant = isam.calculateEstimate<gtsam::Point3>(xk);
    out.ant_cov = jm.at(xk, xk);
    out.full_cov = out.ant_cov;
    const int m = static_cast<int>(amb.size());
    out.n.resize(m);
    out.n_cov.resize(m, m);
    out.ant_n_cross.resize(3, m);
    out.full_n_cross.resize(3, m);
    out.col.clear();
    for (int i = 0; i < m; ++i) {
      out.col[amb[i]] = i;
      out.n(i) = isam.calculateEstimate<double>(amb[i]);
      out.ant_n_cross.col(i) = jm.at(xk, amb[i]);
      out.full_n_cross.col(i) = out.ant_n_cross.col(i);
      for (int j = 0; j < m; ++j)
        out.n_cov(i, j) = jm.at(amb[i], amb[j])(0, 0);
    }
    out.lin_valid = captureLinearizationPoint(isam, amb, out.lin_n) &&
                    linearizationPoint<gtsam::Point3>(isam, xk, out.lin_ant);
  } catch (const std::exception& e) {
    if (err) *err = e.what();
    return false;
  }
  return out.ant.allFinite() && out.full_cov.allFinite() &&
         out.n_cov.allFinite() && out.full_n_cross.allFinite();
}

inline bool buildGraphPosteriorNav(
    const gtsam::ISAM2& isam, gtsam::Key xk, gtsam::Key vk, gtsam::Key bk,
    const gtsam::Point3& lever_arm, const gtsam::Pose3& ecef_T_nav,
    const std::vector<DdAmbiguityPair>& pairs, GraphArPosterior& out,
    std::string* err = nullptr) {
  const auto amb = uniqueAmbiguityKeys(pairs);
  if (amb.empty()) return false;
  try {
    const std::array<gtsam::Key, 3> state_keys{xk, vk, bk};
    const std::array<int, 3> dims{6, 3, 6};
    const std::array<int, 3> offsets{0, 6, 9};
    gtsam::KeyVector q(state_keys.begin(), state_keys.end());
    q.insert(q.end(), amb.begin(), amb.end());
    const gtsam::JointMarginal jm = isam.jointMarginalCovariance(q);

    const gtsam::Pose3 pose = isam.calculateEstimate<gtsam::Pose3>(xk);
    gtsam::Matrix36 H_pose;
    const gtsam::Point3 ant_nav = pose.transformFrom(lever_arm, H_pose);
    const gtsam::Point3 ant_ecef = ecef_T_nav.transformFrom(ant_nav);
    out.ant = Eigen::Vector3d(ant_ecef);
    const Eigen::Matrix<double, 3, 6> J =
        ecef_T_nav.rotation().matrix() * H_pose;

    out.full_cov = Eigen::MatrixXd::Zero(15, 15);
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        out.full_cov.block(offsets[i], offsets[j], dims[i], dims[j]) =
            jm.at(state_keys[i], state_keys[j]);
      }
    }
    out.ant_cov = J * out.full_cov.topLeftCorner<6, 6>() * J.transpose();

    const int m = static_cast<int>(amb.size());
    out.n.resize(m);
    out.n_cov.resize(m, m);
    out.ant_n_cross.resize(3, m);
    out.full_n_cross = Eigen::MatrixXd::Zero(15, m);
    out.col.clear();
    for (int i = 0; i < m; ++i) {
      out.col[amb[i]] = i;
      out.n(i) = isam.calculateEstimate<double>(amb[i]);
      for (int s = 0; s < 3; ++s) {
        out.full_n_cross.block(offsets[s], i, dims[s], 1) =
            jm.at(state_keys[s], amb[i]);
      }
      out.ant_n_cross.col(i) = J * out.full_n_cross.block(0, i, 6, 1);
      for (int j = 0; j < m; ++j)
        out.n_cov(i, j) = jm.at(amb[i], amb[j])(0, 0);
    }
    // Antenna position at the LINEARIZATION point, through the same lever arm
    // and frame transform the DD factors used.
    gtsam::Pose3 lin_pose;
    if (captureLinearizationPoint(isam, amb, out.lin_n) &&
        linearizationPoint<gtsam::Pose3>(isam, xk, lin_pose)) {
      out.lin_ant = Eigen::Vector3d(
          ecef_T_nav.transformFrom(lin_pose.transformFrom(lever_arm)));
      out.lin_valid = true;
    }
  } catch (const std::exception& e) {
    if (err) *err = e.what();
    return false;
  }
  return out.ant.allFinite() && out.full_cov.allFinite() &&
         out.n_cov.allFinite() && out.full_n_cross.allFinite();
}

// Resolve integer DD ambiguities directly from the optimized stage-2 graph.
// No observation is applied here: the means, Qa and all state/ambiguity cross
// covariances are projections of one ISAM2 posterior.
// The robust weights the GRAPH actually applied to this epoch's DD factors,
// reproduced so the post-fit FIX validation subtracts two halves of ONE
// estimator. Three things have to match the front-end or the reconstruction is
// worse than useless, because it looks authoritative:
//
//   1. THE ROWS AND THE COVARIANCE. Both come from DdFactorLayout, recorded as
//      the factors were built - not re-derived from ep.dd. `pr_innov_gate_m` can
//      drop rows at build time, and R is a function of the cfg and signal order
//      then in force, so a second derivation is right only by coincidence.
//   2. THE LINEARIZATION POINT. ISAM2 relinearizes lazily, so the whitening ran
//      at getLinearizationPoint(), not at the current estimate.
//   3. THE FLOAT AMBIGUITIES. A carrier residual is undefined without one, and
//      the value the graph had was the float, not the integer under test.
//
// valid == false means the weights could not be reproduced, and the FIX is then
// rejected (the post-fit validation is fail-closed).
struct FrontEndRobustWeights {
  bool valid{false};
  // (dd_index, carrier) -> (group index, row index within that group)
  std::map<std::pair<int, int>, std::pair<int, int>> loc;
  std::vector<Eigen::MatrixXd> group_reff;  // effective covariance per group

  const Eigen::MatrixXd* block(int dd_index, bool carrier, int* row) const {
    const auto it = loc.find({dd_index, carrier ? 1 : 0});
    if (it == loc.end()) return nullptr;
    *row = it->second.second;
    return &group_reff[it->second.first];
  }
  int group(int dd_index, bool carrier) const {
    const auto it = loc.find({dd_index, carrier ? 1 : 0});
    return it == loc.end() ? -1 : it->second.first;
  }
};

inline FrontEndRobustWeights frontEndRobustWeights(
    const gnss_utils::PreprocessedEpoch& ep,
    const std::vector<DdAmbiguityPair>& pairs,
    const GraphArPosterior& posterior, const DdFactorLayout& layout,
    const AdapterConfig& cfg) {
  FrontEndRobustWeights out;
  if (!layout.valid || !posterior.lin_valid) return out;

  // The float DD ambiguity of each carrier DD, at the linearization point.
  std::map<int, double> dd_float_cycles;
  for (const auto& p : pairs) {
    if (p.dd_index < 0) continue;
    const auto ir = posterior.col.find(p.ref);
    const auto it = posterior.col.find(p.tar);
    if (ir == posterior.col.end() || it == posterior.col.end()) continue;
    if (ir->second >= posterior.lin_n.size() ||
        it->second >= posterior.lin_n.size())
      continue;
    dd_float_cycles[p.dd_index] =
        posterior.lin_n(ir->second) - posterior.lin_n(it->second);
  }

  for (const auto& g : layout.groups) {
    const int n = static_cast<int>(g.dd_index.size());
    if (n == 0 || g.nominal_cov.rows() != n) return FrontEndRobustWeights{};
    const Eigen::MatrixXd& R = g.nominal_cov;

    Eigen::VectorXd r(n);
    for (int i = 0; i < n; ++i) {
      const int di = g.dd_index[i];
      if (di < 0 || di >= static_cast<int>(ep.dd.size()))
        return FrontEndRobustWeights{};
      const auto& d = ep.dd[di];
      if (g.carrier) {
        const auto it = dd_float_cycles.find(di);
        if (it == dd_float_cycles.end()) return FrontEndRobustWeights{};
        r(i) = detail::ddCarrierResidual(d, ep.base_ecef, posterior.lin_ant,
                                         it->second);
      } else {
        r(i) = detail::ddPseudorangeResidual(d, ep.base_ecef, posterior.lin_ant);
      }
    }
    if (!r.allFinite()) return FrontEndRobustWeights{};

    Eigen::MatrixXd R_eff;
    if (!cfg.robust) {
      R_eff = R;
    } else {
      // Block: gtsam whitens with chol(R)^-1 and forms ONE weight from the norm
      // of the whole group, then multiplies sqrt(w) into every row. Every row
      // scaled by 1/sqrt(w) is a covariance scaled by 1/w.
      const Eigen::LLT<Eigen::MatrixXd> llt(R);
      if (llt.info() != Eigen::Success) return FrontEndRobustWeights{};
      const Eigen::MatrixXd L = llt.matrixL();
      const Eigen::VectorXd z = L.triangularView<Eigen::Lower>().solve(r);
      const double nz = z.norm();
      if (!std::isfinite(nz)) return FrontEndRobustWeights{};
      // EXACTLY gtsam::noiseModel::mEstimator::Huber::weight - no floor. This
      // path exists to reproduce what the graph applied, so a different formula
      // defeats its own purpose; the divergence a floor introduces is caught
      // downstream by the allFinite() check instead.
      const double w =
          (cfg.huber_k > 0.0 && nz > cfg.huber_k) ? cfg.huber_k / nz : 1.0;
      R_eff = R / w;
    }
    if (!R_eff.allFinite()) return FrontEndRobustWeights{};

    const int gi = static_cast<int>(out.group_reff.size());
    out.group_reff.push_back(std::move(R_eff));
    for (int i = 0; i < n; ++i)
      out.loc[{g.dd_index[i], g.carrier ? 1 : 0}] = {gi, i};
  }
  out.valid = true;
  return out;
}

inline ArResult resolveAmbiguitiesPosterior(
    const gnss_utils::PreprocessedEpoch& ep,
    const std::vector<DdAmbiguityPair>& pairs,
    const GraphArPosterior& posterior, const DdFactorLayout& layout,
    const AdapterConfig& cfg, const ArOptions& opt) {
  ArResult empty;
  const int m = static_cast<int>(posterior.n.size());
  if (m == 0 || posterior.n_cov.rows() != m ||
      posterior.ant_n_cross.cols() != m ||
      posterior.full_n_cross.cols() != m)
    return empty;

  std::vector<int> eligible;
  int n_excl_half = 0, n_excl_lock = 0, n_excl_nokey = 0;
  for (int i = 0; i < static_cast<int>(pairs.size()); ++i) {
    const auto& p = pairs[i];
    if (p.half_cycle) { ++n_excl_half; continue; }
    if (p.lock < opt.min_lock) { ++n_excl_lock; continue; }
    if (p.dd_index < 0 || p.dd_index >= static_cast<int>(ep.dd.size()) ||
        !posterior.col.count(p.ref) || !posterior.col.count(p.tar)) {
      ++n_excl_nokey;
      continue;
    }
    eligible.push_back(i);
  }
  // Carried on every return path below so the caller can always see why.
  auto stamp = [&](ArResult& r) {
    r.n_eligible = static_cast<int>(eligible.size());
    r.n_excl_half = n_excl_half;
    r.n_excl_lock = n_excl_lock;
    r.n_excl_nokey = n_excl_nokey;
  };
  stamp(empty);
  if (static_cast<int>(eligible.size()) < std::max(opt.min_fix, 1)) {
    empty.fail = ArFail::TooFewEligible;
    return empty;
  }

  auto solve_subset = [&](const std::set<int>& excluded) {
    ArResult r;
    std::vector<int> selected;
    for (const int pi : eligible)
      if (!excluded.count(pi)) selected.push_back(pi);
    const int k = static_cast<int>(selected.size());
    stamp(r);
    if (k < std::max(opt.min_fix, 1)) {
      r.fail = ArFail::TooFewEligible;
      return r;
    }

    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(k, m);
    std::vector<double> elevation(k);
    r.cp_dd_index.reserve(k);
    for (int i = 0; i < k; ++i) {
      const auto& p = pairs[selected[i]];
      D(i, posterior.col.at(p.ref)) = 1.0;
      D(i, posterior.col.at(p.tar)) = -1.0;
      elevation[i] = p.el;
      r.cp_dd_index.push_back(p.dd_index);
    }
    r.a_float = D * posterior.n;
    r.Qa = D * posterior.n_cov * D.transpose();
    r.Qa = 0.5 * (r.Qa + r.Qa.transpose());
    r.float_valid = true;
    r.float_ant_pos = posterior.ant;
    r.float_ant_cov = posterior.ant_cov;

    const double pos_var =
        posterior.ant_cov.diagonal().sum() / 3.0;
    r.pos_var = pos_var;
    if (opt.max_pos_var_m2 > 0.0 && pos_var > opt.max_pos_var_m2) {
      r.fail = ArFail::PosVarGate;
      return r;
    }

    const Eigen::MatrixXd Qant_a = posterior.ant_n_cross * D.transpose();
    detail::lambdaFix(r.a_float, r.Qa, Qant_a, posterior.ant_cov, elevation,
                      opt.ratio_threshold, opt.el_mask_rad, opt.min_fix, r);
    if (!r.candidate) return r;
    r.fixed_pos = posterior.ant - r.state_correction.head<3>();

    // Integer-condition the complete graph state exactly once. lambdaFix may
    // elevation-mask a subset, so use only its finite candidate entries.
    const int nf = static_cast<int>(r.fixed_idx.size());
    if (nf == 0) {
      r.candidate = false;
      r.fixed = false;
      r.fail = ArFail::ConditioningFailed;
      return r;
    }
    Eigen::MatrixXd Qa_f(nf, nf);
    Eigen::MatrixXd Qfull_f(posterior.full_n_cross.rows(), nf);
    Eigen::VectorXd innovation(nf);
    for (int i = 0; i < nf; ++i) {
      const int ii = r.fixed_idx[i];
      innovation(i) = r.a_float(ii) - r.a_fix(ii);
      Qfull_f.col(i) =
          posterior.full_n_cross * D.row(ii).transpose();
      for (int j = 0; j < nf; ++j)
        Qa_f(i, j) = r.Qa(ii, r.fixed_idx[j]);
    }
    const Eigen::LDLT<Eigen::MatrixXd> ldlt(Qa_f);
    if (ldlt.info() != Eigen::Success ||
        ldlt.vectorD().minCoeff() <= 0.0) {
      r.candidate = false;
      r.fixed = false;
      r.fail = ArFail::QaNotPd;
      return r;
    }
    r.full_state_delta = -Qfull_f * ldlt.solve(innovation);
    r.full_state_cov =
        posterior.full_cov - Qfull_f * ldlt.solve(Qfull_f.transpose());
    r.full_state_cov =
        0.5 * (r.full_state_cov + r.full_state_cov.transpose());
    if (!r.full_state_delta.allFinite() || !r.full_state_cov.allFinite()) {
      r.candidate = false;
      r.fixed = false;
      r.fail = ArFail::ConditioningFailed;
    }
    return r;
  };

  // Partial AR removes weak integer rows only. Measurements remain in the
  // graph float posterior, which prevents a second, inconsistent estimator.
  std::set<int> excluded;
  ArResult result;
  for (int dropped = 0;; ++dropped) {
    result = solve_subset(excluded);
    result.n_partial_dropped = dropped;
    if (result.candidate && result.fixed) break;
    if (!opt.partial_ar || dropped >= opt.partial_max_drop) return result;
    int weakest = -1;
    double score = std::numeric_limits<double>::infinity();
    for (const int pi : eligible) {
      if (excluded.count(pi)) continue;
      const double s = (pairs[pi].fresh ? 0.0 : 100.0) + pairs[pi].el;
      if (s < score) {
        score = s;
        weakest = pi;
      }
    }
    if (weakest < 0) return result;
    excluded.insert(weakest);
  }

  // Differential code-DD test: did the integer correction move the antenna in a
  // direction the pseudoranges agree with? See ArOptions::max_code_resid_growth_m.
  // Recorded, never acted on here: rejecting needs the epoch-to-epoch
  // persistence that CodeGrowthMonitor supplies, and this resolver is pure.
  if (result.float_valid) {
    result.code_resid_rms = codeResidualRms(ep, result.float_ant_pos,
                                            opt.code_resid_min_dd,
                                            &result.n_code_dd);
    if (result.fixed)
      result.code_resid_growth = codeResidualGrowth(
          ep, result.float_ant_pos, result.fixed_pos, opt.code_resid_min_dd);
  }
  if (!result.fixed || !opt.fde_enable) return result;

  // Correlated global POST-FIT validation. Code and fixed-carrier rows are
  // tested jointly and the shared-reference covariance is retained.
  //
  // The residual covariance is SUBTRACTIVE, not additive. These residuals are
  // evaluated at result.fixed_pos, which is a projection of a posterior that
  // already absorbed these very rows, so the estimate has been pulled TOWARDS
  // each observation and the residuals are correspondingly smaller than the
  // measurement noise. For the linear-Gaussian posterior mean,
  //
  //     v = z - H*x_hat+   =>   Cov(v) = R - H*P+*H'
  //
  // with P+ the posterior covariance (here result.state_cov, the
  // integer-conditioned antenna block). Writing R + H*P+*H' - the PRE-fit form,
  // which is right in ddPreFitTest because that reference does not use these
  // rows - inflates the denominator instead of shrinking it, so chi^2 comes out
  // too small and the test waves through fixes it should catch. That is the
  // wrong direction for the only test standing between a wrong integer set and
  // a published FIX.
  //
  // Two consequences of doing it properly, both handled below:
  //   - Cov(v) is RANK-DEFICIENT by construction (the 3 position directions the
  //     fit consumed carry no residual information), so it must be inverted on
  //     its own column space and the chi^2 compared against a quantile at that
  //     rank rather than at the row count.
  //   - the DD factors carry a Huber loss, so the posterior was NOT formed
  //     under the nominal R. P+ already reflects the down-weighting, so R has to
  //     be scaled to match (frontEndRobustWeights) or the two halves of the
  //     subtraction describe different estimators.
  struct ValidationRow {
    const gnss_utils::DdSignal* d;
    bool carrier;
    double integer;
    int dd_index;  // index into ep.dd, to look the front-end weight back up
  };
  // The weights the graph applied, and - just as importantly - WHICH rows it
  // applied them to. `pr_innov_gate_m` drops DDs at factor-build time, so
  // ep.dd is a superset of what the estimator actually consumed.
  const FrontEndRobustWeights few =
      frontEndRobustWeights(ep, pairs, posterior, layout, cfg);
  if (!few.valid) {
    result.fixed = false;
    result.fail = ArFail::FdeReject;
    return result;
  }

  std::vector<ValidationRow> rows;
  std::map<int, double> fixed_integer;
  for (int i = 0; i < static_cast<int>(result.cp_dd_index.size()); ++i)
    if (i < result.a_fix.size() && std::isfinite(result.a_fix(i)))
      fixed_integer[result.cp_dd_index[i]] = result.a_fix(i);
  // A row the graph never saw is SKIPPED, not rejected and not repaired. It
  // cannot be tested here: this matrix is the post-fit form R - H*P+*H', which
  // assumes the fit consumed the row. For a gated row the correct residual
  // covariance is the PRE-fit R + H*P+*H', so folding it in would be wrong in
  // the opposite direction. Rejecting on it instead - which is what the
  // fail-closed lookup did before this - threw away the whole epoch's FIX
  // because one satellite had been gated.
  auto in_graph = [&](int di, bool carrier) {
    int row = 0;
    return few.block(di, carrier, &row) != nullptr;
  };
  for (int di = 0; di < static_cast<int>(ep.dd.size()); ++di) {
    const auto& d = ep.dd[di];
    if (d.has_pr) {
      if (in_graph(di, false)) rows.push_back(ValidationRow{&d, false, 0.0, di});
    }
    const auto it = fixed_integer.find(di);
    if (d.has_cp && it != fixed_integer.end() && in_graph(di, true))
      rows.push_back(ValidationRow{&d, true, it->second, di});
  }
  const int nr = static_cast<int>(rows.size());
  if (nr == 0) return result;
  Eigen::VectorXd residual(nr);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(nr, 3);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(nr, nr);
  for (int i = 0; i < nr; ++i) {
    const auto& d = *rows[i].d;
    Eigen::Vector3d er, et, tmp;
    detail::geodistSagnac(d.sat_ref_rov, result.fixed_pos, &er);
    detail::geodistSagnac(d.sat_tar_rov, result.fixed_pos, &et);
    H.row(i) = (et - er).transpose();
    residual(i) = rows[i].carrier
        ? detail::ddCarrierResidual(d, ep.base_ecef, result.fixed_pos,
                                    rows[i].integer)
        : detail::ddPseudorangeResidual(d, ep.base_ecef, result.fixed_pos);
    const double sigma = rows[i].carrier ? cfg.cp_sigma_m : cfg.pr_sigma_m;
    R(i, i) = ddSingleDifferenceVar(d, true, sigma, cfg) +
              ddSingleDifferenceVar(d, false, sigma, cfg);
    for (int j = 0; j < i; ++j) {
      const auto& q = *rows[j].d;
      if (rows[i].carrier == rows[j].carrier && d.sys == q.sys &&
          d.band == q.band && d.code_ref == q.code_ref &&
          d.code_base_ref == q.code_base_ref) {
        R(i, j) = R(j, i) =
            ddSingleDifferenceVar(d, true, sigma, cfg);
      }
    }
  }
  // Huber-effective measurement covariance: the covariance the posterior was
  // ACTUALLY formed under, so both halves of the subtraction below describe one
  // estimator.
  //
  // The weights come from frontEndRobustWeights - the graph's grouping, the
  // graph's linearization point and the FLOAT ambiguities - and NOT from the FIX
  // residuals computed just above. Recomputing them here from the residuals
  // under test was the earlier bug: P+ carries the float-time down-weighting,
  // so pairing it with fix-time weights makes R_eff and H*P+*H' describe
  // different estimators, in a test whose entire job is to compare them.
  //
  // When the weights cannot be reproduced (no linearization point, e.g. straight
  // after a re-anchor) the test declines to run rather than guess: a guessed
  // weight here would let a wrong integer set through, which is the one outcome
  // this test exists to prevent.
  // Every "cannot decide" exit goes through this single fail-closed path.
  auto no_verdict = [&]() -> ArResult {
    result.fixed = false;
    result.fail = ArFail::FdeReject;
    return result;
  };

  Eigen::MatrixXd R_eff = R;
  {
    R_eff.setZero();
    for (int i = 0; i < nr; ++i) {
      int gi = 0;
      const Eigen::MatrixXd* bi =
          few.block(rows[i].dd_index, rows[i].carrier, &gi);
      if (bi == nullptr) return no_verdict();  // filtered above; defensive
      const int g = few.group(rows[i].dd_index, rows[i].carrier);
      for (int j = 0; j < nr; ++j) {
        int gj = 0;
        if (few.group(rows[j].dd_index, rows[j].carrier) != g) continue;
        few.block(rows[j].dd_index, rows[j].carrier, &gj);
        R_eff(i, j) = (*bi)(gi, gj);
      }
    }
  }

  // Residual covariance for an estimator that used ROBUST weights.
  //
  // Any linear estimator is x_hat = x_bar + K (z - H x_bar), so the post-fit
  // residual is v = e - H*delta and
  //
  //     Cov(v) = R - S_hat R - R S_hat' + H P+ H',   S_hat = H K,  K = P+ H' W
  //
  // with R the TRUE measurement covariance and W the weights actually applied.
  // The familiar R - H P+ H' is the special case W = R^-1, where S_hat R
  // collapses to H P+ H' and two of the terms cancel - i.e. it assumes the
  // estimator was EFFICIENT.
  //
  // A robust kernel is not efficient: it applies W = R_eff^-1 while the noise is
  // still R. R_eff is an IRLS device, not an estimate of the noise, so it must
  // not be substituted for R - declaring the down-weighted row's variance to be
  // its true variance is circular, since it was down-weighted BECAUSE its
  // residual was large.
  //
  // With nothing down-weighted R_eff == R and this reduces exactly to
  // R - H P+ H'; the unit test pins that.
  const Eigen::MatrixXd HPHt = H * result.state_cov * H.transpose();
  const Eigen::LDLT<Eigen::MatrixXd> reff_ldlt(R_eff);
  if (reff_ldlt.info() != Eigen::Success) return no_verdict();
  const Eigen::MatrixXd Shat_R = HPHt * reff_ldlt.solve(R);
  Eigen::MatrixXd S = R - Shat_R - Shat_R.transpose() + HPHt;
  S = 0.5 * (S + S.transpose());

  // S is singular by construction: the fit consumed 3 position directions, so
  // Cov(v) is rank-deficient and has no ordinary inverse. Decompose it and work
  // on the column space. The rank is COUNTED from the spectrum below, never
  // assumed to be nr - 3: partial AR, a re-keyed arc or a rank-deficient
  // geometry all move it, and testing chi^2 against the wrong dof is a silent
  // mis-calibration of the only check standing between a wrong integer set and
  // a published FIX.
  //
  // Negative eigenvalues are NOT all alike, and the previous code dropped them
  // all with the null space. Numerically-zero ones are the null space and are
  // dropped. A SIGNIFICANTLY negative one is different: it says
  // R_eff - H*P+*H' is not a covariance, i.e. the posterior claims more
  // information in that direction than the measurements can support, which is
  // exactly the signature of a wrong fix that the graph has already absorbed.
  // Discarding it hides the evidence, so it rejects the FIX instead.
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(S);
  if (es.info() != Eigen::Success) return no_verdict();
  const Eigen::VectorXd lambda = es.eigenvalues();
  const double lambda_max = lambda.maxCoeff();
  if (!(lambda_max > 0.0)) return no_verdict();
  // Null space vs. genuine inconsistency. The floor is the usual numerical
  // rank tolerance; the reject threshold is deliberately looser so ordinary
  // linearization mismatch between the window posterior and this epoch's R does
  // not reject on its own.
  const double lambda_floor = 1e-9 * lambda_max;
  const double lambda_reject = -kFdeNegativeEigenvalueFrac * lambda_max;
  if (lambda.minCoeff() < lambda_reject) {
    result.fixed = false;
    result.fail = ArFail::FdeReject;
    return result;
  }
  double chi2 = 0.0;
  int dof = 0;
  for (int i = 0; i < nr; ++i) {
    if (lambda(i) <= lambda_floor) continue;  // null space (incl. tiny negatives)
    const double proj = es.eigenvectors().col(i).dot(residual);
    chi2 += proj * proj / lambda(i);
    ++dof;
  }
  if (dof == 0) return no_verdict();  // no testable direction

  // Per-row standardized residuals need the diagonal of the same pseudo-inverse
  // basis: reconstruct S on its column space so a row whose residual variance
  // was numerically annihilated is skipped rather than divided by ~0.
  Eigen::VectorXd s_diag = Eigen::VectorXd::Zero(nr);
  for (int i = 0; i < nr; ++i) {
    if (lambda(i) <= lambda_floor) continue;
    s_diag += lambda(i) * es.eigenvectors().col(i).cwiseAbs2();
  }
  double max_standard = 0.0;
  for (int i = 0; i < nr; ++i) {
    if (s_diag(i) <= lambda_floor) continue;
    max_standard =
        std::max(max_standard, std::fabs(residual(i)) / std::sqrt(s_diag(i)));
  }

  result.postfit_chi2 = chi2;
  result.postfit_dof = dof;
  result.postfit_max_standard = max_standard;
  if (!std::isfinite(chi2) || max_standard > opt.fde_nsigma ||
      chi2 > chi2CriticalValue(dof, opt.fde_nsigma)) {
    result.fixed = false;
    result.fail = ArFail::FdeReject;
  }
  return result;
}

struct ConditionedNavState {
  gtsam::Pose3 pose;
  gtsam::Vector3 velocity{gtsam::Vector3::Zero()};
  Eigen::MatrixXd pose_cov;
  Eigen::Matrix3d velocity_cov{Eigen::Matrix3d::Zero()};
  bool ok{false};
};

// Convert the 15-D integer-conditioned tangent state into the public body pose
// and velocity. A final translation-only second-order correction enforces the
// nonlinear antenna lever-arm identity exactly while preserving the conditioned
// rotation and velocity.
inline ConditionedNavState makeConditionedNavState(
    const gtsam::Pose3& float_pose, const gtsam::Vector3& float_velocity,
    const gtsam::Point3& lever_arm, const gtsam::Pose3& ecef_T_nav,
    const ArResult& ar) {
  ConditionedNavState out;
  if (!ar.fixed || ar.full_state_delta.size() != 15 ||
      ar.full_state_cov.rows() != 15 || ar.full_state_cov.cols() != 15)
    return out;
  out.pose = float_pose.retract(ar.full_state_delta.head<6>());
  out.velocity = float_velocity + ar.full_state_delta.segment<3>(6);
  out.pose_cov = ar.full_state_cov.topLeftCorner<6, 6>();
  out.velocity_cov = ar.full_state_cov.block<3, 3>(6, 6);

  const gtsam::Point3 ant_nav = out.pose.transformFrom(lever_arm);
  const gtsam::Point3 ant_ecef = ecef_T_nav.transformFrom(ant_nav);
  const Eigen::Vector3d current_ant(ant_ecef.x(), ant_ecef.y(), ant_ecef.z());
  const Eigen::Vector3d d_nav =
      ecef_T_nav.rotation().matrix().transpose() *
      (ar.fixed_pos - current_ant);
  out.pose = gtsam::Pose3(
      out.pose.rotation(), out.pose.translation() + gtsam::Point3(d_nav));
  const gtsam::Point3 check =
      ecef_T_nav.transformFrom(out.pose.transformFrom(lever_arm));
  out.ok = (Eigen::Vector3d(check.x(), check.y(), check.z()) -
            ar.fixed_pos).norm() <= 1e-3 &&
           out.pose_cov.allFinite() && out.velocity_cov.allFinite();
  return out;
}

// --- GnssSolution ENU frame contract ----------------------------------------

// WHICH TANGENT PLANE EACH ENU FIELD OF GnssSolution USES.
//
// The references are DELIBERATELY different, and this has been "unified" twice
// by mistake, so it lives in one tested place. Per msg/GnssSolution.msg:71-86:
//
//   pos_enu     -> tangent plane at pos_enu_org_ecef (the ANCHOR), because a
//                  local trajectory must accumulate against a fixed frame.
//   pos_enu_cov -> tangent plane at the CURRENT receiver position.
//   vel_enu     -> tangent plane at the CURRENT receiver position.
//   vel_enu_cov -> tangent plane at the CURRENT receiver position.
//
// The covariance convention is the receiver one (u-blox NAV-COV NED, NovAtel
// BESTPOS stddev, Septentrio PosCovGeodetic) and RTKLIB .pos sdn/sde/sdu, which
// every other producer here already follows. The two planes differ by
// baseline/R_earth; use pos_cov_ecef when strict frame consistency matters.
struct EnuFrames {
  // False when no origin is configured and no base station has been seen; the
  // caller must then leave pos_enu at zero, having no frame to express it in.
  bool has_origin{false};
  double origin_lat{0.0}, origin_lon{0.0};  // anchors pos_enu ONLY
  double cur_lat{0.0}, cur_lon{0.0};        // every covariance, and vel_enu
};

// `cur_llh` is the published position as {lat_rad, lon_rad, height}; the
// origin is ECEF metres, all-zero meaning "not set".
inline EnuFrames enuFrames(const double cur_llh[3], const double origin_ecef[3]) {
  EnuFrames f;
  f.cur_lat = cur_llh[0];
  f.cur_lon = cur_llh[1];
  const double n = std::sqrt(origin_ecef[0] * origin_ecef[0] +
                             origin_ecef[1] * origin_ecef[1] +
                             origin_ecef[2] * origin_ecef[2]);
  if (n > 0.0) {
    double origin_llh[3];
    ecef2pos(origin_ecef, origin_llh);
    f.has_origin = true;
    f.origin_lat = origin_llh[0];
    f.origin_lon = origin_llh[1];
  }
  return f;
}

}  // namespace gnss_fgo

#endif  // EXAMPLES_TIGHTLY_COUPLED_FGO_FACTOR_ADAPTERS_HPP
