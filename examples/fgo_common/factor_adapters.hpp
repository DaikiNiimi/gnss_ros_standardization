// SPDX-License-Identifier: MIT
//
// Glue between the GnssPreprocessor output and GTSAM's GNSS DD factors.
// This is intentionally the whole "tight coupling" surface: one DdSignal
// maps onto one factor constructor call. Shared by the tightly_coupled_gnss
// and tightly_coupled_gnss_imu examples.
#ifndef EXAMPLES_FGO_COMMON_FACTOR_ADAPTERS_HPP
#define EXAMPLES_FGO_COMMON_FACTOR_ADAPTERS_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/CarrierPhaseFactor.h>
#include <gtsam/navigation/PseudorangeFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>

#include "gnss_ros_standardization/gnss_preprocessor.hpp"

namespace gnss_fgo {

struct AdapterConfig {
  double pr_sigma_m{0.5};    // UNDIFFERENCED (zenith) pseudorange sigma [m]
  double cp_sigma_m{0.005};  // UNDIFFERENCED (zenith) carrier phase sigma [m]
  bool elevation_weighting{true};
  // Loose prior on a target (non-reference) ambiguity.
  double amb_prior_sigma_cycles{100.0};
  // Tight prior on the per-group reference ambiguity. With the GTSAM
  // (N_ref, N_tar) DD parameterization only differences are observable, so
  // each (system,band) group keeps a 1-D gauge freedom. Pinning the reference
  // tightly removes it without biasing the DD (the value cancels in N_ref -
  // N_tar) - equivalent to the reference MATLAB's DD-direct ambiguity. Leaving
  // it loose makes the joint marginal ill-conditioned (and NaN once several
  // constellations stack their gauge directions).
  double ref_prior_sigma_cycles{1.0e-3};
  // Robust (Huber M-estimator) measurement noise, as in the reference MATLAB.
  bool robust{true};
  double huber_k{1.345};
};

// One carrier DD pair added to the graph this epoch; input to LAMBDA.
struct DdAmbiguityPair {
  gtsam::Key ref;
  gtsam::Key tar;
  double lam;
  double el{0.0};  // target-satellite elevation [rad] (for the AR elevation mask)
};

// 1-D double-difference measurement noise from the UNDIFFERENCED sigma
// (sigma_und = cfg.{pr,cp}_sigma_m, elevation-weighted: sigma_und/sin(el)), built
// by error propagation so the same sigma feeds both this float factor and the
// correlated AR covariance (RTKLIB ddcov; see resolveAmbiguitiesDd). A DD is
// SD_ref - SD_tar with SD_s = rover - base, so Var(DD) = Var(SD_ref) +
// Var(SD_tar) and Var(SD_s) = 2*sigma_und(el_s)^2. (The per-pair factor cannot
// express the off-diagonal reference correlation that the AR covariance does;
// that omission affects the float covariance, not the point estimate.)
inline gtsam::SharedNoiseModel ddNoise(double sigma_und, double el_ref,
                                       double el_tar, const AdapterConfig& cfg) {
  auto sd_var = [&](double el) {
    const double s = cfg.elevation_weighting ? std::max(std::sin(el), 0.1) : 1.0;
    return 2.0 * (sigma_und / s) * (sigma_und / s);
  };
  const double sigma_dd = std::sqrt(sd_var(el_ref) + sd_var(el_tar));
  gtsam::SharedNoiseModel base = gtsam::noiseModel::Isotropic::Sigma(1, sigma_dd);
  if (cfg.robust) {
    base = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(cfg.huber_k), base);
  }
  return base;
}

namespace detail {
// Per-epoch ambiguity allocator. Following the reference structure
// (Taro Suzuki gtsam_gnss / ambiguity_resolution.m), DD carrier ambiguities
// are NOT shared across epochs: every epoch gets a fresh set, each with a
// loose code-minus-carrier prior. Within one epoch a satellite that appears in
// several pairs (the reference satellite) reuses one key. `next_id` keeps keys
// globally unique across epochs so ISAM2 never collides.
//
// GTSAM's DD carrier factor parameterizes the pair as (N_ref, N_tar) and uses
// their difference, so each (sys,band) group keeps a one-dimensional gauge
// freedom; the per-key prior fixes it (the MATLAB parameterizes the ambiguity
// as the DD directly and so needs no such prior).
class EpochAmbiguities {
 public:
  EpochAmbiguities(const AdapterConfig& cfg, std::uint64_t& next_id,
                   gtsam::NonlinearFactorGraph& graph, gtsam::Values& values)
      : cfg_(cfg), next_id_(next_id), graph_(graph), values_(values) {}

  // is_reference: the per-group reference satellite gets a tight prior that
  // fixes the gauge; targets get a loose prior.
  gtsam::Key obtain(int sat, int band, double n0, bool is_reference) {
    const auto id = std::make_pair(sat, band);
    auto it = keys_.find(id);
    if (it != keys_.end()) return it->second;
    const gtsam::Key key = gtsam::symbol_shorthand::N(next_id_++);
    values_.insert(key, n0);
    const double sigma = is_reference ? cfg_.ref_prior_sigma_cycles
                                      : cfg_.amb_prior_sigma_cycles;
    graph_.add(gtsam::PriorFactor<double>(
        key, n0, gtsam::noiseModel::Isotropic::Sigma(1, sigma)));
    keys_[id] = key;
    return key;
  }

 private:
  const AdapterConfig& cfg_;
  std::uint64_t& next_id_;
  gtsam::NonlinearFactorGraph& graph_;
  gtsam::Values& values_;
  std::map<std::pair<int, int>, gtsam::Key> keys_;
};

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
}  // namespace detail

// Map every DD pair of one preprocessed epoch onto GTSAM factors keyed on a
// Point3 rover ECEF position. Ambiguities are fresh this epoch (allocated from
// next_amb_id). Returns the carrier pairs (for ambiguity resolution). This is
// the entire tight-coupling step.
inline std::vector<DdAmbiguityPair> addDdFactors(
    const gnss_utils::PreprocessedEpoch& ep, gtsam::Key position_key,
    std::uint64_t& next_amb_id, const AdapterConfig& cfg,
    gtsam::NonlinearFactorGraph& graph, gtsam::Values& values) {
  std::vector<DdAmbiguityPair> pairs;
  detail::EpochAmbiguities amb(cfg, next_amb_id, graph, values);
  for (const auto& d : ep.dd) {
    if (d.has_pr) {
      graph.add(gtsam::DoubleDifferencePseudorangeFactor(
          position_key,
          d.pr_rov_ref, d.pr_base_ref, d.pr_rov_tar, d.pr_base_tar,
          gtsam::Point3(d.sat_ref_rov), gtsam::Point3(d.sat_tar_rov),
          gtsam::Point3(d.sat_ref_base), gtsam::Point3(d.sat_tar_base),
          gtsam::Point3(ep.base_ecef),
          ddNoise(cfg.pr_sigma_m, d.el_ref, d.el_tar, cfg)));
    }
    if (d.has_cp) {
      const gtsam::Key k_ref = amb.obtain(
          d.sat_ref, d.band,
          detail::codeMinusCarrier(d.cp_rov_ref, d.cp_base_ref, d.pr_rov_ref,
                                   d.pr_base_ref, d.lam),
          true);
      const gtsam::Key k_tar = amb.obtain(
          d.sat_tar, d.band,
          detail::codeMinusCarrier(d.cp_rov_tar, d.cp_base_tar, d.pr_rov_tar,
                                   d.pr_base_tar, d.lam),
          false);
      graph.add(gtsam::DoubleDifferenceCarrierPhaseFactor(
          position_key, k_ref, k_tar,
          d.cp_rov_ref, d.cp_base_ref, d.cp_rov_tar, d.cp_base_tar,
          gtsam::Point3(d.sat_ref_rov), gtsam::Point3(d.sat_tar_rov),
          gtsam::Point3(d.sat_ref_base), gtsam::Point3(d.sat_tar_base),
          gtsam::Point3(ep.base_ecef), d.lam,
          ddNoise(cfg.cp_sigma_m, d.el_ref, d.el_tar, cfg)));
      pairs.push_back({k_ref, k_tar, d.lam, d.el_tar});
    }
  }
  return pairs;
}

// Same mapping as addDdFactors, but onto the "Arm" factor variants keyed on a
// body Pose3 in a local navigation frame: the factors compute the antenna
// position internally as ecef_T_nav * (pose * leverArm). This is the variant
// to combine with GTSAM's IMU preintegration (which needs a gravity-aligned
// nav frame), used by the tightly_coupled_gnss_imu example.
inline std::vector<DdAmbiguityPair> addDdFactorsArm(
    const gnss_utils::PreprocessedEpoch& ep, gtsam::Key pose_key,
    std::uint64_t& next_amb_id, const gtsam::Point3& lever_arm,
    const gtsam::Pose3& ecef_T_nav, const AdapterConfig& cfg,
    gtsam::NonlinearFactorGraph& graph, gtsam::Values& values) {
  std::vector<DdAmbiguityPair> pairs;
  detail::EpochAmbiguities amb(cfg, next_amb_id, graph, values);
  for (const auto& d : ep.dd) {
    if (d.has_pr) {
      graph.add(gtsam::DoubleDifferencePseudorangeFactorArm(
          pose_key,
          d.pr_rov_ref, d.pr_base_ref, d.pr_rov_tar, d.pr_base_tar,
          gtsam::Point3(d.sat_ref_rov), gtsam::Point3(d.sat_tar_rov),
          gtsam::Point3(d.sat_ref_base), gtsam::Point3(d.sat_tar_base),
          gtsam::Point3(ep.base_ecef), lever_arm, ecef_T_nav,
          ddNoise(cfg.pr_sigma_m, d.el_ref, d.el_tar, cfg)));
    }
    if (d.has_cp) {
      const gtsam::Key k_ref = amb.obtain(
          d.sat_ref, d.band,
          detail::codeMinusCarrier(d.cp_rov_ref, d.cp_base_ref, d.pr_rov_ref,
                                   d.pr_base_ref, d.lam),
          true);
      const gtsam::Key k_tar = amb.obtain(
          d.sat_tar, d.band,
          detail::codeMinusCarrier(d.cp_rov_tar, d.cp_base_tar, d.pr_rov_tar,
                                   d.pr_base_tar, d.lam),
          false);
      graph.add(gtsam::DoubleDifferenceCarrierPhaseFactorArm(
          pose_key, k_ref, k_tar,
          d.cp_rov_ref, d.cp_base_ref, d.cp_rov_tar, d.cp_base_tar,
          gtsam::Point3(d.sat_ref_rov), gtsam::Point3(d.sat_tar_rov),
          gtsam::Point3(d.sat_ref_base), gtsam::Point3(d.sat_tar_base),
          gtsam::Point3(ep.base_ecef), d.lam, lever_arm, ecef_T_nav,
          ddNoise(cfg.cp_sigma_m, d.el_ref, d.el_tar, cfg)));
      pairs.push_back({k_ref, k_tar, d.lam, d.el_tar});
    }
  }
  return pairs;
}

struct ArResult {
  bool fixed{false};
  // A best integer candidate was found and the state conditioned on it (set even
  // when the ratio test fails, so post-fix residual FDE can vet the candidate).
  bool candidate{false};
  double ratio{0.0};
  // Tangent-space correction to SUBTRACT from the state estimate at the state
  // key (3-dim for Point3, 6-dim for Pose3), and the conditional covariance of
  // that state given the fixed integers.
  Eigen::VectorXd state_correction;
  Eigen::MatrixXd state_cov;
  Eigen::VectorXd a_fix;    // fixed DD ambiguities [cycles], in `pairs` order
                            // (NaN for pairs dropped by partial AR)
  Eigen::VectorXd a_float;  // float DD ambiguities [cycles] (always populated)
  Eigen::MatrixXd Qa;       // float DD ambiguity covariance [cycles^2]
  std::vector<int> fixed_idx;  // indices into the DD list that were fixed
  // Conditioned (fixed) rover ECEF position [m]. Set by resolveAmbiguitiesDd
  // (which solves its own float internally); the graph-based resolveAmbiguities
  // leaves it zero and the caller applies state_correction instead.
  Eigen::Vector3d fixed_pos{Eigen::Vector3d::Zero()};
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
  if (k < std::max(min_fix, 1)) return;

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
  if (lambda(k, 2, a_sub.data(), Qa_sub.data(), F.data(), s) != 0 || s[1] <= 0.0)
    return;
  res.ratio = s[1] / std::max(s[0], 1e-12);

  // Condition the state on the best integer candidate:
  //   dx     = Qxa * Qa^-1 (a - a_fix)
  //   Qstate = Qxx - Qxa * Qa^-1 Qxa^T   (Schur complement)
  // guarding symmetry and positive-definiteness of both Qa and the result.
  const Eigen::VectorXd a_fix_sub = F.col(0);
  const Eigen::LDLT<Eigen::MatrixXd> solver = Qa_sub.ldlt();
  if (solver.info() != Eigen::Success || solver.vectorD().minCoeff() <= 0.0)
    return;
  Eigen::MatrixXd S = Qxx - Qxa_sub * solver.solve(Qxa_sub.transpose());
  S = 0.5 * (S + S.transpose());
  const Eigen::LDLT<Eigen::MatrixXd> s_ldlt(S);
  if (!S.allFinite() || s_ldlt.info() != Eigen::Success ||
      s_ldlt.vectorD().minCoeff() <= 0.0)
    return;
  const Eigen::VectorXd dc = Qxa_sub * solver.solve(a_sub - a_fix_sub);
  if (!dc.allFinite()) return;
  res.state_correction = dc;
  res.state_cov = S;
  for (int i = 0; i < k; ++i) {
    res.a_fix(active[i]) = a_fix_sub(i);
    res.fixed_idx.push_back(active[i]);
  }
  res.candidate = true;
  res.fixed = (res.ratio >= ratio_threshold);
}
}  // namespace detail

// Integer ambiguity resolution: form the DD-ambiguity vector a = D n and its
// covariance from the joint (state, ambiguity) marginal, search the integer
// candidates with RTKLIB's lambda(), ratio-test them, and return the
// conditioning update  x_fix = x - Qxa Qa^-1 (a - a_fix).
//
// local_subgraph: when true (the GNSS-only example), `factors` is this epoch's
// SELF-CONTAINED subgraph over {state_key} U akeys, and the joint covariance is
// the dense inverse of its information matrix. This is exact for the per-epoch
// independent structure and numerically robust (gtsam::Marginals' incremental
// elimination returns NaN on the wide dynamic range here - tight reference-
// ambiguity gauge prior vs loose position prior). When false (the GNSS/IMU
// example, where the state is linked to past epochs through the IMU chain),
// `factors` is the full graph and the joint marginal is taken with
// gtsam::Marginals.
//
// AR satellite selection: pairs below ar_el_mask_rad are excluded, then one
// LAMBDA + ratio test (no data-adaptive subset search). NOTE: this graph-based
// path uses the GTSAM factors' independent per-pair Qa, so its ratio is only
// approximately calibrated; the GNSS-only node uses resolveAmbiguitiesDd (the
// correctly-correlated covariance) instead. Kept for the GNSS/IMU example.
inline ArResult resolveAmbiguities(const std::vector<DdAmbiguityPair>& pairs,
                                   gtsam::Key state_key,
                                   const gtsam::NonlinearFactorGraph& factors,
                                   const gtsam::Values& estimate,
                                   double ratio_threshold,
                                   bool local_subgraph = true,
                                   double ar_el_mask_rad = 0.0,
                                   int min_fix = 4) {
  ArResult res;
  if (pairs.empty()) return res;

  // Unique ambiguity keys involved.
  std::vector<gtsam::Key> akeys;
  std::map<gtsam::Key, int> kidx;
  for (const auto& p : pairs) {
    for (const gtsam::Key k : {p.ref, p.tar}) {
      if (kidx.emplace(k, static_cast<int>(akeys.size())).second) {
        akeys.push_back(k);
      }
    }
  }
  const int m = static_cast<int>(akeys.size());
  const int n_dd = static_cast<int>(pairs.size());

  Eigen::MatrixXd Qxx, Qxn, Qnn;
  Eigen::VectorXd n_hat(m);
  for (int i = 0; i < m; ++i) n_hat(i) = estimate.at<double>(akeys[i]);

  try {
    if (local_subgraph) {
      gtsam::Ordering ordering;
      ordering.push_back(state_key);
      for (const gtsam::Key k : akeys) ordering.push_back(k);
      const Eigen::MatrixXd H =
          factors.linearize(estimate)->hessian(ordering).first;
      const int total = static_cast<int>(H.rows());
      const int sdim = total - m;
      if (sdim <= 0 || !H.allFinite()) return res;
      // Require a positive-definite information matrix (signed pivots): a
      // negative pivot is an indefinite/degenerate epoch, not a valid marginal.
      const Eigen::LDLT<Eigen::MatrixXd> h_ldlt(H);
      if (h_ldlt.info() != Eigen::Success || h_ldlt.vectorD().minCoeff() <= 0.0)
        return res;
      const Eigen::MatrixXd Cov =
          h_ldlt.solve(Eigen::MatrixXd::Identity(total, total));
      if (!Cov.allFinite()) return res;
      Qxx = Cov.topLeftCorner(sdim, sdim);
      Qxn = Cov.block(0, sdim, sdim, m);
      Qnn = Cov.block(sdim, sdim, m, m);
    } else {
      gtsam::KeyVector keys;
      keys.push_back(state_key);
      keys.insert(keys.end(), akeys.begin(), akeys.end());
      const gtsam::Marginals marginals(factors, estimate);
      const gtsam::JointMarginal jm = marginals.jointMarginalCovariance(keys);
      Qxx = jm(state_key, state_key);
      Qxn.resize(Qxx.rows(), m);
      Qnn.resize(m, m);
      for (int i = 0; i < m; ++i) {
        Qxn.col(i) = jm(state_key, akeys[i]);
        for (int j = 0; j < m; ++j) Qnn(i, j) = jm(akeys[i], akeys[j])(0, 0);
      }
    }
    if (!Qxx.allFinite() || !Qxn.allFinite() || !Qnn.allFinite()) return res;
  } catch (const std::exception&) {
    return res;
  }

  // DD differencing matrix: a = D n (reference minus target, cycles).
  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(n_dd, m);
  for (int i = 0; i < n_dd; ++i) {
    D(i, kidx[pairs[i].ref]) = 1.0;
    D(i, kidx[pairs[i].tar]) = -1.0;
  }
  const Eigen::VectorXd a = D * n_hat;
  res.a_float = a;
  const Eigen::MatrixXd Qa = D * Qnn * D.transpose();
  res.Qa = Qa;
  const Eigen::MatrixXd Qxa = Qxn * D.transpose();

  std::vector<double> el(n_dd);
  for (int i = 0; i < n_dd; ++i) el[i] = pairs[i].el;
  detail::lambdaFix(a, Qa, Qxa, Qxx, el, ratio_threshold, ar_el_mask_rad,
                    min_fix, res);
  return res;
}

// Analytical RTK ambiguity resolution with the correctly-correlated DD
// covariance. The GTSAM DD factors use independent per-pair noise, which gives a
// wrongly-shaped float ambiguity covariance and an uncalibrated ratio test; here
// the AR float ambiguities and covariance are solved directly from this epoch's
// DD observations with the textbook DD stochastic model
//   R_DD = D * diag(undiff var) * D^T   (per (sys,band) group: diagonal
//   Var(SD_ref)+Var(SD_tar), off-diagonal Var(SD_ref); RTKLIB ddcov),
// anchored by the motion-predicted position prior (x_pred, P_pred). The DD
// ambiguity is parameterised directly (one integer per carrier DD), so no gauge
// prior is needed. The conditioned (fixed) ECEF position is returned in
// res.fixed_pos. The GTSAM graph still produces the float trajectory used
// elsewhere; this routine only supplies the (correctly weighted) integer fix.
inline ArResult resolveAmbiguitiesDd(const gnss_utils::PreprocessedEpoch& ep,
                                     const Eigen::Vector3d& x_pred,
                                     const Eigen::Matrix3d& P_pred,
                                     const AdapterConfig& cfg,
                                     double ratio_threshold,
                                     double ar_el_mask_rad, int min_fix,
                                     bool fde_enable = false,
                                     double fde_threshold_m = 0.05,
                                     int fde_max_exclude = 2) {
  // A carrier DD is unusable for integer AR if either satellite has a cycle slip
  // or half-cycle ambiguity (LLI), the latter making the integer a half-integer.
  auto slipped = [](const gnss_utils::DdSignal& d) {
    return d.slip_ref || d.slip_tar;
  };
  // Sagnac-corrected geometric range and rcv->sat unit vector (matches
  // gtsam::gnss::geodist so the residual is consistent with the float factors).
  auto geodist = [](const Eigen::Vector3d& sat, const Eigen::Vector3d& rcv,
                    Eigen::Vector3d* e) {
    const Eigen::Vector3d d = sat - rcv;
    const double r = d.norm();
    if (e) *e = (r > 0.0) ? Eigen::Vector3d(d / r) : Eigen::Vector3d::Zero();
    return r + OMGE * (sat.x() * rcv.y() - sat.y() * rcv.x()) / CLIGHT;
  };
  // Single-difference (rover+base) measurement variance with RTKLIB-style
  // elevation weighting (variance proportional to 1/sin^2(el)).
  auto sd_var = [](double sigma, double el_rad) {
    const double s = std::max(std::sin(el_rad), 0.1);
    return 2.0 * (sigma / s) * (sigma / s);
  };
  // DD carrier-phase residual [m] of one DD at rover position x with integer N.
  auto ddCarrierResidual = [&](const gnss_utils::DdSignal& d,
                               const Eigen::Vector3d& x, double N) {
    Eigen::Vector3d e;
    const double model =
        (geodist(d.sat_ref_rov, x, &e) - geodist(d.sat_ref_base, ep.base_ecef, &e)) -
        (geodist(d.sat_tar_rov, x, &e) - geodist(d.sat_tar_base, ep.base_ecef, &e));
    const double obs =
        (d.cp_rov_ref - d.cp_base_ref) - (d.cp_rov_tar - d.cp_base_tar);
    return obs - model - d.lam * N;
  };

  // One AR attempt over the carrier DDs not in `excluded`. Returns the result
  // (res.candidate / res.fixed) plus the largest post-fix carrier residual and
  // the ep.dd index that produced it (for FDE).
  struct Attempt {
    ArResult res;
    int worst_dd{-1};
    double worst_resid{0.0};
  };
  auto attempt = [&](const std::set<int>& excluded) -> Attempt {
    Attempt out;
    ArResult& res = out.res;
    // Carrier DDs get one ambiguity each (indexed 0..n_cp-1); code DDs constrain
    // only the position. Cycle-slipped / half-cycle / FDE-excluded carriers out.
    std::vector<int> cp_idx;
    for (int i = 0; i < static_cast<int>(ep.dd.size()); ++i) {
      if (ep.dd[i].has_cp && !slipped(ep.dd[i]) && !excluded.count(i))
        cp_idx.push_back(i);
    }
    const int n_cp = static_cast<int>(cp_idx.size());
    if (n_cp < std::max(min_fix, 1)) return out;
    std::map<int, int> amb_col;  // ep.dd index -> ambiguity index
    for (int i = 0; i < n_cp; ++i) amb_col[cp_idx[i]] = i;

    const int nu = 3 + n_cp;  // unknowns: [dx(3), N_dd(n_cp)]
    Eigen::MatrixXd Info = Eigen::MatrixXd::Zero(nu, nu);
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(nu);
    // Motion-predicted prior on dx (LDLT solve, not explicit inverse).
    const Eigen::LDLT<Eigen::Matrix3d> p_ldlt(P_pred);
    if (p_ldlt.info() != Eigen::Success) return out;
    Info.topLeftCorner(3, 3) = p_ldlt.solve(Eigen::Matrix3d::Identity());

    // Group DDs by (sys,band): the shared reference satellite makes the DD
    // errors correlated within a group (off-diagonal Var(SD_ref)).
    std::map<std::pair<int, int>, std::vector<int>> groups;
    for (int i = 0; i < static_cast<int>(ep.dd.size()); ++i) {
      if (excluded.count(i)) continue;
      const auto& d = ep.dd[i];
      if (d.has_cp || d.has_pr) groups[{d.sys, d.band}].push_back(i);
    }
    for (const auto& kv : groups) {
      for (int phase = 0; phase < 2; ++phase) {  // 0: carrier, 1: code
        std::vector<int> rows;
        for (int di : kv.second) {
          const bool ok = phase == 0 ? (ep.dd[di].has_cp && !slipped(ep.dd[di]))
                                     : ep.dd[di].has_pr;
          if (ok) rows.push_back(di);
        }
        const int k = static_cast<int>(rows.size());
        if (k == 0) continue;
        const double sigma = (phase == 0) ? cfg.cp_sigma_m : cfg.pr_sigma_m;
        const double vref = sd_var(sigma, ep.dd[rows[0]].el_ref);  // shared ref
        Eigen::MatrixXd R(k, k), H = Eigen::MatrixXd::Zero(k, nu);
        Eigen::VectorXd r(k);
        for (int i = 0; i < k; ++i) {
          const auto& d = ep.dd[rows[i]];
          Eigen::Vector3d e_ref, e_tar, etmp;
          const double model =
              (geodist(d.sat_ref_rov, x_pred, &e_ref) -
               geodist(d.sat_ref_base, ep.base_ecef, &etmp)) -
              (geodist(d.sat_tar_rov, x_pred, &e_tar) -
               geodist(d.sat_tar_base, ep.base_ecef, &etmp));
          const double obs = (phase == 0)
              ? (d.cp_rov_ref - d.cp_base_ref) - (d.cp_rov_tar - d.cp_base_tar)
              : (d.pr_rov_ref - d.pr_base_ref) - (d.pr_rov_tar - d.pr_base_tar);
          r(i) = obs - model;
          H.block<1, 3>(i, 0) = (e_tar - e_ref).transpose();  // d(model)/d(rcv)
          if (phase == 0) H(i, 3 + amb_col[rows[i]]) = d.lam;  // + lam * N
          for (int j = 0; j < k; ++j) {
            R(i, j) = (i == j) ? vref + sd_var(sigma, d.el_tar) : vref;
          }
        }
        // Apply R^-1 by an LDLT solve (no explicit inverse); reject if R is not
        // positive-definite.
        const Eigen::LDLT<Eigen::MatrixXd> r_ldlt(R);
        if (r_ldlt.info() != Eigen::Success) return out;
        const Eigen::MatrixXd RinvH = r_ldlt.solve(H);
        const Eigen::VectorXd Rinvr = r_ldlt.solve(r);
        if (!RinvH.allFinite() || !Rinvr.allFinite()) return out;
        Info += H.transpose() * RinvH;
        rhs += H.transpose() * Rinvr;
      }
    }

    // Solve via LDLT; require positive-definiteness (signed pivots) and a sane
    // condition number (pivot magnitudes).
    const Eigen::LDLT<Eigen::MatrixXd> info_ldlt(Info);
    if (info_ldlt.info() != Eigen::Success) return out;
    const Eigen::VectorXd dvec = info_ldlt.vectorD();
    const Eigen::ArrayXd dabs = dvec.array().abs();
    if (dvec.minCoeff() <= 0.0 || dabs.maxCoeff() / dabs.minCoeff() > 1e14)
      return out;
    const Eigen::MatrixXd Cov =
        info_ldlt.solve(Eigen::MatrixXd::Identity(nu, nu));
    if (!Cov.allFinite()) return out;
    const Eigen::VectorXd u = Cov * rhs;  // [dx; N_float]
    if (!u.allFinite()) return out;
    const Eigen::Vector3d dx = u.head<3>();
    const Eigen::VectorXd a = u.tail(n_cp);
    const Eigen::Matrix3d Qxx = Cov.topLeftCorner<3, 3>();
    const Eigen::MatrixXd Qa = Cov.block(3, 3, n_cp, n_cp);
    const Eigen::MatrixXd Qxa = Cov.block(0, 3, 3, n_cp);
    res.a_float = a;
    res.Qa = Qa;

    std::vector<double> el(n_cp);
    for (int i = 0; i < n_cp; ++i) el[i] = ep.dd[cp_idx[i]].el_tar;
    detail::lambdaFix(a, Qa, Qxa, Qxx, el, ratio_threshold, ar_el_mask_rad,
                      min_fix, res);
    if (res.candidate) {
      res.fixed_pos = x_pred + dx - Eigen::Vector3d(res.state_correction);
      // Largest post-fix carrier residual identifies the worst outlier DD.
      for (int i = 0; i < n_cp; ++i) {
        if (!std::isfinite(res.a_fix(i))) continue;
        const double rr = std::abs(
            ddCarrierResidual(ep.dd[cp_idx[i]], res.fixed_pos, res.a_fix(i)));
        if (rr > out.worst_resid) {
          out.worst_resid = rr;
          out.worst_dd = cp_idx[i];
        }
      }
    }
    return out;
  };

  // FDE loop: fix, then check the post-fix carrier residuals. A residual far
  // above noise means a wrong integer on that DD (an undetected multipath /
  // half-cycle outlier); exclude it and re-fix. A fix is accepted only when its
  // residuals are all clean AND it passes the ratio test - so removing an
  // outlier can SALVAGE a float (the outlier had spoiled the ratio) and also
  // REJECTS a wrong fix that slipped past the ratio, without ever publishing a
  // fix that still contains an outlier.
  std::set<int> excluded;
  ArResult res;
  for (;;) {
    Attempt at = attempt(excluded);
    res = at.res;
    if (!res.candidate || !fde_enable) break;     // FDE off / no candidate
    if (at.worst_resid <= fde_threshold_m) break;  // residuals clean: trust ratio
    if (static_cast<int>(excluded.size()) >= fde_max_exclude ||
        at.worst_dd < 0) {
      res.fixed = false;  // outlier remains within budget: do not publish a fix
      break;
    }
    excluded.insert(at.worst_dd);  // drop the outlier and re-fix
  }
  return res;
}

}  // namespace gnss_fgo

#endif  // EXAMPLES_FGO_COMMON_FACTOR_ADAPTERS_HPP
