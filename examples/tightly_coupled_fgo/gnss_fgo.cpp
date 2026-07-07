// SPDX-License-Identifier: MIT
//
// Tightly-coupled GNSS factor-graph optimization example.
//
// Pipeline: the standardized topics (/gnss/observation, /base/gnss/observation,
// /gnss/ephemeris) are pushed into gnss_utils::GnssPreprocessor, which does all
// GNSS-domain work (epoch matching, satellite positions at transmission time,
// masks, reference-satellite selection, DD pairing, cycles->meters, and the
// undifferenced Doppler rover velocity). PreprocessedEpochs feed GTSAM's
// official GNSS factors (DoubleDifferencePseudorange/CarrierPhaseFactor, added
// to GTSAM in June 2026), optimized incrementally with ISAM2.
//
// Structure: the rover position is continuous across epochs (a
// constant-velocity motion model), exactly like RTKLIB, whose EKF always
// propagates position (dynamics on). Carrier ambiguities follow
// ambiguity.mode (RTKLIB armode): fresh per epoch (instantaneous, default),
// or carried across epochs and re-keyed on slip/outage (continuous /
// fix_and_hold; see gnss_fgo::AmbMode in factor_adapters.hpp).
//   State per epoch k:  X(k) rover ECEF position (gtsam::Point3)
//   Per (sat, band):    N(k) carrier ambiguity in cycles.
//   Inter-epoch:        BetweenFactor<Point3> = Doppler-derived displacement.
// Ambiguity resolution uses the correctly-correlated DD covariance (RTKLIB
// ddcov) solved analytically + RTKLIB's lambda() and the ratio test (canonical
// LAMBDA AR); see gnss_fgo::resolveAmbiguitiesDd in factor_adapters.hpp.
//
// NOTE: plain ISAM2 keeps the full history, so memory grows without bound
// (fine for an example / hours-scale sessions). A production node would use
// gtsam::IncrementalFixedLagSmoother instead.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include "factor_adapters.hpp"
#include "gnss_ros_standardization/gnss_preprocessor.hpp"
#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"

extern "C" {
#include "rtklib.h"
}

namespace grs = gnss_ros_standardization::msg;
using gtsam::symbol_shorthand::X;

namespace {
constexpr int kPositionLogThrottleMs = 1000;
constexpr int kMinDdForAr = 3;
}  // namespace

class GnssFgoNode : public rclcpp::Node {
 public:
  GnssFgoNode() : Node("gnss_fgo") {
    declareParameters();

    preprocessor_ = std::make_unique<gnss_utils::GnssPreprocessor>(makePreprocessorConfig());
    adapter_cfg_ = makeAdapterConfig();
    amb_mgr_ = std::make_unique<gnss_fgo::PersistentAmbiguities>(next_amb_id_);
    amb_max_outage_ = get_parameter("ambiguity.max_outage_s").as_double();

    gtsam::ISAM2Params isam_params;
    isam_params.relinearizeThreshold =
        get_parameter("isam2.relinearize_threshold").as_double();
    isam_params.relinearizeSkip = get_parameter("isam2.relinearize_skip").as_int();
    isam_ = std::make_unique<gtsam::ISAM2>(isam_params);

    cacheFixedOriginParam();

    rover_obs_sub_ = create_subscription<grs::GnssObservations>(
        get_parameter("topics.rover_observation").as_string(), rclcpp::QoS(50),
        std::bind(&GnssFgoNode::onRoverObs, this, std::placeholders::_1));
    base_obs_sub_ = create_subscription<grs::GnssObservations>(
        get_parameter("topics.base_observation").as_string(), rclcpp::QoS(50),
        std::bind(&GnssFgoNode::onBaseObs, this, std::placeholders::_1));
    nav_sub_ = create_subscription<grs::GnssEphemerides>(
        get_parameter("topics.ephemeris").as_string(),
        rclcpp::QoS(1).transient_local(),
        std::bind(&GnssFgoNode::onNav, this, std::placeholders::_1));
    sol_pub_ = create_publisher<grs::GnssSolution>(
        get_parameter("topics.solution").as_string(), 10);

    const char* mode_str =
        adapter_cfg_.mode == gnss_fgo::AmbMode::FixAndHold  ? "fix_and_hold"
        : adapter_cfg_.mode == gnss_fgo::AmbMode::Continuous ? "continuous"
                                                             : "instantaneous";
    RCLCPP_INFO(
        get_logger(),
        "Tightly-coupled GNSS FGO started (ambiguity mode %s, AR %s, ratio %.1f).",
        mode_str, ar_enable_ ? "on" : "off", ar_ratio_threshold_);
  }

 private:
  void declareParameters() {
    declare_parameter("topics.rover_observation", std::string("/gnss/observation"));
    declare_parameter("topics.base_observation", std::string("/base/gnss/observation"));
    declare_parameter("topics.ephemeris", std::string("/gnss/ephemeris"));
    declare_parameter("topics.solution", std::string("/gnss/fgo/solution"));

    // Base station antenna position. REQUIRED for DD: an error here shifts
    // the whole estimated trajectory by roughly the same amount.
    declare_parameter("base_position.postype", std::string("llh"));
    declare_parameter("base_position.pos", std::vector<double>{0.0, 0.0, 0.0});

    declare_parameter("fixed_origin.postype", std::string("llh"));
    declare_parameter("fixed_origin.pos", std::vector<double>{0.0, 0.0, 0.0});

    declare_parameter("masks.elevation_deg", 15.0);
    declare_parameter("masks.snr_dbhz", 0.0);
    // Elevation-dependent SNR mask, identical model to the RTK example
    // (RTKLIB testsnr): nine thresholds [dBHz] at 5,15,...,85 deg per band.
    // Disabled by default; the flat masks.snr_dbhz above is the fallback.
    declare_parameter("masks.snrmask.enable", false);
    declare_parameter("masks.snrmask.l1", std::vector<double>(9, 0.0));
    declare_parameter("masks.snrmask.l2", std::vector<double>(9, 0.0));
    declare_parameter("masks.snrmask.l5", std::vector<double>(9, 0.0));

    declare_parameter("navsys.gps", true);
    declare_parameter("navsys.glo", false);
    declare_parameter("navsys.gal", true);
    declare_parameter("navsys.bds", true);
    declare_parameter("navsys.qzs", true);
    declare_parameter("glonass_carrier_dd", false);

    declare_parameter("frequencies.enable_l1", true);
    declare_parameter("frequencies.enable_l2", true);
    declare_parameter("frequencies.enable_l5", false);

    declare_parameter("max_age_s", 1.0);

    // Rover-base cycle-slip detection (LLI cycle-slip + half-cycle-bit
    // transition + observed-code change are always on). Important for the
    // continuous / fix_and_hold modes (an undetected slip corrupts a carried
    // ambiguity). SD geometry-free (dual-freq), SD code-minus-carrier gross
    // error (excludes the corrupt carrier this epoch), Doppler-phase
    // (single-freq safeguard, common-clock removed), and a carrier-outage gap.
    declare_parameter("cycle_slip.gf_enable", true);
    declare_parameter("cycle_slip.gf_threshold_m", 0.05);
    declare_parameter("cycle_slip.cmc_enable", true);
    declare_parameter("cycle_slip.cmc_threshold_m", 3.0);
    declare_parameter("cycle_slip.dop_enable", true);
    declare_parameter("cycle_slip.dop_threshold_cyc", 1.0);
    declare_parameter("cycle_slip.max_gap_s", 2.0);

    declare_parameter("noise.pr_sigma_m", 1.0);
    declare_parameter("noise.cp_sigma_m", 0.01);
    declare_parameter("noise.elevation_weighting", true);

    declare_parameter("ambiguity.prior_sigma_cycles", 100.0);
    declare_parameter("ambiguity.ref_prior_sigma_cycles", 1.0e-3);

    // Ambiguity handling over time (RTKLIB armode): "instantaneous" (default,
    // fresh per epoch), "continuous" (carry ambiguities across epochs), or
    // "fix_and_hold" (continuous + inject accepted integers as tight priors).
    declare_parameter("ambiguity.mode", std::string("instantaneous"));
    declare_parameter("ambiguity.init_sigma_cycles", 20.0);
    // std dev [cycle] of the held integer DD constraint (RTKLIB
    // varholdamb=0.001 cycle^2 -> std ~0.03 cycle).
    declare_parameter("ambiguity.hold_sigma_cycles", 0.03);
    declare_parameter("ambiguity.max_outage_s", 5.0);

    // Robust (Huber) measurement noise on the DD factors, as in the reference.
    declare_parameter("robust.enable", true);
    declare_parameter("robust.huber_k", 1.345);

    declare_parameter("ambiguity_resolution.enable", true);
    declare_parameter("ambiguity_resolution.ratio_threshold", 3.0);
    // AR satellite selection: exclude cycle-slipped/half-cycle carriers and
    // pairs below el_mask_deg, then a single LAMBDA + ratio test on at least
    // min_fix DDs (canonical RTKLIB resamb_LAMBDA; no data-adaptive subset
    // search). The correlated DD covariance (resolveAmbiguitiesDd) makes the
    // ratio test statistically valid.
    declare_parameter("ambiguity_resolution.el_mask_deg", 15.0);
    declare_parameter("ambiguity_resolution.min_fix", 5);
    // Fault detection & exclusion (FDE): after fixing, check the post-fix carrier
    // residuals; if the largest exceeds fde_threshold_m (an undetected multipath
    // / half-cycle outlier), exclude that satellite and re-fix, up to
    // fde_max_exclude times. A fix is published only with clean residuals AND a
    // passing ratio - so an outlier can be removed to salvage a float, and a
    // wrong fix that slipped past the ratio is rejected.
    declare_parameter("ambiguity_resolution.fde_enable", false);
    declare_parameter("ambiguity_resolution.fde_threshold_m", 0.05);
    declare_parameter("ambiguity_resolution.fde_max_exclude", 2);
    // The conditioned FIX solution (ar.fixed_pos) is published but never fed
    // into ISAM2 directly, so a wrong fix can never corrupt the graph state.
    // fix_and_hold DOES feed the accepted integer back (as a gauge-free
    // relative constraint, see applyHolds below) once it passes the same
    // ratio test; that is the one exception to "FIX never affects the graph".

    // Loose prior on the rover position, used only on the first epoch and after
    // a motion gap (otherwise the position is carried by the motion factor).
    declare_parameter("position_prior.std_m", 100.0);

    // Inter-epoch motion model: a BetweenFactor<Point3> ties X(k) to X(k-1) by
    // the Doppler-derived displacement (mean = 0.5*(v_{k-1}+v_k)*dt). This is
    // what RTKLIB's EKF does in every armode (dynamics on); it smooths the
    // float position so per-epoch (instantaneous) AR can actually fix. The
    // factor sigma combines the Doppler velocity uncertainty and an unmodelled
    // acceleration term: sigma = sqrt((vel_sigma*dt)^2 + (0.5*accel_sigma*dt^2)^2).
    declare_parameter("motion.vel_sigma_mps", 0.1);   // floor on Doppler vel sigma
    declare_parameter("motion.accel_sigma_mps2", 3.0);  // unmodelled acceleration
    declare_parameter("motion.max_dt_s", 5.0);  // beyond this, reset (re-prior)

    declare_parameter("isam2.relinearize_threshold", 0.01);
    declare_parameter("isam2.relinearize_skip", 1);

    ar_enable_ = get_parameter("ambiguity_resolution.enable").as_bool();
    ar_ratio_threshold_ =
        get_parameter("ambiguity_resolution.ratio_threshold").as_double();
    ar_el_mask_ = get_parameter("ambiguity_resolution.el_mask_deg").as_double() * D2R;
    ar_min_fix_ = static_cast<int>(get_parameter("ambiguity_resolution.min_fix").as_int());
    ar_fde_enable_ = get_parameter("ambiguity_resolution.fde_enable").as_bool();
    ar_fde_threshold_m_ =
        get_parameter("ambiguity_resolution.fde_threshold_m").as_double();
    ar_fde_max_exclude_ = static_cast<int>(
        get_parameter("ambiguity_resolution.fde_max_exclude").as_int());
    position_prior_std_ = get_parameter("position_prior.std_m").as_double();
    motion_vel_sigma_ = get_parameter("motion.vel_sigma_mps").as_double();
    motion_accel_sigma_ = get_parameter("motion.accel_sigma_mps2").as_double();
    motion_max_dt_ = get_parameter("motion.max_dt_s").as_double();
  }

  gnss_utils::GnssPreprocessor::Config makePreprocessorConfig() {
    gnss_utils::GnssPreprocessor::Config cfg;
    cfg.el_mask_rad = get_parameter("masks.elevation_deg").as_double() * D2R;
    cfg.snr_mask_dbhz = get_parameter("masks.snr_dbhz").as_double();
    // Elevation-dependent SNR mask (RTKLIB snrmask_t / testsnr). When enabled it
    // takes precedence over the flat snr_mask_dbhz; same model as the RTK example.
    cfg.snr_mask = snrmask_t{};
    if (get_parameter("masks.snrmask.enable").as_bool()) {
      cfg.snr_mask.ena[0] = 1;  // rover only (the base stream has no SNR mask)
      const auto l1 = get_parameter("masks.snrmask.l1").as_double_array();
      const auto l2 = get_parameter("masks.snrmask.l2").as_double_array();
      const auto l5 = get_parameter("masks.snrmask.l5").as_double_array();
      for (int i = 0; i < 9; ++i) {
        if (i < static_cast<int>(l1.size())) cfg.snr_mask.mask[0][i] = l1[i];
        if (i < static_cast<int>(l2.size())) cfg.snr_mask.mask[1][i] = l2[i];
        if (i < static_cast<int>(l5.size())) cfg.snr_mask.mask[2][i] = l5[i];
      }
    }
    cfg.navsys = 0;
    if (get_parameter("navsys.gps").as_bool()) cfg.navsys |= SYS_GPS;
    if (get_parameter("navsys.glo").as_bool()) cfg.navsys |= SYS_GLO;
    if (get_parameter("navsys.gal").as_bool()) cfg.navsys |= SYS_GAL;
    if (get_parameter("navsys.bds").as_bool()) cfg.navsys |= SYS_CMP;
    if (get_parameter("navsys.qzs").as_bool()) cfg.navsys |= SYS_QZS;
    cfg.glonass_carrier_dd = get_parameter("glonass_carrier_dd").as_bool();
    cfg.bands.clear();
    if (get_parameter("frequencies.enable_l1").as_bool()) cfg.bands.push_back(0);
    if (get_parameter("frequencies.enable_l2").as_bool()) cfg.bands.push_back(1);
    if (get_parameter("frequencies.enable_l5").as_bool()) cfg.bands.push_back(2);
    if (cfg.bands.empty()) cfg.bands.push_back(0);
    cfg.max_age_s = get_parameter("max_age_s").as_double();
    cfg.detect_slip_gf = get_parameter("cycle_slip.gf_enable").as_bool();
    cfg.slip_gf_threshold_m = get_parameter("cycle_slip.gf_threshold_m").as_double();
    cfg.detect_slip_cmc = get_parameter("cycle_slip.cmc_enable").as_bool();
    cfg.slip_cmc_threshold_m = get_parameter("cycle_slip.cmc_threshold_m").as_double();
    cfg.detect_slip_dop = get_parameter("cycle_slip.dop_enable").as_bool();
    cfg.slip_dop_threshold_cyc =
        get_parameter("cycle_slip.dop_threshold_cyc").as_double();
    cfg.slip_max_gap_s = get_parameter("cycle_slip.max_gap_s").as_double();

    const std::string postype = get_parameter("base_position.postype").as_string();
    const auto pos = get_parameter("base_position.pos").as_double_array();
    if (pos.size() == 3 && norm(pos.data(), 3) > 0.0) {
      double ecef[3];
      if (postype == "llh") {
        const double llh[3] = {pos[0] * D2R, pos[1] * D2R, pos[2]};
        pos2ecef(llh, ecef);
      } else {
        std::copy_n(pos.data(), 3, ecef);
      }
      cfg.base_ecef = Eigen::Vector3d(ecef[0], ecef[1], ecef[2]);
      RCLCPP_INFO(get_logger(), "Base position: ECEF %.3f %.3f %.3f",
                  ecef[0], ecef[1], ecef[2]);
    } else {
      RCLCPP_ERROR(get_logger(),
                   "base_position is not set - no DD pairs will be formed.");
    }
    return cfg;
  }

  gnss_fgo::AdapterConfig makeAdapterConfig() {
    gnss_fgo::AdapterConfig cfg;
    cfg.pr_sigma_m = get_parameter("noise.pr_sigma_m").as_double();
    cfg.cp_sigma_m = get_parameter("noise.cp_sigma_m").as_double();
    cfg.elevation_weighting = get_parameter("noise.elevation_weighting").as_bool();
    cfg.amb_prior_sigma_cycles =
        get_parameter("ambiguity.prior_sigma_cycles").as_double();
    cfg.ref_prior_sigma_cycles =
        get_parameter("ambiguity.ref_prior_sigma_cycles").as_double();
    cfg.robust = get_parameter("robust.enable").as_bool();
    cfg.huber_k = get_parameter("robust.huber_k").as_double();
    const std::string mode = get_parameter("ambiguity.mode").as_string();
    if (mode == "continuous") {
      cfg.mode = gnss_fgo::AmbMode::Continuous;
    } else if (mode == "fix_and_hold") {
      cfg.mode = gnss_fgo::AmbMode::FixAndHold;
    } else {
      cfg.mode = gnss_fgo::AmbMode::Instantaneous;
    }
    cfg.init_sigma_cycles = get_parameter("ambiguity.init_sigma_cycles").as_double();
    cfg.hold_sigma_cycles = get_parameter("ambiguity.hold_sigma_cycles").as_double();
    return cfg;
  }

  void cacheFixedOriginParam() {
    const std::string type = get_parameter("fixed_origin.postype").as_string();
    const auto pos = get_parameter("fixed_origin.pos").as_double_array();
    if (pos.size() == 3 && norm(pos.data(), 3) > 0.0) {
      if (type == "llh") {
        const double llh[3] = {pos[0] * D2R, pos[1] * D2R, pos[2]};
        pos2ecef(llh, fixed_origin_ecef_);
      } else {
        std::copy_n(pos.data(), 3, fixed_origin_ecef_);
      }
      fixed_origin_valid_ = true;
    }
  }

  void onNav(const grs::GnssEphemerides::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    preprocessor_->pushEphemerides(*msg);
  }

  void onRoverObs(const grs::GnssObservations::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    preprocessor_->pushRoverObs(msg);
    processEpochs();
  }

  void onBaseObs(const grs::GnssObservations::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    preprocessor_->pushBaseObs(msg);
    processEpochs();
  }

  void processEpochs() {
    const auto epochs = preprocessor_->drainEpochs(
        last_estimate_valid_ ? &last_estimate_ : nullptr);
    for (const auto& ep : epochs) processEpoch(ep);
    for (const auto& d : preprocessor_->takeDropped()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Dropped rover obs (tow %.3f) - no base within window (newest base %.3f)",
          d.rover_tow, d.newest_base_tow);
    }
  }

  void processEpoch(const gnss_utils::PreprocessedEpoch& ep) {
    if (ep.dd.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Epoch tow %.3f has no DD pairs (check base stream / base_position).",
          ep.tow);
      return;
    }

    const gtsam::Key xk = X(epoch_index_);
    const gtsam::Point3 apriori(ep.rover_ecef_apriori);

    // Motion prediction: connect this epoch's position to the previous one by
    // the Doppler-derived displacement (mean = 0.5*(v_{k-1}+v_k)*dt). This is
    // what every RTKLIB armode does with its EKF (dynamics on); it smooths the
    // float position so per-epoch (instantaneous) AR can fix. The predicted
    // prior (x_pred, P_pred) summarises all past + motion information - the
    // Markov blanket of {X(k), N} - so the AR subgraph below stays small.
    // Continuous GPST seconds (week rollover safe) for all time differencing;
    // the message time_tow stays the raw tow.
    const double t_gnss = ep.week * 604800.0 + ep.tow;
    const double dt = last_float_valid_ ? (t_gnss - prev_tow_) : 0.0;
    const bool continuous = last_float_valid_ && dt > 1e-3 && dt <= motion_max_dt_;

    Eigen::Vector3d x_pred = apriori;
    Eigen::Matrix3d P_pred =
        Eigen::Matrix3d::Identity() * (position_prior_std_ * position_prior_std_);
    Eigen::Vector3d delta = Eigen::Vector3d::Zero();
    double motion_sigma = position_prior_std_;
    if (continuous) {
      const Eigen::Vector3d v_now =
          ep.rover_vel_valid ? ep.rover_vel_ecef
          : (prev_vel_valid_ ? prev_vel_ : Eigen::Vector3d::Zero());
      const Eigen::Vector3d v_prev = prev_vel_valid_ ? prev_vel_ : v_now;
      delta = 0.5 * (v_prev + v_now) * dt;
      double vel_sigma = motion_vel_sigma_;
      if (ep.rover_vel_valid)
        vel_sigma = std::max(vel_sigma, std::sqrt(ep.rover_vel_var));
      motion_sigma = std::max(
          std::sqrt(std::pow(vel_sigma * dt, 2) +
                    std::pow(0.5 * motion_accel_sigma_ * dt * dt, 2)),
          1e-3);
      x_pred = prev_float_pos_ + delta;
      P_pred = prev_float_cov_ +
               Eigen::Matrix3d::Identity() * (motion_sigma * motion_sigma);
    }

    // DD factors. Instantaneous: fresh per-epoch ambiguities. Continuous /
    // FixAndHold: carried ambiguities (amb_mgr_), re-keyed on cycle slip or an
    // outage longer than amb_max_outage_.
    const bool carry = adapter_cfg_.mode != gnss_fgo::AmbMode::Instantaneous;
    gnss_fgo::PersistentAmbiguities* amb_ptr = carry ? amb_mgr_.get() : nullptr;
    if (carry) amb_mgr_->retireStale(t_gnss, amb_max_outage_);

    gtsam::NonlinearFactorGraph epoch_graph;
    gtsam::Values values;
    values.insert(xk, gtsam::Point3(x_pred));
    const auto pairs = gnss_fgo::addDdFactors(ep, xk, next_amb_id_, adapter_cfg_,
                                              epoch_graph, values, amb_ptr);

    // Continuous trajectory in ISAM2: tie X(k) to X(k-1) with the Doppler
    // motion (BetweenFactor), or anchor with the loose prior on the first epoch
    // / after a motion gap.
    gtsam::NonlinearFactorGraph isam_graph = epoch_graph;
    if (continuous) {
      // Robust (Huber) motion model: Doppler velocity degrades at low speed
      // (range-rate noise / clock-drift artefacts dominate), so down-weight
      // increments that disagree with the rest of the graph instead of letting
      // a noisy low-speed Doppler pull the float trajectory.
      gtsam::SharedNoiseModel mnoise =
          gtsam::noiseModel::Isotropic::Sigma(3, motion_sigma);
      if (adapter_cfg_.robust) {
        mnoise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(adapter_cfg_.huber_k),
            mnoise);
      }
      isam_graph.add(gtsam::BetweenFactor<gtsam::Point3>(
          X(epoch_index_ - 1), xk, gtsam::Point3(delta), mnoise));
    } else {
      isam_graph.add(gtsam::PriorFactor<gtsam::Point3>(
          xk, apriori,
          gtsam::noiseModel::Isotropic::Sigma(3, position_prior_std_)));
    }

    // The ISAM2 update and the marginal/estimate queries can all throw an
    // IndeterminantLinearSystemException on a degenerate epoch (e.g. an
    // under-constrained ambiguity); guard them together and reset on failure.
    gtsam::Point3 float_pos;
    Eigen::Matrix3d float_cov;
    try {
      isam_->update(isam_graph, values);
      float_pos = isam_->calculateEstimate<gtsam::Point3>(xk);
      float_cov = isam_->marginalCovariance(xk);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "ISAM2 solve failed (%s) - resetting graph.",
                  e.what());
      resetGraph();
      return;
    }

    Eigen::Vector3d pub_pos = float_pos;
    Eigen::Matrix3d pub_cov = float_cov;
    uint8_t status = grs::GnssSolution::STATUS_FLOAT;
    double ratio = 0.0;

    gnss_fgo::ArResult ar;
    if (ar_enable_ && static_cast<int>(pairs.size()) >= kMinDdForAr) {
      // Analytical AR with the correctly-correlated DD covariance (RTKLIB ddcov)
      // + FDE, anchored by the motion-predicted prior (x_pred, P_pred). Used in
      // every mode - the well-calibrated ratio test and FDE the GTSAM per-pair
      // covariance cannot give. In continuous / fix_and_hold the carried
      // ambiguities tighten the float trajectory, so P_pred is tighter and this
      // same AR fixes more reliably; the graph merely supplies the better float.
      ar = gnss_fgo::resolveAmbiguitiesDd(
          ep, x_pred, P_pred, adapter_cfg_, ar_ratio_threshold_, ar_el_mask_,
          ar_min_fix_, ar_fde_enable_, ar_fde_threshold_m_, ar_fde_max_exclude_);
      ratio = ar.ratio;
      if (ar.fixed) {
        pub_pos = ar.fixed_pos;
        pub_cov = ar.state_cov;
        status = grs::GnssSolution::STATUS_FIX;
      }
    }
    if (adapter_cfg_.mode == gnss_fgo::AmbMode::FixAndHold) {
      // Hold accepted integers as two-variable DD constraints on the carried
      // keys (added once, removed on re-key). Runs every epoch so stale holds
      // are cleaned even when this epoch produced no fix. The published FIX
      // stays the analytical ar.fixed_pos; carry the now-tightened float forward.
      const auto specs = gnss_fgo::collectHoldSpecs(*amb_mgr_, ep, ar);
      const auto hr = gnss_fgo::applyHolds(*isam_, *amb_mgr_, held_dd_, specs,
                                           adapter_cfg_.hold_sigma_cycles);
      if (hr == gnss_fgo::HoldResult::Failure) {
        // ISAM2 may be inconsistent after a thrown hold update; reset and skip.
        RCLCPP_WARN(get_logger(), "Hold update failed - resetting graph.");
        resetGraph();
        return;
      }
      if (hr == gnss_fgo::HoldResult::Success) {
        try {
          float_pos = isam_->calculateEstimate<gtsam::Point3>(xk);
          float_cov = isam_->marginalCovariance(xk);
          // Publish the now-tightened FLOAT this epoch too (a new FIX keeps the
          // analytical ar.fixed_pos); otherwise the held cm-level constraint
          // would not reach the current output.
          if (status != grs::GnssSolution::STATUS_FIX) {
            pub_pos = float_pos;
            pub_cov = float_cov;
          }
        } catch (const std::exception& e) {
          // The estimate/marginal is degenerate after the hold update; do not
          // carry a possibly-broken ISAM2 into the next epoch.
          RCLCPP_WARN(get_logger(),
                      "Post-hold estimate failed (%s) - resetting graph.",
                      e.what());
          resetGraph();
          return;
        }
      }
    }

    publishSolution(ep, pub_pos, pub_cov, status, ratio);

    double llh[3];
    const double ecef[3] = {pub_pos.x(), pub_pos.y(), pub_pos.z()};
    ecef2pos(ecef, llh);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), kPositionLogThrottleMs,
        "FGO %s | LLH: %.8f %.8f %.3f | Ratio: %.1f | DD: %zu | Age: %.1f",
        status == grs::GnssSolution::STATUS_FIX ? "FIX" : "FLOAT",
        llh[0] * R2D, llh[1] * R2D, llh[2], ratio, ep.dd.size(), ep.age_s);

    // Carry the FLOAT estimate forward (in fix_and_hold this is the
    // hold-tightened float, above; the published FIX position itself is never
    // fed back, see the ambiguity_resolution.fde_* comment above).
    prev_float_pos_ = float_pos;
    prev_float_cov_ = float_cov;
    last_float_valid_ = true;
    prev_tow_ = t_gnss;  // continuous GPST seconds
    if (ep.rover_vel_valid) {
      prev_vel_ = ep.rover_vel_ecef;
      prev_vel_valid_ = true;
    }
    last_estimate_ = pub_pos;  // preprocessor a-priori for the next epoch
    last_estimate_valid_ = true;
    ++epoch_index_;
  }

  void resetGraph() {
    gtsam::ISAM2Params isam_params;
    isam_params.relinearizeThreshold =
        get_parameter("isam2.relinearize_threshold").as_double();
    isam_params.relinearizeSkip = get_parameter("isam2.relinearize_skip").as_int();
    isam_ = std::make_unique<gtsam::ISAM2>(isam_params);
    epoch_index_ = 0;
    next_amb_id_ = 0;
    last_estimate_valid_ = false;
    last_float_valid_ = false;
    prev_vel_valid_ = false;
    if (amb_mgr_) amb_mgr_->resetAll();
    held_dd_.clear();
  }

  void publishSolution(const gnss_utils::PreprocessedEpoch& ep,
                       const Eigen::Vector3d& pos, const Eigen::Matrix3d& cov,
                       uint8_t status, double ratio) {
    auto msg = std::make_unique<grs::GnssSolution>();
    // Inherit the observation epoch stamp (not now()): keeps the solution's
    // header time tied to the input, so a downstream time-aligned consumer does
    // not fold FGO processing / bag-replay wall-clock into the measurement time.
    msg->header.stamp = ep.stamp;
    msg->header.frame_id = "gnss_link";
    msg->time_week = ep.week;
    msg->time_tow = ep.tow;
    msg->solution_source = grs::GnssSolution::SOLUTION_SOURCE_COMPUTED;
    msg->status = status;
    msg->ratio = static_cast<float>(ratio);
    msg->age_diff = static_cast<float>(ep.age_s);

    std::set<int> sats;
    for (const auto& d : ep.dd) {
      sats.insert(d.sat_ref);
      sats.insert(d.sat_tar);
    }
    msg->num_sats = static_cast<uint8_t>(sats.size());

    const float nan = std::numeric_limits<float>::quiet_NaN();
    msg->gdop = nan;
    msg->pdop = nan;
    msg->hdop = nan;
    msg->vdop = nan;

    const double ecef[3] = {pos.x(), pos.y(), pos.z()};
    double llh[3];
    ecef2pos(ecef, llh);
    msg->latitude = llh[0] * R2D;
    msg->longitude = llh[1] * R2D;
    msg->altitude = llh[2];
    msg->pos_ecef.x = pos.x();
    msg->pos_ecef.y = pos.y();
    msg->pos_ecef.z = pos.z();
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) msg->pos_cov_ecef[3 * r + c] = cov(r, c);
    }

    // ENU origin priority: fixed_origin (configured) > base station.
    double origin[3] = {0.0, 0.0, 0.0};
    if (fixed_origin_valid_) {
      std::copy_n(fixed_origin_ecef_, 3, origin);
    } else if (ep.base_ecef.norm() > 0.0) {
      origin[0] = ep.base_ecef.x();
      origin[1] = ep.base_ecef.y();
      origin[2] = ep.base_ecef.z();
    }
    msg->pos_enu_org_ecef.x = origin[0];
    msg->pos_enu_org_ecef.y = origin[1];
    msg->pos_enu_org_ecef.z = origin[2];
    if (norm(origin, 3) > 0.0) {
      double origin_llh[3];
      ecef2pos(origin, origin_llh);
      const double d_ecef[3] = {ecef[0] - origin[0], ecef[1] - origin[1],
                                ecef[2] - origin[2]};
      double enu[3];
      ecef2enu(origin_llh, d_ecef, enu);
      msg->pos_enu.x = enu[0];
      msg->pos_enu.y = enu[1];
      msg->pos_enu.z = enu[2];
    }

    double q_ecef[9];
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) q_ecef[3 * r + c] = cov(r, c);
    }
    double q_enu[9];
    gnss_utils::rotateCovariance(q_ecef, llh[0], llh[1], q_enu);
    for (int i = 0; i < 9; ++i) msg->pos_enu_cov[i] = q_enu[i];

    // Doppler-derived rover velocity (ECEF + ENU), the same quantity that drives
    // the inter-epoch motion factor.
    if (ep.rover_vel_valid) {
      msg->vel_ecef.x = ep.rover_vel_ecef.x();
      msg->vel_ecef.y = ep.rover_vel_ecef.y();
      msg->vel_ecef.z = ep.rover_vel_ecef.z();
      const double vel_ecef[3] = {ep.rover_vel_ecef.x(), ep.rover_vel_ecef.y(),
                                  ep.rover_vel_ecef.z()};
      double vel_enu[3];
      ecef2enu(llh, vel_ecef, vel_enu);
      msg->vel_enu.x = vel_enu[0];
      msg->vel_enu.y = vel_enu[1];
      msg->vel_enu.z = vel_enu[2];
      for (int i = 0; i < 9; ++i) {
        msg->vel_cov_ecef[i] = (i % 4 == 0) ? ep.rover_vel_var : 0.0;
        msg->vel_enu_cov[i] = (i % 4 == 0) ? ep.rover_vel_var : 0.0;
      }
    }

    sol_pub_->publish(std::move(msg));
  }

  std::mutex mtx_;
  std::unique_ptr<gnss_utils::GnssPreprocessor> preprocessor_;
  std::unique_ptr<gtsam::ISAM2> isam_;
  gnss_fgo::AdapterConfig adapter_cfg_;

  int epoch_index_{0};
  std::uint64_t next_amb_id_{0};  // globally-unique ambiguity keys
  // Carried-across-epochs ambiguities (Continuous / FixAndHold modes only).
  std::unique_ptr<gnss_fgo::PersistentAmbiguities> amb_mgr_;
  double amb_max_outage_{5.0};
  gnss_fgo::HeldDdMap held_dd_;  // FixAndHold: held DD constraints (by sat pair)
  Eigen::Vector3d last_estimate_{Eigen::Vector3d::Zero()};
  bool last_estimate_valid_{false};

  // Filter state carried between epochs for the motion model.
  double prev_tow_{0.0};
  Eigen::Vector3d prev_vel_{Eigen::Vector3d::Zero()};
  bool prev_vel_valid_{false};
  Eigen::Vector3d prev_float_pos_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d prev_float_cov_{Eigen::Matrix3d::Identity()};
  bool last_float_valid_{false};

  bool ar_enable_{true};
  double ar_ratio_threshold_{3.0};
  double ar_el_mask_{15.0 * D2R};
  int ar_min_fix_{5};
  bool ar_fde_enable_{false};
  double ar_fde_threshold_m_{0.05};
  int ar_fde_max_exclude_{2};
  double position_prior_std_{100.0};
  double motion_vel_sigma_{0.1};
  double motion_accel_sigma_{3.0};
  double motion_max_dt_{5.0};

  double fixed_origin_ecef_[3]{0.0, 0.0, 0.0};
  bool fixed_origin_valid_{false};

  rclcpp::Subscription<grs::GnssObservations>::SharedPtr rover_obs_sub_;
  rclcpp::Subscription<grs::GnssObservations>::SharedPtr base_obs_sub_;
  rclcpp::Subscription<grs::GnssEphemerides>::SharedPtr nav_sub_;
  rclcpp::Publisher<grs::GnssSolution>::SharedPtr sol_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssFgoNode>());
  rclcpp::shutdown();
  return 0;
}
