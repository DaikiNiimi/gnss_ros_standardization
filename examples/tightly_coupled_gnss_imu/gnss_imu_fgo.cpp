// SPDX-License-Identifier: MIT
//
// Tightly-coupled GNSS/IMU factor-graph optimization example.
//
// Extends the tightly_coupled_gnss example with GTSAM's IMU preintegration:
// between consecutive GNSS epochs the raw IMU samples are integrated into a
// PreintegratedCombinedMeasurements and added as one CombinedImuFactor; the
// DD pseudorange / carrier-phase measurements attach to the same body pose
// through the "Arm" factor variants (lever arm + ecef_T_nav).
//
// Frame design (the core of this example):
//   - The IMU factor needs a gravity-aligned navigation frame. Poses X(k)
//     therefore live in a local ENU frame anchored at the first GNSS fix.
//   - The DD factors are inherently ECEF. The Arm variants take the constant
//     ecef_T_nav transform and convert internally - exactly the bridge needed
//     to couple both sensor types in one graph.
//
// State per epoch k: X(k) body Pose3 (ENU), V(k) velocity (ENU),
//                    B(k) IMU bias; N(j) carrier ambiguities in cycles.
//
// Time bases: GNSS epochs are paired with IMU samples via the rover
// observation's ROS header stamp (PC arrival time, like the loose EKF does).
// Receiver output latency therefore shifts the IMU integration boundaries by
// tens of milliseconds - an accepted approximation for this example.
//
// Initialization: the platform is assumed STATIC for the first
// init_imu_duration seconds. Roll/pitch come from the averaged accelerometer,
// the gyro bias from the averaged gyro; yaw is unobservable until the
// platform accelerates/turns (prior sigma = pi).
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include "../fgo_common/factor_adapters.hpp"
#include "gnss_ros_standardization/gnss_preprocessor.hpp"
#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"

extern "C" {
#include "rtklib.h"
}

namespace grs = gnss_ros_standardization::msg;
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

namespace {
constexpr int kPositionLogThrottleMs = 1000;
constexpr int kMinDdForAr = 3;
constexpr double kGravity = 9.80665;
}  // namespace

class GnssImuFgoNode : public rclcpp::Node {
 public:
  GnssImuFgoNode() : Node("gnss_imu_fgo") {
    declareParameters();

    preprocessor_ = std::make_unique<gnss_utils::GnssPreprocessor>(makePreprocessorConfig());
    adapter_cfg_ = makeAdapterConfig();

    gtsam::ISAM2Params isam_params;
    isam_params.relinearizeThreshold =
        get_parameter("isam2.relinearize_threshold").as_double();
    isam_params.relinearizeSkip = get_parameter("isam2.relinearize_skip").as_int();
    isam_ = std::make_unique<gtsam::ISAM2>(isam_params);

    const auto lever = get_parameter("lever_arm").as_double_array();
    if (lever.size() == 3) lever_arm_ = gtsam::Point3(lever[0], lever[1], lever[2]);

    rover_obs_sub_ = create_subscription<grs::GnssObservations>(
        get_parameter("topics.rover_observation").as_string(), rclcpp::QoS(50),
        std::bind(&GnssImuFgoNode::onRoverObs, this, std::placeholders::_1));
    base_obs_sub_ = create_subscription<grs::GnssObservations>(
        get_parameter("topics.base_observation").as_string(), rclcpp::QoS(50),
        std::bind(&GnssImuFgoNode::onBaseObs, this, std::placeholders::_1));
    nav_sub_ = create_subscription<grs::GnssEphemerides>(
        get_parameter("topics.ephemeris").as_string(),
        rclcpp::QoS(1).transient_local(),
        std::bind(&GnssImuFgoNode::onNav, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        get_parameter("topics.imu").as_string(), rclcpp::QoS(200),
        std::bind(&GnssImuFgoNode::onImu, this, std::placeholders::_1));
    sol_pub_ = create_publisher<grs::GnssSolution>(
        get_parameter("topics.solution").as_string(), 10);
    // Full navigation state (position + attitude + velocity) - the tightly-
    // coupled formulation estimates attitude that GnssSolution cannot carry.
    // Same convention as the GNSS/IMU EKF example's <solution>_odom topic.
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        get_parameter("topics.solution").as_string() + "_odom", 10);

    RCLCPP_INFO(get_logger(),
                "Tightly-coupled GNSS/IMU FGO started (lever arm %.3f %.3f %.3f).",
                lever_arm_.x(), lever_arm_.y(), lever_arm_.z());
  }

 private:
  struct ImuSample {
    double t;  // ROS time [s]
    Eigen::Vector3d acc;
    Eigen::Vector3d gyr;
  };

  void declareParameters() {
    declare_parameter("topics.rover_observation", std::string("/gnss/observation"));
    declare_parameter("topics.base_observation", std::string("/base/gnss/observation"));
    declare_parameter("topics.ephemeris", std::string("/gnss/ephemeris"));
    declare_parameter("topics.imu", std::string("/gnss/imu/data_raw"));
    declare_parameter("topics.solution", std::string("/gnss/fgo/solution"));

    declare_parameter("base_position.postype", std::string("llh"));
    declare_parameter("base_position.pos", std::vector<double>{0.0, 0.0, 0.0});

    // Body-frame translation from the body/IMU origin to the GNSS antenna
    // phase center [m].
    declare_parameter("lever_arm", std::vector<double>{0.0, 0.0, 0.0});

    // ENU anchor: "gnss_fix" anchors at the first GNSS a-priori position,
    // "manual" uses local_origin.pos.
    declare_parameter("local_origin.mode", std::string("gnss_fix"));
    declare_parameter("local_origin.postype", std::string("llh"));
    declare_parameter("local_origin.pos", std::vector<double>{0.0, 0.0, 0.0});

    declare_parameter("masks.elevation_deg", 15.0);
    declare_parameter("masks.snr_dbhz", 0.0);
    // Elevation-dependent SNR mask (RTKLIB testsnr), same model as gnss_fgo:
    // nine thresholds [dBHz] at 5,15,...,85 deg per band. When enabled it takes
    // precedence over the flat masks.snr_dbhz above.
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

    declare_parameter("noise.pr_sigma_m", 1.0);
    declare_parameter("noise.cp_sigma_m", 0.01);
    declare_parameter("noise.elevation_weighting", true);

    declare_parameter("ambiguity.prior_sigma_cycles", 100.0);
    declare_parameter("ambiguity.ref_prior_sigma_cycles", 1.0e-3);

    declare_parameter("robust.enable", true);
    declare_parameter("robust.huber_k", 1.345);

    declare_parameter("ambiguity_resolution.enable", true);
    declare_parameter("ambiguity_resolution.ratio_threshold", 3.0);
    declare_parameter("ambiguity_resolution.el_mask_deg", 15.0);
    declare_parameter("ambiguity_resolution.min_fix", 5);
    declare_parameter("ambiguity_resolution.fde_enable", false);
    declare_parameter("ambiguity_resolution.fde_threshold_m", 0.05);
    declare_parameter("ambiguity_resolution.fde_max_exclude", 2);

    // ENU output origin: fixed_origin (configured) takes priority over the base
    // station; mirrors gnss_fgo's publishSolution.
    declare_parameter("fixed_origin.postype", std::string("llh"));
    declare_parameter("fixed_origin.pos", std::vector<double>{0.0, 0.0, 0.0});

    // GNSS Doppler velocity factor: an absolute-velocity observation on V(k)
    // (mean = ep.rover_vel). Low-speed Doppler degradation is handled by the
    // robust (Huber) kernel; vel_sigma_mps is a floor on the Doppler sigma.
    declare_parameter("motion.vel_sigma_mps", 0.1);

    // IMU noise densities (defaults follow gnss_imu_kalman_filter.yaml).
    declare_parameter("imu.sigma_acc", 0.3);        // [m/s^2/sqrt(Hz)]
    declare_parameter("imu.sigma_gyr", 0.01);       // [rad/s/sqrt(Hz)]
    declare_parameter("imu.sigma_acc_bias", 1.0e-4);  // [m/s^2/sqrt(s)]
    declare_parameter("imu.sigma_gyr_bias", 1.0e-5);  // [rad/s/sqrt(s)]

    declare_parameter("init.imu_duration", 1.0);    // [s] static init window
    declare_parameter("init.pos_std", 10.0);        // [m]
    declare_parameter("init.vel_std", 0.3);         // [m/s] stationary start
    declare_parameter("init.att_std_rp", 0.1);      // [rad] roll/pitch
    declare_parameter("init.att_std_yaw", 3.14159); // [rad] yaw unknown
    declare_parameter("init.acc_bias_std", 0.1);    // [m/s^2]
    declare_parameter("init.gyr_bias_std", 0.01);   // [rad/s]

    declare_parameter("isam2.relinearize_threshold", 0.01);
    declare_parameter("isam2.relinearize_skip", 1);

    ar_enable_ = get_parameter("ambiguity_resolution.enable").as_bool();
    ar_ratio_threshold_ =
        get_parameter("ambiguity_resolution.ratio_threshold").as_double();
    ar_el_mask_ = get_parameter("ambiguity_resolution.el_mask_deg").as_double() * D2R;
    ar_min_fix_ =
        static_cast<int>(get_parameter("ambiguity_resolution.min_fix").as_int());
    ar_fde_enable_ = get_parameter("ambiguity_resolution.fde_enable").as_bool();
    ar_fde_threshold_m_ =
        get_parameter("ambiguity_resolution.fde_threshold_m").as_double();
    ar_fde_max_exclude_ = static_cast<int>(
        get_parameter("ambiguity_resolution.fde_max_exclude").as_int());
    motion_vel_sigma_ = get_parameter("motion.vel_sigma_mps").as_double();
    init_imu_duration_ = get_parameter("init.imu_duration").as_double();
    cacheFixedOriginParam();
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

  gnss_utils::GnssPreprocessor::Config makePreprocessorConfig() {
    gnss_utils::GnssPreprocessor::Config cfg;
    cfg.el_mask_rad = get_parameter("masks.elevation_deg").as_double() * D2R;
    cfg.snr_mask_dbhz = get_parameter("masks.snr_dbhz").as_double();
    // Elevation-dependent SNR mask (RTKLIB snrmask_t / testsnr). When enabled it
    // takes precedence over the flat snr_mask_dbhz; same model as gnss_fgo.
    cfg.snr_mask = snrmask_t{};
    if (get_parameter("masks.snrmask.enable").as_bool()) {
      cfg.snr_mask.ena[0] = 1;  // rover
      cfg.snr_mask.ena[1] = 1;  // base (DD uses the base obs too)
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
    return cfg;
  }

  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> makePimParams() {
    // ENU nav frame: gravity along -Z handled by MakeSharedU (z-up).
    auto p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(kGravity);
    const double sa = get_parameter("imu.sigma_acc").as_double();
    const double sg = get_parameter("imu.sigma_gyr").as_double();
    const double sab = get_parameter("imu.sigma_acc_bias").as_double();
    const double sgb = get_parameter("imu.sigma_gyr_bias").as_double();
    p->accelerometerCovariance = gtsam::I_3x3 * sa * sa;
    p->gyroscopeCovariance = gtsam::I_3x3 * sg * sg;
    p->biasAccCovariance = gtsam::I_3x3 * sab * sab;
    p->biasOmegaCovariance = gtsam::I_3x3 * sgb * sgb;
    p->integrationCovariance = gtsam::I_3x3 * 1e-8;
    p->biasAccOmegaInt = gtsam::I_6x6 * 1e-5;
    return p;
  }

  void onNav(const grs::GnssEphemerides::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    preprocessor_->pushEphemerides(*msg);
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    ImuSample s;
    s.t = rclcpp::Time(msg->header.stamp).seconds();
    s.acc = Eigen::Vector3d(msg->linear_acceleration.x,
                            msg->linear_acceleration.y,
                            msg->linear_acceleration.z);
    s.gyr = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y,
                            msg->angular_velocity.z);
    imu_buffer_.push_back(s);
    // Bounded buffer: keep ~10 minutes at 200 Hz.
    if (imu_buffer_.size() > 120000) imu_buffer_.pop_front();
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
        last_antenna_ecef_valid_ ? &last_antenna_ecef_ : nullptr);
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
    const double t_ros = rclcpp::Time(ep.stamp).seconds();

    if (!initialized_) {
      tryInitialize(ep, t_ros);
      return;
    }

    // Integrate the IMU samples in (t_{k-1}, t_k].
    gtsam::PreintegratedCombinedMeasurements pim(pim_params_, prev_bias_);
    double t_prev = prev_t_ros_;
    while (!imu_buffer_.empty() && imu_buffer_.front().t <= t_ros) {
      const ImuSample s = imu_buffer_.front();
      imu_buffer_.pop_front();
      if (s.t <= prev_t_ros_) continue;
      const double dt = std::clamp(s.t - t_prev, 1e-4, 0.5);
      pim.integrateMeasurement(s.acc, s.gyr, dt);
      t_prev = s.t;
      last_imu_ = s;
      last_imu_valid_ = true;
    }
    // Integrate the residual (t_last, t_k] with a zero-order hold of the last
    // IMU sample so the preintegration interval exactly covers (t_{k-1}, t_k].
    // Otherwise the sub-sample motion between the last IMU stamp and the GNSS
    // epoch is dropped and the chain silently loses time every epoch.
    if (pim.deltaTij() > 0.0 && last_imu_valid_ && t_prev < t_ros) {
      const double dt = std::clamp(t_ros - t_prev, 0.0, 0.5);
      if (dt > 1e-6) pim.integrateMeasurement(last_imu_.acc, last_imu_.gyr, dt);
    }

    const gtsam::Key xk = X(epoch_index_);
    const gtsam::Key vk = V(epoch_index_);
    const gtsam::Key bk = B(epoch_index_);

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;

    if (pim.deltaTij() > 0.0) {
      graph.add(gtsam::CombinedImuFactor(X(epoch_index_ - 1), V(epoch_index_ - 1),
                                         xk, vk, B(epoch_index_ - 1), bk, pim));
    } else {
      // IMU gap: keep the chain connected with loose constant-state factors.
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "No IMU samples in (%.3f, %.3f] - falling back to constant-state link.",
          prev_t_ros_, t_ros);
      const double dt = std::max(t_ros - prev_t_ros_, 0.1);
      graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
          X(epoch_index_ - 1), xk, gtsam::Pose3(),
          gtsam::noiseModel::Isotropic::Sigma(6, 10.0 * dt)));
      graph.add(gtsam::BetweenFactor<gtsam::Vector3>(
          V(epoch_index_ - 1), vk, gtsam::Vector3::Zero(),
          gtsam::noiseModel::Isotropic::Sigma(3, 1.0 * dt)));
      graph.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
          B(epoch_index_ - 1), bk, gtsam::imuBias::ConstantBias(),
          gtsam::noiseModel::Isotropic::Sigma(6, 1e-3)));
    }

    const gtsam::NavState predicted =
        pim.deltaTij() > 0.0 ? pim.predict(prev_state_, prev_bias_) : prev_state_;
    values.insert(xk, predicted.pose());
    values.insert(vk, predicted.velocity());
    values.insert(bk, prev_bias_);

    const auto pairs = gnss_fgo::addDdFactorsArm(
        ep, xk, next_amb_id_, lever_arm_, ecef_T_nav_, adapter_cfg_, graph,
        values);

    // GNSS Doppler velocity factor on V(k): an absolute-velocity observation
    // (the canonical pseudorange + carrier + Doppler tightly-coupled set).
    // Low-speed Doppler degradation is handled by the robust (Huber) kernel.
    if (ep.rover_vel_valid) {
      const Eigen::Matrix3d R_e_n = ecef_T_nav_.rotation().matrix();
      const gtsam::Vector3 vel_enu = R_e_n.transpose() * ep.rover_vel_ecef;
      const double sigma =
          std::max(std::sqrt(std::max(ep.rover_vel_var, 0.0)), motion_vel_sigma_);
      gtsam::SharedNoiseModel vnoise = gtsam::noiseModel::Isotropic::Sigma(3, sigma);
      if (adapter_cfg_.robust) {
        vnoise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(adapter_cfg_.huber_k),
            vnoise);
      }
      graph.add(gtsam::PriorFactor<gtsam::Vector3>(vk, vel_enu, vnoise));
    }

    gtsam::Values estimate;
    try {
      isam_->update(graph, values);
      estimate = isam_->calculateEstimate();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "ISAM2 update failed (%s) - reinitializing.",
                   e.what());
      initialized_ = false;
      resetGraph();
      return;
    }

    const gtsam::Pose3 pose = estimate.at<gtsam::Pose3>(xk);
    const gtsam::Vector3 vel = estimate.at<gtsam::Vector3>(vk);
    const Eigen::MatrixXd pose_cov = isam_->marginalCovariance(xk);  // 6x6 tangent
    const Eigen::Matrix3d vel_cov = isam_->marginalCovariance(vk);

    // Float antenna position (ECEF) and its 3x3 covariance - the inputs the
    // analytical DD ambiguity resolution expects. The Pose3 tangent translation
    // block is a body-frame perturbation: rotate it to ENU (body rotation) then
    // to ECEF (ecef_T_nav rotation).
    const gtsam::Point3 ant_nav = pose.transformFrom(lever_arm_);
    const gtsam::Point3 ant_ecef = ecef_T_nav_.transformFrom(ant_nav);
    Eigen::Vector3d pub_pos(ant_ecef.x(), ant_ecef.y(), ant_ecef.z());
    const Eigen::Matrix3d R_body = pose.rotation().matrix();
    const Eigen::Matrix3d cov_enu =
        R_body * pose_cov.bottomRightCorner<3, 3>() * R_body.transpose();
    const Eigen::Matrix3d R_e_n = ecef_T_nav_.rotation().matrix();
    Eigen::Matrix3d pub_cov = R_e_n * cov_enu * R_e_n.transpose();

    uint8_t status = grs::GnssSolution::STATUS_FLOAT;
    double ratio = 0.0;
    if (ar_enable_ && static_cast<int>(pairs.size()) >= kMinDdForAr) {
      // Same analytical AR as gnss_fgo: the correctly-correlated DD covariance
      // (RTKLIB ddcov) + LAMBDA, anchored by the float antenna position. The
      // FIX is published only, never fed back, so a wrong fix cannot corrupt
      // the graph (which stays float, like gnss_fgo).
      const auto ar = gnss_fgo::resolveAmbiguitiesDd(
          ep, pub_pos, pub_cov, adapter_cfg_, ar_ratio_threshold_, ar_el_mask_,
          ar_min_fix_, ar_fde_enable_, ar_fde_threshold_m_, ar_fde_max_exclude_);
      ratio = ar.ratio;
      if (ar.fixed) {
        pub_pos = ar.fixed_pos;
        pub_cov = ar.state_cov;
        status = grs::GnssSolution::STATUS_FIX;
      }
    }

    publishSolution(ep, pub_pos, pub_cov, vel, vel_cov, status, ratio);
    publishOdometry(ep, pose, pose_cov, vel, vel_cov);

    prev_state_ = gtsam::NavState(pose, vel);
    prev_bias_ = estimate.at<gtsam::imuBias::ConstantBias>(bk);
    prev_t_ros_ = t_ros;
    last_antenna_ecef_ = pub_pos;
    last_antenna_ecef_valid_ = true;
    ++epoch_index_;

    double llh[3];
    const double e[3] = {last_antenna_ecef_.x(), last_antenna_ecef_.y(),
                         last_antenna_ecef_.z()};
    ecef2pos(e, llh);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), kPositionLogThrottleMs,
        "FGO %s | LLH: %.8f %.8f %.3f | Ratio: %.1f | DD: %zu | dT_imu: %.2f",
        status == grs::GnssSolution::STATUS_FIX ? "FIX" : "FLOAT",
        llh[0] * R2D, llh[1] * R2D, llh[2], ratio, ep.dd.size(), pim.deltaTij());
  }

  // Static initialization: average accelerometer -> roll/pitch, average gyro
  // -> gyro bias, first GNSS a-priori -> ENU anchor and initial position.
  void tryInitialize(const gnss_utils::PreprocessedEpoch& ep, double t_ros) {
    std::vector<const ImuSample*> window;
    for (const auto& s : imu_buffer_) {
      if (s.t >= t_ros - init_imu_duration_ && s.t <= t_ros) window.push_back(&s);
    }
    if (window.size() < 10) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "Waiting for IMU samples to initialize (%zu in window).", window.size());
      return;
    }

    Eigen::Vector3d acc_mean = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyr_mean = Eigen::Vector3d::Zero();
    for (const auto* s : window) {
      acc_mean += s->acc;
      gyr_mean += s->gyr;
    }
    acc_mean /= static_cast<double>(window.size());
    gyr_mean /= static_cast<double>(window.size());

    // ENU anchor.
    Eigen::Vector3d anchor = ep.rover_ecef_apriori;
    if (get_parameter("local_origin.mode").as_string() == "manual") {
      const auto pos = get_parameter("local_origin.pos").as_double_array();
      if (pos.size() == 3 && norm(pos.data(), 3) > 0.0) {
        if (get_parameter("local_origin.postype").as_string() == "llh") {
          const double llh[3] = {pos[0] * D2R, pos[1] * D2R, pos[2]};
          double e[3];
          pos2ecef(llh, e);
          anchor = Eigen::Vector3d(e[0], e[1], e[2]);
        } else {
          anchor = Eigen::Vector3d(pos[0], pos[1], pos[2]);
        }
      }
    }
    double anchor_llh[3];
    const double anchor_e[3] = {anchor.x(), anchor.y(), anchor.z()};
    ecef2pos(anchor_e, anchor_llh);
    double E[9];  // rows e,n,u: enu = E * d_ecef
    xyz2enu(anchor_llh, E);
    Eigen::Matrix3d R_enu_ecef;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) R_enu_ecef(r, c) = E[r + c * 3];
    }
    ecef_T_nav_ = gtsam::Pose3(gtsam::Rot3(R_enu_ecef.transpose()),
                               gtsam::Point3(anchor));

    // Roll/pitch: rotate the measured specific-force direction onto +Z (up).
    const Eigen::Vector3d a_hat = acc_mean.normalized();
    const Eigen::Vector3d z(0.0, 0.0, 1.0);
    const Eigen::Vector3d axis = a_hat.cross(z);
    const double angle = std::atan2(axis.norm(), a_hat.dot(z));
    const gtsam::Rot3 R0 =
        axis.norm() < 1e-9 ? gtsam::Rot3()
                           : gtsam::Rot3::AxisAngle(gtsam::Unit3(axis), angle);

    // Antenna sits at the anchor; body origin = antenna - R0 * lever_arm.
    const gtsam::Point3 body0 = gtsam::Point3(0, 0, 0) - R0.rotate(lever_arm_);
    const gtsam::Pose3 pose0(R0, body0);
    const gtsam::imuBias::ConstantBias bias0(Eigen::Vector3d::Zero(), gyr_mean);

    pim_params_ = makePimParams();

    const double pos_std = get_parameter("init.pos_std").as_double();
    const double rp_std = get_parameter("init.att_std_rp").as_double();
    const double yaw_std = get_parameter("init.att_std_yaw").as_double();
    const double vel_std = get_parameter("init.vel_std").as_double();
    const double ab_std = get_parameter("init.acc_bias_std").as_double();
    const double gb_std = get_parameter("init.gyr_bias_std").as_double();

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    values.insert(X(0), pose0);
    values.insert(V(0), gtsam::Vector3(gtsam::Vector3::Zero()));
    values.insert(B(0), bias0);
    // Pose3 tangent ordering: [rot(3), trans(3)]; yaw = rotation about up.
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(
        X(0), pose0,
        gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << rp_std, rp_std, yaw_std,
             pos_std, pos_std, pos_std).finished())));
    graph.add(gtsam::PriorFactor<gtsam::Vector3>(
        V(0), gtsam::Vector3::Zero(),
        gtsam::noiseModel::Isotropic::Sigma(3, vel_std)));
    graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
        B(0), bias0,
        gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << ab_std, ab_std, ab_std,
             gb_std, gb_std, gb_std).finished())));

    const auto pairs = gnss_fgo::addDdFactorsArm(
        ep, X(0), next_amb_id_, lever_arm_, ecef_T_nav_, adapter_cfg_, graph,
        values);
    (void)pairs;

    try {
      isam_->update(graph, values);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Initialization update failed: %s", e.what());
      resetGraph();
      return;
    }

    prev_state_ = gtsam::NavState(pose0, gtsam::Vector3::Zero());
    prev_bias_ = bias0;
    prev_t_ros_ = t_ros;
    last_antenna_ecef_ = anchor;
    last_antenna_ecef_valid_ = true;
    epoch_index_ = 1;
    initialized_ = true;

    // Drop IMU samples up to the anchor epoch, remembering the last one so the
    // first inter-epoch integration can hold it up to the next GNSS stamp.
    while (!imu_buffer_.empty() && imu_buffer_.front().t <= t_ros) {
      last_imu_ = imu_buffer_.front();
      last_imu_valid_ = true;
      imu_buffer_.pop_front();
    }

    RCLCPP_INFO(get_logger(),
        "Initialized: anchor LLH %.8f %.8f %.3f | roll/pitch from %zu IMU samples "
        "| yaw unobserved (prior sigma %.2f rad) - converges once the platform moves.",
        anchor_llh[0] * R2D, anchor_llh[1] * R2D, anchor_llh[2], window.size(),
        yaw_std);
  }

  void resetGraph() {
    gtsam::ISAM2Params isam_params;
    isam_params.relinearizeThreshold =
        get_parameter("isam2.relinearize_threshold").as_double();
    isam_params.relinearizeSkip = get_parameter("isam2.relinearize_skip").as_int();
    isam_ = std::make_unique<gtsam::ISAM2>(isam_params);
    epoch_index_ = 0;
    next_amb_id_ = 0;
    last_antenna_ecef_valid_ = false;
    initialized_ = false;
  }

  void publishSolution(const gnss_utils::PreprocessedEpoch& ep,
                       const Eigen::Vector3d& pos_ecef,
                       const Eigen::Matrix3d& pos_cov_ecef,
                       const gtsam::Vector3& vel_enu,
                       const Eigen::Matrix3d& vel_cov_enu, uint8_t status,
                       double ratio) {
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

    // Antenna position is supplied directly in ECEF (FLOAT: pose + lever arm;
    // FIX: the analytical AR conditioned position).
    const double ecef[3] = {pos_ecef.x(), pos_ecef.y(), pos_ecef.z()};
    double llh[3];
    ecef2pos(ecef, llh);
    msg->latitude = llh[0] * R2D;
    msg->longitude = llh[1] * R2D;
    msg->altitude = llh[2];
    msg->pos_ecef.x = pos_ecef.x();
    msg->pos_ecef.y = pos_ecef.y();
    msg->pos_ecef.z = pos_ecef.z();
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) msg->pos_cov_ecef[3 * r + c] = pos_cov_ecef(r, c);
    }

    // ENU origin priority: fixed_origin (configured) > local nav anchor > base.
    double origin[3] = {0.0, 0.0, 0.0};
    if (fixed_origin_valid_) {
      std::copy_n(fixed_origin_ecef_, 3, origin);
    } else if (ecef_T_nav_.translation().norm() > 0.0) {
      origin[0] = ecef_T_nav_.translation().x();
      origin[1] = ecef_T_nav_.translation().y();
      origin[2] = ecef_T_nav_.translation().z();
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
      for (int c = 0; c < 3; ++c) q_ecef[3 * r + c] = pos_cov_ecef(r, c);
    }
    double q_enu[9];
    gnss_utils::rotateCovariance(q_ecef, llh[0], llh[1], q_enu);
    for (int i = 0; i < 9; ++i) msg->pos_enu_cov[i] = q_enu[i];

    msg->vel_enu.x = vel_enu.x();
    msg->vel_enu.y = vel_enu.y();
    msg->vel_enu.z = vel_enu.z();
    const Eigen::Matrix3d R_e_n = ecef_T_nav_.rotation().matrix();
    const Eigen::Vector3d vel_ecef = R_e_n * Eigen::Vector3d(vel_enu);
    msg->vel_ecef.x = vel_ecef.x();
    msg->vel_ecef.y = vel_ecef.y();
    msg->vel_ecef.z = vel_ecef.z();
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) msg->vel_enu_cov[3 * r + c] = vel_cov_enu(r, c);
    }
    double qv_enu[9];
    for (int i = 0; i < 9; ++i) qv_enu[i] = msg->vel_enu_cov[i];
    double qv_ecef[9];
    gnss_utils::rotateCovarianceEnuToEcef(qv_enu, llh[0], llh[1], qv_ecef);
    for (int i = 0; i < 9; ++i) msg->vel_cov_ecef[i] = qv_ecef[i];

    sol_pub_->publish(std::move(msg));
  }

  // Full navigation state as nav_msgs/Odometry (body pose + attitude + velocity),
  // same convention as the GNSS/IMU EKF example: frame "odom" (local ENU nav),
  // child "imu_link" (body); twist is expressed in the body frame (REP-105).
  void publishOdometry(const gnss_utils::PreprocessedEpoch& ep,
                       const gtsam::Pose3& pose, const Eigen::MatrixXd& pose_cov,
                       const gtsam::Vector3& vel_enu,
                       const Eigen::Matrix3d& vel_cov_enu) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = ep.stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "imu_link";

    const gtsam::Point3 p = pose.translation();
    odom.pose.pose.position.x = p.x();
    odom.pose.pose.position.y = p.y();
    odom.pose.pose.position.z = p.z();
    const gtsam::Quaternion q = pose.rotation().toQuaternion();
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();

    // Reorder GTSAM Pose3 tangent covariance [rot(0-2), trans(3-5)] into the
    // Odometry [pos(0-2), att(3-5)] layout. The translation tangent is a
    // body-frame perturbation, so rotate its blocks into the nav (ENU) frame.
    const Eigen::Matrix3d R = pose.rotation().matrix();
    const Eigen::Matrix3d Prr = pose_cov.topLeftCorner<3, 3>();      // rot-rot
    const Eigen::Matrix3d Ptt = pose_cov.bottomRightCorner<3, 3>();  // trans-trans
    const Eigen::Matrix3d Ptr = pose_cov.bottomLeftCorner<3, 3>();   // trans-rot
    const Eigen::Matrix3d Cpp = R * Ptt * R.transpose();  // pos-pos (nav)
    const Eigen::Matrix3d Cpa = R * Ptr;                  // pos-att
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        odom.pose.covariance[r * 6 + c] = Cpp(r, c);              // pos-pos
        odom.pose.covariance[(r + 3) * 6 + (c + 3)] = Prr(r, c);  // att-att
        odom.pose.covariance[r * 6 + (c + 3)] = Cpa(r, c);        // pos-att
        odom.pose.covariance[(r + 3) * 6 + c] = Cpa(c, r);        // att-pos
      }
    }

    // Twist in the body/child frame (REP-105).
    const Eigen::Vector3d vel_body = R.transpose() * Eigen::Vector3d(vel_enu);
    odom.twist.twist.linear.x = vel_body.x();
    odom.twist.twist.linear.y = vel_body.y();
    odom.twist.twist.linear.z = vel_body.z();
    const Eigen::Matrix3d vel_cov_body = R.transpose() * vel_cov_enu * R;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        odom.twist.covariance[r * 6 + c] = vel_cov_body(r, c);
      }
    }

    odom_pub_->publish(odom);
  }

  std::mutex mtx_;
  std::unique_ptr<gnss_utils::GnssPreprocessor> preprocessor_;
  std::unique_ptr<gtsam::ISAM2> isam_;
  gnss_fgo::AdapterConfig adapter_cfg_;
  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> pim_params_;

  std::deque<ImuSample> imu_buffer_;
  ImuSample last_imu_{};        // most recent integrated IMU sample (for ZOH)
  bool last_imu_valid_{false};

  bool initialized_{false};
  int epoch_index_{0};
  std::uint64_t next_amb_id_{0};  // globally-unique per-epoch ambiguity keys
  double prev_t_ros_{0.0};
  gtsam::NavState prev_state_;
  gtsam::imuBias::ConstantBias prev_bias_;
  gtsam::Pose3 ecef_T_nav_;
  gtsam::Point3 lever_arm_{0.0, 0.0, 0.0};

  Eigen::Vector3d last_antenna_ecef_{Eigen::Vector3d::Zero()};
  bool last_antenna_ecef_valid_{false};

  bool ar_enable_{true};
  double ar_ratio_threshold_{3.0};
  double ar_el_mask_{15.0 * D2R};
  int ar_min_fix_{5};
  bool ar_fde_enable_{false};
  double ar_fde_threshold_m_{0.05};
  int ar_fde_max_exclude_{2};
  double motion_vel_sigma_{0.1};
  double init_imu_duration_{1.0};

  double fixed_origin_ecef_[3]{0.0, 0.0, 0.0};
  bool fixed_origin_valid_{false};

  rclcpp::Subscription<grs::GnssObservations>::SharedPtr rover_obs_sub_;
  rclcpp::Subscription<grs::GnssObservations>::SharedPtr base_obs_sub_;
  rclcpp::Subscription<grs::GnssEphemerides>::SharedPtr nav_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<grs::GnssSolution>::SharedPtr sol_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssImuFgoNode>());
  rclcpp::shutdown();
  return 0;
}
