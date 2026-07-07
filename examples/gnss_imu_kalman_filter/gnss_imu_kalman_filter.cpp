// SPDX-License-Identifier: MIT
// GNSS/IMU/WheelSpeed Error-State EKF Node
#include "gnss_ros_standardization/gnss_imu_kalman_filter.hpp"
#include "gnss_ros_standardization/gnss_utils.hpp"
#include <iomanip>
#include <sstream>
#include <climits>
#include <unistd.h>

namespace grs = gnss_ros_standardization::msg;

// ===========================================================================
// Error-state attitude convention (read before touching any Jacobian)
// ---------------------------------------------------------------------------
// This filter uses a LOCAL (body-frame) multiplicative attitude error δθ,
// injected by RIGHT-multiplication onto the estimated body→nav quaternion:
//
//     q_true = q_est ⊗ δq(δθ),     C_true = C_bn · exp([δθ×])
//
// where C_bn = R(q_est) maps body → nav. Every Jacobian below is written for
// this single convention; the resulting error-state dynamics are:
//
//   δṗ   =  δv
//   δv̇   = −C_bn·[(a_m − b_a)×]·δθ − C_bn·δb_a                 (specific force)
//   δθ̇   = −[(ω_m − b_g)×]·δθ − δb_g                          (body-frame rate)
//   δḃ_a =  w_a,    δḃ_g = w_g                                 (random walk)
//
// Measurement Jacobians follow the same rule, e.g. for a nav-frame quantity
// y = C_bn·v:  ∂y/∂δθ = −C_bn·[v×]; for a body-frame quantity y = C_bnᵀ·v_nav:
// ∂y/∂δθ = [v_body×] = C_bnᵀ·[v_nav×]·C_bn.
//
// Do NOT mix in the global/nav-frame form −[(C_bn·a)×] = −C_bn·[a×]·C_bnᵀ:
// it differs by a rotation and silently destabilizes the attitude solution.
//
// Reference: J. Solà, "Quaternion kinematics for the error-state Kalman
// filter", 2017 (arXiv:1711.02508), local-error / right-quaternion form.
// ===========================================================================

namespace gnss_imu_kalman_filter {

// Quaternion helpers
Eigen::Quaterniond GnssImuKalmanFilter::getQuaternion() const {
  return Eigen::Quaterniond(x_(IDX_QUAT), x_(IDX_QUAT+1),
                            x_(IDX_QUAT+2), x_(IDX_QUAT+3));
}

void GnssImuKalmanFilter::setQuaternion(const Eigen::Quaterniond& q) {
  x_(IDX_QUAT)   = q.w();
  x_(IDX_QUAT+1) = q.x();
  x_(IDX_QUAT+2) = q.y();
  x_(IDX_QUAT+3) = q.z();
}

Eigen::Vector3d GnssImuKalmanFilter::quaternionToEuler(const Eigen::Quaterniond& q) {
  // Returns (roll, pitch, yaw) in radians
  auto m = q.normalized().toRotationMatrix();
  double roll  = std::atan2(m(2,1), m(2,2));
  double pitch = -std::asin(std::clamp(m(2,0), -1.0, 1.0));
  double yaw   = std::atan2(m(1,0), m(0,0));
  return {roll, pitch, yaw};
}

Eigen::Quaterniond GnssImuKalmanFilter::eulerToQuaternion(double roll, double pitch, double yaw) {
  return Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX());
}

Eigen::Matrix3d GnssImuKalmanFilter::getRotationMatrix() const {
  return getQuaternion().normalized().toRotationMatrix();
}

Eigen::Vector3d GnssImuKalmanFilter::applyImuAxisRotation(const Eigen::Vector3d& raw) const {
  return R_imu_body_ * raw;
}

Eigen::Vector3d GnssImuKalmanFilter::computeLeverArmCorrection() const {
  Eigen::Matrix3d C_bn = getRotationMatrix();
  return C_bn * config_.lever_arm;
}

// Coordinate helpers (navigation frame = local ENU; up is +Z)
namespace {
constexpr double kGravity = 9.80665;                       // [m/s^2]
const Eigen::Vector3d kGravityEnu(0.0, 0.0, -kGravity);    // nav-frame gravity

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m <<     0, -v(2),  v(1),
        v(2),     0, -v(0),
       -v(1),  v(0),     0;
  return m;
}
}  // namespace

Eigen::Vector3d GnssImuKalmanFilter::ecefToEnu(const Eigen::Vector3d& ecef) const {
  double pos_ref[3] = {origin_llh_(0), origin_llh_(1), origin_llh_(2)};
  double dr[3] = {ecef(0) - origin_ecef_(0),
                  ecef(1) - origin_ecef_(1),
                  ecef(2) - origin_ecef_(2)};
  double enu[3];
  ecef2enu(pos_ref, dr, enu);
  return Eigen::Vector3d(enu[0], enu[1], enu[2]);
}

Eigen::Vector3d GnssImuKalmanFilter::enuToEcef(const Eigen::Vector3d& pos) const {
  double pos_ref[3] = {origin_llh_(0), origin_llh_(1), origin_llh_(2)};
  double enu[3] = {pos(0), pos(1), pos(2)};
  double dr[3];
  enu2ecef(pos_ref, enu, dr);
  return Eigen::Vector3d(origin_ecef_(0) + dr[0],
                         origin_ecef_(1) + dr[1],
                         origin_ecef_(2) + dr[2]);
}

Eigen::Vector3d GnssImuKalmanFilter::enuToLlh(const Eigen::Vector3d& pos) const {
  Eigen::Vector3d ecef = enuToEcef(pos);
  double e[3] = {ecef(0), ecef(1), ecef(2)};
  double llh[3];
  ecef2pos(e, llh);
  // Return lat[deg], lon[deg], alt[m]
  return Eigen::Vector3d(llh[0] * 180.0 / M_PI, llh[1] * 180.0 / M_PI, llh[2]);
}

OriginVelocity GnssImuKalmanFilter::computeOriginFrameVelocity(const GnssSnapshot& snap) const {
  OriginVelocity out;
  double pos_ref[3] = {origin_llh_(0), origin_llh_(1), origin_llh_(2)};

  if (snap.vel_ecef.allFinite()) {
    double v_ecef[3] = {snap.vel_ecef(0), snap.vel_ecef(1), snap.vel_ecef(2)};
    double v_enu[3];
    ecef2enu(pos_ref, v_ecef, v_enu);
    out.vel_enu = Eigen::Vector3d(v_enu[0], v_enu[1], v_enu[2]);

    double cov_enu[9];
    gnss_utils::rotateCovariance(snap.vel_cov_ecef.data(), origin_llh_(0), origin_llh_(1), cov_enu);
    out.cov_enu = Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>>(cov_enu);
    out.valid = true;
  } else if (snap.vel_enu.allFinite() && std::isfinite(snap.lat) && std::isfinite(snap.lon)) {
    // Fallback: rotate the current-position ENU velocity through ECEF.
    double pos_cur[3] = {snap.lat * D2R, snap.lon * D2R, snap.alt};
    double v_enu_cur[3] = {snap.vel_enu(0), snap.vel_enu(1), snap.vel_enu(2)};
    double v_ecef[3];
    enu2ecef(pos_cur, v_enu_cur, v_ecef);
    double v_enu[3];
    ecef2enu(pos_ref, v_ecef, v_enu);
    out.vel_enu = Eigen::Vector3d(v_enu[0], v_enu[1], v_enu[2]);

    double cov_ecef[9];
    gnss_utils::rotateCovarianceEnuToEcef(snap.vel_cov_enu.data(), pos_cur[0], pos_cur[1], cov_ecef);
    double cov_enu[9];
    gnss_utils::rotateCovariance(cov_ecef, origin_llh_(0), origin_llh_(1), cov_enu);
    out.cov_enu = Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>>(cov_enu);
    out.valid = true;
  }
  return out;  // both unusable: out.valid stays false, caller skips the update
}

double GnssImuKalmanFilter::dopplerHeadingFromOriginVelocity(const OriginVelocity& vel) const {
  if (!vel.valid) return std::numeric_limits<double>::quiet_NaN();
  const double horiz = std::hypot(vel.vel_enu(0), vel.vel_enu(1));
  if (horiz <= config_.gnss_heading_speed_threshold) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::atan2(vel.vel_enu(1), vel.vel_enu(0));  // ENU yaw: from East, CCW
}

// Constructor / Destructor
GnssImuKalmanFilter::GnssImuKalmanFilter(const rclcpp::NodeOptions& options)
    : Node("gnss_imu_kalman_filter", options) {
  x_.setZero();
  P_.setZero();
  R_imu_body_ = Eigen::Matrix3d::Identity();

  loadParameters();

  // Set manual local origin if configured
  if (config_.local_origin_mode == "manual") {
    double lat_rad = config_.local_origin_pos[0] * M_PI / 180.0;
    double lon_rad = config_.local_origin_pos[1] * M_PI / 180.0;
    double alt = config_.local_origin_pos[2];
    origin_llh_ = Eigen::Vector3d(lat_rad, lon_rad, alt);
    double llh[3] = {lat_rad, lon_rad, alt};
    double ecef[3];
    pos2ecef(llh, ecef);
    origin_ecef_ = Eigen::Vector3d(ecef[0], ecef[1], ecef[2]);
    origin_set_ = true;
    RCLCPP_INFO(get_logger(), "Manual origin set: lat=%.8f lon=%.8f alt=%.3f",
                config_.local_origin_pos[0], config_.local_origin_pos[1], alt);
  }

  // Build IMU axis rotation matrix from RPY
  R_imu_body_ = eulerToQuaternion(config_.imu_orientation_rpy(0),
                                  config_.imu_orientation_rpy(1),
                                  config_.imu_orientation_rpy(2)).toRotationMatrix();

  // Subscribers
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    config_.topic_imu_raw, rclcpp::SensorDataQoS(),
    std::bind(&GnssImuKalmanFilter::onImu, this, std::placeholders::_1));

  gnss_sub_ = create_subscription<grs::GnssSolution>(
    config_.topic_gnss_solution, rclcpp::QoS(10),
    std::bind(&GnssImuKalmanFilter::onGnss, this, std::placeholders::_1));

  if (config_.use_wheel_speed) {
    if (config_.wheel_speed_topic_type == "twist") {
      wheel_sub_raw_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        config_.topic_wheel_speed, rclcpp::QoS(10),
        std::bind(&GnssImuKalmanFilter::onWheelSpeedPoint, this, std::placeholders::_1));
    } else {
      wheel_sub_cov_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        config_.topic_wheel_speed, rclcpp::QoS(10),
        std::bind(&GnssImuKalmanFilter::onWheelSpeedWithCov, this, std::placeholders::_1));
    }
  }

  // Publisher
  solution_pub_ = create_publisher<grs::GnssSolution>(config_.topic_solution, rclcpp::QoS(10));
  odom_pub_     = create_publisher<nav_msgs::msg::Odometry>(config_.topic_solution + "_odom", rclcpp::QoS(10));

  // CSV — resolve output directory (empty = current working directory)
  auto resolveCsvPath = [&](const std::string& filename) -> std::string {
    if (config_.csv.dir.empty()) {
      char cwd[PATH_MAX];
      if (getcwd(cwd, sizeof(cwd))) return std::string(cwd) + "/" + filename;
      return filename;
    }
    return config_.csv.dir + "/" + filename;
  };

  if (config_.csv.sensors_log_enabled) {
    std::string path = resolveCsvPath(config_.csv.sensors_log_filename);
    sensors_csv_.open(path, std::ios::out | std::ios::trunc);
    if (sensors_csv_.is_open()) {
      writeSensorsHeader();
      RCLCPP_INFO(get_logger(), "Sensors CSV: %s", path.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to open sensors CSV: %s", path.c_str());
    }
  }

  if (config_.csv.state_log_enabled) {
    std::string path = resolveCsvPath(config_.csv.state_log_filename);
    state_csv_.open(path, std::ios::out | std::ios::trunc);
    if (state_csv_.is_open()) {
      writeStateHeader();
      RCLCPP_INFO(get_logger(), "State CSV: %s", path.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to open state CSV: %s", path.c_str());
    }
  }

  RCLCPP_INFO(get_logger(), "EKF node started. gnss_mode=%s, wheel=%s, leveling=%s, zupt=%s",
    (config_.gnss_update_mode == GnssUpdateMode::FIX_ONLY ? "fix_only" :
     config_.gnss_update_mode == GnssUpdateMode::FIX_FLOAT ? "fix_float" : "all"),
    config_.use_wheel_speed ? "ON" : "OFF",
    config_.leveling_enable ? "ON" : "OFF",
    config_.zupt_enable ? "ON" : "OFF");
}

GnssImuKalmanFilter::~GnssImuKalmanFilter() {
  if (sensors_csv_.is_open()) sensors_csv_.close();
  if (state_csv_.is_open()) state_csv_.close();
}

// Parameter loading
void GnssImuKalmanFilter::loadParameters() {
  auto& c = config_;

  declare_parameter<std::string>("local_origin.mode", c.local_origin_mode);
  get_parameter("local_origin.mode", c.local_origin_mode);
  declare_parameter<std::vector<double>>("local_origin.pos", {c.local_origin_pos[0], c.local_origin_pos[1], c.local_origin_pos[2]});
  auto lo = get_parameter("local_origin.pos").as_double_array();
  if (lo.size() >= 3) { c.local_origin_pos = {lo[0], lo[1], lo[2]}; }

  declare_parameter<std::string>("topics.gnss_solution", c.topic_gnss_solution);
  declare_parameter<std::string>("topics.imu_raw", c.topic_imu_raw);
  declare_parameter<std::string>("topics.wheel_speed", c.topic_wheel_speed);
  declare_parameter<std::string>("topics.solution", c.topic_solution);
  get_parameter("topics.gnss_solution", c.topic_gnss_solution);
  get_parameter("topics.imu_raw", c.topic_imu_raw);
  get_parameter("topics.wheel_speed", c.topic_wheel_speed);
  get_parameter("topics.solution", c.topic_solution);

  declare_parameter<std::string>("csv.dir", c.csv.dir);
  get_parameter("csv.dir", c.csv.dir);
  declare_parameter<bool>("csv.sensors_log_enabled", c.csv.sensors_log_enabled);
  get_parameter("csv.sensors_log_enabled", c.csv.sensors_log_enabled);
  declare_parameter<std::string>("csv.sensors_log_filename", c.csv.sensors_log_filename);
  get_parameter("csv.sensors_log_filename", c.csv.sensors_log_filename);
  declare_parameter<bool>("csv.state_log_enabled", c.csv.state_log_enabled);
  get_parameter("csv.state_log_enabled", c.csv.state_log_enabled);
  declare_parameter<std::string>("csv.state_log_filename", c.csv.state_log_filename);
  get_parameter("csv.state_log_filename", c.csv.state_log_filename);

  auto load_vec3 = [&](const std::string& name, Eigen::Vector3d& v) {
    declare_parameter<std::vector<double>>(name, {v(0), v(1), v(2)});
    auto arr = get_parameter(name).as_double_array();
    if (arr.size() >= 3) v = Eigen::Vector3d(arr[0], arr[1], arr[2]);
  };
  load_vec3("ekf.sigma_acc",          c.sigma_acc);
  load_vec3("ekf.sigma_gyr",          c.sigma_gyr);
  load_vec3("ekf.sigma_acc_bias",     c.sigma_acc_bias);
  load_vec3("ekf.sigma_gyr_bias",     c.sigma_gyr_bias);
  load_vec3("ekf.init_pos_std",       c.init_pos_std);
  load_vec3("ekf.init_vel_std",       c.init_vel_std);
  load_vec3("ekf.init_att_std",       c.init_att_std);
  load_vec3("ekf.init_acc_bias_std",  c.init_acc_bias_std);
  load_vec3("ekf.init_gyr_bias_std",  c.init_gyr_bias_std);

  declare_parameter<std::string>("gnss_update_mode", "fix_only");
  std::string mode_str;
  get_parameter("gnss_update_mode", mode_str);
  if (mode_str == "fix_float") c.gnss_update_mode = GnssUpdateMode::FIX_FLOAT;
  else if (mode_str == "all") c.gnss_update_mode = GnssUpdateMode::ALL;
  else c.gnss_update_mode = GnssUpdateMode::FIX_ONLY;

  declare_parameter<double>("gnss_heading_speed_threshold", c.gnss_heading_speed_threshold);
  get_parameter("gnss_heading_speed_threshold", c.gnss_heading_speed_threshold);

  load_vec3("gnss.pos_sigma_default_fix",   c.gnss_pos_sigma_default_fix);
  load_vec3("gnss.pos_sigma_default_float", c.gnss_pos_sigma_default_float);
  load_vec3("gnss.pos_sigma_default_other", c.gnss_pos_sigma_default_other);
  declare_parameter<double>("gnss.cov_min_var", c.gnss_cov_min_var);
  get_parameter("gnss.cov_min_var", c.gnss_cov_min_var);
  declare_parameter<double>("gnss.vel_sigma_default", c.gnss_vel_sigma_default);
  get_parameter("gnss.vel_sigma_default", c.gnss_vel_sigma_default);

  declare_parameter<bool>("ekf.time_align_to_gnss", c.time_align_to_gnss);
  get_parameter("ekf.time_align_to_gnss", c.time_align_to_gnss);
  declare_parameter<double>("ekf.imu_buffer_duration", c.imu_buffer_duration);
  get_parameter("ekf.imu_buffer_duration", c.imu_buffer_duration);

  declare_parameter<bool>("use_wheel_speed", c.use_wheel_speed);
  declare_parameter<std::string>("wheel_speed_topic_type", c.wheel_speed_topic_type);
  declare_parameter<std::string>("wheel_speed_mode", c.wheel_speed_mode);
  declare_parameter<double>("wheel_speed_sigma", c.wheel_speed_sigma);
  get_parameter("use_wheel_speed", c.use_wheel_speed);
  get_parameter("wheel_speed_topic_type", c.wheel_speed_topic_type);
  get_parameter("wheel_speed_mode", c.wheel_speed_mode);
  get_parameter("wheel_speed_sigma", c.wheel_speed_sigma);

  declare_parameter<bool>("wheel_nhc_enable", c.wheel_nhc_enable);
  declare_parameter<double>("wheel_nhc_sigma_lateral", c.wheel_nhc_sigma_lateral);
  declare_parameter<double>("wheel_nhc_sigma_vertical", c.wheel_nhc_sigma_vertical);
  get_parameter("wheel_nhc_enable", c.wheel_nhc_enable);
  get_parameter("wheel_nhc_sigma_lateral", c.wheel_nhc_sigma_lateral);
  get_parameter("wheel_nhc_sigma_vertical", c.wheel_nhc_sigma_vertical);

  declare_parameter<bool>("leveling.enable", c.leveling_enable);
  declare_parameter<double>("leveling.window", c.leveling_window);
  get_parameter("leveling.window", c.leveling_window);
  declare_parameter<double>("leveling.sigma_min_deg", c.leveling_sigma_min_deg);
  declare_parameter<double>("leveling.acc_gain", c.leveling_acc_gain);
  declare_parameter<double>("leveling.gyr_gain", c.leveling_gyr_gain);
  declare_parameter<double>("leveling.max_acc", c.leveling_max_acc);
  get_parameter("leveling.enable", c.leveling_enable);
  get_parameter("leveling.sigma_min_deg", c.leveling_sigma_min_deg);
  get_parameter("leveling.acc_gain", c.leveling_acc_gain);
  get_parameter("leveling.gyr_gain", c.leveling_gyr_gain);
  get_parameter("leveling.max_acc", c.leveling_max_acc);

  declare_parameter<bool>("zupt.enable", c.zupt_enable);
  declare_parameter<double>("zupt.window", c.zupt_window);
  declare_parameter<double>("zupt.acc_std_thresh", c.zupt_acc_std_thresh);
  declare_parameter<double>("zupt.gyr_std_thresh", c.zupt_gyr_std_thresh);
  declare_parameter<double>("zupt.sigma", c.zupt_sigma);
  declare_parameter<double>("zupt.min_interval", c.zupt_min_interval);
  declare_parameter<double>("zupt.speed_thresh", c.zupt_speed_thresh);
  declare_parameter<double>("zupt.speed_timeout", c.zupt_speed_timeout);
  get_parameter("zupt.enable", c.zupt_enable);
  get_parameter("zupt.window", c.zupt_window);
  get_parameter("zupt.acc_std_thresh", c.zupt_acc_std_thresh);
  get_parameter("zupt.gyr_std_thresh", c.zupt_gyr_std_thresh);
  get_parameter("zupt.sigma", c.zupt_sigma);
  get_parameter("zupt.min_interval", c.zupt_min_interval);
  get_parameter("zupt.speed_thresh", c.zupt_speed_thresh);
  get_parameter("zupt.speed_timeout", c.zupt_speed_timeout);

  declare_parameter<double>("init_imu_duration", c.init_imu_duration);
  get_parameter("init_imu_duration", c.init_imu_duration);

  declare_parameter<bool>("use_init_yaw", c.use_init_yaw);
  get_parameter("use_init_yaw", c.use_init_yaw);
  declare_parameter<double>("init_yaw_deg", c.init_yaw_deg);
  get_parameter("init_yaw_deg", c.init_yaw_deg);
  
  declare_parameter<std::string>("output_reference_frame", c.output_reference_frame);
  get_parameter("output_reference_frame", c.output_reference_frame);

  declare_parameter<std::vector<double>>("lever_arm", {0.0, 0.0, 0.0});
  auto la = get_parameter("lever_arm").as_double_array();
  if (la.size() >= 3) c.lever_arm = Eigen::Vector3d(la[0], la[1], la[2]);

  declare_parameter<std::vector<double>>("imu_orientation", {0.0, 0.0, 0.0});
  auto io = get_parameter("imu_orientation").as_double_array();
  if (io.size() >= 3) c.imu_orientation_rpy = Eigen::Vector3d(io[0], io[1], io[2]);

  // Validate enum-like string parameters to catch typos early
  if (c.output_reference_frame != "gnss" && c.output_reference_frame != "imu") {
    throw std::invalid_argument(
      "output_reference_frame must be 'gnss' or 'imu', got: '" + c.output_reference_frame + "'");
  }
}

// Initialization
bool GnssImuKalmanFilter::tryInitialize() {
  if (!has_initial_gnss_ || !has_initial_imu_ || !has_initial_yaw_) return false;

  x_.setZero();

  // Set position (initial gnss pos is GNSS antenna position)
  // If output reference frame is "imu", we must translate it to the IMU origin using lever arm.
  Eigen::Vector3d p_gnss = ecefToEnu(init_gnss_pos_ecef_);

  // Set attitude from initial averaged IMU acc (roll/pitch) and GNSS heading (yaw)
  // Gravity vector in nav frame is g = (0, 0, -g).
  // At rest, a_nav = 0, so a_body = R^T * (a_nav - g) = R^T * (0, 0, g).
  // Thus, the measured accelerometer vector ideally points "Up" in body frame (e.g. +Z if flat).
  // a = [ax, ay, az]^T = R(roll, pitch, yaw)^T * [0, 0, g]^T
  // This yields ax = -g*sin(pitch), ay = g*cos(pitch)*sin(roll), az = g*cos(pitch)*cos(roll).
  Eigen::Vector3d a = applyImuAxisRotation(init_imu_acc_sum_ / std::max(1, init_imu_acc_count_));
  double roll  = std::atan2(a(1), a(2));
  double pitch = std::atan2(-a(0), std::sqrt(a(1)*a(1) + a(2)*a(2)));
  setQuaternion(eulerToQuaternion(roll, pitch, init_yaw_));

  // Determine initial position depending on output_reference_frame
  if (config_.output_reference_frame == "imu") {
    Eigen::Vector3d lever = computeLeverArmCorrection();
    x_.segment<3>(IDX_POS) = p_gnss - lever;
  } else {
    x_.segment<3>(IDX_POS) = p_gnss;
  }

  // Initial covariance (per-axis)
  P_.setZero();
  P_.diagonal().segment<3>(EIDX_POS) = config_.init_pos_std.array().square();
  P_.diagonal().segment<3>(EIDX_VEL) = config_.init_vel_std.array().square();
  P_.diagonal().segment<3>(EIDX_ATT) = config_.init_att_std.array().square();
  P_.diagonal().segment<3>(EIDX_AB)  = config_.init_acc_bias_std.array().square();
  P_.diagonal().segment<3>(EIDX_GB)  = config_.init_gyr_bias_std.array().square();

  initialized_ = true;
  RCLCPP_INFO(get_logger(), "EKF initialized. roll=%.2f pitch=%.2f yaw=%.2f [deg]",
    roll * 180.0/M_PI, pitch * 180.0/M_PI, init_yaw_ * 180.0/M_PI);
  return true;
}

// Discrete error-state transition matrix F (LOCAL/body-frame attitude error).
// Pure function of the linearization point so it can be unit-tested against
// finite differences. See the convention note at the top of this file:
//   δṗ = δv,  δv̇ = −C_bn·[acc×]·δθ − C_bn·δb_a,
//   δθ̇ = −[gyr×]·δθ − δb_g,  δḃ_a = δḃ_g = 0.
Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>
GnssImuKalmanFilter::errorStateTransition(const Eigen::Matrix3d& C_bn,
                                          const Eigen::Vector3d& acc_corrected,
                                          const Eigen::Vector3d& gyr_corrected,
                                          double dt) {
  Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> F;
  F.setIdentity();
  F.block<3,3>(EIDX_POS, EIDX_VEL) = Eigen::Matrix3d::Identity() * dt;
  // Body-frame error: ∂a_nav/∂δθ = −C_bn·[acc×]. (NOT the global −[(C_bn·acc)×].)
  F.block<3,3>(EIDX_VEL, EIDX_ATT) = -C_bn * skew(acc_corrected) * dt;
  F.block<3,3>(EIDX_VEL, EIDX_AB)  = -C_bn * dt;
  F.block<3,3>(EIDX_ATT, EIDX_ATT) = Eigen::Matrix3d::Identity() - skew(gyr_corrected) * dt;
  F.block<3,3>(EIDX_ATT, EIDX_GB)  = -Eigen::Matrix3d::Identity() * dt;
  return F;
}

// EKF Prediction (IMU-driven)
void GnssImuKalmanFilter::predict(const Eigen::Vector3d& acc_body, const Eigen::Vector3d& gyr_body, double dt) {
  // Pure-math step. dt validity is the caller's responsibility; we silently
  // no-op on non-positive dt (expected after time-aligned ZOH extrapolation
  // or out-of-order IMU samples) and on a sub-millisecond dt (negligible
  // state change). Only a clock jump (dt > 1 s) is logged.
  if (dt <= 0.0) return;
  if (dt > 1.0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "predict() skipped: dt=%.3f s out of bound (clock jump)", dt);
    return;
  }

  Eigen::Matrix3d C_bn = getRotationMatrix();
  const Eigen::Vector3d& g = kGravityEnu;

  // Corrected measurements
  Eigen::Vector3d ab = x_.segment<3>(IDX_AB);
  Eigen::Vector3d gb = x_.segment<3>(IDX_GB);
  Eigen::Vector3d acc_corrected = acc_body - ab;
  Eigen::Vector3d gyr_corrected = gyr_body - gb;

  // State propagation (lever-arm centripetal/tangential correction is negligible at typical IMU rates)
  Eigen::Vector3d acc_nav = C_bn * acc_corrected + g;
  x_.segment<3>(IDX_POS) += x_.segment<3>(IDX_VEL) * dt + 0.5 * acc_nav * dt * dt;
  x_.segment<3>(IDX_VEL) += acc_nav * dt;

  // Quaternion propagation
  Eigen::Quaterniond q = getQuaternion();
  double w_norm = gyr_corrected.norm();
  if (w_norm > 1e-10) {
    double half_angle = 0.5 * w_norm * dt;
    Eigen::Vector3d axis = gyr_corrected / w_norm;
    Eigen::Quaterniond dq(std::cos(half_angle),
                          std::sin(half_angle) * axis(0),
                          std::sin(half_angle) * axis(1),
                          std::sin(half_angle) * axis(2));
    q = q * dq;
    q.normalize();
  }
  setQuaternion(q);

  // Error-state Jacobian F (15x15), built by the testable static helper using the
  // LOCAL (body-frame) attitude-error convention (see the note above the class).
  const Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> F =
      errorStateTransition(C_bn, acc_corrected, gyr_corrected, dt);

  // Process noise Q (per-axis).
  // sigma_acc / sigma_gyr are continuous-time noise densities ([m/s^2/sqrt(Hz)],
  // [rad/s/sqrt(Hz)]) and sigma_*_bias are bias random-walk densities. Discretize
  // the white-noise-acceleration / -angular-rate model accordingly: velocity and
  // attitude process noise scale with dt (position with dt^3/3), NOT dt^2/dt^4.
  // Using the higher dt powers makes Q vanish at high IMU rates, so the filter
  // becomes overconfident in IMU propagation and diverges between GNSS updates.
  Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> Q;
  Q.setZero();
  double dt3 = dt * dt * dt;
  for (int i = 0; i < 3; ++i) {
    double sa  = config_.sigma_acc(i),      sg  = config_.sigma_gyr(i);
    double sab = config_.sigma_acc_bias(i), sgb = config_.sigma_gyr_bias(i);
    Q(EIDX_POS+i, EIDX_POS+i) = sa*sa * dt3 / 3.0;
    Q(EIDX_VEL+i, EIDX_VEL+i) = sa*sa * dt;
    Q(EIDX_ATT+i, EIDX_ATT+i) = sg*sg * dt;
    Q(EIDX_AB+i,  EIDX_AB+i)  = sab*sab * dt;
    Q(EIDX_GB+i,  EIDX_GB+i)  = sgb*sgb * dt;
  }

  P_ = F * P_ * F.transpose() + Q;
}

// Forward-only time alignment: advance the state from prev_imu_stamp_ to t_obs
// using buffered IMU samples (and ZOH on the latest sample for any tail interval).
void GnssImuKalmanFilter::predictToTime(const rclcpp::Time& t_obs) {
  if (!initialized_ || !has_prev_imu_stamp_) return;

  double total = (t_obs - prev_imu_stamp_).seconds();

  if (total <= 0.0) {
    // Out-of-sequence measurement: t_obs is older than the current state time.
    // Rewinding requires snapshots — not implemented. Caller falls back to
    // applying the observation at the current state (legacy immediate-update).
    if (total < -1e-3) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "OOSM: observation is %.0f ms older than state; applying without time alignment",
        -total * 1000.0);
    }
    return;
  }
  if (total > 1.0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "predictToTime: gap %.2f s too large, skipping forward integration", total);
    return;
  }

  // Walk forward through buffered IMU samples in (prev_imu_stamp_, t_obs).
  for (const auto& s : imu_buffer_) {
    if (s.stamp <= prev_imu_stamp_) continue;
    if (s.stamp >= t_obs) break;
    double dt = (s.stamp - prev_imu_stamp_).seconds();
    if (dt > 1e-6) predict(s.acc_body, s.gyr_body, dt);
    prev_imu_stamp_ = s.stamp;
  }

  // Tail: ZOH-extrapolate the most recent IMU sample up to t_obs — but ONLY for
  // genuinely-ahead observations. For a synchronous / high-rate observation
  // (within ~one IMU interval of the state), apply the update at the current
  // state instead: extrapolating on every observation would re-run a
  // predict-then-update cycle each time and pump the marginally-observed lateral
  // velocity, destabilizing IMU-rate wheel updates. The residual mis-alignment is
  // bounded by one IMU interval (a few cm at 100 Hz IMU).
  double dt_tail = (t_obs - prev_imu_stamp_).seconds();
  if (dt_tail > 1.5 * nominal_imu_dt_ && !imu_buffer_.empty()) {
    const auto& last = imu_buffer_.back();
    predict(last.acc_body, last.gyr_body, dt_tail);
    prev_imu_stamp_ = t_obs;
  }
}

// Push a full state+covariance snapshot for the current state time and bound the
// buffer to imu_buffer_duration (kept aligned with imu_buffer_).
void GnssImuKalmanFilter::pushStateSnapshot(const rclcpp::Time& stamp) {
  StateSnapshot s;
  s.stamp = stamp;
  s.x = x_;
  s.P = P_;
  state_buffer_.push_back(std::move(s));
  while (state_buffer_.size() > 1 &&
         (stamp - state_buffer_.front().stamp).seconds() > config_.imu_buffer_duration) {
    state_buffer_.pop_front();
  }
}

// Store-and-rewind measurement update for out-of-sequence (latent) observations.
void GnssImuKalmanFilter::applyTimeAlignedUpdate(const rclcpp::Time& t_obs,
                                                 const std::function<void()>& doUpdate) {
  // Legacy immediate-update when alignment is off or no state-time reference yet.
  if (!config_.time_align_to_gnss || !initialized_ || !has_prev_imu_stamp_) {
    doUpdate();
    return;
  }

  const double ahead = (t_obs - prev_imu_stamp_).seconds();
  if (ahead >= 0.0) {
    // Synchronous / future observation: forward-align then update (no rewind).
    predictToTime(t_obs);
    doUpdate();
    return;
  }

  // Out-of-sequence: t_obs is in the past. Rewind to the snapshot at/just-before
  // t_obs. If the observation predates the retained buffer we cannot rewind, so
  // fall back to an immediate update (bounded staleness — logged, throttled).
  if (state_buffer_.empty() || t_obs < state_buffer_.front().stamp) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "OOSM %.0f ms older than retained buffer; applying at current state",
        -ahead * 1000.0);
    doUpdate();
    return;
  }

  // Latest snapshot with stamp <= t_obs (guaranteed to exist by the guard above).
  auto snap_it = state_buffer_.rbegin();
  while (snap_it != state_buffer_.rend() && snap_it->stamp > t_obs) ++snap_it;

  const rclcpp::Time head_stamp = prev_imu_stamp_;  // state time to replay back to

  // Restore the filter to the snapshot epoch, drop now-stale later snapshots.
  x_ = snap_it->x;
  P_ = snap_it->P;
  prev_imu_stamp_ = snap_it->stamp;
  while (!state_buffer_.empty() && state_buffer_.back().stamp > prev_imu_stamp_) {
    state_buffer_.pop_back();
  }

  // Forward-integrate to the observation epoch and apply the update there.
  predictToTime(t_obs);
  doUpdate();

  // Re-propagate the buffered IMU samples from the (corrected) observation epoch
  // back up to the previous state time, rebuilding their snapshots.
  for (const auto& s : imu_buffer_) {
    if (s.stamp <= prev_imu_stamp_) continue;
    if (s.stamp > head_stamp) break;
    const double dt = (s.stamp - prev_imu_stamp_).seconds();
    if (dt > 1e-6) predict(s.acc_body, s.gyr_body, dt);
    prev_imu_stamp_ = s.stamp;
    pushStateSnapshot(s.stamp);
  }
}

// ---------------------------------------------------------------------------
// Shared measurement-update machinery. Every observation below builds only its
// innovation / H / R and delegates to kalmanUpdate(), which owns the numerics:
// gain, Joseph-form covariance update, symmetrization, ESKF attitude reset,
// and rejection of non-finite / non-invertible inputs (a single bad GNSS
// message must not NaN the whole filter).
// ---------------------------------------------------------------------------

// Inject dx into the nominal state; ESKF covariance reset for the attitude
// block (right-multiplicative local error): P <- G P G^T, G_att = I - 0.5[δθ×].
void GnssImuKalmanFilter::applyErrorState(
    const Eigen::Matrix<double, ERROR_STATE_DIM, 1>& dx) {
  x_.segment<3>(IDX_POS) += dx.segment<3>(EIDX_POS);
  x_.segment<3>(IDX_VEL) += dx.segment<3>(EIDX_VEL);
  x_.segment<3>(IDX_AB)  += dx.segment<3>(EIDX_AB);
  x_.segment<3>(IDX_GB)  += dx.segment<3>(EIDX_GB);

  const Eigen::Vector3d dtheta = dx.segment<3>(EIDX_ATT);
  const double angle = dtheta.norm();
  if (angle > 1e-12) {
    Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, dtheta / angle));
    Eigen::Quaterniond q = getQuaternion() * dq;
    q.normalize();
    setQuaternion(q);

    // After injection the attitude error is re-defined about the new nominal
    // quaternion; transform P accordingly (Solà 2017, eq. 285ff).
    Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> G;
    G.setIdentity();
    G.block<3,3>(EIDX_ATT, EIDX_ATT) -= 0.5 * skew(dtheta);
    P_ = G * P_ * G.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
  }
}

bool GnssImuKalmanFilter::kalmanUpdate(const Eigen::VectorXd& innovation,
                                       const Eigen::MatrixXd& H,
                                       const Eigen::MatrixXd& R) {
  if (!innovation.allFinite() || !H.allFinite() || !R.allFinite()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "measurement update skipped: non-finite innovation/H/R");
    return false;
  }

  const Eigen::MatrixXd S = H * P_ * H.transpose() + R;
  Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "measurement update skipped: innovation covariance not positive definite");
    return false;
  }

  // K = P H^T S^-1 = (S^-1 H P)^T  (P, S symmetric)
  const Eigen::MatrixXd K = ldlt.solve(H * P_).transpose();
  const Eigen::VectorXd dx = K * innovation;
  if (!dx.allFinite()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "measurement update skipped: non-finite state correction");
    return false;
  }

  const Eigen::MatrixXd I15 =
      Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Identity();
  const Eigen::MatrixXd IKH = I15 - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
  P_ = 0.5 * (P_ + P_.transpose());

  applyErrorState(dx);
  return true;
}

// GNSS Position Update
void GnssImuKalmanFilter::updateGnssPosition(const Eigen::Vector3d& z_pos,
                                  const Eigen::Matrix3d& R_pos) {
  // Observation: GNSS Antenna Position z_pos = p_gnss
  Eigen::Vector3d innovation;
  Eigen::Matrix<double, 3, ERROR_STATE_DIM> H;
  H.setZero();

  if (config_.output_reference_frame == "imu") {
    // EKF state is IMU position: p_gnss = p_imu + R * l^b
    Eigen::Vector3d lever = computeLeverArmCorrection();
    Eigen::Vector3d p_gnss_pred = x_.segment<3>(IDX_POS) + lever;
    innovation = z_pos - p_gnss_pred;

    H.block<3,3>(0, EIDX_POS) = Eigen::Matrix3d::Identity();

    // Jacobian of R * l^b w.r.t rotation delta theta is -R * [l^b]_x
    Eigen::Matrix3d C_bn = getRotationMatrix();
    H.block<3,3>(0, EIDX_ATT) = -C_bn * skew(config_.lever_arm);
  } else {
    // EKF state is already GNSS position
    innovation = z_pos - x_.segment<3>(IDX_POS);
    H.block<3,3>(0, EIDX_POS) = Eigen::Matrix3d::Identity();
  }

  kalmanUpdate(innovation, H, R_pos);
}

// GNSS Heading Update (from Doppler velocity)
void GnssImuKalmanFilter::updateGnssHeading(double heading_rad, double heading_var) {
  Eigen::Quaterniond q_pred = getQuaternion();
  Eigen::Vector3d euler = quaternionToEuler(q_pred);
  double yaw_pred = euler(2);

  double yaw_err = heading_rad - yaw_pred;
  // Normalize to [-pi, pi]
  while (yaw_err >  M_PI) yaw_err -= 2.0 * M_PI;
  while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

  // The state carries a LOCAL (body-frame) attitude error δθ; the nav-frame
  // rotation error is C_bn·δθ, so the observed nav-frame yaw error is its
  // Z-component: yaw_err = (C_bn·δθ)_z = C_bn.row(2)·δθ. Near level C_bn.row(2)
  // ≈ [0,0,1], but using the exact third row keeps the update correct under tilt.
  Eigen::Matrix3d C_bn = getRotationMatrix();
  Eigen::Matrix<double, 1, ERROR_STATE_DIM> H;
  H.setZero();
  H.block<1,3>(0, EIDX_ATT) = C_bn.row(2);

  Eigen::VectorXd innovation(1);
  innovation(0) = yaw_err;
  Eigen::MatrixXd R(1, 1);
  R(0, 0) = heading_var;
  kalmanUpdate(innovation, H, R);
}

// GNSS Velocity Update (nav-frame Doppler velocity)
//   Observation: GNSS antenna velocity z_vel = v_gnss, in the working frame.
//   This is a standard, independent velocity aiding update (complementary to the
//   wheel-speed body-frame longitudinal update).
void GnssImuKalmanFilter::updateGnssVelocity(const Eigen::Vector3d& z_vel,
                                             const Eigen::Matrix3d& R_vel) {
  Eigen::Matrix<double, 3, ERROR_STATE_DIM> H;
  H.setZero();
  H.block<3,3>(0, EIDX_VEL) = Eigen::Matrix3d::Identity();

  // EKF velocity state is the velocity at the output reference frame. When the
  // state is the IMU velocity, the GNSS observes the antenna velocity, which adds
  // the lever-arm rate term v_gnss = v_imu + C_bn * (omega_body x lever).
  Eigen::Vector3d v_pred = x_.segment<3>(IDX_VEL);
  if (config_.output_reference_frame == "imu" && config_.lever_arm.norm() > 0.0) {
    Eigen::Matrix3d C_bn = getRotationMatrix();
    Eigen::Vector3d omega = latest_imu_gyr_ - x_.segment<3>(IDX_GB);
    Eigen::Vector3d wxl = omega.cross(config_.lever_arm);
    v_pred += C_bn * wxl;

    H.block<3,3>(0, EIDX_ATT) = -C_bn * skew(wxl);              // d(C_bn*(w x l))/dtheta
    H.block<3,3>(0, EIDX_GB)  = C_bn * skew(config_.lever_arm); // d(C_bn*(w x l))/d(gyro_bias)
  }

  kalmanUpdate(z_vel - v_pred, H, R_vel);
}

// Continuous Accelerometer Leveling Update (gravity-direction observation)
//   The bias-corrected specific force points along +Up in the body frame:
//   f_body ≈ -C_bnᵀ·g = ||g||·C_bnᵀ·e_z. Observing its unit direction constrains
//   roll/pitch directly (yaw — rotation about gravity — stays unobserved: H's
//   null space contains u_pred). With a low-grade IMU the gravity-dominated
//   specific force turns a sub-degree attitude error into a large spurious
//   horizontal acceleration ("gravity leak") that dead-reckons between GNSS
//   fixes; running this update continuously keeps roll/pitch — and thus the
//   gravity projection — bounded. Instead of a hard quasi-static gate the
//   measurement noise is inflated by the departure from pure gravity
//   (| ||f||-g |) and by the angular rate, so the update self-weights: strong
//   when near-static, negligible under manoeuvre.
void GnssImuKalmanFilter::updateLeveling() {
  if (!config_.leveling_enable || !initialized_ || imu_buffer_.empty()) return;

  // Moving average of the specific force / angular rate over leveling_window to
  // reject per-sample IMU noise (which would otherwise trip the a_lin gate and
  // corrupt the gravity direction on a low-grade IMU).
  const rclcpp::Time now = imu_buffer_.back().stamp;
  Eigen::Vector3d acc_sum = Eigen::Vector3d::Zero(), gyr_sum = Eigen::Vector3d::Zero();
  int cnt = 0;
  for (auto it = imu_buffer_.rbegin(); it != imu_buffer_.rend(); ++it) {
    if ((now - it->stamp).seconds() > config_.leveling_window) break;
    acc_sum += it->acc_body;
    gyr_sum += it->gyr_body;
    ++cnt;
  }
  if (cnt < 1) return;

  const Eigen::Matrix3d C_bn = getRotationMatrix();
  const Eigen::Vector3d f = acc_sum / cnt - x_.segment<3>(IDX_AB);
  const Eigen::Vector3d w = gyr_sum / cnt - x_.segment<3>(IDX_GB);
  if (f.norm() < 1e-3) return;

  // "Specific force == gravity" holds only when the linear (kinematic)
  // acceleration is zero. The correct measure of how far a sample departs from
  // that is the nav-frame kinematic acceleration a_lin = C_bn·f + g — NOT
  // | ||f|| - g |, which is nearly blind to horizontal (centripetal/lateral)
  // acceleration and would let a coordinated turn masquerade as tilt.
  const double a_lin = (C_bn * f + kGravityEnu).norm();
  if (a_lin > config_.leveling_max_acc) return;  // clearly manoeuvring

  const Eigen::Vector3d u_meas = f.normalized();
  const Eigen::Vector3d u_pred = C_bn.transpose() * Eigen::Vector3d::UnitZ();

  // h(δθ) = (C_bn·exp([δθ×]))ᵀ·e_z ≈ u_pred + [u_pred×]·δθ  (local error)
  Eigen::Matrix<double, 3, ERROR_STATE_DIM> H;
  H.setZero();
  H.block<3,3>(0, EIDX_ATT) = skew(u_pred);

  // Adaptive 1-sigma [rad]: floor + linear inflation with the kinematic
  // acceleration and the angular rate, so the update self-weights (strong near
  // static, negligible under manoeuvre).
  const double sigma = (config_.leveling_sigma_min_deg
                        + config_.leveling_acc_gain * a_lin
                        + config_.leveling_gyr_gain * w.norm()) * M_PI / 180.0;
  const Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * sigma * sigma;

  kalmanUpdate(u_meas - u_pred, H, R);
}

// Zero-Velocity Update (ZUPT)
//   Stationarity is detected over the trailing zupt_window of buffered IMU
//   samples (acc/gyro standard deviation thresholds); while stationary a v = 0
//   pseudo-observation bounds velocity/position drift (standstill, GNSS gaps).
void GnssImuKalmanFilter::updateZupt(const rclcpp::Time& stamp) {
  if (!config_.zupt_enable || !initialized_) return;
  if ((stamp - last_zupt_stamp_).seconds() < config_.zupt_min_interval) return;
  // IMU variance cannot distinguish rest from smooth constant-velocity motion,
  // so suppress ZUPT when a fresh MEASURED speed reference (GNSS/wheel) says the
  // platform is moving. When no fresh reference exists (e.g. GNSS outage — where
  // ZUPT matters most) the IMU-variance test below stands alone.
  if (std::isfinite(last_meas_speed_) &&
      (stamp - last_meas_speed_stamp_).seconds() < config_.zupt_speed_timeout &&
      last_meas_speed_ > config_.zupt_speed_thresh) {
    return;
  }

  // Trailing-window mean/variance of the buffered body-frame samples.
  Eigen::Vector3d acc_sum = Eigen::Vector3d::Zero(), acc_sq = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyr_sum = Eigen::Vector3d::Zero(), gyr_sq = Eigen::Vector3d::Zero();
  int n = 0;
  for (auto it = imu_buffer_.rbegin(); it != imu_buffer_.rend(); ++it) {
    if ((stamp - it->stamp).seconds() > config_.zupt_window) break;
    acc_sum += it->acc_body;
    acc_sq  += it->acc_body.cwiseProduct(it->acc_body);
    gyr_sum += it->gyr_body;
    gyr_sq  += it->gyr_body.cwiseProduct(it->gyr_body);
    ++n;
  }
  if (n < 5) return;  // window not sufficiently populated

  const Eigen::Vector3d acc_var =
      (acc_sq / n - (acc_sum / n).cwiseProduct(acc_sum / n)).cwiseMax(0.0);
  const Eigen::Vector3d gyr_var =
      (gyr_sq / n - (gyr_sum / n).cwiseProduct(gyr_sum / n)).cwiseMax(0.0);
  const double acc_std = std::sqrt(acc_var.sum() / 3.0);
  const double gyr_std = std::sqrt(gyr_var.sum() / 3.0);
  if (acc_std > config_.zupt_acc_std_thresh || gyr_std > config_.zupt_gyr_std_thresh) return;

  Eigen::Matrix<double, 3, ERROR_STATE_DIM> H;
  H.setZero();
  H.block<3,3>(0, EIDX_VEL) = Eigen::Matrix3d::Identity();

  const Eigen::Matrix3d R =
      Eigen::Matrix3d::Identity() * config_.zupt_sigma * config_.zupt_sigma;

  const Eigen::Vector3d innovation = -x_.segment<3>(IDX_VEL);  // z = 0
  if (kalmanUpdate(innovation, H, R)) last_zupt_stamp_ = stamp;
}

// Wheel Speed Update
void GnssImuKalmanFilter::onWheelSpeedWithCov(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
  Eigen::Vector3d linear(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
  Eigen::Matrix3d R_vel = Eigen::Matrix3d::Identity() * config_.wheel_speed_sigma * config_.wheel_speed_sigma;
  
  double cov_sum = 0;
  for (int i = 0; i < 3; ++i) cov_sum += msg->twist.covariance[i*6+i];
  if (cov_sum > 1e-10) {
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        R_vel(r,c) = msg->twist.covariance[r*6+c];
    R_vel += Eigen::Matrix3d::Identity() * 1e-6;
  }
  
  std::lock_guard<std::mutex> lock(mtx_);
  latest_wheel_.valid = true;
  latest_wheel_.stamp = msg->header.stamp;
  latest_wheel_.linear = linear;
  for (int i = 0; i < 36; ++i) latest_wheel_.covariance[i] = msg->twist.covariance[i];

  processWheelSpeed(linear, R_vel, msg->header.stamp);
}

void GnssImuKalmanFilter::onWheelSpeedPoint(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  Eigen::Vector3d linear(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
  Eigen::Matrix3d R_vel = Eigen::Matrix3d::Identity() * config_.wheel_speed_sigma * config_.wheel_speed_sigma;

  std::lock_guard<std::mutex> lock(mtx_);
  latest_wheel_.valid = true;
  latest_wheel_.stamp = msg->header.stamp;
  latest_wheel_.linear = linear;
  latest_wheel_.covariance.fill(0);

  processWheelSpeed(linear, R_vel, msg->header.stamp);
}

// Caller must hold mtx_.
void GnssImuKalmanFilter::processWheelSpeed(const Eigen::Vector3d& linear_velocity,
                                            const Eigen::Matrix3d& covariance,
                                            const rclcpp::Time& t_obs) {
  if (!initialized_) return;

  // Cache the measured longitudinal wheel speed for the ZUPT gate.
  last_meas_speed_ = std::fabs(linear_velocity(0));
  last_meas_speed_stamp_ = t_obs;

  // Time alignment (store-and-rewind for out-of-sequence samples) is handled by
  // applyTimeAlignedUpdate(); the update body below reads the (rewound) state.
  applyTimeAlignedUpdate(t_obs, [&]() {
  Eigen::Matrix3d C_bn = getRotationMatrix();
  Eigen::Vector3d v_nav_pred = x_.segment<3>(IDX_VEL);

  // Transform predicted nav velocity into body frame to compare directly with linear_velocity
  Eigen::Vector3d v_body_pred = C_bn.transpose() * v_nav_pred;

  if (config_.wheel_speed_mode == "longitudinal_only" && !config_.wheel_nhc_enable) {
    // Only constrain longitudinal (X) velocity
    Eigen::Matrix<double, 1, ERROR_STATE_DIM> H;
    H.setZero();
    // Jacobian of v_body_x w.r.t v_nav: First row of C_bn^T
    H.block<1,3>(0, EIDX_VEL) = C_bn.transpose().row(0);
    // Jacobian of v_body w.r.t the LOCAL (body-frame) attitude error δθ.
    // With v_body = C_bnᵀ·v_nav and the right-multiplicative error
    // C_true = C_bn·exp([δθ×]), ∂v_body/∂δθ = [v_body×] = C_bnᵀ·[v_nav×]·C_bn.
    H.block<1,3>(0, EIDX_ATT) = (C_bn.transpose() * skew(v_nav_pred) * C_bn).row(0);

    Eigen::VectorXd innovation(1);
    innovation(0) = linear_velocity(0) - v_body_pred(0);
    Eigen::MatrixXd R(1, 1);
    R(0, 0) = covariance(0, 0);
    kalmanUpdate(innovation, H, R);
  } else {
    // 3D body-frame velocity observation. In "3d" mode the full measured body
    // velocity is used. In "longitudinal_only" mode with NHC enabled, the
    // measurement is [wheel, 0, 0] and the lateral/vertical rows are the
    // non-holonomic zero-velocity constraints (their own sigmas) — this directly
    // observes the lateral velocity that the longitudinal-only update would leave
    // free.
    Eigen::Vector3d z_body = linear_velocity;
    Eigen::Matrix3d R_body = covariance;
    if (config_.wheel_speed_mode == "longitudinal_only") {
      z_body = Eigen::Vector3d(linear_velocity(0), 0.0, 0.0);
      R_body = Eigen::Vector3d(
          config_.wheel_speed_sigma * config_.wheel_speed_sigma,
          config_.wheel_nhc_sigma_lateral * config_.wheel_nhc_sigma_lateral,
          config_.wheel_nhc_sigma_vertical * config_.wheel_nhc_sigma_vertical)
          .asDiagonal();
    }
    Eigen::Matrix<double, 3, ERROR_STATE_DIM> H;
    H.setZero();
    H.block<3,3>(0, EIDX_VEL) = C_bn.transpose();

    // ∂v_body/∂δθ = [v_body×] = C_bnᵀ·[v_nav×]·C_bn (LOCAL body-frame error).
    H.block<3,3>(0, EIDX_ATT) = C_bn.transpose() * skew(v_nav_pred) * C_bn;

    kalmanUpdate(z_body - v_body_pred, H, R_body);
  }

  latest_wheel_.used_for_update = true;
  });
}

// IMU Callback
void GnssImuKalmanFilter::onImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mtx_);

  // Raw IMU data (apply axis rotation)
  Eigen::Vector3d acc_raw(msg->linear_acceleration.x,
                          msg->linear_acceleration.y,
                          msg->linear_acceleration.z);
  Eigen::Vector3d gyr_raw(msg->angular_velocity.x,
                          msg->angular_velocity.y,
                          msg->angular_velocity.z);

  Eigen::Vector3d acc_body = applyImuAxisRotation(acc_raw);
  Eigen::Vector3d gyr_body = applyImuAxisRotation(gyr_raw);

  // Store for CSV and initialization
  latest_imu_acc_ = acc_body;
  latest_imu_gyr_ = gyr_body;
  for (int i = 0; i < 9; ++i) {
    latest_imu_acc_cov_[i] = msg->linear_acceleration_covariance[i];
    latest_imu_gyr_cov_[i] = msg->angular_velocity_covariance[i];
  }

  // Collect initial IMU for roll/pitch average
  if (!has_initial_imu_) {
    if (init_imu_acc_count_ == 0) {
      init_imu_start_time_ = msg->header.stamp;
    }
    
    init_imu_acc_sum_ += acc_raw;
    init_imu_acc_count_++;
    
    rclcpp::Time current_time = msg->header.stamp;
    if ((current_time - init_imu_start_time_).seconds() >= config_.init_imu_duration) {
      has_initial_imu_ = true;
      RCLCPP_INFO(get_logger(), "Initial IMU acquired. Averaged %d samples for roll/pitch.", init_imu_acc_count_);
      tryInitialize();
    }
    return;
  }

  rclcpp::Time stamp = msg->header.stamp;

  // Push this sample to the ring buffer used by predictToTime() for time-aligned
  // observation updates. Body-frame values (post mounting rotation) are stored.
  imu_buffer_.push_back(ImuSample{stamp, acc_body, gyr_body});
  while (imu_buffer_.size() > 1 &&
         (stamp - imu_buffer_.front().stamp).seconds() > config_.imu_buffer_duration) {
    imu_buffer_.pop_front();
  }

  // Write sensors CSV and clear snapshots at IMU rate (before EKF initialized).
  // Clearing here (not after predict) ensures each GNSS snapshot appears in exactly one row.
  writeSensorsRow(stamp);
  latest_gnss_.valid = false;
  latest_gnss_.used_for_update = false;
  latest_wheel_.valid = false;
  latest_wheel_.used_for_update = false;

  if (!initialized_) return;

  if (has_prev_imu_stamp_) {
    double dt = (stamp - prev_imu_stamp_).seconds();
    if (dt > 0.0) {
      // Track the nominal IMU interval (used to gate predictToTime()).
      if (dt < 1.0) nominal_imu_dt_ = 0.9 * nominal_imu_dt_ + 0.1 * dt;
      predict(acc_body, gyr_body, dt);
      prev_imu_stamp_ = stamp;
      // IMU-epoch aiding: continuous (adaptive) leveling keeps roll/pitch bounded
      // against gravity leak; ZUPT pins velocity while stationary.
      updateLeveling();
      updateZupt(stamp);
      // Snapshot the state at this epoch so a later out-of-sequence observation
      // can rewind to it (store-and-rewind, see applyTimeAlignedUpdate()).
      pushStateSnapshot(stamp);
    }
    // dt <= 0: state already passed this IMU's epoch (after a GNSS-driven ZOH
    // extrapolation, or because of upstream IMU out-of-order publishing).
    // Leave prev_imu_stamp_ untouched so the next IMU computes dt against the
    // current state time.
  } else {
    prev_imu_stamp_ = stamp;
    has_prev_imu_stamp_ = true;
    pushStateSnapshot(stamp);
  }

  writeStateRow(stamp);
  publishSolution(stamp);
}

// GNSS Callback
void GnssImuKalmanFilter::onGnss(const grs::GnssSolution::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mtx_);

  // Build snapshot regardless of update validity
  GnssSnapshot snap;
  snap.stamp = msg->header.stamp;
  snap.tow = msg->time_tow;
  snap.week = msg->time_week;
  snap.status = msg->status;
  snap.num_sats = msg->num_sats;
  snap.gdop = msg->gdop;
  snap.pdop = msg->pdop;
  snap.hdop = msg->hdop;
  snap.vdop = msg->vdop;
  snap.ratio = msg->ratio;
  snap.age_diff = msg->age_diff;

  snap.lat = msg->latitude;
  snap.lon = msg->longitude;
  snap.alt = msg->altitude;

  snap.pos_ecef = Eigen::Vector3d(msg->pos_ecef.x, msg->pos_ecef.y, msg->pos_ecef.z);
  for (int i=0; i<9; i++) snap.pos_cov_ecef[i] = msg->pos_cov_ecef[i];

  snap.pos_enu = Eigen::Vector3d(msg->pos_enu.x, msg->pos_enu.y, msg->pos_enu.z);
  for (int i=0; i<9; i++) snap.pos_cov_enu[i] = msg->pos_enu_cov[i];

  snap.vel_ecef = Eigen::Vector3d(msg->vel_ecef.x, msg->vel_ecef.y, msg->vel_ecef.z);
  for (int i=0; i<9; i++) snap.vel_cov_ecef[i] = msg->vel_cov_ecef[i];

  snap.vel_enu = Eigen::Vector3d(msg->vel_enu.x, msg->vel_enu.y, msg->vel_enu.z);
  for (int i=0; i<9; i++) snap.vel_cov_enu[i] = msg->vel_enu_cov[i];

  // Check NaN in position
  bool pos_nan = std::isnan(snap.pos_ecef(0)) || std::isnan(snap.pos_ecef(1)) || std::isnan(snap.pos_ecef(2));
  snap.pos_is_nan = pos_nan;
  snap.valid = true;

  latest_gnss_ = snap;

  // A solution is usable iff its quality matches the configured gnss_update_mode.
  // The SAME gate is applied to BOTH initialization and measurement updates: the
  // filter must not start dead-reckoning from (or be aided by) a lower-quality
  // solution than the user asked for. E.g. with fix_only the EKF waits for the
  // first RTK FIX before initializing, instead of starting on an early FLOAT/Single
  // solution and then drifting on IMU alone until FIX arrives.
  auto solution_usable = [&](uint8_t status) {
    if (pos_nan || status == grs::GnssSolution::STATUS_NONE) return false;
    switch (config_.gnss_update_mode) {
      case GnssUpdateMode::FIX_ONLY:
        return status == grs::GnssSolution::STATUS_FIX;
      case GnssUpdateMode::FIX_FLOAT:
        return status == grs::GnssSolution::STATUS_FIX ||
               status == grs::GnssSolution::STATUS_FLOAT;
      case GnssUpdateMode::ALL:
        return true;
    }
    return false;
  };
  const bool usable = solution_usable(msg->status);

  // --- Initialization ---
  if (!has_initial_gnss_ && usable) {
    init_gnss_pos_ecef_ = snap.pos_ecef;

    // Set local origin
    if (config_.local_origin_mode == "gnss_fix" || !origin_set_) {
      origin_ecef_ = snap.pos_ecef;
      double e3[3] = {snap.pos_ecef(0), snap.pos_ecef(1), snap.pos_ecef(2)};
      double llh[3];
      ecef2pos(e3, llh);
      origin_llh_ = Eigen::Vector3d(llh[0], llh[1], llh[2]);
      origin_set_ = true;
    }

    has_initial_gnss_ = true;
    RCLCPP_INFO(get_logger(), "Initial GNSS position acquired");

    // Use configured initial yaw if use_init_yaw is true, otherwise try Doppler heading
    // (from the origin-frame velocity — origin_llh_ is now valid, set just above).
    if (!has_initial_yaw_ && config_.use_init_yaw) {
      init_yaw_ = config_.init_yaw_deg * M_PI / 180.0;
      has_initial_yaw_ = true;
      RCLCPP_INFO(get_logger(), "Initial yaw from config: %.2f [deg]", config_.init_yaw_deg);
    } else if (!has_initial_yaw_) {
      const double origin_doppler_heading =
          dopplerHeadingFromOriginVelocity(computeOriginFrameVelocity(snap));
      if (!std::isnan(origin_doppler_heading)) {
        init_yaw_ = origin_doppler_heading;
        has_initial_yaw_ = true;
        RCLCPP_INFO(get_logger(), "Initial yaw from GNSS Doppler: %.2f [deg]",
                    init_yaw_ * 180.0/M_PI);
      }
    }

    tryInitialize();
    return;
  }

  // Try initial yaw if not yet set (origin-frame Doppler heading)
  if (has_initial_gnss_ && !has_initial_yaw_) {
    const double origin_doppler_heading =
        dopplerHeadingFromOriginVelocity(computeOriginFrameVelocity(snap));
    if (!std::isnan(origin_doppler_heading)) {
      init_yaw_ = origin_doppler_heading;
      has_initial_yaw_ = true;
      RCLCPP_INFO(get_logger(), "Initial yaw from GNSS Doppler: %.2f [deg]",
                  init_yaw_ * 180.0/M_PI);
      tryInitialize();
      return;
    }
  }

  if (!initialized_) return;

  // --- Measurement update (only for solutions matching gnss_update_mode) ---
  if (usable) {
    // GNSS solutions are stamped at their measurement epoch but delivered late
    // (RTK base matching + rtkpos: tens–hundreds of ms, variable), so by arrival
    // the IMU-driven state has moved past that epoch. Precompute the measurement
    // quantities (state-independent, from the message), then apply all three
    // updates at the true epoch via store-and-rewind: applyTimeAlignedUpdate()
    // rewinds the filter to the observation epoch, updates, and re-propagates the
    // buffered IMU forward — eliminating the OOSM position/velocity kick that
    // otherwise makes the trajectory zig-zag.

    // Position observation
    Eigen::Vector3d z_pos = ecefToEnu(snap.pos_ecef);
    double cov_enu[9];
    gnss_utils::rotateCovariance(snap.pos_cov_ecef.data(), origin_llh_(0), origin_llh_(1), cov_enu);
    Eigen::Matrix3d R_pos = Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>>(cov_enu);
    // Guard a degenerate (e.g. all-zero) GNSS position covariance: some receivers
    // publish zero cov meaning "unknown". Without this, the +1e-6 floor below turns
    // it into a ~1 mm, wildly over-confident observation that snaps the fused
    // position (a direct cause of the zig-zag on zero-cov data). Treat it as unknown
    // and substitute a conservative configured default. This mirrors the velocity/
    // heading zero-cov guards, which skip; position is the primary aid so we
    // down-weight instead of skipping.
    if (R_pos.diagonal().maxCoeff() < config_.gnss_cov_min_var) {
      // Choose the fallback by solution status: an RTK fix is cm-level, a coarse
      // SINGLE/DGPS fix is metre-level - trusting them equally is what snaps the
      // fused position on zero-cov data.
      const Eigen::Vector3d& sig =
          snap.status == grs::GnssSolution::STATUS_FIX
              ? config_.gnss_pos_sigma_default_fix
              : (snap.status == grs::GnssSolution::STATUS_FLOAT
                     ? config_.gnss_pos_sigma_default_float
                     : config_.gnss_pos_sigma_default_other);
      R_pos = sig.array().square().matrix().asDiagonal();
    }
    // Ensure R_pos is positive definite (add small diagonal if needed)
    R_pos += Eigen::Matrix3d::Identity() * 1e-6;

    // Velocity observation (nav-frame Doppler velocity), rotated into the
    // FIXED origin ENU frame — the same frame as the EKF velocity state (see
    // computeOriginFrameVelocity()). Independent of and complementary to the
    // wheel-speed update; applied whenever the solution carries a usable
    // velocity covariance.
    const OriginVelocity origin_vel = computeOriginFrameVelocity(snap);
    Eigen::Vector3d z_vel = origin_vel.vel_enu;
    Eigen::Matrix3d R_vel = origin_vel.cov_enu;
    // Note: RTKLIB defines a `trace(...)` macro, so avoid Eigen's .trace().
    double r_diag = R_vel(0,0) + R_vel(1,1) + R_vel(2,2);
    // Degenerate (zero) velocity covariance: substitute a conservative default so
    // the velocity and Doppler-heading updates still work (mirrors the position
    // guard). Skip when the reported velocity is exactly zero: a receiver emitting
    // neither velocity nor covariance sends all-zeros, indistinguishable from a
    // true standstill, so substituting there would inject a false v=0 observation.
    if (origin_vel.valid && r_diag <= 1e-12 && config_.gnss_vel_sigma_default > 0.0 &&
        z_vel.allFinite() && z_vel.squaredNorm() > 0.0) {
      const double s2 =
          config_.gnss_vel_sigma_default * config_.gnss_vel_sigma_default;
      R_vel = Eigen::Matrix3d::Identity() * s2;
      r_diag = R_vel(0,0) + R_vel(1,1) + R_vel(2,2);
    }
    const bool vel_usable = origin_vel.valid && z_vel.allFinite() && r_diag > 1e-12;
    if (vel_usable) R_vel += Eigen::Matrix3d::Identity() * 1e-6;

    // Cache the measured horizontal speed for the ZUPT gate (generic, not the
    // filter's own estimate).
    if (vel_usable) {
      last_meas_speed_ = std::hypot(z_vel(0), z_vel(1));
      last_meas_speed_stamp_ = msg->header.stamp;
    }

    // Doppler heading observation, derived from the same origin-frame z_vel
    // so it stays consistent with R_vel and the EKF's fixed-origin ENU
    // velocity state. Requires a usable velocity covariance: with a missing/
    // zero covariance the propagated variance collapses to the floor and an
    // over-confident yaw update corrupts the attitude — skip instead. Use the
    // (possibly substituted) R_vel so the heading update also benefits from
    // the degenerate-covariance fallback above.
    const double ve = z_vel(0), vn = z_vel(1);
    const double horiz_speed = std::hypot(ve, vn);
    const double var_ve = R_vel(0,0);
    const double var_vn = R_vel(1,1);
    const bool vel_cov_ok = std::isfinite(var_ve) && std::isfinite(var_vn) &&
                            (var_ve > 1e-12 || var_vn > 1e-12);
    const bool heading_usable =
        vel_usable &&
        horiz_speed > config_.gnss_heading_speed_threshold && vel_cov_ok;
    double heading_var = 0.0;
    double doppler_heading = 0.0;
    if (heading_usable) {
      doppler_heading = std::atan2(vn, ve);  // ENU yaw: from East, CCW
      // Error propagation of atan2(vn, ve) through the velocity covariance,
      // floored at (5 deg)^2: the course-over-ground is only an approximate
      // yaw observation (sideslip, antenna sway), never trust it below that.
      constexpr double kHeadingVarFloor = 7.6e-3;  // (5 deg)^2 [rad^2]
      const double s2 = horiz_speed * horiz_speed;
      heading_var = std::max((var_ve * vn*vn + var_vn * ve*ve) / (s2 * s2),
                             kHeadingVarFloor);
    }

    applyTimeAlignedUpdate(msg->header.stamp, [&]() {
      updateGnssPosition(z_pos, R_pos);
      if (vel_usable) updateGnssVelocity(z_vel, R_vel);
      if (heading_usable) updateGnssHeading(doppler_heading, heading_var);
    });
    latest_gnss_.used_for_update = true;
  }
}



// Publish Solution
void GnssImuKalmanFilter::publishSolution(const rclcpp::Time& stamp) {
  auto sol = grs::GnssSolution();
  sol.header.stamp = stamp;
  sol.header.frame_id = "gnss_imu_kalman_filter";

  Eigen::Vector3d pos = x_.segment<3>(IDX_POS);
  Eigen::Vector3d vel = x_.segment<3>(IDX_VEL);
  Eigen::Vector3d llh = enuToLlh(pos);
  Eigen::Vector3d ecef_pos = enuToEcef(pos);

  sol.latitude   = llh(0);
  sol.longitude  = llh(1);
  sol.altitude   = llh(2);

  sol.pos_ecef.x = ecef_pos(0);
  sol.pos_ecef.y = ecef_pos(1);
  sol.pos_ecef.z = ecef_pos(2);

  // Covariance: state covariance is ENU; rotate ENU→ECEF for the ECEF fields.
  Eigen::Matrix3d P_pp = P_.block<3,3>(EIDX_POS, EIDX_POS);
  Eigen::Matrix3d P_vv = P_.block<3,3>(EIDX_VEL, EIDX_VEL);
  for (int i = 0; i < 9; ++i) sol.pos_enu_cov[i] = P_pp(i/3, i%3);
  for (int i = 0; i < 9; ++i) sol.vel_enu_cov[i] = P_vv(i/3, i%3);
  double cov_ecef[9];
  gnss_utils::rotateCovarianceEnuToEcef(sol.pos_enu_cov.data(), origin_llh_(0), origin_llh_(1), cov_ecef);
  for (int i = 0; i < 9; ++i) sol.pos_cov_ecef[i] = cov_ecef[i];
  gnss_utils::rotateCovarianceEnuToEcef(sol.vel_enu_cov.data(), origin_llh_(0), origin_llh_(1), cov_ecef);
  for (int i = 0; i < 9; ++i) sol.vel_cov_ecef[i] = cov_ecef[i];

  sol.pos_enu_org_ecef.x = origin_ecef_(0);
  sol.pos_enu_org_ecef.y = origin_ecef_(1);
  sol.pos_enu_org_ecef.z = origin_ecef_(2);

  sol.pos_enu.x = pos(0);
  sol.pos_enu.y = pos(1);
  sol.pos_enu.z = pos(2);

  // Velocity: ENU state; ECEF via the same ENU→ECEF rotation (valid for vectors).
  sol.vel_enu.x = vel(0);
  sol.vel_enu.y = vel(1);
  sol.vel_enu.z = vel(2);
  {
    double pos_ref[3] = {origin_llh_(0), origin_llh_(1), origin_llh_(2)};
    double v_enu[3] = {vel(0), vel(1), vel(2)};
    double v_ecef[3];
    enu2ecef(pos_ref, v_enu, v_ecef);
    sol.vel_ecef.x = v_ecef[0];
    sol.vel_ecef.y = v_ecef[1];
    sol.vel_ecef.z = v_ecef[2];
  }

  sol.status = grs::GnssSolution::STATUS_EKF;

  solution_pub_->publish(sol);

  // Publish Odometry message
  auto odom = nav_msgs::msg::Odometry();
  odom.header.stamp = stamp;
  odom.header.frame_id = "odom";
  odom.child_frame_id = config_.output_reference_frame == "imu" ? "imu_link" : "gnss_antenna";

  // Pos (local ENU)
  odom.pose.pose.position.x = pos(0);
  odom.pose.pose.position.y = pos(1);
  odom.pose.pose.position.z = pos(2);

  // Quat
  Eigen::Quaterniond q = getQuaternion();
  odom.pose.pose.orientation.w = q.w();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();

  // Vel in body frame (REP-105: twist must be expressed in child_frame_id)
  Eigen::Matrix3d C_bn = getRotationMatrix();
  Eigen::Vector3d vel_body = C_bn.transpose() * vel;
  odom.twist.twist.linear.x = vel_body(0);
  odom.twist.twist.linear.y = vel_body(1);
  odom.twist.twist.linear.z = vel_body(2);

  // Covariance arrays: Odometry uses 36-element arrays for 6D pose/twist
  Eigen::Matrix3d P_vv_body = C_bn.transpose() * P_vv * C_bn;
  for (int min_i = 0; min_i < 3; ++min_i) {
    for (int min_j = 0; min_j < 3; ++min_j) {
      odom.pose.covariance[min_i * 6 + min_j] = P_(EIDX_POS + min_i, EIDX_POS + min_j);        // pos-pos
      odom.pose.covariance[(min_i + 3) * 6 + (min_j + 3)] = P_(EIDX_ATT + min_i, EIDX_ATT + min_j); // att-att
      odom.pose.covariance[min_i * 6 + (min_j + 3)] = P_(EIDX_POS + min_i, EIDX_ATT + min_j);       // pos-att
      odom.pose.covariance[(min_i + 3) * 6 + min_j] = P_(EIDX_ATT + min_i, EIDX_POS + min_j);       // att-pos

      odom.twist.covariance[min_i * 6 + min_j] = P_vv_body(min_i, min_j);  // vel in body frame
    }
  }

  odom_pub_->publish(odom);
}

// CSV Output — Sensors Log
//   Rows at IMU rate; GNSS/wheel columns are empty when no measurement arrived.
//   gnss_stamp_ns lets readers verify timing offset = time_ns - gnss_stamp_ns.
void GnssImuKalmanFilter::writeSensorsHeader() {
  if (!sensors_csv_.is_open()) return;
  sensors_csv_ << "# coordinate_frame: enu"
               << " | output_reference_frame: " << config_.output_reference_frame << "\n";
  sensors_csv_ << std::setprecision(15)
    << "time_ns"
    << ",gnss_week,gnss_tow,gnss_stamp_ns"
    << ",gnss_lat_deg,gnss_lon_deg,gnss_alt_m"
    << ",gnss_vel_E,gnss_vel_N,gnss_vel_U"
    << ",gnss_status,gnss_num_sats"
    << ",gnss_used"
    << ",imu_acc_x,imu_acc_y,imu_acc_z"
    << ",imu_gyr_x,imu_gyr_y,imu_gyr_z"
    << ",ws_vel_x"
    << ",ws_used"
    << "\n";
  sensors_csv_.flush();
}

void GnssImuKalmanFilter::writeSensorsRow(const rclcpp::Time& stamp) {
  if (!sensors_csv_.is_open()) return;

  sensors_csv_ << std::setprecision(15);
  sensors_csv_ << stamp.nanoseconds();

  // GNSS columns (empty if no measurement this row)
  if (latest_gnss_.valid) {
    const auto& g = latest_gnss_;
    sensors_csv_ << "," << g.week
                 << "," << g.tow
                 << "," << g.stamp.nanoseconds()
                 << "," << g.lat
                 << "," << g.lon
                 << "," << g.alt
                 << "," << g.vel_enu(0)
                 << "," << g.vel_enu(1)
                 << "," << g.vel_enu(2)
                 << "," << static_cast<int>(g.status)
                 << "," << static_cast<int>(g.num_sats)
                 << "," << (g.used_for_update ? 1 : 0);
  } else {
    sensors_csv_ << ",,,,,,,,,,,,0";
  }

  // IMU (always present after initialization)
  sensors_csv_ << "," << latest_imu_acc_(0)
               << "," << latest_imu_acc_(1)
               << "," << latest_imu_acc_(2)
               << "," << latest_imu_gyr_(0)
               << "," << latest_imu_gyr_(1)
               << "," << latest_imu_gyr_(2);

  // Wheel speed (empty if no measurement this row)
  if (latest_wheel_.valid) {
    sensors_csv_ << "," << latest_wheel_.linear(0)
                 << "," << (latest_wheel_.used_for_update ? 1 : 0);
  } else {
    sensors_csv_ << ",,0";
  }

  sensors_csv_ << "\n";
  sensors_csv_.flush();
}

// CSV Output — State Log
//   EKF estimated state and diagonal covariance at IMU rate.
void GnssImuKalmanFilter::writeStateHeader() {
  if (!state_csv_.is_open()) return;
  state_csv_ << "# coordinate_frame: enu"
             << " | output_reference_frame: " << config_.output_reference_frame << "\n";
  state_csv_ << std::setprecision(15)
    << "time_ns"
    << ",ekf_pos_0,ekf_pos_1,ekf_pos_2"
    << ",ekf_lat_deg,ekf_lon_deg,ekf_alt_m"
    << ",ekf_vel_0,ekf_vel_1,ekf_vel_2"
    << ",ekf_roll_rad,ekf_pitch_rad,ekf_yaw_rad"
    << ",ekf_acc_bias_x,ekf_acc_bias_y,ekf_acc_bias_z"
    << ",ekf_gyr_bias_x,ekf_gyr_bias_y,ekf_gyr_bias_z"
    << ",ekf_cov_pos_0,ekf_cov_pos_1,ekf_cov_pos_2"
    << ",ekf_cov_vel_0,ekf_cov_vel_1,ekf_cov_vel_2"
    << ",ekf_cov_att_0,ekf_cov_att_1,ekf_cov_att_2"
    << "\n";
  state_csv_.flush();
}

void GnssImuKalmanFilter::writeStateRow(const rclcpp::Time& stamp) {
  if (!state_csv_.is_open() || !initialized_) return;

  Eigen::Vector3d pos   = x_.segment<3>(IDX_POS);
  Eigen::Vector3d vel   = x_.segment<3>(IDX_VEL);
  Eigen::Vector3d euler = quaternionToEuler(getQuaternion());
  Eigen::Vector3d ab    = x_.segment<3>(IDX_AB);
  Eigen::Vector3d gb    = x_.segment<3>(IDX_GB);
  Eigen::Vector3d llh   = enuToLlh(pos);

  state_csv_ << std::setprecision(15);
  state_csv_ << stamp.nanoseconds()
    << "," << pos(0)   << "," << pos(1)   << "," << pos(2)
    << "," << llh(0)   << "," << llh(1)   << "," << llh(2)
    << "," << vel(0)   << "," << vel(1)   << "," << vel(2)
    << "," << euler(0) << "," << euler(1) << "," << euler(2)
    << "," << ab(0)    << "," << ab(1)    << "," << ab(2)
    << "," << gb(0)    << "," << gb(1)    << "," << gb(2)
    << "," << P_(EIDX_POS+0, EIDX_POS+0)
    << "," << P_(EIDX_POS+1, EIDX_POS+1)
    << "," << P_(EIDX_POS+2, EIDX_POS+2)
    << "," << P_(EIDX_VEL+0, EIDX_VEL+0)
    << "," << P_(EIDX_VEL+1, EIDX_VEL+1)
    << "," << P_(EIDX_VEL+2, EIDX_VEL+2)
    << "," << P_(EIDX_ATT+0, EIDX_ATT+0)
    << "," << P_(EIDX_ATT+1, EIDX_ATT+1)
    << "," << P_(EIDX_ATT+2, EIDX_ATT+2)
    << "\n";
  state_csv_.flush();
}

}  // namespace gnss_imu_kalman_filter

// main — guarded so the node implementation can be linked into the gtest
// verification harness (test/test_gnss_imu_ekf.cpp), which provides its own main.
#ifndef GNSS_IMU_KF_NO_MAIN
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gnss_imu_kalman_filter::GnssImuKalmanFilter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif