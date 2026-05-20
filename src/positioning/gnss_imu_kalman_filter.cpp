// SPDX-License-Identifier: MIT
// GNSS/IMU/WheelSpeed Error-State EKF Node
#include "gnss_ros_standardization/gnss_imu_kalman_filter.hpp"
#include "gnss_ros_standardization/gnss_utils.hpp"
#include <iomanip>
#include <sstream>
#include <climits>
#include <unistd.h>

namespace grs = gnss_ros_standardization::msg;

namespace gnss_imu_kalman_filter {

// ============================================================
// Quaternion helpers
// ============================================================
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

// ============================================================
// Coordinate helpers
// ============================================================
Eigen::Vector3d GnssImuKalmanFilter::gravityVector() const {
  if (config_.coordinate_frame == "ecef") {
    // Approximate: gravity in ECEF at current position
    Eigen::Vector3d pos = x_.segment<3>(IDX_POS);
    double r = pos.norm();
    if (r < 1.0) r = 6378137.0;
    constexpr double Re = 6378137.0;
    return -(9.80665 * (Re * Re) / (r * r * r)) * pos;  // g(r) = g0*(Re/r)^2, pointing toward center
  } else {
    return Eigen::Vector3d(0.0, 0.0, -9.80665);  // ENU: up is +Z
  }
}

Eigen::Vector3d GnssImuKalmanFilter::ecefToWorkFrame(const Eigen::Vector3d& ecef) const {
  if (config_.coordinate_frame == "ecef") {
    return ecef;
  }
  // ENU
  double pos_ref[3] = {origin_llh_(0), origin_llh_(1), origin_llh_(2)};
  double dr[3] = {ecef(0) - origin_ecef_(0),
                  ecef(1) - origin_ecef_(1),
                  ecef(2) - origin_ecef_(2)};
  double enu[3];
  ecef2enu(pos_ref, dr, enu);
  return Eigen::Vector3d(enu[0], enu[1], enu[2]);
}

Eigen::Vector3d GnssImuKalmanFilter::workFrameToEcef(const Eigen::Vector3d& pos) const {
  if (config_.coordinate_frame == "ecef") return pos;
  double pos_ref[3] = {origin_llh_(0), origin_llh_(1), origin_llh_(2)};
  double enu[3] = {pos(0), pos(1), pos(2)};
  double dr[3];
  enu2ecef(pos_ref, enu, dr);
  return Eigen::Vector3d(origin_ecef_(0) + dr[0],
                         origin_ecef_(1) + dr[1],
                         origin_ecef_(2) + dr[2]);
}

Eigen::Vector3d GnssImuKalmanFilter::workFrameToLlh(const Eigen::Vector3d& pos) const {
  Eigen::Vector3d ecef = workFrameToEcef(pos);
  double e[3] = {ecef(0), ecef(1), ecef(2)};
  double llh[3];
  ecef2pos(e, llh);
  // Return lat[deg], lon[deg], alt[m]
  return Eigen::Vector3d(llh[0] * 180.0 / M_PI, llh[1] * 180.0 / M_PI, llh[2]);
}

// ============================================================
// Constructor / Destructor
// ============================================================
GnssImuKalmanFilter::GnssImuKalmanFilter() : Node("gnss_imu_kalman_filter") {
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

  RCLCPP_INFO(get_logger(), "EKF node started. frame=%s, gnss_mode=%s, wheel=%s",
    config_.coordinate_frame.c_str(),
    (config_.gnss_update_mode == GnssUpdateMode::FIX_ONLY ? "fix_only" :
     config_.gnss_update_mode == GnssUpdateMode::FIX_FLOAT ? "fix_float" : "all"),
    config_.use_wheel_speed ? "ON" : "OFF");
}

GnssImuKalmanFilter::~GnssImuKalmanFilter() {
  if (sensors_csv_.is_open()) sensors_csv_.close();
  if (state_csv_.is_open()) state_csv_.close();
}

// ============================================================
// Parameter loading
// ============================================================
void GnssImuKalmanFilter::loadParameters() {
  auto& c = config_;

  declare_parameter<std::string>("coordinate_frame", c.coordinate_frame);
  get_parameter("coordinate_frame", c.coordinate_frame);

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

  declare_parameter<double>("ekf.sigma_acc", c.sigma_acc);
  declare_parameter<double>("ekf.sigma_gyr", c.sigma_gyr);
  declare_parameter<double>("ekf.sigma_acc_bias", c.sigma_acc_bias);
  declare_parameter<double>("ekf.sigma_gyr_bias", c.sigma_gyr_bias);
  get_parameter("ekf.sigma_acc", c.sigma_acc);
  get_parameter("ekf.sigma_gyr", c.sigma_gyr);
  get_parameter("ekf.sigma_acc_bias", c.sigma_acc_bias);
  get_parameter("ekf.sigma_gyr_bias", c.sigma_gyr_bias);

  declare_parameter<double>("ekf.init_pos_std", c.init_pos_std);
  declare_parameter<double>("ekf.init_vel_std", c.init_vel_std);
  declare_parameter<double>("ekf.init_att_std", c.init_att_std);
  declare_parameter<double>("ekf.init_acc_bias_std", c.init_acc_bias_std);
  declare_parameter<double>("ekf.init_gyr_bias_std", c.init_gyr_bias_std);
  get_parameter("ekf.init_pos_std", c.init_pos_std);
  get_parameter("ekf.init_vel_std", c.init_vel_std);
  get_parameter("ekf.init_att_std", c.init_att_std);
  get_parameter("ekf.init_acc_bias_std", c.init_acc_bias_std);
  get_parameter("ekf.init_gyr_bias_std", c.init_gyr_bias_std);

  declare_parameter<std::string>("gnss_update_mode", "fix_only");
  std::string mode_str;
  get_parameter("gnss_update_mode", mode_str);
  if (mode_str == "fix_float") c.gnss_update_mode = GnssUpdateMode::FIX_FLOAT;
  else if (mode_str == "all") c.gnss_update_mode = GnssUpdateMode::ALL;
  else c.gnss_update_mode = GnssUpdateMode::FIX_ONLY;

  declare_parameter<double>("gnss_heading_speed_threshold", c.gnss_heading_speed_threshold);
  get_parameter("gnss_heading_speed_threshold", c.gnss_heading_speed_threshold);

  declare_parameter<double>("gnss_pos_gate_chi2", c.gnss_pos_gate_chi2);
  get_parameter("gnss_pos_gate_chi2", c.gnss_pos_gate_chi2);

  declare_parameter<bool>("use_wheel_speed", c.use_wheel_speed);
  declare_parameter<std::string>("wheel_speed_topic_type", c.wheel_speed_topic_type);
  declare_parameter<std::string>("wheel_speed_mode", c.wheel_speed_mode);
  declare_parameter<double>("wheel_speed_sigma", c.wheel_speed_sigma);
  get_parameter("use_wheel_speed", c.use_wheel_speed);
  get_parameter("wheel_speed_topic_type", c.wheel_speed_topic_type);
  get_parameter("wheel_speed_mode", c.wheel_speed_mode);
  get_parameter("wheel_speed_sigma", c.wheel_speed_sigma);

  declare_parameter<double>("init_imu_duration", c.init_imu_duration);
  get_parameter("init_imu_duration", c.init_imu_duration);

  declare_parameter<double>("init_yaw_deg", std::numeric_limits<double>::quiet_NaN());
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
  if (c.coordinate_frame != "ecef" && c.coordinate_frame != "enu") {
    throw std::invalid_argument(
      "coordinate_frame must be 'ecef' or 'enu', got: '" + c.coordinate_frame + "'");
  }
  if (c.output_reference_frame != "gnss" && c.output_reference_frame != "imu") {
    throw std::invalid_argument(
      "output_reference_frame must be 'gnss' or 'imu', got: '" + c.output_reference_frame + "'");
  }
}

// ============================================================
// Initialization
// ============================================================
bool GnssImuKalmanFilter::tryInitialize() {
  if (!has_initial_gnss_ || !has_initial_imu_ || !has_initial_yaw_) return false;

  x_.setZero();

  // Set position (initial gnss pos is GNSS antenna position)
  // If output reference frame is "imu", we must translate it to the IMU origin using lever arm.
  Eigen::Vector3d p_gnss;
  if (config_.coordinate_frame == "enu") {
    p_gnss = ecefToWorkFrame(init_gnss_pos_ecef_);
  } else {
    p_gnss = init_gnss_pos_ecef_;
  }

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

  // Initial covariance
  P_.setZero();
  auto sq = [](double v) { return v * v; };
  P_.diagonal().segment<3>(EIDX_POS).setConstant(sq(config_.init_pos_std));
  P_.diagonal().segment<3>(EIDX_VEL).setConstant(sq(config_.init_vel_std));
  P_.diagonal().segment<3>(EIDX_ATT).setConstant(sq(config_.init_att_std));
  P_.diagonal().segment<3>(EIDX_AB).setConstant(sq(config_.init_acc_bias_std));
  P_.diagonal().segment<3>(EIDX_GB).setConstant(sq(config_.init_gyr_bias_std));

  initialized_ = true;
  RCLCPP_INFO(get_logger(), "EKF initialized. roll=%.2f pitch=%.2f yaw=%.2f [deg]",
    roll * 180.0/M_PI, pitch * 180.0/M_PI, init_yaw_ * 180.0/M_PI);
  return true;
}

// ============================================================
// EKF Prediction (IMU-driven)
// ============================================================
void GnssImuKalmanFilter::predict(const Eigen::Vector3d& acc_body, const Eigen::Vector3d& gyr_body, double dt) {
  if (dt <= 0.0 || dt > 1.0) return;

  Eigen::Matrix3d C_bn = getRotationMatrix();
  Eigen::Vector3d g = gravityVector();

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

  // Error-state Jacobian F (15x15)
  Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> F;
  F.setIdentity();

  // Skew-symmetric of acc in nav frame
  Eigen::Vector3d an = C_bn * acc_corrected;
  Eigen::Matrix3d sk_an;
  sk_an <<     0, -an(2),  an(1),
           an(2),      0, -an(0),
          -an(1),  an(0),      0;

  F.block<3,3>(EIDX_POS, EIDX_VEL) = Eigen::Matrix3d::Identity() * dt;
  F.block<3,3>(EIDX_VEL, EIDX_ATT) = -sk_an * dt;
  F.block<3,3>(EIDX_VEL, EIDX_AB)  = -C_bn * dt;

  Eigen::Matrix3d sk_w;
  sk_w <<              0, -gyr_corrected(2),  gyr_corrected(1),
          gyr_corrected(2),               0, -gyr_corrected(0),
         -gyr_corrected(1),  gyr_corrected(0),               0;
  F.block<3,3>(EIDX_ATT, EIDX_ATT) = Eigen::Matrix3d::Identity() - sk_w * dt;
  F.block<3,3>(EIDX_ATT, EIDX_GB)  = -Eigen::Matrix3d::Identity() * dt;

  // Process noise Q
  Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> Q;
  Q.setZero();
  double dt2 = dt * dt;
  Q.diagonal().segment<3>(EIDX_POS).setConstant(config_.sigma_acc * config_.sigma_acc * dt2 * dt2 * 0.25);
  Q.diagonal().segment<3>(EIDX_VEL).setConstant(config_.sigma_acc * config_.sigma_acc * dt2);
  Q.diagonal().segment<3>(EIDX_ATT).setConstant(config_.sigma_gyr * config_.sigma_gyr * dt2);
  Q.diagonal().segment<3>(EIDX_AB).setConstant(config_.sigma_acc_bias * config_.sigma_acc_bias * dt);
  Q.diagonal().segment<3>(EIDX_GB).setConstant(config_.sigma_gyr_bias * config_.sigma_gyr_bias * dt);

  P_ = F * P_ * F.transpose() + Q;
}

// ============================================================
// GNSS Position Update
// ============================================================
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
    Eigen::Vector3d lb = config_.lever_arm;
    Eigen::Matrix3d sk_lb;
    sk_lb <<     0, -lb(2),  lb(1),
             lb(2),      0, -lb(0),
            -lb(1),  lb(0),      0;
    H.block<3,3>(0, EIDX_ATT) = -C_bn * sk_lb;
  } else {
    // EKF state is already GNSS position
    innovation = z_pos - x_.segment<3>(IDX_POS);
    H.block<3,3>(0, EIDX_POS) = Eigen::Matrix3d::Identity();
  }

  Eigen::Matrix3d S = H * P_ * H.transpose() + R_pos;

  // Mahalanobis distance gate — reject large outliers (multipath, spoofing, etc.)
  double mahal_sq = innovation.transpose() * S.inverse() * innovation;
  if (mahal_sq > config_.gnss_pos_gate_chi2) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "GNSS position outlier rejected (mahal^2=%.1f > %.1f)", mahal_sq, config_.gnss_pos_gate_chi2);
    return;
  }

  Eigen::Matrix<double, ERROR_STATE_DIM, 3> K = P_ * H.transpose() * S.inverse();

  Eigen::Matrix<double, ERROR_STATE_DIM, 1> dx = K * innovation;

  // Apply error-state correction
  x_.segment<3>(IDX_POS) += dx.segment<3>(EIDX_POS);
  x_.segment<3>(IDX_VEL) += dx.segment<3>(EIDX_VEL);
  x_.segment<3>(IDX_AB)  += dx.segment<3>(EIDX_AB);
  x_.segment<3>(IDX_GB)  += dx.segment<3>(EIDX_GB);

  // Attitude correction via small-angle rotation
  Eigen::Vector3d dtheta = dx.segment<3>(EIDX_ATT);
  double angle = dtheta.norm();
  if (angle > 1e-12) {
    Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, dtheta / angle));
    setQuaternion(getQuaternion() * dq);
  }

  // Covariance update
  auto I15 = Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Identity();
  P_ = (I15 - K * H) * P_ * (I15 - K * H).transpose() + K * R_pos * K.transpose();
}

// ============================================================
// GNSS Heading Update (from Doppler velocity)
// ============================================================
void GnssImuKalmanFilter::updateGnssHeading(double heading_rad, double heading_var) {
  Eigen::Quaterniond q_pred = getQuaternion();
  Eigen::Vector3d euler = quaternionToEuler(q_pred);
  double yaw_pred = euler(2);

  double yaw_err = heading_rad - yaw_pred;
  // Normalize to [-pi, pi]
  while (yaw_err >  M_PI) yaw_err -= 2.0 * M_PI;
  while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

  // Assume the observation is the error rotation delta_theta.
  // The error vector delta_theta in nav frame corresponds to the actual orientation:
  // q_true = delta_q * q_pred. We observe the Z-component of delta_theta.
  double innovation = yaw_err;

  Eigen::Matrix<double, 1, ERROR_STATE_DIM> H;
  H.setZero();
  H(0, EIDX_ATT + 2) = 1.0;  // We observe the yaw component (Z-axis in delta_theta)

  double S = (H * P_ * H.transpose())(0,0) + heading_var;
  Eigen::Matrix<double, ERROR_STATE_DIM, 1> K = P_ * H.transpose() / S;

  Eigen::Matrix<double, ERROR_STATE_DIM, 1> dx = K * innovation;

  x_.segment<3>(IDX_POS) += dx.segment<3>(EIDX_POS);
  x_.segment<3>(IDX_VEL) += dx.segment<3>(EIDX_VEL);
  x_.segment<3>(IDX_AB)  += dx.segment<3>(EIDX_AB);
  x_.segment<3>(IDX_GB)  += dx.segment<3>(EIDX_GB);

  Eigen::Vector3d dtheta = dx.segment<3>(EIDX_ATT);
  double angle = dtheta.norm();
  if (angle > 1e-12) {
    Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, dtheta / angle));
    setQuaternion(getQuaternion() * dq);
  }

  auto I15 = Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Identity();
  Eigen::Matrix<double, 1, 1> R_scalar;
  R_scalar(0,0) = heading_var;
  P_ = (I15 - K * H) * P_ * (I15 - K * H).transpose() + K * R_scalar * K.transpose();
}

// ============================================================
// Wheel Speed Update
// ============================================================
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
  
  {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_wheel_.valid = true;
    latest_wheel_.stamp = msg->header.stamp;
    latest_wheel_.linear = linear;
    for (int i = 0; i < 36; ++i) latest_wheel_.covariance[i] = msg->twist.covariance[i];
  }

  processWheelSpeed(linear, R_vel);
}

void GnssImuKalmanFilter::onWheelSpeedPoint(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  Eigen::Vector3d linear(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
  Eigen::Matrix3d R_vel = Eigen::Matrix3d::Identity() * config_.wheel_speed_sigma * config_.wheel_speed_sigma;
  
  {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_wheel_.valid = true;
    latest_wheel_.stamp = msg->header.stamp;
    latest_wheel_.linear = linear;
    latest_wheel_.covariance.fill(0);
  }

  processWheelSpeed(linear, R_vel);
}

void GnssImuKalmanFilter::processWheelSpeed(const Eigen::Vector3d& linear_velocity, const Eigen::Matrix3d& covariance) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (!initialized_) return;

  Eigen::Matrix3d C_bn = getRotationMatrix();
  Eigen::Vector3d v_nav_pred = x_.segment<3>(IDX_VEL);
  
  // Transform predicted nav velocity into body frame to compare directly with linear_velocity
  Eigen::Vector3d v_body_pred = C_bn.transpose() * v_nav_pred;
  
  if (config_.wheel_speed_mode == "longitudinal_only") {
    // Only constrain longitudinal (X) velocity
    double innovation = linear_velocity(0) - v_body_pred(0);
    
    Eigen::Matrix<double, 1, ERROR_STATE_DIM> H;
    H.setZero();
    // Jacobian of v_body_x w.r.t v_nav: First row of C_bn^T
    H.block<1,3>(0, EIDX_VEL) = C_bn.transpose().row(0);
    // Jacobian of v_body w.r.t attitude error theta: v_body = C_bn^T * v_nav
    // dv_body / dtheta = C_bn^T * [v_nav]_x
    Eigen::Matrix3d sk_v_nav;
    sk_v_nav <<           0, -v_nav_pred(2),  v_nav_pred(1),
                 v_nav_pred(2),           0, -v_nav_pred(0),
                -v_nav_pred(1),  v_nav_pred(0),           0;
    H.block<1,3>(0, EIDX_ATT) = (C_bn.transpose() * sk_v_nav).row(0);
    
    double S = (H * P_ * H.transpose())(0,0) + covariance(0,0);
    Eigen::Matrix<double, ERROR_STATE_DIM, 1> K = P_ * H.transpose() / S;
    Eigen::Matrix<double, ERROR_STATE_DIM, 1> dx = K * innovation;

    x_.segment<3>(IDX_POS) += dx.segment<3>(EIDX_POS);
    x_.segment<3>(IDX_VEL) += dx.segment<3>(EIDX_VEL);
    x_.segment<3>(IDX_AB)  += dx.segment<3>(EIDX_AB);
    x_.segment<3>(IDX_GB)  += dx.segment<3>(EIDX_GB);

    Eigen::Vector3d dtheta = dx.segment<3>(EIDX_ATT);
    double angle = dtheta.norm();
    if (angle > 1e-12) {
      Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, dtheta / angle));
      setQuaternion(getQuaternion() * dq);
    }

    auto I15 = Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Identity();
    Eigen::Matrix<double, 1, 1> R_scalar;
    R_scalar(0,0) = covariance(0,0);
    P_ = (I15 - K * H) * P_ * (I15 - K * H).transpose() + K * R_scalar * K.transpose();

  } else {
    // 3D velocity observation (e.g. [v_body_x, 0, 0])
    Eigen::Vector3d innovation = linear_velocity - v_body_pred;
    
    Eigen::Matrix<double, 3, ERROR_STATE_DIM> H;
    H.setZero();
    H.block<3,3>(0, EIDX_VEL) = C_bn.transpose();
    
    Eigen::Matrix3d sk_v_nav;
    sk_v_nav <<           0, -v_nav_pred(2),  v_nav_pred(1),
                 v_nav_pred(2),           0, -v_nav_pred(0),
                -v_nav_pred(1),  v_nav_pred(0),           0;
    H.block<3,3>(0, EIDX_ATT) = C_bn.transpose() * sk_v_nav;
    
    Eigen::Matrix3d S = H * P_ * H.transpose() + covariance;
    Eigen::Matrix<double, ERROR_STATE_DIM, 3> K = P_ * H.transpose() * S.inverse();
    Eigen::Matrix<double, ERROR_STATE_DIM, 1> dx = K * innovation;

    x_.segment<3>(IDX_POS) += dx.segment<3>(EIDX_POS);
    x_.segment<3>(IDX_VEL) += dx.segment<3>(EIDX_VEL);
    x_.segment<3>(IDX_AB)  += dx.segment<3>(EIDX_AB);
    x_.segment<3>(IDX_GB)  += dx.segment<3>(EIDX_GB);

    Eigen::Vector3d dtheta = dx.segment<3>(EIDX_ATT);
    double angle = dtheta.norm();
    if (angle > 1e-12) {
      Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, dtheta / angle));
      setQuaternion(getQuaternion() * dq);
    }

    auto I15 = Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Identity();
    P_ = (I15 - K * H) * P_ * (I15 - K * H).transpose() + K * covariance * K.transpose();
  }

  latest_wheel_.used_for_update = true;
}

// ============================================================
// IMU Callback
// ============================================================
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

  if (!initialized_) return;

  rclcpp::Time stamp = msg->header.stamp;
  if (has_prev_imu_stamp_) {
    double dt = (stamp - prev_imu_stamp_).seconds();
    predict(acc_body, gyr_body, dt);
  }
  prev_imu_stamp_ = stamp;
  has_prev_imu_stamp_ = true;

  // Write CSV rows and publish at IMU rate
  writeSensorsRow(stamp);
  writeStateRow(stamp);
  publishSolution(stamp);

  // After writing CSV, clear GNSS and wheel snapshots
  latest_gnss_.valid = false;
  latest_gnss_.used_for_update = false;
  latest_wheel_.valid = false;
  latest_wheel_.used_for_update = false;
}

// ============================================================
// GNSS Callback
// ============================================================
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

  // Doppler heading from ENU velocity
  double ve = snap.vel_enu(0), vn = snap.vel_enu(1);
  double horiz_speed = std::sqrt(ve*ve + vn*vn);
  if (horiz_speed > config_.gnss_heading_speed_threshold) {
    snap.doppler_heading = std::atan2(vn, ve);  // ENU yaw: from East, CCW (matches EKF internal convention)
  }

  latest_gnss_ = snap;

  // --- Initialization ---
  if (!has_initial_gnss_ && !pos_nan && msg->status != grs::GnssSolution::STATUS_NONE) {
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

    // Use configured initial yaw if provided, otherwise try Doppler heading
    if (!has_initial_yaw_ && !std::isnan(config_.init_yaw_deg)) {
      init_yaw_ = config_.init_yaw_deg * M_PI / 180.0;
      has_initial_yaw_ = true;
      RCLCPP_INFO(get_logger(), "Initial yaw from config init_yaw_deg: %.2f [deg]", config_.init_yaw_deg);
    } else if (!has_initial_yaw_ && !std::isnan(snap.doppler_heading)) {
      init_yaw_ = snap.doppler_heading;
      has_initial_yaw_ = true;
      RCLCPP_INFO(get_logger(), "Initial yaw from GNSS Doppler: %.2f [deg]",
                  init_yaw_ * 180.0/M_PI);
    }

    tryInitialize();
    return;
  }

  // Try initial yaw if not yet set
  if (has_initial_gnss_ && !has_initial_yaw_ && !std::isnan(snap.doppler_heading)) {
    init_yaw_ = snap.doppler_heading;
    has_initial_yaw_ = true;
    RCLCPP_INFO(get_logger(), "Initial yaw from GNSS Doppler: %.2f [deg]",
                init_yaw_ * 180.0/M_PI);
    tryInitialize();
    return;
  }

  if (!initialized_) return;

  // --- Check if this solution is usable for update ---
  bool usable = !pos_nan && (msg->status != grs::GnssSolution::STATUS_NONE);
  if (usable) {
    switch (config_.gnss_update_mode) {
      case GnssUpdateMode::FIX_ONLY:
        usable = (msg->status == grs::GnssSolution::STATUS_FIX);
        break;
      case GnssUpdateMode::FIX_FLOAT:
        usable = (msg->status == grs::GnssSolution::STATUS_FIX ||
                  msg->status == grs::GnssSolution::STATUS_FLOAT);
        break;
      case GnssUpdateMode::ALL:
        break;  // already usable
    }
  }

  if (usable) {
    // Position observation update
    Eigen::Vector3d z_pos;
    Eigen::Matrix3d R_pos;
    if (config_.coordinate_frame == "enu") {
      z_pos = ecefToWorkFrame(snap.pos_ecef);
      Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>> cov_ecef(snap.pos_cov_ecef.data());
      double lat_r = origin_llh_(0), lon_r = origin_llh_(1);
      double cov_enu[9];
      gnss_utils::rotateCovariance(snap.pos_cov_ecef.data(), lat_r, lon_r, cov_enu);
      R_pos = Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor>>(cov_enu);
    } else {
      z_pos = snap.pos_ecef;
      R_pos = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(snap.pos_cov_ecef.data());
    }
    // Ensure R_pos is positive definite (add small diagonal if needed)
    R_pos += Eigen::Matrix3d::Identity() * 1e-6;
    updateGnssPosition(z_pos, R_pos);

    // Doppler heading update
    if (!std::isnan(snap.doppler_heading) && horiz_speed > config_.gnss_heading_speed_threshold) {
      double heading_var = 0.1;  // ~18 deg std (conservative)
      if (horiz_speed > 2.0) heading_var = 0.01;  // ~6 deg at decent speed
      // Derive heading variance from velocity covariance (error propagation of atan2(vn,ve))
      double var_ve = snap.vel_cov_enu[0];
      double var_vn = snap.vel_cov_enu[4];
      double s2 = horiz_speed * horiz_speed;
      heading_var = (var_ve * vn*vn + var_vn * ve*ve) / (s2 * s2);
      heading_var = std::max(heading_var, 1e-4);
      updateGnssHeading(snap.doppler_heading, heading_var);
    }
    latest_gnss_.used_for_update = true;
  }
}



// ============================================================
// Publish Solution
// ============================================================
void GnssImuKalmanFilter::publishSolution(const rclcpp::Time& stamp) {
  auto sol = grs::GnssSolution();
  sol.header.stamp = stamp;
  sol.header.frame_id = "gnss_imu_kalman_filter";

  Eigen::Vector3d pos = x_.segment<3>(IDX_POS);
  Eigen::Vector3d vel = x_.segment<3>(IDX_VEL);
  Eigen::Vector3d llh = workFrameToLlh(pos);
  Eigen::Vector3d ecef_pos = workFrameToEcef(pos);

  sol.latitude   = llh(0);
  sol.longitude  = llh(1);
  sol.altitude   = llh(2);

  sol.pos_ecef.x = ecef_pos(0);
  sol.pos_ecef.y = ecef_pos(1);
  sol.pos_ecef.z = ecef_pos(2);

  // Covariance: write to the frame-appropriate field; rotate ENU→ECEF for pos_cov_ecef
  Eigen::Matrix3d P_pp = P_.block<3,3>(EIDX_POS, EIDX_POS);
  Eigen::Matrix3d P_vv = P_.block<3,3>(EIDX_VEL, EIDX_VEL);
  if (config_.coordinate_frame == "enu" && origin_set_) {
    // Store ENU covariance in the ENU field
    for (int i = 0; i < 9; ++i) sol.pos_enu_cov[i] = P_pp(i/3, i%3);
    // Rotate ENU→ECEF for the ECEF covariance field
    double cov_ecef[9];
    gnss_utils::rotateCovarianceEnuToEcef(sol.pos_enu_cov.data(), origin_llh_(0), origin_llh_(1), cov_ecef);
    for (int i = 0; i < 9; ++i) sol.pos_cov_ecef[i] = cov_ecef[i];
    for (int i = 0; i < 9; ++i) sol.vel_enu_cov[i] = P_vv(i/3, i%3);
    gnss_utils::rotateCovarianceEnuToEcef(sol.vel_enu_cov.data(), origin_llh_(0), origin_llh_(1), cov_ecef);
    for (int i = 0; i < 9; ++i) sol.vel_cov_ecef[i] = cov_ecef[i];
  } else {
    // ECEF frame: covariance is already in ECEF
    for (int i = 0; i < 9; ++i) sol.pos_cov_ecef[i] = P_pp(i/3, i%3);
    for (int i = 0; i < 9; ++i) sol.vel_cov_ecef[i] = P_vv(i/3, i%3);
  }

  if (origin_set_) {
    sol.pos_enu_org_ecef.x = origin_ecef_(0);
    sol.pos_enu_org_ecef.y = origin_ecef_(1);
    sol.pos_enu_org_ecef.z = origin_ecef_(2);

    Eigen::Vector3d enu = ecefToWorkFrame(ecef_pos);
    sol.pos_enu.x = enu(0);
    sol.pos_enu.y = enu(1);
    sol.pos_enu.z = enu(2);
  }

  // Velocity
  if (config_.coordinate_frame == "enu") {
    sol.vel_enu.x = vel(0);
    sol.vel_enu.y = vel(1);
    sol.vel_enu.z = vel(2);
  }

  sol.status = grs::GnssSolution::STATUS_EKF;

  solution_pub_->publish(sol);

  // Publish Odometry message
  auto odom = nav_msgs::msg::Odometry();
  odom.header.stamp = stamp;
  odom.header.frame_id = config_.coordinate_frame == "enu" ? "odom" : "earth";
  odom.child_frame_id = config_.output_reference_frame == "imu" ? "imu_link" : "gnss_antenna";

  // Pos
  if (config_.coordinate_frame == "enu" && origin_set_) {
    Eigen::Vector3d enu = ecefToWorkFrame(ecef_pos);
    odom.pose.pose.position.x = enu(0);
    odom.pose.pose.position.y = enu(1);
    odom.pose.pose.position.z = enu(2);
  } else {
    odom.pose.pose.position.x = ecef_pos(0);
    odom.pose.pose.position.y = ecef_pos(1);
    odom.pose.pose.position.z = ecef_pos(2);
  }

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

// ============================================================
// CSV Output — Sensors Log
//   Rows at IMU rate; GNSS/wheel columns are empty when no measurement arrived.
//   gnss_stamp_ns lets readers verify timing offset = time_ns - gnss_stamp_ns.
// ============================================================
void GnssImuKalmanFilter::writeSensorsHeader() {
  if (!sensors_csv_.is_open()) return;
  sensors_csv_ << "# coordinate_frame: " << config_.coordinate_frame
               << " | output_reference_frame: " << config_.output_reference_frame << "\n";
  sensors_csv_ << std::setprecision(15)
    << "time_ns"
    << ",gnss_week,gnss_tow,gnss_stamp_ns"
    << ",gnss_lat_deg,gnss_lon_deg,gnss_alt_m"
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
  if (!sensors_csv_.is_open() || !initialized_) return;

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
                 << "," << static_cast<int>(g.status)
                 << "," << static_cast<int>(g.num_sats)
                 << "," << (g.used_for_update ? 1 : 0);
  } else {
    sensors_csv_ << ",,,,,,,,,0";
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

// ============================================================
// CSV Output — State Log
//   EKF estimated state and diagonal covariance at IMU rate.
// ============================================================
void GnssImuKalmanFilter::writeStateHeader() {
  if (!state_csv_.is_open()) return;
  state_csv_ << "# coordinate_frame: " << config_.coordinate_frame
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
  Eigen::Vector3d llh   = workFrameToLlh(pos);

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

// ============================================================
// main
// ============================================================
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gnss_imu_kalman_filter::GnssImuKalmanFilter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}