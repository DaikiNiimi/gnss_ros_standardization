// GNSS/IMU/WheelSpeed Error-State EKF Node
#include "gnss_ros_standardization/ekf_node.hpp"
#include "gnss_ros_standardization/gnss_utils.hpp"
#include <iomanip>
#include <sstream>

namespace grs = gnss_ros_standardization::msg;

namespace ekf {

// ============================================================
// Quaternion helpers
// ============================================================
Eigen::Quaterniond EkfNode::getQuaternion() const {
  return Eigen::Quaterniond(x_(IDX_QUAT), x_(IDX_QUAT+1),
                            x_(IDX_QUAT+2), x_(IDX_QUAT+3));
}

void EkfNode::setQuaternion(const Eigen::Quaterniond& q) {
  x_(IDX_QUAT)   = q.w();
  x_(IDX_QUAT+1) = q.x();
  x_(IDX_QUAT+2) = q.y();
  x_(IDX_QUAT+3) = q.z();
}

Eigen::Vector3d EkfNode::quaternionToEuler(const Eigen::Quaterniond& q) {
  // Returns (roll, pitch, yaw) in radians
  auto m = q.normalized().toRotationMatrix();
  double roll  = std::atan2(m(2,1), m(2,2));
  double pitch = -std::asin(std::clamp(m(2,0), -1.0, 1.0));
  double yaw   = std::atan2(m(1,0), m(0,0));
  return {roll, pitch, yaw};
}

Eigen::Quaterniond EkfNode::eulerToQuaternion(double roll, double pitch, double yaw) {
  return Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX());
}

Eigen::Matrix3d EkfNode::getRotationMatrix() const {
  return getQuaternion().normalized().toRotationMatrix();
}

Eigen::Vector3d EkfNode::applyImuAxisRotation(const Eigen::Vector3d& raw) const {
  return R_imu_body_ * raw;
}

Eigen::Vector3d EkfNode::computeLeverArmCorrection() const {
  Eigen::Matrix3d C_bn = getRotationMatrix();
  return C_bn * config_.lever_arm;
}

// ============================================================
// Coordinate helpers
// ============================================================
Eigen::Vector3d EkfNode::gravityVector() const {
  if (config_.coordinate_frame == "ecef") {
    // Approximate: gravity in ECEF at current position
    Eigen::Vector3d pos = x_.segment<3>(IDX_POS);
    double r = pos.norm();
    if (r < 1.0) r = 6378137.0;
    return -(9.80665 / r) * pos;  // pointing toward center
  } else {
    return Eigen::Vector3d(0.0, 0.0, -9.80665);  // ENU: up is +Z
  }
}

Eigen::Vector3d EkfNode::ecefToWorkFrame(const Eigen::Vector3d& ecef) const {
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

Eigen::Vector3d EkfNode::workFrameToEcef(const Eigen::Vector3d& pos) const {
  if (config_.coordinate_frame == "ecef") return pos;
  double pos_ref[3] = {origin_llh_(0), origin_llh_(1), origin_llh_(2)};
  double enu[3] = {pos(0), pos(1), pos(2)};
  double dr[3];
  enu2ecef(pos_ref, enu, dr);
  return Eigen::Vector3d(origin_ecef_(0) + dr[0],
                         origin_ecef_(1) + dr[1],
                         origin_ecef_(2) + dr[2]);
}

Eigen::Vector3d EkfNode::workFrameToLlh(const Eigen::Vector3d& pos) const {
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
EkfNode::EkfNode() : Node("ekf_node") {
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
    std::bind(&EkfNode::onImu, this, std::placeholders::_1));

  gnss_sub_ = create_subscription<grs::GnssSolution>(
    config_.topic_gnss_solution, rclcpp::QoS(10),
    std::bind(&EkfNode::onGnss, this, std::placeholders::_1));

  if (config_.use_wheel_speed) {
    if (config_.wheel_speed_topic_type == "twist") {
      wheel_sub_raw_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        config_.topic_wheel_speed, rclcpp::QoS(10),
        std::bind(&EkfNode::onWheelSpeedPoint, this, std::placeholders::_1));
    } else {
      wheel_sub_cov_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        config_.topic_wheel_speed, rclcpp::QoS(10),
        std::bind(&EkfNode::onWheelSpeedWithCov, this, std::placeholders::_1));
    }
  }

  // Publisher
  solution_pub_ = create_publisher<grs::GnssSolution>(config_.topic_ekf_solution, rclcpp::QoS(10));
  odom_pub_     = create_publisher<nav_msgs::msg::Odometry>(config_.topic_ekf_solution + "_odom", rclcpp::QoS(10));

  // CSV
  csv_file_.open(config_.csv_output_path, std::ios::out | std::ios::trunc);
  if (csv_file_.is_open()) {
    writeCSVHeader();
    RCLCPP_INFO(get_logger(), "CSV output: %s", config_.csv_output_path.c_str());
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to open CSV: %s", config_.csv_output_path.c_str());
  }

  RCLCPP_INFO(get_logger(), "EKF node started. frame=%s, gnss_mode=%s, wheel=%s",
    config_.coordinate_frame.c_str(),
    (config_.gnss_update_mode == GnssUpdateMode::FIX_ONLY ? "fix_only" :
     config_.gnss_update_mode == GnssUpdateMode::FIX_FLOAT ? "fix_float" : "all"),
    config_.use_wheel_speed ? "ON" : "OFF");
}

EkfNode::~EkfNode() {
  if (csv_file_.is_open()) csv_file_.close();
}

// ============================================================
// Parameter loading
// ============================================================
void EkfNode::loadParameters() {
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
  declare_parameter<std::string>("topics.ekf_solution", c.topic_ekf_solution);
  get_parameter("topics.gnss_solution", c.topic_gnss_solution);
  get_parameter("topics.imu_raw", c.topic_imu_raw);
  get_parameter("topics.wheel_speed", c.topic_wheel_speed);
  get_parameter("topics.ekf_solution", c.topic_ekf_solution);

  declare_parameter<std::string>("csv.output_path", c.csv_output_path);
  get_parameter("csv.output_path", c.csv_output_path);

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
  
  declare_parameter<std::string>("output_reference_frame", c.output_reference_frame);
  get_parameter("output_reference_frame", c.output_reference_frame);

  declare_parameter<std::vector<double>>("lever_arm", {0.0, 0.0, 0.0});
  auto la = get_parameter("lever_arm").as_double_array();
  if (la.size() >= 3) c.lever_arm = Eigen::Vector3d(la[0], la[1], la[2]);

  declare_parameter<std::vector<double>>("imu_orientation", {0.0, 0.0, 0.0});
  auto io = get_parameter("imu_orientation").as_double_array();
  if (io.size() >= 3) c.imu_orientation_rpy = Eigen::Vector3d(io[0], io[1], io[2]);
}

// ============================================================
// Initialization
// ============================================================
bool EkfNode::tryInitialize() {
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
void EkfNode::predict(const Eigen::Vector3d& acc_body, const Eigen::Vector3d& gyr_body, double dt) {
  if (dt <= 0.0 || dt > 1.0) return;

  Eigen::Matrix3d C_bn = getRotationMatrix();
  Eigen::Vector3d g = gravityVector();

  // Corrected measurements
  Eigen::Vector3d ab = x_.segment<3>(IDX_AB);
  Eigen::Vector3d gb = x_.segment<3>(IDX_GB);
  Eigen::Vector3d acc_corrected = acc_body - ab;
  Eigen::Vector3d gyr_corrected = gyr_body - gb;

  // State propagation
  Eigen::Vector3d acc_nav = C_bn * acc_corrected + g;
  
  if (config_.output_reference_frame == "gnss") {
    // When outputting GNSS antenna position, the IMU acceleration is at the IMU origin.
    // Strictly, a_gnss = a_imu + alpha x lever + w x (w x lever).
    // For simplicity, we assume small angular rates/accelerations and approximate a_gnss ~= a_imu
    // as the primary translation acceleration.
    x_.segment<3>(IDX_POS) += x_.segment<3>(IDX_VEL) * dt + 0.5 * acc_nav * dt * dt;
    x_.segment<3>(IDX_VEL) += acc_nav * dt;
  } else {
    x_.segment<3>(IDX_POS) += x_.segment<3>(IDX_VEL) * dt + 0.5 * acc_nav * dt * dt;
    x_.segment<3>(IDX_VEL) += acc_nav * dt;
  }

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
void EkfNode::updateGnssPosition(const Eigen::Vector3d& z_pos,
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
    setQuaternion(dq * getQuaternion());
  }

  // Covariance update
  auto I15 = Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Identity();
  P_ = (I15 - K * H) * P_ * (I15 - K * H).transpose() + K * R_pos * K.transpose();
}

// ============================================================
// GNSS Heading Update (from Doppler velocity)
// ============================================================
void EkfNode::updateGnssHeading(double heading_rad, double heading_var) {
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
    setQuaternion(dq * getQuaternion());
  }

  auto I15 = Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Identity();
  Eigen::Matrix<double, 1, 1> R_scalar;
  R_scalar(0,0) = heading_var;
  P_ = (I15 - K * H) * P_ * (I15 - K * H).transpose() + K * R_scalar * K.transpose();
}

// ============================================================
// Wheel Speed Update
// ============================================================
void EkfNode::onWheelSpeedWithCov(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
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
    latest_wheel_.linear = linear;
    for (int i = 0; i < 36; ++i) latest_wheel_.covariance[i] = msg->twist.covariance[i];
  }
  
  processWheelSpeed(linear, R_vel);
}

void EkfNode::onWheelSpeedPoint(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  Eigen::Vector3d linear(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
  Eigen::Matrix3d R_vel = Eigen::Matrix3d::Identity() * config_.wheel_speed_sigma * config_.wheel_speed_sigma;
  
  {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_wheel_.valid = true;
    latest_wheel_.linear = linear;
    latest_wheel_.covariance.fill(0);
  }
  
  processWheelSpeed(linear, R_vel);
}

void EkfNode::processWheelSpeed(const Eigen::Vector3d& linear_velocity, const Eigen::Matrix3d& covariance) {
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
      setQuaternion(dq * getQuaternion());
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
      setQuaternion(dq * getQuaternion());
    }

    auto I15 = Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>::Identity();
    P_ = (I15 - K * H) * P_ * (I15 - K * H).transpose() + K * covariance * K.transpose();
  }

  latest_wheel_.used_for_update = true;
}

// ============================================================
// IMU Callback
// ============================================================
void EkfNode::onImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
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

  // Write CSV row and publish at IMU rate
  writeCSVRow(stamp);
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
void EkfNode::onGnss(const grs::GnssSolution::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mtx_);

  // Build snapshot regardless of update validity
  GnssSnapshot snap;
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
    snap.doppler_heading = std::atan2(ve, vn);  // heading from north, CW
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

    // Try to get yaw from Doppler
    if (!has_initial_yaw_ && !std::isnan(snap.doppler_heading)) {
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
      double horiz_speed_local = std::sqrt(ve*ve + vn*vn);
      // Derive heading variance from velocity covariance
      if (horiz_speed_local > config_.gnss_heading_speed_threshold) {
        double var_ve = snap.vel_cov_enu[0];
        double var_vn = snap.vel_cov_enu[4];
        double s2 = horiz_speed_local * horiz_speed_local;
        heading_var = (var_ve * vn*vn + var_vn * ve*ve) / (s2 * s2);
        heading_var = std::max(heading_var, 1e-4);
      }
      updateGnssHeading(snap.doppler_heading, heading_var);
    }
    latest_gnss_.used_for_update = true;
  }
}



// ============================================================
// Publish Solution
// ============================================================
void EkfNode::publishSolution(const rclcpp::Time& stamp) {
  auto sol = grs::GnssSolution();
  sol.header.stamp = stamp;
  sol.header.frame_id = "ekf";

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

  // Covariance
  for (int i = 0; i < 9; ++i) {
    sol.pos_cov_ecef[i] = P_(EIDX_POS + i/3, EIDX_POS + i%3);
    sol.vel_cov_ecef[i] = P_(EIDX_VEL + i/3, EIDX_VEL + i%3);
  }

  if (origin_set_) {
    sol.org_ecef.x = origin_ecef_(0);
    sol.org_ecef.y = origin_ecef_(1);
    sol.org_ecef.z = origin_ecef_(2);

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

  sol.status = grs::GnssSolution::STATUS_NONE;  // EKF output

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

  // Vel
  odom.twist.twist.linear.x = vel(0);
  odom.twist.twist.linear.y = vel(1);
  odom.twist.twist.linear.z = vel(2);
  
  // Covariance arrays: Odometry uses 36-element arrays for 6D pose/twist
  for (int min_i = 0; min_i < 3; ++min_i) {
    for (int min_j = 0; min_j < 3; ++min_j) {
      odom.pose.covariance[min_i * 6 + min_j] = P_(EIDX_POS + min_i, EIDX_POS + min_j);        // pos-pos
      odom.pose.covariance[(min_i + 3) * 6 + (min_j + 3)] = P_(EIDX_ATT + min_i, EIDX_ATT + min_j); // att-att
      odom.pose.covariance[min_i * 6 + (min_j + 3)] = P_(EIDX_POS + min_i, EIDX_ATT + min_j);       // pos-att
      odom.pose.covariance[(min_i + 3) * 6 + min_j] = P_(EIDX_ATT + min_i, EIDX_POS + min_j);       // att-pos
      
      odom.twist.covariance[min_i * 6 + min_j] = P_(EIDX_VEL + min_i, EIDX_VEL + min_j);       // vel x/y/z
    }
  }

  odom_pub_->publish(odom);
}

// ============================================================
// CSV Output
// ============================================================
void EkfNode::writeCSVHeader() {
  if (!csv_file_.is_open()) return;
  // Write Metadata header line
  csv_file_ << "# config_coordinate_frame: " << config_.coordinate_frame 
            << " | output_reference_frame: " << config_.output_reference_frame << "\n";
  csv_file_ << std::setprecision(15);
  csv_file_
    // Time
    << "ros_time_sec,ros_time_nsec,gnss_tow,gnss_week,ekf_frame"
    // EKF estimated position (work frame)
    << ",ekf_pos_0,ekf_pos_1,ekf_pos_2"
    // EKF estimated position (LLH)
    << ",ekf_lat_deg,ekf_lon_deg,ekf_alt_m"
    // EKF estimated position (ECEF)
    << ",ekf_ecef_x,ekf_ecef_y,ekf_ecef_z"
    // EKF estimated velocity
    << ",ekf_vel_0,ekf_vel_1,ekf_vel_2"
    // EKF estimated attitude (Euler)
    << ",ekf_roll_rad,ekf_pitch_rad,ekf_yaw_rad"
    // EKF estimated IMU biases
    << ",ekf_acc_bias_x,ekf_acc_bias_y,ekf_acc_bias_z"
    << ",ekf_gyr_bias_x,ekf_gyr_bias_y,ekf_gyr_bias_z"
    // EKF covariance diagonal
    << ",ekf_cov_pos_0,ekf_cov_pos_1,ekf_cov_pos_2"
    << ",ekf_cov_vel_0,ekf_cov_vel_1,ekf_cov_vel_2"
    << ",ekf_cov_att_0,ekf_cov_att_1,ekf_cov_att_2"
    << ",ekf_cov_ab_0,ekf_cov_ab_1,ekf_cov_ab_2"
    << ",ekf_cov_gb_0,ekf_cov_gb_1,ekf_cov_gb_2"
    // IMU
    << ",imu_acc_x,imu_acc_y,imu_acc_z"
    << ",imu_gyr_x,imu_gyr_y,imu_gyr_z"
    << ",imu_acc_cov_0,imu_acc_cov_4,imu_acc_cov_8"
    << ",imu_gyr_cov_0,imu_gyr_cov_4,imu_gyr_cov_8"
    // Wheel speed
    << ",ws_vel_x,ws_vel_y,ws_vel_z"
    << ",ws_cov_0,ws_cov_7,ws_cov_14"
    << ",ws_valid,ws_used"
    // GNSS position (LLH)
    << ",gnss_lat_deg,gnss_lon_deg,gnss_alt_m"
    // GNSS position (ECEF)
    << ",gnss_ecef_x,gnss_ecef_y,gnss_ecef_z"
    << ",gnss_pos_cov_ecef_0,gnss_pos_cov_ecef_4,gnss_pos_cov_ecef_8"
    // GNSS position (ENU)
    << ",gnss_enu_e,gnss_enu_n,gnss_enu_u"
    << ",gnss_pos_cov_enu_0,gnss_pos_cov_enu_4,gnss_pos_cov_enu_8"
    // GNSS velocity (ECEF)
    << ",gnss_vel_ecef_x,gnss_vel_ecef_y,gnss_vel_ecef_z"
    << ",gnss_vel_cov_ecef_0,gnss_vel_cov_ecef_4,gnss_vel_cov_ecef_8"
    // GNSS velocity (ENU) / Doppler
    << ",gnss_vel_enu_e,gnss_vel_enu_n,gnss_vel_enu_u"
    << ",gnss_vel_cov_enu_0,gnss_vel_cov_enu_4,gnss_vel_cov_enu_8"
    << ",gnss_doppler_heading_rad"
    // GNSS status
    << ",gnss_status,gnss_num_sats"
    << ",gnss_gdop,gnss_pdop,gnss_hdop,gnss_vdop"
    << ",gnss_ratio,gnss_age_diff"
    << ",gnss_pos_valid,gnss_received,gnss_used"
    << "\n";
  csv_file_.flush();
}

void EkfNode::writeCSVRow(const rclcpp::Time& stamp) {
  if (!csv_file_.is_open() || !initialized_) return;

  Eigen::Vector3d pos = x_.segment<3>(IDX_POS);
  Eigen::Vector3d vel = x_.segment<3>(IDX_VEL);
  Eigen::Vector3d euler = quaternionToEuler(getQuaternion());
  Eigen::Vector3d ab = x_.segment<3>(IDX_AB);
  Eigen::Vector3d gb = x_.segment<3>(IDX_GB);
  Eigen::Vector3d llh = workFrameToLlh(pos);
  Eigen::Vector3d ecef = workFrameToEcef(pos);

  auto nan_or = [](double v) -> std::string {
    if (std::isnan(v)) return "NaN";
    std::ostringstream oss; oss << std::setprecision(15) << v; return oss.str();
  };
  auto nan3 = [&](const Eigen::Vector3d& v) -> std::string {
    return nan_or(v(0)) + "," + nan_or(v(1)) + "," + nan_or(v(2));
  };

  csv_file_ << std::setprecision(15);

  // Time + Frame
  csv_file_ << stamp.seconds() << "," << stamp.nanoseconds();
  // GNSS TOW/week — only if GNSS valid this row
  if (latest_gnss_.valid) {
    csv_file_ << "," << nan_or(latest_gnss_.tow) << "," << latest_gnss_.week;
  } else {
    csv_file_ << ",NaN,NaN";
  }
  csv_file_ << "," << config_.coordinate_frame;

  // EKF position (work frame)
  csv_file_ << "," << pos(0) << "," << pos(1) << "," << pos(2);
  // EKF LLH
  csv_file_ << "," << llh(0) << "," << llh(1) << "," << llh(2);
  // EKF ECEF
  csv_file_ << "," << ecef(0) << "," << ecef(1) << "," << ecef(2);
  // EKF velocity
  csv_file_ << "," << vel(0) << "," << vel(1) << "," << vel(2);
  // EKF attitude
  csv_file_ << "," << euler(0) << "," << euler(1) << "," << euler(2);
  // EKF biases
  csv_file_ << "," << ab(0) << "," << ab(1) << "," << ab(2);
  csv_file_ << "," << gb(0) << "," << gb(1) << "," << gb(2);
  // EKF covariance diagonal
  for (int i = 0; i < ERROR_STATE_DIM; ++i)
    csv_file_ << "," << P_(i,i);

  // IMU
  csv_file_ << "," << latest_imu_acc_(0) << "," << latest_imu_acc_(1) << "," << latest_imu_acc_(2);
  csv_file_ << "," << latest_imu_gyr_(0) << "," << latest_imu_gyr_(1) << "," << latest_imu_gyr_(2);
  csv_file_ << "," << latest_imu_acc_cov_[0] << "," << latest_imu_acc_cov_[4] << "," << latest_imu_acc_cov_[8];
  csv_file_ << "," << latest_imu_gyr_cov_[0] << "," << latest_imu_gyr_cov_[4] << "," << latest_imu_gyr_cov_[8];

  // Wheel speed
  if (latest_wheel_.valid) {
    csv_file_ << "," << latest_wheel_.linear(0) << "," << latest_wheel_.linear(1) << "," << latest_wheel_.linear(2);
    csv_file_ << "," << latest_wheel_.covariance[0] << "," << latest_wheel_.covariance[7] << "," << latest_wheel_.covariance[14];
    csv_file_ << ",1," << (latest_wheel_.used_for_update ? "1" : "0");
  } else {
    csv_file_ << ",NaN,NaN,NaN,NaN,NaN,NaN,0,0";
  }

  // GNSS
  if (latest_gnss_.valid) {
    auto& g = latest_gnss_;
    csv_file_ << "," << nan_or(g.lat) << "," << nan_or(g.lon) << "," << nan_or(g.alt);
    csv_file_ << "," << nan3(g.pos_ecef);
    csv_file_ << "," << g.pos_cov_ecef[0] << "," << g.pos_cov_ecef[4] << "," << g.pos_cov_ecef[8];
    csv_file_ << "," << nan3(g.pos_enu);
    csv_file_ << "," << g.pos_cov_enu[0] << "," << g.pos_cov_enu[4] << "," << g.pos_cov_enu[8];
    csv_file_ << "," << nan3(g.vel_ecef);
    csv_file_ << "," << g.vel_cov_ecef[0] << "," << g.vel_cov_ecef[4] << "," << g.vel_cov_ecef[8];
    csv_file_ << "," << nan3(g.vel_enu);
    csv_file_ << "," << g.vel_cov_enu[0] << "," << g.vel_cov_enu[4] << "," << g.vel_cov_enu[8];
    csv_file_ << "," << nan_or(g.doppler_heading);
    csv_file_ << "," << (int)g.status << "," << (int)g.num_sats;
    csv_file_ << "," << g.gdop << "," << g.pdop << "," << g.hdop << "," << g.vdop;
    csv_file_ << "," << g.ratio << "," << g.age_diff;
    csv_file_ << "," << (g.pos_is_nan ? "0" : "1");
    csv_file_ << ",1," << (g.used_for_update ? "1" : "0");
  } else {
    // No GNSS data this row
    csv_file_ << ",NaN,NaN,NaN";  // LLH
    csv_file_ << ",NaN,NaN,NaN,NaN,NaN,NaN";  // ECEF pos + cov
    csv_file_ << ",NaN,NaN,NaN,NaN,NaN,NaN";  // ENU pos + cov
    csv_file_ << ",NaN,NaN,NaN,NaN,NaN,NaN";  // ECEF vel + cov
    csv_file_ << ",NaN,NaN,NaN,NaN,NaN,NaN";  // ENU vel + cov
    csv_file_ << ",NaN";  // doppler heading
    csv_file_ << ",NaN,NaN";  // status, sats
    csv_file_ << ",NaN,NaN,NaN,NaN";  // DOPs
    csv_file_ << ",NaN,NaN";  // ratio, age
    csv_file_ << ",NaN";  // pos_valid
    csv_file_ << ",0,0";  // gnss_valid, gnss_used
  }

  csv_file_ << "\n";
  csv_file_.flush();
}

}  // namespace ekf

// ============================================================
// main
// ============================================================
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ekf::EkfNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
