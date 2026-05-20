// SPDX-License-Identifier: MIT
#ifndef GNSS_ROS_STANDARDIZATION_GNSS_IMU_KALMAN_FILTER_HPP
#define GNSS_ROS_STANDARDIZATION_GNSS_IMU_KALMAN_FILTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "gnss_ros_standardization/msg/gnss_solution.hpp"

#include <Eigen/Dense>

#include <fstream>
#include <string>
#include <mutex>
#include <array>
#include <cmath>
#include <limits>

extern "C" {
#include "rtklib.h"
}

namespace gnss_imu_kalman_filter {

/**
 * @brief State indices for the 16-dim state vector
 *
 * State vector layout:
 *   [0-2]   position     (ECEF or ENU, meters)
 *   [3-5]   velocity     (ECEF or ENU, m/s)
 *   [6-9]   attitude     (quaternion: w, x, y, z)
 *   [10-12] acc  bias    (m/s^2)
 *   [13-15] gyro bias    (rad/s)
 *
 * Error-state (for covariance P, 15x15):
 *   [0-2]   delta position
 *   [3-5]   delta velocity
 *   [6-8]   delta attitude  (rotation vector / small angle)
 *   [9-11]  delta acc bias
 *   [12-14] delta gyro bias
 */
constexpr int STATE_DIM      = 16;  // full state (quaternion = 4)
constexpr int ERROR_STATE_DIM = 15; // error state (attitude = 3)

// Indices into the full state vector
constexpr int IDX_POS  = 0;
constexpr int IDX_VEL  = 3;
constexpr int IDX_QUAT = 6;   // w, x, y, z
constexpr int IDX_AB   = 10;  // accelerometer bias
constexpr int IDX_GB   = 13;  // gyroscope bias

// Indices into the error-state vector (and P matrix)
constexpr int EIDX_POS = 0;
constexpr int EIDX_VEL = 3;
constexpr int EIDX_ATT = 6;
constexpr int EIDX_AB  = 9;
constexpr int EIDX_GB  = 12;

/**
 * @brief GNSS observation update mode
 */
enum class GnssUpdateMode {
  FIX_ONLY,   // Fix solutions only
  FIX_FLOAT,  // Fix + Float solutions
  ALL         // All solution types
};

/**
 * @brief CSV output configuration
 *
 * Two independent log files:
 *  - sensors_log: time-synchronized raw sensor data (IMU, GNSS, wheel speed)
 *  - state_log:   EKF estimated state and covariance
 */
struct CsvConfig {
  std::string dir = "";  // output directory; empty = current working directory

  bool        sensors_log_enabled  = true;
  std::string sensors_log_filename = "ekf_sensors_log.csv";

  bool        state_log_enabled    = false;
  std::string state_log_filename   = "ekf_state_log.csv";
};

/**
 * @brief EKF configuration parameters
 */
struct EkfConfig {
  // Coordinate frame
  std::string coordinate_frame = "enu";  // "ecef" or "enu"

  // Local origin
  std::string local_origin_mode = "gnss_fix";  // "gnss_fix" or "manual"
  std::array<double, 3> local_origin_pos = {0.0, 0.0, 0.0};  // lat, lon, alt

  // Topics
  std::string topic_gnss_solution = "/gnss/solution";
  std::string topic_imu_raw       = "/imu/data_raw";
  std::string topic_wheel_speed   = "/can_twist";
  std::string topic_solution  = "/gnss_imu_kalman_filter/solution";

  // CSV
  CsvConfig csv;

  // Process noise
  double sigma_acc      = 0.1;     // accelerometer noise [m/s^2/√Hz]
  double sigma_gyr      = 0.01;    // gyroscope noise [rad/s/√Hz]
  double sigma_acc_bias = 1.0e-4;  // acc bias random walk [m/s^2/√Hz]
  double sigma_gyr_bias = 1.0e-5;  // gyro bias random walk [rad/s/√Hz]

  // Initial covariance std
  double init_pos_std      = 10.0;  // [m]
  double init_vel_std      = 1.0;   // [m/s]
  double init_att_std      = 0.5;   // [rad]
  double init_acc_bias_std = 0.1;   // [m/s^2]
  double init_gyr_bias_std = 0.01;  // [rad/s]

  // GNSS update
  GnssUpdateMode gnss_update_mode = GnssUpdateMode::FIX_ONLY;
  double gnss_heading_speed_threshold = 0.5;   // [m/s]
  double gnss_pos_gate_chi2 = 7.815;           // Mahalanobis gate threshold (3DoF 99% = 7.815)

  // Wheel speed
  bool   use_wheel_speed    = false;
  std::string wheel_speed_topic_type = "twist_with_covariance"; // or "twist"
  std::string wheel_speed_mode = "longitudinal_only";           // or "3d"
  double wheel_speed_sigma  = 0.1;  // [m/s]

  // IMU initialization
  double init_imu_duration = 1.0;  // [s]
  double init_yaw_deg = std::numeric_limits<double>::quiet_NaN();  // NaN = wait for Doppler heading

  // Output configuration
  std::string output_reference_frame = "imu";  // "gnss" or "imu"

  // IMU mounting
  Eigen::Vector3d lever_arm = Eigen::Vector3d::Zero();  // [m]
  Eigen::Vector3d imu_orientation_rpy = Eigen::Vector3d::Zero();  // [rad]
};

/**
 * @brief Latest GNSS data snapshot for CSV output
 */
struct GnssSnapshot {
  bool valid = false;            // a new GNSS measurement arrived
  bool used_for_update = false;  // was it used in the EKF measurement update?

  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};  // actual GNSS measurement timestamp

  // Raw GNSS data
  double tow = std::numeric_limits<double>::quiet_NaN();
  uint16_t week = 0;
  uint8_t status = 0;
  uint8_t num_sats = 0;
  float gdop = 0.0f, pdop = 0.0f, hdop = 0.0f, vdop = 0.0f;
  float ratio = 0.0f, age_diff = 0.0f;

  // Position (LLH)
  double lat = std::numeric_limits<double>::quiet_NaN();
  double lon = std::numeric_limits<double>::quiet_NaN();
  double alt = std::numeric_limits<double>::quiet_NaN();

  // Position (ECEF)
  Eigen::Vector3d pos_ecef = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  std::array<double, 9> pos_cov_ecef = {};

  // Position (ENU)
  Eigen::Vector3d pos_enu = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  std::array<double, 9> pos_cov_enu = {};

  // Velocity (ECEF)
  Eigen::Vector3d vel_ecef = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  std::array<double, 9> vel_cov_ecef = {};

  // Velocity (ENU) — includes Doppler velocity
  Eigen::Vector3d vel_enu = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  std::array<double, 9> vel_cov_enu = {};

  // Derived heading from Doppler velocity
  double doppler_heading = std::numeric_limits<double>::quiet_NaN();  // [rad]

  // Position is NaN (not obtained)
  bool pos_is_nan = true;
};

/**
 * @brief Latest wheel speed data snapshot for CSV output
 */
struct WheelSpeedSnapshot {
  bool valid = false;
  bool used_for_update = false;
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};  // actual wheel speed measurement timestamp
  Eigen::Vector3d linear = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular = Eigen::Vector3d::Zero();
  std::array<double, 36> covariance = {};
};

/**
 * @brief GNSS/IMU/WheelSpeed EKF Node
 *
 * Error-state Extended Kalman Filter for position, velocity, attitude,
 * and IMU bias estimation.
 * Internal attitude representation: quaternion (w, x, y, z).
 * Output: Euler angles (roll, pitch, yaw) for publishing and CSV.
 */
class GnssImuKalmanFilter : public rclcpp::Node {
 public:
  GnssImuKalmanFilter();
  ~GnssImuKalmanFilter() override;

 private:
  // ---- Parameter loading ----
  void loadParameters();

  // ---- Callbacks ----
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void onGnss(const gnss_ros_standardization::msg::GnssSolution::SharedPtr msg);
  void onWheelSpeedWithCov(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
  void onWheelSpeedPoint(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  // ---- EKF operations ----

  /**
   * @brief Predict step using IMU measurement
   * @param acc  Corrected acceleration in navigation frame [m/s^2]
   * @param gyr  Corrected angular velocity in body frame [rad/s]
   * @param dt   Time step [s]
   */
  void predict(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr, double dt);

  /**
   * @brief GNSS position measurement update
   * @param z_pos  Observed position (ECEF or ENU)
   * @param R_pos  Measurement noise covariance (3x3)
   */
  void updateGnssPosition(const Eigen::Vector3d& z_pos,
                          const Eigen::Matrix3d& R_pos);

  /**
   * @brief GNSS heading (yaw) measurement update from Doppler velocity
   * @param heading_rad  Observed heading [rad]
   * @param heading_var  Heading variance [rad^2]
   */
  void updateGnssHeading(double heading_rad, double heading_var);

  /**
   * @brief Wheel speed velocity measurement update (called from onWheelSpeed* callbacks)
   * @param linear_velocity  Observed velocity in body frame [m/s]
   * @param covariance       Measurement noise covariance (3x3)
   */
  void processWheelSpeed(const Eigen::Vector3d& linear_velocity,
                         const Eigen::Matrix3d& covariance);

  // ---- Initialization ----

  /**
   * @brief Attempt to initialize EKF state from collected sensor data
   * @return true if initialization was successful
   */
  bool tryInitialize();

  // ---- Output ----
  void publishSolution(const rclcpp::Time& stamp);
  void writeSensorsHeader();
  void writeSensorsRow(const rclcpp::Time& stamp);
  void writeStateHeader();
  void writeStateRow(const rclcpp::Time& stamp);

  // ---- Quaternion helpers ----

  /**
   * @brief Get current quaternion from state vector
   */
  Eigen::Quaterniond getQuaternion() const;

  /**
   * @brief Set quaternion in state vector
   */
  void setQuaternion(const Eigen::Quaterniond& q);

  /**
   * @brief Convert quaternion to Euler angles (roll, pitch, yaw)
   */
  static Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& q);

  /**
   * @brief Convert Euler angles (roll, pitch, yaw) to quaternion
   */
  static Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw);

  /**
   * @brief Get rotation matrix (body to navigation) from state quaternion
   */
  Eigen::Matrix3d getRotationMatrix() const;

  /**
   * @brief Apply IMU axis rotation to raw measurements
   */
  Eigen::Vector3d applyImuAxisRotation(const Eigen::Vector3d& raw) const;

  /**
   * @brief Compute lever-arm correction vector (from GNSS antenna to IMU origin
   *        or vice versa) in navigation frame
   */
  Eigen::Vector3d computeLeverArmCorrection() const;

  // ---- Coordinate helpers ----

  /**
   * @brief Convert ECEF position to the working coordinate frame
   * @param ecef  ECEF position [m]
   * @return Position in working frame (ECEF or ENU)
   */
  Eigen::Vector3d ecefToWorkFrame(const Eigen::Vector3d& ecef) const;

  /**
   * @brief Convert working frame position to ECEF
   */
  Eigen::Vector3d workFrameToEcef(const Eigen::Vector3d& pos) const;

  /**
   * @brief Convert working frame position to LLH
   */
  Eigen::Vector3d workFrameToLlh(const Eigen::Vector3d& pos) const;

  /**
   * @brief Gravity vector in navigation frame
   */
  Eigen::Vector3d gravityVector() const;

  // ---- Members ----

  // Configuration
  EkfConfig config_;

  // IMU axis rotation matrix (body → IMU)
  Eigen::Matrix3d R_imu_body_;

  // State
  Eigen::Matrix<double, STATE_DIM, 1> x_;        // full state
  Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM> P_;  // error-state covariance

  // Initialization flags
  bool initialized_ = false;
  bool has_initial_gnss_ = false;
  bool has_initial_imu_  = false;
  bool has_initial_yaw_  = false;

  // Cached initial data for initialization
  Eigen::Vector3d init_gnss_pos_ecef_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d init_imu_acc_sum_ = Eigen::Vector3d::Zero();
  int init_imu_acc_count_ = 0;
  rclcpp::Time init_imu_start_time_{0, 0, RCL_ROS_TIME};
  double init_yaw_ = 0.0;

  // Local origin (ECEF, set from first GNSS fix or config)
  Eigen::Vector3d origin_ecef_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d origin_llh_  = Eigen::Vector3d::Zero();  // lat[rad], lon[rad], alt[m]
  bool origin_set_ = false;

  // Previous IMU timestamp
  rclcpp::Time prev_imu_stamp_{0, 0, RCL_ROS_TIME};
  bool has_prev_imu_stamp_ = false;

  // Latest sensor snapshots for CSV
  GnssSnapshot latest_gnss_;
  WheelSpeedSnapshot latest_wheel_;

  // Latest IMU raw data (for CSV)
  Eigen::Vector3d latest_imu_acc_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d latest_imu_gyr_ = Eigen::Vector3d::Zero();
  std::array<double, 9> latest_imu_acc_cov_ = {};
  std::array<double, 9> latest_imu_gyr_cov_ = {};

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<gnss_ros_standardization::msg::GnssSolution>::SharedPtr gnss_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr wheel_sub_cov_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr wheel_sub_raw_;

  // Publishers
  rclcpp::Publisher<gnss_ros_standardization::msg::GnssSolution>::SharedPtr solution_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // CSV
  std::ofstream sensors_csv_;
  std::ofstream state_csv_;

  // Mutex
  std::mutex mtx_;
};

}  // namespace gnss_imu_kalman_filter

#endif  // GNSS_ROS_STANDARDIZATION_GNSS_IMU_KALMAN_FILTER_HPP