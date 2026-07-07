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
#include <deque>
#include <functional>

extern "C" {
#include "rtklib.h"
}

namespace gnss_imu_kalman_filter {

/**
 * @brief State indices for the 16-dim state vector
 *
 * State vector layout:
 *   [0-2]   position     (local ENU, meters)
 *   [3-5]   velocity     (local ENU, m/s)
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
  // Local ENU origin (the filter's navigation frame is local ENU)
  std::string local_origin_mode = "gnss_fix";  // "gnss_fix" or "manual"
  std::array<double, 3> local_origin_pos = {0.0, 0.0, 0.0};  // lat, lon, alt

  // Topics
  std::string topic_gnss_solution = "/gnss/solution";
  std::string topic_imu_raw       = "/imu/data_raw";
  std::string topic_wheel_speed   = "/can_twist";
  std::string topic_solution  = "/gnss_imu_kalman_filter/solution";

  // CSV
  CsvConfig csv;

  // Process noise (per-axis spectral density)
  Eigen::Vector3d sigma_acc      = {0.3, 0.3, 0.3};    // [m/s²/√Hz] vibration margin included
  Eigen::Vector3d sigma_gyr      = {0.01, 0.01, 0.01};  // [rad/s/√Hz]
  Eigen::Vector3d sigma_acc_bias = {1e-4, 1e-4, 1e-4};  // [m/s²/√s]
  Eigen::Vector3d sigma_gyr_bias = {1e-5, 1e-5, 1e-5};  // [rad/s/√s]

  // Initial covariance (1σ std per axis)
  Eigen::Vector3d init_pos_std      = {5.0, 5.0, 10.0};   // [m] horiz ~5 m, vert ~10 m (GNSS SPP)
  Eigen::Vector3d init_vel_std      = {0.3, 0.3, 0.3};    // [m/s] stationary start assumed
  Eigen::Vector3d init_att_std      = {0.1, 0.1, M_PI};   // [rad] roll/pitch ~6°, yaw unknown (π)
  Eigen::Vector3d init_acc_bias_std = {0.1, 0.1, 0.1};    // [m/s²] ~10 mg
  Eigen::Vector3d init_gyr_bias_std = {0.01, 0.01, 0.01}; // [rad/s] ~0.6°/s

  // GNSS update
  GnssUpdateMode gnss_update_mode = GnssUpdateMode::FIX_ONLY;
  // Minimum horizontal speed [m/s] to use GNSS Doppler heading update. Below
  // this the Doppler direction is noisy, but the error-propagated heading
  // variance (~1/speed^2, 5-deg floor) already de-weights it, so this can be low.
  double gnss_heading_speed_threshold = 0.5;
  // Fallback when the GNSS position covariance is degenerate (e.g. all-zero,
  // which some receivers publish to mean "unknown"). Below gnss_cov_min_var
  // (max ENU diagonal) the covariance is replaced by a default^2 chosen by the
  // solution status, so a zero cov is not mistaken for a ~1 mm over-confident fix
  // and a coarse SINGLE/DGPS solution is not trusted like an RTK fix.
  Eigen::Vector3d gnss_pos_sigma_default_fix   = {0.05, 0.05, 0.10};  // [m] RTK FIX
  Eigen::Vector3d gnss_pos_sigma_default_float = {0.5, 0.5, 1.0};     // [m] RTK FLOAT
  Eigen::Vector3d gnss_pos_sigma_default_other = {2.0, 2.0, 4.0};     // [m] DGPS/SBAS/SINGLE/PPP
  double gnss_cov_min_var = 1e-6;                                     // [m^2]
  // Fallback GNSS velocity 1-sigma [m/s] when the velocity covariance is
  // degenerate (zero). <= 0 keeps the legacy behaviour (skip velocity/heading
  // updates). Only applied when the reported velocity is not exactly zero, so a
  // receiver emitting neither velocity nor covariance is not read as a v=0 fix.
  double gnss_vel_sigma_default = 0.2;                                // [m/s]

  // Time alignment of out-of-sync GNSS / wheel-speed measurements via IMU buffer
  bool   time_align_to_gnss   = true;   // false: legacy immediate-update (no time alignment)
  double imu_buffer_duration  = 2.0;    // [s] retain IMU samples for this duration

  // Wheel speed
  bool   use_wheel_speed    = false;
  std::string wheel_speed_topic_type = "twist_with_covariance"; // or "twist"
  std::string wheel_speed_mode = "longitudinal_only";           // or "3d"
  double wheel_speed_sigma  = 0.1;  // [m/s]
  // Optional non-holonomic constraint (NHC): augment the longitudinal wheel update
  // with body lateral/vertical velocity ~= 0. Standard for ground vehicles; off by
  // default to keep the example platform-agnostic.
  bool   wheel_nhc_enable        = false;
  double wheel_nhc_sigma_lateral  = 0.3;  // [m/s]
  double wheel_nhc_sigma_vertical = 0.3;  // [m/s]

  // Continuous accelerometer leveling (gravity-reference roll/pitch aid). The
  // measured body specific-force direction observes roll/pitch: with a low-grade
  // IMU the gravity-dominated specific force (~9.8 m/s^2) turns a sub-degree
  // attitude error into a large spurious horizontal acceleration that dead-
  // reckons between GNSS fixes ("gravity leak"). Rather than a hard quasi-static
  // gate, the update runs every IMU sample with an ADAPTIVE measurement noise
  // driven by the nav-frame kinematic acceleration a_lin = ||C_bn*f + g|| (the
  // departure from pure gravity — unlike | ||f||-g |, this correctly captures
  // horizontal/centripetal acceleration): strong when near-static, weak (or
  // skipped) under manoeuvre. This keeps roll/pitch — and thus the gravity
  // projection — continuously bounded. (Yaw does not leak gravity, so roll/pitch
  // is the whole fix.)
  // A SUSTAINED linear acceleration biases the observed gravity direction
  // systematically (~a_lin/g rad), and a systematic innovation accumulates
  // regardless of the measurement noise — so leveling is HARD-SKIPPED whenever
  // the kinematic acceleration exceeds max_acc, and relies on the many quasi-
  // static samples for the correction. Within the gate the noise still inflates
  // with a_lin / angular rate (soft weighting).
  // Leveling is computed from a short MOVING AVERAGE of the specific force
  // (window below), not the instantaneous sample: a low-grade IMU has large
  // per-sample noise (easily >1 m/s^2) that would otherwise trip the a_lin gate
  // every sample and corrupt the gravity direction. Averaging both cleans the
  // direction and lets the gate reflect the true kinematic acceleration.
  bool   leveling_enable        = true;
  double leveling_window        = 0.3;   // [s] specific-force averaging window
  double leveling_sigma_min_deg = 1.0;   // [deg] 1-sigma floor (quasi-static)
  double leveling_acc_gain      = 15.0;  // [deg per m/s^2] sigma inflation vs a_lin
  double leveling_gyr_gain      = 30.0;  // [deg per rad/s] sigma inflation vs ||omega||
  double leveling_max_acc       = 0.5;   // [m/s^2] skip the update above this a_lin

  // Zero-velocity update (ZUPT): stationarity detected over a sliding window of
  // IMU samples (acc/gyr variance thresholds) -> pseudo-observation v = 0.
  // Bounds velocity/position drift while standing still and during GNSS gaps.
  bool   zupt_enable          = true;
  double zupt_window          = 0.5;    // [s] detection window
  double zupt_acc_std_thresh  = 0.15;   // [m/s^2] max acc std-dev in window
  double zupt_gyr_std_thresh  = 0.02;   // [rad/s] max gyro std-dev in window
  double zupt_sigma           = 0.05;   // [m/s] 1-sigma of the v=0 observation
  double zupt_min_interval    = 0.2;    // [s] rate limit between ZUPT updates
  // IMU variance alone cannot distinguish rest from smooth constant-velocity
  // motion, so gate on a MEASURED speed reference (GNSS horizontal speed, or
  // wheel speed) — generic, no self-referential estimate. When a fresh reference
  // exceeds the threshold, ZUPT is suppressed; when none is fresh (e.g. GNSS
  // outage — exactly when ZUPT is most valuable) the IMU-variance test stands
  // alone.
  double zupt_speed_thresh    = 0.5;    // [m/s] suppress ZUPT above this measured speed
  double zupt_speed_timeout   = 1.5;    // [s]   max age of the speed reference to trust

  // IMU initialization
  double init_imu_duration = 1.0;  // [s]
  bool   use_init_yaw = true;      // true: init_yaw_deg used immediately; false: wait for Doppler
  double init_yaw_deg = 0.0;       // [deg] initial yaw when use_init_yaw is true

  // Output configuration
  std::string output_reference_frame = "imu";  // "gnss" or "imu"

  // IMU mounting
  Eigen::Vector3d lever_arm = Eigen::Vector3d::Zero();  // [m]
  Eigen::Vector3d imu_orientation_rpy = Eigen::Vector3d::Zero();  // [rad]
};

/**
 * @brief Single IMU sample retained in the ring buffer for time-aligned propagation
 *        of asynchronous observations (GNSS, wheel speed). Body-frame values are
 *        already corrected for the IMU mounting rotation.
 */
struct ImuSample {
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  Eigen::Vector3d acc_body = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyr_body = Eigen::Vector3d::Zero();
};

/**
 * @brief Full EKF state + covariance snapshot at a past IMU epoch.
 *
 * Kept in a ring buffer parallel to the IMU sample buffer so an out-of-sequence
 * observation (a latent GNSS solution stamped at its measurement epoch but
 * delivered tens–hundreds of ms later) can be applied at the correct epoch by
 * rewinding the filter, updating, and re-propagating the buffered IMU forward.
 */
struct StateSnapshot {
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  Eigen::Matrix<double, 16, 1> x;                    // STATE_DIM
  Eigen::Matrix<double, 15, 15> P;                   // ERROR_STATE_DIM
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
  uint32_t week = 0;
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

  // Position is NaN (not obtained)
  bool pos_is_nan = true;
};

/**
 * @brief GNSS velocity rotated into the FIXED origin ENU frame (origin_llh_)
 *        — the same frame as the EKF velocity state. See
 *        computeOriginFrameVelocity().
 */
struct OriginVelocity {
  bool valid = false;
  Eigen::Vector3d vel_enu = Eigen::Vector3d::Zero();
  Eigen::Matrix3d cov_enu = Eigen::Matrix3d::Zero();
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
  explicit GnssImuKalmanFilter(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
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
   * @brief Build the discrete error-state transition matrix F (15x15).
   *
   * Pure function of the linearization point — exposed (public, static) so the
   * Jacobian can be unit-tested against finite differences (see
   * test/test_gnss_imu_ekf.cpp). Uses the LOCAL (body-frame) attitude-error
   * convention; in particular the velocity-attitude block is
   * F(VEL,ATT) = -C_bn*[acc_corrected x]*dt, NOT the global form
   * -[ (C_bn*acc) x ]*dt. See the convention note in gnss_imu_kalman_filter.cpp.
   *
   * @param C_bn           body->nav rotation at the linearization point
   * @param acc_corrected  bias-corrected specific force, body frame [m/s^2]
   * @param gyr_corrected  bias-corrected angular rate, body frame [rad/s]
   * @param dt             time step [s]
   */
 public:
  static Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>
  errorStateTransition(const Eigen::Matrix3d& C_bn,
                       const Eigen::Vector3d& acc_corrected,
                       const Eigen::Vector3d& gyr_corrected, double dt);

 private:

  /**
   * @brief GNSS position measurement update
   * @param z_pos  Observed position (local ENU)
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
   * @brief GNSS velocity measurement update (nav-frame velocity from Doppler)
   * @param z_vel  Observed velocity in local ENU [m/s]
   * @param R_vel  Measurement noise covariance (3x3)
   *
   * Independent of and complementary to the wheel-speed update: GNSS gives the
   * absolute 3-axis nav-frame velocity (when GNSS is good), wheel speed gives the
   * continuous body-frame longitudinal velocity. Both are applied as separate
   * updates and fused via their covariances.
   */
  void updateGnssVelocity(const Eigen::Vector3d& z_vel,
                          const Eigen::Matrix3d& R_vel);

  /**
   * @brief Continuous accelerometer leveling update (gravity-direction obs).
   *
   * Called every IMU sample; observes roll/pitch only. Uses a moving average of
   * the specific force / angular rate over leveling_window (from imu_buffer_) to
   * reject per-sample IMU noise. The measurement noise self-weights with the
   * kinematic acceleration and angular rate; the update is skipped when the
   * (averaged) kinematic acceleration exceeds leveling_max_acc.
   */
  void updateLeveling();

  /**
   * @brief Zero-velocity update. Called at IMU rate; fires when the sliding-
   *        window acc/gyr standard deviations are below the zupt_* thresholds.
   */
  void updateZupt(const rclcpp::Time& stamp);

  /**
   * @brief Inject an error-state correction dx into the nominal state and apply
   *        the ESKF covariance reset for the attitude block:
   *        P <- G P G^T with G_att = I - 0.5*[dtheta x] (right-multiplicative
   *        local error convention). All measurement updates go through this.
   */
  void applyErrorState(const Eigen::Matrix<double, ERROR_STATE_DIM, 1>& dx);

  /**
   * @brief Shared EKF measurement update: S = H P H^T + R, K = P H^T S^-1,
   *        Joseph-form covariance update + symmetrization, then
   *        applyErrorState(K*innovation) (state injection + ESKF reset).
   *        Skips (returns false, throttled WARN) on non-finite inputs or a
   *        non-invertible innovation covariance S.
   */
  bool kalmanUpdate(const Eigen::VectorXd& innovation,
                    const Eigen::MatrixXd& H,
                    const Eigen::MatrixXd& R);

  /**
   * @brief Wheel speed velocity measurement update (called from onWheelSpeed* callbacks)
   * @param linear_velocity  Observed velocity in body frame [m/s]
   * @param covariance       Measurement noise covariance (3x3)
   */
  void processWheelSpeed(const Eigen::Vector3d& linear_velocity,
                         const Eigen::Matrix3d& covariance,
                         const rclcpp::Time& t_obs);

  /**
   * @brief Advance the EKF state from prev_imu_stamp_ to t_obs using buffered IMU samples.
   *        Used to time-align asynchronous observations (GNSS, wheel speed) before
   *        applying their measurement update. Forward-only: out-of-sequence
   *        measurements (t_obs < prev_imu_stamp_) trigger a warning and the state
   *        is left unchanged (caller may proceed with the legacy immediate-update).
   */
  void predictToTime(const rclcpp::Time& t_obs);

  /**
   * @brief Apply a measurement update at its true observation epoch, handling
   *        out-of-sequence (latent) observations via store-and-rewind.
   *
   * If time alignment is disabled or the filter is not yet initialized, @p doUpdate
   * runs immediately at the current state (legacy behaviour). Otherwise:
   *  - t_obs >= state time: forward-integrate to t_obs, then update (no rewind).
   *  - t_obs <  state time (OOSM): rewind x_/P_ to the snapshot at/just-before
   *    t_obs, forward-integrate to t_obs, run @p doUpdate, then re-propagate the
   *    buffered IMU samples back up to the previous state time. Falls back to an
   *    immediate update if t_obs predates the retained buffer.
   *
   * @p doUpdate must read the current state (it is invoked after the rewind).
   */
  void applyTimeAlignedUpdate(const rclcpp::Time& t_obs,
                              const std::function<void()>& doUpdate);

  /**
   * @brief Push a {stamp, x_, P_} snapshot for the current state time and drop
   *        snapshots older than imu_buffer_duration. Kept aligned with imu_buffer_.
   */
  void pushStateSnapshot(const rclcpp::Time& stamp);

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

  // ---- Coordinate helpers (navigation frame = local ENU) ----

  /**
   * @brief Convert ECEF position to local ENU (relative to origin_ecef_)
   */
  Eigen::Vector3d ecefToEnu(const Eigen::Vector3d& ecef) const;

  /**
   * @brief Convert local ENU position to ECEF
   */
  Eigen::Vector3d enuToEcef(const Eigen::Vector3d& pos) const;

  /**
   * @brief Convert local ENU position to LLH (lat[deg], lon[deg], alt[m])
   */
  Eigen::Vector3d enuToLlh(const Eigen::Vector3d& pos) const;

  /**
   * @brief Rotate a GNSS velocity (and covariance) into the FIXED origin ENU
   *        frame (origin_llh_), the frame of the EKF velocity state.
   *        GnssSolution.vel_enu is defined at the CURRENT receiver position, so
   *        it disagrees with the origin frame for a manual origin or long
   *        travel. Prefers vel_ecef (rotated directly at origin_llh_) and falls
   *        back to a two-stage rotation via the current position; returns
   *        invalid if neither is usable.
   */
  OriginVelocity computeOriginFrameVelocity(const GnssSnapshot& snap) const;

  /**
   * @brief Derive a Doppler heading [rad] from an origin-frame GNSS velocity,
   *        gated on gnss_heading_speed_threshold. Returns NaN when the
   *        velocity is invalid or below the speed gate.
   */
  double dopplerHeadingFromOriginVelocity(const OriginVelocity& vel) const;

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

  // Previous IMU timestamp — also marks the time of the current EKF state.
  rclcpp::Time prev_imu_stamp_{0, 0, RCL_ROS_TIME};
  bool has_prev_imu_stamp_ = false;

  // Running estimate (EMA) of the nominal IMU sample interval [s]. Used to gate
  // predictToTime() so synchronous (high-rate) observations are not ZOH-extrapolated.
  double nominal_imu_dt_ = 0.01;

  // Ring buffer of recent IMU samples for time-aligned observation updates.
  std::deque<ImuSample> imu_buffer_;

  // Ring buffer of full state+covariance snapshots (one per integrated IMU
  // sample), used to rewind for out-of-sequence (latent GNSS) observations.
  std::deque<StateSnapshot> state_buffer_;

  // Rate limiting for ZUPT.
  rclcpp::Time last_zupt_stamp_{0, 0, RCL_ROS_TIME};

  // Latest MEASURED horizontal speed (GNSS or wheel) and its time, used to gate
  // ZUPT generically (no self-referential estimated-speed test).
  double last_meas_speed_ = std::numeric_limits<double>::quiet_NaN();
  rclcpp::Time last_meas_speed_stamp_{0, 0, RCL_ROS_TIME};

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