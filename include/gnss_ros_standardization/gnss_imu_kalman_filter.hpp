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

// NOTE: must come after <Eigen/Dense> - this header pulls in rtklib.h, whose
// trace() macro would otherwise clobber Eigen's MatrixBase::trace().
#include "gnss_ros_standardization/gnss_utils.hpp"

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
  // Continuous yaw aiding from GNSS Doppler course. Assumes forward motion with
  // body X along the travel direction (ground vehicles); independent of the
  // one-time use_init_yaw. Default false (platform assumption); the demo
  // config opts in.
  bool use_doppler_heading = false;
  double gnss_heading_speed_threshold = 0.5;  // [m/s] min speed for the heading update
  // GNSS covariance source: "message" = reported covariance, with the sigma
  // below as the degenerate fallback | "config" = always use the sigma below.
  std::string gnss_pos_cov_source = "message";  // "message" | "config"
  std::string gnss_vel_cov_source = "message";  // "message" | "config"
  // Position 1-sigma [m] by solution status (see *_cov_source above).
  Eigen::Vector3d gnss_pos_sigma_default_fix   = {0.05, 0.05, 0.10};  // [m] RTK FIX
  Eigen::Vector3d gnss_pos_sigma_default_float = {0.5, 0.5, 1.0};     // [m] RTK FLOAT
  Eigen::Vector3d gnss_pos_sigma_default_other = {2.0, 2.0, 4.0};     // [m] DGPS/SBAS/SINGLE/PPP
  double gnss_cov_min_var = 1e-6;  // [m^2] below this a reported cov counts as degenerate
  // Velocity 1-sigma [m/s]; <= 0 skips velocity/heading aiding. Never applied
  // to an all-zero reported velocity (indistinguishable from a true standstill).
  double gnss_vel_sigma_default = 0.2;

  // Time alignment of late GNSS / wheel measurements via the IMU buffer
  bool   time_align_to_gnss   = true;   // false: immediate update (no alignment)
  double imu_buffer_duration  = 2.0;    // [s] IMU/state snapshot retention
  // GNSS epoch time source: "header" (stamp as-is) or "tow_auto_offset"
  // (rebuild from week/tow, strip receiver latency). See gnss_utils::GnssEpochAligner.
  std::string gnss_time_source = "header";
  double gnss_offset_window_s = 60.0;  // [s] tow_auto_offset sliding window

  // Wheel speed
  bool   use_wheel_speed    = false;
  std::string wheel_speed_topic_type = "twist_with_covariance"; // or "twist"
  std::string wheel_speed_mode = "longitudinal_only";           // or "3d"
  double wheel_speed_sigma  = 0.1;  // [m/s]
  // Non-holonomic constraint: body lateral/vertical velocity ~= 0 (ground vehicles).
  bool   wheel_nhc_enable        = false;
  double wheel_nhc_sigma_lateral  = 0.3;  // [m/s]
  double wheel_nhc_sigma_vertical = 0.3;  // [m/s]

  // Accelerometer leveling: gravity-direction observation of roll/pitch, which
  // bounds the "gravity leak" that dead-reckons into position between fixes.
  // Computed from a moving average over leveling_window; the noise inflates
  // with kinematic acceleration / angular rate, and the update is skipped above
  // max_acc (a sustained acceleration would bias the gravity direction).
  bool   leveling_enable        = true;
  double leveling_window        = 0.3;   // [s] specific-force averaging window
  double leveling_sigma_min_deg = 1.0;   // [deg] 1-sigma floor (quasi-static)
  double leveling_acc_gain      = 15.0;  // [deg per m/s^2] sigma inflation vs a_lin
  double leveling_gyr_gain      = 30.0;  // [deg per rad/s] sigma inflation vs ||omega||
  double leveling_max_acc       = 0.5;   // [m/s^2] skip the update above this a_lin

  // Zero-velocity update (ZUPT): v = 0 pseudo-observation, applied only with
  // positive evidence of a stop — a fresh measured speed (GNSS/wheel) at/below
  // zupt_speed_thresh plus the std-dev/mean gates. With no fresh reference it
  // fires only if zupt_allow_imu_only (IMU statistics alone cannot tell rest
  // from smooth cruise; a false ZUPT silently freezes the position).
  bool   zupt_enable          = true;
  double zupt_window          = 0.5;    // [s] detection window
  double zupt_acc_std_thresh  = 0.15;   // [m/s^2] max acc std-dev in window
  double zupt_gyr_std_thresh  = 0.02;   // [rad/s] max gyro std-dev in window
  double zupt_sigma           = 0.05;   // [m/s] 1-sigma of the v=0 observation
  double zupt_min_interval    = 0.2;    // [s] rate limit between ZUPT updates
  bool   zupt_allow_imu_only  = false;  // apply v=0 with no fresh speed reference
  double zupt_speed_thresh    = 0.5;    // [m/s] suppress ZUPT above this measured speed
  double zupt_speed_timeout   = 1.5;    // [s]   max age of the speed reference to trust
  double zupt_gyr_mean_thresh = 0.05;   // [rad/s] reject if |mean(gyr)-b_g| exceeds (turning)
  double zupt_acc_mean_thresh = 0.5;    // [m/s^2] reject if | ||mean(acc)-b_a|| - g | exceeds

  // IMU initialization
  double init_imu_duration = 1.0;  // [s]
  bool   use_init_yaw = true;      // true: init_yaw_deg used immediately; false: wait for Doppler
  double init_yaw_deg = 0.0;       // [deg] initial yaw when use_init_yaw is true

  // Output configuration
  std::string output_reference_frame = "imu";  // "gnss" or "imu"

  // Output frame_ids. World defaults to "map": GNSS updates can move the
  // solution discontinuously, which REP-105 forbids for "odom".
  std::string frame_world = "map";
  std::string frame_child = "";  // "" auto-derives from output_reference_frame

  // IMU mounting
  Eigen::Vector3d lever_arm = Eigen::Vector3d::Zero();  // [m]
  Eigen::Vector3d imu_orientation_rpy = Eigen::Vector3d::Zero();  // [rad]
};

/**
 * @brief Buffered IMU sample (body frame, mounting rotation already applied).
 */
struct ImuSample {
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  Eigen::Vector3d acc_body = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyr_body = Eigen::Vector3d::Zero();
};

/**
 * @brief Buffered wheel-speed measurement, replayed when an OOSM rewind
 *        discards the snapshot that first applied it. @c R is the validated
 *        covariance already used for the live update.
 */
struct WheelMeas {
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  Eigen::Vector3d linear = Eigen::Vector3d::Zero();  // body-frame velocity [m/s]
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();   // measurement covariance
};

/**
 * @brief Full state+covariance snapshot at a past IMU epoch — the rewind
 *        target for out-of-sequence (late GNSS) observations.
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
   * @brief Predict step using one IMU sample.
   * @param acc  Measured specific force, BODY frame (mounting-rotated, raw —
   *             the current bias estimate is subtracted inside) [m/s^2]
   * @param gyr  Measured angular rate, BODY frame (mounting-rotated, raw —
   *             the current bias estimate is subtracted inside) [rad/s]
   * @param dt   Time step [s]
   */
  void predict(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr, double dt);

  /**
   * @brief Discrete error-state transition matrix F (15x15). Static/pure so it
   *        can be finite-difference tested. Uses the LOCAL (body-frame)
   *        attitude-error convention: F(VEL,ATT) = -C_bn*[acc x]*dt (see the
   *        convention note in the .cpp). Inputs are bias-corrected body-frame
   *        specific force / angular rate at the linearization point.
   */
 public:
  static Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>
  errorStateTransition(const Eigen::Matrix3d& C_bn,
                       const Eigen::Vector3d& acc_corrected,
                       const Eigen::Vector3d& gyr_corrected, double dt);

  /**
   * @brief Yaw Jacobian d(yaw)/d(delta_theta_body) for the right-local attitude
   *        error: [0, sin(roll)/cos(pitch), cos(roll)/cos(pitch)]. NOT
   *        C_bn.row(2) (equal only at zero tilt). Static for FD testing.
   */
  static Eigen::Matrix<double, 1, 3> headingJacobian(double roll, double pitch);

  /**
   * @brief Accelerometer-bias Jacobian for the leveling update,
   *        H(AB) = (I - u_pred*u_pred^T) / f_norm with
   *        u_pred = C_bn^T*e_z, f_norm = ||acc_mean - b_a||. Static for FD testing.
   */
  static Eigen::Matrix3d levelingBiasJacobian(const Eigen::Vector3d& u_pred, double f_norm);

  /**
   * @brief Symmetrize and validate a measurement covariance: replaces R with
   *        `fallback` unless it is finite, has all diagonal entries >= min_var,
   *        and is positive semi-definite (checked via LDLT). Returns true iff
   *        the original R (post-symmetrization) was used as-is.
   */
  static bool sanitizeCovariance(Eigen::Matrix3d& R, const Eigen::Matrix3d& fallback, double min_var);

 private:

  /**
   * @brief GNSS position update (local ENU).
   * @return true iff kalmanUpdate() actually applied a correction.
   */
  bool updateGnssPosition(const Eigen::Vector3d& z_pos,
                          const Eigen::Matrix3d& R_pos);

  /**
   * @brief GNSS heading (yaw) update from the Doppler course [rad, rad^2].
   * @return true iff a correction was actually applied.
   */
  bool updateGnssHeading(double heading_rad, double heading_var);

  /**
   * @brief GNSS velocity update (nav-frame Doppler velocity, local ENU).
   *        Complementary to the wheel update (absolute nav-frame vs body-frame).
   * @return true iff a correction was actually applied.
   */
  bool updateGnssVelocity(const Eigen::Vector3d& z_vel,
                          const Eigen::Matrix3d& R_vel);

  /**
   * @brief Accelerometer leveling update (roll/pitch) from a moving average of
   *        the specific force up to @p stamp. At most one update per
   *        leveling_window so successive updates use disjoint data windows;
   *        samples newer than @p stamp are ignored (causal during OOSM replay).
   */
  void updateLeveling(const rclcpp::Time& stamp);

  /**
   * @brief Zero-velocity update; see the zupt_* config gates. Samples newer
   *        than @p stamp are ignored (causal during OOSM replay).
   */
  void updateZupt(const rclcpp::Time& stamp);

  /**
   * @brief Inject dx into the nominal state + ESKF attitude covariance reset
   *        (P <- G P G^T, right-local convention). All updates go through this.
   */
  void applyErrorState(const Eigen::Matrix<double, ERROR_STATE_DIM, 1>& dx);

  /**
   * @brief Shared measurement update: gain, Joseph-form covariance update, then
   *        applyErrorState(). Returns false (throttled WARN) on non-finite
   *        inputs or a non-invertible innovation covariance.
   */
  bool kalmanUpdate(const Eigen::VectorXd& innovation,
                    const Eigen::MatrixXd& H,
                    const Eigen::MatrixXd& R);

  /**
   * @brief Wheel-speed callback body: buffer the measurement (for OOSM replay)
   *        and apply it at its epoch via applyTimeAlignedUpdate.
   */
  void processWheelSpeed(const Eigen::Vector3d& linear_velocity,
                         const Eigen::Matrix3d& covariance,
                         const rclcpp::Time& t_obs);

  /**
   * @brief One wheel-speed update at the current state; shared by the live path
   *        and OOSM replay so both produce identical corrections.
   * @return true iff a correction was applied.
   */
  bool applyWheelUpdate(const Eigen::Vector3d& linear_velocity,
                        const Eigen::Matrix3d& covariance);

  /**
   * @brief Re-apply buffered wheel measurements in (from_excl, to_incl],
   *        skipping @p skip_stamp (applied separately by the current update).
   */
  void replayWheelEvents(const rclcpp::Time& from_excl,
                         const rclcpp::Time& to_incl,
                         const rclcpp::Time& skip_stamp);

  /**
   * @brief One IMU epoch: predict, leveling/ZUPT, optional wheel replay,
   *        snapshot. The single definition shared by the live path and OOSM
   *        re-propagation, so both produce the same state.
   */
  void processImuEpoch(const ImuSample& s, double dt, bool replay_wheel,
                       const rclcpp::Time& skip_wheel_stamp);

  /**
   * @brief Advance the state from prev_imu_stamp_ to t_obs using buffered IMU
   *        samples (via processImuEpoch). Forward-only; the caller handles rewinds.
   */
  void predictToTime(const rclcpp::Time& t_obs,
                     const rclcpp::Time& skip_wheel_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME));

  /**
   * @brief Most recent buffered IMU sample with stamp <= t (nullptr if none).
   *        Never returns a sample from the future of the epoch being evaluated
   *        (which a rewind can otherwise expose).
   */
  const ImuSample* lastImuSampleAtOrBefore(const rclcpp::Time& t) const;

  /**
   * @brief Apply a measurement update at its epoch t_obs.
   *        t_obs >= state time: forward-integrate, then update. t_obs < state
   *        time (late GNSS): store-and-rewind — restore the snapshot at/before
   *        t_obs, update there, re-propagate buffered IMU/wheel forward. Falls
   *        back to an immediate update when alignment is off, before init, or
   *        when t_obs predates the buffer.
   * @param skip_wheel_stamp  wheel stamp excluded from replay (set to t_obs
   *        when the trigger is itself a wheel update, so it is not applied twice).
   */
  void applyTimeAlignedUpdate(const rclcpp::Time& t_obs,
                              const std::function<void()>& doUpdate,
                              const rclcpp::Time& skip_wheel_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME));

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
   *        frame (origin_llh_) — the frame of the EKF velocity state. Prefers
   *        vel_ecef, falls back via the current-position ENU; invalid if
   *        neither is usable.
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

  // Initialization anchors; refreshed on every usable fix while waiting for
  // yaw, so tryInitialize() starts from the freshest fix, not the first one.
  Eigen::Vector3d init_gnss_pos_ecef_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d init_gnss_vel_enu_ = Eigen::Vector3d::Zero();
  bool init_gnss_vel_valid_ = false;
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

  // Ring buffer of recent wheel-speed measurements, replayed at their epoch
  // when an OOSM rewind discards the snapshots that first applied them.
  std::deque<WheelMeas> wheel_buffer_;

  // GNSS epoch time source (config gnss_time_source); holds the sliding
  // clock-offset window for the tow_auto_offset mode.
  gnss_utils::GnssEpochAligner gnss_epoch_aligner_;

  // ZUPT/leveling rate-limit bookkeeping. Deliberately NOT snapshotted/rewound:
  // a replay may re-hit a rate limit; the loss is bounded by the GNSS latency.
  rclcpp::Time last_zupt_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_leveling_stamp_{0, 0, RCL_ROS_TIME};

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