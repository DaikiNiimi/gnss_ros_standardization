// SPDX-License-Identifier: MIT
//
// Vehicle-motion pseudo-measurement factors for the tightly-coupled GNSS/IMU
// FGO example (gnss_imu_fgo.cpp): velocity-direction attitude aiding, the
// Non-Holonomic Constraint (NHC) and the Zero-velocity UPdaTe (ZUPT). All are
// optional, switched by a single enable flag, and built from STOCK GTSAM classes
// only (AttitudeFactor, ExpressionFactor, PriorFactor) - no custom factor
// subclass, no GTSAM modification.
//
// They matter because this example deliberately excludes wheel odometry, so the
// vehicle's own kinematics are the only "free" information left. The three are
// deliberately different in KIND, which matters more than their tuning:
//   - attitude aiding says "the body points where it is going" - rotation only
//   - the NHC says the same physics as a VELOCITY constraint, which is why it
//     also collides with ZUPT and the Doppler prior (see its own comment)
//   - ZUPT says "velocity is exactly zero", which is rotation-invariant
// Between them they recover most of what a wheel encoder would give, sharpening
// the yaw / velocity the ambiguity resolution rides on.
#ifndef GNSS_ROS_STANDARDIZATION_TIGHTLY_COUPLED_FGO_IMU_FACTORS_HPP_
#define GNSS_ROS_STANDARDIZATION_TIGHTLY_COUPLED_FGO_IMU_FACTORS_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/expressions.h>

namespace gnss_fgo {

// --- Non-Holonomic Constraint -----------------------------------------------

struct NhcConfig {
  bool enable{false};
  double sigma_lat_mps{0.3};   // body-lateral (y) velocity sigma
  double sigma_vert_mps{0.2};  // body-vertical (z) velocity sigma
  double min_speed_mps{0.0};   // engage only above this speed (0 = always)
  Eigen::Vector3d lever_frd{0.0, 0.0, 0.0};  // NHC anchor vs IMU, FLU body frame
  // Yaw-rate gate [rad/s] (0 = off): only apply the NHC when |yaw rate| is below
  // this, i.e. the vehicle is driving roughly straight. This is what makes the
  // constraint SAFE without a measured rear-axle lever: the zero-lateral-velocity
  // condition holds at the rear axle, and the IMU's lateral velocity differs from
  // it by omega x r; while turning that term is large (and a wrong/zero lever
  // injects a yaw error), but while straight (omega ~ 0) it vanishes, so the
  // constraint is kinematically exact at the IMU regardless of the lever. It then
  // observes yaw (aligns heading with the velocity vector) without the turn bias.
  double max_yaw_rate_rps{0.02};
};

// Add the NHC to `graph`: in the FLU body frame the lateral (y) and vertical (z)
// components of the body-frame velocity are ~0, while the forward (x) component
// is left free (huge sigma). The body velocity is v_body = R^T * v_nav (evaluated
// through the stock `unrotate(rotation(pose), v)` expression, so GTSAM supplies
// the exact Jacobians in both the pose and velocity keys). `omega_body` is the
// bias-corrected gyro; when a non-zero anchor lever is set the constraint is
// enforced at the anchor (offset = omega x lever) rather than at the IMU origin.
// `predicted_speed` gates the min-speed engagement. Returns true if added.
inline bool addNhcFactor(gtsam::NonlinearFactorGraph& graph, gtsam::Key pose_key,
                         gtsam::Key vel_key, const NhcConfig& cfg,
                         const Eigen::Vector3d& omega_body, double predicted_speed) {
  if (!cfg.enable || cfg.sigma_lat_mps <= 0.0 || cfg.sigma_vert_mps <= 0.0)
    return false;
  if (predicted_speed < cfg.min_speed_mps) return false;
  // Yaw-rate gate: skip while turning (omega_body.z() is the FLU yaw rate).
  if (cfg.max_yaw_rate_rps > 0.0 &&
      std::abs(omega_body.z()) > cfg.max_yaw_rate_rps)
    return false;
  const Eigen::Vector3d offset =
      cfg.lever_frd.norm() > 0.0 ? omega_body.cross(cfg.lever_frd)
                                 : Eigen::Vector3d::Zero();
  // v_body + offset must have y = z = 0  <=>  h(x) = R^T v = -offset on (y,z).
  const gtsam::Pose3_ pose_(pose_key);
  const gtsam::Point3_ vel_(vel_key);
  const gtsam::Point3_ v_body = gtsam::unrotate(gtsam::rotation(pose_), vel_);
  // Forward (x) carries a huge sigma so it is effectively unconstrained; the
  // measurement's x entry is arbitrary (0).
  const gtsam::Vector3 meas(0.0, -offset.y(), -offset.z());
  auto noise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector3(1.0e3, cfg.sigma_lat_mps, cfg.sigma_vert_mps));
  graph.add(gtsam::ExpressionFactor<gtsam::Point3>(noise, gtsam::Point3(meas),
                                                   v_body));
  return true;
}

// --- Velocity-direction attitude aiding -------------------------------------
//
// The heading of a wheeled vehicle IS the direction it is travelling, to within
// side-slip. That statement is about ROTATION ONLY, so it belongs in a rotation
// factor - which is the whole difference between this and the NHC above.
//
// Why it exists. Nothing else in this graph ties the body heading to the
// direction of travel. Yaw is observable only through the CombinedImuFactor's
// "velocity change vs R*(a - b_a)" consistency, and that separates yaw from the
// horizontal accelerometer bias only while the DIRECTION of horizontal
// acceleration is changing. Without this aiding the yaw error wanders over
// tens of degrees - slowly, with a small within-window spread, so it is drift
// and not noise. Through a horizontal lever arm of order a metre that misplaces
// the antenna by decimetres, which is the dominant term in the float-solution
// tail that the ratio test then fails on.
//
// Why not the NHC. The NHC says the same physics but as a VELOCITY constraint,
// so it also duplicates ZUPT at a standstill and the Doppler prior while moving.
// It does cure the heading, but it also makes the velocity over-confident, and
// a wrong fix then survives several times longer before anything can escape it.
// An attitude factor carries no velocity information at all, so that mechanism
// cannot fire.
//
// Assumptions, which are NOT universal - see VelocityAttitudeConfig::enable.

struct VelocityAttitudeConfig {
  // Requires a wheeled vehicle: small side-slip, no sustained reverse driving,
  // body +x along the direction of travel. False for aircraft, boats, and
  // handheld/backpack use, where the velocity direction says nothing about
  // where the body frame points.
  bool enable{true};
  double min_speed_mps{3.0};      // below this the course is noise, not heading
  double sideslip_deg{2.0};       // sigma floor: side-slip + mount yaw offset
  double max_misalign_deg{90.0};  // skip beyond this (guards reverse driving)
};

// Sigma of "body +x points along the velocity" [rad].
//
// Note on units: gtsam's Unit3 error is the CHORDAL measure sin(theta), not
// theta, so a sigma quoted in radians is the small-angle reading - exact below
// ~10 deg and slightly conservative beyond. It also means the error is NOT
// monotone past 90 deg and vanishes at 180 deg (the antipode is a stationary
// point), which is why a large initial offset must be fixed by re-anchoring the
// heading outright rather than by letting the optimizer walk there.
//
// Derived from the velocity uncertainty rather than tuned: a velocity error
// sigma_vel perpendicular to a velocity of magnitude speed is an angular error
// of sigma_vel / speed. The side-slip floor is added in quadrature and normally
// dominates - deliberately so, because it must stay LARGER than the IMU mount
// tilt (~1.6 deg on the PPC unit, from the static specific force) so this factor
// informs the yaw without fighting the roll/pitch that the accelerometer already
// determines well.
inline double velocityAttitudeSigma(double speed_mps, double sigma_vel_mps,
                                    double sideslip_rad) {
  const double from_vel = (speed_mps > 1e-3)
                              ? std::abs(sigma_vel_mps) / speed_mps
                              : std::numeric_limits<double>::infinity();
  const double floor = std::abs(sideslip_rad);
  if (!std::isfinite(from_vel)) return floor > 0.0 ? floor : 1.0;
  return std::sqrt(from_vel * from_vel + floor * floor);
}

// Horizontal angle between the body +x axis and the velocity [rad]; NaN when
// either direction is degenerate. Reverse driving reads ~pi, which is what
// VelocityAttitudeConfig::max_misalign_deg exists to reject: the factor would
// otherwise be perfectly happy to lock the heading 180 deg out.
inline double velocityAttitudeMisalign(const gtsam::Rot3& nRb,
                                       const Eigen::Vector3d& vel_nav) {
  const Eigen::Vector3d fwd = nRb.matrix().col(0);
  const double fn = std::hypot(fwd.x(), fwd.y());
  const double vn = std::hypot(vel_nav.x(), vel_nav.y());
  if (fn < 1e-9 || vn < 1e-9) return std::numeric_limits<double>::quiet_NaN();
  const double c = (fwd.x() * vel_nav.x() + fwd.y() * vel_nav.y()) / (fn * vn);
  return std::acos(std::max(-1.0, std::min(1.0, c)));
}

// Add the aiding factor on `pose_key`. Rotation-only by construction: stock
// gtsam::AttitudeFactor<Pose3> differentiates the 2-D angular error w.r.t. the
// pose's ROTATION block alone, so the velocity and position keys are untouched.
// `huber_k` <= 0 (or non-finite) means plain Gaussian - use that while the
// heading has not converged yet, because Huber caps precisely the large residual
// that has to be corrected then. Returns true if a factor was added.
inline bool addVelocityAttitudeFactor(gtsam::NonlinearFactorGraph& graph,
                                      gtsam::Key pose_key,
                                      const gtsam::Rot3& nRb_predicted,
                                      const Eigen::Vector3d& vel_nav,
                                      double sigma_vel_mps,
                                      const VelocityAttitudeConfig& cfg,
                                      double huber_k) {
  if (!cfg.enable) return false;
  const double speed = vel_nav.norm();
  if (!(speed >= cfg.min_speed_mps) || speed < 1e-6) return false;
  const double misalign = velocityAttitudeMisalign(nRb_predicted, vel_nav);
  if (!std::isfinite(misalign) ||
      misalign > cfg.max_misalign_deg * M_PI / 180.0)
    return false;
  const double sigma = velocityAttitudeSigma(
      speed, sigma_vel_mps, cfg.sideslip_deg * M_PI / 180.0);
  if (!(sigma > 0.0) || !std::isfinite(sigma)) return false;
  // The full 3-D direction, not its horizontal projection: on a graded road the
  // forward axis genuinely follows the velocity, and projecting would force the
  // pitch to zero on every slope.
  gtsam::SharedNoiseModel noise =
      gtsam::noiseModel::Isotropic::Sigma(2, sigma);
  if (huber_k > 0.0 && std::isfinite(huber_k)) {
    noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(huber_k), noise);
  }
  graph.add(gtsam::AttitudeFactor<gtsam::Pose3>(
      pose_key, gtsam::Unit3(vel_nav), noise, gtsam::Unit3(1.0, 0.0, 0.0)));
  return true;
}

// --- One-shot heading alignment ---------------------------------------------
//
// When to accept the course over ground as the heading, while the run has not
// yet had a heading at all (`yaw_aligned_ == false`). alignYawToCourse replaces
// the heading OUTRIGHT, so the decision is guarded three ways:
//
//   1. Candidates are gated on the COURSE SIGMA (sigma_vel/speed, with sigma_vel
//      the measured per-epoch Doppler LS value), not on a bare speed threshold:
//      speed is a poor proxy for course quality, and a gate in degrees compares
//      directly against the re-anchor threshold it protects.
//   2. Several consecutive candidates must AGREE. The quantity voted on is the
//      yaw OFFSET dpsi = course - yaw, not the course, which keeps the test
//      usable while turning (a constant offset stays constant through a turn
//      because the gyro carries the yaw). The median discards isolated blunders.
//   3. A REVERSING vehicle defeats 1 and 2 together: its course is exactly
//      180 deg from its heading, precisely and repeatably. That is not a
//      course-quality problem and no averaging sees it, so an offset beyond
//      reverse_ambiguous_deg may only be applied at or above reverse_speed_mps.
struct YawAlignConfig {
  // Course sigma (velocityAttitudeSigma) a candidate must be under [deg]. With
  // the sigma_vel floor of 0.1 m/s, 10 deg corresponds to speed >= 0.57 m/s.
  double max_sigma_deg{10.0};
  int agree_epochs{3};      // consecutive agreeing candidates required
  double agree_deg{20.0};   // allowed spread across those candidates
  // An agreed offset beyond this is indistinguishable from reverse driving on
  // GNSS alone, and may only be applied at or above reverse_speed_mps. Shares
  // its meaning (and default) with VelocityAttitudeConfig::max_misalign_deg.
  double reverse_ambiguous_deg{90.0};
  // Speed at which sustained reverse driving is ruled out [m/s]. Default 3.0,
  // matching the aiding factor's own gate; the measurement above is what makes
  // that number a property of vehicles rather than a tuning choice.
  double reverse_speed_mps{3.0};
};

// Vote accumulator. Non-candidate epochs clear it: the test is about consecutive
// agreement, and a gap means the votes either side were not measuring the same
// stationary offset.
class YawAlignmentVote {
 public:
  void reset() {
    votes_.clear();
    speeds_.clear();
  }
  int size() const { return static_cast<int>(votes_.size()); }

  // Feed one epoch. `course_rad` and `yaw_rad` are both CCW from East in the
  // nav frame; `sigma_rad` is velocityAttitudeSigma for this epoch and
  // `speed_mps` its horizontal speed. Returns true when the last
  // cfg.agree_epochs candidates agree AND the agreed offset is one the speed
  // permits, writing the agreed offset (their median) and the spread that
  // admitted it.
  bool update(double course_rad, double yaw_rad, double sigma_rad,
              double speed_mps, const YawAlignConfig& cfg,
              double* dpsi_out = nullptr, double* spread_out = nullptr) {
    if (!std::isfinite(course_rad) || !std::isfinite(yaw_rad) ||
        !std::isfinite(sigma_rad) ||
        sigma_rad > cfg.max_sigma_deg * M_PI / 180.0) {
      reset();
      return false;
    }
    const int want = std::max(cfg.agree_epochs, 1);
    votes_.push_back(wrapPi(course_rad - yaw_rad));
    speeds_.push_back(speed_mps);
    while (static_cast<int>(votes_.size()) > want) {
      votes_.pop_front();
      speeds_.pop_front();
    }
    if (static_cast<int>(votes_.size()) < want) return false;

    // Unwrap about the newest vote before comparing: two offsets either side of
    // +/-pi are 1 deg apart, not 359.
    std::vector<double> v;
    v.reserve(votes_.size());
    const double ref = votes_.back();
    for (const double d : votes_) v.push_back(ref + wrapPi(d - ref));
    const double spread =
        *std::max_element(v.begin(), v.end()) - *std::min_element(v.begin(), v.end());
    if (spread_out) *spread_out = spread;
    if (spread > cfg.agree_deg * M_PI / 180.0) return false;
    std::sort(v.begin(), v.end());
    const double dpsi = wrapPi(v[v.size() / 2]);
    // A near-reversed offset is only believable if the speed rules out a
    // reversing vehicle - it is a perfectly precise, perfectly consistent
    // measurement of the wrong thing, so nothing upstream of here can catch it.
    // Every candidate in the window has to clear the bar, not just the median.
    if (std::fabs(dpsi) > cfg.reverse_ambiguous_deg * M_PI / 180.0 &&
        *std::min_element(speeds_.begin(), speeds_.end()) <
            cfg.reverse_speed_mps)
      return false;
    if (dpsi_out) *dpsi_out = dpsi;
    return true;
  }

  static double wrapPi(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

 private:
  std::deque<double> votes_;
  std::deque<double> speeds_;
};

// Replace the yaw of `nRb` with the course over ground, keeping roll and pitch.
//
// Needed because initialization runs on a STATIC window, where yaw is not
// observable at all: the node starts with yaw = 0 by construction under a
// sigma = pi prior, i.e. body +x pointing EAST whatever way the vehicle faces.
// The continuous factor above would eventually walk that out, but only slowly
// and only through the weak channel described at the top - and the +/-90 deg
// misalignment guard cannot be relied on until the error is already small. One
// alignment at the first confidently-moving epoch removes the offset outright.
inline gtsam::Rot3 alignYawToCourse(const gtsam::Rot3& nRb,
                                    const Eigen::Vector3d& vel_nav) {
  if (std::hypot(vel_nav.x(), vel_nav.y()) < 1e-6) return nRb;
  // ENU nav frame: yaw is CCW from East, so the course is atan2(north, east).
  const gtsam::Vector3 rpy = nRb.rpy();
  return gtsam::Rot3::RzRyRx(rpy.x(), rpy.y(),
                             std::atan2(vel_nav.y(), vel_nav.x()));
}

// --- Static-initialization gate ---------------------------------------------

struct StaticInitConfig {
  double max_gyro_radps{0.05};
  double max_acc_std_mps2{0.5};
  double acc_norm_tol_mps2{0.3};  // |mean specific force| must be within this of g
  double max_speed_mps{0.3};      // GNSS Doppler speed gate (when available)
  double gravity_mps2{9.80665};
};

// True when the IMU window (bias-free means/dispersion) plus the optional GNSS
// Doppler speed indicate the platform is stationary, so roll/pitch and gyro bias
// may be initialized from the averages. The gyro-mean and accel-dispersion gates
// alone accept a constant-velocity (or smooth constant-acceleration) drive; the
// specific-force-norm gate rejects scale errors / vertical bias / large tilt /
// sustained vertical acceleration (a horizontal accel only perturbs the norm by
// a^2/2g, so it is a partial guard), and the Doppler-speed gate is the direct
// "am I moving" test the IMU alone cannot make.
inline bool staticInitDetect(const Eigen::Vector3d& acc_mean, double acc_std,
                             const Eigen::Vector3d& gyr_mean, bool doppler_valid,
                             double doppler_speed, const StaticInitConfig& cfg) {
  if (gyr_mean.norm() >= cfg.max_gyro_radps) return false;
  if (acc_std >= cfg.max_acc_std_mps2) return false;
  if (std::abs(acc_mean.norm() - cfg.gravity_mps2) >= cfg.acc_norm_tol_mps2)
    return false;
  if (doppler_valid && doppler_speed >= cfg.max_speed_mps) return false;
  return true;
}

// --- Zero-velocity update ----------------------------------------------------

struct ZuptConfig {
  bool enable{true};
  double max_acc_std{0.55};      // [m/s^2] specific-force dispersion gate
  double max_gyr_std{0.030};     // [rad/s] angular-rate dispersion gate
  double max_gyr_median{0.020};  // [rad/s] bias-corrected rate magnitude gate
  int min_samples{5};
  // Reject the ZUPT when the current estimate says the platform is moving (0=off).
  // ON by default: IMU quietness alone cannot tell rest from smooth slow motion
  // (traffic creep), and a false zero-velocity pin lags the position and induces
  // false fixes - this repo's EKF reaches the same evidence-gate conclusion.
  double max_speed_mps{1.0};
  double sigma_mps{0.5};      // zero-velocity prior sigma (horizontal)
  // Separate VERTICAL sigma. A strapdown INS is weakest in the vertical channel
  // (tilt and vertical accelerometer bias are not separable at rest), and an
  // urban course is stopped a large fraction of the time - exactly when ZUPT
  // fires. Pinning the vertical velocity to zero there removes the only path by
  // which an existing vertical POSITION error can be worked off, because
  // position can only change through velocity; the float vertical error then
  // degrades by more than 2x while stopped, and to metre scale at the epochs
  // where AR fails. A large value here keeps the horizontal benefit of ZUPT
  // (which is real - the vehicle genuinely is not moving) without freezing the
  // vertical. Set equal to sigma_mps to recover the isotropic behaviour.
  double sigma_vert_mps{0.5};
};

// One IMU sample as seen by the detector (specific force + angular rate, already
// in the body frame the node integrates in).
struct ImuStat {
  Eigen::Vector3d acc;
  Eigen::Vector3d gyr;
};

// GICI-style stationarity test on the epoch's IMU window, bias-corrected by
// `gyr_bias`. Returns true when the vehicle is judged stationary. `prior_speed`
// is the current velocity estimate; when `max_speed_mps > 0` a moving prior
// vetoes the ZUPT (the IMU alone cannot tell rest from smooth motion). Diagnostic
// values, when requested, are written to the out-params.
inline bool zuptDetect(const std::vector<ImuStat>& win, const ZuptConfig& cfg,
                       const Eigen::Vector3d& gyr_bias, double prior_speed,
                       double* acc_std_out = nullptr, double* gyr_std_out = nullptr,
                       double* gyr_median_out = nullptr) {
  if (!cfg.enable) return false;
  const int n = static_cast<int>(win.size());
  if (n < cfg.min_samples) return false;
  if (cfg.max_speed_mps > 0.0 && prior_speed > cfg.max_speed_mps) return false;

  Eigen::Vector3d acc_mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyr_mean = Eigen::Vector3d::Zero();
  for (const auto& s : win) {
    acc_mean += s.acc;
    gyr_mean += s.gyr;
  }
  acc_mean /= n;
  gyr_mean /= n;
  double acc_var = 0.0, gyr_var = 0.0;
  std::vector<double> gyr_mag;
  gyr_mag.reserve(n);
  for (const auto& s : win) {
    acc_var += (s.acc - acc_mean).squaredNorm();
    gyr_var += (s.gyr - gyr_mean).squaredNorm();
    gyr_mag.push_back((s.gyr - gyr_bias).norm());
  }
  // Dispersion as the RMS per-sample deviation magnitude (isotropic).
  const double acc_std = std::sqrt(acc_var / n);
  const double gyr_std = std::sqrt(gyr_var / n);
  std::sort(gyr_mag.begin(), gyr_mag.end());
  const double gyr_median = gyr_mag[n / 2];
  if (acc_std_out) *acc_std_out = acc_std;
  if (gyr_std_out) *gyr_std_out = gyr_std;
  if (gyr_median_out) *gyr_median_out = gyr_median;

  if (acc_std > cfg.max_acc_std) return false;
  if (gyr_std > cfg.max_gyr_std) return false;
  if (gyr_median > cfg.max_gyr_median) return false;
  return true;
}

// Add the zero-velocity prior on V(k). Caller supplies the detector verdict.
inline void addZuptFactor(gtsam::NonlinearFactorGraph& graph, gtsam::Key vel_key,
                          const ZuptConfig& cfg) {
  // Axis-wise: the nav frame is ENU, so index 2 is the vertical channel.
  graph.add(gtsam::PriorFactor<gtsam::Vector3>(
      vel_key, gtsam::Vector3::Zero(),
      gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(3) << cfg.sigma_mps, cfg.sigma_mps,
           cfg.sigma_vert_mps).finished())));
}

// --- Earth rotation ---------------------------------------------------------
// Earth rotation rate in the local ENU navigation frame at geodetic latitude
// phi [rad]:  omega_ie^n = omega_e * (0, cos phi, sin phi).
inline gtsam::Vector3 earthRateEnu(double lat_rad, double omega_e = 7.2921151467e-5) {
  return gtsam::Vector3(0.0, omega_e * std::cos(lat_rad),
                        omega_e * std::sin(lat_rad));
}

// Initial gyro bias from a STATIC averaging window.
//
// A stationary gyro does not read the bias: it reads
//     omega_ib^b = b_g + R_bn * omega_ie^n
// so the Earth rate is part of the average. Assigning the raw average to the
// bias while the estimator ALSO removes the Earth rate (GTSAM applies
// NavState::coriolis once, in correctPIM) removes the term TWICE and leaves a
// spurious 15.04 deg/h rotation - 5.6-6.8x the ADIS16505-2 in-run stability of
// 2.2-2.7 deg/h. Subtracting it here keeps the bias state carrying only the
// sensor's own bias.
//
// `nRb` is the initial attitude (nav <- body), so its transpose maps nav -> body.
// Pass omega_ie_nav = 0 to reproduce the uncorrected behaviour (which is the
// self-consistent choice when the Coriolis correction is disabled).
inline gtsam::Vector3 staticInitGyroBias(const gtsam::Vector3& gyro_mean,
                                         const gtsam::Rot3& nRb,
                                         const gtsam::Vector3& omega_ie_nav) {
  return gyro_mean - nRb.transpose() * omega_ie_nav;
}

// --- Dead-reckoning velocity covariance ---------------------------------------
//
// Both publishers take an ENU velocity covariance and convert it themselves:
// publishSolution applies R_e_n to reach ECEF, publishOdometry applies the
// attitude to reach the body frame. Handing either one an already-rotated
// covariance rotates it twice, which is exactly the bug these functions exist to
// make impossible - the conversion now lives in ONE place per path, named for
// the frame it returns, instead of being written inline twice with different
// rotations.
//
// `Pvv_body` is the velocity block of the propagated NavState-chart covariance,
// which is body-frame by that chart's convention.
inline Eigen::Matrix3d gapVelocityCovarianceEnu(const Eigen::Matrix3d& Pvv_body,
                                                const gtsam::Rot3& nRb_pred) {
  const Eigen::Matrix3d R = nRb_pred.matrix();
  const Eigen::Matrix3d out = R * Pvv_body * R.transpose();
  return 0.5 * (out + out.transpose());
}

// Fallback for when no joint posterior was available to propagate. Deliberately
// conservative: an outage is the wrong place to understate uncertainty, and a
// consumer can ignore a large covariance but cannot recover from a small wrong
// one. `last_vel_cov_enu` is already ENU (the graph's own chart); the
// preintegrated velocity block is body-frame and takes the departure attitude.
inline Eigen::Matrix3d gapVelocityCovarianceEnuFallback(
    const Eigen::Matrix3d& last_vel_cov_enu, const Eigen::Matrix3d& Qvv_body,
    const gtsam::Rot3& nRb_prev, double floor_std) {
  const Eigen::Matrix3d R = nRb_prev.matrix();
  const Eigen::Matrix3d out = last_vel_cov_enu + R * Qvv_body * R.transpose() +
                              Eigen::Matrix3d::Identity() * (floor_std * floor_std);
  return 0.5 * (out + out.transpose());
}

// --- Antenna phase-center velocity ------------------------------------------

inline Eigen::Matrix3d skew3(const Eigen::Vector3d& v) {
  Eigen::Matrix3d S;
  S <<   0.0, -v.z(),  v.y(),
       v.z(),    0.0, -v.x(),
      -v.y(),  v.x(),    0.0;
  return S;
}

// Covariance inputs for antennaVelocityNav.
//
// Prefer `joint`: the graph's own 15x15 posterior, which carries the
// attitude/velocity/bias CROSS terms a sum of separate marginals cannot. The
// block-diagonal fields are the fallback for callers that hold only separate
// marginals - dead reckoning, the initialization publish, and a FIX epoch,
// whose integer-conditioned posterior is 3x3. On a FIX epoch `bias_gyro` may
// come from the FLOAT block: AR does not condition the IMU bias.
struct AntennaVelocityCov {
  Eigen::MatrixXd joint;  // 15x15 [rot(0:2) trans(3:5) vel(6:8) ba(9:11) bg(12:14)]
  bool joint_ok{false};
  Eigen::Matrix3d vel_nav{Eigen::Matrix3d::Zero()};    // fallback P_vv
  Eigen::Matrix3d rot{Eigen::Matrix3d::Zero()};        // fallback P_theta_theta
  Eigen::Matrix3d bias_gyro{Eigen::Matrix3d::Zero()};  // fallback P_bg_bg
  // Gyro noise, in the same units the node feeds the Odometry angular-velocity
  // covariance (imu.sigma_gyr used directly as a standard deviation).
  double sigma_gyr{0.0};
};

struct AntennaVelocity {
  Eigen::Vector3d v_nav{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d cov_nav{Eigen::Matrix3d::Zero()};
};

// Velocity of the GNSS antenna phase center, in the nav frame:
//
//   v_ant = v_body + R_nav_body * (omega_body x lever_arm)
//
// The exact inverse of the lever-arm removal the Doppler factor applies to
// observe the body-origin V(k). `omega_body` must be bias-corrected and the
// SAME rate the node publishes as the Odometry twist.angular, so the two
// messages describe one rigid body. GnssSolution names the antenna for every
// position and velocity field, Odometry the body origin: a vehicle turning in
// place has |v_ant| = |omega| * |lever_arm| while v_body is zero.
//
// Covariance, with GTSAM's body-side perturbation R <- R*Exp(dtheta) and
// omega = omega_meas - b_g:
//
//   d v_ant / d v      =  I
//   d v_ant / d dtheta = -R * skew(omega x l)
//   d v_ant / d b_g    =  R * skew(l)
//
// Gyro MEASUREMENT white noise enters through the same path as the bias and is
// ALWAYS added: the graph has no state for it, so it is in no posterior block.
inline AntennaVelocity antennaVelocityNav(const gtsam::Rot3& nRb,
                                          const Eigen::Vector3d& v_body_nav,
                                          const Eigen::Vector3d& omega_body,
                                          const gtsam::Point3& lever_arm,
                                          const AntennaVelocityCov& cov) {
  AntennaVelocity out;
  const Eigen::Vector3d l(lever_arm);
  const Eigen::Matrix3d R = nRb.matrix();
  const Eigen::Vector3d wxl = omega_body.cross(l);
  out.v_nav = v_body_nav + R * wxl;

  const Eigen::Matrix3d J_theta = -R * skew3(wxl);
  const Eigen::Matrix3d M = R * skew3(l);  // = d v_ant / d b_g

  Eigen::Matrix3d P;
  if (cov.joint_ok && cov.joint.rows() == 15 && cov.joint.cols() == 15) {
    Eigen::Matrix<double, 3, 15> J = Eigen::Matrix<double, 3, 15>::Zero();
    J.block<3, 3>(0, 0) = J_theta;                        // rot
    J.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();    // velocity
    J.block<3, 3>(0, 12) = M;                             // gyro bias
    P = J * cov.joint * J.transpose();
  } else {
    // No cross terms: a block-diagonal SUM, an approximation that can sit
    // either side of the truth since the omitted correlation is signed.
    P = cov.vel_nav + J_theta * cov.rot * J_theta.transpose() +
        M * cov.bias_gyro * M.transpose();
  }
  P += M * M.transpose() * (cov.sigma_gyr * cov.sigma_gyr);
  out.cov_nav = 0.5 * (P + P.transpose());
  return out;
}

// --- Gap-publisher outage gate ----------------------------------------------

// May the gap publisher synthesize the grid slot at `slot_ctow`?
//
// The publisher and the epoch pipeline cover DISJOINT epochs: every rover epoch
// that reaches the node publishes exactly one solution - including a DD-less one,
// which the inertial chain bridges - so a slot at or before the newest RECEIVED
// rover epoch already has an owner. Synthesizing it anyway emits a second, worse
// solution for the same GPST and makes the output depend on delivery order.
//
// `latest_rover_ctow` is the newest rover epoch RECEIVED (not necessarily
// processed) and `last_pub_ctow` the newest epoch published, both in continuous
// GPST seconds. A negative value means "none yet".
inline bool gapSlotIsFree(double slot_ctow, double latest_rover_ctow,
                          double last_pub_ctow) {
  if (!std::isfinite(slot_ctow)) return false;
  if (last_pub_ctow >= 0.0 && slot_ctow <= last_pub_ctow + 1e-3) return false;
  if (latest_rover_ctow >= 0.0 && slot_ctow <= latest_rover_ctow + 1e-3)
    return false;
  return true;
}

// ONE EPOCH = ONE OUTPUT SLOT, decided in one place.
//
// The output is a function of the SLOT, not of the event that filled it:
// whichever publisher reaches a slot first owns it, normally the real epoch. An
// observation arriving after the gap publisher claimed its slot still enters
// the graph and improves every later epoch; it simply does not re-publish a
// slot the consumer already has, since the contract cannot retract one.
//
// claim() exists so that decision happens ONCE, before anything is sent, and
// covers every output of the epoch as a unit - solution, odometry and counters.
//
// Not thread-safe by design: every caller runs on the solver callback group.
class EpochSlotArbiter {
 public:
  // Take ownership of `slot_ctow` (continuous GPST seconds) if it is free.
  // Returns true exactly once per slot; the caller publishes only then.
  // `latest_rover_ctow` is the "an epoch may still be in the pipeline" test -
  // pass -1.0 to skip it (the real-epoch path, which IS that epoch, and the
  // data-domain gap path, which has direct evidence instead).
  bool claim(double slot_ctow, double latest_rover_ctow = -1.0) {
    if (!gapSlotIsFree(slot_ctow, latest_rover_ctow, last_pub_ctow_)) {
      // Only a slot already published counts as suppression. Yielding to a
      // rover epoch still in the pipeline is the gate working as intended.
      if (last_pub_ctow_ >= 0.0 && slot_ctow <= last_pub_ctow_ + 1e-3)
        ++n_suppressed_;
      return false;
    }
    last_pub_ctow_ = slot_ctow;
    return true;
  }

  // Highest slot ever published; negative until the first claim succeeds.
  double lastPublished() const { return last_pub_ctow_; }

  // Epochs that arrived after their slot had been published. A quality signal
  // that the node is behind, not a defect.
  std::uint64_t suppressed() const { return n_suppressed_; }

  // Deliberately no reset(): the output high-water mark must survive a graph
  // reset, or the node would re-publish slots the consumer already has.

 private:
  double last_pub_ctow_{-1.0};
  std::uint64_t n_suppressed_{0};
};

// --- Rover epoch grid: interval, and the outage bound -----------------------

// Nominal rover epoch interval [s], as the MEDIAN of the observed epoch spacing.
//
// The dead-reckoning grid has to land exactly on the real epoch times, so the
// interval must come from the observation stream itself. A running average of
// the IMU-side dt drifts, and a mean over the spacings is skewed by any gap; the
// median is unaffected by a gap as long as most spacings are nominal.
//
// Returns 0 when there is not enough evidence (fewer than `min_samples`
// spacings), which the caller must treat as "not yet determined".
inline double medianEpochInterval(std::vector<double> spacings,
                                  std::size_t min_samples = 10) {
  spacings.erase(std::remove_if(spacings.begin(), spacings.end(),
                                [](double d) {
                                  return !std::isfinite(d) || d <= 0.0;
                                }),
                 spacings.end());
  if (spacings.size() < min_samples) return 0.0;
  const std::size_t mid = spacings.size() / 2;
  std::nth_element(spacings.begin(), spacings.begin() + mid, spacings.end());
  return spacings[mid];
}

// True when the rover stream has genuinely stopped, as opposed to the node
// being behind on a stream that is still arriving. Both arguments are IMU-clock
// watermarks - now, and when the last rover observation ARRIVED - so the
// difference is how long the node has listened without hearing anything. The
// anchor is refreshed by arrival, not by progress, so a busy node keeps it
// fresh and only real silence lets this grow.
inline bool gapSlotIsUnobserved(double imu_now_s, double last_arrival_imu_s,
                                double interval_s, double outage_epochs = 3.0) {
  if (!std::isfinite(imu_now_s) || !std::isfinite(last_arrival_imu_s) ||
      !(interval_s > 0.0) || !(outage_epochs > 0.0))
    return false;
  return imu_now_s - last_arrival_imu_s > outage_epochs * interval_s;
}

}  // namespace gnss_fgo

#endif  // GNSS_ROS_STANDARDIZATION_TIGHTLY_COUPLED_FGO_IMU_FACTORS_HPP_
