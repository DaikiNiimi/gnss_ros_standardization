#pragma once
// FGO model: configuration, state keys, DD noise, the NHC and Doppler factors,
// IMU preintegration and the constant-velocity MotionFactor used by gnss_fgo.
// The DD factors are the official GTSAM ones (Engine::buildDd).
// RTKLIB's `#define OMGE` would break gtsam::gnss::OMGE in the official GNSS
// factor headers; hide it while they are parsed, whatever was included first.
#pragma push_macro("OMGE")
#undef OMGE
#include <gtsam/navigation/CarrierPhaseFactor.h>
#include <gtsam/navigation/PseudorangeFactor.h>
#pragma pop_macro("OMGE")
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace gnss_fgo {
// Estimator settings. The public ROS parameters that set them are declared in
// ros_node.hpp; the remaining fields are internal constants.
struct Config {
  bool gnss_motion = false;
  // White acceleration spectral density, m^2/s^3. Q(dt) integrates it exactly.
  double motion_acceleration_psd = 9.0;

  double sigma_pr = 0.3;
  double sigma_cp = 0.003;
  double sigma_amb0 = 30.0;
  double el_mask_deg = 10.0;
  double elmin_deg = 15.0; // pos1.elmask
  // SNR mask, RTKLIB's test per band, [0] rover, [1] base; one row per band
  // with the threshold [dBHz] at 5, 15, ..., 85 deg. A masked band rejects
  // the whole satellite for that receiver (Preprocessor::edit).
  std::array<bool, 2> snrmask_enable{false, false};
  std::array<std::array<double, 9>, 3> snrmask{};
  double snr_ref_dbhz = 42.0;
  double err_a = 0.001;
  double err_b = 0.001;
  double err_eratio_pr = 100.0;
  double err_sclkstab = 5e-12;
  double fde_pr = 4.0;
  double fde_cp = 0.5;
  double fde_max_frac = 0.5;
  int fde_enable = 1;
  double varholdamb = 0.001;
  double pim_break_trans_sigma = 1.0;
  double thres_slip = 0.15;
  double doppler_fde_k = 4.0;
  double doppler_sd_sigma = 0.5;
  double doppler_huber = 1.0;
  int min_dd_for_solve = 4;
  double propagate_pose_sigma = 1.0;
  double propagate_vel_sigma = 1.0;
  double propagate_bias_sigma = 0.1;
  double ddpr_max_res = 2.0;
  double per_sat_res_thresh = 3.0;
  double imu_integ_cov = 0.001;
  double accel_noise = 6e-4;
  double gyro_noise = 2e-4;
  double accel_bias_sigma = 0.000314;
  double gyro_bias_sigma = 2.4e-6;
  double init_pitch_deg = std::numeric_limits<double>::quiet_NaN();
  int ar_mode = 3;
  double ar_thresar = 3.0;
  int ar_minfixsats = 4;
  int exclude_bds_geo = 1;
  double ar_context_worst_sat_max = 4.0;
  double vel_thresh = 1.0;
  int n_collect = 5;
  double body_rot_std = 1.0;
  int nhc_enable = 0;
  double nhc_min_speed = 0.0;
  double nhc_sigma_lat = 0.3;
  double nhc_sigma_vert = 0.2;
  std::string nhc_lever = "0,0,0";
  int zupt_enable = 0;
  double zupt_max_acc_std = 0.55;
  double zupt_max_gyro_std = 0.03;
  double zupt_max_gyro_median = 0.02;
  int zupt_min_samples = 5;
  double zupt_sigma_zero_velocity = 0.05;
  double fls_lag = 1.0;
  int isam2_relinearize_skip = 10;
  double isam2_relinearize_threshold = 0.05;
  void validate() const {
    auto check = [](double v, const char *name, bool positive) {
      if (!std::isfinite(v) || (positive ? v <= 0 : v < 0))
        throw std::invalid_argument(std::string("invalid parameter: ") + name);
    };
    check(motion_acceleration_psd, "motion_acceleration_psd", true);
    check(sigma_pr, "sigma_pr", true);
    check(sigma_cp, "sigma_cp", true);
    check(sigma_amb0, "sigma_amb0", true);
    check(el_mask_deg, "el_mask_deg", false);
    check(elmin_deg, "elmin_deg", false);
    check(snr_ref_dbhz, "snr_ref_dbhz", false);
    check(err_a, "err_a", false);
    check(err_b, "err_b", false);
    check(err_eratio_pr, "err_eratio_pr", false);
    check(err_sclkstab, "err_sclkstab", false);
    check(fde_pr, "fde_pr", false);
    check(fde_cp, "fde_cp", false);
    check(fde_max_frac, "fde_max_frac", false);
    check(fde_enable, "fde_enable", false);
    if (fde_enable > 1)
      throw std::invalid_argument("fde_enable must be 0 or 1");
    check(varholdamb, "varholdamb", true);
    check(pim_break_trans_sigma, "pim_break_trans_sigma", true);
    check(thres_slip, "thres_slip", false);
    check(doppler_fde_k, "doppler_fde_k", false);
    check(doppler_sd_sigma, "doppler_sd_sigma", false);
    check(doppler_huber, "doppler_huber", false);
    check(min_dd_for_solve, "min_dd_for_solve", false);
    check(propagate_pose_sigma, "propagate_pose_sigma", true);
    check(propagate_vel_sigma, "propagate_vel_sigma", true);
    check(propagate_bias_sigma, "propagate_bias_sigma", true);
    check(ddpr_max_res, "ddpr_max_res", false);
    check(per_sat_res_thresh, "per_sat_res_thresh", false);
    check(imu_integ_cov, "imu_integ_cov", true);
    check(accel_noise, "accel_noise", true);
    check(gyro_noise, "gyro_noise", true);
    check(accel_bias_sigma, "accel_bias_sigma", true);
    check(gyro_bias_sigma, "gyro_bias_sigma", true);
    if (!std::isnan(init_pitch_deg) && !std::isfinite(init_pitch_deg))
      throw std::invalid_argument("invalid initial pitch");
    check(ar_mode, "ar_mode", false);
    check(ar_thresar, "ar_thresar", true);
    check(ar_minfixsats, "ar_minfixsats", true);
    check(exclude_bds_geo, "exclude_bds_geo", false);
    if (exclude_bds_geo > 1)
      throw std::invalid_argument("exclude_bds_geo must be 0 or 1");
    check(ar_context_worst_sat_max, "ar_context_worst_sat_max", false);
    check(vel_thresh, "vel_thresh", false);
    check(n_collect, "n_collect", true);
    check(body_rot_std, "body_rot_std", true);
    check(nhc_enable, "nhc_enable", false);
    if (nhc_enable > 1)
      throw std::invalid_argument("nhc_enable must be 0 or 1");
    check(nhc_min_speed, "nhc_min_speed", false);
    check(nhc_sigma_lat, "nhc_sigma_lat", true);
    check(nhc_sigma_vert, "nhc_sigma_vert", true);
    check(zupt_enable, "zupt_enable", false);
    if (zupt_enable > 1)
      throw std::invalid_argument("zupt_enable must be 0 or 1");
    check(zupt_max_acc_std, "zupt_max_acc_std", false);
    check(zupt_max_gyro_std, "zupt_max_gyro_std", false);
    check(zupt_max_gyro_median, "zupt_max_gyro_median", false);
    check(zupt_min_samples, "zupt_min_samples", true);
    check(zupt_sigma_zero_velocity, "zupt_sigma_zero_velocity", true);
    check(fls_lag, "fls_lag", true);
    check(isam2_relinearize_skip, "isam2_relinearize_skip", true);
    check(isam2_relinearize_threshold, "isam2_relinearize_threshold", true);
    if (n_collect < 2 || ar_minfixsats < 2 || fde_max_frac <= 0 ||
        fde_max_frac > 1 || el_mask_deg >= 90 || elmin_deg >= 90)
      throw std::invalid_argument("invalid observation/collection limits");
    if (ar_mode != 0 && ar_mode != 3)
      throw std::invalid_argument("pos2.armode must be 0 (off) or 3 (fix and hold)");
    for (const auto &band : snrmask)
      for (double v : band)
        check(v, "pos1.snrmask", false);
    std::istringstream lever(nhc_lever);
    double x;
    char comma;
    for (int i = 0; i < 3; ++i) {
      if (!(lever >> x) || !std::isfinite(x) ||
          (i < 2 && (!(lever >> comma) || comma != ',')))
        throw std::invalid_argument("invalid NHC lever");
    }
    lever >> std::ws;
    if (!lever.eof())
      throw std::invalid_argument("invalid NHC lever");
  }
};
} // namespace gnss_fgo

namespace gnss_fgo {
using SF = std::pair<int, int>;
using Key = gtsam::Key;
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using AmbMap = std::map<SF, Key>;
inline Key poseKey(int i) { return gtsam::Symbol('P', i); }
inline Key velKey(int i) { return gtsam::Symbol('v', i); }
inline Key biasKey(int i) { return gtsam::Symbol('b', i); }
inline Key ambKey(SF sf, int generation = 0) {
  return gtsam::Symbol('n', std::uint64_t(generation) * 1000000 +
                                sf.first * 10 + sf.second);
}
// `key` is the ambiguity variable of the current arc: ambKey(sf, generation),
// one key for every epoch of the arc. `generation` only grows, so a new arc
// never reuses a key that may still be inside the smoother's lag window.
struct SatelliteState {
  int outage = 0, arc = 0, generation = 0, fix_streak = 0, fix_flag = 0;
  // Satellite system of the observation that created `key` (Engine::buildDd);
  // with the band it names the (system, band) group of the key.
  int system = -1;
  std::optional<Key> key;
  std::optional<double> held, last_held;
  bool seed_pending = false;
  void activate(double value) {
    if (!std::isfinite(value))
      throw std::invalid_argument("nonfinite hold");
    held = last_held = value;
    seed_pending = false;
  }
  // Ends the arc's key: the old key is no longer refreshed and leaves the
  // smoother through the lag; the next carrier DD creates a new key.
  void endArc() {
    key.reset();
    ++generation;
  }
  // Ends the hold and the arc: the next key is new (seeded from last_held
  // when `seed`), not tied to the old key or its hold factors.
  void release(bool seed = true) {
    if (held)
      last_held = held;
    held.reset();
    endArc();
    fix_streak = 0;
    seed_pending = seed && last_held.has_value();
  }
  void clearHold() {
    held.reset();
    last_held.reset();
    fix_streak = 0;
    seed_pending = false;
  }
  void invalidate() {
    endArc();
    clearHold();
    ++arc;
    fix_flag = 0;
  }
};
struct SatelliteStates {
  std::map<SF, SatelliteState> track;
  // First-seen order; Engine::phase2 scans held satellites in this order.
  std::vector<SF> insertion_order;
  std::map<int, double> geometry_free;
  SatelliteState &get(SF sf) {
    auto [it, inserted] = track.try_emplace(sf);
    if (inserted)
      insertion_order.push_back(sf);
    return it->second;
  }
  AmbMap keys() const {
    AmbMap result;
    for (const auto &[sf, s] : track)
      if (s.key)
        result[sf] = *s.key;
    return result;
  }
};
struct EpochState {
  std::set<SF> visible, seeded, consumed;
  std::map<SF, int> accepted;
  std::map<int, int> references;
  bool finalized = false;
  void finalize(SatelliteStates &states, bool fixed) {
    if (finalized)
      throw std::logic_error("epoch finalized twice");
    for (auto &[sf, s] : states.track) {
      auto a = accepted.find(sf);
      const bool confirmed = fixed && a != accepted.end() &&
                             a->second == s.arc && visible.count(sf);
      s.fix_streak = confirmed ? s.fix_streak + 1 : 0;
    }
    finalized = true;
  }
};
enum class UpdateKind { None, Gnss, DeadReckoning };
struct Solution {
  Vec3 position = Vec3::Constant(std::numeric_limits<double>::quiet_NaN());
  Vec3 body_position = Vec3::Constant(std::numeric_limits<double>::quiet_NaN());
  Vec3 body_velocity = Vec3::Constant(std::numeric_limits<double>::quiet_NaN());
  Vec3 angular_velocity =
      Vec3::Constant(std::numeric_limits<double>::quiet_NaN());
  Mat3 body_to_ecef = Mat3::Constant(std::numeric_limits<double>::quiet_NaN());
  // Antenna velocity and the position/velocity covariances, ECEF. NaN when
  // the estimate does not provide them (e.g. before the motion graph exists).
  Vec3 velocity = Vec3::Constant(std::numeric_limits<double>::quiet_NaN());
  Mat3 position_covariance =
      Mat3::Constant(std::numeric_limits<double>::quiet_NaN());
  Mat3 velocity_covariance =
      Mat3::Constant(std::numeric_limits<double>::quiet_NaN());
  bool valid = false, fixed = false;
  UpdateKind kind = UpdateKind::None;
  int nb = 0, phase = 1;
  double gpst = 0, ratio = 0;
  std::string failure;
};
// One epoch of edited rover/base observations with satellite states
// (Preprocessor::prepare output). A missing or edited code or carrier phase
// is 0; a missing or edited Doppler is NaN (0 Hz is a valid Doppler).
struct SatelliteObservation {
  int sat = 0, system = 0, prn = 0;
  double elevation = 0, sat_clock_drift = 0;
  Vec3 rover_sat = Vec3::Zero(), base_sat = Vec3::Zero(),
       sat_velocity = Vec3::Zero();
  Vec3 pr = Vec3::Zero(), pr_base = Vec3::Zero(), phase = Vec3::Zero(),
       phase_base = Vec3::Zero();
  Vec3 doppler = Vec3::Constant(std::numeric_limits<double>::quiet_NaN());
  Vec3 snr = Vec3::Zero(), wavelength = Vec3::Zero();
  Vec3 doppler_wavelength = Vec3::Zero(), doppler_snr = Vec3::Zero();
  Eigen::Vector3i lli = Eigen::Vector3i::Zero();
};
struct ObservationEpoch {
  double gpst = 0, base_gpst = 0;
  std::vector<SatelliteObservation> sats;
  std::set<SF> input_slips;
};
} // namespace gnss_fgo

namespace gnss_fgo {
// Geometric range plus the Sagnac term; the tests synthesise observations
// with it.
inline double range(const Vec3 &satellite, const Vec3 &receiver) {
  const double r = (satellite - receiver).norm();
  if (!(r > 0))
    throw std::invalid_argument("zero satellite range");
  return r + (7.2921151467e-5 / 299792458.) *
                 (satellite.x() * receiver.y() - satellite.y() * receiver.x());
}
inline double ddSigma(const Config &c, bool code, double er, double ej,
                      double dt) {
  const double elevation =
      std::max(std::min(er, ej), c.el_mask_deg * M_PI / 180.);
  const double sinel = std::max(std::sin(elevation), .05),
               fact = code ? c.err_eratio_pr : 1.;
  const double a = fact * c.err_a, b = fact * c.err_b,
               d = 299792458. * c.err_sclkstab * std::abs(dt);
  return std::sqrt(2. * (2. * (a * a + b * b / (sinel * sinel)) + d * d));
}
// Mounting pitch angle theta [rad] of the vehicle travel axis in the IMU
// frame: one constant scalar per IMU graph, estimated by the NHC.
inline Key boreKey() { return gtsam::Symbol('m', 0); }
// Non-holonomic constraint with the mounting pitch (pose, velocity, theta):
// the vehicle moves along its travel axis, which may be pitched against the
// IMU x axis. u = R^T v + omega x lever is the velocity of the constraint
// point in the IMU frame; the vehicle-frame velocity is u_veh = Ry(-theta) u,
// i.e. the travel axis in the IMU frame is Ry(theta) e_x = (cos theta, 0,
// -sin theta): theta > 0 turns it from the IMU x axis towards -z. No yaw
// angle: with the heading aligned to the direction of travel a yaw mounting
// angle is not separable from the heading. Residual u_veh - (speed, 0, 0)
// with sigmas (1e3, sigma_lat, sigma_vert): the forward component is free.
// Pose/velocity Jacobians are those of the constraint without the angle
// left-multiplied by Ry(-theta); the angle Jacobian is analytic.
class NhcBoreFactor final : public gtsam::NoiseModelFactor {
public:
  Vec3 omega, lever;
  double speed;
  NhcBoreFactor(Key pose, Key velocity, Key bore, const Vec3 &w,
                const Vec3 &arm, double sp, double sigma_lat,
                double sigma_vert)
      : gtsam::NoiseModelFactor(gtsam::noiseModel::Diagonal::Sigmas(
                                    Vec3(1e3, sigma_lat, sigma_vert)),
                                gtsam::KeyVector{pose, velocity, bore}),
        omega(w), lever(arm), speed(sp) {}
  gtsam::Vector
  unwhitenedError(const gtsam::Values &values,
                  gtsam::OptionalMatrixVecType H = nullptr) const override {
    const Mat3 R = values.at<gtsam::Pose3>(keys_[0]).rotation().matrix();
    const Vec3 vb = R.transpose() * values.at<gtsam::Vector3>(keys_[1]);
    const double theta = values.at<double>(keys_[2]);
    const double ct = std::cos(theta), st = std::sin(theta);
    const Vec3 u = vb + omega.cross(lever);
    Mat3 Ry; // Ry(-theta)
    Ry << ct, 0, -st, 0, 1, 0, st, 0, ct;
    if (H) {
      H->resize(3);
      (*H)[0] = gtsam::Matrix::Zero(3, 6);
      (*H)[0].leftCols(3) = Ry * gtsam::skewSymmetric(vb);
      (*H)[1] = Ry * R.transpose();
      (*H)[2] = Vec3(-st * u.x() - ct * u.z(), 0, ct * u.x() - st * u.z());
    }
    return Ry * u - Vec3(speed, 0, 0);
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<NhcBoreFactor>(*this);
  }
};
class SdDopplerFactor final : public gtsam::NoiseModelFactor {
public:
  Mat3 R;
  Vec3 omega_cross_lever, h;
  double constant;
  SdDopplerFactor(Key pose, Key vel, double doppler, double ref_doppler,
                  double lam, double ref_lam, const Vec3 &sat,
                  const Vec3 &sat_vel, const Vec3 &ref, const Vec3 &ref_vel,
                  const Vec3 &receiver, const Vec3 &lever, const Mat3 &rotation,
                  const Vec3 &omega, const gtsam::SharedNoiseModel &noise,
                  double sat_clock_drift = 0, double ref_clock_drift = 0)
      : gtsam::NoiseModelFactor(noise, gtsam::KeyVector{pose, vel}),
        R(rotation), omega_cross_lever(omega.cross(lever)) {
    const Vec3 e = (sat - receiver).normalized(),
               er = (ref - receiver).normalized();
    constexpr double k = 7.2921151467e-5 / 299792458.;
    // Difference between two satellites of one epoch: the receiver clock
    // drift cancels, the satellite clock drifts do not.
    h = -(e - er);
    h.x() += k * (sat.y() - ref.y());
    h.y() -= k * (sat.x() - ref.x());
    constant = e.dot(sat_vel) - er.dot(ref_vel) + lam * doppler -
               ref_lam * ref_doppler -
               k * ((sat_vel.x() - ref_vel.x()) * receiver.y() -
                    (sat_vel.y() - ref_vel.y()) * receiver.x()) -
               299792458. * (sat_clock_drift - ref_clock_drift);
  }
  gtsam::Vector
  unwhitenedError(const gtsam::Values &values,
                  gtsam::OptionalMatrixVecType H = nullptr) const override {
    const Mat3 body = values.at<gtsam::Pose3>(keys_[0]).rotation().matrix();
    const Vec3 v =
        R * (values.at<gtsam::Vector3>(keys_[1]) + body * omega_cross_lever);
    if (H) {
      H->resize(2);
      (*H)[0] = gtsam::Matrix::Zero(1, 6);
      (*H)[0].leftCols(3) =
          -h.transpose() * R * body * gtsam::skewSymmetric(omega_cross_lever);
      (*H)[1] = h.transpose() * R;
    }
    return gtsam::Vector1(constant + h.dot(v));
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<SdDopplerFactor>(*this);
  }
};
// Jacobian of the antenna velocity v + R (omega x lever) of a body state
// (pose rotation R, velocity v) for a known body angular rate omega, with
// respect to [pose tangent (rotation, translation; right perturbation
// R Exp(dtheta)), velocity]. The translation does not enter.
inline Eigen::Matrix<double, 3, 9>
antennaVelocityJacobian(const Mat3 &rotation, const Vec3 &omega,
                        const Vec3 &lever) {
  Eigen::Matrix<double, 3, 9> J = Eigen::Matrix<double, 3, 9>::Zero();
  J.leftCols<3>() = -rotation * gtsam::skewSymmetric(omega.cross(lever));
  J.rightCols<3>() = Mat3::Identity();
  return J;
}
} // namespace gnss_fgo

namespace gnss_fgo {
struct ImuIntervalError : std::invalid_argument {
  using std::invalid_argument::invalid_argument;
};
struct ImuSample {
  double gpst = 0;
  Vec3 acc = Vec3::Zero(), gyro = Vec3::Zero();
  std::optional<double> predecessor;
};
// WGS84 ellipsoid and Earth rotation rate.
constexpr double kWgs84A = 6378137., kWgs84F = 1. / 298.257223563,
                 kWgs84E2 = kWgs84F * (2. - kWgs84F),
                 kWgs84B = kWgs84A * (1. - kWgs84F),
                 kWgs84GM = 3.986004418e14, kEarthRate = 7.2921151467e-5;
// Geodetic latitude [rad], longitude [rad] and ellipsoidal height [m] of the
// ECEF point p (WGS84).
inline Vec3 geodeticWgs84(const Vec3 &p) {
  const double r = p.head<2>().norm(), lon = std::atan2(p.y(), p.x());
  double lat = std::atan2(p.z(), r * (1. - kWgs84E2)), h = 0;
  for (int i = 0; i < 10; ++i) {
    const double s = std::sin(lat),
                 n = kWgs84A / std::sqrt(1. - kWgs84E2 * s * s);
    h = std::abs(lat) < M_PI / 4 ? r / std::cos(lat) - n
                                 : p.z() / s - n * (1. - kWgs84E2);
    lat = std::atan2(p.z(), r * (1. - kWgs84E2 * n / (n + h)));
  }
  return {lat, lon, h};
}
// WGS84 normal gravity [m/s^2] at geodetic latitude `lat` [rad] and
// ellipsoidal height `h` [m]: Somigliana's closed formula on the ellipsoid
//   g0 = ge (1 + k sin^2 lat) / sqrt(1 - e^2 sin^2 lat),
//   ge = 9.7803253359, k = (b gp) / (a ge) - 1 = 0.00193185265241,
// with the second-order free-air term (NIMA TR8350.2 eq. 4-3)
//   g = g0 [1 - 2/a (1 + f + m - 2 f sin^2 lat) h + 3 h^2 / a^2],
//   m = omega^2 a^2 b / GM.
inline double normalGravity(double lat, double h) {
  constexpr double ge = 9.7803253359, k = 0.00193185265241;
  const double s2 = std::sin(lat) * std::sin(lat);
  const double g0 = ge * (1. + k * s2) / std::sqrt(1. - kWgs84E2 * s2);
  const double m =
      kEarthRate * kEarthRate * kWgs84A * kWgs84A * kWgs84B / kWgs84GM;
  return g0 * (1. - 2. / kWgs84A * (1. + kWgs84F + m - 2. * kWgs84F * s2) * h +
               3. * h * h / (kWgs84A * kWgs84A));
}
// Ellipsoidal normal (local up) at the ECEF point p, ECEF.
inline Vec3 ellipsoidUp(const Vec3 &p) {
  const Vec3 g = geodeticWgs84(p);
  return {std::cos(g.x()) * std::cos(g.y()), std::cos(g.x()) * std::sin(g.y()),
          std::sin(g.x())};
}
// Gravity vector -normalGravity * up at the ECEF point p, in the ENU frame
// whose ENU-to-ECEF rotation is R.
inline Vec3 localGravity(const Vec3 &p, const Mat3 &R) {
  const Vec3 g = geodeticWgs84(p);
  return -normalGravity(g.x(), g.z()) * (R.transpose() * ellipsoidUp(p));
}
// Earth rotation rate vector in the ENU frame whose ENU-to-ECEF rotation is R:
// R^T (0, 0, W) = W (0, cos lat, sin lat) at that frame's latitude.
inline Vec3 earthRate(const Mat3 &R) {
  return R.transpose() * Vec3(0, 0, kEarthRate);
}
// Navigation frame of one preintegration: the gravity vector and the angular
// rate of the frame (an Earth-fixed ENU frame turns with the Earth), both in
// that frame.
struct ImuFrame {
  Vec3 gravity = Vec3::Zero(), rotation = Vec3::Zero();
};
inline auto imuParams(const Config &c, const ImuFrame &frame) {
  auto p = std::make_shared<gtsam::PreintegrationCombinedParams>(frame.gravity);
  // The rotating-frame (Coriolis) terms of the prediction.
  p->omegaCoriolis = frame.rotation;
  // Logmap error chart for the IMU factor (requires a GTSAM that provides
  // ImuFactorErrorMode::Logmap; see CMakeLists.txt).
  p->setImuFactorErrorMode(gtsam::ImuFactorErrorMode::Logmap);
  p->body_P_sensor = gtsam::Pose3();
  p->accelerometerCovariance = Mat3::Identity() * c.accel_noise * c.accel_noise;
  p->gyroscopeCovariance = Mat3::Identity() * c.gyro_noise * c.gyro_noise;
  p->integrationCovariance = Mat3::Identity() * c.imu_integ_cov;
  p->biasAccCovariance =
      Mat3::Identity() * c.accel_bias_sigma * c.accel_bias_sigma;
  p->biasOmegaCovariance =
      Mat3::Identity() * c.gyro_bias_sigma * c.gyro_bias_sigma;
  return p;
}
// Longest interval [s] one IMU sample may cover in integrateImu: the gap
// between consecutive samples and the tail from the last sample to the
// interval end (held at the last sample's value).
constexpr double kImuMaxSampleInterval = .1;
struct IntegratedImu {
  gtsam::PreintegratedCombinedMeasurements pim;
  Vec3 mean_gyro = Vec3::Zero();
  std::size_t count = 0;
  IntegratedImu(const Config &c, const gtsam::imuBias::ConstantBias &b,
                const ImuFrame &frame)
      : pim(imuParams(c, frame), b) {}
};
inline IntegratedImu integrateImu(const Config &cfg,
                                  const gtsam::imuBias::ConstantBias &bias,
                                  const std::vector<ImuSample> &samples,
                                  double start, double end,
                                  const ImuFrame &frame) {
  if (samples.empty() || !std::isfinite(start) || !std::isfinite(end) ||
      end <= start)
    throw ImuIntervalError("empty or reversed IMU interval");
  IntegratedImu out(cfg, bias, frame);
  double previous = start;
  for (const auto &s : samples) {
    const double raw = s.gpst - s.predecessor.value_or(previous);
    const double dt = std::nearbyint((s.gpst - previous) * 1e4) / 1e4;
    if (!std::isfinite(raw) || raw <= 0 || raw > kImuMaxSampleInterval + 1e-6 ||
        !std::isfinite(dt) || dt <= 0 || dt > kImuMaxSampleInterval)
      throw ImuIntervalError("duplicate, reversed or uncovered IMU interval");
    if (!s.acc.allFinite() || !s.gyro.allFinite())
      throw ImuIntervalError("nonfinite IMU");
    out.pim.integrateMeasurement(s.acc, s.gyro, dt);
    out.mean_gyro += s.gyro;
    ++out.count;
    previous = s.gpst;
  }
  // The tail is one more interval held by the last sample: an epoch is
  // released once a later sample has arrived, so the tail can be up to one
  // sample period.
  const double tail = std::nearbyint((end - previous) * 1e4) / 1e4;
  if (tail < 0 || tail > kImuMaxSampleInterval)
    throw ImuIntervalError("IMU does not cover interval end");
  if (tail > 0)
    out.pim.integrateMeasurement(samples.back().acc, samples.back().gyro, tail);
  out.mean_gyro /= out.count;
  return out;
}
// Leading samples recorded while the vehicle is still: whole 1 s windows whose
// specific force and angular rate scatter stay below an idling vehicle's.
inline std::size_t leadingStillSamples(const std::vector<ImuSample> &samples,
                                       std::size_t count) {
  constexpr double window = 1., max_accel_std = .15, max_gyro_std = .01;
  constexpr std::size_t min_window_samples = 10;
  count = std::min(count, samples.size());
  std::size_t still = 0;
  while (still < count) {
    std::size_t end = still;
    while (end < count && samples[end].gpst - samples[still].gpst < window)
      ++end;
    const std::size_t n = end - still;
    if (end == count || n < min_window_samples)
      break;
    Vec3 acc_mean = Vec3::Zero(), gyro_mean = Vec3::Zero();
    for (std::size_t i = still; i < end; ++i) {
      acc_mean += samples[i].acc;
      gyro_mean += samples[i].gyro;
    }
    acc_mean /= n;
    gyro_mean /= n;
    Vec3 acc_var = Vec3::Zero(), gyro_var = Vec3::Zero();
    for (std::size_t i = still; i < end; ++i) {
      acc_var += (samples[i].acc - acc_mean).cwiseAbs2();
      gyro_var += (samples[i].gyro - gyro_mean).cwiseAbs2();
    }
    if (std::sqrt(acc_var.maxCoeff() / n) >= max_accel_std ||
        std::sqrt(gyro_var.maxCoeff() / n) >= max_gyro_std)
      break;
    still = end;
  }
  return still;
}
// stationaryBias needs more leading still samples than this.
constexpr std::size_t kStillBiasSamples = 100;
// Mean of the leading still samples, the accelerometer part less `gravity`
// along the mean specific force. Zero when the receiver does not start still:
// a mean over motion is not a bias.
inline gtsam::imuBias::ConstantBias
stationaryBias(const std::vector<ImuSample> &samples, std::size_t count,
               double gravity) {
  count = leadingStillSamples(samples, count);
  if (count <= kStillBiasSamples)
    return {};
  Vec3 a = Vec3::Zero(), g = Vec3::Zero();
  for (std::size_t i = 0; i < count; ++i) {
    a += samples[i].acc;
    g += samples[i].gyro;
  }
  a /= count;
  g /= count;
  Vec3 ba = Vec3::Zero();
  if (a.norm() > 1)
    ba = a - gravity * a / a.norm();
  return {ba, g};
}
} // namespace gnss_fgo

namespace gnss_fgo {
// Antenna translation/velocity in one fixed ENU frame. Rotations in Pose3 are
// coordinate gauges, not measured or estimated vehicle attitude.
// Continuous white acceleration: Q = q*[dt^3/3 I, dt^2/2 I; dt^2/2 I, dt I].
// Its semigroup property ensures a split interval does not invent information.
class MotionFactor final : public gtsam::NoiseModelFactor {
public:
  double dt;
  static gtsam::Matrix66 covariance(double dt, double q) {
    if (!std::isfinite(dt) || dt <= 0 || !std::isfinite(q) || q <= 0)
      throw std::invalid_argument(
          "invalid motion interval or acceleration PSD");
    gtsam::Matrix66 Q = gtsam::Matrix66::Zero();
    Q.topLeftCorner<3, 3>() = Mat3::Identity() * (q * dt * dt * dt / 3.);
    Q.topRightCorner<3, 3>() = Q.bottomLeftCorner<3, 3>() =
        Mat3::Identity() * (q * dt * dt / 2.);
    Q.bottomRightCorner<3, 3>() = Mat3::Identity() * (q * dt);
    if (!Q.allFinite() ||
        Eigen::LLT<gtsam::Matrix66>(Q).info() != Eigen::Success)
      throw std::invalid_argument(
          "motion covariance is not finite positive definite");
    return Q;
  }
  MotionFactor(Key p0, Key v0, Key p1, Key v1, double elapsed, double q)
      : gtsam::NoiseModelFactor(
            gtsam::noiseModel::Gaussian::Covariance(covariance(elapsed, q)),
            gtsam::KeyVector{p0, v0, p1, v1}),
        dt(elapsed) {}
  gtsam::Vector
  unwhitenedError(const gtsam::Values &v,
                  gtsam::OptionalMatrixVecType H = nullptr) const override {
    const auto &p0 = v.at<gtsam::Pose3>(keys_[0]);
    const auto &p1 = v.at<gtsam::Pose3>(keys_[2]);
    const auto &v0 = v.at<Vec3>(keys_[1]);
    const auto &v1 = v.at<Vec3>(keys_[3]);
    if (H) {
      H->resize(4);
      (*H)[0] = gtsam::Matrix::Zero(6, 6);
      (*H)[2] = gtsam::Matrix::Zero(6, 6);
      (*H)[0].block<3, 3>(0, 3) = -p0.rotation().matrix();
      (*H)[2].block<3, 3>(0, 3) = p1.rotation().matrix();
      (*H)[1] = gtsam::Matrix::Zero(6, 3);
      (*H)[3] = gtsam::Matrix::Zero(6, 3);
      (*H)[1].topRows(3) = -dt * Mat3::Identity();
      (*H)[1].bottomRows(3) = -Mat3::Identity();
      (*H)[3].bottomRows(3) = Mat3::Identity();
    }
    gtsam::Vector6 e;
    e << p1.translation() - p0.translation() - dt * v0, v1 - v0;
    return e;
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<MotionFactor>(*this);
  }
};
} // namespace gnss_fgo
