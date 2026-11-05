#pragma once
// FGO model: configuration, state keys, GNSS/Doppler factors, IMU preintegration
// and the constant-velocity MotionFactor used by gnss_fgo.
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
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
// Frozen Ratio=3 direct FIX/hold baseline. Units match the Python TcConfig.
struct Config {
  bool gnss_motion = false;
  // White acceleration spectral density, m^2/s^3. Q(dt) integrates it exactly.
  double motion_acceleration_psd = 9.0;

  double sigma_pr = 0.3;
  double sigma_cp = 0.003;
  double sigma_amb0 = 30.0;
  double sigma_cont = 1.0;
  double sigma_n_between = 0.01;
  double sigma_n_between_flt = 0.1;
  int betweenn_enable = 1;
  double el_mask_deg = 10.0;
  double elmin_deg = 15.0;
  double cnr_min_dbhz = 25.0;
  double snr_ref_dbhz = 42.0;
  int varerr_enable = 1;
  double err_a = 0.001;
  double err_b = 0.001;
  double err_eratio_pr = 100.0;
  double err_sclkstab = 5e-12;
  double fde_pr = 4.0;
  double fde_cp = 0.5;
  double fde_max_frac = 0.5;
  int fde_enable = 1;
  int fde_max_iter = 1;
  int sat_band_plan = 1;
  double ar_thresar_min = 3.0;
  double ar_thresar_max = 3.0;
  int ddpr_sanity_enable = 1;
  int sanity_break_pim = 1;
  double sanity_pose_replace_thresh = 5.0;
  double varholdamb = 0.001;
  double pim_break_trans_sigma = 1.0;
  double thres_slip = 0.15;
  int doppler_adaptive_sigma = 1;
  double doppler_fde_k = 4.0;
  int doppler_snr_weight = 1;
  double doppler_gdop_max = 0.0;
  int doppler_require_dd = 1;
  double doppler_sd_sigma = 0.5;
  double doppler_huber = 1.0;
  int min_dd_for_solve = 4;
  double propagate_pose_sigma = 1.0;
  double propagate_vel_sigma = 1.0;
  double propagate_bias_sigma = 0.1;
  double propagate_amb_sigma = 0.1;
  int recov_cp_hold = 5;
  double sanity_max_median_ratio = 5.0;
  int sanity_max_median_min_sats = 6;
  double main_ddpr_res_thresh = 3.0;
  int ddpr_sanity_persist = 3;
  double ddpr_max_res = 2.0;
  double main_ddpr_res_catastrophic = 15.0;
  double ar_ddpr_xvalidate_thresh = 10.0;
  double ddpr_fast_worst_sat_min = 30.0;
  double per_sat_res_thresh = 3.0;
  std::string imu_grade = "tactical";
  double imu_integ_cov = 0.001;
  double accel_noise = 0.00284;
  double gyro_noise = 0.00401;
  double accel_bias_sigma = 0.000314;
  double gyro_bias_sigma = 9.7e-06;
  double bias_between_acc_sigma = 0.0003;
  double bias_between_gyro_sigma = 3e-05;
  double bias_prior_acc_sigma = 0.003;
  double bias_prior_gyro_sigma = 0.0003;
  int bias_prior_mode = 2;
  double init_pitch_deg = std::numeric_limits<double>::quiet_NaN();
  int ar_mode = 3;
  int ar_starve_reset = 50;
  double ar_starve_max_res = 2.0;
  double ar_gdop_max = 8.0;
  double ar_thresar = 3.0;
  int ar_minfixsats = 4;
  int subset_ar_enable = 1;
  int subset_ar_max_candidates = 5;
  int subset_ar_min_nb = 4;
  int subset_ar_max_drop = 2;
  int subset_ar_max_dirty_sats = 2;
  double subset_ar_dirty_sat_res_m = 1.0;
  int exclude_bds_geo = 1;
  double ar_context_main_ddpr_max = 1.2;
  double ar_context_worst_sat_max = 4.0;
  int ar_context_nb_max = 6;
  int low_nb_fix_reject_nb_max = 6;
  int low_nb_fix_only_after_flt = 1;
  int low_nb_fix_reject_max_prev_fix_streak = 2;
  double lambda_corr_hard_max = 0.0;
  int ar_postcheck_enable = 0;
  int diag_factor_residuals = 0;
  double vel_thresh = 1.0;
  int n_collect = 5;
  double body_rot_std = 1.0;
  int nhc_enable = 1;
  double nhc_min_speed = 0.0;
  double nhc_sigma_lat = 0.3;
  double nhc_sigma_vert = 0.2;
  std::string nhc_lever = "0,0,0";
  int zupt_enable = 1;
  double zupt_max_acc_std = 0.55;
  double zupt_max_gyro_std = 0.03;
  double zupt_max_gyro_median = 0.02;
  double zupt_g_dev_thr = 0.0;
  double zupt_max_speed = 0.0;
  int zupt_min_samples = 5;
  double zupt_sigma_zero_velocity = 0.5;
  double zupt_sigma_zero_rotation = 0.01;
  double zupt_anchor_sigma_translation = 1.0;
  double zupt_anchor_sigma_rotation = 0.1;
  double fls_lag = 1.0;
  double phase1_fls_lag = 1.0;
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
    check(sigma_cont, "sigma_cont", true);
    check(sigma_n_between, "sigma_n_between", true);
    check(sigma_n_between_flt, "sigma_n_between_flt", true);
    check(betweenn_enable, "betweenn_enable", false);
    if (betweenn_enable > 1)
      throw std::invalid_argument("betweenn_enable must be 0 or 1");
    check(el_mask_deg, "el_mask_deg", false);
    check(elmin_deg, "elmin_deg", false);
    check(cnr_min_dbhz, "cnr_min_dbhz", false);
    check(snr_ref_dbhz, "snr_ref_dbhz", false);
    check(varerr_enable, "varerr_enable", false);
    if (varerr_enable > 1)
      throw std::invalid_argument("varerr_enable must be 0 or 1");
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
    check(fde_max_iter, "fde_max_iter", true);
    check(sat_band_plan, "sat_band_plan", false);
    if (sat_band_plan > 1)
      throw std::invalid_argument("sat_band_plan must be 0 or 1");
    check(ar_thresar_min, "ar_thresar_min", true);
    check(ar_thresar_max, "ar_thresar_max", true);
    check(ddpr_sanity_enable, "ddpr_sanity_enable", false);
    if (ddpr_sanity_enable > 1)
      throw std::invalid_argument("ddpr_sanity_enable must be 0 or 1");
    check(sanity_break_pim, "sanity_break_pim", false);
    if (sanity_break_pim > 1)
      throw std::invalid_argument("sanity_break_pim must be 0 or 1");
    check(sanity_pose_replace_thresh, "sanity_pose_replace_thresh", false);
    check(varholdamb, "varholdamb", true);
    check(pim_break_trans_sigma, "pim_break_trans_sigma", true);
    check(thres_slip, "thres_slip", false);
    check(doppler_adaptive_sigma, "doppler_adaptive_sigma", true);
    if (doppler_adaptive_sigma > 1)
      throw std::invalid_argument("doppler_adaptive_sigma must be 0 or 1");
    check(doppler_fde_k, "doppler_fde_k", false);
    check(doppler_snr_weight, "doppler_snr_weight", false);
    if (doppler_snr_weight > 1)
      throw std::invalid_argument("doppler_snr_weight must be 0 or 1");
    check(doppler_gdop_max, "doppler_gdop_max", false);
    check(doppler_require_dd, "doppler_require_dd", false);
    if (doppler_require_dd > 1)
      throw std::invalid_argument("doppler_require_dd must be 0 or 1");
    check(doppler_sd_sigma, "doppler_sd_sigma", false);
    check(doppler_huber, "doppler_huber", false);
    check(min_dd_for_solve, "min_dd_for_solve", false);
    check(propagate_pose_sigma, "propagate_pose_sigma", true);
    check(propagate_vel_sigma, "propagate_vel_sigma", true);
    check(propagate_bias_sigma, "propagate_bias_sigma", true);
    check(propagate_amb_sigma, "propagate_amb_sigma", true);
    check(recov_cp_hold, "recov_cp_hold", false);
    check(sanity_max_median_ratio, "sanity_max_median_ratio", false);
    check(sanity_max_median_min_sats, "sanity_max_median_min_sats", false);
    check(main_ddpr_res_thresh, "main_ddpr_res_thresh", false);
    check(ddpr_sanity_persist, "ddpr_sanity_persist", false);
    check(ddpr_max_res, "ddpr_max_res", false);
    check(main_ddpr_res_catastrophic, "main_ddpr_res_catastrophic", false);
    check(ar_ddpr_xvalidate_thresh, "ar_ddpr_xvalidate_thresh", false);
    check(ddpr_fast_worst_sat_min, "ddpr_fast_worst_sat_min", false);
    check(per_sat_res_thresh, "per_sat_res_thresh", false);
    check(imu_integ_cov, "imu_integ_cov", true);
    check(accel_noise, "accel_noise", true);
    check(gyro_noise, "gyro_noise", true);
    check(accel_bias_sigma, "accel_bias_sigma", true);
    check(gyro_bias_sigma, "gyro_bias_sigma", true);
    check(bias_between_acc_sigma, "bias_between_acc_sigma", true);
    check(bias_between_gyro_sigma, "bias_between_gyro_sigma", true);
    check(bias_prior_acc_sigma, "bias_prior_acc_sigma", true);
    check(bias_prior_gyro_sigma, "bias_prior_gyro_sigma", true);
    check(bias_prior_mode, "bias_prior_mode", false);
    if (!std::isnan(init_pitch_deg) && !std::isfinite(init_pitch_deg))
      throw std::invalid_argument("invalid initial pitch");
    check(ar_mode, "ar_mode", false);
    check(ar_starve_reset, "ar_starve_reset", false);
    check(ar_starve_max_res, "ar_starve_max_res", false);
    check(ar_gdop_max, "ar_gdop_max", false);
    check(ar_thresar, "ar_thresar", true);
    check(ar_minfixsats, "ar_minfixsats", true);
    check(subset_ar_enable, "subset_ar_enable", false);
    if (subset_ar_enable > 1)
      throw std::invalid_argument("subset_ar_enable must be 0 or 1");
    check(subset_ar_max_candidates, "subset_ar_max_candidates", false);
    check(subset_ar_min_nb, "subset_ar_min_nb", false);
    check(subset_ar_max_drop, "subset_ar_max_drop", false);
    check(subset_ar_max_dirty_sats, "subset_ar_max_dirty_sats", false);
    check(subset_ar_dirty_sat_res_m, "subset_ar_dirty_sat_res_m", false);
    check(exclude_bds_geo, "exclude_bds_geo", false);
    if (exclude_bds_geo > 1)
      throw std::invalid_argument("exclude_bds_geo must be 0 or 1");
    check(ar_context_main_ddpr_max, "ar_context_main_ddpr_max", false);
    check(ar_context_worst_sat_max, "ar_context_worst_sat_max", false);
    check(ar_context_nb_max, "ar_context_nb_max", false);
    check(low_nb_fix_reject_nb_max, "low_nb_fix_reject_nb_max", false);
    check(low_nb_fix_only_after_flt, "low_nb_fix_only_after_flt", false);
    if (low_nb_fix_only_after_flt > 1)
      throw std::invalid_argument("low_nb_fix_only_after_flt must be 0 or 1");
    check(low_nb_fix_reject_max_prev_fix_streak,
          "low_nb_fix_reject_max_prev_fix_streak", false);
    check(lambda_corr_hard_max, "lambda_corr_hard_max", false);
    check(ar_postcheck_enable, "ar_postcheck_enable", false);
    if (ar_postcheck_enable > 1)
      throw std::invalid_argument("ar_postcheck_enable must be 0 or 1");
    check(diag_factor_residuals, "diag_factor_residuals", false);
    if (diag_factor_residuals > 1)
      throw std::invalid_argument("diag_factor_residuals must be 0 or 1");
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
    check(zupt_g_dev_thr, "zupt_g_dev_thr", false);
    check(zupt_max_speed, "zupt_max_speed", false);
    check(zupt_min_samples, "zupt_min_samples", true);
    check(zupt_sigma_zero_velocity, "zupt_sigma_zero_velocity", true);
    check(zupt_sigma_zero_rotation, "zupt_sigma_zero_rotation", true);
    check(zupt_anchor_sigma_translation, "zupt_anchor_sigma_translation", true);
    check(zupt_anchor_sigma_rotation, "zupt_anchor_sigma_rotation", true);
    check(fls_lag, "fls_lag", true);
    check(phase1_fls_lag, "phase1_fls_lag", true);
    check(isam2_relinearize_skip, "isam2_relinearize_skip", true);
    check(isam2_relinearize_threshold, "isam2_relinearize_threshold", true);
    if (n_collect < 2 || ar_minfixsats < 2 || fde_max_frac <= 0 ||
        fde_max_frac > 1 || el_mask_deg >= 90 || elmin_deg >= 90)
      throw std::invalid_argument("invalid observation/collection limits");
    if (ar_thresar != 3 || ar_thresar_min != 3 || ar_thresar_max != 3 ||
        ar_postcheck_enable || lambda_corr_hard_max || ar_mode != 3)
      throw std::invalid_argument(
          "profile requires Ratio 3 direct FIX/hold");
    if (fde_max_iter != 1 || doppler_gdop_max != 0 || bias_prior_mode > 2 ||
        imu_grade != "tactical")
      throw std::invalid_argument("unsupported profile option");
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
inline Key xp(int i) { return gtsam::Symbol('x', i); }
inline Key poseKey(int i) { return gtsam::Symbol('P', i); }
inline Key velKey(int i) { return gtsam::Symbol('v', i); }
inline Key biasKey(int i) { return gtsam::Symbol('b', i); }
inline Key ambKey(SF sf, int generation = 0) {
  return gtsam::Symbol('n', std::uint64_t(generation) * 1000000 +
                                sf.first * 10 + sf.second);
}
struct SatelliteState {
  int outage = 0, arc = 0, generation = 0, fix_streak = 0, reset_id = 0,
      fix_flag = 0;
  std::optional<Key> key;
  std::optional<double> held, last_held;
  bool seed_pending = false;
  void activate(double value) {
    if (!std::isfinite(value))
      throw std::invalid_argument("nonfinite hold");
    held = last_held = value;
    seed_pending = false;
  }
  void release(bool seed = true) {
    if (held)
      last_held = held;
    held.reset();
    fix_streak = 0;
    ++reset_id;
    seed_pending = seed && last_held.has_value();
  }
  void clearHold() {
    held.reset();
    last_held.reset();
    fix_streak = 0;
    ++reset_id;
    seed_pending = false;
  }
  void invalidate() {
    key.reset();
    clearHold();
    ++generation;
    ++arc;
    fix_flag = 0;
  }
};
struct SatelliteStates {
  std::map<SF, SatelliteState> track;
  // Python insertion order is significant in the held AR block.
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
  bool finalized = false, cp_suppressed = false;
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
// Model input boundary after receiver QC and satellite propagation. Raw input
// preprocessing is tested separately; a prepared replay is not an end-to-end
// test.
struct SatelliteObservation {
  int sat = 0, system = 0, prn = 0;
  double elevation = 0;
  Vec3 rover_sat = Vec3::Zero(), base_sat = Vec3::Zero(),
       sat_velocity = Vec3::Zero();
  Vec3 pr = Vec3::Zero(), pr_base = Vec3::Zero(), phase = Vec3::Zero(),
       phase_base = Vec3::Zero();
  Vec3 doppler = Vec3::Zero(), snr = Vec3::Zero(), wavelength = Vec3::Zero();
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
inline double range(const Vec3 &satellite, const Vec3 &receiver,
                    Vec3 *los = nullptr) {
  const Vec3 d = satellite - receiver;
  const double r = d.norm();
  if (!(r > 0))
    throw std::invalid_argument("zero satellite range");
  if (los)
    *los = d / r;
  return r + (7.2921151467e-5 / 299792458.) *
                 (satellite.x() * receiver.y() - satellite.y() * receiver.x());
}
inline double ddSigma(const Config &c, bool code, double er, double ej,
                      double dt) {
  if (!c.varerr_enable)
    return (code ? c.sigma_pr : c.sigma_cp) * std::sqrt(2.);
  const double elevation =
      std::max(std::min(er, ej), std::max(1., c.el_mask_deg) * M_PI / 180.);
  const double sinel = std::max(std::sin(elevation), .05),
               fact = code ? c.err_eratio_pr : 1.;
  const double a = fact * c.err_a, b = fact * c.err_b,
               d = 299792458. * c.err_sclkstab * std::abs(dt);
  return std::sqrt(2. * (2. * (a * a + b * b / (sinel * sinel)) + d * d));
}
// The ambiguity keys may be absent (held/held), one (held/float), or two.
// Factor identity belongs to its append slot; key tuples are not unique IDs.
class DdFactor final : public gtsam::NoiseModelFactor {
public:
  Vec3 ref, target, base_ref, base_target, base, lever;
  gtsam::Pose3 ecef_T_nav;
  double observed = 0, offset = 0;
  std::vector<double> coefficients;
  bool carrier = false;
  int ref_sat = 0, target_sat = 0, band = 0;
  DdFactor(Key pose, const std::vector<Key> &nkeys,
           const std::vector<double> &coeff, const SatelliteObservation &r,
           const SatelliteObservation &j, int f, const Vec3 &rb,
           const Vec3 &arm, const gtsam::Pose3 &frame, double measurement,
           double constant, double sigma, bool cp)
      : gtsam::NoiseModelFactor(gtsam::noiseModel::Isotropic::Sigma(1, sigma),
                                keysFor(pose, nkeys)),
        ref(r.rover_sat), target(j.rover_sat), base_ref(r.base_sat),
        base_target(j.base_sat), base(rb), lever(arm), ecef_T_nav(frame),
        observed(measurement), offset(constant), coefficients(coeff),
        carrier(cp), ref_sat(r.sat), target_sat(j.sat), band(f) {
    if (coeff.size() != nkeys.size())
      throw std::invalid_argument("DD ambiguity arity");
  }
  static gtsam::KeyVector keysFor(Key pose, const std::vector<Key> &ns) {
    gtsam::KeyVector keys{pose};
    keys.insert(keys.end(), ns.begin(), ns.end());
    return keys;
  }
  gtsam::Vector unwhitenedError(
      const gtsam::Values &values,
      gtsam::OptionalMatrixVecType jacobians = nullptr) const override {
    const auto p = ecef_T_nav.compose(values.at<gtsam::Pose3>(keys_[0]));
    const Mat3 R = p.rotation().matrix();
    const Vec3 antenna = p.translation() + R * lever;
    Vec3 er, ej;
    const double rr = range(ref, antenna, &er),
                 rj = range(target, antenna, &ej);
    double error = (rr - rj) -
                   (range(base_ref, base) - range(base_target, base)) + offset -
                   observed;
    for (std::size_t i = 0; i < coefficients.size(); ++i)
      error += coefficients[i] * values.at<double>(keys_[i + 1]);
    if (jacobians) {
      jacobians->resize(keys_.size());
      Vec3 h = -(er - ej);
      constexpr double omc = 7.2921151467e-5 / 299792458.;
      h.x() -= omc * (ref.y() - target.y());
      h.y() += omc * (ref.x() - target.x());
      const Vec3 hr = R.transpose() * h;
      gtsam::Matrix H(1, 6);
      H.leftCols(3) = lever.cross(hr).transpose();
      H.rightCols(3) = hr.transpose();
      (*jacobians)[0] = H;
      for (std::size_t i = 0; i < coefficients.size(); ++i)
        (*jacobians)[i + 1] = gtsam::Matrix::Constant(1, 1, coefficients[i]);
    }
    return gtsam::Vector1(error);
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<DdFactor>(*this);
  }
};
class NhcFactor final : public gtsam::NoiseModelFactor {
public:
  Vec3 omega, lever;
  double speed;
  NhcFactor(Key pose, Key velocity, const Vec3 &w, const Vec3 &arm, double sp,
            const Config &cfg)
      : gtsam::NoiseModelFactor(
            gtsam::noiseModel::Diagonal::Sigmas(
                Vec3(1e3, cfg.nhc_sigma_lat, cfg.nhc_sigma_vert)),
            gtsam::KeyVector{pose, velocity}),
        omega(w), lever(arm), speed(sp) {}
  gtsam::Vector
  unwhitenedError(const gtsam::Values &values,
                  gtsam::OptionalMatrixVecType H = nullptr) const override {
    const Mat3 R = values.at<gtsam::Pose3>(keys_[0]).rotation().matrix();
    const Vec3 v = R.transpose() * values.at<gtsam::Vector3>(keys_[1]);
    if (H) {
      H->resize(2);
      (*H)[0] = gtsam::Matrix::Zero(3, 6);
      (*H)[0].leftCols(3) = gtsam::skewSymmetric(v);
      (*H)[1] = R.transpose();
    }
    return v + omega.cross(lever) - Vec3(speed, 0, 0);
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<NhcFactor>(*this);
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
                  const Vec3 &omega, const gtsam::SharedNoiseModel &noise)
      : gtsam::NoiseModelFactor(noise, gtsam::KeyVector{pose, vel}),
        R(rotation), omega_cross_lever(omega.cross(lever)) {
    const Vec3 e = (sat - receiver).normalized(),
               er = (ref - receiver).normalized();
    constexpr double k = 7.2921151467e-5 / 299792458.;
    // Match the pinned SingleDifferenceDopplerFactorArm convention. Its
    // rotation-rate term has the opposite sign to the standalone Doppler
    // factor in that wheel; the oracle checks the actual SD factor.
    h = -(e - er);
    h.x() += k * (sat.y() - ref.y());
    h.y() -= k * (sat.x() - ref.x());
    constant = e.dot(sat_vel) - er.dot(ref_vel) + lam * doppler -
               ref_lam * ref_doppler -
               k * ((sat_vel.x() - ref_vel.x()) * receiver.y() -
                    (sat_vel.y() - ref_vel.y()) * receiver.x());
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
inline auto imuParams(const Config &c) {
  auto p = gtsam::PreintegrationCombinedParams::MakeSharedU(9.81);
  // The frozen Python dependency uses this error chart. Older GTSAM's
  // component-wise IMU error changes the estimator even with identical PIMs.
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
struct IntegratedImu {
  gtsam::PreintegratedCombinedMeasurements pim;
  Vec3 mean_gyro = Vec3::Zero();
  std::size_t count = 0;
  IntegratedImu(const Config &c, const gtsam::imuBias::ConstantBias &b)
      : pim(imuParams(c), b) {}
};
inline IntegratedImu integrateImu(const Config &cfg,
                                  const gtsam::imuBias::ConstantBias &bias,
                                  const std::vector<ImuSample> &samples,
                                  double start, double end) {
  if (samples.empty() || !std::isfinite(start) || !std::isfinite(end) ||
      end <= start)
    throw ImuIntervalError("empty or reversed IMU interval");
  IntegratedImu out(cfg, bias);
  double previous = start;
  for (const auto &s : samples) {
    const double raw = s.gpst - s.predecessor.value_or(previous);
    const double dt = std::nearbyint((s.gpst - previous) * 1e4) / 1e4;
    if (!std::isfinite(raw) || raw <= 0 || raw > 0.100001 ||
        !std::isfinite(dt) || dt <= 0 || dt > 0.1)
      throw ImuIntervalError("duplicate, reversed or uncovered IMU interval");
    if (!s.acc.allFinite() || !s.gyro.allFinite())
      throw ImuIntervalError("nonfinite IMU");
    out.pim.integrateMeasurement(s.acc, s.gyro, dt);
    out.mean_gyro += s.gyro;
    ++out.count;
    previous = s.gpst;
  }
  const double tail = std::nearbyint((end - previous) * 1e4) / 1e4;
  if (tail < 0 || tail > 0.01)
    throw ImuIntervalError("IMU does not cover interval end");
  if (tail > 0)
    out.pim.integrateMeasurement(samples.back().acc, samples.back().gyro, tail);
  out.mean_gyro /= out.count;
  return out;
}
inline gtsam::imuBias::ConstantBias
stationaryBias(const std::vector<ImuSample> &samples, std::size_t count) {
  count = std::min(count, samples.size());
  if (count <= 100)
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
    ba = a - 9.81 * a / a.norm();
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
