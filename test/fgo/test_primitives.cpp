// The GTSAM DD factor headers used below; model.hpp keeps RTKLIB's OMGE macro
// away from them whatever the include order.
#include <gtsam/navigation/CarrierPhaseFactor.h>
#include <gtsam/navigation/PseudorangeFactor.h>
#include "model.hpp"
#include "engine.hpp"
#include "ambiguity.hpp"
#include <algorithm>
#include <array>
#include <fstream>
#include <functional>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <iostream>
using namespace gnss_fgo;
void require(bool ok, const char *why) {
  if (!ok)
    throw std::runtime_error(why);
}
template <class F> void throws(F f) {
  bool caught = false;
  try {
    f();
  } catch (const std::exception &) {
    caught = true;
  }
  require(caught, "expected failure");
}
// Independent DD model for the official-factor checks; deliberately does not
// call the official factors or gtsam::gnss::geodist.
constexpr double kOmgeOverC = 7.2921151467e-5 / 299792458.;
// Geometric range plus the Sagnac term OMGE/c (sx ry - sy rx).
double refRange(const Vec3 &sat, const Vec3 &rx) {
  return (sat - rx).norm() + kOmgeOverC * (sat.x() * rx.y() - sat.y() * rx.x());
}
struct DdExpected {
  double error;
  Eigen::Matrix<double, 1, 6> H; // d error / d pose, [rotation, translation]
};
// antenna = (ecef_T_nav * pose).translation() + R lever;
// error = predicted DD range + ambiguity_m - measured DD, where
// ambiguity_m = lam (N_ref - N_target) for carrier and 0 for code.
DdExpected ddReference(const gtsam::Pose3 &ecef_T_nav, const gtsam::Pose3 &pose,
                       const Vec3 &lever, const SatelliteObservation &ref,
                       const SatelliteObservation &tgt, const Vec3 &base,
                       double measured_dd, double ambiguity_m) {
  const gtsam::Pose3 p = ecef_T_nav * pose;
  const Mat3 R = p.rotation().matrix();
  const Vec3 antenna = p.translation() + R * lever;
  DdExpected out;
  out.error = (refRange(ref.rover_sat, antenna) -
               refRange(tgt.rover_sat, antenna)) -
              (refRange(ref.base_sat, base) - refRange(tgt.base_sat, base)) +
              ambiguity_m - measured_dd;
  // Gradient w.r.t. the ECEF antenna: -(LOS_ref - LOS_target) plus Sagnac.
  Vec3 g = -((ref.rover_sat - antenna).normalized() -
             (tgt.rover_sat - antenna).normalized());
  g.x() -= kOmgeOverC * (ref.rover_sat.y() - tgt.rover_sat.y());
  g.y() += kOmgeOverC * (ref.rover_sat.x() - tgt.rover_sat.x());
  // Right perturbation p Exp([w, v]) moves the antenna by R (v - lever x w).
  const Vec3 gb = R.transpose() * g;
  out.H << lever.cross(gb).transpose(), gb.transpose();
  return out;
}
// ECEF of geodetic (lat, lon [deg], h [m]) and the ECEF unit vector of
// (azimuth, elevation [deg]) there, through RTKLIB.
Vec3 geodetic(double lat, double lon, double h) {
  const double pos[3] = {lat * D2R, lon * D2R, h};
  Vec3 r;
  pos2ecef(pos, r.data());
  return r;
}
Vec3 lineOfSight(const Vec3 &receiver, double az, double el) {
  double pos[3];
  ecef2pos(receiver.data(), pos);
  const double enu[3] = {std::sin(az * D2R) * std::cos(el * D2R),
                         std::cos(az * D2R) * std::cos(el * D2R),
                         std::sin(el * D2R)};
  Vec3 e;
  enu2ecef(pos, enu, e.data());
  return e;
}
namespace gnss_fgo {
struct EngineTestAccess {
  // Engine::ddprPosition seeded at the antenna `seed` (ECEF, no lever arm).
  static DdPosition ddpr(Engine &e, const ObservationEpoch &obs,
                         const Vec3 &seed) {
    return e.ddprPosition(
        obs, gtsam::Pose3(gtsam::Rot3(), e.R_.transpose() * (seed - e.base_)),
        Vec3::Zero());
  }
};
} // namespace gnss_fgo
int main() {
  // Receiving the equal-time IMU does not admit a rover ahead of its
  // equal-time base callback; the later IMU is only a trigger, not input data.
  require(!epochReady(1000, std::nullopt), "release without IMU watermark");
  require(!epochReady(1000, 999), "release before IMU coverage");
  require(!epochReady(1000, 1000), "equal-time admission race");
  require(epochReady(1000, 1010), "crossed watermark did not release");
  require(epochReady(1000, 1000, true), "EOF did not drain equal-time tail");
  require(!epochReady(1000, 999, true), "EOF invented missing IMU coverage");
  // An exact integer solution must not be rejected merely because
  // floating-point roundoff reached zero instead of a positive value.
  for (double shift : {0., 1e-10}) {
    const Eigen::VectorXd a = Eigen::VectorXd::Constant(8, shift);
    const Eigen::MatrixXd q = Eigen::MatrixXd::Identity(8, 8) * .001;
    auto result = gnss_fgo::ar::resolve(a, q);
    require(result.accepted && result.kept == 8,
            "zero/near-zero ratio verdict");
  }
  Config c;
  c.validate();
  require(!c.nhc_enable && !c.zupt_enable, "vehicle constraints are off by default");
  throws([&] {
    auto bad = c;
    bad.sigma_cp = -1;
    bad.validate();
  });
  throws([&] {
    auto bad = c;
    bad.fls_lag = 0;
    bad.validate();
  });
  throws([&] {
    auto bad = c;
    bad.ar_mode = 1;
    bad.validate();
  });
  throws([&] {
    auto bad = c;
    bad.nhc_lever = "0,NaN,0";
    bad.validate();
  });
  SatelliteStates states;
  const SF a{2, 0}, b{3, 0};
  auto &st = states.get(a);
  st.key = ambKey(a);
  st.activate(20.25);
  for (int i = 1; i <= 2; ++i) {
    EpochState e;
    e.accepted[a] = st.arc;
    e.visible.insert(a);
    e.finalize(states, true);
    require(st.fix_streak == i, "continuing FIX streak");
    throws([&] { e.finalize(states, true); });
  }
  EpochState partial;
  partial.accepted[b] = states.get(b).arc;
  partial.visible.insert(b);
  partial.finalize(states, true);
  require(st.fix_streak == 0, "unobserved hold is not confirmed");
  require(st.held.has_value(), "unobserved hold retained");
  st.invalidate();
  require(!st.key && !st.held && !st.seed_pending && st.arc == 1,
          "slip invalidation");
  st.activate(1);
  st.release();
  require(st.seed_pending && st.fix_streak == 0, "released seed");
  st.clearHold();
  require(!st.seed_pending && !st.last_held, "reset removes seed");
  const double t = 604800. * 2300 + 604799.98;
  // Any preintegration frame does for the interval checks below.
  const ImuFrame level{Vec3(0, 0, -9.81), Vec3::Zero()};
  std::vector<ImuSample> imu;
  for (int i = 1; i <= 4; ++i)
    imu.push_back(
        {t + .01 * i, Vec3(0, 0, 9.81), Vec3::Zero(), t + .01 * (i - 1)});
  auto pim = integrateImu(c, {}, imu, t, t + .04, level);
  require(std::abs(pim.pim.deltaTij() - .04) < 1e-12,
          "week boundary integration");
  require(pim.count == 4, "IMU count");
  throws([&] {
    auto bad = imu;
    bad[1].gpst = bad[0].gpst;
    integrateImu(c, {}, bad, t, t + .04, level);
  });
  throws([&] {
    auto bad = imu;
    bad[1].gpst = t - .01;
    integrateImu(c, {}, bad, t, t + .04, level);
  });
  throws([&] {
    auto bad = imu;
    bad[0].predecessor = t - .2;
    integrateImu(c, {}, bad, t, t + .04, level);
  });
  // The interval end may lie up to one sample interval (kImuMaxSampleInterval)
  // after the last sample, not more.
  const auto intervalError = [&](const std::vector<ImuSample> &samples,
                                 double end) {
    try {
      integrateImu(c, {}, samples, t, end, level);
    } catch (const ImuIntervalError &) {
      return true;
    }
    return false;
  };
  require(!intervalError(imu, t + .04 + kImuMaxSampleInterval),
          "tail of one sample interval rejected");
  require(intervalError(imu, t + .04 + kImuMaxSampleInterval + 2e-4),
          "tail longer than one sample interval accepted");
  {
    // 50 Hz IMU, interval end 15 ms after the last sample (an epoch released
    // by the next sample): the tail is held at the last sample's value.
    const Vec3 acc(.3, -.2, 9.81), gyro(.01, -.02, .03);
    std::vector<ImuSample> imu50;
    for (int i = 1; i <= 5; ++i)
      imu50.push_back({t + .02 * i, acc, gyro, t + .02 * (i - 1)});
    const double end = t + .1 + .015;
    const auto held = integrateImu(c, {}, imu50, t, end, level);
    // GPS times near 1.4e9 s carry 2.4e-7 s rounding; integrateImu rounds the
    // intervals to 0.1 ms.
    require(std::abs(held.pim.deltaTij() - .115) < 1e-12 &&
                std::abs(held.pim.deltaTij() - (end - t)) < 1e-6,
            "50 Hz interval with a 15 ms tail: preintegrated time");
    // Five 20 ms sample intervals and the 15 ms tail.
    gtsam::PreintegratedCombinedMeasurements expected(imuParams(c, level), {});
    for (int i = 0; i < 5; ++i)
      expected.integrateMeasurement(acc, gyro, .02);
    expected.integrateMeasurement(acc, gyro, .015);
    require(held.count == 5 &&
                held.pim.deltaRij().equals(
                    gtsam::Rot3::Expmap(gyro * .115), 1e-12) &&
                held.pim.deltaRij().equals(expected.deltaRij(), 1e-12) &&
                (held.pim.deltaVij() - expected.deltaVij()).norm() < 1e-12 &&
                (held.pim.deltaPij() - expected.deltaPij()).norm() < 1e-12,
            "50 Hz interval with a 15 ms tail: constant-input deltas");
    require(intervalError(imu50, t + .1 + kImuMaxSampleInterval + 2e-4),
            "50 Hz tail longer than one sample interval accepted");
  }
  {
    // antennaVelocityJacobian against central differences on the Pose3
    // retract and the velocity.
    const gtsam::Pose3 body(gtsam::Rot3::RzRyRx(.3, -.4, 1.2), Vec3(5, -2, 1));
    const Vec3 v(3., -1., .2), omega(.3, -.2, .5), arm(1.2, -.3, .8);
    const auto antennaVelocity = [&](const gtsam::Pose3 &p, const Vec3 &vel) {
      return Vec3(vel + p.rotation().matrix() * omega.cross(arm));
    };
    const auto J = antennaVelocityJacobian(body.rotation().matrix(), omega, arm);
    Eigen::Matrix<double, 3, 9> numeric;
    constexpr double h = 1e-5;
    for (int k = 0; k < 6; ++k) {
      const gtsam::Vector6 d = gtsam::Vector6::Unit(k) * h;
      numeric.col(k) = (antennaVelocity(body.retract(d), v) -
                        antennaVelocity(body.retract(-d), v)) /
                       (2 * h);
    }
    for (int k = 0; k < 3; ++k) {
      const Vec3 d = Vec3::Unit(k) * h;
      numeric.col(6 + k) =
          (antennaVelocity(body, v + d) - antennaVelocity(body, v - d)) / (2 * h);
    }
    require(J.leftCols<3>().cwiseAbs().maxCoeff() > .1,
            "lever-arm rotation term not exercised");
    require((J - numeric).cwiseAbs().maxCoeff() <=
                1e-6 * J.cwiseAbs().maxCoeff(),
            "antenna velocity Jacobian");
  }
  // Only the still block before the vehicle moves off defines the bias.
  std::vector<ImuSample> start_still;
  for (int i = 0; i < 1000; ++i) {
    const bool moving = i >= 500, odd = i % 2;
    start_still.push_back(
        {t + .01 * i,
         Vec3(moving ? (odd ? 2. : -2.) : 0., 0, 9.81),
         Vec3(0, 0, -3e-4 + (moving ? .1 + (odd ? .05 : -.05) : 0.)),
         {}});
  }
  require(leadingStillSamples(start_still, 1000) >= 400 &&
              leadingStillSamples(start_still, 1000) <= 500,
          "still block ends where motion starts");
  require(std::abs(stationaryBias(start_still, 1000, 9.81).gyroscope().z() +
                   3e-4) < 1e-9,
          "gyro bias excludes the motion after the still block");
  std::vector<ImuSample> start_moving(start_still.begin() + 500,
                                      start_still.end());
  require(stationaryBias(start_moving, 500, 9.81).gyroscope().isZero(),
          "no still block gives zero bias");
  SatelliteObservation r, j;
  r.sat = 2;
  j.sat = 3;
  r.rover_sat = Vec3(2.1e7, 1.2e7, 1.8e7);
  j.rover_sat = Vec3(1.1e7, -2.2e7, 1.7e7);
  r.base_sat = r.rover_sat + Vec3(20, 30, -10);
  j.base_sat = j.rover_sat + Vec3(-10, 40, 20);
  const Vec3 rb(-3950000, 3350000, 3700000), lever(.2, -.1, .6);
  const gtsam::Pose3 frame(gtsam::Rot3::RzRyRx(.2, -.3, .4), rb);
  const gtsam::Pose3 pose(gtsam::Rot3::RzRyRx(-.1, .15, .3), Vec3(20, 10, 3));
  // The official DD factors as Engine::buildDd builds them: the four raw
  // observations in metres in the order (rover ref, base ref, rover target,
  // base target); the carrier factor adds the ambiguity keys N_ref, N_target
  // in cycles and the wavelength.
  const auto makeDd = [&](bool carrier, const SatelliteObservation &ref,
                          const SatelliteObservation &tgt,
                          const std::array<double, 4> &raw, double lam,
                          double sigma) -> gtsam::NoiseModelFactor::shared_ptr {
    auto model = gtsam::noiseModel::Isotropic::Sigma(1, sigma);
    if (carrier)
      return std::make_shared<gtsam::DoubleDifferenceCarrierPhaseFactorArm>(
          poseKey(1), ambKey({ref.sat, 0}), ambKey({tgt.sat, 0}), raw[0], raw[1],
          raw[2], raw[3], ref.rover_sat, tgt.rover_sat, ref.base_sat,
          tgt.base_sat, rb, lam, lever, frame, model);
    return std::make_shared<gtsam::DoubleDifferencePseudorangeFactorArm>(
        poseKey(1), raw[0], raw[1], raw[2], raw[3], ref.rover_sat, tgt.rover_sat,
        ref.base_sat, tgt.base_sat, rb, lever, frame, model);
  };
  // Analytic Jacobians against numerical derivatives: the pose (Pose3 right
  // tangent [rotation, translation]) and, for carrier, both ambiguities.
  for (bool carrier : {false, true}) {
    const auto factor = makeDd(carrier, r, j, {20., 5., 8., 5.}, .19, .01);
    gtsam::Values values;
    values.insert(poseKey(1), pose);
    if (carrier) {
      values.insert(ambKey({r.sat, 0}), 21.25);
      values.insert(ambKey({j.sat, 0}), -4.5);
    }
    std::vector<gtsam::Matrix> H(factor->size());
    factor->unwhitenedError(values, &H);
    std::function<gtsam::Vector(const gtsam::Pose3 &)> f = [&](const auto &p) {
      auto v = values;
      v.update(poseKey(1), p);
      return factor->unwhitenedError(v);
    };
    const auto numeric =
        gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(f, pose,
                                                                  1e-3);
    // Subtracting ~2e7 m ranges at a 1 mm step has an ~1e-5 derivative
    // roundoff floor; the reference check below is tighter.
    require((H[0] - numeric).cwiseAbs().maxCoeff() < 1e-5, "DD pose Jacobian");
    for (std::size_t k = 1; k < factor->size(); ++k) {
      const Key key = factor->keys()[k];
      std::function<gtsam::Vector(const double &)> g = [&](const double &n) {
        auto v = values;
        v.update(key, n);
        return factor->unwhitenedError(v);
      };
      // The error is linear in N and the range terms are bit-identical
      // between the two evaluations, so a 1-cycle step is exact to roundoff.
      const auto dn = gtsam::numericalDerivative11<gtsam::Vector, double>(
          g, values.at<double>(key), 1.);
      require((H[k] - dn).cwiseAbs().maxCoeff() < 1e-9 &&
                  std::abs(H[k](0, 0) - (k == 1 ? .19 : -.19)) < 1e-12,
              "DD ambiguity Jacobian");
    }
  }
  require(ddSigma(c, true, .4, .5, .8) > ddSigma(c, true, .4, .5, 0.),
          "base age term");
  {
    // Reference values from an external GTSAM computation. A row is
    // "arity error J..." with the Jacobian in key order. Arity 2 is the
    // official carrier DD (keys pose, N_ref, N_target; lam .19; N = 21.25 on
    // both keys) and -1 the official code DD, both with raw observations
    // (20, 5, 8, 5) m. Rows of arity 0 and 1 (a held-ambiguity factor with a
    // constant offset, which the official factors cannot express) are skipped.
    std::ifstream fixture(std::string(FGO_FIXTURE_DIR) +
                          "/factor-fixture.txt");
    require(bool(fixture), "DD oracle missing");
    int arity, compared = 0, skipped = 0;
    double expected;
    while (fixture >> arity >> expected) {
      if (arity == 0 || arity == 1) {
        for (int i = 0; i < 6 + arity; ++i)
          fixture >> expected;
        ++skipped;
        continue;
      }
      require(arity == 2 || arity == -1, "unknown DD oracle row");
      const bool carrier = arity == 2;
      const auto factor = makeDd(carrier, r, j, {20., 5., 8., 5.}, .19, .01);
      gtsam::Values values;
      values.insert(poseKey(1), pose);
      if (carrier) {
        values.insert(ambKey({r.sat, 0}), 21.25);
        values.insert(ambKey({j.sat, 0}), 21.25);
      }
      std::vector<gtsam::Matrix> H(factor->size());
      const auto error = factor->unwhitenedError(values, &H);
      require(std::abs(error[0] - expected) < 1e-8,
              "DD residual reference");
      for (const auto &matrix : H)
        for (int row = 0; row < matrix.rows(); ++row)
          for (int col = 0; col < matrix.cols(); ++col) {
            fixture >> expected;
            require(std::abs(matrix(row, col) - expected) < 1e-12,
                    "DD Jacobian reference");
          }
      ++compared;
    }
    require(compared == 2 && skipped == 2, "DD oracle rows");
  }
  {
    // Expected values of the official DD factors against ddReference(), an
    // independent implementation of the same model, over realistic geometry.
    const Vec3 up = rb.normalized(),
               east = Vec3(-rb.y(), rb.x(), 0).normalized(),
               north = up.cross(east);
    const double lam = 299792458. / 1575.42e6;
    // GPS-like satellites (orbit radius 2.656e7 m, 2.0-2.4e7 m ranges) and
    // their positions at the older base epoch (~3.9 km/s, 0.8 s earlier).
    std::vector<SatelliteObservation> sv(4);
    const double dir[4][2] = {{0, 0}, {.4, .2}, {-.3, .35}, {.1, -.45}};
    const double clk_sat[4] = {1.2e-4, -3.4e-4, .5e-4, 2.2e-4};
    const double n_rov[4] = {1.1e6 + 3, -7.3e5 + 11, 2.4e6 - 5, -4.1e5 + 2};
    const double n_base[4] = {9.8e5 + 7, -6.6e5 - 4, 2.2e6 + 9, -3.7e5 - 1};
    for (int s = 0; s < 4; ++s) {
      sv[s].sat = 10 + s;
      sv[s].rover_sat =
          2.656e7 * (up + dir[s][0] * east + dir[s][1] * north).normalized();
      const Vec3 v =
          3874. * sv[s].rover_sat.cross(Vec3::UnitZ() + .3 * east).normalized();
      sv[s].base_sat = sv[s].rover_sat - .8 * v;
    }
    const double clk_rov = 299792458. * 2.3e-4, clk_base = 299792458. * -1.1e-4;
    const std::vector<gtsam::Pose3> poses{
        pose,
        gtsam::Pose3(gtsam::Rot3::RzRyRx(.3, -.05, 2.1), Vec3(-350, 1200, 15)),
        gtsam::Pose3(gtsam::Rot3::RzRyRx(-.2, .1, -1.4), Vec3(5000, -2000, 40))};
    const std::vector<std::pair<int, int>> pairs{{0, 1}, {0, 2}, {3, 1}, {2, 3}};
    double max_err = 0, max_jac = 0, max_whitened = 0, max_obs = 0,
           min_res = 1e300;
    for (const auto &nav_pose : poses)
      for (const auto &[ri, ji] : pairs) {
        const auto &r = sv[ri], &j = sv[ji];
        // Raw observations: truth antenna 1.5 m away from the evaluated pose,
        // receiver/satellite clocks of tens of km, integer phase ambiguities.
        const auto ecef = frame.compose(nav_pose);
        const Vec3 truth = ecef.translation() + ecef.rotation().matrix() * lever +
                           Vec3(.8, -.5, 1.2);
        auto raw = [&](int s, bool rover, bool carrier) {
          const double rho = rover ? refRange(sv[s].rover_sat, truth)
                                   : refRange(sv[s].base_sat, rb);
          const double clk = (rover ? clk_rov : clk_base) -
                             299792458. * clk_sat[s] + (rover ? .37 * s : 0.);
          return rho + clk + (carrier ? lam * (rover ? n_rov[s] : n_base[s]) : 0.);
        };
        const double sigma = .3 * std::sqrt(2.);
        for (bool carrier : {false, true}) {
          const std::array<double, 4> obs{
              raw(ri, true, carrier), raw(ri, false, carrier),
              raw(ji, true, carrier), raw(ji, false, carrier)};
          max_obs = std::max({max_obs, std::abs(obs[0]), std::abs(obs[2])});
          const auto official = makeDd(carrier, r, j, obs, lam, sigma);
          gtsam::Values values;
          values.insert(poseKey(1), nav_pose);
          double ambiguity = 0;
          gtsam::KeyVector keys{poseKey(1)};
          if (carrier) {
            // Float ambiguities near, but not at, the true DD integer.
            const double nr = n_rov[ri] - n_base[ri] + .3,
                         nj = n_rov[ji] - n_base[ji] - .2;
            keys.push_back(ambKey({r.sat, 0}));
            keys.push_back(ambKey({j.sat, 0}));
            values.insert(keys[1], nr);
            values.insert(keys[2], nj);
            ambiguity = lam * (nr - nj);
          }
          require(official->keys() == keys, "official DD key order");
          require(official->noiseModel()->dim() == 1 &&
                      std::abs(official->noiseModel()->sigmas()(0) - sigma) <
                          1e-15,
                  "official DD noise model");
          const auto expected =
              ddReference(frame, nav_pose, lever, r, j, rb,
                          (obs[0] - obs[2]) - (obs[1] - obs[3]), ambiguity);
          std::vector<gtsam::Matrix> H(official->size());
          const auto e = official->unwhitenedError(values, &H);
          min_res = std::min(min_res, std::abs(expected.error));
          max_err = std::max(max_err, std::abs(e(0) - expected.error));
          max_whitened =
              std::max(max_whitened, std::abs(official->whitenedError(values)(0) -
                                              expected.error / sigma));
          std::vector<gtsam::Matrix> Hx{expected.H};
          if (carrier) {
            Hx.push_back(gtsam::Matrix::Constant(1, 1, lam));
            Hx.push_back(gtsam::Matrix::Constant(1, 1, -lam));
          }
          require(H.size() == Hx.size(), "official DD Jacobian count");
          for (std::size_t k = 0; k < H.size(); ++k) {
            require(H[k].rows() == 1 && H[k].cols() == Hx[k].cols(),
                    "official DD Jacobian shape");
            for (int col = 0; col < H[k].cols(); ++col)
              max_jac = std::max(max_jac,
                                 std::abs(H[k](0, col) - Hx[k](0, col)) /
                                     std::max(1., std::abs(Hx[k](0, col))));
          }
        }
      }
    std::cout.precision(3);
    std::cout << "official DD vs reference: max |error diff| " << max_err
              << " m, max whitened diff " << max_whitened
              << ", max Jacobian diff " << max_jac << " (raw obs up to "
              << max_obs << " m, min |residual| " << min_res << " m)\n";
    require(min_res > 1e-3, "reference cases exercise a non-zero residual");
    require(max_err <= 1e-6, "official DD residual");
    require(max_jac <= 1e-9, "official DD Jacobian");
  }
  {
    std::ifstream fixture(std::string(FGO_FIXTURE_DIR) +
                          "/motion-factor-fixture.txt");
    require(bool(fixture), "motion oracle missing");
    std::string name;
    double expected;
    const Vec3 omega(.01, -.02, .03);
    gtsam::Values values;
    values.insert(poseKey(1), pose);
    values.insert(velKey(1), Vec3(2, -.1, .3));
    // At zero mounting pitch the NHC with the mounting pitch is the plain
    // constraint of the oracle (residual, pose and velocity Jacobians); its
    // angle Jacobian is checked numerically below.
    values.insert(boreKey(), 0.);
    while (fixture >> name) {
      std::shared_ptr<gtsam::NoiseModelFactor> f;
      if (name == "nhc")
        f = std::make_shared<NhcBoreFactor>(poseKey(1), velKey(1), boreKey(),
                                            omega, lever, 2., c.nhc_sigma_lat,
                                            c.nhc_sigma_vert);
      else if (name == "gps")
        f = std::make_shared<gtsam::GPSFactorArm>(
            poseKey(1), Vec3(19, 11, 2), lever, gtsam::noiseModel::Unit::Create(3));
      else
        f = std::make_shared<SdDopplerFactor>(
            poseKey(1), velKey(1), -1500., -1200., .19, .24, j.rover_sat,
            Vec3(100, 200, 300), r.rover_sat, Vec3(-200, 100, 400), rb, lever,
            frame.rotation().matrix(), omega,
            gtsam::noiseModel::Unit::Create(1));
      std::vector<gtsam::Matrix> H(f->keys().size());
      auto error = f->unwhitenedError(values, &H);
      if (name == "nhc")
        H.resize(2);
      for (double e : error) {
        fixture >> expected;
        if (std::abs(e - expected) >= 1e-10)
          std::cerr << name << " residual " << e << " expected " << expected
                    << "\n";
        require(std::abs(e - expected) < 1e-10,
                "motion residual reference");
      }
      for (int row = 0; row < error.size(); ++row)
        for (auto &mat : H)
          for (int col = 0; col < mat.cols(); ++col) {
            fixture >> expected;
            if (std::abs(mat(row, col) - expected) >= 1e-12)
              std::cerr << name << " row " << row << " col " << col
                        << " actual " << mat(row, col) << " expected "
                        << expected << "\n";
            require(std::abs(mat(row, col) - expected) < 1e-12,
                    "motion Jacobian reference");
          }
    }
    // Only the difference of the two satellite clock drifts shifts the residual.
    const auto dopplerError = [&](double drift, double ref_drift) {
      return SdDopplerFactor(poseKey(1), velKey(1), -1500., -1200., .19, .24,
                             j.rover_sat, Vec3(100, 200, 300), r.rover_sat,
                             Vec3(-200, 100, 400), rb, lever,
                             frame.rotation().matrix(), omega,
                             gtsam::noiseModel::Unit::Create(1), drift,
                             ref_drift)
          .unwhitenedError(values)(0);
    };
    require(std::abs(dopplerError(3e-11, 1e-11) - dopplerError(0, 0) +
                     299792458. * 2e-11) < 1e-9,
            "Doppler residual removes the satellite clock drift difference");
    require(std::abs(dopplerError(2e-11, 2e-11) - dopplerError(0, 0)) < 1e-9,
            "a common satellite clock drift cancels");
  }
  {
    std::ifstream fixture(std::string(FGO_FIXTURE_DIR) + "/imu-fixture.txt");
    require(bool(fixture), "IMU oracle missing");
    gtsam::imuBias::ConstantBias b(Vec3(.01, .02, -.03),
                                   Vec3(.001, -.002, .003));
    // The oracle was generated with these noise densities and a constant
    // 9.81 m/s^2 gravity in a non-rotating frame; it checks the
    // preintegration arithmetic, not the defaults.
    Config oracle = c;
    oracle.accel_noise = .00284;
    oracle.gyro_noise = .00401;
    oracle.gyro_bias_sigma = 9.7e-6;
    gtsam::PreintegratedCombinedMeasurements p(
        imuParams(oracle, ImuFrame{Vec3(0, 0, -9.81), Vec3::Zero()}), b);
    for (int i = 0; i < 20; ++i)
      p.integrateMeasurement(Vec3(.1 + .01 * i, .2 - .003 * i, 9.8),
                             Vec3(.02, .01 + .001 * i, -.03), .01);
    for (const gtsam::Matrix &m :
         std::vector<gtsam::Matrix>{p.deltaRij().matrix(), p.deltaPij(),
                                    p.deltaVij(), p.preintMeasCov()}) {
      double max_diff = 0;
      for (int row = 0; row < m.rows(); ++row)
        for (int col = 0; col < m.cols(); ++col) {
          double e;
          fixture >> e;
          max_diff = std::max(max_diff, std::abs(m(row, col) - e));
        }
      if (max_diff > 1e-12)
        std::cerr << "IMU matrix " << m.rows() << "x" << m.cols()
                  << " maximum difference " << max_diff << "\n";
      require(max_diff < 1e-12, "IMU propagation/covariance reference");
    }
    gtsam::CombinedImuFactor factor(poseKey(0), velKey(0), poseKey(1), velKey(1),
                                    biasKey(0), biasKey(1), p);
    gtsam::Values values;
    values.insert(poseKey(0), pose);
    values.insert(velKey(0), Vec3(2, -.1, .3));
    values.insert(biasKey(0), b);
    values.insert(poseKey(1), gtsam::Pose3(gtsam::Rot3::RzRyRx(-.09, .16, .31),
                                      Vec3(20.4, 10.1, 3.02)));
    values.insert(velKey(1), Vec3(2.1, -.09, .31));
    values.insert(biasKey(1), b);
    auto jf = std::dynamic_pointer_cast<gtsam::JacobianFactor>(
        factor.linearize(values));
    {
      std::ofstream diagnostic("combined-native.txt");
      diagnostic.precision(17);
      diagnostic << jf->getA() << "\n" << jf->getb().transpose() << "\n";
    }
    std::ifstream linear(std::string(FGO_FIXTURE_DIR) +
                         "/combined-imu-fixture.txt");
    require(bool(linear), "Combined IMU oracle missing");
    for (const gtsam::Matrix &m :
         std::vector<gtsam::Matrix>{jf->getA(), jf->getb()}) {
      double max_relative = 0;
      for (int row = 0; row < m.rows(); ++row)
        for (int col = 0; col < m.cols(); ++col) {
          double e;
          linear >> e;
          max_relative = std::max(max_relative, std::abs(m(row, col) - e) /
                                                    std::max(1., std::abs(e)));
        }
      if (max_relative > 1e-10)
        std::cerr << "Combined IMU relative difference " << max_relative
                  << "\n";
      require(max_relative < 1e-10, "Combined IMU factor reference");
    }
  }
  {
    // hydrostaticDelay is RTKLIB's tropmodel (zenith, humidity 0) times
    // tropmapf at the satellite's azimuth/elevation (rtkpos.c zdres).
    const double t = 1.4e9;
    gtime_t time = gpst2time(0, 0);
    time.time += time_t(t);
    const double zenith[] = {0., 90. * D2R};
    std::map<double, double> at40, at105;
    for (const double h : {40., 105.})
      for (const double el : {15., 30., 70.}) {
        const Vec3 rx = geodetic(35.1, 136.9, h);
        const Vec3 sat = rx + 2.2e7 * lineOfSight(rx, 120., el);
        const double pos[3] = {35.1 * D2R, 136.9 * D2R, h},
                     azel[2] = {120. * D2R, el * D2R};
        const double expected = tropmodel(time, pos, zenith, 0.) *
                                tropmapf(time, pos, azel, nullptr);
        const double delay = hydrostaticDelay(rx, sat, t);
        require(expected > 2. && std::abs(delay - expected) < 1e-6,
                "hydrostatic delay is RTKLIB's tropmodel * tropmapf");
        (h == 40. ? at40 : at105)[el] = delay;
      }
    // 65 m of height at 15 deg elevation: about 1.8 cm zenith times 3.8.
    const double low = at40[15.] - at105[15.];
    require(low > .05 && low < .08, "hydrostatic delay height difference");
    require(at40[15.] > at40[30.] && at40[30.] > at40[70.],
            "hydrostatic delay decreases with elevation");
    const Vec3 rx = geodetic(35.1, 136.9, 40.);
    require(hydrostaticDelay(rx, rx + 2.2e7 * lineOfSight(rx, 0., -5.), t) == 0,
            "no hydrostatic delay below the horizon");
    const Vec3 high = geodetic(35.1, 136.9, 2e4);
    require(hydrostaticDelay(high, high + 2.2e7 * lineOfSight(high, 0., 40.),
                             t) == 0,
            "no hydrostatic delay out of the model's height range");
  }
  {
    // Code DDs of a rover 65 m below the base (7 km baseline) whose
    // pseudoranges contain each receiver's hydrostatic delay: ddprPosition
    // removes it and recovers the rover. Without the delay in the
    // observations the same subtraction moves the solution by centimetres,
    // so the tolerance resolves the correction.
    const double t = 1.4e9;
    const Vec3 base = geodetic(35.1, 136.9, 105.),
               rover = geodetic(35.145, 136.966, 40.);
    const double sky[8][2] = {{0, 80},   {45, 50},  {100, 30}, {160, 15},
                              {210, 40}, {260, 20}, {300, 60}, {330, 25}};
    const auto observations = [&](bool delayed) {
      ObservationEpoch obs;
      obs.gpst = obs.base_gpst = t;
      for (int i = 0; i < 8; ++i) {
        SatelliteObservation s;
        s.sat = s.prn = i + 1;
        s.system = 0;
        s.elevation = sky[i][1] * D2R;
        s.rover_sat = s.base_sat =
            base + 2.02e7 * lineOfSight(base, sky[i][0], sky[i][1]);
        s.pr[0] = range(s.rover_sat, rover) +
                  (delayed ? hydrostaticDelay(rover, s.rover_sat, t) : 0.);
        s.pr_base[0] = range(s.base_sat, base) +
                       (delayed ? hydrostaticDelay(base, s.base_sat, t) : 0.);
        obs.sats.push_back(s);
      }
      return obs;
    };
    // Seeded 3.6 m off horizontally (the delays are taken at the seed).
    const Vec3 seed = rover + 3. * lineOfSight(rover, 90., 0.) -
                      2. * lineOfSight(rover, 0., 0.);
    for (const bool delayed : {true, false}) {
      Engine e(Config{}, seed, base, Vec3::Zero());
      const auto dd = EngineTestAccess::ddpr(e, observations(delayed), seed);
      require(dd.position.has_value() && dd.active == 7,
              "DD code solution with hydrostatic delays");
      const double error = (*dd.position - rover).norm();
      std::cout << "DD code solution error, observations "
                << (delayed ? "with" : "without")
                << " hydrostatic delay: " << error << " m\n";
      require(delayed ? error < 1e-3 : error > .01,
              "DD code solution does not remove the hydrostatic delay");
    }
  }
  {
    // Earth rotation rate in the ENU frame of latitude lat: W (0, cos lat,
    // sin lat), from the frame's ENU-to-ECEF rotation.
    const auto enuToEcef = [](double lat, double lon) {
      Mat3 R;
      R << -std::sin(lon), -std::sin(lat) * std::cos(lon),
          std::cos(lat) * std::cos(lon), std::cos(lon),
          -std::sin(lat) * std::sin(lon), std::cos(lat) * std::sin(lon), 0,
          std::cos(lat), std::sin(lat);
      return R;
    };
    for (const double lat_deg : {0., 35.7, -60.})
      for (const double lon_deg : {0., 139.7}) {
        const double lat = lat_deg * M_PI / 180., lon = lon_deg * M_PI / 180.;
        const Vec3 expected = kEarthRate * Vec3(0, std::cos(lat), std::sin(lat));
        require((earthRate(enuToEcef(lat, lon)) - expected).norm() < 1e-18,
                "Earth rate in ENU");
      }
    // WGS84 normal gravity: the equator and pole values of the ellipsoid
    // (NIMA TR8350.2), Somigliana's formula at 45 deg, and the free-air
    // decrease with height (first order, 2 g / a (1 + f + m - 2 f sin^2)).
    require(std::abs(normalGravity(0., 0.) - 9.7803253359) < 1e-10 &&
                std::abs(normalGravity(M_PI / 2, 0.) - 9.8321849378) < 1e-9,
            "normal gravity at the equator and the pole");
    const double s2 = .5, e2 = 6.69437999014e-3, f = 1. / 298.257223563,
                 m = .00344978650684;
    const double g45 = 9.7803253359 * (1. + .00193185265241 * s2) /
                       std::sqrt(1. - e2 * s2);
    require(std::abs(normalGravity(M_PI / 4, 0.) - g45) < 1e-9,
            "normal gravity at 45 deg");
    const double h = 1000., g45h = g45 * (1. - 2. / 6378137. *
                                                   (1. + f + m - 2. * f * s2) * h);
    require(std::abs(normalGravity(M_PI / 4, h) - g45h) < 1e-6 &&
                normalGravity(M_PI / 4, 0.) - normalGravity(M_PI / 4, h) > 3e-3,
            "normal gravity 1000 m above the ellipsoid");
    // The gravity vector is -g along the local up: (0, 0, -g) in the ENU
    // frame of the point itself.
    for (const double lat_deg : {0., 35.7, 89.}) {
      const double lat = lat_deg * M_PI / 180., lon = 2.4, hh = 120.;
      double pos[3] = {lat, lon, hh}, ecef[3];
      pos2ecef(pos, ecef);
      const Vec3 g = localGravity(Vec3(ecef[0], ecef[1], ecef[2]),
                                  enuToEcef(lat, lon));
      require((g - Vec3(0, 0, -normalGravity(lat, hh))).norm() < 1e-9,
              "local gravity vector");
    }
  }
  {
    // NhcBoreFactor Jacobians against central differences (pose retract,
    // velocity, mounting pitch) at a non-zero pitch and a turning body.
    const NhcBoreFactor f(poseKey(1), velKey(1), boreKey(), Vec3(.02, -.05, .3),
                          Vec3(1.2, -.4, .7), 9., .3, .2);
    gtsam::Values values;
    values.insert(poseKey(1), gtsam::Pose3(gtsam::Rot3::RzRyRx(.05, -.08, 1.1),
                                           Vec3(3, -2, 1)));
    values.insert(velKey(1), Vec3(4.5, 7.2, -.3));
    values.insert(boreKey(), -.06);
    std::vector<gtsam::Matrix> H(3);
    f.unwhitenedError(values, &H);
    constexpr double step = 1e-6;
    for (std::size_t key = 0; key < 3; ++key) {
      const int dim = key == 0 ? 6 : key == 1 ? 3 : 1;
      gtsam::Matrix numeric(3, dim);
      for (int k = 0; k < dim; ++k) {
        gtsam::Values plus = values, minus = values;
        if (key == 0) {
          const gtsam::Vector6 d = gtsam::Vector6::Unit(k) * step;
          plus.update(poseKey(1), values.at<gtsam::Pose3>(poseKey(1)).retract(d));
          minus.update(poseKey(1),
                       values.at<gtsam::Pose3>(poseKey(1)).retract(-d));
        } else if (key == 1) {
          const Vec3 d = Vec3::Unit(k) * step;
          plus.update(velKey(1), Vec3(values.at<Vec3>(velKey(1)) + d));
          minus.update(velKey(1), Vec3(values.at<Vec3>(velKey(1)) - d));
        } else {
          plus.update(boreKey(), values.at<double>(boreKey()) + step);
          minus.update(boreKey(), values.at<double>(boreKey()) - step);
        }
        numeric.col(k) =
            (f.unwhitenedError(plus) - f.unwhitenedError(minus)) / (2 * step);
      }
      const double scale = std::max(1., H[key].cwiseAbs().maxCoeff());
      require(H[key].rows() == 3 && H[key].cols() == dim &&
                  (H[key] - numeric).cwiseAbs().maxCoeff() <= 1e-6 * scale,
              "NhcBoreFactor Jacobian");
    }
    require(H[2].cwiseAbs().maxCoeff() > 1., "mounting-pitch Jacobian exercised");
  }
  std::cout << "state, time, factor and IMU checks passed\n";
}
