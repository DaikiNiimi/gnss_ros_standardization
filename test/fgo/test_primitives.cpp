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
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <iostream>
#include <random>
#include <tuple>
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
  std::vector<ImuSample> imu;
  for (int i = 1; i <= 4; ++i)
    imu.push_back(
        {t + .01 * i, Vec3(0, 0, 9.81), Vec3::Zero(), t + .01 * (i - 1)});
  auto pim = integrateImu(c, {}, imu, t, t + .04);
  require(std::abs(pim.pim.deltaTij() - .04) < 1e-12,
          "week boundary integration");
  require(pim.count == 4, "IMU count");
  throws([&] {
    auto bad = imu;
    bad[1].gpst = bad[0].gpst;
    integrateImu(c, {}, bad, t, t + .04);
  });
  throws([&] {
    auto bad = imu;
    bad[1].gpst = t - .01;
    integrateImu(c, {}, bad, t, t + .04);
  });
  throws([&] {
    auto bad = imu;
    bad[0].predecessor = t - .2;
    integrateImu(c, {}, bad, t, t + .04);
  });
  // The interval end may lie up to one sample interval (kImuMaxSampleInterval)
  // after the last sample, not more.
  const auto intervalError = [&](const std::vector<ImuSample> &samples,
                                 double end) {
    try {
      integrateImu(c, {}, samples, t, end);
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
    const auto held = integrateImu(c, {}, imu50, t, end);
    // GPS times near 1.4e9 s carry 2.4e-7 s rounding; integrateImu rounds the
    // intervals to 0.1 ms.
    require(std::abs(held.pim.deltaTij() - .115) < 1e-12 &&
                std::abs(held.pim.deltaTij() - (end - t)) < 1e-6,
            "50 Hz interval with a 15 ms tail: preintegrated time");
    // Five 20 ms sample intervals and the 15 ms tail.
    gtsam::PreintegratedCombinedMeasurements expected(imuParams(c), {});
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
    while (fixture >> name) {
      std::shared_ptr<gtsam::NoiseModelFactor> f;
      if (name == "nhc")
        f = std::make_shared<NhcFactor>(poseKey(1), velKey(1), omega, lever, 2., c);
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
    gtsam::PreintegratedCombinedMeasurements p(imuParams(c), b);
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
    // marginalWithout against the covariance form: the kept keys' information
    // is the inverse of their block of the full covariance, their mean the kept
    // part of the full mean, and the minimum error is unchanged.
    const Key pose = poseKey(0), vel = velKey(0), amb1 = ambKey({2, 1}),
              amb2 = ambKey({2, 2});
    std::mt19937 rng(20261112);
    std::normal_distribution<double> normal;
    Eigen::MatrixXd A(11, 11);
    for (int i = 0; i < A.size(); ++i)
      A.data()[i] = normal(rng);
    Eigen::MatrixXd G = A * A.transpose() + 11. * Eigen::MatrixXd::Identity(11, 11);
    Eigen::VectorXd g(11);
    for (int i = 0; i < 11; ++i)
      g[i] = normal(rng);
    gtsam::Values point;
    point.insert(pose, gtsam::Pose3(gtsam::Rot3::RzRyRx(.1, -.2, .3),
                                    Vec3(1, 2, 3)));
    point.insert(vel, Vec3(.4, -.5, .6));
    point.insert(amb1, 7.);
    point.insert(amb2, -3.);
    const auto container = [&](const Eigen::MatrixXd &info,
                               const Eigen::VectorXd &linear, double constant) {
      Eigen::MatrixXd augmented(12, 12);
      augmented << info, linear, linear.transpose(), constant;
      const gtsam::HessianFactor h(
          gtsam::KeyVector{pose, vel, amb1, amb2},
          gtsam::SymmetricBlockMatrix(std::vector<std::size_t>{6, 3, 1, 1, 1},
                                      augmented));
      return gtsam::LinearContainerFactor(h, point);
    };
    // Marginal of (info, linear, constant) on its first 9 entries, from the
    // inverse of `info`.
    const auto expected = [](const Eigen::MatrixXd &info,
                             const Eigen::VectorXd &linear, double constant) {
      const Eigen::MatrixXd cov = info.inverse();
      const Eigen::VectorXd mean = cov * linear;
      const Eigen::MatrixXd kept = cov.topLeftCorner(9, 9).inverse();
      const Eigen::VectorXd kept_linear = kept * mean.head(9);
      return std::make_tuple(kept, kept_linear,
                             constant - linear.dot(mean) +
                                 kept_linear.dot(mean.head(9)));
    };
    const auto check = [&](const gtsam::LinearContainerFactor &f,
                           const std::tuple<Eigen::MatrixXd, Eigen::VectorXd,
                                            double> &want,
                           const char *why) {
      const auto out = std::dynamic_pointer_cast<gtsam::LinearContainerFactor>(
          marginalWithout(f, {amb1, amb2}));
      require(bool(out), why);
      require(out->keys() == gtsam::KeyVector{pose, vel}, why);
      require(out->linearizationPoint() &&
                  out->linearizationPoint()->size() == 2 &&
                  out->linearizationPoint()->at<gtsam::Pose3>(pose).equals(
                      point.at<gtsam::Pose3>(pose), 0) &&
                  out->linearizationPoint()->at<Vec3>(vel) == point.at<Vec3>(vel),
              why);
      const gtsam::HessianFactor h(*out->factor());
      const auto &[info, linear, constant] = want;
      const double scale = info.cwiseAbs().maxCoeff();
      require((h.information() - info).cwiseAbs().maxCoeff() < 1e-10 * scale &&
                  (h.linearTerm() - linear).cwiseAbs().maxCoeff() <
                      1e-10 * scale &&
                  std::abs(h.constantTerm() - constant) <
                      1e-10 * std::max(1., std::abs(constant)),
              why);
    };
    const double constant = 5. + g.dot(G.inverse() * g);
    check(container(G, g, constant), expected(G, g, constant),
          "marginal of a full-rank factor");
    // amb2 without information or coupling: the result is the marginal over
    // amb1 alone.
    Eigen::MatrixXd G1 = G;
    Eigen::VectorXd g1 = g;
    G1.row(10).setZero();
    G1.col(10).setZero();
    g1[10] = 0;
    check(container(G1, g1, constant),
          expected(G1.topLeftCorner(10, 10), g1.head(10), constant),
          "marginal with an undetermined dropped direction");
    // Nothing to drop, or nothing to keep: no factor.
    const auto f = container(G, g, constant);
    require(!marginalWithout(f, {ambKey({2, 3})}), "nothing dropped");
    require(!marginalWithout(f, {pose, vel, amb1, amb2}), "nothing kept");
  }
  std::cout << "state, time, factor and IMU checks passed\n";
}
