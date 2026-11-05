#include "model.hpp"
#include "engine.hpp"
#include "ambiguity.hpp"
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
int main() {
  // B18: receiving the equal-time IMU does not admit a rover ahead of its
  // equal-time base callback; the later IMU is only a trigger, not input data.
  require(!epochReady(1000, std::nullopt), "release without IMU watermark");
  require(!epochReady(1000, 999), "release before IMU coverage");
  require(!epochReady(1000, 1000), "equal-time admission race");
  require(epochReady(1000, 1010), "crossed watermark did not release");
  require(epochReady(1000, 1000, true), "EOF did not drain equal-time tail");
  require(!epochReady(1000, 999, true), "EOF invented missing IMU coverage");
  // B17: an exact partial integer solution must not be rejected merely
  // because floating-point roundoff reached zero instead of a positive value.
  for (double shift : {0., 1e-10}) {
    Eigen::VectorXd a = Eigen::VectorXd::Constant(29, shift);
    a.head(5).setConstant(.49);
    Eigen::MatrixXd q = Eigen::MatrixXd::Identity(29, 29) * .001;
    q.diagonal().head(5).setConstant(4.);
    auto result = gnss_fgo::ar::resolve(a, q, false, true);
    require(result.accepted && result.kept == 24,
            "partial zero/near-zero ratio verdict");
    for (int i = 0; i < 29; ++i)
      require(bool(result.exact_rows[i]) == (i >= 5),
              "partial zero/near-zero hold permission");
  }
  Config c;
  c.validate();
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
    bad.ar_thresar = 2;
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
  throws([&] { integrateImu(c, {}, imu, t, t + .06); });
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
  for (int arity = 0; arity <= 2; ++arity) {
    std::vector<Key> keys;
    std::vector<double> coeff;
    for (int i = 0; i < arity; ++i) {
      keys.push_back(ambKey({i + 2, 0}));
      coeff.push_back(i ? -.19 : .19);
    }
    DdFactor factor(xp(1), keys, coeff, r, j, 0, rb, lever, frame, 12., 3., .01,
                    true);
    gtsam::Values values;
    values.insert(xp(1), pose);
    for (auto k : keys)
      values.insert(k, 21.25);
    std::vector<gtsam::Matrix> H;
    factor.unwhitenedError(values, &H);
    std::function<gtsam::Vector(const gtsam::Pose3 &)> f = [&](const auto &p) {
      auto v = values;
      v.update(xp(1), p);
      return factor.unwhitenedError(v);
    };
    const auto numeric =
        gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(f, pose,
                                                                  1e-3);
    // Subtracting ~2e7 m ranges at a 1 mm step has an ~1e-5 derivative
    // roundoff floor. Analytic cross-language Jacobians have a tighter test.
    require((H[0] - numeric).cwiseAbs().maxCoeff() < 1e-5, "DD pose Jacobian");
  }
  require(ddSigma(c, true, .4, .5, .8) > ddSigma(c, true, .4, .5, 0.),
          "base age term");
  {
    std::ifstream fixture(std::string(FGO_FIXTURE_DIR) +
                          "/factor-fixture.txt");
    require(bool(fixture), "DD oracle missing");
    int arity;
    double expected;
    while (fixture >> arity >> expected) {
      std::vector<Key> keys;
      std::vector<double> coeff;
      for (int i = 0; i < arity; ++i) {
        keys.push_back(ambKey({i + 2, 0}));
        coeff.push_back(i ? -.19 : .19);
      }
      DdFactor factor(xp(1), keys, coeff, r, j, 0, rb, lever, frame, 12.,
                      arity == 0 || arity == 1 ? 3. : 0., .01, arity != -1);
      gtsam::Values values;
      values.insert(xp(1), pose);
      for (auto k : keys)
        values.insert(k, 21.25);
      std::vector<gtsam::Matrix> H;
      const auto error = factor.unwhitenedError(values, &H);
      require(std::abs(error[0] - expected) < 1e-8,
              "Python DD residual parity");
      for (const auto &matrix : H)
        for (int row = 0; row < matrix.rows(); ++row)
          for (int col = 0; col < matrix.cols(); ++col) {
            fixture >> expected;
            require(std::abs(matrix(row, col) - expected) < 1e-12,
                    "Python DD Jacobian parity");
          }
    }
  }
  {
    std::ifstream fixture(std::string(FGO_FIXTURE_DIR) +
                          "/motion-factor-fixture.txt");
    require(bool(fixture), "motion oracle missing");
    std::string name;
    double expected;
    const Vec3 omega(.01, -.02, .03);
    gtsam::Values values;
    values.insert(xp(1), pose);
    values.insert(velKey(1), Vec3(2, -.1, .3));
    while (fixture >> name) {
      std::shared_ptr<gtsam::NoiseModelFactor> f;
      if (name == "nhc")
        f = std::make_shared<NhcFactor>(xp(1), velKey(1), omega, lever, 2., c);
      else if (name == "gps")
        f = std::make_shared<gtsam::GPSFactorArm>(
            xp(1), Vec3(19, 11, 2), lever, gtsam::noiseModel::Unit::Create(3));
      else
        f = std::make_shared<SdDopplerFactor>(
            xp(1), velKey(1), -1500., -1200., .19, .24, j.rover_sat,
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
                "Python motion residual parity");
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
                    "Python motion Jacobian parity");
          }
    }
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
      require(max_diff < 1e-12, "Python IMU propagation/covariance parity");
    }
    gtsam::CombinedImuFactor factor(xp(0), velKey(0), xp(1), velKey(1),
                                    biasKey(0), biasKey(1), p);
    gtsam::Values values;
    values.insert(xp(0), pose);
    values.insert(velKey(0), Vec3(2, -.1, .3));
    values.insert(biasKey(0), b);
    values.insert(xp(1), gtsam::Pose3(gtsam::Rot3::RzRyRx(-.09, .16, .31),
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
      require(max_relative < 1e-10, "Python Combined IMU factor parity");
    }
  }
  std::cout << "native state, time, factor and IMU parity checks passed\n";
}
