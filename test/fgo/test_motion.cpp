#include "engine.hpp"
#include "model.hpp"
#include <Eigen/Eigenvalues>
#include <gtsam/base/numericalDerivative.h>
#include <iostream>
using namespace gnss_fgo;
void require(bool b, const char *s) {
  if (!b)
    throw std::runtime_error(s);
}
namespace gnss_fgo {
struct EngineTestAccess {
  static void covarianceReuse(Engine &e, const ObservationEpoch &obs) {
    const auto estimate=e.p2_->calculateEstimate();
    const auto pose=poseKey(e.graph_epoch_);
    std::optional<gtsam::JointMarginal> cache;
    e.resolve(*e.p2_,estimate,pose,obs,{},false,&cache);
    require(cache.has_value(), "full AR query did not populate covariance");
    for (const auto &exclude : {std::set<int>{1}, std::set<int>{2,3}, std::set<int>{}}) {
      for (bool partial : {false,true}) {
        auto direct=e.resolve(*e.p2_,estimate,pose,obs,exclude,partial);
        auto reused=e.resolve(*e.p2_,estimate,pose,obs,exclude,partial,&cache);
        require(direct.nb==reused.nb && direct.dropped==reused.dropped && direct.allowed==reused.allowed && direct.fixed==reused.fixed,
                "covariance reuse changed integer set");
        require((direct.fixed_position-reused.fixed_position).norm()<1e-5,
                "covariance reuse changed conditional position");
      }
    }
  }
  static void transactions() {
    Config c;
    c.gnss_motion = true;
    const Vec3 base(6378137, 0, 0);
    Engine e(c, base, base, Vec3::Zero());
    e.current_gpst_ = 1400000000.;
    e.warmReset(base, gtsam::Rot3());
    auto old = e.p2_.get();
    auto epoch = e.graph_epoch_;
    auto time = e.graph_gpst_;
    e.update_probe_ = [](const char *n, bool after) {
      if (std::string(n) == "warm_reset" && after)
        throw std::runtime_error("reset fault");
    };
    bool threw = false;
    try {
      e.warmReset(base + Vec3(1, 2, 3), gtsam::Rot3());
    } catch (const std::exception &) {
      threw = true;
    }
    require(threw && e.p2_.get() == old && e.graph_epoch_ == epoch &&
                e.graph_gpst_ == time,
            "reset partially committed");
    e.update_probe_ = {};
    Engine::Graph graph;
    Engine::Values values;
    values.insert(poseKey(1), gtsam::Pose3());
    values.insert(velKey(1), Vec3::Zero().eval());
    graph.addPrior<gtsam::Pose3>(poseKey(1), gtsam::Pose3(),
                                 gtsam::noiseModel::Isotropic::Sigma(6, 1));
    graph.addPrior<Vec3>(velKey(1), Vec3::Zero(),
                         gtsam::noiseModel::Isotropic::Sigma(3, 1));
    e.current_gpst_ += .37;
    fail(e);
    threw = false;
    try {
      e.flsUpdate(graph, values, 1, {});
    } catch (const std::exception &) {
      threw = true;
    }
    require(threw && !e.healthy_ && e.graph_epoch_ == epoch &&
                e.graph_gpst_ == time,
            "failed motion update committed");
    auto no = e.process(ObservationEpoch{1400000001., 1400000001., {}, {}});
    require(!no.valid && !no.fixed && e.graph_epoch_ == epoch,
            "corrupt graph produced a solution");
    clear(e);
    e.current_gpst_ = 1400000002.;
    e.warmReset(base, gtsam::Rot3());
    check(e);
  }
  static void check(const Engine &e) {
    require(e.imu_.empty(), "IMU retained by GNSS-only");
    if (e.graph_epoch_ < 0)
      return;
    auto est = e.p2_->calculateEstimate();
    for (auto k : est.keys())
      require(gtsam::Symbol(k).chr() != 'b', "IMU bias in GNSS graph");
    require(!e.zupt_anchor_, "GNSS has ZUPT anchor");
  }
  static void fail(Engine &e) {
    e.update_probe_ = [](const char *name, bool after) {
      if (std::string(name) == "fls" && after)
        throw std::runtime_error("injected motion update failure");
    };
  }
  static void clear(Engine &e) { e.update_probe_ = {}; }
};
} // namespace gnss_fgo
int main() {
  EngineTestAccess::transactions();
  const double dt = .37, q = 9.;
  MotionFactor f(poseKey(0), velKey(0), poseKey(1), velKey(1), dt, q);
  gtsam::Values v;
  v.insert(poseKey(0),
           gtsam::Pose3(gtsam::Rot3::RzRyRx(.2, .1, -.4), Vec3(1, 2, 3)));
  v.insert(poseKey(1),
           gtsam::Pose3(gtsam::Rot3::RzRyRx(-.3, .5, .2), Vec3(2, 3, 4)));
  v.insert(velKey(0), Vec3(4, 5, 6));
  v.insert(velKey(1), Vec3(5, 6, 7));
  std::vector<gtsam::Matrix> J;
  f.unwhitenedError(v, &J);
  for (size_t i = 0; i < 4; ++i) {
    auto key = f.keys()[i];
    const int d = i % 2 ? 3 : 6;
    for (int c = 0; c < d; ++c) {
      gtsam::Values plus = v, minus = v;
      gtsam::Vector step = gtsam::Vector::Zero(d);
      step[c] = 1e-6;
      if (i % 2) {
        plus.update(key, Vec3(v.at<Vec3>(key) + step));
        minus.update(key, Vec3(v.at<Vec3>(key) - step));
      } else {
        plus.update(key, v.at<gtsam::Pose3>(key).retract(step));
        minus.update(key, v.at<gtsam::Pose3>(key).retract(-step));
      }
      gtsam::Vector numeric =
          (f.unwhitenedError(plus) - f.unwhitenedError(minus)) / (2e-6);
      require((numeric - J[i].col(c)).norm() < 1e-8, "motion Jacobian");
    }
  }
  auto Q = MotionFactor::covariance(.7, q);
  gtsam::Matrix66 A = gtsam::Matrix66::Identity();
  A.topRightCorner<3, 3>() = .4 * Mat3::Identity();
  require((Q - A * MotionFactor::covariance(.3, q) * A.transpose() -
           MotionFactor::covariance(.4, q))
                  .norm() < 1e-12,
          "interval covariance composition");
  require(Eigen::SelfAdjointEigenSolver<gtsam::Matrix66>(Q)
                  .eigenvalues()
                  .minCoeff() > 0,
          "motion covariance PSD");
  for (double t : {0., -1., std::numeric_limits<double>::quiet_NaN()}) {
    bool threw = false;
    try {
      MotionFactor::covariance(t, q);
    } catch (const std::exception &) {
      threw = true;
    }
    require(threw, "invalid dt accepted");
  }
  Config cfg;
  cfg.gnss_motion = true;
  const Vec3 base(6378137, 0, 0);
  Engine e(cfg, base, base, Vec3::Zero());
  bool threw = false;
  try {
    e.addImu({1., Vec3::Zero(), Vec3::Zero(), {}});
  } catch (const std::exception &) {
    threw = true;
  }
  require(threw, "IMU admitted in GNSS-only");
  auto empty = e.process(ObservationEpoch{1400000000, 1400000000, {}, {}});
  require(!empty.valid && e.graphEpoch() == -1, "empty startup advanced graph");
  int fixes = 0;
  for (int i = 1; i < 45; ++i) {
    ObservationEpoch obs;
    obs.gpst = obs.base_gpst = 1400000000 + i * .2;
    Vec3 pos = base + Vec3(0, i * .4, i * .1);
    for (int j = 0; j < 8; ++j) {
      SatelliteObservation s;
      s.sat = s.prn = j + 1;
      s.system = 0;
      s.elevation = .7;
      s.rover_sat = s.base_sat =
          base + Vec3(.2 + .2 * j, std::sin(j), std::cos(j)).normalized() * 2e7;
      const double rr = range(s.rover_sat, pos), rb = range(s.base_sat, base);
      s.pr[0] = rr;
      s.pr_base[0] = rb;
      s.wavelength[0] = s.doppler_wavelength[0] = .19;
      s.phase[0] = rr / .19 + 20 + j;
      s.phase_base[0] = rb / .19;
      s.doppler[0] = (s.rover_sat - pos).normalized().dot(Vec3(0, 2, .5)) / .19;
      s.snr[0] = s.doppler_snr[0] = 45;
      obs.sats.push_back(s);
    }
    if (i == 20 || i == 21)
      obs.sats.clear();
    auto sol = e.process(obs);
    require(sol.valid, "synthetic GNSS update failed");
    EngineTestAccess::check(e);
    if (i == 4) EngineTestAccess::covarianceReuse(e, obs);
    require((sol.position - pos).norm() < .3,
            "constant velocity trajectory error");
    if (i == 20 || i == 21) {
      require(sol.kind == UpdateKind::DeadReckoning && !sol.fixed,
              "missing DD claimed GNSS/FIX");
      for (auto &[sf, s] : e.satelliteStates().track)
        require(s.fix_streak == 0, "stale fix streak");
    }
    fixes += sol.fixed;
  }
  require(fixes > 0, "synthetic AR never fixed");
  std::cout << "motion Jacobians, covariance, startup, no-IMU graph, "
               "outage/resume and AR passed\n";
}
