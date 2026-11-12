#include "engine.hpp"
#include "model.hpp"
#include <Eigen/Eigenvalues>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <array>
#include <iostream>
using namespace gnss_fgo;
void require(bool b, const char *s) {
  if (!b)
    throw std::runtime_error(s);
}
namespace gnss_fgo {
struct EngineTestAccess {
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
  static SatelliteState &state(Engine &e, SF sf) { return e.sats_.get(sf); }
  // resetAmbiguities() removes every factor of the current keys: ISAM2 then
  // drops the keys themselves and the smoother their timestamps.
  static void resetAmbiguities(Engine &e) {
    const auto keys = e.sats_.keys();
    e.resetAmbiguities();
    require(e.sats_.keys().empty() && e.holds_.empty(),
            "ambiguity reset kept a key or a hold record");
    for (auto [sf, key] : keys)
      require(!e.p2_->getLinearizationPoint().exists(key) &&
                  !e.p2_->timestamps().count(key),
              "ambiguity reset left a key in the smoother");
  }
  static void gaugeOff(Engine &e) { e.ambiguity_gauge_ = false; }
  // Smoother estimate of N_1 - N_s for every satellite s with a key.
  static std::map<int, double> differences(const Engine &e) {
    std::map<int, double> out;
    const auto keys = e.sats_.keys();
    if (!keys.count({1, 0}))
      return out;
    const auto est = e.p2_->calculateEstimate();
    for (auto [sf, key] : keys)
      out[sf.first] = est.at<double>(keys.at({1, 0})) - est.at<double>(key);
    return out;
  }
  // Live factors / variables of the phase-2 smoother and hold records.
  static std::array<std::size_t, 3> size(const Engine &e) {
    std::size_t factors = 0;
    for (const auto &f : e.p2_->getFactors())
      factors += bool(f);
    return {factors, e.p2_->getLinearizationPoint().size(), e.holds_.size()};
  }
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
  int fixes = 0, code_dd = 0, carrier_dd = 0;
  std::vector<gtsam::KeyVector> graph_keys;
  e.observeGraph([&](int, const gtsam::NonlinearFactorGraph &g,
                     const gtsam::Values &) {
    graph_keys.clear();
    for (const auto &f : g) {
      code_dd += bool(
          std::dynamic_pointer_cast<gtsam::DoubleDifferencePseudorangeFactorArm>(
              f));
      carrier_dd += bool(std::dynamic_pointer_cast<
                         gtsam::DoubleDifferenceCarrierPhaseFactorArm>(f));
      graph_keys.push_back(f->keys());
    }
  });
  // A held satellite released after a FIX: next epoch re-seeds it.
  std::optional<std::pair<SF, Key>> released;
  bool release_checked = false;
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
    require((sol.position - pos).norm() < .3,
            "constant velocity trajectory error");
    if (i == 20 || i == 21) {
      require(sol.kind == UpdateKind::DeadReckoning && !sol.fixed,
              "missing DD claimed GNSS/FIX");
      for (auto &[sf, s] : e.satelliteStates().track)
        require(s.fix_streak == 0, "stale fix streak");
    }
    if (released) {
      require(e.epochState().seeded.count(released->first) &&
                  e.satelliteStates().track.at(released->first).key !=
                      released->second,
              "released ambiguity was not re-seeded");
      for (const auto &keys : graph_keys)
        for (auto key : keys)
          require(key != released->second,
                  "released ambiguity stayed chained to its old key");
      released.reset();
      release_checked = true;
    }
    if (sol.fixed) {
      int held = 0;
      for (auto &[sf, s] : e.satelliteStates().track)
        if (s.held) {
          ++held;
          require(s.key.has_value(), "held ambiguity lost its key");
        }
      require(held > 0, "FIX did not activate a hold");
      if (!release_checked && i < 19) {
        const SF sf{3, 0};
        auto &st = EngineTestAccess::state(e, sf);
        if (st.held) {
          released = {{sf, *st.key}};
          st.release();
        }
      }
    }
    fixes += sol.fixed;
  }
  require(fixes > 0, "synthetic AR never fixed");
  require(code_dd > 0 && carrier_dd > 0,
          "engine graph lacks the official DD factors");
  require(release_checked, "release path not exercised");

  // Long run: one key per satellite for the whole run (no BetweenFactor in an
  // epoch graph, no prior on an existing key) and a smoother whose size stops
  // growing.
  Engine run(cfg, base, base, Vec3::Zero());
  AmbMap before;
  int between = 0, stale_priors = 0, key_changes = 0, run_fixes = 0;
  run.observeGraph([&](int, const gtsam::NonlinearFactorGraph &g,
                       const gtsam::Values &) {
    for (const auto &f : g) {
      between += bool(std::dynamic_pointer_cast<gtsam::BetweenFactor<double>>(f));
      if (std::dynamic_pointer_cast<gtsam::PriorFactor<double>>(f))
        for (const auto &[sf, key] : before)
          stale_priors += key == f->keys().front();
    }
  });
  constexpr int kEpochs = 400, kReset = 300;
  AmbMap reset_keys;
  bool reset_checked = false;
  std::vector<std::array<std::size_t, 3>> sizes;
  for (int i = 1; i <= kEpochs; ++i) {
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
    before = run.satelliteStates().keys();
    auto sol = run.process(obs);
    require(sol.valid && (sol.position - pos).norm() < .3,
            "long synthetic run failed");
    run_fixes += sol.fixed;
    const auto after = run.satelliteStates().keys();
    if (!before.empty())
      key_changes += after != before;
    if (i == kReset) {
      EngineTestAccess::resetAmbiguities(run);
      reset_keys = before;
    }
    // The reset starts CP suppression (no keys); the first keys after it
    // are new arcs.
    if (i > kReset && !after.empty() && !reset_checked) {
      for (auto [sf, key] : after)
        require(reset_keys.count(sf) && key != reset_keys.at(sf),
                "arc after the ambiguity reset reused a key");
      reset_checked = true;
    }
    sizes.push_back(EngineTestAccess::size(run));
  }
  require(between == 0 && stale_priors == 0,
          "phase-2 epoch graph has a continuity factor or re-primes a key");
  require(key_changes == 0 && reset_checked,
          "an uninterrupted arc changed its key");
  std::array<std::size_t, 3> early{}, late{};
  for (int i = 50; i < kEpochs; ++i)
    for (int c = 0; c < 3; ++c) {
      auto &m = i < kEpochs / 2 ? early : late;
      m[c] = std::max(m[c], sizes[i][c]);
    }
  std::cout << "long run: " << run_fixes << "/" << kEpochs
            << " FIX; max factors/variables/holds epochs 50-199: " << early[0]
            << "/" << early[1] << "/" << early[2] << ", epochs 200-399: "
            << late[0] << "/" << late[1] << "/" << late[2] << "\n";
  require(run_fixes > kEpochs / 2, "long run rarely fixed");
  for (int c = 0; c < 3; ++c)
    require(late[c] <= early[c], "phase-2 smoother keeps growing");
  // Window: ~lag/dt + 2 poses and velocities plus 8 ambiguity keys.
  require(late[1] <= std::size_t(2 * (int(cfg.fls_lag / .2) + 3) + 8),
          "phase-2 smoother holds more variables than its window");
  require(late[2] <= 7, "more hold records than satellite pairs");

  // The ambiguity gauge prior fixes only the common mode of the group, which
  // the DD factors and holds do not see: the same run without it gives the
  // same positions, FIX decisions and ambiguity differences.
  {
    Engine gauged(cfg, base, base, Vec3::Zero()),
        ungauged(cfg, base, base, Vec3::Zero());
    EngineTestAccess::gaugeOff(ungauged);
    double dpos = 0, ddiff = 0;
    int compared = 0;
    for (int i = 1; i <= kEpochs; ++i) {
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
      const auto a = gauged.process(obs), b = ungauged.process(obs);
      require(a.valid && b.valid && a.fixed == b.fixed && a.nb == b.nb,
              "gauge changed the validity or the FIX decision");
      require((a.position - pos).norm() < .3, "gauged run failed");
      dpos = std::max(dpos, (a.position - b.position).norm());
      const auto da = EngineTestAccess::differences(gauged),
                 db = EngineTestAccess::differences(ungauged);
      require(da.size() == db.size(), "gauge changed the ambiguity keys");
      for (auto [sat, d] : da) {
        require(db.count(sat), "gauge changed the ambiguity keys");
        ddiff = std::max(ddiff, std::abs(d - db.at(sat)));
        ++compared;
      }
      if (i == kReset) {
        EngineTestAccess::resetAmbiguities(gauged);
        EngineTestAccess::resetAmbiguities(ungauged);
      }
    }
    std::cout << "gauge on/off: max |position difference| " << dpos
              << " m, max |N_1 - N_s difference| " << ddiff << " cycles over "
              << compared << " differences\n";
    require(compared > 0 && dpos < 1e-6 && ddiff < 1e-6,
            "gauge prior moved the position or the ambiguity differences");
  }
  std::cout << "motion Jacobians, covariance, startup, no-IMU graph, "
               "outage/resume and AR passed\n";
}
