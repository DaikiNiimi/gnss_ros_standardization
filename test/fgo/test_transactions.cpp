#include "engine.hpp"
#include <gtsam/nonlinear/PriorFactor.h>
#include <iostream>
using namespace gnss_fgo;
static void require(bool pass, const char *message) {
  if (!pass)
    throw std::runtime_error(message);
}
template <class F> static void throws(F fn) {
  bool caught = false;
  try {
    fn();
  } catch (const std::exception &) {
    caught = true;
  }
  require(caught, "fault did not fail");
}
namespace gnss_fgo {
struct EngineTestAccess {
  static auto engine() {
    return Engine(Config{}, Vec3(-3950000, 3350000, 3700000),
                  Vec3(-3950000, 3350000, 3700000), Vec3::Zero());
  }
  static void fault(Engine &e, const char *operation, bool after) {
    e.update_probe_ = [operation, after](const char *called, bool stage) {
      if (std::string(called) == operation && stage == after)
        throw std::runtime_error("injected graph update failure");
    };
  }
  static void holdTest(bool inject) {
    auto e = engine();
    Engine::Graph graph;
    Engine::Values values;
    gtsam::FixedLagSmoother::KeyTimestampMap ts;
    const SF a{2, 0}, b{3, 0};
    auto ka = ambKey(a), kb = ambKey(b);
    for (auto key : {ka, kb}) {
      values.insert(key, 20.25);
      graph.addPrior<double>(key, 20.25,
                             gtsam::noiseModel::Isotropic::Sigma(1, 3));
      ts[key] = 0;
    }
    e.p1_->update(graph, values, ts);
    e.sats_.get(a).key = ka;
    e.sats_.get(b).key = kb;
    ArDecision ar;
    ar.nb = 2;
    ar.fixed = {{a, 20}, {b, 20.25}};
    ar.allowed = {a};
    if (inject) {
      fault(e, "hold", true);
      throws([&] { e.hold(*e.p1_, ar); });
      require(!e.healthy_, "failed hold left graph healthy");
      require(e.sats_.get(a).fix_flag == 0, "failed hold committed flags");
    } else {
      auto size = e.p1_->getFactors().size();
      e.hold(*e.p1_, ar);
      require(e.p1_->getFactors().size() == size + 1,
              "partial hold added wrong priors");
      require(e.p1_->getFactors().back()->keys() == gtsam::KeyVector{ka},
              "held an unconfirmed partial component");
      require(e.sats_.get(b).fix_flag == 0,
              "unconfirmed partial component marked fixed");
    }
  }
  static void fdeTest(bool inject) {
    auto e = engine();
    const auto pose = xp(0), key = ambKey({2, 0});
    Engine::Graph priors, dd;
    Engine::Values values;
    values.insert(pose, gtsam::Pose3());
    values.insert(key, 0.);
    priors.addPrior<gtsam::Pose3>(pose, gtsam::Pose3(),
                                  gtsam::noiseModel::Isotropic::Sigma(6, 1e-8));
    priors.addPrior<double>(key, 0.,
                            gtsam::noiseModel::Isotropic::Sigma(1, 1e-8));
    gtsam::FixedLagSmoother::KeyTimestampMap ts{{pose, 0}, {key, 0}};
    e.p1_->update(priors, values, ts);
    SatelliteObservation ref, target;
    ref.sat = 2;
    ref.rover_sat = ref.base_sat = Vec3(2.1e7, 1.2e7, 1.8e7);
    target.rover_sat = target.base_sat = Vec3(1.1e7, -2.2e7, 1.7e7);
    for (int sat : {3, 4}) {
      target.sat = sat;
      e.sats_.get({sat, 0}).activate(0.);
      dd.emplace_shared<DdFactor>(
          pose, std::vector<Key>{key}, std::vector<double>{.19}, ref, target, 0,
          e.base_, Vec3::Zero(), e.frame_, sat == 3 ? 100. : 0., 0.,
          .003 * std::sqrt(2.), true);
    }
    e.p1_->update(dd, {}, {});
    const auto slots = e.p1_->getISAM2Result().newFactorsIndices;
    auto estimate = e.p1_->calculateEstimate();
    if (inject) {
      fault(e, "fde", true);
      throws([&] { e.fde(*e.p1_, dd, slots, estimate, pose, 10); });
      require(!e.healthy_, "failed FDE left graph healthy");
      require(e.sats_.get({3, 0}).held.has_value(),
              "failed FDE committed management changes");
    } else {
      e.fde(*e.p1_, dd, slots, estimate, pose, 10);
      require(!e.sats_.get({3, 0}).held, "FDE did not release rejected target");
      require(e.sats_.get({4, 0}).held.has_value(),
              "colliding key tuple released other target");
    }
  }
  static void suppressionAndMinimumTest() {
    auto e = engine();
    ObservationEpoch obs;
    obs.gpst = obs.base_gpst = 1400000000;
    for (int sat = 1; sat <= 3; ++sat) {
      SatelliteObservation s;
      s.sat = s.prn = sat;
      s.system = 0;
      s.elevation = 1.;
      s.rover_sat = s.base_sat = Vec3(2.1e7 + sat * 1e5, 1.2e7, 1.8e7);
      s.pr[0] = s.pr_base[0] = 2e7;
      s.phase[0] = s.phase_base[0] = 1e8;
      s.wavelength[0] = .19;
      obs.sats.push_back(s);
    }
    e.sats_.get({1, 0}).activate(20);
    e.sats_.get({1, 0}).release();
    Engine::Graph graph;
    Engine::Values values;
    e.buildDd(graph, values, obs, xp(0), Vec3::Zero(), 0, {}, true);
    require(e.current_.visible.empty() && e.current_.consumed.empty() &&
                e.sats_.get({1, 0}).seed_pending,
            "CP suppression consumed a seed or exposed carrier measurements");
    for (auto f : graph) {
      auto dd = std::dynamic_pointer_cast<DdFactor>(f);
      require(dd && !dd->carrier, "CP suppression added a carrier factor");
    }
    graph.resize(0);
    values.clear();
    e.buildDd(graph, values, obs, xp(0), Vec3::Zero());
    require(e.current_.consumed.count({1, 0}) &&
                e.sats_.get({1, 0}).seed_pending,
            "seed consumed before graph commit");
    auto m = engine();
    Engine::Graph priors;
    Engine::Values initial;
    gtsam::FixedLagSmoother::KeyTimestampMap ts;
    initial.insert(xp(0), gtsam::Pose3());
    priors.addPrior<gtsam::Pose3>(xp(0), gtsam::Pose3(),
                                  gtsam::noiseModel::Isotropic::Sigma(6, 1));
    ts[xp(0)] = 0;
    for (int sat = 1; sat <= 3; ++sat) {
      auto key = ambKey({sat, 0});
      initial.insert(key, double(sat * 10));
      priors.addPrior<double>(key, double(sat * 10),
                              gtsam::noiseModel::Isotropic::Sigma(1, 1e-4));
      ts[key] = 0;
      m.sats_.get({sat, 0}).key = key;
      m.current_.visible.insert({sat, 0});
    }
    m.p1_->update(priors, initial, ts);
    auto est = m.p1_->calculateEstimate();
    auto ar = m.resolve(*m.p1_, est, xp(0), obs);
    require(ar.nb == 0, "AR accepted fewer than four used satellites");
    m.cfg_.ar_minfixsats = 3;
    ar = m.resolve(*m.p1_, est, xp(0), obs);
    require(ar.nb == 2, "minimum-satellite regression did not pose an "
                        "otherwise fixable problem");
  }
  static void resetTest() {
    auto e = engine();
    e.current_gpst_ = 1400000000;
    e.phase_ = 2;
    e.warmReset(e.base_, gtsam::Rot3());
    e.zupt_anchor_ = gtsam::Pose3();
    e.sats_.get({2, 0}).activate(10);
    auto *original = e.p2_.get();
    fault(e, "warm_reset", true);
    throws([&] { e.warmReset(e.base_ + Vec3(1, 2, 3), gtsam::Rot3()); });
    require(e.p2_.get() == original && e.sats_.get({2, 0}).held &&
                e.zupt_anchor_,
            "failed reset partially committed management state");
    e.update_probe_ = {};
    e.warmReset(e.base_, gtsam::Rot3());
    require(!e.zupt_anchor_ && !e.sats_.get({2, 0}).held,
            "reset retained old anchor/hold");
    e.addImu({e.current_gpst_ + .2, Vec3(0, 0, 9.81), Vec3::Zero(), {}});
    ObservationEpoch obs;
    obs.gpst = e.current_gpst_ + .2;
    auto out = e.process(obs);
    require(!out.valid && e.graph_epoch_ == 0, "uncovered IMU advanced graph");
    e.addImu({obs.gpst + .01, Vec3(0, 0, 9.81), Vec3::Zero(), {}});
    obs.gpst += .01;
    out = e.process(obs);
    require(!out.valid && e.graph_epoch_ == 0,
            "failed IMU recovery advanced graph");
    e.current_gpst_ = obs.gpst;
    e.warmReset(e.base_, gtsam::Rot3());
    for (int i = 1; i <= 20; ++i)
      e.addImu({obs.gpst + i * .01, Vec3(0, 0, 9.81), Vec3::Zero(), {}});
    e.collectImu(obs.gpst);
    obs.gpst += .2;
    out = e.process(obs);
    require(out.valid && out.kind == UpdateKind::DeadReckoning &&
                e.graph_epoch_ == 1,
            "IMU did not resume from reset state");
    for (int i = 1; i <= 20; ++i)
      e.addImu({obs.gpst + i * .01, Vec3(0, 0, 9.81), Vec3::Zero(), {}});
    fault(e, "fls", true);
    obs.gpst += .2;
    out = e.process(obs);
    require(!out.valid && !e.healthy_ && e.graph_epoch_ == 1,
            "failed graph/marginalization update advanced committed state");
    Solution invalid;
    invalid.valid = invalid.fixed = true;
    invalid.position = e.base_;
    e.current_ = EpochState{};
    require(!e.finalize(invalid).valid, "unhealthy graph produced FIX");
  }
  static void ddprFinalSetTest() {
    // Independent linear least-squares fixture: successive residual exclusions
    // produce 8 -> 7 -> 6 -> 5 DD rows. Returning the third solve with the final
    // five-row count leaves a 2.5 m error. Nonlinear range curvature here is
    // below the 1 mm tolerance; the final solution is not fit to this code.
    auto e = engine();
    const double direction[9][3] = {
        {0.28557068838838207, -0.93986913757694013, 0.18733762612930802},
        {-0.020274287536557824, 0.023394151999168136, 0.99952071860323388},
        {0.23498053177974657, 0.80295152848442375, 0.54777093076306549},
        {0.27944172506060339, 0.94209898601469155, 0.18536942262747705},
        {-0.87023686817168266, 0.4847843364787755, 0.087589613422908344},
        {-0.18175372711381585, -0.33328338110466449, 0.92514202723672612},
        {0.37348831959002815, 0.51303437279934649, 0.77285328973628498},
        {0.27500847181091087, -0.95640192654890921, 0.098314268169794855},
        {0.095274995703983428, -0.027107021840344393, 0.99508184817157286}};
    const double observed[8] = {-1.2362460101283774, 6.5905450169145725, 9.4236923655560503, -7.5729484119787429, -5.0268013170534704, 0.13984029033625955, 15.470539482499765, -7.6046991407940601};
    ObservationEpoch obs;
    for (int i = 0; i < 9; ++i) {
      SatelliteObservation sat;
      sat.sat = i + 1;
      sat.system = 0;
      sat.elevation = i == 0 ? 1.5 : 1.;
      sat.rover_sat = sat.base_sat =
          e.base_ + e.R_ * Vec3(direction[i]) * 2e7;
      sat.pr_base[0] = 2e7;
      sat.pr[0] = 2e7 - (i ? observed[i - 1] : 0.);
      obs.sats.push_back(sat);
    }
    const auto result = e.ddprPosition(obs, gtsam::Pose3(), Vec3::Zero());
    require(result.position.has_value() && result.active == 5,
            "DDPR fixture did not reach final five observations");
    const Vec3 expected(12.817544807868412, 4.9102208616325376, -2.715105926527194);
    require((e.R_.transpose() * (*result.position - e.base_) - expected).norm()
                < .001 && std::abs(result.rms - 0.20361008793104121) < .001,
            "DDPR estimate/residuals do not belong to final FDE set");
  }
  static void futureImuAdmissionTest() {
    auto past = engine(), future = engine();
    const double start = 1400000000.;
    for (Engine *e : {&past, &future}) {
      e->current_gpst_ = start;
      e->phase_ = 2;
      e->warmReset(e->base_, gtsam::Rot3());
      for (int i = 1; i <= 20; ++i)
        e->addImu({start + .01 * i, Vec3(0, 0, 9.81), Vec3::Zero(), {}});
    }
    // A later sample authorizes intake release, but its deliberately extreme
    // values must not affect this epoch's navigation or reported angular rate.
    future.addImu({start + .21, Vec3::Constant(1e6),
                   Vec3::Constant(1e6), {}});
    ObservationEpoch obs;
    obs.gpst = start + .2;
    const auto a = past.process(obs), b = future.process(obs);
    require(a.valid && b.valid && a.kind == UpdateKind::DeadReckoning &&
                b.kind == UpdateKind::DeadReckoning &&
                (a.position - b.position).norm() < 1e-12 &&
                (a.angular_velocity - b.angular_velocity).norm() < 1e-12 &&
                future.imu_index_ == 20 && future.imu_.size() == 21,
            "release-trigger IMU leaked into the current epoch");
    require(future.collectImu(start + .21).size() == 1,
            "future sample was discarded rather than left for its own epoch");
  }
  static void initializationTest() {
    auto e = engine();
    e.collecting_ = true;
    e.current_.visible.insert({2, 0});
    Solution fixed;
    fixed.valid = fixed.fixed = true;
    fixed.position = e.base_;
    ObservationEpoch obs;
    obs.gpst = 1000.;
    e.collect(obs, fixed);
    obs.gpst += .13;
    e.collect(obs, fixed);
    require(e.collection_.size() == 2 &&
                e.collection_.back().time == obs.gpst,
            "initialization lost real observation times");
    e.sats_.get({2, 0}).invalidate();
    obs.gpst += .3;
    e.collect(obs, fixed);
    require(e.collection_.size() == 1,
            "initialization joined different ambiguity arcs");
    // The no-GNSS early return must interrupt a FIX collection as well.
    obs.gpst += .2;
    auto none = e.process(obs);
    require(!none.valid && e.collection_.empty() &&
                e.fix_history_.empty() && !e.collecting_,
            "early return retained FIX initialization history");
    fixed.position.x() = std::numeric_limits<double>::quiet_NaN();
    e.current_ = EpochState{};
    require(!e.finalize(fixed).fixed, "NaN position was reported FIX");

    auto n = engine();
    const double start = 2000.;
    for (int i = -10; i <= 43; ++i)
      n.addImu({start + .01 * i, Vec3(0, 0, 9.81), Vec3::Zero(), {}});
    n.imu_index_ = n.imu_.size();
    double previous = start;
    for (double time : {start, start + .13, start + .43}) {
      Engine::CollectedFix f;
      f.time = f.observation.gpst = time;
      f.position = n.base_ + n.R_ * Vec3(10 * (time - start), 0, 0);
      f.velocity = Vec3(10, 0, 0);
      for (const auto &sample : n.imu_)
        if (sample.gpst > previous && sample.gpst <= time)
          f.imu.push_back(sample);
      n.collection_.push_back(std::move(f));
      previous = time;
    }
    n.transition();
    const auto &times = n.p2_->timestamps();
    require(n.phase_ == 2 && n.graph_epoch_ == 2 &&
                std::abs(times.at(poseKey(1)) - .13) < 1e-9 &&
                std::abs(times.at(poseKey(2)) - .43) < 1e-9 &&
                n.graph_gpst_ == start + .43,
            "initial graph replaced irregular observation times");
  }
};
} // namespace gnss_fgo
int main() {
  EngineTestAccess::holdTest(false);
  EngineTestAccess::holdTest(true);
  EngineTestAccess::fdeTest(false);
  EngineTestAccess::fdeTest(true);
  EngineTestAccess::resetTest();
  EngineTestAccess::suppressionAndMinimumTest();
  EngineTestAccess::initializationTest();
  EngineTestAccess::ddprFinalSetTest();
  EngineTestAccess::futureImuAdmissionTest();
  std::cout << "graph transaction regressions passed\n";
}
