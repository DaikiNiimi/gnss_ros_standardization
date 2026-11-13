#include "engine.hpp"
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <algorithm>
#include <functional>
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
  static void vehicleConstraintSwitches() {
    for (const bool enabled : {false, true}) {
      Config c;
      c.nhc_enable = enabled;
      c.zupt_enable = enabled;
      Engine e(c, Vec3(-3950000, 3350000, 3700000),
               Vec3(-3950000, 3350000, 3700000), Vec3::Zero());
      Engine::Graph graph;
      Engine::Values values;
      std::vector<ImuSample> samples(20);
      for (auto &s : samples)
        s.acc = Vec3(0, 0, 9.81);
      e.addPseudo(graph, values, 3, Vec3::Zero(), samples, Vec3::Zero(), false);
      require(enabled == (graph.size() > 0),
              "NHC/ZUPT factors follow their switches");
    }
  }
  // An epoch of kRunSats satellites with Doppler for the receiver velocity
  // `velocity` (ENU) at the engine's base; the satellites move.
  static ObservationEpoch dopplerEpoch(const Engine &e, const Vec3 &velocity) {
    ObservationEpoch obs;
    obs.gpst = obs.base_gpst = kT0;
    for (int sat = 1; sat <= kRunSats; ++sat) {
      SatelliteObservation s;
      s.sat = s.prn = sat;
      s.system = 0;
      s.elevation = 1.;
      const Vec3 los =
          Vec3(.2 + .2 * sat, std::sin(sat), std::cos(sat)).normalized();
      s.rover_sat = s.base_sat = e.base_ + los * 2e7;
      s.sat_velocity = Vec3(1e3 * std::sin(2. * sat), 2e3 * std::cos(sat), 5e2);
      s.doppler_wavelength[0] = .19;
      s.doppler[0] = los.dot(e.R_ * velocity - s.sat_velocity) / .19;
      obs.sats.push_back(s);
    }
    return obs;
  }
  // A Doppler of exactly 0 Hz is a measurement (a missing one is NaN): a
  // receiver at rest and satellites whose line-of-sight velocity is zero
  // give every satellite a Doppler row; a NaN Doppler gives none.
  static void zeroDopplerTest() {
    require(std::isnan(SatelliteObservation{}.doppler[0]) &&
                std::isnan(SatelliteObservation{}.doppler[2]),
            "a Doppler not set is not missing (NaN)");
    auto e = engine();
    auto obs = dopplerEpoch(e, Vec3::Zero());
    for (auto &s : obs.sats) {
      s.sat_velocity.setZero();
      s.doppler[0] = (s.rover_sat - e.base_).normalized().dot(s.sat_velocity) / .19;
      require(s.doppler[0] == 0, "zero-Doppler fixture is not exactly zero");
    }
    const gtsam::NavState rest(gtsam::Pose3(), Vec3::Zero());
    auto rows = e.dopplerRows(obs, rest);
    require(rows && rows->rows.size() == obs.sats.size(),
            "a measured Doppler of 0 Hz gives no Doppler row");
    obs.sats[0].doppler[0] = std::numeric_limits<double>::quiet_NaN();
    rows = e.dopplerRows(obs, rest);
    require(rows && rows->rows.size() == obs.sats.size() - 1,
            "a missing (NaN) Doppler gives a Doppler row");
  }
  // Standstill (ZUPT): the IMU statistics of these samples pass the
  // stationarity test. The speed gate is the epoch's Doppler-only speed (a
  // wrong posterior velocity does not matter); without a Doppler-only
  // velocity the previous posterior speed. A stationary epoch gets a zero
  // velocity and a zero relative motion to the previous epoch, never an
  // absolute pose prior.
  static void standstillTest() {
    std::vector<ImuSample> samples(20);
    for (auto &s : samples)
      s.acc = Vec3(0, 0, 9.81);
    struct Count {
      int velocity = 0, between = 0, pose = 0, nhc = 0;
      bool sigmas = true;
    };
    auto count = [](const Engine &e, const Engine::Graph &graph) {
      Count n;
      for (const auto &f : graph) {
        if (const auto *v =
                dynamic_cast<const gtsam::PriorFactor<gtsam::Vector3> *>(f.get())) {
          ++n.velocity;
          n.sigmas = n.sigmas && v->noiseModel()->sigmas().isApprox(
                                     Vec3::Constant(e.cfg_.zupt_sigma_zero_velocity));
        }
        if (const auto *b =
                dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose3> *>(f.get())) {
          ++n.between;
          gtsam::Vector6 expected;
          expected << Vec3::Constant(Engine::kZuptRotationSigma),
              Vec3::Constant(Engine::kZuptTranslationSigma);
          n.sigmas = n.sigmas && b->noiseModel()->sigmas().isApprox(expected) &&
                     b->measured().equals(gtsam::Pose3());
        }
        n.pose += bool(
            dynamic_cast<const gtsam::PriorFactor<gtsam::Pose3> *>(f.get()));
        n.nhc += bool(dynamic_cast<const NhcBoreFactor *>(f.get()));
      }
      return n;
    };
    Config c;
    c.zupt_enable = 1;
    const Vec3 base(-3950000, 3350000, 3700000);
    // Doppler-only gate: at rest with a posterior velocity of 2 m/s gets the
    // ZUPT factors; moving at 5 m/s with a posterior velocity of 0 does not.
    for (const bool moving : {false, true}) {
      Engine e(c, base, base, Vec3::Zero());
      const auto obs = dopplerEpoch(e, moving ? Vec3(4, 3, 0) : Vec3::Zero());
      e.epoch_doppler_ =
          e.dopplerRows(obs, gtsam::NavState(gtsam::Pose3(), Vec3::Zero()));
      require(e.epoch_doppler_ && e.dopplerVelocity(*e.epoch_doppler_),
              "standstill fixture forms no Doppler-only velocity");
      Engine::Graph graph;
      Engine::Values values;
      e.addPseudo(graph, values, 3, moving ? Vec3::Zero() : Vec3(2, 0, 0),
                  samples, Vec3::Zero(), false);
      const auto n = count(e, graph);
      require(moving ? graph.empty()
                     : n.velocity == 1 && n.between == 1 && n.pose == 0 &&
                           n.sigmas,
              "standstill detection does not follow the Doppler-only speed");
    }
    // Without a Doppler-only velocity: consecutive stationary epochs, each
    // with the relative constraint and without an absolute anchor.
    {
      Engine e(c, base, base, Vec3::Zero());
      for (int k = 3; k <= 5; ++k) {
        Engine::Graph graph;
        Engine::Values values;
        e.addPseudo(graph, values, k, Vec3::Zero(), samples, Vec3::Zero(),
                    false);
        const auto n = count(e, graph);
        require(n.velocity == 1 && n.between == 1 && n.pose == 0 && n.sigmas,
                "stationary epoch: zero velocity and relative motion only");
      }
      Engine::Graph moving;
      Engine::Values values;
      e.addPseudo(moving, values, 6, Vec3(1, 0, 0), samples, Vec3::Zero(),
                  false);
      require(moving.empty(), "moving epoch: no ZUPT factors");
    }
    for (const auto &[speed, pass] :
         std::vector<std::pair<double, bool>>{{.49, true}, {.51, false}}) {
      Engine e(c, base, base, Vec3::Zero());
      Engine::Graph graph;
      Engine::Values values;
      e.addPseudo(graph, values, 3, Vec3(0, speed, 0), samples, Vec3::Zero(),
                  false);
      require(pass == (graph.size() > 0), "posterior speed gate at the limit");
    }
    // A moving epoch with NHC: the NHC factor, and the mounting pitch with
    // its prior on the first one only.
    c.nhc_enable = 1;
    Engine e(c, base, base, Vec3::Zero());
    for (const bool live : {false, true}) {
      Engine::Graph graph;
      Engine::Values values;
      e.addPseudo(graph, values, 3, Vec3(1, 0, 0), samples, Vec3::Zero(), live);
      const auto n = count(e, graph);
      require(n.nhc == 1 && n.velocity == 0 && n.between == 0 &&
                  graph.size() == (live ? 1u : 2u) &&
                  values.exists(boreKey()) == !live,
              "moving epoch with NHC: NHC factor and mounting pitch");
    }
  }
  static void holdTest(bool inject) {
    // Motion stage (the graph gnss_imu_fgo starts with): the hold is one
    // BetweenFactor per accepted pair, and only on the pairs of the accepted
    // solution.
    auto e = engine();
    Engine::Graph graph;
    Engine::Values values;
    gtsam::FixedLagSmoother::KeyTimestampMap ts;
    const SF a{2, 0}, b{3, 0}, c{4, 0};
    auto ka = ambKey(a), kb = ambKey(b), kc = ambKey(c);
    for (auto key : {ka, kb, kc}) {
      values.insert(key, 20.25);
      graph.addPrior<double>(key, 20.25,
                             gtsam::noiseModel::Isotropic::Sigma(1, 3));
      ts[key] = 0;
    }
    e.p2_->update(graph, values, ts);
    e.sats_.get(a).key = ka;
    e.sats_.get(b).key = kb;
    e.sats_.get(c).key = kc;
    ArDecision ar;
    ar.nb = 1;
    // Accepted integer N_a - N_b = 3; c is not part of the solution.
    ar.fixed = {{a, 20.25}, {b, 17.25}, {c, 20.25}};
    ar.allowed = {a, b};
    ar.fixed_pairs = {{a, b}};
    if (inject) {
      fault(e, "hold", true);
      throws([&] { e.hold(*e.p2_, ar); });
      require(!e.healthy_, "failed hold left graph healthy");
      require(e.sats_.get(a).fix_flag == 0 && !e.sats_.get(a).held,
              "failed hold committed flags");
    } else {
      auto size = e.p2_->getFactors().size();
      e.hold(*e.p2_, ar);
      require(e.p2_->getFactors().size() == size + 1,
              "hold did not add one factor per accepted pair");
      const auto &factor = e.p2_->getFactors().back();
      require(std::dynamic_pointer_cast<gtsam::BetweenFactor<double>>(factor) &&
                  factor->keys() == gtsam::KeyVector({ka, kb}),
              "hold is not a pair BetweenFactor");
      require(e.sats_.get(c).fix_flag == 0 && e.sats_.get(a).fix_flag == 3,
              "hold marked the wrong satellites fixed");
      // As in gnss_fgo, the accepted satellites are held (bookkeeping).
      require(e.sats_.get(a).held == 20.25 && e.sats_.get(b).held == 17.25 &&
                  !e.sats_.get(c).held,
              "motion-stage hold did not activate the accepted satellites");
      const auto est = e.p2_->calculateEstimate();
      const double dn = est.at<double>(ka) - est.at<double>(kb);
      require(std::abs(dn - 3.) < 1e-3,
              "hold did not pull N_ref - N_target to the fixed integer");
      require(std::abs(est.at<double>(kc) - 20.25) < 1e-9,
              "hold touched an ambiguity outside the solution");
      // The keys persist, so a repeated FIX must not add the hold again.
      const auto factors = e.p2_->getFactors().nrFactors();
      e.hold(*e.p2_, ar);
      require(e.p2_->getFactors().nrFactors() == factors &&
                  e.holds_.size() == 1,
              "motion-stage hold accumulated on persistent keys");
    }
  }
  static void fdeTest(bool inject) {
    // Held ambiguities keep their keys, so every carrier DD has two keys and
    // FDE tests held/held rows too. A rejected row releases its held
    // satellites (and only those) without a seed: their next arcs start from
    // code minus carrier. A code DD 100 m off is not tested (the code DDs
    // carry a Huber loss instead) and stays in the smoother.
    auto e = engine();
    const auto pose = poseKey(0);
    Engine::Graph priors, dd;
    DdRows rows;
    Engine::Values values;
    gtsam::FixedLagSmoother::KeyTimestampMap ts{{pose, 0}};
    values.insert(pose, gtsam::Pose3());
    priors.addPrior<gtsam::Pose3>(pose, gtsam::Pose3(),
                                  gtsam::noiseModel::Isotropic::Sigma(6, 1e-8));
    for (int sat : {2, 3, 4, 5, 6}) {
      const auto key = ambKey({sat, 0});
      values.insert(key, 0.);
      priors.addPrior<double>(key, 0.,
                              gtsam::noiseModel::Isotropic::Sigma(1, 1e-8));
      ts[key] = 0;
      e.sats_.get({sat, 0}).key = key;
      e.sats_.get({sat, 0}).activate(0.);
    }
    e.p2_->update(priors, values, ts);
    const Vec3 ref_sat(2.1e7, 1.2e7, 1.8e7), target_sat(1.1e7, -2.2e7, 1.7e7);
    for (int sat : {3, 4, 5, 6}) {
      rows.push_back({dd.size(), true, 2, sat, 0});
      dd.emplace_shared<gtsam::DoubleDifferenceCarrierPhaseFactorArm>(
          pose, ambKey({2, 0}), ambKey({sat, 0}), 0., 0.,
          sat == 3 ? 100. : 0., 0., ref_sat, target_sat, ref_sat, target_sat,
          e.base_, .19, Vec3::Zero(), e.frame_,
          gtsam::noiseModel::Isotropic::Sigma(1, .003 * std::sqrt(2.)));
    }
    const std::size_t code = dd.size();
    rows.push_back({code, false, 2, 4, 0});
    dd.emplace_shared<gtsam::DoubleDifferencePseudorangeFactorArm>(
        pose, 0., 0., 100., 0., ref_sat, target_sat, ref_sat, target_sat,
        e.base_, Vec3::Zero(), e.frame_,
        gtsam::noiseModel::Isotropic::Sigma(1, .3 * std::sqrt(2.)));
    e.p2_->update(dd, {}, {});
    const auto slots = e.p2_->getISAM2Result().newFactorsIndices;
    auto estimate = e.p2_->calculateEstimate();
    if (inject) {
      fault(e, "fde", true);
      throws([&] { e.fde(*e.p2_, dd, rows, slots, estimate, pose); });
      require(!e.healthy_, "failed FDE left graph healthy");
      require(e.sats_.get({3, 0}).held.has_value() && e.sats_.get({3, 0}).key,
              "failed FDE committed management changes");
    } else {
      e.fde(*e.p2_, dd, rows, slots, estimate, pose);
      for (int sat : {2, 3}) {
        const auto &st = e.sats_.get({sat, 0});
        require(!st.held && !st.key && !st.seed_pending && !st.last_held,
                "FDE did not release the held satellites of the rejected row "
                "without a seed");
      }
      for (int sat : {4, 5, 6})
        require(e.sats_.get({sat, 0}).held.has_value() &&
                    e.sats_.get({sat, 0}).key,
                "FDE released a satellite of a non-rejected row");
      require(!e.p2_->getFactors().at(slots[0]) &&
                  e.p2_->getFactors().at(slots[1]) &&
                  e.p2_->getFactors().at(slots[code]),
              "FDE did not remove exactly the rejected carrier DD");
    }
  }
  // Satellites seen identically from rover and base (pose at the base);
  // sat 1 is the reference (highest); the
  // true single-difference ambiguity of sat s is 10 * s cycles.
  static ObservationEpoch carrierEpoch(const Engine &e, double gpst) {
    const double dirs[5][3] = {{.3, .2, 1},    {-.5, .1, .8}, {.2, -.6, .7},
                               {.7, .4, .5}, {-.3, -.4, .9}};
    ObservationEpoch obs;
    obs.gpst = obs.base_gpst = gpst;
    for (int i = 0; i < 5; ++i) {
      SatelliteObservation s;
      s.sat = s.prn = i + 1;
      s.system = 0;
      s.elevation = 1.4 - .1 * i;
      s.rover_sat = s.base_sat =
          e.base_ + e.R_ * Vec3(dirs[i][0], dirs[i][1], dirs[i][2]).normalized() * 2e7;
      s.pr[0] = s.pr_base[0] = 2e7;
      s.wavelength[0] = .19;
      s.phase[0] = 1e8 + 10. * s.sat;
      s.phase_base[0] = 1e8;
      obs.sats.push_back(s);
    }
    return obs;
  }
  // `obs` without carrier phase on any satellite.
  static ObservationEpoch noCarrier(ObservationEpoch obs) {
    for (auto &s : obs.sats)
      s.phase.setZero();
    return obs;
  }
  // One phase-2 GNSS epoch's ambiguity bookkeeping as Engine::phase2 does it
  // (continueArcs, buildDd, endUnusedArcs), committed to the smoother.
  struct ArcEpoch {
    Engine::Graph graph;
    DdRows rows;
    Engine::Values values;
    AmbMap before;
  };
  static ArcEpoch arcEpoch(Engine &e, const ObservationEpoch &obs) {
    ArcEpoch out;
    out.before = e.sats_.keys();
    const auto estimate = e.p2_->calculateEstimate();
    e.current_ = EpochState{};
    e.slips(obs);
    e.continueArcs(estimate);
    e.buildDd(out.graph, out.rows, out.values, obs, poseKey(0), e.lever_);
    e.endUnusedArcs();
    gtsam::FixedLagSmoother::KeyTimestampMap ts;
    for (auto key : out.values.keys())
      ts[key] = e.graph_time_;
    for (auto [sf, key] : e.sats_.keys())
      ts[key] = e.graph_time_;
    e.p2_->update(out.graph, out.values, ts);
    for (auto sf : e.current_.consumed)
      e.sats_.get(sf).seed_pending = false;
    return out;
  }
  // FIX at the true integers N_1 - N_s = 10 - 10 s on every pair (1, s) whose
  // satellites have keys; `shift` moves the integer of satellite 3.
  static ArDecision trueFix(Engine &e, double shift = 0) {
    const auto estimate = e.p2_->calculateEstimate();
    ArDecision ar;
    ar.nb = 4;
    const SF ref{1, 0};
    ar.fixed[ref] = estimate.at<double>(*e.sats_.get(ref).key);
    for (int sat = 2; sat <= 5; ++sat)
      if (e.sats_.get({sat, 0}).key) {
        ar.fixed[{sat, 0}] =
            ar.fixed[ref] - (10. - 10. * sat) + (sat == 3 ? shift : 0.);
        ar.fixed_pairs.push_back({ref, {sat, 0}});
      }
    for (auto &[sf, v] : ar.fixed)
      ar.allowed.insert(sf);
    return ar;
  }
  // The live BetweenFactor<double> factors (holds) on the pair (a, b).
  static std::vector<double> holdsOn(const Engine &e, Key a, Key b) {
    std::vector<double> out;
    for (const auto &factor : e.p2_->getFactors())
      if (auto between =
              std::dynamic_pointer_cast<gtsam::BetweenFactor<double>>(factor))
        if (between->keys() == gtsam::KeyVector({a, b}))
          out.push_back(between->measured());
    return out;
  }
  static std::size_t liveFactors(const Engine &e) {
    std::size_t n = 0;
    for (const auto &factor : e.p2_->getFactors())
      n += bool(factor);
    return n;
  }
  static Engine phase2Engine() {
    auto e = engine();
    e.current_gpst_ = 1400000000;
    e.phase_ = 2;
    e.warmReset(e.base_, gtsam::Rot3());
    return e;
  }
  static void phase2HoldTest() {
    auto e = phase2Engine();
    const auto obs = carrierEpoch(e, e.current_gpst_);
    auto epoch = arcEpoch(e, obs);
    const auto keys = e.sats_.keys();
    require(keys.size() == 5 && e.current_.seeded.size() == 5,
            "first epoch did not create one key per satellite");
    const SF ref{1, 0};
    const Key k1 = keys.at(ref), k3 = keys.at({3, 0});
    // Hold every pair (1, s) at the true integer.
    auto ar = trueFix(e);
    const auto times_before = e.p2_->timestamps();
    const auto size = liveFactors(e);
    e.hold(*e.p2_, ar);
    require(e.sats_.keys() == keys, "hold removed or replaced an ambiguity key");
    for (auto &[sf, v] : ar.fixed)
      require(e.sats_.get(sf).held == v, "hold bookkeeping not activated");
    require(e.p2_->timestamps() == times_before,
            "phase-2 hold changed smoother key timestamps");
    require(liveFactors(e) == size + 4 && e.holds_.size() == 4,
            "phase-2 hold factor count");
    for (int sat = 2; sat <= 5; ++sat)
      require(holdsOn(e, k1, keys.at({sat, 0})).size() == 1,
              "phase-2 hold is not one BetweenFactor per pair");
    // Next epoch: every satellite continues its key; every DD is the official
    // factor, the carrier one with both ambiguity keys; no new prior, no
    // BetweenFactor.
    epoch = arcEpoch(e, obs);
    require(e.sats_.keys() == keys, "persistent ambiguity key was replaced");
    require(e.current_.seeded.empty() && epoch.values.empty(),
            "continuing ambiguity was re-seeded");
    int carrier = 0;
    for (const auto &row : epoch.rows)
      if (row.carrier) {
        ++carrier;
        auto cp = std::dynamic_pointer_cast<
            gtsam::DoubleDifferenceCarrierPhaseFactorArm>(epoch.graph[row.index]);
        require(cp && cp->keys().size() == 3,
                "carrier DD with a held satellite is not the official factor");
      } else
        require(bool(std::dynamic_pointer_cast<
                     gtsam::DoubleDifferencePseudorangeFactorArm>(
                    epoch.graph[row.index])),
                "code DD is not the official factor");
    require(carrier == 4 && epoch.rows.size() == epoch.graph.size(),
            "epoch graph is not exactly the DD rows");
    for (const auto &factor : epoch.graph)
      require(!std::dynamic_pointer_cast<gtsam::BetweenFactor<double>>(factor) &&
                  !std::dynamic_pointer_cast<gtsam::PriorFactor<double>>(factor),
              "continuing arc got a continuity factor or a new prior");
    for (int sat = 1; sat <= 5; ++sat)
      require(e.sats_.get({sat, 0}).held && e.sats_.get({sat, 0}).key,
              "held satellite has no key");
    // Second FIX at the same integers: the holds are not added again.
    const auto factors_before = liveFactors(e);
    e.hold(*e.p2_, trueFix(e));
    require(liveFactors(e) == factors_before && e.holds_.size() == 4,
            "repeated FIX added hold factors again");
    // A changed integer replaces the pair's factor.
    e.hold(*e.p2_, trueFix(e, 1.));
    const auto on13 = holdsOn(e, k1, k3);
    require(liveFactors(e) == factors_before && on13.size() == 1 &&
                std::abs(on13[0] - (20. + 1.)) < 1e-12 &&
                e.holds_.size() == 4 &&
                std::abs(e.holds_.at({k1, k3}).difference - on13[0]) < 1e-12,
            "changed integer did not replace the hold factor");
    for (const auto &[pair, record] : e.holds_) {
      auto between = std::dynamic_pointer_cast<gtsam::BetweenFactor<double>>(
          e.p2_->getFactors().at(record.slot));
      require(between && between->keys() ==
                             gtsam::KeyVector({pair.first, pair.second}) &&
                  between->measured() == record.difference,
              "hold record does not point at its factor");
    }
  }
  // Every event that ends an arc gives the satellite a new key; a held one is
  // re-seeded from last_held.
  static void arcEndTest() {
    auto e = phase2Engine();
    const auto obs = carrierEpoch(e, e.current_gpst_);
    // Every key value ever given to an arc: a new arc never reuses one.
    std::set<Key> history;
    auto run = [&](const ObservationEpoch &o) {
      const auto before = e.sats_.keys();
      auto epoch = arcEpoch(e, o);
      for (auto [sf, key] : e.sats_.keys())
        if (!before.count(sf) || before.at(sf) != key)
          require(history.insert(key).second, "a new arc reused a key");
      return epoch;
    };
    run(obs);
    e.hold(*e.p2_, trueFix(e));
    const auto keys = e.sats_.keys();
    auto newKey = [&](SF sf, const ArcEpoch &epoch, const AmbMap &before,
                      std::optional<double> seed, const char *message) {
      const auto &st = e.sats_.get(sf);
      require(st.key && before.count(sf) && *st.key != before.at(sf) &&
                  e.current_.seeded.count(sf) && epoch.values.exists(*st.key),
              message);
      for (const auto &[old_sf, old_key] : before)
        require(*st.key != old_key, message);
      if (seed)
        require(epoch.values.at<double>(*st.key) == *seed &&
                    e.current_.consumed.count(sf),
                message);
      else
        require(!e.current_.consumed.count(sf), message);
    };
    // One-epoch carrier gap of held satellite 5: the key ends with the gap
    // epoch; the satellite stays held until the next GNSS epoch releases it.
    auto gap = obs;
    gap.sats[4].phase[0] = 0;
    run(gap);
    const SF s5{5, 0};
    require(!e.sats_.get(s5).key && e.sats_.get(s5).held,
            "carrier gap kept the key or dropped the hold early");
    e.hold(*e.p2_, trueFix(e));
    for (const auto &[pair, record] : e.holds_)
      require(pair.second != keys.at(s5), "hold record kept an ended key");
    const double held5 = *e.sats_.get(s5).last_held;
    auto epoch = run(obs);
    newKey(s5, epoch, keys, held5, "carrier gap did not start a seeded arc");
    for (int sat = 1; sat <= 4; ++sat)
      require(e.sats_.get({sat, 0}).key == keys.at({sat, 0}),
              "carrier gap of one satellite ended another arc");
    // Seeded release (continueArcs releases a held arc whose key ended).
    auto before = e.sats_.keys();
    const SF s3{3, 0};
    const double held3 = *e.sats_.get(s3).held;
    e.sats_.get(s3).release();
    epoch = run(obs);
    newKey(s3, epoch, before, held3, "release did not start a seeded arc");
    // Slip: a new arc from code minus carrier, no seed.
    before = e.sats_.keys();
    auto slip = obs;
    slip.sats[3].lli[0] = 1;
    const SF s4{4, 0};
    epoch = run(slip);
    newKey(s4, epoch, before, std::nullopt, "slip did not start a new arc");
    require(!e.sats_.get(s4).last_held, "slip kept the hold");
    // One-epoch carrier gap of a satellite that is not held.
    before = e.sats_.keys();
    gap = obs;
    gap.sats[3].phase[0] = 0;
    run(gap);
    require(!e.sats_.get(s4).key && !e.sats_.get(s4).held,
            "carrier gap kept the key");
    epoch = run(obs);
    newKey(s4, epoch, before, std::nullopt,
           "carrier gap of a float ambiguity did not start a new arc");
    // An epoch without carrier on any satellite: every arc ends, the held
    // ones stay held until the next epoch re-seeds them.
    e.hold(*e.p2_, trueFix(e));
    before = e.sats_.keys();
    std::map<SF, double> last;
    for (auto &[sf, key] : before)
      last[sf] = *e.sats_.get(sf).held;
    epoch = run(noCarrier(obs));
    require(e.sats_.keys().empty() && e.current_.visible.empty(),
            "epoch without carrier kept an ambiguity key");
    for (auto &[sf, key] : before)
      require(e.sats_.get(sf).held.has_value(),
              "epoch without carrier dropped a hold early");
    epoch = run(obs);
    for (auto &[sf, key] : before)
      newKey(sf, epoch, before, last.at(sf),
             "carrier gap of every satellite did not start seeded arcs");
    require(history.size() == 5 + 1 + 1 + 1 + 1 + 5,
            "unexpected number of ambiguity arcs");
  }
  // The ambiguity gauge priors in `graph`: PriorFactor<double> with the gauge
  // sigma (creation priors have sigma_amb0 or .1 cycles).
  static std::vector<std::shared_ptr<gtsam::PriorFactor<double>>>
  gauges(const gtsam::NonlinearFactorGraph &graph) {
    std::vector<std::shared_ptr<gtsam::PriorFactor<double>>> out;
    for (const auto &factor : graph)
      if (auto prior = std::dynamic_pointer_cast<gtsam::PriorFactor<double>>(factor))
        if (std::abs(prior->noiseModel()->sigmas()(0) -
                     Engine::kAmbiguityGaugeSigma) < 1e-15)
          out.push_back(prior);
    return out;
  }
  // One gauge prior per (system, band) group where the group starts: on the
  // reference key at its initial value, never a DD row; none while a key of the
  // group continues; a new one when every key of the group is renewed.
  static void gaugeTest() {
    auto e = phase2Engine();
    const auto obs = carrierEpoch(e, e.current_gpst_);
    const SF ref{1, 0};
    // Exactly one gauge on the epoch's reference key; returns that key.
    auto one = [&](const ArcEpoch &epoch, const char *message) {
      const auto g = gauges(epoch.graph);
      const Key key = *e.sats_.get(ref).key;
      require(g.size() == 1 && g[0]->keys().front() == key &&
                  g[0]->prior() == epoch.values.at<double>(key) &&
                  e.current_.references.at(0) == ref.first,
              message);
      for (const auto &row : epoch.rows)
        require(epoch.graph[row.index] != g[0], "gauge prior is a DD row");
      require(epoch.rows.size() + epoch.values.size() + 1 == epoch.graph.size(),
              "group start: DD rows, creation priors and one gauge");
      return key;
    };
    auto none = [&](const ArcEpoch &epoch, const char *message) {
      require(gauges(epoch.graph).empty(), message);
    };
    auto gaugesOn = [&](Key key) {
      int n = 0;
      for (const auto &g : gauges(e.p2_->getFactors()))
        n += g->keys().front() == key;
      return n;
    };
    // First carrier epoch, then a continuing one.
    Key gauged = one(arcEpoch(e, obs), "first carrier epoch: one gauge");
    none(arcEpoch(e, obs), "continuing keys got a gauge");
    require(gaugesOn(gauged) == 1 && gauges(e.p2_->getFactors()).size() == 1,
            "gauge prior not in the smoother");
    // A slip of every satellite renews the group: one new gauge.
    {
      auto slipped = obs;
      for (auto &s : slipped.sats)
        s.lli[0] = 1;
      const Key renewed =
          one(arcEpoch(e, slipped), "group after a slip of every arc");
      require(renewed != gauged, "group after a slip kept its key");
      gauged = renewed;
    }
    // A new reference key while the other keys continue: no gauge. Satellite
    // 1 is gone and satellite 2 (now the highest) slips.
    {
      auto o = obs;
      o.sats.erase(o.sats.begin());
      o.sats[0].lli[0] = 1;
      const auto before = e.sats_.keys();
      const auto epoch = arcEpoch(e, o);
      require(e.current_.references.at(0) == 2 &&
                  e.sats_.keys().at({2, 0}) != before.at({2, 0}) &&
                  e.sats_.keys().at({3, 0}) == before.at({3, 0}),
              "reference change fixture");
      none(epoch, "new reference with continuing keys got a gauge");
    }
    // The full set again: satellite 1 (new key) is the reference, 2-5
    // continue: no gauge.
    none(arcEpoch(e, obs), "returning reference with continuing keys");
    // An epoch without carrier ends every arc; the next carrier epoch starts
    // the group again (no held keys, so no seeds): one new gauge.
    none(arcEpoch(e, noCarrier(obs)), "epoch without carrier got a gauge");
    const Key renewed = one(arcEpoch(e, obs), "group after a carrier gap");
    require(renewed != gauged, "group after a carrier gap kept its key");
    // Held keys renewed after a carrier gap are seeded from last_held: the
    // seed priors fix the common mode, so no gauge.
    e.hold(*e.p2_, trueFix(e));
    arcEpoch(e, noCarrier(obs));
    const auto seeded = arcEpoch(e, obs);
    require(e.current_.consumed.size() == 5, "seed fixture");
    none(seeded, "seeded group start got a gauge");
    // Two (system, band) groups start together: one gauge each.
    {
      auto f = phase2Engine();
      auto o = carrierEpoch(f, f.current_gpst_);
      for (int i = 3; i < 5; ++i)
        o.sats[i].system = 2;
      require(gauges(arcEpoch(f, o).graph).size() == 2,
              "two starting groups: one gauge each");
    }
    // A group whose carrier DDs use only keys created in this call starts
    // there and gets its gauge, also while an older key of the group still
    // exists (the motion stage and the IMU graph share this rule; the
    // epoch's endUnusedArcs ends such a key).
    for (const int stage : {1, 2}) {
      auto p = stage == 1 ? engine() : phase2Engine();
      const auto first = carrierEpoch(p, 1400000000);
      Engine::Graph graph;
      DdRows rows;
      Engine::Values values;
      p.buildDd(graph, rows, values, first, poseKey(0), Vec3::Zero());
      require(gauges(graph).size() == 1, "first group start");
      // Satellites 6 and 7 (6 the highest) with carrier, 1-5 without.
      auto second = first;
      for (auto &s : second.sats)
        s.phase[0] = 0;
      for (int sat : {6, 7}) {
        auto s = first.sats[1];
        s.sat = s.prn = sat;
        s.elevation = sat == 6 ? 1.5 : 1.45;
        second.sats.push_back(s);
      }
      p.current_ = EpochState{};
      Engine::Graph next;
      rows.clear();
      values.clear();
      p.buildDd(next, rows, values, second, poseKey(1), Vec3::Zero());
      require(gauges(next).size() == 1u, "group start beside a live key");
    }
  }
  // IMU-coupled phase 2 (Cholesky ISAM2) on a constant-velocity synthetic run
  // with a deliberately weak creation prior (150 cycles): without the gauge
  // the group's common mode becomes indeterminate and the smoother update
  // throws (each failure warm-resets the graph); with it, no failure.
  // Returns the number of failed epochs, `fixes` and the largest position
  // error after epoch 50.
  static int weakPriorImuRun(bool gauge, int &fixes, double &error) {
    Config cfg;
    cfg.sigma_amb0 = 150;
    const Vec3 base(6378137, 0, 0);
    Engine e(cfg, base, base, Vec3::Zero());
    e.ambiguity_gauge_ = gauge;
    const double t0 = 1400000000;
    auto truth = [&](int i) -> Vec3 { return base + Vec3(0, i * .4, i * .1); };
    e.phase_ = 2;
    e.current_gpst_ = t0;
    e.last_gpst_ = t0;
    e.warmReset(truth(0), gtsam::Rot3());
    int failures = 0;
    fixes = 0;
    error = 0;
    for (int i = 1; i <= 300; ++i) {
      for (int m = 1; m <= 20; ++m)
        e.addImu(imuSample(e, t0 + (i - 1) * .2 + m * .01, Mat3::Identity(),
                           Vec3::Zero(), Vec3(2, .5, 0)));
      ObservationEpoch obs;
      obs.gpst = obs.base_gpst = t0 + i * .2;
      const Vec3 pos = truth(i);
      for (int j = 0; j < 12; ++j) {
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
        s.doppler[0] =
            (s.rover_sat - pos).normalized().dot(Vec3(0, 2, .5)) / .19;
        s.snr[0] = s.doppler_snr[0] = 45;
        obs.sats.push_back(s);
      }
      const auto sol = e.process(obs);
      failures += !sol.failure.empty();
      fixes += sol.fixed;
      if (sol.valid && i > 50)
        error = std::max(error, (sol.position - pos).norm());
    }
    return failures;
  }
  static void gaugeImuRegression() {
    int fixes = 0;
    double error = 0;
    const int ungauged = weakPriorImuRun(false, fixes, error);
    // The fixture must reach the failure, or the gauged run proves nothing.
    require(ungauged > 0, "weak-prior fixture no longer fails without the gauge");
    const int gauged = weakPriorImuRun(true, fixes, error);
    std::cout << "weak creation prior, IMU smoother: " << ungauged
              << " failed epochs without the gauge, " << gauged
              << " with it (" << fixes << "/300 FIX, max error after epoch 50 "
              << error << " m)\n";
    require(gauged == 0 && fixes > 250 && error < .01,
            "gauged IMU smoother failed on a weak creation prior");
  }
  static void resetTest() {
    auto e = engine();
    e.current_gpst_ = 1400000000;
    e.phase_ = 2;
    e.warmReset(e.base_, gtsam::Rot3());
    e.sats_.get({2, 0}).activate(10);
    auto *original = e.p2_.get();
    const gtsam::Rot3 tilted = gtsam::Rot3::RzRyRx(.02, -.03, 1.);
    fault(e, "warm_reset", true);
    throws([&] { e.warmReset(e.base_ + Vec3(1, 2, 3), tilted); });
    require(e.p2_.get() == original && e.sats_.get({2, 0}).held &&
                e.last_rotation_ && e.last_rotation_->equals(gtsam::Rot3()),
            "failed reset partially committed management state");
    e.update_probe_ = {};
    e.warmReset(e.base_, tilted);
    require(!e.sats_.get({2, 0}).held && e.last_rotation_ &&
                e.last_rotation_->equals(tilted),
            "reset retained the old hold or lost its attitude");
    e.warmReset(e.base_, gtsam::Rot3());
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
  // IMU backlog without GNSS epochs: once more than kImuBacklog +
  // kImuBacklogBlock samples wait unconsumed, the oldest unconsumed ones are
  // dropped down to kImuBacklog. The count is exact, consumed samples stay,
  // and the retained samples keep their order and predecessors.
  static void imuBacklogTest() {
    Engine e(Config{}, Vec3(6378137, 0, 0), Vec3(6378137, 0, 0),
             Vec3::Zero());
    const double start = 1400000000.;
    // Sample n: time start + .01 n, acc.x = n.
    const auto sample = [&](std::size_t n) {
      return ImuSample{start + .01 * double(n), Vec3(double(n), 0, 9.81),
                       Vec3::Zero(), {}};
    };
    constexpr std::size_t consumed = 10;
    for (std::size_t n = 0; n < consumed; ++n)
      e.addImu(sample(n));
    require(e.collectImu(start + .01 * double(consumed - 1)).size() == consumed &&
                e.imu_index_ == consumed,
            "backlog fixture: consumed prefix");
    constexpr std::size_t limit = Engine::kImuBacklog + Engine::kImuBacklogBlock;
    std::size_t pushed = consumed, most_waiting = 0;
    const auto push = [&](std::size_t count) {
      for (std::size_t i = 0; i < count; ++i) {
        e.addImu(sample(pushed++));
        most_waiting = std::max(most_waiting, e.imu_.size() - e.imu_index_);
      }
    };
    push(limit);
    require(e.imuBacklogDropped() == 0 && e.imu_.size() == consumed + limit,
            "backlog dropped below its limit");
    push(1);
    require(e.imuBacklogDropped() == Engine::kImuBacklogBlock + 1 &&
                e.imu_.size() - e.imu_index_ == Engine::kImuBacklog,
            "backlog block drop count");
    push(2 * Engine::kImuBacklogBlock + 7);
    const std::size_t waiting = e.imu_.size() - e.imu_index_;
    require(most_waiting == limit && waiting <= limit &&
                e.imuBacklogDropped() + waiting + consumed == pushed,
            "backlog bound or drop count");
    for (std::size_t n = 0; n < consumed; ++n)
      require(e.imu_[n].acc.x() == double(n), "consumed IMU samples dropped");
    // The retained unconsumed samples: the newest `waiting`, consecutive; each
    // keeps the sample received before it (also a dropped one) as predecessor.
    for (std::size_t i = e.imu_index_; i < e.imu_.size(); ++i) {
      const std::size_t n = pushed - (e.imu_.size() - i);
      const auto &s = e.imu_[i];
      require(s.acc.x() == double(n) && s.gpst == sample(n).gpst &&
                  s.predecessor && *s.predecessor == sample(n - 1).gpst,
              "retained IMU samples out of order or predecessor changed");
    }
  }
  static void initializationTest() {
    // A non-qualifying epoch (here: no GNSS) restarts the switch window.
    auto e = engine();
    e.window_.push_back(Engine::WindowEpoch{});
    ObservationEpoch obs;
    obs.gpst = 1000.;
    auto none = e.process(obs);
    require(!none.valid && e.window_.empty() && e.phase_ == 1,
            "a non-qualifying epoch kept the switch window");
    Solution fixed;
    fixed.valid = fixed.fixed = true;
    fixed.position.x() = std::numeric_limits<double>::quiet_NaN();
    e.current_ = EpochState{};
    require(!e.finalize(fixed).fixed, "NaN position was reported FIX");

    // transition() builds the IMU graph at the window's real (irregular)
    // observation times; the GPS factors take the window's position sigmas.
    auto n = engine();
    const double start = 2000.;
    switchWindow(n, start);
    n.transition();
    const auto &times = n.p2_->timestamps();
    require(n.phase_ == 2 && n.graph_epoch_ == 2 &&
                std::abs(times.at(poseKey(1)) - .13) < 1e-9 &&
                std::abs(times.at(poseKey(2)) - .43) < 1e-9 &&
                n.graph_gpst_ == start + .43 && n.window_.empty(),
            "initial graph replaced irregular observation times");
    int gps = 0;
    for (const auto &factor : n.p2_->getFactors())
      if (auto g = std::dynamic_pointer_cast<gtsam::GPSFactorArm>(factor))
        gps += std::abs(g->noiseModel()->sigmas()(0) - .5) < 1e-12;
    require(gps == 3, "initial GPS factors do not carry the window sigmas");
    // The IMU graph continues with the ambiguity keys of the initial graph.
    const auto keys = n.sats_.keys();
    const auto estimate = n.p2_->calculateEstimate();
    require(keys.size() == 5, "initial graph has no ambiguity keys");
    for (auto [sf, key] : keys)
      require(estimate.exists(key) &&
                  key == ambKey(sf, n.sats_.get(sf).generation),
              "initial graph ambiguity key");
    // buildDd runs once per window epoch into the one initial graph: the
    // group starts in the first call only, so it has one gauge; the new keys
    // have the sigma_amb0 creation prior.
    const auto initial_gauges = gauges(n.p2_->getFactors());
    require(initial_gauges.size() == 1 &&
                initial_gauges[0]->keys().front() == keys.at({1, 0}),
            "initial graph: one gauge per group");
    for (auto [sf, key] : keys)
      require(creationSigma(n, key) == n.cfg_.sigma_amb0,
              "initial ambiguity key without the sigma_amb0 prior");
    n.current_ = EpochState{};
    n.continueArcs(estimate);
    Engine::Graph graph;
    DdRows rows;
    Engine::Values values;
    n.buildDd(graph, rows, values, carrierEpoch(n, start + .63), poseKey(3),
              n.lever_);
    n.endUnusedArcs();
    require(n.sats_.keys() == keys && values.empty() &&
                n.current_.seeded.empty(),
            "first IMU-graph epoch re-created the initial ambiguity keys");
  }
  // A switch window of three epochs at start, start + .13, start + .43 for
  // transition(): carrierEpoch observations, the antenna moving east at
  // 10 m/s, position sigma .5 m, IMU at rest covering every interval, and the
  // same motion-graph key for every satellite in every epoch.
  static void switchWindow(Engine &n, double start) {
    for (int i = -10; i <= 43; ++i)
      n.addImu({start + .01 * i, Vec3(0, 0, 9.81), Vec3::Zero(), {}});
    n.imu_index_ = n.imu_.size();
    AmbMap arcs;
    for (int sat = 1; sat <= 5; ++sat)
      arcs[{sat, 0}] = ambKey({sat, 0}, 7);
    double previous = start;
    for (double time : {start, start + .13, start + .43}) {
      Engine::WindowEpoch w;
      w.observation = carrierEpoch(n, time);
      w.time = time;
      w.position = n.base_ + n.R_ * Vec3(10 * (time - start), 0, 0);
      w.position_sigma = .5;
      w.fixed = false;
      w.velocity = Vec3(10, 0, 0);
      w.velocity_covariance = Mat3::Identity() * .01;
      w.keys = arcs;
      for (const auto &sample : n.imu_)
        if (sample.gpst > previous && sample.gpst <= time)
          w.imu.push_back(sample);
      n.window_.push_back(std::move(w));
      previous = time;
    }
  }
  // Sigma of the creation prior (the PriorFactor<double> that is not a gauge)
  // of ambiguity `key` in the smoother; NaN when there is none.
  static double creationSigma(const Engine &e, Key key) {
    double sigma = std::numeric_limits<double>::quiet_NaN();
    for (const auto &factor : e.p2_->getFactors())
      if (auto prior = std::dynamic_pointer_cast<gtsam::PriorFactor<double>>(factor))
        if (prior->keys().front() == key &&
            std::abs(prior->noiseModel()->sigmas()(0) -
                     Engine::kAmbiguityGaugeSigma) > 1e-15)
          sigma = prior->noiseModel()->sigmas()(0);
    return sigma;
  }
  static int carrierFactors(const Engine &e) {
    int n = 0;
    for (const auto &factor : e.p2_->getFactors())
      n += bool(std::dynamic_pointer_cast<
                gtsam::DoubleDifferenceCarrierPhaseFactorArm>(factor));
    return n;
  }
  // What the switch carries into the IMU graph's ambiguities.
  static void switchAmbiguityTest() {
    // An epoch's carrier phase enters only on the motion-graph arcs that last
    // to the window end: satellite 3 changed its key after the first epoch,
    // so the first epoch has 3 carrier DDs and the others 4.
    {
      auto n = engine();
      switchWindow(n, 2000.);
      n.window_[0].keys[{3, 0}] = ambKey({3, 0}, 6);
      n.transition();
      require(carrierFactors(n) == 3 + 4 + 4,
              "initial graph joined a key across an arc change");
    }
    // A satellite held in the motion stage gets a new key seeded from its
    // held value (sigma .1 cycle); no hold record of the motion-stage
    // smoother survives. The seed fixes the group's common mode: no gauge.
    {
      auto h = engine();
      switchWindow(h, 2000.);
      auto &held = h.sats_.get({2, 0});
      held.key = ambKey({2, 0});
      held.activate(42.);
      h.holds_[{Key(1), Key(2)}] = {1., 0};
      h.transition();
      const auto &st = h.sats_.get({2, 0});
      require(h.holds_.empty(), "motion-stage hold record survived the switch");
      require(st.key && !st.held && !st.seed_pending && st.last_held == 42.,
              "held satellite was not released and seeded at the switch");
      require(creationSigma(h, *st.key) == .1, "seed prior sigma");
      for (const auto &factor : h.p2_->getFactors())
        if (auto prior =
                std::dynamic_pointer_cast<gtsam::PriorFactor<double>>(factor))
          if (prior->keys().front() == *st.key)
            require(prior->prior() == 42., "seed prior value");
      for (int sat : {1, 3, 4, 5})
        require(creationSigma(h, *h.sats_.get({sat, 0}).key) ==
                    h.cfg_.sigma_amb0,
                "unheld satellite was seeded at the switch");
      require(gauges(h.p2_->getFactors()).empty(), "seeded group got a gauge");
    }
  }
  // ---- Switch from the motion stage to the IMU graph, end to end ----
  static constexpr double kT0 = 1400000000.;
  // Synthetic straight run along the horizontal ENU direction `dir` (0.2 s
  // epochs, epoch i at kT0 + .2 i): at rest for `still` epochs, then the
  // speed v0 + accel * tau (tau = time since the motion started). The body
  // is level; `reverse`: its x axis points against the motion.
  // `vmax` caps the speed (no acceleration from then on).
  struct Motion {
    Vec3 dir = Vec3::UnitX();
    double v0 = 0, accel = 0;
    int still = 0;
    bool reverse = false;
    double vmax = std::numeric_limits<double>::infinity();
    double tau(double t) const {
      return std::max(0., t - (kT0 + .2 * still));
    }
    // Time after the start at which the speed reaches vmax.
    double capped() const {
      return accel > 0 ? std::max(0., (vmax - v0) / accel)
                       : std::numeric_limits<double>::infinity();
    }
    double distance(double t) const {
      const double a = std::min(tau(t), capped());
      return v0 * a + .5 * accel * a * a + speed(t) * (tau(t) - a);
    }
    double speed(double t) const {
      return tau(t) > 0 ? std::min(v0 + accel * tau(t), vmax) : 0.;
    }
    double acceleration(double t) const {
      return tau(t) > 0 && tau(t) < capped() ? accel : 0.;
    }
  };
  // Motion along the horizontal ENU direction of `v` from `still` on.
  static Motion run(const Vec3 &v, double v0, double accel, int still = 0,
                    bool reverse = false) {
    Motion m;
    m.dir = v.normalized();
    m.v0 = v0;
    m.accel = accel;
    m.still = still;
    m.reverse = reverse;
    return m;
  }
  // Antenna position of epoch i.
  static Vec3 truth(const Engine &e, int i, const Motion &m) {
    return e.base_ + e.R_ * (m.dir * m.distance(kT0 + i * .2));
  }
  // The accelerating runs below: 3.4 s at rest (leading still IMU for the
  // bias and roll/pitch), then 2 m/s^2 from rest. Epoch kStill + 3 (1.2 m/s)
  // is the first above vel_thresh, so the switch window (kSwitchWindowTime =
  // 1 s: kWindowEpochs epochs at 5 Hz) ends at kStill + 2 + kWindowEpochs; its
  // speed change is 0.4 m/s per epoch. The start
  // falls inside a 1 s still-detection window (leadingStillSamples): a
  // constant acceleration starting on a window boundary would pass as still
  // and be taken for tilt and bias.
  static constexpr int kStill = 17;
  static constexpr double kAccel = 2.;
  static constexpr int kWindowEpochs = 6;
  static int switchEpoch(const Engine &) { return kStill + 2 + kWindowEpochs; }
  // Satellites of the synthetic runs: one band, enough DD pairs for the fix
  // minimum at rest (Engine::kStationaryMinFixPairs).
  static constexpr int kRunSats = 12;
  // Epoch i (time kT0 + .2 i) of the synthetic run: kRunSats satellites seen
  // from the base and the rover, code, carrier and Doppler without noise
  // unless `noisy` (deterministic code/carrier perturbations, a slip on
  // satellite 4 at epoch 40).
  static ObservationEpoch runEpoch(const Engine &e, int i, const Motion &m,
                                   bool noisy = false) {
    ObservationEpoch obs;
    obs.gpst = obs.base_gpst = kT0 + i * .2;
    const Vec3 pos = truth(e, i, m);
    const Vec3 vel = e.R_ * (m.dir * m.speed(obs.gpst));
    for (int j = 0; j < kRunSats; ++j) {
      SatelliteObservation s;
      s.sat = s.prn = j + 1;
      s.system = 0;
      s.elevation = .7;
      s.rover_sat = s.base_sat =
          e.base_ +
          Vec3(.2 + .2 * j, std::sin(j), std::cos(j)).normalized() * 2e7;
      const double rr = range(s.rover_sat, pos),
                   rb = range(s.base_sat, e.base_);
      s.pr[0] = rr + (noisy ? .4 * std::sin(1.7 * i + 3. * j) : 0.);
      s.pr_base[0] = rb;
      s.wavelength[0] = s.doppler_wavelength[0] = .19;
      s.phase[0] = rr / .19 + 20 + j +
                   (noisy ? .01 * std::cos(2.3 * i + j) : 0.) +
                   (noisy && j == 3 && i >= 40 ? 7. : 0.);
      s.phase_base[0] = rb / .19;
      if (noisy && j == 3 && i == 40)
        s.lli[0] = 1;
      s.doppler[0] = (s.rover_sat - pos).normalized().dot(vel) / .19 +
                     (noisy ? .3 * std::sin(.9 * i + j) : 0.);
      s.snr[0] = s.doppler_snr[0] = 45;
      obs.sats.push_back(s);
    }
    return obs;
  }
  // IMU sample at time t of a body with rotation `body` (FLU to base ENU),
  // ENU acceleration `a` and velocity `v`, without rotation relative to the
  // Earth: the specific force in the Earth-fixed rotating frame (normal
  // gravity at the base, Coriolis) and the Earth rotation, body axes.
  static ImuSample imuSample(const Engine &e, double t, const Mat3 &body,
                             const Vec3 &a, const Vec3 &v) {
    const Vec3 w = earthRate(e.R_), g = localGravity(e.base_, e.R_);
    return {t, body.transpose() * (a + 2. * w.cross(v) - g),
            body.transpose() * w, {}};
  }
  // Body (FLU, IMU axes) of a run: the vehicle level with its x axis along
  // the motion (against it when reversing), the IMU turned by `mount`
  // (IMU to vehicle rotation).
  static Mat3 runBody(const Motion &m, const Mat3 &mount = Mat3::Identity()) {
    const Vec3 x = m.reverse ? Vec3(-m.dir) : m.dir, z = Vec3::UnitZ();
    Mat3 vehicle;
    vehicle << x, z.cross(x), z;
    return vehicle * mount;
  }
  // IMU of a run at 100 Hz from the previous epoch up to epoch i.
  static void runImu(Engine &e, int i, const Motion &m,
                     const Mat3 &mount = Mat3::Identity()) {
    for (int k = 1; k <= 20; ++k) {
      const double t = kT0 + (i - 1) * .2 + k * .01;
      e.addImu(imuSample(e, t, runBody(m, mount), m.dir * m.acceleration(t),
                         m.dir * m.speed(t)));
    }
  }
  static bool same(const Vec3 &a, const Vec3 &b) {
    for (int i = 0; i < 3; ++i)
      if (!(a[i] == b[i] || (std::isnan(a[i]) && std::isnan(b[i]))))
        return false;
    return true;
  }
  static bool same(const Mat3 &a, const Mat3 &b) {
    for (int i = 0; i < 3; ++i)
      if (!same(Vec3(a.row(i).transpose()), Vec3(b.row(i).transpose())))
        return false;
    return true;
  }
  // Bit-identical published content (GnssSolution fields come from these).
  static bool same(const Solution &a, const Solution &b) {
    return a.valid == b.valid && a.fixed == b.fixed && a.kind == b.kind &&
           a.nb == b.nb && a.gpst == b.gpst &&
           (a.ratio == b.ratio || (std::isnan(a.ratio) && std::isnan(b.ratio))) &&
           same(a.position, b.position) &&
           same(a.position_covariance, b.position_covariance) &&
           same(a.velocity, b.velocity) &&
           same(a.velocity_covariance, b.velocity_covariance);
  }
  // Until the switch, gnss_imu_fgo runs exactly the gnss_fgo graph (gnss_fgo's
  // float restart aside, which this run does not reach): its output (a lever
  // arm and IMU input notwithstanding) is bit-identical to gnss_fgo's,
  // through FIX/hold, a slip, a GNSS outage and the switch epoch.
  static void motionStageEqualsGnssFgo() {
    Config gc, ic;
    gc.gnss_motion = true;
    const Vec3 base(6378137, 0, 0);
    Engine g(gc, base, base, Vec3::Zero()),
        m(ic, base, base, Vec3(1., -.3, .8));
    const auto motion = run(Vec3(1.5, 1., 0), 0., 1.5, 25);
    int compared = 0, fixes = 0, switched = -1;
    for (int i = 1; i <= 80 && switched < 0; ++i) {
      auto obs = runEpoch(g, i, motion, true);
      if (i == 30 || i == 31)
        obs.sats.clear();
      runImu(m, i, motion);
      const auto a = g.process(obs), b = m.process(obs);
      require(b.phase == 1, "switch-epoch solution not from the motion graph");
      require((same(a, b) && g.floatPosition() == m.floatPosition()) ||
                  m.phase_ == 2,
              "motion stage differs from gnss_fgo");
      if (m.phase_ == 2) {
        // The switch epoch: its solution is the motion graph's.
        require(same(a, b), "switch-epoch solution differs from gnss_fgo");
        switched = i;
      }
      ++compared;
      fixes += a.fixed;
    }
    std::cout << "motion stage vs gnss_fgo: " << compared
              << " epochs bit-identical (" << fixes << " FIX), switch at epoch "
              << switched << "\n";
    require(switched > 31 && fixes > 5,
            "equivalence fixture did not exercise FIX and the switch");
  }
  // (a) The switch needs no FIX: with ambiguity resolution off it happens
  // after a 1 s window of moving epochs; the IMU graph then keeps the antenna on
  // the trajectory (lever arm included).
  static void switchWithoutFixTest() {
    Config c;
    c.ar_mode = 0;
    const Vec3 base(6378137, 0, 0);
    Engine e(c, base, base, Vec3(.5, .2, 1.));
    const auto motion = run(Vec3(2., .5, 0), 0., kAccel, kStill);
    int switched = -1;
    double error = 0;
    for (int i = 1; i <= 40; ++i) {
      runImu(e, i, motion);
      const auto sol = e.process(runEpoch(e, i, motion));
      require(sol.valid && !sol.fixed && sol.failure.empty(),
              "synthetic run without AR failed or fixed");
      if (switched < 0 && e.phase_ == 2)
        switched = i;
      require(sol.phase == (switched < 0 || i == switched ? 1 : 2),
              "Solution::phase does not follow the stage");
      error = std::max(error, (sol.position - truth(e, i, motion)).norm());
    }
    std::cout << "switch without FIX at epoch " << switched
              << ", max antenna error " << error << " m\n";
    require(switched == switchEpoch(e),
            "no switch after a full window of moving epochs");
    require(error < .1, "IMU graph after the switch left the trajectory");
  }
  // The run's IMU at 50 Hz, sampled mid-period: from the previous epoch up
  // to epoch i, the last sample 15 ms before the epoch (the next one, 5 ms
  // after it, releases the epoch in the node). Samples as runImu.
  static void runImuMidPeriod(Engine &e, int i, const Motion &m) {
    for (int k = 0; k < 10; ++k) {
      const double t = kT0 + (i - 1) * .2 + .005 + k * .02;
      e.addImu(imuSample(e, t, runBody(m), m.dir * m.acceleration(t),
                         m.dir * m.speed(t)));
    }
  }
  // The run of switchWithoutFixTest with a 50 Hz IMU whose GNSS epochs fall
  // mid-period: no epoch fails on the IMU interval, the switch happens at
  // the same epoch and the antenna stays as close to the trajectory as with
  // the 100 Hz IMU.
  static void midPeriodImuTest() {
    Config c;
    c.ar_mode = 0;
    const Vec3 base(6378137, 0, 0);
    const auto motion = run(Vec3(2., .5, 0), 0., kAccel, kStill);
    double error[2] = {0, 0};
    int switched[2] = {-1, -1};
    for (int mid : {0, 1}) {
      Engine e(c, base, base, Vec3(.5, .2, 1.));
      for (int i = 1; i <= 40; ++i) {
        if (mid)
          runImuMidPeriod(e, i, motion);
        else
          runImu(e, i, motion);
        const auto sol = e.process(runEpoch(e, i, motion));
        require(sol.valid && sol.failure.empty(),
                "synthetic run with a mid-period IMU failed");
        if (switched[mid] < 0 && e.phase_ == 2)
          switched[mid] = i;
        error[mid] =
            std::max(error[mid], (sol.position - truth(e, i, motion)).norm());
      }
      require(switched[mid] == switchEpoch(e),
              "50 Hz mid-period IMU did not switch like the 100 Hz IMU");
    }
    std::cout << "50 Hz mid-period IMU: switch at epoch " << switched[1]
              << ", max antenna error " << error[1] << " m (100 Hz: epoch "
              << switched[0] << ", " << error[0] << " m)\n";
    require(error[1] < .1 && error[1] <= error[0] + .01,
            "50 Hz mid-period IMU less accurate than the 100 Hz IMU");
  }
  // (b) A receiver at rest never switches (FIX and holds notwithstanding).
  static void stationaryNeverSwitchesTest() {
    const Vec3 base(6378137, 0, 0);
    Engine e(Config{}, base, base, Vec3(.5, .2, 1.));
    int fixes = 0;
    for (int i = 1; i <= 40; ++i) {
      runImu(e, i, Motion{});
      const auto sol = e.process(runEpoch(e, i, Motion{}));
      require(sol.valid && e.phase_ == 1 && e.window_.empty(),
              "receiver at rest started a switch window");
      fixes += sol.fixed;
    }
    require(fixes > 20, "stationary fixture did not fix");
  }
  // (c) A failed IMU-graph build leaves the motion stage as it was: the
  // epoch keeps its (motion-graph) solution, the output stays bit-identical
  // to gnss_fgo, and the next full window switches.
  static void failedSwitchTest() {
    Config gc;
    gc.gnss_motion = true;
    const Vec3 base(6378137, 0, 0);
    Engine g(gc, base, base, Vec3::Zero()),
        m(Config{}, base, base, Vec3(.5, .2, 1.));
    fault(m, "transition", true);
    const auto motion = run(Vec3(2., .5, 0), 0., kAccel, kStill);
    int failed = -1, switched = -1;
    for (int i = 1; i <= 40 && switched < 0; ++i) {
      runImu(m, i, motion);
      const auto obs = runEpoch(g, i, motion);
      const std::size_t before = m.window_.size();
      const auto a = g.process(obs), b = m.process(obs);
      require(b.valid && same(a, b), "failed switch changed the motion stage");
      // A full window that qualified and was then cleared: the failed build.
      if (failed < 0 && int(before) == kWindowEpochs - 1 &&
          m.window_.empty() && m.phase_ == 1 && m.switch_reject_ == 0) {
        failed = i;
        m.update_probe_ = {};
      }
      if (m.phase_ == 2)
        switched = i;
    }
    require(failed == switchEpoch(m) &&
                switched == failed + kWindowEpochs,
            "failed switch did not retry with the next window");
  }
  // (d) The heading prior follows the horizontal posterior velocity: a
  // straight run to the north-east starts the IMU graph at 45 degrees.
  static void switchHeadingTest() {
    const Vec3 base(6378137, 0, 0);
    Engine e(Config{}, base, base, Vec3(1., 0, .5));
    const auto motion = run(Vec3(2., 2., 0), 0., kAccel, kStill);
    int i = 0;
    Solution sol;
    while (e.phase_ == 1 && i < 40) {
      ++i;
      runImu(e, i, motion);
      sol = e.process(runEpoch(e, i, motion));
    }
    // Direction of the posterior antenna velocity published in the switch
    // epoch (the last window epoch).
    const Vec3 v = e.R_.transpose() * sol.velocity;
    const double velocity_heading = std::atan2(v.x(), v.y()) * 180 / M_PI;
    require(e.phase_ == 2, "north-east run did not switch");
    const Key last = poseKey(e.graph_epoch_);
    std::optional<double> prior_heading, prior_sigma;
    for (const auto &factor : e.p2_->getFactors())
      if (auto prior =
              std::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(factor))
        if (prior->keys().front() == last) {
          const Vec3 forward = prior->prior().rotation().matrix().col(0);
          prior_heading = std::atan2(forward.x(), forward.y()) * 180 / M_PI;
          prior_sigma = prior->noiseModel()->sigmas()(2) * 180 / M_PI;
        }
    const Vec3 forward = e.p2_->calculateEstimate<gtsam::Pose3>(last)
                             .rotation()
                             .matrix()
                             .col(0);
    const double heading = std::atan2(forward.x(), forward.y()) * 180 / M_PI;
    std::cout << "north-east run: heading prior " << prior_heading.value_or(-1)
              << " deg (sigma " << prior_sigma.value_or(-1)
              << " deg), estimate " << heading << " deg\n";
    // The prior is the velocity direction; under the 2 m/s^2 acceleration
    // the motion graph's velocity direction itself is a few tenths of a
    // degree off the true 45 degrees.
    require(prior_heading && std::abs(*prior_heading - velocity_heading) < 1e-9 &&
                std::abs(*prior_heading - 45.) < 1. &&
                std::abs(heading - 45.) < 1.,
            "heading prior does not follow the velocity direction");
    require(*prior_sigma >= 2. - 1e-9, "yaw prior sigma below 2 degrees");
  }
  // Translation sigmas (body frame) of the pose prior on `key`.
  static Vec3 priorTranslationSigmas(const Engine &e, Key key) {
    for (const auto &factor : e.p2_->getFactors())
      if (auto prior =
              std::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(factor))
        if (prior->keys().front() == key)
          return prior->noiseModel()->sigmas().tail<3>();
    return Vec3::Constant(std::numeric_limits<double>::quiet_NaN());
  }
  // The initial pose prior of a float window epoch is not tighter than its
  // GPS sigma (first epoch and later epochs); a FIX epoch keeps .05/.1 m.
  static void posePriorFloorTest() {
    auto n = engine();
    switchWindow(n, 2000.);
    n.window_[1].fixed = true;
    n.window_[1].position_sigma = .02;
    n.transition();
    require((priorTranslationSigmas(n, poseKey(0)) - Vec3(.5, .5, .5)).norm() < 1e-12 &&
                (priorTranslationSigmas(n, poseKey(1)) - Vec3(.1, .1, .1)).norm() < 1e-12 &&
                (priorTranslationSigmas(n, poseKey(2)) - Vec3(.5, .5, .5)).norm() < 1e-12,
            "pose prior translation sigmas do not follow the GPS sigma");
    // A small float sigma keeps the larger .05/.1 m prior sigmas.
    auto m = engine();
    switchWindow(m, 2000.);
    for (auto &w : m.window_)
      w.position_sigma = .07;
    m.transition();
    require((priorTranslationSigmas(m, poseKey(0)) - Vec3(.07, .07, .1)).norm() < 1e-12 &&
                (priorTranslationSigmas(m, poseKey(2)) - Vec3(.1, .1, .1)).norm() < 1e-12,
            "pose prior floor replaced a larger sigma");
  }
  // The stationary code-burst scenario: 10 satellites (two systems, L1/L2)
  // at rest; a 10-epoch burst of 15-24 m code errors on 4 satellites (epochs
  // 200-209), a code outlier, an undetected and a flagged carrier slip, a GNSS
  // outage, satellites leaving and rising; Doppler intact. From epoch 215 to
  // 231 code and carrier move the antenna east at 7 m/s (it stays there
  // after), while the Doppler still says the receiver is at rest.
  static constexpr double kBurst = 30.;
  static ObservationEpoch burstEpoch(const Engine &e, int i) {
    ObservationEpoch obs;
    obs.gpst = obs.base_gpst = kT0 + i * .2;
    const Vec3 pos =
        e.base_ + e.R_ * Vec3(7. * .2 * (std::clamp(i, 215, 231) - 215), 0, 0);
    for (int j = 0; j < 10; ++j) {
      if (j == 8 && i >= 150 && i < 170)
        continue;
      if (j == 9 && i < 60)
        continue;
      SatelliteObservation s;
      s.sat = s.prn = j + 1;
      s.system = j < 7 ? 0 : 2;
      s.elevation = .4 + .05 * j;
      s.rover_sat = s.base_sat =
          e.base_ +
          Vec3(.2 + .2 * j, std::sin(j), std::cos(j)).normalized() * 2e7;
      const double rr = range(s.rover_sat, pos), rb = range(s.base_sat, e.base_);
      double pr_noise =
          .5 * std::sin(1.7 * i + 3. * j) + .2 * std::cos(.31 * i * (j + 1));
      if (i >= 200 && i < 210 && j < 4)
        pr_noise += kBurst + 3 * j;
      if (i == 90 && j == 6)
        pr_noise += 40.;
      s.pr[0] = rr + pr_noise;
      s.pr[1] = rr + .7 * pr_noise;
      s.pr_base[0] = s.pr_base[1] = rb;
      s.wavelength[0] = s.doppler_wavelength[0] = .19;
      s.wavelength[1] = s.doppler_wavelength[1] = .244;
      double cp = .015 * std::cos(2.3 * i + j);
      if (j == 4 && i >= 120)
        cp += 3.;
      if (j == 2 && i >= 240)
        cp += 11.;
      s.phase[0] = rr / .19 + 20 + j + cp;
      s.phase[1] = rr / .244 + 13 - j + cp * .19 / .244;
      s.phase_base[0] = rb / .19;
      s.phase_base[1] = rb / .244;
      if (j == 2 && i == 240)
        s.lli[0] = 1;
      s.doppler[0] = .2 * std::sin(.9 * i + j);
      s.snr[0] = s.doppler_snr[0] = 40 + j;
      obs.sats.push_back(s);
    }
    if (i >= 130 && i <= 132)
      obs.sats.clear();
    return obs;
  }
  // A receiver at rest does not switch on the posterior velocity of a motion
  // graph disturbed by a code burst or by code and carrier that contradict
  // the Doppler: the Doppler-only gate rejects it.
  static void burstNeverSwitchesTest() {
    const Vec3 base(-3950000, 3350000, 3700000);
    // Without the Doppler-only gate the disturbed posterior velocity of the
    // motion graph switches; with it, never.
    for (const bool gate : {false, true}) {
      Engine e(Config{}, base, base, Vec3(.9, -.2, 1.1));
      e.doppler_gate_ = gate;
      int run = 0, longest = 0, fixes = 0, switched = -1;
      double fastest = 0;
      for (int i = 1; i <= 260 && switched < 0; ++i) {
        runImu(e, i, Motion{});
        const auto sol = e.process(burstEpoch(e, i));
        fixes += sol.fixed;
        if (sol.velocity.allFinite())
          fastest = std::max(fastest,
                             (e.R_.transpose() * sol.velocity).head<2>().norm());
        run = e.switch_reject_ >= 3 && e.switch_reject_ <= 6 ? run + 1 : 0;
        longest = std::max(longest, run);
        if (e.phase_ == 2)
          switched = i;
      }
      std::cout << "code burst at rest, Doppler gate " << (gate ? "on" : "off")
                << ": posterior speed up to " << fastest << " m/s, "
                << longest << " consecutive epochs stopped only by the gate, "
                << fixes << " FIX, switch at epoch " << switched << "\n";
      if (gate)
        require(switched < 0 && longest >= kWindowEpochs,
                "receiver at rest switched after a code burst");
      else
        require(switched > 0, "burst fixture no longer needs the Doppler gate");
    }
  }
  // A straight run with consistent (noisy) Doppler still switches after
  // a full window, every one of its epochs qualified.
  static void consistentDopplerSwitchTest() {
    const Vec3 base(6378137, 0, 0);
    Engine e(Config{}, base, base, Vec3(.5, .2, 1.));
    const auto motion = run(Vec3(3., -1., 0), 0., kAccel, kStill);
    int switched = -1;
    for (int i = 1; i <= 40 && switched < 0; ++i) {
      runImu(e, i, motion);
      e.process(runEpoch(e, i, motion, true));
      // From the first moving epoch above vel_thresh on, every epoch
      // qualifies.
      require(i < switchEpoch(e) - kWindowEpochs + 1 ||
                  e.switch_reject_ == 0,
              "consistent straight run was rejected");
      if (e.phase_ == 2)
        switched = i;
    }
    require(switched == switchEpoch(e),
            "consistent straight run did not switch after a full window");
  }
  // Heading [deg] of the body x axis of pose `key`: of its prior and of the
  // smoother estimate.
  static std::pair<double, double> headings(const Engine &e, Key key) {
    double prior_heading = std::numeric_limits<double>::quiet_NaN();
    for (const auto &factor : e.p2_->getFactors())
      if (auto prior =
              std::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(factor))
        if (prior->keys().front() == key) {
          const Vec3 x = prior->prior().rotation().matrix().col(0);
          prior_heading = std::atan2(x.x(), x.y()) * 180 / M_PI;
        }
    const Vec3 x =
        e.p2_->calculateEstimate<gtsam::Pose3>(key).rotation().matrix().col(0);
    return {prior_heading, std::atan2(x.x(), x.y()) * 180 / M_PI};
  }
  static double angleDiff(double a, double b) {
    return std::abs(std::remainder(a - b, 360.));
  }
  // (e) Reversing out of a parking position: accelerating backwards from
  // rest (body x against the velocity). The speed change from GNSS and the
  // longitudinal specific force have opposite signs, so the IMU graph starts
  // with the heading opposite to the velocity direction and keeps it.
  static void reverseStartTest() {
    const Vec3 base(6378137, 0, 0);
    Engine e(Config{}, base, base, Vec3(1.2, .1, .8));
    // Moving north while the body faces south, up to 3 m/s (a reversing
    // speed; at kSwitchForwardSpeed or more for kIntegrityTime check 1 would
    // declare the reversing IMU graph invalid).
    auto motion = run(Vec3(0., 1., 0), 0., kAccel, kStill, true);
    motion.vmax = 3.;
    int switched = -1;
    double error = 0, worst = 0;
    for (int i = 1; i <= switchEpoch(e) + 30; ++i) {
      runImu(e, i, motion);
      const auto sol = e.process(runEpoch(e, i, motion));
      require(sol.valid, "reverse run failed");
      if (switched < 0 && e.phase_ == 2) {
        switched = i;
        const Vec3 v = e.R_.transpose() * sol.velocity;
        const double prior = headings(e, poseKey(e.graph_epoch_)).first;
        require(angleDiff(prior, std::atan2(v.x(), v.y()) * 180 / M_PI + 180.) <
                        1e-9 &&
                    angleDiff(prior, 180.) < 1.,
                "reverse start: heading prior is not opposite to the velocity");
      }
      if (switched > 0 && i > switched) {
        worst = std::max(worst,
                         angleDiff(headings(e, poseKey(e.graph_epoch_)).second, 180.));
        require(sol.body_velocity.allFinite() &&
                    (sol.body_to_ecef.transpose() * sol.body_velocity).x() < 0,
                "reverse start: body x velocity is not negative");
      }
      error = std::max(error, (sol.position - truth(e, i, motion)).norm());
    }
    std::cout << "reverse start: switch at epoch " << switched
              << ", heading error after it up to " << worst
              << " deg, max antenna error " << error << " m\n";
    require(switched == switchEpoch(e), "reverse start did not switch");
    require(worst < 1. && error < .1, "reverse start: heading or position lost");
  }
  // (f) At constant speed the speed changes tell nothing: at 2 m/s the
  // direction stays undetermined and the node never switches; at 6 m/s
  // (above kSwitchForwardSpeed) forward motion is assumed and it switches
  // after a full window.
  static void constantSpeedTest() {
    const Vec3 base(6378137, 0, 0);
    for (const double speed : {2., 6.}) {
      Engine e(Config{}, base, base, Vec3(.5, .2, 1.));
      const auto motion = run(Vec3(1., 2., 0), speed, 0.);
      int switched = -1, undetermined = 0;
      for (int i = 1; i <= 40 && switched < 0; ++i) {
        runImu(e, i, motion);
        require(e.process(runEpoch(e, i, motion)).valid,
                "constant-speed run failed");
        undetermined += e.switch_reject_ == 10;
        if (e.phase_ == 2)
          switched = i;
      }
      if (speed < Engine::kSwitchForwardSpeed) {
        require(switched < 0 && undetermined > 30,
                "slow constant-speed run switched without a direction");
      } else {
        const double velocity_heading = std::atan2(1., 2.) * 180 / M_PI;
        require(switched == kWindowEpochs && undetermined == 0 &&
                    angleDiff(headings(e, poseKey(e.graph_epoch_)).first,
                              velocity_heading) < .1,
                "fast constant-speed run did not switch forward");
      }
    }
  }
  // ---- IMU-graph integrity ----
  // (h) An IMU graph whose heading is forced 180 degrees wrong at 8 m/s
  // (warm reset with the flipped attitude) falls back to the motion stage
  // after kIntegrityTime, switches again and ends with the right heading.
  static void headingFallbackTest() {
    const Vec3 base(6378137, 0, 0);
    Engine e(Config{}, base, base, Vec3(1.2, .1, .8));
    const auto motion = run(Vec3(2., 1., 0), 8., 0.);
    const double heading = std::atan2(2., 1.) * 180 / M_PI;
    int flipped = -1, fell = -1, again = -1;
    for (int i = 1; i <= 80; ++i) {
      runImu(e, i, motion);
      const auto sol = e.process(runEpoch(e, i, motion));
      require(sol.valid, "heading fallback run lost the solution");
      if (fell < 0 && e.n_fallback_ == 1) {
        fell = i;
        require(e.fallback_cause_ == 1 && e.phase_ == 1 && sol.phase == 1,
                "fallback was not a heading fallback to the motion stage");
      }
      if (fell > 0 && again < 0 && e.phase_ == 2)
        again = i;
      if (i == 15) {
        require(e.phase_ == 2, "8 m/s run did not switch");
        const auto pose =
            e.p2_->calculateEstimate<gtsam::Pose3>(poseKey(e.graph_epoch_));
        e.warmReset(e.base_ + e.R_ * pose.translation(),
                    pose.rotation() * gtsam::Rot3::Rz(M_PI));
        flipped = i;
      }
    }
    const double final_heading = headings(e, poseKey(e.graph_epoch_)).second;
    std::cout << "heading flipped at epoch " << flipped << ": fallback at "
              << fell << ", switch again at " << again << ", final heading error "
              << angleDiff(final_heading, heading) << " deg\n";
    require(fell >= flipped + 11 && fell <= flipped + 14,
            "flipped heading did not fall back after about 2 s");
    require(again > fell && e.phase_ == 2 && e.n_fallback_ == 1 &&
                angleDiff(final_heading, heading) < 1.,
            "after the fallback the IMU graph did not restart correctly");
  }
  // (i) A healthy straight run at 8 m/s (noisy data, a slip) never falls
  // back; check 1 sees small angles throughout.
  static void healthyNoFallbackTest() {
    const Vec3 base(6378137, 0, 0);
    Engine e(Config{}, base, base, Vec3(1.2, .1, .8));
    const auto motion = run(Vec3(-1., 3., 0), 8., 0.);
    for (int i = 1; i <= 300; ++i) {
      runImu(e, i, motion);
      require(e.process(runEpoch(e, i, motion, true)).valid,
              "healthy run lost the solution");
      require(e.n_fallback_ == 0, "healthy run fell back");
    }
    require(e.phase_ == 2, "healthy run left the IMU stage");
  }
  // (j) From epoch 20 the reference satellite's code (200 m) and carrier (5
  // cycles) are wrong in every epoch, so every DD row is off. The holds the
  // carrier contradicts are dropped, the reference ambiguity absorbs the
  // constant carrier offset and the Huber loss bounds the code: the IMU
  // graph keeps a valid solution near the trajectory and its heading, so no
  // fallback is needed.
  static void corruptedReferenceTest() {
    const Vec3 base(6378137, 0, 0);
    Engine e(Config{}, base, base, Vec3(1.2, .1, .8));
    const auto motion = run(Vec3(2., 1., 0), 8., 0.);
    double error = 0;
    for (int i = 1; i <= 50; ++i) {
      runImu(e, i, motion);
      auto obs = runEpoch(e, i, motion);
      if (i >= 20) {
        obs.sats[0].pr[0] += 200.;
        obs.sats[0].phase[0] += 5.;
      }
      const auto sol = e.process(obs);
      require(sol.valid, "corrupted reference: solution lost");
      if (i >= 20)
        error = std::max(error, (sol.position - truth(e, i, motion)).norm());
    }
    std::cout << "corrupted reference from epoch 20: " << e.n_fallback_
              << " fallbacks, max antenna error " << error << " m\n";
    require(e.phase_ == 2 && e.n_fallback_ == 0 && error < 2.,
            "corrupted reference: IMU graph lost or left the trajectory");
  }
  // An integer search that finds no acceptable candidate (ArOutcome::Zero),
  // however many epochs in a row, leaves the ambiguity keys and the holds as
  // they are. gnss_fgo at 5 m/s fixes and
  // holds first; from epoch 21 the ratio threshold cannot be met. The carrier
  // phases carry a small deterministic error, so the best candidate's
  // residual is not zero (a zero residual passes any ratio threshold).
  static void noCandidateKeepsAmbiguitiesTest() {
    Config c;
    c.gnss_motion = true;
    const Vec3 base(6378137, 0, 0);
    Engine e(c, base, base, Vec3::Zero());
    const auto motion = run(Vec3(1., .5, 0), 5., 0.);
    auto epoch = [&](int i) {
      auto obs = runEpoch(e, i, motion);
      for (std::size_t j = 0; j < obs.sats.size(); ++j)
        obs.sats[j].phase[0] += .02 * std::cos(2.3 * i + 1.7 * j);
      return obs;
    };
    for (int i = 1; i <= 20; ++i)
      e.process(epoch(i));
    require(!e.holds_.empty(), "no-candidate fixture: no hold before the run");
    const auto keys = e.sats_.keys();
    std::map<std::pair<Key, Key>, double> holds;
    for (const auto &[pair, record] : e.holds_)
      holds[pair] = record.difference;
    e.cfg_.ar_thresar = 1e12;
    for (int i = 21; i <= 80; ++i) {
      const auto sol = e.process(epoch(i));
      require(sol.valid && !sol.fixed && e.ar_outcome_ == Engine::ArOutcome::Zero,
              "no-candidate fixture: not a search without candidate");
      std::map<std::pair<Key, Key>, double> now;
      for (const auto &[pair, record] : e.holds_)
        now[pair] = record.difference;
      require(e.sats_.keys() == keys && now == holds,
              "epochs without an integer candidate changed the ambiguities");
    }
  }
  // ---- Integer search ----
  // A motion-stage engine (`e`, from engine()) with the pose at the base
  // (prior sigma 1 m) and one ambiguity key per satellite 1..n (system 0,
  // band 0) in its smoother, with a prior at value(sat) of sigma sigma(sat),
  // every key eligible for the search. The returned epoch has these
  // satellites with Doppler for the receiver velocity `velocity` (ENU); the
  // satellites move, so a receiver at rest has non-zero Doppler.
  static ObservationEpoch arFixture(Engine &e, int n,
                                    const std::function<double(int)> &value,
                                    const std::function<double(int)> &sigma,
                                    const Vec3 &velocity) {
    Engine::Graph priors;
    Engine::Values initial;
    gtsam::FixedLagSmoother::KeyTimestampMap ts;
    initial.insert(poseKey(0), gtsam::Pose3());
    priors.addPrior<gtsam::Pose3>(poseKey(0), gtsam::Pose3(),
                                  gtsam::noiseModel::Isotropic::Sigma(6, 1));
    ts[poseKey(0)] = 0;
    ObservationEpoch obs;
    obs.gpst = obs.base_gpst = kT0;
    for (int sat = 1; sat <= n; ++sat) {
      const auto key = ambKey({sat, 0});
      initial.insert(key, value(sat));
      priors.addPrior<double>(
          key, value(sat), gtsam::noiseModel::Isotropic::Sigma(1, sigma(sat)));
      ts[key] = 0;
      e.sats_.get({sat, 0}).key = key;
      e.current_.visible.insert({sat, 0});
      SatelliteObservation s;
      s.sat = s.prn = sat;
      s.system = 0;
      s.elevation = 1.;
      const Vec3 los =
          Vec3(.2 + .2 * sat, std::sin(sat), std::cos(sat)).normalized();
      s.rover_sat = s.base_sat = e.base_ + los * 2e7;
      s.sat_velocity = Vec3(1e3 * std::sin(2. * sat), 2e3 * std::cos(sat), 5e2);
      s.pr[0] = s.pr_base[0] = 2e7;
      s.wavelength[0] = s.doppler_wavelength[0] = .19;
      s.doppler[0] = los.dot(e.R_ * velocity - s.sat_velocity) / .19;
      obs.sats.push_back(s);
    }
    e.p2_->update(priors, initial, ts);
    return obs;
  }
  static double exact(int sat) { return 10. * sat; }
  static double tight(int) { return 1e-3; }
  static ArDecision search(Engine &e, const ObservationEpoch &obs) {
    return e.resolve(*e.p2_, e.p2_->calculateEstimate(), poseKey(0), obs);
  }
  // (1) Partial AR: one young arc whose float value lies between two
  // integers makes the full set fail the ratio test; the retry on the arcs
  // at least kMatureArcEpochs old fixes the other ten.
  static void partialArTest() {
    auto e = engine();
    const int n = 11;
    const auto obs = arFixture(
        e, n, [](int sat) { return sat == n ? 110.5 : exact(sat); },
        [](int sat) { return sat == n ? .3 : tight(sat); }, Vec3(5, 0, 0));
    e.gnss_epochs_ = 100;
    for (int sat = 1; sat <= n; ++sat)
      e.arc_created_[ambKey({sat, 0})] =
          sat == n ? 100 - Engine::kMatureArcEpochs + 1
                   : 100 - Engine::kMatureArcEpochs;
    const auto ar = search(e, obs);
    require(ar.nb == n - 2 && !ar.held_only &&
                ar.fixed_pairs.size() == std::size_t(n - 2),
            "partial AR did not fix the mature arcs");
    for (const auto &[r, j] : ar.fixed_pairs)
      require(j.first != n && r.first != n, "partial AR fixed the young arc");
    require(ar.fixed_position.allFinite() && ar.s0 >= 0 && ar.s1 > 0,
            "partial AR decision incomplete");
  }
  // (2) A fix needs kMinFixPairs pairs: 7 pairs that pass the ratio test are
  // refused, 8 are accepted; the minimum-satellite rule applies as well.
  static void minimumPairsTest() {
    for (const int n : {Engine::kMinFixPairs, Engine::kMinFixPairs + 1}) {
      auto e = engine();
      const auto obs = arFixture(e, n, exact, tight, Vec3(5, 0, 0));
      const auto ar = search(e, obs);
      require(n == Engine::kMinFixPairs ? ar.nb == 0 : ar.nb == n - 1,
              "fix minimum of DD pairs");
    }
    auto e = engine();
    e.cfg_.ar_minfixsats = Engine::kMinFixPairs + 2;
    const auto obs =
        arFixture(e, Engine::kMinFixPairs + 1, exact, tight, Vec3(5, 0, 0));
    require(search(e, obs).nb == 0, "AR accepted fewer than ar_minfixsats");
  }
  // (5) With fewer than kFewHeldArcs held arcs, a receiver at rest (Doppler-
  // only speed 0) needs kStationaryMinFixPairs pairs: 9 are refused at rest,
  // accepted when moving, and accepted at rest with kFewHeldArcs held arcs.
  static void stationaryMinimumTest() {
    const int n = Engine::kStationaryMinFixPairs;
    for (const int moving : {0, 1}) {
      auto e = engine();
      const auto obs =
          arFixture(e, n, exact, tight, moving ? Vec3(4, 3, 0) : Vec3::Zero());
      require(e.stationary(obs, e.p2_->calculateEstimate(), poseKey(0)) ==
                  !moving,
              "stationary fixture");
      const auto ar = search(e, obs);
      require(moving ? ar.nb == n - 1 : ar.nb == 0,
              "stationary fix minimum with few held arcs");
    }
    auto e = engine();
    const auto obs = arFixture(e, n, exact, tight, Vec3::Zero());
    for (int sat = 1; sat <= Engine::kFewHeldArcs; ++sat)
      e.sats_.get({sat, 0}).activate(exact(sat));
    require(search(e, obs).nb == n - 1,
            "stationary minimum applied with enough held arcs");
  }
  // (4) The held arcs alone: the full set fails on a mature arc between two
  // integers (so the partial retry is the full set again); the 11 held arcs
  // (10 pairs) that reproduce their hold integers give an output-only FIX,
  // with no state change. With one hold integer different, or 10 held arcs
  // (9 pairs), no FIX.
  static void heldSetTest() {
    const auto run = [](int held, double offset) {
      auto e = engine();
      const int n = 12;
      const auto obs = arFixture(
          e, n, [](int sat) { return sat == n ? 120.5 : exact(sat); },
          [](int sat) { return sat == n ? .3 : tight(sat); }, Vec3(5, 0, 0));
      e.gnss_epochs_ = 100;
      for (int sat = 1; sat <= n; ++sat)
        e.arc_created_[ambKey({sat, 0})] = 0;
      for (int sat = 1; sat <= held; ++sat) {
        e.sats_.get({sat, 0}).activate(exact(sat));
        if (sat > 1)
          e.holds_[{ambKey({1, 0}), ambKey({sat, 0})}] = {
              exact(sat) - exact(1) + (sat == 5 ? offset : 0.), 0};
      }
      const auto sats = e.sats_.track;
      const auto holds = e.holds_.size();
      const auto ar = e.runAr(*e.p2_, e.p2_->calculateEstimate(), poseKey(0),
                              obs);
      require(e.current_.accepted.empty() && e.holds_.size() == holds &&
                  e.ar_outcome_ == Engine::ArOutcome::Zero,
              "held-set attempt changed the AR bookkeeping");
      for (const auto &[sf, st] : sats)
        require(e.sats_.track.at(sf).key == st.key &&
                    e.sats_.track.at(sf).held == st.held,
                "held-set attempt changed an arc or a hold");
      return ar;
    };
    const auto fixed = run(11, 0.);
    require(fixed.held_only && fixed.nb == 10 &&
                fixed.fixed_position.allFinite(),
            "held-set attempt did not fix the agreeing held arcs");
    require(!run(11, 1.).nb, "held-set attempt fixed against a hold integer");
    require(!run(10, 0.).nb, "held-set attempt fixed with 9 pairs");
  }
  // (3) A hold whose residual exceeds kHoldRejectCycles is removed and its
  // satellite, in no other hold record, loses the hold but keeps its key; a
  // hold .2 cycle off and the others stay.
  static void holdRejectTest() {
    auto e = engine();
    arFixture(e, 4, exact, tight, Vec3::Zero());
    ArDecision ar;
    ar.nb = 3;
    const SF ref{1, 0};
    ar.fixed[ref] = exact(1);
    for (int sat = 2; sat <= 4; ++sat) {
      ar.fixed[{sat, 0}] =
          exact(sat) + (sat == 3 ? 1. : 0.) + (sat == 4 ? .2 : 0.);
      ar.fixed_pairs.push_back({ref, {sat, 0}});
    }
    for (auto &[sf, v] : ar.fixed)
      ar.allowed.insert(sf);
    e.hold(*e.p2_, ar);
    require(e.holds_.size() == 3, "hold-rejection fixture");
    const Key k1 = ambKey(ref), k3 = ambKey({3, 0});
    e.rejectHolds(*e.p2_, e.p2_->calculateEstimate(), poseKey(0));
    require(e.holds_.size() == 2 && !e.holds_.count({k1, k3}) &&
                holdsOn(e, k1, k3).empty(),
            "inconsistent hold was not removed");
    for (int sat : {2, 4})
      require(holdsOn(e, k1, ambKey({sat, 0})).size() == 1 &&
                  e.sats_.get({sat, 0}).held,
              "consistent hold was removed");
    const auto &st = e.sats_.get({3, 0});
    require(!st.held && !st.last_held && st.key == k3 && e.sats_.get(ref).held,
            "rejected hold: wrong satellite state");
  }
  // (6) gnss_fgo restarts every un-held arc after kFloatRestartEpochs GNSS
  // epochs without a fix while fewer than kFewHeldArcs arcs are held (here
  // with ambiguity resolution off); gnss_imu_fgo keeps its arcs.
  static void floatRestartTest() {
    const Vec3 base(6378137, 0, 0);
    Config gc, ic;
    gc.gnss_motion = true;
    gc.ar_mode = ic.ar_mode = 0;
    Engine g(gc, base, base, Vec3::Zero()), m(ic, base, base, Vec3(.5, .2, 1.));
    std::optional<AmbMap> first_g, first_m;
    int restart_g = -1;
    for (int i = 1; i <= Engine::kFloatRestartEpochs + 5; ++i) {
      runImu(m, i, Motion{});
      g.process(runEpoch(g, i, Motion{}));
      m.process(runEpoch(m, i, Motion{}));
      if (!first_g) {
        first_g = g.sats_.keys();
        first_m = m.sats_.keys();
      }
      require(m.sats_.keys() == *first_m, "gnss_imu_fgo restarted its arcs");
      if (restart_g < 0 && g.sats_.keys() != *first_g) {
        restart_g = i;
        require(g.sats_.keys().empty(), "float restart kept an arc");
      }
    }
    require(first_g->size() == std::size_t(kRunSats) &&
                restart_g == Engine::kFloatRestartEpochs,
            "gnss_fgo did not restart its float arcs");
    for (const auto &[sf, key] : g.sats_.keys())
      require(key != first_g->at(sf), "restarted arc reused its key");
    // The held count is the one of the stationary fix minimum: held arcs
    // outside this epoch's search (no carrier DD this epoch) do not stop the
    // restart. kFewHeldArcs held arcs, all without a carrier DD this epoch.
    auto e = engine();
    const int n = Engine::kFewHeldArcs + 4;
    const auto obs = arFixture(e, n, exact, tight, Vec3::Zero());
    for (int sat = 1; sat <= Engine::kFewHeldArcs; ++sat) {
      e.sats_.get({sat, 0}).activate(exact(sat));
      e.current_.visible.erase({sat, 0});
    }
    const auto arcs = e.searchArcs(obs, e.p2_->calculateEstimate());
    require(arcs.size() == std::size_t(n - Engine::kFewHeldArcs) &&
                e.heldArcs(arcs) == 0,
            "held arcs without a carrier DD counted as search arcs");
    e.epoch_held_arcs_ = e.heldArcs(arcs);
    e.epoch_hold_fix_ = false;
    e.fixless_epochs_ = Engine::kFloatRestartEpochs - 1;
    e.restartFloatArcs();
    for (int sat = 1; sat <= n; ++sat)
      require(bool(e.sats_.get({sat, 0}).key) == (sat <= Engine::kFewHeldArcs),
              "held arcs outside the search stopped the float restart");
  }
  // (7) An epoch whose code DDs of five satellites are 40 m off (the
  // reference clean) keeps its carrier DDs, in that epoch and after, and the
  // holds stay.
  static void codeOutlierKeepsCarrierTest() {
    const Vec3 base(6378137, 0, 0);
    Config c;
    c.gnss_motion = true;
    Engine e(c, base, base, Vec3::Zero());
    int carrier = 0;
    e.observeGraph([&](int, const gtsam::NonlinearFactorGraph &graph,
                       const gtsam::Values &) {
      carrier = 0;
      for (const auto &f : graph)
        carrier += bool(std::dynamic_pointer_cast<
                        gtsam::DoubleDifferenceCarrierPhaseFactorArm>(f));
    });
    for (int i = 1; i <= 20; ++i)
      e.process(runEpoch(e, i, Motion{}));
    require(e.holds_.size() == std::size_t(kRunSats - 1),
            "code-outlier fixture: no holds");
    std::map<std::pair<Key, Key>, double> holds;
    for (const auto &[pair, record] : e.holds_)
      holds[pair] = record.difference;
    int fixes = 0;
    for (int i = 21; i <= 28; ++i) {
      auto obs = runEpoch(e, i, Motion{});
      if (i == 21)
        for (int j = kRunSats - 5; j < kRunSats; ++j)
          obs.sats[j].pr[0] += 40.;
      const auto sol = e.process(obs);
      fixes += sol.fixed;
      std::map<std::pair<Key, Key>, double> now;
      for (const auto &[pair, record] : e.holds_)
        now[pair] = record.difference;
      require(sol.valid && carrier == kRunSats - 1 && now == holds,
              "code outliers suppressed the carrier or dropped holds");
    }
    require(fixes == 8, "code-outlier epochs lost the fix");
  }
  // (8) The code DDs carry a Huber loss (kCodeHuber): with one code
  // pseudorange 50 m off, the code-only position moves far less than with
  // the same factors without it.
  static void codeHuberTest() {
    auto e = engine();
    const Vec3 truth = e.base_ + e.R_ * Vec3(30, -20, 5);
    ObservationEpoch obs;
    obs.gpst = obs.base_gpst = kT0;
    for (int j = 0; j < kRunSats; ++j) {
      SatelliteObservation s;
      s.sat = s.prn = j + 1;
      s.system = 0;
      s.elevation = .7;
      s.rover_sat = s.base_sat =
          e.base_ +
          Vec3(.2 + .2 * j, std::sin(j), std::cos(j)).normalized() * 2e7;
      s.pr[0] = range(s.rover_sat, truth) + (j == 5 ? 50. : 0.);
      s.pr_base[0] = range(s.base_sat, e.base_);
      obs.sats.push_back(s);
    }
    Engine::Graph graph, robust, plain;
    DdRows rows;
    Engine::Values values;
    e.buildDd(graph, rows, values, obs, poseKey(0), Vec3::Zero());
    require(rows.size() == std::size_t(kRunSats - 1), "Huber fixture rows");
    for (const auto &row : rows) {
      const auto f =
          std::dynamic_pointer_cast<gtsam::NoiseModelFactor>(graph[row.index]);
      const auto model =
          f ? std::dynamic_pointer_cast<gtsam::noiseModel::Robust>(
                  f->noiseModel())
            : nullptr;
      const auto huber =
          model ? std::dynamic_pointer_cast<
                      gtsam::noiseModel::mEstimator::Huber>(model->robust())
                : nullptr;
      require(!row.carrier && huber &&
                  huber->modelParameter() == Engine::kCodeHuber,
              "code DD without the Huber loss");
      robust.push_back(f);
      plain.push_back(f->cloneWithNewNoiseModel(model->noise()));
    }
    const auto solve = [&](Engine::Graph g) {
      gtsam::Vector6 sigmas;
      sigmas << 1e-3, 1e-3, 1e-3, 1e3, 1e3, 1e3;
      g.addPrior<gtsam::Pose3>(poseKey(0), gtsam::Pose3(),
                               gtsam::noiseModel::Diagonal::Sigmas(sigmas));
      Engine::Values v;
      v.insert(poseKey(0), gtsam::Pose3());
      const auto est = gtsam::LevenbergMarquardtOptimizer(g, v).optimize();
      return (e.base_ + e.R_ * est.at<gtsam::Pose3>(poseKey(0)).translation() -
              truth)
          .norm();
    };
    const double with = solve(robust), without = solve(plain);
    std::cout << "code outlier 50 m: position error " << with
              << " m with the Huber loss, " << without << " m without\n";
    require(without > 1. && with < .5 * without,
            "Huber loss did not bound the code outlier");
  }
  // ---- Plausibility guard ----
  // A posterior forced 10,000 km away is not committed or published, in the
  // motion graph (gnss_fgo) and in the IMU graph: the epoch restarts from the
  // DD code position and the next epochs follow the trajectory. The update
  // probe, before the epoch update, moves the previous state there with a
  // prior (as a graph left indeterminate by an earlier update would), so the
  // epoch's posterior follows it through the motion or IMU factor. Returns the
  // larger error, after the faulted epoch, of its published solution (0 when
  // invalid) and of the engine position the node computes the elevation mask
  // from (floatPosition).
  static double implausiblePosteriorRun(bool imu, bool guard) {
    Config c;
    c.gnss_motion = !imu;
    const Vec3 base(6378137, 0, 0);
    Engine e(c, base, base, imu ? Vec3(.5, .2, 1.) : Vec3::Zero());
    e.plausibility_guard_ = guard;
    const auto motion = run(Vec3(2., .5, 0), 0., kAccel, kStill);
    const int inject = switchEpoch(e) + 5;
    bool armed = false;
    e.update_probe_ = [&](const char *operation, bool after) {
      if (!armed || std::string(operation) != "fls" || after)
        return;
      armed = false;
      Engine::Graph pull;
      pull.addPrior<gtsam::Pose3>(
          poseKey(e.graph_epoch_), gtsam::Pose3(gtsam::Rot3(), Vec3(1e7, 0, 0)),
          gtsam::noiseModel::Isotropic::Sigma(6, 1e-3));
      e.p2_->update(pull, {}, {});
    };
    double error = 0;
    for (int i = 1; i <= (guard ? inject + 20 : inject); ++i) {
      if (imu)
        runImu(e, i, motion);
      armed = i == inject;
      if (armed)
        require(e.phase_ == 2 && (imu || e.cfg_.gnss_motion),
                "fault fixture: not in the graph under test");
      const auto sol = e.process(runEpoch(e, i, motion));
      const Vec3 t = truth(e, i, motion);
      if (i == inject)
        error = std::max(sol.valid ? (sol.position - t).norm() : 0.,
                         (e.floatPosition() - t).norm());
      if (!guard)
        continue;
      if (i == inject)
        require(sol.failure.find("implausible") != std::string::npos,
                "implausible posterior was not rejected");
      require((e.floatPosition() - t).norm() < 50.,
              "position for the elevation mask left the trajectory");
      if (i > inject + 10)
        require(sol.valid && sol.kind == UpdateKind::Gnss &&
                    (sol.position - t).norm() < 1.,
                "estimator did not recover from the rejected posterior");
    }
    return error;
  }
  static void implausiblePosteriorTest() {
    for (const bool imu : {false, true}) {
      const double unguarded = implausiblePosteriorRun(imu, false);
      // The fixture must reach the absurd posterior.
      require(unguarded > 1e5, "forced-posterior fixture no longer jumps");
      const double guarded = implausiblePosteriorRun(imu, true);
      std::cout << (imu ? "IMU graph" : "motion graph")
                << ": forced posterior " << unguarded
                << " m off without the plausibility guard, " << guarded
                << " m with it\n";
      require(guarded < 5., "implausible posterior was committed or published");
    }
  }
  // ---- IMU mechanisation and vehicle constraints ----
  // A still start, then 2 m/s^2 along (2, .5) up to `vmax`, with the IMU of
  // runImu (Earth rotation and normal gravity in the samples) plus the gyro
  // bias `gyro_bias`; ambiguity resolution as configured.
  static void imuRun(Engine &e, int i, const Motion &m, const Vec3 &gyro_bias,
                     const Mat3 &mount = Mat3::Identity()) {
    for (int k = 1; k <= 20; ++k) {
      const double t = kT0 + (i - 1) * .2 + k * .01;
      auto sample = imuSample(e, t, runBody(m, mount),
                              m.dir * m.acceleration(t), m.dir * m.speed(t));
      sample.gyro += gyro_bias;
      e.addImu(sample);
    }
  }
  // The gyro senses the Earth rotation: the initial bias from the leading
  // still samples is their mean less the Earth rate in the body frame, and
  // the IMU-graph epochs carry no bias factor besides the IMU factor.
  static void earthRateBiasTest() {
    Config c;
    c.ar_mode = 0;
    const Vec3 base(6378137, 0, 0), bias(2e-4, -1e-4, 3e-4);
    Engine e(c, base, base, Vec3(.5, .2, 1.));
    int bias_factors = 0, imu_epochs = 0;
    e.observeGraph([&](int, const gtsam::NonlinearFactorGraph &graph,
                       const gtsam::Values &) {
      if (e.phase_ != 2)
        return;
      ++imu_epochs;
      for (const auto &f : graph)
        bias_factors +=
            bool(std::dynamic_pointer_cast<gtsam::BetweenFactor<
                     gtsam::imuBias::ConstantBias>>(f)) ||
            bool(std::dynamic_pointer_cast<
                 gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(f));
    });
    const auto motion = run(Vec3(2., .5, 0), 0., kAccel, kStill);
    for (int i = 1; i <= 40; ++i) {
      imuRun(e, i, motion, bias);
      require(e.process(runEpoch(e, i, motion)).valid,
              "Earth-rate run lost the solution");
      if (e.phase_ == 2 && i == switchEpoch(e)) {
        const Vec3 b0 = e.initial_bias_.gyroscope();
        std::cout << "initial gyro bias error " << (b0 - bias).norm()
                  << " rad/s (Earth rate " << kEarthRate << ")\n";
        // The Earth rate is removed in the first window body frame, whose
        // heading comes from the estimated velocity (a fraction of a degree
        // off at 1-2 m/s): 1e-6 rad/s allows 0.8 deg, the Earth rate is 73
        // times larger.
        require((b0 - bias).norm() < 1e-6,
                "initial gyro bias keeps the Earth rotation");
      }
    }
    require(e.phase_ == 2 && imu_epochs > 10 && bias_factors == 0,
            "IMU-graph epochs carry bias factors besides the IMU factor");
  }
  // The IMU chain of an epoch is the CombinedImuFactor alone: the bias
  // evolves through its random walk, no between factor, no bias prior.
  static void biasChainTest() {
    auto e = engine();
    std::vector<ImuSample> samples;
    for (int k = 1; k <= 20; ++k)
      samples.push_back({kT0 + .01 * k, Vec3(0, 0, 9.8), Vec3::Zero(), {}});
    const auto im = integrateImu(e.cfg_, {}, samples, kT0, kT0 + .2,
                                 e.imuFrame(Vec3::Zero()));
    Engine::Graph graph;
    Engine::Values values;
    e.addImuChain(graph, values, 3, im, gtsam::Pose3(), Vec3::Zero());
    require(graph.size() == 1 &&
                std::dynamic_pointer_cast<gtsam::CombinedImuFactor>(graph[0]),
            "IMU chain has factors besides the CombinedImuFactor");
  }
  // Bias priors of the switch window (in the smoother right after the
  // switch): gyroscope sigma kStillBiasGyroSigma when the initial bias is
  // the mean of a still start, all .01 when the run starts moving.
  // The engine right after the switch, from a still or a moving start.
  static std::unique_ptr<Engine> switchedEngine(bool still) {
    Config c;
    c.ar_mode = 0;
    const Vec3 base(6378137, 0, 0);
    auto engine = std::make_unique<Engine>(c, base, base, Vec3(.5, .2, 1.));
    Engine &e = *engine;
    const auto motion = still ? run(Vec3(2., .5, 0), 0., kAccel, kStill)
                              : run(Vec3(2., .5, 0), 6., 0.);
    for (int i = 1; i <= 40 && e.phase_ == 1; ++i) {
      for (int k = 1; k <= 20; ++k) {
        const double t = kT0 + (i - 1) * .2 + k * .01;
        auto sample = imuSample(e, t, runBody(motion),
                                motion.dir * motion.acceleration(t),
                                motion.dir * motion.speed(t));
        // A moving start: vibration in the first second, so no still block.
        if (!still && t < kT0 + 1.)
          sample.acc.x() += k % 2 ? .4 : -.4;
        e.addImu(sample);
      }
      e.process(runEpoch(e, i, motion));
    }
    require(e.phase_ == 2, "bias-prior fixture did not switch");
    require(!still == (e.levelImu().source == Engine::LevelImu::Bias::Zero),
            "bias-prior fixture: wrong initial bias source");
    return engine;
  }
  static std::vector<gtsam::Vector> switchBiasSigmas(bool still) {
    const auto engine = switchedEngine(still);
    const Engine &e = *engine;
    std::vector<gtsam::Vector> sigmas;
    for (const auto &f : e.p2_->getFactors())
      if (const auto prior = std::dynamic_pointer_cast<
              gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(f))
        sigmas.push_back(prior->noiseModel()->sigmas());
    return sigmas;
  }
  // The Earth rate is subtracted only from a measured initial bias: a still
  // start's b0 (the still-block mean) loses exactly the Earth rate in the
  // first window body frame; a moving start's b0 is unknown (zero) and stays
  // exactly zero.
  static void initialBiasEarthRateTest() {
    for (const bool still : {true, false}) {
      const auto engine = switchedEngine(still);
      const Engine &e = *engine;
      const auto measured = e.levelImu().b0;
      const auto &initial = e.initial_bias_;
      if (still) {
        const Vec3 removed = measured.gyroscope() - initial.gyroscope();
        require(initial.accelerometer() == measured.accelerometer() &&
                    std::abs(removed.norm() - kEarthRate) < 1e-12,
                "still start: Earth rate not removed from the initial bias");
      } else {
        require((measured.vector().array() == 0).all() &&
                    (initial.vector().array() == 0).all(),
                "moving start: the unknown initial bias is not zero");
      }
    }
  }
  static void switchBiasPriorTest() {
    gtsam::Vector6 tight, loose;
    tight << .01, .01, .01, Vec3::Constant(Engine::kStillBiasGyroSigma);
    loose.setConstant(.01);
    for (const bool still : {true, false}) {
      const auto sigmas = switchBiasSigmas(still);
      require(!sigmas.empty(), "switch window without bias priors");
      for (const auto &s : sigmas)
        require(s.isApprox(still ? tight : loose),
                "switch-window bias prior sigma");
    }
  }
  // The NHC mounting pitch: an IMU pitched 3 deg against the vehicle (the
  // travel axis 3 deg below the IMU x axis, theta = 3 deg) on a straight
  // run; the IMU graph recovers it.
  static void mountingAngleTest() {
    Config c;
    c.nhc_enable = 1;
    const Vec3 base(6378137, 0, 0);
    Engine e(c, base, base, Vec3(.5, .2, 1.));
    auto motion = run(Vec3(2., .5, 0), 0., kAccel, kStill);
    motion.vmax = 10.;
    const double theta = 3. * M_PI / 180.;
    const Mat3 mount = gtsam::Rot3::Ry(-theta).matrix();
    double error = 0;
    for (int i = 1; i <= 150; ++i) {
      imuRun(e, i, motion, Vec3::Zero(), mount);
      const auto sol = e.process(runEpoch(e, i, motion));
      require(sol.valid, "mounting-angle run lost the solution");
      error = std::max(error, (sol.position - truth(e, i, motion)).norm());
    }
    require(e.phase_ == 2 && e.last_bore_, "mounting-angle run: no estimate");
    const double pitch = *e.last_bore_ * 180. / M_PI;
    std::cout << "mounting pitch " << pitch << " deg (true 3), max antenna "
              << "error " << error << " m\n";
    require(std::abs(pitch - 3.) < .3, "3 deg mounting pitch not recovered");
  }
  // The attitude survives the recovery path: an implausible IMU-graph
  // posterior restarts the graph (process()) from the last committed body
  // rotation, not from an unlevelled identity.
  static void attitudeRecoveryTest() {
    Config c;
    const Vec3 base(6378137, 0, 0);
    Engine e(c, base, base, Vec3(.5, .2, 1.));
    const auto motion = run(Vec3(2., .5, 0), 0., kAccel, kStill);
    const int inject = switchEpoch(e) + 5;
    bool armed = false;
    e.update_probe_ = [&](const char *operation, bool after) {
      if (!armed || std::string(operation) != "fls" || after)
        return;
      armed = false;
      Engine::Graph pull;
      pull.addPrior<gtsam::Pose3>(
          poseKey(e.graph_epoch_), gtsam::Pose3(gtsam::Rot3(), Vec3(1e7, 0, 0)),
          gtsam::noiseModel::Isotropic::Sigma(6, 1e-3));
      e.p2_->update(pull, {}, {});
    };
    for (int i = 1; i <= inject; ++i) {
      runImu(e, i, motion);
      armed = i == inject;
      const auto committed = e.last_rotation_;
      const auto sol = e.process(runEpoch(e, i, motion));
      if (i < inject)
        continue;
      require(committed && sol.failure.find("implausible") != std::string::npos &&
                  e.phase_ == 2 && e.graph_epoch_ == 0,
              "attitude-recovery fixture did not restart the IMU graph");
      const auto restart =
          e.p2_->calculateEstimate<gtsam::Pose3>(poseKey(0)).rotation();
      const Vec3 x = committed->matrix().col(0);
      std::cout << "IMU-graph restart: attitude change "
                << committed->between(restart).axisAngle().second * 180. / M_PI
                << " deg (body x heading "
                << std::atan2(x.x(), x.y()) * 180. / M_PI << " deg)\n";
      require(restart.equals(*committed, 1e-9),
              "IMU-graph restart lost the committed attitude");
    }
  }
  // A body at rest 90 km from the base, its IMU sensing exactly the specific
  // force and the Earth rotation there: one second of prediction in the
  // frame of imuFrame keeps it at rest (GTSAM's centrifugal term for the
  // offset from the base and the normal gravity at the body are not both
  // counted; counting both would be W^2 times the offset across the
  // rotation axis, about 4e-4 m/s^2). The tolerance covers the
  // preintegration's discretization of the Earth rotation at 100 Hz (about
  // 3e-6 m/s per second).
  static void rotatingFrameGravityTest() {
    const Vec3 base(-3950000, 3350000, 3700000), p(80000., 40000., 30.);
    Engine e(Config{}, base, base, Vec3::Zero());
    const Vec3 w = earthRate(e.R_);
    std::vector<ImuSample> samples;
    for (int k = 1; k <= 100; ++k)
      samples.push_back({kT0 + .01 * k,
                         -localGravity(e.base_ + e.R_ * p, e.R_), w, {}});
    const auto im = integrateImu(e.cfg_, {}, samples, kT0, kT0 + 1.,
                                 e.imuFrame(p));
    const auto next = im.pim.predict(
        gtsam::NavState(gtsam::Pose3(gtsam::Rot3(), p), Vec3::Zero()), {});
    std::cout << "body at rest 90 km from the base, 1 s: velocity "
              << next.velocity().norm() << " m/s, position "
              << (next.position() - p).norm() << " m\n";
    require(next.velocity().norm() < 2e-5 && (next.position() - p).norm() < 1e-5,
            "rotating-frame prediction moves a body at rest");
  }
};
} // namespace gnss_fgo
int main() {
  EngineTestAccess::vehicleConstraintSwitches();
  EngineTestAccess::standstillTest();
  EngineTestAccess::zeroDopplerTest();
  EngineTestAccess::holdTest(false);
  EngineTestAccess::holdTest(true);
  EngineTestAccess::fdeTest(false);
  EngineTestAccess::fdeTest(true);
  EngineTestAccess::phase2HoldTest();
  EngineTestAccess::arcEndTest();
  EngineTestAccess::gaugeTest();
  EngineTestAccess::gaugeImuRegression();
  EngineTestAccess::resetTest();
  EngineTestAccess::initializationTest();
  EngineTestAccess::switchAmbiguityTest();
  EngineTestAccess::motionStageEqualsGnssFgo();
  EngineTestAccess::switchWithoutFixTest();
  EngineTestAccess::stationaryNeverSwitchesTest();
  EngineTestAccess::failedSwitchTest();
  EngineTestAccess::switchHeadingTest();
  EngineTestAccess::posePriorFloorTest();
  EngineTestAccess::burstNeverSwitchesTest();
  EngineTestAccess::consistentDopplerSwitchTest();
  EngineTestAccess::reverseStartTest();
  EngineTestAccess::constantSpeedTest();
  EngineTestAccess::headingFallbackTest();
  EngineTestAccess::healthyNoFallbackTest();
  EngineTestAccess::corruptedReferenceTest();
  EngineTestAccess::ddprFinalSetTest();
  EngineTestAccess::futureImuAdmissionTest();
  EngineTestAccess::imuBacklogTest();
  EngineTestAccess::midPeriodImuTest();
  EngineTestAccess::noCandidateKeepsAmbiguitiesTest();
  EngineTestAccess::implausiblePosteriorTest();
  EngineTestAccess::partialArTest();
  EngineTestAccess::minimumPairsTest();
  EngineTestAccess::holdRejectTest();
  EngineTestAccess::heldSetTest();
  EngineTestAccess::stationaryMinimumTest();
  EngineTestAccess::floatRestartTest();
  EngineTestAccess::codeOutlierKeepsCarrierTest();
  EngineTestAccess::codeHuberTest();
  EngineTestAccess::earthRateBiasTest();
  EngineTestAccess::biasChainTest();
  EngineTestAccess::initialBiasEarthRateTest();
  EngineTestAccess::switchBiasPriorTest();
  EngineTestAccess::mountingAngleTest();
  EngineTestAccess::attitudeRecoveryTest();
  EngineTestAccess::rotatingFrameGravityTest();
  std::cout << "graph transaction regressions passed\n";
}
