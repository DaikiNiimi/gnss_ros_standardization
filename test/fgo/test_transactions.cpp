#include "engine.hpp"
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
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
      std::vector<ImuSample> samples(20);
      for (auto &s : samples)
        s.acc = Vec3(0, 0, 9.81);
      e.addPseudo(graph, 3, gtsam::Pose3(), Vec3::Zero(), samples, Vec3::Zero());
      require(enabled == (graph.size() > 0),
              "NHC/ZUPT factors follow their switches");
    }
  }
  // The IMU statistics of these samples pass the stationarity test; the
  // previous posterior speed decides whether ZUPT applies.
  static void zuptSpeedGate() {
    std::vector<ImuSample> samples(20);
    for (auto &s : samples)
      s.acc = Vec3(0, 0, 9.81);
    auto count = [](const Engine::Graph &graph) {
      int velocity = 0, rotation = 0, anchor = 0, nhc = 0;
      for (const auto &f : graph) {
        velocity += bool(
            dynamic_cast<const gtsam::PriorFactor<gtsam::Vector3> *>(f.get()));
        rotation += bool(
            dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose3> *>(f.get()));
        anchor += bool(
            dynamic_cast<const gtsam::PriorFactor<gtsam::Pose3> *>(f.get()));
        nhc += bool(dynamic_cast<const NhcFactor *>(f.get()));
      }
      return std::array<int, 4>{velocity, rotation, anchor, nhc};
    };
    Config c;
    c.zupt_enable = 1;
    {
      Engine e(c, Vec3(-3950000, 3350000, 3700000),
               Vec3(-3950000, 3350000, 3700000), Vec3::Zero());
      Engine::Graph first, second;
      e.addPseudo(first, 3, gtsam::Pose3(), Vec3::Zero(), samples,
                  Vec3::Zero());
      require(count(first) == std::array<int, 4>{1, 1, 0, 0} && e.zupt_anchor_,
              "stationary epoch: zero velocity and rotation, anchor set");
      e.addPseudo(second, 4, gtsam::Pose3(), Vec3::Zero(), samples,
                  Vec3::Zero());
      require(count(second) == std::array<int, 4>{1, 1, 1, 0},
              "next stationary epoch adds the anchor prior");
      Engine::Graph moving;
      e.addPseudo(moving, 5, gtsam::Pose3(), Vec3(1, 0, 0), samples,
                  Vec3::Zero());
      require(moving.empty() && !e.zupt_anchor_,
              "moving epoch: no ZUPT factors and the anchor is dropped");
    }
    for (const auto &[speed, pass] :
         std::vector<std::pair<double, bool>>{{.49, true}, {.51, false}}) {
      Engine e(c, Vec3(-3950000, 3350000, 3700000),
               Vec3(-3950000, 3350000, 3700000), Vec3::Zero());
      Engine::Graph graph;
      e.addPseudo(graph, 3, gtsam::Pose3(), Vec3(0, speed, 0), samples,
                  Vec3::Zero());
      require(pass == (graph.size() > 0) && pass == bool(e.zupt_anchor_),
              "ZUPT speed gate at the limit");
    }
    c.nhc_enable = 1;
    Engine e(c, Vec3(-3950000, 3350000, 3700000),
             Vec3(-3950000, 3350000, 3700000), Vec3::Zero());
    e.zupt_anchor_ = gtsam::Pose3();
    Engine::Graph graph;
    e.addPseudo(graph, 3, gtsam::Pose3(), Vec3(1, 0, 0), samples, Vec3::Zero());
    require(count(graph) == std::array<int, 4>{0, 0, 0, 1} && !e.zupt_anchor_,
            "moving epoch with NHC: only the NHC factor");
  }
  static void sanityKeepsImuChain() {
    auto e = engine();
    e.current_gpst_ = 1400000000;
    e.phase_ = 2;
    e.warmReset(e.base_, gtsam::Rot3());
    e.pim_break_ = false;
    e.main_res_ = 50;
    e.ar_per_sat_res_ = {{2, 60.}, {3, 1.}};
    const auto recovery = e.sanity(Engine::Graph{}, DdRows{}, gtsam::Pose3(),
                                   gtsam::NavState(), 1);
    require(recovery && !e.pim_break_,
            "sanity recovery must keep the IMU factor chain");
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
    // satellites (and only those).
    auto e = engine();
    const auto pose = poseKey(0);
    Engine::Graph priors, dd;
    DdRows rows;
    Engine::Values values;
    gtsam::FixedLagSmoother::KeyTimestampMap ts{{pose, 0}};
    values.insert(pose, gtsam::Pose3());
    priors.addPrior<gtsam::Pose3>(pose, gtsam::Pose3(),
                                  gtsam::noiseModel::Isotropic::Sigma(6, 1e-8));
    for (int sat : {2, 3, 4}) {
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
    for (int sat : {3, 4}) {
      rows.push_back({dd.size(), true, 2, sat, 0});
      dd.emplace_shared<gtsam::DoubleDifferenceCarrierPhaseFactorArm>(
          pose, ambKey({2, 0}), ambKey({sat, 0}), 0., 0.,
          sat == 3 ? 100. : 0., 0., ref_sat, target_sat, ref_sat, target_sat,
          e.base_, .19, Vec3::Zero(), e.frame_,
          gtsam::noiseModel::Isotropic::Sigma(1, .003 * std::sqrt(2.)));
    }
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
        require(!st.held && !st.key && st.seed_pending,
                "FDE did not release the held satellites of the rejected row");
      }
      require(e.sats_.get({4, 0}).held.has_value() && e.sats_.get({4, 0}).key,
              "FDE released a satellite of a non-rejected row");
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
  // One phase-2 GNSS epoch's ambiguity bookkeeping as Engine::phase2 does it
  // (continueArcs, buildDd, endUnusedArcs), committed to the smoother.
  struct ArcEpoch {
    Engine::Graph graph;
    DdRows rows;
    Engine::Values values;
    AmbMap before;
  };
  static ArcEpoch arcEpoch(Engine &e, const ObservationEpoch &obs,
                           bool suppressed = false) {
    ArcEpoch out;
    out.before = e.sats_.keys();
    const auto estimate = e.p2_->calculateEstimate();
    e.current_ = EpochState{};
    e.slips(obs);
    e.current_.cp_suppressed = suppressed;
    e.continueArcs(estimate);
    e.buildDd(out.graph, out.rows, out.values, obs, poseKey(0), e.lever_,
              suppressed);
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
    auto run = [&](const ObservationEpoch &o, bool suppressed = false) {
      const auto before = e.sats_.keys();
      auto epoch = arcEpoch(e, o, suppressed);
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
    // Release (FDE / worst-held rule).
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
    // CP-suppressed epoch: no carrier, every arc ends, held ones re-seed.
    e.hold(*e.p2_, trueFix(e));
    before = e.sats_.keys();
    std::map<SF, double> last;
    for (auto &[sf, key] : before)
      last[sf] = *e.sats_.get(sf).held;
    epoch = run(obs, true);
    require(e.sats_.keys().empty() && e.current_.visible.empty(),
            "CP-suppressed epoch kept an ambiguity key");
    for (auto &[sf, key] : before)
      require(!e.sats_.get(sf).held && e.sats_.get(sf).seed_pending,
              "CP suppression did not release the holds");
    epoch = run(obs);
    for (auto &[sf, key] : before)
      newKey(sf, epoch, before, last.at(sf),
             "CP suppression did not start seeded arcs");
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
    // resetAmbiguities removes the gauge with the other factors of the
    // current keys; the renewed group gets one new gauge.
    e.resetAmbiguities();
    require(gauges(e.p2_->getFactors()).empty(),
            "ambiguity reset kept the gauge of a current key");
    gauged = one(arcEpoch(e, obs), "group after the reset: one gauge");
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
    // A CP-suppressed epoch ends every arc; the next carrier epoch starts the
    // group again (no held keys, so no seeds): one new gauge.
    none(arcEpoch(e, obs, true), "CP-suppressed epoch got a gauge");
    const Key renewed = one(arcEpoch(e, obs), "group after CP suppression");
    require(renewed != gauged, "group after CP suppression kept its key");
    // Held keys renewed through CP suppression are seeded from last_held: the
    // seed priors fix the common mode, so no gauge.
    e.hold(*e.p2_, trueFix(e));
    arcEpoch(e, obs, true);
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
        e.addImu({t0 + (i - 1) * .2 + m * .01, Vec3(0, 0, cfg.gravity),
                  Vec3::Zero(), {}});
      ObservationEpoch obs;
      obs.gpst = obs.base_gpst = t0 + i * .2;
      const Vec3 pos = truth(i);
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
    DdRows rows;
    Engine::Values values;
    e.buildDd(graph, rows, values, obs, poseKey(0), Vec3::Zero(), true);
    require(e.current_.visible.empty() && e.current_.consumed.empty() &&
                e.sats_.get({1, 0}).seed_pending,
            "CP suppression consumed a seed or exposed carrier measurements");
    require(rows.size() == graph.size(), "CP suppression added a non-DD factor");
    for (const auto &row : rows)
      require(!row.carrier &&
                  std::dynamic_pointer_cast<
                      gtsam::DoubleDifferencePseudorangeFactorArm>(
                      graph[row.index]),
              "CP suppression added a carrier factor");
    graph.resize(0);
    rows.clear();
    values.clear();
    e.buildDd(graph, rows, values, obs, poseKey(0), Vec3::Zero());
    require(e.current_.consumed.count({1, 0}) &&
                e.sats_.get({1, 0}).seed_pending,
            "seed consumed before graph commit");
    auto m = engine();
    Engine::Graph priors;
    Engine::Values initial;
    gtsam::FixedLagSmoother::KeyTimestampMap ts;
    initial.insert(poseKey(0), gtsam::Pose3());
    priors.addPrior<gtsam::Pose3>(poseKey(0), gtsam::Pose3(),
                                  gtsam::noiseModel::Isotropic::Sigma(6, 1));
    ts[poseKey(0)] = 0;
    for (int sat = 1; sat <= 3; ++sat) {
      auto key = ambKey({sat, 0});
      initial.insert(key, double(sat * 10));
      priors.addPrior<double>(key, double(sat * 10),
                              gtsam::noiseModel::Isotropic::Sigma(1, 1e-4));
      ts[key] = 0;
      m.sats_.get({sat, 0}).key = key;
      m.current_.visible.insert({sat, 0});
    }
    m.p2_->update(priors, initial, ts);
    auto est = m.p2_->calculateEstimate();
    auto ar = m.resolve(*m.p2_, est, poseKey(0), obs);
    require(ar.nb == 0, "AR accepted fewer than four used satellites");
    m.cfg_.ar_minfixsats = 3;
    ar = m.resolve(*m.p2_, est, poseKey(0), obs);
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
  // Epoch i (time kT0 + .2 i) of the synthetic run: 8 satellites seen from
  // the base and the rover, code, carrier and Doppler without noise unless
  // `noisy` (deterministic code/carrier perturbations, a slip on satellite 4
  // at epoch 40).
  static ObservationEpoch runEpoch(const Engine &e, int i, const Motion &m,
                                   bool noisy = false) {
    ObservationEpoch obs;
    obs.gpst = obs.base_gpst = kT0 + i * .2;
    const Vec3 pos = truth(e, i, m);
    const Vec3 vel = e.R_ * (m.dir * m.speed(obs.gpst));
    for (int j = 0; j < 8; ++j) {
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
  // Level IMU without rotation at 100 Hz from the previous epoch up to epoch
  // i: specific force = gravity plus the run's acceleration along body x
  // (negative when reversing).
  static void runImu(Engine &e, int i, const Motion &m) {
    for (int k = 1; k <= 20; ++k) {
      const double t = kT0 + (i - 1) * .2 + k * .01;
      const double ax = m.acceleration(t) * (m.reverse ? -1. : 1.);
      e.addImu({t, Vec3(ax, 0, e.cfg_.gravity), Vec3::Zero(), {}});
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
  // Until the switch, gnss_imu_fgo runs exactly the gnss_fgo graph: its
  // output (a lever arm and IMU input notwithstanding) is bit-identical to
  // gnss_fgo's, through FIX/hold, a slip, a GNSS outage and the switch epoch.
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
  // Level IMU without rotation at 50 Hz, sampled mid-period: from the
  // previous epoch up to epoch i, the last sample 15 ms before the epoch (the
  // next one, 5 ms after it, releases the epoch in the node). Specific force
  // as runImu.
  static void runImuMidPeriod(Engine &e, int i, const Motion &m) {
    for (int k = 0; k < 10; ++k) {
      const double t = kT0 + (i - 1) * .2 + .005 + k * .02;
      const double ax = m.acceleration(t) * (m.reverse ? -1. : 1.);
      e.addImu({t, Vec3(ax, 0, e.cfg_.gravity), Vec3::Zero(), {}});
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
  // outage, satellites leaving and rising; Doppler intact.
  static constexpr double kBurst = 30.;
  static ObservationEpoch burstEpoch(const Engine &e, int i) {
    ObservationEpoch obs;
    obs.gpst = obs.base_gpst = kT0 + i * .2;
    const Vec3 pos = e.base_;
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
  // graph disturbed by a code burst: the Doppler-only gate rejects it.
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
  // (j) Check 2. The update-probe hooks only throw (warm resets) and cannot
  // produce sanity recoveries, so (j1) drives the check directly: a recovery
  // in every GNSS epoch declares the graph invalid after kIntegrityTime;
  // epochs without GNSS neither count nor reset; an epoch without recovery
  // resets; a gap longer than kIntegrityTime restarts the run.
  static void recoveryTimerTest() {
    auto e = engine();
    e.phase_ = 2;
    double t = 1400000000.;
    auto epoch = [&](bool gnss, bool recovery, double step = .2) {
      t += step;
      e.current_gpst_ = t;
      e.integrity_epoch_ = gnss;
      e.sanity_recovery_ = recovery;
      e.epoch_doppler_.reset();  // check 1 not evaluated
      return e.integrity(gnss);
    };
    for (int i = 0; i < 10; ++i)
      require(epoch(true, true) == 0, "recovery run declared invalid early");
    require(epoch(false, false) == 0 && std::abs(e.recovery_timer_.length() - 1.8) < 1e-6,
            "epoch without GNSS changed the recovery run");
    require(epoch(true, true) == 2, "2 s of recoveries did not declare the graph invalid");
    e.recovery_timer_.clear();
    for (int i = 0; i < 5; ++i)
      epoch(true, true);
    require(epoch(true, false) == 0 && !e.recovery_timer_.start,
            "an epoch without recovery did not reset the run");
    for (int i = 0; i < 5; ++i)
      epoch(true, true);
    require(epoch(true, true, 2.5) == 0 && e.recovery_timer_.length() == 0,
            "a gap longer than kIntegrityTime did not restart the run");
  }
  // (j2) End to end: from epoch 20 the reference satellite's code (200 m)
  // and carrier (5 cycles) are wrong, every DD row is off and FDE skips.
  // The IMU graph falls back within about 2.5 s with a valid solution in the
  // fallback epoch. (In this synthetic case the heading is wrecked as well
  // and the recovery run is interrupted by the max/median veto, so check 1
  // can decide; the plausibility guard rejects the epoch-23 posterior, a
  // speed of several hundred m/s, and check 2 decides after that restart.)
  static void corruptedDataFallbackTest() {
    const Vec3 base(6378137, 0, 0);
    Engine e(Config{}, base, base, Vec3(1.2, .1, .8));
    const auto motion = run(Vec3(2., 1., 0), 8., 0.);
    int fell = -1;
    for (int i = 1; i <= 50 && fell < 0; ++i) {
      runImu(e, i, motion);
      auto obs = runEpoch(e, i, motion);
      if (i >= 20) {
        obs.sats[0].pr[0] += 200.;
        obs.sats[0].phase[0] += 5.;
      }
      const auto sol = e.process(obs);
      if (e.n_fallback_) {
        fell = i;
        require(sol.valid && e.phase_ == 1, "fallback epoch lost its solution");
      }
    }
    std::cout << "corrupted reference from epoch 20: fallback at epoch " << fell
              << " (cause " << e.fallback_cause_ << ")\n";
    require(fell > 20 && fell <= 34, "corrupted data did not fall back");
  }
  // An integer search that finds no acceptable candidate (ArOutcome::Zero),
  // however many epochs in a row and with a small code residual, leaves the
  // ambiguity keys and the holds as they are. gnss_fgo at 5 m/s fixes and
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
      require(sol.valid && !sol.fixed && e.ar_outcome_ == Engine::ArOutcome::Zero &&
                  e.main_res_ <= e.cfg_.ar_starve_max_res,
              "no-candidate fixture: not a search without candidate");
      std::map<std::pair<Key, Key>, double> now;
      for (const auto &[pair, record] : e.holds_)
        now[pair] = record.difference;
      require(e.sats_.keys() == keys && now == holds,
              "epochs without an integer candidate changed the ambiguities");
    }
  }
  // ---- Plausibility guard ----
  // Epoch i of `m` reduced to satellites 1-4, with 1-2 in system 0 and 3-4 in
  // system 2: one code DD (and one carrier DD) per system, so 2 code DDs, fewer
  // than the 3 a position needs.
  static ObservationEpoch twoDdEpoch(const Engine &e, int i, const Motion &m) {
    auto obs = runEpoch(e, i, m, true);
    obs.sats.resize(4);
    obs.sats[2].system = obs.sats[3].system = 2;
    return obs;
  }
  // gnss_fgo at 5 m/s: 8 satellites, from epoch 31 the 2-DD epochs, at the end
  // of epoch 40 the ambiguity reset (as a sanity recovery does it), then 2
  // code DDs without carrier (suppressed after the reset), from epoch 51 the
  // 8 satellites again. The reset keeps the information of the marginal
  // factor of the epochs before the lag on the states; without it the next
  // update's position would be indeterminate along the direction the 2 DDs do
  // not see. Returns the largest error of a published position (epochs 41-80)
  // and counts the valid GNSS epochs 61-80 within 1 m of the truth.
  static double indeterminateResetRun(bool guard, int &good_after) {
    Config c;
    c.gnss_motion = true;
    const Vec3 base(6378137, 0, 0);
    Engine e(c, base, base, Vec3::Zero());
    e.plausibility_guard_ = guard;
    const auto motion = run(Vec3(1., .5, 0), 5., 0.);
    double worst = 0;
    good_after = 0;
    for (int i = 1; i <= 80; ++i) {
      if (i == 41)
        e.resetAmbiguities();
      const bool reduced = i > 30 && i <= 50;
      const auto sol = e.process(reduced ? twoDdEpoch(e, i, motion)
                                         : runEpoch(e, i, motion, true));
      const Vec3 t = truth(e, i, motion);
      if (i > 40 && sol.valid)
        worst = std::max(worst, (sol.position - t).norm());
      if (guard)
        require((e.floatPosition() - t).norm() < 1e3,
                "position for the elevation mask left the trajectory");
      good_after += i > 60 && sol.valid && sol.kind == UpdateKind::Gnss &&
                    (sol.position - t).norm() < 1.;
    }
    return worst;
  }
  static void indeterminateResetTest() {
    for (const bool guard : {false, true}) {
      int good = 0;
      const double worst = indeterminateResetRun(guard, good);
      std::cout << "ambiguity reset with 2 code DDs"
                << (guard ? "" : " (no plausibility guard)")
                << ": largest published error " << worst << " m, " << good
                << "/20 valid GNSS epochs after\n";
      require(worst < 2. && good == 20,
              "ambiguity reset left the states without their marginal information");
    }
  }
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
};
} // namespace gnss_fgo
int main() {
  EngineTestAccess::vehicleConstraintSwitches();
  EngineTestAccess::zuptSpeedGate();
  EngineTestAccess::sanityKeepsImuChain();
  EngineTestAccess::holdTest(false);
  EngineTestAccess::holdTest(true);
  EngineTestAccess::fdeTest(false);
  EngineTestAccess::fdeTest(true);
  EngineTestAccess::phase2HoldTest();
  EngineTestAccess::arcEndTest();
  EngineTestAccess::gaugeTest();
  EngineTestAccess::gaugeImuRegression();
  EngineTestAccess::resetTest();
  EngineTestAccess::suppressionAndMinimumTest();
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
  EngineTestAccess::recoveryTimerTest();
  EngineTestAccess::corruptedDataFallbackTest();
  EngineTestAccess::ddprFinalSetTest();
  EngineTestAccess::futureImuAdmissionTest();
  EngineTestAccess::imuBacklogTest();
  EngineTestAccess::midPeriodImuTest();
  EngineTestAccess::noCandidateKeepsAmbiguitiesTest();
  EngineTestAccess::indeterminateResetTest();
  EngineTestAccess::implausiblePosteriorTest();
  std::cout << "graph transaction regressions passed\n";
}
