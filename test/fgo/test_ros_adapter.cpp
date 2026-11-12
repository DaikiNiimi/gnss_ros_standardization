#include "gnss_ros_standardization/ephemeris_store.hpp"
#include "ros_adapter.hpp"
#include <functional>
#include <iostream>
#include <memory>
using namespace gnss_fgo;
static void require(bool ok, const char *why) {
  if (!ok)
    throw std::runtime_error(why);
}
int main() {
  builtin_interfaces::msg::Time utc;
  utc.sec = 1710000000;
  utc.nanosec = 123456789;
  auto gp = decodeImuTime(utc, true);
  auto plain = decodeImuTime(utc, false);
  require(gp.seconds - plain.seconds == 18 && gp.fraction == plain.fraction,
          "UTC/GPST leap-second offset");
  auto roundtrip = encodeHeaderTime(gp, true);
  require(roundtrip.sec == utc.sec && roundtrip.nanosec == utc.nanosec,
          "UTC stamp round trip");
  utc.sec = 1483228799;
  require(decodeImuTime(utc, true).seconds -
                  decodeImuTime(utc, false).seconds ==
              17,
          "pre-leap second offset");
  utc.sec = 1483228800;
  require(decodeImuTime(utc, true).seconds -
                  decodeImuTime(utc, false).seconds ==
              18,
          "post-leap second offset");
  std::optional<Vec3> origin;
  Solution s;
  s.gpst = 2301. * 604800.;
  s.position = Vec3(6378137, 0, 0);
  s.valid = true;
  s.fixed = true;
  s.kind = UpdateKind::Gnss;
  auto m = solutionMessage(s, origin);
  require(m.status == m.STATUS_FIX && m.time_week == 2301 && m.time_tow == 0,
          "FIX week boundary");
  require(m.header.stamp.sec == std::int64_t(s.gpst) + 315964800,
          "GPST-derived stamp");
  require(std::abs(m.latitude) + std::abs(m.longitude) + std::abs(m.altitude) <
              1e-6,
          "geodetic conversion");
  require(std::isnan(m.vel_ecef.x) && std::isnan(m.pos_cov_ecef[0]),
          "unknowns must be NaN");
  s.fixed = false;
  m = solutionMessage(s, origin);
  require(m.status == m.STATUS_FLOAT, "FLOAT status");
  s.kind = UpdateKind::DeadReckoning;
  m = solutionMessage(s, origin);
  require(m.status == m.STATUS_DR, "DR status");
  s.valid = false;
  m = solutionMessage(s, origin);
  require(m.status == m.STATUS_NONE && std::isnan(m.pos_ecef.x) &&
              std::isnan(m.latitude),
          "NONE must not reuse prior position");
  s.valid = s.fixed = true;
  s.position.x() = std::numeric_limits<double>::quiet_NaN();
  m = solutionMessage(s, origin);
  require(m.status == m.STATUS_NONE, "nonfinite FIX output");
  // local_origin mode fixed: the configured origin applies before any FIX and
  // a later FIX does not replace it.
  std::optional<Vec3> fixed_origin = Vec3(6378037, 0, 0);
  s.position = Vec3(6378137, 0, 0);
  s.fixed = false;
  s.kind = UpdateKind::Gnss;
  m = solutionMessage(s, fixed_origin);
  require(m.status == m.STATUS_FLOAT && m.pos_enu_org_ecef.x == 6378037 &&
              std::abs(m.pos_enu.z - 100) < 1e-9 &&
              std::abs(m.pos_enu.x) + std::abs(m.pos_enu.y) < 1e-9,
          "fixed origin before the first FIX");
  s.fixed = true;
  m = solutionMessage(s, fixed_origin);
  require(fixed_origin && fixed_origin->x() == 6378037 &&
              m.pos_enu_org_ecef.x == 6378037,
          "a FIX keeps the fixed origin");
  // Signal selection is shared with the RTK node.
  rosmsg::GnssObservations rover_obs, base_obs;
  rover_obs.week = base_obs.week = 2301;
  rover_obs.tow = base_obs.tow = .2;
  const auto add = [](rosmsg::GnssObservations &epoch, const std::string &system,
                      int prn, const std::string &code) {
    rosmsg::GnssObservation row;
    row.system = system;
    row.prn = prn;
    row.code_str = code;
    row.p = 2.1e7;
    row.l = 1.1e8;
    row.d = -1000;
    row.snr = 45;
    epoch.observations.push_back(row);
  };
  for (const char *code : {"1C", "2W", "2L", "5Q"})
    add(rover_obs, "G", 1, code);
  for (const char *code : {"1C", "2W", "2X", "5X"})
    add(base_obs, "G", 1, code);
  for (const char *code : {"1P", "2I", "7I"})
    add(rover_obs, "C", 26, code);
  for (const char *code : {"1X", "2I", "7I"})
    add(base_obs, "C", 26, code);
  add(rover_obs, "E", 3, "1C");
  add(base_obs, "E", 3, "1X");
  const auto decodePair = [&](const SignalPolicy &policy) {
    gnss_utils::SignalSelection selection;
    const auto choice = gnss_utils::selectSignals(
        rover_obs, &base_obs, policy.frequencies, selection);
    return std::make_pair(decodeObservation(rover_obs, 1, policy, choice),
                          decodeObservation(base_obs, 2, policy, choice));
  };
  const auto find = [](const RawEpoch &epoch, int sat) -> const RawSatellite * {
    for (const auto &candidate : epoch.sats)
      if (candidate.sat == sat)
        return &candidate;
    return nullptr;
  };
  SignalPolicy policy;
  auto [rover_epoch, base_epoch] = decodePair(policy);
  const auto *g = find(rover_epoch, satelliteId("G", 1).second);
  const auto *gb = find(base_epoch, satelliteId("G", 1).second);
  const auto *cb26 = find(rover_epoch, satelliteId("C", 26).second);
  const auto *er = find(rover_epoch, satelliteId("E", 3).second);
  const auto *eb = find(base_epoch, satelliteId("E", 3).second);
  require(g && gb && cb26 && er && eb, "selected satellites decoded");
  require(g->signals[0] == "L1C" && g->signals[1] == "L2W" && gb->signals[1] == "L2W",
          "common L2 code chosen by the RTKLIB priority");
  require(g->pr[2] == 0, "disabled L5 band not decoded");
  require(std::abs(g->wavelength[1] - 299792458. / 1227.60e6) < 1e-12 &&
              g->doppler_wavelength[1] == g->wavelength[1] && g->doppler_snr[1] == 45,
          "Doppler uses the selected signal of its band");
  require(cb26->signals[0] == "L2I" && cb26->signals[1] == "L7I",
          "BDS slots follow the RTKLIB frequency index");
  require(er->signals[0] == "L1C" && eb->signals[0] == "L1X",
          "heterogeneous Galileo pairing kept");
  {
    gnss_utils::SignalSelection rtk_selection;
    const auto rtk = gnss_utils::buildObsEpoch(
        rover_obs, &base_obs, policy.frequencies, nullptr, &rtk_selection);
    int compared = 0;
    for (const auto &o : rtk) {
      int prn = 0;
      const int sys = satsys(o.sat, &prn);
      const std::string letter = sys == SYS_GPS ? "G" : sys == SYS_GAL ? "E" : "C";
      const auto *fgo = find(o.rcv == 1 ? rover_epoch : base_epoch,
                             satelliteId(letter, prn).second);
      require(fgo != nullptr, "RTK satellite present in FGO");
      for (int f = 0; f < 3; ++f) {
        if (!o.code[f])
          continue;
        require(fgo->signals[f] == std::string("L") + code2obs(o.code[f]),
                "FGO and RTK selected the same code");
        ++compared;
      }
    }
    require(compared >= 8, "RTK comparison covered the signals");
  }
  {
    SignalPolicy with_l5;
    with_l5.frequencies.l5 = true;
    auto [r5, b5] = decodePair(with_l5);
    const auto *g5 = find(r5, satelliteId("G", 1).second);
    const auto *gb5 = find(b5, satelliteId("G", 1).second);
    require(g5->signals[2] == "L5Q" && gb5->signals[2] == "L5X",
            "L5 band follows frequencies.enable_l5");
    SignalPolicy without;
    without.excluded.insert(satid2no("G01"));
    without.systems.erase("C");
    auto [rf, bf] = decodePair(without);
    require(!find(rf, satelliteId("G", 1).second) &&
                !find(rf, satelliteId("C", 26).second) &&
                find(rf, satelliteId("E", 3).second),
            "excluded satellite and disabled constellation removed");
  }
  {
    // The SNR mask rejects the whole satellite: a band seen before (code and
    // phase) that is masked or missing in an epoch removes the satellite for
    // that receiver and epoch. Bands never seen are not required.
    //   G1-G3, J193: dual band on both receivers; J194: unhealthy
    //   G4: rover L1 only, base dual; G5: rover dual, base L1 only
    //   G6: L1 only on both receivers
    struct Bench {
      Config cfg;
      Vec3 base_position{-3950000, 3350000, 3700000};
      std::unique_ptr<Preprocessor> front;
      int epochs = 0;
    };
    const auto makeBench = [](bool mask_on, double elevation_mask) {
      auto bench = std::make_unique<Bench>();
      bench->cfg.elmin_deg = elevation_mask;
      bench->cfg.snrmask_enable = {mask_on, mask_on};
      for (auto &band : bench->cfg.snrmask)
        band.fill(35.);
      bench->front = std::make_unique<Preprocessor>(bench->cfg, bench->base_position);
      gnss_utils::EphemerisStore store;
      rosmsg::GnssEphemeris orbit;
      orbit.week = 2301;
      orbit.toe = orbit.toc = orbit.ttr = 1000.;
      orbit.a = 26560000.;
      orbit.system = "G";
      for (int prn = 1; prn <= 6; ++prn) {
        orbit.prn = prn;
        orbit.m0 = .5 * prn;
        store.ingestEph(gnss_utils::msgToEph(orbit));
      }
      // QZSS health as in RTKLIB satexclude(): only the LEX bit may be set.
      orbit.system = "J";
      for (const auto &[prn, health] :
           std::vector<std::pair<int, int>>{{193, 1}, {194, 46}}) {
        orbit.prn = prn;
        orbit.svh = health;
        store.ingestEph(gnss_utils::msgToEph(orbit));
      }
      store.applyToNav(bench->front->navigation(), true);
      return bench;
    };
    // One epoch, one second after the previous; `change` edits the rover and
    // base observations of a satellite (system letter, PRN) before preparing.
    using Change = std::function<void(const std::string &, int, RawSatellite &,
                                      RawSatellite &)>;
    const auto epoch = [](Bench &bench, const Change &change) {
      RawEpoch rover_raw, base_raw;
      rover_raw.time = base_raw.time = weekTime(2301, 1000.2 + bench.epochs++);
      std::vector<std::pair<std::string, int>> sats;
      for (int prn = 1; prn <= 6; ++prn)
        sats.emplace_back("G", prn);
      sats.emplace_back("J", 193);
      sats.emplace_back("J", 194);
      for (const auto &[system, prn] : sats) {
        RawSatellite r;
        r.sat = satelliteId(system, prn).second;
        r.system = system == "G" ? 0 : 2;
        r.prn = prn;
        r.pr = Vec3(2.2e7, 2.2e7, 0);
        r.phase = Vec3(1.2e8, 9e7, 0);
        r.wavelength = Vec3(.19, .24, 0);
        r.snr = Vec3(45, 45, 0);
        RawSatellite b = r;
        const auto singleBand = [](RawSatellite &s) {
          s.pr[1] = s.phase[1] = s.snr[1] = 0;
        };
        if (system == "G" && (prn == 4 || prn == 6))
          singleBand(r);
        if (system == "G" && (prn == 5 || prn == 6))
          singleBand(b);
        if (change)
          change(system, prn, r, b);
        rover_raw.sats.push_back(r);
        base_raw.sats.push_back(b);
      }
      return bench.front->prepare(rover_raw, base_raw, bench.base_position);
    };
    const auto findSat = [](const ObservationEpoch &e, const std::string &system,
                         int prn) -> const SatelliteObservation * {
      const int sat = satelliteId(system, prn).second;
      for (const auto &v : e.sats)
        if (v.sat == sat)
          return &v;
      return nullptr;
    };
    // Satellites not changed in an epoch keep their usual bands.
    const auto usual = [](const ObservationEpoch &e, const char *why) {
      for (const auto &v : e.sats) {
        const bool rover_l2 = !(v.system == 0 && (v.prn == 4 || v.prn == 6));
        const bool base_l2 = !(v.system == 0 && (v.prn == 5 || v.prn == 6));
        require(v.pr[0] != 0 && v.phase[0] != 0 && v.pr_base[0] != 0 &&
                    v.phase_base[0] != 0 &&
                    (v.pr[1] != 0) == rover_l2 && (v.phase[1] != 0) == rover_l2 &&
                    (v.pr_base[1] != 0) == base_l2 &&
                    (v.phase_base[1] != 0) == base_l2,
                why);
      }
    };
    const auto bench = makeBench(true, -90.);
    // (a) both bands above the mask: admitted on both bands. (c) G6 is admitted
    // on L1 alone. (e) the base's G4 L2 does not make the rover require it,
    // and the rover's G5 L2 does not make the base require it.
    const auto first = epoch(*bench, nullptr);
    require(first.sats.size() == 7, "dual and single band satellites admitted");
    require(!findSat(first, "J", 194),
            "unhealthy QZSS satellite excluded, LEX-only health kept");
    usual(first, "admitted satellites keep the bands they have");
    // (b) one seen band below the mask rejects the satellite in this epoch.
    const auto masked = epoch(*bench, [](const std::string &system, int prn,
                                         RawSatellite &r, RawSatellite &) {
      if (system == "G" && prn == 1)
        r.snr[1] = 20;
    });
    require(masked.sats.size() == 6 && !findSat(masked, "G", 1),
            "satellite rejected when one band is masked");
    usual(masked, "other satellites unaffected by the masked band");
    const auto recovered = epoch(*bench, nullptr);
    require(recovered.sats.size() == 7 && findSat(recovered, "G", 1) &&
                findSat(recovered, "G", 1)->pr[1] != 0,
            "satellite admitted again once both bands pass the mask");
    usual(recovered, "recovered epoch keeps the usual bands");
    // (c) the only band of a single-band satellite below the mask rejects it.
    const auto single_masked = epoch(*bench, [](const std::string &system, int prn,
                                                RawSatellite &r, RawSatellite &) {
      if (system == "G" && prn == 6)
        r.snr[0] = 20;
    });
    require(single_masked.sats.size() == 6 && !findSat(single_masked, "G", 6),
            "single band satellite rejected when its band is masked");
    // (d) a seen band missing in this epoch rejects the satellite: phase only
    // (G2) or code and phase (G3).
    const auto missing = epoch(*bench, [](const std::string &system, int prn,
                                          RawSatellite &r, RawSatellite &) {
      if (system == "G" && prn == 2)
        r.phase[1] = 0;
      if (system == "G" && prn == 3)
        r.pr[1] = r.phase[1] = r.snr[1] = 0;
    });
    require(missing.sats.size() == 5 && !findSat(missing, "G", 2) &&
                !findSat(missing, "G", 3),
            "satellite rejected when a seen band is missing");
    usual(missing, "missing band leaves the other satellites unaffected");
    // The base applies the same rule with its own mask and memory.
    const auto base_masked = epoch(*bench, [](const std::string &system, int prn,
                                              RawSatellite &, RawSatellite &b) {
      if (system == "G" && prn == 1)
        b.snr[0] = 20;
      if (system == "G" && prn == 4)
        b.phase[1] = 0;
    });
    require(base_masked.sats.size() == 5 && !findSat(base_masked, "G", 1) &&
                !findSat(base_masked, "G", 4),
            "base band masked or missing rejects the satellite");
    usual(base_masked, "base rejection leaves the other satellites unaffected");
    // (e) G4 L2 seen by the base only, G5 L2 seen by the rover only: dropping
    // it on the side that has it rejects, the other side is unaffected.
    const auto rover_drop = epoch(*bench, [](const std::string &system, int prn,
                                             RawSatellite &r, RawSatellite &) {
      if (system == "G" && prn == 5)
        r.snr[1] = 20;
    });
    require(rover_drop.sats.size() == 6 && !findSat(rover_drop, "G", 5) &&
                findSat(rover_drop, "G", 4),
            "rover rejection uses the rover's memory only");
    usual(rover_drop, "rover rejection leaves the base-only band alone");
    const auto again = epoch(*bench, nullptr);
    require(again.sats.size() == 7, "all satellites admitted again");
    usual(again, "memories stay per receiver");
    // Mask off: a low band is kept; elevation mask still applies.
    const auto unmasked_bench = makeBench(false, -90.);
    const auto unmasked = epoch(*unmasked_bench, [](const std::string &system,
                                                    int prn, RawSatellite &r,
                                                    RawSatellite &) {
      if (system == "G" && prn == 1)
        r.snr[1] = 20;
    });
    require(unmasked.sats.size() == 7 && findSat(unmasked, "G", 1)->pr[1] != 0,
            "SNR mask off keeps both bands");
    const auto high_bench = makeBench(false, 89.9);
    require(epoch(*high_bench, nullptr).sats.empty(), "elevation mask applied");
  }
  std::cout << "ROS conversion regressions passed\n";
}
