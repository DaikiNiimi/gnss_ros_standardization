#include "ros_adapter.hpp"
#include <iostream>
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
  rosmsg::GnssEphemeris ep;
  ep.system = "J";
  ep.prn = 1;
  ep.week = 2301;
  ep.toe = 10;
  ep.toc = 604790;
  ep.ttr = 604780;
  ep.a = 26560000;
  ep.e = .01;
  auto e = decodeEphemeris(ep, 9);
  require(e.sat == 69 && e.prn == 193 && e.toc - e.toe == -20 &&
              e.transmitted - e.toe == -30,
          "QZSS identifier/nearest week");
  // RTKLIB satexclude() masks the QZSS LEX health bit (svh & 0xFE); RINEX
  // and receiver QZSS NAV commonly carry svh = 1 for healthy satellites.
  ep.svh = 1;
  require(decodeEphemeris(ep, 9).health == 0, "QZSS LEX health bit ignored");
  ep.svh = 3;
  require(decodeEphemeris(ep, 9).health == 2, "QZSS other health bits kept");
  ep.system = "G";
  ep.prn = 1;
  ep.svh = 1;
  require(decodeEphemeris(ep, 9).health == 1, "GPS health bit 0 kept");
  ep.system = "J";
  ep.svh = 0;
  ReceiverPlan plans;
  plans["G"].codes = {
      std::vector<std::string>{"1C"}, std::vector<std::string>{"1W"},
      std::vector<std::string>{"1C"}, std::vector<std::string>{"1W"}};
  rosmsg::GnssObservations obs;
  obs.week = 2301;
  obs.tow = .02;
  rosmsg::GnssObservation row;
  row.system = "G";
  row.prn = 1;
  row.code_str = "1C";
  row.p = 20000000;
  row.d = -1000;
  obs.observations.push_back(row);
  row.code_str = "1W";
  row.l = 100000;
  row.snr = 20;
  row.lli = 5;
  obs.observations.push_back(row);
  auto decoded = decodeObservation(obs, plans);
  auto r = decoded.sats.at(0);
  require(r.pr[0] == 20000000 && r.phase[0] == 100000 &&
              r.doppler[0] == -1000 && r.snr[0] == 20 && r.lli[0] == 1 &&
              r.gps_py[0],
          "independent signal field selection");
  ReceiverPlan bds;
  bds["C"].codes = {std::vector<std::string>{"2I", "7I", "6I"},
                    std::vector<std::string>{"2I", "7I", "6I"},
                    std::vector<std::string>{"2I", "6I", "7I"},
                    std::vector<std::string>{"2I", "7I", "6I"}};
  obs.observations.clear();
  row.system = "C";
  row.prn = 26;
  row.lli = 0;
  row.code_str = "6I";
  row.d = 2530;
  row.snr = 40;
  obs.observations.push_back(row);
  row.code_str = "7I";
  row.d = 0;
  row.snr = 30;
  obs.observations.push_back(row);
  auto cb = decodeObservation(obs, bds).sats.at(0);
  require(cb.doppler[1] == 2530 &&
              std::abs(cb.doppler_wavelength[1] - 299792458. / 1268.52e6) <
                  1e-15 &&
              cb.doppler_wavelength[1] != cb.wavelength[1] &&
              cb.doppler_snr[1] == 40,
          "Doppler used phase band wavelength/SNR");
  BroadcastStore store;
  ep.svh = 46;
  ep.a = 0;
  ep.toe = 10;
  ep.toc = 10;
  ep.ttr = 10;
  store.insert(decodeEphemeris(ep, 10));
  require(
      !store.position(69, 2, weekTime(2301, 12), 20000000, weekTime(2301, 12))
           .available,
      "empty unhealthy orbit must not propagate");
  std::cout << "ROS conversion regressions passed\n";
}
