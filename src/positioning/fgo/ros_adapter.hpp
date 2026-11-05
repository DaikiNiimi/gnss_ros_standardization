#pragma once
// ROS message <-> FGO input conversion (no rclcpp): stamps, observations, ephemerides.
#include "preprocessor.hpp"
#include "gnss_ros_standardization/rtklib_compat.hpp"
#include "gnss_ros_standardization/gnss_utils.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include <gnss_ros_standardization/msg/gnss_ephemerides.hpp>
#include <gnss_ros_standardization/msg/gnss_observations.hpp>
#include <gnss_ros_standardization/msg/gnss_solution.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <algorithm>
#include <array>
#include <cstdint>

namespace gnss_fgo {
// Public receiver drivers publish UTC-converted GPS stamps; the frozen PPC
// bags use GPST-derived Unix stamps. Keep the convention explicit at intake.
inline PreciseTime decodeImuTime(const builtin_interfaces::msg::Time &stamp,
                                 bool utc) {
  if (stamp.nanosec >= 1000000000)
    throw std::invalid_argument("invalid IMU stamp");
  gtime_t t{};
  t.time = stamp.sec;
  t.sec = 0;
  if (utc)
    t = utc2gpst(t);
  return {static_cast<std::int64_t>(t.time) - 315964800, stamp.nanosec * 1e-9};
}
inline builtin_interfaces::msg::Time encodeHeaderTime(const PreciseTime &gpst,
                                                      bool utc) {
  gtime_t t{};
  t.time = gpst.seconds + 315964800;
  t.sec = 0;
  if (utc)
    t = gpst2utc(t);
  auto ns = static_cast<std::int64_t>(std::llround(gpst.fraction * 1e9));
  auto sec = static_cast<std::int64_t>(t.time) + ns / 1000000000;
  ns %= 1000000000;
  if (sec < 0 || sec > std::numeric_limits<std::int32_t>::max())
    throw std::invalid_argument("time outside ROS stamp range");
  builtin_interfaces::msg::Time result;
  result.sec = sec;
  result.nanosec = ns;
  return result;
}
} // namespace gnss_fgo

namespace gnss_fgo {
namespace rosmsg = gnss_ros_standardization::msg;
struct SignalPlan {
  // P, L, D and S may name different codes in the same frequency band.
  std::array<std::vector<std::string>, 4> codes;
};
using ReceiverPlan = std::map<std::string, SignalPlan>;
// Rows skipped while decoding an observation message (counted, not fatal).
struct DecodeStats {
  std::uint64_t unsupported_satellites = 0; // PRN outside the estimator's range
  std::uint64_t duplicate_signals = 0;      // repeated satellite/signal row
};
inline bool supportedSatellite(const std::string &system, int prn) {
  return (system == "G" && prn >= 1 && prn <= 32) ||
         (system == "E" && prn >= 1 && prn <= 36) ||
         (system == "J" && prn >= 193 && prn <= 202) ||
         (system == "C" && prn >= 1 && prn <= 63);
}
inline std::pair<int, int> satelliteId(const std::string &system, int prn) {
  if (system == "G" && prn >= 1 && prn <= 32)
    return {0, prn};
  if (system == "E" && prn >= 1 && prn <= 36)
    return {1, 32 + prn};
  if (system == "J" && prn >= 193 && prn <= 202)
    return {2, 69 + prn - 193};
  if (system == "C" && prn >= 1 && prn <= 63)
    return {3, 85 + prn};
  throw std::invalid_argument("unsupported satellite system/PRN: " + system +
                              std::to_string(prn));
}
inline double wavelength(const std::string &system, const std::string &code) {
  if (code.empty())
    throw std::invalid_argument("empty signal");
  double hz = 0;
  switch (code[0]) {
  case '1':
    hz = 1575.42e6;
    break;
  case '2':
    hz = system == "C" ? 1561.098e6 : 1227.60e6;
    break;
  case '5':
    hz = 1176.45e6;
    break;
  case '6':
    hz = system == "C" ? 1268.52e6 : 1278.75e6;
    break;
  case '7':
    hz = 1207.14e6;
    break;
  case '8':
    hz = 1191.795e6;
    break;
  default:
    throw std::invalid_argument("unsupported signal frequency");
  }
  return 299792458. / hz;
}
inline PreciseTime weekTime(std::uint32_t week, double tow) {
  if (!week || !std::isfinite(tow) || tow < 0 || tow >= 604800.)
    throw std::invalid_argument("invalid GPST week/tow");
  const auto whole = static_cast<std::int64_t>(std::floor(tow));
  return {static_cast<std::int64_t>(week) * 604800 + whole, tow - whole};
}
inline RawEpoch decodeObservation(const rosmsg::GnssObservations &m,
                                  const ReceiverPlan &plans,
                                  DecodeStats *stats = nullptr) {
  RawEpoch out;
  out.time = weekTime(m.week, m.tow);
  struct Group {
    std::string system;
    int prn;
    std::map<std::string, const rosmsg::GnssObservation *> rows;
  };
  std::vector<Group> groups;
  std::map<int, std::size_t> indices;
  for (const auto &r : m.observations) {
    if (!plans.count(r.system))
      continue;
    if (!supportedSatellite(r.system, r.prn)) {
      if (stats)
        ++stats->unsupported_satellites;
      continue;
    }
    auto [system, sat] = satelliteId(r.system, r.prn);
    if (!indices.count(sat)) {
      indices[sat] = groups.size();
      groups.push_back({r.system, r.prn, {}});
    }
    // The first row of a satellite/signal is used; a repeat is counted.
    if (!groups[indices[sat]].rows.emplace(r.code_str, &r).second && stats)
      ++stats->duplicate_signals;
  }
  for (const auto &g : groups) {
    const auto &plan = plans.at(g.system);
    RawSatellite s;
    std::tie(s.system, s.sat) = satelliteId(g.system, g.prn);
    s.prn = g.prn;
    s.bands = std::min(plan.codes[0].size(), plan.codes[1].size());
    if (s.bands > 3)
      throw std::invalid_argument("at most three signal bands are supported");
    for (int typ = 0; typ < 4; ++typ)
      for (std::size_t j = 0; j < plan.codes[typ].size(); ++j) {
        if (j >= 3)
          throw std::invalid_argument(
              "at most three signal bands are supported");
        const auto &code = plan.codes[typ][j];
        auto found = g.rows.find(code);
        if (typ == 1) {
          s.signals[j] = "L" + code;
          s.wavelength[j] = wavelength(g.system, code);
        }
        if (typ == 3)
          s.gps_py[j] = g.system == "G" && (code == "1W" || code == "2W");
        if (found == g.rows.end())
          continue;
        const auto &row = *found->second;
        const double v = typ == 0   ? row.p
                         : typ == 1 ? row.l
                         : typ == 2 ? row.d
                                    : row.snr;
        (typ == 0   ? s.pr
         : typ == 1 ? s.phase
         : typ == 2 ? s.doppler
                    : s.snr)[j] = std::isfinite(v) ? v : 0.;
        if (typ == 1)
          s.lli[j] = row.lli & 3;
      }
    // Doppler columns have their own order, independent of phase columns.
    for (std::size_t d = 0; d < plan.codes[2].size(); ++d) {
      const auto &code = plan.codes[2][d];
      s.doppler_wavelength[d] = wavelength(g.system, code);
      for (std::size_t j = 0; j < plan.codes[3].size(); ++j)
        if (plan.codes[3][j][0] == code[0]) {
          s.doppler_snr[d] = s.snr[j];
          break;
        }
    }
    out.sats.push_back(std::move(s));
  }
  return out;
}
inline BroadcastEphemeris decodeEphemeris(const rosmsg::GnssEphemeris &m,
                                          std::int64_t id) {
  BroadcastEphemeris e;
  e.id = id;
  // RINEX-derived ephemeris messages can carry the J01 identifier number;
  // observations use the absolute QZSS PRN 193.
  e.prn = m.system == "J" && m.prn >= 1 && m.prn <= 10 ? m.prn + 192 : m.prn;
  std::tie(e.system, e.sat) = satelliteId(m.system, e.prn);
  e.toe = weekTime(m.week, m.toe);
  auto near = [&](double tow) {
    if (!std::isfinite(tow))
      throw std::invalid_argument("invalid navigation time");
    double d = std::fmod(tow - m.toe + 302400., 604800.);
    if (d < 0)
      d += 604800.;
    return e.toe.add(d - 302400.);
  };
  e.toc = near(m.toc);
  e.transmitted = near(m.ttr);
  // As RTKLIB satexclude(): bit 0 of the QZSS health word is the LEX signal
  // health and does not affect L1/L2/L5 ranging (healthy QZSS NAV records
  // commonly carry svh = 1).
  e.health = m.system == "J" ? (m.svh & 0xFE) : m.svh;
  e.mode = m.system == "E" && (m.code & 2) ? 1 : 0;
  e.A = m.a;
  e.e = m.e;
  e.M0 = m.m0;
  e.omg = m.omg;
  e.i0 = m.i0;
  e.OMG0 = m.omg0;
  e.OMGd = m.omgd;
  e.deln = m.deln;
  e.delnd = m.ndot;
  e.Adot = m.adot;
  e.idot = m.idot;
  e.crc = m.crc;
  e.crs = m.crs;
  e.cuc = m.cuc;
  e.cus = m.cus;
  e.cic = m.cic;
  e.cis = m.cis;
  e.af0 = m.f0;
  e.af1 = m.f1;
  e.af2 = m.f2;
  e.toes = m.toes;
  return e;
}
inline Vec3 geodetic(const Vec3 &p) {
  constexpr double a = 6378137., e2 = 6.6943799901413165e-3;
  const double r2 = p.head<2>().squaredNorm();
  double z = p.z(), v = a;
  for (int i = 0; i < 100; ++i) {
    double old = z, s = z / std::sqrt(r2 + z * z);
    v = a / std::sqrt(1 - e2 * s * s);
    z = p.z() + v * e2 * s;
    if (std::abs(z - old) < 1e-4)
      break;
  }
  return {std::atan2(z, std::sqrt(r2)), std::atan2(p.y(), p.x()),
          std::sqrt(r2 + z * z) - v};
}
inline Mat3 enuFromEcef(const Vec3 &xyz) {
  auto llh = geodetic(xyz);
  double s = std::sin(llh[0]), c = std::cos(llh[0]), sn = std::sin(llh[1]),
         cn = std::cos(llh[1]);
  Mat3 R;
  R << -sn, cn, 0, -s * cn, -s * sn, c, c * cn, c * sn, s;
  return R;
}
inline Vec3 positionParameter(const std::vector<double> &v,
                              const std::string &type) {
  if (v.size() != 3 || !std::all_of(v.begin(), v.end(),
                                    [](double x) { return std::isfinite(x); }))
    throw std::invalid_argument(
        "position must contain three finite coordinates");
  Vec3 p(v[0], v[1], v[2]);
  if (type == "llh") {
    if (std::abs(v[0]) > 90 || std::abs(v[1]) > 180)
      throw std::invalid_argument("invalid latitude/longitude");
    double lat = v[0] * M_PI / 180, lon = v[1] * M_PI / 180, s = std::sin(lat),
           c = std::cos(lat),
           n = 6378137. / std::sqrt(1 - 6.6943799901413165e-3 * s * s);
    p = Vec3((n + v[2]) * c * std::cos(lon), (n + v[2]) * c * std::sin(lon),
             (n * (1 - 6.6943799901413165e-3) + v[2]) * s);
  } else if (type != "ecef")
    throw std::invalid_argument("position type must be llh or ecef");
  if (p.norm() < 6e6 || p.norm() > 7e6)
    throw std::invalid_argument(
        "position must be a surveyed terrestrial coordinate");
  return p;
}
template <class T> inline void assignVector(T &target, const Vec3 &value) {
  target.x = value.x();
  target.y = value.y();
  target.z = value.z();
}
template <class T> inline void assignCovariance(T &target, const Mat3 &value) {
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      target[3 * r + c] = value(r, c);
}
// A solution message for one rover epoch. NONE follows the common rule
// (gnss_utils::setNoSolution): NaN position, velocity and covariances.
// Covariances and the ENU velocity use the local ENU frame at the position;
// pos_enu is relative to `origin` (the node sets local_origin.pos or the base
// station); without one, the first FIX becomes the origin.
inline rosmsg::GnssSolution solutionMessage(const Solution &s,
                                            std::optional<Vec3> &origin) {
  rosmsg::GnssSolution m;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto ms = static_cast<std::int64_t>(std::llround(s.gpst * 1000.));
  m.time_week = ms / 604800000;
  m.time_tow = (ms % 604800000) * .001;
  m.header.stamp.sec = ms / 1000 + 315964800;
  m.header.stamp.nanosec = (ms % 1000) * 1000000;
  m.header.frame_id = "gnss_link";
  m.solution_source = m.SOLUTION_SOURCE_COMPUTED;
  m.ratio = std::isfinite(s.ratio) && s.ratio > 0 ? s.ratio : nan;
  m.age_diff = m.gdop = m.pdop = m.hdop = m.vdop = nan;
  m.num_sats = 0;
  assignVector(m.pos_enu_org_ecef, origin.value_or(Vec3::Constant(nan)));
  const bool valid = s.valid && s.position.allFinite();
  if (!valid) {
    gnss_utils::setNoSolution(m);
    return m;
  }
  m.status = s.kind == UpdateKind::DeadReckoning ? m.STATUS_DR
             : s.fixed                           ? m.STATUS_FIX
                                                 : m.STATUS_FLOAT;
  const Vec3 &p = s.position;
  assignVector(m.pos_ecef, p);
  const Vec3 llh = geodetic(p);
  m.latitude = llh[0] * 180 / M_PI;
  m.longitude = llh[1] * 180 / M_PI;
  m.altitude = llh[2];
  if (s.fixed && !origin)
    origin = p;
  assignVector(m.pos_enu_org_ecef, origin.value_or(Vec3::Constant(nan)));
  assignVector(m.pos_enu, origin ? Vec3(enuFromEcef(*origin) * (p - *origin))
                                 : Vec3::Constant(nan));
  const Mat3 E = enuFromEcef(p);
  const bool pos_cov = s.position_covariance.allFinite();
  assignCovariance(m.pos_cov_ecef,
                   pos_cov ? s.position_covariance : Mat3::Constant(nan));
  assignCovariance(m.pos_enu_cov,
                   pos_cov ? Mat3(E * s.position_covariance * E.transpose())
                           : Mat3::Constant(nan));
  if (!s.velocity.allFinite()) {
    gnss_utils::setUnknownVelocity(m);
    return m;
  }
  assignVector(m.vel_ecef, s.velocity);
  assignVector(m.vel_enu, Vec3(E * s.velocity));
  const bool vel_cov = s.velocity_covariance.allFinite();
  assignCovariance(m.vel_cov_ecef,
                   vel_cov ? s.velocity_covariance : Mat3::Constant(nan));
  assignCovariance(m.vel_enu_cov,
                   vel_cov ? Mat3(E * s.velocity_covariance * E.transpose())
                           : Mat3::Constant(nan));
  return m;
}
} // namespace gnss_fgo
