#pragma once
// ROS message <-> FGO input conversion (no rclcpp): stamps and observations.
#include "preprocessor.hpp"
#include "gnss_ros_standardization/rtklib_compat.hpp"
#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/obs_converter.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include <gnss_ros_standardization/msg/gnss_observations.hpp>
#include <gnss_ros_standardization/msg/gnss_solution.hpp>
#include <algorithm>
#include <array>
#include <cstdint>

namespace gnss_fgo {
// Header stamps may be UTC (receiver drivers) or GPST (some converted bags);
// the caller passes which.
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
// Which observations the node may use. The codes themselves are chosen per
// epoch pair by gnss_utils::selectSignals, shared with the RTK node.
struct SignalPolicy {
  gnss_utils::FrequencyMask frequencies;
  std::set<std::string> systems{"G", "E", "J", "C"};
  std::set<int> excluded; // RTKLIB satellite numbers
};
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
// Estimator constellation index and RTKLIB satellite number.
inline std::pair<int, int> satelliteId(const std::string &system, int prn) {
  const int sat =
      !supportedSatellite(system, prn) ? 0
      : system == "G"                  ? satno(SYS_GPS, prn)
      : system == "E"                  ? satno(SYS_GAL, prn)
      : system == "J"                  ? satno(SYS_QZS, prn)
                                       : satno(SYS_CMP, prn);
  if (sat <= 0)
    throw std::invalid_argument("unsupported satellite system/PRN: " + system +
                                std::to_string(prn));
  return {system == "G" ? 0 : system == "E" ? 1 : system == "J" ? 2 : 3, sat};
}
// Carrier wavelength of an RTKLIB observation code such as "1C".
inline double wavelength(const std::string &system, const std::string &code) {
  const int sys = system == "G"   ? SYS_GPS
                  : system == "E" ? SYS_GAL
                  : system == "J" ? SYS_QZS
                  : system == "C" ? SYS_CMP
                                  : SYS_NONE;
  const double frequency = code2freq(sys, obs2code(code.c_str()), 0);
  if (!(frequency > 0))
    throw std::invalid_argument("unsupported signal: " + system + code);
  return CLIGHT / frequency;
}
inline PreciseTime weekTime(std::uint32_t week, double tow) {
  if (!week || !std::isfinite(tow) || tow < 0 || tow >= 604800.)
    throw std::invalid_argument("invalid GPST week/tow");
  const auto whole = static_cast<std::int64_t>(std::floor(tow));
  return {static_cast<std::int64_t>(week) * 604800 + whole, tow - whole};
}
// receiver: 1 = rover, 2 = base (RTKLIB's obsd_t.rcv).
inline RawEpoch decodeObservation(const rosmsg::GnssObservations &m,
                                  int receiver, const SignalPolicy &policy,
                                  const gnss_utils::SignalChoice &choice,
                                  DecodeStats *stats = nullptr) {
  RawEpoch out;
  out.time = weekTime(m.week, m.tow);
  std::map<int, RawSatellite> satellites;
  std::map<int, std::array<bool, 3>> filled;
  const gtime_t time = gpst2time(m.week, m.tow);
  for (const auto &r : m.observations) {
    if (!policy.systems.count(r.system))
      continue;
    if (!supportedSatellite(r.system, r.prn)) {
      if (stats)
        ++stats->unsupported_satellites;
      continue;
    }
    const obsd_t o =
        gnss_utils::convertObs(r, time, receiver, policy.frequencies);
    if (!o.sat || policy.excluded.count(o.sat))
      continue;
    for (int f = 0; f < 3; ++f) {
      if (!o.code[f])
        continue;
      const auto selected = choice.codes.find({o.sat, f});
      if (selected == choice.codes.end() ||
          selected->second[receiver - 1] != o.code[f])
        continue;
      auto [system, sat] = satelliteId(r.system, r.prn);
      auto &s = satellites[sat];
      s.system = system;
      s.sat = sat;
      s.prn = r.prn;
      if (filled[sat][f]) {
        if (stats)
          ++stats->duplicate_signals;
        continue;
      }
      filled[sat][f] = true;
      const std::string code = code2obs(o.code[f]);
      s.signals[f] = "L" + code;
      s.wavelength[f] = s.doppler_wavelength[f] = wavelength(r.system, code);
      s.pr[f] = o.P[f];
      s.phase[f] = o.L[f];
      // convertObs maps a missing Doppler to 0, which is also a valid
      // Doppler; the message's NaN marks a missing one.
      s.doppler[f] = std::isfinite(r.d) &&
                             std::abs(r.d) <= std::numeric_limits<float>::max()
                         ? double(o.D[f])
                         : std::numeric_limits<double>::quiet_NaN();
      s.snr[f] = s.doppler_snr[f] = o.SNR[f];
      s.lli[f] = o.LLI[f] & 3;
      if (choice.switched.count({o.sat, f}))
        s.lli[f] |= 1;
    }
  }
  for (auto &[sat, s] : satellites)
    out.sats.push_back(std::move(s));
  return out;
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
