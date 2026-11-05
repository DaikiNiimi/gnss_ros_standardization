#include "gnss_ros_standardization/gnss_utils.hpp"
#include <cmath>
#include <stdexcept>
#include <limits>
#include <cstring>
#include <ctime>
#include <algorithm>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

namespace gnss_utils {

std::string systemCode(int sys) {
  switch (sys) {
    case SYS_GPS: return "G";
    case SYS_GLO: return "R";
    case SYS_GAL: return "E";
    case SYS_QZS: return "J";
    case SYS_CMP: return "C";
    case SYS_IRN: return "I";
    case SYS_SBS: return "S";
    default:      return "U";
  }
}

std::string satId(int sat) {
  char id[8] = {0};
  satno2id(sat, id);
  return std::string(id);
}

std::string solqToString(int stat) {
  switch (stat) {
    case SOLQ_FIX:    return "FIX";
    case SOLQ_FLOAT:  return "FLOAT";
    case SOLQ_SBAS:   return "SBAS";
    case SOLQ_DGPS:   return "DGPS";
    case SOLQ_SINGLE: return "SINGLE";
    case SOLQ_PPP:    return "PPP";
    case SOLQ_DR:     return "DR";
    case SOLQ_NONE:
    default:          return "NONE";
  }
}

int resolveSatellite(const std::string& id, const std::string& system, int prn) {
  // RTKLIB accepts a numeric prefix; the public identifier must be complete.
  if (!id.empty()) {
    const size_t first_digit = (id[0] >= '0' && id[0] <= '9') ? 0 : 1;
    if (first_digit == id.size() || id.size() - first_digit > 3 ||
        id.find_first_not_of("0123456789", first_digit) != std::string::npos) return 0;
  }
  const int from_id = id.empty() ? 0 : satid2no(id.c_str());
  if (!id.empty() && !from_id) return 0;
  if (system.empty()) {
    if (prn == 0) return from_id;
    if (!from_id) return 0;
    return resolveSatellite(id, systemCode(satsys(from_id, nullptr)), prn);
  }
  int mask = 0;
  if (system == "G") mask = SYS_GPS;
  else if (system == "R") mask = SYS_GLO;
  else if (system == "E") mask = SYS_GAL;
  else if (system == "J") { mask = SYS_QZS; if (prn >= 1 && prn <= 10) prn += 192; }
  else if (system == "C") mask = SYS_CMP;
  else if (system == "I") mask = SYS_IRN;
  else if (system == "S") { mask = SYS_SBS; if (prn >= 20 && prn <= 58) prn += 100; }
  else return 0;
  // Allow a constellation with only a satid; reject conflicting constellations.
  if (prn == 0 && from_id) return satsys(from_id, nullptr) == mask ? from_id : 0;
  const int from_fields = satno(mask, prn);
  if (!from_fields || (from_id && from_fields != from_id)) return 0;
  return from_fields;
}

int resolveSignal(const std::string& signal, int backend_code) {
  const int from_string = signal.empty() ? 0 : obs2code(signal.c_str());
  if (!signal.empty() && (!from_string || signal != code2obs(from_string))) return 0;
  if (backend_code < 0 || backend_code > MAXCODE) return 0;
  if (from_string && backend_code && from_string != backend_code) return 0;
  return from_string ? from_string : backend_code;
}

bool isRosTimeRepresentable(gtime_t t_gpst) {
  if (!std::isfinite(t_gpst.sec) || t_gpst.sec<0 || t_gpst.sec>=1) return false;
  const auto utc=gpst2utc(t_gpst);
  return utc.time>=0 && utc.time<=std::numeric_limits<int32_t>::max();
}

rclcpp::Time gpstToUtcRosTime(gtime_t t_gpst) {
  if (!isRosTimeRepresentable(t_gpst))
    throw std::out_of_range("GNSS epoch outside ROS builtin Time range");
  const gtime_t t_utc = gpst2utc(t_gpst);
  const int64_t nsec = static_cast<int64_t>(t_utc.time) * 1000000000LL +
                       std::llround(t_utc.sec * 1e9);
  if (nsec / 1000000000LL > std::numeric_limits<int32_t>::max())
    throw std::out_of_range("GNSS epoch rounds outside ROS builtin Time range");
  return rclcpp::Time(nsec);
}

bool nmeaUtcToGpsTime(int year, int month, int day, double hms,
                      uint32_t& week, double& tow) {
  if (year < 1980 || year > 2099 || month < 1 || month > 12 || day < 1 || day > 31 ||
      !std::isfinite(hms) || hms < 0 || hms >= 240000) return false;
  const int hh = static_cast<int>(hms / 10000.0);
  const int mm = static_cast<int>((hms - hh * 10000) / 100.0);
  const double ss = hms - hh * 10000 - mm * 100;
  if (mm > 59 || ss < 0 || ss >= 60) return false;
  double ep[6] = {static_cast<double>(year), static_cast<double>(month),
                  static_cast<double>(day), static_cast<double>(hh),
                  static_cast<double>(mm), ss};
  gtime_t t_utc = epoch2time(ep);
  if (t_utc.time == 0) return false;
  double normalized[6];
  time2epoch(t_utc, normalized);
  for (int i = 0; i < 5; ++i) if (normalized[i] != ep[i]) return false;
  gtime_t t_gpst = utc2gpst(t_utc);
  int w = 0;
  const double t = time2gpst(t_gpst, &w);
  if (w <= 0) return false;
  week = static_cast<uint32_t>(w);
  tow = t;
  return true;
}

bool utcRosTimeToGpst(const rclcpp::Time& t_utc_ros, uint32_t& week, double& tow) {
  gtime_t t_utc;
  const int64_t ns = t_utc_ros.nanoseconds();
  t_utc.time = static_cast<time_t>(ns / 1000000000LL);
  t_utc.sec = static_cast<double>(ns % 1000000000LL) / 1e9;
  const gtime_t t_gpst = utc2gpst(t_utc);
  int w = 0;
  const double t = time2gpst(t_gpst, &w);
  if (w <= 0) return false;
  week = static_cast<uint32_t>(w);
  tow = t;
  return true;
}

bool GnssEpochAligner::parseSource(const std::string& name, Source& out) {
  if (name == "header") out = Source::Header;
  else if (name == "tow_auto_offset") out = Source::TowAutoOffset;
  else return false;
  return true;
}

rclcpp::Time GnssEpochAligner::align(uint32_t week, double tow,
                                     const rclcpp::Time& stamp,
                                     std::string* warn) {
  if (warn) warn->clear();
  if (cfg_.source == Source::Header) return stamp;
  // The RANGE matters, not just finiteness. gpst2time() happily accepts a tow
  // of -1 or 604800 and returns a time one second before the week, or the
  // first second of the next one; the measurement would then be applied a week
  // away from where it belongs, and this function's contract ("on ANY fallback
  // to Header, warn is set") would be broken by a path that never falls back.
  // Same canonical range the epoch matcher rejects on
  // (RoverBaseEpochMatcher::validTow).
  if (week == 0 || week > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
      !std::isfinite(tow) || tow < 0.0 || tow >= 604800.0 ||
      !isRosTimeRepresentable(gpst2time(static_cast<int>(week),tow))) {
    if (warn) *warn = "GNSS week/tow invalid - applying at header.stamp";
    return stamp;
  }
  // All arithmetic in raw nanoseconds: gpstToUtcRosTime() returns a
  // system-clock rclcpp::Time and mixing clock types in +/- throws.
  const int64_t meas_ns =
      gpstToUtcRosTime(gpst2time(static_cast<int>(week), tow)).nanoseconds();
  const double ctow = week * 604800.0 + tow;
  if (last_ctow_ >= 0 && ctow < last_ctow_ - 5.0) {
    reset();
    if (warn) *warn = "GNSS epoch moved backwards - resetting clock offset";
  }
  last_ctow_ = ctow;
  const double d = (stamp.nanoseconds() - meas_ns) / 1e9;

  // TowAutoOffset: sliding-window minimum of d_k. A backwards stamp step
  // (bag replay restart) invalidates the history, so restart the window.
  const double t_s = stamp.nanoseconds() / 1e9;
  if (!offsets_.empty() && t_s < offsets_.back().first) offsets_.clear();
  offsets_.emplace_back(t_s, d);
  // size() > 1, not !empty(): the sample just pushed has age 0, so with a
  // non-negative window the loop stops on its own - but offset_window_s is a
  // plain ROS parameter (ekf.gnss_offset_window_s) with no validation, and a
  // negative value would make `0 > window` true, empty the deque, and leave
  // front() below dereferencing nothing. Keeping the newest sample makes that
  // impossible whatever the configuration says.
  while (offsets_.size() > 1 &&
         t_s - offsets_.front().first > cfg_.offset_window_s) {
    offsets_.pop_front();
  }
  double offset = offsets_.front().second;
  for (const auto& p : offsets_) offset = std::min(offset, p.second);
  return rclcpp::Time(meas_ns + static_cast<int64_t>(offset * 1e9),
                      stamp.get_clock_type());
}

int canonicalGalCode(int code) {
  if (code & (1 << 8)) return (1 << 1) | (1 << 8);             // F/NAV
  if (code & (1 << 9)) return (1 << 0) | (1 << 2) | (1 << 9);  // I/NAV
  return code;
}

// Local helper from original ros2_rinex_writer.cpp
static inline gtime_t adjweek(gtime_t ref, int week, double tow_sec) {
  gtime_t t = gpst2time(week, tow_sec);
  double dt = timediff(t, ref);
  if (dt < -302400.0) t = timeadd(t, 604800.0);
  else if (dt > 302400.0) t = timeadd(t, -604800.0);
  return t;
}

eph_t msgToEph(const gnss_ros_standardization::msg::GnssEphemeris& m) {
  eph_t e{};
  e.sat = resolveSatellite(m.satid, m.system, m.prn);
  if (m.week > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
      !std::isfinite(m.toe) || m.toe < 0 || m.toe >= 604800) return {};
  int w = static_cast<int>(m.week);
  if (!std::isfinite(m.toc) || !std::isfinite(m.ttr) || m.toc < 0 || m.toc >= 604800 || m.ttr < 0 || m.ttr >= 604800) return {};
  e.toe = gpst2time(w, m.toe);
  
  // Use adjweek for toc/ttr relative to toe
  e.toc = adjweek(e.toe, w, m.toc);
  e.ttr = adjweek(e.toe, w, m.ttr);

  // Recalculate/normalize week based on system (as per original writer)
  int sys = satsys(e.sat, nullptr);
  int final_week = 0;
  if (sys == SYS_CMP) {
      if (e.toe.time != 0) (void)time2bdt(gpst2bdt(e.toe), &final_week);
  } else {
      if (e.toe.time != 0) (void)time2gpst(e.toe, &final_week);
  }
  e.week = final_week;

  e.A = m.a;
  e.e = m.e;
  e.i0 = m.i0;
  e.OMG0 = m.omg0;
  e.omg = m.omg;
  e.M0 = m.m0;
  e.deln = m.deln;
  e.OMGd = m.omgd;
  e.idot = m.idot;
  e.Adot = m.adot;
  e.ndot = m.ndot;

  e.crc = m.crc;
  e.crs = m.crs;
  e.cuc = m.cuc;
  e.cus = m.cus;
  e.cic = m.cic;
  e.cis = m.cis;
  
  e.f0 = m.f0;
  e.f1 = m.f1;
  e.f2 = m.f2;

  if (m.tgd.size() > 6) {
    RCLCPP_WARN_ONCE(rclcpp::get_logger("gnss_utils"),
                    "Rejected ephemeris: tgd has more than 6 backend slots");
    return eph_t{};
  }
  std::copy(m.tgd.begin(), m.tgd.end(), e.tgd);

  e.iode = static_cast<int>(m.iode);
  e.iodc = static_cast<int>(m.iodc);

  e.svh = static_cast<int>(m.svh);
  e.sva = static_cast<int>(m.sva);
  e.code = static_cast<int>(m.code);
  e.flag = static_cast<int>(m.flag);
  e.fit  = static_cast<int>(m.fit);
  e.toes = m.toes;

  return e;
}

geph_t msgToGeph(const gnss_ros_standardization::msg::GlonassEphemeris& m) {
  geph_t g{};
  g.sat = resolveSatellite(m.satid, m.system, m.prn);
  if (m.week > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
      !std::isfinite(m.toe) || m.toe < 0 || m.toe >= 604800) return {};
  int w = static_cast<int>(m.week);
  
  // RTKLIB geph_t stores GPST, just like eph_t. UTC conversion belongs
  // only at the RINEX text/receiver boundary, not this ROS interchange.
  if (m.pos.size() != 3 || m.vel.size() != 3 || m.acc.size() != 3) return geph_t{};
  if (!std::isfinite(m.tof) || m.tof < 0 || m.tof >= 604800) return {};
  g.toe = gpst2time(w, m.toe);
  g.tof = adjweek(g.toe, w, m.tof);
  
  g.frq = static_cast<signed char>(m.frq);
  
  g.pos[0] = m.pos[0]; g.pos[1] = m.pos[1]; g.pos[2] = m.pos[2];
  g.vel[0] = m.vel[0]; g.vel[1] = m.vel[1]; g.vel[2] = m.vel[2];
  g.acc[0] = m.acc[0]; g.acc[1] = m.acc[1]; g.acc[2] = m.acc[2];
  
  g.iode = static_cast<int>(m.iode);
  g.svh = static_cast<int>(m.svh);
  g.age = static_cast<int>(m.age);
  g.flags = m.flags;
  g.sva = m.sva;
  
  g.gamn = m.gamn;
  g.taun = m.taun;
  g.dtaun = m.dtaun;
  
  return g;
}

namespace {
bool fitsUnsigned(int value, int maximum) { return value >= 0 && value <= maximum; }
bool representableTime(gtime_t time) {
  // Bound before RTKLIB's time2gpst casts its week to signed int.
  const long double seconds = static_cast<long double>(time.time) - 315964800.L;
  return std::isfinite(time.sec) && time.sec >= 0 && time.sec < 1 && seconds >= 0 &&
         seconds < (static_cast<long double>(std::numeric_limits<int>::max()) + 1) * 604800.L;
}
bool nearestWeekPreserves(gtime_t reference, gtime_t other) {
  if (!representableTime(reference) || !representableTime(other)) return false;
  int week = 0;
  (void)time2gpst(reference, &week);
  const auto restored = adjweek(reference, week, time2gpst(other, nullptr));
  return std::abs(timediff(restored, other)) < 1e-6;
}
}

const char* ephemerisMessageError(const eph_t& e) {
  const int sys = satsys(e.sat, nullptr);
  if (sys == SYS_NONE || sys == SYS_GLO || sys == SYS_SBS) return "satellite is not Keplerian";
  if (!fitsUnsigned(e.iode, 65535)) return "iode outside uint16 [0,65535]";
  if (!fitsUnsigned(e.iodc, 65535)) return "iodc outside uint16 [0,65535]";
  if (!fitsUnsigned(e.svh, 65535)) return "svh outside uint16 [0,65535]";
  if (!fitsUnsigned(e.code, 65535)) return "code outside uint16 [0,65535]";
  if (!fitsUnsigned(e.flag, 255)) return "flag outside uint8 [0,255]";
  if (!std::isfinite(e.fit) || e.fit < 0 || e.fit > 255 || std::trunc(e.fit) != e.fit)
    return "fit is not an integer in uint8 [0,255]";
  if (!nearestWeekPreserves(e.toe, e.toc) || !nearestWeekPreserves(e.toe, e.ttr))
    return "reference times cannot round-trip through week/TOW";
  for (double v : {e.A, e.e, e.i0, e.OMG0, e.omg, e.M0, e.deln, e.OMGd, e.idot,
                   e.crc, e.crs, e.cuc, e.cus, e.cic, e.cis, e.toes, e.f0, e.f1, e.f2,
                   e.Adot, e.ndot})
    if (!std::isfinite(v)) return "non-finite orbit/clock coefficient";
  for (double v : e.tgd) if (!std::isfinite(v)) return "non-finite group delay";
  if (e.A <= 0 || e.e < 0 || e.e >= 1) return "invalid semi-major axis/eccentricity";
  return nullptr;
}

const char* ephemerisMessageError(const geph_t& g) {
  if (satsys(g.sat, nullptr) != SYS_GLO) return "satellite is not GLONASS";
  if (!fitsUnsigned(g.iode, 255)) return "iode outside uint8 [0,255]";
  if (!fitsUnsigned(g.svh, 255)) return "svh outside uint8 [0,255]";
  if (!fitsUnsigned(g.age, 255)) return "age outside uint8 [0,255]";
  if (g.frq < -128 || g.frq > 127) return "frq outside int8 [-128,127]";
  for (int i = 0; i < 3; ++i)
    if (!std::isfinite(g.pos[i]) || !std::isfinite(g.vel[i]) || !std::isfinite(g.acc[i]))
      return "non-finite GLONASS state vector";
  if (!std::isfinite(g.taun) || !std::isfinite(g.gamn) || !std::isfinite(g.dtaun))
    return "non-finite GLONASS clock coefficient";
  if (!nearestWeekPreserves(g.toe, g.tof)) return "reference times cannot round-trip through week/TOW";
  return nullptr;
}

gnss_ros_standardization::msg::GnssEphemeris ephToMsg(const eph_t& e) {
  if (const char* reason = ephemerisMessageError(e)) throw std::invalid_argument(reason);
  gnss_ros_standardization::msg::GnssEphemeris m;

  int prn = 0;
  const int sys = satsys(e.sat, &prn);
  m.system = systemCode(sys);
  m.prn    = prn;
  m.satid  = satId(e.sat);

  int w = 0;
  m.toe  = time2gpst(e.toe, &w);
  m.week = static_cast<uint32_t>(w);
  m.toc  = time2gpst(e.toc, &w);
  m.ttr  = time2gpst(e.ttr, &w);
  m.toes = e.toes;

  m.a = e.A;
  m.e = e.e;
  m.i0 = e.i0;
  m.omg0 = e.OMG0;
  m.omg = e.omg;
  m.m0 = e.M0;
  m.deln = e.deln;
  m.omgd = e.OMGd;
  m.idot = e.idot;
  m.adot = e.Adot;
  m.ndot = e.ndot;

  m.crc = e.crc;
  m.crs = e.crs;
  m.cuc = e.cuc;
  m.cus = e.cus;
  m.cic = e.cic;
  m.cis = e.cis;

  m.f0 = e.f0;
  m.f1 = e.f1;
  m.f2 = e.f2;

  // Keep the legacy two-element shape when extended slots carry no data.
  // The existing message already permits all six backend slots.
  size_t delays = 2;
  for (size_t i = 2; i < 6; ++i) if (e.tgd[i] != 0) delays = i + 1;
  m.tgd.assign(e.tgd, e.tgd + delays);

  m.iode = e.iode;
  m.iodc = e.iodc;

  m.svh = e.svh;
  m.sva = e.sva;
  m.code = e.code;
  m.flag = static_cast<uint8_t>(e.flag);
  m.fit  = static_cast<uint8_t>(e.fit);

  return m;
}

gnss_ros_standardization::msg::GlonassEphemeris gephToMsg(const geph_t& g) {
  if (const char* reason = ephemerisMessageError(g)) throw std::invalid_argument(reason);
  gnss_ros_standardization::msg::GlonassEphemeris m;
  m.system = "R";

  int prn = 0; (void)satsys(g.sat, &prn);
  m.prn = prn;
  m.satid = satId(g.sat);
  m.frq = g.frq;

  int w = 0;
  
  m.toe  = time2gpst(g.toe, &w);
  m.week = static_cast<uint32_t>(w);
  m.tof  = time2gpst(g.tof, &w);

  m.pos = { g.pos[0], g.pos[1], g.pos[2] };
  m.vel = { g.vel[0], g.vel[1], g.vel[2] };
  m.acc = { g.acc[0], g.acc[1], g.acc[2] };

  m.iode = g.iode;
  m.svh = g.svh;
  m.age = g.age;
  m.flags = g.flags;
  m.sva = g.sva;
  m.gamn = g.gamn;
  m.taun = g.taun;
  m.dtaun = g.dtaun;
  
  return m;
}

gnss_ros_standardization::msg::GnssObservation obsToMsg(const obsd_t& o, int kf, const nav_t* nav) {
  if (kf < 0 || kf >= NFREQ + NEXOBS) throw std::out_of_range("Observation backend slot");
  gnss_ros_standardization::msg::GnssObservation obs;
  
  int prn = 0;
  const int sys = satsys(o.sat, &prn);
  if (sys == SYS_NONE || !o.code[kf] || !*code2obs(o.code[kf]) || code2idx(sys, o.code[kf]) < 0) return obs;
  obs.system = systemCode(sys);
  obs.prn    = prn;
  obs.sat    = o.sat;
  if (sys == SYS_GLO) {
    if (o.freq <= 13) obs.freq = o.freq;
    else if (nav) {
      // glo_fcn has no timestamp; it cannot establish the channel at o.time.
      // Explicit per-observation information above always survives conflicts.
      const auto* g = selectGlonassEphemeris(*nav, o.sat, o.time);
      if (g && g->frq >= -7 && g->frq <= 6) obs.freq = g->frq + 7;
    }
  }
  obs.satid  = satId(o.sat);

  if (o.code[kf]) {
    obs.code = o.code[kf];
    if (const char* sig = code2obs(o.code[kf]); sig) obs.code_str = sig;
  }

  const double unknown = std::numeric_limits<double>::quiet_NaN();
  obs.p             = std::isfinite(o.P[kf]) && o.P[kf] >= 0 ? o.P[kf] : unknown;
  obs.l             = std::isfinite(o.L[kf]) ? o.L[kf] : unknown;
  obs.d             = std::isfinite(o.D[kf]) ? o.D[kf] : unknown;
  obs.snr           = std::isfinite(o.SNR[kf]) && o.SNR[kf] >= 0 ? o.SNR[kf] : unknown;
  // Preserve the backend byte, including HALFA/HALFS (0x40/0x80).
  // Restricting LLI to RINEX's low three bits loses half-cycle state in ROS.
  obs.lli           = o.LLI[kf];

  return obs;
}

// ---- Math Helpers ----

void rotateCovariance(const double cov_ecef[9], double lat_rad, double lon_rad, double cov_enu[9]) {
  // R matrix (ECEF to ENU)
  // R = [-sin(lon)           cos(lon)          0      ]
  //     [-sin(lat)cos(lon)  -sin(lat)sin(lon)  cos(lat)]
  //     [ cos(lat)cos(lon)   cos(lat)sin(lon)  sin(lat)]
  
  double sl = sin(lat_rad);
  double cl = cos(lat_rad);
  double sL = sin(lon_rad);
  double cL = cos(lon_rad);
  
  double R[9];
  R[0] = -sL;      R[1] = cL;       R[2] = 0.0;
  R[3] = -sl*cL;   R[4] = -sl*sL;   R[5] = cl;
  R[6] = cl*cL;    R[7] = cl*sL;    R[8] = sl;
  
  // cov_enu = R * cov_ecef * R'
  // Intermediate T = cov_ecef * R'
  // T[i,j] = sum(cov_ecef[i,k] * R[j,k])  (since R' element (k,j) is R(j,k))
  
  double T[9];
  for (int i=0; i<3; ++i) {
    for (int j=0; j<3; ++j) {
      T[3*i + j] = cov_ecef[3*i + 0] * R[3*j + 0] +
                   cov_ecef[3*i + 1] * R[3*j + 1] +
                   cov_ecef[3*i + 2] * R[3*j + 2];
    }
  }
  
  // Result = R * T
  for (int i=0; i<3; ++i) {
    for (int j=0; j<3; ++j) {
      cov_enu[3*i + j] = R[3*i + 0] * T[0*3 + j] +
                         R[3*i + 1] * T[1*3 + j] +
                         R[3*i + 2] * T[2*3 + j];
    }
  }
}

void rotateCovarianceEnuToEcef(const double cov_enu[9], double lat_rad, double lon_rad, double cov_ecef[9]) {
  // R matrix (ECEF to ENU)
  // R = [-sin(lon)           cos(lon)          0      ]
  //     [-sin(lat)cos(lon)  -sin(lat)sin(lon)  cos(lat)]
  //     [ cos(lat)cos(lon)   cos(lat)sin(lon)  sin(lat)]
  
  double sl = sin(lat_rad);
  double cl = cos(lat_rad);
  double sL = sin(lon_rad);
  double cL = cos(lon_rad);
  
  double R[9];
  R[0] = -sL;      R[1] = cL;       R[2] = 0.0;
  R[3] = -sl*cL;   R[4] = -sl*sL;   R[5] = cl;
  R[6] = cl*cL;    R[7] = cl*sL;    R[8] = sl;
  
  // ECEF to ENU: C_enu = R * C_ecef * R^T
  // ENU to ECEF: C_ecef = R^T * C_enu * R
  
  // T = C_enu * R
  double T[9];
  for (int i=0; i<3; ++i) {
    for (int j=0; j<3; ++j) {
      T[3*i + j] = cov_enu[3*i + 0] * R[3*0 + j] +
                   cov_enu[3*i + 1] * R[3*1 + j] +
                   cov_enu[3*i + 2] * R[3*2 + j];
    }
  }
  
  // C_ecef = R^T * T
  for (int i=0; i<3; ++i) {
    for (int j=0; j<3; ++j) {
      cov_ecef[3*i + j] = R[3*0 + i] * T[0*3 + j] +
                          R[3*1 + i] * T[1*3 + j] +
                          R[3*2 + i] * T[2*3 + j];
    }
  }
}

Dops calculateDops(const ssat_t* ssat, int ns_max, double el_min_rad) {
    Dops d;
    double azel[MAXSAT * 2];
    int ns = 0;
    
    // Collect valid satellites from ssat array
    // RTKLIB stores azel even if vs is not set? 
    // Usually rtkpos updates ssat.vs (valid solution) for used satellites.
    // However, if we want DOP for the solution, we should check vs.
    
    for (int i = 0; i < ns_max; ++i) {
        if (ssat[i].vs) {
            azel[2*ns]   = ssat[i].azel[0]; // az
            azel[2*ns+1] = ssat[i].azel[1]; // el
            ns++;
        }
    }
    
    if (ns > 0) {
        double dop[4] = {0};
        dops(ns, azel, el_min_rad, dop);
        d.gdop = dop[0];
        d.pdop = dop[1];
        d.hdop = dop[2];
        d.vdop = dop[3];
    }
    
    return d;
}

// ---- DOP cache + staleness gate (msg-coupled DOP from receiver blocks) ----

void applyDopWithStaleness(gnss_ros_standardization::msg::GnssSolution& sol,
                           const DopCache& cache,
                           uint32_t pvt_week,
                           uint32_t pvt_tow_ms,
                           uint32_t pvt_period_ms) {
  constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
  auto set_nan = [&]() {
    sol.gdop = kNaN; sol.pdop = kNaN; sol.hdop = kNaN; sol.vdop = kNaN;
  };
  if (!cache.valid) { set_nan(); return; }
  if (cache.week != 0 && cache.week != pvt_week) { set_nan(); return; }
  if (pvt_period_ms == 0) { set_nan(); return; }
  // Asymmetric acceptance window [0, period]: DOP must belong to this PVT
  // epoch (dt = 0, arrived before PVT in same frame) or the immediately-prior
  // PVT epoch (dt = period, DOP arrived just after the previous PVT flush —
  // common SBF block ordering). Reject future-direction (dt < 0) and >1-cycle
  // stale (dt > period).
  const int64_t dt = static_cast<int64_t>(pvt_tow_ms) -
                     static_cast<int64_t>(cache.tow_ms);
  if (dt < 0 || dt > static_cast<int64_t>(pvt_period_ms)) { set_nan(); return; }
  sol.gdop = cache.gdop;
  sol.pdop = cache.pdop;
  sol.hdop = cache.hdop;
  sol.vdop = cache.vdop;
}

bool validatePublishRate(int& rate_hz, int /*fallback*/, int max_hz, const rclcpp::Logger& /*logger*/) {
  constexpr int kMin = 1;
  if (rate_hz < kMin || rate_hz > max_hz) {
    throw std::invalid_argument("rate_hz must be within [1, " +
                                std::to_string(max_hz) + "] Hz");
  }
  return true;
}

LocalOrigin declareLocalOrigin(rclcpp::Node& node) {
  const auto mode = node.declare_parameter<std::string>("local_origin.mode", "auto");
  const auto pos = node.declare_parameter<std::vector<double>>(
      "local_origin.pos", std::vector<double>{0.0, 0.0, 0.0});
  const auto type = node.declare_parameter<std::string>("local_origin.postype", "llh");
  if (mode != "auto" && mode != "fixed")
    throw std::invalid_argument("local_origin.mode must be auto or fixed");
  if (type != "llh" && type != "ecef")
    throw std::invalid_argument("local_origin.postype must be llh or ecef");
  if (pos.size() != 3 || !std::isfinite(pos[0]) || !std::isfinite(pos[1]) ||
      !std::isfinite(pos[2]))
    throw std::invalid_argument("local_origin.pos must be three finite numbers");
  const bool zero = pos[0] == 0.0 && pos[1] == 0.0 && pos[2] == 0.0;
  LocalOrigin origin;
  if (mode == "auto") {
    if (!zero)
      throw std::invalid_argument(
          "local_origin.pos is set but local_origin.mode is auto: set "
          "local_origin.mode to fixed to use it, or leave pos at [0, 0, 0]");
    return origin;
  }
  origin.fixed = true;
  if (type == "llh") {
    if (std::abs(pos[0]) > 90.0 || std::abs(pos[1]) > 180.0)
      throw std::invalid_argument(
          "local_origin.pos: latitude must be within +-90 deg and longitude "
          "within +-180 deg");
    origin.llh[0] = pos[0] * (M_PI / 180.0);
    origin.llh[1] = pos[1] * (M_PI / 180.0);
    origin.llh[2] = pos[2];
    pos2ecef(origin.llh, origin.ecef);
  } else {
    if (zero)
      throw std::invalid_argument(
          "local_origin.pos [0, 0, 0] with postype ecef is the centre of the Earth");
    origin.ecef[0] = pos[0];
    origin.ecef[1] = pos[1];
    origin.ecef[2] = pos[2];
    ecef2pos(origin.ecef, origin.llh);
  }
  return origin;
}

// ---- Lightweight NMEA Parser ----

std::vector<std::string> NmeaParser::splitString(const std::string& str, char delimiter) {
  std::vector<std::string> tokens;
  size_t start = 0;
  size_t end = str.find(delimiter);
  while (end != std::string::npos) {
    tokens.push_back(str.substr(start, end - start));
    start = end + 1;
    end = str.find(delimiter, start);
  }
  tokens.push_back(str.substr(start));
  return tokens;
}

double NmeaParser::parseDouble(const std::string& str) {
  const double missing = std::numeric_limits<double>::quiet_NaN();
  if (str.empty() || str.find_first_of(" \t\r\n") != std::string::npos) return missing;
  try {
    size_t end = 0;
    double value = std::stod(str, &end);
    return end == str.size() && std::isfinite(value) ? value : missing;
  } catch (...) { return missing; }
}

int NmeaParser::parseInteger(const std::string& str) {
  if (str.empty() || str.find_first_not_of("0123456789") != std::string::npos) return -1;
  try { return std::stoi(str); } catch (...) { return -1; }
}

// ddmm.mmmmm latitude / dddmm.mmmmm longitude; decimal point is optional.
double NmeaParser::parseCoordinate(const std::string& coord_str, const std::string& hem) {
  const double missing = std::numeric_limits<double>::quiet_NaN();
  const bool latitude = hem == "N" || hem == "S";
  if (!latitude && hem != "E" && hem != "W") return missing;
  const size_t degrees = latitude ? 2 : 3;
  if (coord_str.size() < degrees + 2 ||
      coord_str.substr(0, degrees + 2).find_first_not_of("0123456789") != std::string::npos ||
      (coord_str.size() > degrees + 2 && coord_str[degrees + 2] != '.')) return missing;
  const double deg = parseDouble(coord_str.substr(0, degrees));
  const double minutes = parseDouble(coord_str.substr(degrees));
  const double result = deg + minutes / 60;
  if (!std::isfinite(result) || minutes < 0 || minutes >= 60 ||
      result > (latitude ? 90 : 180)) return missing;
  return hem == "S" || hem == "W" ? -result : result;
}

namespace {

// hhmmss.ss (e.g. 123459.50) → seconds-of-day.
// Returns -1.0 sentinel for non-finite, negative, or out-of-range inputs.
double hmsToSecondsOfDay(double hms) {
  if (!std::isfinite(hms) || hms < 0.0 || hms >= 240000.0) return -1.0;
  const int ihms = static_cast<int>(hms);
  const int hh =  ihms / 10000;
  const int mm = (ihms / 100) % 100;
  const double ss = hms - hh * 10000 - mm * 100;
  if (hh < 0 || hh > 23 || mm < 0 || mm > 59 || ss < 0.0 || ss >= 60.0) return -1.0;
  return hh * 3600.0 + mm * 60.0 + ss;
}

// Same-epoch check for two seconds-of-day values with UTC day-rollover handling.
// Returns false if either sod is the -1 sentinel.
bool sameEpoch(double sod_a, double sod_b, double tol_sec) {
  if (sod_a < 0.0 || sod_b < 0.0) return false;
  double dt = std::abs(sod_a - sod_b);
  if (dt > 43200.0) dt = 86400.0 - dt;  // shortest wrap (00:00 UTC crossing)
  return dt < tol_sec;
}

}  // namespace

void NmeaParser::reset() {
  const int y=reference_year_, m=reference_month_, d=reference_day_;
  *this=NmeaParser{};
  if (y) setReferenceDate(y,m,d);
}

bool NmeaParser::parseSentence(const std::string& sentence, gnss_ros_standardization::msg::GnssSolution& solution,
                               Clock::time_point now) {
  input_arrival_=now;
  // RMC/ZDA must refresh the live date. GGA advancing the cached day must not
  // perpetually refresh the reception age of the original date-bearing sentence.
  if (has_date_cache_ && !explicit_date_ &&
      (now<date_arrival_ || now-date_arrival_>std::chrono::seconds(60))) {
    has_date_cache_=false;
    pgga_week_=0; pgga_tow_=0;
  }
  if (sentence.empty() || sentence[0] != '$') return false;

  // Checksums remain optional for direct library callers, but a present
  // checksum must contain exactly two hexadecimal digits and no trailing data.
  const size_t end = sentence.find_last_not_of("\r\n") + 1;
  const std::string line = sentence.substr(0, end);
  const size_t asterisk_pos = line.find('*');
  if (asterisk_pos != std::string::npos) {
    if (asterisk_pos + 3 != line.size()) return false;
    auto hex = [](char c) -> int {
      if (c >= '0' && c <= '9') return c - '0';
      if (c >= 'A' && c <= 'F') return c - 'A' + 10;
      if (c >= 'a' && c <= 'f') return c - 'a' + 10;
      return -1;
    };
    const int high = hex(line[asterisk_pos + 1]), low = hex(line[asterisk_pos + 2]);
    if (high < 0 || low < 0) return false;
    int checksum = 0;
    for (size_t i = 1; i < asterisk_pos; ++i) checksum ^= line[i];
    if (checksum != high * 16 + low) return false;
  }
  std::string payload = line.substr(1, asterisk_pos == std::string::npos ?
                                    std::string::npos : asterisk_pos - 1);
  std::vector<std::string> fields = splitString(payload, ',');
  if (fields.empty()) return false;
  std::string type = fields[0];
  if (type.length() < 3) return false;
  std::string sentence_id = type.substr(type.length() - 3);

  // GSA is recognized but intentionally ignored. PDOP/VDOP from GSA have no
  // robust TOW-match against GGA (GSA has no timestamp), so we never apply it.
  if (sentence_id == "GSA") {
    applyGsa(fields);
    return false;
  }

  // GGA/RMC/GST all carry hhmmss.ss in field[1]; derive the epoch sod for the
  // pending-buffer state machine.
  uint8_t bit = 0;
  if      (sentence_id == "GGA") bit = SENT_GGA;
  else if (sentence_id == "RMC") bit = SENT_RMC;
  else if (sentence_id == "GST") bit = SENT_GST;
  else if (sentence_id == "ZDA") bit = SENT_ZDA;
  else return false;

  if (fields.size() < 2) return false;
  const double sod = hmsToSecondsOfDay(parseDouble(fields[1]));
  if (sod < 0.0) return false;

  // Tolerance for sameEpoch: half of learned epoch period, default 0.5s.
  const double tol = (epoch_period_ > 0.0) ? std::max(0.05, epoch_period_ * 0.5) : 0.5;

  // ---- Step 1: epoch-boundary flush ----
  // If a pending epoch exists and this sentence's sod is outside its tolerance,
  // the previous epoch is over. Flush it (NaN for unseen types) and start fresh.
  bool produced = false;
  if (pending_sod_ >= 0.0 && !sameEpoch(pending_sod_, sod, tol)) {
    flushPending(solution);
    produced = true;
    learned_ = true;  // first complete boundary flush — enable eager flush
  }

  // ---- Step 2: apply the new sentence to (possibly fresh) pending ----
  if (pending_sod_ < 0.0) pending_sod_ = sod;

  switch (bit) {
    case SENT_GGA: applyGga(fields); break;
    case SENT_RMC: applyRmc(fields); break;
    case SENT_GST: applyGst(fields); break;
    case SENT_ZDA: applyZda(fields); break;
    default: break;
  }
  pending_received_ |= bit;
  sentences_ever_seen_ |= bit;

  // ---- Step 3: eager flush ----
  // Only after the learning phase (first boundary observed) do we trust the
  // ever-seen set. Eager-flush also requires GGA in this epoch — a solution
  // without GGA has no position and is not worth publishing.
  if (!produced && learned_ &&
      (pending_received_ & SENT_GGA) &&
      pending_received_ == sentences_ever_seen_) {
    flushPending(solution);
    produced = true;
  }

  return produced;
}

bool NmeaParser::applyGga(const std::vector<std::string>& fields) {
  if (fields.size() < 15) return false;

  const double hms = parseDouble(fields[1]);
  pgga_hms_ = hms;
  using Sol = gnss_ros_standardization::msg::GnssSolution;
  const int status = parseInteger(fields[6]);
  switch (status) {
    case 0: pgga_status_ = Sol::STATUS_NONE;   break;
    case 1: pgga_status_ = Sol::STATUS_SINGLE; break;
    case 2: pgga_status_ = Sol::STATUS_DGPS;   break;
    case 4: pgga_status_ = Sol::STATUS_FIX;    break;
    case 5: pgga_status_ = Sol::STATUS_FLOAT;  break;
    case 6: pgga_status_ = Sol::STATUS_DR;     break;  // estimated (dead reckoning)
    default: pgga_status_ = Sol::STATUS_NONE;  break;
  }

  pgga_lat_ = parseCoordinate(fields[2], fields[3]);
  pgga_lon_ = parseCoordinate(fields[4], fields[5]);
  const double msl_alt   = parseDouble(fields[9]);
  const double geoid_sep = parseDouble(fields[11]);
  pgga_alt_      = fields[10] == "M" && fields[12] == "M" ?
                   msl_alt + geoid_sep : std::numeric_limits<double>::quiet_NaN();
  if (!std::isfinite(pgga_alt_)) pgga_alt_ = std::numeric_limits<double>::quiet_NaN();
  const int satellites = parseInteger(fields[7]);
  pgga_num_sats_ = satellites >= 0 && satellites <= 255 ? satellites : 0;
  pgga_age_diff_ = parseDouble(fields[13]);
  pgga_hdop_     = fields[8].empty()
                     ? std::numeric_limits<float>::quiet_NaN()
                     : static_cast<float>(parseDouble(fields[8]));
  // Missing coordinates can never become a valid fix at (0,0).
  if (!std::isfinite(pgga_lat_) || !std::isfinite(pgga_lon_)) pgga_status_ = Sol::STATUS_NONE;
  if (!(pgga_hdop_ >= 0)) pgga_hdop_ = std::numeric_limits<float>::quiet_NaN();
  if (!(pgga_age_diff_ >= 0)) pgga_age_diff_ = std::numeric_limits<double>::quiet_NaN();
  pgga_present_  = true;

  // Learn epoch period from successive GGA sod (used by sameEpoch tolerance).
  const double gga_sod = hmsToSecondsOfDay(hms);
  if (last_gga_sod_ >= 0.0 && gga_sod >= 0.0) {
    double dt = gga_sod - last_gga_sod_;
    if (dt < 0.0) dt += 86400.0;
    if (dt > 0.0 && dt < 10.0) epoch_period_ = dt;
  }
  last_gga_sod_ = gga_sod;
  return true;
}

bool NmeaParser::applyRmc(const std::vector<std::string>& fields) {
  if (fields.size() < 10) return false;

  const std::string& date = fields[9];
  if (date.size() == 6 && date.find_first_not_of("0123456789") == std::string::npos) {
    const int yy = parseInteger(date.substr(4, 2));
    // NMEA's two-digit year: deterministic GNSS-era window, 1980..2079.
    setDate(yy >= 80 ? 1900 + yy : 2000 + yy,
            parseInteger(date.substr(2, 2)), parseInteger(date.substr(0, 2)),
            parseDouble(fields[1]));
  }
  if (fields[2] != "A") { prmc_present_ = false; return false; }
  const double speed_kt   = parseDouble(fields[7]);
  const double course_deg = parseDouble(fields[8]);
  if (!std::isfinite(speed_kt) || !std::isfinite(course_deg) ||
      speed_kt < 0 || course_deg < 0 || course_deg >= 360) {
    prmc_present_ = false;
    return false;
  }
  const double speed_ms   = speed_kt * 0.514444;             // knots → m/s
  const double course_rad = course_deg * (M_PI / 180.0);
  prmc_vel_north_ = speed_ms * std::cos(course_rad);
  prmc_vel_east_  = speed_ms * std::sin(course_rad);
  prmc_present_   = true;
  return true;
}

bool NmeaParser::setDate(int year, int month, int day, double hms) {
  uint32_t week; double tow;
  if (!nmeaUtcToGpsTime(year, month, day, hms, week, tow)) return false;
  cached_year_ = year; cached_month_ = month; cached_day_ = day;
  cached_date_sod_ = hmsToSecondsOfDay(hms);
  has_date_cache_ = true;
  date_arrival_=input_arrival_; explicit_date_=false;
  return true;
}

bool NmeaParser::setReferenceDate(int year, int month, int day) {
  // Explicit reference for GGA-only recordings. Never substitute the host date.
  if (!setDate(year, month, day, 0)) return false;
  cached_date_sod_ = -1;
  reference_year_=year; reference_month_=month; reference_day_=day; explicit_date_=true;
  return true;
}

bool NmeaParser::applyZda(const std::vector<std::string>& fields) {
  return fields.size() >= 5 && setDate(parseInteger(fields[4]), parseInteger(fields[3]),
                                      parseInteger(fields[2]), parseDouble(fields[1]));
}

void NmeaParser::resolveGgaTime() {
  if (!has_date_cache_) return;
  const double sod = hmsToSecondsOfDay(pgga_hms_);
  if (sod < 0) return;
  double ep[6] = {double(cached_year_), double(cached_month_), double(cached_day_), 0, 0, 0};
  double shift = 0;
  if (cached_date_sod_ >= 0) {
    if (sod - cached_date_sod_ < -43200) shift = 86400;
    else if (sod - cached_date_sod_ > 43200) shift = -86400;
  }
  time2epoch(timeadd(epoch2time(ep), shift), ep);
  if (nmeaUtcToGpsTime(ep[0], ep[1], ep[2], pgga_hms_, pgga_week_, pgga_tow_)) {
    const auto age=date_arrival_; const bool explicit_date=explicit_date_;
    setDate(ep[0], ep[1], ep[2], pgga_hms_);
    date_arrival_=age; explicit_date_=explicit_date;
  }
}

bool NmeaParser::applyGsa(const std::vector<std::string>& fields) {
  // $xxGSA fields: 1=mode, 2=fix_type, 3..14=PRNs, 15=PDOP, 16=HDOP, 17=VDOP, cs
  // GSA has no timestamp — it never gates flushing (not in pending_received_).
  // Values cached persistently across resetPending() and invalidated by the
  // cycles_since_gsa_ counter inside flushPending().
  if (fields.size() < 18) return false;
  const double pdop = parseDouble(fields[15]);
  const double vdop = parseDouble(fields[17]);
  if (!std::isfinite(pdop) || !std::isfinite(vdop) || pdop <= 0.0 || vdop <= 0.0) {
    return false;
  }
  pgsa_pdop_         = static_cast<float>(pdop);
  pgsa_vdop_         = static_cast<float>(vdop);
  pgsa_present_      = true;
  cycles_since_gsa_  = 0;
  return true;
}

bool NmeaParser::applyGst(const std::vector<std::string>& fields) {
  if (fields.size() < 9) return false;
  // $xxGST,time,rms_range,std_major,std_minor,orient,std_lat,std_lon,std_alt,cs
  const double std_lat = parseDouble(fields[6]);
  const double std_lon = parseDouble(fields[7]);
  const double std_alt = parseDouble(fields[8]);
  if (!std::isfinite(std_lat) || !std::isfinite(std_lon) || !std::isfinite(std_alt) ||
      std_lat < 0 || std_lon < 0 || std_alt < 0 ||
      !std::isfinite(std_lat * std_lat) || !std::isfinite(std_lon * std_lon) ||
      !std::isfinite(std_alt * std_alt)) {
    pgst_present_ = false;
    return false;
  }
  pgst_var_lat_ = std_lat * std_lat;
  pgst_var_lon_ = std_lon * std_lon;
  pgst_var_alt_ = std_alt * std_alt;
  pgst_present_ = true;
  return true;
}

void NmeaParser::flushPending(gnss_ros_standardization::msg::GnssSolution& solution) {
  using Sol = gnss_ros_standardization::msg::GnssSolution;
  const double nan_d = std::numeric_limits<double>::quiet_NaN();
  const float  nan_f = std::numeric_limits<float>::quiet_NaN();

  solution.solution_source = Sol::SOLUTION_SOURCE_NMEA;

  // ---- Time / position from GGA ----
  if (pgga_present_) {
    resolveGgaTime();
    solution.time_week = pgga_week_;
    solution.time_tow  = pgga_tow_;
    solution.status    = pgga_status_;
    solution.latitude  = pgga_lat_;
    solution.longitude = pgga_lon_;
    solution.altitude  = pgga_alt_;
    solution.num_sats  = pgga_num_sats_;
    solution.age_diff  = pgga_age_diff_;
    solution.hdop      = pgga_hdop_;
    if (std::isfinite(pgga_lat_) && std::isfinite(pgga_lon_) && std::isfinite(pgga_alt_)) {
      double pos[3] = {pgga_lat_ * (M_PI/180.0), pgga_lon_ * (M_PI/180.0), pgga_alt_};
      double rr[3]  = {0};
      pos2ecef(pos, rr);
      solution.pos_ecef.x = rr[0];
      solution.pos_ecef.y = rr[1];
      solution.pos_ecef.z = rr[2];
    } else {
      solution.pos_ecef.x = nan_d;
      solution.pos_ecef.y = nan_d;
      solution.pos_ecef.z = nan_d;
    }
  } else {
    // No GGA this epoch — boundary-flush with only RMC/GST. Mark position
    // fields invalid; downstream sees status=NONE and NaN position.
    solution.time_week = 0;
    solution.time_tow  = 0.0;
    solution.status    = Sol::STATUS_NONE;
    solution.latitude  = nan_d;
    solution.longitude = nan_d;
    solution.altitude  = nan_d;
    solution.num_sats  = 0;
    solution.age_diff  = 0.0;
    solution.hdop      = nan_f;
    solution.pos_ecef.x = nan_d;
    solution.pos_ecef.y = nan_d;
    solution.pos_ecef.z = nan_d;
  }

  // PDOP/VDOP from cached GSA if fresh (≤1 cycle since arrival). GDOP is not
  // derivable from GSA (no tdop). HDOP keeps the value already set from GGA.
  if (pgsa_present_ && cycles_since_gsa_ <= 1) {
    solution.pdop = pgsa_pdop_;
    solution.vdop = pgsa_vdop_;
  } else {
    solution.pdop = nan_f;
    solution.vdop = nan_f;
  }
  solution.gdop = nan_f;

  // Velocity covariance has no NMEA source.
  for (int i = 0; i < 9; ++i) {
    solution.vel_enu_cov[i]  = nan_d;
    solution.vel_cov_ecef[i] = nan_d;
  }

  // ---- Velocity from RMC ----
  // ENU velocity comes directly from RMC speed/course (no rotation).
  // ECEF velocity needs GGA's lat/lon for ENU→ECEF rotation.
  if (prmc_present_ && pgga_present_ &&
      std::isfinite(pgga_lat_) && std::isfinite(pgga_lon_)) {
    solution.vel_enu.x = prmc_vel_east_;
    solution.vel_enu.y = prmc_vel_north_;
    solution.vel_enu.z = 0.0;

    double pos[3]      = {pgga_lat_ * (M_PI/180.0), pgga_lon_ * (M_PI/180.0), pgga_alt_};
    double vel_enu[3]  = {prmc_vel_east_, prmc_vel_north_, 0.0};
    double vel_ecef[3] = {0};
    enu2ecef(pos, vel_enu, vel_ecef);
    solution.vel_ecef.x = vel_ecef[0];
    solution.vel_ecef.y = vel_ecef[1];
    solution.vel_ecef.z = vel_ecef[2];
  } else {
    solution.vel_enu.x  = nan_d;
    solution.vel_enu.y  = nan_d;
    solution.vel_enu.z  = nan_d;
    solution.vel_ecef.x = nan_d;
    solution.vel_ecef.y = nan_d;
    solution.vel_ecef.z = nan_d;
  }

  // ---- Position covariance from GST ----
  if (pgst_present_ && pgga_present_ &&
      std::isfinite(pgga_lat_) && std::isfinite(pgga_lon_)) {
    for (int i = 0; i < 9; ++i) solution.pos_enu_cov[i] = 0.0;
    solution.pos_enu_cov[0] = pgst_var_lon_;  // East-East
    solution.pos_enu_cov[4] = pgst_var_lat_;  // North-North
    solution.pos_enu_cov[8] = pgst_var_alt_;  // Up-Up

    double cov_enu[9] = {0};
    cov_enu[0] = pgst_var_lon_;
    cov_enu[4] = pgst_var_lat_;
    cov_enu[8] = pgst_var_alt_;
    double cov_ecef[9] = {0};
    rotateCovarianceEnuToEcef(cov_enu, pgga_lat_ * (M_PI/180.0), pgga_lon_ * (M_PI/180.0), cov_ecef);
    for (int i = 0; i < 9; ++i) solution.pos_cov_ecef[i] = cov_ecef[i];
  } else {
    for (int i = 0; i < 9; ++i) {
      solution.pos_enu_cov[i]  = nan_d;
      solution.pos_cov_ecef[i] = nan_d;
    }
  }

  resetPending();

  // GSA staleness counter: advance after each flush. The cache survives one
  // cycle without a fresh GSA; after that, pgsa_present_ is cleared so DOP
  // goes to NaN until GSA returns.
  if (cycles_since_gsa_ < 255) ++cycles_since_gsa_;
  if (cycles_since_gsa_ > 1) pgsa_present_ = false;
}

void NmeaParser::resetPending() {
  pending_sod_       = -1.0;
  pending_received_  = 0;
  pgga_present_      = false;
  prmc_present_      = false;
  pgst_present_      = false;
  pgga_week_         = 0;
  pgga_tow_          = 0.0;
  pgga_status_       = 0;
  pgga_lat_ = pgga_lon_ = pgga_alt_ = 0.0;
  pgga_num_sats_     = 0;
  pgga_age_diff_     = 0.0;
  pgga_hdop_         = 0.0f;
  prmc_vel_east_ = prmc_vel_north_ = 0.0;
  pgst_var_lat_ = pgst_var_lon_ = pgst_var_alt_ = 0.0;
}

} // namespace gnss_utils
