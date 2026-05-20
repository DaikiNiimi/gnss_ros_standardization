// SPDX-License-Identifier: MIT
#include "gnss_ros_standardization/gnss_utils.hpp"
#include <cmath>
#include <limits>
#include <cstring>
#include <ctime>
#include <algorithm>
#include <rclcpp/logging.hpp>

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

// Helper to determine satellite number from msg fields
static int satFromMsg(const std::string& satid, const std::string& sys, int prn) {
  if (!satid.empty()) {
    int s = satid2no(satid.c_str());
    if (s > 0) return s;
  }
  
  if (sys.empty()) return 0;
  
  int sys_mask = 0;
  if      (sys=="G") sys_mask = SYS_GPS;
  else if (sys=="R") sys_mask = SYS_GLO;
  else if (sys=="E") sys_mask = SYS_GAL;
  else if (sys=="J") { sys_mask = SYS_QZS; if (prn >= 193 && prn <= 202) prn -= 192; }
  else if (sys=="C") sys_mask = SYS_CMP;
  else if (sys=="I") sys_mask = SYS_IRN;
  else if (sys=="S") sys_mask = SYS_SBS;
  else return 0;

  return satno(sys_mask, prn);
}

rclcpp::Time gpstToUtcRosTime(gtime_t t_gpst) {
  const gtime_t t_utc = gpst2utc(t_gpst);
  const int64_t nsec = static_cast<int64_t>(t_utc.time) * 1000000000LL +
                       static_cast<int64_t>(t_utc.sec * 1e9);
  return rclcpp::Time(nsec);
}

bool nmeaUtcToGpsTime(int year, int month, int day, double hms,
                      uint16_t& week, double& tow) {
  if (year <= 0 || month <= 0 || day <= 0) return false;
  const int hh = static_cast<int>(hms / 10000.0);
  const int mm = static_cast<int>((hms - hh * 10000) / 100.0);
  const double ss = hms - hh * 10000 - mm * 100;
  double ep[6] = {static_cast<double>(year), static_cast<double>(month),
                  static_cast<double>(day), static_cast<double>(hh),
                  static_cast<double>(mm), ss};
  gtime_t t_utc = epoch2time(ep);
  if (t_utc.time == 0) return false;
  gtime_t t_gpst = utc2gpst(t_utc);
  int w = 0;
  const double t = time2gpst(t_gpst, &w);
  if (w <= 0) return false;
  week = static_cast<uint16_t>(w);
  tow = t;
  return true;
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
  e.sat = satFromMsg(m.satid, m.system, m.prn);
  int w = static_cast<int>(m.week);
  e.toe = gpst2time(w, m.toe);
  
  // Use adjweek for toc/ttr relative to toe
  e.toc = adjweek(e.toe, w, m.toc);
  e.ttr = adjweek(e.toe, w, m.ttr);

  // Recalculate/normalize week based on system (as per original writer)
  int sys = satsys(e.sat, nullptr);
  int final_week = 0;
  if (sys == SYS_CMP) {
      if (e.toe.time != 0) (void)time2bdt(e.toe, &final_week);
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

  e.crc = m.crc;
  e.crs = m.crs;
  e.cuc = m.cuc;
  e.cus = m.cus;
  e.cic = m.cic;
  e.cis = m.cis;
  
  e.f0 = m.f0;
  e.f1 = m.f1;
  e.f2 = m.f2;

  e.tgd[0] = (m.tgd.size() > 0 ? m.tgd[0] : 0.0);
  e.tgd[1] = (m.tgd.size() > 1 ? m.tgd[1] : 0.0);

  e.iode = static_cast<int>(m.iode);
  e.iodc = static_cast<int>(m.iodc);

  // Fix RTKLIB's synthetic pseudo-IODE for BeiDou
  // For BDS, RTKLIB generates a pseudo-iode = (toc/720) % 240 which breaks RINEX AODE.
  // The true AODE (5 bits, 0-31) is typically preserved in iodc (AODC).
  if (m.system == "C") {
      if (e.iode > 31 && e.iodc <= 31) {
          e.iode = e.iodc;
      }
  }

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
  g.sat = satFromMsg(m.satid, m.system, m.prn);
  int w = static_cast<int>(m.week);
  
  g.toe = gpst2utc(gpst2time(w, m.toe));
  g.tof = gpst2utc(gpst2time(w, m.tof));
  
  g.frq = static_cast<signed char>(m.frq);
  
  g.pos[0] = m.pos[0]; g.pos[1] = m.pos[1]; g.pos[2] = m.pos[2];
  g.vel[0] = m.vel[0]; g.vel[1] = m.vel[1]; g.vel[2] = m.vel[2];
  g.acc[0] = m.acc[0]; g.acc[1] = m.acc[1]; g.acc[2] = m.acc[2];
  
  g.iode = static_cast<int>(m.iode);
  g.svh = static_cast<int>(m.svh);
  g.age = static_cast<int>(m.age);
  
  g.gamn = m.gamn;
  g.taun = m.taun;
  g.dtaun = m.dtaun;
  
  return g;
}

gnss_ros_standardization::msg::GnssEphemeris ephToMsg(const eph_t& e) {
  gnss_ros_standardization::msg::GnssEphemeris m;

  int prn = 0;
  const int sys = satsys(e.sat, &prn);
  m.system = systemCode(sys);
  m.prn    = prn;
  m.satid  = satId(e.sat);

  int w = 0;
  m.toe  = time2gpst(e.toe, &w);
  m.week = static_cast<uint16_t>(w);
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

  m.crc = e.crc;
  m.crs = e.crs;
  m.cuc = e.cuc;
  m.cus = e.cus;
  m.cic = e.cic;
  m.cis = e.cis;

  m.f0 = e.f0;
  m.f1 = e.f1;
  m.f2 = e.f2;

  m.tgd.push_back(e.tgd[0]);
  m.tgd.push_back(e.tgd[1]);

  m.iode = e.iode;
  m.iodc = e.iodc;

  // Fix RTKLIB's synthetic pseudo-IODE for BeiDou
  // If the internal iode is a pseudo-value (>31), fallback to iodc (AODC)
  // to ensure downstream nodes and RINEX files receive the correct AODE.
  if (m.system == "C") {
      if (m.iode > 31 && m.iodc <= 31) {
          m.iode = m.iodc;
      }
  }

  m.svh = e.svh;
  m.sva = e.sva;
  m.code = e.code;
  m.flag = static_cast<uint8_t>(e.flag);
  m.fit  = static_cast<uint8_t>(e.fit);

  return m;
}

gnss_ros_standardization::msg::GlonassEphemeris gephToMsg(const geph_t& g) {
  gnss_ros_standardization::msg::GlonassEphemeris m;
  m.system = "R";

  int prn = 0; (void)satsys(g.sat, &prn);
  m.prn = prn;
  m.satid = satId(g.sat);
  m.frq = g.frq;

  int w = 0;
  
  m.toe  = time2gpst(utc2gpst(g.toe), &w);
  m.week = static_cast<uint16_t>(w);
  m.tof  = time2gpst(utc2gpst(g.tof), &w);

  m.pos = { g.pos[0], g.pos[1], g.pos[2] };
  m.vel = { g.vel[0], g.vel[1], g.vel[2] };
  m.acc = { g.acc[0], g.acc[1], g.acc[2] };

  m.iode = g.iode;
  m.svh = g.svh;
  m.age = g.age;
  m.gamn = g.gamn;
  m.taun = g.taun;
  m.dtaun = g.dtaun;
  
  return m;
}

gnss_ros_standardization::msg::GnssObservation obsToMsg(const obsd_t& o, int kf) {
  gnss_ros_standardization::msg::GnssObservation obs;
  
  int prn = 0;
  const int sys = satsys(o.sat, &prn);
  obs.system = systemCode(sys);
  obs.prn    = prn;
  obs.sat    = o.sat;
  obs.satid  = satId(o.sat);

  if (o.code[kf]) {
    obs.code = o.code[kf];
    if (const char* sig = code2obs(o.code[kf]); sig) obs.code_str = sig;
  }

  obs.p             = o.P[kf];
  obs.l             = o.L[kf];
  obs.d             = o.D[kf];
  obs.snr           = static_cast<float>(o.SNR[kf]);
  obs.lli           = o.LLI[kf] & 0x07;   // bit0 SLIP | bit1 HALFC | bit2 BOCTRK

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
                           uint16_t pvt_week,
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
  if (str.empty()) return 0.0;
  try {
    return std::stod(str);
  } catch (...) {
    return 0.0;
  }
}

int NmeaParser::parseInteger(const std::string& str) {
  if (str.empty()) return 0;
  try {
    return std::stoi(str);
  } catch (...) {
    return 0;
  }
}

// coordinate: ddmm.mmmmm
// hemisphere: N/S or E/W
double NmeaParser::parseCoordinate(const std::string& coord_str, const std::string& hem) {
  if (coord_str.empty() || hem.empty()) return 0.0;

  double dot_pos = coord_str.find('.');
  if (dot_pos == std::string::npos || dot_pos < 2) return 0.0;

  std::string deg_str = coord_str.substr(0, dot_pos - 2);
  std::string min_str = coord_str.substr(dot_pos - 2);

  double deg = parseDouble(deg_str);
  double min = parseDouble(min_str);
  double decimal_deg = deg + (min / 60.0);

  if (hem == "S" || hem == "W") {
    decimal_deg = -decimal_deg;
  }
  return decimal_deg;
}

namespace {

// hhmmss.ss (e.g. 123459.50) → seconds-of-day.
// Returns -1.0 sentinel for non-finite, negative, or out-of-range inputs.
double hmsToSecondsOfDay(double hms) {
  if (!std::isfinite(hms) || hms < 0.0) return -1.0;
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

bool NmeaParser::parseSentence(const std::string& sentence, gnss_ros_standardization::msg::GnssSolution& solution) {
  if (sentence.empty() || sentence[0] != '$') return false;

  // Verify checksum
  size_t asterisk_pos = sentence.find('*');
  if (asterisk_pos != std::string::npos && asterisk_pos + 2 < sentence.length()) {
    int checksum = 0;
    for (size_t i = 1; i < asterisk_pos; ++i) {
      checksum ^= sentence[i];
    }
    std::string provided_checksum_str = sentence.substr(asterisk_pos + 1, 2);
    int provided_checksum = 0;
    try {
      provided_checksum = std::stoi(provided_checksum_str, nullptr, 16);
    } catch (...) {}

    if (checksum != provided_checksum) {
      RCLCPP_DEBUG(rclcpp::get_logger("nmea_parser"),
        "NMEA checksum mismatch: got %02X want %02X head='%s'",
        provided_checksum, checksum,
        sentence.substr(0, std::min<size_t>(sentence.size(), 12)).c_str());
      return false;
    }
  }

  std::string payload = sentence.substr(1, asterisk_pos != std::string::npos ? asterisk_pos - 1 : std::string::npos);
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

  // Time-of-day → GPS week/tow using cached date (or system UTC fallback).
  const double hms = parseDouble(fields[1]);
  if (hms > 0.0) {
    int year = cached_year_, month = cached_month_, day = cached_day_;
    if (!has_date_cache_) {
      const std::time_t now_t = std::time(nullptr);
      std::tm tm_utc{};
      gmtime_r(&now_t, &tm_utc);
      year  = tm_utc.tm_year + 1900;
      month = tm_utc.tm_mon + 1;
      day   = tm_utc.tm_mday;
      RCLCPP_WARN_ONCE(rclcpp::get_logger("nmea_parser"),
        "NMEA date unavailable (no RMC seen); falling back to system UTC date for GPSTime assembly");
    } else {
      // Day-rollover guard: cached date may be one day stale until the next RMC.
      const double cached_hms = cached_last_hms_;
      if (cached_hms > 0.0 && (cached_hms - hms) > 12.0 * 3600.0) {
        std::tm tm_in{};
        tm_in.tm_year = year - 1900;
        tm_in.tm_mon  = month - 1;
        tm_in.tm_mday = day + 1;
        std::mktime(&tm_in);
        year  = tm_in.tm_year + 1900;
        month = tm_in.tm_mon + 1;
        day   = tm_in.tm_mday;
      }
    }
    cached_last_hms_ = hms;
    uint16_t week = 0;
    double   tow  = 0.0;
    if (nmeaUtcToGpsTime(year, month, day, hms, week, tow)) {
      pgga_week_ = week;
      pgga_tow_  = tow;
    }
  }

  using Sol = gnss_ros_standardization::msg::GnssSolution;
  const int status = parseInteger(fields[6]);
  switch (status) {
    case 0: pgga_status_ = Sol::STATUS_NONE;   break;
    case 1: pgga_status_ = Sol::STATUS_SINGLE; break;
    case 2: pgga_status_ = Sol::STATUS_DGPS;   break;
    case 4: pgga_status_ = Sol::STATUS_FIX;    break;
    case 5: pgga_status_ = Sol::STATUS_FLOAT;  break;
    case 6: pgga_status_ = Sol::STATUS_NONE;   break;  // DR
    default: pgga_status_ = Sol::STATUS_NONE;  break;
  }

  pgga_lat_ = parseCoordinate(fields[2], fields[3]);
  pgga_lon_ = parseCoordinate(fields[4], fields[5]);
  const double msl_alt   = parseDouble(fields[9]);
  const double geoid_sep = parseDouble(fields[11]);
  pgga_alt_      = msl_alt + geoid_sep;
  pgga_num_sats_ = static_cast<uint8_t>(parseInteger(fields[7]));
  pgga_age_diff_ = parseDouble(fields[13]);
  pgga_hdop_     = static_cast<float>(parseDouble(fields[8]));
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

  // Cache UTC date from RMC field[9] = ddmmyy (independent of velocity branch).
  const std::string& date_str = fields[9];
  if (date_str.size() == 6) {
    int dd = parseInteger(date_str.substr(0, 2));
    int mo = parseInteger(date_str.substr(2, 2));
    int yy = parseInteger(date_str.substr(4, 2));
    if (dd >= 1 && dd <= 31 && mo >= 1 && mo <= 12) {
      cached_year_   = 2000 + yy;
      cached_month_  = mo;
      cached_day_    = dd;
      has_date_cache_ = true;
    }
  }

  // Accept velocity regardless of the A/V flag. The flag indicates nav-validity,
  // not whether the speed/course values exist — many receivers report V even
  // when valid speed/course are present (e.g. fixed-position mode). Downstream
  // consumers can gate on solution.status (driven by GGA quality) instead.
  const double speed_kt   = parseDouble(fields[7]);
  const double course_deg = parseDouble(fields[8]);
  if (!std::isfinite(speed_kt) || !std::isfinite(course_deg)) {
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
    solution.time_week = pgga_week_;
    solution.time_tow  = pgga_tow_;
    solution.status    = pgga_status_;
    solution.latitude  = pgga_lat_;
    solution.longitude = pgga_lon_;
    solution.altitude  = pgga_alt_;
    solution.num_sats  = pgga_num_sats_;
    solution.age_diff  = pgga_age_diff_;
    solution.hdop      = pgga_hdop_;
    if (!(pgga_lat_ == 0.0 && pgga_lon_ == 0.0)) {
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
    solution.latitude  = 0.0;
    solution.longitude = 0.0;
    solution.altitude  = 0.0;
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
      !(pgga_lat_ == 0.0 && pgga_lon_ == 0.0)) {
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
      !(pgga_lat_ == 0.0 && pgga_lon_ == 0.0)) {
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