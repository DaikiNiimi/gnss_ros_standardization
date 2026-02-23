#include "gnss_ros_standardization/gnss_utils.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>

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
  e.svh = static_cast<int>(m.svh);
  e.sva = static_cast<int>(m.sva);
  e.code = static_cast<int>(m.code);
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
  m.svh = e.svh;
  m.sva = e.sva;
  m.code = e.code;
  
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
  obs.snr           = static_cast<float>(o.SNR[kf]) * 0.001f; 
  obs.lli           = o.LLI[kf] & 0x03;

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
      T[3*i + j] = cov_enu[3*i + 0] * R[3*j + 0] +
                   cov_enu[3*i + 1] * R[3*j + 1] +
                   cov_enu[3*i + 2] * R[3*j + 2];
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
      return false; // Invalid checksum
    }
  }

  // Extract payload (between $ and *)
  std::string payload = sentence.substr(1, asterisk_pos != std::string::npos ? asterisk_pos - 1 : std::string::npos);
  std::vector<std::string> fields = splitString(payload, ',');

  if (fields.empty()) return false;

  std::string type = fields[0];
  if (type.length() < 3) return false;
  
  std::string sentence_id = type.substr(type.length() - 3);

  if (sentence_id == "GGA") {
    return parseGga(fields, solution);
  } else if (sentence_id == "RMC") {
    parseRmc(fields, solution);
    return false;
  } else if (sentence_id == "GSA") {
    parseGsa(fields, solution);
    return false;
  } else if (sentence_id == "GST") {
    parseGst(fields, solution);
    return false;
  }

  return false;
}

bool NmeaParser::parseGga(const std::vector<std::string>& fields, gnss_ros_standardization::msg::GnssSolution& solution) {
  if (fields.size() < 15) return false;

  int status = parseInteger(fields[6]);
  
  // Set ROS STATUS
  switch (status) {
    case 0: solution.status = gnss_ros_standardization::msg::GnssSolution::STATUS_NONE; break;
    case 1: solution.status = gnss_ros_standardization::msg::GnssSolution::STATUS_SINGLE; break;
    case 2: solution.status = gnss_ros_standardization::msg::GnssSolution::STATUS_DGPS; break;
    case 4: solution.status = gnss_ros_standardization::msg::GnssSolution::STATUS_FIX; break;
    case 5: solution.status = gnss_ros_standardization::msg::GnssSolution::STATUS_FLOAT; break;
    case 6: solution.status = gnss_ros_standardization::msg::GnssSolution::STATUS_NONE; break; // DR
    default: solution.status = gnss_ros_standardization::msg::GnssSolution::STATUS_NONE; break;
  }

  if (solution.status == gnss_ros_standardization::msg::GnssSolution::STATUS_NONE) {
    return true; // Still return true so the node emits the un-fixed status
  }

  // Extract lat, lon, alt
  double lat = parseCoordinate(fields[2], fields[3]);
  double lon = parseCoordinate(fields[4], fields[5]);
  double msl_alt = parseDouble(fields[9]);
  double geoid_sep = parseDouble(fields[11]);
  double ell_alt = msl_alt + geoid_sep;

  solution.latitude = lat;
  solution.longitude = lon;
  solution.altitude = ell_alt;

  // Convert to ECEF (using MALIB pos2ecef)
  double pos[3] = {lat * (M_PI/180.0), lon * (M_PI/180.0), ell_alt};
  double rr[3] = {0};
  pos2ecef(pos, rr);
  
  solution.pos_ecef.x = rr[0];
  solution.pos_ecef.y = rr[1];
  solution.pos_ecef.z = rr[2];

  // Other GGA fields
  solution.num_sats = parseInteger(fields[7]);
  last_hdop_ = parseDouble(fields[8]);
  solution.age_diff = parseDouble(fields[13]);

  // Apply buffered DOP / Velocity
  solution.hdop = last_hdop_;
  solution.pdop = last_pdop_;
  solution.vdop = last_vdop_;

  // Apply buffered Covariance (GST) or 0.0 if not available
  if (has_variance_) {
    // NMEA GST is: 6=lat(N), 7=lon(E), 8=alt(U). We buffered them as var_lat, var_lon, var_alt
    solution.pos_enu_cov[0] = var_lon_; // East-East
    solution.pos_enu_cov[4] = var_lat_; // North-North
    solution.pos_enu_cov[8] = var_alt_; // Up-Up
    
    // ENU Covariance (diagonal only) -> ECEF Covariance
    double cov_enu[9] = {0};
    cov_enu[0] = var_lon_;
    cov_enu[4] = var_lat_;
    cov_enu[8] = var_alt_;

    double cov_ecef[9] = {0};
    rotateCovarianceEnuToEcef(cov_enu, lat * (M_PI/180.0), lon * (M_PI/180.0), cov_ecef);

    for (int i = 0; i < 9; ++i) {
      solution.pos_cov_ecef[i] = cov_ecef[i];
    }
  } else {
    // Zero out covariance
    for (int i = 0; i < 9; ++i) {
      solution.pos_enu_cov[i] = 0.0;
      solution.pos_cov_ecef[i] = 0.0;
    }
  }

  if (has_velocity_) {
    // Note: Since NMEA only gives horizontal velocity component, 
    // vertical is strictly 0 and accuracy is limited. 
    // Need ENU to ECEF for vel_ecef using MALIB's enu2ecef.
    double vel_enu[3] = {vel_east_, vel_north_, 0.0};
    double vel_ecef[3] = {0};
    enu2ecef(pos, vel_enu, vel_ecef);
    
    solution.vel_ecef.x = vel_ecef[0];
    solution.vel_ecef.y = vel_ecef[1];
    solution.vel_ecef.z = vel_ecef[2];
  } else {
    solution.vel_ecef.x = 0;
    solution.vel_ecef.y = 0;
    solution.vel_ecef.z = 0;
  }

  return true;
}

bool NmeaParser::parseRmc(const std::vector<std::string>& fields, gnss_ros_standardization::msg::GnssSolution& /*solution*/) {
  if (fields.size() < 10) return false;
  
  if (fields[2] != "A") { // A = Active, V = Void
    has_velocity_ = false;
    return false;
  }

  double speed_knots = parseDouble(fields[7]);
  double true_course_deg = parseDouble(fields[8]);

  double speed_ms = speed_knots * 0.514444; // static KNOT2M from RTKLIB

  // Convert course to EN components
  // Course is clockwise from true North
  double course_rad = true_course_deg * (M_PI / 180.0);
  
  vel_north_ = speed_ms * cos(course_rad);
  vel_east_ = speed_ms * sin(course_rad);
  has_velocity_ = true;

  return true;
}

bool NmeaParser::parseGsa(const std::vector<std::string>& fields, gnss_ros_standardization::msg::GnssSolution& /*solution*/) {
  if (fields.size() < 18) return false;
  
  // NMEA specifies indices: 15=PDOP, 16=HDOP, 17=VDOP
  last_pdop_ = parseDouble(fields[15]);
  last_hdop_ = parseDouble(fields[16]);
  last_vdop_ = parseDouble(fields[17]);
  
  return true;
}

bool gnss_utils::NmeaParser::parseGst(const std::vector<std::string>& fields, gnss_ros_standardization::msg::GnssSolution& /*solution*/) {
  if (fields.size() < 9) return false;
  
  // $xxGST,time,rms_range,std_major,std_minor,orient,std_lat,std_lon,std_alt,cs
  // 6: std_lat, 7: std_lon, 8: std_alt
  double std_lat = parseDouble(fields[6]);
  double std_lon = parseDouble(fields[7]);
  double std_alt = parseDouble(fields[8]);

  var_lat_ = std_lat * std_lat;
  var_lon_ = std_lon * std_lon;
  var_alt_ = std_alt * std_alt;
  has_variance_ = true;

  return true;
}

} // namespace gnss_utils
