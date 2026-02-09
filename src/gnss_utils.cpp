#include "gnss_utils.hpp"
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
  
  // Note: GlonassEphemeris messages typically store TOE/TOF as week+seconds relative to GPST??
  // In single_point_positioning.cpp: 
  //   g.toe = gpst2utc(gpst2time(w, m.toe));
  //   g.tof = gpst2utc(gpst2time(w, m.tof));
  //
  // Because RTKLIB's geph_t expects UTC for GLO time fields?
  // Let's verify. RTKLIB: geph.toe is Time of Ephemeris (UTC).
  // The ROS msg says 'gnss_ephemeris' has week/toe in GPST generally.
  // But wait, the existing code in single_point_positioning does: gpst2time -> gpst2utc. 
  // Does the msg store TOE in GPST? Yes, likely standardized that way in the msg def (implied).
  
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
  // geph_t stores UTC. Msg desires GPST week/sec?
  // In rtcm_decoder_node, it does:
  // m.toe  = time2gpst(utc2gpst(g.toe), &w);
  // This implies g.toe is UTC, convert to GPST, then extract week/sec.
  
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
  obs.satid  = satId(o.sat);

  if (o.code[kf]) {
    if (const char* sig = code2obs(o.code[kf]); sig) obs.code = sig;
  }

  obs.pseudorange   = o.P[kf];
  obs.carrier_phase = o.L[kf];
  obs.doppler       = o.D[kf];
  obs.snr           = static_cast<float>(o.SNR[kf]) * 0.001f; 
  obs.lli           = o.LLI[kf];

  return obs;
}

} // namespace gnss_utils
