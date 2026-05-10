/*
MIT License

Copyright (c) 2025 …

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the “Software”), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

(license text continues…)
*/

#include <rclcpp/rclcpp.hpp>

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <map>
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>
#include <queue>
#include <cstdint>
#include <filesystem>

#include "gnss_ros_standardization/gnss_utils.hpp"

using gnss_ros_standardization::msg::GnssObservation;
using gnss_ros_standardization::msg::GnssObservations;
using gnss_ros_standardization::msg::GnssEphemeris;
using gnss_ros_standardization::msg::GlonassEphemeris;
using gnss_ros_standardization::msg::GnssEphemerides;


/*============================== Utilities ==============================*/

static inline std::string normalizeBagUri(std::string uri) {
  while (!uri.empty() && (uri.back()=='/' || uri.back()=='\\')) uri.pop_back();
  if (uri.size() >= 4) {
    std::string tail = uri.substr(uri.size()-4);
    for (auto &c: tail) c = (char)std::tolower((unsigned char)c);
    if (tail == ".db3") {
      auto pos = uri.find_last_of("/\\");
      if (pos != std::string::npos) {
        std::string dir = uri.substr(0, pos);
        return dir.empty() ? "." : dir;
      }
    }
  }
  return uri;
}

static inline int maskFromSystemString(const std::string &s) {
  int m = 0;
  for (char c : s) {
    if (c == 'G') m |= SYS_GPS;
    else if (c == 'R') m |= SYS_GLO;
    else if (c == 'E') m |= SYS_GAL;
    else if (c == 'J') m |= SYS_QZS;
    else if (c == 'C') m |= SYS_CMP;
    else if (c == 'I') m |= SYS_IRN;
    else if (c == 'S') m |= SYS_SBS;
  }
  return m;
}

static inline int satFromSysPrn(const std::string &sys, int prn_in) {
  int sys_mask = 0;
  int prn = prn_in;

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

static inline int satFromFields(const std::string& satid,
                                const std::string& sys, int prn) {
  if (!satid.empty()) {
    int s = satid2no(satid.c_str());
    if (s > 0) return s;
  }
  return satFromSysPrn(sys, prn);
}

static inline int filterByBuildFlags(int mask) {
#ifndef ENAGLO
  mask &= ~SYS_GLO;
#endif
#ifndef ENAGAL
  mask &= ~SYS_GAL;
#endif
#ifndef ENACMP
  mask &= ~SYS_CMP;
#endif
#ifndef ENAQZS
  mask &= ~SYS_QZS;
#endif
#ifndef ENAIRN
  mask &= ~SYS_IRN;
#endif
#ifndef ENASBS
  mask &= ~SYS_SBS;
#endif
  return mask;
}

// base RF slot by first digit
static inline int baseSlotFromSigChar(char c) {
  switch (c) {
    case '1': return 0;
    case '2': return 1;
    case '5': return 2;
    case '7': return 2; // E5b as L5-family
    case '9': return 2; // E5  as L5-family
    case '6': return 3;
    case '8': return 4;
    default : return 0;
  }
}

// choose (or reuse) an obsd_t slot for a code on the same freq
static inline int chooseObsSlot(obsd_t& d, int base, int code_id) {
  if (base < 0) base = 0;
  if (base >= NFREQ) base = NFREQ - 1;
  if (d.code[base] == 0 || d.code[base] == code_id) return base;
  for (int s = NFREQ; s < NFREQ + NEXOBS; ++s) {
    if (d.code[s] == 0 || d.code[s] == code_id) return s;
  }
  return NFREQ + NEXOBS - 1;
}

// "1C","2W"... -> RTKLIB code id
static inline int codeIdFromSig2(const std::string &sig) {
  if (sig.empty()) return 0;
  char tmp[8]; std::snprintf(tmp, sizeof(tmp), "%s", sig.c_str());
  return obs2code(tmp);
}

/*============================== Models ==============================*/

struct Options {
  std::string bag_uri;
  std::string topic_obs = "/gnss/observation";
  std::string topic_nav = "/gnss/ephemeris";
  std::string out_obs_path = "";
  std::string out_nav_path = "";
  double rinex_version = 3.04;
  std::string nav_systems = "GREJCI";
  bool flush_immediately = true;
  std::string program_name = "rosbag_to_rinex";
  std::string run_by = "user";
};

struct TimeSpan { bool init=false; gtime_t first{}, last{}; };

struct ObsTypeSet {
  std::map<char, std::unordered_set<std::string>> types; // "C1C", ...
};

struct GloFcnInfo {
  bool any = false;
  int8_t fcn_plus8[MAXPRNGLO] = {0};
};

/*============================== Bag Reader ==============================*/

class BagReader {
public:
  explicit BagReader(const std::string &raw) {
    rosbag2_storage::StorageOptions sopt;
    sopt.uri = normalizeBagUri(raw);
    sopt.storage_id = "sqlite3";
    rosbag2_cpp::ConverterOptions copt;
    copt.input_serialization_format  = "";
    copt.output_serialization_format = "";
    reader_.open(sopt, copt);
  }
  bool has_next() { try { return reader_.has_next(); } catch (...) { return false; } }
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> read_next() {
    try { return reader_.read_next(); } catch (...) { return nullptr; }
  }
private:
  rosbag2_cpp::Reader reader_;
};

template<typename ROSMsg>
static inline bool deserializeRos(const rosbag2_storage::SerializedBagMessage &in, ROSMsg &out) {
  if (!in.serialized_data || in.serialized_data->buffer_length==0) return false;
  try {
    rclcpp::SerializedMessage smsg(*in.serialized_data);
    rclcpp::Serialization<ROSMsg> ser;
    ser.deserialize_message(&smsg, &out);
    return true;
  } catch (...) { return false; }
}

/*============================== Pass1: scan types & timespan ==============================*/

static inline void addObsType(ObsTypeSet &set, char sys, const std::string &prefixed) {
  if (prefixed.size()>=3) set.types[sys].insert(prefixed.substr(0,3));
}

static inline bool allowFreqDigit(char sys, char d){
  switch(sys){
    case 'G': return d=='1'||d=='2'||d=='5';
    case 'R': return d=='1'||d=='2';
    case 'E': return d=='1'||d=='5'||d=='6'||d=='7'||d=='9';
    case 'J': return d=='1'||d=='2'||d=='5'||d=='6';
    case 'S': return d=='1'||d=='5'||d=='6';
    case 'C': return d=='1'||d=='2'||d=='5'||d=='6'||d=='7'||d=='8';
    case 'I': return d=='5'||d=='9'; // L5, S
    default : return false;
  }
}

static inline int sysIdx(char sys){
  switch(sys){
    case 'G': return 0; case 'R': return 1; case 'E': return 2;
    case 'J': return 3; case 'S': return 4; case 'C': return 5;
    case 'I': return 6;
    default: return -1;
  }
}
static inline int sysMaskByIdx(int idx){
  switch(idx){
    case 0: return SYS_GPS; case 1: return SYS_GLO; case 2: return SYS_GAL;
    case 3: return SYS_QZS; case 4: return SYS_SBS; case 5: return SYS_CMP;
    case 6: return SYS_IRN;
    default: return 0;
  }
}
static inline char sysCharByIdx(int idx){
  switch(idx){
    case 0: return 'G'; case 1: return 'R'; case 2: return 'E';
    case 3: return 'J'; case 4: return 'S'; case 5: return 'C';
    case 6: return 'I';
    default: return '?';
  }
}

static inline int freqRank(char sys_ch, char digit) {
  const int big = 100;
  switch (sys_ch) {
    case 'G': return digit=='1'?0: digit=='2'?1: digit=='5'?2: big;
    case 'R': return digit=='1'?0: digit=='2'?1: big;
    case 'E': return digit=='1'?0: digit=='7'?1: digit=='5'?2: digit=='6'?3: digit=='9'?4: big;
    case 'J': return digit=='1'?0: digit=='2'?1: digit=='5'?2: digit=='6'?3: big;
    case 'S': return digit=='1'?0: digit=='5'?1: digit=='6'?2: big;
    case 'C': return digit=='2'?0: digit=='7'?1: digit=='6'?2: digit=='1'?3: digit=='5'?4: digit=='8'?5: big;
    case 'I': return digit=='5'?0: digit=='9'?1: big;
    default:  return big;
  }
}
static inline int typeRank(char t){
  switch(t){ case 'C': return 0; case 'L': return 1; case 'D': return 2; case 'S': return 3; default: return 9; }
}

static inline void orderTobs(rnxopt_t& o) {
  for (int i = 0; i < 7; ++i) {
    const int n = o.nobs[i];
    if (n <= 1) continue;

    const int sys_mask = sysMaskByIdx(i);
    const char sys_ch  = sysCharByIdx(i);
    if (!sys_mask || sys_ch=='?') continue;

    struct Item { std::string s; int fr; int pri; int tr; };
    std::vector<Item> v; v.reserve(n);

    for (int k = 0; k < n; ++k) {
      const char* t = o.tobs[i][k];
      if (!t || !t[0] || !t[1] || !t[2]) continue;
      char kind = t[0], fdig = t[1], cls = t[2];

      char sig2[3] = { fdig, cls, 0 };
      int  code_id = obs2code(sig2);
      int  pri     = (code_id>0) ? getcodepri(sys_mask, (uint8_t)code_id, nullptr) : 0;

      v.push_back(Item{std::string(t, t+3), freqRank(sys_ch, fdig), pri, typeRank(kind)});
    }

    std::stable_sort(v.begin(), v.end(), [](const Item& a, const Item& b){
      if (a.fr  != b.fr ) return a.fr  < b.fr;
      if (a.pri != b.pri) return a.pri > b.pri;   // higher first
      if (a.tr  != b.tr ) return a.tr  < b.tr;    // C->L->D->S
      return a.s < b.s;
    });

    int w = 0;
    for (const auto &it : v) {
      std::snprintf(o.tobs[i][w], sizeof(o.tobs[i][w]), "%.3s", it.s.c_str());
      if (++w >= MAXOBSTYPE) break;
    }
    for (int j = w; j < MAXOBSTYPE; ++j) o.tobs[i][j][0] = '\0';
    o.nobs[i] = w;
  }
}

class Scanner {
public:
  explicit Scanner(const Options& opt): opt_(opt) {}
  std::tuple<ObsTypeSet, TimeSpan, GloFcnInfo> run() {
    ObsTypeSet types; 
    TimeSpan span;
    GloFcnInfo glo;
    BagReader reader(opt_.bag_uri);

    while (reader.has_next()) {
      auto msg = reader.read_next(); 
      if (!msg) break;

      if (msg->topic_name == opt_.topic_obs) {
        GnssObservations obs;
        if (!deserializeRos(*msg, obs)) continue;

        gtime_t t = gpst2time((int)obs.week, obs.tow);
        if (!span.init) { span.init=true; span.first=t; span.last=t; }
        else {
          if (timediff(t, span.first)<0) span.first=t;
          if (timediff(t, span.last)>0)  span.last=t;
        }

        for (const auto &o : obs.observations) {
          if (o.system.size()!=1) continue;
          char sys = o.system[0];
          if (o.p != 0.0) addObsType(types, sys, "C"+o.code_str);
          if (o.l != 0.0) addObsType(types, sys, "L"+o.code_str);
          if (o.d != 0.0) addObsType(types, sys, "D"+o.code_str);
          if (o.snr >  0.0) addObsType(types, sys, "S"+o.code_str);
        }
      }
      else if (msg->topic_name == opt_.topic_nav) {
        GnssEphemerides navs;
        if (!deserializeRos(*msg, navs)) continue;
        for (const auto& ge : navs.glonass_ephemeris) {
          if (ge.prn >= 1 && ge.prn <= MAXPRNGLO) {
            int val = (int)ge.frq + 8;
            if (val >= 1 && val <= 15) {
              glo.fcn_plus8[ge.prn - 1] = (int8_t)val;
              glo.any = true;
            }
          }
        }
      }
    }
    return {types, span, glo};
  }
private:
  Options opt_;
};

static inline void fillRnxTobs(rnxopt_t &opt, const ObsTypeSet &set) {
  std::memset(opt.nobs, 0, sizeof(opt.nobs));
  for (int i=0;i<7;i++) for (int j=0;j<MAXOBSTYPE;j++) opt.tobs[i][j][0]='\0';

  int eff_mask = 0;

  for (const auto &kv : set.types) {
    const char sys = kv.first;
    const int idx = sysIdx(sys);
    if (idx < 0) continue;

    int w = 0;
    for (const auto &name : kv.second) {
      if (name.size()<3) continue;
      if (!allowFreqDigit(sys, name[1])) continue;
      if (w >= MAXOBSTYPE) break;
      std::snprintf(opt.tobs[idx][w], sizeof(opt.tobs[idx][w]), "%.3s", name.c_str());
      ++w;
    }
    opt.nobs[idx] = w;
    if (w>0 && opt.tobs[idx][0][0]) eff_mask |= sysMaskByIdx(idx);
  }

  opt.navsys = eff_mask;
  orderTobs(opt);
}

static inline void sanitizeTobs(rnxopt_t& o){
  for (int i=0;i<7;i++){
    if (o.nobs[i] <= 0) continue;
    int w = 0;
    for (int j=0;j<o.nobs[i]; j++){
      const char* s = o.tobs[i][j];
      if (!s || !s[0]) continue;
      char sig[3] = { s[1], s[2] };
      if (obs2code(sig) == 0) continue;
      if (w!=j) {
        char tmp_buf[4];
        std::snprintf(tmp_buf, sizeof(tmp_buf), "%.3s", s);
        std::memcpy(o.tobs[i][w], tmp_buf, 4);
      }
      ++w;
    }
    for (int j=w;j<MAXOBSTYPE;j++) o.tobs[i][j][0] = '\0';
    o.nobs[i] = w;
  }
}

/*============================== RINEX Option Builders ==============================*/

static inline rnxopt_t makeObsRnxOpt(double ver, int navsys, const std::string& pgm, const std::string& runby) {
  rnxopt_t o; std::memset(&o, 0, sizeof(o));
  o.rnxver = static_cast<double>(std::lround(ver * 100.0));
  o.navsys = filterByBuildFlags(navsys);
  std::snprintf(o.prog,  sizeof(o.prog),  "%s", pgm.c_str());
  std::snprintf(o.runby, sizeof(o.runby), "%s", runby.c_str());
  for (int i=0;i<7;i++) o.mask[i][0] = '\0';
  return o;
}
static inline rnxopt_t makeNavRnxOpt(double ver, int navsys, const std::string& pgm, const std::string& runby) {
  rnxopt_t o; std::memset(&o, 0, sizeof(o));
  o.rnxver = static_cast<double>(std::lround(ver * 100.0));
  o.navsys = filterByBuildFlags(navsys);
  std::snprintf(o.prog,  sizeof(o.prog),  "%s", pgm.c_str());
  std::snprintf(o.runby, sizeof(o.runby), "%s", runby.c_str());
  for (int i=0;i<7;i++) o.mask[i][0] = '\0';
  return o;
}

static inline gtime_t adjweek(gtime_t ref, int week, double tow_sec) {
  gtime_t t = gpst2time(week, tow_sec);
  double dt = timediff(t, ref);
  if (dt < -302400.0) t = timeadd(t, 604800.0);
  else if (dt > 302400.0) t = timeadd(t, -604800.0);
  return t;
}

/*============================== Writers ==============================*/

class NavWriter {
public:
  NavWriter(const Options &opt, FILE *fp)
    : opt_(opt), fp_(fp),
      rnx_(makeNavRnxOpt(opt.rinex_version, maskFromSystemString(opt.nav_systems),
                         opt.program_name, opt.run_by)) {
    std::memset(&nav_dummy_, 0, sizeof(nav_dummy_));
    if (outrnxnavh(fp_, &rnx_, &nav_dummy_) == 0) throw std::runtime_error("outrnxnavh failed");
    std::fflush(fp_);
  }

  void onEphemerides(const GnssEphemerides &batch) {
    auto sys_of_sat = [](int sat)->int{ int prn=0; return satsys(sat, &prn); };

    for (const auto &m : batch.gnss_ephemeris) {
      eph_t e = gnss_utils::msgToEph(m);
      if (!e.sat) continue;

      int sys = sys_of_sat(e.sat);
      if ((sys & rnx_.navsys) == 0) continue;
      
      // Override logic for RINEX writer specifics (flags etc)
      e.flag = 0;
      if (sys == SYS_QZS) {
        if (e.code == 0) e.code = 2;
        e.flag = 1;
      }
      else if (sys == SYS_GPS) {
        if      (e.code == 1) e.flag = 0;
        else if (e.code == 2) e.flag = 1;
      }

      KeyK key{e.sat, e.iode, e.iodc, e.code};
      if (seen_k_.insert(key).second) {
        outrnxnavb(fp_, &rnx_, &e);
      }
    }

    if (rnx_.navsys & SYS_GLO) {
      for (const auto &m : batch.glonass_ephemeris) {
        geph_t g = gnss_utils::msgToGeph(m);
        // Note: msgToGeph already converts TOF/TOE to UTC if we assumed so, but wait.
        // In original ros2_rinex_writer.cpp:
        // g.toe = gpst2utc(gpst2time((int)m.week, m.toe));
        // g.tof = gpst2utc(gpst2time((int)m.week, m.tof));
        //
        // msgToGeph in gnss_utils does exactly this:
        // g.toe = gpst2utc(gpst2time(w, m.toe));
        // So we can use it directly.

        if (g.toe.time==0 || g.tof.time==0) continue;

        KeyR key{g.sat,g.iode};
        if (seen_r_.insert(key).second) outrnxgnavb(fp_, &rnx_, &g);
      }
    }

    if (opt_.flush_immediately) std::fflush(fp_);
  }

private:
  struct KeyK {
    int sat; int iode; int iodc; int code;
    bool operator==(const KeyK& o) const {
      return sat==o.sat && iode==o.iode && iodc==o.iodc && code==o.code;
    }
  };
  struct KeyKHash {
    size_t operator()(const KeyK& k) const {
      return (size_t)k.sat ^ ((size_t)k.iode<<16) ^ ((size_t)k.iodc<<1) ^ ((size_t)k.code<<24);
    }
  };
  struct KeyR { int sat; int iode; bool operator==(const KeyR&o)const{return sat==o.sat&&iode==o.iode;} };
  struct KeyRHash { size_t operator()(const KeyR&k) const { return (size_t)k.sat ^ ((size_t)k.iode<<16); } };

  Options opt_;
  FILE *fp_{nullptr};
  rnxopt_t rnx_{};
  nav_t nav_dummy_{};
  std::unordered_set<KeyK, KeyKHash> seen_k_;
  std::unordered_set<KeyR, KeyRHash> seen_r_;
};

class ObsWriter {
  public:
    ObsWriter(const Options &opt, FILE *fp, 
              const ObsTypeSet &types, const TimeSpan* span,
              const GloFcnInfo* glo_fcn_info)
      : opt_(opt), fp_(fp),
        rnx_(makeObsRnxOpt(opt.rinex_version, maskFromSystemString(opt.nav_systems),
                           opt.program_name, opt.run_by)) {
  
      fillRnxTobs(rnx_, types);
  
      const int user_mask = filterByBuildFlags(maskFromSystemString(opt.nav_systems));
      for (int i=0;i<7;i++){
        if ((sysMaskByIdx(i) & user_mask) == 0) {
          rnx_.nobs[i]=0;
          for (int j=0;j<MAXOBSTYPE;j++) rnx_.tobs[i][j][0]='\0';
        }
      }
      sanitizeTobs(rnx_);
      orderTobs(rnx_);
  
      int eff=0;
      for (int i=0;i<7;i++) if (rnx_.nobs[i]>0 && rnx_.tobs[i][0][0]) eff |= sysMaskByIdx(i);
      rnx_.navsys = eff & user_mask;
  
      if (span && span->init) { rnx_.tstart=span->first; rnx_.tend=span->last; }
  
      std::memset(&nav_dummy_, 0, sizeof(nav_dummy_));
      if ((rnx_.navsys & SYS_GLO) && glo_fcn_info && glo_fcn_info->any) {
        nav_dummy_.geph = (geph_t*)std::calloc(MAXPRNGLO, sizeof(geph_t));
        for (int i = 0; i < MAXPRNGLO; ++i) if (glo_fcn_info->fcn_plus8[i] != 0) nav_dummy_.glo_fcn[i] = glo_fcn_info->fcn_plus8[i];
      }
  
      if (outrnxobsh(fp_, &rnx_, &nav_dummy_) == 0) throw std::runtime_error("outrnxobsh failed");
      std::fflush(fp_);
    }
  
    ~ObsWriter() {
      flushAll();
    }

    void writeEpoch(const GnssObservations &m) {
      gtime_t t = gpst2time((int)m.week, m.tow);
      if (!have_last_ || timediff(t, last_seen_) > 0.0) { last_seen_ = t; have_last_ = true; }
      queue_.push(Epoch{t, seq_++, m});
  
      gtime_t threshold = timeadd(last_seen_, -reorder_window_sec_);
      while (!queue_.empty()) {
        const auto &e = queue_.top();
        if (timediff(e.t, threshold) <= 0.0) { writeOne(e.msg, e.t); queue_.pop(); }
        else break;
      }
    }

  private:
    struct Epoch {
      gtime_t t{};
      uint64_t seq{0};
      GnssObservations msg;
    };
    struct Cmp {
      bool operator()(const Epoch& a, const Epoch& b) const {
        double dt = timediff(a.t, b.t);
        if (std::fabs(dt) > 1e-9) return dt > 0;
        return a.seq > b.seq; 
      }
    };
  
    void writeOne(const GnssObservations &m, gtime_t t) {
      auto blankForSat = [&](int sat)->obsd_t{
        obsd_t d{}; d.time=t; d.sat=sat; d.rcv=0;
        for (int k=0;k<NFREQ+NEXOBS;k++){ d.code[k]=0; d.P[k]=0; d.L[k]=0; d.D[k]=0; d.SNR[k]=0; d.LLI[k]=0; }
        return d;
      };

  
      std::unordered_map<int, obsd_t> satmap;
      for (const auto &o : m.observations) {
        int sat = satFromFields(o.satid, o.system, o.prn);
        if (!sat) continue;
  
        auto it = satmap.find(sat);
        if (it == satmap.end()) it = satmap.emplace(sat, blankForSat(sat)).first;
  
        int base    = baseSlotFromSigChar(o.code_str.empty() ? '\0' : o.code_str[0]);
        int code_id = codeIdFromSig2(o.code_str);
        if (code_id == 0) continue;
  
        int k = chooseObsSlot(it->second, base, code_id);
        it->second.code[k] = code_id;
  
        if (o.p   != 0.0) it->second.P[k]   = o.p;
        if (o.l   != 0.0) it->second.L[k]   = o.l;
        if (o.d   != 0.0) it->second.D[k]   = o.d;
        if (o.snr >  0.0) it->second.SNR[k] = o.snr * 1000.0f;
        it->second.LLI[k] = static_cast<unsigned char>(o.lli);
  
        it->second.time = t;
      }
  
      if (satmap.empty()) return;
  
      std::vector<obsd_t> epoch;
      epoch.reserve(satmap.size());
      for (auto &kv : satmap) epoch.push_back(kv.second);
      std::sort(epoch.begin(), epoch.end(), [](const obsd_t&a, const obsd_t&b){ return a.sat < b.sat; });
  
      outrnxobsb(fp_, &rnx_, epoch.data(), (int)epoch.size(), 0);
      if (opt_.flush_immediately) std::fflush(fp_);
    }
  
    void flushAll() {
      while (!queue_.empty()) { auto e = queue_.top(); queue_.pop(); writeOne(e.msg, e.t); }
      std::fflush(fp_);
    }
  
  private:
    Options opt_;
    FILE *fp_{nullptr};
    rnxopt_t rnx_{};
    nav_t nav_dummy_{};
  
    const double reorder_window_sec_ = 3.0;
    bool have_last_ = false;
    gtime_t last_seen_{};
    uint64_t seq_ = 0;
    std::priority_queue<Epoch, std::vector<Epoch>, Cmp> queue_;
  };  

/*============================== App ==============================*/

class App {
public:
  static int run(int argc, char **argv) {
    rclcpp::init(argc, argv);
    Options opt = parseArgs(argc, argv);
    if (opt.bag_uri.empty()) {
      std::fprintf(stderr, "Usage: rosbag_to_rinex --bag <bag_dir_or_db3> "
                           "[--obs OBS_PATH --nav NAV_PATH "
                           "--topic-obs TOPIC --topic-nav TOPIC "
                           "--rnx-version X.YY --nav-systems GREJCI "
                           "--no-flush --pgm NAME --runby NAME]\n");
      return 2;
    }

    if (opt.out_obs_path.empty() && opt.out_nav_path.empty()) {
      // Auto-derive output paths from bag URI stem
      namespace fs = std::filesystem;
      fs::path bag_path(opt.bag_uri);
      // Strip trailing slashes (bag URI may be a directory)
      while (!bag_path.empty() && !bag_path.has_filename()) bag_path = bag_path.parent_path();
      std::string stem = bag_path.stem().string();
      if (stem.empty()) stem = "output";
      std::string base_dir = bag_path.parent_path().string();
      if (!base_dir.empty()) base_dir += "/";
      opt.out_obs_path = base_dir + stem + ".obs";
      opt.out_nav_path = base_dir + stem + ".nav";
      std::fprintf(stderr, "Info: output paths auto-derived: %s, %s\n",
                   opt.out_obs_path.c_str(), opt.out_nav_path.c_str());
    }

    Scanner scanner(opt);
    auto [types, span, glo] = scanner.run();

    FILE *fp_obs = nullptr;
    FILE *fp_nav = nullptr;
    std::unique_ptr<ObsWriter> obs_writer;
    std::unique_ptr<NavWriter> nav_writer;

    if (!opt.out_obs_path.empty()) {
      fp_obs = std::fopen(opt.out_obs_path.c_str(), "w");
      if (!fp_obs) { std::perror(opt.out_obs_path.c_str()); throw std::runtime_error("cannot open obs"); }
      setvbuf(fp_obs, nullptr, _IOFBF, 1<<20);
      obs_writer = std::make_unique<ObsWriter>(opt, fp_obs, types, &span, &glo);
    }

    if (!opt.out_nav_path.empty()) {
      fp_nav = std::fopen(opt.out_nav_path.c_str(), "w");
      if (!fp_nav) { std::perror(opt.out_nav_path.c_str()); throw std::runtime_error("cannot open nav"); }
      setvbuf(fp_nav, nullptr, _IOFBF, 1<<20);
      nav_writer = std::make_unique<NavWriter>(opt, fp_nav);
    }

    BagReader reader(opt.bag_uri);
    while (reader.has_next()) {
      auto msg = reader.read_next(); if (!msg) break;

      if (obs_writer && msg->topic_name == opt.topic_obs) {
        GnssObservations o; if (deserializeRos(*msg, o)) obs_writer->writeEpoch(o);
      } else if (nav_writer && msg->topic_name == opt.topic_nav) {
        GnssEphemerides n; if (deserializeRos(*msg, n)) nav_writer->onEphemerides(n);
      }
    }

    // Destroy writers to ensure flushAll() is called while files are open
    obs_writer.reset();
    nav_writer.reset();

    if (fp_obs) { std::fflush(fp_obs); std::fclose(fp_obs); }
    if (fp_nav) { std::fflush(fp_nav); std::fclose(fp_nav); }
    rclcpp::shutdown();
    return 0;
  }

private:
  static Options parseArgs(int argc, char **argv) {
    Options o;
    for (int i=1;i<argc;i++) {
      std::string a=argv[i];
      auto next=[&](){ if(i+1<argc) return std::string(argv[++i]); throw std::runtime_error("missing value for "+a); };
      if (a=="--bag") o.bag_uri = next();
      else if (a=="--obs") o.out_obs_path = next();
      else if (a=="--nav") o.out_nav_path = next();
      else if (a=="--topic-obs") o.topic_obs = next();
      else if (a=="--topic-nav") o.topic_nav = next();
      else if (a=="--rnx-version") {
        o.rinex_version = std::stod(next());
        const double v = std::lround(o.rinex_version * 100.0) / 100.0;
        if (v < 3.00 || v > 3.05) {
          throw std::runtime_error(
            "--rnx-version must be in [3.00, 3.05] (supported: 3.00, 3.01, 3.02, 3.03, 3.04, 3.05)");
        }
        o.rinex_version = v;
      }
      else if (a=="--nav-systems") o.nav_systems = next();
      else if (a=="--no-flush") o.flush_immediately=false;
      else if (a=="--pgm") o.program_name = next();
      else if (a=="--runby") o.run_by = next();
      else std::fprintf(stderr,"[warn] unknown arg %s\n", a.c_str());
    }
    int filtered = filterByBuildFlags(maskFromSystemString(o.nav_systems));
    std::string s;
    auto push=[&](char c,int bit){ if(filtered & bit) s.push_back(c); };
    push('G', SYS_GPS); push('R', SYS_GLO); push('E', SYS_GAL);
    push('J', SYS_QZS); push('C', SYS_CMP); push('S', SYS_SBS);
    push('I', SYS_IRN);
    o.nav_systems = s.empty() ? "G" : s;
    return o;
  }
};

/*============================== Main ==============================*/

int main(int argc, char **argv) {
  try { return App::run(argc, argv); }
  catch (const std::exception &e) { std::fprintf(stderr, "Fatal: %s\n", e.what()); return 1; }
  catch (...) { std::fprintf(stderr, "Fatal: unknown error\n"); return 1; }
}
