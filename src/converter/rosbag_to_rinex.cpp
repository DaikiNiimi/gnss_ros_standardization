#include "gnss_ros_standardization/bag_io_utils.hpp"
#include "gnss_ros_standardization/ephemeris_store.hpp"
// Convert a ROS 2 bag containing GnssObservations/GnssEphemerides messages
// to RINEX 3.0x observation and navigation files using RTKLIB writers.
//
// Design overview (two-pass):
//   Pass 1 (Scanner): walk the bag once to collect the union of observation
//                     types per GNSS system, the timespan, and GLONASS FCN
//                     assignments. The RINEX OBS header must declare every
//                     observation column up-front, so this scan determines
//                     the header before any data lines are written.
//   Pass 2 (Writers): walk the bag again. ObsWriter buffers epochs through a
//                     short reorder window (3 s) to tolerate slight out-of-
//                     order delivery from multiple publishers, then emits
//                     RINEX OBS records. NavWriter deduplicates ephemerides
//                     by complete converted contents, including absolute times.
//
// Reading the bag twice is the cost of producing a spec-conformant header;
// rosbag2 readers don't allow rewinding without re-opening.

#include <array>
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
#include "gnss_ros_standardization/gnss_utils.hpp"
#include <cstdint>
#include <filesystem>


using gnss_converter_io::deserializeRos;
using gnss_converter_io::deriveOutputPath;
using gnss_converter_io::normalizeBagUri;
using gnss_converter_io::safeParentClimb;

using gnss_ros_standardization::msg::GnssObservation;
using gnss_ros_standardization::msg::GnssObservations;
using gnss_ros_standardization::msg::GnssEphemeris;
using gnss_ros_standardization::msg::GlonassEphemeris;
using gnss_ros_standardization::msg::GnssEphemerides;


/*============================== Utilities ==============================*/

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

static inline int satFromFields(const std::string& satid,
                                const std::string& sys, int prn) {
  return gnss_utils::resolveSatellite(satid, sys, prn);
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
  bool topic_nav_explicit = false;
  std::string out_obs_path = "";
  std::string out_nav_path = "";
  double rinex_version = 3.05;
  std::string nav_systems = "GREJCIS";
  bool flush_immediately = true;
  bool nav_all_transmissions = false;
  bool legacy_glonass_time = false;
  std::string program_name = "rosbag_to_rinex";
  std::string run_by = "";   // empty → resolved from $USER/$LOGNAME at parseArgs
};

static inline std::string defaultRunBy() {
  if (const char* u = std::getenv("USER");    u && *u) return u;
  if (const char* l = std::getenv("LOGNAME"); l && *l) return l;
  return "user";
}

struct TimeSpan { bool init=false; gtime_t first{}, last{}; };

struct ObsTypeSet {
  std::map<char, std::unordered_set<std::string>> types; // "C1C", ...
};

struct GloFcnInfo {
  bool any = false;
  int8_t fcn_plus8[MAXPRNGLO] = {0};
  void add(int prn, int fcn) {
    // Keep the existing navigation-header range (FCN -7..+7). Observation
    // freq contributes only the backend's narrower FCN -7..+6 range.
    if (prn < 1 || prn > MAXPRNGLO || fcn < 1 || fcn > 15) return;
    if (fcn_plus8[prn-1] && fcn_plus8[prn-1] != fcn)
      throw std::runtime_error("Conflicting GLONASS frequency channels cannot be represented by one RINEX observation header");
    fcn_plus8[prn-1] = static_cast<int8_t>(fcn);
    any = true;
  }
};

/*============================== Bag Reader ==============================*/

class BagReader {
public:
  explicit BagReader(const std::string &raw) {
    rosbag2_storage::StorageOptions sopt;
    sopt.uri = normalizeBagUri(raw);
    sopt.storage_id = "";  // auto-detect from metadata.yaml (sqlite3 / mcap)
    rosbag2_cpp::ConverterOptions copt;
    copt.input_serialization_format  = "";
    copt.output_serialization_format = "";
    reader_.open(sopt, copt);
  }
  auto topics() { return reader_.get_all_topics_and_types(); }
  bool has_next() { return reader_.has_next(); }
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> read_next() {
    return reader_.read_next();
  }
private:
  rosbag2_cpp::Reader reader_;
};

/*============================== Pass1: scan types & timespan ==============================*/

static inline void addObsType(ObsTypeSet &set, char sys, const std::string &prefixed) {
  if (prefixed.size()>=3) set.types[sys].insert(prefixed.substr(0,3));
}

static inline bool allowFreqDigit(char sys, char d){
  switch(sys){
    case 'G': return d=='1'||d=='2'||d=='5';
    case 'R': return d=='1'||d=='2'||d=='3'||d=='4'||d=='6';
    case 'E': return d=='1'||d=='5'||d=='6'||d=='7'||d=='8';
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
    case 'E': return digit=='1'?0: digit=='7'?1: digit=='5'?2: digit=='6'?3: big;
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

// First broadcast ionosphere set of each system in the bag (RINEX NAV header).
struct HeaderIonosphere {
  std::array<double, 8> gps{}, qzs{}, bds{}, irn{};
  std::array<double, 4> gal{};
  template <std::size_t N, class A>
  static void take(std::array<double, N>& held, const A& in) {
    const bool empty = std::all_of(held.begin(), held.end(), [](double v) { return v == 0.0; });
    const bool usable = std::all_of(in.begin(), in.end(), [](double v) { return std::isfinite(v); }) &&
                        std::any_of(in.begin(), in.end(), [](double v) { return v != 0.0; });
    if (empty && usable) std::copy(in.begin(), in.end(), held.begin());
  }
  void add(const GnssEphemerides& m) {
    take(gps, m.ion_gps); take(gal, m.ion_gal); take(qzs, m.ion_qzs);
    take(bds, m.ion_bds); take(irn, m.ion_irn);
  }
  void apply(nav_t& nav) const {
    std::copy(gps.begin(), gps.end(), nav.ion_gps);
    std::copy(gal.begin(), gal.end(), nav.ion_gal);
    std::copy(qzs.begin(), qzs.end(), nav.ion_qzs);
    std::copy(bds.begin(), bds.end(), nav.ion_cmp);
    std::copy(irn.begin(), irn.end(), nav.ion_irn);
  }
};

class Scanner {
public:
  explicit Scanner(const Options& opt): opt_(opt) {}
  std::tuple<ObsTypeSet, TimeSpan, GloFcnInfo, HeaderIonosphere> run() {
    HeaderIonosphere ion;
    ObsTypeSet types;
    TimeSpan span;
    GloFcnInfo glo;
    BagReader reader(opt_.bag_uri);
    rclcpp::Serialization<GnssObservations> ser_obs;
    rclcpp::Serialization<GnssEphemerides> ser_nav;

    while (reader.has_next()) {
      auto msg = reader.read_next();
      if (!msg) break;

      if (msg->topic_name == opt_.topic_obs) {
        GnssObservations obs;
        if (!deserializeRos(*msg, ser_obs, obs)) continue;

        if (obs.week == 0 || !std::isfinite(obs.tow) || obs.tow < 0 || obs.tow >= 604800) continue;
        gtime_t t = gpst2time((int)obs.week, obs.tow);
        if (!span.init) { span.init=true; span.first=t; span.last=t; }
        else {
          if (timediff(t, span.first)<0) span.first=t;
          if (timediff(t, span.last)>0)  span.last=t;
        }

        for (const auto &o : obs.observations) {
          const int sat = satFromFields(o.satid, o.system, o.prn);
          int prn = 0;
          if (sat && satsys(sat, &prn) == SYS_GLO && o.freq <= 13)
            glo.add(prn, static_cast<int>(o.freq) + 1);
          if (o.system.size()!=1) continue;
          char sys = o.system[0];
          if (std::isfinite(o.p)) addObsType(types, sys, "C"+o.code_str);
          if (std::isfinite(o.l)) addObsType(types, sys, "L"+o.code_str);
          if (std::isfinite(o.d)) addObsType(types, sys, "D"+o.code_str);
          if (std::isfinite(o.snr)) addObsType(types, sys, "S"+o.code_str);
        }
      }
      else if (msg->topic_name == opt_.topic_nav) {
        GnssEphemerides navs;
        if (!deserializeRos(*msg, ser_nav, navs)) continue;
        ion.add(navs);
        for (const auto& ge : navs.glonass_ephemeris) {
          if (ge.prn >= 1 && ge.prn <= MAXPRNGLO) {
            int val = (int)ge.frq + 8;
            glo.add(ge.prn, val);
          }
        }
      }
    }
    return {types, span, glo, ion};
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
      if (!allowFreqDigit(sys, name[1])) throw std::runtime_error("Unsupported RINEX observation: " + std::string(1,sys) + name);
      if (w >= MAXOBSTYPE) throw std::runtime_error("Too many RINEX observation types");
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
      char sig[3] = { s[1], s[2], 0 };
      if (obs2code(sig) == 0) throw std::runtime_error("Unknown RINEX signal: " + std::string(sig));
      if (w!=j) {
        char tmp[4];
        std::memcpy(tmp, s, 4);          // break alias: s == o.tobs[i][j]
        std::memcpy(o.tobs[i][w], tmp, sizeof(o.tobs[i][w]));
      }
      ++w;
    }
    for (int j=w;j<MAXOBSTYPE;j++) o.tobs[i][j][0] = '\0';
    o.nobs[i] = w;
  }
}

/*============================== RINEX Option Builders ==============================*/

static inline rnxopt_t makeRnxOpt(double ver, int navsys, const std::string& pgm, const std::string& runby) {
  rnxopt_t o; std::memset(&o, 0, sizeof(o));
  o.rnxver = static_cast<double>(std::lround(ver * 100.0));
  o.navsys = filterByBuildFlags(navsys);
  std::snprintf(o.prog,  sizeof(o.prog),  "%s", pgm.c_str());
  std::snprintf(o.runby, sizeof(o.runby), "%s", runby.c_str());
  for (int i=0;i<7;i++) o.mask[i][0] = '\0';
  return o;
}

/*============================== Writers ==============================*/

class NavWriter {
public:
  NavWriter(const Options &opt, FILE *fp, const HeaderIonosphere& ion)
    : opt_(opt), fp_(fp),
      rnx_(makeRnxOpt(opt.rinex_version, maskFromSystemString(opt.nav_systems),
                         opt.program_name, opt.run_by)) {
    std::memset(&nav_dummy_, 0, sizeof(nav_dummy_));
    ion.apply(nav_dummy_);
    rnx_.outiono = 1;
    if (outrnxnavh(fp_, &rnx_, &nav_dummy_) == 0) throw std::runtime_error("outrnxnavh failed");
    std::fflush(fp_);
  }

  void onEphemerides(const GnssEphemerides &batch) {
    auto sys_of_sat = [](int sat)->int{ int prn=0; return satsys(sat, &prn); };

    // Copy to sort so output is deterministic and matches RTKLIB
    std::vector<gnss_ros_standardization::msg::GnssEphemeris> gnss_sorted = batch.gnss_ephemeris;
    std::sort(gnss_sorted.begin(), gnss_sorted.end(), [](const auto& a, const auto& b) {
      if (a.system != b.system) return a.system < b.system;
      if (a.prn != b.prn) return a.prn < b.prn;
      if (a.toe != b.toe) return a.toe < b.toe;
      return a.ttr < b.ttr;
    });

    for (const auto &m : gnss_sorted) {
      eph_t e = gnss_utils::msgToEph(m);
      if (!e.sat) throw std::runtime_error("Invalid Keplerian navigation record");

      int sys = sys_of_sat(e.sat);
      if ((sys & rnx_.navsys) == 0) continue;
      
      if (const char* error = gnss_utils::rinexNavigationError(m, e, rnx_.rnxver))
        throw std::runtime_error(std::string(error) + "; retain the navigation bag");

      const auto record = renderRecord([&](FILE* stream) { return outrnxnavb(stream, &rnx_, &e); });
      const auto key = opt_.nav_all_transmissions ? gnss_utils::navigationRecordKey(e)
                                                : contentKey(record, false);
      if (seen_k_.insert(key).second) writeRecord(record);
    }

    if (rnx_.navsys & SYS_GLO) {
      std::vector<gnss_ros_standardization::msg::GlonassEphemeris> glo_sorted = batch.glonass_ephemeris;
      std::sort(glo_sorted.begin(), glo_sorted.end(), [](const auto& a, const auto& b) {
        if (a.prn != b.prn) return a.prn < b.prn;
        if (a.toe != b.toe) return a.toe < b.toe;
        return a.tof < b.tof;
      });

      for (const auto &m : glo_sorted) {
        geph_t g = gnss_utils::msgToGeph(m);
        if (g.sat == 0) throw std::runtime_error("Invalid GLONASS ephemeris arrays or satellite");
        if (opt_.legacy_glonass_time) {
          g.toe = gpst2utc(g.toe); g.tof = gpst2utc(g.tof);
        }
        if (const char* error = gnss_utils::rinexNavigationError(m, g, rnx_.rnxver))
          throw std::runtime_error(std::string(error) + "; retain the navigation bag");

        const auto record = renderRecord([&](FILE* stream) { return outrnxgnavb(stream, &rnx_, &g); });
        const auto key = opt_.nav_all_transmissions ? gnss_utils::navigationRecordKey(g)
                                                  : contentKey(record, true);
        if (seen_r_.insert(key).second) writeRecord(record);
      }
    }

    if (opt_.flush_immediately) std::fflush(fp_);
  }

private:
  template<class Writer> static std::string renderRecord(Writer writer) {
    char* buffer=nullptr;
    size_t length=0;
    FILE* stream=open_memstream(&buffer,&length);
    if (!stream) throw std::runtime_error("Cannot allocate RINEX record stream");
    const bool written=writer(stream)!=0;
    const bool closed=std::fclose(stream)==0;
    std::unique_ptr<char, decltype(&std::free)> owned(buffer,&std::free);
    if (!written || !closed) throw std::runtime_error("Cannot format RINEX navigation record");
    return std::string(buffer,length);
  }
  std::string contentKey(std::string record, bool glonass) const {
    // Only transmission time is ignored, at the precision of this RINEX
    // version. Never blank GLONASS's last line (3.05 adds flags/accuracy).
    const size_t field=glonass ? (rnx_.rnxver>=300 ? 61 : 60)
        : record.rfind('\n',record.size()-2)+1+(rnx_.rnxver>=300 ? 4 : 3);
    if (field+19>record.size() || record.substr(field,19).find('\n')!=std::string::npos)
      throw std::runtime_error("Unexpected RINEX navigation layout");
    record.replace(field,19,19,' ');
    return record;
  }
  void writeRecord(const std::string& record) {
    if (std::fwrite(record.data(),1,record.size(),fp_)!=record.size())
      throw std::runtime_error("Cannot write RINEX navigation record");
  }
  Options opt_;
  FILE *fp_{nullptr};
  rnxopt_t rnx_{};
  nav_t nav_dummy_{};
  std::unordered_set<std::string> seen_k_;
  std::unordered_set<std::string> seen_r_;
};

class ObsWriter {
  public:
    ObsWriter(const Options &opt, FILE *fp, 
              const ObsTypeSet &types, const TimeSpan* span,
              const GloFcnInfo* glo_fcn_info)
      : opt_(opt), fp_(fp),
        rnx_(makeRnxOpt(opt.rinex_version, maskFromSystemString(opt.nav_systems),
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
        if (opt_.rinex_version < 3.02)
          throw std::runtime_error("GLONASS frequency channels require a RINEX observation header version >= 3.02");
        geph_buf_.reset(new geph_t[MAXPRNGLO]());
        nav_dummy_.geph = geph_buf_.get();
        for (int i = 0; i < MAXPRNGLO; ++i) if (glo_fcn_info->fcn_plus8[i] != 0) nav_dummy_.glo_fcn[i] = glo_fcn_info->fcn_plus8[i];
      }
  
      if (outrnxobsh(fp_, &rnx_, &nav_dummy_) == 0) throw std::runtime_error("outrnxobsh failed");
      std::fflush(fp_);
    }
  
    std::size_t invalid_time_epochs_{0};

    void finish() {
      flushAll();
      if (invalid_time_epochs_)
        std::cerr << "Warning: skipped " << invalid_time_epochs_
                  << " observation epochs with invalid GPS week/tow\n";
    }

    void writeEpoch(const GnssObservations &m) {
      if (m.week == 0 || !std::isfinite(m.tow) || m.tow < 0 || m.tow >= 604800) {
        ++invalid_time_epochs_;
        return;
      }
      gtime_t t = gpst2time((int)m.week, m.tow);
      if (!have_last_ || timediff(t, last_seen_) > 0.0) { last_seen_ = t; have_last_ = true; }
      auto same = std::find_if(queue_.begin(),queue_.end(),[&](const Epoch& e) {
        return std::abs(timediff(t,e.t)) <= 1e-6;
      });
      if (same == queue_.end()) {
        auto pos=std::lower_bound(queue_.begin(),queue_.end(),t,
            [](const Epoch& e,gtime_t time) { return timediff(e.t,time)<0; });
        queue_.insert(pos,Epoch{t,m});
      } else {
        for (const auto& o : m.observations) {
          auto previous=std::find_if(same->msg.observations.begin(),same->msg.observations.end(),
              [&](const auto& v) {
                return satFromFields(v.satid,v.system,v.prn)==satFromFields(o.satid,o.system,o.prn) &&
                       v.code_str==o.code_str;
              });
          if (previous == same->msg.observations.end()) same->msg.observations.push_back(o);
          else {
            // Identity uses the same stable-field fallback as the writer.
            // Optional/deprecated aliases must not turn equal observations
            // into conflicts (or merge two empty satid fields from different SVs).
            auto normalized=o;
            normalized.sat=previous->sat; normalized.system=previous->system;
            normalized.prn=previous->prn; normalized.satid=previous->satid;
            normalized.code=previous->code;
            if (!gnss_utils::sameObservation(*previous,normalized))
              throw std::runtime_error("Conflicting duplicate observation " + o.satid + "/" + o.code_str);
          }
        }
      }
      const auto threshold = timeadd(last_seen_, -reorder_window_sec_);
      while (!queue_.empty() && timediff(queue_.front().t,threshold)<=0) {
        writeOne(queue_.front().msg,queue_.front().t);
        queue_.erase(queue_.begin());
      }
    }

  private:
    struct Epoch {
      gtime_t t{};
      GnssObservations msg;
    };

    void writeOne(const GnssObservations &m, gtime_t t) {
      if (have_written_ && timediff(t,last_written_) <= 0)
        throw std::runtime_error("Observation epoch out of order beyond reorder window (3 s); input is not monotonic");
      // Write directly from ROS values: obsd_t uses zero as missing and only
      // six slots, neither of which expresses the ROS observation contract.
      using Observation = gnss_ros_standardization::msg::GnssObservation;
      std::map<int, std::map<std::string, const Observation*>> satellites;
      for (const auto& o : m.observations) {
        const int sat = satFromFields(o.satid, o.system, o.prn);
        if (!sat) throw std::runtime_error("Invalid observation satellite");
        if (!(satsys(sat, nullptr) & rnx_.navsys)) continue;
        if (!codeIdFromSig2(o.code_str)) throw std::runtime_error("Invalid observation code");
        if (!satellites[sat].emplace(o.code_str, &o).second)
          throw std::runtime_error("Duplicate satellite/signal in one observation epoch");
      }
      if (satellites.empty()) return;
      last_written_=t; have_written_=true;
      double ep[6]; time2epoch_n(t, ep, 7);
      std::fprintf(fp_, "> %04.0f %02.0f %02.0f %02.0f %02.0f %010.7f  0%3zu\n",
                   ep[0],ep[1],ep[2],ep[3],ep[4],ep[5],satellites.size());
      for (const auto& [sat, signals] : satellites) {
        char id[16]; satno2id(sat, id);
        int prn=0;
        if (satsys(sat,&prn)==SYS_SBS) std::snprintf(id,sizeof(id),"S%02d",prn-100);
        const int idx = sysIdx(id[0]);
        std::fprintf(fp_, "%.3s", id);
        for (int j = 0; j < rnx_.nobs[idx]; ++j) {
          const std::string type(rnx_.tobs[idx][j]);
          const auto it = signals.find(type.substr(1));
          double value = std::numeric_limits<double>::quiet_NaN();
          unsigned lli = 0;
          if (it != signals.end()) {
            const auto& o = *it->second;
            switch (type[0]) {
              case 'C': value=o.p; break;
              case 'L': value=o.l; lli=o.lli & 7; break;
              case 'D': value=o.d; break;
              case 'S': value=o.snr; break;
            }
          }
          if (!std::isfinite(value)) { std::fprintf(fp_, "%16s", ""); continue; }
          char field[64];
          if (std::snprintf(field, sizeof(field), "%14.3f", value) != 14)
            throw std::runtime_error("Observation exceeds RINEX F14.3 range");
          std::fprintf(fp_, "%s%c ", field, lli ? char('0'+lli) : ' ');
        }
        std::fputc('\n', fp_);
      }
      if (opt_.flush_immediately) std::fflush(fp_);
    }
  
    void flushAll() {
      for (const auto& e : queue_) writeOne(e.msg,e.t);
      queue_.clear();
      std::fflush(fp_);
    }
  
  private:
    Options opt_;
    FILE *fp_{nullptr};
    rnxopt_t rnx_{};
    nav_t nav_dummy_{};
    std::unique_ptr<geph_t[]> geph_buf_;

    const double reorder_window_sec_ = 3.0;
    bool have_last_ = false;
    gtime_t last_seen_{};
    bool have_written_ = false;
    gtime_t last_written_{};
    std::vector<Epoch> queue_;
  };  

/*============================== App ==============================*/

using gnss_converter_io::OutputTransaction;

class App {
public:
  static void printUsage(FILE* out) {
    std::fprintf(out,
      "Usage: rosbag_to_rinex --bag <bag_dir_or_db3> "
      "[--obs OBS_PATH --nav NAV_PATH "
      "--topic-obs TOPIC --topic-nav TOPIC "
      "--rnx-version X.YY --nav-systems GREJCIS "
      "--nav-all-transmissions --no-flush --legacy-glonass-time --pgm NAME --runby NAME] "
      "[--help] [--version]\n");
  }

  static int run(int argc, char **argv) {
    for (int i = 1; i < argc; ++i) {
      std::string a = argv[i];
      if (a == "--help" || a == "-h") { printUsage(stdout); return 0; }
      if (a == "--version" || a == "-V") {
#ifdef PACKAGE_VERSION
        std::fprintf(stdout, "rosbag_to_rinex %s\n", PACKAGE_VERSION);
#else
        std::fprintf(stdout, "rosbag_to_rinex (unknown version)\n");
#endif
        return 0;
      }
    }
    rclcpp::InitOptions init_options;
  init_options.auto_initialize_logging(false);
  rclcpp::init(argc, argv, init_options);
    Options opt = parseArgs(argc, argv);
    if (opt.bag_uri.empty()) {
      printUsage(stderr);
      return 2;
    }

    if (opt.out_obs_path.empty() && opt.out_nav_path.empty()) {
      opt.out_obs_path = deriveOutputPath(opt.bag_uri, ".obs");
      opt.out_nav_path = deriveOutputPath(opt.bag_uri, ".nav");
      std::fprintf(stderr, "Info: output paths auto-derived: %s, %s\n",
                   opt.out_obs_path.c_str(), opt.out_nav_path.c_str());
    }

    // Check declared topic/type before opening (and potentially truncating) output files.
    BagReader metadata(opt.bag_uri);
    const auto topics = metadata.topics();
    auto require_topic = [&](const std::string& topic, const std::string& type) {
      const auto it = std::find_if(topics.begin(), topics.end(),
          [&](const auto& value) { return value.name == topic; });
      if (it == topics.end()) throw std::runtime_error("Missing input topic: " + topic);
      if (it->type != "gnss_ros_standardization/msg/" + type)
        throw std::runtime_error("Wrong input topic type: " + topic);
    };
    if (!opt.out_obs_path.empty()) require_topic(opt.topic_obs, "GnssObservations");
    if (!opt.out_nav_path.empty()) {
      const bool present = std::any_of(topics.begin(), topics.end(),
          [&](const auto& t) { return t.name == opt.topic_nav; });
      if (present || opt.topic_nav_explicit || opt.out_obs_path.empty())
        require_topic(opt.topic_nav, "GnssEphemerides");
      else std::fprintf(stderr, "Warning: default navigation topic absent; writing header-only NAV alongside observations\n");
    }

    Scanner scanner(opt);
    auto [types, span, glo, ion] = scanner.run();

    OutputTransaction outputs;
    FILE *fp_obs = nullptr;
    FILE *fp_nav = nullptr;
    std::unique_ptr<ObsWriter> obs_writer;
    std::unique_ptr<NavWriter> nav_writer;

    if (!opt.out_obs_path.empty()) {
      fp_obs = outputs.open(opt.out_obs_path);
      obs_writer = std::make_unique<ObsWriter>(opt, fp_obs, types, &span, &glo);
    }

    if (!opt.out_nav_path.empty()) {
      fp_nav = outputs.open(opt.out_nav_path);
      nav_writer = std::make_unique<NavWriter>(opt, fp_nav, ion);
    }

    BagReader reader(opt.bag_uri);
    rclcpp::Serialization<GnssObservations> ser_obs;
    rclcpp::Serialization<GnssEphemerides> ser_nav;
    while (reader.has_next()) {
      auto msg = reader.read_next(); if (!msg) throw std::runtime_error("Bag returned an empty record before EOF");

      if (obs_writer && msg->topic_name == opt.topic_obs) {
        GnssObservations o; deserializeRos(*msg, ser_obs, o); obs_writer->writeEpoch(o);
      } else if (nav_writer && msg->topic_name == opt.topic_nav) {
        GnssEphemerides n; deserializeRos(*msg, ser_nav, n); nav_writer->onEphemerides(n);
      }
    }

    if (obs_writer) obs_writer->finish();
    obs_writer.reset(); nav_writer.reset();
    outputs.commit();
    rclcpp::shutdown();
    return 0;
  }

private:
  static Options parseArgs(int argc, char **argv) {
    Options o;
    const auto args = rclcpp::remove_ros_arguments(argc, argv);
    for (size_t i=1;i<args.size();i++) {
      std::string a=args[i];
      auto next=[&](){ if(i+1<args.size()) return args[++i]; throw std::runtime_error("missing value for "+a); };
      if (a=="--bag") o.bag_uri = next();
      else if (a=="--obs") o.out_obs_path = next();
      else if (a=="--nav") o.out_nav_path = next();
      else if (a=="--topic-obs") o.topic_obs = next();
      else if (a=="--topic-nav") { o.topic_nav = next(); o.topic_nav_explicit = true; }
      else if (a=="--rnx-version") {
        const auto text = next();
        size_t consumed = 0;
        o.rinex_version = std::stod(text, &consumed);
        if (consumed != text.size() || !std::isfinite(o.rinex_version) ||
            o.rinex_version < 3.0 || o.rinex_version > 3.05)
          throw std::runtime_error("invalid --rnx-version: " + text);
        const double v = std::lround(o.rinex_version * 100.0) / 100.0;
        if (v < 3.00 || v > 3.05) {
          throw std::runtime_error(
            "--rnx-version must be in [3.00, 3.05] (supported: 3.00, 3.01, 3.02, 3.03, 3.04, 3.05)");
        }
        o.rinex_version = v;
      }
      else if (a=="--nav-systems") o.nav_systems = next();
      else if (a=="--legacy-glonass-time") o.legacy_glonass_time=true;
      else if (a=="--nav-all-transmissions") o.nav_all_transmissions=true;
      else if (a=="--no-flush") o.flush_immediately=false;
      else if (a=="--pgm") o.program_name = next();
      else if (a=="--runby") o.run_by = next();
      else throw std::runtime_error("unknown argument: " + a);
    }
    if (o.run_by.empty()) o.run_by = defaultRunBy();
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
