#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "gnss_ros_standardization/msg/gnss_observation.hpp"
#include "gnss_ros_standardization/msg/gnss_observations.hpp"
#include "gnss_ros_standardization/msg/gnss_ephemeris.hpp"
#include "gnss_ros_standardization/msg/glonass_ephemeris.hpp"
#include "gnss_ros_standardization/msg/gnss_ephemerides.hpp"

extern "C" {
  #include "rtklib.h"
}

using namespace std::chrono_literals;

namespace {
constexpr uint8_t  RTCM3_PREAMBLE = 0xD3;
constexpr uint16_t RTCM3_MAX_LEN  = 1023;
constexpr size_t   RTCM3_HDR_LEN  = 3;     // preamble + length(2)
constexpr size_t   RTCM3_CRC_LEN  = 3;
constexpr uint32_t CRC24Q_POLY    = 0x1864CFB; // CRC-24Q
constexpr double   TOE_EQ_EPS     = 1e-3;      // sec
} // namespace

class RtcmDecoderNode : public rclcpp::Node {
public:
  RtcmDecoderNode() : Node("rtcm_decoder_node") {
    declare_parameter<std::string>("stream_path", "tcpcli://127.0.0.1:28003");
    declare_parameter<int>("assemble_delay_ms", 200);  // reserved

    obs_pub_ = create_publisher<gnss_ros_standardization::msg::GnssObservations>("/gnss/observation", 10);
    nav_pub_ = create_publisher<gnss_ros_standardization::msg::GnssEphemerides>("/gnss/ephemeris", 10);

    if (init_rtcm(&rtcm_) != 1) {
      RCLCPP_ERROR(get_logger(), "init_rtcm failed");
      throw std::runtime_error("init_rtcm failed");
    }

    openStream();
    timer_ = create_wall_timer(10ms, std::bind(&RtcmDecoderNode::onTimer, this));
    RCLCPP_INFO(get_logger(), "RTCM Decoder started");
  }

  ~RtcmDecoderNode() override {
    try { strclose(&stream_); } catch (...) {}
    free_rtcm(&rtcm_);
  }

private:
  // ---- small utilities -----------------------------------------------------
  static std::string systemCode(int sys) {
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

  static std::string satId(int sat) {
    char id[8] = {0};
    satno2id(sat, id);
    return std::string(id);
  }

  static uint32_t crc24q(const uint8_t* p, size_t len) {
    uint32_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
      crc ^= (uint32_t)p[i] << 16;
      for (int b = 0; b < 8; ++b) {
        crc <<= 1;
        if (crc & 0x1000000) crc ^= CRC24Q_POLY;
      }
      crc &= 0xFFFFFF;
    }
    return crc;
  }

  // ---- stream --------------------------------------------------------------
  void openStream() {
    const std::string original = get_parameter("stream_path").as_string();
    std::string path = original;

    struct Def { std::string_view prefix; int type; };
    static const Def defs[] = {
      {"tcpcli://", STR_TCPCLI},
      {"serial://", STR_SERIAL},
      {"ntrip://",  STR_NTRIPCLI},
      {"file://",   STR_FILE}
    };

    int stype = 0;
    bool matched = false;
    for (const auto& d : defs) {
      if (path.rfind(d.prefix, 0) == 0) {
        stype = d.type;
        path.erase(0, d.prefix.size());
        matched = true;
        break;
      }
    }
    if (!matched) {
      RCLCPP_ERROR(get_logger(), "Unsupported stream_path: %s", original.c_str());
      throw std::runtime_error("bad stream_path");
    }
    if (!stropen(&stream_, stype, STR_MODE_R, path.c_str())) {
      RCLCPP_ERROR(get_logger(), "stropen failed: %s", original.c_str());
      throw std::runtime_error("stropen failed");
    }
    RCLCPP_INFO(get_logger(), "Stream opened: %s", original.c_str());
  }

  void onTimer() {
    uint8_t buf[4096];
    const int n = strread(&stream_, buf, sizeof(buf));
    if (n > 0) {
      rx_.insert(rx_.end(), buf, buf + n);
      decodeRtcm3FromBuffer();
    }
    flushEpochs();
  }

  // ---- RTCM3 framing & decode ---------------------------------------------
  void decodeRtcm3FromBuffer() {
    while (true) {
      if (rx_.size() < RTCM3_HDR_LEN) return;
      if (rx_[0] != RTCM3_PREAMBLE) { rx_.erase(rx_.begin()); continue; }

      const uint16_t plen = ((rx_[1] & 0x03) << 8) | rx_[2];
      if (plen > RTCM3_MAX_LEN) { rx_.erase(rx_.begin()); continue; }

      const size_t flen = RTCM3_HDR_LEN + plen + RTCM3_CRC_LEN;
      if (rx_.size() < flen) return;

      const uint32_t crc_calc = crc24q(rx_.data(), flen - RTCM3_CRC_LEN);
      const uint32_t crc_recv =
          (uint32_t(rx_[flen - 3]) << 16) |
          (uint32_t(rx_[flen - 2]) << 8)  |
           uint32_t(rx_[flen - 1]);

      if (crc_calc != crc_recv) { rx_.erase(rx_.begin()); continue; }

      int ret = 0;
      for (size_t i = 0; i < flen; ++i) ret = input_rtcm3(&rtcm_, rx_[i]);
      rx_.erase(rx_.begin(), rx_.begin() + static_cast<long>(flen));

      if (ret == 1 && rtcm_.obs.n > 0) accumulateObservations();
      publishEphemeridesIfChanged();
    }
  }

  // ---- observations --------------------------------------------------------
  struct EpochKey {
    int week{0};
    int64_t tow_ms{0};
    bool operator==(const EpochKey& o) const { return week == o.week && tow_ms == o.tow_ms; }
  };
  struct EpochKeyHash {
    size_t operator()(const EpochKey& k) const {
      return (size_t(uint32_t(k.week)) << 32) ^ size_t(k.tow_ms);
    }
  };
  struct EpochBuffer {
    std::vector<gnss_ros_standardization::msg::GnssObservation> observations;
    int cnt_G=0,cnt_R=0,cnt_E=0,cnt_J=0,cnt_C=0,cnt_S=0,cnt_U=0;
    int    week{0};
    double tow{0.0};
    gtime_t gpst_time{};
    rclcpp::Time last_update_wall;
  };

  void accumulateObservations() {
    int week = 0;
    const double tow = time2gpst(rtcm_.time, &week);
    const int64_t tow_ms = static_cast<int64_t>(std::llround(tow * 1000.0));

    EpochKey key{week, tow_ms};
    auto& epoch = epochs_[key];
    epoch.gpst_time = rtcm_.time;
    epoch.week = week;
    epoch.tow = tow;
    epoch.last_update_wall = now();

    for (int j = 0; j < rtcm_.obs.n; ++j) {
      const obsd_t* o = &rtcm_.obs.data[j];

      int prn = 0;
      const int sys = satsys(o->sat, &prn);
      const std::string syscode = systemCode(sys);

      for (int kf = 0; kf < NFREQ + NEXOBS; ++kf) {
        const bool empty = (o->P[kf] == 0.0) && (o->L[kf] == 0.0) &&
                           (o->D[kf] == 0.0) && (o->SNR[kf] == 0);
        if (empty) continue;

        gnss_ros_standardization::msg::GnssObservation obs;
        obs.system = syscode;
        obs.prn    = prn;
        obs.satid  = satId(o->sat);

        if (o->code[kf]) {
          if (const char* sig = code2obs(o->code[kf]); sig) obs.code = sig;
        }

        obs.pseudorange   = o->P[kf];
        obs.carrier_phase = o->L[kf];
        obs.doppler       = o->D[kf];
        obs.snr           = static_cast<float>(o->SNR[kf]) * 0.001f; // dB-Hz -> *1000 -> float
        obs.lli           = o->LLI[kf];

        epoch.observations.push_back(std::move(obs));
      }

      switch (sys) {
        case SYS_GPS: ++epoch.cnt_G; break;
        case SYS_GLO: ++epoch.cnt_R; break;
        case SYS_GAL: ++epoch.cnt_E; break;
        case SYS_QZS: ++epoch.cnt_J; break;
        case SYS_CMP: ++epoch.cnt_C; break;
        case SYS_SBS: ++epoch.cnt_S; break;
        default:      ++epoch.cnt_U; break;
      }
    }
  }

  void flushEpochs() {
    const auto stamp = now();
    for (auto it = epochs_.begin(); it != epochs_.end(); ) {
      auto& epoch = it->second;

      gnss_ros_standardization::msg::GnssObservations msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = "gnss_receiver";
      msg.week = static_cast<uint16_t>(epoch.week);
      msg.tow  = epoch.tow;
      msg.observations = std::move(epoch.observations);

      obs_pub_->publish(msg);

      RCLCPP_INFO(get_logger(),
        "obs published: week=%d tow=%.3f num=%zu sats(G/R/E/J/C/S/U)=(%d/%d/%d/%d/%d/%d/%d)",
        epoch.week, epoch.tow, msg.observations.size(),
        epoch.cnt_G, epoch.cnt_R, epoch.cnt_E, epoch.cnt_J, epoch.cnt_C, epoch.cnt_S, epoch.cnt_U);

      it = epochs_.erase(it);
    }
  }

  // ---- ephemerides (publish on change) -------------------------------------
  void publishEphemeridesIfChanged() {
    // helpful hint if base is not sending 1020
    if (rtcm_.nav.ng == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No GLONASS ephemerides (RTCM 1020). Check base settings.");
    }

    bool changed = false;

    // Kepler (GPS/GAL/QZS/BeiDou/SBAS): choose latest toe per sat (Galileo: same toe -> larger code)
    std::unordered_map<int, int> best_kepler;
    for (int i = 0; i < rtcm_.nav.n; ++i) {
      const eph_t& e = rtcm_.nav.eph[i];
      if (!e.sat) continue;
      int prn = 0;
      if (satsys(e.sat, &prn) == SYS_GLO) continue;

      int w = 0; const double toe = time2gpst(e.toe, &w);
      auto it = best_kepler.find(e.sat);
      if (it == best_kepler.end()) {
        best_kepler[e.sat] = i;
      } else {
        const eph_t& prev = rtcm_.nav.eph[it->second];
        int wp = 0; const double toe_p = time2gpst(prev.toe, &wp);
        bool better = (toe > toe_p + TOE_EQ_EPS);
        if (!better && std::fabs(toe - toe_p) <= TOE_EQ_EPS && satsys(e.sat, &prn) == SYS_GAL) {
          better = (e.code > prev.code);
        }
        if (better) best_kepler[e.sat] = i;
      }
    }

    std::vector<gnss_ros_standardization::msg::GnssEphemeris> kmsgs;
    kmsgs.reserve(rtcm_.nav.n);
    
    for (int i = 0; i < rtcm_.nav.n; ++i) {
      const eph_t& e = rtcm_.nav.eph[i];
      if (!e.sat) continue;
      int prn = 0;
      if (satsys(e.sat, &prn) == SYS_GLO) continue;
    
      KKey k{e.sat, (int)e.iode, (int)e.iodc, (int)e.code};
      if (seen_kepler_.insert(k).second) {
        kmsgs.push_back(toKeplerMsg(e));
        changed = true;
      }
    }

    // GLONASS: choose latest toe per sat
    std::unordered_map<int, int> best_glo;
    for (int i = 0; i < rtcm_.nav.ng; ++i) {
      const geph_t& g = rtcm_.nav.geph[i];
      if (!g.sat) continue;

      int w = 0; const double toe = time2gpst(utc2gpst(g.toe), &w);
      auto it = best_glo.find(g.sat);
      if (it == best_glo.end()) {
        best_glo[g.sat] = i;
      } else {
        const geph_t& prev = rtcm_.nav.geph[it->second];
        int wp = 0; const double toe_p = time2gpst(utc2gpst(prev.toe), &wp);
        if (toe > toe_p + TOE_EQ_EPS) best_glo[g.sat] = i;
      }
    }

    std::vector<gnss_ros_standardization::msg::GlonassEphemeris> rmsgs;
    rmsgs.reserve(best_glo.size());
    for (auto& kv : best_glo) {
      const geph_t& g = rtcm_.nav.geph[kv.second];
      auto it = last_glo_iode_.find(g.sat);
      if (it == last_glo_iode_.end() || it->second != (int)g.iode) {
        last_glo_iode_[g.sat] = (int)g.iode;
        changed = true;
      }
      rmsgs.push_back(toGlonassMsg(g));
    }

    static bool first = true;
    if (!changed && !first) return;
    first = false;

    gnss_ros_standardization::msg::GnssEphemerides out;
    out.header.stamp      = now();
    out.gnss_ephemeris    = std::move(kmsgs);
    out.glonass_ephemeris = std::move(rmsgs);
    nav_pub_->publish(out);

    RCLCPP_INFO(get_logger(), "nav published: GNSS=%zu GLO=%zu (changed=%s)",
                out.gnss_ephemeris.size(), out.glonass_ephemeris.size(),
                changed ? "yes" : "no");
  }

  // ---- builders ------------------------------------------------------------
  static gnss_ros_standardization::msg::GnssEphemeris toKeplerMsg(const eph_t& e) {
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

    m.a=e.A; m.e=e.e; m.i0=e.i0; m.omg0=e.OMG0; m.omg=e.omg; m.m0=e.M0;
    m.deln=e.deln; m.omgd=e.OMGd; m.idot=e.idot;
    m.crc=e.crc; m.crs=e.crs; m.cuc=e.cuc; m.cus=e.cus; m.cic=e.cic; m.cis=e.cis;
    m.f0=e.f0; m.f1=e.f1; m.f2=e.f2;

    m.tgd.clear();
    m.tgd.push_back(e.tgd[0]);
    m.tgd.push_back(e.tgd[1]);

    m.iode=e.iode; m.iodc=e.iodc; m.svh=e.svh; m.sva=e.sva; m.code=e.code;
    return m;
  }

  static gnss_ros_standardization::msg::GlonassEphemeris toGlonassMsg(const geph_t& g) {
    gnss_ros_standardization::msg::GlonassEphemeris m;
    m.system = "R";

    int prn = 0; (void)satsys(g.sat, &prn);
    m.prn = prn;
    m.satid = satId(g.sat);
    m.frq = g.frq;

    int w = 0;
    m.toe  = time2gpst(utc2gpst(g.toe), &w);
    m.week = static_cast<uint16_t>(w);
    m.tof = time2gpst(utc2gpst(g.tof), &w);

    m.pos = { g.pos[0], g.pos[1], g.pos[2] };
    m.vel = { g.vel[0], g.vel[1], g.vel[2] };
    m.acc = { g.acc[0], g.acc[1], g.acc[2] };

    m.iode = g.iode; m.svh = g.svh; m.age = g.age;
    m.gamn = g.gamn; m.taun = g.taun; m.dtaun = g.dtaun;
    return m;
  }

private:
  // pubs/timer
  rclcpp::Publisher<gnss_ros_standardization::msg::GnssObservations>::SharedPtr  obs_pub_;
  rclcpp::Publisher<gnss_ros_standardization::msg::GnssEphemerides>::SharedPtr   nav_pub_;
  rclcpp::TimerBase::SharedPtr                                         timer_;

  // stream/decoder
  stream_t  stream_{};
  rtcm_t    rtcm_{};
  std::vector<uint8_t> rx_;

  // epoch buffer
  std::unordered_map<EpochKey, EpochBuffer, EpochKeyHash> epochs_;

  struct KKey {
    int sat; int iode; int iodc; int code;
    bool operator==(const KKey& o) const {
      return sat==o.sat && iode==o.iode && iodc==o.iodc && code==o.code;
    }
  };
  struct KKeyHash {
    size_t operator()(const KKey& k) const {
      return (size_t)k.sat ^ ((size_t)k.iode<<16) ^ ((size_t)k.iodc<<1) ^ ((size_t)k.code<<24);
    }
  };
  
  std::unordered_set<KKey, KKeyHash> seen_kepler_;
  std::unordered_map<int, int>       last_glo_iode_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RtcmDecoderNode>());
  rclcpp::shutdown();
  return 0;
}
