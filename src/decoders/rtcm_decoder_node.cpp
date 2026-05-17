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

#include "gnss_ros_standardization/ephemeris_store.hpp"
#include "gnss_ros_standardization/gnss_utils.hpp"


using namespace std::chrono_literals;

namespace {
constexpr uint8_t  RTCM3_PREAMBLE = 0xD3;
constexpr uint16_t RTCM3_MAX_LEN  = 1023;
constexpr size_t   RTCM3_HDR_LEN  = 3;     // preamble + length(2)
constexpr size_t   RTCM3_CRC_LEN  = 3;
constexpr uint32_t CRC24Q_POLY    = 0x1864CFB; // CRC-24Q
} // namespace

class RtcmDecoderNode : public rclcpp::Node {
public:
  RtcmDecoderNode() : Node("rtcm_decoder_node") {
    declare_parameter<std::string>("stream_path", "");
    declare_parameter<int>("assemble_delay_ms", 200);  // reserved
    declare_parameter<std::string>("observation_topic", "/gnss/observation");
    declare_parameter<std::string>("ephemeris_topic", "/gnss/ephemeris");
    declare_parameter<double>("ephemeris.snapshot_period_s", 30.0);
    declare_parameter<double>("ephemeris.max_age_s", 0.0);  // 0 = keep all

    eph_store_.setSnapshotPeriod(get_parameter("ephemeris.snapshot_period_s").as_double());
    eph_store_.setMaxAge(get_parameter("ephemeris.max_age_s").as_double());

    const auto stream_path = get_parameter("stream_path").as_string();
    if (stream_path.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'stream_path' is required but not set. "
        "Example: --ros-args -p stream_path:=\"ntrip://user:pass@host:port/mountpoint\"");
      throw std::runtime_error("stream_path parameter is required");
    }

    obs_pub_ = create_publisher<gnss_ros_standardization::msg::GnssObservations>(get_parameter("observation_topic").as_string(), 10);
    nav_pub_ = create_publisher<gnss_ros_standardization::msg::GnssEphemerides>(get_parameter("ephemeris_topic").as_string(), rclcpp::QoS(1).transient_local());

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
    int cnt_G=0,cnt_R=0,cnt_E=0,cnt_J=0,cnt_C=0,cnt_I=0,cnt_S=0,cnt_U=0;
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
      const std::string syscode = gnss_utils::systemCode(sys);

      for (int kf = 0; kf < NFREQ + NEXOBS; ++kf) {
        const bool empty = (o->P[kf] == 0.0) && (o->L[kf] == 0.0) &&
                           (o->D[kf] == 0.0) && (o->SNR[kf] == 0);
        if (empty) continue;

        gnss_ros_standardization::msg::GnssObservation obs = gnss_utils::obsToMsg(*o, kf);

        epoch.observations.push_back(std::move(obs));
      }

      switch (sys) {
        case SYS_GPS: ++epoch.cnt_G; break;
        case SYS_GLO: ++epoch.cnt_R; break;
        case SYS_GAL: ++epoch.cnt_E; break;
        case SYS_QZS: ++epoch.cnt_J; break;
        case SYS_CMP: ++epoch.cnt_C; break;
        case SYS_IRN: ++epoch.cnt_I; break;
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
        "obs published: week=%d tow=%.3f num=%zu sats(G/R/E/J/C/I/S/U)=(%d/%d/%d/%d/%d/%d/%d/%d)",
        epoch.week, epoch.tow, msg.observations.size(),
        epoch.cnt_G, epoch.cnt_R, epoch.cnt_E, epoch.cnt_J, epoch.cnt_C, epoch.cnt_I, epoch.cnt_S, epoch.cnt_U);

      it = epochs_.erase(it);
    }
  }

  // ---- ephemerides (snapshot-on-change + heartbeat) ------------------------
  void publishEphemeridesIfChanged() {
    if (rtcm_.nav.ng == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No GLONASS ephemerides (RTCM 1020). Check base settings.");
    }

    bool changed = false;
    for (int i = 0; i < rtcm_.nav.n; ++i) {
      changed = eph_store_.ingestEph(rtcm_.nav.eph[i]) || changed;
    }
    for (int i = 0; i < rtcm_.nav.ng; ++i) {
      changed = eph_store_.ingestGeph(rtcm_.nav.geph[i]) || changed;
    }

    if (changed || eph_store_.heartbeatDue(now())) {
      auto out = eph_store_.buildSnapshot(now());
      nav_pub_->publish(out);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "nav published: GNSS=%zu GLO=%zu",
        out.gnss_ephemeris.size(), out.glonass_ephemeris.size());
    }
  }

  // ---- builders ------------------------------------------------------------


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

  // Unified ephemeris store
  gnss_utils::EphemerisStore eph_store_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RtcmDecoderNode>());
  rclcpp::shutdown();
  return 0;
}
