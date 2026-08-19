// SPDX-License-Identifier: MIT
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "gnss_ros_standardization/decoder_common.hpp"
#include "gnss_ros_standardization/ephemeris_store.hpp"
#include "gnss_ros_standardization/gnss_utils.hpp"


using namespace std::chrono_literals;

namespace gnss_ros_standardization {

namespace {
constexpr uint8_t  RTCM3_PREAMBLE = 0xD3;
constexpr uint16_t RTCM3_MAX_LEN  = 1023;
constexpr size_t   RTCM3_HDR_LEN  = 3;     // preamble + length(2)
constexpr size_t   RTCM3_CRC_LEN  = 3;
constexpr uint32_t CRC24Q_POLY    = 0x1864CFB; // CRC-24Q

// Relay-output carry cap: about one minute of corrections at typical RTCM
// rates (2-5 KB/s). Overflow means the relay target has been down for minutes
// or the output link is persistently too slow; corrections that old are
// rejected by receivers via age-of-differential anyway, so the whole carry is
// dropped with a WARN (still gentler than STRSVR's immediate silent drop).
constexpr size_t RELAY_CARRY_MAX  = 256 * 1024;
} // namespace

/// @brief ROS 2 node for decoding RTCM3 messages from NTRIP/serial/file streams
///
/// Reads RTCM3 frames (preamble 0xD3 + length + payload + CRC-24Q) from the
/// configured stream and publishes observation epochs (/gnss/observation) and
/// ephemerides (/gnss/ephemeris) via RTKLIB's input_rtcm3() decoder.
class RtcmDecoderNode : public rclcpp::Node {
public:
  RtcmDecoderNode() : Node("rtcm_decoder_node") {
    initializeParameters();
    initializePublishers();
    initializeDecoder();
    openStream();
    openRtcmRelay();
    startPolling();
    RCLCPP_INFO(get_logger(), "RTCM Decoder started");
  }

  ~RtcmDecoderNode() override {
    try {
      strclose(&stream_);
      if (relay_enabled_) strclose(&relay_out_);
    } catch (...) {}
    free_rtcm(&rtcm_);
  }

private:
  // Initialization

  void initializeParameters() {
    declare_parameter<std::string>("stream_path", "");
    declare_parameter<std::string>("observation_topic", "/gnss/observation");
    declare_parameter<std::string>("ephemeris_topic", "/gnss/ephemeris");
    declare_parameter<double>("ephemeris.snapshot_period_s", 30.0);
    declare_parameter<double>("ephemeris.max_age_s", 0.0);  // 0 = keep all

    // RTCM relay output (opt-in, see openRtcmRelay). Empty = disabled.
    declare_parameter<std::string>("rtcm_relay", "");

    eph_store_.setSnapshotPeriod(get_parameter("ephemeris.snapshot_period_s").as_double());
    eph_store_.setMaxAge(get_parameter("ephemeris.max_age_s").as_double());

    if (get_parameter("stream_path").as_string().empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'stream_path' is required but not set. "
        "Example: --ros-args -p stream_path:=\"ntrip://user:pass@host:port/mountpoint\"");
      throw std::runtime_error("stream_path parameter is required");
    }
  }

  void initializePublishers() {
    obs_pub_ = create_publisher<gnss_ros_standardization::msg::GnssObservations>(
        get_parameter("observation_topic").as_string(), 10);
    nav_pub_ = create_publisher<gnss_ros_standardization::msg::GnssEphemerides>(
        get_parameter("ephemeris_topic").as_string(),
        rclcpp::QoS(256).transient_local()  /* depth 256: latch full ephemeris set, not just the last satellite (per-sat messages) */);
  }

  void initializeDecoder() {
    if (init_rtcm(&rtcm_) != 1) {
      RCLCPP_ERROR(get_logger(), "init_rtcm failed");
      throw std::runtime_error("init_rtcm failed");
    }
  }

  void startPolling() {
    timer_ = create_wall_timer(10ms, std::bind(&RtcmDecoderNode::onTimer, this));
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
    if (!decoder_common::openStreamPath(get_logger(), stream_,
                                        get_parameter("stream_path").as_string(),
                                        STR_MODE_R, "Input")) {
      throw std::runtime_error("stropen failed");
    }
  }

  // RTCM relay output (RTKLIB STRSVR-style): when `rtcm_relay` is set (e.g.
  // "tcpcli://127.0.0.1:5556"), forward raw source bytes verbatim to that
  // stream — typically the TCP server hosted by a receiver driver/decoder node
  // (ubx=5556 / sbf=5557 / novatel=5558), which writes them to the receiver
  // for on-chip RTK. A spare receiver port can also be fed directly
  // (e.g. "serial:///dev/ttyUSB2:230400"). Decoding to topics is unaffected.
  void openRtcmRelay() {
    const auto target = get_parameter("rtcm_relay").as_string();
    if (target.empty()) return;
    if (!decoder_common::openStreamPath(get_logger(), relay_out_, target,
                                        STR_MODE_RW, "RTCM relay")) {
      throw std::runtime_error("RTCM relay stropen failed");
    }
    relay_enabled_ = true;
    RCLCPP_INFO(get_logger(), "RTCM relay enabled: source -> '%s'", target.c_str());
  }

  void onTimer() {
    uint8_t buf[4096];
    const int n = strread(&stream_, buf, sizeof(buf));
    // STRSVR-style relay: forward the raw bytes verbatim toward the receiver
    // driver before (and independent of) local decoding. Runs on idle reads
    // too, so a pending carry keeps draining once the relay target recovers.
    // Unlike the receiver-side relay, source reads cannot be paused for
    // backpressure (that would stall topic decoding and NTRIP casters drop
    // slow clients), so the CarriedStreamWriter is capped instead.
    if (relay_enabled_) {
      const size_t before = relay_writer_.bytes;
      relay_writer_.write(get_logger(), *get_clock(), relay_out_,
                          n > 0 ? buf : nullptr, n > 0 ? n : 0);
      if (relay_writer_.bytes > before) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "RTCM relay: %zu bytes forwarded (total)",
                             relay_writer_.bytes);
      }
    }
    if (n > 0) {
      rx_.insert(rx_.end(), buf, buf + n);
      decodeRtcm3FromBuffer();
    }
    flushEpochs();
  }

  // ---- RTCM3 framing & decode ---------------------------------------------
  //
  // Read-offset + amortized compact strategy: rx_off_ tracks the read head into
  // rx_. Resync (preamble miss / CRC fail) is O(1) (just ++rx_off_), and erase
  // only fires when the unread tail exceeds half of rx_, yielding amortized O(n)
  // even on noisy streams (vs. O(n²) of per-byte erase(begin())).
  void decodeRtcm3FromBuffer() {
    while (true) {
      const size_t avail = rx_.size() - rx_off_;
      if (avail < RTCM3_HDR_LEN) break;
      const uint8_t* head = rx_.data() + rx_off_;
      if (head[0] != RTCM3_PREAMBLE) { ++rx_off_; continue; }

      const uint16_t plen = ((head[1] & 0x03) << 8) | head[2];
      if (plen > RTCM3_MAX_LEN) { ++rx_off_; continue; }

      const size_t flen = RTCM3_HDR_LEN + plen + RTCM3_CRC_LEN;
      if (avail < flen) break;  // wait for more data

      const uint32_t crc_calc = crc24q(head, flen - RTCM3_CRC_LEN);
      const uint32_t crc_recv =
          (uint32_t(head[flen - 3]) << 16) |
          (uint32_t(head[flen - 2]) << 8)  |
           uint32_t(head[flen - 1]);

      if (crc_calc != crc_recv) { ++rx_off_; continue; }

      // Feed the CRC-validated frame byte-by-byte; process each completion
      // inline. Standard RTCM3 yields one ret>0 per frame, but the inline
      // check guarantees that any middle-of-frame completion (variant MSM /
      // future spec extensions) is not silently dropped.
      for (size_t i = 0; i < flen; ++i) {
        const int r = input_rtcm3(&rtcm_, head[i]);
        if (r == 1 && rtcm_.obs.n > 0) accumulateObservations();
      }
      rx_off_ += flen;
      publishEphemeridesIfChanged();
    }
    // Amortized compact: only erase when read head has moved past half of rx_.
    if (rx_off_ * 2 > rx_.size()) {
      rx_.erase(rx_.begin(), rx_.begin() + static_cast<long>(rx_off_));
      rx_off_ = 0;
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
  };

  void accumulateObservations() {
    int week = 0;
    const double tow = time2gpst(rtcm_.time, &week);
    const int64_t tow_ms = static_cast<int64_t>(std::llround(tow * 1000.0));

    EpochKey key{week, tow_ms};
    auto& epoch = epochs_[key];
    epoch.week = week;
    epoch.tow = tow;

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
      msg.week = static_cast<uint32_t>(epoch.week);
      msg.tow  = epoch.tow;
      msg.observations = std::move(epoch.observations);

      obs_pub_->publish(msg);

      RCLCPP_INFO(get_logger(),
        "Published obs: week=%d tow=%.3f n=%zu sats(G/R/E/J/C/I/S/U)=(%d/%d/%d/%d/%d/%d/%d/%d)",
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
        "Published ephemeris snapshot: GNSS=%zu GLO=%zu",
        out.gnss_ephemeris.size(), out.glonass_ephemeris.size());
    }
  }

private:
  // pubs/timer
  rclcpp::Publisher<gnss_ros_standardization::msg::GnssObservations>::SharedPtr  obs_pub_;
  rclcpp::Publisher<gnss_ros_standardization::msg::GnssEphemerides>::SharedPtr   nav_pub_;
  rclcpp::TimerBase::SharedPtr                                         timer_;

  // stream/decoder
  stream_t  stream_{};
  rtcm_t    rtcm_{};

  // RTCM relay output (STRSVR-style): forward raw source bytes to a TCP client.
  // Capped carry absorbs partial/failed writes (see RELAY_CARRY_MAX).
  stream_t  relay_out_{};
  bool      relay_enabled_{false};
  decoder_common::CarriedStreamWriter relay_writer_{RELAY_CARRY_MAX};
  std::vector<uint8_t> rx_;
  size_t    rx_off_{0};   // read head into rx_ (see decodeRtcm3FromBuffer)

  // epoch buffer
  std::unordered_map<EpochKey, EpochBuffer, EpochKeyHash> epochs_;

  // Unified ephemeris store
  gnss_utils::EphemerisStore eph_store_;
};

}  // namespace gnss_ros_standardization

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gnss_ros_standardization::RtcmDecoderNode>());
  rclcpp::shutdown();
  return 0;
}