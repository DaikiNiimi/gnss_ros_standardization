#include "gnss_ros_standardization/receiver_health.hpp"
#include "gnss_ros_standardization/receiver_session.hpp"
#include "gnss_ros_standardization/binary_framer.hpp"
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstring>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include <sensor_msgs/msg/imu.hpp>

#include "gnss_ros_standardization/decoder_common.hpp"
#include "gnss_ros_standardization/ephemeris_store.hpp"
#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/ins_utils.hpp"
#include "gnss_ros_standardization/ubx_protocol.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"

using namespace std::chrono_literals;

namespace ubx = gnss_ros_standardization::ubx;
namespace ins = gnss_ros_standardization::ins_utils;

namespace gnss_ros_standardization {

namespace {

/// Timer interval for stream polling
constexpr auto kTimerInterval = 10ms;

/// Watchdog timeout for incomplete PVT epochs (seconds)


}  // namespace

/// @brief ROS 2 node for decoding u-blox UBX protocol messages
///
/// Decodes UBX-RXM-RAWX (raw observations) and UBX-RXM-SFRBX (navigation data)
/// messages from u-blox receivers and publishes them as standardized ROS messages.
/// Also parses NMEA sentences for GnssSolution and UBX-ESF-INS for calibrated IMU data.
class UbxDecoderNode : public rclcpp::Node {
 public:
  UbxDecoderNode() : Node("ubx_decoder_node") {
    initializeParameters();
    initializePublishers();
    initializeDecoder();
    openStream();
    openRtcmRelay();
    startPolling();

    RCLCPP_INFO(get_logger(), "UBX Decoder initialized");
  }

  ~UbxDecoderNode() override {
    try {
      strclose(&stream_);
      relay_.close();
    } catch (...) {
      // Ignore errors during cleanup
    }
    free_raw(&raw_);
  }

 private:
  // Initialization

  decoder_common::EpochFlushPolicy epoch_flush_;
  wire::ReceiverHealth receiver_health_;
  void initializeParameters() {
    receiver_health_.configure(5.0);
    receiver_health_.limits(MAXOBS, NFREQ, NEXOBS, MAXPRNCMP);
    epoch_flush_.configure(0.05);
    const double tick_s = declare_parameter<double>("imu.sttag_tick_s", ubx::esf_raw::STTAG_TICK_S);
    if (!std::isfinite(tick_s) || tick_s <= 0.0 || tick_s >= 1.0)
      throw std::invalid_argument("imu.sttag_tick_s must be finite and in (0, 1)");
    imu_clock_ = ins::SensorTickClock(ins::SensorTickClock::Config{tick_s, 1ull << 32});
    RCLCPP_INFO(get_logger(), "ESF-RAW uses estimated time: tick=%.9g s, GNSS anchor expiry=2 s", tick_s);

    declare_parameter<std::string>("stream_path", "serial:///dev/ttyACM0:115200");
    // frame_id: ROS TF frame name attached to the GNSS antenna phase center.
    // Default "gnss_link" — override in launch to integrate with vehicle TF tree.
    declare_parameter<std::string>("frame_id", "gnss_link");
    declare_parameter<std::string>("topics.observation", "/gnss/observation");
    declare_parameter<std::string>("topics.ephemeris", "/gnss/ephemeris");
    declare_parameter<std::string>("topics.solution", "/gnss/nmea_solution");
    declare_parameter<std::string>("topics.imu",     "/gnss/imu/data");
    declare_parameter<std::string>("topics.imu_raw", "/gnss/imu/data_raw");
    declare_parameter<double>("ephemeris.snapshot_period_s", 30.0);
    declare_parameter<double>("ephemeris.max_age_s", 0.0);  // 0 = keep all
    declare_parameter<bool>("use_gps_timestamp", false);
    // RTCM relay listen URI (opt-in, see openRtcmRelay). Empty = disabled.
    declare_parameter<std::string>("rtcm_input.listen", "");
    eph_store_.setSnapshotPeriod(get_parameter("ephemeris.snapshot_period_s").as_double());
    eph_store_.setMaxAge(get_parameter("ephemeris.max_age_s").as_double());
    constexpr int history_limit = 8;
    eph_store_.setHistoryPerSatellite(static_cast<size_t>(history_limit));

    frame_id_ = get_parameter("frame_id").as_string();
    use_gps_timestamp_ = get_parameter("use_gps_timestamp").as_bool();
  }

  void initializePublishers() {
    decoder_common::configureNmeaDate(*this, nmea_parser_);
    obs_pub_     = create_publisher<msg::GnssObservations>(get_parameter("topics.observation").as_string(), 10);
    eph_pub_     = create_publisher<msg::GnssEphemerides>(get_parameter("topics.ephemeris").as_string(), rclcpp::QoS(256).transient_local()  /* depth 256: latch full ephemeris set, not just the last satellite (per-sat messages) */);
    sol_pub_     = create_publisher<msg::GnssSolution>(get_parameter("topics.solution").as_string(), 10);
    imu_pub_     = create_publisher<sensor_msgs::msg::Imu>(get_parameter("topics.imu").as_string(), decoder_common::kImuPublisherDepth);
    imu_raw_pub_ = create_publisher<sensor_msgs::msg::Imu>(get_parameter("topics.imu_raw").as_string(), decoder_common::kImuPublisherDepth);

    const auto origin = gnss_utils::declareLocalOrigin(*this);
    if (origin.fixed) {
      std::copy_n(origin.llh, 3, local_origin_pos_);
      std::copy_n(origin.ecef, 3, local_origin_ecef_);
      has_local_origin_ = true;
      RCLCPP_INFO(get_logger(), "ENU origin set from config: lat=%.6f lon=%.6f alt=%.2f",
        local_origin_pos_[0] * R2D, local_origin_pos_[1] * R2D, local_origin_pos_[2]);
    }
    logConfiguration();
  }

  void logConfiguration() {
    auto on = [](bool v) { return v ? "ON" : "OFF"; };
    RCLCPP_INFO(get_logger(), "Configuration:");
    RCLCPP_INFO(get_logger(), "  GPS timestamp : %s", on(use_gps_timestamp_));
    if (has_local_origin_) {
      RCLCPP_INFO(get_logger(), "  ENU origin    : fixed (lat=%.6f lon=%.6f alt=%.2f)",
        local_origin_pos_[0] * R2D, local_origin_pos_[1] * R2D, local_origin_pos_[2]);
    } else {
      RCLCPP_INFO(get_logger(), "  ENU origin    : auto (first valid solution)");
    }
  }

  void initializeDecoder() {
    const auto rawx_layout = declare_parameter<std::string>("rawx_layout", "auto");
    const char* layout_option = ubx::rawxLayoutOption(rawx_layout);
    if (init_raw(&raw_, STRFMT_UBX) != 1) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize raw decoder");
      throw std::runtime_error("init_raw failed");
    }
    // Preserve RAWX sigId (e.g. L2 CL=2L, L2 CM=2S), rather than labelling
    // individual signals as combined 2X/5X measurements. resetRaw keeps options.
    std::strcpy(raw_.opt, "-EPHALL -MULTICODE");
    std::strcat(raw_.opt, layout_option);
  }

  void startPolling() {
    timer_ = create_wall_timer(kTimerInterval, std::bind(&UbxDecoderNode::pollStream, this));
  }

  // Stream Management

  void openStream() {
    // The RTCM relay writes corrections back down this stream, so it needs a
    // read-write open; without relay keep the passive read-only open.
    const int mode = get_parameter("rtcm_input.listen").as_string().empty()
                     ? STR_MODE_R : STR_MODE_RW;
    if (!decoder_common::openStreamPath(get_logger(), stream_,
                                        get_parameter("stream_path").as_string(),
                                        mode, "Input")) {
      throw std::runtime_error("stropen failed");
    }
  }

  // RTCM relay (RTKLIB STRSVR-style): host a TCP server (recommended
  // "tcpsvr://:5556", fed by rtcm_decoder_node's rtcm_relay) and write incoming
  // corrections to the receiver stream in pollStream() for on-chip RTK. No
  // receiver command is sent; the port's inProtoMask must already include
  // RTCM3 (F9P default) — see src/decoders/README.md.
  void openRtcmRelay() {
    const std::string listen = get_parameter("rtcm_input.listen").as_string();
    if (listen.empty()) return;
    if (get_parameter("stream_path").as_string().rfind("file://", 0) == 0) {
      RCLCPP_WARN(get_logger(),
        "rtcm_relay is set but stream_path is a file:// stream — forwarded "
        "corrections have no receiver to reach.");
    }
    if (!relay_.open(get_logger(), listen)) {
      throw std::runtime_error("RTCM relay stropen failed");
    }
  }

  wire::StreamSession stream_session_;
  wire::ReceiverTimeline receiver_timeline_;
  void resetReceiverSession(bool disconnected) {
    // Release validated pending data before retiring the old session. Partial
    // wire frames and compressed references are never carried across sessions.
    flushPending();
    wire::resetRaw(raw_,STRFMT_UBX);
    if (disconnected) wire_framer_.reset();
    receiver_health_.resetSession();
    receiver_timeline_.reset();
    nmea_parser_.reset(); nmea_solution_=msg::GnssSolution(); binary_solution_=msg::GnssSolution();
    pending_=decltype(pending_)(); last_dop_={};
    prev_pvt_tow_ms_=UINT32_MAX; pvt_period_ms_=0;
    last_flushed_week_=0; last_flushed_tow_ms_=UINT32_MAX;
    eph_store_.clear(); publishSnapshot();
    source_=SolutionSource::UNDETERMINED; grace_started_=false; saw_binary_during_grace_=false;
    imu_clock_.reset(); gnss_anchor_.reset(); imu_offset_clock_.reset();
    last_raw_imu_ns_=-1; has_gnss_anchor_=false; imu_gps_warned_=false;
    gnss_anchor_offset_s_=0; gnss_anchor_arrival_ns_=0;
    has_nav_clock_=false; last_nav_clock_={}; pvt_time_valid_=false; nav_time_ref_.reset(); observation_time_ref_.reset();
    imu_clock_drift_reported_=false;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Receiver session reset: %s; waiting for fresh time/navigation references",
                disconnected ? "connection lost" : "measurement rewind or clock reset");
  }

  void pollStream() {
    // RTCM relay: drain corrections from the TCP server and write them to the
    // receiver stream. Same poll thread as the read below, so no lock.
    relay_.drainTo(get_logger(), *get_clock(), stream_);

    uint8_t buffer[ubx::READ_BUFFER_SIZE];
    const int bytes_read = strread(&stream_, buffer, sizeof(buffer));
    if (stream_session_.poll(stream_)) resetReceiverSession(true);
    const auto wire_now = std::chrono::steady_clock::now();
    receiver_health_.input(bytes_read);
    receiver_health_.framing(wire_framer_);
    receiver_health_.pvtPending(pending_.has_pvt || pending_.cov_received);
    receiver_health_.report(get_logger());
    if (bytes_read > 0) wire_last_input_ = wire_now;
    else if (wire_now - wire_last_input_ > std::chrono::seconds(1)) {
      wire_framer_.expire([this](const wire::Frame& f) { dispatchFrame(f); });
      wire_last_input_ = wire_now;
    }
    if (wire_framer_.rejected() != wire_last_rejected_) {
      wire_last_rejected_ = wire_framer_.rejected();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Rejected binary frame/header count: %llu",
          static_cast<unsigned long long>(wire_last_rejected_));
    }


    for (int i = 0; i < bytes_read; ++i) {
      parseUbxByte(buffer[i]);
    }

    maybePublishHeartbeat();
    commitSourceLockIfDue();  // ensure grace finalizes even on idle stream
    maybeWatchdogFlushPendingPvt();
  }

  // UBX mixed stream decoder (shared boundary with RTKLIB, for ESF and NAV blocks)

  void dispatchFrame(const wire::Frame& f) {
      auto diagnostic_scope = receiver_health_.backendScope();
      receiver_health_.frame((unsigned(f.cls)<<16)|f.id, f.revision);
      receiver_health_.inspect(wire::Protocol::Ubx,f,std::strstr(raw_.opt,"-RAWX0")!=nullptr);
      if (receiver_timeline_.frame(wire::Protocol::Ubx,f)) {
        resetReceiverSession(false);
        receiver_timeline_.frame(wire::Protocol::Ubx,f);
      }
      for (uint8_t byte : f.bytes) {
        const int status = input_ubx(&raw_, byte);
        // A successfully decoded RAW/RAWX dates the receiver even with zero
        // observations. Do not refresh from unrelated, weekless messages.
        if (status==1 && f.cls==2 && (f.id==0x15 || f.id==0x10))
          observation_time_ref_.update(raw_.time);
        handleDecodeResult(status);
      }
      ubx_cls_ = f.cls;
      ubx_id_ = static_cast<uint8_t>(f.id);
      ubx_payload_ = f.payload;
      handleUbxFrame();
  }

  void parseUbxByte(uint8_t byte) {
    wire_framer_.feed(byte, [this](const wire::Frame& f) { dispatchFrame(f); },
        [this](const std::string& sentence) { handleNmeaSentence(sentence); });
  }

  void handleUbxFrame() {
    if (ubx_cls_ == ubx::CLASS_ESF && ubx_id_ == ubx::ID_ESF_RAW) handleEsfRaw();
    if (ubx_cls_ == ubx::CLASS_ESF && ubx_id_ == ubx::ID_ESF_INS) handleEsfIns();
    if (ubx_cls_ == ubx::CLASS_NAV && ubx_id_ == ubx::ID_NAV_PVT)     handleNavPvt();
    if (ubx_cls_ == ubx::CLASS_NAV && ubx_id_ == ubx::ID_NAV_DOP)     handleNavDop();
    if (ubx_cls_ == ubx::CLASS_NAV && ubx_id_ == ubx::ID_NAV_CLOCK)   handleNavClock();
    if (ubx_cls_ == ubx::CLASS_NAV && ubx_id_ == ubx::ID_NAV_COV)     handleNavCov();
    if (ubx_cls_ == ubx::CLASS_NAV && ubx_id_ == ubx::ID_NAV_POSECEF) handleNavPosEcef();
    if (ubx_cls_ == ubx::CLASS_NAV && ubx_id_ == ubx::ID_NAV_VELECEF) handleNavVelEcef();
    if (ubx_cls_ == ubx::CLASS_NAV && ubx_id_ == ubx::ID_NAV_HPPOSECEF) handleNavHpPosEcef();
    if (ubx_cls_ == ubx::CLASS_NAV && ubx_id_ == ubx::ID_NAV_HPPOSLLH)  handleNavHpPosLlh();
  }

  // Solution source policy: publish nothing for kGracePeriodSec; binary PVT
  // seen during grace → lock BINARY, else lock NMEA. Deterministic and
  // PVT-preferred regardless of arrival order.
  enum class SolutionSource { UNDETERMINED, BINARY, NMEA };
  // 1.0 s covers a 1 Hz PVT cadence; higher rates lock BINARY in milliseconds.
  static constexpr double kGracePeriodSec = 1.0;

  void startGraceIfNeeded() {
    if (!grace_started_) {
      grace_start_ = std::chrono::steady_clock::now();
      grace_started_ = true;
    }
  }

  void commitSourceLockIfDue() {
    if (source_ != SolutionSource::UNDETERMINED) return;
    if (!grace_started_) return;
    const bool elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - grace_start_).count() >= kGracePeriodSec;
    if (saw_binary_during_grace_) {
      source_ = SolutionSource::BINARY;
      RCLCPP_INFO(get_logger(), "Solution source locked: BINARY (PVT detected during grace)");
    } else if (elapsed) {
      source_ = SolutionSource::NMEA;
      RCLCPP_INFO(get_logger(), "Solution source locked: NMEA (no PVT during grace)");
    }
  }

  // PVT TOW aggregation: buffer NAV-* blocks per iTOW epoch and flush when the
  // learned block set is complete (cov_ever_seen is sticky — it self-calibrates
  // to whichever blocks the receiver actually emits).
  static constexpr uint8_t COV_BIT_COV       = 0x1;
  static constexpr uint8_t COV_BIT_POSECEF   = 0x2;
  static constexpr uint8_t COV_BIT_VELECEF   = 0x4;
  static constexpr uint8_t COV_BIT_HPPOSLLH  = 0x8;
  static constexpr uint8_t COV_BIT_HPPOSECEF = 0x10;

  struct PendingPvt {
    uint32_t tow_ms{UINT32_MAX};
    bool has_pvt{false};
    bool has_pos_cov{false};
    bool has_vel_cov{false};
    uint8_t cov_received{0};
    uint8_t cov_ever_seen{0};
    msg::GnssSolution buf{};
    // High-precision scratch (NAV-HPPOSLLH / NAV-HPPOSECEF). Kept outside buf
    // and applied at flush time: NAV-PVT writes lat/lon/alt unconditionally,
    // so writing HP values into buf would make the result depend on in-epoch
    // arrival order.
    ubx::nav_hpposllh::Result hp_llh{};
    double hp_ecef[3]{0.0, 0.0, 0.0};
  };

  gnss_utils::DopCache last_dop_;
  ubx::nav_clock::Result last_nav_clock_{};
  bool has_nav_clock_{false};
  uint32_t prev_pvt_tow_ms_{UINT32_MAX};
  uint32_t pvt_period_ms_{0};

  // After a publish, record the (week, tow) to suppress orphan re-publish.
  uint32_t last_flushed_week_{0};
  uint32_t last_flushed_tow_ms_{UINT32_MAX};

  // offset: iTOW position — 0 for most NAV-* payloads, OFFSET_ITOW (4) for the
  // HP messages; the wrong offset breaks TOW grouping (stalls until watchdog).
  static uint32_t readItowMs(const uint8_t* p, size_t len, size_t offset = 0) {
    if (len < offset + 4) return UINT32_MAX;
    uint32_t v = 0;
    std::memcpy(&v, p + offset, 4);
    return v;
  }


  // Epoch-gap diagnostic: learns the nominal cadence as the smallest positive
  // epoch delta seen so far, and warns when a delta exceeds 2.5x that cadence —
  // i.e., whole epochs are missing from the stream (link loss or the receiver
  // skipping epochs). last_tow_s / min_period_s are caller-owned state;
  // a backwards tow (week rollover) only reseeds the state.
  void warnOnEpochGap(const char* what, double tow_s,
                      double& last_tow_s, double& min_period_s) {
    if (last_tow_s >= 0.0 && tow_s > last_tow_s) {
      const double dt = tow_s - last_tow_s;
      if (min_period_s == 0.0 || dt < min_period_s) min_period_s = dt;
      if (dt > min_period_s * 2.5) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "%s epoch gap: %.1fs (expected %.3fs) — epochs missing from the stream",
          what, dt, min_period_s);
      }
    }
    last_tow_s = tow_s;
  }

  // Orphan guard: same TOW as the last publish → late arrival from an eager
  // flush. Update cov_ever_seen only; re-accumulating would duplicate-publish.
  // Week falls back to TOW-only while this epoch has no PVT (time_week == 0).
  bool isLateOrphan(uint32_t tow, uint8_t cov_bit) {
    const uint32_t week = pending_.buf.time_week;
    if (last_flushed_tow_ms_ != UINT32_MAX &&
        tow == last_flushed_tow_ms_ &&
        (week == 0 || week == last_flushed_week_)) {
      pending_.cov_ever_seen |= cov_bit;
      GNSS_RX_EVENT(GNSS_RX_DUPLICATE_INPUT,1); return true;
    }
    return false;
  }

  void handleNavPvt() {
    startGraceIfNeeded();
    const uint32_t tow = readItowMs(ubx_payload_.data(), ubx_payload_.size());
    if (isLateOrphan(tow, 0)) return;
    if (tow != pending_.tow_ms && (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = tow;
    if (!ubx::pvt::parseNavPvt(ubx_payload_.data(), ubx_payload_.size(),
                               pending_.buf, pending_.has_pos_cov, pending_.has_vel_cov)) {
      GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return;
    }
    pending_.has_pvt = true;
    nav_time_ref_.reset();  // Do not retain a previous epoch's UTC validity.
    const auto* utc = ubx_payload_.data();
    if (ubx::pvt::navPvtTimeIsValid(utc, ubx_payload_.size())) {
      const int year = wire::u16(utc + 4);
      uint32_t week=0; double tow=0;
      if (gnss_utils::nmeaUtcToGpsTime(year,utc[6],utc[7],
          utc[8]*10000.+utc[9]*100.+utc[10],week,tow)) {
        nav_time_ref_.update(gpst2time(static_cast<int>(week),tow));
      }
    }
    // NAV-PVT's own time-validity flags, kept for the GNSS time anchor.
    pvt_time_valid_ =
        ubx::pvt::navPvtTimeIsValid(ubx_payload_.data(), ubx_payload_.size());
    epoch_flush_.touch();
    saw_binary_during_grace_ = true;
    warnOnEpochGap("NAV-PVT", tow * 1e-3,
                   pvt_gap_last_tow_s_, pvt_gap_min_period_s_);
    commitSourceLockIfDue();
    // Flush at the epoch boundary or shared quiet-period deadline.
  }

  // NAV-DOP: cached in last_dop_; applied at flush by applyDopWithStaleness().
  // NAV-DOP carries no GPS week → last_dop_.week = 0 (helper skips week check).
  void handleNavDop() {
    startGraceIfNeeded();
    const uint32_t tow = readItowMs(ubx_payload_.data(), ubx_payload_.size());
    msg::GnssSolution scratch{};
    if (!ubx::nav_dop::parseNavDop(ubx_payload_.data(), ubx_payload_.size(),
                                   scratch)) {
      GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return;
    }
    last_dop_.valid  = true;
    last_dop_.week   = 0;       // NAV-DOP has no week field
    last_dop_.tow_ms = tow;
    last_dop_.gdop   = scratch.gdop;
    last_dop_.pdop   = scratch.pdop;
    last_dop_.hdop   = scratch.hdop;
    last_dop_.vdop   = scratch.vdop;
    commitSourceLockIfDue();
  }

  // NAV-CLOCK: the receiver's own clock solution (iTOW + clkB, the clock's
  // bias against GPS time, + tAcc). Not part of the published solution; it is
  // kept as the preferred anchor for tying the free-running ESF-RAW sTtag
  // timeline to GPS time, because it states the clock model rather than only
  // the epoch. tAcc gates it: the Integration manual warns that iTOW values
  // may not be valid.
  void handleNavClock() {
    startGraceIfNeeded();
    ubx::nav_clock::Result r{};
    if (!ubx::nav_clock::parseNavClock(ubx_payload_.data(), ubx_payload_.size(),
                                       r)) {
      GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return;
    }
    last_nav_clock_ = r;
    has_nav_clock_ = true;
    commitSourceLockIfDue();
  }

  void handleNavCov() {
    startGraceIfNeeded();
    const uint32_t tow = readItowMs(ubx_payload_.data(), ubx_payload_.size());
    if (isLateOrphan(tow, COV_BIT_COV)) return;
    if (tow != pending_.tow_ms && (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = tow;
    if (!ubx::nav_cov::parseNavCov(ubx_payload_.data(), ubx_payload_.size(),
                                   pending_.buf)) {
      GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return;
    }
    pending_.has_pos_cov |= ubx_payload_[5] == 1;
    pending_.has_vel_cov |= ubx_payload_[6] == 1;
    pending_.cov_received |= COV_BIT_COV;
    pending_.cov_ever_seen |= COV_BIT_COV;
    epoch_flush_.touch();
    commitSourceLockIfDue();
    // Flush at the epoch boundary or shared quiet-period deadline.
  }

  void handleNavPosEcef() {
    startGraceIfNeeded();
    const uint32_t tow = readItowMs(ubx_payload_.data(), ubx_payload_.size());
    if (isLateOrphan(tow, COV_BIT_POSECEF)) return;
    if (tow != pending_.tow_ms && (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = tow;
    if (!ubx::nav_posecef::parseNavPosEcef(ubx_payload_.data(), ubx_payload_.size(),
                                           pending_.buf)) {
      GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return;
    }
    pending_.cov_received  |= COV_BIT_POSECEF;
    pending_.cov_ever_seen |= COV_BIT_POSECEF;
    epoch_flush_.touch();
    commitSourceLockIfDue();
    // Flush at the epoch boundary or shared quiet-period deadline.
  }

  void handleNavVelEcef() {
    startGraceIfNeeded();
    const uint32_t tow = readItowMs(ubx_payload_.data(), ubx_payload_.size());
    if (isLateOrphan(tow, COV_BIT_VELECEF)) return;
    if (tow != pending_.tow_ms && (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = tow;
    if (!ubx::nav_velecef::parseNavVelEcef(ubx_payload_.data(), ubx_payload_.size(),
                                           pending_.buf)) {
      GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return;
    }
    pending_.cov_received  |= COV_BIT_VELECEF;
    pending_.cov_ever_seen |= COV_BIT_VELECEF;
    epoch_flush_.touch();
    commitSourceLockIfDue();
    // Flush at the epoch boundary or shared quiet-period deadline.
  }

  void handleNavHpPosEcef() {
    startGraceIfNeeded();
    const uint32_t tow = readItowMs(ubx_payload_.data(), ubx_payload_.size(),
                                    ubx::nav_hpposecef::OFFSET_ITOW);
    if (isLateOrphan(tow, COV_BIT_HPPOSECEF)) return;
    if (tow != pending_.tow_ms && (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = tow;
    if (!ubx::nav_hpposecef::parseNavHpPosEcef(ubx_payload_.data(), ubx_payload_.size(),
                                               pending_.hp_ecef)) {
      GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return;
    }
    pending_.cov_received  |= COV_BIT_HPPOSECEF;
    pending_.cov_ever_seen |= COV_BIT_HPPOSECEF;
    epoch_flush_.touch();
    commitSourceLockIfDue();
    // Flush at the epoch boundary or shared quiet-period deadline.
  }

  void handleNavHpPosLlh() {
    startGraceIfNeeded();
    const uint32_t tow = readItowMs(ubx_payload_.data(), ubx_payload_.size(),
                                    ubx::nav_hpposllh::OFFSET_ITOW);
    if (isLateOrphan(tow, COV_BIT_HPPOSLLH)) return;
    if (tow != pending_.tow_ms && (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = tow;
    if (!ubx::nav_hpposllh::parseNavHpPosLlh(ubx_payload_.data(), ubx_payload_.size(),
                                             pending_.hp_llh)) {
      GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return;
    }
    pending_.cov_received  |= COV_BIT_HPPOSLLH;
    pending_.cov_ever_seen |= COV_BIT_HPPOSLLH;
    epoch_flush_.touch();
    commitSourceLockIfDue();
    // Flush at the epoch boundary or shared quiet-period deadline.
  }

  void flushPending() {
    const uint8_t ever_seen = pending_.cov_ever_seen;
    const uint8_t recv      = pending_.cov_received;
    if (!pending_.has_pvt) {
      if (pending_.cov_received) receiver_health_.pvtIncompleteDiscarded();
      pending_ = {};
      pending_.cov_ever_seen = ever_seen;
      return;
    }
    // Snapshot ECEF-direct fields before LLH-driven derivation overwrites them.
    msg::GnssSolution ecef_direct = pending_.buf;
    binary_solution_ = pending_.buf;

    // HPPOSLLH override (highest-priority LLH source), applied before geometry
    // derivation. Its accuracy fields refresh the covariance diagonal unless
    // NAV-COV already provided the full 3x3 (explicit valid NAV-COV block).
    if (recv & COV_BIT_HPPOSLLH) {
      binary_solution_.latitude  = pending_.hp_llh.lat_deg;
      binary_solution_.longitude = pending_.hp_llh.lon_deg;
      binary_solution_.altitude  = pending_.hp_llh.alt_m;
      const bool pos_cov_from_nav_cov = pending_.has_pos_cov;
      if (!pos_cov_from_nav_cov) {
        const double h_var_per_axis =
            pending_.hp_llh.hacc_m * pending_.hp_llh.hacc_m * 0.5;
        binary_solution_.pos_enu_cov[0] = h_var_per_axis;
        binary_solution_.pos_enu_cov[4] = h_var_per_axis;
        binary_solution_.pos_enu_cov[8] =
            pending_.hp_llh.vacc_m * pending_.hp_llh.vacc_m;
      }
    }

    decoder_common::finalizeBinarySolutionGeometry(binary_solution_);

    // Derive ECEF covariance from ENU covariance via current-LLH rotation.
    const double lat = binary_solution_.latitude  * D2R;
    const double lon = binary_solution_.longitude * D2R;
    double n[9], e[9];
    std::copy(binary_solution_.pos_enu_cov.begin(),
              binary_solution_.pos_enu_cov.end(), n);
    gnss_utils::rotateCovarianceEnuToEcef(n, lat, lon, e);
    std::copy(std::begin(e), std::end(e),
              binary_solution_.pos_cov_ecef.begin());
    std::copy(binary_solution_.vel_enu_cov.begin(),
              binary_solution_.vel_enu_cov.end(), n);
    gnss_utils::rotateCovarianceEnuToEcef(n, lat, lon, e);
    std::copy(std::begin(e), std::end(e),
              binary_solution_.vel_cov_ecef.begin());

    // ECEF-direct overrides, by priority: HPPOSECEF > NAV-POSECEF > LLH-derived.
    if (recv & COV_BIT_HPPOSECEF) {
      binary_solution_.pos_ecef.x = pending_.hp_ecef[0];
      binary_solution_.pos_ecef.y = pending_.hp_ecef[1];
      binary_solution_.pos_ecef.z = pending_.hp_ecef[2];
    } else if (recv & COV_BIT_POSECEF) {
      binary_solution_.pos_ecef = ecef_direct.pos_ecef;
    }
    if (recv & COV_BIT_VELECEF) {
      binary_solution_.vel_ecef = ecef_direct.vel_ecef;
    }

    // PVT cadence auto-detection (used by DOP staleness gate).
    if (prev_pvt_tow_ms_ != UINT32_MAX) {
      const uint32_t dt = pending_.tow_ms - prev_pvt_tow_ms_;
      if (dt > 0 && dt < 10000) pvt_period_ms_ = dt;
    }
    prev_pvt_tow_ms_ = pending_.tow_ms;

    // DOP staleness gate (see gnss_utils::applyDopWithStaleness).
    gnss_utils::applyDopWithStaleness(binary_solution_, last_dop_,
                                      binary_solution_.time_week,
                                      pending_.tow_ms, pvt_period_ms_);

    if (source_ == SolutionSource::BINARY) publishSolution(binary_solution_);

    // Record this epoch as the last published (week, tow) for orphan-guard.
    // publishSolution() will have populated time_week via raw_.time mapping.
    last_flushed_week_   = binary_solution_.time_week;
    last_flushed_tow_ms_ = pending_.tow_ms;

    pending_ = {};
    pending_.cov_ever_seen = ever_seen;
  }

  void maybeWatchdogFlushPendingPvt() {
    if (!pending_.has_pvt && !pending_.cov_received) return;
    if (epoch_flush_.due()) {
      flushPending();
    }
  }

  // ---- GNSS time anchor for the ESF-RAW timeline -------------------------
  //
  // ESF-RAW carries no GNSS time, and u-blox publishes no field tying sTtag to
  // it: the Interface Description leaves sTtag's Scale and Unit blank, and
  // ESF-MEAS's calibTtag - the one field that would give receiver local time -
  // is specified as "Always set to zero" on this hardware. So the only way to
  // put the IMU on GPS time is the documented practice of enabling ESF-RAW
  // alongside NAV-PVT / NAV-CLOCK and ESTIMATING the offset from them.
  //
  // The estimate is the same one GnssEpochAligner already makes and is tested
  // on: the sliding-window minimum of (host arrival - GPS time of the epoch),
  // which is the host/GPS clock difference plus the receiver's minimum output
  // latency. Subtracting it from the sTtag timeline lands the IMU on UTC(GPST),
  // the same base the GNSS topics use when use_gps_timestamp is set.
  //
  // What this does NOT remove is that minimum latency difference: GNSS epochs
  // and IMU samples do not leave the receiver with the same delay, and u-blox
  // documents neither. NAV-CLOCK's clkB does not help here - it is the
  // receiver clock's bias against GPS time, not its output latency, and sTtag
  // has no documented relation to that clock either. What NAV-CLOCK does add
  // is tAcc, a quality gate on the epoch's time. Expect an O(10-100 ms)
  // systematic offset in ABSOLUTE terms; what a fusion node consumes - the
  // IMU/GNSS relative alignment - is what this fixes.
  void feedGnssAnchor(int week, double tow, const rclcpp::Time& arrival) {
    if (week <= 0 || !std::isfinite(tow) || tow < 0.0 || tow >= 604800.0) { GNSS_RX_EVENT(GNSS_RX_TIME,1); return; }
    if (!gnss_utils::isRosTimeRepresentable(gpst2time(week,tow))) { GNSS_RX_EVENT(GNSS_RX_TIME,1); return; }
    // NAV-CLOCK tAcc gate: the Integration manual warns that iTOW values may
    // not be valid, so do not anchor on an epoch the receiver itself calls
    // uncertain. 1 ms is generous - a receiver with time has tAcc in the tens
    // of nanoseconds - so this only excludes genuinely bad time.
    if (has_nav_clock_ && last_nav_clock_.time_acc_s > 1e-3) { GNSS_RX_EVENT(GNSS_RX_TIME,1); return; }
    const rclcpp::Time aligned =
        gnss_anchor_.align(static_cast<uint32_t>(week), tow, arrival);
    const int64_t meas_ns =
        gnss_utils::gpstToUtcRosTime(gpst2time(week, tow)).nanoseconds();
    const double offset_s = (aligned.nanoseconds() - meas_ns) / 1e9;
    if (has_gnss_anchor_ && std::fabs(offset_s - gnss_anchor_offset_s_) > 1.0) {
      imu_offset_clock_.reset(); last_raw_imu_ns_ = -1;
      RCLCPP_WARN(get_logger(), "GNSS anchor discontinuity: resetting estimated IMU time");
    }
    gnss_anchor_offset_s_ = offset_s;
    has_gnss_anchor_ = true;
    gnss_anchor_arrival_ns_ = arrival.nanoseconds();
    gnss_anchor_steady_ = std::chrono::steady_clock::now();
  }

  /// Map a host-clock ESF-RAW stamp onto UTC(GPST) using that offset.
  ins::SlewedTimeOffset imu_offset_clock_;
  int64_t last_raw_imu_ns_{-1};
  rclcpp::Time imuHostToGps(const rclcpp::Time& host) {
    return imu_offset_clock_.map(host, gnss_anchor_offset_s_);
  }

  // STTAG_TICK_S is not a documented constant: the ZED-F9R Interface
  // Description leaves both Scale and Unit blank for sTtag, so the value in
  // ubx_protocol.hpp is an assumption that a firmware or product change could
  // invalidate silently. The offset estimate drifting against the host clock
  // measures (nominal_tick / true_tick - 1) directly, which turns that
  // invisible failure into a log line. Report once when the window first
  // fills, then only while it stays implausible.
  void reportImuClockDrift() {
    if (!imu_clock_.windowFilled()) return;
    const double ppm = imu_clock_.driftRate() * 1e6;

    // A "drift" of whole percent is not a clock error at all: it means the
    // stream is not real-time paced, which is normal when replaying a
    // recorded log through file:// (minutes of receiver time handed over in
    // seconds). The tick constant cannot be judged against a clock that is
    // not running at wall speed, so say so rather than cry wolf.
    constexpr double kNotRealTimePpm = 50000.0;  // 5 %
    if (std::fabs(ppm) > kNotRealTimePpm) {
      if (!imu_clock_drift_reported_) {
        imu_clock_drift_reported_ = true;
        RCLCPP_INFO(get_logger(),
          "ESF-RAW stream is not real-time paced (%.0f %% off wall speed) — "
          "offline replay assumed; the sTtag tick diagnostic is skipped.",
          ppm / 1e4);
      }
      return;
    }

    if (!imu_clock_drift_reported_) {
      imu_clock_drift_reported_ = true;
      RCLCPP_INFO(get_logger(),
        "ESF-RAW sTtag clock: %.1f ppm vs the host clock", ppm);
    }
    if (std::fabs(ppm) > 100.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
        "ESF-RAW sTtag clock drifting at %.0f ppm vs the host — "
        "imu.sttag_tick_s may be wrong for this receiver/firmware; measured tick %.12g s.",
        ppm, get_parameter("imu.sttag_tick_s").as_double() * (1.0 + imu_clock_.driftRate()));
    }
  }

  // UBX-ESF-RAW: batched IMU samples, one Imu published per sTtag sample set
  // (see ubx::esf_raw::parseEsfRawSamples). ESF-RAW carries no GNSS time —
  // only the receiver-internal sTtag — so the topic is host-clock based.
  //
  // The sTtag is mapped through ins::SensorTickClock rather than used to
  // spread samples backwards from the read time. The old scheme anchored each
  // batch at now() and walked backwards by the batch span, which discarded
  // the receiver clock BETWEEN batches: whenever the arrival gap was shorter
  // than the batch span — routine with a 10 ms poll and ~10 ms batches, and
  // guaranteed when several frames land in one strread() — the next batch's
  // first sample was stamped before the previous batch's last. Measured on a
  // synthetic 100 Hz file:// replay: 25.8 % of intervals did not advance, and
  // dt ranged down to -30 ms. The clock keeps one monotone timeline instead,
  // and removes the minimum observed transport latency along the way.
  ubx::EsfRawAssembler esf_assembler_;
  void handleEsfRaw() {
    const auto samples = esf_assembler_.push(ubx_payload_);
    receiver_health_.imuPending(esf_assembler_.pending());
    if (esf_assembler_.pending()) GNSS_RX_EVENT(GNSS_RX_IMU_WAIT,1);
    if (ubx_payload_.size()<4 || (ubx_payload_.size()-4)%8) { GNSS_RX_EVENT(GNSS_RX_LENGTH,1); GNSS_RX_EVENT(GNSS_RX_IMU_REJECT,1); }
    if (samples.empty()) return;

    const rclcpp::Time arrival = now();
    std::string clock_warn;
    const auto stamps = ins::stampBatch(
        imu_clock_, samples, [](const ubx::esf_raw::Sample& s) {
          return static_cast<uint64_t>(s.sttag);
        }, arrival, &clock_warn);
    if (!clock_warn.empty() && clock_warn.find("clamped") == std::string::npos) {
      imu_offset_clock_.reset(); last_raw_imu_ns_ = -1;
    }
    if (!clock_warn.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "ESF-RAW sTtag clock re-seeded: %s", clock_warn.c_str());
    }
    reportImuClockDrift();

    // use_gps_timestamp applies to this topic too, so the IMU can never sit
    // on a different base from the GNSS topics of the same node. Without an
    // anchor yet, GPS mode withholds IMU samples until a fresh anchor exists.
    if (use_gps_timestamp_ && !has_gnss_anchor_ && !imu_gps_warned_) {
      imu_gps_warned_ = true;
      RCLCPP_WARN(get_logger(),
        "use_gps_timestamp is set but no GNSS epoch has anchored the ESF-RAW "
        "timeline yet — enable UBX-NAV-PVT, UBX-NAV-CLOCK or RXM-RAWX so the "
        "IMU can be put on GPS time.");
    }

    for (size_t i = 0; i < samples.size(); ++i) {
      const auto& s = samples[i];
      sensor_msgs::msg::Imu imu;
      const auto measurement_stamp = time_base_.stampPrecomputed(
          use_gps_timestamp_, has_gnss_anchor_ &&
          arrival.nanoseconds() >= gnss_anchor_arrival_ns_ &&
          arrival.nanoseconds() - gnss_anchor_arrival_ns_ <= 2000000000LL &&
          std::chrono::steady_clock::now()-gnss_anchor_steady_ <= std::chrono::seconds(2), has_gnss_anchor_ ? imuHostToGps(stamps[i]) : stamps[i], stamps[i],
          get_logger(), *get_clock());
    if (!measurement_stamp) { receiver_health_.timeWithheld(); return; }
      if (measurement_stamp->nanoseconds() <= last_raw_imu_ns_) { GNSS_RX_EVENT(GNSS_RX_TIME,1); continue; }
      last_raw_imu_ns_ = measurement_stamp->nanoseconds();
      imu.header.stamp = *measurement_stamp;
      imu.header.frame_id = frame_id_;

      imu.orientation_covariance[0] = -1.0;

      imu.angular_velocity.x = s.gyro[0];
      imu.angular_velocity.y = s.gyro[1];
      imu.angular_velocity.z = s.gyro[2];
      imu.linear_acceleration.x = s.accel[0];
      imu.linear_acceleration.y = s.accel[1];
      imu.linear_acceleration.z = s.accel[2];

      // Values above are always provided by ESF-RAW; only their covariance is
      // unknown -> all-zero, not "-1" (which claims the measurement is absent).
      auto unk = ins::makeZeroCovariance();
      std::copy(unk.begin(), unk.end(), imu.angular_velocity_covariance.begin());
      std::copy(unk.begin(), unk.end(), imu.linear_acceleration_covariance.begin());

      imu_raw_pub_->publish(imu);
    }
  }

  void handleEsfIns() {
    if (static_cast<int>(ubx_payload_.size()) < ubx::esf_ins::MIN_LEN) { GNSS_RX_EVENT(GNSS_RX_LENGTH,1); return; }

    // Validity bits in bitfield0 (bytes 0-3)
    uint32_t bitfield0 = 0;
    std::memcpy(&bitfield0, ubx_payload_.data() + ubx::esf_ins::OFFSET_BITFIELD, 4);
    const bool ang_valid = (bitfield0 & (0x7u << 8)) == (0x7u << 8);  // bits 8,9,10
    const bool acc_valid = (bitfield0 & (0x7u << 11)) == (0x7u << 11); // bits 11,12,13

    int32_t xAngRate_raw = 0, yAngRate_raw = 0, zAngRate_raw = 0;
    int32_t xAccel_raw = 0, yAccel_raw = 0, zAccel_raw = 0;
    std::memcpy(&xAngRate_raw, ubx_payload_.data() + ubx::esf_ins::OFFSET_XANGRATE, 4);
    std::memcpy(&yAngRate_raw, ubx_payload_.data() + ubx::esf_ins::OFFSET_YANGRATE, 4);
    std::memcpy(&zAngRate_raw, ubx_payload_.data() + ubx::esf_ins::OFFSET_ZANGRATE, 4);
    std::memcpy(&xAccel_raw,   ubx_payload_.data() + ubx::esf_ins::OFFSET_XACCEL,   4);
    std::memcpy(&yAccel_raw,   ubx_payload_.data() + ubx::esf_ins::OFFSET_YACCEL,   4);
    std::memcpy(&zAccel_raw,   ubx_payload_.data() + ubx::esf_ins::OFFSET_ZACCEL,   4);

    // Scale: angular rate 1e-3 deg/s → rad/s; acceleration 1e-2 m/s²
    const double deg2rad = M_PI / 180.0;

    // U2 pp278-279 / U3 pp71-72 define version 1 with this layout.
    // Keep legacy version 0 compatibility; do not accept unknown versions.
    sensor_msgs::msg::Imu imu;
    if (!ang_valid || !acc_valid || (ubx_payload_[0] > 1)) {
      GNSS_RX_EVENT(ubx_payload_[0]>1 ? GNSS_RX_VERSION : GNSS_RX_QUALITY,1); return;
    }
    const double tow = wire::u32(ubx_payload_.data() + ubx::esf_ins::OFFSET_ITOW) * 1e-3;
    int week = 0;
    const gtime_t ref = nav_time_ref_.get().time != 0 ? nav_time_ref_.get() : observation_time_ref_.get();
    const bool valid_time = decoder_common::resolveGpsWeekForTow(ref, tow, week);
    const auto measurement_stamp = time_base_.stamp(
        use_gps_timestamp_, valid_time, valid_time ? gpst2time(week, tow) : gtime_t{},
        now(), get_logger(), *get_clock());
    if (!measurement_stamp) { receiver_health_.timeWithheld(); return; }
    imu.header.stamp = *measurement_stamp;
    imu.header.frame_id = frame_id_;

    // orientation: not provided by ESF-INS — leave identity, mark unknown
    imu.orientation_covariance[0] = -1.0;

    imu.angular_velocity.x = xAngRate_raw * 1e-3 * deg2rad;
    imu.angular_velocity.y = yAngRate_raw * 1e-3 * deg2rad;
    imu.angular_velocity.z = zAngRate_raw * 1e-3 * deg2rad;
    auto ang_cov = ang_valid ? ins::makeZeroCovariance() : ins::makeUnknownCovariance();
    std::copy(ang_cov.begin(), ang_cov.end(), imu.angular_velocity_covariance.begin());

    imu.linear_acceleration.x = xAccel_raw * 1e-2;
    imu.linear_acceleration.y = yAccel_raw * 1e-2;
    imu.linear_acceleration.z = zAccel_raw * 1e-2;
    auto acc_cov = acc_valid ? ins::makeZeroCovariance() : ins::makeUnknownCovariance();
    std::copy(acc_cov.begin(), acc_cov.end(), imu.linear_acceleration_covariance.begin());

    imu_pub_->publish(imu);
  }

  // Message Handling

  void handleNmeaSentence(const std::string& sentence) {
    startGraceIfNeeded();
    if (!nmea_parser_.parseSentence(sentence, nmea_solution_)) return;
    commitSourceLockIfDue();
    if (source_ == SolutionSource::NMEA) {
      publishSolution(nmea_solution_);
    }
  }

  void handleDecodeResult(int result) {
    receiver_health_.backendResult(result);
    if (result < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Rejected receiver payload (invalid layout, time, version or content)");
      return;
    }
    switch (result) {
      case 1:  publishObservations(); break;
      case 2:  publishEphemerides();  break;
      case 9:  publishIonosphere();   break;
      default: break;
    }
  }

  // Solution Publishing

  void publishSolution(msg::GnssSolution& sol) {
    // BINARY: keep the epoch's own iTOW; raw_.time supplies only the GPS week
    // (UBX NAV-* payloads carry none). NMEA: NmeaParser filled week/tow.
    gtime_t t_gpst{};
    int week_for_stamp = 0;
    if (source_ == SolutionSource::BINARY) {
      const gtime_t ref = nav_time_ref_.get().time != 0 ? nav_time_ref_.get() : observation_time_ref_.get();
      if (decoder_common::resolveGpsWeekForTow(ref, sol.time_tow, week_for_stamp)) {
        sol.time_week = static_cast<uint32_t>(week_for_stamp);
        t_gpst = gpst2time(week_for_stamp, sol.time_tow);
      }
      sol.solution_source = msg::GnssSolution::SOLUTION_SOURCE_BINARY;
    } else {
      sol.solution_source = msg::GnssSolution::SOLUTION_SOURCE_NMEA;
      if (sol.time_week > 0) {
        week_for_stamp = sol.time_week;
        t_gpst = gpst2time(sol.time_week, sol.time_tow);
      }
    }

    // Anchor the ESF-RAW timeline from the solution epoch too. Gated on
    // NAV-PVT's own `valid` flags: the Integration manual warns that "iTOW
    // values may not be valid" and points at NAV-PVT precisely because it
    // reports that.
    const rclcpp::Time sol_arrival = now();
    if (source_ == SolutionSource::BINARY && week_for_stamp > 0 &&
        pvt_time_valid_) {
      feedGnssAnchor(week_for_stamp, sol.time_tow, sol_arrival);
    }

    const auto measurement_stamp = time_base_.stamp(
        use_gps_timestamp_, week_for_stamp > 0 &&
        (source_ != SolutionSource::BINARY || pvt_time_valid_),
        t_gpst, sol_arrival, get_logger(), *get_clock());
    if (!measurement_stamp) { receiver_health_.timeWithheld(); return; }
    sol.header.stamp = *measurement_stamp;
    sol.header.frame_id = frame_id_;

    if (decoder_common::finalizeReceiverSolution(
            sol, has_local_origin_, local_origin_ecef_, local_origin_pos_)) {
      RCLCPP_INFO(get_logger(), "Set local ENU origin: lat=%.6f lon=%.6f alt=%.2f",
                  local_origin_pos_[0] * R2D, local_origin_pos_[1] * R2D,
                  local_origin_pos_[2]);
    }

    sol_pub_->publish(sol);
  }

  // Observation Publishing

  void publishObservations() {
    if (raw_.obs.n <= 0) return;
    receiver_health_.observationCapacity(raw_.obs.n, MAXOBS);

    // obs.data[0].time, not raw_.time (already advanced to the next epoch).
    int week = 0;
    const gtime_t obs_time = raw_.obs.data[0].time;
    const double tow = time2gpst(obs_time, &week);

    warnOnEpochGap("Observation", tow,
                   obs_gap_last_tow_s_, obs_gap_min_period_s_);

    // Anchor the ESF-RAW timeline. RXM-RAWX epochs are RTKLIB-validated, so
    // they need no extra time-validity gate; now() is the host arrival the
    // offset estimate is built from, taken before any stamping.
    const rclcpp::Time obs_arrival = now();
    feedGnssAnchor(week, tow, obs_arrival);

    msg::GnssObservations msg;
    const auto measurement_stamp = time_base_.stamp(
        use_gps_timestamp_, week > 0,
        obs_time, obs_arrival, get_logger(), *get_clock());
    if (!measurement_stamp) { receiver_health_.timeWithheld(); return; }
    msg.header.stamp = *measurement_stamp;
    msg.header.frame_id = frame_id_;
    msg.week = static_cast<uint32_t>(week);
    msg.tow = tow;

    decoder_common::SatelliteCount sat_count{};
    for (int i = 0; i < raw_.obs.n; ++i) {
      const obsd_t& obs = raw_.obs.data[i];
      decoder_common::countSatellite(obs.sat, sat_count);
      decoder_common::appendObservations(obs, msg.observations, &raw_.nav);
    }

    obs_pub_->publish(msg);
    receiver_health_.published(msg.week, msg.tow, msg.observations.size());

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "Published obs: week=%d tow=%.3f n=%zu sats(G/R/E/J/C/I/S/U)=(%d/%d/%d/%d/%d/%d/%d/%d)",
      week, tow, msg.observations.size(),
      sat_count.gps, sat_count.glo, sat_count.gal, sat_count.qzs,
      sat_count.bds, sat_count.irn, sat_count.sbs, sat_count.unknown);
  }

  // Ephemeris Publishing

  void publishEphemerides() {
    bool changed = false;
    for (int i = 0; i < raw_.nav.n; ++i) {
      changed = eph_store_.ingestEph(raw_.nav.eph[i]) || changed;
    }
    for (int i = 0; i < raw_.nav.ng; ++i) {
      changed = eph_store_.ingestGeph(raw_.nav.geph[i]) || changed;
    }
    changed = eph_store_.ingestIon(raw_.nav) || changed;
    if (changed) publishSnapshot();
  }

  // Broadcast ionosphere parameters (backend status 9): part of the snapshot.
  void publishIonosphere() {
    if (eph_store_.ingestIon(raw_.nav)) publishSnapshot();
  }

  void maybePublishHeartbeat() {
    if (eph_store_.heartbeatDue(now())) publishSnapshot();
  }

  void publishSnapshot() {
    auto m = eph_store_.buildSnapshot(now());
    receiver_health_.snapshotRejected(eph_store_.lastSnapshotRejected());
    eph_pub_->publish(m);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "Published ephemeris snapshot: GNSS=%zu GLO=%zu",
      m.gnss_ephemeris.size(), m.glonass_ephemeris.size());
  }

  // Member Variables

  std::string frame_id_;
  bool        use_gps_timestamp_{false};

  rclcpp::Publisher<msg::GnssObservations>::SharedPtr   obs_pub_;
  rclcpp::Publisher<msg::GnssEphemerides>::SharedPtr    eph_pub_;
  rclcpp::Publisher<msg::GnssSolution>::SharedPtr       sol_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr   imu_pub_;       // ESF-INS (calibrated)
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr   imu_raw_pub_;   // ESF-RAW (uncalibrated)

  // sTtag -> ROS time for ESF-RAW. Nominal tick; the drift diagnostic above
  // is what catches a wrong constant.
  ins::SensorTickClock imu_clock_{ins::SensorTickClock::Config{
      ubx::esf_raw::STTAG_TICK_S, 1ull << 32}};
  bool imu_clock_drift_reported_{false};

  // GNSS time anchor for the ESF-RAW timeline (see feedGnssAnchor). The
  // estimator is GnssEpochAligner in TowAutoOffset mode, reused rather than
  // reimplemented because it is the tested one.
  gnss_utils::GnssEpochAligner gnss_anchor_{
      gnss_utils::GnssEpochAligner::Config{
          gnss_utils::GnssEpochAligner::Source::TowAutoOffset, 60.0}};
  double gnss_anchor_offset_s_{0.0};
  bool   has_gnss_anchor_{false};
  int64_t gnss_anchor_arrival_ns_{0};
  std::chrono::steady_clock::time_point gnss_anchor_steady_{};
  bool   imu_gps_warned_{false};
  bool   pvt_time_valid_{false};
  wire::FreshGnssReference nav_time_ref_, observation_time_ref_;
  rclcpp::TimerBase::SharedPtr timer_;

  wire::Framer wire_framer_{wire::Protocol::Ubx};
  std::chrono::steady_clock::time_point wire_last_input_{std::chrono::steady_clock::now()};
  uint64_t wire_last_rejected_{0};
  stream_t stream_{};
  raw_t    raw_{};


  // One time base per session, latched at the first published message and
  // never changed. Shared by EVERY topic this node publishes, which is what
  // structurally guarantees the IMU and the GNSS topics cannot end up on
  // different bases. See decoder_common::TimeBaseLatch.
  decoder_common::TimeBaseLatch time_base_;

  // RTCM relay (PC -> receiver): STRSVR-style TCP server -> receiver stream
  decoder_common::RtcmRelayServer relay_;

  // NMEA parsing state
  gnss_utils::NmeaParser nmea_parser_;
  msg::GnssSolution      nmea_solution_;
  msg::GnssSolution      binary_solution_;
  PendingPvt             pending_;
  SolutionSource         source_{SolutionSource::UNDETERMINED};
  std::chrono::steady_clock::time_point grace_start_{};
  bool                   grace_started_{false};
  bool                   saw_binary_during_grace_{false};

  // ENU origin
  bool   has_local_origin_{false};
  double local_origin_ecef_[3]{0.0};
  double local_origin_pos_[3]{0.0};

  // UBX decoded frame payload (ESF-INS / ESF-RAW)
  uint8_t              ubx_cls_{0};
  uint8_t              ubx_id_{0};
  std::vector<uint8_t> ubx_payload_;

  // Epoch-gap detection state (see warnOnEpochGap)
  double pvt_gap_last_tow_s_{-1.0};
  double pvt_gap_min_period_s_{0.0};
  double obs_gap_last_tow_s_{-1.0};
  double obs_gap_min_period_s_{0.0};

  // Ephemeris dedup
  gnss_utils::EphemerisStore eph_store_;
};

}  // namespace gnss_ros_standardization

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<gnss_ros_standardization::UbxDecoderNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("ubx_decoder_node"), "Startup/runtime failure: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
