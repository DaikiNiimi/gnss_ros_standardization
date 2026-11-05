#include "gnss_ros_standardization/receiver_health.hpp"
#include "gnss_ros_standardization/receiver_session.hpp"
#include "gnss_ros_standardization/binary_framer.hpp"
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstring>
#include <limits>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <sensor_msgs/msg/imu.hpp>

#include "gnss_ros_standardization/decoder_common.hpp"
#include "gnss_ros_standardization/ephemeris_store.hpp"
#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/ins_utils.hpp"
#include "gnss_ros_standardization/novatel_protocol.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"

using namespace std::chrono_literals;
namespace ins = gnss_ros_standardization::ins_utils;

namespace gnss_ros_standardization {

namespace {

constexpr auto kTimerInterval  = 10ms;


}  // namespace

/// @brief ROS 2 node for decoding NovAtel receiver protocol messages
///
/// Decodes NovAtel binary format (OEM3/OEM4/OEM7) raw observations and navigation
/// data messages from NovAtel receivers and publishes them as standardized ROS messages.
/// Also parses NMEA sentences for GnssSolution and OEM4 IMU logs (RAWIMUSX, CORRIMUDATA).
///
/// Supported formats:
///   - oem4: OEM4/OEM6/OEM7 binary format
class NovatelDecoderNode : public rclcpp::Node {
 public:
  NovatelDecoderNode() : Node("novatel_decoder_node") {
    initializeParameters();
    initializePublishers();
    initializeDecoder();
    openStream();
    openRtcmRelay();
    startPolling();

    RCLCPP_INFO(get_logger(), "NovAtel Decoder initialized");
  }

  ~NovatelDecoderNode() override {
    try {
      strclose(&stream_);
      relay_.close();
      if (relay_out_open_) strclose(&relay_out_);
    } catch (...) {}
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
    declare_parameter<std::string>("stream_path", "serial:///dev/ttyUSB0:115200");
    // frame_id: ROS TF frame name attached to the GNSS antenna phase center.
    // Default "gnss_link" — override in launch to integrate with vehicle TF tree.
    declare_parameter<std::string>("frame_id", "gnss_link");
    declare_parameter<std::string>("topics.observation", "/gnss/observation");
    declare_parameter<std::string>("topics.ephemeris", "/gnss/ephemeris");
    declare_parameter<std::string>("topics.solution", "/gnss/nmea_solution");
    declare_parameter<std::string>("topics.imu",      "/gnss/imu/data");
    declare_parameter<std::string>("topics.imu_raw",  "/gnss/imu/data_raw");
    declare_parameter<double>("imu_scale_override.accel", 0.0);
    declare_parameter<double>("imu_scale_override.gyro",  0.0);
    declare_parameter<double>("ephemeris.snapshot_period_s", 30.0);
    declare_parameter<double>("ephemeris.max_age_s", 0.0);  // 0 = keep all
    declare_parameter<bool>("use_gps_timestamp", false);
    // RTCM relay (opt-in, see openRtcmRelay): listen URI for corrections plus
    // the dedicated RTCMV3 output port. See src/decoders/README.md.
    declare_parameter<std::string>("rtcm_input.listen", "");
    declare_parameter<std::string>("rtcm_input.receiver_stream", "");
    declare_parameter<std::string>("rtcm_input.receiver_port", "THISPORT");
    use_gps_timestamp_ = get_parameter("use_gps_timestamp").as_bool();
    eph_store_.setSnapshotPeriod(get_parameter("ephemeris.snapshot_period_s").as_double());
    eph_store_.setMaxAge(get_parameter("ephemeris.max_age_s").as_double());
    constexpr int history_limit = 8;
    eph_store_.setHistoryPerSatellite(static_cast<size_t>(history_limit));

    frame_id_ = get_parameter("frame_id").as_string();
  }

  void initializePublishers() {
    decoder_common::configureNmeaDate(*this, nmea_parser_);
    obs_pub_ = create_publisher<msg::GnssObservations>(get_parameter("topics.observation").as_string(), 10);
    eph_pub_ = create_publisher<msg::GnssEphemerides>(get_parameter("topics.ephemeris").as_string(), rclcpp::QoS(256).transient_local()  /* depth 256: latch full ephemeris set, not just the last satellite (per-sat messages) */);
    sol_pub_ = create_publisher<msg::GnssSolution>(get_parameter("topics.solution").as_string(), 10);
    imu_pub_     = create_publisher<sensor_msgs::msg::Imu>(get_parameter("topics.imu").as_string(), decoder_common::kImuPublisherDepth);
    imu_raw_pub_ = create_publisher<sensor_msgs::msg::Imu>(get_parameter("topics.imu_raw").as_string(), decoder_common::kImuPublisherDepth);
    imu_scale_override_accel_ = get_parameter("imu_scale_override.accel").as_double();
    imu_scale_override_gyro_  = get_parameter("imu_scale_override.gyro").as_double();

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
    if (imu_scale_override_accel_ != 0.0 && imu_scale_override_gyro_ != 0.0) {
      RCLCPP_INFO(get_logger(), "  IMU scale     : override (accel=%.4e gyro=%.4e)",
        imu_scale_override_accel_, imu_scale_override_gyro_);
    } else {
      RCLCPP_INFO(get_logger(), "  IMU scale     : auto (by IMU type at runtime)");
    }
  }

  void initializeDecoder() {
    if (init_raw(&raw_, STRFMT_OEM4) != 1) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize raw decoder");
      throw std::runtime_error("init_raw failed");
    }
    std::strcpy(raw_.opt, "-EPHALL");
  }

  void startPolling() {
    timer_ = create_wall_timer(kTimerInterval, std::bind(&NovatelDecoderNode::pollStream, this));
  }

  // Stream Management

  void openStream() {
    // Main-port RW is only needed for the single-port relay fallback (no
    // dedicated output); otherwise keep the passive read-only open.
    const bool relay_to_main =
        !get_parameter("rtcm_input.listen").as_string().empty() &&
        get_parameter("rtcm_input.receiver_stream").as_string().empty();
    if (!decoder_common::openStreamPath(get_logger(), stream_,
                                        get_parameter("stream_path").as_string(),
                                        relay_to_main ? STR_MODE_RW : STR_MODE_R,
                                        "Input")) {
      throw std::runtime_error("stropen failed");
    }
  }

  // RTCM relay (RTKLIB STRSVR-style): host a TCP server for incoming
  // base-station corrections and write them to the receiver in pollStream().
  // NovAtel: corrections go to a DEDICATED port (`rtcm_input.receiver_stream`), same
  // design as novatel_driver_node — the port is put into RTCMV3 input mode by
  // sending INTERFACEMODE over that very port (THISPORT), so the user never
  // needs its NovAtel logical name and the main decoded-log port stays
  // untouched (passive).
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

    const std::string output = get_parameter("rtcm_input.receiver_stream").as_string();
    if (output.empty()) {
      RCLCPP_WARN(get_logger(),
        "rtcm_input.receiver_stream is empty: corrections go to the main port. NovAtel "
        "cannot mix RTCM input with NOVATEL logs on one logical port — set "
        "rtcm_input.receiver_stream to a second /dev device for on-chip RTK.");
      return;
    }
    if (!decoder_common::openStreamPath(get_logger(), relay_out_, output,
                                        STR_MODE_RW, "RTCM relay output")) {
      throw std::runtime_error("RTCM relay output stropen failed");
    }
    relay_out_open_ = true;

    // Put the correction port into RTCMV3 input mode over the port itself.
    // If it is already in RTCMV3 (e.g. saved with SAVECONFIG), the command
    // bytes are simply discarded by the receiver's RTCM parser — harmless.
    const std::string correction_port =
        get_parameter("rtcm_input.receiver_port").as_string();
    const std::string cmd = std::string(novatel::CMD_INTERFACEMODE) + " " +
                            correction_port + " RTCMV3 NONE OFF\r\n";
    if (!decoder_common::writeAll(relay_out_, reinterpret_cast<const uint8_t*>(cmd.data()), cmd.size()))
      throw std::runtime_error("Correction port command write failed");
    std::this_thread::sleep_for(200ms);
    RCLCPP_INFO(get_logger(),
      "RTCM relay: '%s' -> dedicated port '%s' (INTERFACEMODE %s sent; acceptance unverified)",
      decoder_common::redactStreamUri(listen).c_str(), decoder_common::redactStreamUri(output).c_str(), correction_port.c_str());
  }

  // Polling

  wire::StreamSession stream_session_;
  wire::ReceiverTimeline receiver_timeline_;
  void resetReceiverSession(bool disconnected) {
    // Release validated pending data before retiring the old session. Partial
    // wire frames and compressed references are never carried across sessions.
    flushPending();
    wire::resetRaw(raw_,STRFMT_OEM4);
    if (disconnected) wire_framer_.reset();
    receiver_health_.resetSession();
    receiver_timeline_.reset();
    nmea_parser_.reset(); nmea_solution_=msg::GnssSolution(); binary_solution_=msg::GnssSolution();
    pending_=decltype(pending_)(); last_dop_={};
    prev_pvt_tow_ms_=UINT32_MAX; pvt_period_ms_=0;
    last_flushed_week_=0; last_flushed_tow_ms_=UINT32_MAX;
    eph_store_.clear(); publishSnapshot();
    source_=SolutionSource::UNDETERMINED; grace_started_=false; saw_binary_during_grace_=false;
    oem4_time_quality_.reset(); oem4_time_valid_=false; binary_time_valid_=false;
    has_corrimu_prev_=false; has_rawimu_prev_=false;
    corrimu_prev_seconds_=0; rawimu_prev_seconds_=0;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Receiver session reset: %s; waiting for fresh time/navigation references",
                disconnected ? "connection lost" : "measurement rewind or clock reset");
  }

  void pollStream() {
    // RTCM relay: drain corrections from the TCP server and write them to the
    // dedicated RTCMV3 port if configured, else the main stream (single-port
    // fallback). Same poll thread as the read below, so no lock.
    relay_.drainTo(get_logger(), *get_clock(),
                   relay_out_open_ ? relay_out_ : stream_);

    uint8_t buffer[novatel::READ_BUFFER_SIZE];
    const int bytes_read = strread(&stream_, buffer, sizeof(buffer));
    if (stream_session_.poll(stream_)) resetReceiverSession(true);
    const auto wire_now = std::chrono::steady_clock::now();
    receiver_health_.input(bytes_read);
    receiver_health_.framing(wire_framer_);
    receiver_health_.pvtPending(pending_.has_pvt || pending_.cov_received);
    if (has_rawimu_prev_ || has_corrimu_prev_)
      receiver_health_.imuPending((has_rawimu_prev_ && rawimu_period_==0) || (has_corrimu_prev_ && corrimu_period_==0));
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
      parseOem4Byte(buffer[i]);
    }

    maybePublishHeartbeat();
    commitSourceLockIfDue();  // ensure grace finalizes even on idle stream
    maybeWatchdogFlushPendingPvt();
  }

  // OEM4 mixed stream decoder (shared boundary with RTKLIB, for IMU and PVT logs)
  //
  // OEM4 binary frame: SYNC(0xAA,0x44,0x12) HDRLEN(1) MSGID(2,LE) MSGTYPE(1)
  //                    PORT(1) MSGLEN(2,LE) ... rest of header ... BODY CRC32

  void dispatchFrame(const wire::Frame& f) {
      auto diagnostic_scope = receiver_health_.backendScope();
      receiver_health_.frame((unsigned(f.cls)<<16)|f.id, f.revision);
      receiver_health_.inspect(wire::Protocol::Novatel,f);
      oem4_time_valid_ = oem4_time_quality_.observe(f);
      if (!oem4_time_valid_) {
        receiver_health_.timeUnresolved();
        receiver_health_.timeWithheld();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "NovAtel invalid receiver time (status=%u, week=%u); output withheld",
            f.time_status, f.week);
        return; // Do not let invalid time reset session, PVT, or IMU state.
      }
      if (receiver_timeline_.frame(wire::Protocol::Novatel,f)) {
        resetReceiverSession(false);
        receiver_timeline_.frame(wire::Protocol::Novatel,f);
        oem4_time_valid_ = oem4_time_quality_.observe(f);
      }
      for (uint8_t byte : novatel::backendFrame(f.bytes)) {
        const int status = input_oem4(&raw_, byte);
        handleDecodeResult(status);
      }
      oem4_msg_id_ = f.id;
      oem4_gps_week_ = f.week;
      oem4_gps_ms_ = f.tow_ms;
      oem4_body_ = f.payload;
      handleOem4Frame();
  }

  void parseOem4Byte(uint8_t byte) {
    wire_framer_.feed(byte, [this](const wire::Frame& f) { dispatchFrame(f); },
        [this](const std::string& sentence) { handleNmeaSentence(sentence); });
  }

  void handleOem4Frame() {
    if (oem4_msg_id_ == novatel::ID_RAWIMUSX)    handleRawImuSx();
    if (oem4_msg_id_ == novatel::ID_CORRIMUDATA) handleCorrImuData();
    if (oem4_msg_id_ == novatel::ID_BESTPOS)     handleBestPos();
    if (oem4_msg_id_ == novatel::ID_BESTVEL)     handleBestVel();
    if (oem4_msg_id_ == novatel::ID_PSRDOP)      handlePsrDop();
    if (oem4_msg_id_ == novatel::ID_BESTXYZ)     handleBestXyz();
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

  // PVT TOW aggregation: BEST* logs merge into pending_ keyed on the OEM4
  // header TOW (oem4_gps_ms_); PSRDOP is cached separately (last_dop_).
  static constexpr uint8_t COV_BIT_VEL = 0x1;
  static constexpr uint8_t COV_BIT_XYZ = 0x2;

  struct PendingPvt {
    uint32_t tow_ms{UINT32_MAX};
    bool has_pvt{false};
    bool time_valid{false};
    uint32_t week{0};
    uint8_t cov_received{0};
    uint8_t cov_ever_seen{0};
    msg::GnssSolution buf{};
  };

  gnss_utils::DopCache last_dop_;
  uint32_t prev_pvt_tow_ms_{UINT32_MAX};
  uint32_t pvt_period_ms_{0};

  // After a publish, record the (week, tow) to suppress orphan re-publish.
  uint32_t last_flushed_week_{0};
  uint32_t last_flushed_tow_ms_{UINT32_MAX};


  // Orphan guard: same TOW as last published → only update learning.
  bool isLateOrphan(uint8_t cov_bit) {
    if (last_flushed_tow_ms_ != UINT32_MAX &&
        oem4_gps_ms_   == last_flushed_tow_ms_ &&
        oem4_gps_week_ == last_flushed_week_) {
      pending_.cov_ever_seen |= cov_bit;
      GNSS_RX_EVENT(GNSS_RX_DUPLICATE_INPUT,1); return true;
    }
    return false;
  }

  void aggregateEpochBoundary() {
    if ((oem4_gps_ms_ != pending_.tow_ms || oem4_gps_week_ != pending_.week) &&
        (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = oem4_gps_ms_;
    pending_.week = oem4_gps_week_;
    pending_.time_valid = (pending_.has_pvt || pending_.cov_received) ?
        pending_.time_valid && oem4_time_valid_ : oem4_time_valid_;
    // BEST*/PSRDOP bodies carry no GPS time — (week, ms) comes from the OEM4
    // header seen by the mini-framer.
    pending_.buf.time_week = oem4_gps_week_;
    pending_.buf.time_tow  = oem4_gps_ms_ / 1000.0;
  }

  void handleBestPos() {
    startGraceIfNeeded();
    if (isLateOrphan(0)) return;
    aggregateEpochBoundary();
    if (!novatel::pvt::parseBESTPOS(oem4_body_.data(), oem4_body_.size(), pending_.buf)) { GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return; }
    pending_.has_pvt = true;
    epoch_flush_.touch();
    saw_binary_during_grace_ = true;
    commitSourceLockIfDue();
    // Flush at the epoch boundary or shared quiet-period deadline.
  }

  void handleBestVel() {
    startGraceIfNeeded();
    if (isLateOrphan(COV_BIT_VEL)) return;
    aggregateEpochBoundary();
    if (!novatel::pvt::parseBESTVEL(oem4_body_.data(), oem4_body_.size(), pending_.buf)) { GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return; }
    pending_.cov_received  |= COV_BIT_VEL;
    pending_.cov_ever_seen |= COV_BIT_VEL;
    epoch_flush_.touch();
    // Flush at the epoch boundary or shared quiet-period deadline.
  }

  // PSRDOP: cached in last_dop_; applied at flush by applyDopWithStaleness()
  // (does not gate epoch completion).
  void handlePsrDop() {
    startGraceIfNeeded();
    msg::GnssSolution scratch{};
    if (!novatel::pvt::parsePSRDOP(oem4_body_.data(), oem4_body_.size(), scratch)) { GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return; }
    last_dop_.valid  = true;
    last_dop_.week   = oem4_gps_week_;
    last_dop_.tow_ms = oem4_gps_ms_;
    last_dop_.gdop   = scratch.gdop;
    last_dop_.pdop   = scratch.pdop;
    last_dop_.hdop   = scratch.hdop;
    last_dop_.vdop   = scratch.vdop;
  }

  void handleBestXyz() {
    // BESTXYZ contributes native ECEF covariance diagonals only —
    // BESTPOS/BESTVEL remain the source of truth for pos/vel values.
    startGraceIfNeeded();
    if (isLateOrphan(COV_BIT_XYZ)) return;
    aggregateEpochBoundary();
    if (!novatel::pvt::parseBESTXYZ(oem4_body_.data(), oem4_body_.size(), pending_.buf)) { GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return; }
    pending_.cov_received  |= COV_BIT_XYZ;
    pending_.cov_ever_seen |= COV_BIT_XYZ;
    epoch_flush_.touch();
    // Flush at the epoch boundary or shared quiet-period deadline.
  }

  void flushPending() {
    const uint8_t ever_seen = pending_.cov_ever_seen;
    if (!pending_.has_pvt) {
      if (pending_.cov_received) receiver_health_.pvtIncompleteDiscarded();
      pending_ = {};
      pending_.cov_ever_seen = ever_seen;
      return;
    }
    binary_solution_ = pending_.buf;
    binary_time_valid_ = pending_.time_valid;
    if (!(pending_.cov_received & COV_BIT_VEL)) {
      binary_solution_.vel_enu.x = binary_solution_.vel_enu.y = binary_solution_.vel_enu.z =
          std::numeric_limits<double>::quiet_NaN();
    }
    if (!(pending_.cov_received & COV_BIT_XYZ)) {
      binary_solution_.vel_cov_ecef.fill(std::numeric_limits<double>::quiet_NaN());
      binary_solution_.vel_enu_cov.fill(std::numeric_limits<double>::quiet_NaN());
    }
    decoder_common::finalizeBinarySolutionGeometry(binary_solution_);
    // Covariance source policy (epoch-local: based only on THIS epoch's blocks):
    //   - BESTXYZ not received this epoch: BESTPOS ENU diagonals are authoritative;
    //     rotate ENU→ECEF to fill pos_cov_ecef.
    //   - BESTXYZ received this epoch: BESTXYZ owns pos covariance; rotate ECEF→ENU.
    const bool has_xyz_this_epoch = (pending_.cov_received & COV_BIT_XYZ) != 0;
    rotatePosCovariance(binary_solution_, has_xyz_this_epoch &&
        std::isfinite(binary_solution_.pos_cov_ecef[0]));
    if (has_xyz_this_epoch) rotateVelCovariance(binary_solution_);

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
    last_flushed_week_   = pending_.week;
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

  // Covariance rotation helpers (diagonal-only sources; BESTPOS/BESTXYZ publish
  // diagonals). `has_bestxyz_this_epoch` picks the rotation direction per epoch.
  void rotatePosCovariance(msg::GnssSolution& s, bool has_bestxyz_this_epoch) {
    const double lat = s.latitude  * D2R;
    const double lon = s.longitude * D2R;
    if (has_bestxyz_this_epoch) {
      double cov_ecef[9], cov_enu[9];
      std::copy(s.pos_cov_ecef.begin(), s.pos_cov_ecef.end(), cov_ecef);
      gnss_utils::rotateCovariance(cov_ecef, lat, lon, cov_enu);
      std::copy(std::begin(cov_enu), std::end(cov_enu), s.pos_enu_cov.begin());
    } else {
      double cov_enu[9], cov_ecef[9];
      std::copy(s.pos_enu_cov.begin(), s.pos_enu_cov.end(), cov_enu);
      gnss_utils::rotateCovarianceEnuToEcef(cov_enu, lat, lon, cov_ecef);
      std::copy(std::begin(cov_ecef), std::end(cov_ecef), s.pos_cov_ecef.begin());
    }
  }

  void rotateVelCovariance(msg::GnssSolution& s) {
    const double lat = s.latitude  * D2R;
    const double lon = s.longitude * D2R;
    double cov_ecef[9], cov_enu[9];
    std::copy(s.vel_cov_ecef.begin(), s.vel_cov_ecef.end(), cov_ecef);
    gnss_utils::rotateCovariance(cov_ecef, lat, lon, cov_enu);
    std::copy(std::begin(cov_enu), std::end(cov_enu), s.vel_enu_cov.begin());
  }


  // RAWIMUSX body (40 B): see novatel_driver_node.cpp::handleRawImuSx for layout.
  double rawimu_period_{0.0};
  double corrimu_period_{0.0};
  void handleRawImuSx() {
    if (oem4_body_.size() != 40) { GNSS_RX_EVENT(GNSS_RX_LENGTH,1); return; }

    const uint8_t imu_type = oem4_body_[1];
    if (oem4_body_[0] & 0x03) { GNSS_RX_EVENT(GNSS_RX_QUALITY,1); GNSS_RX_EVENT(GNSS_RX_IMU_REJECT,1); return; }  // error or encrypted data
    uint16_t imu_week = 0;
    double seconds = 0.0;
    int32_t z_acc = 0, ny_acc = 0, x_acc = 0, z_gyr = 0, ny_gyr = 0, x_gyr = 0;
    std::memcpy(&imu_week, oem4_body_.data() + 2, 2);
    std::memcpy(&seconds,  oem4_body_.data() + 4, 8);
    std::memcpy(&z_acc,   oem4_body_.data() + 16, 4);
    std::memcpy(&ny_acc,  oem4_body_.data() + 20, 4);
    std::memcpy(&x_acc,   oem4_body_.data() + 24, 4);
    std::memcpy(&z_gyr,   oem4_body_.data() + 28, 4);
    std::memcpy(&ny_gyr,  oem4_body_.data() + 32, 4);
    std::memcpy(&x_gyr,   oem4_body_.data() + 36, 4);

    novatel::ImuScale scale = novatel::getImuScale(imu_type);
    if (imu_scale_override_accel_ != 0.0 && imu_scale_override_gyro_ != 0.0) {
      scale = {imu_scale_override_accel_, imu_scale_override_gyro_};
    }
    if (!std::isfinite(scale.accel) || !std::isfinite(scale.gyro) || scale.accel <= 0.0 || scale.gyro <= 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Unknown NovAtel IMU type %u — RAWIMUSX skipped. Set imu_scale_override.{accel,gyro} "
        "in yaml or add this type to novatel_protocol.hpp.", imu_type);
      GNSS_RX_EVENT(GNSS_RX_VALUE,1); GNSS_RX_EVENT(GNSS_RX_IMU_REJECT,1); return;
    }

    if ((oem4_body_[0] & 0x03) || imu_week == 0 ||
        !std::isfinite(seconds) || seconds < 0 || seconds >= 604800) { GNSS_RX_EVENT(GNSS_RX_TIME,1); return; }
    seconds += static_cast<double>(imu_week) * 604800.0;
    if (!has_rawimu_prev_) {
      rawimu_prev_seconds_ = seconds;
      has_rawimu_prev_     = true;
      GNSS_RX_EVENT(GNSS_RX_IMU_WAIT,1);
      return;
    }
    const double dt = seconds - rawimu_prev_seconds_;
    rawimu_prev_seconds_ = seconds;
    if (dt <= 0.0 || dt > 1.0) { GNSS_RX_EVENT(GNSS_RX_TIME,1); rawimu_period_ = 0.0; return; }
    if (rawimu_period_ > 0.0 && dt > rawimu_period_ * 1.5) { rawimu_period_ = dt; GNSS_RX_EVENT(GNSS_RX_IMU_WAIT,1); return; }
    rawimu_period_ = rawimu_period_ == 0.0 ? dt : std::min(rawimu_period_, dt);

    sensor_msgs::msg::Imu imu;
    const auto measurement_stamp = time_base_.stamp(
        use_gps_timestamp_, oem4_time_valid_ && imu_week > 0,
        imu_week > 0 ? gpst2time(imu_week, seconds - static_cast<double>(imu_week) * 604800.0) : gtime_t{}, now(), get_logger(), *get_clock());
    if (!measurement_stamp) { receiver_health_.timeWithheld(); return; }
    imu.header.stamp = *measurement_stamp;
    imu.header.frame_id = frame_id_;
    imu.orientation_covariance[0] = -1.0;

    imu.angular_velocity.x =  x_gyr  * scale.gyro / dt;
    imu.angular_velocity.y = -static_cast<double>(ny_gyr) * scale.gyro / dt;
    imu.angular_velocity.z =  z_gyr  * scale.gyro / dt;
    imu.linear_acceleration.x =  x_acc  * scale.accel / dt;
    imu.linear_acceleration.y = -static_cast<double>(ny_acc) * scale.accel / dt;
    imu.linear_acceleration.z =  z_acc  * scale.accel / dt;

    // Values above are always provided by RAWIMU; only their covariance is
    // unknown -> all-zero, not "-1" (which claims the measurement is absent).
    auto unk = ins::makeZeroCovariance();
    std::copy(unk.begin(), unk.end(), imu.angular_velocity_covariance.begin());
    std::copy(unk.begin(), unk.end(), imu.linear_acceleration_covariance.begin());

    imu_raw_pub_->publish(imu);
  }

  // CORRIMUDATA body (60 B): Week(4) Seconds(8) PitchRate(8) RollRate(8) YawRate(8)
  //                          LateralAcc(8) LongitudinalAcc(8) VerticalAcc(8)
  // Rates and accelerations are SI increments accumulated over the IMU sampling
  // period; divide by dt (from successive Seconds fields) to recover rad/s and m/s².
  void handleCorrImuData() {
    if (oem4_body_.size() < 60) { GNSS_RX_EVENT(GNSS_RX_LENGTH,1); return; }

    uint32_t corrimu_week = 0;
    double seconds = 0.0, pitch_rate = 0.0, roll_rate = 0.0, yaw_rate = 0.0;
    double lat_acc = 0.0, lon_acc = 0.0, vert_acc = 0.0;
    std::memcpy(&corrimu_week, oem4_body_.data() +  0, 4);
    std::memcpy(&seconds,      oem4_body_.data() +  4, 8);
    std::memcpy(&pitch_rate,   oem4_body_.data() + 12, 8);
    std::memcpy(&roll_rate,  oem4_body_.data() + 20, 8);
    std::memcpy(&yaw_rate,   oem4_body_.data() + 28, 8);
    std::memcpy(&lat_acc,    oem4_body_.data() + 36, 8);
    std::memcpy(&lon_acc,    oem4_body_.data() + 44, 8);
    std::memcpy(&vert_acc,   oem4_body_.data() + 52, 8);

    if (corrimu_week == 0 || corrimu_week > 65535 || !std::isfinite(seconds) ||
        seconds < 0 || seconds >= 604800 || !std::isfinite(pitch_rate) ||
        !std::isfinite(roll_rate) || !std::isfinite(yaw_rate) ||
        !std::isfinite(lat_acc) || !std::isfinite(lon_acc) || !std::isfinite(vert_acc)) { GNSS_RX_EVENT(GNSS_RX_VALUE,1); GNSS_RX_EVENT(GNSS_RX_IMU_REJECT,1); return; }
    const double tow = seconds;
    seconds += static_cast<double>(corrimu_week) * 604800.0;
    if (!has_corrimu_prev_) {
      corrimu_prev_seconds_ = seconds;
      has_corrimu_prev_     = true;
      GNSS_RX_EVENT(GNSS_RX_IMU_WAIT,1);
      return;
    }
    const double dt = seconds - corrimu_prev_seconds_;
    corrimu_prev_seconds_ = seconds;
    if (dt <= 0.0 || dt > 1.0) { GNSS_RX_EVENT(GNSS_RX_TIME,1); corrimu_period_ = 0.0; return; }
    if (corrimu_period_ > 0.0 && dt > corrimu_period_ * 1.5) { corrimu_period_ = dt; GNSS_RX_EVENT(GNSS_RX_IMU_WAIT,1); return; }
    corrimu_period_ = corrimu_period_ == 0.0 ? dt : std::min(corrimu_period_, dt);

    sensor_msgs::msg::Imu imu;
    const auto measurement_stamp = time_base_.stamp(
        use_gps_timestamp_, oem4_time_valid_ && corrimu_week > 0 && std::isfinite(tow) && tow >= 0 && tow < 604800,
        corrimu_week > 0
            ? gpst2time(static_cast<int>(corrimu_week), tow) : gtime_t{}, now(), get_logger(), *get_clock());
    if (!measurement_stamp) { receiver_health_.timeWithheld(); return; }
    imu.header.stamp = *measurement_stamp;
    imu.header.frame_id = frame_id_;

    // NovAtel native right-handed frame: pitch/lateral X, roll/longitudinal Y, Z up.
    imu.angular_velocity.x    = pitch_rate / dt;
    imu.angular_velocity.y    = roll_rate / dt;
    imu.angular_velocity.z    = yaw_rate / dt;
    imu.linear_acceleration.x = lat_acc / dt;
    imu.linear_acceleration.y = lon_acc / dt;
    imu.linear_acceleration.z = vert_acc / dt;

    // orientation: not provided by CORRIMUDATA — leave identity, mark unknown
    imu.orientation_covariance[0] = -1.0;

    // Values above are always provided by CORRIMUDATA; only their covariance
    // is unknown -> all-zero, not "-1" (which claims the measurement is absent).
    auto unk = ins::makeZeroCovariance();
    std::copy(unk.begin(), unk.end(), imu.angular_velocity_covariance.begin());
    std::copy(unk.begin(), unk.end(), imu.linear_acceleration_covariance.begin());

    imu_pub_->publish(imu);
  }

  // Message Handling

  void handleNmeaSentence(const std::string& sentence) {
    startGraceIfNeeded();
    if (!nmea_parser_.parseSentence(sentence, nmea_solution_)) return;
    commitSourceLockIfDue();
    if (source_ == SolutionSource::NMEA) publishSolution(nmea_solution_);
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
    // BINARY: the OEM4 header (week, ms) is already in time_week/time_tow —
    // trust them; raw_.time is only a fallback. NMEA: NmeaParser filled them.
    gtime_t t_gpst{};
    int week_for_stamp = 0;
    if (source_ == SolutionSource::BINARY) {
      if (binary_time_valid_ && sol.time_week > 0) {
        week_for_stamp = static_cast<int>(sol.time_week);
        t_gpst = gpst2time(week_for_stamp, sol.time_tow);
      } else {
        sol.time_week = 0;
        sol.time_tow = 0;
      }
      sol.solution_source = msg::GnssSolution::SOLUTION_SOURCE_BINARY;
    } else {
      sol.solution_source = msg::GnssSolution::SOLUTION_SOURCE_NMEA;
      if (sol.time_week > 0) {
        week_for_stamp = sol.time_week;
        t_gpst = gpst2time(sol.time_week, sol.time_tow);
      }
    }

    const auto measurement_stamp = time_base_.stamp(
        use_gps_timestamp_, week_for_stamp > 0 && std::isfinite(sol.time_tow) && sol.time_tow >= 0 && sol.time_tow < 604800,
        t_gpst, now(), get_logger(), *get_clock());
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

    msg::GnssObservations msg;
    const auto measurement_stamp = time_base_.stamp(
        use_gps_timestamp_, oem4_time_valid_ && week > 0,
        obs_time, now(), get_logger(), *get_clock());
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

  rclcpp::Publisher<msg::GnssObservations>::SharedPtr  obs_pub_;
  rclcpp::Publisher<msg::GnssEphemerides>::SharedPtr   eph_pub_;
  rclcpp::Publisher<msg::GnssSolution>::SharedPtr      sol_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr  imu_pub_;       // CORRIMUDATA (calibrated)
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr  imu_raw_pub_;   // RAWIMUSX (uncalibrated)
  double imu_scale_override_accel_{0.0};
  double imu_scale_override_gyro_{0.0};
  rclcpp::TimerBase::SharedPtr timer_;

  wire::Framer wire_framer_{wire::Protocol::Novatel};
  std::chrono::steady_clock::time_point wire_last_input_{std::chrono::steady_clock::now()};
  uint64_t wire_last_rejected_{0};
  stream_t stream_{};
  raw_t    raw_{};


  // One time base per session, latched at the first published message and
  // never changed. Shared by EVERY topic this node publishes, which is what
  // structurally guarantees the IMU and the GNSS topics cannot end up on
  // different bases. See decoder_common::TimeBaseLatch.
  decoder_common::TimeBaseLatch time_base_;
  wire::NovatelTimeQuality oem4_time_quality_;
  bool oem4_time_valid_{false};
  bool binary_time_valid_{false};

  // RTCM relay (PC -> receiver): STRSVR-style TCP server -> dedicated RTCMV3
  // port (relay_out_) when rtcm_input.receiver_stream is set, else the main stream.
  decoder_common::RtcmRelayServer relay_;
  stream_t relay_out_{};
  bool     relay_out_open_{false};

  // NMEA parsing state
  gnss_utils::NmeaParser nmea_parser_;
  msg::GnssSolution      nmea_solution_;
  msg::GnssSolution      binary_solution_;
  SolutionSource         source_{SolutionSource::UNDETERMINED};
  std::chrono::steady_clock::time_point grace_start_{};
  bool                   grace_started_{false};
  bool                   saw_binary_during_grace_{false};

  // ENU local origin
  bool   has_local_origin_{false};
  double local_origin_ecef_[3]{0.0};
  double local_origin_pos_[3]{0.0};

  // OEM4 decoded frame payload
  uint16_t             oem4_msg_id_{0};
  uint32_t             oem4_gps_week_{0};
  uint32_t             oem4_gps_ms_{0};
  std::vector<uint8_t> oem4_body_;
  PendingPvt           pending_;

  // Ephemeris dedup
  gnss_utils::EphemerisStore eph_store_;

  // CORRIMUDATA dt tracking (SI-increment → rate conversion)
  double corrimu_prev_seconds_{0.0};
  bool   has_corrimu_prev_{false};
  double rawimu_prev_seconds_{0.0};
  bool   has_rawimu_prev_{false};
};

}  // namespace gnss_ros_standardization

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<gnss_ros_standardization::NovatelDecoderNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("novatel_decoder_node"), "Startup/runtime failure: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
