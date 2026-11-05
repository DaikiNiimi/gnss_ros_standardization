#include "gnss_ros_standardization/receiver_health.hpp"
#include "gnss_ros_standardization/receiver_session.hpp"
#include "gnss_ros_standardization/sbf_protocol.hpp"
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
#include "gnss_ros_standardization/msg/gnss_solution.hpp"

using namespace std::chrono_literals;
namespace ins = gnss_ros_standardization::ins_utils;

namespace gnss_ros_standardization {

namespace {

constexpr auto kTimerInterval  = 10ms;


}  // namespace

/// @brief ROS 2 node for decoding Septentrio SBF protocol messages
///
/// Decodes Septentrio Binary Format (SBF) raw observations and navigation data
/// messages from Septentrio receivers and publishes them as standardized ROS messages.
/// Also parses NMEA sentences for GnssSolution and SBF ExtSensorMeas for raw IMU data.
class SbfDecoderNode : public rclcpp::Node {
 public:
  SbfDecoderNode() : Node("sbf_decoder_node") {
    initializeParameters();
    initializePublishers();
    initializeDecoder();
    openStream();
    openRtcmRelay();
    startPolling();

    RCLCPP_INFO(get_logger(), "SBF Decoder initialized");
  }

  ~SbfDecoderNode() override {
    try {
      strclose(&stream_);
      relay_.close();
    } catch (...) {}
    free_raw(&raw_);
  }

 private:
  // Initialization

  decoder_common::EpochFlushPolicy epoch_flush_;
  SbfObservationCompletion observation_completion_;
  wire::ReceiverHealth receiver_health_;
  void initializeParameters() {
    receiver_health_.configure(5.0);
    receiver_health_.limits(MAXOBS, NFREQ, NEXOBS, MAXPRNCMP);
    observation_completion_.configure(0.2);
    epoch_flush_.configure(0.05);
    declare_parameter<std::string>("stream_path", "serial:///dev/ttyUSB0:115200");
    // frame_id: ROS TF frame name attached to the GNSS antenna phase center.
    // Default "gnss_link" — override in launch to integrate with vehicle TF tree.
    declare_parameter<std::string>("frame_id", "gnss_link");
    declare_parameter<std::string>("topics.observation", "/gnss/observation");
    declare_parameter<std::string>("topics.ephemeris", "/gnss/ephemeris");
    declare_parameter<std::string>("topics.solution",     "/gnss/nmea_solution");
    declare_parameter<std::string>("topics.imu_raw",      "/gnss/imu/data_raw");
    declare_parameter<double>("ephemeris.snapshot_period_s", 30.0);
    // 0.0 disables aging: keep every received eph so post-processing tools
    // (rosbag_to_rinex etc.) can output them even when observations from
    // earlier in the recording are processed later.
    declare_parameter<double>("ephemeris.max_age_s", 0.0);

    declare_parameter<bool>("use_gps_timestamp", false);

    // RTCM relay listen URI (opt-in, see openRtcmRelay). Empty = disabled.
    declare_parameter<std::string>("rtcm_input.listen", "");

    frame_id_ = get_parameter("frame_id").as_string();
    eph_store_.setSnapshotPeriod(get_parameter("ephemeris.snapshot_period_s").as_double());
    eph_store_.setMaxAge(get_parameter("ephemeris.max_age_s").as_double());
    constexpr int history_limit = 8;
    eph_store_.setHistoryPerSatellite(static_cast<size_t>(history_limit));
    use_gps_timestamp_ = get_parameter("use_gps_timestamp").as_bool();
  }

  void initializePublishers() {
    decoder_common::configureNmeaDate(*this, nmea_parser_);
    obs_pub_          = create_publisher<msg::GnssObservations>(get_parameter("topics.observation").as_string(), 10);
    eph_pub_          = create_publisher<msg::GnssEphemerides>(get_parameter("topics.ephemeris").as_string(), rclcpp::QoS(256).transient_local()  /* depth 256: latch full ephemeris set, not just the last satellite (per-sat messages) */);
    sol_pub_          = create_publisher<msg::GnssSolution>(get_parameter("topics.solution").as_string(), 10);
    imu_raw_pub_      = create_publisher<sensor_msgs::msg::Imu>(get_parameter("topics.imu_raw").as_string(), decoder_common::kImuPublisherDepth);

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
    if (init_raw(&raw_, STRFMT_SEPT) != 1) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize raw decoder");
      throw std::runtime_error("init_raw failed");
    }
    std::strcpy(raw_.opt, "-EPHALL");
  }

  void startPolling() {
    timer_ = create_wall_timer(kTimerInterval, std::bind(&SbfDecoderNode::pollStream, this));
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
  // "tcpsvr://:5557", fed by rtcm_decoder_node's rtcm_relay) and write incoming
  // corrections to the receiver stream in pollStream() for on-chip RTK. No
  // receiver command is sent; the port must already accept RTCMv3
  // (setDataInOut) — see src/decoders/README.md.
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

  // Polling

  wire::StreamSession stream_session_;
  wire::ReceiverTimeline receiver_timeline_;
  void resetReceiverSession(bool disconnected) {
    // Release validated pending data before retiring the old session. Partial
    // wire frames and compressed references are never carried across sessions.
    if (gnss_flush_sbf_observations(&raw_)==1) publishObservations();
    observation_completion_.reset();
    flushPending();
    wire::resetRaw(raw_,STRFMT_SEPT);
    if (disconnected) wire_framer_.reset();
    receiver_health_.resetSession();
    receiver_timeline_.reset();
    nmea_parser_.reset(); nmea_solution_=msg::GnssSolution(); binary_solution_=msg::GnssSolution();
    pending_=decltype(pending_)(); last_dop_={};
    prev_pvt_tow_ms_=UINT32_MAX; pvt_period_ms_=0;
    last_flushed_week_=0; last_flushed_tow_ms_=UINT32_MAX;
    eph_store_.clear(); publishSnapshot();
    source_=SolutionSource::UNDETERMINED; grace_started_=false; saw_binary_during_grace_=false;
    esm_tow_ms_=UINT32_MAX; esm_wnc_=UINT16_MAX; esm_sensor_=UINT16_MAX;
    esm_has_accel_=false; esm_has_gyro_=false;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Receiver session reset: %s; waiting for fresh time/navigation references",
                disconnected ? "connection lost" : "measurement rewind or clock reset");
  }

  void pollStream() {
    // RTCM relay: drain corrections from the TCP server and write them to the
    // receiver stream. Same poll thread as the read below, so no lock.
    relay_.drainTo(get_logger(), *get_clock(), stream_);

    uint8_t buffer[sbf::READ_BUFFER_SIZE];
    const int bytes_read = strread(&stream_, buffer, sizeof(buffer));
    if (stream_session_.poll(stream_)) resetReceiverSession(true);
    const auto wire_now = std::chrono::steady_clock::now();
    receiver_health_.input(bytes_read);
    receiver_health_.framing(wire_framer_);
    receiver_health_.assemblyPending(raw_.obuf.n>0);
    receiver_health_.pvtPending(pending_.blocks_received != 0);
    receiver_health_.imuPending(esm_has_accel_ || esm_has_gyro_);
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
      parseSbfByte(buffer[i]);
    }

    if (observation_completion_.flushIfDue(raw_)) {
      RCLCPP_DEBUG(get_logger(), "Publishing final MeasEpoch after observation quiet timeout");
      publishObservations();
    }
    maybePublishHeartbeat();
    maybeWatchdogFlushPendingPvt();
    commitSourceLockIfDue();  // ensure grace finalizes even on idle stream
  }

  // SBF mixed stream decoder (shared boundary with RTKLIB, for ExtSensorMeas and PVT blocks)
  //
  // SBF block layout: SYNC1('$') SYNC2('@') CRC(2) ID(2,LE) Length(2,LE) Body(Length-8)
  // ID bits 0-12 are the block number; bits 13-15 are the revision.

  void dispatchFrame(const wire::Frame& f) {
      auto diagnostic_scope = receiver_health_.backendScope();
      receiver_health_.frame((unsigned(f.cls)<<16)|f.id, f.revision);
      receiver_health_.inspect(wire::Protocol::Sbf,f);
      if (receiver_timeline_.frame(wire::Protocol::Sbf,f)) {
        resetReceiverSession(false);
        receiver_timeline_.frame(wire::Protocol::Sbf,f);
      }
      if (!gnss_ros_standardization::supportedSbfNavigation(f)) {
        receiver_health_.unsupportedNavigation();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Unsupported SBF navigation model/layout: id=%u revision=%u length=%zu; retain the original receiver log",
            f.id, f.revision, f.bytes.size());
        return;
      }
      int result = 0;
      for (uint8_t byte : f.bytes) {
        const int status = input_sbf(&raw_, byte);
        if (status != 0) result = status;
        handleDecodeResult(status);
      }
      if (result >= 0) observation_completion_.decoded(f.id, raw_);
      receiver_health_.assemblyPending(raw_.obuf.n>0);
      sbf_id_ = f.id;
      sbf_body_ = f.payload;
      handleSbfBlock();
  }

  void parseSbfByte(uint8_t byte) {
    wire_framer_.feed(byte, [this](const wire::Frame& f) { dispatchFrame(f); },
        [this](const std::string& sentence) { handleNmeaSentence(sentence); });
  }

  void handleSbfBlock() {
    switch (sbf_id_) {
      case sbf::ID_EXTSENSORMEAS:    handleExtSensorMeas();    break;
      // Decoded *Nav blocks are handled by RTKLIB input_sbf(); decoding them
      // here too produced duplicate, slightly-different ephemerides.
      case sbf::ID_PVTGEODETIC:      handlePvtGeodetic();      break;
      case sbf::ID_POSCOVGEODETIC:   handlePosCovGeodetic();   break;
      case sbf::ID_VELCOVGEODETIC:   handleVelCovGeodetic();   break;
      case sbf::ID_PVTCARTESIAN:     handlePvtCartesian();     break;
      case sbf::ID_POSCOVCARTESIAN:  handlePosCovCartesian();  break;
      case sbf::ID_VELCOVCARTESIAN:  handleVelCovCartesian();  break;
      case sbf::ID_DOP:              handleDop();              break;
      default: break;
    }
  }

  // Binary PVT handlers. The decoder is passive, so the expected block set is
  // learned at runtime (blocks_ever_seen sticky bitmask).
  static constexpr uint8_t BLK_PVT_GEO = 1 << 0;
  static constexpr uint8_t BLK_POS_GEO = 1 << 1;
  static constexpr uint8_t BLK_VEL_GEO = 1 << 2;
  static constexpr uint8_t BLK_PVT_XYZ = 1 << 3;
  static constexpr uint8_t BLK_POS_XYZ = 1 << 4;
  static constexpr uint8_t BLK_VEL_XYZ = 1 << 5;

  // Pending aggregator: Geo+Xyz PVT/Cov blocks for one TOW merge into
  // pending_.buf; flush when blocks_received == blocks_ever_seen or a new TOW
  // arrives. DOP is cached separately (last_dop_).
  struct Pending {
    uint32_t tow_ms{UINT32_MAX};
    uint8_t  blocks_received{0};
    uint8_t  blocks_ever_seen{0};
    msg::GnssSolution buf{};
  };

  // Persistent DOP snapshot (survives Pending resets).
  gnss_utils::DopCache last_dop_;
  uint32_t prev_pvt_tow_ms_{UINT32_MAX};
  uint32_t pvt_period_ms_{0};

  // Last flushed (week, tow) — late arrivals for it must not re-accumulate
  // (would duplicate-publish); they only update blocks_ever_seen.
  uint32_t last_flushed_week_{0};
  uint32_t last_flushed_tow_ms_{UINT32_MAX};

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

  // Mark "binary seen during grace" — called from every PVT block arrival.
  void markBinarySeen() {
    saw_binary_during_grace_ = true;
    commitSourceLockIfDue();
  }


  template <typename ParseFn>
  void mergeBlock(uint8_t bit, ParseFn parse) {
    startGraceIfNeeded();
    const uint8_t* p = sbf_body_.data();
    const size_t   len = sbf_body_.size();
    const uint32_t tow = sbf::pvt::getTowMs(p, len);

    // Late-arrival guard: same (week, tow) as the last flush → update learning
    // only. Week falls back to TOW-only while this epoch has no PVT yet.
    const uint32_t week = pending_.buf.time_week;
    if (last_flushed_tow_ms_ != UINT32_MAX &&
        tow == last_flushed_tow_ms_ &&
        (week == 0 || week == last_flushed_week_)) {
      pending_.blocks_ever_seen |= bit;
      GNSS_RX_EVENT(GNSS_RX_DUPLICATE_INPUT,1); return;
    }

    if (tow != pending_.tow_ms && pending_.blocks_received != 0) {
      flushPending();  // TOW boundary — flush previous epoch
    }
    pending_.tow_ms = tow;
    if (!parse(p, len, pending_.buf)) { GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return; }
    pending_.blocks_received  |= bit;
    pending_.blocks_ever_seen |= bit;
    epoch_flush_.touch();
    if (bit == BLK_PVT_GEO || bit == BLK_PVT_XYZ) markBinarySeen();
    // Flush at the epoch boundary or shared quiet-period deadline.
  }

  void handlePvtGeodetic()    { mergeBlock(BLK_PVT_GEO, &sbf::pvt::parsePVTGeodetic);    }
  void handlePosCovGeodetic() { mergeBlock(BLK_POS_GEO, &sbf::pvt::parsePosCovGeodetic); }
  void handleVelCovGeodetic() { mergeBlock(BLK_VEL_GEO, &sbf::pvt::parseVelCovGeodetic); }
  void handlePvtCartesian()   { mergeBlock(BLK_PVT_XYZ, &sbf::pvt::parsePVTCartesian);   }
  void handlePosCovCartesian(){ mergeBlock(BLK_POS_XYZ, &sbf::pvt::parsePosCovCartesian);}
  void handleVelCovCartesian(){ mergeBlock(BLK_VEL_XYZ, &sbf::pvt::parseVelCovCartesian);}

  // DOP block: cached in last_dop_; applied at flush by applyDopWithStaleness()
  // (does not gate epoch completion).
  void handleDop() {
    const uint8_t* p   = sbf_body_.data();
    const size_t   len = sbf_body_.size();
    msg::GnssSolution scratch{};
    if (!sbf::pvt::parseDop(p, len, scratch)) { GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return; }
    last_dop_.valid  = true;
    last_dop_.week   = sbf::pvt::getWeek(p, len);   // from SBF block header (WNc)
    last_dop_.tow_ms = sbf::pvt::getTowMs(p, len);
    last_dop_.gdop   = scratch.gdop;
    last_dop_.pdop   = scratch.pdop;
    last_dop_.hdop   = scratch.hdop;
    last_dop_.vdop   = scratch.vdop;
  }

  void flushPending() {
    const uint8_t ever_seen = pending_.blocks_ever_seen;
    const uint8_t recv      = pending_.blocks_received;
    const bool has_geo_pvt  = recv & BLK_PVT_GEO;
    const bool has_xyz_pvt  = recv & BLK_PVT_XYZ;
    if (!has_geo_pvt && !has_xyz_pvt) {
      if (pending_.blocks_received) receiver_health_.pvtIncompleteDiscarded();
      pending_ = {};
      pending_.blocks_ever_seen = ever_seen;
      return;
    }

    // Snapshot Cartesian-direct fields before LLH-driven derivation overwrites
    // them. They are restored at the end (Cartesian wins for ECEF truth).
    msg::GnssSolution xyz_direct = pending_.buf;
    binary_solution_ = pending_.buf;

    // Position: PVTGeodetic primary. Only PVTCartesian → derive LLH from ECEF.
    if (!has_geo_pvt && has_xyz_pvt) {
      double r[3] = {binary_solution_.pos_ecef.x,
                     binary_solution_.pos_ecef.y,
                     binary_solution_.pos_ecef.z};
      double llh[3];
      ecef2pos(r, llh);
      constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;
      binary_solution_.latitude  = llh[0] * kRad2Deg;
      binary_solution_.longitude = llh[1] * kRad2Deg;
      binary_solution_.altitude  = llh[2];
    }
    decoder_common::finalizeBinarySolutionGeometry(binary_solution_);

    // Covariance bidirectional derivation (NovAtel pattern):
    //   - ENU only → derive ECEF, ECEF only → derive ENU, both → no rotation.
    const double lat = binary_solution_.latitude  * D2R;
    const double lon = binary_solution_.longitude * D2R;
    auto rotate_pos = [&]() {
      const bool has_geo = recv & BLK_POS_GEO;
      const bool has_xyz = recv & BLK_POS_XYZ;
      double n[9], e[9];
      if (has_xyz && !has_geo) {
        std::copy(binary_solution_.pos_cov_ecef.begin(),
                  binary_solution_.pos_cov_ecef.end(), e);
        gnss_utils::rotateCovariance(e, lat, lon, n);
        std::copy(std::begin(n), std::end(n), binary_solution_.pos_enu_cov.begin());
      } else if (has_geo && !has_xyz) {
        std::copy(binary_solution_.pos_enu_cov.begin(),
                  binary_solution_.pos_enu_cov.end(), n);
        gnss_utils::rotateCovarianceEnuToEcef(n, lat, lon, e);
        std::copy(std::begin(e), std::end(e), binary_solution_.pos_cov_ecef.begin());
      }
    };
    auto rotate_vel = [&]() {
      const bool has_geo = recv & BLK_VEL_GEO;
      const bool has_xyz = recv & BLK_VEL_XYZ;
      double n[9], e[9];
      if (has_xyz && !has_geo) {
        std::copy(binary_solution_.vel_cov_ecef.begin(),
                  binary_solution_.vel_cov_ecef.end(), e);
        gnss_utils::rotateCovariance(e, lat, lon, n);
        std::copy(std::begin(n), std::end(n), binary_solution_.vel_enu_cov.begin());
      } else if (has_geo && !has_xyz) {
        std::copy(binary_solution_.vel_enu_cov.begin(),
                  binary_solution_.vel_enu_cov.end(), n);
        gnss_utils::rotateCovarianceEnuToEcef(n, lat, lon, e);
        std::copy(std::begin(e), std::end(e), binary_solution_.vel_cov_ecef.begin());
      }
    };
    rotate_pos();
    rotate_vel();

    // PVTCartesian-direct overrides for pos/vel ECEF.
    if (has_xyz_pvt) {
      binary_solution_.pos_ecef = xyz_direct.pos_ecef;
      binary_solution_.vel_ecef = xyz_direct.vel_ecef;
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
    last_flushed_week_   = binary_solution_.time_week;
    last_flushed_tow_ms_ = pending_.tow_ms;

    pending_ = {};
    pending_.blocks_ever_seen = ever_seen;
  }


  void maybeWatchdogFlushPendingPvt() {
    if (pending_.blocks_received == 0) return;
    if (epoch_flush_.due()) {
      flushPending();
    }
  }

  void handleExtSensorMeas() {
    if (static_cast<int>(sbf_body_.size()) < sbf::ext_sensor_meas::MIN_BODY_LEN) { GNSS_RX_EVENT(GNSS_RX_LENGTH,1); return; }

    const uint8_t n         = sbf_body_[sbf::ext_sensor_meas::OFFSET_N];
    const uint8_t sb_length = sbf_body_[sbf::ext_sensor_meas::OFFSET_SB_LENGTH];
    if (sb_length < sbf::ext_sensor_meas::SB_MIN_LEN) { GNSS_RX_EVENT(GNSS_RX_LENGTH,1); return; }
    if (static_cast<int>(sbf_body_.size()) < sbf::ext_sensor_meas::MIN_BODY_LEN + n * sb_length) { GNSS_RX_EVENT(GNSS_RX_LENGTH,1); return; }

    // TOW [ms] to detect epoch boundaries
    uint32_t tow_ms = 0;
    std::memcpy(&tow_ms, sbf_body_.data(), 4);

    const uint16_t week = wire::u16(sbf_body_.data() + 4);
    if (tow_ms != esm_tow_ms_ || week != esm_wnc_) {
      if (esm_has_accel_ != esm_has_gyro_) GNSS_RX_EVENT(GNSS_RX_ASSEMBLY_REPLACED,1);
      esm_has_accel_ = false; esm_has_gyro_ = false;
    }
    esm_tow_ms_ = tow_ms;
    esm_wnc_ = week;

    // Parse sub-blocks: each Type carries a full 3-vector
    for (uint8_t i = 0; i < n; ++i) {
      const int base = sbf::ext_sensor_meas::OFFSET_SUBBLOCKS + i * sb_length;
      const uint8_t type = sbf_body_[base + sbf::ext_sensor_meas::SB_OFFSET_TYPE];

      // S2 pp332-333: never combine different Source/SensorModel values.
      // Unsupported sub-block types do not disturb an IMU pair in progress.
      if (type != sbf::ext_sensor_meas::TYPE_ACCEL && type != sbf::ext_sensor_meas::TYPE_GYRO) {
        GNSS_RX_EVENT(GNSS_RX_SELECTION,1); continue;
      }
      const uint16_t sensor = wire::u16(sbf_body_.data() + base);
      if (sensor != esm_sensor_) {
        if (esm_has_accel_ || esm_has_gyro_) GNSS_RX_EVENT(GNSS_RX_ASSEMBLY_REPLACED,1);
        esm_has_accel_ = esm_has_gyro_ = false;
        esm_sensor_ = sensor;
      }
      // Saturated or scrambled samples cannot be published as usable SI values.
      if (sbf_body_[base + 3] & 0x06u) {
        GNSS_RX_EVENT(GNSS_RX_QUALITY,1); GNSS_RX_EVENT(GNSS_RX_IMU_REJECT,1);
        esm_has_accel_ = esm_has_gyro_ = false; continue;
      }

      if (type == sbf::ext_sensor_meas::TYPE_ACCEL) {
        std::memcpy(&esm_accel_[0], sbf_body_.data() + base + sbf::ext_sensor_meas::SB_OFFSET_X, 8);
        std::memcpy(&esm_accel_[1], sbf_body_.data() + base + sbf::ext_sensor_meas::SB_OFFSET_Y, 8);
        std::memcpy(&esm_accel_[2], sbf_body_.data() + base + sbf::ext_sensor_meas::SB_OFFSET_Z, 8);
        esm_has_accel_ = true;
      } else if (type == sbf::ext_sensor_meas::TYPE_GYRO) {
        std::memcpy(&esm_gyro_[0], sbf_body_.data() + base + sbf::ext_sensor_meas::SB_OFFSET_X, 8);
        std::memcpy(&esm_gyro_[1], sbf_body_.data() + base + sbf::ext_sensor_meas::SB_OFFSET_Y, 8);
        std::memcpy(&esm_gyro_[2], sbf_body_.data() + base + sbf::ext_sensor_meas::SB_OFFSET_Z, 8);
        esm_has_gyro_ = true;
      } else { GNSS_RX_EVENT(GNSS_RX_SELECTION,1); }
    }

    // Publish immediately when both accel and gyro are accumulated
    if (esm_has_accel_ != esm_has_gyro_) GNSS_RX_EVENT(GNSS_RX_IMU_WAIT,1);
    if (esm_has_accel_ && esm_has_gyro_) publishEsmAccum();
  }

  void publishEsmAccum() {
    if (!esm_has_accel_ || !esm_has_gyro_) return;
    for (int i = 0; i < 3; ++i) {
      if (!std::isfinite(esm_accel_[i]) || !std::isfinite(esm_gyro_[i]) ||
          esm_accel_[i] == -2e10 || esm_gyro_[i] == -2e10) {
        GNSS_RX_EVENT(GNSS_RX_VALUE,1); GNSS_RX_EVENT(GNSS_RX_IMU_REJECT,1);
        esm_has_accel_ = esm_has_gyro_ = false; return;
      }
    }
    sensor_msgs::msg::Imu imu;
    const bool esm_time_valid =
        esm_tow_ms_ < 604800000u && esm_wnc_ > 0 && esm_wnc_ != 0xFFFFu;
    const auto measurement_stamp = time_base_.stamp(
        use_gps_timestamp_, esm_time_valid,
        esm_time_valid ? gpst2time(static_cast<int>(esm_wnc_),
                                   esm_tow_ms_ / 1000.0)
                       : gtime_t{},
        now(), get_logger(), *get_clock());
    if (!measurement_stamp) { receiver_health_.timeWithheld(); return; }
    imu.header.stamp = *measurement_stamp;
    imu.header.frame_id = frame_id_;

    // Orientation is not estimated by this message at all -> "-1" (field not provided).
    auto orient_unk = ins::makeUnknownCovariance();
    std::copy(orient_unk.begin(), orient_unk.end(), imu.orientation_covariance.begin());

    // Acceleration/angular-rate VALUES are provided below; only their
    // covariance is unknown -> all-zero, not "-1" (which would claim the
    // measurement itself is absent).
    auto meas_unk = ins::makeZeroCovariance();

    imu.linear_acceleration.x = esm_accel_[0];
    imu.linear_acceleration.y = esm_accel_[1];
    imu.linear_acceleration.z = esm_accel_[2];
    std::copy(meas_unk.begin(), meas_unk.end(), imu.linear_acceleration_covariance.begin());

    imu.angular_velocity.x = esm_gyro_[0] * (M_PI / 180.0);
    imu.angular_velocity.y = esm_gyro_[1] * (M_PI / 180.0);
    imu.angular_velocity.z = esm_gyro_[2] * (M_PI / 180.0);
    std::copy(meas_unk.begin(), meas_unk.end(), imu.angular_velocity_covariance.begin());

    imu_raw_pub_->publish(imu);

    esm_has_accel_ = false;
    esm_has_gyro_  = false;
    esm_accel_[0] = esm_accel_[1] = esm_accel_[2] = 0.0;
    esm_gyro_[0]  = esm_gyro_[1]  = esm_gyro_[2]  = 0.0;
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
    // BINARY: SBF PVT blocks carry WNc+TOW — trust them; raw_.time is only a
    // fallback. NMEA: NmeaParser filled week/tow from the sentence.
    gtime_t t_gpst{};
    int week_for_stamp = 0;
    if (source_ == SolutionSource::BINARY) {
      if (sol.time_week > 0) {
        week_for_stamp = static_cast<int>(sol.time_week);
        t_gpst = gpst2time(week_for_stamp, sol.time_tow);
      } else {
        const double tow = time2gpst(raw_.time, &week_for_stamp);
        if (raw_.time.time != 0 && week_for_stamp > 0) {
          sol.time_week = static_cast<uint32_t>(week_for_stamp);
          sol.time_tow  = tow;
        } else {
          week_for_stamp = 0;
        }
        t_gpst = raw_.time;
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

    int week = 0;
    const gtime_t obs_time = raw_.obs.data[0].time;
    const double tow = time2gpst(obs_time, &week);

    msg::GnssObservations msg;
    const auto measurement_stamp = time_base_.stamp(
        use_gps_timestamp_, week > 0,
        obs_time, now(), get_logger(), *get_clock());
    if (!measurement_stamp) { receiver_health_.timeWithheld(); return; }
    msg.header.stamp = *measurement_stamp;
    msg.header.frame_id = frame_id_;
    msg.week = static_cast<uint32_t>(week);
    msg.tow  = tow;

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

  // Ephemeris Publishing (snapshot-on-change + heartbeat via EphemerisStore)

  void publishEphemerides() {
    bool changed = false;
    int prn = 0;
    const int sys = satsys(raw_.ephsat, &prn);
    if (sys == SYS_GLO && prn >= 1 && prn <= raw_.nav.ng) {
      changed = eph_store_.ingestGeph(raw_.nav.geph[prn-1]);
    } else if (raw_.ephsat > 0 && raw_.ephsat <= MAXSAT && sys != SYS_SBS) {
      const int index = raw_.ephsat-1 + MAXSAT*raw_.ephset;
      if (index >= 0 && index < raw_.nav.n)
        changed = eph_store_.ingestEph(raw_.nav.eph[index]);
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
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr  imu_raw_pub_;       // ExtSensorMeas accel/gyro (uncalibrated)
  rclcpp::TimerBase::SharedPtr timer_;

  wire::Framer wire_framer_{wire::Protocol::Sbf};
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
  SolutionSource         source_{SolutionSource::UNDETERMINED};
  std::chrono::steady_clock::time_point grace_start_{};
  bool                   grace_started_{false};
  bool                   saw_binary_during_grace_{false};
  Pending                pending_;

  // ENU local origin
  bool   has_local_origin_{false};
  double local_origin_ecef_[3]{0.0};
  double local_origin_pos_[3]{0.0};

  // SBF decoded frame payload
  uint16_t             sbf_id_{0};
  std::vector<uint8_t> sbf_body_;

  // ExtSensorMeas accumulator, keyed by week/TOW and Source/SensorModel
  uint32_t esm_tow_ms_{0xFFFFFFFFu};
  uint16_t esm_wnc_{0xFFFFu};
  uint16_t esm_sensor_{0xFFFFu};
  double   esm_accel_[3]{0.0, 0.0, 0.0};
  double   esm_gyro_[3]{0.0, 0.0, 0.0};
  bool     esm_has_accel_{false};
  bool     esm_has_gyro_{false};

  // Unified ephemeris store (shared between raw RTKLIB path and decoded *Nav path)
  gnss_utils::EphemerisStore eph_store_;
};

}  // namespace gnss_ros_standardization

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<gnss_ros_standardization::SbfDecoderNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("sbf_decoder_node"), "Startup/runtime failure: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
