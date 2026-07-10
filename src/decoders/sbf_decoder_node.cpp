// SPDX-License-Identifier: MIT
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
#include "gnss_ros_standardization/sbf_protocol.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"

using namespace std::chrono_literals;
namespace ins = gnss_ros_standardization::ins_utils;

namespace gnss_ros_standardization {

namespace {

constexpr auto kTimerInterval  = 10ms;
constexpr double kPvtWatchdogTimeoutSec = 1.5;

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

  void initializeParameters() {
    declare_parameter<std::string>("stream_path", "serial:///dev/ttyUSB0:115200");
    // frame_id: ROS TF frame name attached to the GNSS antenna phase center.
    // Default "gnss_link" — override in launch to integrate with vehicle TF tree.
    declare_parameter<std::string>("frame_id", "gnss_link");
    declare_parameter<std::string>("observation_topic", "/gnss/observation");
    declare_parameter<std::string>("ephemeris_topic", "/gnss/ephemeris");
    declare_parameter<std::string>("solution_topic",     "/gnss/nmea_solution");
    declare_parameter<std::string>("imu_raw_topic",      "/gnss/imu/data_raw");
    declare_parameter<double>("ephemeris.snapshot_period_s", 30.0);
    // 0.0 disables aging: keep every received eph so post-processing tools
    // (rosbag_to_rinex etc.) can output them even when observations from
    // earlier in the recording are processed later.
    declare_parameter<double>("ephemeris.max_age_s", 0.0);

    declare_parameter<bool>("use_gps_timestamp", false);
    declare_parameter<std::vector<double>>("origin", {0.0, 0.0, 0.0});

    // RTCM relay listen URI (opt-in, see openRtcmRelay). Empty = disabled.
    declare_parameter<std::string>("rtcm_relay", "");

    frame_id_ = get_parameter("frame_id").as_string();
    eph_store_.setSnapshotPeriod(get_parameter("ephemeris.snapshot_period_s").as_double());
    eph_store_.setMaxAge(get_parameter("ephemeris.max_age_s").as_double());
    use_gps_timestamp_ = get_parameter("use_gps_timestamp").as_bool();
    {
      const auto v = get_parameter("origin").as_double_array();
      if (v.size() == 3) {
        origin_latitude_  = v[0];
        origin_longitude_ = v[1];
        origin_altitude_  = v[2];
      }
    }
  }

  void initializePublishers() {
    obs_pub_          = create_publisher<msg::GnssObservations>(get_parameter("observation_topic").as_string(), 10);
    eph_pub_          = create_publisher<msg::GnssEphemerides>(get_parameter("ephemeris_topic").as_string(), rclcpp::QoS(1).transient_local());
    sol_pub_          = create_publisher<msg::GnssSolution>(get_parameter("solution_topic").as_string(), 10);
    imu_raw_pub_      = create_publisher<sensor_msgs::msg::Imu>(get_parameter("imu_raw_topic").as_string(), 10);

    if (origin_latitude_ != 0.0 || origin_longitude_ != 0.0) {
      local_origin_pos_[0] = origin_latitude_  * (M_PI / 180.0);
      local_origin_pos_[1] = origin_longitude_ * (M_PI / 180.0);
      local_origin_pos_[2] = origin_altitude_;
      pos2ecef(local_origin_pos_, local_origin_ecef_);
      has_local_origin_ = true;
      RCLCPP_INFO(get_logger(), "ENU origin set from config: lat=%.6f lon=%.6f alt=%.2f",
        origin_latitude_, origin_longitude_, origin_altitude_);
    }
    logConfiguration();
  }

  void logConfiguration() {
    auto on = [](bool v) { return v ? "ON" : "OFF"; };
    RCLCPP_INFO(get_logger(), "Configuration:");
    RCLCPP_INFO(get_logger(), "  GPS timestamp : %s", on(use_gps_timestamp_));
    if (origin_latitude_ != 0.0 || origin_longitude_ != 0.0) {
      RCLCPP_INFO(get_logger(), "  ENU origin    : fixed (lat=%.6f lon=%.6f alt=%.2f)",
        origin_latitude_, origin_longitude_, origin_altitude_);
    } else {
      RCLCPP_INFO(get_logger(), "  ENU origin    : auto (first valid solution)");
    }
  }

  void initializeDecoder() {
    if (init_raw(&raw_, STRFMT_SEPT) != 1) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize raw decoder");
      throw std::runtime_error("init_raw failed");
    }
  }

  void startPolling() {
    timer_ = create_wall_timer(kTimerInterval, std::bind(&SbfDecoderNode::pollStream, this));
  }

  // Stream Management

  void openStream() {
    // The RTCM relay writes corrections back down this stream, so it needs a
    // read-write open; without relay keep the passive read-only open.
    const int mode = get_parameter("rtcm_relay").as_string().empty()
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
    const std::string listen = get_parameter("rtcm_relay").as_string();
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

  void pollStream() {
    // RTCM relay: drain corrections from the TCP server and write them to the
    // receiver stream. Same poll thread as the read below, so no lock.
    relay_.drainTo(get_logger(), *get_clock(), stream_);

    uint8_t buffer[sbf::READ_BUFFER_SIZE];
    const int bytes_read = strread(&stream_, buffer, sizeof(buffer));

    for (int i = 0; i < bytes_read; ++i) {
      const uint8_t byte = buffer[i];

      // Feed SBF binary decoder (obs/eph)
      const int result = input_sbf(&raw_, byte);
      handleDecodeResult(result);

      // Parallel SBF mini-framer (ExtSensorMeas / PVT blocks)
      parseSbfByte(byte);

      // Feed NMEA text parser (SBF and NMEA are interleaved on the same stream).
      // SBF blocks start with '$@' (sync1=0x24, sync2=0x40), so a plain
      // `byte=='$'` capture-start would fire on every SBF block and shred any
      // in-progress NMEA capture. Peek one byte after '$' and only start NMEA
      // capture when followed by an ASCII uppercase letter (talker ID).
      if (nmea_pending_dollar_) {
        nmea_pending_dollar_ = false;
        if (byte == sbf::SBF_SYNC2) {
          // SBF block start — leave nmea_buffer_ untouched.
        } else if (std::isupper(static_cast<unsigned char>(byte))) {
          nmea_buffer_.clear();
          nmea_buffer_.push_back('$');
          nmea_buffer_.push_back(byte);
        }
      } else if (byte == '$') {
        nmea_pending_dollar_ = true;
      } else if (!nmea_buffer_.empty()) {
        nmea_buffer_.push_back(byte);
        if (byte == '\n') {
          handleNmeaSentence(nmea_buffer_);
          nmea_buffer_.clear();
        } else if (nmea_buffer_.size() > sbf::NMEA_MAX_LINE_LEN) {
          nmea_buffer_.clear();
        }
      }
    }

    maybePublishHeartbeat();
    maybeWatchdogFlushPendingPvt();
    commitSourceLockIfDue();  // ensure grace finalizes even on idle stream
  }

  // SBF Mini-Framer (parallel to RTKLIB, for ExtSensorMeas and PVT blocks)
  //
  // SBF block layout: SYNC1('$') SYNC2('@') CRC(2) ID(2,LE) Length(2,LE) Body(Length-8)
  // ID bits 0-12 are the block number; bits 13-15 are the revision.

  void parseSbfByte(uint8_t byte) {
    switch (sbf_state_) {
      case 0:  // wait SYNC1
        if (byte == sbf::SBF_SYNC1) sbf_state_ = 1;
        break;
      case 1:  // wait SYNC2
        sbf_state_ = (byte == sbf::SBF_SYNC2) ? 2 : (byte == sbf::SBF_SYNC1 ? 1 : 0);
        break;
      case 2: sbf_state_ = 3; break;  // CRC LSB (skip)
      case 3: sbf_state_ = 4; break;  // CRC MSB (skip)
      case 4: sbf_id_ = byte;  sbf_state_ = 5; break;  // ID LSB
      case 5:
        sbf_id_ |= static_cast<uint16_t>(byte << 8);
        sbf_id_ &= 0x1FFF;  // mask off revision bits 13-15
        sbf_state_ = 6;
        break;
      case 6: sbf_len_ = byte; sbf_state_ = 7; break;  // Length LSB
      case 7:
        sbf_len_ |= static_cast<uint16_t>(byte << 8);
        sbf_body_.clear();
        sbf_body_pos_ = 0;
        if (sbf_len_ <= 8) { handleSbfBlock(); sbf_state_ = 0; }
        else { sbf_body_.reserve(sbf_len_ - 8); sbf_state_ = 8; }
        break;
      case 8:
        sbf_body_.push_back(byte);
        if (++sbf_body_pos_ >= sbf_len_ - 8) { handleSbfBlock(); sbf_state_ = 0; }
        break;
      default:
        sbf_state_ = 0;
    }
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
    rclcpp::Time last_update{0, 0, RCL_ROS_TIME};
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
      grace_start_ = now();
      grace_started_ = true;
    }
  }

  void commitSourceLockIfDue() {
    if (source_ != SolutionSource::UNDETERMINED) return;
    if (!grace_started_) return;
    const bool elapsed = (now() - grace_start_).seconds() >= kGracePeriodSec;
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

  bool pendingComplete() const {
    return pending_.blocks_received == pending_.blocks_ever_seen;
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
      return;
    }

    if (tow != pending_.tow_ms && pending_.blocks_received != 0) {
      flushPending();  // TOW boundary — flush previous epoch
    }
    pending_.tow_ms = tow;
    if (!parse(p, len, pending_.buf)) return;
    pending_.blocks_received  |= bit;
    pending_.blocks_ever_seen |= bit;
    pending_.last_update = now();
    markBinarySeen();
    if (pendingComplete()) flushPending();
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
    if (!sbf::pvt::parseDop(p, len, scratch)) return;
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
    if ((now() - pending_.last_update).seconds() > kPvtWatchdogTimeoutSec) {
      flushPending();
    }
  }

  void handleExtSensorMeas() {
    if (static_cast<int>(sbf_body_.size()) < sbf::ext_sensor_meas::MIN_BODY_LEN) return;

    const uint8_t n         = sbf_body_[sbf::ext_sensor_meas::OFFSET_N];
    const uint8_t sb_length = sbf_body_[sbf::ext_sensor_meas::OFFSET_SB_LENGTH];
    if (sb_length < sbf::ext_sensor_meas::SB_MIN_LEN) return;
    if (static_cast<int>(sbf_body_.size()) < sbf::ext_sensor_meas::MIN_BODY_LEN + n * sb_length) return;

    // TOW [ms] to detect epoch boundaries
    uint32_t tow_ms = 0;
    std::memcpy(&tow_ms, sbf_body_.data(), 4);

    // New epoch: publish accumulated data from the previous epoch, then reset
    if (tow_ms != esm_tow_ms_ && (esm_has_accel_ || esm_has_gyro_)) {
      publishEsmAccum();
    }
    esm_tow_ms_ = tow_ms;
    std::memcpy(&esm_wnc_, sbf_body_.data() + 4, 2);

    // Parse sub-blocks: each Type carries a full 3-vector
    for (uint8_t i = 0; i < n; ++i) {
      const int base = sbf::ext_sensor_meas::OFFSET_SUBBLOCKS + i * sb_length;
      const uint8_t type = sbf_body_[base + sbf::ext_sensor_meas::SB_OFFSET_TYPE];

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
      }
    }

    // Publish immediately when both accel and gyro are accumulated
    if (esm_has_accel_ && esm_has_gyro_) publishEsmAccum();
  }

  void publishEsmAccum() {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp    = (use_gps_timestamp_ && esm_tow_ms_ != 0xFFFFFFFFu && esm_wnc_ != 0xFFFFu)
        ? gnss_utils::gpstToUtcRosTime(gpst2time(adjgpsweek(static_cast<int>(esm_wnc_)), esm_tow_ms_ / 1000.0))
        : now();
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
    switch (result) {
      case 1:  publishObservations(); break;
      case 2:  publishEphemerides();  break;
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

    sol.header.stamp    = (use_gps_timestamp_ && week_for_stamp > 0)
                          ? gnss_utils::gpstToUtcRosTime(t_gpst) : now();
    sol.header.frame_id = frame_id_;

    const bool has_fix =
      sol.status == msg::GnssSolution::STATUS_FIX    ||
      sol.status == msg::GnssSolution::STATUS_FLOAT  ||
      sol.status == msg::GnssSolution::STATUS_SINGLE ||
      sol.status == msg::GnssSolution::STATUS_DGPS;

    if (has_fix) {
      if (!has_local_origin_) {
        local_origin_ecef_[0] = sol.pos_ecef.x;
        local_origin_ecef_[1] = sol.pos_ecef.y;
        local_origin_ecef_[2] = sol.pos_ecef.z;
        ecef2pos(local_origin_ecef_, local_origin_pos_);
        has_local_origin_ = true;
        RCLCPP_INFO(get_logger(), "Set local ENU origin: lat=%.6f lon=%.6f alt=%.2f",
          local_origin_pos_[0] * (180.0 / M_PI),
          local_origin_pos_[1] * (180.0 / M_PI),
          local_origin_pos_[2]);
      }

      sol.pos_enu_org_ecef.x = local_origin_ecef_[0];
      sol.pos_enu_org_ecef.y = local_origin_ecef_[1];
      sol.pos_enu_org_ecef.z = local_origin_ecef_[2];

      double d_ecef[3] = {
        sol.pos_ecef.x - local_origin_ecef_[0],
        sol.pos_ecef.y - local_origin_ecef_[1],
        sol.pos_ecef.z - local_origin_ecef_[2]
      };
      double enu[3] = {0};
      ecef2enu(local_origin_pos_, d_ecef, enu);
      sol.pos_enu.x = enu[0];
      sol.pos_enu.y = enu[1];
      sol.pos_enu.z = enu[2];

      // vel_enu uses CURRENT-position frame (matches msg comment & receiver convention).
      double vel_ecef[3] = {sol.vel_ecef.x, sol.vel_ecef.y, sol.vel_ecef.z};
      double vel_enu[3] = {0};
      const double cur_llh[3] = {sol.latitude * D2R, sol.longitude * D2R, sol.altitude};
      ecef2enu(cur_llh, vel_ecef, vel_enu);
      sol.vel_enu.x = vel_enu[0];
      sol.vel_enu.y = vel_enu[1];
      sol.vel_enu.z = vel_enu[2];
    } else {
      sol.pos_enu.x = std::numeric_limits<double>::quiet_NaN();
      sol.pos_enu.y = std::numeric_limits<double>::quiet_NaN();
      sol.pos_enu.z = std::numeric_limits<double>::quiet_NaN();
      sol.vel_enu.x = std::numeric_limits<double>::quiet_NaN();
      sol.vel_enu.y = std::numeric_limits<double>::quiet_NaN();
      sol.vel_enu.z = std::numeric_limits<double>::quiet_NaN();
    }

    sol_pub_->publish(sol);
  }

  // Observation Publishing

  void publishObservations() {
    if (raw_.obs.n <= 0) return;

    int week = 0;
    const gtime_t obs_time = raw_.obs.data[0].time;
    const double tow = time2gpst(obs_time, &week);

    msg::GnssObservations msg;
    msg.header.stamp    = (use_gps_timestamp_ && week > 0)
                          ? gnss_utils::gpstToUtcRosTime(obs_time) : now();
    msg.header.frame_id = frame_id_;
    msg.week = static_cast<uint32_t>(week);
    msg.tow  = tow;

    decoder_common::SatelliteCount sat_count{};
    for (int i = 0; i < raw_.obs.n; ++i) {
      const obsd_t& obs = raw_.obs.data[i];
      decoder_common::countSatellite(obs.sat, sat_count);
      decoder_common::appendObservations(obs, msg.observations);
    }

    obs_pub_->publish(msg);

    RCLCPP_INFO(get_logger(),
      "Published obs: week=%d tow=%.3f n=%zu sats(G/R/E/J/C/I/S/U)=(%d/%d/%d/%d/%d/%d/%d/%d)",
      week, tow, msg.observations.size(),
      sat_count.gps, sat_count.glo, sat_count.gal, sat_count.qzs,
      sat_count.bds, sat_count.irn, sat_count.sbs, sat_count.unknown);
  }

  // Ephemeris Publishing (snapshot-on-change + heartbeat via EphemerisStore)

  void publishEphemerides() {
    bool changed = false;
    for (int i = 0; i < raw_.nav.n; ++i) {
      changed = eph_store_.ingestEph(raw_.nav.eph[i]) || changed;
    }
    for (int i = 0; i < raw_.nav.ng; ++i) {
      changed = eph_store_.ingestGeph(raw_.nav.geph[i]) || changed;
    }
    if (changed) publishSnapshot();
  }

  void maybePublishHeartbeat() {
    if (eph_store_.heartbeatDue(now())) publishSnapshot();
  }

  void publishSnapshot() {
    auto m = eph_store_.buildSnapshot(now());
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

  stream_t stream_{};
  raw_t    raw_{};

  // RTCM relay (PC -> receiver): STRSVR-style TCP server -> receiver stream
  decoder_common::RtcmRelayServer relay_;

  // NMEA parsing state
  gnss_utils::NmeaParser nmea_parser_;
  std::string            nmea_buffer_;
  bool                   nmea_pending_dollar_{false};  // saw '$', awaiting next byte to disambiguate from SBF '$@'
  msg::GnssSolution      nmea_solution_;
  msg::GnssSolution      binary_solution_;
  SolutionSource         source_{SolutionSource::UNDETERMINED};
  rclcpp::Time           grace_start_{0, 0, RCL_ROS_TIME};
  bool                   grace_started_{false};
  bool                   saw_binary_during_grace_{false};
  Pending                pending_;

  // ENU local origin
  double origin_latitude_{0.0};
  double origin_longitude_{0.0};
  double origin_altitude_{0.0};
  bool   has_local_origin_{false};
  double local_origin_ecef_[3]{0.0};
  double local_origin_pos_[3]{0.0};

  // SBF mini-framer state
  int                  sbf_state_{0};
  uint16_t             sbf_id_{0};
  uint16_t             sbf_len_{0};
  int                  sbf_body_pos_{0};
  std::vector<uint8_t> sbf_body_;

  // ExtSensorMeas accumulator (TOW-based, collects accel + gyro 3-vectors across blocks)
  uint32_t esm_tow_ms_{0xFFFFFFFFu};
  uint16_t esm_wnc_{0xFFFFu};
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
  rclcpp::spin(std::make_shared<gnss_ros_standardization::SbfDecoderNode>());
  rclcpp::shutdown();
  return 0;
}