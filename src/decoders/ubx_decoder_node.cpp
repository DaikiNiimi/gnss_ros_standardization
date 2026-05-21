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
constexpr double kPvtWatchdogTimeoutSec = 1.5;

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
    startPolling();

    RCLCPP_INFO(get_logger(), "UBX Decoder initialized");
  }

  ~UbxDecoderNode() override {
    try {
      strclose(&stream_);
    } catch (...) {
      // Ignore errors during cleanup
    }
    free_raw(&raw_);
    free_rtcm(&rtcm_);
  }

 private:
  // Initialization

  void initializeParameters() {
    declare_parameter<std::string>("stream_path", "serial:///dev/ttyACM0:115200");
    // frame_id: ROS TF frame name attached to the GNSS antenna phase center.
    // Default "gnss_link" — override in launch to integrate with vehicle TF tree.
    declare_parameter<std::string>("frame_id", "gnss_link");
    declare_parameter<std::string>("observation_topic", "/gnss/observation");
    declare_parameter<std::string>("ephemeris_topic", "/gnss/ephemeris");
    declare_parameter<std::string>("solution_topic", "/gnss/nmea_solution");
    declare_parameter<std::string>("imu_topic",     "/gnss/imu/data");
    declare_parameter<std::string>("imu_raw_topic", "/gnss/imu/data_raw");
    declare_parameter<double>("ephemeris.snapshot_period_s", 30.0);
    declare_parameter<double>("ephemeris.max_age_s", 0.0);  // 0 = keep all
    declare_parameter<bool>("use_gps_timestamp", false);
    declare_parameter<std::vector<double>>("origin", {0.0, 0.0, 0.0});
    eph_store_.setSnapshotPeriod(get_parameter("ephemeris.snapshot_period_s").as_double());
    eph_store_.setMaxAge(get_parameter("ephemeris.max_age_s").as_double());

    frame_id_ = get_parameter("frame_id").as_string();
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
    obs_pub_     = create_publisher<msg::GnssObservations>(get_parameter("observation_topic").as_string(), 10);
    eph_pub_     = create_publisher<msg::GnssEphemerides>(get_parameter("ephemeris_topic").as_string(), rclcpp::QoS(1).transient_local());
    sol_pub_     = create_publisher<msg::GnssSolution>(get_parameter("solution_topic").as_string(), 10);
    imu_pub_     = create_publisher<sensor_msgs::msg::Imu>(get_parameter("imu_topic").as_string(), 10);
    imu_raw_pub_ = create_publisher<sensor_msgs::msg::Imu>(get_parameter("imu_raw_topic").as_string(), 10);

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
    if (init_raw(&raw_, STRFMT_UBX) != 1) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize raw decoder");
      throw std::runtime_error("init_raw failed");
    }
    if (init_rtcm(&rtcm_) != 1) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize RTCM decoder");
      throw std::runtime_error("init_rtcm failed");
    }
  }

  void startPolling() {
    timer_ = create_wall_timer(kTimerInterval, std::bind(&UbxDecoderNode::pollStream, this));
  }

  // Stream Management

  void openStream() {
    const std::string stream_path = get_parameter("stream_path").as_string();
    std::string path = stream_path;

    // Match stream type from URI prefix
    int stream_type = 0;
    bool matched = false;
    for (const auto& def : ubx::kStreamTypes) {
      if (path.rfind(def.prefix, 0) == 0) {
        stream_type = def.type;
        path.erase(0, def.prefix.size());
        matched = true;
        break;
      }
    }

    if (!matched) {
      RCLCPP_ERROR(get_logger(), "Unsupported stream path format: %s", stream_path.c_str());
      throw std::runtime_error("Unsupported stream_path format");
    }

    if (stream_type == STR_SERIAL && path.rfind("/dev/", 0) == 0) {
      path.erase(0, 5);
    }

    std::vector<char> path_buf(path.begin(), path.end());
    path_buf.push_back('\0');

    if (!stropen(&stream_, stream_type, STR_MODE_R, path_buf.data())) {
      RCLCPP_ERROR(get_logger(), "Failed to open stream: %s", stream_path.c_str());
      throw std::runtime_error("stropen failed");
    }

    RCLCPP_INFO(get_logger(), "Stream opened: %s", stream_path.c_str());
  }

  void pollStream() {
    uint8_t buffer[ubx::READ_BUFFER_SIZE];
    const int bytes_read = strread(&stream_, buffer, sizeof(buffer));

    for (int i = 0; i < bytes_read; ++i) {
      uint8_t byte = buffer[i];

      // Feed RTKLIB for UBX binary parsing (obs/eph)
      const int result = input_ubx(&raw_, byte);
      handleDecodeResult(result);

      // Feed parallel UBX mini-framer for ESF-INS and NAV-ATT
      parseUbxByte(byte);

      // Feed NMEA text parser
      if (byte == '$') {
        nmea_buffer_.clear();
        nmea_buffer_.push_back(byte);
      } else if (!nmea_buffer_.empty()) {
        nmea_buffer_.push_back(byte);
        if (byte == '\n') {
          handleNmeaSentence(nmea_buffer_);
          nmea_buffer_.clear();
        } else if (nmea_buffer_.size() > ubx::NMEA_MAX_LINE_LEN) {
          nmea_buffer_.clear();
        }
      }
    }

    maybePublishHeartbeat();
    commitSourceLockIfDue();  // ensure grace finalizes even on idle stream
    maybeWatchdogFlushPendingPvt();
  }

  // UBX Mini-Framer (parallel to RTKLIB, for ESF-INS and NAV-ATT)

  void parseUbxByte(uint8_t byte) {
    switch (ubx_state_) {
      case 0:
        if (byte == ubx::SYNC1) ubx_state_ = 1;
        break;
      case 1:
        ubx_state_ = (byte == ubx::SYNC2) ? 2 : (byte == ubx::SYNC1 ? 1 : 0);
        break;
      case 2:
        ubx_cls_   = byte;
        ubx_state_ = 3;
        break;
      case 3:
        ubx_id_    = byte;
        ubx_state_ = 4;
        break;
      case 4:
        ubx_len_   = byte;
        ubx_state_ = 5;
        break;
      case 5:
        ubx_len_  |= static_cast<uint16_t>(byte << 8);
        ubx_payload_.clear();
        ubx_pos_   = 0;
        ubx_state_ = (ubx_len_ == 0) ? 7 : 6;
        break;
      case 6:
        ubx_payload_.push_back(byte);
        if (++ubx_pos_ >= ubx_len_) ubx_state_ = 7;
        break;
      case 7:  // CK_A (skip)
        ubx_state_ = 8;
        break;
      case 8:  // CK_B (skip), frame complete
        handleUbxFrame();
        ubx_state_ = 0;
        break;
      default:
        ubx_state_ = 0;
    }
  }

  void handleUbxFrame() {
    if (ubx_cls_ == ubx::CLASS_ESF && ubx_id_ == ubx::ID_ESF_RAW) handleEsfRaw();
    if (ubx_cls_ == ubx::CLASS_ESF && ubx_id_ == ubx::ID_ESF_INS) handleEsfIns();
    if (ubx_cls_ == ubx::CLASS_NAV && ubx_id_ == ubx::ID_NAV_PVT)     handleNavPvt();
    if (ubx_cls_ == ubx::CLASS_NAV && ubx_id_ == ubx::ID_NAV_DOP)     handleNavDop();
    if (ubx_cls_ == ubx::CLASS_NAV && ubx_id_ == ubx::ID_NAV_COV)     handleNavCov();
    if (ubx_cls_ == ubx::CLASS_NAV && ubx_id_ == ubx::ID_NAV_POSECEF) handleNavPosEcef();
    if (ubx_cls_ == ubx::CLASS_NAV && ubx_id_ == ubx::ID_NAV_VELECEF) handleNavVelEcef();
  }

  // Solution source policy (grace-period detection):
  //   Start in UNDETERMINED. Parse both NMEA and binary into their own
  //   buffers but publish nothing for the first kGracePeriodSec.
  //   If any binary PVT arrived during grace → lock BINARY (PVT wins when both
  //   are present in the stream). Else after grace expires → lock NMEA.
  // This guarantees deterministic, PVT-preferred behavior regardless of
  // whether NMEA or PVT happens to arrive first in the byte stream.
  enum class SolutionSource { UNDETERMINED, BINARY, NMEA };
  // 1.0 s safely covers a 1 Hz PVT cadence worst-case. Higher PVT rates lock
  // BINARY in milliseconds — grace only affects NMEA-only commit latency.
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

  // PVT TOW Aggregation (mirrors SBF decoder pattern)
  //
  // All NAV-* messages carry iTOW at payload offset 0. We buffer per-epoch into
  // PendingPvt and flush only when the set of cov blocks seen so far is
  // complete. cov_ever_seen is sticky and self-calibrates after the first
  // epoch of each cov type — so the flush condition adapts to whichever subset
  // of NAV-DOP/NAV-COV the receiver is configured to emit.
  // NAV-DOP is tracked separately in a persistent cache (last_dop_) so it can
  // survive Pending resets and be staleness-gated against the PVT cadence.
  static constexpr uint8_t COV_BIT_COV     = 0x1;
  static constexpr uint8_t COV_BIT_POSECEF = 0x2;
  static constexpr uint8_t COV_BIT_VELECEF = 0x4;

  struct PendingPvt {
    uint32_t tow_ms{UINT32_MAX};
    bool has_pvt{false};
    uint8_t cov_received{0};
    uint8_t cov_ever_seen{0};
    msg::GnssSolution buf{};
    rclcpp::Time last_update{0, 0, RCL_ROS_TIME};
  };

  gnss_utils::DopCache last_dop_;
  uint32_t prev_pvt_tow_ms_{UINT32_MAX};
  uint32_t pvt_period_ms_{0};

  // After a publish, record the (week, tow) to suppress orphan re-publish.
  uint32_t last_flushed_week_{0};
  uint32_t last_flushed_tow_ms_{UINT32_MAX};

  static uint32_t readItowMs(const uint8_t* p, size_t len) {
    if (len < 4) return UINT32_MAX;
    uint32_t v = 0;
    std::memcpy(&v, p, 4);
    return v;
  }

  bool pendingComplete() const {
    return pending_.has_pvt &&
           pending_.cov_received == pending_.cov_ever_seen;
  }

  // Orphan guard: if this block's TOW matches the last published TOW, it's a
  // late arrival from an eager-flush'd epoch. Update cov_ever_seen for learning
  // (so the next epoch waits for the full set), but do NOT re-accumulate into
  // pending_.buf (would cause an orphan duplicate publish at next TOW boundary).
  // NAV-* bodies don't carry GPS week directly; we fall back to TOW-only when
  // pending_.buf.time_week is 0 (it's only set at publishSolution time).
  bool isLateOrphan(uint32_t tow, uint8_t cov_bit) {
    const uint32_t week = pending_.buf.time_week;
    if (last_flushed_tow_ms_ != UINT32_MAX &&
        tow == last_flushed_tow_ms_ &&
        (week == 0 || week == last_flushed_week_)) {
      pending_.cov_ever_seen |= cov_bit;
      return true;
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
                               pending_.buf)) {
      return;
    }
    pending_.has_pvt = true;
    pending_.last_update = now();
    saw_binary_during_grace_ = true;
    commitSourceLockIfDue();
    if (pendingComplete()) flushPending();
  }

  // NAV-DOP: parsed into the persistent last_dop_ cache, NOT into pending_.
  // Does NOT participate in completion and does NOT trigger flush. At flush
  // time, applyDopWithStaleness() decides whether this cached DOP is fresh
  // enough (within 0..PVT_period after PVT) to populate the published solution.
  // NAV-DOP carries no GPS week → last_dop_.week = 0 (helper skips week check).
  void handleNavDop() {
    startGraceIfNeeded();
    const uint32_t tow = readItowMs(ubx_payload_.data(), ubx_payload_.size());
    msg::GnssSolution scratch{};
    if (!ubx::nav_dop::parseNavDop(ubx_payload_.data(), ubx_payload_.size(),
                                   scratch)) {
      return;
    }
    last_dop_.valid  = true;
    last_dop_.week   = 0;       // NAV-DOP has no week field
    last_dop_.tow_ms = tow;
    last_dop_.gdop   = scratch.gdop;
    last_dop_.pdop   = scratch.pdop;
    last_dop_.hdop   = scratch.hdop;
    last_dop_.vdop   = scratch.vdop;
    saw_binary_during_grace_ = true;
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
      return;
    }
    pending_.cov_received |= COV_BIT_COV;
    pending_.cov_ever_seen |= COV_BIT_COV;
    pending_.last_update = now();
    saw_binary_during_grace_ = true;
    commitSourceLockIfDue();
    if (pendingComplete()) flushPending();
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
      return;
    }
    pending_.cov_received  |= COV_BIT_POSECEF;
    pending_.cov_ever_seen |= COV_BIT_POSECEF;
    pending_.last_update = now();
    saw_binary_during_grace_ = true;
    commitSourceLockIfDue();
    if (pendingComplete()) flushPending();
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
      return;
    }
    pending_.cov_received  |= COV_BIT_VELECEF;
    pending_.cov_ever_seen |= COV_BIT_VELECEF;
    pending_.last_update = now();
    saw_binary_during_grace_ = true;
    commitSourceLockIfDue();
    if (pendingComplete()) flushPending();
  }

  void flushPending() {
    const uint8_t ever_seen = pending_.cov_ever_seen;
    const uint8_t recv      = pending_.cov_received;
    if (!pending_.has_pvt) {
      pending_ = {};
      pending_.cov_ever_seen = ever_seen;
      return;
    }
    // Snapshot ECEF-direct fields before LLH-driven derivation overwrites them.
    msg::GnssSolution ecef_direct = pending_.buf;
    binary_solution_ = pending_.buf;

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

    // ECEF-direct overrides: NAV-POSECEF / NAV-VELECEF take priority if seen.
    if (recv & COV_BIT_POSECEF) {
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

    // DOP staleness gate: populate from last_dop_ iff its timestamp is within
    // [0, PVT_period] of this epoch; else NaN. Asymmetric — no future-DOP bleed.
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
    if ((now() - pending_.last_update).seconds() > kPvtWatchdogTimeoutSec) {
      flushPending();
    }
  }

  // Sign-extend the lower 24 bits of a uint32 into an int32.
  static int32_t signExtend24(uint32_t v) {
    return (v & 0x00800000u) ? static_cast<int32_t>(v | 0xFF000000u)
                             : static_cast<int32_t>(v & 0x00FFFFFFu);
  }

  // UBX-ESF-RAW: 4-byte reserved header + N × { data(u4) + sTtag(u4) }.
  // data low 24 bits = signed measurement, high 8 bits = data type.
  void handleEsfRaw() {
    constexpr int kHeaderLen = 4;
    constexpr int kBlockLen  = 8;
    const int n = (static_cast<int>(ubx_payload_.size()) - kHeaderLen) / kBlockLen;
    if (n <= 0) return;

    double accel[3] = {0, 0, 0};
    double gyro[3]  = {0, 0, 0};
    bool   has_accel[3] = {false, false, false};
    bool   has_gyro[3]  = {false, false, false};
    const double deg2rad = M_PI / 180.0;

    for (int i = 0; i < n; ++i) {
      const uint8_t* p = ubx_payload_.data() + kHeaderLen + i * kBlockLen;
      uint32_t data = 0;
      std::memcpy(&data, p, 4);
      const uint8_t type = static_cast<uint8_t>((data >> 24) & 0xFFu);
      const int32_t val  = signExtend24(data);

      switch (type) {
        case ubx::esf_raw::TYPE_GYRO_X:
          gyro[0] = val * ubx::esf_raw::GYRO_SCALE * deg2rad; has_gyro[0] = true; break;
        case ubx::esf_raw::TYPE_GYRO_Y:
          gyro[1] = val * ubx::esf_raw::GYRO_SCALE * deg2rad; has_gyro[1] = true; break;
        case ubx::esf_raw::TYPE_GYRO_Z:
          gyro[2] = val * ubx::esf_raw::GYRO_SCALE * deg2rad; has_gyro[2] = true; break;
        case ubx::esf_raw::TYPE_ACCEL_X:
          accel[0] = val * ubx::esf_raw::ACCEL_SCALE; has_accel[0] = true; break;
        case ubx::esf_raw::TYPE_ACCEL_Y:
          accel[1] = val * ubx::esf_raw::ACCEL_SCALE; has_accel[1] = true; break;
        case ubx::esf_raw::TYPE_ACCEL_Z:
          accel[2] = val * ubx::esf_raw::ACCEL_SCALE; has_accel[2] = true; break;
        default: break;
      }
    }

    if (!(has_accel[0] || has_accel[1] || has_accel[2] ||
          has_gyro[0]  || has_gyro[1]  || has_gyro[2])) return;

    sensor_msgs::msg::Imu imu;
    imu.header.stamp    = now();
    imu.header.frame_id = frame_id_;

    imu.orientation_covariance[0] = -1.0;

    imu.angular_velocity.x = gyro[0];
    imu.angular_velocity.y = gyro[1];
    imu.angular_velocity.z = gyro[2];
    imu.linear_acceleration.x = accel[0];
    imu.linear_acceleration.y = accel[1];
    imu.linear_acceleration.z = accel[2];

    auto unk = ins::makeUnknownCovariance();
    std::copy(unk.begin(), unk.end(), imu.angular_velocity_covariance.begin());
    std::copy(unk.begin(), unk.end(), imu.linear_acceleration_covariance.begin());

    imu_raw_pub_->publish(imu);
  }

  void handleEsfIns() {
    if (static_cast<int>(ubx_payload_.size()) < ubx::esf_ins::MIN_LEN) return;

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

    sensor_msgs::msg::Imu imu;
    imu.header.stamp    = now();
    imu.header.frame_id = frame_id_;

    // orientation: not provided by ESF-INS — leave identity, mark unknown
    imu.orientation_covariance[0] = -1.0;

    imu.angular_velocity.x = xAngRate_raw * 1e-3 * deg2rad;
    imu.angular_velocity.y = yAngRate_raw * 1e-3 * deg2rad;
    imu.angular_velocity.z = zAngRate_raw * 1e-3 * deg2rad;
    auto ang_cov = ang_valid ? ins::makeDiagCovariance(1e-4, 1e-4, 1e-4) : ins::makeUnknownCovariance();
    std::copy(ang_cov.begin(), ang_cov.end(), imu.angular_velocity_covariance.begin());

    imu.linear_acceleration.x = xAccel_raw * 1e-2;
    imu.linear_acceleration.y = yAccel_raw * 1e-2;
    imu.linear_acceleration.z = zAccel_raw * 1e-2;
    auto acc_cov = acc_valid ? ins::makeDiagCovariance(1e-3, 1e-3, 1e-3) : ins::makeUnknownCovariance();
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
    switch (result) {
      case 1:  publishObservations(); break;
      case 2:  publishEphemerides();  break;
      default: break;
    }
  }

  // Solution Publishing

  void publishSolution(msg::GnssSolution& sol) {
    // BINARY path uses raw_.time (RTKLIB binary decoder timestamp); NMEA path
    // trusts time_week/time_tow already filled by NmeaParser from the sentence
    // itself.
    gtime_t t_gpst{};
    int week_for_stamp = 0;
    if (source_ == SolutionSource::BINARY) {
      const double tow = time2gpst(raw_.time, &week_for_stamp);
      if (week_for_stamp > 0) {
        sol.time_week = static_cast<uint32_t>(week_for_stamp);
        sol.time_tow  = tow;
      }
      sol.solution_source = msg::GnssSolution::SOLUTION_SOURCE_BINARY;
      t_gpst = raw_.time;
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

    if (sol.status == msg::GnssSolution::STATUS_FIX ||
        sol.status == msg::GnssSolution::STATUS_FLOAT ||
        sol.status == msg::GnssSolution::STATUS_SINGLE ||
        sol.status == msg::GnssSolution::STATUS_DGPS) {

      if (!has_local_origin_) {
        local_origin_ecef_[0] = sol.pos_ecef.x;
        local_origin_ecef_[1] = sol.pos_ecef.y;
        local_origin_ecef_[2] = sol.pos_ecef.z;
        ecef2pos(local_origin_ecef_, local_origin_pos_);
        has_local_origin_ = true;
        RCLCPP_INFO(get_logger(), "Set local ENU origin: lat=%.6f, lon=%.6f, alt=%.2f",
                    local_origin_pos_[0] * (180.0/M_PI), local_origin_pos_[1] * (180.0/M_PI), local_origin_pos_[2]);
      }

      sol.pos_enu_org_ecef.x = local_origin_ecef_[0];
      sol.pos_enu_org_ecef.y = local_origin_ecef_[1];
      sol.pos_enu_org_ecef.z = local_origin_ecef_[2];

      double ecef[3] = {
        sol.pos_ecef.x - local_origin_ecef_[0],
        sol.pos_ecef.y - local_origin_ecef_[1],
        sol.pos_ecef.z - local_origin_ecef_[2]
      };
      double enu[3] = {0};
      ecef2enu(local_origin_pos_, ecef, enu);

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

    // Use per-observation time, NOT raw_.time: RTKLIB advances raw_.time to
    // the next epoch when input_*()==1 fires (the trigger is the next epoch's
    // first byte); raw_.obs.data[*] still holds the just-completed epoch.
    int week = 0;
    const gtime_t obs_time = raw_.obs.data[0].time;
    const double tow = time2gpst(obs_time, &week);

    msg::GnssObservations msg;
    msg.header.stamp    = (use_gps_timestamp_ && week > 0)
                          ? gnss_utils::gpstToUtcRosTime(obs_time) : now();
    msg.header.frame_id = frame_id_;
    msg.week = static_cast<uint32_t>(week);
    msg.tow = tow;

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

  // Ephemeris Publishing

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

  rclcpp::Publisher<msg::GnssObservations>::SharedPtr   obs_pub_;
  rclcpp::Publisher<msg::GnssEphemerides>::SharedPtr    eph_pub_;
  rclcpp::Publisher<msg::GnssSolution>::SharedPtr       sol_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr   imu_pub_;       // ESF-INS (calibrated)
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr   imu_raw_pub_;   // ESF-RAW (uncalibrated)
  rclcpp::TimerBase::SharedPtr timer_;

  stream_t stream_{};
  raw_t    raw_{};
  rtcm_t   rtcm_{};

  // NMEA parsing state
  gnss_utils::NmeaParser nmea_parser_;
  std::string            nmea_buffer_;
  msg::GnssSolution      nmea_solution_;
  msg::GnssSolution      binary_solution_;
  PendingPvt             pending_;
  SolutionSource         source_{SolutionSource::UNDETERMINED};
  rclcpp::Time           grace_start_{0, 0, RCL_ROS_TIME};
  bool                   grace_started_{false};
  bool                   saw_binary_during_grace_{false};

  // ENU origin
  double origin_latitude_{0.0};
  double origin_longitude_{0.0};
  double origin_altitude_{0.0};
  bool   has_local_origin_{false};
  double local_origin_ecef_[3]{0.0};
  double local_origin_pos_[3]{0.0};

  // UBX mini-framer state (ESF-INS / ESF-RAW)
  int                  ubx_state_{0};
  uint8_t              ubx_cls_{0};
  uint8_t              ubx_id_{0};
  uint16_t             ubx_len_{0};
  uint16_t             ubx_pos_{0};
  std::vector<uint8_t> ubx_payload_;

  // Ephemeris dedup
  gnss_utils::EphemerisStore eph_store_;
};

}  // namespace gnss_ros_standardization

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gnss_ros_standardization::UbxDecoderNode>());
  rclcpp::shutdown();
  return 0;
}