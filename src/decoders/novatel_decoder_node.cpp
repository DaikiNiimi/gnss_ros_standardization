// SPDX-License-Identifier: MIT
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
#include "gnss_ros_standardization/novatel_imu_scales.hpp"
#include "gnss_ros_standardization/novatel_protocol.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"

using namespace std::chrono_literals;
namespace ins = gnss_ros_standardization::ins_utils;

namespace gnss_ros_standardization {

namespace {

constexpr auto kTimerInterval  = 10ms;
constexpr double kPvtWatchdogTimeoutSec = 1.5;

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

    RCLCPP_INFO(get_logger(), "NovAtel Decoder initialized (format: %s)", format_.c_str());
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

  void initializeParameters() {
    declare_parameter<std::string>("stream_path", "serial:///dev/ttyUSB0:115200");
    declare_parameter<std::string>("format", "oem4");
    // frame_id: ROS TF frame name attached to the GNSS antenna phase center.
    // Default "gnss_link" — override in launch to integrate with vehicle TF tree.
    declare_parameter<std::string>("frame_id", "gnss_link");
    declare_parameter<std::string>("observation_topic", "/gnss/observation");
    declare_parameter<std::string>("ephemeris_topic", "/gnss/ephemeris");
    declare_parameter<std::string>("solution_topic", "/gnss/nmea_solution");
    declare_parameter<std::string>("imu_topic",      "/gnss/imu/data");
    declare_parameter<std::string>("imu_raw_topic",  "/gnss/imu/data_raw");
    declare_parameter<double>("imu_scale_override.accel", 0.0);
    declare_parameter<double>("imu_scale_override.gyro",  0.0);
    declare_parameter<double>("ephemeris.snapshot_period_s", 30.0);
    declare_parameter<double>("ephemeris.max_age_s", 0.0);  // 0 = keep all
    declare_parameter<bool>("use_gps_timestamp", false);
    declare_parameter<std::vector<double>>("origin", {0.0, 0.0, 0.0});
    // RTCM relay (opt-in, see openRtcmRelay): listen URI for corrections plus
    // the dedicated RTCMV3 output port. See src/decoders/README.md.
    declare_parameter<std::string>("rtcm_relay", "");
    declare_parameter<std::string>("rtcm_relay_output", "");
    declare_parameter<std::string>("rtcm_relay_correction_port", "THISPORT");
    use_gps_timestamp_ = get_parameter("use_gps_timestamp").as_bool();
    {
      const auto v = get_parameter("origin").as_double_array();
      if (v.size() == 3) {
        origin_latitude_  = v[0];
        origin_longitude_ = v[1];
        origin_altitude_  = v[2];
      }
    }
    eph_store_.setSnapshotPeriod(get_parameter("ephemeris.snapshot_period_s").as_double());
    eph_store_.setMaxAge(get_parameter("ephemeris.max_age_s").as_double());

    format_   = get_parameter("format").as_string();
    frame_id_ = get_parameter("frame_id").as_string();

    if (format_ != "oem4") {
      RCLCPP_WARN(get_logger(), "Unknown format '%s', defaulting to oem4", format_.c_str());
      format_ = "oem4";
    }
  }

  void initializePublishers() {
    obs_pub_ = create_publisher<msg::GnssObservations>(get_parameter("observation_topic").as_string(), 10);
    eph_pub_ = create_publisher<msg::GnssEphemerides>(get_parameter("ephemeris_topic").as_string(), rclcpp::QoS(1).transient_local());
    sol_pub_ = create_publisher<msg::GnssSolution>(get_parameter("solution_topic").as_string(), 10);
    imu_pub_     = create_publisher<sensor_msgs::msg::Imu>(get_parameter("imu_topic").as_string(), 10);
    imu_raw_pub_ = create_publisher<sensor_msgs::msg::Imu>(get_parameter("imu_raw_topic").as_string(), 10);
    imu_scale_override_accel_ = get_parameter("imu_scale_override.accel").as_double();
    imu_scale_override_gyro_  = get_parameter("imu_scale_override.gyro").as_double();

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
  }

  void startPolling() {
    timer_ = create_wall_timer(kTimerInterval, std::bind(&NovatelDecoderNode::pollStream, this));
  }

  // Stream Management

  void openStream() {
    // Main-port RW is only needed for the single-port relay fallback (no
    // dedicated output); otherwise keep the passive read-only open.
    const bool relay_to_main =
        !get_parameter("rtcm_relay").as_string().empty() &&
        get_parameter("rtcm_relay_output").as_string().empty();
    if (!decoder_common::openStreamPath(get_logger(), stream_,
                                        get_parameter("stream_path").as_string(),
                                        relay_to_main ? STR_MODE_RW : STR_MODE_R,
                                        "Input")) {
      throw std::runtime_error("stropen failed");
    }
  }

  // RTCM relay (RTKLIB STRSVR-style): host a TCP server for incoming
  // base-station corrections and write them to the receiver in pollStream().
  // NovAtel: corrections go to a DEDICATED port (`rtcm_relay_output`), same
  // design as novatel_driver_node — the port is put into RTCMV3 input mode by
  // sending INTERFACEMODE over that very port (THISPORT), so the user never
  // needs its NovAtel logical name and the main decoded-log port stays
  // untouched (passive).
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

    const std::string output = get_parameter("rtcm_relay_output").as_string();
    if (output.empty()) {
      RCLCPP_WARN(get_logger(),
        "rtcm_relay_output is empty: corrections go to the main port. NovAtel "
        "cannot mix RTCM input with NOVATEL logs on one logical port — set "
        "rtcm_relay_output to a second /dev device for on-chip RTK.");
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
        get_parameter("rtcm_relay_correction_port").as_string();
    const std::string cmd = std::string(novatel::CMD_INTERFACEMODE) + " " +
                            correction_port + " RTCMV3 NONE OFF\r\n";
    strwrite(&relay_out_, (uint8_t*)cmd.c_str(), static_cast<int>(cmd.size()));
    std::this_thread::sleep_for(200ms);
    RCLCPP_INFO(get_logger(),
      "RTCM relay: '%s' -> dedicated port '%s' (set to RTCMV3 via INTERFACEMODE %s)",
      listen.c_str(), output.c_str(), correction_port.c_str());
  }

  // Polling

  void pollStream() {
    // RTCM relay: drain corrections from the TCP server and write them to the
    // dedicated RTCMV3 port if configured, else the main stream (single-port
    // fallback). Same poll thread as the read below, so no lock.
    relay_.drainTo(get_logger(), *get_clock(),
                   relay_out_open_ ? relay_out_ : stream_);

    uint8_t buffer[novatel::READ_BUFFER_SIZE];
    const int bytes_read = strread(&stream_, buffer, sizeof(buffer));

    for (int i = 0; i < bytes_read; ++i) {
      const uint8_t byte = buffer[i];

      // Feed NovAtel binary decoder
      int result = input_oem4(&raw_, byte);
      handleDecodeResult(result);

      // Parallel OEM4 mini-framer (IMU / PVT logs)
      parseOem4Byte(byte);

      // Feed NMEA text parser (NovAtel binary and NMEA are interleaved)
      if (byte == '$') {
        nmea_buffer_.clear();
        nmea_buffer_.push_back(byte);
      } else if (!nmea_buffer_.empty()) {
        nmea_buffer_.push_back(byte);
        if (byte == '\n') {
          handleNmeaSentence(nmea_buffer_);
          nmea_buffer_.clear();
        } else if (nmea_buffer_.size() > novatel::NMEA_MAX_LINE_LEN) {
          nmea_buffer_.clear();
        }
      }
    }

    maybePublishHeartbeat();
    commitSourceLockIfDue();  // ensure grace finalizes even on idle stream
    maybeWatchdogFlushPendingPvt();
  }

  // OEM4 Mini-Framer (parallel to RTKLIB, for IMU and PVT logs)
  //
  // OEM4 binary frame: SYNC(0xAA,0x44,0x12) HDRLEN(1) MSGID(2,LE) MSGTYPE(1)
  //                    PORT(1) MSGLEN(2,LE) ... rest of header ... BODY CRC32

  void parseOem4Byte(uint8_t byte) {
    switch (oem4_state_) {
      case 0:  if (byte == novatel::OEM4_SYNC1) oem4_state_ = 1; break;
      case 1:  oem4_state_ = (byte == novatel::OEM4_SYNC2) ? 2 : (byte == novatel::OEM4_SYNC1 ? 1 : 0); break;
      case 2:  oem4_state_ = (byte == novatel::OEM4_SYNC3) ? 3 : (byte == novatel::OEM4_SYNC1 ? 1 : 0); break;
      case 3:  oem4_hdr_len_  = byte; oem4_state_ = 4; break;  // header length
      case 4:  oem4_msg_id_   = byte; oem4_state_ = 5; break;  // MSGID LSB
      case 5:  oem4_msg_id_  |= static_cast<uint16_t>(byte << 8); oem4_state_ = 6; break;  // MSGID MSB
      case 6:  oem4_state_ = 7; break;  // MSGTYPE skip
      case 7:  oem4_state_ = 8; break;  // PORT skip
      case 8:  oem4_msg_len_  = byte; oem4_state_ = 9; break;  // MSGLEN LSB
      case 9:
        oem4_msg_len_ |= static_cast<uint16_t>(byte << 8);
        oem4_hdr_rem_  = static_cast<int>(oem4_hdr_len_) - 10;  // bytes already consumed: SYNC(3)+HDRLEN(1)+MSGID(2)+MSGTYPE(1)+PORT(1)+MSGLEN(2)=10
        oem4_hdr_pos_  = 10;  // next byte's offset within the OEM4 header
        oem4_gps_week_ = 0;
        oem4_gps_ms_   = 0;
        oem4_body_.clear();
        oem4_body_pos_ = 0;
        oem4_state_    = (oem4_hdr_rem_ > 0) ? 10 : 11;
        break;
      case 10:  // skip remaining header bytes, capturing GPS week (off 14-15) and ms (off 16-19)
        if (oem4_hdr_pos_ == 14)      oem4_gps_week_  = byte;
        else if (oem4_hdr_pos_ == 15) oem4_gps_week_ |= static_cast<uint32_t>(byte) << 8;
        else if (oem4_hdr_pos_ == 16) oem4_gps_ms_    = byte;
        else if (oem4_hdr_pos_ == 17) oem4_gps_ms_   |= static_cast<uint32_t>(byte) <<  8;
        else if (oem4_hdr_pos_ == 18) oem4_gps_ms_   |= static_cast<uint32_t>(byte) << 16;
        else if (oem4_hdr_pos_ == 19) oem4_gps_ms_   |= static_cast<uint32_t>(byte) << 24;
        ++oem4_hdr_pos_;
        if (--oem4_hdr_rem_ <= 0) oem4_state_ = (oem4_msg_len_ > 0) ? 11 : 13;
        break;
      case 11:  // collect body
        oem4_body_.push_back(byte);
        if (++oem4_body_pos_ >= oem4_msg_len_) { handleOem4Frame(); oem4_state_ = 12; }
        break;
      case 12: oem4_state_ = 13; break;  // CRC bytes skip (4 total)
      case 13: oem4_state_ = 14; break;
      case 14: oem4_state_ = 15; break;
      case 15: oem4_state_ = 0; break;
      default: oem4_state_ = 0;
    }
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

  // PVT TOW aggregation: BEST* logs merge into pending_ keyed on the OEM4
  // header TOW (oem4_gps_ms_); PSRDOP is cached separately (last_dop_).
  static constexpr uint8_t COV_BIT_VEL = 0x1;
  static constexpr uint8_t COV_BIT_XYZ = 0x2;

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

  bool pendingComplete() const {
    return pending_.has_pvt &&
           pending_.cov_received == pending_.cov_ever_seen;
  }

  // Orphan guard: same TOW as last published → only update learning.
  bool isLateOrphan(uint8_t cov_bit) {
    if (last_flushed_tow_ms_ != UINT32_MAX &&
        oem4_gps_ms_   == last_flushed_tow_ms_ &&
        oem4_gps_week_ == last_flushed_week_) {
      pending_.cov_ever_seen |= cov_bit;
      return true;
    }
    return false;
  }

  void aggregateEpochBoundary() {
    if (oem4_gps_ms_ != pending_.tow_ms && (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = oem4_gps_ms_;
    // BEST*/PSRDOP bodies carry no GPS time — (week, ms) comes from the OEM4
    // header seen by the mini-framer.
    pending_.buf.time_week = oem4_gps_week_;
    pending_.buf.time_tow  = oem4_gps_ms_ / 1000.0;
  }

  void handleBestPos() {
    startGraceIfNeeded();
    if (isLateOrphan(0)) return;
    aggregateEpochBoundary();
    if (!novatel::pvt::parseBESTPOS(oem4_body_.data(), oem4_body_.size(), pending_.buf)) return;
    pending_.has_pvt = true;
    pending_.last_update = now();
    saw_binary_during_grace_ = true;
    commitSourceLockIfDue();
    if (pendingComplete()) flushPending();
  }

  void handleBestVel() {
    startGraceIfNeeded();
    if (isLateOrphan(COV_BIT_VEL)) return;
    aggregateEpochBoundary();
    if (!novatel::pvt::parseBESTVEL(oem4_body_.data(), oem4_body_.size(), pending_.buf)) return;
    pending_.cov_received  |= COV_BIT_VEL;
    pending_.cov_ever_seen |= COV_BIT_VEL;
    pending_.last_update = now();
    saw_binary_during_grace_ = true;
    commitSourceLockIfDue();
    if (pendingComplete()) flushPending();
  }

  // PSRDOP: cached in last_dop_; applied at flush by applyDopWithStaleness()
  // (does not gate epoch completion).
  void handlePsrDop() {
    startGraceIfNeeded();
    msg::GnssSolution scratch{};
    if (!novatel::pvt::parsePSRDOP(oem4_body_.data(), oem4_body_.size(), scratch)) return;
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
    if (!novatel::pvt::parseBESTXYZ(oem4_body_.data(), oem4_body_.size(), pending_.buf)) return;
    pending_.cov_received  |= COV_BIT_XYZ;
    pending_.cov_ever_seen |= COV_BIT_XYZ;
    pending_.last_update = now();
    if (pendingComplete()) flushPending();
  }

  void flushPending() {
    const uint8_t ever_seen = pending_.cov_ever_seen;
    if (!pending_.has_pvt) {
      pending_ = {};
      pending_.cov_ever_seen = ever_seen;
      return;
    }
    binary_solution_ = pending_.buf;
    decoder_common::finalizeBinarySolutionGeometry(binary_solution_);
    // Covariance source policy (epoch-local: based only on THIS epoch's blocks):
    //   - BESTXYZ not received this epoch: BESTPOS ENU diagonals are authoritative;
    //     rotate ENU→ECEF to fill pos_cov_ecef.
    //   - BESTXYZ received this epoch: BESTXYZ owns pos covariance; rotate ECEF→ENU.
    const bool has_xyz_this_epoch = (pending_.cov_received & COV_BIT_XYZ) != 0;
    rotatePosCovariance(binary_solution_, has_xyz_this_epoch);
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


  // RAWIMUSX body (60 B): see novatel_driver_node.cpp::handleRawImuSx for layout.
  void handleRawImuSx() {
    if (oem4_body_.size() < 60) return;

    const uint8_t imu_type = oem4_body_[2];
    uint16_t imu_week = 0;
    double seconds = 0.0;
    int32_t z_acc = 0, ny_acc = 0, x_acc = 0, z_gyr = 0, ny_gyr = 0, x_gyr = 0;
    std::memcpy(&imu_week, oem4_body_.data() +  4, 2);
    std::memcpy(&seconds,  oem4_body_.data() +  8, 8);
    std::memcpy(&z_acc,   oem4_body_.data() + 20, 4);
    std::memcpy(&ny_acc,  oem4_body_.data() + 24, 4);
    std::memcpy(&x_acc,   oem4_body_.data() + 28, 4);
    std::memcpy(&z_gyr,   oem4_body_.data() + 32, 4);
    std::memcpy(&ny_gyr,  oem4_body_.data() + 36, 4);
    std::memcpy(&x_gyr,   oem4_body_.data() + 40, 4);

    novatel::ImuScale scale = novatel::getImuScale(imu_type);
    if (imu_scale_override_accel_ != 0.0 && imu_scale_override_gyro_ != 0.0) {
      scale = {imu_scale_override_accel_, imu_scale_override_gyro_};
    }
    if (scale.accel == 0.0 || scale.gyro == 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Unknown NovAtel IMU type %u — RAWIMUSX skipped. Set imu_scale_override.{accel,gyro} "
        "in yaml or add this type to novatel_imu_scales.hpp.", imu_type);
      return;
    }

    if (!has_rawimu_prev_) {
      rawimu_prev_seconds_ = seconds;
      has_rawimu_prev_     = true;
      return;
    }
    const double dt = seconds - rawimu_prev_seconds_;
    rawimu_prev_seconds_ = seconds;
    if (dt <= 0.0 || dt > 1.0) return;

    sensor_msgs::msg::Imu imu;
    imu.header.stamp    = (use_gps_timestamp_ && imu_week > 0)
                          ? gnss_utils::gpstToUtcRosTime(gpst2time(imu_week, seconds)) : now();
    imu.header.frame_id = frame_id_;
    imu.orientation_covariance[0] = -1.0;

    imu.angular_velocity.x =  x_gyr  * scale.gyro / dt;
    imu.angular_velocity.y = -ny_gyr * scale.gyro / dt;
    imu.angular_velocity.z =  z_gyr  * scale.gyro / dt;
    imu.linear_acceleration.x =  x_acc  * scale.accel / dt;
    imu.linear_acceleration.y = -ny_acc * scale.accel / dt;
    imu.linear_acceleration.z =  z_acc  * scale.accel / dt;

    auto unk = ins::makeUnknownCovariance();
    std::copy(unk.begin(), unk.end(), imu.angular_velocity_covariance.begin());
    std::copy(unk.begin(), unk.end(), imu.linear_acceleration_covariance.begin());

    imu_raw_pub_->publish(imu);
  }

  // CORRIMUDATA body (60 B): Week(4) Seconds(8) PitchRate(8) RollRate(8) YawRate(8)
  //                          LateralAcc(8) LongitudinalAcc(8) VerticalAcc(8)
  // Rates and accelerations are SI increments accumulated over the IMU sampling
  // period; divide by dt (from successive Seconds fields) to recover rad/s and m/s².
  void handleCorrImuData() {
    if (oem4_body_.size() < 60) return;

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

    if (!has_corrimu_prev_) {
      corrimu_prev_seconds_ = seconds;
      has_corrimu_prev_     = true;
      return;
    }
    const double dt = seconds - corrimu_prev_seconds_;
    corrimu_prev_seconds_ = seconds;
    if (dt <= 0.0 || dt > 1.0) return;

    sensor_msgs::msg::Imu imu;
    imu.header.stamp    = (use_gps_timestamp_ && corrimu_week > 0)
                          ? gnss_utils::gpstToUtcRosTime(gpst2time(static_cast<int>(corrimu_week), seconds)) : now();
    imu.header.frame_id = frame_id_;

    // SPAN body frame: roll=X, pitch=Y, yaw=Z; longitudinal=X, lateral=Y, vertical=Z
    imu.angular_velocity.x    = roll_rate / dt;
    imu.angular_velocity.y    = pitch_rate / dt;
    imu.angular_velocity.z    = yaw_rate / dt;
    imu.linear_acceleration.x = lon_acc / dt;
    imu.linear_acceleration.y = lat_acc / dt;
    imu.linear_acceleration.z = vert_acc / dt;

    // orientation: not provided by CORRIMUDATA — leave identity, mark unknown
    imu.orientation_covariance[0] = -1.0;

    auto unk = ins::makeUnknownCovariance();
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
    switch (result) {
      case 1:  publishObservations(); break;
      case 2:  publishEphemerides();  break;
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
      if (sol.time_week > 0) {
        week_for_stamp = static_cast<int>(sol.time_week);
        t_gpst = gpst2time(week_for_stamp, sol.time_tow);
      } else {
        const double tow = time2gpst(raw_.time, &week_for_stamp);
        if (week_for_stamp > 0) {
          sol.time_week = static_cast<uint32_t>(week_for_stamp);
          sol.time_tow  = tow;
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

    // obs.data[0].time, not raw_.time (already advanced to the next epoch).
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

  std::string format_{"oem4"};
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

  stream_t stream_{};
  raw_t    raw_{};

  // RTCM relay (PC -> receiver): STRSVR-style TCP server -> dedicated RTCMV3
  // port (relay_out_) when rtcm_relay_output is set, else the main stream.
  decoder_common::RtcmRelayServer relay_;
  stream_t relay_out_{};
  bool     relay_out_open_{false};

  // NMEA parsing state
  gnss_utils::NmeaParser nmea_parser_;
  std::string            nmea_buffer_;
  msg::GnssSolution      nmea_solution_;
  msg::GnssSolution      binary_solution_;
  SolutionSource         source_{SolutionSource::UNDETERMINED};
  rclcpp::Time           grace_start_{0, 0, RCL_ROS_TIME};
  bool                   grace_started_{false};
  bool                   saw_binary_during_grace_{false};

  // ENU local origin
  double origin_latitude_{0.0};
  double origin_longitude_{0.0};
  double origin_altitude_{0.0};
  bool   has_local_origin_{false};
  double local_origin_ecef_[3]{0.0};
  double local_origin_pos_[3]{0.0};

  // OEM4 mini-framer state
  int                  oem4_state_{0};
  uint16_t             oem4_msg_id_{0};
  uint8_t              oem4_hdr_len_{28};
  uint16_t             oem4_msg_len_{0};
  int                  oem4_hdr_rem_{0};
  int                  oem4_hdr_pos_{0};
  uint32_t             oem4_gps_week_{0};
  uint32_t             oem4_gps_ms_{0};
  int                  oem4_body_pos_{0};
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
  rclcpp::spin(std::make_shared<gnss_ros_standardization::NovatelDecoderNode>());
  rclcpp::shutdown();
  return 0;
}