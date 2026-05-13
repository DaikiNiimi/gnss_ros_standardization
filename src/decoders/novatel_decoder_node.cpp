#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstring>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include <sensor_msgs/msg/imu.hpp>

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
constexpr size_t kNmeaMaxLineLen = 256;

// OEM4/7 binary header: SYNC(3) HDRLEN(1) MSGID(2) MSGTYPE(1) PORT(1) MSGLEN(2) ...
// Standard header length = 28 bytes; MSGLEN field is at bytes 8-9 of the header
constexpr int OEM4_MSGID_OFFSET  = 4;   // bytes 4-5 from SYNC1
constexpr int OEM4_MSGLEN_OFFSET = 8;   // bytes 8-9 from SYNC1

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
    startPolling();

    RCLCPP_INFO(get_logger(), "NovAtel Decoder initialized (format: %s)", format_.c_str());
  }

  ~NovatelDecoderNode() override {
    try {
      strclose(&stream_);
    } catch (...) {}
    free_raw(&raw_);
  }

 private:
  // ============================================================================
  // Initialization
  // ============================================================================

  void initializeParameters() {
    declare_parameter<std::string>("stream_path", "serial:///dev/ttyUSB0:115200");
    declare_parameter<std::string>("format", "oem4");
    declare_parameter<std::string>("frame_id", "gnss_link");
    declare_parameter<std::string>("observation_topic", "/gnss/observation");
    declare_parameter<std::string>("ephemeris_topic", "/gnss/ephemeris");
    declare_parameter<std::string>("solution_topic", "/gnss/nmea_solution");
    declare_parameter<std::string>("imu_topic",      "/gnss/imu/data");
    declare_parameter<std::string>("imu_raw_topic",  "/gnss/imu/data_raw");
    declare_parameter<double>("imu_scale_override.accel", 0.0);
    declare_parameter<double>("imu_scale_override.gyro",  0.0);
    declare_parameter<double>("ephemeris.snapshot_period_s", 30.0);
    declare_parameter<double>("ephemeris.max_age_s", 7200.0);
    declare_parameter<bool>("use_gps_timestamp", false);
    declare_parameter<std::vector<double>>("origin", {0.0, 0.0, 0.0});
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

  // ============================================================================
  // Stream Management
  // ============================================================================

  void openStream() {
    const std::string stream_path = get_parameter("stream_path").as_string();
    std::string path = stream_path;

    int stream_type = 0;
    bool matched = false;
    for (const auto& def : novatel::kStreamTypes) {
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

    if (!stropen(&stream_, stream_type, STR_MODE_R, path.c_str())) {
      RCLCPP_ERROR(get_logger(), "Failed to open stream: %s", stream_path.c_str());
      throw std::runtime_error("stropen failed");
    }

    RCLCPP_INFO(get_logger(), "Stream opened: %s", stream_path.c_str());
  }

  // ============================================================================
  // Polling
  // ============================================================================

  void pollStream() {
    uint8_t buffer[novatel::READ_BUFFER_SIZE];
    const int bytes_read = strread(&stream_, buffer, sizeof(buffer));

    for (int i = 0; i < bytes_read; ++i) {
      const uint8_t byte = buffer[i];

      // Feed NovAtel binary decoder
      int result = input_oem4(&raw_, byte);
      handleDecodeResult(result);

      // Parallel OEM4 mini-framer for CORRIMUDATA
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
        } else if (nmea_buffer_.size() > kNmeaMaxLineLen) {
          nmea_buffer_.clear();
        }
      }
    }

    maybePublishHeartbeat();
    commitSourceLockIfDue();  // ensure grace finalizes even on idle stream
  }

  // ============================================================================
  // OEM4 Mini-Framer (parallel to RTKLIB, for CORRIMUDATA)
  //
  // OEM4 binary frame: SYNC(0xAA,0x44,0x12) HDRLEN(1) MSGID(2,LE) MSGTYPE(1)
  //                    PORT(1) MSGLEN(2,LE) ... rest of header ... BODY CRC32
  // ============================================================================

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
        oem4_body_.clear();
        oem4_body_pos_ = 0;
        oem4_state_    = (oem4_hdr_rem_ > 0) ? 10 : 11;
        break;
      case 10:  // skip remaining header bytes
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

  // Solution source policy (grace-period detection):
  //   Start in UNDETERMINED. Wait kGracePeriodSec to detect whether the stream
  //   carries BESTPOS. If yes → BINARY (PVT wins when both are present). If no
  //   PVT seen by grace expiry → NMEA. Deterministic regardless of stream
  //   ordering between NMEA and binary frames.
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

  void handleBestPos() {
    startGraceIfNeeded();
    if (!novatel::pvt::parseBESTPOS(oem4_body_.data(), oem4_body_.size(), binary_solution_)) return;
    finalizeBinarySolutionGeometry(binary_solution_);
    saw_binary_during_grace_ = true;
    commitSourceLockIfDue();
    if (source_ == SolutionSource::BINARY) publishSolution(binary_solution_);
  }

  void handleBestVel() {
    if (!novatel::pvt::parseBESTVEL(oem4_body_.data(), oem4_body_.size(), binary_solution_)) return;
    double llh[3] = {binary_solution_.latitude * D2R,
                     binary_solution_.longitude * D2R,
                     binary_solution_.altitude};
    double vel_e[3] = {binary_solution_.vel_enu.x, binary_solution_.vel_enu.y, binary_solution_.vel_enu.z};
    double vel_ec[3] = {0};
    enu2ecef(llh, vel_e, vel_ec);
    binary_solution_.vel_ecef.x = vel_ec[0];
    binary_solution_.vel_ecef.y = vel_ec[1];
    binary_solution_.vel_ecef.z = vel_ec[2];
  }

  void handlePsrDop() {
    novatel::pvt::parsePSRDOP(oem4_body_.data(), oem4_body_.size(), binary_solution_);
  }

  void handleBestXyz() {
    novatel::pvt::parseBESTXYZ(oem4_body_.data(), oem4_body_.size(), binary_solution_);
  }

  static void finalizeBinarySolutionGeometry(msg::GnssSolution& s) {
    double llh[3] = {s.latitude * D2R, s.longitude * D2R, s.altitude};
    double ecef[3] = {0};
    pos2ecef(llh, ecef);
    s.pos_ecef.x = ecef[0]; s.pos_ecef.y = ecef[1]; s.pos_ecef.z = ecef[2];
    double vel_e[3] = {s.vel_enu.x, s.vel_enu.y, s.vel_enu.z};
    double vel_ec[3] = {0};
    enu2ecef(llh, vel_e, vel_ec);
    s.vel_ecef.x = vel_ec[0]; s.vel_ecef.y = vel_ec[1]; s.vel_ecef.z = vel_ec[2];
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

  // ============================================================================
  // Message Handling
  // ============================================================================

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

  // ============================================================================
  // Solution Publishing
  // ============================================================================

  void publishSolution(msg::GnssSolution& sol) {
    int week = 0;
    const double tow = time2gpst(raw_.time, &week);
    if (week > 0) {
      sol.time_week = static_cast<uint16_t>(week);
      sol.time_tow  = tow;
    }

    sol.header.stamp    = (use_gps_timestamp_ && week > 0)
                          ? gnss_utils::gpstToUtcRosTime(raw_.time) : now();
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

      sol.org_ecef.x = local_origin_ecef_[0];
      sol.org_ecef.y = local_origin_ecef_[1];
      sol.org_ecef.z = local_origin_ecef_[2];

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

      double vel_ecef[3] = {sol.vel_ecef.x, sol.vel_ecef.y, sol.vel_ecef.z};
      double vel_enu[3] = {0};
      ecef2enu(local_origin_pos_, vel_ecef, vel_enu);
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

  // ============================================================================
  // Observation Publishing
  // ============================================================================

  void publishObservations() {
    if (raw_.obs.n <= 0) return;

    int week = 0;
    const double tow = time2gpst(raw_.time, &week);

    msg::GnssObservations msg;
    msg.header.stamp    = (use_gps_timestamp_ && week > 0)
                          ? gnss_utils::gpstToUtcRosTime(raw_.time) : now();
    msg.header.frame_id = frame_id_;
    msg.week = static_cast<uint16_t>(week);
    msg.tow  = tow;

    SatelliteCount sat_count{};
    for (int i = 0; i < raw_.obs.n; ++i) {
      const obsd_t& obs = raw_.obs.data[i];
      countSatellite(obs.sat, sat_count);
      appendObservations(obs, msg.observations);
    }

    obs_pub_->publish(msg);

    RCLCPP_INFO(get_logger(),
      "Published observations: week=%d tow=%.3f n=%zu sats(G/R/E/J/C/I/S/U)=(%d/%d/%d/%d/%d/%d/%d/%d)",
      week, tow, msg.observations.size(),
      sat_count.gps, sat_count.glo, sat_count.gal, sat_count.qzs,
      sat_count.bds, sat_count.irn, sat_count.sbs, sat_count.unknown);
  }

  void appendObservations(const obsd_t& obs, std::vector<msg::GnssObservation>& observations) {
    for (int freq = 0; freq < NFREQ + NEXOBS; ++freq) {
      if (obs.P[freq] == 0.0 && obs.L[freq] == 0.0 && obs.D[freq] == 0.0 && obs.SNR[freq] == 0) continue;
      observations.push_back(gnss_utils::obsToMsg(obs, freq));
    }
  }

  // ============================================================================
  // Ephemeris Publishing
  // ============================================================================

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

  // ============================================================================
  // Helper Types
  // ============================================================================

  struct SatelliteCount { int gps=0, glo=0, gal=0, qzs=0, bds=0, irn=0, sbs=0, unknown=0; };

  static void countSatellite(int sat, SatelliteCount& c) {
    int prn = 0;
    switch (satsys(sat, &prn)) {
      case SYS_GPS: ++c.gps; break; case SYS_GLO: ++c.glo; break;
      case SYS_GAL: ++c.gal; break; case SYS_QZS: ++c.qzs; break;
      case SYS_CMP: ++c.bds; break; case SYS_IRN: ++c.irn; break;
      case SYS_SBS: ++c.sbs; break; default: ++c.unknown; break;
    }
  }

  // ============================================================================
  // Member Variables
  // ============================================================================

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
  int                  oem4_body_pos_{0};
  std::vector<uint8_t> oem4_body_;

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
