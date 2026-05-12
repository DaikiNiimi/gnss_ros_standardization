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
#include "gnss_ros_standardization/sbf_nav_decoder.hpp"
#include "gnss_ros_standardization/sbf_protocol.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"

using namespace std::chrono_literals;
namespace ins = gnss_ros_standardization::ins_utils;

namespace gnss_ros_standardization {

namespace {

constexpr auto kTimerInterval  = 10ms;
constexpr size_t kNmeaMaxLineLen = 256;

// SBF ExtSensorMeas (ID 4050) body offsets
// Body layout: TOW(4) WNc(2) N(1) SBLength(1) [N × ExtSensorMeasSub of size SBLength]
//
// ExtSensorMeasSub (SBLength = 28 bytes):
//   Source(u1) SensorModel(u1) Type(u1) ObsInfo(u1) X(f8) Y(f8) Z(f8)
//
// IMPORTANT: Each Type is a 3-axis VECTOR (Type 0 = entire accel 3-vector,
// Type 1 = entire gyro 3-vector). NOT one axis per Type as in some misreadings.
constexpr int ESM_OFFSET_N          = 6;
constexpr int ESM_OFFSET_SB_LENGTH  = 7;
constexpr int ESM_OFFSET_SUBBLOCKS  = 8;
constexpr int ESM_SB_OFFSET_TYPE    = 2;   // Type field
constexpr int ESM_SB_OFFSET_X       = 4;   // float64 X-axis
constexpr int ESM_SB_OFFSET_Y       = 12;  // float64 Y-axis
constexpr int ESM_SB_OFFSET_Z       = 20;  // float64 Z-axis
constexpr int ESM_SB_MIN_LEN        = 28;
constexpr int ESM_MIN_BODY_LEN      = 8;

// ExtSensorMeas Type values (each Type carries a full 3-vector)
constexpr uint8_t ESM_TYPE_ACCEL    = 0;   // Accelerations [m/s²]
constexpr uint8_t ESM_TYPE_GYRO     = 1;   // Angular rates [rad/s]

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
    startPolling();

    RCLCPP_INFO(get_logger(), "SBF Decoder initialized");
  }

  ~SbfDecoderNode() override {
    try {
      strclose(&stream_);
    } catch (...) {}
    free_raw(&raw_);
    free_rtcm(&rtcm_);
  }

 private:
  // ============================================================================
  // Initialization
  // ============================================================================

  void initializeParameters() {
    declare_parameter<std::string>("stream_path", "serial:///dev/ttyUSB0:115200");
    declare_parameter<std::string>("frame_id", "gnss_link");
    declare_parameter<std::string>("observation_topic", "/gnss/observation");
    declare_parameter<std::string>("ephemeris_topic", "/gnss/ephemeris");
    declare_parameter<std::string>("solution_topic",     "/gnss/nmea_solution");
    declare_parameter<std::string>("imu_raw_topic",      "/gnss/imu/data_raw");
    declare_parameter<double>("ephemeris.snapshot_period_s", 30.0);
    declare_parameter<double>("ephemeris.max_age_s", 7200.0);

    declare_parameter<bool>("use_gps_timestamp", false);

    frame_id_ = get_parameter("frame_id").as_string();
    eph_store_.setSnapshotPeriod(get_parameter("ephemeris.snapshot_period_s").as_double());
    eph_store_.setMaxAge(get_parameter("ephemeris.max_age_s").as_double());
    use_gps_timestamp_ = get_parameter("use_gps_timestamp").as_bool();
  }

  void initializePublishers() {
    obs_pub_          = create_publisher<msg::GnssObservations>(get_parameter("observation_topic").as_string(), 10);
    eph_pub_          = create_publisher<msg::GnssEphemerides>(get_parameter("ephemeris_topic").as_string(), rclcpp::QoS(1).transient_local());
    sol_pub_          = create_publisher<msg::GnssSolution>(get_parameter("solution_topic").as_string(), 10);
    imu_raw_pub_      = create_publisher<sensor_msgs::msg::Imu>(get_parameter("imu_raw_topic").as_string(), 10);
  }

  void initializeDecoder() {
    if (init_raw(&raw_, STRFMT_SEPT) != 1) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize raw decoder");
      throw std::runtime_error("init_raw failed");
    }
    if (init_rtcm(&rtcm_) != 1) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize RTCM decoder");
      throw std::runtime_error("init_rtcm failed");
    }
  }

  void startPolling() {
    timer_ = create_wall_timer(kTimerInterval, std::bind(&SbfDecoderNode::pollStream, this));
  }

  // ============================================================================
  // Stream Management
  // ============================================================================

  void openStream() {
    const std::string stream_path = get_parameter("stream_path").as_string();
    std::string path = stream_path;

    int stream_type = 0;
    bool matched = false;
    for (const auto& def : sbf::kStreamTypes) {
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
    uint8_t buffer[sbf::READ_BUFFER_SIZE];
    const int bytes_read = strread(&stream_, buffer, sizeof(buffer));

    for (int i = 0; i < bytes_read; ++i) {
      const uint8_t byte = buffer[i];

      // Feed SBF binary decoder (obs/eph)
      const int result = input_sbf(&raw_, byte);
      handleDecodeResult(result);

      // Parallel SBF mini-framer for AttEuler
      parseSbfByte(byte);

      // Feed NMEA text parser (SBF and NMEA are interleaved on the same stream)
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
    maybeWatchdogFlushPendingPvt();
    commitSourceLockIfDue();  // ensure grace finalizes even on idle stream
  }

  // ============================================================================
  // SBF Mini-Framer (parallel to RTKLIB, for AttEuler)
  //
  // SBF block layout: SYNC1('$') SYNC2('@') CRC(2) ID(2,LE) Length(2,LE) Body(Length-8)
  // ID bits 0-12 are the block number; bits 13-15 are the revision.
  // ============================================================================

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
      case sbf::ID_GPSNAV:           handleGpsNav();           break;
      case sbf::ID_GLONAV:           handleGloNav();           break;
      case sbf::ID_GALNAV:           handleGalNav();           break;
      case sbf::ID_BDSNAV:           handleBdsNav();           break;
      case sbf::ID_QZSNAV:           handleQzsNav();           break;
      case sbf::ID_NAVICNAV:         handleNavicNav();         break;
      case sbf::ID_PVTGEODETIC:      handlePvtGeodetic();      break;
      case sbf::ID_POSCOVGEODETIC:   handlePosCovGeodetic();   break;
      case sbf::ID_PVTCARTESIAN:     handlePvtCartesian();     break;
      case sbf::ID_POSCOVCARTESIAN:  handlePosCovCartesian();  break;
      default: break;
    }
  }

  // ============================================================================
  // Binary PVT handlers (per-system TOW aggregation, first-msg source lock)
  // ============================================================================
  struct PendingPvt {
    uint32_t tow_ms{UINT32_MAX};
    bool has_pvt{false};
    bool has_cov{false};
    msg::GnssSolution buf{};
    rclcpp::Time last_update{0, 0, RCL_ROS_TIME};
  };

  // Solution source policy (grace-period detection):
  //   Start in UNDETERMINED. Wait kGracePeriodSec to detect whether the stream
  //   carries PVT blocks. If yes → BINARY (PVT wins when both are present).
  //   If grace expires with no PVT → NMEA. Deterministic regardless of stream
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

  // Mark "binary seen during grace" — called from every PVT block arrival.
  void markBinarySeen() {
    saw_binary_during_grace_ = true;
    commitSourceLockIfDue();
  }

  void handlePvtGeodetic() {
    startGraceIfNeeded();
    const uint8_t* p = sbf_body_.data();
    const size_t len = sbf_body_.size();
    const uint32_t tow = sbf::pvt::getTowMs(p, len);
    if (tow != pending_geo_.tow_ms && (pending_geo_.has_pvt || pending_geo_.has_cov)) {
      flushPendingGeo();
    }
    pending_geo_.tow_ms = tow;
    if (!sbf::pvt::parsePVTGeodetic(p, len, pending_geo_.buf)) return;
    pending_geo_.has_pvt = true;
    pending_geo_.last_update = now();
    markBinarySeen();
    if (pending_geo_.has_cov) flushPendingGeo();
  }

  void handlePosCovGeodetic() {
    startGraceIfNeeded();
    const uint8_t* p = sbf_body_.data();
    const size_t len = sbf_body_.size();
    const uint32_t tow = sbf::pvt::getTowMs(p, len);
    if (tow != pending_geo_.tow_ms && (pending_geo_.has_pvt || pending_geo_.has_cov)) {
      flushPendingGeo();
    }
    pending_geo_.tow_ms = tow;
    if (!sbf::pvt::parsePosCovGeodetic(p, len, pending_geo_.buf)) return;
    pending_geo_.has_cov = true;
    pending_geo_.last_update = now();
    markBinarySeen();
    if (pending_geo_.has_pvt) flushPendingGeo();
  }

  void handlePvtCartesian() {
    startGraceIfNeeded();
    const uint8_t* p = sbf_body_.data();
    const size_t len = sbf_body_.size();
    const uint32_t tow = sbf::pvt::getTowMs(p, len);
    if (tow != pending_xyz_.tow_ms && (pending_xyz_.has_pvt || pending_xyz_.has_cov)) {
      flushPendingXyz();
    }
    pending_xyz_.tow_ms = tow;
    if (!sbf::pvt::parsePVTCartesian(p, len, pending_xyz_.buf)) return;
    pending_xyz_.has_pvt = true;
    pending_xyz_.last_update = now();
    markBinarySeen();
    if (pending_xyz_.has_cov) flushPendingXyz();
  }

  void handlePosCovCartesian() {
    startGraceIfNeeded();
    const uint8_t* p = sbf_body_.data();
    const size_t len = sbf_body_.size();
    const uint32_t tow = sbf::pvt::getTowMs(p, len);
    if (tow != pending_xyz_.tow_ms && (pending_xyz_.has_pvt || pending_xyz_.has_cov)) {
      flushPendingXyz();
    }
    pending_xyz_.tow_ms = tow;
    if (!sbf::pvt::parsePosCovCartesian(p, len, pending_xyz_.buf)) return;
    pending_xyz_.has_cov = true;
    pending_xyz_.last_update = now();
    markBinarySeen();
    if (pending_xyz_.has_pvt) flushPendingXyz();
  }

  void flushPendingGeo() {
    if (!pending_geo_.has_pvt) { pending_geo_ = {}; return; }
    binary_solution_.status      = pending_geo_.buf.status;
    binary_solution_.num_sats    = pending_geo_.buf.num_sats;
    binary_solution_.time_tow    = pending_geo_.buf.time_tow;
    binary_solution_.time_week   = pending_geo_.buf.time_week;
    binary_solution_.latitude    = pending_geo_.buf.latitude;
    binary_solution_.longitude   = pending_geo_.buf.longitude;
    binary_solution_.altitude    = pending_geo_.buf.altitude;
    binary_solution_.vel_enu     = pending_geo_.buf.vel_enu;
    binary_solution_.pos_enu_cov = pending_geo_.buf.pos_enu_cov;
    finalizeBinarySolutionGeometry(binary_solution_);
    if (source_ == SolutionSource::BINARY) publishSolution(binary_solution_);
    pending_geo_ = {};
  }

  void flushPendingXyz() {
    if (!pending_xyz_.has_pvt) { pending_xyz_ = {}; return; }
    binary_solution_.status        = pending_xyz_.buf.status;
    binary_solution_.num_sats      = pending_xyz_.buf.num_sats;
    binary_solution_.time_tow      = pending_xyz_.buf.time_tow;
    binary_solution_.time_week     = pending_xyz_.buf.time_week;
    binary_solution_.pos_ecef      = pending_xyz_.buf.pos_ecef;
    binary_solution_.vel_ecef      = pending_xyz_.buf.vel_ecef;
    binary_solution_.pos_cov_ecef  = pending_xyz_.buf.pos_cov_ecef;
    double pos[3] = {0};
    double r[3] = {binary_solution_.pos_ecef.x, binary_solution_.pos_ecef.y, binary_solution_.pos_ecef.z};
    ecef2pos(r, pos);
    constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;
    binary_solution_.latitude  = pos[0] * kRad2Deg;
    binary_solution_.longitude = pos[1] * kRad2Deg;
    binary_solution_.altitude  = pos[2];
    if (source_ == SolutionSource::BINARY) publishSolution(binary_solution_);
    pending_xyz_ = {};
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

  void maybeWatchdogFlushPendingPvt() {
    const auto now_t = now();
    if (pending_geo_.has_pvt && (now_t - pending_geo_.last_update).seconds() > 1.5) {
      flushPendingGeo();
    }
    if (pending_xyz_.has_pvt && (now_t - pending_xyz_.last_update).seconds() > 1.5) {
      flushPendingXyz();
    }
  }

  void handleExtSensorMeas() {
    if (static_cast<int>(sbf_body_.size()) < ESM_MIN_BODY_LEN) return;

    const uint8_t n         = sbf_body_[ESM_OFFSET_N];
    const uint8_t sb_length = sbf_body_[ESM_OFFSET_SB_LENGTH];
    if (sb_length < ESM_SB_MIN_LEN) return;
    if (static_cast<int>(sbf_body_.size()) < ESM_MIN_BODY_LEN + n * sb_length) return;

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
      const int base = ESM_OFFSET_SUBBLOCKS + i * sb_length;
      const uint8_t type = sbf_body_[base + ESM_SB_OFFSET_TYPE];

      if (type == ESM_TYPE_ACCEL) {
        std::memcpy(&esm_accel_[0], sbf_body_.data() + base + ESM_SB_OFFSET_X, 8);
        std::memcpy(&esm_accel_[1], sbf_body_.data() + base + ESM_SB_OFFSET_Y, 8);
        std::memcpy(&esm_accel_[2], sbf_body_.data() + base + ESM_SB_OFFSET_Z, 8);
        esm_has_accel_ = true;
      } else if (type == ESM_TYPE_GYRO) {
        std::memcpy(&esm_gyro_[0], sbf_body_.data() + base + ESM_SB_OFFSET_X, 8);
        std::memcpy(&esm_gyro_[1], sbf_body_.data() + base + ESM_SB_OFFSET_Y, 8);
        std::memcpy(&esm_gyro_[2], sbf_body_.data() + base + ESM_SB_OFFSET_Z, 8);
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

    auto unk = ins::makeUnknownCovariance();
    std::copy(unk.begin(), unk.end(), imu.orientation_covariance.begin());

    imu.linear_acceleration.x = esm_accel_[0];
    imu.linear_acceleration.y = esm_accel_[1];
    imu.linear_acceleration.z = esm_accel_[2];
    std::copy(unk.begin(), unk.end(), imu.linear_acceleration_covariance.begin());

    imu.angular_velocity.x = esm_gyro_[0];
    imu.angular_velocity.y = esm_gyro_[1];
    imu.angular_velocity.z = esm_gyro_[2];
    std::copy(unk.begin(), unk.end(), imu.angular_velocity_covariance.begin());

    imu_raw_pub_->publish(imu);

    esm_has_accel_ = false;
    esm_has_gyro_  = false;
    esm_accel_[0] = esm_accel_[1] = esm_accel_[2] = 0.0;
    esm_gyro_[0]  = esm_gyro_[1]  = esm_gyro_[2]  = 0.0;
  }

  // ============================================================================
  // Decoded *Nav block handlers
  // ============================================================================

  void handleGpsNav() {
    eph_t eph{};
    if (!sbf::nav::parseGPSNav(sbf_body_, eph)) return;
    ingestAndMaybePublish(eph);
  }

  void handleGloNav() {
    geph_t geph{};
    if (!sbf::nav::parseGLONav(sbf_body_, geph)) return;
    ingestAndMaybePublish(geph);
  }

  void handleGalNav() {
    eph_t eph{};
    if (!sbf::nav::parseGALNav(sbf_body_, eph)) return;
    ingestAndMaybePublish(eph);
  }

  void handleBdsNav() {
    eph_t eph{};
    if (!sbf::nav::parseBDSNav(sbf_body_, eph)) return;
    ingestAndMaybePublish(eph);
  }

  void handleQzsNav() {
    eph_t eph{};
    if (!sbf::nav::parseQZSNav(sbf_body_, eph)) return;
    ingestAndMaybePublish(eph);
  }

  void handleNavicNav() {
    eph_t eph{};
    if (!sbf::nav::parseNavICNav(sbf_body_, eph)) return;
    ingestAndMaybePublish(eph);
  }

  void ingestAndMaybePublish(const eph_t& e) {
    if (eph_store_.ingestEph(e)) publishSnapshot();
  }
  void ingestAndMaybePublish(const geph_t& g) {
    if (eph_store_.ingestGeph(g)) publishSnapshot();
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
    if (week != 0) {
      sol.time_week = static_cast<uint16_t>(week);
      sol.time_tow  = tow;
    }

    sol.header.stamp    = (use_gps_timestamp_ && week != 0)
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
    msg.header.stamp    = (use_gps_timestamp_ && week != 0)
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
  // Ephemeris Publishing (snapshot-on-change + heartbeat via EphemerisStore)
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

  std::string frame_id_;
  bool        use_gps_timestamp_{false};

  rclcpp::Publisher<msg::GnssObservations>::SharedPtr  obs_pub_;
  rclcpp::Publisher<msg::GnssEphemerides>::SharedPtr   eph_pub_;
  rclcpp::Publisher<msg::GnssSolution>::SharedPtr      sol_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr  imu_raw_pub_;       // ExtSensorMeas accel/gyro (uncalibrated)
  rclcpp::TimerBase::SharedPtr timer_;

  stream_t stream_{};
  raw_t    raw_{};
  rtcm_t   rtcm_{};

  // NMEA parsing state
  gnss_utils::NmeaParser nmea_parser_;
  std::string            nmea_buffer_;
  msg::GnssSolution      nmea_solution_;
  msg::GnssSolution      binary_solution_;
  SolutionSource         source_{SolutionSource::UNDETERMINED};
  rclcpp::Time           grace_start_{0, 0, RCL_ROS_TIME};
  bool                   grace_started_{false};
  bool                   saw_binary_during_grace_{false};
  PendingPvt             pending_geo_;
  PendingPvt             pending_xyz_;

  // ENU local origin
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
