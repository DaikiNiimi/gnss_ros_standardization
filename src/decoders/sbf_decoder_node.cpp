#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstring>
#include <limits>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <sensor_msgs/msg/imu.hpp>

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

// SBF AttEuler (ID 5938) body offsets (after 8-byte SBF header consumed by mini-framer)
constexpr int ATT_EULER_OFFSET_ERROR   = 7;
constexpr int ATT_EULER_OFFSET_HEADING = 10;
constexpr int ATT_EULER_OFFSET_PITCH   = 14;
constexpr int ATT_EULER_OFFSET_ROLL    = 18;
constexpr int ATT_EULER_MIN_LEN        = 22;

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
/// Also parses NMEA sentences for GnssSolution and SBF AttEuler blocks for IMU data.
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
    declare_parameter<std::string>("imu_attitude_topic", "/gnss/imu/attitude");
    declare_parameter<std::string>("imu_raw_topic",      "/gnss/imu/data_raw");

    frame_id_ = get_parameter("frame_id").as_string();
  }

  void initializePublishers() {
    obs_pub_          = create_publisher<msg::GnssObservations>(get_parameter("observation_topic").as_string(), 10);
    eph_pub_          = create_publisher<msg::GnssEphemerides>(get_parameter("ephemeris_topic").as_string(), rclcpp::QoS(100).transient_local());
    sol_pub_          = create_publisher<msg::GnssSolution>(get_parameter("solution_topic").as_string(), 10);
    imu_attitude_pub_ = create_publisher<sensor_msgs::msg::Imu>(get_parameter("imu_attitude_topic").as_string(), 10);
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
      const int result = input_sbf(&raw_, &rtcm_, byte);
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
      case sbf::ID_ATTEULER:      handleAttEuler();      break;
      case sbf::ID_EXTSENSORMEAS: handleExtSensorMeas(); break;
      case sbf::ID_GPSNAV:        handleGpsNav();        break;
      case sbf::ID_GLONAV:        handleGloNav();        break;
      case sbf::ID_GALNAV:        handleGalNav();        break;
      case sbf::ID_BDSNAV:        handleBdsNav();        break;
      case sbf::ID_QZSNAV:        handleQzsNav();        break;
      case sbf::ID_NAVICNAV:      handleNavicNav();      break;
      default: break;
    }
  }

  void handleAttEuler() {
    if (static_cast<int>(sbf_body_.size()) < ATT_EULER_MIN_LEN) return;

    // Byte 7 (Error): non-zero means attitude not valid
    if (sbf_body_[ATT_EULER_OFFSET_ERROR] != 0) return;

    // Heading, Pitch, Roll: float32 (4 bytes each), in degrees
    float heading_deg = 0.0f, pitch_deg = 0.0f, roll_deg = 0.0f;
    std::memcpy(&heading_deg, sbf_body_.data() + ATT_EULER_OFFSET_HEADING, 4);
    std::memcpy(&pitch_deg,   sbf_body_.data() + ATT_EULER_OFFSET_PITCH,   4);
    std::memcpy(&roll_deg,    sbf_body_.data() + ATT_EULER_OFFSET_ROLL,    4);

    const double deg2rad = M_PI / 180.0;

    sensor_msgs::msg::Imu imu;
    imu.header.stamp    = now();
    imu.header.frame_id = frame_id_;

    imu.orientation = ins::eulerToQuaternion(
        roll_deg    * deg2rad,
        pitch_deg   * deg2rad,
        heading_deg * deg2rad);

    // Septentrio AttEuler does not provide per-axis accuracy in the basic block
    auto unk = ins::makeUnknownCovariance();
    std::copy(unk.begin(), unk.end(), imu.orientation_covariance.begin());
    std::copy(unk.begin(), unk.end(), imu.angular_velocity_covariance.begin());
    std::copy(unk.begin(), unk.end(), imu.linear_acceleration_covariance.begin());

    imu_attitude_pub_->publish(imu);
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
    imu.header.stamp    = now();
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
    EphemerisKey key{eph.sat, eph.iode, eph.iodc, eph.code};
    if (!seen_ephemeris_.insert(key).second) return;
    pending_gnss_eph_.push_back(gnss_utils::ephToMsg(eph));
    flushPendingEphemerides();
  }

  void handleGloNav() {
    geph_t geph{};
    if (!sbf::nav::parseGLONav(sbf_body_, geph)) return;
    auto it = last_glo_iode_.find(geph.sat);
    if (it != last_glo_iode_.end() && it->second == geph.iode) return;
    last_glo_iode_[geph.sat] = geph.iode;
    pending_glonass_eph_.push_back(gnss_utils::gephToMsg(geph));
    flushPendingEphemerides();
  }

  void handleGalNav() {
    eph_t eph{};
    if (!sbf::nav::parseGALNav(sbf_body_, eph)) return;
    EphemerisKey key{eph.sat, eph.iode, eph.iodc, eph.code};
    if (!seen_ephemeris_.insert(key).second) return;
    pending_gnss_eph_.push_back(gnss_utils::ephToMsg(eph));
    flushPendingEphemerides();
  }

  void handleBdsNav() {
    eph_t eph{};
    if (!sbf::nav::parseBDSNav(sbf_body_, eph)) return;
    EphemerisKey key{eph.sat, eph.iode, eph.iodc, eph.code};
    if (!seen_ephemeris_.insert(key).second) return;
    pending_gnss_eph_.push_back(gnss_utils::ephToMsg(eph));
    flushPendingEphemerides();
  }

  void handleQzsNav() {
    eph_t eph{};
    if (!sbf::nav::parseQZSNav(sbf_body_, eph)) return;
    EphemerisKey key{eph.sat, eph.iode, eph.iodc, eph.code};
    if (!seen_ephemeris_.insert(key).second) return;
    pending_gnss_eph_.push_back(gnss_utils::ephToMsg(eph));
    flushPendingEphemerides();
  }

  void handleNavicNav() {
    eph_t eph{};
    if (!sbf::nav::parseNavICNav(sbf_body_, eph)) return;
    EphemerisKey key{eph.sat, eph.iode, eph.iodc, eph.code};
    if (!seen_ephemeris_.insert(key).second) return;
    pending_gnss_eph_.push_back(gnss_utils::ephToMsg(eph));
    flushPendingEphemerides();
  }

  // ============================================================================
  // Message Handling
  // ============================================================================

  void handleNmeaSentence(const std::string& sentence) {
    if (nmea_parser_.parseSentence(sentence, current_solution_)) {
      publishSolution();
    }
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

  void publishSolution() {
    int week = 0;
    const double tow = time2gpst(raw_.time, &week);
    if (week != 0) {
      current_solution_.time_week = static_cast<uint16_t>(week);
      current_solution_.time_tow  = tow;
    }

    current_solution_.header.stamp    = now();
    current_solution_.header.frame_id = frame_id_;

    const bool has_fix =
      current_solution_.status == msg::GnssSolution::STATUS_FIX    ||
      current_solution_.status == msg::GnssSolution::STATUS_FLOAT  ||
      current_solution_.status == msg::GnssSolution::STATUS_SINGLE ||
      current_solution_.status == msg::GnssSolution::STATUS_DGPS;

    if (has_fix) {
      if (!has_local_origin_) {
        local_origin_ecef_[0] = current_solution_.pos_ecef.x;
        local_origin_ecef_[1] = current_solution_.pos_ecef.y;
        local_origin_ecef_[2] = current_solution_.pos_ecef.z;
        ecef2pos(local_origin_ecef_, local_origin_pos_);
        has_local_origin_ = true;
        RCLCPP_INFO(get_logger(), "Set local ENU origin: lat=%.6f lon=%.6f alt=%.2f",
          local_origin_pos_[0] * (180.0 / M_PI),
          local_origin_pos_[1] * (180.0 / M_PI),
          local_origin_pos_[2]);
      }

      current_solution_.org_ecef.x = local_origin_ecef_[0];
      current_solution_.org_ecef.y = local_origin_ecef_[1];
      current_solution_.org_ecef.z = local_origin_ecef_[2];

      double d_ecef[3] = {
        current_solution_.pos_ecef.x - local_origin_ecef_[0],
        current_solution_.pos_ecef.y - local_origin_ecef_[1],
        current_solution_.pos_ecef.z - local_origin_ecef_[2]
      };
      double enu[3] = {0};
      ecef2enu(local_origin_pos_, d_ecef, enu);
      current_solution_.pos_enu.x = enu[0];
      current_solution_.pos_enu.y = enu[1];
      current_solution_.pos_enu.z = enu[2];

      double vel_ecef[3] = {
        current_solution_.vel_ecef.x,
        current_solution_.vel_ecef.y,
        current_solution_.vel_ecef.z
      };
      double vel_enu[3] = {0};
      ecef2enu(local_origin_pos_, vel_ecef, vel_enu);
      current_solution_.vel_enu.x = vel_enu[0];
      current_solution_.vel_enu.y = vel_enu[1];
      current_solution_.vel_enu.z = vel_enu[2];
    } else {
      current_solution_.pos_enu.x = std::numeric_limits<double>::quiet_NaN();
      current_solution_.pos_enu.y = std::numeric_limits<double>::quiet_NaN();
      current_solution_.pos_enu.z = std::numeric_limits<double>::quiet_NaN();
      current_solution_.vel_enu.x = std::numeric_limits<double>::quiet_NaN();
      current_solution_.vel_enu.y = std::numeric_limits<double>::quiet_NaN();
      current_solution_.vel_enu.z = std::numeric_limits<double>::quiet_NaN();
    }

    sol_pub_->publish(current_solution_);
  }

  // ============================================================================
  // Observation Publishing
  // ============================================================================

  void publishObservations() {
    if (raw_.obs.n <= 0) return;

    int week = 0;
    const double tow = time2gpst(raw_.time, &week);

    msg::GnssObservations msg;
    msg.header.stamp    = now();
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
  // Ephemeris Publishing (pending queue style — shared by raw and decoded paths)
  // ============================================================================

  void accumulateRtklibEphemerides() {
    for (int i = 0; i < raw_.nav.n; ++i) {
      const eph_t& eph = raw_.nav.eph[i];
      if (eph.sat == 0) continue;
      int prn = 0;
      if (satsys(eph.sat, &prn) == SYS_GLO) continue;
      EphemerisKey key{eph.sat, eph.iode, eph.iodc, eph.code};
      if (seen_ephemeris_.insert(key).second) {
        pending_gnss_eph_.push_back(gnss_utils::ephToMsg(eph));
      }
    }
    for (int i = 0; i < raw_.nav.ng; ++i) {
      const geph_t& geph = raw_.nav.geph[i];
      if (geph.sat == 0) continue;
      auto it = last_glo_iode_.find(geph.sat);
      if (it == last_glo_iode_.end() || it->second != geph.iode) {
        last_glo_iode_[geph.sat] = geph.iode;
        pending_glonass_eph_.push_back(gnss_utils::gephToMsg(geph));
      }
    }
  }

  void flushPendingEphemerides() {
    if (pending_gnss_eph_.empty() && pending_glonass_eph_.empty()) return;

    msg::GnssEphemerides msg;
    msg.header.stamp      = now();
    msg.gnss_ephemeris    = std::move(pending_gnss_eph_);
    msg.glonass_ephemeris = std::move(pending_glonass_eph_);
    eph_pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "Published ephemerides: GNSS=%zu GLO=%zu",
      msg.gnss_ephemeris.size(), msg.glonass_ephemeris.size());
  }

  void publishEphemerides() {
    accumulateRtklibEphemerides();
    flushPendingEphemerides();
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

  struct EphemerisKey {
    int sat, iode, iodc, code;
    bool operator==(const EphemerisKey& o) const {
      return sat==o.sat && iode==o.iode && iodc==o.iodc && code==o.code;
    }
  };
  struct EphemerisKeyHash {
    size_t operator()(const EphemerisKey& k) const {
      return static_cast<size_t>(k.sat) ^ (static_cast<size_t>(k.iode) << 16) ^
             (static_cast<size_t>(k.iodc) << 1) ^ (static_cast<size_t>(k.code) << 24);
    }
  };

  // ============================================================================
  // Member Variables
  // ============================================================================

  std::string frame_id_;

  rclcpp::Publisher<msg::GnssObservations>::SharedPtr  obs_pub_;
  rclcpp::Publisher<msg::GnssEphemerides>::SharedPtr   eph_pub_;
  rclcpp::Publisher<msg::GnssSolution>::SharedPtr      sol_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr  imu_attitude_pub_;  // AttEuler orientation
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr  imu_raw_pub_;       // ExtSensorMeas accel/gyro
  rclcpp::TimerBase::SharedPtr timer_;

  stream_t stream_{};
  raw_t    raw_{};
  rtcm_t   rtcm_{};

  // NMEA parsing state
  gnss_utils::NmeaParser nmea_parser_;
  std::string            nmea_buffer_;
  msg::GnssSolution      current_solution_;

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
  double   esm_accel_[3]{0.0, 0.0, 0.0};
  double   esm_gyro_[3]{0.0, 0.0, 0.0};
  bool     esm_has_accel_{false};
  bool     esm_has_gyro_{false};

  // Ephemeris dedup (shared between raw RTKLIB path and decoded *Nav path)
  std::unordered_set<EphemerisKey, EphemerisKeyHash> seen_ephemeris_;
  std::unordered_map<int, int> last_glo_iode_;

  // Pending ephemeris queues (populated by both paths, flushed together)
  std::vector<msg::GnssEphemeris>    pending_gnss_eph_;
  std::vector<msg::GlonassEphemeris> pending_glonass_eph_;
};

}  // namespace gnss_ros_standardization

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gnss_ros_standardization::SbfDecoderNode>());
  rclcpp::shutdown();
  return 0;
}
