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

// SBF AttEuler (ID 5938) body offsets
constexpr int ATT_EULER_OFFSET_ERROR   = 7;
constexpr int ATT_EULER_OFFSET_HEADING = 10;
constexpr int ATT_EULER_OFFSET_PITCH   = 14;
constexpr int ATT_EULER_OFFSET_ROLL    = 18;
constexpr int ATT_EULER_MIN_LEN        = 22;

// SBF ExtSensorMeas (ID 4050) sub-block layout (SBLength = 28):
//   Source(u1) SensorModel(u1) Type(u1) ObsInfo(u1) X(f8) Y(f8) Z(f8)
// Type carries a full 3-vector (Type 0 = accel 3-axis, Type 1 = gyro 3-axis).
constexpr int ESM_OFFSET_N          = 6;
constexpr int ESM_OFFSET_SB_LENGTH  = 7;
constexpr int ESM_OFFSET_SUBBLOCKS  = 8;
constexpr int ESM_SB_OFFSET_TYPE    = 2;
constexpr int ESM_SB_OFFSET_X       = 4;
constexpr int ESM_SB_OFFSET_Y       = 12;
constexpr int ESM_SB_OFFSET_Z       = 20;
constexpr int ESM_SB_MIN_LEN        = 28;
constexpr int ESM_MIN_BODY_LEN      = 8;
constexpr uint8_t ESM_TYPE_ACCEL    = 0;   // Accelerations [m/s²]
constexpr uint8_t ESM_TYPE_GYRO     = 1;   // Angular rates [rad/s]

}  // namespace

/// @brief Configuration structure for Septentrio driver
struct SbfConfig {
  std::string stream_path{"serial:///dev/ttyACM0:115200"};
  std::string frame_id{"gnss_link"};
  int publish_rate{1};
  std::string receiver_port{"USB1"};
  bool configure_on_startup{true};

  // SBF message settings
  bool enable_meas_epoch{true};

  // Decoded *Nav blocks (receiver-assembled, GPSNav/GLONav/GALNav/BDSNav/QZSNav/NavICNav)
  bool enable_gps_nav{true};
  bool enable_glo_nav{true};
  bool enable_gal_nav{true};
  bool enable_bds_nav{true};
  bool enable_qzs_nav{true};
  bool enable_navic_nav{true};

  // Raw subframe blocks decoded by RTKLIB (GPSRawCA/GLORawCA/GALRawINAV+FNAV/BDSRaw/QZSRawL1CA/NAVICRaw)
  bool enable_gps_nav_raw{true};
  bool enable_glo_nav_raw{true};
  bool enable_gal_nav_raw{true};
  bool enable_bds_nav_raw{true};
  bool enable_qzs_nav_raw{true};
  bool enable_navic_nav_raw{true};

  bool enable_pvt_geodetic{false};
  bool enable_pos_cov_geodetic{false};
  bool enable_pvt_cartesian{false};
  bool enable_pos_cov_cartesian{false};

  bool enable_att_euler{false};
  bool enable_att_cov_euler{false};
  bool enable_ext_sensor_meas{false};  // Raw IMU accel + gyro (AsteRx-i / mosaic-X5 with IMU)

  bool enable_receiver_status{false};
  bool enable_quality_ind{false};

  // NMEA message settings (for GnssSolution output)
  bool enable_nmea_gga{true};
  bool enable_nmea_rmc{true};
  bool enable_nmea_gsa{true};
  bool enable_nmea_gst{true};

  // Topics
  std::string observation_topic{"/gnss/observation"};
  std::string ephemeris_topic{"/gnss/ephemeris"};
  std::string solution_topic{"/gnss/solution"};
  std::string imu_attitude_topic{"/gnss/imu/attitude"};
  std::string imu_raw_topic{"/gnss/imu/data_raw"};
};

/// @brief ROS 2 driver node for Septentrio GNSS receivers
class SbfDriverNode : public rclcpp::Node {
 public:
  SbfDriverNode() : Node("sbf_driver_node") {
    initializeParameters();
    initializePublishers();
    initializeDecoder();
    openStream();

    if (config_.configure_on_startup) {
      configureReceiver();
    }

    startPolling();
    RCLCPP_INFO(get_logger(), "SBF Driver initialized");
  }

  ~SbfDriverNode() override {
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
    declare_parameter<std::string>("stream_path",          config_.stream_path);
    declare_parameter<std::string>("frame_id",             config_.frame_id);
    declare_parameter<int>        ("publish_rate",         config_.publish_rate);
    declare_parameter<std::string>("receiver_port",        config_.receiver_port);
    declare_parameter<bool>       ("configure_on_startup", config_.configure_on_startup);

    declare_parameter<bool>("messages.meas_epoch",         config_.enable_meas_epoch);
    declare_parameter<bool>("messages.gps_nav",            config_.enable_gps_nav);
    declare_parameter<bool>("messages.glo_nav",            config_.enable_glo_nav);
    declare_parameter<bool>("messages.gal_nav",            config_.enable_gal_nav);
    declare_parameter<bool>("messages.bds_nav",            config_.enable_bds_nav);
    declare_parameter<bool>("messages.qzs_nav",            config_.enable_qzs_nav);
    declare_parameter<bool>("messages.navic_nav",          config_.enable_navic_nav);
    declare_parameter<bool>("messages.gps_nav_raw",        config_.enable_gps_nav_raw);
    declare_parameter<bool>("messages.glo_nav_raw",        config_.enable_glo_nav_raw);
    declare_parameter<bool>("messages.gal_nav_raw",        config_.enable_gal_nav_raw);
    declare_parameter<bool>("messages.bds_nav_raw",        config_.enable_bds_nav_raw);
    declare_parameter<bool>("messages.qzs_nav_raw",        config_.enable_qzs_nav_raw);
    declare_parameter<bool>("messages.navic_nav_raw",      config_.enable_navic_nav_raw);
    declare_parameter<bool>("messages.pvt_geodetic",       config_.enable_pvt_geodetic);
    declare_parameter<bool>("messages.pos_cov_geodetic",   config_.enable_pos_cov_geodetic);
    declare_parameter<bool>("messages.pvt_cartesian",      config_.enable_pvt_cartesian);
    declare_parameter<bool>("messages.pos_cov_cartesian",  config_.enable_pos_cov_cartesian);
    declare_parameter<bool>("messages.att_euler",          config_.enable_att_euler);
    declare_parameter<bool>("messages.att_cov_euler",      config_.enable_att_cov_euler);
    declare_parameter<bool>("messages.ext_sensor_meas",    config_.enable_ext_sensor_meas);
    declare_parameter<bool>("messages.receiver_status",    config_.enable_receiver_status);
    declare_parameter<bool>("messages.quality_ind",        config_.enable_quality_ind);
    declare_parameter<bool>("messages.nmea_gga",           config_.enable_nmea_gga);
    declare_parameter<bool>("messages.nmea_rmc",           config_.enable_nmea_rmc);
    declare_parameter<bool>("messages.nmea_gsa",           config_.enable_nmea_gsa);
    declare_parameter<bool>("messages.nmea_gst",           config_.enable_nmea_gst);

    declare_parameter<std::string>("observation_topic", config_.observation_topic);
    declare_parameter<std::string>("ephemeris_topic",   config_.ephemeris_topic);
    declare_parameter<std::string>("solution_topic",    config_.solution_topic);
    declare_parameter<std::string>("imu_attitude_topic", config_.imu_attitude_topic);
    declare_parameter<std::string>("imu_raw_topic",     config_.imu_raw_topic);

    config_.stream_path          = get_parameter("stream_path").as_string();
    config_.frame_id             = get_parameter("frame_id").as_string();
    config_.publish_rate         = get_parameter("publish_rate").as_int();
    config_.receiver_port        = get_parameter("receiver_port").as_string();
    config_.configure_on_startup = get_parameter("configure_on_startup").as_bool();

    config_.enable_meas_epoch        = get_parameter("messages.meas_epoch").as_bool();
    config_.enable_gps_nav           = get_parameter("messages.gps_nav").as_bool();
    config_.enable_glo_nav           = get_parameter("messages.glo_nav").as_bool();
    config_.enable_gal_nav           = get_parameter("messages.gal_nav").as_bool();
    config_.enable_bds_nav           = get_parameter("messages.bds_nav").as_bool();
    config_.enable_qzs_nav           = get_parameter("messages.qzs_nav").as_bool();
    config_.enable_navic_nav         = get_parameter("messages.navic_nav").as_bool();
    config_.enable_gps_nav_raw       = get_parameter("messages.gps_nav_raw").as_bool();
    config_.enable_glo_nav_raw       = get_parameter("messages.glo_nav_raw").as_bool();
    config_.enable_gal_nav_raw       = get_parameter("messages.gal_nav_raw").as_bool();
    config_.enable_bds_nav_raw       = get_parameter("messages.bds_nav_raw").as_bool();
    config_.enable_qzs_nav_raw       = get_parameter("messages.qzs_nav_raw").as_bool();
    config_.enable_navic_nav_raw     = get_parameter("messages.navic_nav_raw").as_bool();
    config_.enable_pvt_geodetic      = get_parameter("messages.pvt_geodetic").as_bool();
    config_.enable_pos_cov_geodetic  = get_parameter("messages.pos_cov_geodetic").as_bool();
    config_.enable_pvt_cartesian     = get_parameter("messages.pvt_cartesian").as_bool();
    config_.enable_pos_cov_cartesian = get_parameter("messages.pos_cov_cartesian").as_bool();
    config_.enable_att_euler         = get_parameter("messages.att_euler").as_bool();
    config_.enable_att_cov_euler     = get_parameter("messages.att_cov_euler").as_bool();
    config_.enable_ext_sensor_meas   = get_parameter("messages.ext_sensor_meas").as_bool();
    config_.enable_receiver_status   = get_parameter("messages.receiver_status").as_bool();
    config_.enable_quality_ind       = get_parameter("messages.quality_ind").as_bool();
    config_.enable_nmea_gga          = get_parameter("messages.nmea_gga").as_bool();
    config_.enable_nmea_rmc          = get_parameter("messages.nmea_rmc").as_bool();
    config_.enable_nmea_gsa          = get_parameter("messages.nmea_gsa").as_bool();
    config_.enable_nmea_gst          = get_parameter("messages.nmea_gst").as_bool();

    config_.observation_topic = get_parameter("observation_topic").as_string();
    config_.ephemeris_topic   = get_parameter("ephemeris_topic").as_string();
    config_.solution_topic    = get_parameter("solution_topic").as_string();
    config_.imu_attitude_topic = get_parameter("imu_attitude_topic").as_string();
    config_.imu_raw_topic      = get_parameter("imu_raw_topic").as_string();
  }

  void initializePublishers() {
    obs_pub_          = create_publisher<msg::GnssObservations>(config_.observation_topic, 10);
    eph_pub_          = create_publisher<msg::GnssEphemerides>(config_.ephemeris_topic, rclcpp::QoS(100).transient_local());
    sol_pub_          = create_publisher<msg::GnssSolution>(config_.solution_topic, 10);
    imu_attitude_pub_ = create_publisher<sensor_msgs::msg::Imu>(config_.imu_attitude_topic, 10);
    imu_raw_pub_      = create_publisher<sensor_msgs::msg::Imu>(config_.imu_raw_topic, 10);
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
    timer_ = create_wall_timer(kTimerInterval, std::bind(&SbfDriverNode::pollStream, this));
  }

  // ============================================================================
  // Stream Management
  // ============================================================================

  void openStream() {
    std::string path = config_.stream_path;

    int stream_type = STR_SERIAL;
    bool matched = false;
    for (const auto& def : sbf::kStreamTypes) {
      if (path.rfind(def.prefix, 0) == 0) {
        stream_type = def.type;
        path.erase(0, def.prefix.size());
        matched = true;
        break;
      }
    }

    if (!matched && path.rfind("/dev/", 0) == 0) {
      stream_type = STR_SERIAL;
    }

    if (stream_type == STR_SERIAL && path.rfind("/dev/", 0) == 0) {
      path.erase(0, 5);
    }

    strinit(&stream_);

    if (!stropen(&stream_, stream_type, STR_MODE_RW, path.c_str())) {
      RCLCPP_ERROR(get_logger(), "Failed to open stream: %s", config_.stream_path.c_str());
      throw std::runtime_error("stropen failed");
    }

    is_serial_connection_ = (stream_type == STR_SERIAL);
    RCLCPP_INFO(get_logger(), "Stream opened: %s", config_.stream_path.c_str());
  }

  // ============================================================================
  // Receiver Configuration
  // ============================================================================

  void sendCommand(const std::string& cmd) {
    std::string full_cmd = cmd + "\r\n";
    int written = strwrite(&stream_, (uint8_t*)full_cmd.c_str(), full_cmd.size());
    if (written < 0 || (written == 0 && !is_serial_connection_)) {
      RCLCPP_ERROR(get_logger(), "Failed to write command: %s", cmd.c_str());
    }
    std::this_thread::sleep_for(50ms);
  }

  void configureReceiver() {
    RCLCPP_INFO(get_logger(), "Configuring receiver...");

    const std::string interval_str = sbf::getIntervalString(config_.publish_rate);

    // --- SBF output (Stream1) ---
    std::vector<std::string> blocks;
    if (config_.enable_meas_epoch)       blocks.push_back(sbf::BLOCK_MEASEPOCH);
    // Decoded *Nav blocks (receiver-assembled)
    if (config_.enable_gps_nav)          blocks.push_back(sbf::BLOCK_GPSNAV);
    if (config_.enable_glo_nav)          blocks.push_back(sbf::BLOCK_GLONAV);
    if (config_.enable_gal_nav)          blocks.push_back(sbf::BLOCK_GALNAV);
    if (config_.enable_bds_nav)          blocks.push_back(sbf::BLOCK_BDSNAV);
    if (config_.enable_qzs_nav)          blocks.push_back(sbf::BLOCK_QZSNAV);
    if (config_.enable_navic_nav)        blocks.push_back(sbf::BLOCK_NAVICNAV);
    // Raw subframe blocks (decoded by RTKLIB)
    if (config_.enable_gps_nav_raw)      blocks.push_back(sbf::BLOCK_GPSNAV_RAW);
    if (config_.enable_glo_nav_raw)      blocks.push_back(sbf::BLOCK_GLONAV_RAW);
    if (config_.enable_gal_nav_raw)      blocks.push_back(sbf::BLOCK_GALNAV_RAW);
    if (config_.enable_bds_nav_raw)      blocks.push_back(sbf::BLOCK_BDSNAV_RAW);
    if (config_.enable_qzs_nav_raw)      blocks.push_back(sbf::BLOCK_QZSNAV_RAW);
    if (config_.enable_navic_nav_raw)    blocks.push_back(sbf::BLOCK_NAVICNAV_RAW);
    if (config_.enable_pvt_geodetic)     blocks.push_back(sbf::BLOCK_PVTGEODETIC);
    if (config_.enable_pos_cov_geodetic) blocks.push_back(sbf::BLOCK_POSCOVGEODETIC);
    if (config_.enable_pvt_cartesian)    blocks.push_back(sbf::BLOCK_PVTCARTESIAN);
    if (config_.enable_pos_cov_cartesian)blocks.push_back(sbf::BLOCK_POSCOVCARTESIAN);
    if (config_.enable_att_euler)        blocks.push_back(sbf::BLOCK_ATTEULER);
    if (config_.enable_att_cov_euler)    blocks.push_back(sbf::BLOCK_ATTCOVEULER);
    if (config_.enable_ext_sensor_meas)  blocks.push_back(sbf::BLOCK_EXTSENSORMEAS);
    if (config_.enable_receiver_status)  blocks.push_back(sbf::BLOCK_RECEIVERSTATUS);
    if (config_.enable_quality_ind)      blocks.push_back(sbf::BLOCK_QUALITYIND);

    if (!blocks.empty()) {
      std::string messages = blocks[0];
      for (size_t i = 1; i < blocks.size(); ++i) messages += "+" + blocks[i];

      std::string sbf_cmd = std::string(sbf::CMD_SET_SBF_OUTPUT) + ", Stream1, " +
                            config_.receiver_port + ", " + messages + ", " + interval_str;
      RCLCPP_INFO(get_logger(), "SBF config: %s", sbf_cmd.c_str());
      sendCommand(sbf_cmd);
    } else {
      RCLCPP_WARN(get_logger(), "No SBF blocks enabled.");
    }

    // --- NMEA output (Stream2, same physical port) ---
    std::vector<std::string> nmea_sentences;
    if (config_.enable_nmea_gga) nmea_sentences.push_back("GGA");
    if (config_.enable_nmea_rmc) nmea_sentences.push_back("RMC");
    if (config_.enable_nmea_gsa) nmea_sentences.push_back("GSA");
    if (config_.enable_nmea_gst) nmea_sentences.push_back("GST");

    if (!nmea_sentences.empty()) {
      std::string nmea_list = nmea_sentences[0];
      for (size_t i = 1; i < nmea_sentences.size(); ++i) nmea_list += "+" + nmea_sentences[i];

      // Use Stream2 for NMEA on the same physical port (sec1 = 1 Hz)
      std::string nmea_cmd = std::string(sbf::CMD_SET_NMEA_OUTPUT) + ", Stream2, " +
                             config_.receiver_port + ", " + nmea_list + ", sec1";
      RCLCPP_INFO(get_logger(), "NMEA config: %s", nmea_cmd.c_str());
      sendCommand(nmea_cmd);
    }

    std::this_thread::sleep_for(200ms);

    // Read and log receiver response
    uint8_t resp[1024];
    int n = strread(&stream_, resp, sizeof(resp) - 1);
    if (n > 0) {
      resp[n] = '\0';
      std::string response(reinterpret_cast<char*>(resp));
      RCLCPP_INFO(get_logger(), "Receiver response: %s", response.c_str());

      if (response.find("invalid") != std::string::npos ||
          response.find("Error")   != std::string::npos) {
        if (config_.enable_navic_nav || config_.enable_navic_nav_raw) {
          RCLCPP_WARN(get_logger(), "Receiver rejected config with NavIC. Retrying without NavIC...");
          config_.enable_navic_nav     = false;
          config_.enable_navic_nav_raw = false;
          configureReceiver();
        } else {
          RCLCPP_ERROR(get_logger(), "Receiver rejected configuration. Check port and settings.");
        }
      }
    } else {
      RCLCPP_WARN(get_logger(), "No response from receiver after configuration.");
    }
  }

  // ============================================================================
  // Polling
  // ============================================================================

  void pollStream() {
    uint8_t buffer[sbf::READ_BUFFER_SIZE];
    const int bytes_read = strread(&stream_, buffer, sizeof(buffer));

    for (int i = 0; i < bytes_read; ++i) {
      const uint8_t byte = buffer[i];

      const int result = input_sbf(&raw_, &rtcm_, byte);
      handleDecodeResult(result);

      // Parallel SBF mini-framer for AttEuler
      parseSbfByte(byte);

      // NMEA sentences (interleaved with SBF binary)
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
  // ============================================================================

  void parseSbfByte(uint8_t byte) {
    switch (sbf_state_) {
      case 0: if (byte == sbf::SBF_SYNC1) sbf_state_ = 1; break;
      case 1: sbf_state_ = (byte == sbf::SBF_SYNC2) ? 2 : (byte == sbf::SBF_SYNC1 ? 1 : 0); break;
      case 2: sbf_state_ = 3; break;  // CRC LSB skip
      case 3: sbf_state_ = 4; break;  // CRC MSB skip
      case 4: sbf_id_ = byte; sbf_state_ = 5; break;
      case 5:
        sbf_id_ |= static_cast<uint16_t>(byte << 8);
        sbf_id_ &= 0x1FFF;
        sbf_state_ = 6;
        break;
      case 6: sbf_len_ = byte; sbf_state_ = 7; break;
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
      default: sbf_state_ = 0;
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
    if (!config_.enable_att_euler) return;
    if (static_cast<int>(sbf_body_.size()) < ATT_EULER_MIN_LEN) return;
    if (sbf_body_[ATT_EULER_OFFSET_ERROR] != 0) return;

    float heading_deg = 0.0f, pitch_deg = 0.0f, roll_deg = 0.0f;
    std::memcpy(&heading_deg, sbf_body_.data() + ATT_EULER_OFFSET_HEADING, 4);
    std::memcpy(&pitch_deg,   sbf_body_.data() + ATT_EULER_OFFSET_PITCH,   4);
    std::memcpy(&roll_deg,    sbf_body_.data() + ATT_EULER_OFFSET_ROLL,    4);

    const double deg2rad = M_PI / 180.0;

    sensor_msgs::msg::Imu imu;
    imu.header.stamp    = now();
    imu.header.frame_id = config_.frame_id;
    imu.orientation = ins::eulerToQuaternion(
        roll_deg    * deg2rad,
        pitch_deg   * deg2rad,
        heading_deg * deg2rad);

    auto unk = ins::makeUnknownCovariance();
    std::copy(unk.begin(), unk.end(), imu.orientation_covariance.begin());
    std::copy(unk.begin(), unk.end(), imu.angular_velocity_covariance.begin());
    std::copy(unk.begin(), unk.end(), imu.linear_acceleration_covariance.begin());

    imu_attitude_pub_->publish(imu);
  }

  void handleExtSensorMeas() {
    if (!config_.enable_ext_sensor_meas) return;
    if (static_cast<int>(sbf_body_.size()) < ESM_MIN_BODY_LEN) return;

    const uint8_t n         = sbf_body_[ESM_OFFSET_N];
    const uint8_t sb_length = sbf_body_[ESM_OFFSET_SB_LENGTH];
    if (sb_length < ESM_SB_MIN_LEN) return;
    if (static_cast<int>(sbf_body_.size()) < ESM_MIN_BODY_LEN + n * sb_length) return;

    uint32_t tow_ms = 0;
    std::memcpy(&tow_ms, sbf_body_.data(), 4);

    if (tow_ms != esm_tow_ms_ && (esm_has_accel_ || esm_has_gyro_)) {
      publishEsmAccum();
    }
    esm_tow_ms_ = tow_ms;

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

    if (esm_has_accel_ && esm_has_gyro_) publishEsmAccum();
  }

  void publishEsmAccum() {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp    = now();
    imu.header.frame_id = config_.frame_id;

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
    if (!config_.enable_gps_nav) return;
    eph_t eph{};
    if (!sbf::nav::parseGPSNav(sbf_body_, eph)) return;
    EphemerisKey key{eph.sat, eph.iode, eph.iodc, eph.code};
    if (!seen_ephemeris_.insert(key).second) return;
    pending_gnss_eph_.push_back(gnss_utils::ephToMsg(eph));
    flushPendingEphemerides();
  }

  void handleGloNav() {
    if (!config_.enable_glo_nav) return;
    geph_t geph{};
    if (!sbf::nav::parseGLONav(sbf_body_, geph)) return;
    auto it = last_glo_iode_.find(geph.sat);
    if (it != last_glo_iode_.end() && it->second == geph.iode) return;
    last_glo_iode_[geph.sat] = geph.iode;
    pending_glonass_eph_.push_back(gnss_utils::gephToMsg(geph));
    flushPendingEphemerides();
  }

  void handleGalNav() {
    if (!config_.enable_gal_nav) return;
    eph_t eph{};
    if (!sbf::nav::parseGALNav(sbf_body_, eph)) return;
    EphemerisKey key{eph.sat, eph.iode, eph.iodc, eph.code};
    if (!seen_ephemeris_.insert(key).second) return;
    pending_gnss_eph_.push_back(gnss_utils::ephToMsg(eph));
    flushPendingEphemerides();
  }

  void handleBdsNav() {
    if (!config_.enable_bds_nav) return;
    eph_t eph{};
    if (!sbf::nav::parseBDSNav(sbf_body_, eph)) return;
    EphemerisKey key{eph.sat, eph.iode, eph.iodc, eph.code};
    if (!seen_ephemeris_.insert(key).second) return;
    pending_gnss_eph_.push_back(gnss_utils::ephToMsg(eph));
    flushPendingEphemerides();
  }

  void handleQzsNav() {
    if (!config_.enable_qzs_nav) return;
    eph_t eph{};
    if (!sbf::nav::parseQZSNav(sbf_body_, eph)) return;
    EphemerisKey key{eph.sat, eph.iode, eph.iodc, eph.code};
    if (!seen_ephemeris_.insert(key).second) return;
    pending_gnss_eph_.push_back(gnss_utils::ephToMsg(eph));
    flushPendingEphemerides();
  }

  void handleNavicNav() {
    if (!config_.enable_navic_nav) return;
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
      default:
        if (result > 0) {
          RCLCPP_DEBUG(get_logger(), "SBF message type %d (not handled)", result);
        }
        break;
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
    current_solution_.header.frame_id = config_.frame_id;

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
    msg.header.frame_id = config_.frame_id;
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
      "Published obs: week=%d tow=%.3f n=%zu sats(G/R/E/J/C/I/S/U)=(%d/%d/%d/%d/%d/%d/%d/%d)",
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

  // Accumulate new ephemerides from RTKLIB's raw_.nav arrays into pending queues.
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

  // Publish all pending ephemerides in one message, then clear the pending queues.
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

  // Called by handleDecodeResult(2) — walk RTKLIB nav arrays then flush.
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

  SbfConfig config_;

  rclcpp::Publisher<msg::GnssObservations>::SharedPtr  obs_pub_;
  rclcpp::Publisher<msg::GnssEphemerides>::SharedPtr   eph_pub_;
  rclcpp::Publisher<msg::GnssSolution>::SharedPtr      sol_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr  imu_attitude_pub_;  // AttEuler orientation
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr  imu_raw_pub_;       // ExtSensorMeas accel/gyro
  rclcpp::TimerBase::SharedPtr timer_;

  stream_t stream_{};
  raw_t    raw_{};
  rtcm_t   rtcm_{};
  bool is_serial_connection_{false};

  // SBF mini-framer state
  int                  sbf_state_{0};
  uint16_t             sbf_id_{0};
  uint16_t             sbf_len_{0};
  int                  sbf_body_pos_{0};
  std::vector<uint8_t> sbf_body_;

  // ExtSensorMeas accumulator (TOW-based, collects accel + gyro 3-vectors)
  uint32_t esm_tow_ms_{0xFFFFFFFFu};
  double   esm_accel_[3]{0.0, 0.0, 0.0};
  double   esm_gyro_[3]{0.0, 0.0, 0.0};
  bool     esm_has_accel_{false};
  bool     esm_has_gyro_{false};

  // NMEA parsing state
  gnss_utils::NmeaParser nmea_parser_;
  std::string            nmea_buffer_;
  msg::GnssSolution      current_solution_;

  // ENU local origin
  bool   has_local_origin_{false};
  double local_origin_ecef_[3]{0.0};
  double local_origin_pos_[3]{0.0};

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
  rclcpp::spin(std::make_shared<gnss_ros_standardization::SbfDriverNode>());
  rclcpp::shutdown();
  return 0;
}
