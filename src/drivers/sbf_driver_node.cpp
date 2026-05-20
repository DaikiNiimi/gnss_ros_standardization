// SPDX-License-Identifier: MIT
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
#include "gnss_ros_standardization/sbf_protocol.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"

using namespace std::chrono_literals;
namespace ins = gnss_ros_standardization::ins_utils;

namespace gnss_ros_standardization {

namespace {

constexpr auto kTimerInterval  = 10ms;
constexpr size_t kNmeaMaxLineLen = 256;

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
  bool enable_vel_cov_geodetic{false};
  bool enable_pvt_cartesian{false};
  bool enable_pos_cov_cartesian{false};
  bool enable_vel_cov_cartesian{false};
  bool enable_dop{false};

  bool enable_ext_sensor_meas{false};  // Raw IMU accel + gyro (AsteRx-i / mosaic-X5 with IMU)

  // NMEA message settings (for GnssSolution output)
  bool enable_nmea_gga{true};
  bool enable_nmea_rmc{true};
  bool enable_nmea_gsa{true};
  bool enable_nmea_gst{true};

  // Topics
  std::string observation_topic{"/gnss/observation"};
  std::string ephemeris_topic{"/gnss/ephemeris"};
  std::string solution_topic{"/gnss/nmea_solution"};
  std::string imu_raw_topic{"/gnss/imu/data_raw"};  // ExtSensorMeas (uncalibrated)

  // Ephemeris snapshot behavior
  double ephemeris_snapshot_period_s{30.0};
  double ephemeris_max_age_s{0.0};  // 0 = keep all

  bool use_gps_timestamp{false};

  // ENU local origin
  bool auto_origin{true};
  double origin_latitude{0.0};
  double origin_longitude{0.0};
  double origin_altitude{0.0};
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
    declare_parameter<bool>("messages.vel_cov_geodetic",   config_.enable_vel_cov_geodetic);
    declare_parameter<bool>("messages.pvt_cartesian",      config_.enable_pvt_cartesian);
    declare_parameter<bool>("messages.pos_cov_cartesian",  config_.enable_pos_cov_cartesian);
    declare_parameter<bool>("messages.vel_cov_cartesian",  config_.enable_vel_cov_cartesian);
    declare_parameter<bool>("messages.dop",                config_.enable_dop);
    declare_parameter<bool>("messages.ext_sensor_meas",    config_.enable_ext_sensor_meas);
    declare_parameter<bool>("messages.nmea_gga",           config_.enable_nmea_gga);
    declare_parameter<bool>("messages.nmea_rmc",           config_.enable_nmea_rmc);
    declare_parameter<bool>("messages.nmea_gsa",           config_.enable_nmea_gsa);
    declare_parameter<bool>("messages.nmea_gst",           config_.enable_nmea_gst);

    declare_parameter<std::string>("observation_topic", config_.observation_topic);
    declare_parameter<std::string>("ephemeris_topic",   config_.ephemeris_topic);
    declare_parameter<std::string>("solution_topic",    config_.solution_topic);
    declare_parameter<std::string>("imu_raw_topic",     config_.imu_raw_topic);

    declare_parameter<double>("ephemeris.snapshot_period_s", config_.ephemeris_snapshot_period_s);
    declare_parameter<double>("ephemeris.max_age_s",         config_.ephemeris_max_age_s);
    declare_parameter<bool>("use_gps_timestamp",             config_.use_gps_timestamp);
    declare_parameter<bool>("auto_origin",                   config_.auto_origin);
    declare_parameter<std::vector<double>>("origin",         {0.0, 0.0, 0.0});

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
    config_.enable_vel_cov_geodetic  = get_parameter("messages.vel_cov_geodetic").as_bool();
    config_.enable_pvt_cartesian     = get_parameter("messages.pvt_cartesian").as_bool();
    config_.enable_pos_cov_cartesian = get_parameter("messages.pos_cov_cartesian").as_bool();
    config_.enable_vel_cov_cartesian = get_parameter("messages.vel_cov_cartesian").as_bool();
    config_.enable_dop               = get_parameter("messages.dop").as_bool();
    config_.enable_ext_sensor_meas   = get_parameter("messages.ext_sensor_meas").as_bool();
    config_.enable_nmea_gga          = get_parameter("messages.nmea_gga").as_bool();
    config_.enable_nmea_rmc          = get_parameter("messages.nmea_rmc").as_bool();
    config_.enable_nmea_gsa          = get_parameter("messages.nmea_gsa").as_bool();
    config_.enable_nmea_gst          = get_parameter("messages.nmea_gst").as_bool();

    config_.observation_topic = get_parameter("observation_topic").as_string();
    config_.ephemeris_topic   = get_parameter("ephemeris_topic").as_string();
    config_.solution_topic    = get_parameter("solution_topic").as_string();
    config_.imu_raw_topic     = get_parameter("imu_raw_topic").as_string();

    config_.ephemeris_snapshot_period_s = get_parameter("ephemeris.snapshot_period_s").as_double();
    config_.ephemeris_max_age_s         = get_parameter("ephemeris.max_age_s").as_double();
    config_.use_gps_timestamp           = get_parameter("use_gps_timestamp").as_bool();
    config_.auto_origin                 = get_parameter("auto_origin").as_bool();
    {
      const auto v = get_parameter("origin").as_double_array();
      if (v.size() == 3) {
        config_.origin_latitude  = v[0];
        config_.origin_longitude = v[1];
        config_.origin_altitude  = v[2];
      }
    }

    eph_store_.setSnapshotPeriod(config_.ephemeris_snapshot_period_s);
    eph_store_.setMaxAge(config_.ephemeris_max_age_s);

    initPendingExpectedMasks();

    // Lock solution source at startup. BINARY if either PVTGeodetic or
    // PVTCartesian is enabled; otherwise NMEA. No mid-session switching.
    const bool binary_enabled = config_.enable_pvt_geodetic || config_.enable_pvt_cartesian;
    source_ = binary_enabled ? SolutionSource::BINARY : SolutionSource::NMEA;
    start_time_ = now();
    RCLCPP_INFO(get_logger(), "Solution source locked: %s",
                source_ == SolutionSource::BINARY ? "BINARY (PVTGeodetic/PVTCartesian)" : "NMEA");

    // Recommend Cov block when PVT block is enabled (publishing still proceeds
    // with zero covariance if Cov is missing — flush-on-next-TOW behavior).
    if (config_.enable_pvt_geodetic && !config_.enable_pos_cov_geodetic) {
      RCLCPP_WARN(get_logger(),
        "PVTGeodetic enabled without PosCovGeodetic — published pos_enu_cov will be zero.");
    }
    if (config_.enable_pvt_cartesian && !config_.enable_pos_cov_cartesian) {
      RCLCPP_WARN(get_logger(),
        "PVTCartesian enabled without PosCovCartesian — published pos_cov_ecef will be zero.");
    }
    // Both PVTGeodetic and PVTCartesian are unnecessary together: PVTGeodetic
    // alone is sufficient (ECEF derived from LLH), and Cartesian fills ECEF
    // directly when available. Warn but don't block.
    if (config_.enable_pos_cov_geodetic && config_.enable_pos_cov_cartesian) {
      RCLCPP_WARN(get_logger(),
        "Both PosCovGeodetic and PosCovCartesian are enabled. Cartesian wins "
        "for pos_cov_ecef (rotation-derived value is overridden). Consider "
        "disabling PosCovGeodetic to save receiver bandwidth.");
    }
    if (config_.enable_vel_cov_geodetic && config_.enable_vel_cov_cartesian) {
      RCLCPP_WARN(get_logger(),
        "Both VelCovGeodetic and VelCovCartesian are enabled. Cartesian wins "
        "for vel_cov_ecef. Consider disabling VelCovGeodetic.");
    }
  }

  void initializePublishers() {
    obs_pub_          = create_publisher<msg::GnssObservations>(config_.observation_topic, 10);
    eph_pub_          = create_publisher<msg::GnssEphemerides>(config_.ephemeris_topic, rclcpp::QoS(1).transient_local());
    sol_pub_          = create_publisher<msg::GnssSolution>(config_.solution_topic, 10);
    imu_raw_pub_      = create_publisher<sensor_msgs::msg::Imu>(config_.imu_raw_topic, 10);

    if (!config_.auto_origin) {
      if (config_.origin_latitude == 0.0 && config_.origin_longitude == 0.0) {
        RCLCPP_WARN(get_logger(),
          "auto_origin=false but origin is [0,0,0] — falling back to auto (first fix)");
      } else {
        local_origin_pos_[0] = config_.origin_latitude  * (M_PI / 180.0);
        local_origin_pos_[1] = config_.origin_longitude * (M_PI / 180.0);
        local_origin_pos_[2] = config_.origin_altitude;
        pos2ecef(local_origin_pos_, local_origin_ecef_);
        has_local_origin_ = true;
        RCLCPP_INFO(get_logger(), "ENU origin set from config: lat=%.6f lon=%.6f alt=%.2f",
          config_.origin_latitude, config_.origin_longitude, config_.origin_altitude);
      }
    }
    logEnabledMessages();
  }

  void logEnabledMessages() {
    auto on = [](bool v) { return v ? "ON" : "OFF"; };
    RCLCPP_INFO(get_logger(), "Enabled messages:");
    RCLCPP_INFO(get_logger(), "  Observation  : MeasEpoch=%s", on(config_.enable_meas_epoch));
    RCLCPP_INFO(get_logger(), "  Navigation   : GPS=%s GLO=%s GAL=%s BDS=%s QZS=%s NavIC=%s",
      on(config_.enable_gps_nav), on(config_.enable_glo_nav), on(config_.enable_gal_nav),
      on(config_.enable_bds_nav), on(config_.enable_qzs_nav), on(config_.enable_navic_nav));
    RCLCPP_INFO(get_logger(), "  Nav (raw)    : GPS=%s GLO=%s GAL=%s BDS=%s QZS=%s NavIC=%s",
      on(config_.enable_gps_nav_raw), on(config_.enable_glo_nav_raw), on(config_.enable_gal_nav_raw),
      on(config_.enable_bds_nav_raw), on(config_.enable_qzs_nav_raw), on(config_.enable_navic_nav_raw));
    RCLCPP_INFO(get_logger(), "  PVT (binary) : PVTGeodetic=%s PosCovGeodetic=%s VelCovGeodetic=%s PVTCartesian=%s PosCovCartesian=%s VelCovCartesian=%s DOP=%s",
      on(config_.enable_pvt_geodetic), on(config_.enable_pos_cov_geodetic), on(config_.enable_vel_cov_geodetic),
      on(config_.enable_pvt_cartesian), on(config_.enable_pos_cov_cartesian), on(config_.enable_vel_cov_cartesian),
      on(config_.enable_dop));
    RCLCPP_INFO(get_logger(), "  NMEA         : GGA=%s RMC=%s GSA=%s GST=%s",
      on(config_.enable_nmea_gga), on(config_.enable_nmea_rmc),
      on(config_.enable_nmea_gsa), on(config_.enable_nmea_gst));
    RCLCPP_INFO(get_logger(), "  IMU          : ExtSensorMeas=%s", on(config_.enable_ext_sensor_meas));
    RCLCPP_INFO(get_logger(), "  GPS timestamp : %s", on(config_.use_gps_timestamp));
    if (!config_.auto_origin && (config_.origin_latitude != 0.0 || config_.origin_longitude != 0.0)) {
      RCLCPP_INFO(get_logger(), "  ENU origin    : fixed (lat=%.6f lon=%.6f alt=%.2f)",
        config_.origin_latitude, config_.origin_longitude, config_.origin_altitude);
    } else {
      RCLCPP_INFO(get_logger(), "  ENU origin    : auto (first valid solution)");
    }
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
    if (config_.enable_vel_cov_geodetic) blocks.push_back(sbf::BLOCK_VELCOVGEODETIC);
    if (config_.enable_pvt_cartesian)    blocks.push_back(sbf::BLOCK_PVTCARTESIAN);
    if (config_.enable_pos_cov_cartesian)blocks.push_back(sbf::BLOCK_POSCOVCARTESIAN);
    if (config_.enable_vel_cov_cartesian)blocks.push_back(sbf::BLOCK_VELCOVCARTESIAN);
    if (config_.enable_dop)              blocks.push_back(sbf::BLOCK_DOP);
    if (config_.enable_ext_sensor_meas)  blocks.push_back(sbf::BLOCK_EXTSENSORMEAS);

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

      const int result = input_sbf(&raw_, byte);
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
          std::string trimmed = nmea_buffer_;
          while (!trimmed.empty() && (trimmed.back() == '\n' || trimmed.back() == '\r')) {
            trimmed.pop_back();
          }
          handleNmeaSentence(nmea_buffer_);
          nmea_buffer_.clear();
        } else if (nmea_buffer_.size() > kNmeaMaxLineLen) {
          nmea_buffer_.clear();
        }
      }
    }

    maybePublishHeartbeat();
    maybeWatchdogFlushPendingPvt();
    warnIfBinaryStarvation();
  }

  // Watchdog: if a pending TOW has been sitting incomplete for too long
  // (no further blocks arriving), flush whatever we have.
  void maybeWatchdogFlushPendingPvt() {
    if (pending_.blocks_received == 0) return;
    if ((now() - pending_.last_update).seconds() > 1.5) {
      flushPending();
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
      case sbf::ID_EXTSENSORMEAS:    handleExtSensorMeas();    break;
      // SBF Decoded nav blocks are handled by RTKLIB input_sbf() on the
      // parallel byte stream. See handleDecodeResult; the mini-framer is kept
      // here only for AttEuler / ExtSensorMeas.
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

  // ============================================================================
  // Binary PVT handlers with per-system TOW aggregation
  // ============================================================================

  // Single pending aggregator: all Geo+Xyz PVT/Cov blocks for one TOW merge into
  // pending_.buf. Flush triggers when blocks_received == blocks_expected, or when
  // a new TOW arrives (previous epoch is flushed with whatever was received).
  //
  // Policy in flushPending():
  //   - Position: PVTGeodetic is primary (LLH direct). If only PVTCartesian
  //     is available, LLH is derived via ecef2pos.
  //   - Velocity/Cov: receiver-provided ENU values are the truth source;
  //     ECEF is derived via current-LLH rotation.
  //   - Cartesian-direct values (pos_ecef, vel_ecef, pos_cov_ecef, vel_cov_ecef)
  //     override the rotation-derived values when present.
  static constexpr uint8_t BLK_PVT_GEO = 1 << 0;
  static constexpr uint8_t BLK_POS_GEO = 1 << 1;
  static constexpr uint8_t BLK_VEL_GEO = 1 << 2;
  static constexpr uint8_t BLK_PVT_XYZ = 1 << 3;
  static constexpr uint8_t BLK_POS_XYZ = 1 << 4;
  static constexpr uint8_t BLK_VEL_XYZ = 1 << 5;

  struct Pending {
    uint32_t tow_ms{UINT32_MAX};
    uint8_t  blocks_received{0};
    uint8_t  blocks_expected{0};
    msg::GnssSolution buf{};
    rclcpp::Time last_update{0, 0, RCL_ROS_TIME};
  };

  // Persistent DOP cache + PVT cadence tracking for the staleness gate.
  gnss_utils::DopCache last_dop_;
  uint32_t prev_pvt_tow_ms_{UINT32_MAX};
  uint32_t pvt_period_ms_{0};

  enum class SolutionSource { BINARY, NMEA };

  void initPendingExpectedMasks() {
    pending_.blocks_expected =
        (config_.enable_pvt_geodetic      ? BLK_PVT_GEO : uint8_t{0}) |
        (config_.enable_pos_cov_geodetic  ? BLK_POS_GEO : uint8_t{0}) |
        (config_.enable_vel_cov_geodetic  ? BLK_VEL_GEO : uint8_t{0}) |
        (config_.enable_pvt_cartesian     ? BLK_PVT_XYZ : uint8_t{0}) |
        (config_.enable_pos_cov_cartesian ? BLK_POS_XYZ : uint8_t{0}) |
        (config_.enable_vel_cov_cartesian ? BLK_VEL_XYZ : uint8_t{0});
  }

  bool pendingComplete() const {
    return pending_.blocks_received == pending_.blocks_expected;
  }

  template <typename ParseFn>
  void mergeBlock(uint8_t bit, ParseFn parse) {
    const uint8_t* p = sbf_body_.data();
    const size_t   len = sbf_body_.size();
    const uint32_t tow = sbf::pvt::getTowMs(p, len);
    if (tow != pending_.tow_ms && pending_.blocks_received != 0) {
      flushPending();  // TOW boundary — flush previous epoch
    }
    pending_.tow_ms = tow;
    if (!parse(p, len, pending_.buf)) return;
    pending_.blocks_received |= bit;
    pending_.last_update = now();
    if (pendingComplete()) flushPending();
  }

  void handlePvtGeodetic() {
    if (!config_.enable_pvt_geodetic) return;
    mergeBlock(BLK_PVT_GEO, &sbf::pvt::parsePVTGeodetic);
  }
  void handlePosCovGeodetic() {
    if (!config_.enable_pos_cov_geodetic) return;
    mergeBlock(BLK_POS_GEO, &sbf::pvt::parsePosCovGeodetic);
  }
  void handleVelCovGeodetic() {
    if (!config_.enable_vel_cov_geodetic) return;
    mergeBlock(BLK_VEL_GEO, &sbf::pvt::parseVelCovGeodetic);
  }
  void handlePvtCartesian() {
    if (!config_.enable_pvt_cartesian) return;
    mergeBlock(BLK_PVT_XYZ, &sbf::pvt::parsePVTCartesian);
  }
  void handlePosCovCartesian() {
    if (!config_.enable_pos_cov_cartesian) return;
    mergeBlock(BLK_POS_XYZ, &sbf::pvt::parsePosCovCartesian);
  }
  void handleVelCovCartesian() {
    if (!config_.enable_vel_cov_cartesian) return;
    mergeBlock(BLK_VEL_XYZ, &sbf::pvt::parseVelCovCartesian);
  }

  // DOP block: parsed into the persistent last_dop_ cache, NOT into pending_.
  // Does NOT participate in completion and does NOT trigger flush. At flush
  // time, applyDopWithStaleness() decides whether this cached DOP is fresh
  // enough (within 0..PVT_period after PVT) to populate the published solution.
  void handleDop() {
    if (!config_.enable_dop) return;
    const uint8_t* p   = sbf_body_.data();
    const size_t   len = sbf_body_.size();
    msg::GnssSolution scratch{};
    if (!sbf::pvt::parseDop(p, len, scratch)) return;
    last_dop_.valid  = true;
    last_dop_.week   = sbf::pvt::getWeek(p, len);
    last_dop_.tow_ms = sbf::pvt::getTowMs(p, len);
    last_dop_.gdop   = scratch.gdop;
    last_dop_.pdop   = scratch.pdop;
    last_dop_.hdop   = scratch.hdop;
    last_dop_.vdop   = scratch.vdop;
  }

  void flushPending() {
    const uint8_t expected = pending_.blocks_expected;
    const uint8_t recv     = pending_.blocks_received;
    const bool has_geo_pvt = recv & BLK_PVT_GEO;
    const bool has_xyz_pvt = recv & BLK_PVT_XYZ;
    if (!has_geo_pvt && !has_xyz_pvt) {
      pending_ = {};
      pending_.blocks_expected = expected;
      return;
    }

    // Snapshot Cartesian-direct fields from buf before LLH-driven derivation
    // overwrites them. These will be restored at the end (Cartesian wins).
    msg::GnssSolution xyz_direct = pending_.buf;
    binary_solution_ = pending_.buf;

    // Position: if Geo is unavailable but Xyz is, derive LLH from ECEF.
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

    // Derive ECEF position/velocity from LLH/ENU using current solution's LLH.
    finalizeBinarySolutionGeometry(binary_solution_);

    // Covariance bidirectional derivation (NovAtel pattern):
    //   - ENU only (Geo) → derive ECEF by rotation at current lat/lon
    //   - ECEF only (Xyz) → derive ENU by rotation
    //   - Both present → keep receiver values for each side (no rotation needed)
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

    // PVTCartesian-direct overrides for pos/vel ECEF (truth source when present).
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

    // DOP staleness gate: populate from last_dop_ iff its timestamp is within
    // [0, PVT_period] of this epoch; else NaN. Asymmetric — no future-DOP bleed.
    gnss_utils::applyDopWithStaleness(binary_solution_, last_dop_,
                                      binary_solution_.time_week,
                                      pending_.tow_ms, pvt_period_ms_);

    ever_received_binary_ = true;
    if (source_ == SolutionSource::BINARY) publishSolution(binary_solution_);
    pending_ = {};
    pending_.blocks_expected = expected;
  }

  // Warn if BINARY source was selected via YAML but no PVT block ever produced
  // a publishable solution after a generous startup window. Prevents silent
  // failure under no-fallback policy.
  void warnIfBinaryStarvation() {
    if (source_ != SolutionSource::BINARY) return;
    if (ever_received_binary_) return;
    if ((now() - start_time_).seconds() < 15.0) return;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
      "Solution source locked to BINARY (PVTGeodetic/PVTCartesian) but no PVT received yet — "
      "check receiver firmware/configuration.");
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
    std::memcpy(&esm_wnc_, sbf_body_.data() + 4, 2);

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
    imu.header.stamp    = (config_.use_gps_timestamp && esm_tow_ms_ != 0xFFFFFFFFu && esm_wnc_ != 0xFFFFu)
        ? gnss_utils::gpstToUtcRosTime(gpst2time(adjgpsweek(static_cast<int>(esm_wnc_)), esm_tow_ms_ / 1000.0))
        : now();
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
  // Decoded *Nav block handlers — removed.
  //
  // SBF Decoded nav blocks (GPSNav / GLONav / GALNav / BDSNav / QZSSNav /
  // NavICLNav) are handled by RTKLIB's input_sbf() on the parallel byte stream.
  // Running our own parsers in parallel produced two slightly-different eph_t
  // per satellite (different angular unit conventions and iode/iodc byte
  // widths), which surfaced as duplicate RINEX records when converted via
  // rosbag_to_rinex and an incorrect Galileo SVH value. The mini-framer is
  // kept solely for AttEuler + ExtSensorMeas.
  //
  // config_.enable_{gps,glo,gal,bds,qzs,navic}_nav still gates whether the
  // receiver is *told* to emit those decoded blocks (in the startup
  // configuration command path); the runtime decoding is now done by RTKLIB.
  // ============================================================================

  // ============================================================================
  // Message Handling
  // ============================================================================

  void handleNmeaSentence(const std::string& sentence) {
    if (nmea_parser_.parseSentence(sentence, nmea_solution_)) {
      if (source_ == SolutionSource::NMEA) {
        publishSolution(nmea_solution_);
      }
    } else {
      std::string head = sentence.substr(0, std::min<size_t>(sentence.size(), 6));
      RCLCPP_DEBUG(get_logger(), "NMEA parseSentence=false (head='%s')", head.c_str());
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

  void publishSolution(msg::GnssSolution& sol) {
    // BINARY path uses raw_.time (RTKLIB binary decoder timestamp); NMEA path
    // trusts time_week/time_tow already filled by NmeaParser from the sentence
    // itself.
    gtime_t t_gpst{};
    int week_for_stamp = 0;
    if (source_ == SolutionSource::BINARY) {
      const double tow = time2gpst(raw_.time, &week_for_stamp);
      if (raw_.time.time != 0 && week_for_stamp > 0) {
        sol.time_week = static_cast<uint16_t>(week_for_stamp);
        sol.time_tow  = tow;
      } else {
        week_for_stamp = 0;
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

    sol.header.stamp    = (config_.use_gps_timestamp && week_for_stamp > 0)
                          ? gnss_utils::gpstToUtcRosTime(t_gpst) : now();
    sol.header.frame_id = config_.frame_id;

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

  // ============================================================================
  // Observation Publishing
  // ============================================================================

  void publishObservations() {
    if (raw_.obs.n <= 0) return;

    // Use per-observation time (raw_.obs.data[0].time), NOT raw_.time:
    // RTKLIB returns input_*()==1 when the *next* epoch's first byte arrives,
    // at which point raw_.time has already advanced by one epoch period
    // while raw_.obs.data[*] still holds the just-completed (correct) epoch.
    int week = 0;
    const gtime_t obs_time = raw_.obs.data[0].time;
    const double tow = time2gpst(obs_time, &week);

    msg::GnssObservations msg;
    msg.header.stamp    = (config_.use_gps_timestamp && week > 0)
                          ? gnss_utils::gpstToUtcRosTime(obs_time) : now();
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
  // Ephemeris Publishing (snapshot-on-change + heartbeat via EphemerisStore)
  // ============================================================================

  // Called by handleDecodeResult(2) — ingest RTKLIB raw_.nav arrays into the
  // store, then publish a snapshot if anything changed.
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

  // Heartbeat hook — call from the poll timer; emits a snapshot if the
  // configured period has elapsed since the last publish.
  void maybePublishHeartbeat() {
    if (eph_store_.heartbeatDue(now())) publishSnapshot();
  }

  void publishSnapshot() {
    auto msg = eph_store_.buildSnapshot(now());
    eph_pub_->publish(msg);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "Published ephemeris snapshot: GNSS=%zu GLO=%zu",
      msg.gnss_ephemeris.size(), msg.glonass_ephemeris.size());
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

  SbfConfig config_;

  rclcpp::Publisher<msg::GnssObservations>::SharedPtr  obs_pub_;
  rclcpp::Publisher<msg::GnssEphemerides>::SharedPtr   eph_pub_;
  rclcpp::Publisher<msg::GnssSolution>::SharedPtr      sol_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr  imu_raw_pub_;       // ExtSensorMeas accel/gyro (uncalibrated)
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
  uint16_t esm_wnc_{0xFFFFu};
  double   esm_accel_[3]{0.0, 0.0, 0.0};
  double   esm_gyro_[3]{0.0, 0.0, 0.0};
  bool     esm_has_accel_{false};
  bool     esm_has_gyro_{false};

  // NMEA parsing state
  gnss_utils::NmeaParser nmea_parser_;
  std::string            nmea_buffer_;
  msg::GnssSolution      nmea_solution_;
  msg::GnssSolution      binary_solution_;
  SolutionSource         source_{SolutionSource::NMEA};
  rclcpp::Time           start_time_{0, 0, RCL_ROS_TIME};
  bool                   ever_received_binary_{false};
  Pending                pending_;

  // ENU local origin
  bool   has_local_origin_{false};
  double local_origin_ecef_[3]{0.0};
  double local_origin_pos_[3]{0.0};

  // Unified ephemeris store (shared between raw RTKLIB path and decoded *Nav path)
  gnss_utils::EphemerisStore eph_store_;
};

}  // namespace gnss_ros_standardization

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gnss_ros_standardization::SbfDriverNode>());
  rclcpp::shutdown();
  return 0;
}