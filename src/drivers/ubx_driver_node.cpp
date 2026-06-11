// SPDX-License-Identifier: MIT
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstring>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

#include <sensor_msgs/msg/imu.hpp>

#include "gnss_ros_standardization/ephemeris_store.hpp"
#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/ins_utils.hpp"
#include "gnss_ros_standardization/ubx_protocol.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"

using namespace std::chrono_literals;
#include "gnss_ros_standardization/decoder_common.hpp"

namespace ubx = gnss_ros_standardization::ubx;
namespace ins = gnss_ros_standardization::ins_utils;

namespace gnss_ros_standardization {

namespace {

/// Timer interval for stream polling
constexpr auto kTimerInterval = 10ms;

/// Watchdog timeout for incomplete PVT epochs (seconds)
constexpr double kPvtWatchdogTimeoutSec = 1.5;

}  // namespace

/// @brief Configuration structure for u-blox driver
struct UbxConfig {
  // Basic settings
  std::string stream_path{"serial:///dev/ttyACM0:115200"};
  int rate_hz{5};
  bool configure_on_startup{true};
  std::string dynamic_model{"portable"};
  std::string generation{"G9"};
  std::string frame_id{"gnss_link"};
  
  // Message settings
  bool enable_rawx{true};
  bool enable_sfrbx{true};
  bool enable_nav_pvt{false};
  bool enable_nav_dop{false};
  bool enable_nav_cov{false};
  bool enable_nav_posecef{false};
  bool enable_nav_velecef{false};
  bool enable_nav_hpposllh{false};
  bool enable_nav_hpposecef{false};
  bool enable_nmea_gga{false};
  bool enable_nmea_rmc{false};
  bool enable_nmea_gsa{false};
  bool enable_nmea_gst{false};
  bool nmea_high_precision{false};

  // IMU settings (ZED-F9R / IMU-enabled receivers)
  bool enable_esf_raw{true};    // ESF-RAW: uncalibrated raw IMU       → /gnss/imu/data_raw
  bool enable_esf_ins{false};   // ESF-INS: calibrated angular rate + acceleration → /gnss/imu/data
  std::string imu_topic{"/gnss/imu/data"};
  std::string imu_raw_topic{"/gnss/imu/data_raw"};


  // GNSS constellation settings
  bool enable_gps{true};
  bool enable_glonass{true};
  bool enable_galileo{true};
  bool enable_beidou{true};
  bool enable_qzss{true};
  bool enable_navic{false};
  bool enable_sbas{false};

  // Topics
  std::string observation_topic{"/gnss/observation"};
  std::string ephemeris_topic{"/gnss/ephemeris"};
  std::string solution_topic{"/gnss/nmea_solution"};

  // ENU local origin settings
  bool auto_origin{true};
  double origin_latitude{0.0};   // populated from origin[0]
  double origin_longitude{0.0};  // populated from origin[1]
  double origin_altitude{0.0};   // populated from origin[2]

  // Ephemeris snapshot behavior
  double ephemeris_snapshot_period_s{30.0};
  double ephemeris_max_age_s{0.0};  // 0 = keep all

  bool use_gps_timestamp{false};
};

/// @brief ROS 2 driver node for u-blox GNSS receivers
///
/// This node configures u-blox receivers based on YAML parameters,
/// decodes the received data, and publishes GNSS observations and ephemerides.
/// Supports both Gen 9 (legacy CFG commands) and Gen 10+ (CFG-VALSET).
class UbxDriverNode : public rclcpp::Node {
 public:
  UbxDriverNode() : Node("ubx_driver_node") {
    initializeParameters();
    initializePublishers();
    initializeDecoder();
    openStream();

    if (config_.configure_on_startup) {
      configureReceiver();
    }

    startPolling();
    RCLCPP_INFO(get_logger(), "UBX Driver initialized (rate: %d Hz)", config_.rate_hz);
  }

  ~UbxDriverNode() override {
    try {
      strclose(&stream_);
    } catch (...) {
    }
    free_raw(&raw_);
    free_rtcm(&rtcm_);
  }

 private:
  // Initialization

  void initializeParameters() {
    // Basic settings
    declare_parameter<std::string>("stream_path", config_.stream_path);
    declare_parameter<int>("rate_hz", config_.rate_hz);
    declare_parameter<bool>("configure_on_startup", config_.configure_on_startup);
    declare_parameter<std::string>("dynamic_model", config_.dynamic_model);
    declare_parameter<std::string>("generation", config_.generation);
    // frame_id: ROS TF frame name attached to the GNSS antenna phase center.
    // Default "gnss_link" — override in launch to integrate with vehicle TF tree.
    declare_parameter<std::string>("frame_id", config_.frame_id);
    
    // Message settings
    declare_parameter<bool>("messages.rawx", config_.enable_rawx);
    declare_parameter<bool>("messages.sfrbx", config_.enable_sfrbx);
    declare_parameter<bool>("messages.nav_pvt",     config_.enable_nav_pvt);
    declare_parameter<bool>("messages.nav_dop",     config_.enable_nav_dop);
    declare_parameter<bool>("messages.nav_cov",     config_.enable_nav_cov);
    declare_parameter<bool>("messages.nav_posecef", config_.enable_nav_posecef);
    declare_parameter<bool>("messages.nav_velecef", config_.enable_nav_velecef);
    declare_parameter<bool>("messages.nav_hpposllh",  config_.enable_nav_hpposllh);
    declare_parameter<bool>("messages.nav_hpposecef", config_.enable_nav_hpposecef);
    declare_parameter<bool>("messages.nmea_gga", config_.enable_nmea_gga);
    declare_parameter<bool>("messages.nmea_rmc", config_.enable_nmea_rmc);
    declare_parameter<bool>("messages.nmea_gsa", config_.enable_nmea_gsa);
    declare_parameter<bool>("messages.nmea_gst", config_.enable_nmea_gst);
    declare_parameter<bool>("messages.nmea_high_precision", config_.nmea_high_precision);
    declare_parameter<bool>("messages.esf_raw", config_.enable_esf_raw);
    declare_parameter<bool>("messages.esf_ins", config_.enable_esf_ins);
    declare_parameter<std::string>("imu_topic",     config_.imu_topic);
    declare_parameter<std::string>("imu_raw_topic", config_.imu_raw_topic);
    
    // GNSS constellation settings
    declare_parameter<bool>("gnss.gps", config_.enable_gps);
    declare_parameter<bool>("gnss.glonass", config_.enable_glonass);
    declare_parameter<bool>("gnss.galileo", config_.enable_galileo);
    declare_parameter<bool>("gnss.beidou", config_.enable_beidou);
    declare_parameter<bool>("gnss.qzss", config_.enable_qzss);
    declare_parameter<bool>("gnss.navic", config_.enable_navic);
    declare_parameter<bool>("gnss.sbas", config_.enable_sbas);

    declare_parameter<std::string>("observation_topic", config_.observation_topic);
    declare_parameter<std::string>("ephemeris_topic", config_.ephemeris_topic);
    declare_parameter<std::string>("solution_topic", config_.solution_topic);
    declare_parameter<bool>("auto_origin", config_.auto_origin);
    declare_parameter<std::vector<double>>("origin", {0.0, 0.0, 0.0});

    declare_parameter<double>("ephemeris.snapshot_period_s", config_.ephemeris_snapshot_period_s);
    declare_parameter<double>("ephemeris.max_age_s",         config_.ephemeris_max_age_s);
    declare_parameter<bool>("use_gps_timestamp",             config_.use_gps_timestamp);

    // Read parameters
    config_.stream_path = get_parameter("stream_path").as_string();
    config_.rate_hz = get_parameter("rate_hz").as_int();
    config_.configure_on_startup = get_parameter("configure_on_startup").as_bool();
    config_.dynamic_model = get_parameter("dynamic_model").as_string();
    config_.generation = get_parameter("generation").as_string();
    config_.frame_id = get_parameter("frame_id").as_string();
    
    config_.enable_rawx = get_parameter("messages.rawx").as_bool();
    config_.enable_sfrbx = get_parameter("messages.sfrbx").as_bool();
    config_.enable_nav_pvt     = get_parameter("messages.nav_pvt").as_bool();
    config_.enable_nav_dop     = get_parameter("messages.nav_dop").as_bool();
    config_.enable_nav_cov     = get_parameter("messages.nav_cov").as_bool();
    config_.enable_nav_posecef = get_parameter("messages.nav_posecef").as_bool();
    config_.enable_nav_velecef = get_parameter("messages.nav_velecef").as_bool();
    config_.enable_nav_hpposllh  = get_parameter("messages.nav_hpposllh").as_bool();
    config_.enable_nav_hpposecef = get_parameter("messages.nav_hpposecef").as_bool();
    config_.enable_nmea_gga = get_parameter("messages.nmea_gga").as_bool();
    config_.enable_nmea_rmc = get_parameter("messages.nmea_rmc").as_bool();
    config_.enable_nmea_gsa = get_parameter("messages.nmea_gsa").as_bool();
    config_.enable_nmea_gst = get_parameter("messages.nmea_gst").as_bool();
    config_.nmea_high_precision = get_parameter("messages.nmea_high_precision").as_bool();
    config_.enable_esf_raw      = get_parameter("messages.esf_raw").as_bool();
    config_.enable_esf_ins      = get_parameter("messages.esf_ins").as_bool();
    config_.imu_topic           = get_parameter("imu_topic").as_string();
    config_.imu_raw_topic       = get_parameter("imu_raw_topic").as_string();
    
    config_.enable_gps = get_parameter("gnss.gps").as_bool();
    config_.enable_glonass = get_parameter("gnss.glonass").as_bool();
    config_.enable_galileo = get_parameter("gnss.galileo").as_bool();
    config_.enable_beidou = get_parameter("gnss.beidou").as_bool();
    config_.enable_qzss = get_parameter("gnss.qzss").as_bool();
    config_.enable_navic = get_parameter("gnss.navic").as_bool();
    config_.enable_sbas = get_parameter("gnss.sbas").as_bool();

    config_.observation_topic = get_parameter("observation_topic").as_string();
    config_.ephemeris_topic = get_parameter("ephemeris_topic").as_string();
    config_.solution_topic = get_parameter("solution_topic").as_string();
    config_.auto_origin = get_parameter("auto_origin").as_bool();
    {
      const auto v = get_parameter("origin").as_double_array();
      if (v.size() == 3) {
        config_.origin_latitude  = v[0];
        config_.origin_longitude = v[1];
        config_.origin_altitude  = v[2];
      }
    }

    config_.ephemeris_snapshot_period_s = get_parameter("ephemeris.snapshot_period_s").as_double();
    config_.ephemeris_max_age_s         = get_parameter("ephemeris.max_age_s").as_double();
    config_.use_gps_timestamp           = get_parameter("use_gps_timestamp").as_bool();

    eph_store_.setSnapshotPeriod(config_.ephemeris_snapshot_period_s);
    eph_store_.setMaxAge(config_.ephemeris_max_age_s);

    // Detect USB connection from path
    is_usb_connection_ = (config_.stream_path.find("ttyACM") != std::string::npos);

    // Validate rate
    gnss_utils::validatePublishRate(config_.rate_hz, /*fallback=*/5, get_logger());
    
    // Log configuration
    RCLCPP_INFO(get_logger(), "Configuration loaded:");
    RCLCPP_INFO(get_logger(), "  Stream: %s", config_.stream_path.c_str());
    RCLCPP_INFO(get_logger(), "  Rate: %d Hz", config_.rate_hz);
    RCLCPP_INFO(get_logger(), "  Generation: %s", config_.generation.c_str());
    RCLCPP_INFO(get_logger(), "  Dynamic model: %s", config_.dynamic_model.c_str());
    RCLCPP_INFO(get_logger(), "  GNSS: GPS=%d GLO=%d GAL=%d BDS=%d QZS=%d IRN=%d SBAS=%d",
                config_.enable_gps, config_.enable_glonass, config_.enable_galileo,
                config_.enable_beidou, config_.enable_qzss, config_.enable_navic, config_.enable_sbas);

    // Lock solution source at startup. BINARY if NAV-PVT is enabled, else NMEA.
    // No mid-session switching: if binary stops, output pauses rather than falling back.
    // HPPOS messages refine the BINARY solution but cannot anchor an epoch alone.
    if ((config_.enable_nav_hpposllh || config_.enable_nav_hpposecef) &&
        !config_.enable_nav_pvt) {
      RCLCPP_WARN(get_logger(),
        "messages.nav_hpposllh/nav_hpposecef require messages.nav_pvt: true — "
        "HPPOS carries position only; no binary solution will be published without NAV-PVT.");
    }
    source_ = config_.enable_nav_pvt ? SolutionSource::BINARY : SolutionSource::NMEA;
    initPendingExpectedMask();
    start_time_ = now();
    RCLCPP_INFO(get_logger(), "Solution source locked: %s",
                source_ == SolutionSource::BINARY ? "BINARY (UBX-NAV-PVT)" : "NMEA");
  }

  void initializePublishers() {
    obs_pub_     = create_publisher<msg::GnssObservations>(config_.observation_topic, 10);
    eph_pub_     = create_publisher<msg::GnssEphemerides>(config_.ephemeris_topic, rclcpp::QoS(1).transient_local());
    sol_pub_     = create_publisher<msg::GnssSolution>(config_.solution_topic, 10);
    imu_pub_     = create_publisher<sensor_msgs::msg::Imu>(config_.imu_topic, 10);
    imu_raw_pub_ = create_publisher<sensor_msgs::msg::Imu>(config_.imu_raw_topic, 10);

    if (!config_.auto_origin) {
      if (config_.origin_latitude == 0.0 && config_.origin_longitude == 0.0) {
        RCLCPP_WARN(get_logger(),
          "auto_origin=false but origin is [0,0,0] — falling back to auto (first fix)");
      } else {
        local_origin_pos_[0] = config_.origin_latitude * (M_PI / 180.0);
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
    RCLCPP_INFO(get_logger(), "  Observation  : RAWX=%s SFRBX=%s",
      on(config_.enable_rawx), on(config_.enable_sfrbx));
    RCLCPP_INFO(get_logger(), "  PVT (binary) : NAV-PVT=%s NAV-DOP=%s NAV-COV=%s NAV-POSECEF=%s NAV-VELECEF=%s NAV-HPPOSLLH=%s NAV-HPPOSECEF=%s",
      on(config_.enable_nav_pvt), on(config_.enable_nav_dop), on(config_.enable_nav_cov),
      on(config_.enable_nav_posecef), on(config_.enable_nav_velecef),
      on(config_.enable_nav_hpposllh), on(config_.enable_nav_hpposecef));
    RCLCPP_INFO(get_logger(), "  NMEA         : GGA=%s RMC=%s GSA=%s GST=%s HiPrec=%s",
      on(config_.enable_nmea_gga), on(config_.enable_nmea_rmc),
      on(config_.enable_nmea_gsa), on(config_.enable_nmea_gst),
      on(config_.nmea_high_precision));
    RCLCPP_INFO(get_logger(), "  IMU          : ESF-RAW=%s ESF-INS=%s",
      on(config_.enable_esf_raw), on(config_.enable_esf_ins));
    RCLCPP_INFO(get_logger(), "  Constellation: GPS=%s GLO=%s GAL=%s BDS=%s QZS=%s NavIC=%s SBAS=%s",
      on(config_.enable_gps), on(config_.enable_glonass), on(config_.enable_galileo),
      on(config_.enable_beidou), on(config_.enable_qzss), on(config_.enable_navic),
      on(config_.enable_sbas));
    RCLCPP_INFO(get_logger(), "  GPS timestamp : %s", on(config_.use_gps_timestamp));
    if (!config_.auto_origin && (config_.origin_latitude != 0.0 || config_.origin_longitude != 0.0)) {
      RCLCPP_INFO(get_logger(), "  ENU origin    : fixed (lat=%.6f lon=%.6f alt=%.2f)",
        config_.origin_latitude, config_.origin_longitude, config_.origin_altitude);
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
    timer_ = create_wall_timer(kTimerInterval, std::bind(&UbxDriverNode::pollStream, this));
  }

  // Stream Management

  void openStream() {
    strinit(&stream_);

    int stream_type = STR_SERIAL;
    std::string path = config_.stream_path;

    for (const auto& st : ubx::kStreamTypes) {
      if (path.substr(0, st.prefix.size()) == st.prefix) {
        stream_type = st.type;
        path = path.substr(st.prefix.size());
        break;
      }
    }

    // MALIB serial path format: device name without /dev/ prefix
    // e.g., "ttyACM0:115200" instead of "/dev/ttyACM0:115200"
    if (stream_type == STR_SERIAL && path.rfind("/dev/", 0) == 0) {
      path.erase(0, 5);
      RCLCPP_DEBUG(get_logger(), "Adjusted serial path for MALIB: %s", path.c_str());
    }

    char path_buf[256];
    snprintf(path_buf, sizeof(path_buf), "%s", path.c_str());

    if (stropen(&stream_, stream_type, STR_MODE_RW, path_buf) == 0) {
      RCLCPP_ERROR(get_logger(), "Failed to open stream: %s", config_.stream_path.c_str());
      throw std::runtime_error("stropen failed");
    }

    RCLCPP_INFO(get_logger(), "Stream opened: %s", config_.stream_path.c_str());
  }

  // Receiver Configuration (Unified Entry Point)

  bool isGen10() const { return config_.generation == "G10"; }

  void configureReceiver() {
    RCLCPP_INFO(get_logger(), "Configuring receiver (Gen: %s)...", config_.generation.c_str());
    
    // Step 0: Ping receiver to verify connection and baud rate
    if (!pingReceiver()) {
      RCLCPP_ERROR(get_logger(), "Failed to communicate with receiver. Check connection and baud rate.");
      return; 
    }

    std::this_thread::sleep_for(500ms);

    // Step 1: GNSS signal configuration
    setupGnssSignals();

    // Step 2: Measurement rate
    setupMeasurementRate();
    
    // Step 3: Dynamic model
    setupDynamicModel();
    
    // Step 4: Output messages
    setupOutputMessages();

    is_configured_ = true;
    RCLCPP_INFO(get_logger(), "Receiver configuration complete");
  }

  bool pingReceiver() {
    // Send MON-VER (0x0A 0x04) with empty payload (Poll request)
    std::vector<uint8_t> payload = {};
    sendUbxRaw(ubx::CLASS_MON, ubx::ID_MON_VER, payload);
    
    return waitForResponse(ubx::CLASS_MON, ubx::ID_MON_VER);
  }

  bool waitForResponse(uint8_t msg_class, uint8_t msg_id) {
    auto start = std::chrono::steady_clock::now();
    uint8_t buffer[256];
    
    RCLCPP_INFO(get_logger(), "Pinging receiver (MON-VER poll)...");
    
    // Simple state machine for packet header detection
    int state = 0;
    
    // Wait up to 2 seconds for a response
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(2000)) {
        int n = strread(&stream_, buffer, sizeof(buffer));
        for (int i=0; i<n; ++i) {
             uint8_t b = buffer[i];
             
             // Keep the main decoder updated to avoid buffer overflow/loss
             if (input_ubx(&raw_, b)) {
                 // decoded something (maybe not what we want, but keep engine running)
             }
             
             // Check for target packet header: SYNC1 SYNC2 CLASS ID
             switch (state) {
                 case 0: if (b == ubx::SYNC1) state++; else state=0; break;
                 case 1: if (b == ubx::SYNC2) state++; else state=0; break;
                 case 2: if (b == msg_class)  state++; else state=0; break;
                 case 3: if (b == msg_id)     return true; else state=0; break;
             }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    RCLCPP_WARN(get_logger(), "Ping timeout (no response from receiver)");
    return false;
  }

  // Step 1: GNSS Signal Configuration

  void setupGnssSignals() {
    if (isGen10()) {
      setupGnssSignalsG10();
    } else {
      RCLCPP_INFO(get_logger(), "GNSS configuration: GPS=%d GLO=%d GAL=%d BDS=%d QZS=%d IRN=%d SBAS=%d",
                  config_.enable_gps, config_.enable_glonass, config_.enable_galileo,
                  config_.enable_beidou, config_.enable_qzss, config_.enable_navic, config_.enable_sbas);
      setupGnssSignalsLegacy();
    }
  }

  void setupGnssSignalsLegacy() {
    RCLCPP_INFO(get_logger(), "Polling CFG-GNSS from receiver...");
    
    // Send empty CFG-GNSS to poll current configuration
    std::vector<uint8_t> req;
    sendUbxRaw(ubx::CLASS_CFG, ubx::ID_CFG_GNSS, req);
    
    auto start = std::chrono::steady_clock::now();
    uint8_t buffer[256];
    std::vector<uint8_t> payload;
    int state = 0;
    int length = 0;
    int payload_pos = 0;
    
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(3000)) {
        int n = strread(&stream_, buffer, sizeof(buffer));
        for (int i = 0; i < n; ++i) {
             uint8_t b = buffer[i];
             input_ubx(&raw_, b);
             checkAckNak(b); // Feed ACK/NAK state machine just in case
             
             switch (state) {
                 case 0: if (b == ubx::SYNC1) state=1; else state=0; break;
                 case 1: if (b == ubx::SYNC2) state=2; else { state = (b==ubx::SYNC1)?1:0; } break;
                 case 2: if (b == ubx::CLASS_CFG) state=3; else state=0; break;
                 case 3: if (b == ubx::ID_CFG_GNSS) state=4; else state=0; break;
                 case 4: length = b; state=5; break;
                 case 5: length |= (b << 8); 
                         if (length > 0 && length < 256) {
                             state=6; 
                             payload.resize(length); 
                             payload_pos=0; 
                         } else {
                             state=0;
                         }
                         break;
                 case 6: 
                     payload[payload_pos++] = b;
                     if (payload_pos == length) state = 7;
                     break;
                 case 7: // ck_a (ignore)
                     state = 8;
                     break;
                 case 8: // ck_b (ignore)
                     if (applyAndSendCfgGnss(payload)) {
                         RCLCPP_INFO(get_logger(), "Successfully applied CFG-GNSS settings");
                         performGnssReset();
                         return;
                     } else {
                         RCLCPP_WARN(get_logger(), "Failed to apply CFG-GNSS settings");
                         return;
                     }
             }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    RCLCPP_WARN(get_logger(), "CFG-GNSS poll timed out");
  }

  bool applyAndSendCfgGnss(std::vector<uint8_t>& payload) {
      if (payload.size() < 4) return false;
      int numBlocks = payload[3];
      if (payload.size() < static_cast<size_t>(4 + numBlocks * 8)) return false;
      
      for (int i = 0; i < numBlocks; ++i) {
          int offset = 4 + i * 8;
          uint8_t gnssId = payload[offset];
          uint32_t flags = payload[offset+4] | (payload[offset+5]<<8) | (payload[offset+6]<<16) | (payload[offset+7]<<24);
          
          bool enable = false;
          switch (gnssId) {
             case 0: enable = config_.enable_gps; break; // GPS
             case 1: enable = config_.enable_sbas; break; // SBAS
             case 2: enable = config_.enable_galileo; break; // Galileo
             case 3: enable = config_.enable_beidou; break; // BeiDou
             case 5: enable = config_.enable_qzss; break; // QZSS
             case 6: enable = config_.enable_glonass; break; // GLONASS
             case 7: enable = config_.enable_navic; break; // NavIC
             default: continue;
          }
          
          if (enable) {
              flags |= 0x01; // set enable bit
          } else {
              flags &= ~0x01; // clear enable bit
          }
          
          payload[offset+4] = flags & 0xFF;
          payload[offset+5] = (flags >> 8) & 0xFF;
          payload[offset+6] = (flags >> 16) & 0xFF;
          payload[offset+7] = (flags >> 24) & 0xFF;
      }
      
      RCLCPP_INFO(get_logger(), "Writing modified CFG-GNSS back to receiver...");
      sendUbxRaw(ubx::CLASS_CFG, ubx::ID_CFG_GNSS, payload);
      return waitForAck(ubx::CLASS_CFG, ubx::ID_CFG_GNSS);
  }

  void setupGnssSignalsG10() {
    // Gen 10 uses CFG-VALSET with CFG-SIGNAL-* keys for per-band signal enable.
    // Note: X20P supports GPS/GAL/BDS(B1C,B2A,B3I)/QZSS/SBAS but NOT GLONASS.

    auto sendSignal = [this](uint32_t key, bool enable, const char* name) {
      uint32_t val = enable ? 1 : 0;
      if (!sendCfgValset({{key, val, 1}})) {
        // NAK is expected for unsupported signals (e.g., GLONASS on X20P)
        RCLCPP_DEBUG(get_logger(), "Signal %s not accepted by receiver (NAK)", name);
      }
    };

    // GPS: L1C/A, L2C, L5
    sendSignal(ubx::CFG_SIGNAL_GPS_ENA, config_.enable_gps, "GPS ENA");
    sendSignal(ubx::CFG_SIGNAL_GPS_L1CA_ENA, config_.enable_gps, "GPS L1C/A");
    sendSignal(ubx::CFG_SIGNAL_GPS_L2C_ENA, config_.enable_gps, "GPS L2C");
    sendSignal(ubx::CFG_SIGNAL_GPS_L5_ENA, config_.enable_gps, "GPS L5");

    // SBAS: L1C/A
    sendSignal(ubx::CFG_SIGNAL_SBAS_ENA, config_.enable_sbas, "SBAS ENA");
    sendSignal(ubx::CFG_SIGNAL_SBAS_L1CA_ENA, config_.enable_sbas, "SBAS L1C/A");

    // Galileo: E1, E5a, E5b
    sendSignal(ubx::CFG_SIGNAL_GAL_ENA, config_.enable_galileo, "Galileo ENA");
    sendSignal(ubx::CFG_SIGNAL_GAL_E1_ENA, config_.enable_galileo, "Galileo E1");
    sendSignal(ubx::CFG_SIGNAL_GAL_E5A_ENA, config_.enable_galileo, "Galileo E5a");
    sendSignal(ubx::CFG_SIGNAL_GAL_E5B_ENA, config_.enable_galileo, "Galileo E5b");

    // BeiDou: B1C, B2a (X20P); B1I, B2I (F9P/M10)
    sendSignal(ubx::CFG_SIGNAL_BDS_ENA, config_.enable_beidou, "BeiDou ENA");
    sendSignal(ubx::CFG_SIGNAL_BDS_B1C_ENA, config_.enable_beidou, "BeiDou B1C");
    sendSignal(ubx::CFG_SIGNAL_BDS_B2A_ENA, config_.enable_beidou, "BeiDou B2a");
    sendSignal(ubx::CFG_SIGNAL_BDS_B1I_ENA, config_.enable_beidou, "BeiDou B1I");
    sendSignal(ubx::CFG_SIGNAL_BDS_B2I_ENA, config_.enable_beidou, "BeiDou B2I");

    // QZSS: L1C/A, L1S, L2C, L5
    sendSignal(ubx::CFG_SIGNAL_QZSS_ENA, config_.enable_qzss, "QZSS ENA");
    sendSignal(ubx::CFG_SIGNAL_QZSS_L1CA_ENA, config_.enable_qzss, "QZSS L1C/A");
    sendSignal(ubx::CFG_SIGNAL_QZSS_L1S_ENA, config_.enable_qzss, "QZSS L1S");
    sendSignal(ubx::CFG_SIGNAL_QZSS_L2C_ENA, config_.enable_qzss, "QZSS L2C");
    sendSignal(ubx::CFG_SIGNAL_QZSS_L5_ENA, config_.enable_qzss, "QZSS L5");

    // GLONASS: L1, L2
    sendSignal(ubx::CFG_SIGNAL_GLO_ENA, config_.enable_glonass, "GLO ENA");
    sendSignal(ubx::CFG_SIGNAL_GLO_L1_ENA, config_.enable_glonass, "GLO L1");
    sendSignal(ubx::CFG_SIGNAL_GLO_L2_ENA, config_.enable_glonass, "GLO L2");
    
    RCLCPP_INFO(get_logger(), "GNSS signals configured (GPS:%d GLO:%d GAL:%d BDS:%d QZS:%d IRN:%d SBAS:%d)",
        config_.enable_gps ? 1 : 0, config_.enable_glonass ? 1 : 0,
        config_.enable_galileo ? 1 : 0,
        config_.enable_beidou ? 1 : 0, config_.enable_qzss ? 1 : 0,
        config_.enable_navic ? 1 : 0,
        config_.enable_sbas ? 1 : 0);

    // Signal configuration changes require a GNSS reset to take effect.
    performGnssReset();
  }

  void performGnssReset() {
    // UBX-CFG-RST: navBbrMask=0x0000 (Hot start), resetMode=0x02 (GNSS-only restart)
    // CFG-RST does NOT send ACK, so we send raw frame and wait.
    std::vector<uint8_t> rst_payload = {
        ubx::RST_BBR_HOTSTART & 0xFF,
        (ubx::RST_BBR_HOTSTART >> 8) & 0xFF,
        ubx::RST_MODE_GNSS_ONLY,
        0x00
    };
    RCLCPP_INFO(get_logger(), "Sending GNSS reset to apply signal config...");
    sendUbxRaw(ubx::CLASS_CFG, ubx::ID_CFG_RST, rst_payload);

    // Wait for GNSS engine to restart
    std::this_thread::sleep_for(std::chrono::milliseconds(ubx::GNSS_RESET_WAIT_MS));

    // Drain any stale data from the stream buffer after reset
    drainStreamBuffer();

    RCLCPP_INFO(get_logger(), "GNSS engine restarted");
  }

  /// Drain all pending data from stream buffer after GNSS reset.
  /// This prevents stale bytes from interfering with subsequent ACK detection.
  void drainStreamBuffer() {
    uint8_t buf[ubx::READ_BUFFER_SIZE];
    int drained = 0;
    int n;
    while ((n = strread(&stream_, buf, sizeof(buf))) > 0) {
      drained += n;
    }
    if (drained > 0) {
      RCLCPP_DEBUG(get_logger(), "Drained %d stale bytes from stream buffer", drained);
    }
  }

  // Step 2: Measurement Rate

  void setupMeasurementRate() {
    int meas_rate_ms = 1000 / config_.rate_hz;

    if (isGen10()) {
      std::vector<ubx::ValsetItem> items = {
          {ubx::CFG_RATE_MEAS, static_cast<uint32_t>(meas_rate_ms), 2},
          {ubx::CFG_RATE_NAV, 1, 2},
      };
      if (!sendCfgValsetWithRetry(items)) {
        RCLCPP_WARN(get_logger(), "Failed to set measurement rate");
      }
    } else {
      char cmd[64];
      snprintf(cmd, sizeof(cmd), "CFG-RATE %d 1 0", meas_rate_ms);
      if (!sendLegacyCommand(cmd, ubx::CLASS_CFG, ubx::ID_CFG_RATE)) {
        RCLCPP_WARN(get_logger(), "Failed to set measurement rate");
      }
    }
  }

  // Step 3: Dynamic Model

  void setupDynamicModel() {
    ubx::DynamicModel model = ubx::parseDynamicModel(config_.dynamic_model);

    if (isGen10()) {
      std::vector<ubx::ValsetItem> items = {
          {ubx::CFG_NAVSPG_DYNMODEL, static_cast<uint32_t>(model), 1},
      };
      if (!sendCfgValsetWithRetry(items)) {
        RCLCPP_WARN(get_logger(), "Failed to set dynamic model");
      } else {
        RCLCPP_INFO(get_logger(), "Dynamic model configured");
      }
    } else {
      char cmd[256];
      snprintf(cmd, sizeof(cmd),
               "CFG-NAV5 1 %d 3 0 10000 5 0 250 500 100 300 0 60 3 0 0 0 0 0 0 0 0 0 0",
               static_cast<int>(model));
      if (!sendLegacyCommand(cmd, ubx::CLASS_CFG, ubx::ID_CFG_NAV5)) {
        RCLCPP_WARN(get_logger(), "Failed to set dynamic model (receiver may not support CFG-NAV5)");
      }
    }
  }

  // Step 4: Output Messages

  void setupOutputMessages() {
    if (isGen10()) {
      setupOutputMessagesG10();
    } else {
      setupOutputMessagesLegacy();
    }
  }

  void setupOutputMessagesG10() {
    // Determine port priority based on connection type
    std::vector<int> target_ports;
    if (is_usb_connection_) {
      target_ports = {ubx::PORT_USB, ubx::PORT_UART1, ubx::PORT_UART2};
    } else {
      target_ports = {ubx::PORT_UART1, ubx::PORT_UART2, ubx::PORT_USB};
    }

    for (int port : target_ports) {
      // 1. Configure standard UBX output messages
      std::vector<ubx::ValsetItem> items_ubx = {
          {ubx::CFG_MSGOUT_UBX_RXM_RAWX_I2C  + port, config_.enable_rawx    ? 1u : 0u, 1},
          {ubx::CFG_MSGOUT_UBX_RXM_SFRBX_I2C + port, config_.enable_sfrbx   ? 1u : 0u, 1},
          {ubx::CFG_MSGOUT_UBX_NAV_PVT_I2C     + port, config_.enable_nav_pvt     ? 1u : 0u, 1},
          {ubx::CFG_MSGOUT_UBX_NAV_DOP_I2C     + port, config_.enable_nav_dop     ? 1u : 0u, 1},
          {ubx::CFG_MSGOUT_UBX_NAV_COV_I2C     + port, config_.enable_nav_cov     ? 1u : 0u, 1},
          {ubx::CFG_MSGOUT_UBX_NAV_POSECEF_I2C + port, config_.enable_nav_posecef ? 1u : 0u, 1},
          {ubx::CFG_MSGOUT_UBX_NAV_VELECEF_I2C + port, config_.enable_nav_velecef ? 1u : 0u, 1},
          {ubx::CFG_MSGOUT_UBX_NAV_HPPOSLLH_I2C  + port, config_.enable_nav_hpposllh  ? 1u : 0u, 1},
          {ubx::CFG_MSGOUT_UBX_NAV_HPPOSECEF_I2C + port, config_.enable_nav_hpposecef ? 1u : 0u, 1},
          {ubx::CFG_MSGOUT_UBX_ESF_RAW_I2C   + port, config_.enable_esf_raw ? 1u : 0u, 1},
          {ubx::CFG_MSGOUT_UBX_ESF_INS_I2C   + port, config_.enable_esf_ins ? 1u : 0u, 1},
      };

      if (!sendCfgValset(items_ubx)) {
        RCLCPP_DEBUG(get_logger(), "UBX output config failed on port %d", port);
      } else {
        RCLCPP_INFO(get_logger(), "UBX output configured on port %d", port);
      }

      // 2. Configure NMEA outputs separately to avoid potential receiver firmware bugs
      std::vector<ubx::ValsetItem> items_nmea = {
          {ubx::CFG_MSGOUT_NMEA_ID_GGA_I2C + port, config_.enable_nmea_gga ? 1u : 0u, 1},
          {ubx::CFG_MSGOUT_NMEA_ID_RMC_I2C + port, config_.enable_nmea_rmc ? 1u : 0u, 1},
          {ubx::CFG_MSGOUT_NMEA_ID_GSA_I2C + port, config_.enable_nmea_gsa ? 1u : 0u, 1},
          {ubx::CFG_MSGOUT_NMEA_ID_GST_I2C + port, config_.enable_nmea_gst ? 1u : 0u, 1},
      };

      // Disable unused NMEA to save bandwidth
      if (!config_.enable_nmea_gga && !config_.enable_nmea_rmc &&
          !config_.enable_nmea_gsa && !config_.enable_nmea_gst) {
        items_nmea.push_back({ubx::CFG_MSGOUT_NMEA_ID_GLL_I2C + port, 0u, 1});
        items_nmea.push_back({ubx::CFG_MSGOUT_NMEA_ID_VTG_I2C + port, 0u, 1});
        items_nmea.push_back({ubx::CFG_MSGOUT_NMEA_ID_ZDA_I2C + port, 0u, 1});
        items_nmea.push_back({ubx::CFG_MSGOUT_NMEA_ID_GSV_I2C + port, 0u, 1});
      }

      if (!sendCfgValset(items_nmea)) {
        RCLCPP_DEBUG(get_logger(), "NMEA output config failed on port %d", port);
      } else {
        RCLCPP_INFO(get_logger(), "NMEA output configured on port %d", port);
      }
    }

    // NMEA High Precision Mode (applies globally, not per-port)
    if (config_.nmea_high_precision) {
      if (!sendCfgValset({{ubx::CFG_NMEA_HIGHPREC, 1u, 1}})) {
        RCLCPP_WARN(get_logger(), "Failed to enable NMEA High Precision mode");
      } else {
        RCLCPP_INFO(get_logger(), "NMEA High Precision mode enabled");
      }
    }
  }

  void setupOutputMessagesLegacy() {
    auto sendMsg = [this](uint8_t cls, uint8_t id, bool enable, const char* name) {
      char cmd[64];
      int rate = enable ? 1 : 0;
      snprintf(cmd, sizeof(cmd), "CFG-MSG %d %d %d %d %d %d 0 0", cls, id, rate, rate, rate, rate);
      if (!sendLegacyCommand(cmd, ubx::CLASS_CFG, ubx::ID_CFG_MSG)) {
        RCLCPP_WARN(get_logger(), "Failed to configure %s", name);
      }
    };

    auto sendNmea = [this](uint8_t id, bool enable, const char* name) {
      char cmd[64];
      int rate = enable ? 1 : 0;
      snprintf(cmd, sizeof(cmd), "CFG-MSG 240 %d %d %d %d %d 0 0", id, rate, rate, rate, rate);
      if (!sendLegacyCommand(cmd, ubx::CLASS_CFG, ubx::ID_CFG_MSG)) {
        RCLCPP_DEBUG(get_logger(), "Failed to configure NMEA %s", name);
      }
    };

    sendMsg(ubx::CLASS_RXM, ubx::ID_RXM_RAWX,  config_.enable_rawx,    "RXM-RAWX");
    sendMsg(ubx::CLASS_RXM, ubx::ID_RXM_SFRBX, config_.enable_sfrbx,   "RXM-SFRBX");
    sendMsg(ubx::CLASS_NAV, ubx::ID_NAV_PVT,     config_.enable_nav_pvt,     "NAV-PVT");
    sendMsg(ubx::CLASS_NAV, ubx::ID_NAV_DOP,     config_.enable_nav_dop,     "NAV-DOP");
    sendMsg(ubx::CLASS_NAV, ubx::ID_NAV_COV,     config_.enable_nav_cov,     "NAV-COV");
    sendMsg(ubx::CLASS_NAV, ubx::ID_NAV_POSECEF, config_.enable_nav_posecef, "NAV-POSECEF");
    sendMsg(ubx::CLASS_NAV, ubx::ID_NAV_VELECEF, config_.enable_nav_velecef, "NAV-VELECEF");
    sendMsg(ubx::CLASS_NAV, ubx::ID_NAV_HPPOSLLH,  config_.enable_nav_hpposllh,  "NAV-HPPOSLLH");
    sendMsg(ubx::CLASS_NAV, ubx::ID_NAV_HPPOSECEF, config_.enable_nav_hpposecef, "NAV-HPPOSECEF");
    sendMsg(ubx::CLASS_ESF, ubx::ID_ESF_RAW,   config_.enable_esf_raw, "ESF-RAW");
    sendMsg(ubx::CLASS_ESF, ubx::ID_ESF_INS,   config_.enable_esf_ins, "ESF-INS");

    sendNmea(ubx::NMEA_GGA, config_.enable_nmea_gga, "GGA");
    sendNmea(ubx::NMEA_RMC, config_.enable_nmea_rmc, "RMC");
    sendNmea(ubx::NMEA_GSA, config_.enable_nmea_gsa, "GSA");
    sendNmea(ubx::NMEA_GST, config_.enable_nmea_gst, "GST");

    // Disable unused NMEA to save bandwidth
    if (!config_.enable_nmea_gga && !config_.enable_nmea_rmc &&
        !config_.enable_nmea_gsa && !config_.enable_nmea_gst) {
      sendNmea(ubx::NMEA_GLL, false, "GLL");
      sendNmea(ubx::NMEA_VTG, false, "VTG");
      sendNmea(ubx::NMEA_ZDA, false, "ZDA");
    }

    // NMEA High Precision Mode (applies globally, not per-port)
    if (config_.nmea_high_precision) {
      if (!sendCfgValset({{ubx::CFG_NMEA_HIGHPREC, 1u, 1}})) {
        RCLCPP_WARN(get_logger(), "Failed to enable NMEA High Precision mode");
      } else {
        RCLCPP_INFO(get_logger(), "NMEA High Precision mode enabled");
      }
    }
  }

  // UBX Communication (Gen 10: CFG-VALSET)

  bool sendCfgValset(const std::vector<ubx::ValsetItem>& items) {
    if (items.empty()) return true;
    
    std::vector<uint8_t> payload;
    payload.reserve(4 + items.size() * 8);
    
    payload.push_back(0x00); // Version
    payload.push_back(0x01); // Layer (RAM)
    payload.push_back(0x00); // Reserved
    payload.push_back(0x00);
    
    for (const auto& item : items) {
       // Little-endian Key
       payload.push_back((item.key >>  0) & 0xFF);
       payload.push_back((item.key >>  8) & 0xFF);
       payload.push_back((item.key >> 16) & 0xFF);
       payload.push_back((item.key >> 24) & 0xFF);
       
       // Little-endian Value
       for (int i = 0; i < item.size; ++i) {
           payload.push_back((item.value >> (i * 8)) & 0xFF);
       }
    }
    
    return sendUbxPacket(ubx::CLASS_CFG, ubx::ID_CFG_VALSET, payload);
  }

  /// Send CFG-VALSET with retry logic. Useful after GNSS reset when receiver
  /// may not respond to the first command immediately.
  bool sendCfgValsetWithRetry(const std::vector<ubx::ValsetItem>& items) {
    for (int attempt = 0; attempt <= ubx::MAX_RETRIES; ++attempt) {
      if (sendCfgValset(items)) return true;
      if (attempt < ubx::MAX_RETRIES) {
        RCLCPP_DEBUG(get_logger(), "Retrying CFG-VALSET (attempt %d/%d)...", 
                     attempt + 1, ubx::MAX_RETRIES);
        std::this_thread::sleep_for(std::chrono::milliseconds(ubx::RETRY_DELAY_MS));
        drainStreamBuffer();
      }
    }
    return false;
  }

  // UBX Communication (Low-level)

  /// Send a UBX frame and wait for ACK.
  bool sendUbxPacket(uint8_t msg_class, uint8_t msg_id, const std::vector<uint8_t>& payload) {
    auto frame = ubx::buildUbxFrame(msg_class, msg_id, payload);
    RCLCPP_DEBUG(get_logger(), "Sending UBX 0x%02X 0x%02X (len=%zu)", 
                msg_class, msg_id, payload.size());
    
    int written = strwrite(&stream_, frame.data(), frame.size());
    if (written < 0) {
      RCLCPP_WARN(get_logger(), "strwrite error: %d", written);
    }
    
    return waitForAck(msg_class, msg_id);
  }

  /// Send a UBX frame without waiting for ACK (for commands like CFG-RST).
  void sendUbxRaw(uint8_t msg_class, uint8_t msg_id, const std::vector<uint8_t>& payload) {
    auto frame = ubx::buildUbxFrame(msg_class, msg_id, payload);
    RCLCPP_DEBUG(get_logger(), "Sending UBX 0x%02X 0x%02X (len=%zu, no ACK)", 
                msg_class, msg_id, payload.size());
    strwrite(&stream_, frame.data(), frame.size());
  }

  /// Send a legacy UBX command string (Gen 9). Uses MALIB's gen_ubx().
  bool sendLegacyCommand(const char* cmd, uint8_t expected_class, uint8_t expected_id) {
    uint8_t buff[256];
    int len = gen_ubx(cmd, buff);
    if (len <= 0) {
      RCLCPP_ERROR(get_logger(), "Failed to generate UBX command: %s", cmd);
      return false;
    }

    RCLCPP_DEBUG(get_logger(), "Sending legacy: %s", cmd);

    int written = strwrite(&stream_, buff, len);
    // MALIB's writeserial on Linux has a bug where write() return is not assigned,
    // causing strwrite to always return 0. We treat 0 as success.
    if (written < 0) {
      RCLCPP_WARN(get_logger(), "strwrite error: %d", written);
    } else if (written != len && written > 0) {
      RCLCPP_WARN(get_logger(), "Partial write: %d/%d bytes", written, len);
    }

    return waitForAck(expected_class, expected_id);
  }

  // ACK/NAK Detection

  bool waitForAck(uint8_t msg_class, uint8_t msg_id) {
    auto start = std::chrono::steady_clock::now();
    uint8_t buffer[256];
    const auto timeout = std::chrono::milliseconds(ubx::ACK_TIMEOUT_MS);

    // Reset ACK state
    pending_ack_class_ = msg_class;
    pending_ack_id_ = msg_id;
    ack_received_ = false;
    nak_received_ = false;

    while (std::chrono::steady_clock::now() - start < timeout) {
      int n = strread(&stream_, buffer, sizeof(buffer));
      for (int i = 0; i < n; ++i) {
        int result = input_ubx(&raw_, buffer[i]);

        checkAckNak(buffer[i]);

        if (ack_received_) {
          RCLCPP_DEBUG(get_logger(), "ACK received for CFG 0x%02X", msg_id);
          return true;
        }
        if (nak_received_) {
          RCLCPP_DEBUG(get_logger(), "NAK received for CFG 0x%02X", msg_id);
          return false;
        }

        if (result > 0) {
          handleDecodeResult(result);
        }
      }
      std::this_thread::sleep_for(10ms);
    }

    RCLCPP_WARN(get_logger(), "ACK timeout for CFG 0x%02X", msg_id);
    return false;
  }

  void checkAckNak(uint8_t byte) {
    // Simple state machine for ACK/NAK detection in UBX byte stream
    switch (ack_state_) {
      case 0:  // Wait for sync byte 1
        if (byte == ubx::SYNC1) { ack_state_ = 1; ack_pos_ = 0; }
        break;
      case 1:  // Wait for sync byte 2
        if (byte == ubx::SYNC2) ack_state_ = 2;
        else ack_state_ = 0;
        break;
      case 2:  // Class
        ack_buf_[ack_pos_++] = byte;
        if (byte == ubx::CLASS_ACK) ack_state_ = 3;
        else ack_state_ = 0;
        break;
      case 3:  // ID (ACK=0x01, NAK=0x00)
        ack_buf_[ack_pos_++] = byte;
        if (byte == ubx::ID_ACK_ACK || byte == ubx::ID_ACK_NAK) ack_state_ = 4;
        else ack_state_ = 0;
        break;
      case 4:  // Length LSB
        ack_buf_[ack_pos_++] = byte;
        ack_state_ = 5;
        break;
      case 5:  // Length MSB
        ack_buf_[ack_pos_++] = byte;
        ack_state_ = 6;
        break;
      case 6:  // Payload: clsID
        ack_buf_[ack_pos_++] = byte;
        ack_state_ = 7;
        break;
      case 7:  // Payload: msgID
        ack_buf_[ack_pos_++] = byte;
        if (ack_buf_[4] == ubx::CLASS_CFG) {
          if (ack_buf_[1] == ubx::ID_ACK_ACK) {
            ack_received_ = true;
          } else if (ack_buf_[1] == ubx::ID_ACK_NAK) {
            nak_received_ = true;
          }
        }
        ack_state_ = 0;
        break;
      default:
        ack_state_ = 0;
    }
  }

  // UBX Mini-Framer (parallel to RTKLIB, for ESF-INS and NAV-ATT)

  void parseUbxByte(uint8_t byte) {
    switch (ubx_frm_state_) {
      case 0: if (byte == ubx::SYNC1) ubx_frm_state_ = 1; break;
      case 1: ubx_frm_state_ = (byte == ubx::SYNC2) ? 2 : (byte == ubx::SYNC1 ? 1 : 0); break;
      case 2: ubx_frm_cls_   = byte; ubx_frm_state_ = 3; break;
      case 3: ubx_frm_id_    = byte; ubx_frm_state_ = 4; break;
      case 4: ubx_frm_len_   = byte; ubx_frm_state_ = 5; break;
      case 5:
        ubx_frm_len_ |= static_cast<uint16_t>(byte << 8);
        ubx_frm_payload_.clear();
        ubx_frm_pos_   = 0;
        ubx_frm_state_ = (ubx_frm_len_ == 0) ? 7 : 6;
        break;
      case 6:
        ubx_frm_payload_.push_back(byte);
        if (++ubx_frm_pos_ >= ubx_frm_len_) ubx_frm_state_ = 7;
        break;
      case 7: ubx_frm_state_ = 8; break;  // CK_A skip
      case 8: handleUbxFrame(); ubx_frm_state_ = 0; break;  // CK_B, frame done
      default: ubx_frm_state_ = 0;
    }
  }

  void handleUbxFrame() {
    if (ubx_frm_cls_ == ubx::CLASS_ESF && ubx_frm_id_ == ubx::ID_ESF_RAW) handleEsfRaw();
    if (ubx_frm_cls_ == ubx::CLASS_ESF && ubx_frm_id_ == ubx::ID_ESF_INS) handleEsfIns();
    if (ubx_frm_cls_ == ubx::CLASS_NAV && ubx_frm_id_ == ubx::ID_NAV_PVT)     handleNavPvt();
    if (ubx_frm_cls_ == ubx::CLASS_NAV && ubx_frm_id_ == ubx::ID_NAV_DOP)     handleNavDop();
    if (ubx_frm_cls_ == ubx::CLASS_NAV && ubx_frm_id_ == ubx::ID_NAV_COV)     handleNavCov();
    if (ubx_frm_cls_ == ubx::CLASS_NAV && ubx_frm_id_ == ubx::ID_NAV_POSECEF) handleNavPosEcef();
    if (ubx_frm_cls_ == ubx::CLASS_NAV && ubx_frm_id_ == ubx::ID_NAV_VELECEF) handleNavVelEcef();
    if (ubx_frm_cls_ == ubx::CLASS_NAV && ubx_frm_id_ == ubx::ID_NAV_HPPOSECEF) handleNavHpPosEcef();
    if (ubx_frm_cls_ == ubx::CLASS_NAV && ubx_frm_id_ == ubx::ID_NAV_HPPOSLLH)  handleNavHpPosLlh();
  }

  // Sign-extend the lower 24 bits of a uint32 into an int32.
  static int32_t signExtend24(uint32_t v) {
    return (v & 0x00800000u) ? static_cast<int32_t>(v | 0xFF000000u)
                             : static_cast<int32_t>(v & 0x00FFFFFFu);
  }

  // UBX-ESF-RAW: 4-byte reserved header + N × { data(u4) + sTtag(u4) }.
  // data low 24 bits = signed measurement, high 8 bits = data type.
  // Accumulate accel x/y/z + gyro x/y/z found in this message; publish if any axis was set.
  void handleEsfRaw() {
    constexpr int kHeaderLen = 4;
    constexpr int kBlockLen  = 8;
    const int n = (static_cast<int>(ubx_frm_payload_.size()) - kHeaderLen) / kBlockLen;
    if (n <= 0) return;

    double accel[3] = {0, 0, 0};
    double gyro[3]  = {0, 0, 0};
    bool   has_accel[3] = {false, false, false};
    bool   has_gyro[3]  = {false, false, false};
    const double deg2rad = M_PI / 180.0;

    for (int i = 0; i < n; ++i) {
      const uint8_t* p = ubx_frm_payload_.data() + kHeaderLen + i * kBlockLen;
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
        default: break;  // ignore other sensor types (temp, wheel speed, etc.)
      }
    }

    if (!(has_accel[0] || has_accel[1] || has_accel[2] ||
          has_gyro[0]  || has_gyro[1]  || has_gyro[2])) return;

    sensor_msgs::msg::Imu imu;
    imu.header.stamp    = now();
    imu.header.frame_id = config_.frame_id;

    // orientation: not provided by ESF-RAW — leave identity, mark unknown
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
    if (static_cast<int>(ubx_frm_payload_.size()) < ubx::esf_ins::MIN_LEN) return;

    uint32_t bitfield0 = 0;
    std::memcpy(&bitfield0, ubx_frm_payload_.data() + ubx::esf_ins::OFFSET_BITFIELD, 4);
    const bool ang_valid = (bitfield0 & (0x7u << 8))  == (0x7u << 8);
    const bool acc_valid = (bitfield0 & (0x7u << 11)) == (0x7u << 11);

    int32_t xAng = 0, yAng = 0, zAng = 0, xAcc = 0, yAcc = 0, zAcc = 0;
    std::memcpy(&xAng, ubx_frm_payload_.data() + ubx::esf_ins::OFFSET_XANGRATE, 4);
    std::memcpy(&yAng, ubx_frm_payload_.data() + ubx::esf_ins::OFFSET_YANGRATE, 4);
    std::memcpy(&zAng, ubx_frm_payload_.data() + ubx::esf_ins::OFFSET_ZANGRATE, 4);
    std::memcpy(&xAcc, ubx_frm_payload_.data() + ubx::esf_ins::OFFSET_XACCEL,   4);
    std::memcpy(&yAcc, ubx_frm_payload_.data() + ubx::esf_ins::OFFSET_YACCEL,   4);
    std::memcpy(&zAcc, ubx_frm_payload_.data() + ubx::esf_ins::OFFSET_ZACCEL,   4);

    const double deg2rad = M_PI / 180.0;

    sensor_msgs::msg::Imu imu;
    imu.header.stamp    = now();
    imu.header.frame_id = config_.frame_id;

    // orientation: not provided by ESF-INS — leave identity, mark unknown
    imu.orientation_covariance[0] = -1.0;

    imu.angular_velocity.x = xAng * 1e-3 * deg2rad;
    imu.angular_velocity.y = yAng * 1e-3 * deg2rad;
    imu.angular_velocity.z = zAng * 1e-3 * deg2rad;
    auto ang_cov = ang_valid ? ins::makeDiagCovariance(1e-4, 1e-4, 1e-4) : ins::makeUnknownCovariance();
    std::copy(ang_cov.begin(), ang_cov.end(), imu.angular_velocity_covariance.begin());

    imu.linear_acceleration.x = xAcc * 1e-2;
    imu.linear_acceleration.y = yAcc * 1e-2;
    imu.linear_acceleration.z = zAcc * 1e-2;
    auto acc_cov = acc_valid ? ins::makeDiagCovariance(1e-3, 1e-3, 1e-3) : ins::makeUnknownCovariance();
    std::copy(acc_cov.begin(), acc_cov.end(), imu.linear_acceleration_covariance.begin());

    imu_pub_->publish(imu);
  }

  // Data Processing

  void pollStream() {
    uint8_t buffer[ubx::READ_BUFFER_SIZE];
    
    // Read loop to drain all available data from the stream
    while (rclcpp::ok()) {
      const int bytes_read = strread(&stream_, buffer, sizeof(buffer));
      
      if (bytes_read <= 0) {
        break; // No more data
      }

      for (int i = 0; i < bytes_read; ++i) {
        uint8_t byte = buffer[i];

        const int result = input_ubx(&raw_, byte);
        if (result > 0) {
          handleDecodeResult(result);
        }

        // Parallel UBX mini-framer for ESF-INS and NAV-ATT
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
          } else if (nmea_buffer_.size() > ubx::NMEA_MAX_LINE_LEN) { // Safeguard against missing newlines
            nmea_buffer_.clear();
          }
        }
      }
      
      // If we read less than the full buffer, the stream is likely empty now
      if (bytes_read < static_cast<int>(sizeof(buffer))) {
        break;
      }
    }

    maybePublishHeartbeat();
    warnIfBinaryStarvation();
    maybeWatchdogFlushPendingPvt();
  }

  // Solution source policy:
  //   BINARY (NAV-PVT)  : if config_.enable_nav_pvt → use NAV-PVT only, drop NMEA
  //   NMEA             : otherwise → use NMEA-parsed solution
  // Source is locked at startup; no mid-session switching.
  enum class SolutionSource { BINARY, NMEA };

  void handleNmeaSentence(const std::string& sentence) {
    if (nmea_parser_.parseSentence(sentence, nmea_solution_)) {
      if (source_ == SolutionSource::NMEA) {
        publishSolution(nmea_solution_);
      }
    }
  }

  // PVT TOW Aggregation (mirrors SBF driver pattern; see ubx_decoder_node.cpp
  // for design notes). All NAV-* payloads carry iTOW at offset 0.
  // NAV-DOP is tracked separately in a persistent cache (last_dop_) so it can
  // survive Pending resets and be staleness-gated against the PVT cadence.
  // Only NAV-COV/POSECEF/VELECEF contribute to completion.
  static constexpr uint8_t COV_BIT_COV       = 0x1;
  static constexpr uint8_t COV_BIT_POSECEF   = 0x2;
  static constexpr uint8_t COV_BIT_VELECEF   = 0x4;
  static constexpr uint8_t COV_BIT_HPPOSLLH  = 0x8;
  static constexpr uint8_t COV_BIT_HPPOSECEF = 0x10;

  struct PendingPvt {
    uint32_t tow_ms{UINT32_MAX};
    bool has_pvt{false};
    uint8_t cov_received{0};
    uint8_t cov_expected{0};   // derived from YAML config (fixed at startup)
    msg::GnssSolution buf{};
    // High-precision scratch (NAV-HPPOSLLH / NAV-HPPOSECEF). Kept outside buf
    // and applied at flush time so the result does not depend on the in-epoch
    // arrival order vs NAV-PVT (which writes lat/lon/alt unconditionally).
    ubx::nav_hpposllh::Result hp_llh{};
    double hp_ecef[3]{0.0, 0.0, 0.0};
    rclcpp::Time last_update{0, 0, RCL_ROS_TIME};
  };

  gnss_utils::DopCache last_dop_;
  uint32_t prev_pvt_tow_ms_{UINT32_MAX};
  uint32_t pvt_period_ms_{0};

  // Last published (week, tow) for orphan guard. NAV-* bodies carry only TOW;
  // we compare against the week stored at the previous publishSolution call.
  uint32_t last_flushed_week_{0};
  uint32_t last_flushed_tow_ms_{UINT32_MAX};

  void initPendingExpectedMask() {
    pending_.cov_expected =
        (config_.enable_nav_cov       ? COV_BIT_COV       : uint8_t{0}) |
        (config_.enable_nav_posecef   ? COV_BIT_POSECEF   : uint8_t{0}) |
        (config_.enable_nav_velecef   ? COV_BIT_VELECEF   : uint8_t{0}) |
        (config_.enable_nav_hpposllh  ? COV_BIT_HPPOSLLH  : uint8_t{0}) |
        (config_.enable_nav_hpposecef ? COV_BIT_HPPOSECEF : uint8_t{0});
  }

  // offset: iTOW position in the payload — 0 for most NAV-* messages,
  // ubx::nav_hpposllh/nav_hpposecef::OFFSET_ITOW (4) for the HP messages.
  // Reading the wrong offset breaks TOW epoch grouping and stalls publishing
  // until the watchdog fires, so HP handlers must pass their OFFSET_ITOW.
  static uint32_t readItowMs(const uint8_t* p, size_t len, size_t offset = 0) {
    if (len < offset + 4) return UINT32_MAX;
    uint32_t v = 0;
    std::memcpy(&v, p + offset, 4);
    return v;
  }

  bool pendingComplete() const {
    return pending_.has_pvt &&
           pending_.cov_received == pending_.cov_expected;
  }

  // Orphan guard: TOW matches the most-recently-flushed epoch. NAV-* bodies
  // don't carry a week field, so fall back to TOW-only when the current pending
  // has no PVT yet (buf.time_week is 0). Returns true → caller should skip.
  bool isLateOrphan(uint32_t tow) const {
    const uint32_t week = pending_.buf.time_week;
    return last_flushed_tow_ms_ != UINT32_MAX &&
           tow == last_flushed_tow_ms_ &&
           (week == 0 || week == last_flushed_week_);
  }

  void handleNavPvt() {
    if (!config_.enable_nav_pvt) return;
    const uint32_t tow = readItowMs(ubx_frm_payload_.data(), ubx_frm_payload_.size());
    if (isLateOrphan(tow)) return;
    if (tow != pending_.tow_ms && (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = tow;
    if (!ubx::pvt::parseNavPvt(ubx_frm_payload_.data(), ubx_frm_payload_.size(),
                               pending_.buf)) {
      return;
    }
    pending_.has_pvt = true;
    pending_.last_update = now();
    ever_received_binary_ = true;
    if (pendingComplete()) flushPending();
  }

  // NAV-DOP: parsed into the persistent last_dop_ cache, NOT into pending_.
  // Does NOT participate in completion and does NOT trigger flush. At flush
  // time, applyDopWithStaleness() decides whether this cached DOP is fresh
  // enough (within 0..PVT_period after PVT) to populate the published solution.
  // NAV-DOP carries no GPS week → last_dop_.week = 0 (helper skips week check).
  void handleNavDop() {
    if (!config_.enable_nav_dop) return;
    const uint32_t tow = readItowMs(ubx_frm_payload_.data(), ubx_frm_payload_.size());
    msg::GnssSolution scratch{};
    if (!ubx::nav_dop::parseNavDop(ubx_frm_payload_.data(), ubx_frm_payload_.size(),
                                   scratch)) {
      return;
    }
    last_dop_.valid  = true;
    last_dop_.week   = 0;
    last_dop_.tow_ms = tow;
    last_dop_.gdop   = scratch.gdop;
    last_dop_.pdop   = scratch.pdop;
    last_dop_.hdop   = scratch.hdop;
    last_dop_.vdop   = scratch.vdop;
  }

  void handleNavCov() {
    if (!config_.enable_nav_cov) return;
    const uint32_t tow = readItowMs(ubx_frm_payload_.data(), ubx_frm_payload_.size());
    if (isLateOrphan(tow)) return;
    if (tow != pending_.tow_ms && (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = tow;
    if (!ubx::nav_cov::parseNavCov(ubx_frm_payload_.data(), ubx_frm_payload_.size(),
                                   pending_.buf)) {
      return;
    }
    pending_.cov_received |= COV_BIT_COV;
    pending_.last_update = now();
    if (pendingComplete()) flushPending();
  }

  void handleNavPosEcef() {
    if (!config_.enable_nav_posecef) return;
    const uint32_t tow = readItowMs(ubx_frm_payload_.data(), ubx_frm_payload_.size());
    if (isLateOrphan(tow)) return;
    if (tow != pending_.tow_ms && (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = tow;
    if (!ubx::nav_posecef::parseNavPosEcef(ubx_frm_payload_.data(),
                                           ubx_frm_payload_.size(),
                                           pending_.buf)) {
      return;
    }
    pending_.cov_received  |= COV_BIT_POSECEF;
    pending_.last_update = now();
    if (pendingComplete()) flushPending();
  }

  void handleNavVelEcef() {
    if (!config_.enable_nav_velecef) return;
    const uint32_t tow = readItowMs(ubx_frm_payload_.data(), ubx_frm_payload_.size());
    if (isLateOrphan(tow)) return;
    if (tow != pending_.tow_ms && (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = tow;
    if (!ubx::nav_velecef::parseNavVelEcef(ubx_frm_payload_.data(),
                                           ubx_frm_payload_.size(),
                                           pending_.buf)) {
      return;
    }
    pending_.cov_received  |= COV_BIT_VELECEF;
    pending_.last_update = now();
    if (pendingComplete()) flushPending();
  }

  void handleNavHpPosEcef() {
    if (!config_.enable_nav_hpposecef) return;
    const uint32_t tow = readItowMs(ubx_frm_payload_.data(), ubx_frm_payload_.size(),
                                    ubx::nav_hpposecef::OFFSET_ITOW);
    if (isLateOrphan(tow)) return;
    if (tow != pending_.tow_ms && (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = tow;
    if (!ubx::nav_hpposecef::parseNavHpPosEcef(ubx_frm_payload_.data(),
                                               ubx_frm_payload_.size(),
                                               pending_.hp_ecef)) {
      return;
    }
    pending_.cov_received  |= COV_BIT_HPPOSECEF;
    pending_.last_update = now();
    if (pendingComplete()) flushPending();
  }

  void handleNavHpPosLlh() {
    if (!config_.enable_nav_hpposllh) return;
    const uint32_t tow = readItowMs(ubx_frm_payload_.data(), ubx_frm_payload_.size(),
                                    ubx::nav_hpposllh::OFFSET_ITOW);
    if (isLateOrphan(tow)) return;
    if (tow != pending_.tow_ms && (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = tow;
    if (!ubx::nav_hpposllh::parseNavHpPosLlh(ubx_frm_payload_.data(),
                                             ubx_frm_payload_.size(),
                                             pending_.hp_llh)) {
      return;
    }
    pending_.cov_received  |= COV_BIT_HPPOSLLH;
    pending_.last_update = now();
    if (pendingComplete()) flushPending();
  }

  void flushPending() {
    const uint8_t expected = pending_.cov_expected;
    const uint8_t recv     = pending_.cov_received;
    if (!pending_.has_pvt) {
      pending_ = {};
      pending_.cov_expected = expected;
      return;
    }
    // Snapshot ECEF-direct fields before LLH-driven derivation overwrites them.
    msg::GnssSolution ecef_direct = pending_.buf;
    binary_solution_ = pending_.buf;

    // HPPOSLLH override (highest-priority LLH source): replace the NAV-PVT
    // position before geometry derivation so the derived ECEF inherits the
    // high-precision values. The accuracy fields refresh the diagonal-only
    // covariance unless NAV-COV already provided the full 3x3 (signalled by a
    // non-zero off-diagonal, same contract as parseNavPvt).
    if (recv & COV_BIT_HPPOSLLH) {
      binary_solution_.latitude  = pending_.hp_llh.lat_deg;
      binary_solution_.longitude = pending_.hp_llh.lon_deg;
      binary_solution_.altitude  = pending_.hp_llh.alt_m;
      const bool pos_cov_from_nav_cov =
          binary_solution_.pos_enu_cov[1] != 0.0 ||
          binary_solution_.pos_enu_cov[2] != 0.0 ||
          binary_solution_.pos_enu_cov[5] != 0.0;
      if (!pos_cov_from_nav_cov) {
        const double h_var_per_axis =
            pending_.hp_llh.hacc_m * pending_.hp_llh.hacc_m * 0.5;
        binary_solution_.pos_enu_cov[0] = h_var_per_axis;
        binary_solution_.pos_enu_cov[4] = h_var_per_axis;
        binary_solution_.pos_enu_cov[8] =
            pending_.hp_llh.vacc_m * pending_.hp_llh.vacc_m;
      }
    }

    // Derive ECEF position/velocity from LLH/ENU using current solution's LLH.
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

    // ECEF-direct overrides, by priority: HPPOSECEF > NAV-POSECEF > LLH-derived.
    if (recv & COV_BIT_HPPOSECEF) {
      binary_solution_.pos_ecef.x = pending_.hp_ecef[0];
      binary_solution_.pos_ecef.y = pending_.hp_ecef[1];
      binary_solution_.pos_ecef.z = pending_.hp_ecef[2];
    } else if (recv & COV_BIT_POSECEF) {
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
    last_flushed_week_   = binary_solution_.time_week;
    last_flushed_tow_ms_ = pending_.tow_ms;

    pending_ = {};
    pending_.cov_expected = expected;
  }

  void maybeWatchdogFlushPendingPvt() {
    if (!pending_.has_pvt && !pending_.cov_received) return;
    if ((now() - pending_.last_update).seconds() > kPvtWatchdogTimeoutSec) {
      flushPending();
    }
  }

  // Warn if BINARY source was selected via YAML but no PVT has arrived after
  // a generous startup window — typically indicates a receiver-side config
  // or capability issue. Prevents silent failure under no-fallback policy.

  void warnIfBinaryStarvation() {
    if (source_ != SolutionSource::BINARY) return;
    if (ever_received_binary_) return;
    if ((now() - start_time_).seconds() < 15.0) return;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
      "Solution source locked to BINARY (NAV-PVT) but no PVT received yet — "
      "check receiver firmware/configuration.");
  }

  // Solution Publishing

  void publishSolution(msg::GnssSolution& sol) {
    // Block publishing until configuration is complete, unless configure_on_startup is false
    if (!is_configured_ && config_.configure_on_startup) {
        return;
    }

    // BINARY path: time_tow keeps the PVT epoch's own iTOW (set by
    // parseNavPvt); raw_.time (RTKLIB observation-decode clock) supplies only
    // the week number, which UBX NAV-* payloads do not carry. Overwriting tow
    // with raw_.time would timestamp the position with the latest
    // *observation* epoch instead of the solution's own epoch. NMEA path
    // trusts time_week/time_tow already filled by NmeaParser from the sentence
    // itself.
    gtime_t t_gpst{};
    int week_for_stamp = 0;
    if (source_ == SolutionSource::BINARY) {
      const double raw_tow = time2gpst(raw_.time, &week_for_stamp);
      if (week_for_stamp > 0) {
        if (sol.time_tow > 0.0) {
          // Week-rollover guard: iTOW vs raw_tow more than half a week apart
          // means the two clocks straddle a GPS week boundary.
          if      (sol.time_tow - raw_tow >  302400.0) week_for_stamp -= 1;
          else if (sol.time_tow - raw_tow < -302400.0) week_for_stamp += 1;
          sol.time_week = static_cast<uint32_t>(week_for_stamp);
          t_gpst = gpst2time(week_for_stamp, sol.time_tow);
        } else {
          // iTOW unavailable (exact week-start epoch) — previous behavior.
          sol.time_week = static_cast<uint32_t>(week_for_stamp);
          sol.time_tow  = raw_tow;
          t_gpst = raw_.time;
        }
      } else {
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

    sol.header.stamp    = (config_.use_gps_timestamp && week_for_stamp > 0)
                          ? gnss_utils::gpstToUtcRosTime(t_gpst) : now();
    sol.header.frame_id = config_.frame_id;

    // Handle ENU conversion if a reference origin exists or setup if it doesn't
    if (sol.status == msg::GnssSolution::STATUS_FIX ||
        sol.status == msg::GnssSolution::STATUS_FLOAT ||
        sol.status == msg::GnssSolution::STATUS_SINGLE ||
        sol.status == msg::GnssSolution::STATUS_DGPS ||
        sol.status == msg::GnssSolution::STATUS_SBAS) {

      if (!has_local_origin_) {
        local_origin_ecef_[0] = sol.pos_ecef.x;
        local_origin_ecef_[1] = sol.pos_ecef.y;
        local_origin_ecef_[2] = sol.pos_ecef.z;
        ecef2pos(local_origin_ecef_, local_origin_pos_);
        has_local_origin_ = true;
        RCLCPP_INFO(get_logger(), "Auto-set local ENU origin: lat=%.6f, lon=%.6f, alt=%.2f",
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

  void handleDecodeResult(int result) {
    // Block publishing until configuration is complete, unless verify_on_startup is false
    if (!is_configured_ && config_.configure_on_startup) {
        return;
    }

    if (result == 1) {
        publishObservations();
    } else if (result == 2) {
        publishEphemerides();
    }
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
    msg.header.stamp    = (config_.use_gps_timestamp && week > 0)
                          ? gnss_utils::gpstToUtcRosTime(obs_time) : now();
    msg.header.frame_id = config_.frame_id;
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

  // Configuration
  UbxConfig config_;
  bool is_usb_connection_{false};

  // Publishers
  rclcpp::Publisher<msg::GnssObservations>::SharedPtr  obs_pub_;
  rclcpp::Publisher<msg::GnssEphemerides>::SharedPtr   eph_pub_;
  rclcpp::Publisher<msg::GnssSolution>::SharedPtr      sol_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr  imu_pub_;       // ESF-INS (calibrated)
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr  imu_raw_pub_;   // ESF-RAW (uncalibrated)
  rclcpp::TimerBase::SharedPtr timer_;

  // MALIB structures
  stream_t stream_{};
  raw_t raw_{};
  rtcm_t rtcm_{};

  // NMEA Parsing state
  gnss_utils::NmeaParser nmea_parser_;
  std::string nmea_buffer_;
  msg::GnssSolution nmea_solution_;
  msg::GnssSolution binary_solution_;
  PendingPvt        pending_;
  SolutionSource    source_{SolutionSource::NMEA};
  rclcpp::Time      start_time_{0, 0, RCL_ROS_TIME};
  bool              ever_received_binary_{false};

  // ENU Origin state
  bool has_local_origin_{false};
  double local_origin_ecef_[3]{0.0};
  double local_origin_pos_[3]{0.0}; // lat/lon/hgt (rad, rad, m)

  // UBX mini-framer state (for ESF-INS / ESF-RAW)
  int                  ubx_frm_state_{0};
  uint8_t              ubx_frm_cls_{0};
  uint8_t              ubx_frm_id_{0};
  uint16_t             ubx_frm_len_{0};
  uint16_t             ubx_frm_pos_{0};
  std::vector<uint8_t> ubx_frm_payload_;

  // ACK/NAK detection (instance variables instead of function-static)
  uint8_t pending_ack_class_{0};
  uint8_t pending_ack_id_{0};
  bool ack_received_{false};
  bool nak_received_{false};
  int ack_state_{0};
  uint8_t ack_buf_[10]{};
  int ack_pos_{0};

  // Unified ephemeris store
  gnss_utils::EphemerisStore eph_store_;

  // State
  bool is_configured_{false};
};

}  // namespace gnss_ros_standardization

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gnss_ros_standardization::UbxDriverNode>());
  rclcpp::shutdown();
  return 0;
}