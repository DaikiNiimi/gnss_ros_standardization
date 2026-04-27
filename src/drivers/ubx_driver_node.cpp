#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstring>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <sensor_msgs/msg/imu.hpp>

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

// UBX-ESF-INS payload offsets (version 0, 36 bytes)
constexpr int ESF_INS_OFFSET_BITFIELD  = 0;
constexpr int ESF_INS_OFFSET_XANGRATE  = 12;
constexpr int ESF_INS_OFFSET_YANGRATE  = 16;
constexpr int ESF_INS_OFFSET_ZANGRATE  = 20;
constexpr int ESF_INS_OFFSET_XACCEL    = 24;
constexpr int ESF_INS_OFFSET_YACCEL    = 28;
constexpr int ESF_INS_OFFSET_ZACCEL    = 32;
constexpr int ESF_INS_MIN_LEN          = 36;

// UBX-NAV-ATT payload offsets (32 bytes)
constexpr int NAV_ATT_OFFSET_ROLL      = 8;
constexpr int NAV_ATT_OFFSET_PITCH     = 12;
constexpr int NAV_ATT_OFFSET_HEADING   = 16;
constexpr int NAV_ATT_OFFSET_ACCROLL   = 20;
constexpr int NAV_ATT_OFFSET_ACCPITCH  = 24;
constexpr int NAV_ATT_OFFSET_ACCHEADING= 28;
constexpr int NAV_ATT_MIN_LEN          = 32;

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
  bool enable_nmea_gga{false};
  bool enable_nmea_rmc{false};
  bool enable_nmea_gsa{false};
  bool enable_nmea_gst{false};
  bool nmea_high_precision{false};

  // IMU settings (ZED-F9R / IMU-enabled receivers)
  bool enable_esf_ins{false};   // ESF-INS: calibrated angular rate + acceleration
  bool enable_nav_att{false};   // NAV-ATT: attitude (roll/pitch/heading)
  std::string imu_raw_topic{"/gnss/imu/data_raw"};
  std::string imu_attitude_topic{"/gnss/imu/attitude"};
  
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
  std::string solution_topic{"/gnss/solution"};

  // ENU local origin settings
  bool auto_origin{true};
  double origin_latitude{0.0};
  double origin_longitude{0.0};
  double origin_altitude{0.0};
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
  // ============================================================================
  // Initialization
  // ============================================================================

  void initializeParameters() {
    // Basic settings
    declare_parameter<std::string>("stream_path", config_.stream_path);
    declare_parameter<int>("rate_hz", config_.rate_hz);
    declare_parameter<bool>("configure_on_startup", config_.configure_on_startup);
    declare_parameter<std::string>("dynamic_model", config_.dynamic_model);
    declare_parameter<std::string>("generation", config_.generation);
    declare_parameter<std::string>("frame_id", config_.frame_id);
    
    // Message settings
    declare_parameter<bool>("messages.rawx", config_.enable_rawx);
    declare_parameter<bool>("messages.sfrbx", config_.enable_sfrbx);
    declare_parameter<bool>("messages.nav_pvt", config_.enable_nav_pvt);
    declare_parameter<bool>("messages.nmea_gga", config_.enable_nmea_gga);
    declare_parameter<bool>("messages.nmea_rmc", config_.enable_nmea_rmc);
    declare_parameter<bool>("messages.nmea_gsa", config_.enable_nmea_gsa);
    declare_parameter<bool>("messages.nmea_gst", config_.enable_nmea_gst);
    declare_parameter<bool>("messages.nmea_high_precision", config_.nmea_high_precision);
    declare_parameter<bool>("messages.esf_ins", config_.enable_esf_ins);
    declare_parameter<bool>("messages.nav_att", config_.enable_nav_att);
    declare_parameter<std::string>("imu_raw_topic", config_.imu_raw_topic);
    declare_parameter<std::string>("imu_attitude_topic",     config_.imu_attitude_topic);
    
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
    declare_parameter<double>("origin.latitude", config_.origin_latitude);
    declare_parameter<double>("origin.longitude", config_.origin_longitude);
    declare_parameter<double>("origin.altitude", config_.origin_altitude);

    // Read parameters
    config_.stream_path = get_parameter("stream_path").as_string();
    config_.rate_hz = get_parameter("rate_hz").as_int();
    config_.configure_on_startup = get_parameter("configure_on_startup").as_bool();
    config_.dynamic_model = get_parameter("dynamic_model").as_string();
    config_.generation = get_parameter("generation").as_string();
    config_.frame_id = get_parameter("frame_id").as_string();
    
    config_.enable_rawx = get_parameter("messages.rawx").as_bool();
    config_.enable_sfrbx = get_parameter("messages.sfrbx").as_bool();
    config_.enable_nav_pvt = get_parameter("messages.nav_pvt").as_bool();
    config_.enable_nmea_gga = get_parameter("messages.nmea_gga").as_bool();
    config_.enable_nmea_rmc = get_parameter("messages.nmea_rmc").as_bool();
    config_.enable_nmea_gsa = get_parameter("messages.nmea_gsa").as_bool();
    config_.enable_nmea_gst = get_parameter("messages.nmea_gst").as_bool();
    config_.nmea_high_precision = get_parameter("messages.nmea_high_precision").as_bool();
    config_.enable_esf_ins      = get_parameter("messages.esf_ins").as_bool();
    config_.enable_nav_att      = get_parameter("messages.nav_att").as_bool();
    config_.imu_raw_topic       = get_parameter("imu_raw_topic").as_string();
    config_.imu_attitude_topic           = get_parameter("imu_attitude_topic").as_string();
    
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
    config_.origin_latitude = get_parameter("origin.latitude").as_double();
    config_.origin_longitude = get_parameter("origin.longitude").as_double();
    config_.origin_altitude = get_parameter("origin.altitude").as_double();

    // Detect USB connection from path
    is_usb_connection_ = (config_.stream_path.find("ttyACM") != std::string::npos);

    // Validate rate
    if (config_.rate_hz < 1 || config_.rate_hz > 20) {
      RCLCPP_WARN(get_logger(), "rate_hz out of range [1-20], clamping to 5");
      config_.rate_hz = 5;
    }
    
    // Log configuration
    RCLCPP_INFO(get_logger(), "Configuration loaded:");
    RCLCPP_INFO(get_logger(), "  Stream: %s", config_.stream_path.c_str());
    RCLCPP_INFO(get_logger(), "  Rate: %d Hz", config_.rate_hz);
    RCLCPP_INFO(get_logger(), "  Generation: %s", config_.generation.c_str());
    RCLCPP_INFO(get_logger(), "  Dynamic model: %s", config_.dynamic_model.c_str());
    RCLCPP_INFO(get_logger(), "  GNSS: GPS=%d GLO=%d GAL=%d BDS=%d QZS=%d IRN=%d SBAS=%d",
                config_.enable_gps, config_.enable_glonass, config_.enable_galileo,
                config_.enable_beidou, config_.enable_qzss, config_.enable_navic, config_.enable_sbas);
  }

  void initializePublishers() {
    obs_pub_     = create_publisher<msg::GnssObservations>(config_.observation_topic, 10);
    eph_pub_     = create_publisher<msg::GnssEphemerides>(config_.ephemeris_topic, rclcpp::QoS(100).transient_local());
    sol_pub_     = create_publisher<msg::GnssSolution>(config_.solution_topic, 10);
    imu_raw_pub_ = create_publisher<sensor_msgs::msg::Imu>(config_.imu_raw_topic, 10);
    imu_attitude_pub_     = create_publisher<sensor_msgs::msg::Imu>(config_.imu_attitude_topic, 10);

    // Pre-configure ENU origin if not auto
    if (!config_.auto_origin) {
      local_origin_pos_[0] = config_.origin_latitude * (M_PI / 180.0);
      local_origin_pos_[1] = config_.origin_longitude * (M_PI / 180.0);
      local_origin_pos_[2] = config_.origin_altitude;
      pos2ecef(local_origin_pos_, local_origin_ecef_);
      has_local_origin_ = true;
      RCLCPP_INFO(get_logger(), "Configured local ENU origin: lat=%.6f, lon=%.6f, alt=%.2f",
                  config_.origin_latitude, config_.origin_longitude, config_.origin_altitude);
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

  // ============================================================================
  // Stream Management
  // ============================================================================

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

  // ============================================================================
  // Receiver Configuration (Unified Entry Point)
  // ============================================================================

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
             if (input_ubx(&raw_, &rtcm_, b)) {
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

  // ============================================================================
  // Step 1: GNSS Signal Configuration
  // ============================================================================

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
             input_ubx(&raw_, &rtcm_, b);
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

  // ============================================================================
  // Step 2: Measurement Rate
  // ============================================================================

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

  // ============================================================================
  // Step 3: Dynamic Model
  // ============================================================================

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

  // ============================================================================
  // Step 4: Output Messages
  // ============================================================================

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
          {ubx::CFG_MSGOUT_UBX_NAV_PVT_I2C   + port, config_.enable_nav_pvt ? 1u : 0u, 1},
          {ubx::CFG_MSGOUT_UBX_ESF_INS_I2C   + port, config_.enable_esf_ins ? 1u : 0u, 1},
          {ubx::CFG_MSGOUT_UBX_NAV_ATT_I2C   + port, config_.enable_nav_att ? 1u : 0u, 1},
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
    sendMsg(ubx::CLASS_NAV, ubx::ID_NAV_PVT,   config_.enable_nav_pvt, "NAV-PVT");
    sendMsg(ubx::CLASS_ESF, ubx::ID_ESF_INS,   config_.enable_esf_ins, "ESF-INS");
    sendMsg(ubx::CLASS_NAV, ubx::ID_NAV_ATT,   config_.enable_nav_att, "NAV-ATT");

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

  // ============================================================================
  // UBX Communication (Gen 10: CFG-VALSET)
  // ============================================================================

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

  // ============================================================================
  // UBX Communication (Low-level)
  // ============================================================================

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

  // ============================================================================
  // ACK/NAK Detection
  // ============================================================================

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
        int result = input_ubx(&raw_, &rtcm_, buffer[i]);

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

  // ============================================================================
  // UBX Mini-Framer (parallel to RTKLIB, for ESF-INS and NAV-ATT)
  // ============================================================================

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
    if (ubx_frm_cls_ == ubx::CLASS_ESF && ubx_frm_id_ == ubx::ID_ESF_INS) handleEsfIns();
    if (ubx_frm_cls_ == ubx::CLASS_NAV && ubx_frm_id_ == ubx::ID_NAV_ATT) handleNavAtt();
  }

  void handleEsfIns() {
    if (static_cast<int>(ubx_frm_payload_.size()) < ESF_INS_MIN_LEN) return;

    uint32_t bitfield0 = 0;
    std::memcpy(&bitfield0, ubx_frm_payload_.data() + ESF_INS_OFFSET_BITFIELD, 4);
    const bool ang_valid = (bitfield0 & (0x7u << 8))  == (0x7u << 8);
    const bool acc_valid = (bitfield0 & (0x7u << 11)) == (0x7u << 11);

    int32_t xAng = 0, yAng = 0, zAng = 0, xAcc = 0, yAcc = 0, zAcc = 0;
    std::memcpy(&xAng, ubx_frm_payload_.data() + ESF_INS_OFFSET_XANGRATE, 4);
    std::memcpy(&yAng, ubx_frm_payload_.data() + ESF_INS_OFFSET_YANGRATE, 4);
    std::memcpy(&zAng, ubx_frm_payload_.data() + ESF_INS_OFFSET_ZANGRATE, 4);
    std::memcpy(&xAcc, ubx_frm_payload_.data() + ESF_INS_OFFSET_XACCEL,   4);
    std::memcpy(&yAcc, ubx_frm_payload_.data() + ESF_INS_OFFSET_YACCEL,   4);
    std::memcpy(&zAcc, ubx_frm_payload_.data() + ESF_INS_OFFSET_ZACCEL,   4);

    const double deg2rad = M_PI / 180.0;
    last_ang_x_ = xAng * 1e-3 * deg2rad;
    last_ang_y_ = yAng * 1e-3 * deg2rad;
    last_ang_z_ = zAng * 1e-3 * deg2rad;
    last_acc_x_ = xAcc * 1e-2;
    last_acc_y_ = yAcc * 1e-2;
    last_acc_z_ = zAcc * 1e-2;

    sensor_msgs::msg::Imu imu;
    imu.header.stamp    = now();
    imu.header.frame_id = config_.frame_id;

    auto unk = ins::makeUnknownCovariance();
    std::copy(unk.begin(), unk.end(), imu.orientation_covariance.begin());

    imu.angular_velocity.x = last_ang_x_;
    imu.angular_velocity.y = last_ang_y_;
    imu.angular_velocity.z = last_ang_z_;
    auto ang_cov = ang_valid ? ins::makeDiagCovariance(1e-4, 1e-4, 1e-4) : ins::makeUnknownCovariance();
    std::copy(ang_cov.begin(), ang_cov.end(), imu.angular_velocity_covariance.begin());

    imu.linear_acceleration.x = last_acc_x_;
    imu.linear_acceleration.y = last_acc_y_;
    imu.linear_acceleration.z = last_acc_z_;
    auto acc_cov = acc_valid ? ins::makeDiagCovariance(1e-3, 1e-3, 1e-3) : ins::makeUnknownCovariance();
    std::copy(acc_cov.begin(), acc_cov.end(), imu.linear_acceleration_covariance.begin());

    imu_raw_pub_->publish(imu);
  }

  void handleNavAtt() {
    if (static_cast<int>(ubx_frm_payload_.size()) < NAV_ATT_MIN_LEN) return;

    int32_t  roll_raw = 0, pitch_raw = 0, heading_raw = 0;
    uint32_t acc_roll = 0, acc_pitch = 0, acc_heading = 0;
    std::memcpy(&roll_raw,    ubx_frm_payload_.data() + NAV_ATT_OFFSET_ROLL,       4);
    std::memcpy(&pitch_raw,   ubx_frm_payload_.data() + NAV_ATT_OFFSET_PITCH,      4);
    std::memcpy(&heading_raw, ubx_frm_payload_.data() + NAV_ATT_OFFSET_HEADING,    4);
    std::memcpy(&acc_roll,    ubx_frm_payload_.data() + NAV_ATT_OFFSET_ACCROLL,    4);
    std::memcpy(&acc_pitch,   ubx_frm_payload_.data() + NAV_ATT_OFFSET_ACCPITCH,   4);
    std::memcpy(&acc_heading, ubx_frm_payload_.data() + NAV_ATT_OFFSET_ACCHEADING, 4);

    const double scale   = 1e-5 * M_PI / 180.0;
    const double roll    = roll_raw    * scale;
    const double pitch   = pitch_raw   * scale;
    const double heading = heading_raw * scale;

    sensor_msgs::msg::Imu imu;
    imu.header.stamp    = now();
    imu.header.frame_id = config_.frame_id;

    imu.orientation = ins::eulerToQuaternion(roll, pitch, heading);
    auto ori_cov = ins::makeDiagCovariance(acc_roll * scale, acc_pitch * scale, acc_heading * scale);
    std::copy(ori_cov.begin(), ori_cov.end(), imu.orientation_covariance.begin());

    imu.angular_velocity.x = last_ang_x_;
    imu.angular_velocity.y = last_ang_y_;
    imu.angular_velocity.z = last_ang_z_;
    imu.linear_acceleration.x = last_acc_x_;
    imu.linear_acceleration.y = last_acc_y_;
    imu.linear_acceleration.z = last_acc_z_;

    auto unk = ins::makeUnknownCovariance();
    std::copy(unk.begin(), unk.end(), imu.angular_velocity_covariance.begin());
    std::copy(unk.begin(), unk.end(), imu.linear_acceleration_covariance.begin());

    imu_attitude_pub_->publish(imu);
  }

  // ============================================================================
  // Data Processing
  // ============================================================================

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

        const int result = input_ubx(&raw_, &rtcm_, byte);
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
          } else if (nmea_buffer_.size() > 256) { // Safeguard against missing newlines
            nmea_buffer_.clear();
          }
        }
      }
      
      // If we read less than the full buffer, the stream is likely empty now
      if (bytes_read < static_cast<int>(sizeof(buffer))) {
        break;
      }
    }
  }

  void handleNmeaSentence(const std::string& sentence) {
    if (nmea_parser_.parseSentence(sentence, current_solution_)) {
      publishSolution();
    }
  }

  // ============================================================================
  // Solution Publishing
  // ============================================================================

  void publishSolution() {
    // Block publishing until configuration is complete, unless configure_on_startup is false
    if (!is_configured_ && config_.configure_on_startup) {
        return;
    }

    // Determine header timestamp
    int week = 0;
    const double tow = time2gpst(raw_.time, &week);
    if (week != 0) {
      current_solution_.time_week = static_cast<uint16_t>(week);
      current_solution_.time_tow = tow;
    }

    current_solution_.header.stamp = now();
    current_solution_.header.frame_id = config_.frame_id;

    // Handle ENU conversion if a reference origin exists or setup if it doesn't
    if (current_solution_.status == msg::GnssSolution::STATUS_FIX ||
        current_solution_.status == msg::GnssSolution::STATUS_FLOAT ||
        current_solution_.status == msg::GnssSolution::STATUS_SINGLE ||
        current_solution_.status == msg::GnssSolution::STATUS_DGPS ||
        current_solution_.status == msg::GnssSolution::STATUS_SBAS) {
      
      if (!has_local_origin_) {
        // Set first valid position as ENU origin
        local_origin_ecef_[0] = current_solution_.pos_ecef.x;
        local_origin_ecef_[1] = current_solution_.pos_ecef.y;
        local_origin_ecef_[2] = current_solution_.pos_ecef.z;
        ecef2pos(local_origin_ecef_, local_origin_pos_);
        has_local_origin_ = true;
        RCLCPP_INFO(get_logger(), "Auto-set local ENU origin: lat=%.6f, lon=%.6f, alt=%.2f",
                    local_origin_pos_[0] * (180.0/M_PI), local_origin_pos_[1] * (180.0/M_PI), local_origin_pos_[2]);
      }
      
      // Transform position and origin to message
      current_solution_.org_ecef.x = local_origin_ecef_[0];
      current_solution_.org_ecef.y = local_origin_ecef_[1];
      current_solution_.org_ecef.z = local_origin_ecef_[2];

      double ecef[3] = {
        current_solution_.pos_ecef.x - local_origin_ecef_[0],
        current_solution_.pos_ecef.y - local_origin_ecef_[1],
        current_solution_.pos_ecef.z - local_origin_ecef_[2]
      };
      double enu[3] = {0};
      ecef2enu(local_origin_pos_, ecef, enu);

      current_solution_.pos_enu.x = enu[0];
      current_solution_.pos_enu.y = enu[1];
      current_solution_.pos_enu.z = enu[2];
      
      // Transform velocity
      double vel_ecef[3] = {current_solution_.vel_ecef.x, current_solution_.vel_ecef.y, current_solution_.vel_ecef.z};
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

  // ============================================================================
  // Observation Publishing
  // ============================================================================

  void publishObservations() {
    if (raw_.obs.n <= 0) return;

    int week = 0;
    const double tow = time2gpst(raw_.time, &week);

    msg::GnssObservations msg;
    msg.header.stamp = now();
    msg.header.frame_id = config_.frame_id;
    msg.week = static_cast<uint16_t>(week);
    msg.tow = tow;

    SatelliteCount sat_count{};

    for (int i = 0; i < raw_.obs.n; ++i) {
      const obsd_t& obs = raw_.obs.data[i];
      countSatellite(obs.sat, sat_count);
      appendObservations(obs, msg.observations);
    }

    obs_pub_->publish(msg);

    RCLCPP_INFO(
        get_logger(),
        "Published obs: week=%d tow=%.3f n=%zu sats(G/R/E/J/C/I/S/U)=(%d/%d/%d/%d/%d/%d/%d/%d)",
        week, tow, msg.observations.size(), sat_count.gps, sat_count.glo, sat_count.gal,
        sat_count.qzs, sat_count.bds, sat_count.irn, sat_count.sbs, sat_count.unknown);
  }

  void appendObservations(const obsd_t& obs, std::vector<msg::GnssObservation>& observations) {
    for (int freq = 0; freq < NFREQ + NEXOBS; ++freq) {
      if (obs.P[freq] == 0.0 && obs.L[freq] == 0.0 && obs.D[freq] == 0.0 && obs.SNR[freq] == 0) {
        continue;
      }
      observations.push_back(gnss_utils::obsToMsg(obs, freq));
    }
  }

  // ============================================================================
  // Ephemeris Publishing
  // ============================================================================

  void publishEphemerides() {
    bool has_new = false;

    std::vector<msg::GnssEphemeris> gnss_eph;
    std::vector<msg::GlonassEphemeris> glo_eph;

    for (int i = 0; i < raw_.nav.n; ++i) {
      const eph_t& eph = raw_.nav.eph[i];
      if (eph.sat == 0) continue;

      int prn = 0;
      if (satsys(eph.sat, &prn) == SYS_GLO) continue;

      EphemerisKey key{eph.sat, eph.iode, eph.iodc, eph.code};
      if (seen_ephemeris_.insert(key).second) {
        gnss_eph.push_back(gnss_utils::ephToMsg(eph));
        has_new = true;
      }
    }

    for (int i = 0; i < raw_.nav.ng; ++i) {
      const geph_t& geph = raw_.nav.geph[i];
      if (geph.sat == 0) continue;

      auto it = last_glo_iode_.find(geph.sat);
      if (it == last_glo_iode_.end() || it->second != geph.iode) {
        last_glo_iode_[geph.sat] = geph.iode;
        has_new = true;
        glo_eph.push_back(gnss_utils::gephToMsg(geph));
      }
    }

    if (!has_new && !first_ephemeris_) return;
    first_ephemeris_ = false;

    msg::GnssEphemerides msg;
    msg.header.stamp = now();
    msg.gnss_ephemeris = std::move(gnss_eph);
    msg.glonass_ephemeris = std::move(glo_eph);
    eph_pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "Published eph: GNSS=%zu GLO=%zu (new=%s)",
                msg.gnss_ephemeris.size(), msg.glonass_ephemeris.size(), has_new ? "yes" : "no");
  }

  // ============================================================================
  // Helper Types
  // ============================================================================

  struct SatelliteCount {
    int gps = 0, glo = 0, gal = 0, qzs = 0, bds = 0, irn = 0, sbs = 0, unknown = 0;
  };

  static void countSatellite(int sat, SatelliteCount& count) {
    int prn = 0;
    switch (satsys(sat, &prn)) {
      case SYS_GPS: ++count.gps; break;
      case SYS_GLO: ++count.glo; break;
      case SYS_GAL: ++count.gal; break;
      case SYS_QZS: ++count.qzs; break;
      case SYS_CMP: ++count.bds; break;
      case SYS_IRN: ++count.irn; break;
      case SYS_SBS: ++count.sbs; break;
      default: ++count.unknown; break;
    }
  }

  struct EphemerisKey {
    int sat, iode, iodc, code;
    bool operator==(const EphemerisKey& o) const {
      return sat == o.sat && iode == o.iode && iodc == o.iodc && code == o.code;
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

  // Configuration
  UbxConfig config_;
  bool is_usb_connection_{false};

  // Publishers
  rclcpp::Publisher<msg::GnssObservations>::SharedPtr  obs_pub_;
  rclcpp::Publisher<msg::GnssEphemerides>::SharedPtr   eph_pub_;
  rclcpp::Publisher<msg::GnssSolution>::SharedPtr      sol_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr  imu_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr  imu_attitude_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // MALIB structures
  stream_t stream_{};
  raw_t raw_{};
  rtcm_t rtcm_{};

  // NMEA Parsing state
  gnss_utils::NmeaParser nmea_parser_;
  std::string nmea_buffer_;
  msg::GnssSolution current_solution_;

  // ENU Origin state
  bool has_local_origin_{false};
  double local_origin_ecef_[3]{0.0};
  double local_origin_pos_[3]{0.0}; // lat/lon/hgt (rad, rad, m)

  // UBX mini-framer state (for ESF-INS and NAV-ATT)
  int                  ubx_frm_state_{0};
  uint8_t              ubx_frm_cls_{0};
  uint8_t              ubx_frm_id_{0};
  uint16_t             ubx_frm_len_{0};
  uint16_t             ubx_frm_pos_{0};
  std::vector<uint8_t> ubx_frm_payload_;

  // Latest ESF-INS values (merged into NAV-ATT message)
  double last_ang_x_{0.0}, last_ang_y_{0.0}, last_ang_z_{0.0};
  double last_acc_x_{0.0}, last_acc_y_{0.0}, last_acc_z_{0.0};

  // ACK/NAK detection (instance variables instead of function-static)
  uint8_t pending_ack_class_{0};
  uint8_t pending_ack_id_{0};
  bool ack_received_{false};
  bool nak_received_{false};
  int ack_state_{0};
  uint8_t ack_buf_[10]{};
  int ack_pos_{0};

  // Ephemeris tracking
  std::unordered_set<EphemerisKey, EphemerisKeyHash> seen_ephemeris_;
  std::unordered_map<int, int> last_glo_iode_;
  bool first_ephemeris_{true};
  
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
