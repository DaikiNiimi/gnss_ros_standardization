#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstring>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/novatel_protocol.hpp"

using namespace std::chrono_literals;

namespace gnss_ros_standardization {

namespace {

/// Timer interval for stream polling
constexpr auto kTimerInterval = 10ms;

}  // namespace

/// @brief Configuration structure for NovAtel driver
struct NovatelConfig {
  std::string stream_path{"serial:///dev/ttyUSB0:115200"};
  std::string frame_id{"gnss_link"};
  int publish_rate{1};           // Hz
  std::string receiver_port{"USB1"}; // Port on receiver to output data
  std::string format{"oem4"};    // oem4 or oem3
  bool configure_on_startup{true};

  // Message settings
  bool enable_rangecmp{true};
  bool enable_range{false};
  bool enable_bestpos{false};
  bool enable_bestvel{false};

  // Ephemeris
  bool enable_gps_ephem{true};
  bool enable_glo_ephem{true};
  bool enable_gal_ephem{true};
  bool enable_bds_ephem{true};
  bool enable_qzs_ephem{true};
  bool enable_navic_ephem{false};
  
  bool enable_ionutc{true};

  // Topics
  std::string observation_topic{"/gnss/observation"};
  std::string ephemeris_topic{"/gnss/ephemeris"};
};

/// @brief ROS 2 driver node for NovAtel GNSS receivers
class NovatelDriverNode : public rclcpp::Node {
 public:
  NovatelDriverNode() : Node("novatel_driver_node") {
    initializeParameters();
    initializePublishers();
    initializeDecoder();
    openStream();

    if (config_.configure_on_startup) {
      configureReceiver();
    }

    startPolling();
    RCLCPP_INFO(get_logger(), "NovAtel Driver initialized");
  }

  ~NovatelDriverNode() override {
    try {
      strclose(&stream_);
    } catch (...) {
      // Ignore errors during cleanup
    }
    free_raw(&raw_);
  }

 private:
  // ============================================================================
  // Initialization
  // ============================================================================

  void initializeParameters() {
    declare_parameter<std::string>("stream_path", config_.stream_path);
    declare_parameter<std::string>("frame_id", config_.frame_id);
    declare_parameter<int>("publish_rate", config_.publish_rate);
    declare_parameter<std::string>("receiver_port", config_.receiver_port);
    declare_parameter<std::string>("format", config_.format);
    declare_parameter<bool>("configure_on_startup", config_.configure_on_startup);
    
    declare_parameter<bool>("messages.rangecmp", config_.enable_rangecmp);
    declare_parameter<bool>("messages.range", config_.enable_range);
    declare_parameter<bool>("messages.bestpos", config_.enable_bestpos);
    declare_parameter<bool>("messages.bestvel", config_.enable_bestvel);
    
    declare_parameter<bool>("messages.gps_ephem", config_.enable_gps_ephem);
    declare_parameter<bool>("messages.glo_ephem", config_.enable_glo_ephem);
    declare_parameter<bool>("messages.gal_ephem", config_.enable_gal_ephem);
    declare_parameter<bool>("messages.bds_ephem", config_.enable_bds_ephem);
    declare_parameter<bool>("messages.qzs_ephem", config_.enable_qzs_ephem);
    declare_parameter<bool>("messages.navic_ephem", config_.enable_navic_ephem);
    
    declare_parameter<bool>("messages.ionutc", config_.enable_ionutc);

    declare_parameter<std::string>("observation_topic", config_.observation_topic);
    declare_parameter<std::string>("ephemeris_topic", config_.ephemeris_topic);

    config_.stream_path = get_parameter("stream_path").as_string();
    config_.frame_id = get_parameter("frame_id").as_string();
    config_.publish_rate = get_parameter("publish_rate").as_int();
    config_.receiver_port = get_parameter("receiver_port").as_string();
    config_.format = get_parameter("format").as_string();
    config_.configure_on_startup = get_parameter("configure_on_startup").as_bool();
    
    if (config_.format != "oem3" && config_.format != "oem4") {
        RCLCPP_WARN(get_logger(), "Unknown format '%s', defaulting to oem4", config_.format.c_str());
        config_.format = "oem4";
    }
    
    config_.enable_rangecmp = get_parameter("messages.rangecmp").as_bool();
    config_.enable_range = get_parameter("messages.range").as_bool();
    config_.enable_bestpos = get_parameter("messages.bestpos").as_bool();
    config_.enable_bestvel = get_parameter("messages.bestvel").as_bool();
    
    config_.enable_gps_ephem = get_parameter("messages.gps_ephem").as_bool();
    config_.enable_glo_ephem = get_parameter("messages.glo_ephem").as_bool();
    config_.enable_gal_ephem = get_parameter("messages.gal_ephem").as_bool();
    config_.enable_bds_ephem = get_parameter("messages.bds_ephem").as_bool();
    config_.enable_qzs_ephem = get_parameter("messages.qzs_ephem").as_bool();
    config_.enable_navic_ephem = get_parameter("messages.navic_ephem").as_bool();
    
    config_.enable_ionutc = get_parameter("messages.ionutc").as_bool();

    config_.observation_topic = get_parameter("observation_topic").as_string();
    config_.ephemeris_topic = get_parameter("ephemeris_topic").as_string();
  }

  void initializePublishers() {
    obs_pub_ = create_publisher<msg::GnssObservations>(config_.observation_topic, 10);
    eph_pub_ = create_publisher<msg::GnssEphemerides>(config_.ephemeris_topic, 10);
  }

  void initializeDecoder() {
    int strfmt = STRFMT_OEM4;
    if (config_.format == "oem3") {
        strfmt = STRFMT_OEM3;
    }

    if (init_raw(&raw_, strfmt) != 1) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize raw decoder");
      throw std::runtime_error("init_raw failed");
    }
  }

  void startPolling() {
    timer_ = create_wall_timer(kTimerInterval, std::bind(&NovatelDriverNode::pollStream, this));
  }

  // ============================================================================
  // Stream Management
  // ============================================================================

  void openStream() {
    std::string path = config_.stream_path;

    // Match stream type from URI prefix
    int stream_type = STR_SERIAL; // Default
    bool matched = false;
    for (const auto& def : novatel::kStreamTypes) {
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

    // MALIB serial path format: device name without /dev/ prefix
    if (stream_type == STR_SERIAL && path.rfind("/dev/", 0) == 0) {
      path.erase(0, 5);
      RCLCPP_DEBUG(get_logger(), "Adjusted serial path for MALIB: %s", path.c_str());
    }

    // We need to initialize stream first
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
    if (cmd.empty()) return;
    
    std::string full_cmd = cmd + "\r\n";
    RCLCPP_INFO(get_logger(), "Sending command: %s", cmd.c_str());

    int written = strwrite(&stream_, (uint8_t*)full_cmd.c_str(), full_cmd.size());
    
    // Workaround for MALIB writeserial bug on Linux (returns 0 on success)
    if (written < 0 || (written == 0 && !is_serial_connection_)) {
        RCLCPP_ERROR(get_logger(), "Failed to write command (result: %d)", written);
    }
    
    // Short delay
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  void configureReceiverOem4(const std::string& port_prefix, const std::string& ontime_suffix, const std::string& onchanged_suffix) {
    if (config_.enable_rangecmp) {
        sendCommand(std::string(novatel::CMD_LOG) + " " + port_prefix + novatel::LOG_RANGECMP + ontime_suffix);
    } else {
        sendCommand(std::string(novatel::CMD_UNLOG) + " " + port_prefix + novatel::LOG_RANGECMP);
    }
    if (config_.enable_range) {
        sendCommand(std::string(novatel::CMD_LOG) + " " + port_prefix + novatel::LOG_RANGE + ontime_suffix);
    } else {
        sendCommand(std::string(novatel::CMD_UNLOG) + " " + port_prefix + novatel::LOG_RANGE);
    }
    if (config_.enable_bestpos) {
        sendCommand(std::string(novatel::CMD_LOG) + " " + port_prefix + novatel::LOG_BESTPOS + ontime_suffix);
    } else {
        sendCommand(std::string(novatel::CMD_UNLOG) + " " + port_prefix + novatel::LOG_BESTPOS);
    }
    if (config_.enable_bestvel) {
        sendCommand(std::string(novatel::CMD_LOG) + " " + port_prefix + novatel::LOG_BESTVEL + ontime_suffix);
    } else {
        sendCommand(std::string(novatel::CMD_UNLOG) + " " + port_prefix + novatel::LOG_BESTVEL);
    }

    if (config_.enable_gps_ephem) {
        sendCommand(std::string(novatel::CMD_LOG) + " " + port_prefix + novatel::LOG_GPSEPHEM + onchanged_suffix);
    } else {
        sendCommand(std::string(novatel::CMD_UNLOG) + " " + port_prefix + novatel::LOG_GPSEPHEM);
    }
    if (config_.enable_glo_ephem) {
        sendCommand(std::string(novatel::CMD_LOG) + " " + port_prefix + novatel::LOG_GLOEPHEMERIS + onchanged_suffix);
    } else {
        sendCommand(std::string(novatel::CMD_UNLOG) + " " + port_prefix + novatel::LOG_GLOEPHEMERIS);
    }
    if (config_.enable_gal_ephem) {
        sendCommand(std::string(novatel::CMD_LOG) + " " + port_prefix + novatel::LOG_GALEPHEMERIS + onchanged_suffix);
        sendCommand(std::string(novatel::CMD_LOG) + " " + port_prefix + novatel::LOG_GALINAVEPHEMERIS + onchanged_suffix);
    } else {
        sendCommand(std::string(novatel::CMD_UNLOG) + " " + port_prefix + novatel::LOG_GALEPHEMERIS);
        sendCommand(std::string(novatel::CMD_UNLOG) + " " + port_prefix + novatel::LOG_GALINAVEPHEMERIS);
    }
    if (config_.enable_bds_ephem) {
        sendCommand(std::string(novatel::CMD_LOG) + " " + port_prefix + novatel::LOG_BDSEPHEMERIS + onchanged_suffix);
    } else {
        sendCommand(std::string(novatel::CMD_UNLOG) + " " + port_prefix + novatel::LOG_BDSEPHEMERIS);
    }
    if (config_.enable_qzs_ephem) {
        sendCommand(std::string(novatel::CMD_LOG) + " " + port_prefix + novatel::LOG_QZSSEPHEMERIS + onchanged_suffix);
    } else {
        sendCommand(std::string(novatel::CMD_UNLOG) + " " + port_prefix + novatel::LOG_QZSSEPHEMERIS);
    }
    if (config_.enable_navic_ephem) {
        sendCommand(std::string(novatel::CMD_LOG) + " " + port_prefix + novatel::LOG_NAVICEPHEMERIS + onchanged_suffix);
    } else {
        sendCommand(std::string(novatel::CMD_UNLOG) + " " + port_prefix + novatel::LOG_NAVICEPHEMERIS);
    }
    
    if (config_.enable_ionutc) {
        sendCommand(std::string(novatel::CMD_LOG) + " " + port_prefix + novatel::LOG_IONUTC + onchanged_suffix);
        sendCommand(std::string(novatel::CMD_LOG) + " " + port_prefix + novatel::LOG_GALIONO + onchanged_suffix);
        sendCommand(std::string(novatel::CMD_LOG) + " " + port_prefix + novatel::LOG_QZSSIONUTC + onchanged_suffix);
    } else {
        sendCommand(std::string(novatel::CMD_UNLOG) + " " + port_prefix + novatel::LOG_IONUTC);
        sendCommand(std::string(novatel::CMD_UNLOG) + " " + port_prefix + novatel::LOG_GALIONO);
        sendCommand(std::string(novatel::CMD_UNLOG) + " " + port_prefix + novatel::LOG_QZSSIONUTC);
    }
  }

  void configureReceiverOem3(const std::string& port_prefix, const std::string& ontime_suffix, const std::string& onchanged_suffix) {
    if (config_.enable_rangecmp) {
        sendCommand(std::string(novatel::CMD_LOG) + " " + port_prefix + novatel::LOG_OEM3_RGED + ontime_suffix);
    } else {
        sendCommand(std::string(novatel::CMD_UNLOG) + " " + port_prefix + novatel::LOG_OEM3_RGED);
    }
    if (config_.enable_range) {
        sendCommand(std::string(novatel::CMD_LOG) + " " + port_prefix + novatel::LOG_OEM3_RGEB + ontime_suffix);
    } else {
        sendCommand(std::string(novatel::CMD_UNLOG) + " " + port_prefix + novatel::LOG_OEM3_RGEB);
    }
    // OEM3 doesn't support BESTPOS/BESTVEL in the same way or MALIB doesn't support it.
    // Logging warning if requested.
    if (config_.enable_bestpos || config_.enable_bestvel) {
        RCLCPP_WARN(get_logger(), "BESTPOS/BESTVEL not supported/implemented for OEM3 mode");
    }

    if (config_.enable_gps_ephem) {
        sendCommand(std::string(novatel::CMD_LOG) + " " + port_prefix + novatel::LOG_OEM3_REPB + onchanged_suffix);
    } else {
        sendCommand(std::string(novatel::CMD_UNLOG) + " " + port_prefix + novatel::LOG_OEM3_REPB);
    }
    // OEM3 (Millennium) usually GPS only, maybe GLONASS? 
    // MALIB novatel.c seems to imply REPB is for GPS.
    
    if (config_.enable_ionutc) {
        sendCommand(std::string(novatel::CMD_LOG) + " " + port_prefix + novatel::LOG_OEM3_IONB + onchanged_suffix);
        sendCommand(std::string(novatel::CMD_LOG) + " " + port_prefix + novatel::LOG_OEM3_UTCB + onchanged_suffix);
    } else {
        sendCommand(std::string(novatel::CMD_UNLOG) + " " + port_prefix + novatel::LOG_OEM3_IONB);
        sendCommand(std::string(novatel::CMD_UNLOG) + " " + port_prefix + novatel::LOG_OEM3_UTCB);
    }
  }

  void configureReceiver() {
    RCLCPP_INFO(get_logger(), "Configuring receiver...");

    // 1. Unlog all messages first
        sendCommand(std::string(novatel::CMD_UNLOGALL) + " " + config_.receiver_port);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 2. Configure messages
    // Format: LOG [port] [message] ONTIME [interval]
    // Or:     LOG [port] [message] ONCHANGED
    
    std::string port_prefix = config_.receiver_port + " ";
    std::string ontime_suffix = " ONTIME " + std::to_string(1.0 / config_.publish_rate); // Use float string for period
    std::string onchanged_suffix = " ONCHANGED";

    if (config_.format == "oem3") {
        configureReceiverOem3(port_prefix, ontime_suffix, onchanged_suffix);
    } else {
        configureReceiverOem4(port_prefix, ontime_suffix, onchanged_suffix);
    }
    
    RCLCPP_INFO(get_logger(), "Configuration commands sent.");
  }
  void pollStream() {
    uint8_t buffer[novatel::READ_BUFFER_SIZE];
    const int bytes_read = strread(&stream_, buffer, sizeof(buffer));

    for (int i = 0; i < bytes_read; ++i) {
        int result = 0;
        if (config_.format == "oem3") {
            result = input_oem3(&raw_, buffer[i]);
        } else {
            result = input_oem4(&raw_, buffer[i]);
        }
        handleDecodeResult(result);
    }
  }

  void handleDecodeResult(int result) {
    switch (result) {
      case 1:  // Observation data
        publishObservations();
        break;
      case 2:  // Ephemeris data
        publishEphemerides();
        break;
      default:
        break;
    }
  }

  // ============================================================================
  // Observation Publishing
  // ============================================================================

  void publishObservations() {
    // Software filtering: if neither range nor rangecmp is enabled, don't publish
    if (!config_.enable_rangecmp && !config_.enable_range) {
        return;
    }

    if (raw_.obs.n <= 0) {
      return;
    }

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
        "Published observations: week=%d tow=%.3f n=%zu sats(G/R/E/J/C/S/U)=(%d/%d/%d/%d/%d/%d/%d)",
        week, tow, msg.observations.size(), sat_count.gps, sat_count.glo, sat_count.gal,
        sat_count.qzs, sat_count.bds, sat_count.sbs, sat_count.unknown);
  }

  void appendObservations(const obsd_t& obs,
                          std::vector<msg::GnssObservation>& observations) {
    for (int freq = 0; freq < NFREQ + NEXOBS; ++freq) {
      if (isEmptyObservation(obs, freq)) {
        continue;
      }
      observations.push_back(gnss_utils::obsToMsg(obs, freq));
    }
  }

  static bool isEmptyObservation(const obsd_t& obs, int freq) {
    return obs.P[freq] == 0.0 && obs.L[freq] == 0.0 && obs.D[freq] == 0.0 && obs.SNR[freq] == 0;
  }

  // ============================================================================
  // Ephemeris Publishing
  // ============================================================================

  void publishEphemerides() {
    bool has_new_ephemeris = false;

    std::vector<msg::GnssEphemeris> gnss_eph;
    std::vector<msg::GlonassEphemeris> glo_eph;

    // Collect Kepler ephemerides (GPS, Galileo, QZSS, BeiDou)
    for (int i = 0; i < raw_.nav.n; ++i) {
      const eph_t& eph = raw_.nav.eph[i];
      if (eph.sat == 0) continue;

      int prn = 0;
      int sys = satsys(eph.sat, &prn);
      
      // Filter based on configuration
      if (sys == SYS_GPS && !config_.enable_gps_ephem) continue;
      if (sys == SYS_GAL && !config_.enable_gal_ephem) continue;
      if (sys == SYS_CMP && !config_.enable_bds_ephem) continue;
      if (sys == SYS_QZS && !config_.enable_qzs_ephem) continue;
      if (sys == SYS_IRN && !config_.enable_navic_ephem) continue;

      if (sys == SYS_GLO) continue; // Handled below in geph

      EphemerisKey key{eph.sat, eph.iode, eph.iodc, eph.code};
      if (seen_ephemeris_.insert(key).second) {
        gnss_eph.push_back(gnss_utils::ephToMsg(eph));
        has_new_ephemeris = true;
      }
    }

    // Collect GLONASS ephemerides
    for (int i = 0; i < raw_.nav.ng; ++i) {
      const geph_t& geph = raw_.nav.geph[i];
      if (geph.sat == 0) continue;
      
      if (!config_.enable_glo_ephem) continue; // Filter GLONASS

      auto it = last_glo_iode_.find(geph.sat);
      if (it == last_glo_iode_.end() || it->second != geph.iode) {
        last_glo_iode_[geph.sat] = geph.iode;
        has_new_ephemeris = true;
      }
      glo_eph.push_back(gnss_utils::gephToMsg(geph));
    }

    // Publish on first call or when new ephemeris is available
    if (!has_new_ephemeris && !first_ephemeris_) {
      return;
    }
    first_ephemeris_ = false;

    msg::GnssEphemerides msg;
    msg.header.stamp = now();
    msg.gnss_ephemeris = std::move(gnss_eph);
    msg.glonass_ephemeris = std::move(glo_eph);
    eph_pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "Published ephemerides: GNSS=%zu GLO=%zu (new=%s)",
            msg.gnss_ephemeris.size(), msg.glonass_ephemeris.size(),
            has_new_ephemeris ? "yes" : "no");
  }

  // ============================================================================
  // Helper Types and Functions
  // ============================================================================

  struct SatelliteCount {
    int gps = 0;
    int glo = 0;
    int gal = 0;
    int qzs = 0;
    int bds = 0;
    int sbs = 0;
    int unknown = 0;
  };

  static void countSatellite(int sat, SatelliteCount& count) {
    int prn = 0;
    switch (satsys(sat, &prn)) {
      case SYS_GPS: ++count.gps; break;
      case SYS_GLO: ++count.glo; break;
      case SYS_GAL: ++count.gal; break;
      case SYS_QZS: ++count.qzs; break;
      case SYS_CMP: ++count.bds; break;
      case SYS_SBS: ++count.sbs; break;
      default: ++count.unknown; break;
    }
  }

  /// Key for tracking unique ephemeris entries
  struct EphemerisKey {
    int sat;
    int iode;
    int iodc;
    int code;

    bool operator==(const EphemerisKey& other) const {
      return sat == other.sat && iode == other.iode && iodc == other.iodc && code == other.code;
    }
  };

  struct EphemerisKeyHash {
    size_t operator()(const EphemerisKey& key) const {
      return static_cast<size_t>(key.sat) ^ (static_cast<size_t>(key.iode) << 16) ^
             (static_cast<size_t>(key.iodc) << 1) ^ (static_cast<size_t>(key.code) << 24);
    }
  };

  // ============================================================================
  // Member Variables
  // ============================================================================

  // Publishers
  rclcpp::Publisher<msg::GnssObservations>::SharedPtr obs_pub_;
  rclcpp::Publisher<msg::GnssEphemerides>::SharedPtr eph_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  NovatelConfig config_;

  // MALIB stream and decoder structures
  stream_t stream_{};
  raw_t raw_{};
  bool is_serial_connection_{false};

  // Ephemeris change tracking
  std::unordered_set<EphemerisKey, EphemerisKeyHash> seen_ephemeris_;
  std::unordered_map<int, int> last_glo_iode_;
  bool first_ephemeris_{true};
};

}  // namespace gnss_ros_standardization

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gnss_ros_standardization::NovatelDriverNode>());
  rclcpp::shutdown();
  return 0;
}
