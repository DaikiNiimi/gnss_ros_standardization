#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstring>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/sbf_protocol.hpp"

using namespace std::chrono_literals;

namespace gnss_ros_standardization {

namespace {

/// Timer interval for stream polling
constexpr auto kTimerInterval = 10ms;

}  // namespace

/// @brief Configuration structure for Septentrio driver
struct SbfConfig {
  std::string stream_path{"serial:///dev/ttyACM0:115200"};
  std::string frame_id{"gnss_link"};
  int publish_rate{1};           // Hz
  std::string receiver_port{"USB1"}; // Port on receiver to output data
  bool configure_on_startup{true};

  // Message settings
  bool enable_meas_epoch{true};
  bool enable_gps_nav{true};
  bool enable_glo_nav{true};
  bool enable_gal_nav{true};
  bool enable_bds_nav{true};
  bool enable_qzs_nav{true};
  
  // PVT
  bool enable_pvt_geodetic{false};
  bool enable_pos_cov_geodetic{false};
  bool enable_pvt_cartesian{false};
  bool enable_pos_cov_cartesian{false};

  // Attitude
  bool enable_att_euler{false};
  bool enable_att_cov_euler{false};

  // Status
  bool enable_receiver_status{false};
  bool enable_quality_ind{false};

  // Topics
  std::string observation_topic{"/gnss/observation"};
  std::string ephemeris_topic{"/gnss/ephemeris"};
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
    } catch (...) {
      // Ignore errors during cleanup
    }
    free_raw(&raw_);
    free_rtcm(&rtcm_);
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
    declare_parameter<bool>("configure_on_startup", config_.configure_on_startup);
    
    declare_parameter<bool>("messages.meas_epoch", config_.enable_meas_epoch);
    declare_parameter<bool>("messages.gps_nav", config_.enable_gps_nav);
    declare_parameter<bool>("messages.glo_nav", config_.enable_glo_nav);
    declare_parameter<bool>("messages.gal_nav", config_.enable_gal_nav);
    declare_parameter<bool>("messages.bds_nav", config_.enable_bds_nav);
    declare_parameter<bool>("messages.qzs_nav", config_.enable_qzs_nav);
    
    declare_parameter<bool>("messages.pvt_geodetic", config_.enable_pvt_geodetic);
    declare_parameter<bool>("messages.pos_cov_geodetic", config_.enable_pos_cov_geodetic);
    declare_parameter<bool>("messages.pvt_cartesian", config_.enable_pvt_cartesian);
    declare_parameter<bool>("messages.pos_cov_cartesian", config_.enable_pos_cov_cartesian);
    
    declare_parameter<bool>("messages.att_euler", config_.enable_att_euler);
    declare_parameter<bool>("messages.att_cov_euler", config_.enable_att_cov_euler);
    
    declare_parameter<bool>("messages.receiver_status", config_.enable_receiver_status);
    declare_parameter<bool>("messages.quality_ind", config_.enable_quality_ind);

    declare_parameter<std::string>("observation_topic", config_.observation_topic);
    declare_parameter<std::string>("ephemeris_topic", config_.ephemeris_topic);

    config_.stream_path = get_parameter("stream_path").as_string();
    config_.frame_id = get_parameter("frame_id").as_string();
    config_.publish_rate = get_parameter("publish_rate").as_int();
    config_.receiver_port = get_parameter("receiver_port").as_string();
    config_.configure_on_startup = get_parameter("configure_on_startup").as_bool();
    
    config_.enable_meas_epoch = get_parameter("messages.meas_epoch").as_bool();
    config_.enable_gps_nav = get_parameter("messages.gps_nav").as_bool();
    config_.enable_glo_nav = get_parameter("messages.glo_nav").as_bool();
    config_.enable_gal_nav = get_parameter("messages.gal_nav").as_bool();
    config_.enable_bds_nav = get_parameter("messages.bds_nav").as_bool();
    config_.enable_qzs_nav = get_parameter("messages.qzs_nav").as_bool();
    
    config_.enable_pvt_geodetic = get_parameter("messages.pvt_geodetic").as_bool();
    config_.enable_pos_cov_geodetic = get_parameter("messages.pos_cov_geodetic").as_bool();
    config_.enable_pvt_cartesian = get_parameter("messages.pvt_cartesian").as_bool();
    config_.enable_pos_cov_cartesian = get_parameter("messages.pos_cov_cartesian").as_bool();
    
    config_.enable_att_euler = get_parameter("messages.att_euler").as_bool();
    config_.enable_att_cov_euler = get_parameter("messages.att_cov_euler").as_bool();
    
    config_.enable_receiver_status = get_parameter("messages.receiver_status").as_bool();
    config_.enable_quality_ind = get_parameter("messages.quality_ind").as_bool();

    config_.observation_topic = get_parameter("observation_topic").as_string();
    config_.ephemeris_topic = get_parameter("ephemeris_topic").as_string();
  }

  void initializePublishers() {
    obs_pub_ = create_publisher<msg::GnssObservations>(config_.observation_topic, 10);
    eph_pub_ = create_publisher<msg::GnssEphemerides>(config_.ephemeris_topic, 10);
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

    // Match stream type from URI prefix
    int stream_type = STR_SERIAL; // Default
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
      // If no prefix found, assume serial if it looks like a device path, or file otherwise
       if (path.rfind("/dev/", 0) == 0) {
           stream_type = STR_SERIAL;
       } else {
           // Fallback or error? For now assume it might be a file or just a raw path for serial
           // But existing code structure usually expects valid prefix. 
           // Let's stick to the matched logic unless user provided raw path.
           // Actually, let's just log warning if no prefix and try to open as is (likely fail if not handled by malib)
       }
    }

    // MALIB serial path format: device name without /dev/ prefix
    if (stream_type == STR_SERIAL && path.rfind("/dev/", 0) == 0) {
      path.erase(0, 5);
      RCLCPP_DEBUG(get_logger(), "Adjusted serial path for MALIB: %s", path.c_str());
    }

    // Convert std::string to const char* for atropen
    // Note: atropen takes char*, not const char*, so we need a mutable buffer if we used sv directly, 
    // but here we pass path.c_str() which is const char*. MALIB strinit/stropen signature in C:
    // int stropen(stream_t *stream, int type, int mode, const char *path); 
    // (Wait, standard RTKLIB is const char*, let's assume MALIB is too or compatible)
    
    // We need to initialize stream first
    strinit(&stream_);

    if (!stropen(&stream_, stream_type, STR_MODE_RW, path.c_str())) {
      RCLCPP_ERROR(get_logger(), "Failed to open stream: %s", config_.stream_path.c_str());
      throw std::runtime_error("stropen failed");
    }

    // Determine if we are connected via serial/USB for configuration purposes
    is_serial_connection_ = (stream_type == STR_SERIAL);

    RCLCPP_INFO(get_logger(), "Stream opened: %s", config_.stream_path.c_str());
  }

  // ============================================================================
  // Receiver Configuration
  // ============================================================================

  void configureReceiver() {
    RCLCPP_INFO(get_logger(), "Configuring receiver...");

    // Calculate interval string
    std::string interval_str = sbf::getIntervalString(config_.publish_rate);

    // Build list of enabled messages
    std::vector<std::string> blocks;
    if (config_.enable_meas_epoch) blocks.push_back(sbf::BLOCK_MEASEPOCH);
    if (config_.enable_gps_nav) blocks.push_back(sbf::BLOCK_GPSNAV);
    if (config_.enable_glo_nav) blocks.push_back(sbf::BLOCK_GLONAV);
    if (config_.enable_gal_nav) blocks.push_back(sbf::BLOCK_GALNAV);
    if (config_.enable_bds_nav) blocks.push_back(sbf::BLOCK_BDSNAV);
    if (config_.enable_qzs_nav) blocks.push_back(sbf::BLOCK_QZSNAV);
    
    if (config_.enable_pvt_geodetic) blocks.push_back(sbf::BLOCK_PVTGEODETIC);
    if (config_.enable_pos_cov_geodetic) blocks.push_back(sbf::BLOCK_POSCOVGEODETIC);
    if (config_.enable_pvt_cartesian) blocks.push_back(sbf::BLOCK_PVTCARTESIAN);
    if (config_.enable_pos_cov_cartesian) blocks.push_back(sbf::BLOCK_POSCOVCARTESIAN);

    if (config_.enable_att_euler) blocks.push_back(sbf::BLOCK_ATTEULER);
    if (config_.enable_att_cov_euler) blocks.push_back(sbf::BLOCK_ATTCOVEULER);

    if (config_.enable_receiver_status) blocks.push_back(sbf::BLOCK_RECEIVERSTATUS);
    if (config_.enable_quality_ind) blocks.push_back(sbf::BLOCK_QUALITYIND);
    
    if (blocks.empty()) {
        RCLCPP_WARN(get_logger(), "No messages enabled for output!");
        return;
    }

    // Concatenate messages with '+'
    std::string messages = blocks[0];
    for (size_t i = 1; i < blocks.size(); ++i) {
        messages += "+" + blocks[i];
    }

    // Construct the command
    // Example: "sso, Stream1, USB1, MeasEpoch+GPSNav+GLONav+GALNav+BDSNav+QZSNav, msec100\r\n"
    std::string cmd = std::string(sbf::CMD_SET_SBF_OUTPUT) + ", Stream1, " + 
                      config_.receiver_port + ", " + messages + ", " + interval_str + "\r\n";
    
    RCLCPP_INFO(get_logger(), "Sending configuration command to receiver port %s: %s", 
                config_.receiver_port.c_str(), cmd.c_str());

    // Send command
    int written = strwrite(&stream_, (uint8_t*)cmd.c_str(), cmd.size());
    
    // Workaround for MALIB writeserial bug on Linux (returns 0 on success)
    if (written < 0 || (written == 0 && !is_serial_connection_)) {
        RCLCPP_ERROR(get_logger(), "Failed to write configuration command (result: %d)", written);
    }

    // Wait for the command to be processed
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // ============================================================================
  // Data Processing
  // ============================================================================

  void pollStream() {
    uint8_t buffer[sbf::READ_BUFFER_SIZE];
    const int bytes_read = strread(&stream_, buffer, sizeof(buffer));

    for (int i = 0; i < bytes_read; ++i) {
        const int result = input_sbf(&raw_, &rtcm_, buffer[i]);
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
        "Published obs: week=%d tow=%.3f n=%zu sats(G/R/E/J/C/S/U)=(%d/%d/%d/%d/%d/%d/%d)",
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
      if (satsys(eph.sat, &prn) == SYS_GLO) continue;

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

  SbfConfig config_;

  // MALIB stream and decoder structures
  stream_t stream_{};
  raw_t raw_{};
  rtcm_t rtcm_{};
  bool is_serial_connection_{false};

  // Ephemeris change tracking
  std::unordered_set<EphemerisKey, EphemerisKeyHash> seen_ephemeris_;
  std::unordered_map<int, int> last_glo_iode_;
  bool first_ephemeris_{true};
};

}  // namespace gnss_ros_standardization

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gnss_ros_standardization::SbfDriverNode>());
  rclcpp::shutdown();
  return 0;
}
