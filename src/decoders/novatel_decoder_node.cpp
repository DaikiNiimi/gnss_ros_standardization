#include <rclcpp/rclcpp.hpp>

#include <chrono>
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

/// @brief ROS 2 node for decoding NovAtel receiver protocol messages
///
/// Decodes NovAtel binary format (OEM3/OEM4/OEM7) raw observations and navigation
/// data messages from NovAtel receivers and publishes them as standardized ROS messages.
///
/// Supported formats:
///   - oem4: OEM4/OEM6/OEM7 binary format (default)
///   - oem3: OEM3 binary format (legacy)
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
    declare_parameter<std::string>("stream_path", "serial:///dev/ttyUSB0:115200");
    declare_parameter<std::string>("format", "oem4");
    declare_parameter<std::string>("observation_topic", "/gnss/observation");
    declare_parameter<std::string>("ephemeris_topic", "/gnss/ephemeris");

    format_ = get_parameter("format").as_string();
    if (format_ != "oem3" && format_ != "oem4") {
        RCLCPP_WARN(get_logger(), "Unknown format '%s', defaulting to oem4", format_.c_str());
        format_ = "oem4";
    }
  }

  void initializePublishers() {
    obs_pub_ = create_publisher<msg::GnssObservations>(get_parameter("observation_topic").as_string(), 10);
    eph_pub_ = create_publisher<msg::GnssEphemerides>(get_parameter("ephemeris_topic").as_string(), rclcpp::QoS(100).transient_local());
  }

  void initializeDecoder() {
    int strfmt = STRFMT_OEM4;
    if (format_ == "oem3") {
        strfmt = STRFMT_OEM3;
    }
    
    // Default to OEM4 format (covers OEM4/OEM6/OEM7)
    if (init_raw(&raw_, strfmt) != 1) {
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

    // Match stream type from URI prefix
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

    // MALIB serial path format: device name without /dev/ prefix
    if (stream_type == STR_SERIAL && path.rfind("/dev/", 0) == 0) {
      path.erase(0, 5);
      RCLCPP_DEBUG(get_logger(), "Adjusted serial path for MALIB: %s", path.c_str());
    }

    if (!stropen(&stream_, stream_type, STR_MODE_R, path.c_str())) {
      RCLCPP_ERROR(get_logger(), "Failed to open stream: %s", stream_path.c_str());
      throw std::runtime_error("stropen failed");
    }

    RCLCPP_INFO(get_logger(), "Stream opened: %s", stream_path.c_str());
  }

  void pollStream() {
    uint8_t buffer[novatel::READ_BUFFER_SIZE];
    const int bytes_read = strread(&stream_, buffer, sizeof(buffer));

    for (int i = 0; i < bytes_read; ++i) {
      int result = 0;
      if (format_ == "oem3") {
        result = input_oem3(&raw_, buffer[i]);
      } else {
        result = input_oem4(&raw_, buffer[i]);
      }
      handleDecodeResult(result);
    }
  }

  // ============================================================================
  // Message Handling
  // ============================================================================

  void handleDecodeResult(int result) {
    switch (result) {
      case 1:  // Observation data
        publishObservations();
        break;
      case 2:  // Ephemeris data
        publishEphemerides();
        break;
      case 3:  // SBAS message (not implemented)
        break;
      case 9:  // Ionosphere/UTC parameters (not implemented)
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
    msg.header.frame_id = "gnss_receiver";
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
        "Published observations: week=%d tow=%.3f n=%zu sats(G/R/E/J/C/I/S/U)=(%d/%d/%d/%d/%d/%d/%d/%d)",
        week, tow, msg.observations.size(), sat_count.gps, sat_count.glo, sat_count.gal,
        sat_count.qzs, sat_count.bds, sat_count.irn, sat_count.sbs, sat_count.unknown);
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
        glo_eph.push_back(gnss_utils::gephToMsg(geph));
      }
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
    int irn = 0;
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
      case SYS_IRN: ++count.irn; break;
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

  // Configuration
  std::string format_{"oem4"};

  // Publishers
  rclcpp::Publisher<msg::GnssObservations>::SharedPtr obs_pub_;
  rclcpp::Publisher<msg::GnssEphemerides>::SharedPtr eph_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // MALIB stream and decoder structures
  stream_t stream_{};
  raw_t raw_{};

  // Ephemeris change tracking
  std::unordered_set<EphemerisKey, EphemerisKeyHash> seen_ephemeris_;
  std::unordered_map<int, int> last_glo_iode_;
  bool first_ephemeris_{true};
};

}  // namespace gnss_ros_standardization

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gnss_ros_standardization::NovatelDecoderNode>());
  rclcpp::shutdown();
  return 0;
}
