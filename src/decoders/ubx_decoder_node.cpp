#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "gnss_ros_standardization/gnss_utils.hpp"

using namespace std::chrono_literals;

namespace gnss_ros_standardization {

namespace {

/// Stream type definition for URI prefix matching
struct StreamTypeDef {
  std::string_view prefix;
  int type;
};

/// Supported stream type prefixes
constexpr StreamTypeDef kStreamTypes[] = {
    {"tcpcli://", STR_TCPCLI},
    {"serial://", STR_SERIAL},
    {"ntrip://", STR_NTRIPCLI},
    {"file://", STR_FILE},
};

/// Buffer size for stream reading
constexpr size_t kReadBufferSize = 4096;

/// Timer interval for stream polling
constexpr auto kTimerInterval = 10ms;

}  // namespace

/// @brief ROS 2 node for decoding u-blox UBX protocol messages
///
/// Decodes UBX-RXM-RAWX (raw observations) and UBX-RXM-SFRBX (navigation data)
/// messages from u-blox receivers and publishes them as standardized ROS messages.
class UbxDecoderNode : public rclcpp::Node {
 public:
  UbxDecoderNode() : Node("ubx_decoder_node") {
    initializeParameters();
    initializePublishers();
    initializeDecoder();
    openStream();
    startPolling();

    RCLCPP_INFO(get_logger(), "UBX Decoder initialized");
  }

  ~UbxDecoderNode() override {
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
    declare_parameter<std::string>("stream_path", "serial:///dev/ttyACM0:115200");
    declare_parameter<std::string>("frame_id", "gnss_link");
    
    frame_id_ = get_parameter("frame_id").as_string();
  }

  void initializePublishers() {
    obs_pub_ = create_publisher<msg::GnssObservations>("/gnss/observation", 10);
    eph_pub_ = create_publisher<msg::GnssEphemerides>("/gnss/ephemeris", 10);
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
    timer_ = create_wall_timer(kTimerInterval, std::bind(&UbxDecoderNode::pollStream, this));
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
    for (const auto& def : kStreamTypes) {
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
    // e.g., "ttyACM0:115200" instead of "/dev/ttyACM0:115200"
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
    uint8_t buffer[kReadBufferSize];
    const int bytes_read = strread(&stream_, buffer, sizeof(buffer));

    for (int i = 0; i < bytes_read; ++i) {
      const int result = input_ubx(&raw_, &rtcm_, buffer[i]);
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
    msg.header.frame_id = frame_id_;
    msg.week = static_cast<uint16_t>(week);
    msg.tow = tow;

    SatelliteCount sat_count{};

    for (int i = 0; i < raw_.obs.n; ++i) {
      const obsd_t& obs = raw_.obs.data[i];
      countSatellite(obs.sat, sat_count);
      appendObservations(obs, msg.observations);
    }

    obs_pub_->publish(msg);

    RCLCPP_DEBUG(
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

    RCLCPP_DEBUG(get_logger(), "Published ephemerides: GNSS=%zu GLO=%zu (new=%s)",
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

  // Parameters
  std::string frame_id_;

  // MALIB stream and decoder structures
  stream_t stream_{};
  raw_t raw_{};
  rtcm_t rtcm_{};

  // Ephemeris change tracking
  std::unordered_set<EphemerisKey, EphemerisKeyHash> seen_ephemeris_;
  std::unordered_map<int, int> last_glo_iode_;
  bool first_ephemeris_{true};
};

}  // namespace gnss_ros_standardization

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gnss_ros_standardization::UbxDecoderNode>());
  rclcpp::shutdown();
  return 0;
}
