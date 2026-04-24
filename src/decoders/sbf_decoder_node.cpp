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
constexpr int ATT_EULER_MIN_LEN        = 22;  // body must have at least 22 bytes

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
    declare_parameter<std::string>("solution_topic", "/gnss/nmea_solution");
    declare_parameter<std::string>("imu_topic", "/gnss/imu/data");

    frame_id_ = get_parameter("frame_id").as_string();
  }

  void initializePublishers() {
    obs_pub_ = create_publisher<msg::GnssObservations>(get_parameter("observation_topic").as_string(), 10);
    eph_pub_ = create_publisher<msg::GnssEphemerides>(get_parameter("ephemeris_topic").as_string(), rclcpp::QoS(100).transient_local());
    sol_pub_ = create_publisher<msg::GnssSolution>(get_parameter("solution_topic").as_string(), 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(get_parameter("imu_topic").as_string(), 10);
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
    if (sbf_id_ == sbf::ID_ATTEULER) handleAttEuler();
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

    imu_pub_->publish(imu);
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
      if (seen_ephemeris_.insert(key).second) { gnss_eph.push_back(gnss_utils::ephToMsg(eph)); has_new = true; }
    }
    for (int i = 0; i < raw_.nav.ng; ++i) {
      const geph_t& geph = raw_.nav.geph[i];
      if (geph.sat == 0) continue;
      auto it = last_glo_iode_.find(geph.sat);
      if (it == last_glo_iode_.end() || it->second != geph.iode) {
        last_glo_iode_[geph.sat] = geph.iode;
        glo_eph.push_back(gnss_utils::gephToMsg(geph)); has_new = true;
      }
    }

    if (!has_new && !first_ephemeris_) return;
    first_ephemeris_ = false;

    msg::GnssEphemerides msg;
    msg.header.stamp = now();
    msg.gnss_ephemeris    = std::move(gnss_eph);
    msg.glonass_ephemeris = std::move(glo_eph);
    eph_pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "Published ephemerides: GNSS=%zu GLO=%zu (new=%s)",
      msg.gnss_ephemeris.size(), msg.glonass_ephemeris.size(), has_new ? "yes" : "no");
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
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr  imu_pub_;
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

  // Ephemeris dedup
  std::unordered_set<EphemerisKey, EphemerisKeyHash> seen_ephemeris_;
  std::unordered_map<int, int> last_glo_iode_;
  bool first_ephemeris_{true};
};

}  // namespace gnss_ros_standardization

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gnss_ros_standardization::SbfDecoderNode>());
  rclcpp::shutdown();
  return 0;
}
