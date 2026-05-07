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
constexpr int ESF_INS_OFFSET_ITOW      = 8;
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

/// @brief ROS 2 node for decoding u-blox UBX protocol messages
///
/// Decodes UBX-RXM-RAWX (raw observations) and UBX-RXM-SFRBX (navigation data)
/// messages from u-blox receivers and publishes them as standardized ROS messages.
/// Also parses NMEA sentences for GnssSolution and UBX-ESF-INS / UBX-NAV-ATT for IMU data.
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
    declare_parameter<std::string>("observation_topic", "/gnss/observation");
    declare_parameter<std::string>("ephemeris_topic", "/gnss/ephemeris");
    declare_parameter<std::string>("imu_raw_topic", "/gnss/imu/data_raw");
    declare_parameter<std::string>("imu_attitude_topic", "/gnss/imu/attitude");

    frame_id_ = get_parameter("frame_id").as_string();
  }

  void initializePublishers() {
    obs_pub_     = create_publisher<msg::GnssObservations>(get_parameter("observation_topic").as_string(), 10);
    eph_pub_     = create_publisher<msg::GnssEphemerides>(get_parameter("ephemeris_topic").as_string(), rclcpp::QoS(100).transient_local());
    sol_pub_     = create_publisher<msg::GnssSolution>("/gnss/nmea_solution", 10);
    imu_raw_pub_ = create_publisher<sensor_msgs::msg::Imu>(get_parameter("imu_raw_topic").as_string(), 10);
    imu_attitude_pub_     = create_publisher<sensor_msgs::msg::Imu>(get_parameter("imu_attitude_topic").as_string(), 10);
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
    for (const auto& def : ubx::kStreamTypes) {
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

    std::vector<char> path_buf(path.begin(), path.end());
    path_buf.push_back('\0');

    if (!stropen(&stream_, stream_type, STR_MODE_R, path_buf.data())) {
      RCLCPP_ERROR(get_logger(), "Failed to open stream: %s", stream_path.c_str());
      throw std::runtime_error("stropen failed");
    }

    RCLCPP_INFO(get_logger(), "Stream opened: %s", stream_path.c_str());
  }

  void pollStream() {
    uint8_t buffer[ubx::READ_BUFFER_SIZE];
    const int bytes_read = strread(&stream_, buffer, sizeof(buffer));

    if (bytes_read > 0) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Stream: read %d bytes", bytes_read);
    }

    for (int i = 0; i < bytes_read; ++i) {
      uint8_t byte = buffer[i];

      // Feed RTKLIB for UBX binary parsing (obs/eph)
      const int result = input_ubx(&raw_, &rtcm_, byte);
      handleDecodeResult(result);

      // Feed parallel UBX mini-framer for ESF-INS and NAV-ATT
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
        } else if (nmea_buffer_.size() > 256) {
          nmea_buffer_.clear();
        }
      }
    }
  }

  // ============================================================================
  // UBX Mini-Framer (parallel to RTKLIB, for ESF-INS and NAV-ATT)
  // ============================================================================

  void parseUbxByte(uint8_t byte) {
    switch (ubx_state_) {
      case 0:
        if (byte == ubx::SYNC1) ubx_state_ = 1;
        break;
      case 1:
        ubx_state_ = (byte == ubx::SYNC2) ? 2 : (byte == ubx::SYNC1 ? 1 : 0);
        break;
      case 2:
        ubx_cls_   = byte;
        ubx_state_ = 3;
        break;
      case 3:
        ubx_id_    = byte;
        ubx_state_ = 4;
        break;
      case 4:
        ubx_len_   = byte;
        ubx_state_ = 5;
        break;
      case 5:
        ubx_len_  |= static_cast<uint16_t>(byte << 8);
        ubx_payload_.clear();
        ubx_pos_   = 0;
        ubx_state_ = (ubx_len_ == 0) ? 7 : 6;
        break;
      case 6:
        ubx_payload_.push_back(byte);
        if (++ubx_pos_ >= ubx_len_) ubx_state_ = 7;
        break;
      case 7:  // CK_A (skip)
        ubx_state_ = 8;
        break;
      case 8:  // CK_B (skip), frame complete
        handleUbxFrame();
        ubx_state_ = 0;
        break;
      default:
        ubx_state_ = 0;
    }
  }

  void handleUbxFrame() {
    if (ubx_cls_ == ubx::CLASS_ESF && ubx_id_ == ubx::ID_ESF_INS) {
      handleEsfIns();
    } else if (ubx_cls_ == ubx::CLASS_NAV && ubx_id_ == ubx::ID_NAV_ATT) {
      handleNavAtt();
    } else {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                           "UBX: received unhandled frame class=0x%02x id=0x%02x len=%d",
                           ubx_cls_, ubx_id_, ubx_len_);
    }
  }

  void handleEsfIns() {
    if (static_cast<int>(ubx_payload_.size()) < ESF_INS_MIN_LEN) return;

    // Validity bits in bitfield0 (bytes 0-3)
    uint32_t bitfield0 = 0;
    std::memcpy(&bitfield0, ubx_payload_.data() + ESF_INS_OFFSET_BITFIELD, 4);
    const bool ang_valid = (bitfield0 & (0x7u << 8)) == (0x7u << 8);  // bits 8,9,10
    const bool acc_valid = (bitfield0 & (0x7u << 11)) == (0x7u << 11); // bits 11,12,13

    int32_t xAngRate_raw = 0, yAngRate_raw = 0, zAngRate_raw = 0;
    int32_t xAccel_raw = 0, yAccel_raw = 0, zAccel_raw = 0;
    std::memcpy(&xAngRate_raw, ubx_payload_.data() + ESF_INS_OFFSET_XANGRATE, 4);
    std::memcpy(&yAngRate_raw, ubx_payload_.data() + ESF_INS_OFFSET_YANGRATE, 4);
    std::memcpy(&zAngRate_raw, ubx_payload_.data() + ESF_INS_OFFSET_ZANGRATE, 4);
    std::memcpy(&xAccel_raw,   ubx_payload_.data() + ESF_INS_OFFSET_XACCEL,   4);
    std::memcpy(&yAccel_raw,   ubx_payload_.data() + ESF_INS_OFFSET_YACCEL,   4);
    std::memcpy(&zAccel_raw,   ubx_payload_.data() + ESF_INS_OFFSET_ZACCEL,   4);

    // Scale: angular rate 1e-3 deg/s → rad/s; acceleration 1e-2 m/s²
    const double deg2rad = M_PI / 180.0;
    last_ang_x_ = xAngRate_raw * 1e-3 * deg2rad;
    last_ang_y_ = yAngRate_raw * 1e-3 * deg2rad;
    last_ang_z_ = zAngRate_raw * 1e-3 * deg2rad;
    last_acc_x_ = xAccel_raw   * 1e-2;
    last_acc_y_ = yAccel_raw   * 1e-2;
    last_acc_z_ = zAccel_raw   * 1e-2;

    sensor_msgs::msg::Imu imu;
    imu.header.stamp    = now();
    imu.header.frame_id = frame_id_;

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
    if (static_cast<int>(ubx_payload_.size()) < NAV_ATT_MIN_LEN) return;

    int32_t roll_raw = 0, pitch_raw = 0, heading_raw = 0;
    uint32_t acc_roll = 0, acc_pitch = 0, acc_heading = 0;
    std::memcpy(&roll_raw,    ubx_payload_.data() + NAV_ATT_OFFSET_ROLL,       4);
    std::memcpy(&pitch_raw,   ubx_payload_.data() + NAV_ATT_OFFSET_PITCH,      4);
    std::memcpy(&heading_raw, ubx_payload_.data() + NAV_ATT_OFFSET_HEADING,    4);
    std::memcpy(&acc_roll,    ubx_payload_.data() + NAV_ATT_OFFSET_ACCROLL,    4);
    std::memcpy(&acc_pitch,   ubx_payload_.data() + NAV_ATT_OFFSET_ACCPITCH,   4);
    std::memcpy(&acc_heading, ubx_payload_.data() + NAV_ATT_OFFSET_ACCHEADING, 4);

    // Scale: 1e-5 degrees → radians
    const double scale = 1e-5 * M_PI / 180.0;
    const double roll    = roll_raw    * scale;
    const double pitch   = pitch_raw   * scale;
    const double heading = heading_raw * scale;  // 0..2π, clockwise from North

    const double sig_roll    = acc_roll    * scale;
    const double sig_pitch   = acc_pitch   * scale;
    const double sig_heading = acc_heading * scale;

    sensor_msgs::msg::Imu imu;
    imu.header.stamp    = now();
    imu.header.frame_id = frame_id_;

    imu.orientation = ins::eulerToQuaternion(roll, pitch, heading);
    auto ori_cov = ins::makeDiagCovariance(sig_roll, sig_pitch, sig_heading);
    std::copy(ori_cov.begin(), ori_cov.end(), imu.orientation_covariance.begin());

    // Populate angular velocity and acceleration from latest ESF-INS if available
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
      case 0:  break; // No message complete
      default:
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "RTKLIB: input_ubx returned result=%d", result);
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
      current_solution_.time_tow = tow;
    }

    current_solution_.header.stamp = now();
    current_solution_.header.frame_id = frame_id_;

    if (current_solution_.status == msg::GnssSolution::STATUS_FIX ||
        current_solution_.status == msg::GnssSolution::STATUS_FLOAT ||
        current_solution_.status == msg::GnssSolution::STATUS_SINGLE ||
        current_solution_.status == msg::GnssSolution::STATUS_DGPS) {

      if (!has_local_origin_) {
        local_origin_ecef_[0] = current_solution_.pos_ecef.x;
        local_origin_ecef_[1] = current_solution_.pos_ecef.y;
        local_origin_ecef_[2] = current_solution_.pos_ecef.z;
        ecef2pos(local_origin_ecef_, local_origin_pos_);
        has_local_origin_ = true;
        RCLCPP_INFO(get_logger(), "Set local ENU origin: lat=%.6f, lon=%.6f, alt=%.2f",
                    local_origin_pos_[0] * (180.0/M_PI), local_origin_pos_[1] * (180.0/M_PI), local_origin_pos_[2]);
      }

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

  // ============================================================================
  // Observation Publishing
  // ============================================================================

  void publishObservations() {
    if (raw_.obs.n <= 0) return;

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

    RCLCPP_INFO(
        get_logger(),
        "Published obs: week=%d tow=%.3f n=%zu sats(G/R/E/J/C/I/S/U)=(%d/%d/%d/%d/%d/%d/%d/%d)",
        week, tow, msg.observations.size(), sat_count.gps, sat_count.glo, sat_count.gal,
        sat_count.qzs, sat_count.bds, sat_count.irn, sat_count.sbs, sat_count.unknown);
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

    RCLCPP_DEBUG(get_logger(), "Published ephemerides: GNSS=%zu GLO=%zu (new=%s)",
                msg.gnss_ephemeris.size(), msg.glonass_ephemeris.size(), has_new ? "yes" : "no");
  }

  // ============================================================================
  // Helper Types
  // ============================================================================

  struct SatelliteCount {
    int gps=0, glo=0, gal=0, qzs=0, bds=0, irn=0, sbs=0, unknown=0;
  };

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

  rclcpp::Publisher<msg::GnssObservations>::SharedPtr   obs_pub_;
  rclcpp::Publisher<msg::GnssEphemerides>::SharedPtr    eph_pub_;
  rclcpp::Publisher<msg::GnssSolution>::SharedPtr       sol_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr   imu_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr   imu_attitude_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  stream_t stream_{};
  raw_t    raw_{};
  rtcm_t   rtcm_{};

  // NMEA parsing state
  gnss_utils::NmeaParser nmea_parser_;
  std::string            nmea_buffer_;
  msg::GnssSolution      current_solution_;

  // ENU origin
  bool   has_local_origin_{false};
  double local_origin_ecef_[3]{0.0};
  double local_origin_pos_[3]{0.0};

  // UBX mini-framer state
  int                  ubx_state_{0};
  uint8_t              ubx_cls_{0};
  uint8_t              ubx_id_{0};
  uint16_t             ubx_len_{0};
  uint16_t             ubx_pos_{0};
  std::vector<uint8_t> ubx_payload_;

  // Latest ESF-INS values (merged into NAV-ATT message)
  double last_ang_x_{0.0}, last_ang_y_{0.0}, last_ang_z_{0.0};
  double last_acc_x_{0.0}, last_acc_y_{0.0}, last_acc_z_{0.0};

  // Ephemeris dedup
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
