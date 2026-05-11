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
#include "gnss_ros_standardization/novatel_imu_scales.hpp"
#include "gnss_ros_standardization/novatel_protocol.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"

using namespace std::chrono_literals;
namespace ins = gnss_ros_standardization::ins_utils;

namespace gnss_ros_standardization {

namespace {

constexpr auto kTimerInterval  = 10ms;
constexpr size_t kNmeaMaxLineLen = 256;

}  // namespace

/// @brief Configuration structure for NovAtel driver
struct NovatelConfig {
  std::string stream_path{"serial:///dev/ttyUSB0:115200"};
  std::string frame_id{"gnss_link"};
  int publish_rate{1};
  std::string receiver_port{"USB1"};
  std::string format{"oem4"};
  bool configure_on_startup{true};

  // Observation / PVT
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

  // NMEA message settings (for GnssSolution output)
  bool enable_nmea_gpgga{true};
  bool enable_nmea_gprmc{true};
  bool enable_nmea_gpgsa{true};
  bool enable_nmea_gpgst{true};

  // IMU messages (requires NovAtel receiver with IMU connection)
  bool enable_rawimusx{true};      // RAWIMUSX: raw uncalibrated IMU → /gnss/imu/data_raw
  bool enable_corrimudata{false};  // CORRIMUDATA: SPAN-corrected IMU → /gnss/imu/data (SPAN required)

  // IMU scale override (used when IMU type is unknown to the table; both must be non-zero to apply)
  double imu_scale_override_accel{0.0};  // counts → m/s per sample
  double imu_scale_override_gyro{0.0};   // counts → rad per sample

  // Topics
  std::string observation_topic{"/gnss/observation"};
  std::string ephemeris_topic{"/gnss/ephemeris"};
  std::string solution_topic{"/gnss/nmea_solution"};
  std::string imu_topic{"/gnss/imu/data"};         // CORRIMUDATA (calibrated)
  std::string imu_raw_topic{"/gnss/imu/data_raw"}; // RAWIMUSX (uncalibrated)

  // Ephemeris snapshot behavior
  double ephemeris_snapshot_period_s{30.0};
  double ephemeris_max_age_s{7200.0};
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
    } catch (...) {}
    free_raw(&raw_);
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
    declare_parameter<std::string>("format",               config_.format);
    declare_parameter<bool>       ("configure_on_startup", config_.configure_on_startup);

    declare_parameter<bool>("messages.rangecmp",    config_.enable_rangecmp);
    declare_parameter<bool>("messages.range",       config_.enable_range);
    declare_parameter<bool>("messages.bestpos",     config_.enable_bestpos);
    declare_parameter<bool>("messages.bestvel",     config_.enable_bestvel);
    declare_parameter<bool>("messages.gps_ephem",   config_.enable_gps_ephem);
    declare_parameter<bool>("messages.glo_ephem",   config_.enable_glo_ephem);
    declare_parameter<bool>("messages.gal_ephem",   config_.enable_gal_ephem);
    declare_parameter<bool>("messages.bds_ephem",   config_.enable_bds_ephem);
    declare_parameter<bool>("messages.qzs_ephem",   config_.enable_qzs_ephem);
    declare_parameter<bool>("messages.navic_ephem", config_.enable_navic_ephem);
    declare_parameter<bool>("messages.ionutc",      config_.enable_ionutc);
    declare_parameter<bool>("messages.nmea_gpgga",  config_.enable_nmea_gpgga);
    declare_parameter<bool>("messages.nmea_gprmc",  config_.enable_nmea_gprmc);
    declare_parameter<bool>("messages.nmea_gpgsa",  config_.enable_nmea_gpgsa);
    declare_parameter<bool>("messages.nmea_gpgst",  config_.enable_nmea_gpgst);
    declare_parameter<bool>("messages.rawimusx",    config_.enable_rawimusx);
    declare_parameter<bool>("messages.corrimudata", config_.enable_corrimudata);
    declare_parameter<double>("imu_scale_override.accel", config_.imu_scale_override_accel);
    declare_parameter<double>("imu_scale_override.gyro",  config_.imu_scale_override_gyro);

    declare_parameter<std::string>("observation_topic", config_.observation_topic);
    declare_parameter<std::string>("ephemeris_topic",   config_.ephemeris_topic);
    declare_parameter<std::string>("solution_topic",    config_.solution_topic);
    declare_parameter<std::string>("imu_topic",         config_.imu_topic);
    declare_parameter<std::string>("imu_raw_topic",     config_.imu_raw_topic);

    declare_parameter<double>("ephemeris.snapshot_period_s", config_.ephemeris_snapshot_period_s);
    declare_parameter<double>("ephemeris.max_age_s",         config_.ephemeris_max_age_s);

    config_.stream_path          = get_parameter("stream_path").as_string();
    config_.frame_id             = get_parameter("frame_id").as_string();
    config_.publish_rate         = get_parameter("publish_rate").as_int();
    config_.receiver_port        = get_parameter("receiver_port").as_string();
    config_.format               = get_parameter("format").as_string();
    config_.configure_on_startup = get_parameter("configure_on_startup").as_bool();

    if (config_.format != "oem4") {
      RCLCPP_WARN(get_logger(), "Unknown format '%s', defaulting to oem4", config_.format.c_str());
      config_.format = "oem4";
    }

    config_.enable_rangecmp    = get_parameter("messages.rangecmp").as_bool();
    config_.enable_range       = get_parameter("messages.range").as_bool();
    config_.enable_bestpos     = get_parameter("messages.bestpos").as_bool();
    config_.enable_bestvel     = get_parameter("messages.bestvel").as_bool();
    config_.enable_gps_ephem   = get_parameter("messages.gps_ephem").as_bool();
    config_.enable_glo_ephem   = get_parameter("messages.glo_ephem").as_bool();
    config_.enable_gal_ephem   = get_parameter("messages.gal_ephem").as_bool();
    config_.enable_bds_ephem   = get_parameter("messages.bds_ephem").as_bool();
    config_.enable_qzs_ephem   = get_parameter("messages.qzs_ephem").as_bool();
    config_.enable_navic_ephem = get_parameter("messages.navic_ephem").as_bool();
    config_.enable_ionutc      = get_parameter("messages.ionutc").as_bool();
    config_.enable_nmea_gpgga  = get_parameter("messages.nmea_gpgga").as_bool();
    config_.enable_nmea_gprmc  = get_parameter("messages.nmea_gprmc").as_bool();
    config_.enable_nmea_gpgsa  = get_parameter("messages.nmea_gpgsa").as_bool();
    config_.enable_nmea_gpgst  = get_parameter("messages.nmea_gpgst").as_bool();
    config_.enable_rawimusx    = get_parameter("messages.rawimusx").as_bool();
    config_.enable_corrimudata = get_parameter("messages.corrimudata").as_bool();
    config_.imu_scale_override_accel = get_parameter("imu_scale_override.accel").as_double();
    config_.imu_scale_override_gyro  = get_parameter("imu_scale_override.gyro").as_double();

    config_.observation_topic  = get_parameter("observation_topic").as_string();
    config_.ephemeris_topic    = get_parameter("ephemeris_topic").as_string();
    config_.solution_topic     = get_parameter("solution_topic").as_string();
    config_.imu_topic          = get_parameter("imu_topic").as_string();
    config_.imu_raw_topic      = get_parameter("imu_raw_topic").as_string();

    config_.ephemeris_snapshot_period_s = get_parameter("ephemeris.snapshot_period_s").as_double();
    config_.ephemeris_max_age_s         = get_parameter("ephemeris.max_age_s").as_double();

    eph_store_.setSnapshotPeriod(config_.ephemeris_snapshot_period_s);
    eph_store_.setMaxAge(config_.ephemeris_max_age_s);

    // Lock solution source at startup. BINARY if BESTPOS is enabled, else NMEA.
    source_ = config_.enable_bestpos ? SolutionSource::BINARY : SolutionSource::NMEA;
    RCLCPP_INFO(get_logger(), "Solution source locked: %s",
                source_ == SolutionSource::BINARY ? "BINARY (BESTPOS/BESTVEL)" : "NMEA");
  }

  void initializePublishers() {
    obs_pub_ = create_publisher<msg::GnssObservations>(config_.observation_topic, 10);
    eph_pub_ = create_publisher<msg::GnssEphemerides>(config_.ephemeris_topic, rclcpp::QoS(1).transient_local());
    sol_pub_ = create_publisher<msg::GnssSolution>(config_.solution_topic, 10);
    imu_pub_     = create_publisher<sensor_msgs::msg::Imu>(config_.imu_topic, 10);
    imu_raw_pub_ = create_publisher<sensor_msgs::msg::Imu>(config_.imu_raw_topic, 10);
  }

  void initializeDecoder() {
    if (init_raw(&raw_, STRFMT_OEM4) != 1) {
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

    int stream_type = STR_SERIAL;
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
    if (cmd.empty()) return;
    std::string full_cmd = cmd + "\r\n";
    RCLCPP_INFO(get_logger(), "Sending command: %s", cmd.c_str());
    int written = strwrite(&stream_, (uint8_t*)full_cmd.c_str(), full_cmd.size());
    if (written < 0 || (written == 0 && !is_serial_connection_)) {
      RCLCPP_ERROR(get_logger(), "Failed to write command (result: %d)", written);
    }
    std::this_thread::sleep_for(100ms);
  }

  void configureReceiverOem4(const std::string& port_pfx, const std::string& ontime, const std::string& onchanged) {
    if (config_.enable_rangecmp) sendCommand("LOG " + port_pfx + novatel::LOG_RANGECMP + ontime);
    else                         sendCommand("UNLOG " + port_pfx + novatel::LOG_RANGECMP);

    if (config_.enable_range)    sendCommand("LOG " + port_pfx + novatel::LOG_RANGE + ontime);
    else                         sendCommand("UNLOG " + port_pfx + novatel::LOG_RANGE);

    if (config_.enable_bestpos)  sendCommand("LOG " + port_pfx + novatel::LOG_BESTPOS + ontime);
    else                         sendCommand("UNLOG " + port_pfx + novatel::LOG_BESTPOS);

    if (config_.enable_bestvel)  sendCommand("LOG " + port_pfx + novatel::LOG_BESTVEL + ontime);
    else                         sendCommand("UNLOG " + port_pfx + novatel::LOG_BESTVEL);

    if (config_.enable_gps_ephem) {
      sendCommand("LOG " + port_pfx + novatel::LOG_GPSEPHEM + onchanged);
    } else { sendCommand("UNLOG " + port_pfx + novatel::LOG_GPSEPHEM); }

    if (config_.enable_glo_ephem) {
      sendCommand("LOG " + port_pfx + novatel::LOG_GLOEPHEMERIS + onchanged);
    } else { sendCommand("UNLOG " + port_pfx + novatel::LOG_GLOEPHEMERIS); }

    if (config_.enable_gal_ephem) {
      sendCommand("LOG " + port_pfx + novatel::LOG_GALEPHEMERIS + onchanged);
      sendCommand("LOG " + port_pfx + novatel::LOG_GALINAVEPHEMERIS + onchanged);
    } else {
      sendCommand("UNLOG " + port_pfx + novatel::LOG_GALEPHEMERIS);
      sendCommand("UNLOG " + port_pfx + novatel::LOG_GALINAVEPHEMERIS);
    }

    if (config_.enable_bds_ephem) {
      sendCommand("LOG " + port_pfx + novatel::LOG_BDSEPHEMERIS + onchanged);
    } else { sendCommand("UNLOG " + port_pfx + novatel::LOG_BDSEPHEMERIS); }

    if (config_.enable_qzs_ephem) {
      sendCommand("LOG " + port_pfx + novatel::LOG_QZSSEPHEMERIS + onchanged);
    } else { sendCommand("UNLOG " + port_pfx + novatel::LOG_QZSSEPHEMERIS); }

    if (config_.enable_navic_ephem) {
      sendCommand("LOG " + port_pfx + novatel::LOG_NAVICEPHEMERIS + onchanged);
    } else { sendCommand("UNLOG " + port_pfx + novatel::LOG_NAVICEPHEMERIS); }

    if (config_.enable_ionutc) {
      sendCommand("LOG " + port_pfx + novatel::LOG_IONUTC + onchanged);
      sendCommand("LOG " + port_pfx + novatel::LOG_GALIONO + onchanged);
      sendCommand("LOG " + port_pfx + novatel::LOG_QZSSIONUTC + onchanged);
    } else {
      sendCommand("UNLOG " + port_pfx + novatel::LOG_IONUTC);
      sendCommand("UNLOG " + port_pfx + novatel::LOG_GALIONO);
      sendCommand("UNLOG " + port_pfx + novatel::LOG_QZSSIONUTC);
    }

    // NMEA sentences (for GnssSolution)
    if (config_.enable_nmea_gpgga) sendCommand("LOG " + port_pfx + novatel::LOG_GPGGA + " ONTIME 1");
    else                           sendCommand("UNLOG " + port_pfx + novatel::LOG_GPGGA);

    if (config_.enable_nmea_gprmc) sendCommand("LOG " + port_pfx + novatel::LOG_GPRMC + " ONTIME 1");
    else                           sendCommand("UNLOG " + port_pfx + novatel::LOG_GPRMC);

    if (config_.enable_nmea_gpgsa) sendCommand("LOG " + port_pfx + novatel::LOG_GPGSA + " ONTIME 1");
    else                           sendCommand("UNLOG " + port_pfx + novatel::LOG_GPGSA);

    if (config_.enable_nmea_gpgst) sendCommand("LOG " + port_pfx + novatel::LOG_GPGST + " ONTIME 1");
    else                           sendCommand("UNLOG " + port_pfx + novatel::LOG_GPGST);

    // IMU outputs (require IMU-connected receiver; CORRIMUDATA additionally requires SPAN)
    if (config_.enable_rawimusx)    sendCommand(std::string("LOG ")   + port_pfx + novatel::LOG_RAWIMUSXB    + " ONNEW");
    else                            sendCommand(std::string("UNLOG ") + port_pfx + novatel::LOG_RAWIMUSXB);

    if (config_.enable_corrimudata) sendCommand(std::string("LOG ")   + port_pfx + novatel::LOG_CORRIMUDATAB + " ONNEW");
    else                            sendCommand(std::string("UNLOG ") + port_pfx + novatel::LOG_CORRIMUDATAB);
  }

  void configureReceiver() {
    RCLCPP_INFO(get_logger(), "Configuring receiver...");

    sendCommand(novatel::CMD_UNLOGALL);
    if (!config_.receiver_port.empty()) {
      sendCommand(std::string(novatel::CMD_UNLOGALL) + " " + config_.receiver_port);
    }
    std::this_thread::sleep_for(500ms);

    const std::string port_pfx     = config_.receiver_port + " ";
    const std::string ontime        = " ONTIME " + novatel::getIntervalString(config_.publish_rate);
    const std::string onchanged     = " ONCHANGED";

    configureReceiverOem4(port_pfx, ontime, onchanged);

    RCLCPP_INFO(get_logger(), "Configuration commands sent.");
  }

  // ============================================================================
  // Polling
  // ============================================================================

  void pollStream() {
    uint8_t buffer[novatel::READ_BUFFER_SIZE];
    const int bytes_read = strread(&stream_, buffer, sizeof(buffer));

    for (int i = 0; i < bytes_read; ++i) {
      const uint8_t byte = buffer[i];

      int result = input_oem4(&raw_, byte);
      handleDecodeResult(result);

      // Parallel OEM4 mini-framer for CORRIMUDATA
      parseOem4Byte(byte);

      // NMEA sentences (interleaved with NovAtel binary)
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

    maybePublishHeartbeat();
  }

  // ============================================================================
  // Message Handling
  // ============================================================================

  // Solution source policy:
  //   BINARY (BESTPOS/VEL) : if config_.enable_bestpos → use BESTPOS only, drop NMEA
  //   NMEA                : otherwise → use NMEA-parsed solution
  // Source is locked at startup; no mid-session switching.
  enum class SolutionSource { BINARY, NMEA };

  void handleNmeaSentence(const std::string& sentence) {
    if (nmea_parser_.parseSentence(sentence, nmea_solution_)) {
      if (source_ == SolutionSource::NMEA) {
        publishSolution(nmea_solution_);
      }
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
  // OEM4 Mini-Framer (parallel to RTKLIB, for CORRIMUDATA)
  //
  // OEM4 binary frame: SYNC(0xAA,0x44,0x12) HDRLEN(1) MSGID(2,LE) MSGTYPE(1)
  //                    PORT(1) MSGLEN(2,LE) ... rest of header ... BODY CRC32
  // ============================================================================

  void parseOem4Byte(uint8_t byte) {
    switch (oem4_state_) {
      case 0:  if (byte == novatel::OEM4_SYNC1) oem4_state_ = 1; break;
      case 1:  oem4_state_ = (byte == novatel::OEM4_SYNC2) ? 2 : (byte == novatel::OEM4_SYNC1 ? 1 : 0); break;
      case 2:  oem4_state_ = (byte == novatel::OEM4_SYNC3) ? 3 : (byte == novatel::OEM4_SYNC1 ? 1 : 0); break;
      case 3:  oem4_hdr_len_  = byte; oem4_state_ = 4; break;
      case 4:  oem4_msg_id_   = byte; oem4_state_ = 5; break;
      case 5:  oem4_msg_id_  |= static_cast<uint16_t>(byte << 8); oem4_state_ = 6; break;
      case 6:  oem4_state_ = 7; break;  // MSGTYPE skip
      case 7:  oem4_state_ = 8; break;  // PORT skip
      case 8:  oem4_msg_len_  = byte; oem4_state_ = 9; break;
      case 9:
        oem4_msg_len_ |= static_cast<uint16_t>(byte << 8);
        oem4_hdr_rem_  = static_cast<int>(oem4_hdr_len_) - 10;
        oem4_body_.clear();
        oem4_body_pos_ = 0;
        oem4_state_    = (oem4_hdr_rem_ > 0) ? 10 : 11;
        break;
      case 10:
        if (--oem4_hdr_rem_ <= 0) oem4_state_ = (oem4_msg_len_ > 0) ? 11 : 13;
        break;
      case 11:
        oem4_body_.push_back(byte);
        if (++oem4_body_pos_ >= oem4_msg_len_) { handleOem4Frame(); oem4_state_ = 12; }
        break;
      case 12: oem4_state_ = 13; break;
      case 13: oem4_state_ = 14; break;
      case 14: oem4_state_ = 15; break;
      case 15: oem4_state_ = 0;  break;
      default: oem4_state_ = 0;
    }
  }

  void handleOem4Frame() {
    if (oem4_msg_id_ == novatel::ID_RAWIMUSX)    handleRawImuSx();
    if (oem4_msg_id_ == novatel::ID_CORRIMUDATA) handleCorrImuData();
    if (oem4_msg_id_ == novatel::ID_BESTPOS)     handleBestPos();
    if (oem4_msg_id_ == novatel::ID_BESTVEL)     handleBestVel();
  }

  void handleBestPos() {
    if (!novatel::pvt::parseBESTPOS(oem4_body_.data(), oem4_body_.size(), binary_solution_)) {
      return;
    }
    finalizeBinarySolutionGeometry(binary_solution_);
    if (source_ == SolutionSource::BINARY) {
      publishSolution(binary_solution_);
    }
  }

  void handleBestVel() {
    // BESTVEL only updates velocity in binary_solution_; do not trigger a publish
    // by itself — BESTPOS is the publish-driver per epoch and typically arrives
    // adjacent. We just refresh velocity fields so the next BESTPOS publish has
    // current vel_enu/vel_ecef.
    if (!novatel::pvt::parseBESTVEL(oem4_body_.data(), oem4_body_.size(), binary_solution_)) {
      return;
    }
    // Re-derive vel_ecef from updated vel_enu (needs current LLH from prior BESTPOS).
    double llh[3] = {binary_solution_.latitude * D2R,
                     binary_solution_.longitude * D2R,
                     binary_solution_.altitude};
    double vel_e[3] = {binary_solution_.vel_enu.x, binary_solution_.vel_enu.y, binary_solution_.vel_enu.z};
    double vel_ec[3] = {0};
    enu2ecef(llh, vel_e, vel_ec);
    binary_solution_.vel_ecef.x = vel_ec[0];
    binary_solution_.vel_ecef.y = vel_ec[1];
    binary_solution_.vel_ecef.z = vel_ec[2];
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

  // RAWIMUSX body (60 B):
  //   HealthBytes(u2) IMUType(u1) Reserved1(u1) Week(u2) Reserved2(u2) Seconds(f8)
  //   IMUStatus(u4)
  //   ZAccel(i4) -YAccel(i4) XAccel(i4)  ZGyro(i4) -YGyro(i4) XGyro(i4)
  // Counts × IMU-type-specific scale = SI increments per IMU sample;
  // divide by dt (Seconds delta) to get m/s² and rad/s.
  // Note: -Y fields are the actual Y-axis value already negated by the receiver,
  // so reading them as int32 gives -Y; we negate again to recover the +Y reading.
  void handleRawImuSx() {
    if (oem4_body_.size() < 60) return;

    const uint8_t  imu_type = oem4_body_[2];
    double seconds = 0.0;
    int32_t z_acc = 0, ny_acc = 0, x_acc = 0, z_gyr = 0, ny_gyr = 0, x_gyr = 0;
    std::memcpy(&seconds, oem4_body_.data() +  8, 8);
    std::memcpy(&z_acc,   oem4_body_.data() + 20, 4);
    std::memcpy(&ny_acc,  oem4_body_.data() + 24, 4);
    std::memcpy(&x_acc,   oem4_body_.data() + 28, 4);
    std::memcpy(&z_gyr,   oem4_body_.data() + 32, 4);
    std::memcpy(&ny_gyr,  oem4_body_.data() + 36, 4);
    std::memcpy(&x_gyr,   oem4_body_.data() + 40, 4);

    // Resolve scale: yaml override takes precedence if both fields are non-zero.
    novatel::ImuScale scale = novatel::getImuScale(imu_type);
    if (config_.imu_scale_override_accel != 0.0 && config_.imu_scale_override_gyro != 0.0) {
      scale = {config_.imu_scale_override_accel, config_.imu_scale_override_gyro};
    }
    if (scale.accel == 0.0 || scale.gyro == 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Unknown NovAtel IMU type %u — RAWIMUSX skipped. Set imu_scale_override.{accel,gyro} "
        "in yaml or add this type to novatel_imu_scales.hpp.", imu_type);
      return;
    }

    if (!has_rawimu_prev_) {
      rawimu_prev_seconds_ = seconds;
      has_rawimu_prev_     = true;
      return;
    }
    const double dt = seconds - rawimu_prev_seconds_;
    rawimu_prev_seconds_ = seconds;
    if (dt <= 0.0 || dt > 1.0) return;  // sanity: skip across-week wraps

    // Recover +Y by negating the -Y fields, then scale and divide by dt.
    sensor_msgs::msg::Imu imu;
    imu.header.stamp    = now();
    imu.header.frame_id = config_.frame_id;

    imu.orientation_covariance[0] = -1.0;  // unknown

    imu.angular_velocity.x =  x_gyr  * scale.gyro / dt;
    imu.angular_velocity.y = -ny_gyr * scale.gyro / dt;
    imu.angular_velocity.z =  z_gyr  * scale.gyro / dt;
    imu.linear_acceleration.x =  x_acc  * scale.accel / dt;
    imu.linear_acceleration.y = -ny_acc * scale.accel / dt;
    imu.linear_acceleration.z =  z_acc  * scale.accel / dt;

    auto unk = ins::makeUnknownCovariance();
    std::copy(unk.begin(), unk.end(), imu.angular_velocity_covariance.begin());
    std::copy(unk.begin(), unk.end(), imu.linear_acceleration_covariance.begin());

    imu_raw_pub_->publish(imu);
  }

  // CORRIMUDATA body (60 B): Week(4) Seconds(8) PitchRate(8) RollRate(8) YawRate(8)
  //                          LateralAcc(8) LongitudinalAcc(8) VerticalAcc(8)
  // Rates and accelerations are SI increments accumulated over the IMU sampling
  // period; divide by dt (from successive Seconds fields) to recover rad/s and m/s².
  void handleCorrImuData() {
    if (oem4_body_.size() < 60) return;

    double seconds = 0.0, pitch_rate = 0.0, roll_rate = 0.0, yaw_rate = 0.0;
    double lat_acc = 0.0, lon_acc = 0.0, vert_acc = 0.0;
    std::memcpy(&seconds,    oem4_body_.data() +  4, 8);
    std::memcpy(&pitch_rate, oem4_body_.data() + 12, 8);
    std::memcpy(&roll_rate,  oem4_body_.data() + 20, 8);
    std::memcpy(&yaw_rate,   oem4_body_.data() + 28, 8);
    std::memcpy(&lat_acc,    oem4_body_.data() + 36, 8);
    std::memcpy(&lon_acc,    oem4_body_.data() + 44, 8);
    std::memcpy(&vert_acc,   oem4_body_.data() + 52, 8);

    if (!has_corrimu_prev_) {
      corrimu_prev_seconds_ = seconds;
      has_corrimu_prev_     = true;
      return;
    }
    const double dt = seconds - corrimu_prev_seconds_;
    corrimu_prev_seconds_ = seconds;
    if (dt <= 0.0 || dt > 1.0) return;  // sanity: skip across-week wraps and outliers

    sensor_msgs::msg::Imu imu;
    imu.header.stamp    = now();
    imu.header.frame_id = config_.frame_id;

    // SPAN body frame: roll=X, pitch=Y, yaw=Z; longitudinal=X, lateral=Y, vertical=Z
    imu.angular_velocity.x    = roll_rate / dt;
    imu.angular_velocity.y    = pitch_rate / dt;
    imu.angular_velocity.z    = yaw_rate / dt;
    imu.linear_acceleration.x = lon_acc / dt;
    imu.linear_acceleration.y = lat_acc / dt;
    imu.linear_acceleration.z = vert_acc / dt;

    // orientation: not provided by CORRIMUDATA — leave identity, mark unknown
    imu.orientation_covariance[0] = -1.0;

    auto unk = ins::makeUnknownCovariance();
    std::copy(unk.begin(), unk.end(), imu.angular_velocity_covariance.begin());
    std::copy(unk.begin(), unk.end(), imu.linear_acceleration_covariance.begin());

    imu_pub_->publish(imu);
  }

  // ============================================================================
  // Solution Publishing
  // ============================================================================

  void publishSolution(msg::GnssSolution& sol) {
    int week = 0;
    const double tow = time2gpst(raw_.time, &week);
    if (week != 0) {
      sol.time_week = static_cast<uint16_t>(week);
      sol.time_tow  = tow;
    }

    sol.header.stamp    = now();
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

      sol.org_ecef.x = local_origin_ecef_[0];
      sol.org_ecef.y = local_origin_ecef_[1];
      sol.org_ecef.z = local_origin_ecef_[2];

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

      double vel_ecef[3] = {
        sol.vel_ecef.x,
        sol.vel_ecef.y,
        sol.vel_ecef.z
      };
      double vel_enu[3] = {0};
      ecef2enu(local_origin_pos_, vel_ecef, vel_enu);
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
    if (!config_.enable_rangecmp && !config_.enable_range) return;
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
    bool changed = false;

    for (int i = 0; i < raw_.nav.n; ++i) {
      const eph_t& eph = raw_.nav.eph[i];
      if (eph.sat == 0) continue;
      int prn = 0;
      const int sys = satsys(eph.sat, &prn);
      if (sys == SYS_GPS && !config_.enable_gps_ephem) continue;
      if (sys == SYS_GAL && !config_.enable_gal_ephem) continue;
      if (sys == SYS_CMP && !config_.enable_bds_ephem) continue;
      if (sys == SYS_QZS && !config_.enable_qzs_ephem) continue;
      if (sys == SYS_IRN && !config_.enable_navic_ephem) continue;
      changed = eph_store_.ingestEph(eph) || changed;
    }

    for (int i = 0; i < raw_.nav.ng; ++i) {
      if (!config_.enable_glo_ephem) break;
      const geph_t& geph = raw_.nav.geph[i];
      if (geph.sat == 0) continue;
      changed = eph_store_.ingestGeph(geph) || changed;
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

  NovatelConfig config_;

  rclcpp::Publisher<msg::GnssObservations>::SharedPtr obs_pub_;
  rclcpp::Publisher<msg::GnssEphemerides>::SharedPtr  eph_pub_;
  rclcpp::Publisher<msg::GnssSolution>::SharedPtr     sol_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;       // CORRIMUDATA (calibrated)
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_pub_;   // RAWIMUSX (uncalibrated)
  rclcpp::TimerBase::SharedPtr timer_;

  stream_t stream_{};
  raw_t    raw_{};
  bool is_serial_connection_{false};

  // NMEA parsing state
  gnss_utils::NmeaParser nmea_parser_;
  std::string            nmea_buffer_;
  msg::GnssSolution      nmea_solution_;
  msg::GnssSolution      binary_solution_;
  SolutionSource         source_{SolutionSource::NMEA};

  // ENU local origin
  bool   has_local_origin_{false};
  double local_origin_ecef_[3]{0.0};
  double local_origin_pos_[3]{0.0};

  // Unified ephemeris store
  gnss_utils::EphemerisStore eph_store_;

  // OEM4 mini-framer state (for CORRIMUDATA / RAWIMUSX)
  int      oem4_state_{0};
  uint8_t  oem4_hdr_len_{0};
  uint16_t oem4_msg_id_{0};
  uint16_t oem4_msg_len_{0};
  int      oem4_hdr_rem_{0};
  int      oem4_body_pos_{0};
  std::vector<uint8_t> oem4_body_;

  // CORRIMUDATA dt tracking (SI-increment → rate conversion)
  double corrimu_prev_seconds_{0.0};
  bool   has_corrimu_prev_{false};
  double rawimu_prev_seconds_{0.0};
  bool   has_rawimu_prev_{false};
};

}  // namespace gnss_ros_standardization

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gnss_ros_standardization::NovatelDriverNode>());
  rclcpp::shutdown();
  return 0;
}
