#include "gnss_ros_standardization/receiver_health.hpp"
#include "gnss_ros_standardization/receiver_session.hpp"
#include "gnss_ros_standardization/sbf_protocol.hpp"
#include "gnss_ros_standardization/binary_framer.hpp"
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstring>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include <sensor_msgs/msg/imu.hpp>

#include "gnss_ros_standardization/decoder_common.hpp"
#include "gnss_ros_standardization/ephemeris_store.hpp"
#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/ins_utils.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"

using namespace std::chrono_literals;
namespace ins = gnss_ros_standardization::ins_utils;

namespace gnss_ros_standardization {

namespace {

constexpr auto kTimerInterval  = 10ms;


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

  // RTCM relay (PC -> receiver) for on-chip RTK — see openRtcmRelay().
  bool rtcm_relay_enabled{false};
  std::string rtcm_relay_listen{"tcpsvr://:5557"};

  // Ephemeris snapshot behavior
  double ephemeris_snapshot_period_s{30.0};
  double ephemeris_max_age_s{0.0};  // 0 = keep all

  bool use_gps_timestamp{false};

  // ENU local origin
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

    openRtcmRelay();
    if (config_.configure_on_startup && config_.enable_meas_epoch) receiver_health_.expectObservations(config_.publish_rate);
    startPolling();
    RCLCPP_INFO(get_logger(), "SBF Driver initialized");
  }

  ~SbfDriverNode() override {
    try {
      strclose(&stream_);
      relay_.close();
    } catch (...) {}
    free_raw(&raw_);
  }

 private:
  // Initialization

  decoder_common::EpochFlushPolicy epoch_flush_;
  SbfObservationCompletion observation_completion_;
  wire::ReceiverHealth receiver_health_;
  void initializeParameters() {
    receiver_health_.configure(5.0);
    receiver_health_.limits(MAXOBS, NFREQ, NEXOBS, MAXPRNCMP);
    observation_completion_.configure(0.2);
    epoch_flush_.configure(0.05);
    declare_parameter<std::string>("stream_path",          config_.stream_path);
    // frame_id: ROS TF frame name attached to the GNSS antenna phase center.
    // Default "gnss_link" — override in launch to integrate with vehicle TF tree.
    declare_parameter<std::string>("frame_id",             config_.frame_id);
    declare_parameter<int>        ("rate_hz",         config_.publish_rate);
    declare_parameter<std::string>("receiver_port",        config_.receiver_port);
    declare_parameter<bool>       ("configure_on_startup", config_.configure_on_startup);

    declare_parameter<bool>("messages.meas_epoch",         config_.enable_meas_epoch);
    declare_parameter<bool>("messages.gps_nav",            config_.enable_gps_nav);
    declare_parameter<bool>("messages.glo_nav",            config_.enable_glo_nav);
    declare_parameter<bool>("messages.gal_nav",            config_.enable_gal_nav);
    declare_parameter<bool>("messages.bds_nav",            config_.enable_bds_nav);
    declare_parameter<bool>("messages.qzs_nav",            config_.enable_qzs_nav);
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

    declare_parameter<std::string>("topics.observation", config_.observation_topic);
    declare_parameter<std::string>("topics.ephemeris",   config_.ephemeris_topic);
    declare_parameter<std::string>("topics.solution",    config_.solution_topic);
    declare_parameter<std::string>("topics.imu_raw",     config_.imu_raw_topic);
    declare_parameter<bool>("rtcm_input.enable",       config_.rtcm_relay_enabled);
    declare_parameter<std::string>("rtcm_input.listen", config_.rtcm_relay_listen);

    declare_parameter<double>("ephemeris.snapshot_period_s", config_.ephemeris_snapshot_period_s);
    declare_parameter<double>("ephemeris.max_age_s",         config_.ephemeris_max_age_s);
    declare_parameter<bool>("use_gps_timestamp",             config_.use_gps_timestamp);

    config_.stream_path          = get_parameter("stream_path").as_string();
    config_.frame_id             = get_parameter("frame_id").as_string();
    config_.publish_rate         = get_parameter("rate_hz").as_int();
    // Upper bound 200 Hz: getIntervalString maps 200->msec5, 100->msec10. The
    // receiver caps GNSS blocks at its licensed measurement rate on its own,
    // while ExtSensorMeas streams up to the IMU's native sample rate.
    gnss_utils::validatePublishRate(config_.publish_rate, /*fallback=*/1, /*max_hz=*/200, get_logger());
    config_.receiver_port        = get_parameter("receiver_port").as_string();
    config_.configure_on_startup = get_parameter("configure_on_startup").as_bool();

    config_.enable_meas_epoch        = get_parameter("messages.meas_epoch").as_bool();
    config_.enable_gps_nav           = get_parameter("messages.gps_nav").as_bool();
    config_.enable_glo_nav           = get_parameter("messages.glo_nav").as_bool();
    config_.enable_gal_nav           = get_parameter("messages.gal_nav").as_bool();
    config_.enable_bds_nav           = get_parameter("messages.bds_nav").as_bool();
    config_.enable_qzs_nav           = get_parameter("messages.qzs_nav").as_bool();
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

    config_.observation_topic = get_parameter("topics.observation").as_string();
    config_.ephemeris_topic   = get_parameter("topics.ephemeris").as_string();
    config_.solution_topic    = get_parameter("topics.solution").as_string();
    config_.imu_raw_topic     = get_parameter("topics.imu_raw").as_string();
    config_.rtcm_relay_enabled = get_parameter("rtcm_input.enable").as_bool();
    config_.rtcm_relay_listen  = get_parameter("rtcm_input.listen").as_string();

    config_.ephemeris_snapshot_period_s = get_parameter("ephemeris.snapshot_period_s").as_double();
    config_.ephemeris_max_age_s         = get_parameter("ephemeris.max_age_s").as_double();
    config_.use_gps_timestamp           = get_parameter("use_gps_timestamp").as_bool();

    eph_store_.setSnapshotPeriod(config_.ephemeris_snapshot_period_s);
    eph_store_.setMaxAge(config_.ephemeris_max_age_s);
    constexpr int history_limit = 8;
    eph_store_.setHistoryPerSatellite(static_cast<size_t>(history_limit));

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
    decoder_common::configureNmeaDate(*this, nmea_parser_);
    obs_pub_          = create_publisher<msg::GnssObservations>(config_.observation_topic, 10);
    eph_pub_          = create_publisher<msg::GnssEphemerides>(config_.ephemeris_topic, rclcpp::QoS(256).transient_local()  /* depth 256: latch full ephemeris set, not just the last satellite (per-sat messages) */);
    sol_pub_          = create_publisher<msg::GnssSolution>(config_.solution_topic, 10);
    imu_raw_pub_      = create_publisher<sensor_msgs::msg::Imu>(config_.imu_raw_topic, decoder_common::kImuPublisherDepth);

    const auto origin = gnss_utils::declareLocalOrigin(*this);
    if (origin.fixed) {
      std::copy_n(origin.llh, 3, local_origin_pos_);
      std::copy_n(origin.ecef, 3, local_origin_ecef_);
      has_local_origin_ = true;
      RCLCPP_INFO(get_logger(), "ENU origin set from config: lat=%.6f lon=%.6f alt=%.2f",
        local_origin_pos_[0] * R2D, local_origin_pos_[1] * R2D, local_origin_pos_[2]);
    }
    logEnabledMessages();
  }

  void logEnabledMessages() {
    auto on = [](bool v) { return v ? "ON" : "OFF"; };
    RCLCPP_INFO(get_logger(), "Enabled messages:");
    RCLCPP_INFO(get_logger(), "  Observation  : MeasEpoch=%s", on(config_.enable_meas_epoch));
    RCLCPP_INFO(get_logger(), "  Navigation   : GPS=%s GLO=%s GAL=%s BDS=%s QZS=%s",
      on(config_.enable_gps_nav), on(config_.enable_glo_nav), on(config_.enable_gal_nav),
      on(config_.enable_bds_nav), on(config_.enable_qzs_nav));
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
    if (has_local_origin_) {
      RCLCPP_INFO(get_logger(), "  ENU origin    : fixed (lat=%.6f lon=%.6f alt=%.2f)",
        local_origin_pos_[0] * R2D, local_origin_pos_[1] * R2D, local_origin_pos_[2]);
    } else {
      RCLCPP_INFO(get_logger(), "  ENU origin    : auto (first valid solution)");
    }
  }

  void initializeDecoder() {
    if (init_raw(&raw_, STRFMT_SEPT) != 1) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize raw decoder");
      throw std::runtime_error("init_raw failed");
    }
    std::strcpy(raw_.opt, "-EPHALL");
  }

  void startPolling() {
    timer_ = create_wall_timer(kTimerInterval, std::bind(&SbfDriverNode::pollStream, this));
  }

  // Stream Management

  void openStream() {
    const auto path = config_.stream_path.find("://") == std::string::npos
        ? "serial://" + config_.stream_path : config_.stream_path;
    if (!decoder_common::openStreamPath(get_logger(), stream_, path, STR_MODE_RW, "Receiver"))
      throw std::runtime_error("stropen failed");
    is_serial_connection_ = stream_.type == STR_SERIAL;
  }

  // RTCM relay (RTKLIB STRSVR-style): host a TCP server (or any RTKLIB stream)
  // for incoming base-station corrections. Bytes received here are written to the
  // receiver serial in pollStream() — the driver stays the sole owner of the
  // serial port; corrections arrive over TCP, not via a second serial opener.
  void openRtcmRelay() {
    if (!config_.rtcm_relay_enabled) return;
    if (!relay_.open(get_logger(), config_.rtcm_relay_listen)) {
      throw std::runtime_error("RTCM relay stropen failed");
    }
  }

  // Receiver Configuration

  void sendCommand(const std::string& cmd) {
    if (cmd.empty()) return;
    decoder_common::configureAscii(stream_, wire::Protocol::Sbf, cmd,
        [this](uint8_t byte) {
          parseSbfByte(byte);
        });
  }

  void configureReceiver() {
    RCLCPP_INFO(get_logger(), "Configuring receiver...");

    const std::string interval_str = sbf::getIntervalString(config_.publish_rate);

    // --- NMEA output (Stream2, same physical port) ---
    // Send NMEA setup FIRST while the port is quiet. If SBF were enabled
    // first, its binary stream would saturate the port and bury the `sno`
    // ACK (and empirically appears to prevent NMEA from routing at all).
    std::vector<std::string> nmea_sentences;
    if (config_.enable_nmea_gga) nmea_sentences.push_back("GGA");
    if (config_.enable_nmea_rmc) nmea_sentences.push_back("RMC");
    if (config_.enable_nmea_gsa) nmea_sentences.push_back("GSA");
    if (config_.enable_nmea_gst) nmea_sentences.push_back("GST");

    if (!nmea_sentences.empty()) {
      std::string nmea_list = nmea_sentences[0];
      for (size_t i = 1; i < nmea_sentences.size(); ++i) nmea_list += "+" + nmea_sentences[i];

      // Use Stream2 for NMEA at the requested publish rate.
      std::string nmea_cmd = std::string(sbf::CMD_SET_NMEA_OUTPUT) + ", Stream2, " +
                             config_.receiver_port + ", " + nmea_list + ", " + interval_str;
      RCLCPP_INFO(get_logger(), "NMEA config: %s", nmea_cmd.c_str());
      sendCommand(nmea_cmd);
    }

    // --- SBF output (Stream1) ---
    std::vector<std::string> blocks;
    if (config_.enable_meas_epoch) {
      blocks.push_back(sbf::BLOCK_MEASEPOCH);
      blocks.push_back("EndOfMeas");
    }
    // Decoded *Nav blocks (receiver-assembled)
    if (config_.enable_gps_nav)          blocks.push_back(sbf::BLOCK_GPSNAV);
    if (config_.enable_glo_nav)          blocks.push_back(sbf::BLOCK_GLONAV);
    if (config_.enable_gal_nav)          blocks.push_back(sbf::BLOCK_GALNAV);
    if (config_.enable_bds_nav)          blocks.push_back(sbf::BLOCK_BDSNAV);
    if (config_.enable_qzs_nav)          blocks.push_back(sbf::BLOCK_QZSNAV);
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
    // ExtSensorMeas shares Stream1: configuring it on a separate stream with
    // OnChange produced no output on real hardware. With a single stream the
    // receiver caps each block individually — GNSS blocks at the licensed
    // measurement rate, ExtSensorMeas at the IMU's native sample rate — so
    // rate_hz: 100 (msec10) yields e.g. GNSS 10 Hz + IMU 100 Hz, matching
    // the WebUI Interval=10ms behavior.
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

    RCLCPP_INFO(get_logger(), "Receiver acknowledged configuration commands; verify actual message types and rates");
  }

  // Polling

  wire::StreamSession stream_session_;
  wire::ReceiverTimeline receiver_timeline_;
  void resetReceiverSession(bool disconnected) {
    // Release validated pending data before retiring the old session. Partial
    // wire frames and compressed references are never carried across sessions.
    if (gnss_flush_sbf_observations(&raw_)==1) publishObservations();
    observation_completion_.reset();
    flushPending();
    wire::resetRaw(raw_,STRFMT_SEPT);
    if (disconnected) wire_framer_.reset();
    receiver_health_.resetSession();
    if (disconnected && config_.configure_on_startup)
      RCLCPP_WARN(get_logger(), "Configuration is startup-only: if receiver settings were lost, restart this driver to reconfigure; ACK does not verify output");
    receiver_timeline_.reset();
    nmea_parser_.reset(); nmea_solution_=msg::GnssSolution(); binary_solution_=msg::GnssSolution();
    pending_=decltype(pending_)(); last_dop_={};
    prev_pvt_tow_ms_=UINT32_MAX; pvt_period_ms_=0;
    last_flushed_week_=0; last_flushed_tow_ms_=UINT32_MAX;
    eph_store_.clear(); publishSnapshot();
    initPendingExpectedMasks();
    esm_tow_ms_=UINT32_MAX; esm_wnc_=UINT16_MAX; esm_sensor_=UINT16_MAX;
    esm_has_accel_=false; esm_has_gyro_=false;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Receiver session reset: %s; waiting for fresh time/navigation references",
                disconnected ? "connection lost" : "measurement rewind or clock reset");
  }

  void pollStream() {
    // RTCM relay (STRSVR-style): drain corrections from the TCP server and write
    // them to the receiver serial. Same poll thread as the read below, so no lock.
    relay_.drainTo(get_logger(), *get_clock(), stream_);

    uint8_t buffer[sbf::READ_BUFFER_SIZE];
    for (int batch=0; batch<16 && rclcpp::ok(); ++batch) {
    const int bytes_read = strread(&stream_, buffer, sizeof(buffer));
    if (stream_session_.poll(stream_)) resetReceiverSession(true);
    const auto wire_now = std::chrono::steady_clock::now();
    receiver_health_.input(bytes_read);
    receiver_health_.framing(wire_framer_);
    receiver_health_.assemblyPending(raw_.obuf.n>0);
    receiver_health_.pvtPending(pending_.blocks_received != 0);
    receiver_health_.imuPending(esm_has_accel_ || esm_has_gyro_);
    receiver_health_.report(get_logger());
    if (bytes_read > 0) wire_last_input_ = wire_now;
    else if (wire_now - wire_last_input_ > std::chrono::seconds(1)) {
      wire_framer_.expire([this](const wire::Frame& f) { dispatchFrame(f); });
      wire_last_input_ = wire_now;
    }
    if (wire_framer_.rejected() != wire_last_rejected_) {
      wire_last_rejected_ = wire_framer_.rejected();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Rejected binary frame/header count: %llu",
          static_cast<unsigned long long>(wire_last_rejected_));
    }


    for (int i = 0; i < bytes_read; ++i) {
      parseSbfByte(buffer[i]);
    }
    if (bytes_read < static_cast<int>(sizeof(buffer))) break;
    }


    if (observation_completion_.flushIfDue(raw_)) {
      RCLCPP_DEBUG(get_logger(), "Publishing final MeasEpoch after observation quiet timeout");
      publishObservations();
    }
    maybePublishHeartbeat();
    maybeWatchdogFlushPendingPvt();
    warnIfBinaryStarvation();
  }

  // Watchdog: if a pending TOW has been sitting incomplete for too long
  // (no further blocks arriving), flush whatever we have.
  void maybeWatchdogFlushPendingPvt() {
    if (pending_.blocks_received == 0) return;
    if (epoch_flush_.due()) {
      flushPending();
    }
  }

  // SBF mixed stream decoder (shared boundary with RTKLIB, for ExtSensorMeas and PVT blocks)

  void dispatchFrame(const wire::Frame& f) {
      auto diagnostic_scope = receiver_health_.backendScope();
      receiver_health_.frame((unsigned(f.cls)<<16)|f.id, f.revision);
      receiver_health_.inspect(wire::Protocol::Sbf,f);
      if (receiver_timeline_.frame(wire::Protocol::Sbf,f)) {
        resetReceiverSession(false);
        receiver_timeline_.frame(wire::Protocol::Sbf,f);
      }
      if (!gnss_ros_standardization::supportedSbfNavigation(f)) {
        receiver_health_.unsupportedNavigation();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Unsupported SBF navigation model/layout: id=%u revision=%u length=%zu; retain the original receiver log",
            f.id, f.revision, f.bytes.size());
        return;
      }
      int result = 0;
      for (uint8_t byte : f.bytes) {
        const int status = input_sbf(&raw_, byte);
        if (status != 0) result = status;
        handleDecodeResult(status);
      }
      if (result >= 0) observation_completion_.decoded(f.id, raw_);
      receiver_health_.assemblyPending(raw_.obuf.n>0);
      sbf_id_ = f.id;
      sbf_body_ = f.payload;
      handleSbfBlock();
  }

  void parseSbfByte(uint8_t byte) {
    wire_framer_.feed(byte, [this](const wire::Frame& f) { dispatchFrame(f); },
        [this](const std::string& sentence) { handleNmeaSentence(sentence); });
  }

  void handleSbfBlock() {
    switch (sbf_id_) {
      case sbf::ID_EXTSENSORMEAS:    handleExtSensorMeas();    break;
      // Decoded *Nav blocks are handled by RTKLIB input_sbf(); decoding them
      // here too produced duplicate, slightly-different ephemerides.
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

  // PVT TOW aggregation: Geo+Xyz PVT/Cov blocks for one TOW merge into
  // pending_.buf; flush when blocks_received == blocks_expected or a new TOW
  // arrives. Policy in flushPending(): PVTGeodetic is the primary position
  // (LLH derived from Cartesian only as fallback); ENU values are the truth
  // source, ECEF derived by rotation; Cartesian-direct values override.
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
  };

  // Persistent DOP cache + PVT cadence tracking for the staleness gate.
  gnss_utils::DopCache last_dop_;
  uint32_t prev_pvt_tow_ms_{UINT32_MAX};
  uint32_t pvt_period_ms_{0};

  // Last published (week, tow) — used by the orphan guard (see mergeBlock).
  uint32_t last_flushed_week_{0};
  uint32_t last_flushed_tow_ms_{UINT32_MAX};

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


  template <typename ParseFn>
  void mergeBlock(uint8_t bit, ParseFn parse) {
    const uint8_t* p = sbf_body_.data();
    const size_t   len = sbf_body_.size();
    const uint32_t tow = sbf::pvt::getTowMs(p, len);

    // Orphan guard: this block belongs to the most-recently-flushed epoch
    // (eager-flush picked up an incomplete view). Skip to suppress a duplicate
    // publish at the next TOW boundary. Week falls back to TOW-only when
    // pending_.buf.time_week is 0 (no PVT seen yet this epoch).
    const uint32_t week = pending_.buf.time_week;
    if (last_flushed_tow_ms_ != UINT32_MAX &&
        tow == last_flushed_tow_ms_ &&
        (week == 0 || week == last_flushed_week_)) {
      return;
    }

    if (tow != pending_.tow_ms && pending_.blocks_received != 0) {
      flushPending();  // TOW boundary — flush previous epoch
    }
    pending_.tow_ms = tow;
    if (!parse(p, len, pending_.buf)) { GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return; }
    pending_.blocks_received |= bit;
    epoch_flush_.touch();
    // Flush at the epoch boundary or shared quiet-period deadline.
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

  // DOP block: cached in last_dop_; applied at flush by applyDopWithStaleness()
  // (does not gate epoch completion).
  void handleDop() {
    if (!config_.enable_dop) return;
    const uint8_t* p   = sbf_body_.data();
    const size_t   len = sbf_body_.size();
    msg::GnssSolution scratch{};
    if (!sbf::pvt::parseDop(p, len, scratch)) { GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return; }
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
      if (pending_.blocks_received) receiver_health_.pvtIncompleteDiscarded();
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
    decoder_common::finalizeBinarySolutionGeometry(binary_solution_);

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

    // DOP staleness gate (see gnss_utils::applyDopWithStaleness).
    gnss_utils::applyDopWithStaleness(binary_solution_, last_dop_,
                                      binary_solution_.time_week,
                                      pending_.tow_ms, pvt_period_ms_);

    ever_received_binary_ = true;
    if (source_ == SolutionSource::BINARY) publishSolution(binary_solution_);

    // Record this epoch as the last published (week, tow) for orphan-guard.
    last_flushed_week_   = binary_solution_.time_week;
    last_flushed_tow_ms_ = pending_.tow_ms;

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


  void handleExtSensorMeas() {
    if (!config_.enable_ext_sensor_meas) return;
    if (static_cast<int>(sbf_body_.size()) < sbf::ext_sensor_meas::MIN_BODY_LEN) { GNSS_RX_EVENT(GNSS_RX_LENGTH,1); return; }

    const uint8_t n         = sbf_body_[sbf::ext_sensor_meas::OFFSET_N];
    const uint8_t sb_length = sbf_body_[sbf::ext_sensor_meas::OFFSET_SB_LENGTH];
    if (sb_length < sbf::ext_sensor_meas::SB_MIN_LEN) { GNSS_RX_EVENT(GNSS_RX_LENGTH,1); return; }
    if (static_cast<int>(sbf_body_.size()) < sbf::ext_sensor_meas::MIN_BODY_LEN + n * sb_length) { GNSS_RX_EVENT(GNSS_RX_LENGTH,1); return; }

    uint32_t tow_ms = 0;
    std::memcpy(&tow_ms, sbf_body_.data(), 4);

    const uint16_t week = wire::u16(sbf_body_.data() + 4);
    if (tow_ms != esm_tow_ms_ || week != esm_wnc_) {
      if (esm_has_accel_ != esm_has_gyro_) GNSS_RX_EVENT(GNSS_RX_ASSEMBLY_REPLACED,1);
      esm_has_accel_ = false; esm_has_gyro_ = false;
    }
    esm_tow_ms_ = tow_ms;
    esm_wnc_ = week;

    for (uint8_t i = 0; i < n; ++i) {
      const int base = sbf::ext_sensor_meas::OFFSET_SUBBLOCKS + i * sb_length;
      const uint8_t type = sbf_body_[base + sbf::ext_sensor_meas::SB_OFFSET_TYPE];

      // S2 pp332-333: never combine different Source/SensorModel values.
      // Unsupported sub-block types do not disturb an IMU pair in progress.
      if (type != sbf::ext_sensor_meas::TYPE_ACCEL && type != sbf::ext_sensor_meas::TYPE_GYRO) {
        GNSS_RX_EVENT(GNSS_RX_SELECTION,1); continue;
      }
      const uint16_t sensor = wire::u16(sbf_body_.data() + base);
      if (sensor != esm_sensor_) {
        if (esm_has_accel_ || esm_has_gyro_) GNSS_RX_EVENT(GNSS_RX_ASSEMBLY_REPLACED,1);
        esm_has_accel_ = esm_has_gyro_ = false;
        esm_sensor_ = sensor;
      }
      // Saturated or scrambled samples cannot be published as usable SI values.
      if (sbf_body_[base + 3] & 0x06u) {
        GNSS_RX_EVENT(GNSS_RX_QUALITY,1); GNSS_RX_EVENT(GNSS_RX_IMU_REJECT,1);
        esm_has_accel_ = esm_has_gyro_ = false; continue;
      }

      if (type == sbf::ext_sensor_meas::TYPE_ACCEL) {
        std::memcpy(&esm_accel_[0], sbf_body_.data() + base + sbf::ext_sensor_meas::SB_OFFSET_X, 8);
        std::memcpy(&esm_accel_[1], sbf_body_.data() + base + sbf::ext_sensor_meas::SB_OFFSET_Y, 8);
        std::memcpy(&esm_accel_[2], sbf_body_.data() + base + sbf::ext_sensor_meas::SB_OFFSET_Z, 8);
        esm_has_accel_ = true;
      } else if (type == sbf::ext_sensor_meas::TYPE_GYRO) {
        std::memcpy(&esm_gyro_[0], sbf_body_.data() + base + sbf::ext_sensor_meas::SB_OFFSET_X, 8);
        std::memcpy(&esm_gyro_[1], sbf_body_.data() + base + sbf::ext_sensor_meas::SB_OFFSET_Y, 8);
        std::memcpy(&esm_gyro_[2], sbf_body_.data() + base + sbf::ext_sensor_meas::SB_OFFSET_Z, 8);
        esm_has_gyro_ = true;
      } else { GNSS_RX_EVENT(GNSS_RX_SELECTION,1); }
    }

    if (esm_has_accel_ != esm_has_gyro_) GNSS_RX_EVENT(GNSS_RX_IMU_WAIT,1);
    if (esm_has_accel_ && esm_has_gyro_) publishEsmAccum();
  }

  void publishEsmAccum() {
    if (!esm_has_accel_ || !esm_has_gyro_) return;
    for (int i = 0; i < 3; ++i) {
      if (!std::isfinite(esm_accel_[i]) || !std::isfinite(esm_gyro_[i]) ||
          esm_accel_[i] == -2e10 || esm_gyro_[i] == -2e10) {
        GNSS_RX_EVENT(GNSS_RX_VALUE,1); GNSS_RX_EVENT(GNSS_RX_IMU_REJECT,1);
        esm_has_accel_ = esm_has_gyro_ = false; return;
      }
    }
    sensor_msgs::msg::Imu imu;
    const bool esm_time_valid =
        esm_tow_ms_ < 604800000u && esm_wnc_ > 0 && esm_wnc_ != 0xFFFFu;
    const auto measurement_stamp = time_base_.stamp(
        config_.use_gps_timestamp, esm_time_valid,
        esm_time_valid ? gpst2time(static_cast<int>(esm_wnc_),
                                   esm_tow_ms_ / 1000.0)
                       : gtime_t{},
        now(), get_logger(), *get_clock());
    if (!measurement_stamp) { receiver_health_.timeWithheld(); return; }
    imu.header.stamp = *measurement_stamp;
    imu.header.frame_id = config_.frame_id;

    // Orientation is not estimated by this message at all -> "-1" (field not provided).
    auto orient_unk = ins::makeUnknownCovariance();
    std::copy(orient_unk.begin(), orient_unk.end(), imu.orientation_covariance.begin());

    // Acceleration/angular-rate VALUES are provided below; only their
    // covariance is unknown -> all-zero, not "-1" (which would claim the
    // measurement itself is absent).
    auto meas_unk = ins::makeZeroCovariance();

    imu.linear_acceleration.x = esm_accel_[0];
    imu.linear_acceleration.y = esm_accel_[1];
    imu.linear_acceleration.z = esm_accel_[2];
    std::copy(meas_unk.begin(), meas_unk.end(), imu.linear_acceleration_covariance.begin());

    imu.angular_velocity.x = esm_gyro_[0] * (M_PI / 180.0);
    imu.angular_velocity.y = esm_gyro_[1] * (M_PI / 180.0);
    imu.angular_velocity.z = esm_gyro_[2] * (M_PI / 180.0);
    std::copy(meas_unk.begin(), meas_unk.end(), imu.angular_velocity_covariance.begin());

    imu_raw_pub_->publish(imu);

    esm_has_accel_ = false;
    esm_has_gyro_  = false;
    esm_accel_[0] = esm_accel_[1] = esm_accel_[2] = 0.0;
    esm_gyro_[0]  = esm_gyro_[1]  = esm_gyro_[2]  = 0.0;
  }

  // The messages.*_nav flags only control which decoded nav blocks the
  // receiver is told to emit; decoding is done by RTKLIB input_sbf().

  // Message Handling

  void handleNmeaSentence(const std::string& sentence) {
    if (!nmea_parser_.parseSentence(sentence, nmea_solution_)) return;
    if (source_ == SolutionSource::NMEA) publishSolution(nmea_solution_);
  }

  void handleDecodeResult(int result) {
    receiver_health_.backendResult(result);
    if (result < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Rejected receiver payload (invalid layout, time, version or content)");
      return;
    }
    switch (result) {
      case 1:  publishObservations(); break;
      case 2:  publishEphemerides();  break;
      case 9:  publishIonosphere();   break;
      default: break;
    }
  }

  // Solution Publishing

  void publishSolution(msg::GnssSolution& sol) {
    // BINARY: SBF PVT blocks carry WNc+TOW — trust them; raw_.time is only a
    // fallback. NMEA: NmeaParser filled week/tow from the sentence.
    gtime_t t_gpst{};
    int week_for_stamp = 0;
    if (source_ == SolutionSource::BINARY) {
      if (sol.time_week > 0) {
        week_for_stamp = static_cast<int>(sol.time_week);
        t_gpst = gpst2time(week_for_stamp, sol.time_tow);
      } else {
        const double tow = time2gpst(raw_.time, &week_for_stamp);
        if (raw_.time.time != 0 && week_for_stamp > 0) {
          sol.time_week = static_cast<uint32_t>(week_for_stamp);
          sol.time_tow  = tow;
        } else {
          week_for_stamp = 0;
        }
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

    const auto measurement_stamp = time_base_.stamp(
        config_.use_gps_timestamp, week_for_stamp > 0 && std::isfinite(sol.time_tow) && sol.time_tow >= 0 && sol.time_tow < 604800,
        t_gpst, now(), get_logger(), *get_clock());
    if (!measurement_stamp) { receiver_health_.timeWithheld(); return; }
    sol.header.stamp = *measurement_stamp;
    sol.header.frame_id = config_.frame_id;

    if (decoder_common::finalizeReceiverSolution(
            sol, has_local_origin_, local_origin_ecef_, local_origin_pos_)) {
      RCLCPP_INFO(get_logger(), "Set local ENU origin: lat=%.6f lon=%.6f alt=%.2f",
                  local_origin_pos_[0] * R2D, local_origin_pos_[1] * R2D,
                  local_origin_pos_[2]);
    }

    sol_pub_->publish(sol);
  }

  // Observation Publishing

  void publishObservations() {
    if (raw_.obs.n <= 0) return;
    receiver_health_.observationCapacity(raw_.obs.n, MAXOBS);

    // obs.data[0].time, not raw_.time (already advanced to the next epoch).
    int week = 0;
    const gtime_t obs_time = raw_.obs.data[0].time;
    const double tow = time2gpst(obs_time, &week);

    msg::GnssObservations msg;
    const auto measurement_stamp = time_base_.stamp(
        config_.use_gps_timestamp, week > 0,
        obs_time, now(), get_logger(), *get_clock());
    if (!measurement_stamp) { receiver_health_.timeWithheld(); return; }
    msg.header.stamp = *measurement_stamp;
    msg.header.frame_id = config_.frame_id;
    msg.week = static_cast<uint32_t>(week);
    msg.tow  = tow;

    decoder_common::SatelliteCount sat_count{};
    for (int i = 0; i < raw_.obs.n; ++i) {
      const obsd_t& obs = raw_.obs.data[i];
      decoder_common::countSatellite(obs.sat, sat_count);
      decoder_common::appendObservations(obs, msg.observations, &raw_.nav);
    }

    obs_pub_->publish(msg);
    receiver_health_.published(msg.week, msg.tow, msg.observations.size());

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "Published obs: week=%d tow=%.3f n=%zu sats(G/R/E/J/C/I/S/U)=(%d/%d/%d/%d/%d/%d/%d/%d)",
      week, tow, msg.observations.size(),
      sat_count.gps, sat_count.glo, sat_count.gal, sat_count.qzs,
      sat_count.bds, sat_count.irn, sat_count.sbs, sat_count.unknown);
  }

  // Ephemeris Publishing (snapshot-on-change + heartbeat via EphemerisStore)

  // Called by handleDecodeResult(2) — ingest RTKLIB raw_.nav arrays into the
  // store, then publish a snapshot if anything changed.
  void publishEphemerides() {
    bool changed = false;
    int prn = 0;
    const int sys = satsys(raw_.ephsat, &prn);
    if (sys == SYS_GLO && prn >= 1 && prn <= raw_.nav.ng) {
      changed = eph_store_.ingestGeph(raw_.nav.geph[prn-1]);
    } else if (raw_.ephsat > 0 && raw_.ephsat <= MAXSAT && sys != SYS_SBS) {
      const int index = raw_.ephsat-1 + MAXSAT*raw_.ephset;
      if (index >= 0 && index < raw_.nav.n)
        changed = eph_store_.ingestEph(raw_.nav.eph[index]);
    }
    changed = eph_store_.ingestIon(raw_.nav) || changed;
    if (changed) publishSnapshot();
  }

  // Broadcast ionosphere parameters (backend status 9): part of the snapshot.
  void publishIonosphere() {
    if (eph_store_.ingestIon(raw_.nav)) publishSnapshot();
  }

  // Heartbeat hook — call from the poll timer; emits a snapshot if the
  // configured period has elapsed since the last publish.
  void maybePublishHeartbeat() {
    if (eph_store_.heartbeatDue(now())) publishSnapshot();
  }

  void publishSnapshot() {
    auto msg = eph_store_.buildSnapshot(now());
    receiver_health_.snapshotRejected(eph_store_.lastSnapshotRejected());
    eph_pub_->publish(msg);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "Published ephemeris snapshot: GNSS=%zu GLO=%zu",
      msg.gnss_ephemeris.size(), msg.glonass_ephemeris.size());
  }


  // Member Variables

  SbfConfig config_;

  rclcpp::Publisher<msg::GnssObservations>::SharedPtr  obs_pub_;
  rclcpp::Publisher<msg::GnssEphemerides>::SharedPtr   eph_pub_;
  rclcpp::Publisher<msg::GnssSolution>::SharedPtr      sol_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr  imu_raw_pub_;       // ExtSensorMeas accel/gyro (uncalibrated)
  rclcpp::TimerBase::SharedPtr timer_;

  wire::Framer wire_framer_{wire::Protocol::Sbf};
  std::chrono::steady_clock::time_point wire_last_input_{std::chrono::steady_clock::now()};
  uint64_t wire_last_rejected_{0};
  stream_t stream_{};
  raw_t    raw_{};

  // One time base per session, latched at the first published message and
  // never changed. Shared by EVERY topic this node publishes, which is what
  // structurally guarantees the IMU and the GNSS topics cannot end up on
  // different bases. See decoder_common::TimeBaseLatch.
  decoder_common::TimeBaseLatch time_base_;

  bool is_serial_connection_{false};

  // RTCM relay (PC -> receiver): STRSVR-style TCP server -> serial
  decoder_common::RtcmRelayServer relay_;

  // SBF mini-framer state
  int                  sbf_state_{0};
  uint16_t             sbf_id_{0};
  uint16_t             sbf_len_{0};
  int                  sbf_body_pos_{0};
  std::vector<uint8_t> sbf_body_;

  // ExtSensorMeas accumulator, keyed by week/TOW and Source/SensorModel
  uint32_t esm_tow_ms_{0xFFFFFFFFu};
  uint16_t esm_wnc_{0xFFFFu};
  uint16_t esm_sensor_{0xFFFFu};
  double   esm_accel_[3]{0.0, 0.0, 0.0};
  double   esm_gyro_[3]{0.0, 0.0, 0.0};
  bool     esm_has_accel_{false};
  bool     esm_has_gyro_{false};

  // NMEA parsing state
  gnss_utils::NmeaParser nmea_parser_;
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
  try {
    rclcpp::spin(std::make_shared<gnss_ros_standardization::SbfDriverNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("sbf_driver_node"), "Startup/runtime failure: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
