#include "gnss_ros_standardization/receiver_health.hpp"
#include "gnss_ros_standardization/receiver_session.hpp"
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
#include "gnss_ros_standardization/novatel_protocol.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"

using namespace std::chrono_literals;
namespace ins = gnss_ros_standardization::ins_utils;

namespace gnss_ros_standardization {

namespace {

constexpr auto kTimerInterval  = 10ms;


}  // namespace

/// @brief Configuration structure for NovAtel driver
struct NovatelConfig {
  std::string stream_path{"serial:///dev/ttyUSB0:115200"};
  std::string frame_id{"gnss_link"};
  int publish_rate{1};
  std::string receiver_port{"THISPORT"};
  bool configure_on_startup{true};

  // Observation / PVT
  bool enable_rangecmp{true};
  bool enable_range{false};
  bool enable_bestpos{false};
  bool enable_bestvel{false};
  bool enable_psrdop{false};
  bool enable_bestxyz{false};

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

  // RTCM relay (PC -> receiver) for on-chip RTK — see openRtcmRelay(). NovAtel
  // needs a dedicated RTCMV3 correction port (`rtcm_input.receiver_stream`); the main
  // port stays NOVATEL for commands+logs.
  bool rtcm_relay_enabled{false};
  std::string rtcm_relay_listen{"tcpsvr://:5558"};
  std::string rtcm_relay_output{""};
  std::string rtcm_relay_correction_port{"THISPORT"};

  // Ephemeris snapshot behavior
  double ephemeris_snapshot_period_s{30.0};
  double ephemeris_max_age_s{0.0};  // 0 = keep all

  bool use_gps_timestamp{false};

  // ENU local origin
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

    openRtcmRelay();
    if (config_.configure_on_startup && (config_.enable_range || config_.enable_rangecmp)) receiver_health_.expectObservations(config_.publish_rate);
    startPolling();
    RCLCPP_INFO(get_logger(), "NovAtel Driver initialized");
  }

  ~NovatelDriverNode() override {
    try {
      strclose(&stream_);
      relay_.close();
      if (relay_out_open_) strclose(&relay_out_);
    } catch (...) {}
    free_raw(&raw_);
  }

 private:
  // Initialization

  decoder_common::EpochFlushPolicy epoch_flush_;
  wire::ReceiverHealth receiver_health_;
  void initializeParameters() {
    receiver_health_.configure(5.0);
    receiver_health_.limits(MAXOBS, NFREQ, NEXOBS, MAXPRNCMP);
    epoch_flush_.configure(0.05);
    declare_parameter<std::string>("stream_path",          config_.stream_path);
    // frame_id: ROS TF frame name attached to the GNSS antenna phase center.
    // Default "gnss_link" — override in launch to integrate with vehicle TF tree.
    declare_parameter<std::string>("frame_id",             config_.frame_id);
    declare_parameter<int>        ("rate_hz",         config_.publish_rate);
    declare_parameter<std::string>("receiver_port",        config_.receiver_port);
    declare_parameter<bool>       ("configure_on_startup", config_.configure_on_startup);

    declare_parameter<bool>("messages.rangecmp",    config_.enable_rangecmp);
    declare_parameter<bool>("messages.range",       config_.enable_range);
    declare_parameter<bool>("messages.bestpos",     config_.enable_bestpos);
    declare_parameter<bool>("messages.bestvel",     config_.enable_bestvel);
    declare_parameter<bool>("messages.psrdop",      config_.enable_psrdop);
    declare_parameter<bool>("messages.bestxyz",     config_.enable_bestxyz);
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

    declare_parameter<std::string>("topics.observation", config_.observation_topic);
    declare_parameter<std::string>("topics.ephemeris",   config_.ephemeris_topic);
    declare_parameter<std::string>("topics.solution",    config_.solution_topic);
    declare_parameter<std::string>("topics.imu",         config_.imu_topic);
    declare_parameter<std::string>("topics.imu_raw",     config_.imu_raw_topic);
    declare_parameter<bool>("rtcm_input.enable",          config_.rtcm_relay_enabled);
    declare_parameter<std::string>("rtcm_input.listen",    config_.rtcm_relay_listen);
    declare_parameter<std::string>("rtcm_input.receiver_stream",          config_.rtcm_relay_output);
    declare_parameter<std::string>("rtcm_input.receiver_port", config_.rtcm_relay_correction_port);

    declare_parameter<double>("ephemeris.snapshot_period_s", config_.ephemeris_snapshot_period_s);
    declare_parameter<double>("ephemeris.max_age_s",         config_.ephemeris_max_age_s);
    declare_parameter<bool>("use_gps_timestamp",             config_.use_gps_timestamp);

    config_.stream_path          = get_parameter("stream_path").as_string();
    config_.frame_id             = get_parameter("frame_id").as_string();
    config_.publish_rate         = get_parameter("rate_hz").as_int();
    gnss_utils::validatePublishRate(config_.publish_rate, /*fallback=*/1, /*max_hz=*/20, get_logger());
    config_.receiver_port        = get_parameter("receiver_port").as_string();
    config_.configure_on_startup = get_parameter("configure_on_startup").as_bool();

    config_.enable_rangecmp    = get_parameter("messages.rangecmp").as_bool();
    config_.enable_range       = get_parameter("messages.range").as_bool();
    config_.enable_bestpos     = get_parameter("messages.bestpos").as_bool();
    config_.enable_bestvel     = get_parameter("messages.bestvel").as_bool();
    config_.enable_psrdop      = get_parameter("messages.psrdop").as_bool();
    config_.enable_bestxyz     = get_parameter("messages.bestxyz").as_bool();
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

    config_.observation_topic  = get_parameter("topics.observation").as_string();
    config_.ephemeris_topic    = get_parameter("topics.ephemeris").as_string();
    config_.solution_topic     = get_parameter("topics.solution").as_string();
    config_.imu_topic          = get_parameter("topics.imu").as_string();
    config_.imu_raw_topic      = get_parameter("topics.imu_raw").as_string();
    config_.rtcm_relay_enabled = get_parameter("rtcm_input.enable").as_bool();
    config_.rtcm_relay_listen  = get_parameter("rtcm_input.listen").as_string();
    config_.rtcm_relay_output          = get_parameter("rtcm_input.receiver_stream").as_string();
    config_.rtcm_relay_correction_port = get_parameter("rtcm_input.receiver_port").as_string();

    config_.ephemeris_snapshot_period_s = get_parameter("ephemeris.snapshot_period_s").as_double();
    config_.ephemeris_max_age_s         = get_parameter("ephemeris.max_age_s").as_double();
    config_.use_gps_timestamp           = get_parameter("use_gps_timestamp").as_bool();

    eph_store_.setSnapshotPeriod(config_.ephemeris_snapshot_period_s);
    eph_store_.setMaxAge(config_.ephemeris_max_age_s);
    constexpr int history_limit = 8;
    eph_store_.setHistoryPerSatellite(static_cast<size_t>(history_limit));

    // Lock solution source at startup. BINARY if BESTPOS is enabled, else NMEA.
    source_ = config_.enable_bestpos ? SolutionSource::BINARY : SolutionSource::NMEA;
    initPendingExpectedMask();
    start_time_ = now();
    RCLCPP_INFO(get_logger(), "Solution source locked: %s",
                source_ == SolutionSource::BINARY ? "BINARY (BESTPOS/BESTVEL)" : "NMEA");
  }

  void initializePublishers() {
    decoder_common::configureNmeaDate(*this, nmea_parser_);
    obs_pub_ = create_publisher<msg::GnssObservations>(config_.observation_topic, 10);
    eph_pub_ = create_publisher<msg::GnssEphemerides>(config_.ephemeris_topic, rclcpp::QoS(256).transient_local()  /* depth 256: latch full ephemeris set, not just the last satellite (per-sat messages) */);
    sol_pub_ = create_publisher<msg::GnssSolution>(config_.solution_topic, 10);
    imu_pub_     = create_publisher<sensor_msgs::msg::Imu>(config_.imu_topic, decoder_common::kImuPublisherDepth);
    imu_raw_pub_ = create_publisher<sensor_msgs::msg::Imu>(config_.imu_raw_topic, decoder_common::kImuPublisherDepth);

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
    RCLCPP_INFO(get_logger(), "  Observation  : RANGECMP=%s RANGE=%s",
      on(config_.enable_rangecmp), on(config_.enable_range));
    RCLCPP_INFO(get_logger(), "  PVT (binary) : BESTPOS=%s BESTVEL=%s PSRDOP=%s BESTXYZ=%s",
      on(config_.enable_bestpos), on(config_.enable_bestvel),
      on(config_.enable_psrdop), on(config_.enable_bestxyz));
    RCLCPP_INFO(get_logger(), "  Ephemeris    : GPS=%s GLO=%s GAL=%s BDS=%s QZS=%s NavIC=%s IONUTC=%s",
      on(config_.enable_gps_ephem), on(config_.enable_glo_ephem), on(config_.enable_gal_ephem),
      on(config_.enable_bds_ephem), on(config_.enable_qzs_ephem), on(config_.enable_navic_ephem),
      on(config_.enable_ionutc));
    RCLCPP_INFO(get_logger(), "  NMEA         : GGA=%s RMC=%s GSA=%s GST=%s",
      on(config_.enable_nmea_gpgga), on(config_.enable_nmea_gprmc),
      on(config_.enable_nmea_gpgsa), on(config_.enable_nmea_gpgst));
    RCLCPP_INFO(get_logger(), "  IMU          : RAWIMUSX=%s CORRIMUDATA=%s",
      on(config_.enable_rawimusx), on(config_.enable_corrimudata));
    RCLCPP_INFO(get_logger(), "  GPS timestamp : %s", on(config_.use_gps_timestamp));
    if (has_local_origin_) {
      RCLCPP_INFO(get_logger(), "  ENU origin    : fixed (lat=%.6f lon=%.6f alt=%.2f)",
        local_origin_pos_[0] * R2D, local_origin_pos_[1] * R2D, local_origin_pos_[2]);
    } else {
      RCLCPP_INFO(get_logger(), "  ENU origin    : auto (first valid solution)");
    }
  }

  void initializeDecoder() {
    if (init_raw(&raw_, STRFMT_OEM4) != 1) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize raw decoder");
      throw std::runtime_error("init_raw failed");
    }
    std::strcpy(raw_.opt, "-EPHALL");
  }

  void startPolling() {
    timer_ = create_wall_timer(kTimerInterval, std::bind(&NovatelDriverNode::pollStream, this));
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

    // NovAtel: corrections go to a DEDICATED port (separate from the NOVATEL
    // command/log port). Open it, then put IT into RTCMV3 input mode by sending
    // INTERFACEMODE over that port itself (THISPORT = this very port) while it is
    // still in NOVATEL mode — so the user only needs to point `output` at a
    // second /dev device; no NovAtel logical port name needs to be known.
    if (!config_.rtcm_relay_output.empty()) {
      if (!decoder_common::openStreamPath(get_logger(), relay_out_,
                                          config_.rtcm_relay_output,
                                          STR_MODE_RW, "RTCM relay output")) {
        throw std::runtime_error("RTCM relay output stropen failed");
      }
      relay_out_open_ = true;

      if (config_.configure_on_startup) {
        // correction_port defaults to "THISPORT" (the correction port itself).
        const std::string cmd = std::string(novatel::CMD_INTERFACEMODE) + " " +
                                config_.rtcm_relay_correction_port + " RTCMV3 NONE OFF\r\n";
        if (!decoder_common::writeAll(relay_out_, reinterpret_cast<const uint8_t*>(cmd.data()), cmd.size()))
      throw std::runtime_error("Correction port command write failed");
        std::this_thread::sleep_for(200ms);
        RCLCPP_INFO(get_logger(), "RTCM relay: sent RTCMV3 port command (%s); acceptance unverified because replies are disabled",
                    config_.rtcm_relay_correction_port.c_str());
      }

      RCLCPP_INFO(get_logger(), "RTCM relay: corrections go to dedicated port '%s'",
                  decoder_common::redactStreamUri(config_.rtcm_relay_output).c_str());
    } else {
      RCLCPP_WARN(get_logger(),
        "rtcm_input.receiver_stream is empty: corrections go to the main port. NovAtel "
        "cannot mix RTCM input with NOVATEL logs on one logical port — set "
        "rtcm_input.receiver_stream to a second /dev device for on-chip RTK.");
    }
  }

  // Receiver Configuration

  void sendCommand(const std::string& cmd) {
    if (cmd.empty()) return;
    decoder_common::configureAscii(stream_, wire::Protocol::Novatel, cmd,
        [this](uint8_t byte) {
          parseOem4Byte(byte);
        });
  }

  void configureReceiverOem4(const std::string& port_pfx, const std::string& ontime, const std::string& onchanged) {
    // NMEA sentences first — issued while the port is still quiet (caller
    // has just sent UNLOGALL). Sending NMEA after high-rate binary `LOG`s
    // risks the command being silently dropped under port saturation, as
    // observed on the SBF driver. Each command waits for its receiver response.
    if (config_.enable_nmea_gpgga) sendCommand("LOG " + port_pfx + novatel::LOG_GPGGA + ontime);

    if (config_.enable_nmea_gprmc) sendCommand("LOG " + port_pfx + novatel::LOG_GPRMC + ontime);

    if (config_.enable_nmea_gpgsa) sendCommand("LOG " + port_pfx + novatel::LOG_GPGSA + ontime);

    if (config_.enable_nmea_gpgst) sendCommand("LOG " + port_pfx + novatel::LOG_GPGST + ontime);

    if (config_.enable_rangecmp) sendCommand("LOG " + port_pfx + novatel::LOG_RANGECMP + ontime);

    if (config_.enable_range)    sendCommand("LOG " + port_pfx + novatel::LOG_RANGE + ontime);

    if (config_.enable_bestpos)  sendCommand("LOG " + port_pfx + novatel::LOG_BESTPOS + ontime);

    if (config_.enable_bestvel)  sendCommand("LOG " + port_pfx + novatel::LOG_BESTVEL + ontime);

    if (config_.enable_psrdop)   sendCommand("LOG " + port_pfx + novatel::LOG_PSRDOPB + ontime);

    if (config_.enable_bestxyz)  sendCommand("LOG " + port_pfx + novatel::LOG_BESTXYZB + ontime);

    if (config_.enable_gps_ephem) {
      sendCommand("LOG " + port_pfx + novatel::LOG_RAWEPHEM + onchanged);
    }

    if (config_.enable_glo_ephem) {
      sendCommand("LOG " + port_pfx + novatel::LOG_GLOEPHEMERIS + onchanged);
    }

    if (config_.enable_gal_ephem) {
      sendCommand("LOG " + port_pfx + novatel::LOG_GALEPHEMERIS + onchanged);
      // Note: GALINAVEPHEMERISB (ID 1309) is not supported by RTKLIB.
      // GALEPHEMERISB (ID 1122) already contains both I/NAV and F/NAV data.
    }

    if (config_.enable_bds_ephem) {
      sendCommand("LOG " + port_pfx + novatel::LOG_BDSEPHEMERIS + onchanged);
    }

    if (config_.enable_qzs_ephem) {
      sendCommand("LOG " + port_pfx + novatel::LOG_QZSSRAWEPHEM + onchanged);
    }

    if (config_.enable_navic_ephem) {
      sendCommand("LOG " + port_pfx + novatel::LOG_NAVICEPHEMERIS + onchanged);
    }

    if (config_.enable_ionutc) {
      sendCommand("LOG " + port_pfx + novatel::LOG_IONUTC + onchanged);
      sendCommand("LOG " + port_pfx + novatel::LOG_GALIONO + onchanged);
      sendCommand("LOG " + port_pfx + novatel::LOG_QZSSIONUTC + onchanged);
    }

    // IMU outputs (require IMU-connected receiver; CORRIMUDATA additionally requires SPAN)
    if (config_.enable_rawimusx)    sendCommand(std::string("LOG ")   + port_pfx + novatel::LOG_RAWIMUSXB    + " ONNEW");

    if (config_.enable_corrimudata) sendCommand(std::string("LOG ")   + port_pfx + novatel::LOG_CORRIMUDATAB + " ONNEW");
  }

  void configureReceiver() {
    RCLCPP_INFO(get_logger(), "Configuring receiver...");

    sendCommand(novatel::CMD_UNLOGALL);
    if (!config_.receiver_port.empty()) {
      sendCommand(std::string(novatel::CMD_UNLOGALL) + " " + config_.receiver_port);
    }
    std::this_thread::sleep_for(500ms);

    // Note: the correction port is put into RTCMV3 mode in openRtcmRelay(), by
    // sending INTERFACEMODE over that port itself (not here on the main port).

    const std::string port_pfx     = config_.receiver_port + " ";
    const std::string ontime        = " ONTIME " + novatel::getIntervalString(config_.publish_rate);
    const std::string onchanged     = " ONCHANGED";

    configureReceiverOem4(port_pfx, ontime, onchanged);

    RCLCPP_INFO(get_logger(), "Receiver acknowledged configuration commands.");
  }

  // Polling

  wire::StreamSession stream_session_;
  wire::ReceiverTimeline receiver_timeline_;
  void resetReceiverSession(bool disconnected) {
    // Release validated pending data before retiring the old session. Partial
    // wire frames and compressed references are never carried across sessions.
    flushPending();
    wire::resetRaw(raw_,STRFMT_OEM4);
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
    initPendingExpectedMask();
    oem4_time_quality_.reset(); oem4_time_valid_=false; binary_time_valid_=false;
    has_corrimu_prev_=false; has_rawimu_prev_=false;
    corrimu_prev_seconds_=0; rawimu_prev_seconds_=0;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Receiver session reset: %s; waiting for fresh time/navigation references",
                disconnected ? "connection lost" : "measurement rewind or clock reset");
  }

  void pollStream() {
    // RTCM relay (STRSVR-style): drain corrections from the TCP server and write
    // them to the dedicated RTCMV3 port if configured, else the main stream
    // (single-port fallback). Same poll thread as the read below, so no lock.
    relay_.drainTo(get_logger(), *get_clock(),
                   relay_out_open_ ? relay_out_ : stream_);

    uint8_t buffer[novatel::READ_BUFFER_SIZE];
    for (int batch=0; batch<16 && rclcpp::ok(); ++batch) {
    const int bytes_read = strread(&stream_, buffer, sizeof(buffer));
    if (stream_session_.poll(stream_)) resetReceiverSession(true);
    const auto wire_now = std::chrono::steady_clock::now();
    receiver_health_.input(bytes_read);
    receiver_health_.framing(wire_framer_);
    receiver_health_.pvtPending(pending_.has_pvt || pending_.cov_received);
    if (has_rawimu_prev_ || has_corrimu_prev_)
      receiver_health_.imuPending((has_rawimu_prev_ && rawimu_period_==0) || (has_corrimu_prev_ && corrimu_period_==0));
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
      parseOem4Byte(buffer[i]);
    }
    if (bytes_read < static_cast<int>(sizeof(buffer))) break;
    }


    maybePublishHeartbeat();
    warnIfBinaryStarvation();
    maybeWatchdogFlushPendingPvt();
  }

  // Message Handling

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

  // OEM4 mixed stream decoder (shared boundary with RTKLIB, for IMU and PVT logs)
  //
  // OEM4 binary frame: SYNC(0xAA,0x44,0x12) HDRLEN(1) MSGID(2,LE) MSGTYPE(1)
  //                    PORT(1) MSGLEN(2,LE) ... rest of header ... BODY CRC32

  void dispatchFrame(const wire::Frame& f) {
      auto diagnostic_scope = receiver_health_.backendScope();
      receiver_health_.frame((unsigned(f.cls)<<16)|f.id, f.revision);
      receiver_health_.inspect(wire::Protocol::Novatel,f);
      oem4_time_valid_ = oem4_time_quality_.observe(f);
      if (!oem4_time_valid_) {
        receiver_health_.timeUnresolved();
        receiver_health_.timeWithheld();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "NovAtel invalid receiver time (status=%u, week=%u); output withheld",
            f.time_status, f.week);
        return; // Do not let invalid time reset session, PVT, or IMU state.
      }
      if (receiver_timeline_.frame(wire::Protocol::Novatel,f)) {
        resetReceiverSession(false);
        receiver_timeline_.frame(wire::Protocol::Novatel,f);
        oem4_time_valid_ = oem4_time_quality_.observe(f);
      }
      for (uint8_t byte : novatel::backendFrame(f.bytes)) {
        const int status = input_oem4(&raw_, byte);
        handleDecodeResult(status);
      }
      oem4_msg_id_ = f.id;
      oem4_gps_week_ = f.week;
      oem4_gps_ms_ = f.tow_ms;
      oem4_body_ = f.payload;
      handleOem4Frame();
  }

  void parseOem4Byte(uint8_t byte) {
    wire_framer_.feed(byte, [this](const wire::Frame& f) { dispatchFrame(f); },
        [this](const std::string& sentence) { handleNmeaSentence(sentence); });
  }

  void handleOem4Frame() {
    if (oem4_msg_id_ == novatel::ID_RAWIMUSX)    handleRawImuSx();
    if (oem4_msg_id_ == novatel::ID_CORRIMUDATA) handleCorrImuData();
    if (oem4_msg_id_ == novatel::ID_BESTPOS)     handleBestPos();
    if (oem4_msg_id_ == novatel::ID_BESTVEL)     handleBestVel();
    if (oem4_msg_id_ == novatel::ID_PSRDOP)      handlePsrDop();
    if (oem4_msg_id_ == novatel::ID_BESTXYZ)     handleBestXyz();
  }

  // PVT TOW aggregation: BEST* logs merge into pending_ keyed on the OEM4
  // header TOW; PSRDOP is cached separately (last_dop_).
  static constexpr uint8_t COV_BIT_VEL = 0x1;
  static constexpr uint8_t COV_BIT_XYZ = 0x2;

  struct PendingPvt {
    uint32_t tow_ms{UINT32_MAX};
    bool has_pvt{false};
    bool time_valid{false};
    uint32_t week{0};
    uint8_t cov_received{0};
    uint8_t cov_expected{0};      // derived from YAML config (fixed at startup)
    msg::GnssSolution buf{};
  };

  gnss_utils::DopCache last_dop_;
  uint32_t prev_pvt_tow_ms_{UINT32_MAX};
  uint32_t pvt_period_ms_{0};

  // Last flushed (week, tow) — late arrivals for it are skipped (orphan guard).
  uint32_t last_flushed_week_{0};
  uint32_t last_flushed_tow_ms_{UINT32_MAX};

  void initPendingExpectedMask() {
    pending_.cov_expected =
        (config_.enable_bestvel  ? COV_BIT_VEL : uint8_t{0}) |
        (config_.enable_bestxyz  ? COV_BIT_XYZ : uint8_t{0});
  }


  // Orphan guard: the current OEM4 frame's (week, tow) matches the most recently
  // flushed epoch. Caller should skip aggregation so no duplicate publish occurs.
  bool isLateOrphan() const {
    return last_flushed_tow_ms_ != UINT32_MAX &&
           oem4_gps_ms_   == last_flushed_tow_ms_ &&
           oem4_gps_week_ == last_flushed_week_;
  }

  void aggregateEpochBoundary() {
    if ((oem4_gps_ms_ != pending_.tow_ms || oem4_gps_week_ != pending_.week) &&
        (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = oem4_gps_ms_;
    pending_.week = oem4_gps_week_;
    pending_.time_valid = (pending_.has_pvt || pending_.cov_received) ?
        pending_.time_valid && oem4_time_valid_ : oem4_time_valid_;
    // BEST*/PSRDOP bodies carry no GPS time — (week, ms) comes from the OEM4
    // header seen by the mini-framer.
    pending_.buf.time_week = oem4_gps_week_;
    pending_.buf.time_tow  = oem4_gps_ms_ / 1000.0;
  }

  void handleBestPos() {
    if (!config_.enable_bestpos) return;
    if (isLateOrphan()) return;
    aggregateEpochBoundary();
    if (!novatel::pvt::parseBESTPOS(oem4_body_.data(), oem4_body_.size(), pending_.buf)) {
      GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return;
    }
    pending_.has_pvt = true;
    epoch_flush_.touch();
    ever_received_binary_ = true;
    // Flush at the epoch boundary or shared quiet-period deadline.
  }

  // PSRDOP: cached in last_dop_; applied at flush by applyDopWithStaleness()
  // (does not gate epoch completion).
  void handlePsrDop() {
    if (!config_.enable_psrdop) return;
    msg::GnssSolution scratch{};
    if (!novatel::pvt::parsePSRDOP(oem4_body_.data(), oem4_body_.size(), scratch)) { GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return; }
    last_dop_.valid  = true;
    last_dop_.week   = oem4_gps_week_;
    last_dop_.tow_ms = oem4_gps_ms_;
    last_dop_.gdop   = scratch.gdop;
    last_dop_.pdop   = scratch.pdop;
    last_dop_.hdop   = scratch.hdop;
    last_dop_.vdop   = scratch.vdop;
  }

  void handleBestXyz() {
    // BESTXYZ provides native ECEF covariance diagonals. Pos/vel values are not
    // overwritten — BESTPOS/BESTVEL remain the single source of truth.
    if (!config_.enable_bestxyz) return;
    if (isLateOrphan()) return;
    aggregateEpochBoundary();
    if (!novatel::pvt::parseBESTXYZ(oem4_body_.data(), oem4_body_.size(), pending_.buf)) { GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return; }
    pending_.cov_received  |= COV_BIT_XYZ;
    epoch_flush_.touch();
    // Flush at the epoch boundary or shared quiet-period deadline.
  }

  void flushPending() {
    const uint8_t expected = pending_.cov_expected;
    if (!pending_.has_pvt) {
      if (pending_.cov_received) receiver_health_.pvtIncompleteDiscarded();
      pending_ = {};
      pending_.cov_expected = expected;
      return;
    }
    binary_solution_ = pending_.buf;
    binary_time_valid_ = pending_.time_valid;
    if (!(pending_.cov_received & COV_BIT_VEL)) {
      binary_solution_.vel_enu.x = binary_solution_.vel_enu.y = binary_solution_.vel_enu.z =
          std::numeric_limits<double>::quiet_NaN();
    }
    if (!(pending_.cov_received & COV_BIT_XYZ)) {
      binary_solution_.vel_cov_ecef.fill(std::numeric_limits<double>::quiet_NaN());
      binary_solution_.vel_enu_cov.fill(std::numeric_limits<double>::quiet_NaN());
    }
    decoder_common::finalizeBinarySolutionGeometry(binary_solution_);
    // Covariance rotation direction is decided per epoch (not session-sticky).
    const bool has_xyz_this_epoch = (pending_.cov_received & COV_BIT_XYZ) != 0;
    rotatePosCovariance(binary_solution_, has_xyz_this_epoch &&
        std::isfinite(binary_solution_.pos_cov_ecef[0]));
    if (has_xyz_this_epoch) rotateVelCovariance(binary_solution_);

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

    if (source_ == SolutionSource::BINARY) publishSolution(binary_solution_);

    // Record this epoch as the last published (week, tow) for orphan-guard.
    last_flushed_week_   = pending_.week;
    last_flushed_tow_ms_ = pending_.tow_ms;

    pending_ = {};
    pending_.cov_expected = expected;
  }

  void maybeWatchdogFlushPendingPvt() {
    if (!pending_.has_pvt && !pending_.cov_received) return;
    if (epoch_flush_.due()) {
      flushPending();
    }
  }

  // `has_bestxyz_this_epoch` decides the rotation direction per epoch (NOT a
  // session-sticky flag) — see flushPending().
  void rotatePosCovariance(msg::GnssSolution& s, bool has_bestxyz_this_epoch) {
    const double lat = s.latitude  * D2R;
    const double lon = s.longitude * D2R;
    if (has_bestxyz_this_epoch) {
      double cov_ecef[9], cov_enu[9];
      std::copy(s.pos_cov_ecef.begin(), s.pos_cov_ecef.end(), cov_ecef);
      gnss_utils::rotateCovariance(cov_ecef, lat, lon, cov_enu);
      std::copy(std::begin(cov_enu), std::end(cov_enu), s.pos_enu_cov.begin());
    } else {
      double cov_enu[9], cov_ecef[9];
      std::copy(s.pos_enu_cov.begin(), s.pos_enu_cov.end(), cov_enu);
      gnss_utils::rotateCovarianceEnuToEcef(cov_enu, lat, lon, cov_ecef);
      std::copy(std::begin(cov_ecef), std::end(cov_ecef), s.pos_cov_ecef.begin());
    }
  }

  void rotateVelCovariance(msg::GnssSolution& s) {
    const double lat = s.latitude  * D2R;
    const double lon = s.longitude * D2R;
    double cov_ecef[9], cov_enu[9];
    std::copy(s.vel_cov_ecef.begin(), s.vel_cov_ecef.end(), cov_ecef);
    gnss_utils::rotateCovariance(cov_ecef, lat, lon, cov_enu);
    std::copy(std::begin(cov_enu), std::end(cov_enu), s.vel_enu_cov.begin());
  }

  void warnIfBinaryStarvation() {
    if (source_ != SolutionSource::BINARY) return;
    if (ever_received_binary_) return;
    if ((now() - start_time_).seconds() < 15.0) return;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
      "Solution source locked to BINARY (BESTPOS) but no PVT received yet — "
      "check receiver firmware/configuration.");
  }

  void handleBestVel() {
    if (!config_.enable_bestvel) return;
    if (isLateOrphan()) return;
    aggregateEpochBoundary();
    if (!novatel::pvt::parseBESTVEL(oem4_body_.data(), oem4_body_.size(), pending_.buf)) {
      GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return;
    }
    pending_.cov_received  |= COV_BIT_VEL;
    epoch_flush_.touch();
    // Flush at the epoch boundary or shared quiet-period deadline.
  }


  // RAWIMUSX body (40 B):
  //   IMUInfo(u1) IMUType(u1) Week(u2) Seconds(f8)
  //   IMUStatus(u4)
  //   ZAccel(i4) -YAccel(i4) XAccel(i4)  ZGyro(i4) -YGyro(i4) XGyro(i4)
  // Counts × IMU-type-specific scale = SI increments per IMU sample;
  // divide by dt (Seconds delta) to get m/s² and rad/s.
  // Note: -Y fields are the actual Y-axis value already negated by the receiver,
  // so reading them as int32 gives -Y; we negate again to recover the +Y reading.
  double rawimu_period_{0.0};
  double corrimu_period_{0.0};
  void handleRawImuSx() {
    if (oem4_body_.size() != 40) { GNSS_RX_EVENT(GNSS_RX_LENGTH,1); return; }

    const uint8_t  imu_type = oem4_body_[1];
    if (oem4_body_[0] & 0x03) { GNSS_RX_EVENT(GNSS_RX_QUALITY,1); GNSS_RX_EVENT(GNSS_RX_IMU_REJECT,1); return; }  // error or encrypted data
    uint16_t imu_week = 0;
    double seconds = 0.0;
    int32_t z_acc = 0, ny_acc = 0, x_acc = 0, z_gyr = 0, ny_gyr = 0, x_gyr = 0;
    std::memcpy(&imu_week, oem4_body_.data() + 2, 2);
    std::memcpy(&seconds,  oem4_body_.data() + 4, 8);
    std::memcpy(&z_acc,   oem4_body_.data() + 16, 4);
    std::memcpy(&ny_acc,  oem4_body_.data() + 20, 4);
    std::memcpy(&x_acc,   oem4_body_.data() + 24, 4);
    std::memcpy(&z_gyr,   oem4_body_.data() + 28, 4);
    std::memcpy(&ny_gyr,  oem4_body_.data() + 32, 4);
    std::memcpy(&x_gyr,   oem4_body_.data() + 36, 4);

    // Resolve scale: yaml override takes precedence if both fields are non-zero.
    novatel::ImuScale scale = novatel::getImuScale(imu_type);
    if (config_.imu_scale_override_accel != 0.0 && config_.imu_scale_override_gyro != 0.0) {
      scale = {config_.imu_scale_override_accel, config_.imu_scale_override_gyro};
    }
    if (!std::isfinite(scale.accel) || !std::isfinite(scale.gyro) || scale.accel <= 0.0 || scale.gyro <= 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Unknown NovAtel IMU type %u — RAWIMUSX skipped. Set imu_scale_override.{accel,gyro} "
        "in yaml or add this type to novatel_protocol.hpp.", imu_type);
      GNSS_RX_EVENT(GNSS_RX_VALUE,1); GNSS_RX_EVENT(GNSS_RX_IMU_REJECT,1); return;
    }

    if ((oem4_body_[0] & 0x03) || imu_week == 0 ||
        !std::isfinite(seconds) || seconds < 0 || seconds >= 604800) { GNSS_RX_EVENT(GNSS_RX_TIME,1); return; }
    seconds += static_cast<double>(imu_week) * 604800.0;
    if (!has_rawimu_prev_) {
      rawimu_prev_seconds_ = seconds;
      has_rawimu_prev_     = true;
      GNSS_RX_EVENT(GNSS_RX_IMU_WAIT,1);
      return;
    }
    const double dt = seconds - rawimu_prev_seconds_;
    rawimu_prev_seconds_ = seconds;
    if (dt <= 0.0 || dt > 1.0) { GNSS_RX_EVENT(GNSS_RX_TIME,1); rawimu_period_ = 0.0; return; }
    if (rawimu_period_ > 0.0 && dt > rawimu_period_ * 1.5) { rawimu_period_ = dt; GNSS_RX_EVENT(GNSS_RX_IMU_WAIT,1); return; }
    rawimu_period_ = rawimu_period_ == 0.0 ? dt : std::min(rawimu_period_, dt);  // sanity: skip across-week wraps

    // Recover +Y by negating the -Y fields, then scale and divide by dt.
    sensor_msgs::msg::Imu imu;
    const auto measurement_stamp = time_base_.stamp(
        config_.use_gps_timestamp, oem4_time_valid_ && imu_week > 0,
        imu_week > 0 ? gpst2time(imu_week, seconds - static_cast<double>(imu_week) * 604800.0) : gtime_t{}, now(), get_logger(), *get_clock());
    if (!measurement_stamp) { receiver_health_.timeWithheld(); return; }
    imu.header.stamp = *measurement_stamp;
    imu.header.frame_id = config_.frame_id;

    imu.orientation_covariance[0] = -1.0;  // unknown

    imu.angular_velocity.x =  x_gyr  * scale.gyro / dt;
    imu.angular_velocity.y = -static_cast<double>(ny_gyr) * scale.gyro / dt;
    imu.angular_velocity.z =  z_gyr  * scale.gyro / dt;
    imu.linear_acceleration.x =  x_acc  * scale.accel / dt;
    imu.linear_acceleration.y = -static_cast<double>(ny_acc) * scale.accel / dt;
    imu.linear_acceleration.z =  z_acc  * scale.accel / dt;

    // Values above are always provided by RAWIMU; only their covariance is
    // unknown -> all-zero, not "-1" (which claims the measurement is absent).
    auto unk = ins::makeZeroCovariance();
    std::copy(unk.begin(), unk.end(), imu.angular_velocity_covariance.begin());
    std::copy(unk.begin(), unk.end(), imu.linear_acceleration_covariance.begin());

    imu_raw_pub_->publish(imu);
  }

  // CORRIMUDATA body (60 B): Week(4) Seconds(8) PitchRate(8) RollRate(8) YawRate(8)
  //                          LateralAcc(8) LongitudinalAcc(8) VerticalAcc(8)
  // Rates and accelerations are SI increments accumulated over the IMU sampling
  // period; divide by dt (from successive Seconds fields) to recover rad/s and m/s².
  void handleCorrImuData() {
    if (oem4_body_.size() < 60) { GNSS_RX_EVENT(GNSS_RX_LENGTH,1); return; }

    uint32_t corrimu_week = 0;
    double seconds = 0.0, pitch_rate = 0.0, roll_rate = 0.0, yaw_rate = 0.0;
    double lat_acc = 0.0, lon_acc = 0.0, vert_acc = 0.0;
    std::memcpy(&corrimu_week, oem4_body_.data() +  0, 4);
    std::memcpy(&seconds,      oem4_body_.data() +  4, 8);
    std::memcpy(&pitch_rate,   oem4_body_.data() + 12, 8);
    std::memcpy(&roll_rate,    oem4_body_.data() + 20, 8);
    std::memcpy(&yaw_rate,     oem4_body_.data() + 28, 8);
    std::memcpy(&lat_acc,      oem4_body_.data() + 36, 8);
    std::memcpy(&lon_acc,      oem4_body_.data() + 44, 8);
    std::memcpy(&vert_acc,     oem4_body_.data() + 52, 8);

    if (corrimu_week == 0 || corrimu_week > 65535 || !std::isfinite(seconds) ||
        seconds < 0 || seconds >= 604800 || !std::isfinite(pitch_rate) ||
        !std::isfinite(roll_rate) || !std::isfinite(yaw_rate) ||
        !std::isfinite(lat_acc) || !std::isfinite(lon_acc) || !std::isfinite(vert_acc)) { GNSS_RX_EVENT(GNSS_RX_VALUE,1); GNSS_RX_EVENT(GNSS_RX_IMU_REJECT,1); return; }
    const double tow = seconds;
    seconds += static_cast<double>(corrimu_week) * 604800.0;
    if (!has_corrimu_prev_) {
      corrimu_prev_seconds_ = seconds;
      has_corrimu_prev_     = true;
      GNSS_RX_EVENT(GNSS_RX_IMU_WAIT,1);
      return;
    }
    const double dt = seconds - corrimu_prev_seconds_;
    corrimu_prev_seconds_ = seconds;
    if (dt <= 0.0 || dt > 1.0) { GNSS_RX_EVENT(GNSS_RX_TIME,1); corrimu_period_ = 0.0; return; }
    if (corrimu_period_ > 0.0 && dt > corrimu_period_ * 1.5) { corrimu_period_ = dt; GNSS_RX_EVENT(GNSS_RX_IMU_WAIT,1); return; }
    corrimu_period_ = corrimu_period_ == 0.0 ? dt : std::min(corrimu_period_, dt);  // sanity: skip across-week wraps and outliers

    sensor_msgs::msg::Imu imu;
    const auto measurement_stamp = time_base_.stamp(
        config_.use_gps_timestamp, oem4_time_valid_ && corrimu_week > 0 && std::isfinite(tow) && tow >= 0 && tow < 604800,
        corrimu_week > 0
            ? gpst2time(static_cast<int>(corrimu_week), tow) : gtime_t{}, now(), get_logger(), *get_clock());
    if (!measurement_stamp) { receiver_health_.timeWithheld(); return; }
    imu.header.stamp = *measurement_stamp;
    imu.header.frame_id = config_.frame_id;

    // NovAtel native right-handed frame: pitch/lateral X, roll/longitudinal Y, Z up.
    imu.angular_velocity.x    = pitch_rate / dt;
    imu.angular_velocity.y    = roll_rate / dt;
    imu.angular_velocity.z    = yaw_rate / dt;
    imu.linear_acceleration.x = lat_acc / dt;
    imu.linear_acceleration.y = lon_acc / dt;
    imu.linear_acceleration.z = vert_acc / dt;

    // orientation: not provided by CORRIMUDATA — leave identity, mark unknown
    imu.orientation_covariance[0] = -1.0;

    // Values above are always provided by CORRIMUDATA; only their covariance
    // is unknown -> all-zero, not "-1" (which claims the measurement is absent).
    auto unk = ins::makeZeroCovariance();
    std::copy(unk.begin(), unk.end(), imu.angular_velocity_covariance.begin());
    std::copy(unk.begin(), unk.end(), imu.linear_acceleration_covariance.begin());

    imu_pub_->publish(imu);
  }

  // Solution Publishing

  void publishSolution(msg::GnssSolution& sol) {
    // BINARY: the OEM4 header (week, ms) is already in time_week/time_tow —
    // trust them; raw_.time is only a fallback. NMEA: NmeaParser filled them.
    gtime_t t_gpst{};
    int week_for_stamp = 0;
    if (source_ == SolutionSource::BINARY) {
      if (binary_time_valid_ && sol.time_week > 0) {
        week_for_stamp = static_cast<int>(sol.time_week);
        t_gpst = gpst2time(week_for_stamp, sol.time_tow);
      } else {
        sol.time_week = 0;
        sol.time_tow = 0;
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
    if (!config_.enable_rangecmp && !config_.enable_range) return;
    if (raw_.obs.n <= 0) return;
    receiver_health_.observationCapacity(raw_.obs.n, MAXOBS);

    // obs.data[0].time, not raw_.time (already advanced to the next epoch).
    int week = 0;
    const gtime_t obs_time = raw_.obs.data[0].time;
    const double tow = time2gpst(obs_time, &week);

    msg::GnssObservations msg;
    const auto measurement_stamp = time_base_.stamp(
        config_.use_gps_timestamp, oem4_time_valid_ && week > 0,
        obs_time, now(), get_logger(), *get_clock());
    if (!measurement_stamp) { receiver_health_.timeWithheld(); return; }
    msg.header.stamp = *measurement_stamp;
    msg.header.frame_id = config_.frame_id;
    msg.week = static_cast<uint32_t>(week);
    msg.tow = tow;

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

  // Ephemeris Publishing

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

    changed = eph_store_.ingestIon(raw_.nav) || changed;
    if (changed) publishSnapshot();
  }

  // Broadcast ionosphere parameters (backend status 9): part of the snapshot.
  void publishIonosphere() {
    if (eph_store_.ingestIon(raw_.nav)) publishSnapshot();
  }

  void maybePublishHeartbeat() {
    if (eph_store_.heartbeatDue(now())) publishSnapshot();
  }

  void publishSnapshot() {
    auto m = eph_store_.buildSnapshot(now());
    receiver_health_.snapshotRejected(eph_store_.lastSnapshotRejected());
    eph_pub_->publish(m);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "Published ephemeris snapshot: GNSS=%zu GLO=%zu",
      m.gnss_ephemeris.size(), m.glonass_ephemeris.size());
  }

  // Member Variables

  NovatelConfig config_;

  rclcpp::Publisher<msg::GnssObservations>::SharedPtr obs_pub_;
  rclcpp::Publisher<msg::GnssEphemerides>::SharedPtr  eph_pub_;
  rclcpp::Publisher<msg::GnssSolution>::SharedPtr     sol_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;       // CORRIMUDATA (calibrated)
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_pub_;   // RAWIMUSX (uncalibrated)
  rclcpp::TimerBase::SharedPtr timer_;

  wire::Framer wire_framer_{wire::Protocol::Novatel};
  std::chrono::steady_clock::time_point wire_last_input_{std::chrono::steady_clock::now()};
  uint64_t wire_last_rejected_{0};
  stream_t stream_{};
  raw_t    raw_{};

  // One time base per session, latched at the first published message and
  // never changed. Shared by EVERY topic this node publishes, which is what
  // structurally guarantees the IMU and the GNSS topics cannot end up on
  // different bases. See decoder_common::TimeBaseLatch.
  decoder_common::TimeBaseLatch time_base_;
  wire::NovatelTimeQuality oem4_time_quality_;
  bool oem4_time_valid_{false};
  bool binary_time_valid_{false};

  bool is_serial_connection_{false};

  // RTCM relay (PC -> receiver): STRSVR-style TCP server -> serial
  decoder_common::RtcmRelayServer relay_;
  // Dedicated RTCMV3 correction output port (NovAtel two-port). If unopened,
  // corrections are written to the main stream_ (single-port fallback).
  stream_t relay_out_{};
  bool     relay_out_open_{false};

  // NMEA parsing state
  gnss_utils::NmeaParser nmea_parser_;
  msg::GnssSolution      nmea_solution_;
  msg::GnssSolution      binary_solution_;
  SolutionSource         source_{SolutionSource::NMEA};
  rclcpp::Time           start_time_{0, 0, RCL_ROS_TIME};
  bool                   ever_received_binary_{false};

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
  int      oem4_hdr_pos_{0};
  uint32_t oem4_gps_week_{0};
  uint32_t oem4_gps_ms_{0};
  int      oem4_body_pos_{0};
  std::vector<uint8_t> oem4_body_;
  PendingPvt pending_;

  // CORRIMUDATA dt tracking (SI-increment → rate conversion)
  double corrimu_prev_seconds_{0.0};
  bool   has_corrimu_prev_{false};
  double rawimu_prev_seconds_{0.0};
  bool   has_rawimu_prev_{false};
};

}  // namespace gnss_ros_standardization

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<gnss_ros_standardization::NovatelDriverNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("novatel_driver_node"), "Startup/runtime failure: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
