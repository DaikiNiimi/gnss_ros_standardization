#include <algorithm>
#include "gnss_ros_standardization/receiver_health.hpp"
#include "gnss_ros_standardization/receiver_session.hpp"
#include "gnss_ros_standardization/binary_framer.hpp"
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

#include <sensor_msgs/msg/imu.hpp>

#include "gnss_ros_standardization/decoder_common.hpp"
#include "gnss_ros_standardization/ephemeris_store.hpp"
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
  bool enable_nav_dop{false};
  bool enable_nav_cov{false};
  bool enable_nav_clock{false};
  bool enable_nav_posecef{false};
  bool enable_nav_velecef{false};
  bool enable_nav_hpposllh{false};
  bool enable_nav_hpposecef{false};
  bool enable_nmea_gga{false};
  bool enable_nmea_rmc{false};
  bool enable_nmea_gsa{false};
  bool enable_nmea_gst{false};
  bool nmea_high_precision{false};

  // IMU settings (ZED-F9R / IMU-enabled receivers)
  bool enable_esf_raw{false};    // ESF-RAW: uncalibrated raw IMU       → /gnss/imu/data_raw
  bool enable_esf_ins{false};   // ESF-INS: calibrated angular rate + acceleration → /gnss/imu/data
  std::string imu_topic{"/gnss/imu/data"};
  std::string imu_raw_topic{"/gnss/imu/data_raw"};


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
  std::string solution_topic{"/gnss/nmea_solution"};

  // RTCM relay (PC -> receiver) for on-chip RTK — see openRtcmRelay().
  bool rtcm_relay_enabled{false};
  std::string rtcm_relay_listen{"tcpsvr://:5556"};

  // ENU local origin settings

  // Ephemeris snapshot behavior
  double ephemeris_snapshot_period_s{30.0};
  double ephemeris_max_age_s{0.0};  // 0 = keep all

  bool use_gps_timestamp{false};
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

    openRtcmRelay();
    if (config_.configure_on_startup && config_.enable_rawx) receiver_health_.expectObservations(config_.rate_hz);
    startPolling();
    RCLCPP_INFO(get_logger(), "UBX Driver initialized (rate: %d Hz)", config_.rate_hz);
  }

  ~UbxDriverNode() override {
    try {
      strclose(&stream_);
      relay_.close();
    } catch (...) {
    }
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
    const double tick_s = declare_parameter<double>("imu.sttag_tick_s", ubx::esf_raw::STTAG_TICK_S);
    if (!std::isfinite(tick_s) || tick_s <= 0.0 || tick_s >= 1.0)
      throw std::invalid_argument("imu.sttag_tick_s must be finite and in (0, 1)");
    imu_clock_ = ins::SensorTickClock(ins::SensorTickClock::Config{tick_s, 1ull << 32});
    RCLCPP_INFO(get_logger(), "ESF-RAW uses estimated time: tick=%.9g s, GNSS anchor expiry=2 s", tick_s);

    // Basic settings
    declare_parameter<std::string>("stream_path", config_.stream_path);
    declare_parameter<int>("rate_hz", config_.rate_hz);
    declare_parameter<bool>("configure_on_startup", config_.configure_on_startup);
    declare_parameter<std::string>("dynamic_model", config_.dynamic_model);
    declare_parameter<std::string>("generation", config_.generation);
    // frame_id: ROS TF frame name attached to the GNSS antenna phase center.
    // Default "gnss_link" — override in launch to integrate with vehicle TF tree.
    declare_parameter<std::string>("frame_id", config_.frame_id);
    
    // Message settings
    declare_parameter<bool>("messages.rawx", config_.enable_rawx);
    declare_parameter<bool>("messages.sfrbx", config_.enable_sfrbx);
    declare_parameter<bool>("messages.nav_pvt",     config_.enable_nav_pvt);
    declare_parameter<bool>("messages.nav_dop",     config_.enable_nav_dop);
    declare_parameter<bool>("messages.nav_cov",     config_.enable_nav_cov);
    declare_parameter<bool>("messages.nav_clock",   config_.enable_nav_clock);
    declare_parameter<bool>("messages.nav_posecef", config_.enable_nav_posecef);
    declare_parameter<bool>("messages.nav_velecef", config_.enable_nav_velecef);
    declare_parameter<bool>("messages.nav_hpposllh",  config_.enable_nav_hpposllh);
    declare_parameter<bool>("messages.nav_hpposecef", config_.enable_nav_hpposecef);
    declare_parameter<bool>("messages.nmea_gga", config_.enable_nmea_gga);
    declare_parameter<bool>("messages.nmea_rmc", config_.enable_nmea_rmc);
    declare_parameter<bool>("messages.nmea_gsa", config_.enable_nmea_gsa);
    declare_parameter<bool>("messages.nmea_gst", config_.enable_nmea_gst);
    declare_parameter<bool>("messages.nmea_high_precision", config_.nmea_high_precision);
    declare_parameter<bool>("messages.esf_raw", config_.enable_esf_raw);
    declare_parameter<bool>("messages.esf_ins", config_.enable_esf_ins);
    declare_parameter<std::string>("topics.imu",     config_.imu_topic);
    declare_parameter<std::string>("topics.imu_raw", config_.imu_raw_topic);
    
    // GNSS constellation settings
    declare_parameter<bool>("navsys.gps", config_.enable_gps);
    declare_parameter<bool>("navsys.glo", config_.enable_glonass);
    declare_parameter<bool>("navsys.gal", config_.enable_galileo);
    declare_parameter<bool>("navsys.bds", config_.enable_beidou);
    declare_parameter<bool>("navsys.qzs", config_.enable_qzss);
    declare_parameter<bool>("navsys.irn", config_.enable_navic);
    declare_parameter<bool>("navsys.sbs", config_.enable_sbas);

    declare_parameter<std::string>("topics.observation", config_.observation_topic);
    declare_parameter<std::string>("topics.ephemeris", config_.ephemeris_topic);
    declare_parameter<std::string>("topics.solution", config_.solution_topic);
    declare_parameter<bool>("rtcm_input.enable", config_.rtcm_relay_enabled);
    declare_parameter<std::string>("rtcm_input.listen", config_.rtcm_relay_listen);

    declare_parameter<double>("ephemeris.snapshot_period_s", config_.ephemeris_snapshot_period_s);
    declare_parameter<double>("ephemeris.max_age_s",         config_.ephemeris_max_age_s);
    declare_parameter<bool>("use_gps_timestamp",             config_.use_gps_timestamp);

    // Read parameters
    config_.stream_path = get_parameter("stream_path").as_string();
    config_.rate_hz = get_parameter("rate_hz").as_int();
    config_.configure_on_startup = get_parameter("configure_on_startup").as_bool();
    config_.dynamic_model = get_parameter("dynamic_model").as_string();
    config_.generation = get_parameter("generation").as_string();
    if (config_.generation != "G9" && config_.generation != "G10")
      throw std::invalid_argument("generation must be G9 or G10");
    {
      static const char* const kModels[] = {"portable", "stationary", "pedestrian",
          "automotive", "sea", "airborne_1g", "airborne_2g", "airborne_4g", "watch"};
      if (std::find(std::begin(kModels), std::end(kModels), config_.dynamic_model) ==
          std::end(kModels))
        throw std::invalid_argument("dynamic_model must be one of portable, stationary, "
            "pedestrian, automotive, sea, airborne_1g, airborne_2g, airborne_4g, watch");
    }
    config_.frame_id = get_parameter("frame_id").as_string();
    
    config_.enable_rawx = get_parameter("messages.rawx").as_bool();
    config_.enable_sfrbx = get_parameter("messages.sfrbx").as_bool();
    config_.enable_nav_pvt     = get_parameter("messages.nav_pvt").as_bool();
    config_.enable_nav_dop     = get_parameter("messages.nav_dop").as_bool();
    config_.enable_nav_cov     = get_parameter("messages.nav_cov").as_bool();
    config_.enable_nav_clock   = get_parameter("messages.nav_clock").as_bool();
    config_.enable_nav_posecef = get_parameter("messages.nav_posecef").as_bool();
    config_.enable_nav_velecef = get_parameter("messages.nav_velecef").as_bool();
    config_.enable_nav_hpposllh  = get_parameter("messages.nav_hpposllh").as_bool();
    config_.enable_nav_hpposecef = get_parameter("messages.nav_hpposecef").as_bool();
    config_.enable_nmea_gga = get_parameter("messages.nmea_gga").as_bool();
    config_.enable_nmea_rmc = get_parameter("messages.nmea_rmc").as_bool();
    config_.enable_nmea_gsa = get_parameter("messages.nmea_gsa").as_bool();
    config_.enable_nmea_gst = get_parameter("messages.nmea_gst").as_bool();
    config_.nmea_high_precision = get_parameter("messages.nmea_high_precision").as_bool();
    config_.enable_esf_raw      = get_parameter("messages.esf_raw").as_bool();
    config_.enable_esf_ins      = get_parameter("messages.esf_ins").as_bool();
    config_.imu_topic           = get_parameter("topics.imu").as_string();
    config_.imu_raw_topic       = get_parameter("topics.imu_raw").as_string();
    
    config_.enable_gps = get_parameter("navsys.gps").as_bool();
    config_.enable_glonass = get_parameter("navsys.glo").as_bool();
    config_.enable_galileo = get_parameter("navsys.gal").as_bool();
    config_.enable_beidou = get_parameter("navsys.bds").as_bool();
    config_.enable_qzss = get_parameter("navsys.qzs").as_bool();
    config_.enable_navic = get_parameter("navsys.irn").as_bool();
    config_.enable_sbas = get_parameter("navsys.sbs").as_bool();

    config_.observation_topic = get_parameter("topics.observation").as_string();
    config_.ephemeris_topic = get_parameter("topics.ephemeris").as_string();
    config_.solution_topic = get_parameter("topics.solution").as_string();
    config_.rtcm_relay_enabled = get_parameter("rtcm_input.enable").as_bool();
    config_.rtcm_relay_listen = get_parameter("rtcm_input.listen").as_string();

    config_.ephemeris_snapshot_period_s = get_parameter("ephemeris.snapshot_period_s").as_double();
    config_.ephemeris_max_age_s         = get_parameter("ephemeris.max_age_s").as_double();
    config_.use_gps_timestamp           = get_parameter("use_gps_timestamp").as_bool();

    eph_store_.setSnapshotPeriod(config_.ephemeris_snapshot_period_s);
    eph_store_.setMaxAge(config_.ephemeris_max_age_s);
    constexpr int history_limit = 8;
    eph_store_.setHistoryPerSatellite(static_cast<size_t>(history_limit));

    // Detect USB connection from path
    is_usb_connection_ = (config_.stream_path.find("ttyACM") != std::string::npos);

    // Validate rate
    gnss_utils::validatePublishRate(config_.rate_hz, /*fallback=*/5, /*max_hz=*/20, get_logger());
    
    // Log configuration
    RCLCPP_INFO(get_logger(), "Configuration loaded:");
    RCLCPP_INFO(get_logger(), "  Stream: %s", decoder_common::redactStreamUri(config_.stream_path).c_str());
    RCLCPP_INFO(get_logger(), "  Rate: %d Hz", config_.rate_hz);
    RCLCPP_INFO(get_logger(), "  Generation: %s", config_.generation.c_str());
    RCLCPP_INFO(get_logger(), "  Dynamic model: %s", config_.dynamic_model.c_str());
    RCLCPP_INFO(get_logger(), "  GNSS: GPS=%d GLO=%d GAL=%d BDS=%d QZS=%d IRN=%d SBAS=%d",
                config_.enable_gps, config_.enable_glonass, config_.enable_galileo,
                config_.enable_beidou, config_.enable_qzss, config_.enable_navic, config_.enable_sbas);

    // Lock solution source at startup. BINARY if NAV-PVT is enabled, else NMEA.
    // No mid-session switching: if binary stops, output pauses rather than falling back.
    // HPPOS messages refine the BINARY solution but cannot anchor an epoch alone.
    if ((config_.enable_nav_hpposllh || config_.enable_nav_hpposecef) &&
        !config_.enable_nav_pvt) {
      RCLCPP_WARN(get_logger(),
        "messages.nav_hpposllh/nav_hpposecef require messages.nav_pvt: true — "
        "HPPOS carries position only; no binary solution will be published without NAV-PVT.");
    }
    source_ = config_.enable_nav_pvt ? SolutionSource::BINARY : SolutionSource::NMEA;
    initPendingExpectedMask();
    start_time_ = now();
    RCLCPP_INFO(get_logger(), "Solution source locked: %s",
                source_ == SolutionSource::BINARY ? "BINARY (UBX-NAV-PVT)" : "NMEA");
  }

  void initializePublishers() {
    decoder_common::configureNmeaDate(*this, nmea_parser_);
    obs_pub_     = create_publisher<msg::GnssObservations>(config_.observation_topic, 10);
    eph_pub_     = create_publisher<msg::GnssEphemerides>(config_.ephemeris_topic, rclcpp::QoS(256).transient_local()  /* depth 256: latch full ephemeris set, not just the last satellite (per-sat messages) */);
    sol_pub_     = create_publisher<msg::GnssSolution>(config_.solution_topic, 10);
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
    RCLCPP_INFO(get_logger(), "  Observation  : RAWX=%s SFRBX=%s",
      on(config_.enable_rawx), on(config_.enable_sfrbx));
    RCLCPP_INFO(get_logger(), "  PVT (binary) : NAV-PVT=%s NAV-DOP=%s NAV-COV=%s NAV-POSECEF=%s NAV-VELECEF=%s NAV-HPPOSLLH=%s NAV-HPPOSECEF=%s",
      on(config_.enable_nav_pvt), on(config_.enable_nav_dop), on(config_.enable_nav_cov),
      on(config_.enable_nav_posecef), on(config_.enable_nav_velecef),
      on(config_.enable_nav_hpposllh), on(config_.enable_nav_hpposecef));
    RCLCPP_INFO(get_logger(), "  NMEA         : GGA=%s RMC=%s GSA=%s GST=%s HiPrec=%s",
      on(config_.enable_nmea_gga), on(config_.enable_nmea_rmc),
      on(config_.enable_nmea_gsa), on(config_.enable_nmea_gst),
      on(config_.nmea_high_precision));
    RCLCPP_INFO(get_logger(), "  IMU          : ESF-RAW=%s ESF-INS=%s",
      on(config_.enable_esf_raw), on(config_.enable_esf_ins));
    RCLCPP_INFO(get_logger(), "  Constellation: GPS=%s GLO=%s GAL=%s BDS=%s QZS=%s NavIC=%s SBAS=%s",
      on(config_.enable_gps), on(config_.enable_glonass), on(config_.enable_galileo),
      on(config_.enable_beidou), on(config_.enable_qzss), on(config_.enable_navic),
      on(config_.enable_sbas));
    RCLCPP_INFO(get_logger(), "  GPS timestamp : %s", on(config_.use_gps_timestamp));
    if (has_local_origin_) {
      RCLCPP_INFO(get_logger(), "  ENU origin    : fixed (lat=%.6f lon=%.6f alt=%.2f)",
        local_origin_pos_[0] * R2D, local_origin_pos_[1] * R2D, local_origin_pos_[2]);
    } else {
      RCLCPP_INFO(get_logger(), "  ENU origin    : auto (first valid solution)");
    }
  }

  void initializeDecoder() {
    const auto rawx_layout = declare_parameter<std::string>("rawx_layout", "auto");
    const char* layout_option = ubx::rawxLayoutOption(rawx_layout);
    if (init_raw(&raw_, STRFMT_UBX) != 1) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize raw decoder");
      throw std::runtime_error("init_raw failed");
    }
    // Preserve RAWX sigId (e.g. L2 CL=2L, L2 CM=2S), rather than labelling
    // individual signals as combined 2X/5X measurements. resetRaw keeps options.
    std::strcpy(raw_.opt, "-EPHALL -MULTICODE");
    std::strcat(raw_.opt, layout_option);
  }

  void startPolling() {
    timer_ = create_wall_timer(kTimerInterval, std::bind(&UbxDriverNode::pollStream, this));
  }

  // RTCM relay (RTKLIB STRSVR-style): host a TCP server for incoming
  // base-station corrections; pollStream() writes them to the receiver serial.
  // The driver stays the sole owner of the serial port — corrections arrive
  // over TCP, not via a second serial opener.
  void openRtcmRelay() {
    if (!config_.rtcm_relay_enabled) return;
    if (!relay_.open(get_logger(), config_.rtcm_relay_listen)) {
      throw std::runtime_error("RTCM relay stropen failed");
    }
  }

  // Stream Management

  // Open `original` (scheme-prefixed, e.g. serial:// / tcpsvr:// / tcpcli://)
  // into `s` as a read-write RTKLIB stream. Returns false on failure.
  bool openStreamPath(stream_t& s, const std::string& original, const char* what) {
    const auto path = original.find("://") == std::string::npos ? "serial://" + original : original;
    return decoder_common::openStreamPath(get_logger(), s, path, STR_MODE_RW, what);
  }

  void openStream() {
    if (!openStreamPath(stream_, config_.stream_path, "Receiver")) {
      throw std::runtime_error("stropen failed");
    }
  }

  // Receiver Configuration (Unified Entry Point)

  bool isGen10() const { return config_.generation == "G10"; }

  void configureReceiver() {
    RCLCPP_INFO(get_logger(), "Configuring receiver (Gen: %s)...", config_.generation.c_str());
    
    // Step 0: Ping receiver to verify connection and baud rate
    if (!pingReceiver()) {
      RCLCPP_ERROR(get_logger(), "Failed to communicate with receiver. Check connection and baud rate.");
      throw std::runtime_error("Receiver MON-VER poll failed");
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
    if (rejected_config_transactions_ == 0)
      RCLCPP_INFO(get_logger(), "Receiver acknowledged configuration");
    else
      RCLCPP_WARN(get_logger(), "Receiver configuration is incomplete: %zu rejected transactions; verify requested signals and outputs", rejected_config_transactions_);
  }

  bool pingReceiver() {
    // Send MON-VER (0x0A 0x04) with empty payload (Poll request)
    std::vector<uint8_t> payload = {};
    sendUbxRaw(ubx::CLASS_MON, ubx::ID_MON_VER, payload);
    
    return waitForResponse(ubx::CLASS_MON, ubx::ID_MON_VER);
  }

  std::optional<wire::Frame> readResponse(uint8_t cls, uint8_t id) {
    wire::Framer framer(wire::Protocol::Ubx);
    std::optional<wire::Frame> response;
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    uint8_t buffer[4096];
    while (std::chrono::steady_clock::now() < deadline) {
      const int n = strread(&stream_, buffer, sizeof(buffer));
      for (int i = 0; i < n; ++i) {
        parseUbxByte(buffer[i]);
        framer.feed(buffer[i], [&](const wire::Frame& f) {
          if (f.cls == cls && f.id == id) response = f;
        });
      }
      if (response) return response;
      std::this_thread::sleep_for(2ms);
    }
    return std::nullopt;
  }
  bool waitForResponse(uint8_t cls, uint8_t id) {
    const auto response = readResponse(cls, id);
    return response && response->payload.size() >= 40;
  }

  // Step 1: GNSS Signal Configuration

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
    sendUbxRaw(ubx::CLASS_CFG, ubx::ID_CFG_GNSS, {});
    auto response = readResponse(ubx::CLASS_CFG, ubx::ID_CFG_GNSS);
    if (!response || response->payload.empty() || response->payload[0] != 0)
      throw std::runtime_error("CFG-GNSS poll failed or returned unsupported version");
    if (!applyAndSendCfgGnss(response->payload))
      throw std::runtime_error("Receiver rejected CFG-GNSS settings");
    performGnssReset();
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

    auto sendSignal = [this](uint32_t key, bool enable, const char* name, bool required = false) {
      uint32_t val = enable ? 1 : 0;
      if (!sendCfgValset({{key, val, 1}}, required && enable)) {
        // NAK is expected for unsupported signals (e.g., GLONASS on X20P)
        RCLCPP_DEBUG(get_logger(), "Signal %s not accepted by receiver (NAK)", name);
      }
    };

    // GPS: L1C/A, L2C, L5
    sendSignal(ubx::CFG_SIGNAL_GPS_ENA, config_.enable_gps, "GPS ENA", true);
    sendSignal(ubx::CFG_SIGNAL_GPS_L1CA_ENA, config_.enable_gps, "GPS L1C/A");
    sendSignal(ubx::CFG_SIGNAL_GPS_L2C_ENA, config_.enable_gps, "GPS L2C");
    sendSignal(ubx::CFG_SIGNAL_GPS_L5_ENA, config_.enable_gps, "GPS L5");

    // SBAS: L1C/A
    sendSignal(ubx::CFG_SIGNAL_SBAS_ENA, config_.enable_sbas, "SBAS ENA", true);
    sendSignal(ubx::CFG_SIGNAL_SBAS_L1CA_ENA, config_.enable_sbas, "SBAS L1C/A");

    // Galileo: E1, E5a, E5b
    sendSignal(ubx::CFG_SIGNAL_GAL_ENA, config_.enable_galileo, "Galileo ENA", true);
    sendSignal(ubx::CFG_SIGNAL_GAL_E1_ENA, config_.enable_galileo, "Galileo E1");
    sendSignal(ubx::CFG_SIGNAL_GAL_E5A_ENA, config_.enable_galileo, "Galileo E5a");
    sendSignal(ubx::CFG_SIGNAL_GAL_E5B_ENA, config_.enable_galileo, "Galileo E5b");

    // BeiDou: B1C, B2a (X20P); B1I, B2I (F9P/M10)
    sendSignal(ubx::CFG_SIGNAL_BDS_ENA, config_.enable_beidou, "BeiDou ENA", true);
    sendSignal(ubx::CFG_SIGNAL_BDS_B1C_ENA, config_.enable_beidou, "BeiDou B1C");
    sendSignal(ubx::CFG_SIGNAL_BDS_B2A_ENA, config_.enable_beidou, "BeiDou B2a");
    sendSignal(ubx::CFG_SIGNAL_BDS_B1I_ENA, config_.enable_beidou, "BeiDou B1I");
    sendSignal(ubx::CFG_SIGNAL_BDS_B2I_ENA, config_.enable_beidou, "BeiDou B2I");

    // QZSS: L1C/A, L1S, L2C, L5
    sendSignal(ubx::CFG_SIGNAL_QZSS_ENA, config_.enable_qzss, "QZSS ENA", true);
    sendSignal(ubx::CFG_SIGNAL_QZSS_L1CA_ENA, config_.enable_qzss, "QZSS L1C/A");
    sendSignal(ubx::CFG_SIGNAL_QZSS_L1S_ENA, config_.enable_qzss, "QZSS L1S");
    sendSignal(ubx::CFG_SIGNAL_QZSS_L2C_ENA, config_.enable_qzss, "QZSS L2C");
    sendSignal(ubx::CFG_SIGNAL_QZSS_L5_ENA, config_.enable_qzss, "QZSS L5");

    // GLONASS: L1, L2
    sendSignal(ubx::CFG_SIGNAL_GLO_ENA, config_.enable_glonass, "GLO ENA", true);
    sendSignal(ubx::CFG_SIGNAL_GLO_L1_ENA, config_.enable_glonass, "GLO L1");
    sendSignal(ubx::CFG_SIGNAL_GLO_L2_ENA, config_.enable_glonass, "GLO L2");

    sendSignal(ubx::CFG_SIGNAL_NAVIC_ENA, config_.enable_navic, "NavIC ENA", true);
    sendSignal(ubx::CFG_SIGNAL_NAVIC_L5_ENA, config_.enable_navic, "NavIC L5");
    
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
    for (int read = 0; read < 32 && (n = strread(&stream_, buf, sizeof(buf))) > 0; ++read) {
      drained += n;
    }
    if (drained > 0) {
      RCLCPP_DEBUG(get_logger(), "Drained %d stale bytes from stream buffer", drained);
    }
  }

  // Step 2: Measurement Rate

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

  // Step 3: Dynamic Model

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

  // Step 4: Output Messages

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
          {ubx::CFG_MSGOUT_UBX_NAV_PVT_I2C     + port, config_.enable_nav_pvt     ? 1u : 0u, 1},
          {ubx::CFG_MSGOUT_UBX_NAV_DOP_I2C     + port, config_.enable_nav_dop     ? 1u : 0u, 1},
          {ubx::CFG_MSGOUT_UBX_NAV_COV_I2C     + port, config_.enable_nav_cov     ? 1u : 0u, 1},
          {ubx::CFG_MSGOUT_UBX_NAV_CLOCK_I2C   + port, config_.enable_nav_clock   ? 1u : 0u, 1},
          {ubx::CFG_MSGOUT_UBX_NAV_POSECEF_I2C + port, config_.enable_nav_posecef ? 1u : 0u, 1},
          {ubx::CFG_MSGOUT_UBX_NAV_VELECEF_I2C + port, config_.enable_nav_velecef ? 1u : 0u, 1},
          {ubx::CFG_MSGOUT_UBX_NAV_HPPOSLLH_I2C  + port, config_.enable_nav_hpposllh  ? 1u : 0u, 1},
          {ubx::CFG_MSGOUT_UBX_NAV_HPPOSECEF_I2C + port, config_.enable_nav_hpposecef ? 1u : 0u, 1},
          {ubx::CFG_MSGOUT_UBX_ESF_RAW_I2C   + port, config_.enable_esf_raw ? 1u : 0u, 1},
          {ubx::CFG_MSGOUT_UBX_ESF_INS_I2C   + port, config_.enable_esf_ins ? 1u : 0u, 1},
      };

      if (!sendOutputCfgValset(items_ubx)) {
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

      if (!sendOutputCfgValset(items_nmea)) {
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
    sendMsg(ubx::CLASS_NAV, ubx::ID_NAV_PVT,     config_.enable_nav_pvt,     "NAV-PVT");
    sendMsg(ubx::CLASS_NAV, ubx::ID_NAV_DOP,     config_.enable_nav_dop,     "NAV-DOP");
    sendMsg(ubx::CLASS_NAV, ubx::ID_NAV_COV,     config_.enable_nav_cov,     "NAV-COV");
    sendMsg(ubx::CLASS_NAV, ubx::ID_NAV_CLOCK,   config_.enable_nav_clock,   "NAV-CLOCK");
    sendMsg(ubx::CLASS_NAV, ubx::ID_NAV_POSECEF, config_.enable_nav_posecef, "NAV-POSECEF");
    sendMsg(ubx::CLASS_NAV, ubx::ID_NAV_VELECEF, config_.enable_nav_velecef, "NAV-VELECEF");
    sendMsg(ubx::CLASS_NAV, ubx::ID_NAV_HPPOSLLH,  config_.enable_nav_hpposllh,  "NAV-HPPOSLLH");
    sendMsg(ubx::CLASS_NAV, ubx::ID_NAV_HPPOSECEF, config_.enable_nav_hpposecef, "NAV-HPPOSECEF");
    sendMsg(ubx::CLASS_ESF, ubx::ID_ESF_RAW,   config_.enable_esf_raw, "ESF-RAW");
    sendMsg(ubx::CLASS_ESF, ubx::ID_ESF_INS,   config_.enable_esf_ins, "ESF-INS");

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
      sendNmea(ubx::NMEA_GSV, false, "GSV");
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

  // UBX Communication (Gen 10: CFG-VALSET)

  bool sendOutputCfgValset(const std::vector<ubx::ValsetItem>& items) {
    if (sendCfgValset(items, false)) return true;
    // An unsupported optional key rejects a whole VALSET. Retry output-rate keys
    // separately so a missing ESF message/port cannot disable RAWX/SFRBX setup.
    bool complete = true;
    for (const auto& item : items) {
      if (!sendCfgValset({item}, item.value != 0) && item.value != 0) complete = false;
    }
    return complete;
  }

  bool sendCfgValset(const std::vector<ubx::ValsetItem>& items, bool required = true) {
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
    
    return sendUbxPacket(ubx::CLASS_CFG, ubx::ID_CFG_VALSET, payload, required);
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

  // UBX Communication (Low-level)

  /// Send a UBX frame and wait for ACK.
  bool sendUbxPacket(uint8_t msg_class, uint8_t msg_id, const std::vector<uint8_t>& payload, bool required = true) {
    auto frame = ubx::buildUbxFrame(msg_class, msg_id, payload);
    RCLCPP_DEBUG(get_logger(), "Sending UBX 0x%02X 0x%02X (len=%zu)", 
                msg_class, msg_id, payload.size());
    
    if (!decoder_common::writeAll(stream_, frame.data(), frame.size())) throw std::runtime_error("UBX command write failed");
    
    return waitForAck(msg_class, msg_id, required);
  }

  /// Send a UBX frame without waiting for ACK (for commands like CFG-RST).
  void sendUbxRaw(uint8_t msg_class, uint8_t msg_id, const std::vector<uint8_t>& payload) {
    auto frame = ubx::buildUbxFrame(msg_class, msg_id, payload);
    RCLCPP_DEBUG(get_logger(), "Sending UBX 0x%02X 0x%02X (len=%zu, no ACK)", 
                msg_class, msg_id, payload.size());
    if (!decoder_common::writeAll(stream_, frame.data(), frame.size()))
      throw std::runtime_error("UBX command write timed out");
  }

  /// Send a legacy UBX command string (Gen 9). Uses RTKLIB's gen_ubx().
  bool sendLegacyCommand(const char* cmd, uint8_t expected_class, uint8_t expected_id) {
    uint8_t buff[256];
    int len = gen_ubx(cmd, buff);
    if (len <= 0) {
      RCLCPP_ERROR(get_logger(), "Failed to generate UBX command: %s", cmd);
      return false;
    }

    RCLCPP_DEBUG(get_logger(), "Sending legacy: %s", cmd);

    if (!decoder_common::writeAll(stream_, buff, len)) throw std::runtime_error("UBX command write failed");

    return waitForAck(expected_class, expected_id);
  }

  // ACK/NAK Detection

  size_t rejected_config_transactions_{0};
  bool waitForAck(uint8_t msg_class, uint8_t msg_id, bool required = true) {
    auto start = std::chrono::steady_clock::now();
    uint8_t buffer[256];
    const auto timeout = std::chrono::milliseconds(ubx::ACK_TIMEOUT_MS);

    // Reset ACK state
    pending_ack_class_ = msg_class;
    pending_ack_id_ = msg_id;
    ack_framer_.reset();
    ack_received_ = false;
    nak_received_ = false;

    while (std::chrono::steady_clock::now() - start < timeout) {
      int n = strread(&stream_, buffer, sizeof(buffer));
      for (int i = 0; i < n; ++i) {
        parseUbxByte(buffer[i]);
        checkAckNak(buffer[i]);
      }
      if (ack_received_) return true;
      if (nak_received_) { if (required) ++rejected_config_transactions_; return false; }
      std::this_thread::sleep_for(10ms);
    }

    throw std::runtime_error("UBX ACK timed out; configuration stopped to prevent a delayed ACK matching a later command");
  }

  void checkAckNak(uint8_t byte) {
    ack_framer_.feed(byte, [this](const wire::Frame& f) {
      if (!wire::matchesAck(f, pending_ack_class_, pending_ack_id_)) return;
      ack_received_ = f.id == ubx::ID_ACK_ACK;
      nak_received_ = f.id == ubx::ID_ACK_NAK;
    });
  }

  // UBX mixed stream decoder (shared boundary with RTKLIB, for ESF and NAV blocks)

  void dispatchFrame(const wire::Frame& f) {
      auto diagnostic_scope = receiver_health_.backendScope();
      receiver_health_.frame((unsigned(f.cls)<<16)|f.id, f.revision);
      receiver_health_.inspect(wire::Protocol::Ubx,f,std::strstr(raw_.opt,"-RAWX0")!=nullptr);
      if (receiver_timeline_.frame(wire::Protocol::Ubx,f)) {
        resetReceiverSession(false);
        receiver_timeline_.frame(wire::Protocol::Ubx,f);
      }
      for (uint8_t byte : f.bytes) {
        const int status = input_ubx(&raw_, byte);
        // A successfully decoded RAW/RAWX dates the receiver even with zero
        // observations. Do not refresh from unrelated, weekless messages.
        if (status==1 && f.cls==2 && (f.id==0x15 || f.id==0x10))
          observation_time_ref_.update(raw_.time);
        handleDecodeResult(status);
      }
      ubx_frm_cls_ = f.cls;
      ubx_frm_id_ = static_cast<uint8_t>(f.id);
      ubx_frm_payload_ = f.payload;
      handleUbxFrame();
  }

  void parseUbxByte(uint8_t byte) {
    wire_framer_.feed(byte, [this](const wire::Frame& f) { dispatchFrame(f); },
        [this](const std::string& sentence) { handleNmeaSentence(sentence); });
  }

  void handleUbxFrame() {
    if (ubx_frm_cls_ == ubx::CLASS_ESF && ubx_frm_id_ == ubx::ID_ESF_RAW) handleEsfRaw();
    if (ubx_frm_cls_ == ubx::CLASS_ESF && ubx_frm_id_ == ubx::ID_ESF_INS) handleEsfIns();
    if (ubx_frm_cls_ == ubx::CLASS_NAV && ubx_frm_id_ == ubx::ID_NAV_PVT)     handleNavPvt();
    if (ubx_frm_cls_ == ubx::CLASS_NAV && ubx_frm_id_ == ubx::ID_NAV_DOP)     handleNavDop();
    if (ubx_frm_cls_ == ubx::CLASS_NAV && ubx_frm_id_ == ubx::ID_NAV_CLOCK)   handleNavClock();
    if (ubx_frm_cls_ == ubx::CLASS_NAV && ubx_frm_id_ == ubx::ID_NAV_COV)     handleNavCov();
    if (ubx_frm_cls_ == ubx::CLASS_NAV && ubx_frm_id_ == ubx::ID_NAV_POSECEF) handleNavPosEcef();
    if (ubx_frm_cls_ == ubx::CLASS_NAV && ubx_frm_id_ == ubx::ID_NAV_VELECEF) handleNavVelEcef();
    if (ubx_frm_cls_ == ubx::CLASS_NAV && ubx_frm_id_ == ubx::ID_NAV_HPPOSECEF) handleNavHpPosEcef();
    if (ubx_frm_cls_ == ubx::CLASS_NAV && ubx_frm_id_ == ubx::ID_NAV_HPPOSLLH)  handleNavHpPosLlh();
  }

  // ---- GNSS time anchor for the ESF-RAW timeline -------------------------
  //
  // ESF-RAW carries no GNSS time, and u-blox publishes no field tying sTtag to
  // it: the Interface Description leaves sTtag's Scale and Unit blank, and
  // ESF-MEAS's calibTtag - the one field that would give receiver local time -
  // is specified as "Always set to zero" on this hardware. So the only way to
  // put the IMU on GPS time is the documented practice of enabling ESF-RAW
  // alongside NAV-PVT / NAV-CLOCK and ESTIMATING the offset from them.
  //
  // The estimate is the same one GnssEpochAligner already makes and is tested
  // on: the sliding-window minimum of (host arrival - GPS time of the epoch),
  // which is the host/GPS clock difference plus the receiver's minimum output
  // latency. Subtracting it from the sTtag timeline lands the IMU on UTC(GPST),
  // the same base the GNSS topics use when use_gps_timestamp is set.
  //
  // What this does NOT remove is that minimum latency difference: GNSS epochs
  // and IMU samples do not leave the receiver with the same delay, and u-blox
  // documents neither. NAV-CLOCK's clkB does not help here - it is the
  // receiver clock's bias against GPS time, not its output latency, and sTtag
  // has no documented relation to that clock either. What NAV-CLOCK does add
  // is tAcc, a quality gate on the epoch's time. Expect an O(10-100 ms)
  // systematic offset in ABSOLUTE terms; what a fusion node consumes - the
  // IMU/GNSS relative alignment - is what this fixes.
  void feedGnssAnchor(int week, double tow, const rclcpp::Time& arrival) {
    if (week <= 0 || !std::isfinite(tow) || tow < 0.0 || tow >= 604800.0) { GNSS_RX_EVENT(GNSS_RX_TIME,1); return; }
    if (!gnss_utils::isRosTimeRepresentable(gpst2time(week,tow))) { GNSS_RX_EVENT(GNSS_RX_TIME,1); return; }
    // NAV-CLOCK tAcc gate: the Integration manual warns that iTOW values may
    // not be valid, so do not anchor on an epoch the receiver itself calls
    // uncertain. 1 ms is generous - a receiver with time has tAcc in the tens
    // of nanoseconds - so this only excludes genuinely bad time.
    if (has_nav_clock_ && last_nav_clock_.time_acc_s > 1e-3) { GNSS_RX_EVENT(GNSS_RX_TIME,1); return; }
    const rclcpp::Time aligned =
        gnss_anchor_.align(static_cast<uint32_t>(week), tow, arrival);
    const int64_t meas_ns =
        gnss_utils::gpstToUtcRosTime(gpst2time(week, tow)).nanoseconds();
    const double offset_s = (aligned.nanoseconds() - meas_ns) / 1e9;
    if (has_gnss_anchor_ && std::fabs(offset_s - gnss_anchor_offset_s_) > 1.0) {
      imu_offset_clock_.reset(); last_raw_imu_ns_ = -1;
      RCLCPP_WARN(get_logger(), "GNSS anchor discontinuity: resetting estimated IMU time");
    }
    gnss_anchor_offset_s_ = offset_s;
    has_gnss_anchor_ = true;
    gnss_anchor_arrival_ns_ = arrival.nanoseconds();
    gnss_anchor_steady_ = std::chrono::steady_clock::now();
  }

  /// Map a host-clock ESF-RAW stamp onto UTC(GPST) using that offset.
  ins::SlewedTimeOffset imu_offset_clock_;
  int64_t last_raw_imu_ns_{-1};
  rclcpp::Time imuHostToGps(const rclcpp::Time& host) {
    return imu_offset_clock_.map(host, gnss_anchor_offset_s_);
  }

  // STTAG_TICK_S is not a documented constant: the ZED-F9R Interface
  // Description leaves both Scale and Unit blank for sTtag, so the value in
  // ubx_protocol.hpp is an assumption that a firmware or product change could
  // invalidate silently. The offset estimate drifting against the host clock
  // measures (nominal_tick / true_tick - 1) directly, which turns that
  // invisible failure into a log line. Report once when the window first
  // fills, then only while it stays implausible.
  void reportImuClockDrift() {
    if (!imu_clock_.windowFilled()) return;
    const double ppm = imu_clock_.driftRate() * 1e6;

    // A "drift" of whole percent is not a clock error at all: it means the
    // stream is not real-time paced, which is normal when replaying a
    // recorded log through file:// (minutes of receiver time handed over in
    // seconds). The tick constant cannot be judged against a clock that is
    // not running at wall speed, so say so rather than cry wolf.
    constexpr double kNotRealTimePpm = 50000.0;  // 5 %
    if (std::fabs(ppm) > kNotRealTimePpm) {
      if (!imu_clock_drift_reported_) {
        imu_clock_drift_reported_ = true;
        RCLCPP_INFO(get_logger(),
          "ESF-RAW stream is not real-time paced (%.0f %% off wall speed) — "
          "offline replay assumed; the sTtag tick diagnostic is skipped.",
          ppm / 1e4);
      }
      return;
    }

    if (!imu_clock_drift_reported_) {
      imu_clock_drift_reported_ = true;
      RCLCPP_INFO(get_logger(),
        "ESF-RAW sTtag clock: %.1f ppm vs the host clock", ppm);
    }
    if (std::fabs(ppm) > 100.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
        "ESF-RAW sTtag clock drifting at %.0f ppm vs the host — "
        "imu.sttag_tick_s may be wrong for this receiver/firmware; measured tick %.12g s.",
        ppm, get_parameter("imu.sttag_tick_s").as_double() * (1.0 + imu_clock_.driftRate()));
    }
  }

  // UBX-ESF-RAW: batched IMU samples. No GNSS time (sTtag only) — the topic is
  // host-clock based, mapped through ins::SensorTickClock so the stream lands
  // on ONE monotone timeline (see ubx_decoder_node.cpp for the full rationale
  // and the measured before/after).
  ubx::EsfRawAssembler esf_assembler_;
  void handleEsfRaw() {
    const auto samples = esf_assembler_.push(ubx_frm_payload_);
    receiver_health_.imuPending(esf_assembler_.pending());
    if (esf_assembler_.pending()) GNSS_RX_EVENT(GNSS_RX_IMU_WAIT,1);
    if (ubx_frm_payload_.size()<4 || (ubx_frm_payload_.size()-4)%8) GNSS_RX_EVENT(GNSS_RX_IMU_REJECT,1);
    if (samples.empty()) return;

    const rclcpp::Time arrival = now();
    std::string clock_warn;
    const auto stamps = ins::stampBatch(
        imu_clock_, samples, [](const ubx::esf_raw::Sample& s) {
          return static_cast<uint64_t>(s.sttag);
        }, arrival, &clock_warn);
    if (!clock_warn.empty() && clock_warn.find("clamped") == std::string::npos) {
      imu_offset_clock_.reset(); last_raw_imu_ns_ = -1;
    }
    if (!clock_warn.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "ESF-RAW sTtag clock re-seeded: %s", clock_warn.c_str());
    }
    reportImuClockDrift();

    // use_gps_timestamp applies to this topic too, so the IMU can never sit
    // on a different base from the GNSS topics of the same node. Without an
    // anchor yet, GPS mode withholds IMU samples until a fresh anchor exists.
    if (config_.use_gps_timestamp && !has_gnss_anchor_ && !imu_gps_warned_) {
      imu_gps_warned_ = true;
      RCLCPP_WARN(get_logger(),
        "use_gps_timestamp is set but no GNSS epoch has anchored the ESF-RAW "
        "timeline yet — enable UBX-NAV-PVT, UBX-NAV-CLOCK or RXM-RAWX so the "
        "IMU can be put on GPS time.");
    }

    for (size_t i = 0; i < samples.size(); ++i) {
      const auto& s = samples[i];
      sensor_msgs::msg::Imu imu;
      const auto measurement_stamp = time_base_.stampPrecomputed(
          config_.use_gps_timestamp, has_gnss_anchor_ &&
          arrival.nanoseconds() >= gnss_anchor_arrival_ns_ &&
          arrival.nanoseconds() - gnss_anchor_arrival_ns_ <= 2000000000LL &&
          std::chrono::steady_clock::now()-gnss_anchor_steady_ <= std::chrono::seconds(2), has_gnss_anchor_ ? imuHostToGps(stamps[i]) : stamps[i], stamps[i],
          get_logger(), *get_clock());
    if (!measurement_stamp) { receiver_health_.timeWithheld(); return; }
      if (measurement_stamp->nanoseconds() <= last_raw_imu_ns_) { GNSS_RX_EVENT(GNSS_RX_TIME,1); continue; }
      last_raw_imu_ns_ = measurement_stamp->nanoseconds();
      imu.header.stamp = *measurement_stamp;
      imu.header.frame_id = config_.frame_id;

      // orientation: not provided by ESF-RAW — leave identity, mark unknown
      imu.orientation_covariance[0] = -1.0;

      imu.angular_velocity.x = s.gyro[0];
      imu.angular_velocity.y = s.gyro[1];
      imu.angular_velocity.z = s.gyro[2];
      imu.linear_acceleration.x = s.accel[0];
      imu.linear_acceleration.y = s.accel[1];
      imu.linear_acceleration.z = s.accel[2];

      // Values above are always provided by ESF-RAW; only their covariance is
      // unknown -> all-zero, not "-1" (which claims the measurement is absent).
      auto unk = ins::makeZeroCovariance();
      std::copy(unk.begin(), unk.end(), imu.angular_velocity_covariance.begin());
      std::copy(unk.begin(), unk.end(), imu.linear_acceleration_covariance.begin());

      imu_raw_pub_->publish(imu);
    }
  }

  void handleEsfIns() {
    if (static_cast<int>(ubx_frm_payload_.size()) < ubx::esf_ins::MIN_LEN) { GNSS_RX_EVENT(GNSS_RX_LENGTH,1); return; }

    uint32_t bitfield0 = 0;
    std::memcpy(&bitfield0, ubx_frm_payload_.data() + ubx::esf_ins::OFFSET_BITFIELD, 4);
    const bool ang_valid = (bitfield0 & (0x7u << 8))  == (0x7u << 8);
    const bool acc_valid = (bitfield0 & (0x7u << 11)) == (0x7u << 11);

    int32_t xAng = 0, yAng = 0, zAng = 0, xAcc = 0, yAcc = 0, zAcc = 0;
    std::memcpy(&xAng, ubx_frm_payload_.data() + ubx::esf_ins::OFFSET_XANGRATE, 4);
    std::memcpy(&yAng, ubx_frm_payload_.data() + ubx::esf_ins::OFFSET_YANGRATE, 4);
    std::memcpy(&zAng, ubx_frm_payload_.data() + ubx::esf_ins::OFFSET_ZANGRATE, 4);
    std::memcpy(&xAcc, ubx_frm_payload_.data() + ubx::esf_ins::OFFSET_XACCEL,   4);
    std::memcpy(&yAcc, ubx_frm_payload_.data() + ubx::esf_ins::OFFSET_YACCEL,   4);
    std::memcpy(&zAcc, ubx_frm_payload_.data() + ubx::esf_ins::OFFSET_ZACCEL,   4);

    const double deg2rad = M_PI / 180.0;

    // U2 pp278-279 / U3 pp71-72 define version 1 with this layout.
    // Keep legacy version 0 compatibility; do not accept unknown versions.
    sensor_msgs::msg::Imu imu;
    if (!ang_valid || !acc_valid || (ubx_frm_payload_[0] > 1)) {
      GNSS_RX_EVENT(ubx_frm_payload_[0]>1 ? GNSS_RX_VERSION : GNSS_RX_QUALITY,1); return;
    }
    const double tow = wire::u32(ubx_frm_payload_.data() + ubx::esf_ins::OFFSET_ITOW) * 1e-3;
    int week = 0;
    const gtime_t ref = nav_time_ref_.get().time != 0 ? nav_time_ref_.get() : observation_time_ref_.get();
    const bool valid_time = decoder_common::resolveGpsWeekForTow(ref, tow, week);
    const auto measurement_stamp = time_base_.stamp(
        config_.use_gps_timestamp, valid_time, valid_time ? gpst2time(week, tow) : gtime_t{},
        now(), get_logger(), *get_clock());
    if (!measurement_stamp) { receiver_health_.timeWithheld(); return; }
    imu.header.stamp = *measurement_stamp;
    imu.header.frame_id = config_.frame_id;

    // orientation: not provided by ESF-INS — leave identity, mark unknown
    imu.orientation_covariance[0] = -1.0;

    imu.angular_velocity.x = xAng * 1e-3 * deg2rad;
    imu.angular_velocity.y = yAng * 1e-3 * deg2rad;
    imu.angular_velocity.z = zAng * 1e-3 * deg2rad;
    auto ang_cov = ang_valid ? ins::makeZeroCovariance() : ins::makeUnknownCovariance();
    std::copy(ang_cov.begin(), ang_cov.end(), imu.angular_velocity_covariance.begin());

    imu.linear_acceleration.x = xAcc * 1e-2;
    imu.linear_acceleration.y = yAcc * 1e-2;
    imu.linear_acceleration.z = zAcc * 1e-2;
    auto acc_cov = acc_valid ? ins::makeZeroCovariance() : ins::makeUnknownCovariance();
    std::copy(acc_cov.begin(), acc_cov.end(), imu.linear_acceleration_covariance.begin());

    imu_pub_->publish(imu);
  }

  // Data Processing

  wire::StreamSession stream_session_;
  wire::ReceiverTimeline receiver_timeline_;
  void resetReceiverSession(bool disconnected) {
    // Release validated pending data before retiring the old session. Partial
    // wire frames and compressed references are never carried across sessions.
    flushPending();
    wire::resetRaw(raw_,STRFMT_UBX);
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
    imu_clock_.reset(); gnss_anchor_.reset(); imu_offset_clock_.reset();
    last_raw_imu_ns_=-1; has_gnss_anchor_=false; imu_gps_warned_=false;
    gnss_anchor_offset_s_=0; gnss_anchor_arrival_ns_=0;
    has_nav_clock_=false; last_nav_clock_={}; pvt_time_valid_=false; nav_time_ref_.reset(); observation_time_ref_.reset();
    imu_clock_drift_reported_=false;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Receiver session reset: %s; waiting for fresh time/navigation references",
                disconnected ? "connection lost" : "measurement rewind or clock reset");
  }

  void pollStream() {
    uint8_t buffer[ubx::READ_BUFFER_SIZE];

    // RTCM relay: drain corrections from the TCP server to the receiver serial.
    // Same poll thread as the read below, so no lock.
    relay_.drainTo(get_logger(), *get_clock(), stream_);

    // Read loop to drain all available data from the stream
    for (int batch = 0; batch < 16 && rclcpp::ok(); ++batch) {
      const int bytes_read = strread(&stream_, buffer, sizeof(buffer));
    if (stream_session_.poll(stream_)) resetReceiverSession(true);
    const auto wire_now = std::chrono::steady_clock::now();
    receiver_health_.input(bytes_read);
    receiver_health_.framing(wire_framer_);
    receiver_health_.pvtPending(pending_.has_pvt || pending_.cov_received);
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


      if (bytes_read <= 0) {
        break; // No more data
      }

      for (int i = 0; i < bytes_read; ++i) {
      parseUbxByte(buffer[i]);
    }
      
      // If we read less than the full buffer, the stream is likely empty now
      if (bytes_read < static_cast<int>(sizeof(buffer))) {
        break;
      }
    }

    maybePublishHeartbeat();
    warnIfBinaryStarvation();
    maybeWatchdogFlushPendingPvt();
  }

  // Solution source policy:
  //   BINARY (NAV-PVT)  : if config_.enable_nav_pvt → use NAV-PVT only, drop NMEA
  //   NMEA             : otherwise → use NMEA-parsed solution
  // Source is locked at startup; no mid-session switching.
  enum class SolutionSource { BINARY, NMEA };

  void handleNmeaSentence(const std::string& sentence) {
    if (nmea_parser_.parseSentence(sentence, nmea_solution_)) {
      if (source_ == SolutionSource::NMEA) {
        publishSolution(nmea_solution_);
      }
    }
  }

  // PVT TOW Aggregation (mirrors SBF driver pattern; see ubx_decoder_node.cpp
  // for design notes). All NAV-* payloads carry iTOW at offset 0.
  // NAV-DOP is tracked separately in a persistent cache (last_dop_) so it can
  // survive Pending resets and be staleness-gated against the PVT cadence.
  // Only NAV-COV/POSECEF/VELECEF contribute to completion.
  static constexpr uint8_t COV_BIT_COV       = 0x1;
  static constexpr uint8_t COV_BIT_POSECEF   = 0x2;
  static constexpr uint8_t COV_BIT_VELECEF   = 0x4;
  static constexpr uint8_t COV_BIT_HPPOSLLH  = 0x8;
  static constexpr uint8_t COV_BIT_HPPOSECEF = 0x10;

  struct PendingPvt {
    uint32_t tow_ms{UINT32_MAX};
    bool has_pvt{false};
    bool has_pos_cov{false};
    bool has_vel_cov{false};
    uint8_t cov_received{0};
    uint8_t cov_expected{0};   // derived from YAML config (fixed at startup)
    msg::GnssSolution buf{};
    // High-precision scratch (NAV-HPPOSLLH / NAV-HPPOSECEF). Kept outside buf
    // and applied at flush time so the result does not depend on the in-epoch
    // arrival order vs NAV-PVT (which writes lat/lon/alt unconditionally).
    ubx::nav_hpposllh::Result hp_llh{};
    double hp_ecef[3]{0.0, 0.0, 0.0};
  };

  gnss_utils::DopCache last_dop_;
  ubx::nav_clock::Result last_nav_clock_{};
  bool has_nav_clock_{false};
  uint32_t prev_pvt_tow_ms_{UINT32_MAX};
  uint32_t pvt_period_ms_{0};

  // Last published (week, tow) for orphan guard. NAV-* bodies carry only TOW;
  // we compare against the week stored at the previous publishSolution call.
  uint32_t last_flushed_week_{0};
  uint32_t last_flushed_tow_ms_{UINT32_MAX};

  void initPendingExpectedMask() {
    pending_.cov_expected =
        (config_.enable_nav_cov       ? COV_BIT_COV       : uint8_t{0}) |
        (config_.enable_nav_posecef   ? COV_BIT_POSECEF   : uint8_t{0}) |
        (config_.enable_nav_velecef   ? COV_BIT_VELECEF   : uint8_t{0}) |
        (config_.enable_nav_hpposllh  ? COV_BIT_HPPOSLLH  : uint8_t{0}) |
        (config_.enable_nav_hpposecef ? COV_BIT_HPPOSECEF : uint8_t{0});
  }

  // offset: iTOW position — 0 for most NAV-* payloads, OFFSET_ITOW (4) for the
  // HP messages; the wrong offset breaks TOW grouping (stalls until watchdog).
  static uint32_t readItowMs(const uint8_t* p, size_t len, size_t offset = 0) {
    if (len < offset + 4) return UINT32_MAX;
    uint32_t v = 0;
    std::memcpy(&v, p + offset, 4);
    return v;
  }


  // Orphan guard: TOW matches the most-recently-flushed epoch. NAV-* bodies
  // don't carry a week field, so fall back to TOW-only when the current pending
  // has no PVT yet (buf.time_week is 0). Returns true → caller should skip.
  bool isLateOrphan(uint32_t tow) const {
    const uint32_t week = pending_.buf.time_week;
    return last_flushed_tow_ms_ != UINT32_MAX &&
           tow == last_flushed_tow_ms_ &&
           (week == 0 || week == last_flushed_week_);
  }

  void handleNavPvt() {
    if (!config_.enable_nav_pvt) return;
    const uint32_t tow = readItowMs(ubx_frm_payload_.data(), ubx_frm_payload_.size());
    if (isLateOrphan(tow)) return;
    if (tow != pending_.tow_ms && (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = tow;
    if (!ubx::pvt::parseNavPvt(ubx_frm_payload_.data(), ubx_frm_payload_.size(),
                               pending_.buf, pending_.has_pos_cov, pending_.has_vel_cov)) {
      GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return;
    }
    pending_.has_pvt = true;
    nav_time_ref_.reset();  // Do not retain a previous epoch's UTC validity.
    const auto* utc = ubx_frm_payload_.data();
    if (ubx::pvt::navPvtTimeIsValid(utc, ubx_frm_payload_.size())) {
      const int year = wire::u16(utc + 4);
      uint32_t week=0; double tow=0;
      if (gnss_utils::nmeaUtcToGpsTime(year,utc[6],utc[7],
          utc[8]*10000.+utc[9]*100.+utc[10],week,tow)) {
        nav_time_ref_.update(gpst2time(static_cast<int>(week),tow));
      }
    }
    pvt_time_valid_ =
        ubx::pvt::navPvtTimeIsValid(ubx_frm_payload_.data(), ubx_frm_payload_.size());
    epoch_flush_.touch();
    ever_received_binary_ = true;
    // Flush at the epoch boundary or shared quiet-period deadline.
  }

  // NAV-DOP: cached in last_dop_; applied at flush by applyDopWithStaleness().
  // NAV-DOP carries no GPS week → last_dop_.week = 0 (helper skips week check).
  void handleNavDop() {
    if (!config_.enable_nav_dop) return;
    const uint32_t tow = readItowMs(ubx_frm_payload_.data(), ubx_frm_payload_.size());
    msg::GnssSolution scratch{};
    if (!ubx::nav_dop::parseNavDop(ubx_frm_payload_.data(), ubx_frm_payload_.size(),
                                   scratch)) {
      GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return;
    }
    last_dop_.valid  = true;
    last_dop_.week   = 0;
    last_dop_.tow_ms = tow;
    last_dop_.gdop   = scratch.gdop;
    last_dop_.pdop   = scratch.pdop;
    last_dop_.hdop   = scratch.hdop;
    last_dop_.vdop   = scratch.vdop;
  }

  // NAV-CLOCK: receiver clock solution (iTOW + clkB + tAcc). See
  // ubx_decoder_node.cpp for why it is the preferred GPS-time anchor.
  void handleNavClock() {
    ubx::nav_clock::Result r{};
    if (!ubx::nav_clock::parseNavClock(ubx_frm_payload_.data(),
                                       ubx_frm_payload_.size(), r)) {
      GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return;
    }
    last_nav_clock_ = r;
    has_nav_clock_ = true;
  }

  void handleNavCov() {
    if (!config_.enable_nav_cov) return;
    const uint32_t tow = readItowMs(ubx_frm_payload_.data(), ubx_frm_payload_.size());
    if (isLateOrphan(tow)) return;
    if (tow != pending_.tow_ms && (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = tow;
    if (!ubx::nav_cov::parseNavCov(ubx_frm_payload_.data(), ubx_frm_payload_.size(),
                                   pending_.buf)) {
      GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return;
    }
    pending_.has_pos_cov |= ubx_frm_payload_[5] == 1;
    pending_.has_vel_cov |= ubx_frm_payload_[6] == 1;
    pending_.cov_received |= COV_BIT_COV;
    epoch_flush_.touch();
    // Flush at the epoch boundary or shared quiet-period deadline.
  }

  void handleNavPosEcef() {
    if (!config_.enable_nav_posecef) return;
    const uint32_t tow = readItowMs(ubx_frm_payload_.data(), ubx_frm_payload_.size());
    if (isLateOrphan(tow)) return;
    if (tow != pending_.tow_ms && (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = tow;
    if (!ubx::nav_posecef::parseNavPosEcef(ubx_frm_payload_.data(),
                                           ubx_frm_payload_.size(),
                                           pending_.buf)) {
      GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return;
    }
    pending_.cov_received  |= COV_BIT_POSECEF;
    epoch_flush_.touch();
    // Flush at the epoch boundary or shared quiet-period deadline.
  }

  void handleNavVelEcef() {
    if (!config_.enable_nav_velecef) return;
    const uint32_t tow = readItowMs(ubx_frm_payload_.data(), ubx_frm_payload_.size());
    if (isLateOrphan(tow)) return;
    if (tow != pending_.tow_ms && (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = tow;
    if (!ubx::nav_velecef::parseNavVelEcef(ubx_frm_payload_.data(),
                                           ubx_frm_payload_.size(),
                                           pending_.buf)) {
      GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return;
    }
    pending_.cov_received  |= COV_BIT_VELECEF;
    epoch_flush_.touch();
    // Flush at the epoch boundary or shared quiet-period deadline.
  }

  void handleNavHpPosEcef() {
    if (!config_.enable_nav_hpposecef) return;
    const uint32_t tow = readItowMs(ubx_frm_payload_.data(), ubx_frm_payload_.size(),
                                    ubx::nav_hpposecef::OFFSET_ITOW);
    if (isLateOrphan(tow)) return;
    if (tow != pending_.tow_ms && (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = tow;
    if (!ubx::nav_hpposecef::parseNavHpPosEcef(ubx_frm_payload_.data(),
                                               ubx_frm_payload_.size(),
                                               pending_.hp_ecef)) {
      GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return;
    }
    pending_.cov_received  |= COV_BIT_HPPOSECEF;
    epoch_flush_.touch();
    // Flush at the epoch boundary or shared quiet-period deadline.
  }

  void handleNavHpPosLlh() {
    if (!config_.enable_nav_hpposllh) return;
    const uint32_t tow = readItowMs(ubx_frm_payload_.data(), ubx_frm_payload_.size(),
                                    ubx::nav_hpposllh::OFFSET_ITOW);
    if (isLateOrphan(tow)) return;
    if (tow != pending_.tow_ms && (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = tow;
    if (!ubx::nav_hpposllh::parseNavHpPosLlh(ubx_frm_payload_.data(),
                                             ubx_frm_payload_.size(),
                                             pending_.hp_llh)) {
      GNSS_RX_EVENT(GNSS_RX_FRONTEND_REJECT,1); return;
    }
    pending_.cov_received  |= COV_BIT_HPPOSLLH;
    epoch_flush_.touch();
    // Flush at the epoch boundary or shared quiet-period deadline.
  }

  void flushPending() {
    const uint8_t expected = pending_.cov_expected;
    const uint8_t recv     = pending_.cov_received;
    if (!pending_.has_pvt) {
      if (pending_.cov_received) receiver_health_.pvtIncompleteDiscarded();
      pending_ = {};
      pending_.cov_expected = expected;
      return;
    }
    // Snapshot ECEF-direct fields before LLH-driven derivation overwrites them.
    msg::GnssSolution ecef_direct = pending_.buf;
    binary_solution_ = pending_.buf;

    // HPPOSLLH override (highest-priority LLH source), applied before geometry
    // derivation. Its accuracy fields refresh the covariance diagonal unless
    // NAV-COV already provided the full 3x3 (explicit valid NAV-COV block).
    if (recv & COV_BIT_HPPOSLLH) {
      binary_solution_.latitude  = pending_.hp_llh.lat_deg;
      binary_solution_.longitude = pending_.hp_llh.lon_deg;
      binary_solution_.altitude  = pending_.hp_llh.alt_m;
      const bool pos_cov_from_nav_cov = pending_.has_pos_cov;
      if (!pos_cov_from_nav_cov) {
        const double h_var_per_axis =
            pending_.hp_llh.hacc_m * pending_.hp_llh.hacc_m * 0.5;
        binary_solution_.pos_enu_cov[0] = h_var_per_axis;
        binary_solution_.pos_enu_cov[4] = h_var_per_axis;
        binary_solution_.pos_enu_cov[8] =
            pending_.hp_llh.vacc_m * pending_.hp_llh.vacc_m;
      }
    }

    // Derive ECEF position/velocity from LLH/ENU using current solution's LLH.
    decoder_common::finalizeBinarySolutionGeometry(binary_solution_);

    // Derive ECEF covariance from ENU covariance via current-LLH rotation.
    const double lat = binary_solution_.latitude  * D2R;
    const double lon = binary_solution_.longitude * D2R;
    double n[9], e[9];
    std::copy(binary_solution_.pos_enu_cov.begin(),
              binary_solution_.pos_enu_cov.end(), n);
    gnss_utils::rotateCovarianceEnuToEcef(n, lat, lon, e);
    std::copy(std::begin(e), std::end(e),
              binary_solution_.pos_cov_ecef.begin());
    std::copy(binary_solution_.vel_enu_cov.begin(),
              binary_solution_.vel_enu_cov.end(), n);
    gnss_utils::rotateCovarianceEnuToEcef(n, lat, lon, e);
    std::copy(std::begin(e), std::end(e),
              binary_solution_.vel_cov_ecef.begin());

    // ECEF-direct overrides, by priority: HPPOSECEF > NAV-POSECEF > LLH-derived.
    if (recv & COV_BIT_HPPOSECEF) {
      binary_solution_.pos_ecef.x = pending_.hp_ecef[0];
      binary_solution_.pos_ecef.y = pending_.hp_ecef[1];
      binary_solution_.pos_ecef.z = pending_.hp_ecef[2];
    } else if (recv & COV_BIT_POSECEF) {
      binary_solution_.pos_ecef = ecef_direct.pos_ecef;
    }
    if (recv & COV_BIT_VELECEF) {
      binary_solution_.vel_ecef = ecef_direct.vel_ecef;
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

    if (source_ == SolutionSource::BINARY) publishSolution(binary_solution_);

    // Record this epoch as the last published (week, tow) for orphan-guard.
    last_flushed_week_   = binary_solution_.time_week;
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

  // Warn if BINARY source was selected via YAML but no PVT has arrived after
  // a generous startup window — typically indicates a receiver-side config
  // or capability issue. Prevents silent failure under no-fallback policy.

  void warnIfBinaryStarvation() {
    if (source_ != SolutionSource::BINARY) return;
    if (ever_received_binary_) return;
    if ((now() - start_time_).seconds() < 15.0) return;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
      "Solution source locked to BINARY (NAV-PVT) but no PVT received yet — "
      "check receiver firmware/configuration.");
  }

  // Solution Publishing

  void publishSolution(msg::GnssSolution& sol) {
    // Block publishing until configuration is complete, unless configure_on_startup is false
    if (!is_configured_ && config_.configure_on_startup) {
        return;
    }

    // BINARY: keep the epoch's own iTOW; raw_.time supplies only the GPS week
    // (UBX NAV-* payloads carry none). NMEA: NmeaParser filled week/tow.
    gtime_t t_gpst{};
    int week_for_stamp = 0;
    if (source_ == SolutionSource::BINARY) {
      const gtime_t ref = nav_time_ref_.get().time != 0 ? nav_time_ref_.get() : observation_time_ref_.get();
      if (decoder_common::resolveGpsWeekForTow(ref, sol.time_tow, week_for_stamp)) {
        sol.time_week = static_cast<uint32_t>(week_for_stamp);
        t_gpst = gpst2time(week_for_stamp, sol.time_tow);
      }
      sol.solution_source = msg::GnssSolution::SOLUTION_SOURCE_BINARY;
    } else {
      sol.solution_source = msg::GnssSolution::SOLUTION_SOURCE_NMEA;
      if (sol.time_week > 0) {
        week_for_stamp = sol.time_week;
        t_gpst = gpst2time(sol.time_week, sol.time_tow);
      }
    }

    const rclcpp::Time sol_arrival = now();
    if (source_ == SolutionSource::BINARY && week_for_stamp > 0 &&
        pvt_time_valid_) {
      feedGnssAnchor(week_for_stamp, sol.time_tow, sol_arrival);
    }

    const auto measurement_stamp = time_base_.stamp(
        config_.use_gps_timestamp, week_for_stamp > 0 &&
        (source_ != SolutionSource::BINARY || pvt_time_valid_),
        t_gpst, sol_arrival, get_logger(), *get_clock());
    if (!measurement_stamp) { receiver_health_.timeWithheld(); return; }
    sol.header.stamp = *measurement_stamp;
    sol.header.frame_id = config_.frame_id;

    // Handle ENU conversion if a reference origin exists or setup if it doesn't
    if (decoder_common::finalizeReceiverSolution(
            sol, has_local_origin_, local_origin_ecef_, local_origin_pos_)) {
      RCLCPP_INFO(get_logger(), "Set local ENU origin: lat=%.6f lon=%.6f alt=%.2f",
                  local_origin_pos_[0] * R2D, local_origin_pos_[1] * R2D,
                  local_origin_pos_[2]);
    }

    sol_pub_->publish(sol);
  }

  void handleDecodeResult(int result) {
    receiver_health_.backendResult(result);
    if (result < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Rejected receiver payload (invalid layout, time, version or content)");
      return;
    }
    // Block publishing until configuration is complete, unless verify_on_startup is false
    if (!is_configured_ && config_.configure_on_startup) {
        return;
    }

    if (result == 1) {
        publishObservations();
    } else if (result == 2) {
        publishEphemerides();
    } else if (result == 9) {
        publishIonosphere();
    }
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
    const rclcpp::Time obs_arrival = now();
    // Anchor the ESF-RAW timeline; RXM-RAWX epochs are RTKLIB-validated.
    feedGnssAnchor(week, tow, obs_arrival);

    const auto measurement_stamp = time_base_.stamp(
        config_.use_gps_timestamp, week > 0,
        obs_time, obs_arrival, get_logger(), *get_clock());
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
      changed = eph_store_.ingestEph(raw_.nav.eph[i]) || changed;
    }
    for (int i = 0; i < raw_.nav.ng; ++i) {
      changed = eph_store_.ingestGeph(raw_.nav.geph[i]) || changed;
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

  // Configuration
  UbxConfig config_;
  bool is_usb_connection_{false};

  // Publishers
  rclcpp::Publisher<msg::GnssObservations>::SharedPtr  obs_pub_;
  rclcpp::Publisher<msg::GnssEphemerides>::SharedPtr   eph_pub_;
  rclcpp::Publisher<msg::GnssSolution>::SharedPtr      sol_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr  imu_pub_;       // ESF-INS (calibrated)
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr  imu_raw_pub_;   // ESF-RAW (uncalibrated)

  // sTtag -> ROS time for ESF-RAW. Nominal tick; the drift diagnostic
  // above is what catches a wrong constant.
  ins::SensorTickClock imu_clock_{ins::SensorTickClock::Config{
      ubx::esf_raw::STTAG_TICK_S, 1ull << 32}};
  bool imu_clock_drift_reported_{false};

  // GNSS time anchor for the ESF-RAW timeline (see feedGnssAnchor). The
  // estimator is GnssEpochAligner in TowAutoOffset mode, reused rather than
  // reimplemented because it is the tested one.
  gnss_utils::GnssEpochAligner gnss_anchor_{
      gnss_utils::GnssEpochAligner::Config{
          gnss_utils::GnssEpochAligner::Source::TowAutoOffset, 60.0}};
  double gnss_anchor_offset_s_{0.0};
  bool   has_gnss_anchor_{false};
  int64_t gnss_anchor_arrival_ns_{0};
  std::chrono::steady_clock::time_point gnss_anchor_steady_{};
  bool   imu_gps_warned_{false};
  bool   pvt_time_valid_{false};
  wire::FreshGnssReference nav_time_ref_, observation_time_ref_;
  rclcpp::TimerBase::SharedPtr timer_;

  // RTCM relay (PC -> receiver): STRSVR-style TCP server -> serial
  decoder_common::RtcmRelayServer relay_;

  // RTKLIB structures
  wire::Framer wire_framer_{wire::Protocol::Ubx};
  std::chrono::steady_clock::time_point wire_last_input_{std::chrono::steady_clock::now()};
  uint64_t wire_last_rejected_{0};
  stream_t stream_{};
  raw_t raw_{};


  // One time base per session, latched at the first published message and
  // never changed. Shared by EVERY topic this node publishes, which is what
  // structurally guarantees the IMU and the GNSS topics cannot end up on
  // different bases. See decoder_common::TimeBaseLatch.
  decoder_common::TimeBaseLatch time_base_;

  // NMEA Parsing state
  gnss_utils::NmeaParser nmea_parser_;
  msg::GnssSolution nmea_solution_;
  msg::GnssSolution binary_solution_;
  PendingPvt        pending_;
  SolutionSource    source_{SolutionSource::NMEA};
  rclcpp::Time      start_time_{0, 0, RCL_ROS_TIME};
  bool              ever_received_binary_{false};

  // ENU Origin state
  bool has_local_origin_{false};
  double local_origin_ecef_[3]{0.0};
  double local_origin_pos_[3]{0.0}; // lat/lon/hgt (rad, rad, m)

  // UBX mini-framer state (for ESF-INS / ESF-RAW)
  int                  ubx_frm_state_{0};
  uint8_t              ubx_frm_cls_{0};
  uint8_t              ubx_frm_id_{0};
  uint16_t             ubx_frm_len_{0};
  uint16_t             ubx_frm_pos_{0};
  std::vector<uint8_t> ubx_frm_payload_;

  // ACK/NAK detection (instance variables instead of function-static)
  uint8_t pending_ack_class_{0};
  uint8_t pending_ack_id_{0};
  bool ack_received_{false};
  bool nak_received_{false};
  wire::Framer ack_framer_{wire::Protocol::Ubx};
  uint8_t ack_buf_[10]{};
  int ack_pos_{0};

  // Unified ephemeris store
  gnss_utils::EphemerisStore eph_store_;

  // State
  bool is_configured_{false};
};

}  // namespace gnss_ros_standardization

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<gnss_ros_standardization::UbxDriverNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("ubx_driver_node"), "Startup/runtime failure: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
