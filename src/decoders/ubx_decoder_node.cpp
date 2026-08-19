// SPDX-License-Identifier: MIT
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
#include "gnss_ros_standardization/ubx_protocol.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"

using namespace std::chrono_literals;

namespace ubx = gnss_ros_standardization::ubx;
namespace ins = gnss_ros_standardization::ins_utils;

namespace gnss_ros_standardization {

namespace {

/// Timer interval for stream polling
constexpr auto kTimerInterval = 10ms;

/// Watchdog timeout for incomplete PVT epochs (seconds)
constexpr double kPvtWatchdogTimeoutSec = 1.5;

}  // namespace

/// @brief ROS 2 node for decoding u-blox UBX protocol messages
///
/// Decodes UBX-RXM-RAWX (raw observations) and UBX-RXM-SFRBX (navigation data)
/// messages from u-blox receivers and publishes them as standardized ROS messages.
/// Also parses NMEA sentences for GnssSolution and UBX-ESF-INS for calibrated IMU data.
class UbxDecoderNode : public rclcpp::Node {
 public:
  UbxDecoderNode() : Node("ubx_decoder_node") {
    initializeParameters();
    initializePublishers();
    initializeDecoder();
    openStream();
    openRtcmRelay();
    startPolling();

    RCLCPP_INFO(get_logger(), "UBX Decoder initialized");
  }

  ~UbxDecoderNode() override {
    try {
      strclose(&stream_);
      relay_.close();
    } catch (...) {
      // Ignore errors during cleanup
    }
    free_raw(&raw_);
  }

 private:
  // Initialization

  void initializeParameters() {
    declare_parameter<std::string>("stream_path", "serial:///dev/ttyACM0:115200");
    // frame_id: ROS TF frame name attached to the GNSS antenna phase center.
    // Default "gnss_link" — override in launch to integrate with vehicle TF tree.
    declare_parameter<std::string>("frame_id", "gnss_link");
    declare_parameter<std::string>("observation_topic", "/gnss/observation");
    declare_parameter<std::string>("ephemeris_topic", "/gnss/ephemeris");
    declare_parameter<std::string>("solution_topic", "/gnss/nmea_solution");
    declare_parameter<std::string>("imu_topic",     "/gnss/imu/data");
    declare_parameter<std::string>("imu_raw_topic", "/gnss/imu/data_raw");
    declare_parameter<double>("ephemeris.snapshot_period_s", 30.0);
    declare_parameter<double>("ephemeris.max_age_s", 0.0);  // 0 = keep all
    declare_parameter<bool>("use_gps_timestamp", false);
    declare_parameter<std::vector<double>>("origin", {0.0, 0.0, 0.0});
    // RTCM relay listen URI (opt-in, see openRtcmRelay). Empty = disabled.
    declare_parameter<std::string>("rtcm_relay", "");
    eph_store_.setSnapshotPeriod(get_parameter("ephemeris.snapshot_period_s").as_double());
    eph_store_.setMaxAge(get_parameter("ephemeris.max_age_s").as_double());

    frame_id_ = get_parameter("frame_id").as_string();
    use_gps_timestamp_ = get_parameter("use_gps_timestamp").as_bool();
    {
      const auto v = get_parameter("origin").as_double_array();
      if (v.size() == 3) {
        origin_latitude_  = v[0];
        origin_longitude_ = v[1];
        origin_altitude_  = v[2];
      }
    }
  }

  void initializePublishers() {
    obs_pub_     = create_publisher<msg::GnssObservations>(get_parameter("observation_topic").as_string(), 10);
    eph_pub_     = create_publisher<msg::GnssEphemerides>(get_parameter("ephemeris_topic").as_string(), rclcpp::QoS(256).transient_local()  /* depth 256: latch full ephemeris set, not just the last satellite (per-sat messages) */);
    sol_pub_     = create_publisher<msg::GnssSolution>(get_parameter("solution_topic").as_string(), 10);
    imu_pub_     = create_publisher<sensor_msgs::msg::Imu>(get_parameter("imu_topic").as_string(), 10);
    imu_raw_pub_ = create_publisher<sensor_msgs::msg::Imu>(get_parameter("imu_raw_topic").as_string(), 10);

    if (origin_latitude_ != 0.0 || origin_longitude_ != 0.0) {
      local_origin_pos_[0] = origin_latitude_  * (M_PI / 180.0);
      local_origin_pos_[1] = origin_longitude_ * (M_PI / 180.0);
      local_origin_pos_[2] = origin_altitude_;
      pos2ecef(local_origin_pos_, local_origin_ecef_);
      has_local_origin_ = true;
      RCLCPP_INFO(get_logger(), "ENU origin set from config: lat=%.6f lon=%.6f alt=%.2f",
        origin_latitude_, origin_longitude_, origin_altitude_);
    }
    logConfiguration();
  }

  void logConfiguration() {
    auto on = [](bool v) { return v ? "ON" : "OFF"; };
    RCLCPP_INFO(get_logger(), "Configuration:");
    RCLCPP_INFO(get_logger(), "  GPS timestamp : %s", on(use_gps_timestamp_));
    if (origin_latitude_ != 0.0 || origin_longitude_ != 0.0) {
      RCLCPP_INFO(get_logger(), "  ENU origin    : fixed (lat=%.6f lon=%.6f alt=%.2f)",
        origin_latitude_, origin_longitude_, origin_altitude_);
    } else {
      RCLCPP_INFO(get_logger(), "  ENU origin    : auto (first valid solution)");
    }
  }

  void initializeDecoder() {
    if (init_raw(&raw_, STRFMT_UBX) != 1) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize raw decoder");
      throw std::runtime_error("init_raw failed");
    }
  }

  void startPolling() {
    timer_ = create_wall_timer(kTimerInterval, std::bind(&UbxDecoderNode::pollStream, this));
  }

  // Stream Management

  void openStream() {
    // The RTCM relay writes corrections back down this stream, so it needs a
    // read-write open; without relay keep the passive read-only open.
    const int mode = get_parameter("rtcm_relay").as_string().empty()
                     ? STR_MODE_R : STR_MODE_RW;
    if (!decoder_common::openStreamPath(get_logger(), stream_,
                                        get_parameter("stream_path").as_string(),
                                        mode, "Input")) {
      throw std::runtime_error("stropen failed");
    }
  }

  // RTCM relay (RTKLIB STRSVR-style): host a TCP server (recommended
  // "tcpsvr://:5556", fed by rtcm_decoder_node's rtcm_relay) and write incoming
  // corrections to the receiver stream in pollStream() for on-chip RTK. No
  // receiver command is sent; the port's inProtoMask must already include
  // RTCM3 (F9P default) — see src/decoders/README.md.
  void openRtcmRelay() {
    const std::string listen = get_parameter("rtcm_relay").as_string();
    if (listen.empty()) return;
    if (get_parameter("stream_path").as_string().rfind("file://", 0) == 0) {
      RCLCPP_WARN(get_logger(),
        "rtcm_relay is set but stream_path is a file:// stream — forwarded "
        "corrections have no receiver to reach.");
    }
    if (!relay_.open(get_logger(), listen)) {
      throw std::runtime_error("RTCM relay stropen failed");
    }
  }

  void pollStream() {
    // RTCM relay: drain corrections from the TCP server and write them to the
    // receiver stream. Same poll thread as the read below, so no lock.
    relay_.drainTo(get_logger(), *get_clock(), stream_);

    uint8_t buffer[ubx::READ_BUFFER_SIZE];
    const int bytes_read = strread(&stream_, buffer, sizeof(buffer));

    for (int i = 0; i < bytes_read; ++i) {
      uint8_t byte = buffer[i];

      // Feed RTKLIB for UBX binary parsing (obs/eph)
      const int result = input_ubx(&raw_, byte);
      handleDecodeResult(result);

      // Parallel UBX mini-framer (ESF / NAV blocks)
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
        } else if (nmea_buffer_.size() > ubx::NMEA_MAX_LINE_LEN) {
          nmea_buffer_.clear();
        }
      }
    }

    maybePublishHeartbeat();
    commitSourceLockIfDue();  // ensure grace finalizes even on idle stream
    maybeWatchdogFlushPendingPvt();
  }

  // UBX Mini-Framer (parallel to RTKLIB, for ESF and NAV blocks)

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
    if (ubx_cls_ == ubx::CLASS_ESF && ubx_id_ == ubx::ID_ESF_RAW) handleEsfRaw();
    if (ubx_cls_ == ubx::CLASS_ESF && ubx_id_ == ubx::ID_ESF_INS) handleEsfIns();
    if (ubx_cls_ == ubx::CLASS_NAV && ubx_id_ == ubx::ID_NAV_PVT)     handleNavPvt();
    if (ubx_cls_ == ubx::CLASS_NAV && ubx_id_ == ubx::ID_NAV_DOP)     handleNavDop();
    if (ubx_cls_ == ubx::CLASS_NAV && ubx_id_ == ubx::ID_NAV_COV)     handleNavCov();
    if (ubx_cls_ == ubx::CLASS_NAV && ubx_id_ == ubx::ID_NAV_POSECEF) handleNavPosEcef();
    if (ubx_cls_ == ubx::CLASS_NAV && ubx_id_ == ubx::ID_NAV_VELECEF) handleNavVelEcef();
    if (ubx_cls_ == ubx::CLASS_NAV && ubx_id_ == ubx::ID_NAV_HPPOSECEF) handleNavHpPosEcef();
    if (ubx_cls_ == ubx::CLASS_NAV && ubx_id_ == ubx::ID_NAV_HPPOSLLH)  handleNavHpPosLlh();
  }

  // Solution source policy: publish nothing for kGracePeriodSec; binary PVT
  // seen during grace → lock BINARY, else lock NMEA. Deterministic and
  // PVT-preferred regardless of arrival order.
  enum class SolutionSource { UNDETERMINED, BINARY, NMEA };
  // 1.0 s covers a 1 Hz PVT cadence; higher rates lock BINARY in milliseconds.
  static constexpr double kGracePeriodSec = 1.0;

  void startGraceIfNeeded() {
    if (!grace_started_) {
      grace_start_ = now();
      grace_started_ = true;
    }
  }

  void commitSourceLockIfDue() {
    if (source_ != SolutionSource::UNDETERMINED) return;
    if (!grace_started_) return;
    const bool elapsed = (now() - grace_start_).seconds() >= kGracePeriodSec;
    if (saw_binary_during_grace_) {
      source_ = SolutionSource::BINARY;
      RCLCPP_INFO(get_logger(), "Solution source locked: BINARY (PVT detected during grace)");
    } else if (elapsed) {
      source_ = SolutionSource::NMEA;
      RCLCPP_INFO(get_logger(), "Solution source locked: NMEA (no PVT during grace)");
    }
  }

  // PVT TOW aggregation: buffer NAV-* blocks per iTOW epoch and flush when the
  // learned block set is complete (cov_ever_seen is sticky — it self-calibrates
  // to whichever blocks the receiver actually emits).
  static constexpr uint8_t COV_BIT_COV       = 0x1;
  static constexpr uint8_t COV_BIT_POSECEF   = 0x2;
  static constexpr uint8_t COV_BIT_VELECEF   = 0x4;
  static constexpr uint8_t COV_BIT_HPPOSLLH  = 0x8;
  static constexpr uint8_t COV_BIT_HPPOSECEF = 0x10;

  struct PendingPvt {
    uint32_t tow_ms{UINT32_MAX};
    bool has_pvt{false};
    uint8_t cov_received{0};
    uint8_t cov_ever_seen{0};
    msg::GnssSolution buf{};
    // High-precision scratch (NAV-HPPOSLLH / NAV-HPPOSECEF). Kept outside buf
    // and applied at flush time: NAV-PVT writes lat/lon/alt unconditionally,
    // so writing HP values into buf would make the result depend on in-epoch
    // arrival order.
    ubx::nav_hpposllh::Result hp_llh{};
    double hp_ecef[3]{0.0, 0.0, 0.0};
    rclcpp::Time last_update{0, 0, RCL_ROS_TIME};
  };

  gnss_utils::DopCache last_dop_;
  uint32_t prev_pvt_tow_ms_{UINT32_MAX};
  uint32_t pvt_period_ms_{0};

  // After a publish, record the (week, tow) to suppress orphan re-publish.
  uint32_t last_flushed_week_{0};
  uint32_t last_flushed_tow_ms_{UINT32_MAX};

  // offset: iTOW position — 0 for most NAV-* payloads, OFFSET_ITOW (4) for the
  // HP messages; the wrong offset breaks TOW grouping (stalls until watchdog).
  static uint32_t readItowMs(const uint8_t* p, size_t len, size_t offset = 0) {
    if (len < offset + 4) return UINT32_MAX;
    uint32_t v = 0;
    std::memcpy(&v, p + offset, 4);
    return v;
  }

  bool pendingComplete() const {
    return pending_.has_pvt &&
           pending_.cov_received == pending_.cov_ever_seen;
  }

  // Epoch-gap diagnostic: learns the nominal cadence as the smallest positive
  // epoch delta seen so far, and warns when a delta exceeds 2.5x that cadence —
  // i.e., whole epochs are missing from the stream (link loss or the receiver
  // skipping epochs). last_tow_s / min_period_s are caller-owned state;
  // a backwards tow (week rollover) only reseeds the state.
  void warnOnEpochGap(const char* what, double tow_s,
                      double& last_tow_s, double& min_period_s) {
    if (last_tow_s >= 0.0 && tow_s > last_tow_s) {
      const double dt = tow_s - last_tow_s;
      if (min_period_s == 0.0 || dt < min_period_s) min_period_s = dt;
      if (dt > min_period_s * 2.5) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "%s epoch gap: %.1fs (expected %.3fs) — epochs missing from the stream",
          what, dt, min_period_s);
      }
    }
    last_tow_s = tow_s;
  }

  // Orphan guard: same TOW as the last publish → late arrival from an eager
  // flush. Update cov_ever_seen only; re-accumulating would duplicate-publish.
  // Week falls back to TOW-only while this epoch has no PVT (time_week == 0).
  bool isLateOrphan(uint32_t tow, uint8_t cov_bit) {
    const uint32_t week = pending_.buf.time_week;
    if (last_flushed_tow_ms_ != UINT32_MAX &&
        tow == last_flushed_tow_ms_ &&
        (week == 0 || week == last_flushed_week_)) {
      pending_.cov_ever_seen |= cov_bit;
      return true;
    }
    return false;
  }

  void handleNavPvt() {
    startGraceIfNeeded();
    const uint32_t tow = readItowMs(ubx_payload_.data(), ubx_payload_.size());
    if (isLateOrphan(tow, 0)) return;
    if (tow != pending_.tow_ms && (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = tow;
    if (!ubx::pvt::parseNavPvt(ubx_payload_.data(), ubx_payload_.size(),
                               pending_.buf)) {
      return;
    }
    pending_.has_pvt = true;
    pending_.last_update = now();
    saw_binary_during_grace_ = true;
    warnOnEpochGap("NAV-PVT", tow * 1e-3,
                   pvt_gap_last_tow_s_, pvt_gap_min_period_s_);
    commitSourceLockIfDue();
    if (pendingComplete()) flushPending();
  }

  // NAV-DOP: cached in last_dop_; applied at flush by applyDopWithStaleness().
  // NAV-DOP carries no GPS week → last_dop_.week = 0 (helper skips week check).
  void handleNavDop() {
    startGraceIfNeeded();
    const uint32_t tow = readItowMs(ubx_payload_.data(), ubx_payload_.size());
    msg::GnssSolution scratch{};
    if (!ubx::nav_dop::parseNavDop(ubx_payload_.data(), ubx_payload_.size(),
                                   scratch)) {
      return;
    }
    last_dop_.valid  = true;
    last_dop_.week   = 0;       // NAV-DOP has no week field
    last_dop_.tow_ms = tow;
    last_dop_.gdop   = scratch.gdop;
    last_dop_.pdop   = scratch.pdop;
    last_dop_.hdop   = scratch.hdop;
    last_dop_.vdop   = scratch.vdop;
    saw_binary_during_grace_ = true;
    commitSourceLockIfDue();
  }

  void handleNavCov() {
    startGraceIfNeeded();
    const uint32_t tow = readItowMs(ubx_payload_.data(), ubx_payload_.size());
    if (isLateOrphan(tow, COV_BIT_COV)) return;
    if (tow != pending_.tow_ms && (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = tow;
    if (!ubx::nav_cov::parseNavCov(ubx_payload_.data(), ubx_payload_.size(),
                                   pending_.buf)) {
      return;
    }
    pending_.cov_received |= COV_BIT_COV;
    pending_.cov_ever_seen |= COV_BIT_COV;
    pending_.last_update = now();
    saw_binary_during_grace_ = true;
    commitSourceLockIfDue();
    if (pendingComplete()) flushPending();
  }

  void handleNavPosEcef() {
    startGraceIfNeeded();
    const uint32_t tow = readItowMs(ubx_payload_.data(), ubx_payload_.size());
    if (isLateOrphan(tow, COV_BIT_POSECEF)) return;
    if (tow != pending_.tow_ms && (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = tow;
    if (!ubx::nav_posecef::parseNavPosEcef(ubx_payload_.data(), ubx_payload_.size(),
                                           pending_.buf)) {
      return;
    }
    pending_.cov_received  |= COV_BIT_POSECEF;
    pending_.cov_ever_seen |= COV_BIT_POSECEF;
    pending_.last_update = now();
    saw_binary_during_grace_ = true;
    commitSourceLockIfDue();
    if (pendingComplete()) flushPending();
  }

  void handleNavVelEcef() {
    startGraceIfNeeded();
    const uint32_t tow = readItowMs(ubx_payload_.data(), ubx_payload_.size());
    if (isLateOrphan(tow, COV_BIT_VELECEF)) return;
    if (tow != pending_.tow_ms && (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = tow;
    if (!ubx::nav_velecef::parseNavVelEcef(ubx_payload_.data(), ubx_payload_.size(),
                                           pending_.buf)) {
      return;
    }
    pending_.cov_received  |= COV_BIT_VELECEF;
    pending_.cov_ever_seen |= COV_BIT_VELECEF;
    pending_.last_update = now();
    saw_binary_during_grace_ = true;
    commitSourceLockIfDue();
    if (pendingComplete()) flushPending();
  }

  void handleNavHpPosEcef() {
    startGraceIfNeeded();
    const uint32_t tow = readItowMs(ubx_payload_.data(), ubx_payload_.size(),
                                    ubx::nav_hpposecef::OFFSET_ITOW);
    if (isLateOrphan(tow, COV_BIT_HPPOSECEF)) return;
    if (tow != pending_.tow_ms && (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = tow;
    if (!ubx::nav_hpposecef::parseNavHpPosEcef(ubx_payload_.data(), ubx_payload_.size(),
                                               pending_.hp_ecef)) {
      return;
    }
    pending_.cov_received  |= COV_BIT_HPPOSECEF;
    pending_.cov_ever_seen |= COV_BIT_HPPOSECEF;
    pending_.last_update = now();
    saw_binary_during_grace_ = true;
    commitSourceLockIfDue();
    if (pendingComplete()) flushPending();
  }

  void handleNavHpPosLlh() {
    startGraceIfNeeded();
    const uint32_t tow = readItowMs(ubx_payload_.data(), ubx_payload_.size(),
                                    ubx::nav_hpposllh::OFFSET_ITOW);
    if (isLateOrphan(tow, COV_BIT_HPPOSLLH)) return;
    if (tow != pending_.tow_ms && (pending_.has_pvt || pending_.cov_received)) {
      flushPending();
    }
    pending_.tow_ms = tow;
    if (!ubx::nav_hpposllh::parseNavHpPosLlh(ubx_payload_.data(), ubx_payload_.size(),
                                             pending_.hp_llh)) {
      return;
    }
    pending_.cov_received  |= COV_BIT_HPPOSLLH;
    pending_.cov_ever_seen |= COV_BIT_HPPOSLLH;
    pending_.last_update = now();
    saw_binary_during_grace_ = true;
    commitSourceLockIfDue();
    if (pendingComplete()) flushPending();
  }

  void flushPending() {
    const uint8_t ever_seen = pending_.cov_ever_seen;
    const uint8_t recv      = pending_.cov_received;
    if (!pending_.has_pvt) {
      pending_ = {};
      pending_.cov_ever_seen = ever_seen;
      return;
    }
    // Snapshot ECEF-direct fields before LLH-driven derivation overwrites them.
    msg::GnssSolution ecef_direct = pending_.buf;
    binary_solution_ = pending_.buf;

    // HPPOSLLH override (highest-priority LLH source), applied before geometry
    // derivation. Its accuracy fields refresh the covariance diagonal unless
    // NAV-COV already provided the full 3x3 (non-zero off-diagonal).
    if (recv & COV_BIT_HPPOSLLH) {
      binary_solution_.latitude  = pending_.hp_llh.lat_deg;
      binary_solution_.longitude = pending_.hp_llh.lon_deg;
      binary_solution_.altitude  = pending_.hp_llh.alt_m;
      const bool pos_cov_from_nav_cov =
          binary_solution_.pos_enu_cov[1] != 0.0 ||
          binary_solution_.pos_enu_cov[2] != 0.0 ||
          binary_solution_.pos_enu_cov[5] != 0.0;
      if (!pos_cov_from_nav_cov) {
        const double h_var_per_axis =
            pending_.hp_llh.hacc_m * pending_.hp_llh.hacc_m * 0.5;
        binary_solution_.pos_enu_cov[0] = h_var_per_axis;
        binary_solution_.pos_enu_cov[4] = h_var_per_axis;
        binary_solution_.pos_enu_cov[8] =
            pending_.hp_llh.vacc_m * pending_.hp_llh.vacc_m;
      }
    }

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
    // publishSolution() will have populated time_week via raw_.time mapping.
    last_flushed_week_   = binary_solution_.time_week;
    last_flushed_tow_ms_ = pending_.tow_ms;

    pending_ = {};
    pending_.cov_ever_seen = ever_seen;
  }

  void maybeWatchdogFlushPendingPvt() {
    if (!pending_.has_pvt && !pending_.cov_received) return;
    if ((now() - pending_.last_update).seconds() > kPvtWatchdogTimeoutSec) {
      flushPending();
    }
  }

  // UBX-ESF-RAW: batched IMU samples, one Imu published per sTtag sample set
  // (see ubx::esf_raw::parseEsfRawSamples). ESF-RAW carries no GNSS time —
  // only the receiver-internal sTtag — so stamps stay host-clock based even
  // with use_gps_timestamp: the read time anchors the newest sample and
  // earlier samples are spaced backwards by sTtag deltas, keeping per-sample
  // dt accurate for downstream integration.
  void handleEsfRaw() {
    const auto samples = ubx::esf_raw::parseEsfRawSamples(ubx_payload_);
    if (samples.empty()) return;

    const rclcpp::Time anchor = now();
    const uint32_t last_sttag = samples.back().sttag;
    // Sanity guard: a batch spanning > 1 s means a wrong tick constant or
    // corrupted time tags — fall back to stamping the whole batch at arrival.
    const double span_s =
        (last_sttag - samples.front().sttag) * ubx::esf_raw::STTAG_TICK_S;
    const bool spread_ok = span_s <= 1.0;
    if (!spread_ok) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "ESF-RAW sTtag span %.3f s exceeds 1 s — stamping batch at arrival time", span_s);
    }

    for (const auto& s : samples) {
      sensor_msgs::msg::Imu imu;
      const double back_s =
          spread_ok ? (last_sttag - s.sttag) * ubx::esf_raw::STTAG_TICK_S : 0.0;
      imu.header.stamp    = anchor - rclcpp::Duration::from_seconds(back_s);
      imu.header.frame_id = frame_id_;

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
    if (static_cast<int>(ubx_payload_.size()) < ubx::esf_ins::MIN_LEN) return;

    // Validity bits in bitfield0 (bytes 0-3)
    uint32_t bitfield0 = 0;
    std::memcpy(&bitfield0, ubx_payload_.data() + ubx::esf_ins::OFFSET_BITFIELD, 4);
    const bool ang_valid = (bitfield0 & (0x7u << 8)) == (0x7u << 8);  // bits 8,9,10
    const bool acc_valid = (bitfield0 & (0x7u << 11)) == (0x7u << 11); // bits 11,12,13

    int32_t xAngRate_raw = 0, yAngRate_raw = 0, zAngRate_raw = 0;
    int32_t xAccel_raw = 0, yAccel_raw = 0, zAccel_raw = 0;
    std::memcpy(&xAngRate_raw, ubx_payload_.data() + ubx::esf_ins::OFFSET_XANGRATE, 4);
    std::memcpy(&yAngRate_raw, ubx_payload_.data() + ubx::esf_ins::OFFSET_YANGRATE, 4);
    std::memcpy(&zAngRate_raw, ubx_payload_.data() + ubx::esf_ins::OFFSET_ZANGRATE, 4);
    std::memcpy(&xAccel_raw,   ubx_payload_.data() + ubx::esf_ins::OFFSET_XACCEL,   4);
    std::memcpy(&yAccel_raw,   ubx_payload_.data() + ubx::esf_ins::OFFSET_YACCEL,   4);
    std::memcpy(&zAccel_raw,   ubx_payload_.data() + ubx::esf_ins::OFFSET_ZACCEL,   4);

    // Scale: angular rate 1e-3 deg/s → rad/s; acceleration 1e-2 m/s²
    const double deg2rad = M_PI / 180.0;

    sensor_msgs::msg::Imu imu;
    imu.header.stamp    = now();
    imu.header.frame_id = frame_id_;

    // orientation: not provided by ESF-INS — leave identity, mark unknown
    imu.orientation_covariance[0] = -1.0;

    imu.angular_velocity.x = xAngRate_raw * 1e-3 * deg2rad;
    imu.angular_velocity.y = yAngRate_raw * 1e-3 * deg2rad;
    imu.angular_velocity.z = zAngRate_raw * 1e-3 * deg2rad;
    auto ang_cov = ang_valid ? ins::makeDiagCovariance(1e-4, 1e-4, 1e-4) : ins::makeUnknownCovariance();
    std::copy(ang_cov.begin(), ang_cov.end(), imu.angular_velocity_covariance.begin());

    imu.linear_acceleration.x = xAccel_raw * 1e-2;
    imu.linear_acceleration.y = yAccel_raw * 1e-2;
    imu.linear_acceleration.z = zAccel_raw * 1e-2;
    auto acc_cov = acc_valid ? ins::makeDiagCovariance(1e-3, 1e-3, 1e-3) : ins::makeUnknownCovariance();
    std::copy(acc_cov.begin(), acc_cov.end(), imu.linear_acceleration_covariance.begin());

    imu_pub_->publish(imu);
  }

  // Message Handling

  void handleNmeaSentence(const std::string& sentence) {
    startGraceIfNeeded();
    if (!nmea_parser_.parseSentence(sentence, nmea_solution_)) return;
    commitSourceLockIfDue();
    if (source_ == SolutionSource::NMEA) {
      publishSolution(nmea_solution_);
    }
  }

  void handleDecodeResult(int result) {
    switch (result) {
      case 1:  publishObservations(); break;
      case 2:  publishEphemerides();  break;
      default: break;
    }
  }

  // Solution Publishing

  void publishSolution(msg::GnssSolution& sol) {
    // BINARY: keep the epoch's own iTOW; raw_.time supplies only the GPS week
    // (UBX NAV-* payloads carry none). NMEA: NmeaParser filled week/tow.
    gtime_t t_gpst{};
    int week_for_stamp = 0;
    if (source_ == SolutionSource::BINARY) {
      const double raw_tow = time2gpst(raw_.time, &week_for_stamp);
      if (week_for_stamp > 0) {
        if (sol.time_tow > 0.0) {
          // Week-rollover guard: iTOW vs raw_tow more than half a week apart
          // means the two clocks straddle a GPS week boundary.
          if      (sol.time_tow - raw_tow >  302400.0) week_for_stamp -= 1;
          else if (sol.time_tow - raw_tow < -302400.0) week_for_stamp += 1;
          sol.time_week = static_cast<uint32_t>(week_for_stamp);
          t_gpst = gpst2time(week_for_stamp, sol.time_tow);
        } else {
          // iTOW unavailable (exact week-start epoch) — previous behavior.
          sol.time_week = static_cast<uint32_t>(week_for_stamp);
          sol.time_tow  = raw_tow;
          t_gpst = raw_.time;
        }
      } else {
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

    sol.header.stamp    = (use_gps_timestamp_ && week_for_stamp > 0)
                          ? gnss_utils::gpstToUtcRosTime(t_gpst) : now();
    sol.header.frame_id = frame_id_;

    if (sol.status == msg::GnssSolution::STATUS_FIX ||
        sol.status == msg::GnssSolution::STATUS_FLOAT ||
        sol.status == msg::GnssSolution::STATUS_SINGLE ||
        sol.status == msg::GnssSolution::STATUS_DGPS) {

      if (!has_local_origin_) {
        local_origin_ecef_[0] = sol.pos_ecef.x;
        local_origin_ecef_[1] = sol.pos_ecef.y;
        local_origin_ecef_[2] = sol.pos_ecef.z;
        ecef2pos(local_origin_ecef_, local_origin_pos_);
        has_local_origin_ = true;
        RCLCPP_INFO(get_logger(), "Set local ENU origin: lat=%.6f, lon=%.6f, alt=%.2f",
                    local_origin_pos_[0] * (180.0/M_PI), local_origin_pos_[1] * (180.0/M_PI), local_origin_pos_[2]);
      }

      sol.pos_enu_org_ecef.x = local_origin_ecef_[0];
      sol.pos_enu_org_ecef.y = local_origin_ecef_[1];
      sol.pos_enu_org_ecef.z = local_origin_ecef_[2];

      double ecef[3] = {
        sol.pos_ecef.x - local_origin_ecef_[0],
        sol.pos_ecef.y - local_origin_ecef_[1],
        sol.pos_ecef.z - local_origin_ecef_[2]
      };
      double enu[3] = {0};
      ecef2enu(local_origin_pos_, ecef, enu);

      sol.pos_enu.x = enu[0];
      sol.pos_enu.y = enu[1];
      sol.pos_enu.z = enu[2];

      // vel_enu uses CURRENT-position frame (matches msg comment & receiver convention).
      double vel_ecef[3] = {sol.vel_ecef.x, sol.vel_ecef.y, sol.vel_ecef.z};
      double vel_enu[3] = {0};
      const double cur_llh[3] = {sol.latitude * D2R, sol.longitude * D2R, sol.altitude};
      ecef2enu(cur_llh, vel_ecef, vel_enu);

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

  // Observation Publishing

  void publishObservations() {
    if (raw_.obs.n <= 0) return;

    // obs.data[0].time, not raw_.time (already advanced to the next epoch).
    int week = 0;
    const gtime_t obs_time = raw_.obs.data[0].time;
    const double tow = time2gpst(obs_time, &week);

    warnOnEpochGap("Observation", tow,
                   obs_gap_last_tow_s_, obs_gap_min_period_s_);

    msg::GnssObservations msg;
    msg.header.stamp    = (use_gps_timestamp_ && week > 0)
                          ? gnss_utils::gpstToUtcRosTime(obs_time) : now();
    msg.header.frame_id = frame_id_;
    msg.week = static_cast<uint32_t>(week);
    msg.tow = tow;

    decoder_common::SatelliteCount sat_count{};
    for (int i = 0; i < raw_.obs.n; ++i) {
      const obsd_t& obs = raw_.obs.data[i];
      decoder_common::countSatellite(obs.sat, sat_count);
      decoder_common::appendObservations(obs, msg.observations);
    }

    obs_pub_->publish(msg);

    RCLCPP_INFO(get_logger(),
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

  // Member Variables

  std::string frame_id_;
  bool        use_gps_timestamp_{false};

  rclcpp::Publisher<msg::GnssObservations>::SharedPtr   obs_pub_;
  rclcpp::Publisher<msg::GnssEphemerides>::SharedPtr    eph_pub_;
  rclcpp::Publisher<msg::GnssSolution>::SharedPtr       sol_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr   imu_pub_;       // ESF-INS (calibrated)
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr   imu_raw_pub_;   // ESF-RAW (uncalibrated)
  rclcpp::TimerBase::SharedPtr timer_;

  stream_t stream_{};
  raw_t    raw_{};

  // RTCM relay (PC -> receiver): STRSVR-style TCP server -> receiver stream
  decoder_common::RtcmRelayServer relay_;

  // NMEA parsing state
  gnss_utils::NmeaParser nmea_parser_;
  std::string            nmea_buffer_;
  msg::GnssSolution      nmea_solution_;
  msg::GnssSolution      binary_solution_;
  PendingPvt             pending_;
  SolutionSource         source_{SolutionSource::UNDETERMINED};
  rclcpp::Time           grace_start_{0, 0, RCL_ROS_TIME};
  bool                   grace_started_{false};
  bool                   saw_binary_during_grace_{false};

  // ENU origin
  double origin_latitude_{0.0};
  double origin_longitude_{0.0};
  double origin_altitude_{0.0};
  bool   has_local_origin_{false};
  double local_origin_ecef_[3]{0.0};
  double local_origin_pos_[3]{0.0};

  // UBX mini-framer state (ESF-INS / ESF-RAW)
  int                  ubx_state_{0};
  uint8_t              ubx_cls_{0};
  uint8_t              ubx_id_{0};
  uint16_t             ubx_len_{0};
  uint16_t             ubx_pos_{0};
  std::vector<uint8_t> ubx_payload_;

  // Epoch-gap detection state (see warnOnEpochGap)
  double pvt_gap_last_tow_s_{-1.0};
  double pvt_gap_min_period_s_{0.0};
  double obs_gap_last_tow_s_{-1.0};
  double obs_gap_min_period_s_{0.0};

  // Ephemeris dedup
  gnss_utils::EphemerisStore eph_store_;
};

}  // namespace gnss_ros_standardization

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gnss_ros_standardization::UbxDecoderNode>());
  rclcpp::shutdown();
  return 0;
}