#ifndef GNSS_ROS_STANDARDIZATION_DECODER_COMMON_HPP
#define GNSS_ROS_STANDARDIZATION_DECODER_COMMON_HPP

/// @file decoder_common.hpp
/// @brief Shared helpers used by the per-receiver decoder and driver nodes.
///
/// Receiver-agnostic glue logic (observation publication helpers, satellite
/// counting, ECEF↔LLH/ENU finalization, etc.) lives here so all
/// decoder/driver pairs share one implementation. Receiver-specific framing
/// and protocol parsing remain in `*_protocol.hpp`.

#include <cmath>
// ---- command reply ----
#include "gnss_ros_standardization/binary_framer.hpp"
#include <cstdint>
#include <string>
#include <vector>

namespace gnss_ros_standardization::wire {
// ASCII replies are serialized (one outstanding command). Binary log payloads
// are skipped by their wire length, so an embedded '<OK' is never an ACK.
class CommandReply {
public:
  CommandReply(Protocol protocol, std::string command)
      : protocol_(protocol), command_(trim(std::move(command))) {}
  int feed(uint8_t c) {
    if (result_ != 0)
      return result_;
    if (remaining_) {
      --remaining_;
      return result_;
    }
    if (!header_.empty()) {
      header_.push_back(c);
      if (header_.size() == 2 &&
          c != (protocol_ == Protocol::Sbf ? 0x40 : 0x44)) {
        for (auto b : header_)
          ascii(b);
        header_.clear();
      } else if (protocol_ == Protocol::Sbf && header_.size() == 8) {
        const size_t len = u16(header_.data() + 6);
        if (len >= 8 && len % 4 == 0)
          remaining_ = len - 8;
        header_.clear();
        line_.clear();
      } else if (protocol_ == Protocol::Novatel) {
        if (header_.size() == 3 && c != 0x12 && c != 0x13) {
          header_.clear();
          line_.clear();
        } else if (header_.size() == 10) {
          const bool short_header = header_[2] == 0x13;
          const size_t len = short_header
                                 ? 12 + header_[3] + 4
                                 : header_[3] + u16(header_.data() + 8) + 4;
          if (len >= 14 && (short_header || header_[3] >= 28))
            remaining_ = len - 10;
          header_.clear();
          line_.clear();
        }
      }
      return result_;
    }
    if (c == (protocol_ == Protocol::Sbf ? 0x24 : 0xAA))
      header_.push_back(c);
    else
      ascii(c);
    return result_;
  }

private:
  static std::string trim(std::string s) {
    const auto first = s.find_first_not_of(" \r\n\t");
    if (first == std::string::npos)
      return {};
    return s.substr(first, s.find_last_not_of(" \r\n\t") - first + 1);
  }
  // A receiver prompt has no line terminator and can arrive in a later read
  // than the preceding ACK. Strip only a complete, recognized port prompt;
  // arbitrary echo text containing an ACK must never acknowledge a command.
  static size_t promptLength(Protocol protocol, const std::string& line) {
    const bool bracket = protocol == Protocol::Novatel;
    if (bracket && (line.empty() || line[0] != '[')) return 0;
    size_t i = bracket ? 1 : 0;
    const size_t start = i;
    while (i < line.size() && line[i] >= 'A' && line[i] <= 'Z') ++i;
    const size_t letters = i - start, digit_start = i;
    while (i < line.size() && line[i] >= '0' && line[i] <= '9') ++i;
    const size_t digits = i - digit_start;
    if (i == line.size()) return 0;
    if (!bracket)
      return letters >= 2 && letters <= 4 && digits >= 1 && digits <= 2 && line[i] == '>' ? i + 1 : 0;
    return letters >= 2 && letters <= 6 && digits <= 2 && line[i] == ']' ? i + 1 : 0;
  }
  std::string withoutPrompt(std::string line) const {
    while (const size_t length = promptLength(protocol_, line))
      line = trim(line.substr(length));
    return line;
  }
  void ascii(uint8_t c) {
    if (c == '\r' || c == '\n') {
      const auto line = withoutPrompt(trim(line_));
      if (protocol_ == Protocol::Sbf) {
        if (line.rfind("$R?", 0) == 0)
          result_ = -1;
        else if (line.rfind("$R:", 0) == 0 && trim(line.substr(3)) == command_)
          result_ = 1;
      } else {
        if (line == "<OK")
          result_ = 1;
        else if (line.rfind("<ERROR", 0) == 0)
          result_ = -1;
      }
      line_.clear();
    } else if (c >= 32 && c < 127 && line_.size() < 4096)
      line_ += char(c);
    else
      line_.clear();
  }
  Protocol protocol_;
  std::string command_, line_;
  std::vector<uint8_t> header_;
  size_t remaining_{0};
  int result_{0};
};
} // namespace gnss_ros_standardization::wire
#include <optional>
#include <limits>
#include <chrono>
#include <thread>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/msg/gnss_observation.hpp"
#include "gnss_ros_standardization/msg/gnss_observations.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"

#include "gnss_ros_standardization/rtklib_compat.hpp"

namespace gnss_ros_standardization {
namespace decoder_common {

/// History depth of the receiver IMU publishers. One stream read (a file:// or
/// TCP chunk, a u-blox ESF-RAW batch) can carry many IMU samples at once; a
/// KEEP_LAST history shorter than that burst overwrites samples a reliable
/// subscriber has not yet acknowledged.
constexpr std::size_t kImuPublisherDepth = 1000;

/// Finish the receiver-independent part of PVT publication. Position ENU uses
/// the persistent origin; velocity ENU uses the current geodetic position.
/// Returns true only when this solution establishes the automatic origin.
inline bool finalizeReceiverSolution(msg::GnssSolution& sol, bool& has_origin,
                                     double (&origin_ecef)[3], double (&origin_llh)[3]) {
  if (sol.status == msg::GnssSolution::STATUS_NONE || !std::isfinite(sol.pos_ecef.x) ||
      !std::isfinite(sol.pos_ecef.y) || !std::isfinite(sol.pos_ecef.z)) {
    gnss_utils::setNoSolution(sol);
    return false;
  }
  // A velocity the receiver could not provide carries no covariance either.
  if (!std::isfinite(sol.vel_ecef.x) || !std::isfinite(sol.vel_ecef.y) ||
      !std::isfinite(sol.vel_ecef.z))
    gnss_utils::setUnknownVelocity(sol);
  // Only a GNSS position fix defines the automatic origin; a dead-reckoned
  // position is expressed relative to an existing one.
  const bool established = !has_origin && gnss_utils::hasPositionFix(sol);
  if (established) {
    origin_ecef[0] = sol.pos_ecef.x;
    origin_ecef[1] = sol.pos_ecef.y;
    origin_ecef[2] = sol.pos_ecef.z;
    ecef2pos(origin_ecef, origin_llh);
    has_origin = true;
  }
  double enu[3]{};
  if (has_origin) {
    sol.pos_enu_org_ecef.x = origin_ecef[0];
    sol.pos_enu_org_ecef.y = origin_ecef[1];
    sol.pos_enu_org_ecef.z = origin_ecef[2];
    const double delta[3] = {sol.pos_ecef.x - origin_ecef[0],
                            sol.pos_ecef.y - origin_ecef[1],
                            sol.pos_ecef.z - origin_ecef[2]};
    ecef2enu(origin_llh, delta, enu);
    sol.pos_enu.x = enu[0]; sol.pos_enu.y = enu[1]; sol.pos_enu.z = enu[2];
  } else {
    sol.pos_enu.x = sol.pos_enu.y = sol.pos_enu.z =
        std::numeric_limits<double>::quiet_NaN();
  }
  const double velocity[3] = {sol.vel_ecef.x, sol.vel_ecef.y, sol.vel_ecef.z};
  const double current_llh[3] = {sol.latitude * D2R, sol.longitude * D2R, sol.altitude};
  ecef2enu(current_llh, velocity, enu);
  sol.vel_enu.x = enu[0]; sol.vel_enu.y = enu[1]; sol.vel_enu.z = enu[2];
  return established;
}

inline void configureNmeaDate(rclcpp::Node& node, gnss_utils::NmeaParser& parser) {
  const auto date = node.declare_parameter<std::vector<int64_t>>("nmea.reference_date", std::vector<int64_t>{});
  if (date.empty()) return;
  if (date.size() != 3 || date[0] < 1980 || date[0] > 2099 || date[1] < 1 || date[1] > 12 ||
      date[2] < 1 || date[2] > 31 || !parser.setReferenceDate(date[0], date[1], date[2]))
    throw std::invalid_argument("nmea.reference_date must be [] or a valid UTC [year, month, day]");
}

// All receiver paths use the same bounded quiet period to assemble optional
// blocks of an epoch. Never flush on the first PVT merely because no covariance
// block has been seen yet. A different epoch still flushes immediately.
class EpochFlushPolicy {
 public:
  void configure(double quiet_s) {
    if (!std::isfinite(quiet_s) || quiet_s <= 0 || quiet_s > 5)
      throw std::invalid_argument("solution.assembly_timeout_s must be in (0, 5]");
    quiet_s_ = quiet_s;
  }
  void touch() { updated_ = std::chrono::steady_clock::now(); }
  bool due() const {
    return std::chrono::duration<double>(std::chrono::steady_clock::now()-updated_).count() >= quiet_s_;
  }
 private:
  double quiet_s_{0.05};
  std::chrono::steady_clock::time_point updated_{std::chrono::steady_clock::now()};
};

/// Per-constellation satellite counter, populated by `countSatellite`.
struct SatelliteCount {
  int gps{0};
  int glo{0};
  int gal{0};
  int qzs{0};
  int bds{0};
  int irn{0};
  int sbs{0};
  int unknown{0};
};

/// Classify an RTKLIB satellite number into the SatelliteCount bucket.
inline void countSatellite(int sat, SatelliteCount& c) {
  int prn = 0;
  switch (satsys(sat, &prn)) {
    case SYS_GPS: ++c.gps; break;
    case SYS_GLO: ++c.glo; break;
    case SYS_GAL: ++c.gal; break;
    case SYS_QZS: ++c.qzs; break;
    case SYS_CMP: ++c.bds; break;
    case SYS_IRN: ++c.irn; break;
    case SYS_SBS: ++c.sbs; break;
    default:      ++c.unknown; break;
  }
}

/// Append each channel with a supported satellite/signal identity. Numeric
/// zero is a measurement value, not evidence that a coded channel is empty.
inline void appendObservations(const obsd_t& obs,
                               std::vector<msg::GnssObservation>& observations,
                               const nav_t* nav = nullptr) {
  for (int freq = 0; freq < NFREQ + NEXOBS; ++freq) {
    auto value = gnss_utils::obsToMsg(obs, freq, nav);
    if (value.sat) observations.push_back(std::move(value));
  }
}

/// One explicit clock policy for all measurement topics. In GPS mode an epoch
/// without valid GNSS time is withheld, never relabelled with host time. This
/// permits cold-start recovery without switching the time base of a topic.
class TimeBaseLatch {
 public:
  std::optional<rclcpp::Time> stamp(bool want_gps, bool gps_valid, gtime_t t_gpst,
      const rclcpp::Time& host_now, const rclcpp::Logger& log, rclcpp::Clock& clock) {
    if (want_gps && gps_valid && !gnss_utils::isRosTimeRepresentable(t_gpst)) {
      RCLCPP_WARN_THROTTLE(log, clock, 10000, "GNSS epoch outside ROS Time range: withholding measurement");
      return std::nullopt;
    }
    return stampPrecomputed(want_gps, gps_valid,
        want_gps && gps_valid ? gnss_utils::gpstToUtcRosTime(t_gpst) : host_now,
        host_now, log, clock);
  }
  std::optional<rclcpp::Time> stampPrecomputed(bool want_gps, bool gps_valid,
      const rclcpp::Time& gps_utc, const rclcpp::Time& host_now,
      const rclcpp::Logger& log, rclcpp::Clock& clock) {
    latched_ = true;
    use_gps_ = want_gps;
    if (!want_gps) return host_now;
    if (gps_valid) return rclcpp::Time(gps_utc.nanoseconds(), host_now.get_clock_type());
    RCLCPP_WARN_THROTTLE(log, clock, 10000,
        "GNSS timestamp unavailable: withholding measurement in GPS mode");
    return std::nullopt;
  }
  bool latched() const { return latched_; }
  bool usingGpsTime() const { return use_gps_; }
 private:
  bool latched_{false};
  bool use_gps_{false};
};

/// Resolve the GPS week for a standalone time-of-week against a reference epoch.
///
/// UBX NAV-* / ESF-INS payloads carry an iTOW but no week number; `ref`
/// (typically `raw_.time`, which RTKLIB populates from RXM-RAWX / RXM-SFRBX)
/// supplies the week. When `tow_s` and `ref` straddle a GPS week boundary they
/// sit almost a full week apart, so a half-week (302400 s) threshold decides
/// which side of the boundary `tow_s` belongs to.
///
/// @param ref   reference epoch carrying a valid week (e.g. `raw_.time`)
/// @param tow_s standalone time-of-week [s] to date
/// @param week  out: the resolved GPS week; untouched when this returns false
/// @return false when `ref` carries no valid week (nothing decoded yet), in
///         which case the caller must fall back to the host clock. Also false
///         for a non-finite or out-of-range `tow_s`: dating those would place
///         the measurement an arbitrary distance from where it belongs, and a
///         silent bad week is worse than an honest "cannot date this".
inline bool resolveGpsWeekForTow(gtime_t ref, double tow_s, int& week) {
  int ref_week = 0;
  const double ref_tow = time2gpst(ref, &ref_week);
  if (ref_week <= 0) return false;
  if (!std::isfinite(tow_s) || tow_s < 0.0 || tow_s >= 604800.0) return false;

  if      (tow_s - ref_tow >  302400.0) ref_week -= 1;
  else if (tow_s - ref_tow < -302400.0) ref_week += 1;
  week = ref_week;
  return true;
}

/// Derive ECEF position from LLH and ECEF velocity from ENU velocity using the
/// solution's current LLH as the reference frame. Used by every binary PVT
/// decoder/driver path after primary fields are populated.
inline void finalizeBinarySolutionGeometry(msg::GnssSolution& s) {
  double llh[3] = {s.latitude * D2R, s.longitude * D2R, s.altitude};
  double ecef[3] = {0};
  pos2ecef(llh, ecef);
  s.pos_ecef.x = ecef[0];
  s.pos_ecef.y = ecef[1];
  s.pos_ecef.z = ecef[2];

  double vel_e[3] = {s.vel_enu.x, s.vel_enu.y, s.vel_enu.z};
  double vel_ec[3] = {0};
  enu2ecef(llh, vel_e, vel_ec);
  s.vel_ecef.x = vel_ec[0];
  s.vel_ecef.y = vel_ec[1];
  s.vel_ecef.z = vel_ec[2];
}

/// Mask userinfo before reporting a stream URI, including on open failures.
inline std::string redactStreamUri(std::string uri) {
  const auto scheme = uri.find("://");
  const size_t start = scheme == std::string::npos ? 0 : scheme + 3;
  // RTKLIB decodetcppath uses strrchr over the entire path.
  const auto at = uri.rfind('@');
  if (at != std::string::npos && at >= start) uri.replace(start, at - start, "***");
  return uri;
}

/// Write a command completely, or report failure within a bounded deadline.
/// The caller owns the stream; no concurrent writer may interleave commands.
template<class Write>
inline bool writeAllWith(Write write, const uint8_t* data, size_t size,
                     std::chrono::milliseconds timeout = std::chrono::milliseconds(3000)) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  size_t offset = 0;
  while (offset < size) {
    const int n = write(data + offset, size - offset);
    if (n < 0 || static_cast<size_t>(n) > size - offset) return false;
    offset += static_cast<size_t>(n);
    if (offset == size) return true;
    if (std::chrono::steady_clock::now() >= deadline) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return true;
}
inline bool writeAll(stream_t& stream, const uint8_t* data, size_t size,
                     std::chrono::milliseconds timeout = std::chrono::milliseconds(3000)) {
  return writeAllWith([&](const uint8_t* p, size_t n) {
    return strwrite(&stream, const_cast<uint8_t*>(p), static_cast<int>(n));
  }, data, size, timeout);
}


/// Open a scheme-prefixed RTKLIB stream path (`tcpcli://` `tcpsvr://`
/// `serial://` `ntrip://` `file://`) into `s`. A `/dev/` prefix after
/// `serial://` is stripped (RTKLIB prepends it itself on Linux). Returns false
/// on failure after logging; `what` names the stream in log lines.
inline bool openStreamPath(const rclcpp::Logger& log, stream_t& s,
                           const std::string& original, int mode,
                           const char* what) {
  struct Def { std::string_view prefix; int type; };
  static constexpr Def kDefs[] = {
    {"tcpcli://", STR_TCPCLI},
    {"tcpsvr://", STR_TCPSVR},
    {"serial://", STR_SERIAL},
    {"ntrip://",  STR_NTRIPCLI},
    {"file://",   STR_FILE},
  };

  std::string path = original;
  int stream_type = 0;
  bool matched = false;
  for (const auto& def : kDefs) {
    if (path.rfind(def.prefix, 0) == 0) {
      stream_type = def.type;
      path.erase(0, def.prefix.size());
      matched = true;
      break;
    }
  }
  if (!matched) {
    RCLCPP_ERROR(log, "Unsupported %s stream path: %s", what, redactStreamUri(original).c_str());
    return false;
  }
  if (stream_type == STR_SERIAL && path.rfind("/dev/", 0) == 0) {
    path.erase(0, 5);
  }

  if (path.empty() || path.size() >= MAXSTRPATH || path.find('\0') != std::string::npos ||
      path.find_first_of("\r\n") != std::string::npos) {
    RCLCPP_ERROR(log, "Invalid %s stream path: empty, too long or containing control characters", what);
    return false;
  }
  if ((stream_type == STR_TCPCLI || stream_type == STR_TCPSVR || stream_type == STR_NTRIPCLI) &&
      path.find('[') != std::string::npos) {
    RCLCPP_ERROR(log, "IPv6 stream addresses are unsupported; use an IPv4 endpoint");
    return false;
  }
  if (stream_type == STR_NTRIPCLI)
    RCLCPP_INFO(log, "NTRIP transport: IPv4, HTTP/1.0 request and ICY response; no TLS, v2/chunked decoding or automatic VRS GGA");

  strinit(&s);
  if (!stropen(&s, stream_type, mode, path.c_str())) {
    RCLCPP_ERROR(log, "Failed to open %s stream: %s", what, redactStreamUri(original).c_str());
    return false;
  }
  RCLCPP_INFO(log, "%s stream opened: %s", what, redactStreamUri(original).c_str());
  return true;
}

/// Lossless-by-default writer for non-blocking RTKLIB output streams.
///
/// `strwrite` only accepts what fits the OS buffer right now: serial returns
/// 0 on EAGAIN and w<n on a partial write (never negative on Linux), tcpcli
/// returns 0 while disconnected. Ignoring that return value silently drops
/// the remainder mid-RTCM-frame (CRC failure at the receiver). This writer
/// keeps the unaccepted remainder in a carry buffer and retries it on the
/// next call before any new data, preserving byte order and frame integrity.
///
/// `max_carry > 0` caps the carry for producers that cannot stop reading
/// their source (e.g. rtcm_decoder's relay output): a persistently stalled
/// output (relay target down for minutes, output link slower than the source)
/// drops the WHOLE carry with a WARN. Corrections that old are already
/// rejected by receivers via age-of-differential, and flushing a stale
/// backlog on recovery would only delay fresh corrections — this is still
/// strictly gentler than RTKLIB STRSVR, which drops immediately and silently.
struct CarriedStreamWriter {
  CarriedStreamWriter() = default;
  explicit CarriedStreamWriter(size_t cap) : max_carry(cap) {}

  size_t max_carry{0};  // 0 = uncapped (caller bounds it via backpressure)
  size_t bytes{0};      // total bytes actually accepted by the stream
  size_t dropped{0};    // total bytes discarded on carry overflow

  bool   hasCarry()  const { return !carry_.empty(); }
  size_t carrySize() const { return carry_.size(); }

  /// Flush the carry, then write buf[0..n); whatever the stream does not
  /// accept is carried over (appended after any still-pending carry, so byte
  /// order is preserved).
  void write(const rclcpp::Logger& log, rclcpp::Clock& clock, stream_t& out,
             const uint8_t* buf, int n) {
    if (!carry_.empty()) {
      const int w = strwrite(&out, carry_.data(), static_cast<int>(carry_.size()));
      if (w < 0) {
        RCLCPP_WARN_THROTTLE(log, clock, 5000, "RTCM relay strwrite error: %d", w);
      } else if (w > 0) {
        bytes += static_cast<size_t>(w);
        carry_.erase(carry_.begin(), carry_.begin() + w);
      }
    }
    if (n > 0 && buf != nullptr) {
      if (carry_.empty()) {
        int w = strwrite(&out, const_cast<uint8_t*>(buf), n);
        if (w < 0) {
          RCLCPP_WARN_THROTTLE(log, clock, 5000, "RTCM relay strwrite error: %d", w);
          w = 0;
        }
        bytes += static_cast<size_t>(w);
        if (w < n) carry_.assign(buf + w, buf + n);
      } else {
        carry_.insert(carry_.end(), buf, buf + n);
      }
    }
    if (max_carry > 0 && carry_.size() > max_carry) {
      dropped += carry_.size();
      RCLCPP_WARN_THROTTLE(log, clock, 5000,
        "RTCM relay: output stalled — dropping %zu stale buffered bytes "
        "(%zu dropped total). Relay target down or output link too slow?",
        carry_.size(), dropped);
      carry_.clear();
    }
  }

  /// Retry only the pending carry (no new data).
  void flush(const rclcpp::Logger& log, rclcpp::Clock& clock, stream_t& out) {
    write(log, clock, out, nullptr, 0);
  }

 private:
  std::vector<uint8_t> carry_;  // unaccepted remainder awaiting the output stream
};

// A failed/ambiguous transaction stops configuration: a late response cannot
// then acknowledge another command. Receiver response output must be enabled.
template<class Consume>
void configureAscii(stream_t& stream, wire::Protocol protocol,
                    const std::string& command, Consume consume) {
  uint8_t data[4096];
  // Drain stale replies before sending, preserving measurement input.
  for (int count = 0; count < 32; ++count) {
    const int n = strread(&stream, data, sizeof(data));
    if (n <= 0) break;
    for (int i = 0; i < n; ++i) consume(data[i]);
  }
  const auto line = command + "\r\n";
  if (!writeAll(stream, reinterpret_cast<const uint8_t*>(line.data()), line.size()))
    throw std::runtime_error("Receiver command write timed out: " + command);
  wire::CommandReply reply(protocol, command);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline) {
    const int n = strread(&stream, data, sizeof(data));
    int result = 0;
    for (int i = 0; i < n; ++i) { consume(data[i]); result = reply.feed(data[i]); }
    if (result < 0) throw std::runtime_error("Receiver rejected command: " + command);
    if (result > 0) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  throw std::runtime_error("Receiver command ACK timed out: " + command);
}

/// RTCM relay input for the decoder nodes (RTKLIB STRSVR-style): host a stream
/// (typically a `tcpsvr://` TCP server fed by rtcm_decoder_node's `rtcm_relay`
/// TCP client) and write the received correction bytes back down the
/// exclusively-owned receiver stream, so the receiver runs RTK on-chip even in
/// passive decoder operation. Same pattern as the drivers' `rtcm_relay`, but
/// no receiver command is ever sent — any per-receiver input setup (u-blox
/// inProtoMask, Septentrio setDataInOut, NovAtel INTERFACEMODE) must be done
/// once beforehand; see src/decoders/README.md.
struct RtcmRelayServer {
  stream_t in{};
  bool     enabled{false};

  /// Open the listen stream. An empty `listen_uri` keeps the relay disabled
  /// and returns true; false means the URI was set but could not be opened.
  bool open(const rclcpp::Logger& log, const std::string& listen_uri) {
    if (listen_uri.empty()) return true;
    if (!openStreamPath(log, in, listen_uri, STR_MODE_RW, "RTCM relay")) {
      return false;
    }
    enabled = true;
    RCLCPP_INFO(log, "RTCM relay enabled: '%s' -> receiver stream", redactStreamUri(listen_uri).c_str());
    return true;
  }

  /// Drain pending correction bytes and write them to `out` (the receiver
  /// stream). Returns immediately when the relay is disabled or idle.
  ///
  /// Partial writes are carried over by the CarriedStreamWriter and no more
  /// TCP data is read until the carry flushes — backpressure sits in the TCP
  /// socket instead of bytes being dropped, which bounds the carry to one
  /// read chunk (no cap needed).
  void drainTo(const rclcpp::Logger& log, rclcpp::Clock& clock, stream_t& out) {
    if (!enabled) return;
    const size_t before = writer_.bytes;

    writer_.flush(log, clock, out);
    if (!writer_.hasCarry()) {
      uint8_t buf[2048];
      int n;
      for (int batch = 0; batch < 16 && (n = strread(&in, buf, sizeof(buf))) > 0; ++batch) {
        writer_.write(log, clock, out, buf, n);
        if (writer_.hasCarry()) break;  // saturated — leave the rest in the TCP socket
      }
    }

    if (writer_.hasCarry()) {
      RCLCPP_WARN_THROTTLE(log, clock, 5000,
        "RTCM relay: receiver stream saturated, %zu bytes pending", writer_.carrySize());
    }
    if (writer_.bytes > before) {
      RCLCPP_INFO_THROTTLE(log, clock, 5000,
                           "RTCM relay: %zu bytes forwarded to receiver (total)",
                           writer_.bytes);
    }
  }

  void close() {
    if (enabled) strclose(&in);
    enabled = false;
  }

 private:
  CarriedStreamWriter writer_;
};

}  // namespace decoder_common
}  // namespace gnss_ros_standardization

#endif  // GNSS_ROS_STANDARDIZATION_DECODER_COMMON_HPP
