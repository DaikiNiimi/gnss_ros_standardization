// SPDX-License-Identifier: MIT
#ifndef GNSS_ROS_STANDARDIZATION_DECODER_COMMON_HPP
#define GNSS_ROS_STANDARDIZATION_DECODER_COMMON_HPP

/// @file decoder_common.hpp
/// @brief Shared helpers used by the per-receiver decoder and driver nodes.
///
/// Receiver-agnostic glue logic (observation publication helpers, satellite
/// counting, ECEF↔LLH/ENU finalization, etc.) lives here so all
/// decoder/driver pairs share one implementation. Receiver-specific framing
/// and protocol parsing remain in `*_protocol.hpp`.

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

extern "C" {
#include "rtklib.h"
}

namespace gnss_ros_standardization {
namespace decoder_common {

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

/// Append every non-empty (P / L / D / SNR) frequency channel of `obs` into
/// `observations` as a `GnssObservation` message.
inline void appendObservations(const obsd_t& obs,
                               std::vector<msg::GnssObservation>& observations) {
  for (int freq = 0; freq < NFREQ + NEXOBS; ++freq) {
    if (obs.P[freq] == 0.0 && obs.L[freq] == 0.0 &&
        obs.D[freq] == 0.0 && obs.SNR[freq] == 0) continue;
    observations.push_back(gnss_utils::obsToMsg(obs, freq));
  }
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
    RCLCPP_ERROR(log, "Unsupported %s stream path: %s", what, original.c_str());
    return false;
  }
  if (stream_type == STR_SERIAL && path.rfind("/dev/", 0) == 0) {
    path.erase(0, 5);
  }

  strinit(&s);
  if (!stropen(&s, stream_type, mode, path.c_str())) {
    RCLCPP_ERROR(log, "Failed to open %s stream: %s", what, original.c_str());
    return false;
  }
  RCLCPP_INFO(log, "%s stream opened: %s", what, original.c_str());
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
    RCLCPP_INFO(log, "RTCM relay enabled: '%s' -> receiver stream", listen_uri.c_str());
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
      while ((n = strread(&in, buf, sizeof(buf))) > 0) {
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
