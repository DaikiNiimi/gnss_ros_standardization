// SPDX-License-Identifier: MIT
#ifndef GNSS_ROS_STANDARDIZATION_EPHEMERIS_STORE_HPP
#define GNSS_ROS_STANDARDIZATION_EPHEMERIS_STORE_HPP

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <unordered_map>

#include <rclcpp/time.hpp>

#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/msg/glonass_ephemeris.hpp"
#include "gnss_ros_standardization/msg/gnss_ephemerides.hpp"
#include "gnss_ros_standardization/msg/gnss_ephemeris.hpp"

extern "C" {
#include "rtklib.h"
}

namespace gnss_utils {

// Unified ephemeris store shared by all publishers (drivers, RTCM decoder) and
// all consumers (SPP, RTK, visualizer). Keyed by (sat, code, iode, iodc, toe)
// for GNSS and (sat, iode, toe, tof) for GLO so that distinct broadcast frames
// (e.g. BeiDou AODE counter increments, GLONASS rebroadcasts with same toe but
// different tof) are preserved as separate entries — matching convbin's
// per-reception RINEX output. RTKLIB's seleph() picks the toe-closest entry
// downstream, so SPP/RTK semantics are unchanged.
//
// Publish semantics: emit a snapshot whenever ingest() reports a change, OR
// whenever heartbeatDue() returns true. Both publisher restart and late-join
// recovery are bounded by snapshot_period_s.
class EphemerisStore {
 public:
  EphemerisStore() = default;

  void setSnapshotPeriod(double seconds) { snapshot_period_s_ = seconds; }
  void setMaxAge(double seconds) { max_age_s_ = seconds; }

  // Insert/update an ephemeris. The stored entry is always refreshed to the
  // latest received copy and the call always returns true so consumers see
  // ttr / clock / health refinements immediately. Upstream RTKLIB decoders
  // already filter bit-identical re-broadcasts (with -EPHALL off), so this
  // does not produce extra publishes in normal operation. Downstream can
  // throttle via ROS QoS if needed.
  bool ingestEph(const eph_t& e_in) {
    if (e_in.sat <= 0 || e_in.sat > MAXSAT) return false;
    int prn = 0;
    const int sys = satsys(e_in.sat, &prn);
    if (sys == SYS_GLO) return false;
    eph_t e = e_in;
    if (sys == SYS_GAL) e.code = gnss_utils::canonicalGalCode(e.code);
    const KKey k = makeKKey(e);
    // Within the same key, allow refresh (ttr / clock / health refinements
    // arrive on rebroadcast). Different (iode/iodc/toe) lands in a separate
    // slot, so the toe regression check is per-key and effectively a no-op
    // — kept here to preserve refresh-only semantics when receivers replay
    // bit-identical frames out of order.
    auto it = kepler_.find(k);
    if (it != kepler_.end() && timediff(e.toe, it->second.toe) < 0.0) {
      return false;
    }
    kepler_[k] = e;
    return true;
  }

  bool ingestGeph(const geph_t& g) {
    if (g.sat <= 0 || g.sat > MAXSAT) return false;
    const GKey k = makeGKey(g);
    auto it = glonass_.find(k);
    if (it != glonass_.end() && timediff(g.toe, it->second.toe) < 0.0) {
      return false;
    }
    glonass_[k] = g;
    return true;
  }

  // True if heartbeat publish is due.
  bool heartbeatDue(const rclcpp::Time& now) const {
    if (!last_publish_initialized_) return true;
    return (now - last_publish_).seconds() >= snapshot_period_s_;
  }

  // Build a snapshot message containing every currently-valid ephemeris.
  // Updates internal last_publish_ time.
  gnss_ros_standardization::msg::GnssEphemerides buildSnapshot(const rclcpp::Time& now) {
    gnss_ros_standardization::msg::GnssEphemerides msg;
    msg.header.stamp = now;
    msg.gnss_ephemeris.reserve(kepler_.size());
    msg.glonass_ephemeris.reserve(glonass_.size());

    const gtime_t now_gt = nowGtime(now);

    for (const auto& kv : kepler_) {
      const eph_t& e = kv.second;
      if (max_age_s_ > 0.0 && tooOld(e.toe, now_gt)) continue;
      msg.gnss_ephemeris.push_back(gnss_utils::ephToMsg(e));
    }
    for (const auto& kv : glonass_) {
      const geph_t& g = kv.second;
      if (max_age_s_ > 0.0 && tooOld(g.toe, now_gt)) continue;
      msg.glonass_ephemeris.push_back(gnss_utils::gephToMsg(g));
    }

    last_publish_ = now;
    last_publish_initialized_ = true;
    return msg;
  }

  // Copy all held ephemerides into an RTKLIB nav_t, replacing same-(sat,code)
  // entries and growing the buffers as needed. Used by consumers (SPP/RTK).
  void applyToNav(nav_t& nav) const {
    for (const auto& kv : kepler_) upsertEphInNav(nav, kv.second);
    for (const auto& kv : glonass_) upsertGephInNav(nav, kv.second);
  }

  size_t keplerCount() const { return kepler_.size(); }
  size_t glonassCount() const { return glonass_.size(); }

 private:
  // Composite key matching RTKLIB's most-strict dedup (decode_galrawinav etc.):
  // distinct (iode, iodc, toe) within the same (sat, code) are preserved as
  // separate entries — required to keep BeiDou AODE history and out-of-order
  // raw-path receptions.
  struct KKey {
    int sat;
    int code;
    int iode;
    int iodc;
    int64_t toe_s;
    bool operator==(const KKey& o) const {
      return sat == o.sat && code == o.code && iode == o.iode &&
             iodc == o.iodc && toe_s == o.toe_s;
    }
  };
  struct KKeyHash {
    size_t operator()(const KKey& k) const {
      size_t h = static_cast<size_t>(k.sat) << 16;
      h ^= static_cast<size_t>(k.code) << 24;
      h ^= static_cast<size_t>(k.iode);
      h ^= static_cast<size_t>(k.iodc) << 8;
      h ^= static_cast<size_t>(k.toe_s) << 1;
      return h;
    }
  };
  // GLONASS: tof must also distinguish entries because RTKLIB's decode_glostr
  // dedup is (iode, svh, toe) only — same-toe rebroadcasts with different tof
  // reach ingest and need to be retained for convbin-compatible RINEX output.
  struct GKey {
    int sat;
    int iode;
    int64_t toe_s;
    int64_t tof_s;
    bool operator==(const GKey& o) const {
      return sat == o.sat && iode == o.iode &&
             toe_s == o.toe_s && tof_s == o.tof_s;
    }
  };
  struct GKeyHash {
    size_t operator()(const GKey& k) const {
      size_t h = static_cast<size_t>(k.sat) << 24;
      h ^= static_cast<size_t>(k.iode) << 16;
      h ^= static_cast<size_t>(k.toe_s);
      h ^= static_cast<size_t>(k.tof_s) << 1;
      return h;
    }
  };

  static KKey makeKKey(const eph_t& e) {
    return KKey{e.sat, static_cast<int>(e.code), e.iode, e.iodc,
                static_cast<int64_t>(e.toe.time)};
  }
  static GKey makeGKey(const geph_t& g) {
    return GKey{g.sat, g.iode, static_cast<int64_t>(g.toe.time),
                static_cast<int64_t>(g.tof.time)};
  }

  static gtime_t nowGtime(const rclcpp::Time& now) {
    const double sec = static_cast<double>(now.seconds());
    gtime_t t{};
    t.time = static_cast<time_t>(sec);
    t.sec = sec - static_cast<double>(t.time);
    // Convert from Unix time to GPST. RTKLIB stores gtime_t as UTC/Unix internally,
    // and timediff() works on the same scale, so direct comparison with eph.toe
    // (also gtime_t in GPST) is safe modulo the GPS-UTC offset. For max_age
    // comparison (default 7200s), this offset is negligible.
    return t;
  }

  bool tooOld(const gtime_t& toe, const gtime_t& now_gt) const {
    return std::fabs(timediff(now_gt, toe)) > max_age_s_;
  }

  // upsert conditions must mirror KKey/GKey, otherwise distinct store entries
  // collapse into one slot in nav.eph[] and the per-reception preservation
  // intended by the store key is lost downstream.
  static void upsertEphInNav(nav_t& nav, const eph_t& e) {
    if (e.sat <= 0 || e.sat > MAXSAT) return;
    for (int i = 0; i < nav.n; ++i) {
      if (nav.eph[i].sat == e.sat &&
          nav.eph[i].code == e.code &&
          nav.eph[i].iode == e.iode &&
          nav.eph[i].iodc == e.iodc &&
          timediff(nav.eph[i].toe, e.toe) == 0.0) {
        nav.eph[i] = e;
        return;
      }
    }
    if (nav.n >= nav.nmax) {
      const int newmax = nav.nmax == 0 ? 8 : nav.nmax * 2;
      auto* p = static_cast<eph_t*>(std::realloc(nav.eph, sizeof(eph_t) * newmax));
      if (!p) return;
      nav.eph = p;
      nav.nmax = newmax;
    }
    nav.eph[nav.n++] = e;
  }

  static void upsertGephInNav(nav_t& nav, const geph_t& g) {
    if (g.sat <= 0 || g.sat > MAXSAT) return;
    for (int i = 0; i < nav.ng; ++i) {
      if (nav.geph[i].sat == g.sat &&
          nav.geph[i].iode == g.iode &&
          timediff(nav.geph[i].toe, g.toe) == 0.0 &&
          timediff(nav.geph[i].tof, g.tof) == 0.0) {
        nav.geph[i] = g;
        return;
      }
    }
    if (nav.ng >= nav.ngmax) {
      const int newmax = nav.ngmax == 0 ? 8 : nav.ngmax * 2;
      auto* p = static_cast<geph_t*>(std::realloc(nav.geph, sizeof(geph_t) * newmax));
      if (!p) return;
      nav.geph = p;
      nav.ngmax = newmax;
    }
    nav.geph[nav.ng++] = g;
  }

  std::unordered_map<KKey, eph_t, KKeyHash> kepler_;
  std::unordered_map<GKey, geph_t, GKeyHash> glonass_;

  double snapshot_period_s_{30.0};
  // 0.0 disables aging; keep every received eph (default). Set positive to
  // drop entries whose toe is older than this many seconds.
  double max_age_s_{0.0};
  rclcpp::Time last_publish_{0, 0, RCL_ROS_TIME};
  bool last_publish_initialized_{false};
};

}  // namespace gnss_utils

#endif  // GNSS_ROS_STANDARDIZATION_EPHEMERIS_STORE_HPP