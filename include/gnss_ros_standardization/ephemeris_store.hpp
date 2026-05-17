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
// all consumers (SPP, RTK, visualizer). Holds the latest valid ephemeris for
// every (sat, code) pair and provides snapshot publishing with heartbeat.
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
    const KKey k{e.sat, static_cast<int>(e.code)};
    auto it = kepler_.find(k);
    // Reject toe regression (multi-source / out-of-order delivery safety).
    // Same toe still passes through so ttr / clock / health refinements
    // within the same IODE are captured.
    if (it != kepler_.end() && timediff(e.toe, it->second.toe) < 0.0) {
      return false;
    }
    kepler_[k] = e;
    return true;
  }

  bool ingestGeph(const geph_t& g) {
    if (g.sat <= 0 || g.sat > MAXSAT) return false;
    auto it = glonass_.find(g.sat);
    if (it != glonass_.end() && timediff(g.toe, it->second.toe) < 0.0) {
      return false;
    }
    glonass_[g.sat] = g;
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
  struct KKey {
    int sat;
    int code;
    bool operator==(const KKey& o) const { return sat == o.sat && code == o.code; }
  };
  struct KKeyHash {
    size_t operator()(const KKey& k) const {
      return (static_cast<size_t>(k.sat) << 16) ^ static_cast<size_t>(k.code);
    }
  };

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

  static void upsertEphInNav(nav_t& nav, const eph_t& e) {
    if (e.sat <= 0 || e.sat > MAXSAT) return;
    for (int i = 0; i < nav.n; ++i) {
      if (nav.eph[i].sat == e.sat && nav.eph[i].code == e.code) {
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
      if (nav.geph[i].sat == g.sat) {
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
  std::unordered_map<int, geph_t> glonass_;

  double snapshot_period_s_{30.0};
  // 0.0 disables aging; keep every received eph (default). Set positive to
  // drop entries whose toe is older than this many seconds.
  double max_age_s_{0.0};
  rclcpp::Time last_publish_{0, 0, RCL_ROS_TIME};
  bool last_publish_initialized_{false};
};

}  // namespace gnss_utils

#endif  // GNSS_ROS_STANDARDIZATION_EPHEMERIS_STORE_HPP
