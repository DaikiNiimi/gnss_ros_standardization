#ifndef GNSS_ROS_STANDARDIZATION_EPHEMERIS_STORE_HPP
#define GNSS_ROS_STANDARDIZATION_EPHEMERIS_STORE_HPP

#include <algorithm>
#include <array>
#include <map>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <unordered_map>
#include <unordered_set>
#include <rclcpp/logging.hpp>
#include "gnss_ros_standardization/rtklib_compat.hpp"

#include <rclcpp/time.hpp>

#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/msg/glonass_ephemeris.hpp"
#include "gnss_ros_standardization/msg/gnss_ephemerides.hpp"
#include "gnss_ros_standardization/msg/gnss_ephemeris.hpp"

#include "gnss_ros_standardization/rtklib_compat.hpp"

namespace gnss_utils {

// Unified ephemeris store shared by all publishers (drivers, RTCM decoder) and
// all consumers (SPP, RTK, visualizer). Keyed by (sat, code, iode, iodc, toe)
// for GNSS and (sat, iode, toe, tof) for GLO so that distinct broadcast frames
// (e.g. BeiDou AODE counter increments, GLONASS rebroadcasts with same toe but
// different tof) are preserved as separate entries — matching convbin's
// per-reception RINEX output. RTKLIB's seleph() resolves equal-toe ties by
// array order. Galileo positioning uses explicit current/previous family slots;
// raw history and non-Galileo reception order are preserved.
//
// Publish semantics: emit a snapshot whenever ingest() reports a change, OR
// whenever heartbeatDue() returns true. Both publisher restart and late-join
// recovery are bounded by snapshot_period_s.
class EphemerisStore {
 public:
  EphemerisStore() = default;

  // Retire one receiver session while preserving configured retention limits.
  void clear() {
    kepler_.clear(); positioning_kepler_.clear(); glonass_.clear();
    positioning_order_.clear(); glonass_order_.clear();
    ion_ = Ionosphere{};
    // The bounded warning vocabulary is process/store lifetime state. Clearing
    // it on every receiver seek/reconnect can turn malformed nav into log spam.
    last_snapshot_rejected_=0; last_publish_initialized_=false;
  }

  void setSnapshotPeriod(double seconds) { snapshot_period_s_ = seconds; }
  void setMaxAge(double seconds) { max_age_s_ = seconds; }
  // Zero explicitly requests archival/unbounded retention.
  void setHistoryPerSatellite(size_t count) { history_per_satellite_ = count; }

  // True only if retained backend contents changed. Publication separately
  // rejects records that cannot be represented by the fixed ROS schema. Identical
  // rebroadcasts do not trigger a snapshot; health/clock refinements do.
  bool ingestEph(const eph_t& e_in) {
    if (satsys(e_in.sat, nullptr)==SYS_GAL)
      RCLCPP_DEBUG(rclcpp::get_logger("ephemeris_store"),
          "Galileo input sat=%d code=%d iode=%d toe=%.3f ttr=%.3f f0=%.17g bgd=%.17g/%.17g",
          e_in.sat,e_in.code,e_in.iode,time2gpst(e_in.toe,nullptr),
          time2gpst(e_in.ttr,nullptr),e_in.f0,e_in.tgd[0],e_in.tgd[1]);
    if (e_in.sat <= 0 || e_in.sat > MAXSAT) return false;
    int prn = 0;
    const int sys = satsys(e_in.sat, &prn);
    if (sys == SYS_GLO) return false;
    eph_t e = e_in;
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
    // Keep the solver projection in reception order, including refreshes of
    // distinct raw codes that describe the same I/NAV source. The raw map
    // remains authoritative for publication and field preservation.
    auto solver_e=e;
    if (sys==SYS_GAL) solver_e.code=canonicalGalCode(e.code);
    const auto solver_key=makeKKey(solver_e);
    if (!positioning_kepler_.count(solver_key)) positioning_order_.push_back(solver_key);
    positioning_kepler_[solver_key]=solver_e;
    if (it != kepler_.end() && gnss_utils::navigationRecordKey(it->second) == gnss_utils::navigationRecordKey(e)) return false;
    kepler_[k] = e;
    trim(kepler_, e.sat);
    std::unordered_set<KKey,KKeyHash> retained;
    for (const auto& kv : kepler_) if (kv.second.sat==e.sat) {
      auto value=kv.second;
      if (sys==SYS_GAL) value.code=canonicalGalCode(value.code);
      retained.insert(makeKKey(value));
    }
    for (auto it=positioning_kepler_.begin();it!=positioning_kepler_.end();) {
      if (it->second.sat==e.sat && !retained.count(it->first)) it=positioning_kepler_.erase(it);
      else ++it;
    }
    positioning_order_.erase(std::remove_if(positioning_order_.begin(),positioning_order_.end(),
        [&](const KKey& key){return !positioning_kepler_.count(key);}),positioning_order_.end());
    return kepler_.find(k) != kepler_.end();
  }

  bool ingestGeph(const geph_t& g) {
    if (g.sat <= 0 || g.sat > MAXSAT || satsys(g.sat, nullptr) != SYS_GLO) return false;
    const GKey k = makeGKey(g);
    auto it = glonass_.find(k);
    if (it != glonass_.end() && timediff(g.toe, it->second.toe) < 0.0) {
      return false;
    }
    if (it != glonass_.end() && gnss_utils::navigationRecordKey(it->second) == gnss_utils::navigationRecordKey(g)) return false;
    if (it == glonass_.end()) glonass_order_.push_back(k);
    glonass_[k] = g;
    trim(glonass_, g.sat);
    glonass_order_.erase(std::remove_if(glonass_order_.begin(),glonass_order_.end(),
        [&](const GKey& key){return !glonass_.count(key);}),glonass_order_.end());
    return glonass_.find(k) != glonass_.end();
  }

  // Broadcast ionosphere parameters (RTKLIB nav_t arrays). An all-zero or
  // non-finite set means "not received" and never replaces a received one.
  // True when a retained set changed.
  bool ingestIon(const nav_t& nav) {
    bool changed = false;
    changed = updateIon(ion_.gps, nav.ion_gps) || changed;
    changed = updateIon(ion_.gal, nav.ion_gal) || changed;
    changed = updateIon(ion_.qzs, nav.ion_qzs) || changed;
    changed = updateIon(ion_.bds, nav.ion_cmp) || changed;
    changed = updateIon(ion_.irn, nav.ion_irn) || changed;
    return changed;
  }
  bool ingestIon(const gnss_ros_standardization::msg::GnssEphemerides& m) {
    bool changed = false;
    changed = updateIon(ion_.gps, m.ion_gps.data()) || changed;
    changed = updateIon(ion_.gal, m.ion_gal.data()) || changed;
    changed = updateIon(ion_.qzs, m.ion_qzs.data()) || changed;
    changed = updateIon(ion_.bds, m.ion_bds.data()) || changed;
    changed = updateIon(ion_.irn, m.ion_irn.data()) || changed;
    return changed;
  }

  // True if heartbeat publish is due.
  bool heartbeatDue(const rclcpp::Time& now) const {
    if (!last_publish_initialized_) return true;
    return now.get_clock_type() != last_publish_.get_clock_type() ||
           now < last_publish_ || (now - last_publish_).seconds() >= snapshot_period_s_;
  }

  // Build a snapshot message containing every currently-valid ephemeris.
  // Updates internal last_publish_ time.
  gnss_ros_standardization::msg::GnssEphemerides buildSnapshot(const rclcpp::Time& now) {
    gnss_ros_standardization::msg::GnssEphemerides msg;
    msg.header.stamp = now;
    last_snapshot_rejected_ = 0;
    msg.gnss_ephemeris.reserve(kepler_.size());
    msg.glonass_ephemeris.reserve(glonass_.size());

    const gtime_t now_gt = nowGtime(now);

    for (const auto& kv : kepler_) {
      const eph_t& e = kv.second;
      if (max_age_s_ > 0.0 && tooOld(e.toe, now_gt)) continue;
      if (const char* reason = gnss_utils::ephemerisMessageError(e)) {
        reportRejected(e.sat, reason);
        continue;
      }
      msg.gnss_ephemeris.push_back(gnss_utils::ephToMsg(e));
    }
    for (const auto& kv : glonass_) {
      const geph_t& g = kv.second;
      if (max_age_s_ > 0.0 && tooOld(g.toe, now_gt)) continue;
      if (const char* reason = gnss_utils::ephemerisMessageError(g)) {
        reportRejected(g.sat, reason);
        continue;
      }
      msg.glonass_ephemeris.push_back(gnss_utils::gephToMsg(g));
    }

    std::copy(ion_.gps.begin(), ion_.gps.end(), msg.ion_gps.begin());
    std::copy(ion_.gal.begin(), ion_.gal.end(), msg.ion_gal.begin());
    std::copy(ion_.qzs.begin(), ion_.qzs.end(), msg.ion_qzs.begin());
    std::copy(ion_.bds.begin(), ion_.bds.end(), msg.ion_bds.begin());
    std::copy(ion_.irn.begin(), ion_.irn.end(), msg.ion_irn.begin());

    last_publish_ = now;
    last_publish_initialized_ = true;
    return msg;
  }

  // Copy all held ephemerides into an RTKLIB nav_t, replacing same-(sat,code)
  // entries and growing the buffers as needed. Used by consumers (SPP/RTK).
  // for_positioning excludes nonzero/non-finite rates unsupported by eph2pos;
  // the default backend-copy path and ROS snapshots retain all fields.
  void applyToNav(nav_t& nav, bool for_positioning = false) const {
    copyIon(ion_.gps, nav.ion_gps);
    copyIon(ion_.gal, nav.ion_gal);
    copyIon(ion_.qzs, nav.ion_qzs);
    copyIon(ion_.bds, nav.ion_cmp);
    copyIon(ion_.irn, nav.ion_irn);
    // Retire evicted entries without reordering survivors. A full rebuild
    // from unordered_map changes RTKLIB's equal-toe tie winner (notably the
    // Galileo I/NAV versus F/NAV clock), even when no record changed.
    const auto& source=for_positioning ? positioning_kepler_ : kepler_;
    std::unordered_set<KKey,KKeyHash> active_eph;
    for (const auto& kv : source) {
      auto e=kv.second;
      if (for_positioning && !supportedByLegacyOrbitModel(e)) continue;
      if (for_positioning && satsys(e.sat,nullptr)==SYS_GAL) e.code=canonicalGalCode(e.code);
      active_eph.insert(makeKKey(e));
    }
    int count=0;
    for (int i=0;i<nav.n;++i)
      if (active_eph.count(makeKKey(nav.eph[i])) &&
          !(for_positioning && satsys(nav.eph[i].sat,nullptr)==SYS_GAL))
        nav.eph[count++]=nav.eph[i];
    nav.n=count;
    count=0;
    for (int i=0;i<nav.ng;++i)
      if (glonass_.count(makeGKey(nav.geph[i]))) nav.geph[count++]=nav.geph[i];
    nav.ng=count;
    auto append = [&](const eph_t& e) {
      if (for_positioning && !supportedByLegacyOrbitModel(e)) {
        RCLCPP_WARN_ONCE(rclcpp::get_logger("ephemeris_store"),
                        "Positioning excludes nonzero/non-finite Adot/ndot: pinned RTKLIB orbit model lacks support; ROS navigation retains the data");
        return;
      }
      upsertEphInNav(nav, e);
    };
    if (for_positioning) {
      // New entries also need a defined order. Map iteration would let an
      // unrelated satellite (even one with no observations) flip Galileo's
      // equal-toe I/NAV/F/NAV winner when both arrive in one snapshot.
      // First-seen source order is retained across refreshes and rehashes.
      for (const auto& key : positioning_order_)
        if (satsys(key.sat,nullptr)!=SYS_GAL) append(positioning_kepler_.at(key));
      for (const auto& e : galileoProjection()) append(e);
      for (const auto& key : glonass_order_) upsertGephInNav(nav,glonass_.at(key));
    } else {
      for (const auto& kv : source) append(kv.second);
      for (const auto& kv : glonass_) upsertGephInNav(nav,kv.second);
    }
    if (for_positioning) for (int j=0;j<nav.n;++j) {
      const auto& e=nav.eph[j];
      if (satsys(e.sat,nullptr)==SYS_GAL)
        RCLCPP_DEBUG(rclcpp::get_logger("ephemeris_store"),
            "Galileo projection index=%d sat=%d code=%d iode=%d toe=%.3f ttr=%.3f f0=%.17g bgd=%.17g/%.17g",
            j,e.sat,e.code,e.iode,time2gpst(e.toe,nullptr),time2gpst(e.ttr,nullptr),e.f0,e.tgd[0],e.tgd[1]);
    }
  }

  size_t keplerCount() const { return kepler_.size(); }
  size_t glonassCount() const { return glonass_.size(); }

  // Count records omitted from the most recent snapshot (not received frames).
  size_t lastSnapshotRejected() const { return last_snapshot_rejected_; }

 private:
  struct Ionosphere {
    std::array<double, 8> gps{}, qzs{}, bds{}, irn{};
    std::array<double, 4> gal{};
  };
  template <std::size_t N>
  static bool updateIon(std::array<double, N>& held, const double* in) {
    bool any = false;
    for (std::size_t i = 0; i < N; ++i) {
      if (!std::isfinite(in[i])) return false;
      any = any || in[i] != 0.0;
    }
    if (!any || std::equal(held.begin(), held.end(), in)) return false;
    std::copy(in, in + N, held.begin());
    return true;
  }
  template <std::size_t N>
  static void copyIon(const std::array<double, N>& held, double* out) {
    if (std::any_of(held.begin(), held.end(), [](double v) { return v != 0.0; }))
      std::copy(held.begin(), held.end(), out);
  }
  Ionosphere ion_{};

  std::vector<eph_t> galileoProjection() const {
    // Rebuild from raw retained contents, not the last-arriving canonical
    // alias: 513/516 describe the same I/NAV source but can carry refinements.
    std::vector<eph_t> input;
    for (const auto& [key,e] : kepler_) {
      if (satsys(e.sat,nullptr)!=SYS_GAL || !supportedByLegacyOrbitModel(e)) continue;
      auto canonical=e; canonical.code=canonicalGalCode(e.code);
      input.push_back(canonical);
    }
    std::sort(input.begin(),input.end(),[](const eph_t& a,const eph_t& b) {
      for (const auto delta : {timediff(a.ttr,b.ttr),timediff(a.toe,b.toe),timediff(a.toc,b.toc)})
        if (delta!=0) return delta<0;
      return navigationRecordKey(a)<navigationRecordKey(b);
    });
    struct Slots { eph_t current[2]{}, previous[2]{}; };
    std::map<int,Slots> satellites;
    for (const auto& e : input) {
      const int family=(e.code&(1<<9)) ? 0 : 1;
      auto& slots=satellites[e.sat];
      auto& current=slots.current[family]; auto& previous=slots.previous[family];
      if (current.sat && makeKKey(current)==makeKKey(e)) {
        current=e; // Content refinement, not a new generation.
      } else if (!current.sat ||
          (e.iode!=previous.iode && e.iode!=current.iode) ||
          (timediff(e.toe,previous.toe)!=0 && timediff(e.toe,current.toe)!=0) ||
          (timediff(e.toc,previous.toc)!=0 && timediff(e.toc,current.toc)!=0)) {
        previous=current; current=e; // Pinned rtksvr update_eph promotion rule.
      } else if (previous.sat && makeKKey(previous)==makeKKey(e)) previous=e;
    }
    std::vector<eph_t> result;
    for (const auto& [sat,slots] : satellites)
      for (const auto* e : {&slots.current[0],&slots.current[1],&slots.previous[0],&slots.previous[1]})
        if (e->sat) result.push_back(*e);
    return result;
  }

  void reportRejected(int sat, const char* reason) {
    ++last_snapshot_rejected_;
    // Bounded by the fixed reason vocabulary, not by satellites/epochs.
    if (reported_losses_.insert(reason).second) {
      RCLCPP_WARN(rclcpp::get_logger("ephemeris_store"),
                  "Ephemeris omitted from ROS snapshot (%s): %s; observations continue; retain original receiver data",
                  gnss_utils::satId(sat).c_str(), reason);
    }
  }
  std::unordered_set<std::string> reported_losses_;
  size_t last_snapshot_rejected_{0};
  template<class Map> void trim(Map& map, int sat) {
    if (history_per_satellite_ == 0) return;
    size_t count = 0;
    for (const auto& kv : map) if (kv.second.sat == sat) ++count;
    while (count > history_per_satellite_) {
      auto oldest = map.end();
      for (auto it = map.begin(); it != map.end(); ++it) {
        if (it->second.sat != sat) continue;
        if (oldest == map.end() || timediff(it->second.toe, oldest->second.toe) < 0)
          oldest = it;
      }
      if (oldest == map.end()) break;
      map.erase(oldest); --count;
    }
  }
  size_t history_per_satellite_{8};
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
    // ROS header time is UTC; broadcast toe is GPST.
    return utc2gpst(t);
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
  std::unordered_map<KKey, eph_t, KKeyHash> positioning_kepler_;
  std::unordered_map<GKey, geph_t, GKeyHash> glonass_;
  std::vector<KKey> positioning_order_;
  std::vector<GKey> glonass_order_;

  double snapshot_period_s_{30.0};
  // 0.0 disables aging; keep every received eph (default). Set positive to
  // drop entries whose toe is older than this many seconds.
  double max_age_s_{0.0};
  rclcpp::Time last_publish_{0, 0, RCL_ROS_TIME};
  bool last_publish_initialized_{false};
};

}  // namespace gnss_utils


// ---- navigation rinex validation ----
#include "gnss_ros_standardization/gnss_utils.hpp"

namespace gnss_utils {
// RINEX 3 retains navigation values to its decimal precision, not binary/CDR
// identity. Keep original receiver logs for information outside these fields.
inline const char* rinexNavigationError(const gnss_ros_standardization::msg::GnssEphemeris&,
                                      const eph_t& e, int version) {
  if (const char* error = ephemerisMessageError(e)) return error;
  if (!supportedByLegacyOrbitModel(e)) return "RINEX 3 export cannot preserve Adot/ndot";
  const int sys = satsys(e.sat, nullptr);
  if ((sys == SYS_IRN && version < 303) || ((sys == SYS_CMP || sys == SYS_QZS) && version < 302))
    return "Selected RINEX version cannot represent this constellation";
  const int delays = (sys == SYS_GAL || sys == SYS_CMP) ? 2 : 1;
  for (int i = delays; i < 6; ++i)
    if (e.tgd[i] != 0) return "RINEX 3 cannot preserve the extra tgd components";
  // D1/D2 decoders use flag=1 (IGSO/MEO) or 2 (GEO), not an extra
  // broadcast field. RINEX 3 carries the orbit, but not this backend tag.
  const bool bds_tag = sys == SYS_CMP && (e.flag == 1 || e.flag == 2);
  if (sys != SYS_GPS && sys != SYS_QZS && ((!bds_tag && e.flag != 0) || e.fit != 0))
    return "RINEX 3 cannot preserve flag/fit for this constellation";
  if ((sys == SYS_GAL || sys == SYS_IRN) && e.iodc != 0 && e.iodc != e.iode)
    return "RINEX 3 cannot preserve independent iodc for this constellation";
  if ((sys == SYS_CMP || sys == SYS_IRN) && e.code != 0)
    return "RINEX 3 reader cannot preserve code for this constellation";
  if (sys == SYS_QZS && e.fit != 2 && e.fit != 4)
    return "RINEX 3 QZSS fit supports only the 2-hour/greater-than-2-hour categories (restored as 2/4)";
  if ((sys == SYS_GAL && !((e.sva >= 0 && e.sva <= 125) || e.sva == 255)) ||
      (sys != SYS_GAL && (e.sva < 0 || e.sva > 15)))
    return "RINEX 3 cannot preserve this accuracy index";
  if (e.toc.sec != 0) return "RINEX 3 navigation epoch cannot preserve fractional clock-reference seconds";
  const double toe = sys == SYS_CMP ? time2bdt(gpst2bdt(e.toe), nullptr) : time2gpst(e.toe, nullptr);
  if (std::abs(toe-e.toes) > 1e-6) return "RINEX 3 cannot preserve inconsistent toe/toes";
  // Reader resolves ttr relative to toc, whereas ROS resolves relative to toe.
  if (std::abs(timediff(e.ttr, e.toc)) > 302400)
    return "RINEX 3 reader would resolve transmission time into a different week";
  return nullptr;
}
inline const char* rinexNavigationError(const gnss_ros_standardization::msg::GlonassEphemeris&,
                                      const geph_t& g, int version) {
  if (const char* error = ephemerisMessageError(g)) return error;
  if (version < 305 && (g.flags != 0 || g.sva != 0 || g.dtaun != 0 || (g.svh & ~1)))
    return "GLONASS flags/sva/dtaun/extended health require --rnx-version 3.05";
  if (g.svh & ~15) return "RINEX 3.05 cannot preserve health bits above bit 3";
  if (g.flags < 0 || g.flags > 511 || g.sva < 0 || g.sva > 15)
    return "RINEX 3.05 GLONASS status/accuracy outside representable broadcast range";
  const double utc_tow = time2gpst(gpst2utc(g.toe), nullptr);
  if (std::fmod(utc_tow, 900.) != 0)
    return "RINEX GLONASS reader would round toe to 15 minutes";
  const int iode = static_cast<int>(std::fmod(utc_tow+10800.,86400.)/900.);
  if (g.iode != iode) return "RINEX GLONASS reader would regenerate a different iode from toe";
  if (std::abs(timediff(g.tof, g.toe)) > 43200)
    return "RINEX GLONASS reader would resolve tof into a different day";
  return nullptr;
}
} // namespace gnss_utils

#endif  // GNSS_ROS_STANDARDIZATION_EPHEMERIS_STORE_HPP
