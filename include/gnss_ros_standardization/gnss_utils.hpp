#ifndef GNSS_ROS_STANDARDIZATION_GNSS_UTILS_HPP
#define GNSS_ROS_STANDARDIZATION_GNSS_UTILS_HPP

#include <cstdint>
#include <chrono>
#include <deque>
#include <limits>
#include <string>
#include <utility>
#include <vector>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>

#include "gnss_ros_standardization/msg/gnss_ephemeris.hpp"
#include "gnss_ros_standardization/msg/glonass_ephemeris.hpp"
#include "gnss_ros_standardization/msg/gnss_ephemerides.hpp"
#include "gnss_ros_standardization/msg/gnss_observation.hpp"
#include "gnss_ros_standardization/msg/gnss_observations.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"

#include "gnss_ros_standardization/rtklib_compat.hpp"

namespace rclcpp { class Node; }

namespace gnss_utils {

// Unknown velocity must not look like a perfectly certain stationary fix.
inline bool hasPositionFix(const gnss_ros_standardization::msg::GnssSolution& sol) {
  using S = gnss_ros_standardization::msg::GnssSolution;
  return sol.status == S::STATUS_SINGLE || sol.status == S::STATUS_DGPS ||
         sol.status == S::STATUS_SBAS || sol.status == S::STATUS_FLOAT ||
         sol.status == S::STATUS_FIX || sol.status == S::STATUS_PPP;
}

inline void setUnknownVelocity(gnss_ros_standardization::msg::GnssSolution& msg) {
  const double nan=std::numeric_limits<double>::quiet_NaN();
  msg.vel_ecef.x=msg.vel_ecef.y=msg.vel_ecef.z=nan;
  msg.vel_enu.x=msg.vel_enu.y=msg.vel_enu.z=nan;
  msg.vel_cov_ecef.fill(nan);msg.vel_enu_cov.fill(nan);
}

// One rule for every producer: an epoch without a solution is published as
// STATUS_NONE with NaN position and velocity (never a stale or predicted value).
inline void setNoSolution(gnss_ros_standardization::msg::GnssSolution& msg) {
  const double nan=std::numeric_limits<double>::quiet_NaN();
  const float nanf=std::numeric_limits<float>::quiet_NaN();
  msg.status=gnss_ros_standardization::msg::GnssSolution::STATUS_NONE;
  msg.num_sats=0;
  msg.gdop=msg.pdop=msg.hdop=msg.vdop=nanf;
  msg.latitude=msg.longitude=msg.altitude=nan;
  msg.pos_ecef.x=msg.pos_ecef.y=msg.pos_ecef.z=nan;
  msg.pos_enu.x=msg.pos_enu.y=msg.pos_enu.z=nan;
  msg.pos_cov_ecef.fill(nan);msg.pos_enu_cov.fill(nan);
  setUnknownVelocity(msg);
}

// ---- Conversion Helpers ----
// Stable fields take precedence over build-dependent numeric indices. When both
// satid and constellation/PRN are supplied, disagreement is an error (sat=0).
int resolveSatellite(const std::string& satid, const std::string& system, int prn);
int resolveSignal(const std::string& signal, int backend_code);

/**
 * Convert internal RTKLIB system constant (SYS_GPS, etc.) to 1-char string ("G", "R", etc.)
 */
std::string systemCode(int sys);

/**
 * Convert RTKLIB satellite number to string ID (e.g. "G01")
 */
std::string satId(int sat);

/**
 * Convert RTKLIB solution-quality enum (SOLQ_*) to short string label
 * ("FIX", "FLOAT", "SINGLE", "DGPS", "SBAS", "PPP", "DR", "NONE").
 */
std::string solqToString(int stat);

/**
 * Convert Ros message to RTKLIB eph_t
 */
eph_t msgToEph(const gnss_ros_standardization::msg::GnssEphemeris& m);

/**
 * Convert Ros message to RTKLIB geph_t
 */
geph_t msgToGeph(const gnss_ros_standardization::msg::GlonassEphemeris& m);

// Return nullptr when the backend record can be represented without narrowing
// and finite orbit/clock coefficients; otherwise return a stable reason.
// A>0 and 0<=e<1 are required; this is not full manufacturer conformance.
const char* ephemerisMessageError(const eph_t& e);
const char* ephemerisMessageError(const geph_t& g);

// The pinned RTKLIB eph2pos does not implement CNAV rate terms. Acquisition
// retains them, but the bundled positioning paths must not silently ignore them.
inline bool supportedByLegacyOrbitModel(const eph_t& e) {
  return e.Adot == 0.0 && e.ndot == 0.0;
}

/**
 * Convert RTKLIB eph_t to Ros message; throws std::invalid_argument when the
 * record cannot be represented. Use ephemerisMessageError at stream boundaries.
 */
gnss_ros_standardization::msg::GnssEphemeris ephToMsg(const eph_t& e);

/**
 * Convert RTKLIB geph_t to Ros message; throws std::invalid_argument on narrowing.
 */
gnss_ros_standardization::msg::GlonassEphemeris gephToMsg(const geph_t& g);

/**
 * Normalize Galileo eph.code to canonical RINEX values. RTKLIB's raw I/NAV
 * decode path produces 513/516/517 depending on src signal and the mix
 * flag, but the ephemeris is the same per IODnav. Bit 8/9 selects F/NAV vs
 * I/NAV clock reference, which is the only semantically meaningful split.
 */
int canonicalGalCode(int code);

/**
 * Convert RTKLIB obsd_t (specific frequency index) to Ros message
 */
// Returns an empty message (sat=0) for an unsupported satellite/signal.
// Invalid components become NaN in ROS; normalize them before calling RTKLIB.
// Patched acquisition backends use 255 for an unknown obsd_t.freq.
// Optional dated navigation supplies a channel for formats without an observation
// FCN (closest toe within MAXDTOE_GLO; ambiguous channels remain unknown).
// The unaged glo_fcn cache alone is not evidence for this observation epoch.
gnss_ros_standardization::msg::GnssObservation obsToMsg(const obsd_t& o, int kf, const nav_t* nav = nullptr);

// ---- Time Helpers ----

/**
 * Convert RTKLIB GPS time (gtime_t) to UTC ROS Time, applying leap seconds via gpst2utc().
 */
// ROS builtin Time has signed int32 seconds. Invalid/out-of-range input throws
// std::out_of_range before integer conversion; use this predicate at live boundaries.
bool isRosTimeRepresentable(gtime_t t_gpst);
rclcpp::Time gpstToUtcRosTime(gtime_t t_gpst);

/**
 * Assemble a UTC (year, month, day, hhmmss.ss) and convert to GPS week / tow
 * using RTKLIB epoch2time() + utc2gpst() (leap-second correction included).
 * @return true on success.
 */
bool nmeaUtcToGpsTime(int year, int month, int day, double hms,
                      uint32_t& week, double& tow);

/**
 * Convert a UTC ROS Time to GPS week / time-of-week via utc2gpst() (leap-
 * second correction included). Inverse of gpstToUtcRosTime().
 * @return true on success (week > 0).
 */
bool utcRosTimeToGpst(const rclcpp::Time& t_utc_ros, uint32_t& week, double& tow);

/**
 * Selects the epoch time a GNSS measurement is applied at when fusing against
 * other ROS-stamped sensors. An arrival-time header.stamp lags the true epoch
 * (week/tow) by the receiver output latency (tens~150 ms, jittery) — applying
 * there drags a moving estimate backwards by v x latency.
 *
 *  - Header:        header.stamp as-is (correct when the producer stamps at
 *                   the measurement epoch).
 *  - TowAutoOffset: utc(week,tow) + sliding-window min of (stamp - utc(tow))
 *                   (= clock offset + min latency); works on an unsynced PC
 *                   clock, residual ~= the min latency.
 *
 * Invalid week/tow falls back to Header. The returned time uses the stamp's
 * clock type.
 */
class GnssEpochAligner {
 public:
  enum class Source { Header, TowAutoOffset };

  struct Config {
    Source source{Source::Header};
    double offset_window_s{60.0};  // [s] sliding window of the min-d_k estimate
  };

  /** Map a parameter string ("header" | "tow_auto_offset") to a Source. */
  static bool parseSource(const std::string& name, Source& out);

  void reset() { offsets_.clear(); last_ctow_ = -1.0; }
  GnssEpochAligner() = default;
  explicit GnssEpochAligner(const Config& cfg) : cfg_(cfg) {}

  /**
   * Return the epoch time to apply a measurement stamped `stamp` with GNSS
   * time (week, tow). On any fallback to Header, `warn` (if non-null) is set
   * to a human-readable reason (empty otherwise); callers throttle-log it.
   */
  rclcpp::Time align(uint32_t week, double tow, const rclcpp::Time& stamp,
                     std::string* warn = nullptr);

 private:
  Config cfg_;
  double last_ctow_{-1.0};
  // (stamp seconds, d_k = stamp - utc(week,tow)) history for TowAutoOffset.
  std::deque<std::pair<double, double>> offsets_;
};

// ---- Math Helpers ----

/**
 * Rotate 3x3 covariance matrix from ECEF to ENU.
 * @param cov_ecef 3x3 covariance in ECEF (row-major: xx, xy, xz, yx, yy, yz, zx, zy, zz)
 * @param lat_rad Latitude in radians
 * @param lon_rad Longitude in radians
 * @param cov_enu Output 3x3 covariance in ENU
 */
void rotateCovariance(const double cov_ecef[9], double lat_rad, double lon_rad, double cov_enu[9]);

/**
 * Rotate 3x3 covariance matrix from ENU to ECEF.
 * @param cov_enu 3x3 covariance in ENU (row-major: ee, en, eu, ne, nn, nu, ue, un, uu)
 * @param lat_rad Latitude in radians
 * @param lon_rad Longitude in radians
 * @param cov_ecef Output 3x3 covariance in ECEF
 */
void rotateCovarianceEnuToEcef(const double cov_enu[9], double lat_rad, double lon_rad, double cov_ecef[9]);

/**
 * Calculate DOPs from current satellite configuration.
 * @param sats List of satellites (obsd_t) used in solution
 * @param ns Number of satellites
 * @param nav Navigation data (for ephemeris/positions if needed, but dops uses azel)
 * Note: RTKLIB dops() needs azel. We can pass existing azel from RTK struct if available.
 * Actually, better to pass the rtk_t struct or ssat_t array?
 * Let's keep it simple: Pass ssat_t array and count.
 */
struct Dops {
    double gdop{0.0}, pdop{0.0}, hdop{0.0}, vdop{0.0};
};
Dops calculateDops(const ssat_t* ssat, int ns_max, double el_min_rad);

// ---- DOP cache + staleness gate (msg-coupled DOP from receiver blocks) ----

/**
 * Snapshot of the most-recently-parsed DOP block from any receiver.
 * Persists across PVT epochs inside the decoder/driver node.
 */
struct DopCache {
  bool     valid{false};
  uint32_t week{0};   // 0 = receiver's DOP block has no week field (UBX NAV-DOP) → skip week check
  uint32_t tow_ms{0};
  float    gdop{std::numeric_limits<float>::quiet_NaN()};
  float    pdop{std::numeric_limits<float>::quiet_NaN()};
  float    hdop{std::numeric_limits<float>::quiet_NaN()};
  float    vdop{std::numeric_limits<float>::quiet_NaN()};
};

/**
 * Populate sol.{gdop,pdop,hdop,vdop} from the DOP cache if it belongs to the
 * current or immediately-prior PVT epoch. The asymmetric window matches the
 * common receiver block ordering where the DOP block follows PVT within the
 * same frame and is therefore cached for use at the next PVT publish.
 *
 * Conditions for populate (all required):
 *   - cache.valid
 *   - cache.week == 0 OR cache.week == pvt_week
 *   - pvt_period_ms > 0 (≥ 2 PVT epochs observed; period auto-detected)
 *   - 0 <= (pvt_tow_ms - cache.tow_ms) <= pvt_period_ms
 *     (DOP for this PVT epoch or the prior one — never a future epoch, never
 *     more than one period stale).
 *
 * On failure, the four DOP fields are explicitly set to NaN (so the caller
 * does not need to pre-initialize them).
 */
void applyDopWithStaleness(gnss_ros_standardization::msg::GnssSolution& sol,
                           const DopCache& cache,
                           uint32_t pvt_week,
                           uint32_t pvt_tow_ms,
                           uint32_t pvt_period_ms);

/**
 * Validate a publish/measurement rate in Hz. Logs a WARN via the supplied
 * rclcpp::Logger and clamps to `fallback` when the value is outside
 * [1, max_hz]. Centralized so all GNSS driver nodes use the same warning
 * text; the upper bound is per-driver (e.g. 100 for Septentrio, 20 for
 * u-blox/NovAtel).
 *
 * @param rate_hz   the requested rate (mutated in place)
 * @param fallback  the clamp target on out-of-range input
 * @param max_hz    driver-specific upper bound in Hz
 * @param logger    rclcpp::Logger to emit the warning on
 * @return true if the input was in range; false if it was clamped
 */
bool validatePublishRate(int& rate_hz, int fallback, int max_hz, const rclcpp::Logger& logger);

/**
 * ENU origin of a node's published local frame. Every node that publishes one
 * declares the same three parameters through this function:
 *   local_origin.mode    "auto" (default): the node's automatic origin
 *                        "fixed": local_origin.pos
 *   local_origin.pos     [lat_deg, lon_deg, h_m] for "llh", [x, y, z] m for "ecef"
 *   local_origin.postype "llh" (default) | "ecef"
 * "auto" requires pos to stay [0, 0, 0], so a position given without
 * mode "fixed" is rejected rather than silently ignored.
 * @throws std::invalid_argument on an invalid combination
 */
struct LocalOrigin {
  bool fixed = false;
  double llh[3] = {0.0, 0.0, 0.0};   ///< rad, rad, m (when fixed)
  double ecef[3] = {0.0, 0.0, 0.0};  ///< m (when fixed)
};
LocalOrigin declareLocalOrigin(rclcpp::Node& node);

// ---- Lightweight NMEA Parser ----

/**
 * A lightweight stateful NMEA parser that aggregates GGA / RMC / GST belonging
 * to the same UTC epoch into a single GnssSolution.
 *
 * Sentence emission order varies across receivers (GGA-first vs GGA-last), so
 * the parser buffers a "pending epoch" keyed by seconds-of-day. The epoch is
 * eager-flushed the moment all sentence types ever observed for one cycle have
 * arrived (latency ≈ 0 in steady state), or boundary-flushed when a sentence
 * with a new sod arrives (one-epoch latency only during the initial learning
 * cycle).
 */
class NmeaParser {
 public:
  using Clock=std::chrono::steady_clock;
  NmeaParser() = default;
  // Clear receiver-derived date/assembly state; preserve an explicit file reference.
  void reset();
  ~NmeaParser() = default;

  // UTC reference for recordings without RMC/ZDA; no implicit host-date guess.
  bool setReferenceDate(int year, int month, int day);

  /**
   * Feed one NMEA sentence (starting with '$', checksum optional but verified
   * if present).
   * @param sentence Single ASCII NMEA sentence string.
   * @param[out] solution Populated only when the return value is true. The
   *             fields supplied by sentences not seen in this epoch are
   *             written as NaN.
   * @return True iff an epoch was flushed into `solution` by this call.
   */
  bool parseSentence(const std::string& sentence, gnss_ros_standardization::msg::GnssSolution& solution,
                     Clock::time_point now=Clock::now());

 private:
  // Sentence-type bits for pending_received_ / sentences_ever_seen_.
  enum SentenceBit : uint8_t {
    SENT_GGA = 1 << 0,
    SENT_RMC = 1 << 1,
    SENT_GST = 1 << 2,
    SENT_ZDA = 1 << 3,
  };

  // Dispatchers fill pending_* state; they do NOT decide flushing.
  bool applyGga(const std::vector<std::string>& fields);
  bool applyRmc(const std::vector<std::string>& fields);
  bool applyZda(const std::vector<std::string>& fields);
  bool setDate(int year, int month, int day, double hms);
  void resolveGgaTime();
  bool applyGsa(const std::vector<std::string>& fields);
  bool applyGst(const std::vector<std::string>& fields);

  // Compose the final solution from pending_* state and reset for the next epoch.
  void flushPending(gnss_ros_standardization::msg::GnssSolution& out);
  void resetPending();

  static std::vector<std::string> splitString(const std::string& str, char delimiter);
  static double parseCoordinate(const std::string& coord_str, const std::string& hem);
  static double parseDouble(const std::string& str);
  static int parseInteger(const std::string& str);

  // ---- Date cache (RMC field[9] supplies the date; GGA only carries time-of-day) ----
  int reference_year_{0}, reference_month_{0}, reference_day_{0};
  Clock::time_point date_arrival_{}, input_arrival_{};
  bool explicit_date_{false};
  int cached_year_{0};
  int cached_month_{0};
  int cached_day_{0};
  bool has_date_cache_{false};
  double cached_date_sod_{-1.0}; // epoch of the UTC date anchor

  // ---- Pending epoch state ----
  double pending_sod_{-1.0};        // seconds-of-day of the epoch being assembled; <0 = none
  uint8_t pending_received_{0};     // SentenceBit OR'd as each type arrives
  uint8_t sentences_ever_seen_{0};  // learned set of types this receiver emits per epoch
  bool learned_{false};             // true after first boundary flush (enables eager flush)

  // GGA fields (raw, applied to solution at flush time)
  bool   pgga_present_{false};
  uint32_t pgga_week_{0};
  double pgga_tow_{0.0};
  double pgga_hms_{0.0};
  uint8_t pgga_status_{0};
  double pgga_lat_{0.0};
  double pgga_lon_{0.0};
  double pgga_alt_{0.0};
  uint8_t pgga_num_sats_{0};
  double pgga_age_diff_{0.0};
  float  pgga_hdop_{0.0f};

  // RMC fields
  bool   prmc_present_{false};   // RMC parsed AND status=Active
  double prmc_vel_east_{0.0};
  double prmc_vel_north_{0.0};

  // GST fields
  bool   pgst_present_{false};
  double pgst_var_lat_{0.0};
  double pgst_var_lon_{0.0};
  double pgst_var_alt_{0.0};

  // GSA fields. Persistent across resetPending so they survive flushes;
  // staleness gated by cycles_since_gsa_ (≤1 = fresh, >1 = invalidated).
  // GSA has no timestamp, so it never gates flushing; it just rides along.
  bool    pgsa_present_{false};
  float   pgsa_pdop_{0.0f};
  float   pgsa_vdop_{0.0f};
  uint8_t cycles_since_gsa_{0};

  // ---- Epoch period learning (for sameEpoch tolerance) ----
  double last_gga_sod_{-1.0};
  double epoch_period_{-1.0};
};

} // namespace gnss_utils


// ---- observation comparison ----
#include <algorithm>
#include <cmath>
#include <tuple>
#include "gnss_ros_standardization/msg/gnss_observation.hpp"

namespace gnss_utils {
// ROS floating-point NaNs compare unequal by default; repeated invalid values
// must not turn an identical RTCM epoch into an endless stream of updates.
inline bool sameObservation(const gnss_ros_standardization::msg::GnssObservation& a,
                            const gnss_ros_standardization::msg::GnssObservation& b) {
  const auto equal=[](double x,double y) { return x==y || (std::isnan(x) && std::isnan(y)); };
  return std::tie(a.sat,a.freq,a.code,a.lli,a.system,a.prn,a.satid,a.code_str) ==
         std::tie(b.sat,b.freq,b.code,b.lli,b.system,b.prn,b.satid,b.code_str) &&
         equal(a.p,b.p) && equal(a.l,b.l) && equal(a.d,b.d) && equal(a.snr,b.snr);
}
}


// ---- glonass channel ----
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <map>
#include <vector>
#include "gnss_ros_standardization/rtklib_compat.hpp"

namespace gnss_utils {

// Match RTKLIB selgeph's age limit and closest-toe rule. A channel conflict
// between equally close records is ambiguous; do not let array order decide.
inline const geph_t* selectGlonassEphemeris(const nav_t& nav, int sat, gtime_t time) {
  const geph_t* best = nullptr;
  double distance = MAXDTOE_GLO;
  bool ambiguous = false;
  if (sat <= 0 || !std::isfinite(time.sec)) return nullptr;
  for (int i = 0; i < nav.ng; ++i) {
    const auto& g = nav.geph[i];
    if (g.sat != sat) continue;
    const double dt = std::abs(timediff(g.toe, time));
    if (!std::isfinite(dt) || dt > distance) continue;
    if (!best || dt < distance) ambiguous = false;
    else if (g.frq != best->frq) ambiguous = true;
    best = &g;
    distance = dt;
  }
  return ambiguous ? nullptr : best;
}

struct ObservationRejectCounters {
  std::uint64_t signal_channel_conflict{0}; // one receiver/satellite entry
  std::uint64_t invalid_epoch{0};           // one buildObsEpoch call
  std::uint64_t missing_glonass_navigation{0};
  std::uint64_t invalid_navigation_channel{0};
  std::uint64_t navigation_channel_conflict{0};
  std::uint64_t total() const {
    return signal_channel_conflict + invalid_epoch + missing_glonass_navigation +
           invalid_navigation_channel + navigation_channel_conflict;
  }
};

// Borrow all other navigation data, but own one age-checked GLONASS record per
// satellite. sat2freq() takes the FIRST geph while satposs() selects by time:
// this view makes both use the same record without modifying the shared store.
// The source must outlive this view; never pass get() to freenav().
class GlonassNavigationView {
 public:
  GlonassNavigationView(const nav_t& source, gtime_t epoch) : source_(source), nav_(source) {
    for (int prn = 1; prn <= MAXPRNGLO; ++prn) {
      const auto* g = selectGlonassEphemeris(source, satno(SYS_GLO, prn), epoch);
      if (g) geph_.push_back(*g);
    }
    nav_.geph = geph_.data();
    nav_.ng = nav_.ngmax = static_cast<int>(geph_.size());
    // An unaged cache must not resurrect a missing/stale orbit or channel.
    std::fill(std::begin(nav_.glo_fcn), std::end(nav_.glo_fcn), 0);
  }
  GlonassNavigationView(const GlonassNavigationView&) = delete;
  GlonassNavigationView& operator=(const GlonassNavigationView&) = delete;
  const nav_t& get() const { return nav_; }

  void filter(std::vector<obsd_t>& obs, ObservationRejectCounters& counters) const {
    obs.erase(std::remove_if(obs.begin(), obs.end(), [&](obsd_t& o) {
      if (satsys(o.sat, nullptr) != SYS_GLO) return false;
      const auto* g = selectGlonassEphemeris(nav_, o.sat, o.time);
      const auto* own_epoch = selectGlonassEphemeris(source_, o.sat, o.time);
      if (!g || !own_epoch) {
        ++counters.missing_glonass_navigation;
        return true;
      }
      if (g->frq < -7 || g->frq > 6 || own_epoch->frq < -7 || own_epoch->frq > 6) {
        ++counters.invalid_navigation_channel;
        return true;
      }
      if (g->frq != own_epoch->frq || (o.freq <= 13 && o.freq != g->frq + 7)) {
        ++counters.navigation_channel_conflict;
        return true;
      }
      // Only the private estimator input is filled. Published unknown values
      // and the original ROS observations remain untouched.
      o.freq = static_cast<uint8_t>(g->frq + 7);
      return false;
    }), obs.end());
  }

 private:
  const nav_t& source_;
  nav_t nav_;
  std::vector<geph_t> geph_;
};

// A confirmed channel change breaks carrier continuity even if a receiver did
// not set LLI. Mark only the private solver copy; preserve all original LLI bits.
class GlonassChannelTracker {
 public:
  void apply(std::vector<obsd_t>& obs) {
    for (auto& o : obs) {
      if (satsys(o.sat, nullptr) != SYS_GLO || o.freq > 13) continue;
      for (int f = 0; f < NFREQ; ++f) {
        if (!o.code[f] || !std::isfinite(o.L[f]) || o.L[f] == 0.) continue;
        const std::uint32_t key = (static_cast<std::uint32_t>(o.rcv) << 24) | (o.sat << 8) | f;
        auto previous = channels_.find(key);
        if (previous != channels_.end() && previous->second != o.freq) o.LLI[f] |= LLI_SLIP;
        channels_[key] = o.freq;
      }
    }
  }
  void clear() { channels_.clear(); }
 private:
  std::map<std::uint32_t, uint8_t> channels_;
};

}  // namespace gnss_utils

#endif // GNSS_ROS_STANDARDIZATION_GNSS_UTILS_HPP
