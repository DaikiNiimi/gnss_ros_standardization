// SPDX-License-Identifier: MIT
#ifndef GNSS_ROS_STANDARDIZATION_GNSS_UTILS_HPP
#define GNSS_ROS_STANDARDIZATION_GNSS_UTILS_HPP

#include <cstdint>
#include <limits>
#include <string>
#include <vector>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>

#include "gnss_ros_standardization/msg/gnss_ephemeris.hpp"
#include "gnss_ros_standardization/msg/glonass_ephemeris.hpp"
#include "gnss_ros_standardization/msg/gnss_ephemerides.hpp"
#include "gnss_ros_standardization/msg/gnss_observation.hpp"
#include "gnss_ros_standardization/msg/gnss_observations.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"

extern "C" {
#include "rtklib.h"
}

namespace gnss_utils {

// ---- Conversion Helpers ----

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

/**
 * Convert RTKLIB eph_t to Ros message
 */
gnss_ros_standardization::msg::GnssEphemeris ephToMsg(const eph_t& e);

/**
 * Convert RTKLIB geph_t to Ros message
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
gnss_ros_standardization::msg::GnssObservation obsToMsg(const obsd_t& o, int kf);

// ---- Time Helpers ----

/**
 * Convert RTKLIB GPS time (gtime_t) to UTC ROS Time, applying leap seconds via gpst2utc().
 */
rclcpp::Time gpstToUtcRosTime(gtime_t t_gpst);

/**
 * Assemble a UTC (year, month, day, hhmmss.ss) and convert to GPS week / tow
 * using RTKLIB epoch2time() + utc2gpst() (leap-second correction included).
 * @return true on success.
 */
bool nmeaUtcToGpsTime(int year, int month, int day, double hms,
                      uint32_t& week, double& tow);

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
 * rclcpp::Logger and clamps to `fallback` when the value is outside [1, 20].
 * Centralized so all GNSS driver nodes use the same range and warning text.
 *
 * @param rate_hz   the requested rate (mutated in place)
 * @param fallback  the clamp target on out-of-range input (default 5)
 * @param logger    rclcpp::Logger to emit the warning on
 * @return true if the input was in range; false if it was clamped
 */
bool validatePublishRate(int& rate_hz, int fallback, const rclcpp::Logger& logger);

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
  NmeaParser() = default;
  ~NmeaParser() = default;

  /**
   * Feed one NMEA sentence (starting with '$', checksum optional but verified
   * if present).
   * @param sentence Single ASCII NMEA sentence string.
   * @param[out] solution Populated only when the return value is true. The
   *             fields supplied by sentences not seen in this epoch are
   *             written as NaN.
   * @return True iff an epoch was flushed into `solution` by this call.
   */
  bool parseSentence(const std::string& sentence, gnss_ros_standardization::msg::GnssSolution& solution);

 private:
  // Sentence-type bits for pending_received_ / sentences_ever_seen_.
  enum SentenceBit : uint8_t {
    SENT_GGA = 1 << 0,
    SENT_RMC = 1 << 1,
    SENT_GST = 1 << 2,
  };

  // Dispatchers fill pending_* state; they do NOT decide flushing.
  bool applyGga(const std::vector<std::string>& fields);
  bool applyRmc(const std::vector<std::string>& fields);
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
  int cached_year_{0};
  int cached_month_{0};
  int cached_day_{0};
  bool has_date_cache_{false};
  double cached_last_hms_{0.0};  // most recent GGA hhmmss.ss for UTC day-rollover detection

  // ---- Pending epoch state ----
  double pending_sod_{-1.0};        // seconds-of-day of the epoch being assembled; <0 = none
  uint8_t pending_received_{0};     // SentenceBit OR'd as each type arrives
  uint8_t sentences_ever_seen_{0};  // learned set of types this receiver emits per epoch
  bool learned_{false};             // true after first boundary flush (enables eager flush)

  // GGA fields (raw, applied to solution at flush time)
  bool   pgga_present_{false};
  uint32_t pgga_week_{0};
  double pgga_tow_{0.0};
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

#endif // GNSS_ROS_STANDARDIZATION_GNSS_UTILS_HPP