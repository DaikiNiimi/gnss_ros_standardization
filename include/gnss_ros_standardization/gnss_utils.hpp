#ifndef GNSS_ROS_STANDARDIZATION_GNSS_UTILS_HPP
#define GNSS_ROS_STANDARDIZATION_GNSS_UTILS_HPP

#include <string>
#include <vector>
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
                      uint16_t& week, double& tow);

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

// ---- Lightweight NMEA Parser ----

/**
 * A lightweight stateful NMEA parser that specifically handles
 * GGA (position, status), RMC (velocity), and GSA (DOPs).
 */
class NmeaParser {
 public:
  NmeaParser() = default;
  ~NmeaParser() = default;

  /**
   * Parse a single NMEA sentence (starting with $ and ending with \r\n).
   * @param sentence Single ASCII NMEA sentence string.
   * @param[out] solution Populated GnssSolution message. May only be partially
   *             populated depending on the sentences received so far.
   * @return True if a GGA was parsed and the system should consider the solution
   *         updated/ready for publishing.
   */
  bool parseSentence(const std::string& sentence, gnss_ros_standardization::msg::GnssSolution& solution);

 private:
  bool parseGga(const std::vector<std::string>& fields, gnss_ros_standardization::msg::GnssSolution& solution);
  bool parseRmc(const std::vector<std::string>& fields, gnss_ros_standardization::msg::GnssSolution& solution);
  bool parseGsa(const std::vector<std::string>& fields, gnss_ros_standardization::msg::GnssSolution& solution);
  bool parseGst(const std::vector<std::string>& fields, gnss_ros_standardization::msg::GnssSolution& solution);
  
  static std::vector<std::string> splitString(const std::string& str, char delimiter);
  static double parseCoordinate(const std::string& coord_str, const std::string& hem);
  static double parseDouble(const std::string& str);
  static int parseInteger(const std::string& str);

  // Buffer state
  bool has_time_{false};
  // Cached UTC date from the last RMC (field[9] ddmmyy) or ZDA. Used to assemble
  // GPS week/tow when GGA-only sentences arrive (GGA carries time-of-day but no date).
  int cached_year_{0};
  int cached_month_{0};
  int cached_day_{0};
  bool has_date_cache_{false};
  double cached_last_hms_{0.0};  // most recent GGA hhmmss.ss seen, for UTC day-rollover detection
  double last_pdop_{0.0};
  double last_hdop_{0.0};
  double last_vdop_{0.0};
  
  // RMC buffered velocity (EN components)
  double vel_east_{0.0};
  double vel_north_{0.0};
  bool has_velocity_{false};
  
  // GST buffered variance
  double var_lat_{0.0};
  double var_lon_{0.0};
  double var_alt_{0.0};
  bool has_variance_{false};
};

} // namespace gnss_utils

#endif // GNSS_ROS_STANDARDIZATION_GNSS_UTILS_HPP
