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
// (Optional if strictly needed, otherwise we can keep using inline calls)

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

} // namespace gnss_utils

#endif // GNSS_ROS_STANDARDIZATION_GNSS_UTILS_HPP
