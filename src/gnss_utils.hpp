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

} // namespace gnss_utils

#endif // GNSS_ROS_STANDARDIZATION_GNSS_UTILS_HPP
