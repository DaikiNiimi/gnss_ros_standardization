// Observation processing following RTKPLOT: azimuth/elevation, DOP, multipath,
// ionosphere and geometry-free slip flags.
#pragma once

#include <vector>

#include "gnss_ros_standardization/rtklib_compat.hpp"

#include "plot_data.hpp"

namespace rtk_bag_plot {

// DOP of the satellites at or above `mask_deg`. `azel` holds (az, el) pairs in
// radians; `values` receives GDOP, PDOP, HDOP and VDOP. Returns the satellite count.
int maskedDop(const std::vector<double>& azel, int mask_deg, double* values);

// Sorts epochs by GPST, drops duplicated epochs and orders signals so that the
// rows of one epoch are grouped by satellite, band and code priority. Returns
// the number of duplicated epochs removed.
size_t normalizeObservationTopic(RawObservationTopic& raw);

// Converts a normalized topic into plot series. `nav` may be null when the bag
// has no ephemeris; geometry-dependent series are then left unavailable.
ObservationResult processObservationTopic(RawObservationTopic& raw, const nav_t* nav,
                                          double origin_gpst);

}  // namespace rtk_bag_plot
