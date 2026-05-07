#pragma once
/// @file ins_utils.hpp
/// @brief Covariance helpers for sensor_msgs/Imu used by IMU decoder/driver nodes.

#include <array>

namespace gnss_ros_standardization {
namespace ins_utils {

/// Return a 3×3 row-major covariance array indicating "unknown" (-1 at [0,0]).
inline std::array<double, 9> makeUnknownCovariance()
{
    std::array<double, 9> cov{};
    cov[0] = -1.0;
    return cov;
}

/// Return a 3×3 diagonal covariance array from per-axis standard deviations.
/// Off-diagonal terms are zero (no cross-correlation assumed).
inline std::array<double, 9> makeDiagCovariance(
    double sigma_x, double sigma_y, double sigma_z)
{
    std::array<double, 9> cov{};
    cov[0] = sigma_x * sigma_x;
    cov[4] = sigma_y * sigma_y;
    cov[8] = sigma_z * sigma_z;
    return cov;
}

}  // namespace ins_utils
}  // namespace gnss_ros_standardization
