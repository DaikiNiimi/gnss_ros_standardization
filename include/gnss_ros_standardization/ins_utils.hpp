#pragma once
/// @file ins_utils.hpp
/// @brief Utility functions for INS/IMU data conversion.
///
/// Provides Euler-to-quaternion conversion and sensor_msgs/Imu covariance helpers
/// used by all vendor decoder/driver nodes.

#include <array>
#include <cmath>

#include <geometry_msgs/msg/quaternion.hpp>

namespace gnss_ros_standardization {
namespace ins_utils {

/// Convert Euler angles (ZYX extrinsic, aerospace convention) to a unit quaternion.
/// @param roll_rad  Right-wing-down rotation about the body X-axis (rad).
/// @param pitch_rad Nose-up rotation about the body Y-axis (rad).
/// @param yaw_rad   Clockwise-from-North heading about the body Z-axis (rad).
/// @return Quaternion representing the same rotation.
inline geometry_msgs::msg::Quaternion eulerToQuaternion(
    double roll_rad, double pitch_rad, double yaw_rad)
{
    const double cr = std::cos(roll_rad  * 0.5);
    const double sr = std::sin(roll_rad  * 0.5);
    const double cp = std::cos(pitch_rad * 0.5);
    const double sp = std::sin(pitch_rad * 0.5);
    const double cy = std::cos(yaw_rad   * 0.5);
    const double sy = std::sin(yaw_rad   * 0.5);

    geometry_msgs::msg::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}

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
