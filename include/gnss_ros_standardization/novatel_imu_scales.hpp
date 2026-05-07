#pragma once
/// @file novatel_imu_scales.hpp
/// @brief Counts → SI scale factors for NovAtel RAWIMUSX, indexed by IMUType field.
///
/// RAWIMUSX delta-velocity / delta-angle counts must be multiplied by these scale
/// factors to obtain values in (m/s) and (rad) per IMU sample. Divide by the IMU
/// sample interval (dt from successive Seconds fields) to recover m/s² and rad/s.
///
/// Reference: NovAtel SPAN documentation (OEM7 / OEM6 Firmware Reference Manuals)
/// and IMU-specific datasheets. To add support for a new IMU, simply append a
/// `case <type>:` to `getImuScale()`.

#include <cstdint>

namespace gnss_ros_standardization {
namespace novatel {

struct ImuScale {
  double accel;  ///< counts → m/s per sample (delta-velocity scale)
  double gyro;   ///< counts → rad per sample (delta-angle scale)
};

/// Look up scale factors for a given NovAtel IMUType code.
/// Returns {0.0, 0.0} when the type is not in the table — caller should fall
/// back to a yaml-configured override or skip publishing.
inline constexpr ImuScale getImuScale(uint8_t type) {
  switch (type) {
    // Honeywell HG1700 family (AG58 / AG62 / AG17)
    case  4:                    return {0x1p-22, 0x1p-33};  // HG1700-AG11
    case  5:                    return {0x1p-22, 0x1p-33};  // HG1700-AG17
    case  8:                    return {0x1p-22, 0x1p-33};  // HG1700-AG58
    case 11:                    return {0x1p-22, 0x1p-33};  // HG1700-AG62

    // Honeywell HG1900 / HG1930
    case 13:                    return {0x1p-21, 0x1p-33};  // HG1900-CA50
    case 19:                    return {0x1p-22, 0x1p-33};  // HG1930-AA99 / similar

    // Northrop Grumman LN200
    case 16:                    return {0x1p-14, 0x1p-19};  // LN200

    // Analog Devices ADIS16488 (built into IMU-IGM-S1 etc.)
    case 26: case 28:           return {0x1p-12, 0x1p-21};

    // Epson G320N / G370N / G382PR
    case 41:                    return {0x1p-15, 0x1p-21};  // Epson G320N
    case 56:                    return {0x1p-15, 0x1p-21};  // KVH 1750
    case 58: case 68: case 69:  return {0x1p-15, 0x1p-21};  // Epson G370N / G382PR

    // STIM 300
    case 31:                    return {0x1p-21, 0x1p-25};

    default:                    return {0.0, 0.0};
  }
}

}  // namespace novatel
}  // namespace gnss_ros_standardization
