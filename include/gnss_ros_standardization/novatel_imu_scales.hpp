// SPDX-License-Identifier: MIT
#pragma once
/// @file novatel_imu_scales.hpp
/// @brief Counts → SI scale factors for NovAtel RAWIMUSX, indexed by IMUType.
///
/// RAWIMUSX delta-velocity / delta-angle counts must be multiplied by these
/// scale factors to obtain m/s and rad per IMU sample. The driver then divides
/// by the inter-sample interval (dt from successive Seconds fields) to recover
/// m/s² and rad/s.
///
/// IMU Type IDs follow the official NovAtel CONNECTIMU table:
///   https://docs.novatel.com/OEM7/Content/SPAN_Commands/CONNECTIMU.htm
/// Scale factors follow the "Raw IMU Scale Factors" table on the RAWIMUSX page:
///   https://docs.novatel.com/OEM7/Content/SPAN_Logs/RAWIMUSX.htm
///
/// Only IMUs with an explicit scale entry in the RAWIMUSX page are included.
/// For any other IMU, set imu_scale_override.{accel,gyro} at runtime.

#include <cstdint>

namespace gnss_ros_standardization {
namespace novatel {

struct ImuScale {
  double accel;  ///< counts → m/s per sample (delta-velocity)
  double gyro;   ///< counts → rad per sample (delta-angle)
};

namespace detail {
constexpr double kPi      = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kFt2M    = 0.3048;
constexpr double kG       = 9.80665;       // standard gravity [m/s²]

// EPSON G320N is configured at 125 Hz by default (NovAtel SPAN default).
// EPSON G320N (ID 62) variant runs at 200 Hz. EPSON G370N is fixed at 200 Hz.
// Per-sample scale = (NovAtel published per-LSB value) — already incorporates
// the assumed DataRate. If the IMU is operated at a different rate than the
// assumption baked into the ID, use imu_scale_override.{accel,gyro}.
constexpr double kG320Rate125 = 125.0;
constexpr double kG320Rate200 = 200.0;
constexpr double kG370Rate200 = 200.0;
}  // namespace detail

/// Look up scale factors for a given NovAtel IMUType code.
/// Returns {0.0, 0.0} when the type has no documented scale in the RAWIMUSX
/// page — the caller should then fall back to a yaml-configured override.
inline constexpr ImuScale getImuScale(uint8_t type) {
  using namespace detail;
  switch (type) {
    // ── Honeywell HG1700 / HG1900 / HG1930 (2⁻³³ rad, 2⁻²⁷ ft/s) ──
    // Accel published in ft/s/LSB → convert to m/s/LSB via × 0.3048.
    case  5:  // HG1900_CA29
    case 11:  // HG1700_AG58
    case 20:  // HG1930_AA99
    case 27:  // HG1900_CA50
    case 28:  // HG1930_CA50
      return {0x1p-27 * kFt2M, 0x1p-33};

    // ── Honeywell HG1700-AG62 (2⁻³³ rad, 2⁻²⁶ ft/s) ──
    case 12:  // HG1700_AG62
      return {0x1p-26 * kFt2M, 0x1p-33};

    // ── Honeywell HG4930 / CPT7 / CPT7700 (2⁻³³ rad, 2⁻²⁹ m/s) ──
    case 58:  // HG4930_AN01 (also CPT7 / CPT7700)
    case 68:  // HG4930_AN04
    case 69:  // HG4930_AN04 400 Hz
      return {0x1p-29, 0x1p-33};

    // ── Northrop Grumman LN-200 (2⁻¹⁹ rad, 2⁻¹⁴ m/s) ──
    case  8:  // LN200
      return {0x1p-14, 0x1p-19};

    // ── NovAtel ISA-100C / µIMU-IC (1.0e-9 rad, 2.0e-8 m/s) ──
    case 26:  // ISA100C
      return {2.0e-8, 1.0e-9};

    // ── OEM-IMU-STIM300 (2⁻²¹ deg, 2⁻²² m/s) ──
    case 32:  // STIM300
    case 56:  // STIM300D
      return {0x1p-22, 0x1p-21 * kDeg2Rad};

    // ── OEM-IMU-EG320N at 125 Hz (DataRate baked in) ──
    case 41:  // EPSON_G320 (125 Hz default)
      return { (0.200 / 65536.0) * (kG / 1000.0) / kG320Rate125,
               (0.008 / 65536.0) * kDeg2Rad      / kG320Rate125 };

    // ── OEM-IMU-EG320N at 200 Hz ──
    case 62:  // EPSON_G320_200HZ
      return { (0.200 / 65536.0) * (kG / 1000.0) / kG320Rate200,
               (0.008 / 65536.0) * kDeg2Rad      / kG320Rate200 };

    // ── OEM-IMU-EG370N (fixed 200 Hz) ──
    case 61:  // EPSON_G370
      return { (0.400      / 65536.0) * (kG / 1000.0) / kG370Rate200,
               (0.0151515  / 65536.0) * kDeg2Rad      / kG370Rate200 };

    default:
      return {0.0, 0.0};
  }
}

}  // namespace novatel
}  // namespace gnss_ros_standardization