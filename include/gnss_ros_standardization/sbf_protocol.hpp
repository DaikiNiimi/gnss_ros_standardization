#pragma once
/// @file sbf_protocol.hpp
/// @brief Septentrio SBF protocol constants and utility types.
///
/// This header centralizes all SBF protocol definitions used by the driver and decoder nodes.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

// Include rtklib.h for STR_ constants
#include "rtklib.h"

#include "gnss_ros_standardization/msg/gnss_solution.hpp"

namespace gnss_ros_standardization {
namespace sbf {

// =============================================================================
// Constants
// =============================================================================

constexpr size_t READ_BUFFER_SIZE = 4096;

// SBF sync bytes — every SBF block starts with these two bytes
constexpr uint8_t SBF_SYNC1 = 0x24;  // '$'
constexpr uint8_t SBF_SYNC2 = 0x40;  // '@'

// =============================================================================
// SBF Block IDs
// =============================================================================
// Measurement Blocks
constexpr uint16_t ID_MEASEPOCH   = 4027;
constexpr uint16_t ID_MEASEXTRA   = 4000;

// Raw Navigation Subframe Blocks (decoded by RTKLIB input_sbf())
constexpr uint16_t ID_GPSRAWCA    = 4017;
constexpr uint16_t ID_GLORAWCA    = 4026;
constexpr uint16_t ID_GALRAWFNAV  = 4022;
constexpr uint16_t ID_GALRAWINAV  = 4023;
constexpr uint16_t ID_GEORAWL1    = 4020;
constexpr uint16_t ID_BDSRAW      = 4047;
constexpr uint16_t ID_QZSRAWL1CA  = 4066;
constexpr uint16_t ID_QZSRAWL6    = 4069;
constexpr uint16_t ID_NAVICRAW    = 4093;

// Decoded Navigation Blocks (receiver-assembled, complete ephemeris per block)
constexpr uint16_t ID_GPSNAV      = 5891;
constexpr uint16_t ID_GLONAV      = 4004;
constexpr uint16_t ID_GALNAV      = 4002;
constexpr uint16_t ID_BDSNAV      = 4081;
constexpr uint16_t ID_QZSNAV      = 4095;
constexpr uint16_t ID_NAVICNAV    = 4099;

// PVT Blocks
constexpr uint16_t ID_PVTCARTESIAN      = 4006;
constexpr uint16_t ID_PVTGEODETIC       = 4007;
constexpr uint16_t ID_POSCOVCARTESIAN   = 5905;
constexpr uint16_t ID_POSCOVGEODETIC    = 5906;

// INS / IMU Blocks (AsteRx-i / mosaic-X5 with integrated IMU)
constexpr uint16_t ID_EXTSENSORMEAS     = 4050;  // Raw accelerometer + gyro measurements

// Status & Other Blocks
constexpr uint16_t ID_RECEIVERSTATUS    = 4014;
constexpr uint16_t ID_QUALITYIND        = 4001;
constexpr uint16_t ID_RXSETUP           = 4012;
constexpr uint16_t ID_CHANSTATUS        = 4013;

// =============================================================================
// SBF Block Names
// =============================================================================
// Used in setSBFOutput command
constexpr const char* BLOCK_MEASEPOCH = "MeasEpoch";

// Raw navigation subframe block names (decoded by RTKLIB input_sbf())
constexpr const char* BLOCK_GPSNAV_RAW    = "GPSRawCA";
constexpr const char* BLOCK_GLONAV_RAW    = "GLORawCA";
constexpr const char* BLOCK_GALNAV_RAW    = "GALRawINAV+GALRawFNAV";
constexpr const char* BLOCK_BDSNAV_RAW    = "BDSRaw";
constexpr const char* BLOCK_QZSNAV_RAW    = "QZSRawL1CA";
constexpr const char* BLOCK_NAVICNAV_RAW  = "NAVICRaw";

// Decoded navigation block names (receiver-assembled, complete ephemeris per block)
constexpr const char* BLOCK_GPSNAV    = "GPSNav";
constexpr const char* BLOCK_GLONAV    = "GLONav";
constexpr const char* BLOCK_GALNAV    = "GALNav";
constexpr const char* BLOCK_BDSNAV    = "BDSNav";
constexpr const char* BLOCK_QZSNAV    = "QZSNav";
constexpr const char* BLOCK_NAVICNAV  = "NavICNav";

constexpr const char* BLOCK_PVTGEODETIC = "PVTGeodetic";
constexpr const char* BLOCK_POSCOVGEODETIC = "PosCovGeodetic";
constexpr const char* BLOCK_PVTCARTESIAN = "PVTCartesian";
constexpr const char* BLOCK_POSCOVCARTESIAN = "PosCovCartesian";
constexpr const char* BLOCK_EXTSENSORMEAS = "ExtSensorMeas";
constexpr const char* BLOCK_RECEIVERSTATUS = "ReceiverStatus";
constexpr const char* BLOCK_QUALITYIND = "QualityInd";

// =============================================================================
// SBF Command Strings
// =============================================================================
constexpr const char* CMD_SET_SBF_OUTPUT  = "sso"; // setSBFOutput
constexpr const char* CMD_SET_NMEA_OUTPUT = "sno"; // setNMEAOutput

// =============================================================================
// Stream Type Definition
// =============================================================================
struct StreamTypeDef {
  std::string_view prefix;
  int type;
};

/// Supported stream type prefixes
constexpr StreamTypeDef kStreamTypes[] = {
    {"tcpcli://", STR_TCPCLI},
    {"serial://", STR_SERIAL},
    {"ntrip://", STR_NTRIPCLI},
    {"file://", STR_FILE},
};

// =============================================================================
// Utility Functions
// =============================================================================

/// @brief Get the SBF interval string for a given rate in Hz
/// @param rate_hz Publish rate in Hz
/// @return Interval string (e.g., "sec1", "msec100")
inline std::string getIntervalString(int rate_hz) {
    if (rate_hz >= 100) return "msec10";
    if (rate_hz >= 20)  return "msec50";
    if (rate_hz >= 10)  return "msec100";
    if (rate_hz >= 5)   return "msec200";
    if (rate_hz >= 2)   return "msec500";
    return "sec1"; // Default to 1Hz
}

// =============================================================================
// Binary PVT parsers (PVTGeodetic / PosCovGeodetic / PVTCartesian / PosCovCartesian)
// =============================================================================
namespace pvt {

// Common SBF block body header: TOW(u4 ms) + WNc(u2) + Mode(u1) + Error(u1)
constexpr int SBF_BODY_OFFSET_TOW   = 0;
constexpr int SBF_BODY_OFFSET_WNC   = 4;
constexpr int SBF_BODY_OFFSET_MODE  = 6;

// PVTGeodetic body field offsets
constexpr int PVT_GEO_OFFSET_LAT     = 8;    // f8 [rad]
constexpr int PVT_GEO_OFFSET_LON     = 16;   // f8 [rad]
constexpr int PVT_GEO_OFFSET_HGT     = 24;   // f8 [m]
constexpr int PVT_GEO_OFFSET_VN      = 36;   // f4 [m/s]
constexpr int PVT_GEO_OFFSET_VE      = 40;   // f4 [m/s]
constexpr int PVT_GEO_OFFSET_VU      = 44;   // f4 [m/s]
constexpr int PVT_GEO_OFFSET_NRSV    = 66;   // u1
constexpr int PVT_GEO_MIN_LEN        = 67;

// PVTCartesian body field offsets
constexpr int PVT_XYZ_OFFSET_X       = 8;    // f8 [m]
constexpr int PVT_XYZ_OFFSET_Y       = 16;
constexpr int PVT_XYZ_OFFSET_Z       = 24;
constexpr int PVT_XYZ_OFFSET_VX      = 36;   // f4 [m/s]
constexpr int PVT_XYZ_OFFSET_VY      = 40;
constexpr int PVT_XYZ_OFFSET_VZ      = 44;
constexpr int PVT_XYZ_OFFSET_NRSV    = 66;   // u1
constexpr int PVT_XYZ_MIN_LEN        = 67;

// PosCovGeodetic body field offsets (variances/covariances in m²; lat≡N, lon≡E, hgt≡U)
constexpr int POSCOV_GEO_OFFSET_LATLAT = 8;
constexpr int POSCOV_GEO_OFFSET_LONLON = 12;
constexpr int POSCOV_GEO_OFFSET_HGTHGT = 16;
constexpr int POSCOV_GEO_OFFSET_LATLON = 24;
constexpr int POSCOV_GEO_OFFSET_LATHGT = 28;
constexpr int POSCOV_GEO_OFFSET_LONHGT = 36;
constexpr int POSCOV_GEO_MIN_LEN       = 48;

// PosCovCartesian body field offsets
constexpr int POSCOV_XYZ_OFFSET_XX = 8;
constexpr int POSCOV_XYZ_OFFSET_YY = 12;
constexpr int POSCOV_XYZ_OFFSET_ZZ = 16;
constexpr int POSCOV_XYZ_OFFSET_XY = 24;
constexpr int POSCOV_XYZ_OFFSET_XZ = 28;
constexpr int POSCOV_XYZ_OFFSET_YZ = 36;
constexpr int POSCOV_XYZ_MIN_LEN   = 48;

// SBF Mode field (lower 4 bits = PVT solution type)
constexpr uint8_t MODE_NO_PVT       = 0;
constexpr uint8_t MODE_STANDALONE   = 1;
constexpr uint8_t MODE_DIFFERENTIAL = 2;
constexpr uint8_t MODE_FIXED_LOC    = 3;
constexpr uint8_t MODE_RTK_FIXED    = 4;
constexpr uint8_t MODE_RTK_FLOAT    = 5;
constexpr uint8_t MODE_SBAS_AIDED   = 6;
constexpr uint8_t MODE_MOVING_BASE  = 7;
constexpr uint8_t MODE_PRECISE      = 8;
constexpr uint8_t MODE_PPP          = 10;

template <typename T>
inline T read_le(const uint8_t* p) {
  T v;
  std::memcpy(&v, p, sizeof(T));
  return v;
}

inline uint8_t mapModeToStatus(uint8_t mode_byte) {
  using S = gnss_ros_standardization::msg::GnssSolution;
  const uint8_t mode = mode_byte & 0x0F;
  switch (mode) {
    case MODE_NO_PVT:       return S::STATUS_NONE;
    case MODE_STANDALONE:   return S::STATUS_SINGLE;
    case MODE_DIFFERENTIAL: return S::STATUS_DGPS;
    case MODE_RTK_FIXED:
    case MODE_MOVING_BASE:  return S::STATUS_FIX;
    case MODE_RTK_FLOAT:    return S::STATUS_FLOAT;
    case MODE_SBAS_AIDED:   return S::STATUS_SBAS;
    case MODE_PRECISE:
    case MODE_PPP:          return S::STATUS_PPP;
    default:                return S::STATUS_NONE;
  }
}

/// Read the TOW (in ms) from any SBF block body. Returns 0 if too short.
inline uint32_t getTowMs(const uint8_t* p, size_t len) {
  if (len < 4) return 0;
  return read_le<uint32_t>(p + SBF_BODY_OFFSET_TOW);
}

inline uint16_t getWeek(const uint8_t* p, size_t len) {
  if (len < 6) return 0;
  return read_le<uint16_t>(p + SBF_BODY_OFFSET_WNC);
}

/// Parse PVTGeodetic body into GnssSolution.
/// Fills: status, num_sats, time_tow, time_week, latitude, longitude, altitude, vel_enu.
inline bool parsePVTGeodetic(const uint8_t* p, size_t len,
                             gnss_ros_standardization::msg::GnssSolution& out) {
  if (len < static_cast<size_t>(PVT_GEO_MIN_LEN)) return false;

  const uint32_t tow_ms = read_le<uint32_t>(p + SBF_BODY_OFFSET_TOW);
  const uint16_t wnc    = read_le<uint16_t>(p + SBF_BODY_OFFSET_WNC);
  const uint8_t  mode   = p[SBF_BODY_OFFSET_MODE];

  const double lat_rad = read_le<double>(p + PVT_GEO_OFFSET_LAT);
  const double lon_rad = read_le<double>(p + PVT_GEO_OFFSET_LON);
  const double hgt     = read_le<double>(p + PVT_GEO_OFFSET_HGT);
  const float  vn      = read_le<float>(p + PVT_GEO_OFFSET_VN);
  const float  ve      = read_le<float>(p + PVT_GEO_OFFSET_VE);
  const float  vu      = read_le<float>(p + PVT_GEO_OFFSET_VU);
  const uint8_t nr_sv  = p[PVT_GEO_OFFSET_NRSV];

  constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;
  out.status    = mapModeToStatus(mode);
  out.num_sats  = nr_sv;
  out.time_tow  = static_cast<double>(tow_ms) * 1e-3;
  out.time_week = wnc;
  out.latitude  = lat_rad * kRad2Deg;
  out.longitude = lon_rad * kRad2Deg;
  out.altitude  = hgt;
  out.vel_enu.x = static_cast<double>(ve);
  out.vel_enu.y = static_cast<double>(vn);
  out.vel_enu.z = static_cast<double>(vu);
  return true;
}

/// Parse PVTCartesian body into GnssSolution.
/// Fills: status, num_sats, time_tow, time_week, pos_ecef, vel_ecef.
inline bool parsePVTCartesian(const uint8_t* p, size_t len,
                              gnss_ros_standardization::msg::GnssSolution& out) {
  if (len < static_cast<size_t>(PVT_XYZ_MIN_LEN)) return false;

  const uint32_t tow_ms = read_le<uint32_t>(p + SBF_BODY_OFFSET_TOW);
  const uint16_t wnc    = read_le<uint16_t>(p + SBF_BODY_OFFSET_WNC);
  const uint8_t  mode   = p[SBF_BODY_OFFSET_MODE];

  const double x  = read_le<double>(p + PVT_XYZ_OFFSET_X);
  const double y  = read_le<double>(p + PVT_XYZ_OFFSET_Y);
  const double z  = read_le<double>(p + PVT_XYZ_OFFSET_Z);
  const float  vx = read_le<float>(p + PVT_XYZ_OFFSET_VX);
  const float  vy = read_le<float>(p + PVT_XYZ_OFFSET_VY);
  const float  vz = read_le<float>(p + PVT_XYZ_OFFSET_VZ);
  const uint8_t nr_sv = p[PVT_XYZ_OFFSET_NRSV];

  out.status     = mapModeToStatus(mode);
  out.num_sats   = nr_sv;
  out.time_tow   = static_cast<double>(tow_ms) * 1e-3;
  out.time_week  = wnc;
  out.pos_ecef.x = x;
  out.pos_ecef.y = y;
  out.pos_ecef.z = z;
  out.vel_ecef.x = vx;
  out.vel_ecef.y = vy;
  out.vel_ecef.z = vz;
  return true;
}

/// Parse PosCovGeodetic body into GnssSolution's pos_enu_cov (row-major E,N,U).
/// Septentrio convention: lat≡N, lon≡E, hgt≡U.
inline bool parsePosCovGeodetic(const uint8_t* p, size_t len,
                                gnss_ros_standardization::msg::GnssSolution& out) {
  if (len < static_cast<size_t>(POSCOV_GEO_MIN_LEN)) return false;
  const float c_lat_lat = read_le<float>(p + POSCOV_GEO_OFFSET_LATLAT);
  const float c_lon_lon = read_le<float>(p + POSCOV_GEO_OFFSET_LONLON);
  const float c_hgt_hgt = read_le<float>(p + POSCOV_GEO_OFFSET_HGTHGT);
  const float c_lat_lon = read_le<float>(p + POSCOV_GEO_OFFSET_LATLON);
  const float c_lat_hgt = read_le<float>(p + POSCOV_GEO_OFFSET_LATHGT);
  const float c_lon_hgt = read_le<float>(p + POSCOV_GEO_OFFSET_LONHGT);

  // E↔lon, N↔lat, U↔hgt
  out.pos_enu_cov[0] = c_lon_lon;  // EE
  out.pos_enu_cov[1] = c_lat_lon;  // EN
  out.pos_enu_cov[2] = c_lon_hgt;  // EU
  out.pos_enu_cov[3] = c_lat_lon;  // NE
  out.pos_enu_cov[4] = c_lat_lat;  // NN
  out.pos_enu_cov[5] = c_lat_hgt;  // NU
  out.pos_enu_cov[6] = c_lon_hgt;  // UE
  out.pos_enu_cov[7] = c_lat_hgt;  // UN
  out.pos_enu_cov[8] = c_hgt_hgt;  // UU
  return true;
}

/// Parse PosCovCartesian body into GnssSolution's pos_cov_ecef (row-major).
inline bool parsePosCovCartesian(const uint8_t* p, size_t len,
                                 gnss_ros_standardization::msg::GnssSolution& out) {
  if (len < static_cast<size_t>(POSCOV_XYZ_MIN_LEN)) return false;
  const float c_xx = read_le<float>(p + POSCOV_XYZ_OFFSET_XX);
  const float c_yy = read_le<float>(p + POSCOV_XYZ_OFFSET_YY);
  const float c_zz = read_le<float>(p + POSCOV_XYZ_OFFSET_ZZ);
  const float c_xy = read_le<float>(p + POSCOV_XYZ_OFFSET_XY);
  const float c_xz = read_le<float>(p + POSCOV_XYZ_OFFSET_XZ);
  const float c_yz = read_le<float>(p + POSCOV_XYZ_OFFSET_YZ);

  out.pos_cov_ecef[0] = c_xx;
  out.pos_cov_ecef[1] = c_xy;
  out.pos_cov_ecef[2] = c_xz;
  out.pos_cov_ecef[3] = c_xy;
  out.pos_cov_ecef[4] = c_yy;
  out.pos_cov_ecef[5] = c_yz;
  out.pos_cov_ecef[6] = c_xz;
  out.pos_cov_ecef[7] = c_yz;
  out.pos_cov_ecef[8] = c_zz;
  return true;
}

}  // namespace pvt

}  // namespace sbf
}  // namespace gnss_ros_standardization
