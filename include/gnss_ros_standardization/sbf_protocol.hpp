#pragma once
#include "gnss_ros_standardization/receiver_diagnostics.h"
/// @file sbf_protocol.hpp
/// @brief Septentrio SBF protocol constants and utility types.
///
/// This header centralizes all SBF protocol definitions used by the driver and decoder nodes.

#include <algorithm>
#include <cmath>
#include <limits>
#include "gnss_ros_standardization/measurement_validation.hpp"
#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

// Include rtklib.h for STR_ constants
#include "gnss_ros_standardization/rtklib_compat.hpp"

#include "gnss_ros_standardization/msg/gnss_solution.hpp"

namespace gnss_ros_standardization {
namespace sbf {

// Constants

constexpr size_t READ_BUFFER_SIZE = 4096;
constexpr size_t NMEA_MAX_LINE_LEN = 256;

// SBF sync bytes — every SBF block starts with these two bytes
constexpr uint8_t SBF_SYNC1 = 0x24;  // '$'
constexpr uint8_t SBF_SYNC2 = 0x40;  // '@'

// ExtSensorMeas (ID 4050) layout
// Body: TOW(u4, ms) WNc(u2) N(u1) SBLength(u1) [N × ExtSensorMeasSub of size SBLength]
//
// ExtSensorMeasSub (SBLength = 28 bytes):
//   Source(u1) SensorModel(u1) Type(u1) ObsInfo(u1) X(f8) Y(f8) Z(f8)
//
// Each Type carries a full 3-axis vector (Type 0 = accel 3-vector, Type 1 = gyro 3-vector).
namespace ext_sensor_meas {
  constexpr int OFFSET_N          = 6;
  constexpr int OFFSET_SB_LENGTH  = 7;
  constexpr int OFFSET_SUBBLOCKS  = 8;
  constexpr int SB_OFFSET_TYPE    = 2;   // Type field
  constexpr int SB_OFFSET_X       = 4;   // float64 X-axis
  constexpr int SB_OFFSET_Y       = 12;  // float64 Y-axis
  constexpr int SB_OFFSET_Z       = 20;  // float64 Z-axis
  constexpr int SB_MIN_LEN        = 28;
  constexpr int MIN_BODY_LEN      = 8;
  constexpr uint8_t TYPE_ACCEL    = 0;   // Accelerations [m/s²]
  constexpr uint8_t TYPE_GYRO     = 1;   // Wire angular rates [degree/s]; ROS uses rad/s
}

// SBF Block IDs
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
constexpr uint16_t ID_NAVICNAV    = 4254;   // SBF NavICLNav (decoded ephemeris).
                                            // 4099 is ID_GNSSSymbols, not a nav block.

// PVT Blocks
constexpr uint16_t ID_PVTCARTESIAN      = 4006;
constexpr uint16_t ID_PVTGEODETIC       = 4007;
constexpr uint16_t ID_POSCOVCARTESIAN   = 5905;
constexpr uint16_t ID_POSCOVGEODETIC    = 5906;
constexpr uint16_t ID_VELCOVCARTESIAN   = 5907;
constexpr uint16_t ID_VELCOVGEODETIC    = 5908;
constexpr uint16_t ID_DOP               = 4001;

// INS / IMU Blocks (AsteRx-i / mosaic-X5 with integrated IMU)
constexpr uint16_t ID_EXTSENSORMEAS     = 4050;  // Raw accelerometer + gyro measurements

// Status & Other Blocks
constexpr uint16_t ID_RECEIVERSTATUS    = 4014;
constexpr uint16_t ID_RXSETUP           = 4012;
constexpr uint16_t ID_CHANSTATUS        = 4013;

// SBF Block Names
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

constexpr const char* BLOCK_PVTGEODETIC = "PVTGeodetic";
constexpr const char* BLOCK_POSCOVGEODETIC = "PosCovGeodetic";
constexpr const char* BLOCK_VELCOVGEODETIC = "VelCovGeodetic";
constexpr const char* BLOCK_PVTCARTESIAN = "PVTCartesian";
constexpr const char* BLOCK_POSCOVCARTESIAN = "PosCovCartesian";
constexpr const char* BLOCK_VELCOVCARTESIAN = "VelCovCartesian";
constexpr const char* BLOCK_DOP = "DOP";
constexpr const char* BLOCK_EXTSENSORMEAS = "ExtSensorMeas";
constexpr const char* BLOCK_RECEIVERSTATUS = "ReceiverStatus";

// SBF Command Strings
constexpr const char* CMD_SET_SBF_OUTPUT  = "sso"; // setSBFOutput
constexpr const char* CMD_SET_NMEA_OUTPUT = "sno"; // setNMEAOutput

// Stream Type Definition
struct StreamTypeDef {
  std::string_view prefix;
  int type;
};

/// Supported stream type prefixes
constexpr StreamTypeDef kStreamTypes[] = {
    {"tcpcli://", STR_TCPCLI},
    {"tcpsvr://", STR_TCPSVR},
    {"serial://", STR_SERIAL},
    {"ntrip://", STR_NTRIPCLI},
    {"file://", STR_FILE},
};

// Utility Functions

/// @brief Get the SBF interval string for a given rate in Hz
/// @param rate_hz Publish rate in Hz
/// @return Interval string (e.g., "sec1", "msec100")
inline std::string getIntervalString(int rate_hz) {
    if (rate_hz >= 200) return "msec5";
    if (rate_hz >= 100) return "msec10";
    if (rate_hz >= 50)  return "msec20";
    if (rate_hz >= 25)  return "msec40";
    if (rate_hz >= 20)  return "msec50";
    if (rate_hz >= 10)  return "msec100";
    if (rate_hz >= 5)   return "msec200";
    if (rate_hz >= 2)   return "msec500";
    return "sec1"; // Default to 1Hz
}

// Binary PVT parsers (PVTGeodetic / PosCovGeodetic / PVTCartesian / PosCovCartesian)
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

// VelCovGeodetic body field offsets (variances in (m/s)²; Vn≡N, Ve≡E, Vu≡U)
constexpr int VELCOV_GEO_OFFSET_VNVN = 8;
constexpr int VELCOV_GEO_OFFSET_VEVE = 12;
constexpr int VELCOV_GEO_OFFSET_VUVU = 16;
constexpr int VELCOV_GEO_OFFSET_VNVE = 24;
constexpr int VELCOV_GEO_OFFSET_VNVU = 28;
constexpr int VELCOV_GEO_OFFSET_VEVU = 36;
constexpr int VELCOV_GEO_MIN_LEN     = 48;

// VelCovCartesian body field offsets
constexpr int VELCOV_XYZ_OFFSET_VXVX = 8;
constexpr int VELCOV_XYZ_OFFSET_VYVY = 12;
constexpr int VELCOV_XYZ_OFFSET_VZVZ = 16;
constexpr int VELCOV_XYZ_OFFSET_VXVY = 24;
constexpr int VELCOV_XYZ_OFFSET_VXVZ = 28;
constexpr int VELCOV_XYZ_OFFSET_VYVZ = 36;
constexpr int VELCOV_XYZ_MIN_LEN     = 48;

// DOP body field offsets (Septentrio block 4001).
// Layout: TOW(u4)+WNc(u2)+NrSV(u1)+Reserved(u1)+PDOP(u2)+TDOP(u2)+HDOP(u2)+VDOP(u2)+HPL(f4)+VPL(f4)
// PDOP/TDOP/HDOP/VDOP are u16 scaled by 0.01; 0xFFFF = Not-Available.
constexpr int DOP_OFFSET_PDOP = 8;
constexpr int DOP_OFFSET_TDOP = 10;
constexpr int DOP_OFFSET_HDOP = 12;
constexpr int DOP_OFFSET_VDOP = 14;
constexpr int DOP_MIN_LEN     = 16;
constexpr uint16_t DOP_NOT_AVAILABLE = 0xFFFF;

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
    case MODE_STANDALONE:
    case 3: /* fixed location, not a measured RTK fix */ return S::STATUS_SINGLE;
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

// A PVT block with Mode "no PVT" or a non-zero Error is a timed epoch without
// a solution: it is published as STATUS_NONE (positions become NaN downstream),
// not dropped. Returns false when the epoch itself is not usable.
inline bool pvtEpochWithoutSolution(const uint8_t* p, uint32_t tow_ms, uint16_t wnc,
                                    gnss_ros_standardization::msg::GnssSolution& out,
                                    bool& no_solution) {
  if (wnc == 0 || wnc == 0xFFFF || tow_ms >= 604800000u) return GNSS_RX_REJECT(GNSS_RX_TIME);
  no_solution = (p[SBF_BODY_OFFSET_MODE] & 0x0F) == MODE_NO_PVT || p[7] != 0;
  if (no_solution) {
    GNSS_RX_EVENT(GNSS_RX_QUALITY, 1);
    out.status    = gnss_ros_standardization::msg::GnssSolution::STATUS_NONE;
    out.num_sats  = 0;
    out.time_tow  = static_cast<double>(tow_ms) * 1e-3;
    out.time_week = wnc;
  }
  return true;
}

/// Parse PVTGeodetic body into GnssSolution.
/// Fills: status, num_sats, time_tow, time_week, latitude, longitude, altitude, vel_enu.
/// Velocity Do-Not-Use leaves the position valid and the velocity NaN.
inline bool parsePVTGeodetic(const uint8_t* p, size_t len,
                             gnss_ros_standardization::msg::GnssSolution& out) {
  if (len < static_cast<size_t>(PVT_GEO_MIN_LEN)) return GNSS_RX_REJECT(GNSS_RX_LENGTH);

  const uint32_t tow_ms = read_le<uint32_t>(p + SBF_BODY_OFFSET_TOW);
  const uint16_t wnc    = read_le<uint16_t>(p + SBF_BODY_OFFSET_WNC);
  const uint8_t  mode   = p[SBF_BODY_OFFSET_MODE];
  bool no_solution = false;
  if (!pvtEpochWithoutSolution(p, tow_ms, wnc, out, no_solution)) return false;
  if (no_solution) return true;

  const double lat_rad = read_le<double>(p + PVT_GEO_OFFSET_LAT);
  const double lon_rad = read_le<double>(p + PVT_GEO_OFFSET_LON);
  const double hgt     = read_le<double>(p + PVT_GEO_OFFSET_HGT);
  const float  vn      = read_le<float>(p + PVT_GEO_OFFSET_VN);
  const float  ve      = read_le<float>(p + PVT_GEO_OFFSET_VE);
  const float  vu      = read_le<float>(p + PVT_GEO_OFFSET_VU);
  const uint8_t nr_sv  = p[PVT_GEO_OFFSET_NRSV];

  if (!std::isfinite(lat_rad) || !std::isfinite(lon_rad) ||
      std::abs(lat_rad) > 1.5707963267948966 || std::abs(lon_rad) > 3.141592653589793 ||
      !std::isfinite(hgt) || hgt == -2e10) return GNSS_RX_REJECT(GNSS_RX_VALUE);
  const bool vel_ok = std::isfinite(vn) && std::isfinite(ve) && std::isfinite(vu) &&
                      vn != -2e10f && ve != -2e10f && vu != -2e10f;
  constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;
  out.status    = mapModeToStatus(mode);
  out.num_sats  = nr_sv;
  out.time_tow  = static_cast<double>(tow_ms) * 1e-3;
  out.time_week = wnc;
  out.latitude  = lat_rad * kRad2Deg;
  out.longitude = lon_rad * kRad2Deg;
  out.altitude  = hgt;
  if (vel_ok) {
    out.vel_enu.x = static_cast<double>(ve);
    out.vel_enu.y = static_cast<double>(vn);
    out.vel_enu.z = static_cast<double>(vu);
  } else {
    out.vel_enu.x = out.vel_enu.y = out.vel_enu.z = std::numeric_limits<double>::quiet_NaN();
  }
  return true;
}

/// Parse PVTCartesian body into GnssSolution.
/// Fills: status, num_sats, time_tow, time_week, pos_ecef, vel_ecef.
inline bool parsePVTCartesian(const uint8_t* p, size_t len,
                              gnss_ros_standardization::msg::GnssSolution& out) {
  if (len < static_cast<size_t>(PVT_XYZ_MIN_LEN)) return GNSS_RX_REJECT(GNSS_RX_LENGTH);

  const uint32_t tow_ms = read_le<uint32_t>(p + SBF_BODY_OFFSET_TOW);
  const uint16_t wnc    = read_le<uint16_t>(p + SBF_BODY_OFFSET_WNC);
  const uint8_t  mode   = p[SBF_BODY_OFFSET_MODE];
  bool no_solution = false;
  if (!pvtEpochWithoutSolution(p, tow_ms, wnc, out, no_solution)) return false;
  if (no_solution) return true;

  const double x  = read_le<double>(p + PVT_XYZ_OFFSET_X);
  const double y  = read_le<double>(p + PVT_XYZ_OFFSET_Y);
  const double z  = read_le<double>(p + PVT_XYZ_OFFSET_Z);
  const float  vx = read_le<float>(p + PVT_XYZ_OFFSET_VX);
  const float  vy = read_le<float>(p + PVT_XYZ_OFFSET_VY);
  const float  vz = read_le<float>(p + PVT_XYZ_OFFSET_VZ);
  const uint8_t nr_sv = p[PVT_XYZ_OFFSET_NRSV];

  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
      x == -2e10 || y == -2e10 || z == -2e10) return GNSS_RX_REJECT(GNSS_RX_VALUE);
  const bool vel_ok = std::isfinite(vx) && std::isfinite(vy) && std::isfinite(vz) &&
                      vx != -2e10f && vy != -2e10f && vz != -2e10f;
  out.status     = mapModeToStatus(mode);
  out.num_sats   = nr_sv;
  out.time_tow   = static_cast<double>(tow_ms) * 1e-3;
  out.time_week  = wnc;
  out.pos_ecef.x = x;
  out.pos_ecef.y = y;
  out.pos_ecef.z = z;
  if (vel_ok) {
    out.vel_ecef.x = vx;
    out.vel_ecef.y = vy;
    out.vel_ecef.z = vz;
  } else {
    out.vel_ecef.x = out.vel_ecef.y = out.vel_ecef.z = std::numeric_limits<double>::quiet_NaN();
  }
  return true;
}

inline bool validCovarianceBlock(const uint8_t* p) {
  if (p[7] != 0 || (p[6] & 0x0f) == 0 || read_le<uint16_t>(p + 4) == 65535 ||
      read_le<uint32_t>(p) >= 604800000) return GNSS_RX_REJECT(p[7]!=0 || (p[6]&0x0f)==0 ? GNSS_RX_QUALITY : GNSS_RX_TIME);
  for (int offset : {8, 12, 16, 24, 28, 36})
    if (read_le<float>(p + offset) == -2e10f) return GNSS_RX_REJECT(GNSS_RX_VALUE);
  return validation::covariance(read_le<float>(p + 8), read_le<float>(p + 12),
      read_le<float>(p + 16), read_le<float>(p + 24), read_le<float>(p + 28),
      read_le<float>(p + 36)) || GNSS_RX_REJECT(GNSS_RX_VALUE);
}

/// Parse PosCovGeodetic body into GnssSolution's pos_enu_cov (row-major E,N,U).
/// Septentrio convention: lat≡N, lon≡E, hgt≡U.
inline bool parsePosCovGeodetic(const uint8_t* p, size_t len,
                                gnss_ros_standardization::msg::GnssSolution& out) {
  if (len < static_cast<size_t>(POSCOV_GEO_MIN_LEN)) return GNSS_RX_REJECT(GNSS_RX_LENGTH);
  if (!validCovarianceBlock(p)) return false;
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
  if (len < static_cast<size_t>(POSCOV_XYZ_MIN_LEN)) return GNSS_RX_REJECT(GNSS_RX_LENGTH);
  if (!validCovarianceBlock(p)) return false;
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

/// Parse VelCovGeodetic body into GnssSolution's vel_enu_cov (row-major E,N,U).
/// Septentrio convention: Vn≡N, Ve≡E, Vu≡U.
inline bool parseVelCovGeodetic(const uint8_t* p, size_t len,
                                gnss_ros_standardization::msg::GnssSolution& out) {
  if (len < static_cast<size_t>(VELCOV_GEO_MIN_LEN)) return GNSS_RX_REJECT(GNSS_RX_LENGTH);
  if (!validCovarianceBlock(p)) return false;
  const float c_vn_vn = read_le<float>(p + VELCOV_GEO_OFFSET_VNVN);
  const float c_ve_ve = read_le<float>(p + VELCOV_GEO_OFFSET_VEVE);
  const float c_vu_vu = read_le<float>(p + VELCOV_GEO_OFFSET_VUVU);
  const float c_vn_ve = read_le<float>(p + VELCOV_GEO_OFFSET_VNVE);
  const float c_vn_vu = read_le<float>(p + VELCOV_GEO_OFFSET_VNVU);
  const float c_ve_vu = read_le<float>(p + VELCOV_GEO_OFFSET_VEVU);

  // E↔Ve, N↔Vn, U↔Vu
  out.vel_enu_cov[0] = c_ve_ve;  // EE
  out.vel_enu_cov[1] = c_vn_ve;  // EN
  out.vel_enu_cov[2] = c_ve_vu;  // EU
  out.vel_enu_cov[3] = c_vn_ve;  // NE
  out.vel_enu_cov[4] = c_vn_vn;  // NN
  out.vel_enu_cov[5] = c_vn_vu;  // NU
  out.vel_enu_cov[6] = c_ve_vu;  // UE
  out.vel_enu_cov[7] = c_vn_vu;  // UN
  out.vel_enu_cov[8] = c_vu_vu;  // UU
  return true;
}

/// Parse VelCovCartesian body into GnssSolution's vel_cov_ecef (row-major).
inline bool parseVelCovCartesian(const uint8_t* p, size_t len,
                                 gnss_ros_standardization::msg::GnssSolution& out) {
  if (len < static_cast<size_t>(VELCOV_XYZ_MIN_LEN)) return GNSS_RX_REJECT(GNSS_RX_LENGTH);
  if (!validCovarianceBlock(p)) return false;
  const float c_vx_vx = read_le<float>(p + VELCOV_XYZ_OFFSET_VXVX);
  const float c_vy_vy = read_le<float>(p + VELCOV_XYZ_OFFSET_VYVY);
  const float c_vz_vz = read_le<float>(p + VELCOV_XYZ_OFFSET_VZVZ);
  const float c_vx_vy = read_le<float>(p + VELCOV_XYZ_OFFSET_VXVY);
  const float c_vx_vz = read_le<float>(p + VELCOV_XYZ_OFFSET_VXVZ);
  const float c_vy_vz = read_le<float>(p + VELCOV_XYZ_OFFSET_VYVZ);

  out.vel_cov_ecef[0] = c_vx_vx;
  out.vel_cov_ecef[1] = c_vx_vy;
  out.vel_cov_ecef[2] = c_vx_vz;
  out.vel_cov_ecef[3] = c_vx_vy;
  out.vel_cov_ecef[4] = c_vy_vy;
  out.vel_cov_ecef[5] = c_vy_vz;
  out.vel_cov_ecef[6] = c_vx_vz;
  out.vel_cov_ecef[7] = c_vy_vz;
  out.vel_cov_ecef[8] = c_vz_vz;
  return true;
}

/// Parse DOP body. Fills gdop/pdop/hdop/vdop. GDOP is derived as sqrt(PDOP²+TDOP²)
/// since the Septentrio DOP block exposes only PDOP/TDOP/HDOP/VDOP.
inline bool parseDop(const uint8_t* p, size_t len,
                     gnss_ros_standardization::msg::GnssSolution& out) {
  if (len < static_cast<size_t>(DOP_MIN_LEN)) return GNSS_RX_REJECT(GNSS_RX_LENGTH);
  if (read_le<uint16_t>(p + 4) == 65535 || read_le<uint32_t>(p) >= 604800000) return GNSS_RX_REJECT(GNSS_RX_TIME);
  const uint16_t pdop_raw = read_le<uint16_t>(p + DOP_OFFSET_PDOP);
  const uint16_t tdop_raw = read_le<uint16_t>(p + DOP_OFFSET_TDOP);
  const uint16_t hdop_raw = read_le<uint16_t>(p + DOP_OFFSET_HDOP);
  const uint16_t vdop_raw = read_le<uint16_t>(p + DOP_OFFSET_VDOP);

  const float pdop = (pdop_raw == DOP_NOT_AVAILABLE) ? std::numeric_limits<float>::quiet_NaN() : pdop_raw * 0.01f;
  const float tdop = (tdop_raw == DOP_NOT_AVAILABLE) ? std::numeric_limits<float>::quiet_NaN() : tdop_raw * 0.01f;
  const float hdop = (hdop_raw == DOP_NOT_AVAILABLE) ? std::numeric_limits<float>::quiet_NaN() : hdop_raw * 0.01f;
  const float vdop = (vdop_raw == DOP_NOT_AVAILABLE) ? std::numeric_limits<float>::quiet_NaN() : vdop_raw * 0.01f;

  out.pdop = pdop;
  out.hdop = hdop;
  out.vdop = vdop;
  out.gdop = std::hypot(pdop, tdop);
  return true;
}

}  // namespace pvt

}  // namespace sbf
}  // namespace gnss_ros_standardization

// ---- sbf navigation ----
#include "gnss_ros_standardization/binary_framer.hpp"

namespace gnss_ros_standardization {
// No model/provenance side channel is part of the ROS navigation contract.
// Do not publish CNAV as indistinguishable legacy eph_t, even with zero rates.
// Keep this gate before input_sbf so rejected frames cannot alter cached nav.
inline bool supportedSbfNavigation(const wire::Frame& f) {
  switch (f.id) {
    case 4254:  // NavICLNav is disabled in the normal backend build.
    case 4042: case 4258: case 4251: case 4252: case 4253: return false;
    case 4004: return f.payload.size() >= (f.revision == 0 ? 88u : 92u);
    default: return true;
  }
}
}  // namespace gnss_ros_standardization

// ---- sbf observation completion ----
#include <chrono>
#include <cmath>
#include <stdexcept>
extern "C" {
#include "gnss_ros_standardization/rtklib_compat.hpp"
// Defined in the narrowly patched build-tree copy of septentrio.c.
int gnss_flush_sbf_observations(raw_t* raw);
}

namespace gnss_ros_standardization {
// MeasEpoch is a complete, CRC-checked measurement block. Wait for its optional
// MeasExtra/EndOfMeas, but release it after a quiet period even at EOF or a lost
// connection. Meas3Ranges follows the same deadline; optional blocks only extend
// a pending validated epoch. A partial wire frame is never a measurement.
class SbfObservationCompletion {
 public:
  using Clock = std::chrono::steady_clock;
  void configure(double seconds) {
    if (!std::isfinite(seconds) || seconds <= 0 || seconds > 5)
      throw std::invalid_argument("observation.assembly_timeout_s must be in (0, 5]");
    timeout_ = std::chrono::duration<double>(seconds);
  }
  void decoded(uint16_t id, const raw_t& raw, Clock::time_point now=Clock::now()) {
    if (id == 4027 || id == 4109) { armed_ = raw.obuf.n > 0; updated_ = now; }
    else if ((id == 4000 || id == 4110 || id == 4111) && armed_ &&
             raw.obuf.n > 0 && timediff(raw.time,raw.obuf.data[0].time)==0) updated_ = now;
    if (raw.obuf.n == 0) armed_ = false;
  }
  void reset() { armed_=false; }
  bool flushIfDue(raw_t& raw, Clock::time_point now=Clock::now()) {
    if (!armed_ || now-updated_ < timeout_) return false;
    armed_ = false;
    return gnss_flush_sbf_observations(&raw) == 1;
  }
 private:
  bool armed_{false};
  std::chrono::duration<double> timeout_{0.2};
  Clock::time_point updated_{};
};
} // namespace gnss_ros_standardization
