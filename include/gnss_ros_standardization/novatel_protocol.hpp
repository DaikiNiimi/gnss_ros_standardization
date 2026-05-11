#pragma once
/// @file novatel_protocol.hpp
/// @brief NovAtel OEM7/6/4 protocol constants and utility types.
///
/// This header centralizes all NovAtel protocol definitions used by the driver and decoder nodes.

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
namespace novatel {

// =============================================================================
// Constants
// =============================================================================

constexpr size_t READ_BUFFER_SIZE = 4096;

// =============================================================================
// NovAtel Message IDs (decimal)
// =============================================================================
// Reference: MALIB/src/rcv/novatel.c and OEM7 Commands and Logs Reference Manual

// Measurement
constexpr uint16_t ID_RANGECMP          = 140;
constexpr uint16_t ID_RANGE             = 43;
constexpr uint16_t ID_BESTPOS           = 42;
constexpr uint16_t ID_BESTVEL           = 99;

// Ephemeris / Navigation Data
constexpr uint16_t ID_RAWEPHEM          = 41;
constexpr uint16_t ID_IONUTC            = 8;
constexpr uint16_t ID_RAWWAASFRAME      = 287;
constexpr uint16_t ID_RAWSBASFRAME      = 973; // OEM7/6
constexpr uint16_t ID_GPSEPHEM          = 7;
constexpr uint16_t ID_GLOEPHEMERIS      = 723;
constexpr uint16_t ID_GALEPHEMERIS      = 1122;
constexpr uint16_t ID_GALINAVEPHEMERIS  = 1309;
constexpr uint16_t ID_GALIONO           = 1127;
constexpr uint16_t ID_GALCLOCK          = 1121;
constexpr uint16_t ID_QZSSRAWEPHEM      = 1331;
constexpr uint16_t ID_QZSSRAWSUBFRAME   = 1330;
constexpr uint16_t ID_QZSSEPHEMERIS     = 1336;
constexpr uint16_t ID_QZSSIONUTC        = 1347;
constexpr uint16_t ID_BDSEPHEMERIS      = 1696;
constexpr uint16_t ID_NAVICEPHEMERIS    = 2123;

// SPAN / IMU Message IDs (OEM7/6 with IMU connection)
constexpr uint16_t ID_RAWIMU            = 268;   // Raw IMU data (IMU-type-dependent scale, deprecated, use RAWIMUSX)
constexpr uint16_t ID_RAWIMUSX          = 1462;  // Raw IMU data with extended status (uncalibrated, IMU-type-dependent)
constexpr uint16_t ID_CORRIMUDATA       = 812;   // Bias/gravity/earth-rate corrected IMU data (SPAN required)

// OEM4 binary sync bytes
constexpr uint8_t OEM4_SYNC1 = 0xAA;
constexpr uint8_t OEM4_SYNC2 = 0x44;
constexpr uint8_t OEM4_SYNC3 = 0x12;
constexpr int     OEM4_HEADER_LEN = 28;  // Standard OEM4/7 binary header length

// OEM3 Message IDs
constexpr uint16_t ID_OEM3_RGEB         = 32;
constexpr uint16_t ID_OEM3_RGED         = 65;
constexpr uint16_t ID_OEM3_REPB         = 14;
constexpr uint16_t ID_OEM3_IONB         = 16;
constexpr uint16_t ID_OEM3_UTCB         = 17;
constexpr uint16_t ID_OEM3_FRMB         = 54;

// =============================================================================
// NovAtel Log Names (for LOG command)
// =============================================================================
constexpr const char* LOG_RANGECMP      = "RANGECMPB"; // Binary
constexpr const char* LOG_RANGE         = "RANGEB";    // Binary
constexpr const char* LOG_BESTPOS       = "BESTPOSB";  // Binary
constexpr const char* LOG_BESTVEL       = "BESTVELB";  // Binary

constexpr const char* LOG_RAWEPHEM      = "RAWEPHEMB";
constexpr const char* LOG_IONUTC        = "IONUTCB";
constexpr const char* LOG_RAWWAASFRAME  = "RAWWAASFRAMEB";
constexpr const char* LOG_GPSEPHEM      = "GPSEPHEMB";
constexpr const char* LOG_GLOEPHEMERIS  = "GLOEPHEMERISB";
constexpr const char* LOG_GALEPHEMERIS  = "GALEPHEMERISB";
constexpr const char* LOG_GALINAVEPHEMERIS = "GALINAVEPHEMERISB";
constexpr const char* LOG_GALIONO       = "GALIONOB";
constexpr const char* LOG_GALCLOCK      = "GALCLOCKB";
constexpr const char* LOG_QZSSEPHEMERIS = "QZSSEPHEMERISB";
constexpr const char* LOG_QZSSIONUTC    = "QZSSIONUTCB";
constexpr const char* LOG_BDSEPHEMERIS  = "BDSEPHEMERISB";
constexpr const char* LOG_NAVICEPHEMERIS = "NAVICEPHEMERISB";

// NMEA Log Names (standard sentences, talker-ID-agnostic naming)
constexpr const char* LOG_GPGGA         = "GPGGA";
constexpr const char* LOG_GPRMC         = "GPRMC";
constexpr const char* LOG_GPGSA         = "GPGSA";
constexpr const char* LOG_GPGST         = "GPGST";

// IMU Log Names
constexpr const char* LOG_RAWIMUSXB     = "RAWIMUSXB";    // Raw IMU (binary, ONNEW@IMU rate)
constexpr const char* LOG_CORRIMUDATAB  = "CORRIMUDATAB"; // Corrected IMU (binary, ONNEW@IMU rate, SPAN required)

// OEM3 Log Names
constexpr const char* LOG_OEM3_RGEB     = "RGEB"; // Range
constexpr const char* LOG_OEM3_RGED     = "RGED"; // Range Compressed
constexpr const char* LOG_OEM3_REPB     = "REPB"; // Raw Ephemeris
constexpr const char* LOG_OEM3_IONB     = "IONB"; // Iono Parameters
constexpr const char* LOG_OEM3_UTCB     = "UTCB"; // UTC Parameters

// =============================================================================
// Command Constants
// =============================================================================
constexpr const char* CMD_UNLOGALL = "UNLOGALL";
constexpr const char* CMD_UNLOG    = "UNLOG";
constexpr const char* CMD_LOG = "LOG";
constexpr const char* CMD_ONTIME = "ONTIME";
constexpr const char* CMD_ONCHANGED = "ONCHANGED";

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

/// @brief Get the interval string for a given rate in Hz
/// @param rate_hz Publish rate in Hz
/// @return Interval string (e.g. "1.0", "0.1")
inline std::string getIntervalString(int rate_hz) {
    if (rate_hz <= 0) return "1.0";
    return std::to_string(1.0 / static_cast<double>(rate_hz));
}

// =============================================================================
// Binary PVT parsers (BESTPOS / BESTVEL)
// =============================================================================
namespace pvt {

// BESTPOS body field offsets (after 28-byte OEM4 header)
constexpr int BESTPOS_OFFSET_SOLSTAT    = 0;
constexpr int BESTPOS_OFFSET_POSTYPE    = 4;
constexpr int BESTPOS_OFFSET_LAT        = 8;
constexpr int BESTPOS_OFFSET_LON        = 16;
constexpr int BESTPOS_OFFSET_HGT        = 24;
constexpr int BESTPOS_OFFSET_LAT_SIGMA  = 40;
constexpr int BESTPOS_OFFSET_LON_SIGMA  = 44;
constexpr int BESTPOS_OFFSET_HGT_SIGMA  = 48;
constexpr int BESTPOS_OFFSET_DIFF_AGE   = 56;
constexpr int BESTPOS_OFFSET_NUM_SATS   = 64;
constexpr int BESTPOS_MIN_LEN           = 72;

// BESTVEL body field offsets
constexpr int BESTVEL_OFFSET_HOR_SPEED  = 16;
constexpr int BESTVEL_OFFSET_TRK_GND    = 24;
constexpr int BESTVEL_OFFSET_VERT_SPEED = 32;
constexpr int BESTVEL_MIN_LEN           = 44;

// NovAtel sol_stat values (subset used to gate publishing)
constexpr uint32_t SOL_COMPUTED = 0;

// NovAtel pos_type values (subset; full list in OEM7 manual)
constexpr uint32_t POSTYPE_NONE         = 0;
constexpr uint32_t POSTYPE_SINGLE       = 16;
constexpr uint32_t POSTYPE_PSRDIFF      = 17;
constexpr uint32_t POSTYPE_WAAS         = 18;  // SBAS
constexpr uint32_t POSTYPE_L1_FLOAT     = 32;
constexpr uint32_t POSTYPE_IONOFREE_FLOAT = 33;
constexpr uint32_t POSTYPE_NARROW_FLOAT = 34;
constexpr uint32_t POSTYPE_L1_INT       = 48;
constexpr uint32_t POSTYPE_WIDE_INT     = 49;
constexpr uint32_t POSTYPE_NARROW_INT   = 50;
constexpr uint32_t POSTYPE_INS_RTKFIXED = 56;
constexpr uint32_t POSTYPE_PPP_CONVERGING = 68;
constexpr uint32_t POSTYPE_PPP          = 69;
constexpr uint32_t POSTYPE_OPERATIONAL  = 70;

template <typename T>
inline T read_le(const uint8_t* p) {
  T v;
  std::memcpy(&v, p, sizeof(T));
  return v;
}

inline uint8_t mapPosTypeToStatus(uint32_t pos_type) {
  using S = gnss_ros_standardization::msg::GnssSolution;
  switch (pos_type) {
    case POSTYPE_NONE:           return S::STATUS_NONE;
    case POSTYPE_SINGLE:         return S::STATUS_SINGLE;
    case POSTYPE_PSRDIFF:        return S::STATUS_DGPS;
    case POSTYPE_WAAS:           return S::STATUS_SBAS;
    case POSTYPE_L1_FLOAT:
    case POSTYPE_IONOFREE_FLOAT:
    case POSTYPE_NARROW_FLOAT:   return S::STATUS_FLOAT;
    case POSTYPE_L1_INT:
    case POSTYPE_WIDE_INT:
    case POSTYPE_NARROW_INT:
    case POSTYPE_INS_RTKFIXED:   return S::STATUS_FIX;
    case POSTYPE_PPP_CONVERGING:
    case POSTYPE_PPP:
    case POSTYPE_OPERATIONAL:    return S::STATUS_PPP;
    default:                     return S::STATUS_SINGLE;  // best-effort for unknown variants
  }
}

/// Parse BESTPOS body (post-header) into GnssSolution.
/// Fills: status, num_sats, age_diff, latitude, longitude, altitude,
/// pos_enu_cov diagonal (from sigma values).
inline bool parseBESTPOS(const uint8_t* p, size_t len,
                         gnss_ros_standardization::msg::GnssSolution& out) {
  if (len < static_cast<size_t>(BESTPOS_MIN_LEN)) return false;
  const uint32_t sol_stat = read_le<uint32_t>(p + BESTPOS_OFFSET_SOLSTAT);
  const uint32_t pos_type = read_le<uint32_t>(p + BESTPOS_OFFSET_POSTYPE);
  const double   lat      = read_le<double>(p + BESTPOS_OFFSET_LAT);
  const double   lon      = read_le<double>(p + BESTPOS_OFFSET_LON);
  const double   hgt      = read_le<double>(p + BESTPOS_OFFSET_HGT);
  const float    lat_s    = read_le<float>(p + BESTPOS_OFFSET_LAT_SIGMA);
  const float    lon_s    = read_le<float>(p + BESTPOS_OFFSET_LON_SIGMA);
  const float    hgt_s    = read_le<float>(p + BESTPOS_OFFSET_HGT_SIGMA);
  const float    diff_age = read_le<float>(p + BESTPOS_OFFSET_DIFF_AGE);
  const uint8_t  num_sats = p[BESTPOS_OFFSET_NUM_SATS];

  out.status    = (sol_stat == SOL_COMPUTED) ? mapPosTypeToStatus(pos_type)
                                              : gnss_ros_standardization::msg::GnssSolution::STATUS_NONE;
  out.num_sats  = num_sats;
  out.age_diff  = diff_age;
  out.latitude  = lat;
  out.longitude = lon;
  out.altitude  = hgt;

  // sigma values are in meters of local tangent-plane error
  std::fill(out.pos_enu_cov.begin(), out.pos_enu_cov.end(), 0.0);
  out.pos_enu_cov[0] = static_cast<double>(lon_s) * lon_s;  // EE
  out.pos_enu_cov[4] = static_cast<double>(lat_s) * lat_s;  // NN
  out.pos_enu_cov[8] = static_cast<double>(hgt_s) * hgt_s;  // UU
  return true;
}

/// Parse BESTVEL body (post-header) into GnssSolution.
/// Fills: vel_enu (from hor_speed/trk_gnd/vert_speed). Status is not modified
/// (BESTPOS owns status).
inline bool parseBESTVEL(const uint8_t* p, size_t len,
                         gnss_ros_standardization::msg::GnssSolution& out) {
  if (len < static_cast<size_t>(BESTVEL_MIN_LEN)) return false;
  const double hor_speed  = read_le<double>(p + BESTVEL_OFFSET_HOR_SPEED);
  const double trk_deg    = read_le<double>(p + BESTVEL_OFFSET_TRK_GND);
  const double vert_speed = read_le<double>(p + BESTVEL_OFFSET_VERT_SPEED);

  constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
  const double trk_rad = trk_deg * kDeg2Rad;
  out.vel_enu.x = hor_speed * std::sin(trk_rad);  // East
  out.vel_enu.y = hor_speed * std::cos(trk_rad);  // North
  out.vel_enu.z = vert_speed;                     // Up
  return true;
}

}  // namespace pvt

}  // namespace novatel
}  // namespace gnss_ros_standardization
