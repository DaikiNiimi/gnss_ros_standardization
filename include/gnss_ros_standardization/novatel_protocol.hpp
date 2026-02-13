#pragma once
/// @file novatel_protocol.hpp
/// @brief NovAtel OEM7/6/4 protocol constants and utility types.
///
/// This header centralizes all NovAtel protocol definitions used by the driver and decoder nodes.

#include <string>
#include <string_view>
#include <vector>

// Include rtklib.h for STR_ constants
#include "rtklib.h"

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

}  // namespace novatel
}  // namespace gnss_ros_standardization
