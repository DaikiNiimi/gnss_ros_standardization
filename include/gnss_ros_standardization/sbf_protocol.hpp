#pragma once
/// @file sbf_protocol.hpp
/// @brief Septentrio SBF protocol constants and utility types.
///
/// This header centralizes all SBF protocol definitions used by the driver and decoder nodes.

#include <string>
#include <string_view>
#include <vector>

// Include rtklib.h for STR_ constants
#include "rtklib.h"

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

// Navigation Blocks
constexpr uint16_t ID_GPSRAWCA    = 4017;
constexpr uint16_t ID_GLORAWCA    = 4026;
constexpr uint16_t ID_GALRAWFNAV  = 4022;
constexpr uint16_t ID_GALRAWINAV  = 4023;
constexpr uint16_t ID_GEORAWL1    = 4020;
constexpr uint16_t ID_BDSRAW      = 4047;
constexpr uint16_t ID_QZSRAWL1CA  = 4066;
constexpr uint16_t ID_QZSRAWL6    = 4069;
constexpr uint16_t ID_NAVICRAW    = 4093;

// PVT Blocks
constexpr uint16_t ID_PVTCARTESIAN      = 4006;
constexpr uint16_t ID_PVTGEODETIC       = 4007;
constexpr uint16_t ID_POSCOVCARTESIAN   = 5905;
constexpr uint16_t ID_POSCOVGEODETIC    = 5906;
constexpr uint16_t ID_ATTEULER          = 5938;
constexpr uint16_t ID_ATTCOVEULER       = 5939;

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
constexpr const char* BLOCK_GPSNAV    = "GPSRawCA";
constexpr const char* BLOCK_GLONAV    = "GLORawCA";
constexpr const char* BLOCK_GALNAV    = "GALRawINAV+GALRawFNAV";
constexpr const char* BLOCK_BDSNAV    = "BDSRaw";
constexpr const char* BLOCK_QZSNAV    = "QZSRawL1CA";
constexpr const char* BLOCK_NAVICNAV  = "NAVICRaw";
constexpr const char* BLOCK_PVTGEODETIC = "PVTGeodetic";
constexpr const char* BLOCK_POSCOVGEODETIC = "PosCovGeodetic";
constexpr const char* BLOCK_PVTCARTESIAN = "PVTCartesian";
constexpr const char* BLOCK_POSCOVCARTESIAN = "PosCovCartesian";
constexpr const char* BLOCK_ATTEULER = "AttEuler";
constexpr const char* BLOCK_ATTCOVEULER = "AttCovEuler";
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

}  // namespace sbf
}  // namespace gnss_ros_standardization
