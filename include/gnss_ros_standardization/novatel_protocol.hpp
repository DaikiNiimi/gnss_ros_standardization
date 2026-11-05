#pragma once
#include "gnss_ros_standardization/receiver_diagnostics.h"
/// @file novatel_protocol.hpp
/// @brief NovAtel OEM7/6/4 protocol constants and utility types.
///
/// This header centralizes all NovAtel protocol definitions used by the driver and decoder nodes.

#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

// Include rtklib.h for STR_ constants
#include "gnss_ros_standardization/rtklib_compat.hpp"

#include "gnss_ros_standardization/msg/gnss_solution.hpp"

namespace gnss_ros_standardization {
namespace novatel {

// N1 Binary header: fields may be appended; RTKLIB expects the original 28 bytes.
// Input has already passed framing and CRC validation. Short logs are frontend-only.
inline std::vector<uint8_t> backendFrame(const std::vector<uint8_t>& frame) {
  if (frame.size() < 32 || frame[2] != 0x12) return {};
  if (frame[3] == 28) return frame;
  const size_t header = frame[3];
  if (header < 28 || frame.size() < header + 4) return {};
  std::vector<uint8_t> out(frame.begin(), frame.begin() + 28);
  out[3] = 28;
  out.insert(out.end(), frame.begin() + header, frame.end() - 4);
  const uint32_t crc = rtk_crc32(out.data(), static_cast<int>(out.size()));
  for (int i = 0; i < 4; ++i) out.push_back(static_cast<uint8_t>(crc >> (8*i)));
  return out;
}

// Constants

constexpr size_t READ_BUFFER_SIZE = 4096;
constexpr size_t NMEA_MAX_LINE_LEN = 256;

// NovAtel Message IDs (decimal)
// Reference: RTKLIB src/rcv/novatel.c and OEM7 Commands and Logs Reference Manual

// Measurement
constexpr uint16_t ID_RANGECMP          = 140;
constexpr uint16_t ID_RANGE             = 43;
constexpr uint16_t ID_BESTPOS           = 42;
constexpr uint16_t ID_BESTVEL           = 99;
constexpr uint16_t ID_PSRDOP            = 174;
constexpr uint16_t ID_BESTXYZ           = 241;

// Ephemeris / Navigation Data
constexpr uint16_t ID_RAWEPHEM          = 41;
constexpr uint16_t ID_IONUTC            = 8;
constexpr uint16_t ID_RAWWAASFRAME      = 287;
constexpr uint16_t ID_RAWSBASFRAME      = 973; // OEM7/6
constexpr uint16_t ID_GLOEPHEMERIS      = 723;
constexpr uint16_t ID_GALEPHEMERIS      = 1122;
constexpr uint16_t ID_GALIONO           = 1127;
constexpr uint16_t ID_GALCLOCK          = 1121;
constexpr uint16_t ID_QZSSRAWEPHEM      = 1331;
constexpr uint16_t ID_QZSSRAWSUBFRAME   = 1330;
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

// OEM4/7 binary header offsets
// Layout: SYNC(3) HDRLEN(1) MSGID(2) MSGTYPE(1) PORT(1) MSGLEN(2) ...
// Standard header length = 28 bytes; MSGLEN field is at bytes 8-9 of the header.
namespace oem4 {
  constexpr int MSGID_OFFSET  = 4;   // bytes 4-5 from SYNC1
  constexpr int MSGLEN_OFFSET = 8;   // bytes 8-9 from SYNC1
}
constexpr int     OEM4_HEADER_LEN = 28;  // Standard OEM4/7 binary header length

// OEM3 Message IDs
constexpr uint16_t ID_OEM3_RGEB         = 32;
constexpr uint16_t ID_OEM3_RGED         = 65;
constexpr uint16_t ID_OEM3_REPB         = 14;
constexpr uint16_t ID_OEM3_IONB         = 16;
constexpr uint16_t ID_OEM3_UTCB         = 17;
constexpr uint16_t ID_OEM3_FRMB         = 54;

// NovAtel Log Names (for LOG command)
constexpr const char* LOG_RANGECMP      = "RANGECMPB"; // Binary
constexpr const char* LOG_RANGE         = "RANGEB";    // Binary
constexpr const char* LOG_BESTPOS       = "BESTPOSB";  // Binary
constexpr const char* LOG_BESTVEL       = "BESTVELB";  // Binary
constexpr const char* LOG_PSRDOPB       = "PSRDOPB";   // Binary
constexpr const char* LOG_BESTXYZB      = "BESTXYZB";  // Binary

// GPS ephemeris: RTKLIB decodes RAWEPHEMB (ID 41) only. GPSEPHEMB (ID 7) is
// not handled by RTKLIB's novatel.c, so we request RAWEPHEMB instead.
constexpr const char* LOG_RAWEPHEM      = "RAWEPHEMB";
constexpr const char* LOG_IONUTC        = "IONUTCB";
constexpr const char* LOG_RAWWAASFRAME  = "RAWWAASFRAMEB";
constexpr const char* LOG_GLOEPHEMERIS  = "GLOEPHEMERISB";
// GALEPHEMERISB (ID 1122) carries both I/NAV and F/NAV data; GALINAVEPHEMERISB
// (ID 1309) is not supported by RTKLIB and unnecessary.
constexpr const char* LOG_GALEPHEMERIS  = "GALEPHEMERISB";
constexpr const char* LOG_GALIONO       = "GALIONOB";
constexpr const char* LOG_GALCLOCK      = "GALCLOCKB";
// QZSS ephemeris: RTKLIB decodes QZSSRAWEPHEMB (ID 1331) only. QZSSEPHEMERISB
// (ID 1336) is not handled by RTKLIB's novatel.c.
constexpr const char* LOG_QZSSRAWEPHEM  = "QZSSRAWEPHEMB";
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

// Command Constants
constexpr const char* CMD_UNLOGALL = "UNLOGALL";
constexpr const char* CMD_UNLOG    = "UNLOG";
constexpr const char* CMD_LOG = "LOG";
constexpr const char* CMD_ONTIME = "ONTIME";
constexpr const char* CMD_ONCHANGED = "ONCHANGED";
constexpr const char* CMD_INTERFACEMODE = "INTERFACEMODE";

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

/// @brief Get the interval string for a given rate in Hz
/// @param rate_hz Publish rate in Hz
/// @return Interval string (e.g. "1.0", "0.1")
inline std::string getIntervalString(int rate_hz) {
    if (rate_hz <= 0) return "1.0";
    return std::to_string(1.0 / static_cast<double>(rate_hz));
}

// Binary PVT parsers (BESTPOS / BESTVEL)
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
constexpr int BESTPOS_OFFSET_NUM_SATS   = 65;
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
    // OEM7 BESTPOS Position/velocity type, D100402 v6A (2026-09).
    // Fixed coordinates and external INS carry no ambiguity quality.
    // Legacy OEM7 51 (RTK_DIRECT_INS) initializes RTK from INS; it does
    // not assert resolved carrier ambiguities. Retain position without FIX.
    // OEM7 7.09.05+ reserves 51; the legacy value is kept for recorded data.
    case 1: case 2: case 51: case 53: case 67: case 72:
    case POSTYPE_SINGLE:         return S::STATUS_SINGLE;
    // PROPAGATED: carried forward by the receiver filter without new
    // observations (RTKLIB SOLQ_DR).
    case 19:                     return S::STATUS_DR;
    case 54:
    case POSTYPE_PSRDIFF:        return S::STATUS_DGPS;
    case 52:
    case POSTYPE_WAAS:           return S::STATUS_SBAS;
    case 55: case 71:
    case POSTYPE_L1_FLOAT:
    case POSTYPE_IONOFREE_FLOAT:
    case POSTYPE_NARROW_FLOAT:   return S::STATUS_FLOAT;
    // UAL quality follows the OEM7 supplemental NMEA mapping: 70/71/72=4/5/1.
    case POSTYPE_OPERATIONAL:
    case POSTYPE_L1_INT:
    case POSTYPE_WIDE_INT:
    case POSTYPE_NARROW_INT:
    case POSTYPE_INS_RTKFIXED:   return S::STATUS_FIX;
    case 73: case 74: case 77: case 78: case 79: case 80:
    case POSTYPE_PPP_CONVERGING:
    case POSTYPE_PPP:    return S::STATUS_PPP;
    default:                     return S::STATUS_NONE;
  }
}

/// Parse BESTPOS body (post-header) into GnssSolution.
/// Fills: status, num_sats, age_diff, latitude, longitude, altitude,
/// pos_enu_cov diagonal (from lat/lon/hgt sigma; native local-tangent ENU,
/// off-diagonals not provided). ECEF position and rotated ECEF covariance
/// are derived downstream from LLH and the local origin.
inline bool parseBESTPOS(const uint8_t* p, size_t len,
                         gnss_ros_standardization::msg::GnssSolution& out) {
  if (len < static_cast<size_t>(BESTPOS_MIN_LEN)) return GNSS_RX_REJECT(GNSS_RX_LENGTH);
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

  if (!std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(hgt) ||
      !std::isfinite(read_le<float>(p + 32)) || std::abs(lat) > 90.0 ||
      std::abs(lon) > 180.0 || read_le<uint32_t>(p + 36) != 61) return GNSS_RX_REJECT(read_le<uint32_t>(p+36)!=61 ? GNSS_RX_FIELD : GNSS_RX_VALUE);
  out.status    = (sol_stat == SOL_COMPUTED) ? mapPosTypeToStatus(pos_type)
                                              : gnss_ros_standardization::msg::GnssSolution::STATUS_NONE;
  out.num_sats  = num_sats;
  out.age_diff  = diff_age;
  out.latitude  = lat;
  out.longitude = lon;
  out.altitude  = hgt + read_le<float>(p + 32);

  const bool covariance_valid = std::isfinite(lat_s) && std::isfinite(lon_s) &&
      std::isfinite(hgt_s) && lat_s >= 0 && lon_s >= 0 && hgt_s >= 0;
  std::fill(out.pos_enu_cov.begin(), out.pos_enu_cov.end(),
            covariance_valid ? 0.0 : std::numeric_limits<double>::quiet_NaN());
  if (covariance_valid) {
    out.pos_enu_cov[0] = static_cast<double>(lon_s) * lon_s;
    out.pos_enu_cov[4] = static_cast<double>(lat_s) * lat_s;
    out.pos_enu_cov[8] = static_cast<double>(hgt_s) * hgt_s;
  }
  if (!std::isfinite(diff_age) || diff_age < 0)
    out.age_diff = std::numeric_limits<double>::quiet_NaN();
  return true;
}

/// Parse BESTVEL body (post-header) into GnssSolution.
/// Fills: vel_enu (from hor_speed/trk_gnd/vert_speed). Status is not modified
/// (BESTPOS owns status).
inline bool parseBESTVEL(const uint8_t* p, size_t len,
                         gnss_ros_standardization::msg::GnssSolution& out) {
  if (len < static_cast<size_t>(BESTVEL_MIN_LEN)) return GNSS_RX_REJECT(GNSS_RX_LENGTH);
  if (read_le<uint32_t>(p) != SOL_COMPUTED) return GNSS_RX_REJECT(GNSS_RX_QUALITY);
  // A single GnssSolution epoch cannot label a delayed velocity as current.
  if (read_le<float>(p + 8) != 0.0f) return GNSS_RX_REJECT(GNSS_RX_TIME);
  const double hor_speed  = read_le<double>(p + BESTVEL_OFFSET_HOR_SPEED);
  const double trk_deg    = read_le<double>(p + BESTVEL_OFFSET_TRK_GND);
  const double vert_speed = read_le<double>(p + BESTVEL_OFFSET_VERT_SPEED);

  if (!std::isfinite(hor_speed) || !std::isfinite(trk_deg) ||
      !std::isfinite(vert_speed) || hor_speed < 0 || trk_deg < 0 || trk_deg > 360) return GNSS_RX_REJECT(GNSS_RX_VALUE);
  constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
  const double trk_rad = trk_deg * kDeg2Rad;
  out.vel_enu.x = hor_speed * std::sin(trk_rad);  // East
  out.vel_enu.y = hor_speed * std::cos(trk_rad);  // North
  out.vel_enu.z = vert_speed;                     // Up
  return true;
}

// PSRDOP body field offsets (after 28-byte OEM4 header)
constexpr int PSRDOP_OFFSET_GDOP    = 0;   // float32
constexpr int PSRDOP_OFFSET_PDOP    = 4;
constexpr int PSRDOP_OFFSET_HDOP    = 8;
constexpr int PSRDOP_OFFSET_HTDOP   = 12;
constexpr int PSRDOP_OFFSET_TDOP    = 16;
constexpr int PSRDOP_MIN_LEN        = 20;

// BESTXYZ body field offsets (after 28-byte OEM4 header)
constexpr int BESTXYZ_OFFSET_PX     = 8;   // double64 ECEF pos
constexpr int BESTXYZ_OFFSET_PY     = 16;
constexpr int BESTXYZ_OFFSET_PZ     = 24;
constexpr int BESTXYZ_OFFSET_SPX    = 32;  // float32 pos stddev
constexpr int BESTXYZ_OFFSET_SPY    = 36;
constexpr int BESTXYZ_OFFSET_SPZ    = 40;
constexpr int BESTXYZ_OFFSET_VX     = 52;  // double64 ECEF vel
constexpr int BESTXYZ_OFFSET_VY     = 60;
constexpr int BESTXYZ_OFFSET_VZ     = 68;
constexpr int BESTXYZ_OFFSET_SVX    = 76;  // float32 vel stddev
constexpr int BESTXYZ_OFFSET_SVY    = 80;
constexpr int BESTXYZ_OFFSET_SVZ    = 84;
constexpr int BESTXYZ_MIN_LEN       = 112;

/// Parse PSRDOP body into GnssSolution.
/// Fills: gdop, pdop, hdop; vdop derived as sqrt(max(pdop²-hdop²,0)).
inline bool parsePSRDOP(const uint8_t* p, size_t len,
                        gnss_ros_standardization::msg::GnssSolution& out) {
  if (len < static_cast<size_t>(PSRDOP_MIN_LEN)) return GNSS_RX_REJECT(GNSS_RX_LENGTH);
  const float gdop = read_le<float>(p + PSRDOP_OFFSET_GDOP);
  const float pdop = read_le<float>(p + PSRDOP_OFFSET_PDOP);
  const float hdop = read_le<float>(p + PSRDOP_OFFSET_HDOP);
  for (int offset : {0, 4, 8, 12, 16}) {
    const float value = read_le<float>(p + offset);
    if (!std::isfinite(value) || value < 0) return GNSS_RX_REJECT(GNSS_RX_VALUE);
  }
  if (gdop + 1e-5f < pdop || pdop + 1e-5f < hdop) return GNSS_RX_REJECT(GNSS_RX_VALUE);
  const double vdop2 = double(pdop) * pdop - double(hdop) * hdop;
  out.gdop = gdop;
  out.pdop = pdop;
  out.hdop = hdop;
  out.vdop = (vdop2 > 0.0f) ? std::sqrt(vdop2) : 0.0f;
  return true;
}

/// Parse BESTXYZ body into GnssSolution.
///
/// BESTXYZ represents the same internal solution as BESTPOS/BESTVEL expressed
/// in ECEF Cartesian coordinates. Position and velocity values are therefore
/// redundant with BESTPOS/BESTVEL; this parser intentionally does NOT overwrite
/// `pos_ecef` / `vel_ecef` to keep BESTPOS/BESTVEL as the single source of truth
/// for those values.
///
/// What this fills: native ECEF covariance diagonals of position and velocity
/// (`pos_cov_ecef[0,4,8]` and `vel_cov_ecef[0,4,8]`, in m² and (m/s)²
/// respectively). Off-diagonal terms are not provided by BESTXYZ; the rotated
/// 3x3 in ENU computed downstream is a rotated-diagonal approximation and not
/// the true full covariance.
inline bool parseBESTXYZ(const uint8_t* p, size_t len,
                         gnss_ros_standardization::msg::GnssSolution& out) {
  if (len < static_cast<size_t>(BESTXYZ_MIN_LEN)) return GNSS_RX_REJECT(GNSS_RX_LENGTH);
  auto valid = [&](int status, int type, int sigma) {
    if (read_le<uint32_t>(p + status) != SOL_COMPUTED || read_le<uint32_t>(p + type) == 0)
      return GNSS_RX_REJECT(GNSS_RX_QUALITY);
    for (int offset : {sigma, sigma + 4, sigma + 8}) {
      const float value = read_le<float>(p + offset);
      if (!std::isfinite(value) || value < 0) return GNSS_RX_REJECT(GNSS_RX_VALUE);
    }
    return true;
  };
  const bool position_valid = valid(0, 4, BESTXYZ_OFFSET_SPX);
  const bool aligned = read_le<float>(p + 92) == 0.0f;
  if (!aligned) GNSS_RX_EVENT(GNSS_RX_TIME,1);
  const bool velocity_valid = aligned && valid(44, 48, BESTXYZ_OFFSET_SVX);
  if (!position_valid && !velocity_valid) return false;
  auto copy = [&](auto& covariance, bool available, int offset) {
    covariance.fill(available ? 0.0 : std::numeric_limits<double>::quiet_NaN());
    if (available) for (int i = 0; i < 3; ++i) {
      const double sigma = read_le<float>(p + offset + i * 4);
      covariance[i * 4] = sigma * sigma;
    }
  };
  copy(out.pos_cov_ecef, position_valid, BESTXYZ_OFFSET_SPX);
  copy(out.vel_cov_ecef, velocity_valid, BESTXYZ_OFFSET_SVX);
  return true;
}

}  // namespace pvt

}  // namespace novatel
}  // namespace gnss_ros_standardization

// ---- novatel imu scales ----
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
