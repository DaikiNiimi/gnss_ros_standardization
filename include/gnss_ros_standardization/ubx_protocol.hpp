// SPDX-License-Identifier: MIT
#pragma once
/// @file ubx_protocol.hpp
/// @brief u-blox UBX protocol constants, CFG-VALSET keys, and utility types.
///
/// This header centralizes all UBX protocol definitions used by the driver node,
/// eliminating hardcoded magic numbers from the implementation.

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>
#include <vector>

#include "gnss_ros_standardization/msg/gnss_solution.hpp"

namespace gnss_ros_standardization {
namespace ubx {

// UBX Message Classes
constexpr uint8_t CLASS_NAV = 0x01;
constexpr uint8_t CLASS_RXM = 0x02;
constexpr uint8_t CLASS_ACK = 0x05;
constexpr uint8_t CLASS_CFG = 0x06;
constexpr uint8_t CLASS_MON = 0x0A;
constexpr uint8_t CLASS_ESF = 0x10;  // External Sensor Fusion (ZED-F9R / IMU-enabled)
constexpr uint8_t CLASS_HNR = 0x28;  // High Navigation Rate (legacy F9R)

// UBX Message IDs

// ACK
constexpr uint8_t ID_ACK_NAK = 0x00;
constexpr uint8_t ID_ACK_ACK = 0x01;

// NAV
constexpr uint8_t ID_NAV_PVT     = 0x07;
constexpr uint8_t ID_NAV_DOP     = 0x04;
constexpr uint8_t ID_NAV_COV     = 0x36;
constexpr uint8_t ID_NAV_POSECEF = 0x01;
constexpr uint8_t ID_NAV_VELECEF = 0x11;

// ESF (External Sensor Fusion)
constexpr uint8_t ID_ESF_RAW = 0x03;  // Raw IMU measurements (uncalibrated, IMU-internal scale)
constexpr uint8_t ID_ESF_INS = 0x15;  // Calibrated IMU angular rate + acceleration

// ESF-RAW data types (per ZED-F9R Interface Description)
namespace esf_raw {
  constexpr uint8_t TYPE_GYRO_Z = 5;   // Z-axis gyroscope angular rate, deg/s, scale 2^-12
  constexpr uint8_t TYPE_GYRO_Y = 13;  // Y-axis gyroscope angular rate, deg/s, scale 2^-12
  constexpr uint8_t TYPE_GYRO_X = 14;  // X-axis gyroscope angular rate, deg/s, scale 2^-12
  constexpr uint8_t TYPE_ACCEL_X = 16; // X-axis accel specific force,    m/s², scale 2^-10
  constexpr uint8_t TYPE_ACCEL_Y = 17; // Y-axis accel specific force,    m/s², scale 2^-10
  constexpr uint8_t TYPE_ACCEL_Z = 18; // Z-axis accel specific force,    m/s², scale 2^-10
  constexpr double  GYRO_SCALE   = 0.000244140625;  // 2^-12 (deg/s) — convert to rad/s by × π/180
  constexpr double  ACCEL_SCALE  = 0.0009765625;    // 2^-10 (m/s²)
}

// ESF-INS payload offsets (version 0, 36 bytes)
namespace esf_ins {
  constexpr int OFFSET_BITFIELD  = 0;
  constexpr int OFFSET_ITOW      = 8;
  constexpr int OFFSET_XANGRATE  = 12;
  constexpr int OFFSET_YANGRATE  = 16;
  constexpr int OFFSET_ZANGRATE  = 20;
  constexpr int OFFSET_XACCEL    = 24;
  constexpr int OFFSET_YACCEL    = 28;
  constexpr int OFFSET_ZACCEL    = 32;
  constexpr int MIN_LEN          = 36;
}

// MON
constexpr uint8_t ID_MON_VER = 0x04;

// RXM
constexpr uint8_t ID_RXM_SFRBX = 0x13;
constexpr uint8_t ID_RXM_RAWX  = 0x15;

// CFG (Legacy Gen 9)
constexpr uint8_t ID_CFG_MSG   = 0x01;
constexpr uint8_t ID_CFG_RST   = 0x04;
constexpr uint8_t ID_CFG_RATE  = 0x08;
constexpr uint8_t ID_CFG_NAV5  = 0x24;
constexpr uint8_t ID_CFG_GNSS  = 0x3E;
constexpr uint8_t ID_CFG_VALSET = 0x8A;

// NMEA Message IDs (Class 0xF0)
constexpr uint8_t NMEA_GGA = 0x00;
constexpr uint8_t NMEA_GLL = 0x01;
constexpr uint8_t NMEA_GSA = 0x02;
constexpr uint8_t NMEA_GSV = 0x03;
constexpr uint8_t NMEA_RMC = 0x04;
constexpr uint8_t NMEA_VTG = 0x05;
constexpr uint8_t NMEA_GST = 0x07;
constexpr uint8_t NMEA_ZDA = 0x08;

// UBX Sync Bytes
constexpr uint8_t SYNC1 = 0xB5;
constexpr uint8_t SYNC2 = 0x62;

// CFG-VALSET Keys (Gen 10+)

// --- Measurement Rate ---
constexpr uint32_t CFG_RATE_MEAS     = 0x30210001;  // U2: Measurement period (ms)
constexpr uint32_t CFG_RATE_NAV      = 0x30210002;  // U2: Navigation rate cycles

// --- Dynamic Model ---
constexpr uint32_t CFG_NAVSPG_DYNMODEL = 0x20110021; // E1: Dynamic platform model

// --- Signal Configuration ---
// GPS
constexpr uint32_t CFG_SIGNAL_GPS_ENA      = 0x1031001F;
constexpr uint32_t CFG_SIGNAL_GPS_L1CA_ENA = 0x10310001;
constexpr uint32_t CFG_SIGNAL_GPS_L2C_ENA  = 0x10310003;
constexpr uint32_t CFG_SIGNAL_GPS_L5_ENA   = 0x10310004;

// SBAS
constexpr uint32_t CFG_SIGNAL_SBAS_ENA     = 0x10310020;
constexpr uint32_t CFG_SIGNAL_SBAS_L1CA_ENA = 0x10310005;

// Galileo
constexpr uint32_t CFG_SIGNAL_GAL_ENA     = 0x10310021;
constexpr uint32_t CFG_SIGNAL_GAL_E1_ENA  = 0x10310007;
constexpr uint32_t CFG_SIGNAL_GAL_E5A_ENA = 0x10310009;
constexpr uint32_t CFG_SIGNAL_GAL_E5B_ENA = 0x1031000A;

// BeiDou
// Note: X20P supports B1C/B2A/B3I but MALIB NOT support B1C/B2A/B3I.
constexpr uint32_t CFG_SIGNAL_BDS_ENA     = 0x10310022;
constexpr uint32_t CFG_SIGNAL_BDS_B1I_ENA = 0x1031000D;  // B1I (F9P)
constexpr uint32_t CFG_SIGNAL_BDS_B1C_ENA = 0x1031000F;  // B1C (X20P)
constexpr uint32_t CFG_SIGNAL_BDS_B2I_ENA = 0x1031000E;  // B2I (F9P)
constexpr uint32_t CFG_SIGNAL_BDS_B2A_ENA = 0x10310028;  // B2a (X20P)

// GLONASS
constexpr uint32_t CFG_SIGNAL_GLO_ENA    = 0x10310025;
constexpr uint32_t CFG_SIGNAL_GLO_L1_ENA = 0x10310018;
constexpr uint32_t CFG_SIGNAL_GLO_L2_ENA = 0x10310019;  // G2C/A (GLO L2 OF)

// QZSS
constexpr uint32_t CFG_SIGNAL_QZSS_ENA      = 0x10310024;
constexpr uint32_t CFG_SIGNAL_QZSS_L1CA_ENA = 0x10310012;
constexpr uint32_t CFG_SIGNAL_QZSS_L1S_ENA  = 0x10310014;  // L1S (CLAS)
constexpr uint32_t CFG_SIGNAL_QZSS_L2C_ENA  = 0x10310015;
constexpr uint32_t CFG_SIGNAL_QZSS_L5_ENA   = 0x10310017;

// --- Output Message Rate (per-port) ---
// Base addresses (I2C = offset 0). Add port offset for UART1(+1), UART2(+2), USB(+3).
constexpr uint32_t CFG_MSGOUT_UBX_RXM_RAWX_I2C  = 0x209102A4;
constexpr uint32_t CFG_MSGOUT_UBX_RXM_SFRBX_I2C = 0x20910231;
constexpr uint32_t CFG_MSGOUT_UBX_NAV_PVT_I2C     = 0x20910006;
constexpr uint32_t CFG_MSGOUT_UBX_NAV_DOP_I2C     = 0x20910038;
constexpr uint32_t CFG_MSGOUT_UBX_NAV_COV_I2C     = 0x20910083;
constexpr uint32_t CFG_MSGOUT_UBX_NAV_POSECEF_I2C = 0x20910024;
constexpr uint32_t CFG_MSGOUT_UBX_NAV_VELECEF_I2C = 0x2091003D;

constexpr uint32_t CFG_MSGOUT_NMEA_ID_GGA_I2C   = 0x209100BA;
constexpr uint32_t CFG_MSGOUT_NMEA_ID_GLL_I2C   = 0x209100C9;
constexpr uint32_t CFG_MSGOUT_NMEA_ID_GSA_I2C   = 0x209100C0;
constexpr uint32_t CFG_MSGOUT_NMEA_ID_GSV_I2C   = 0x209100C5;
constexpr uint32_t CFG_MSGOUT_NMEA_ID_RMC_I2C   = 0x209100AB;
constexpr uint32_t CFG_MSGOUT_NMEA_ID_VTG_I2C   = 0x209100B0;
constexpr uint32_t CFG_MSGOUT_NMEA_ID_GST_I2C   = 0x209100D3;
constexpr uint32_t CFG_MSGOUT_NMEA_ID_ZDA_I2C   = 0x209100D8;

// --- IMU Output ---
// ESF-INS (calibrated angular rate + acceleration): base I2C key, +port offset for UART1/2/USB
constexpr uint32_t CFG_MSGOUT_UBX_ESF_INS_I2C  = 0x20910036;
// ESF-RAW (uncalibrated raw IMU measurements): base I2C key
constexpr uint32_t CFG_MSGOUT_UBX_ESF_RAW_I2C  = 0x209102BC;

// --- NMEA Configuration ---
constexpr uint32_t CFG_NMEA_HIGHPREC = 0x10930006;  // L: Enable high-precision NMEA output

// Port offsets
constexpr int PORT_I2C  = 0;
constexpr int PORT_UART1 = 1;
constexpr int PORT_UART2 = 2;
constexpr int PORT_USB   = 3;

// CFG-RST Payload Constants
constexpr uint16_t RST_BBR_HOTSTART = 0x0000;
constexpr uint8_t  RST_MODE_GNSS_ONLY = 0x02;  // Controlled GNSS-only restart

// Dynamic Model
enum class DynamicModel : uint8_t {
  kPortable    = 0,
  kWatch       = 1,
  kStationary  = 2,
  kPedestrian  = 3,
  kAutomotive  = 4,
  kSea         = 5,
  kAirborne1g  = 6,
  kAirborne2g  = 7,
  kAirborne4g  = 8,
};

inline DynamicModel parseDynamicModel(const std::string& model) {
  if (model == "stationary")  return DynamicModel::kStationary;
  if (model == "pedestrian")  return DynamicModel::kPedestrian;
  if (model == "automotive")  return DynamicModel::kAutomotive;
  if (model == "sea")         return DynamicModel::kSea;
  if (model == "airborne_1g") return DynamicModel::kAirborne1g;
  if (model == "airborne_2g") return DynamicModel::kAirborne2g;
  if (model == "airborne_4g") return DynamicModel::kAirborne4g;
  if (model == "watch")       return DynamicModel::kWatch;
  return DynamicModel::kPortable;
}

// CFG-VALSET Item
struct ValsetItem {
  uint32_t key;
  uint32_t value;
  int size;  ///< Value size in bytes (1, 2, or 4)
};

// Stream Type Definition
struct StreamTypeDef {
  std::string_view prefix;
  int type;
};

/// Supported stream type prefixes (using MALIB stream types)
constexpr StreamTypeDef kStreamTypes[] = {
    {"tcpcli://", STR_TCPCLI},
    {"serial://", STR_SERIAL},
    {"ntrip://", STR_NTRIPCLI},
    {"file://", STR_FILE},
};

// Timing Constants
constexpr size_t READ_BUFFER_SIZE = 4096;
constexpr size_t NMEA_MAX_LINE_LEN = 256;
constexpr int ACK_TIMEOUT_MS   = 3000;
constexpr int GNSS_RESET_WAIT_MS = 3000;
constexpr int RETRY_DELAY_MS   = 500;
constexpr int MAX_RETRIES      = 2;

// UBX Frame Utility

/// Build a complete UBX frame (sync + header + payload + checksum).
inline std::vector<uint8_t> buildUbxFrame(uint8_t msg_class, uint8_t msg_id,
                                           const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> frame;
  frame.reserve(payload.size() + 8);
  frame.push_back(SYNC1);
  frame.push_back(SYNC2);
  frame.push_back(msg_class);
  frame.push_back(msg_id);
  frame.push_back(payload.size() & 0xFF);
  frame.push_back((payload.size() >> 8) & 0xFF);
  frame.insert(frame.end(), payload.begin(), payload.end());

  // Fletcher checksum (skip sync bytes)
  uint8_t ck_a = 0, ck_b = 0;
  for (size_t i = 2; i < frame.size(); ++i) {
    ck_a += frame[i];
    ck_b += ck_a;
  }
  frame.push_back(ck_a);
  frame.push_back(ck_b);
  return frame;
}

// Binary PVT parser (UBX-NAV-PVT)
namespace pvt {

constexpr int NAV_PVT_MIN_LEN = 92;

// Field offsets in NAV-PVT payload
constexpr int NAV_PVT_OFFSET_ITOW    = 0;
constexpr int NAV_PVT_OFFSET_FIXTYPE = 20;
constexpr int NAV_PVT_OFFSET_FLAGS   = 21;
constexpr int NAV_PVT_OFFSET_NUMSV   = 23;
constexpr int NAV_PVT_OFFSET_LON     = 24;
constexpr int NAV_PVT_OFFSET_LAT     = 28;
constexpr int NAV_PVT_OFFSET_HEIGHT  = 32;  // height above ellipsoid (mm)
constexpr int NAV_PVT_OFFSET_HACC    = 40;
constexpr int NAV_PVT_OFFSET_VACC    = 44;
constexpr int NAV_PVT_OFFSET_VELN    = 48;
constexpr int NAV_PVT_OFFSET_VELE    = 52;
constexpr int NAV_PVT_OFFSET_VELD    = 56;
constexpr int NAV_PVT_OFFSET_SACC    = 68;
constexpr int NAV_PVT_OFFSET_PDOP    = 76;

// UBX fixType values
constexpr uint8_t FIX_NONE       = 0;
constexpr uint8_t FIX_DR_ONLY    = 1;
constexpr uint8_t FIX_2D         = 2;
constexpr uint8_t FIX_3D         = 3;
constexpr uint8_t FIX_GNSS_DR    = 4;
constexpr uint8_t FIX_TIME_ONLY  = 5;

// flags byte bitmasks
constexpr uint8_t FLAG_GNSSFIXOK   = 1u << 0;
constexpr uint8_t FLAG_DIFFSOLN    = 1u << 1;
constexpr uint8_t FLAG_CARRSOLN_FLOAT = 1u << 6;  // bits 6:7 = 01
constexpr uint8_t FLAG_CARRSOLN_FIXED = 1u << 7;  // bits 6:7 = 10

template <typename T>
inline T read_le(const uint8_t* p) {
  T v;
  std::memcpy(&v, p, sizeof(T));
  return v;
}

/// Parse UBX-NAV-PVT payload into a GnssSolution.
/// Fills: time_tow (from iTOW), status, num_sats, latitude, longitude, altitude,
/// vel_enu (NED→ENU), pos_enu_cov diagonal (from hAcc/vAcc), vel_enu_cov diagonal
/// (from sAcc), pdop. time_week is left untouched (filled later from system time).
/// Returns false if payload too short or fix invalid.
///
/// Covariance contract: NAV-PVT writes the diagonal of pos_enu_cov/vel_enu_cov
/// only as a fallback when NAV-COV is not in use. A non-zero off-diagonal in
/// pos_enu_cov (resp. vel_enu_cov) is the signal that NAV-COV has already
/// populated the full 3×3 for this epoch; in that case the entire matrix is
/// left untouched so the NAV-COV values survive regardless of stream order.
inline bool parseNavPvt(const uint8_t* p, size_t len,
                        gnss_ros_standardization::msg::GnssSolution& out) {
  if (len < static_cast<size_t>(NAV_PVT_MIN_LEN)) return false;

  const uint32_t itow_ms = read_le<uint32_t>(p + NAV_PVT_OFFSET_ITOW);
  const uint8_t  fix_type = p[NAV_PVT_OFFSET_FIXTYPE];
  const uint8_t  flags    = p[NAV_PVT_OFFSET_FLAGS];
  const uint8_t  num_sv   = p[NAV_PVT_OFFSET_NUMSV];

  const int32_t lon_e7 = read_le<int32_t>(p + NAV_PVT_OFFSET_LON);
  const int32_t lat_e7 = read_le<int32_t>(p + NAV_PVT_OFFSET_LAT);
  const int32_t height_mm = read_le<int32_t>(p + NAV_PVT_OFFSET_HEIGHT);

  const uint32_t h_acc_mm = read_le<uint32_t>(p + NAV_PVT_OFFSET_HACC);
  const uint32_t v_acc_mm = read_le<uint32_t>(p + NAV_PVT_OFFSET_VACC);

  const int32_t vel_n_mms = read_le<int32_t>(p + NAV_PVT_OFFSET_VELN);
  const int32_t vel_e_mms = read_le<int32_t>(p + NAV_PVT_OFFSET_VELE);
  const int32_t vel_d_mms = read_le<int32_t>(p + NAV_PVT_OFFSET_VELD);
  const uint32_t s_acc_mms = read_le<uint32_t>(p + NAV_PVT_OFFSET_SACC);

  const uint16_t pdop_u = read_le<uint16_t>(p + NAV_PVT_OFFSET_PDOP);

  // Map UBX fix type/flags to GnssSolution status
  uint8_t status = gnss_ros_standardization::msg::GnssSolution::STATUS_NONE;
  const bool fix_ok = (flags & FLAG_GNSSFIXOK) != 0;
  if (fix_ok && (fix_type == FIX_2D || fix_type == FIX_3D || fix_type == FIX_GNSS_DR)) {
    const uint8_t carr = static_cast<uint8_t>((flags >> 6) & 0x03);
    if (carr == 2) {
      status = gnss_ros_standardization::msg::GnssSolution::STATUS_FIX;
    } else if (carr == 1) {
      status = gnss_ros_standardization::msg::GnssSolution::STATUS_FLOAT;
    } else if (flags & FLAG_DIFFSOLN) {
      status = gnss_ros_standardization::msg::GnssSolution::STATUS_DGPS;
    } else {
      status = gnss_ros_standardization::msg::GnssSolution::STATUS_SINGLE;
    }
  }

  out.status    = status;
  out.num_sats  = num_sv;
  out.time_tow  = static_cast<double>(itow_ms) * 1e-3;
  out.latitude  = static_cast<double>(lat_e7) * 1e-7;
  out.longitude = static_cast<double>(lon_e7) * 1e-7;
  out.altitude  = static_cast<double>(height_mm) * 1e-3;
  out.pdop      = static_cast<float>(pdop_u) * 0.01f;

  // Velocity: NED → ENU
  out.vel_enu.x = static_cast<double>(vel_e_mms) * 1e-3;            // East
  out.vel_enu.y = static_cast<double>(vel_n_mms) * 1e-3;            // North
  out.vel_enu.z = -static_cast<double>(vel_d_mms) * 1e-3;           // Up = -Down

  // Position covariance diagonal in ENU.
  // hAcc is combined horizontal 1-sigma; split equally between E and N.
  const double h_sigma_m = static_cast<double>(h_acc_mm) * 1e-3;
  const double v_sigma_m = static_cast<double>(v_acc_mm) * 1e-3;
  const double h_var_per_axis = (h_sigma_m * h_sigma_m) * 0.5;
  const double v_var = v_sigma_m * v_sigma_m;
  // Skip the diagonal fallback if NAV-COV has already populated this epoch
  // (signalled by any non-zero off-diagonal entry).
  const bool pos_cov_from_nav_cov =
      out.pos_enu_cov[1] != 0.0 || out.pos_enu_cov[2] != 0.0 || out.pos_enu_cov[5] != 0.0;
  if (!pos_cov_from_nav_cov) {
    out.pos_enu_cov[0] = h_var_per_axis;  // EE
    out.pos_enu_cov[4] = h_var_per_axis;  // NN
    out.pos_enu_cov[8] = v_var;           // UU
  }

  // Velocity covariance diagonal (sAcc is 3D speed 1-sigma; split across 3 axes)
  const double s_sigma_mps = static_cast<double>(s_acc_mms) * 1e-3;
  const double v_var_per_axis = (s_sigma_mps * s_sigma_mps) / 3.0;
  const bool vel_cov_from_nav_cov =
      out.vel_enu_cov[1] != 0.0 || out.vel_enu_cov[2] != 0.0 || out.vel_enu_cov[5] != 0.0;
  if (!vel_cov_from_nav_cov) {
    out.vel_enu_cov[0] = v_var_per_axis;
    out.vel_enu_cov[4] = v_var_per_axis;
    out.vel_enu_cov[8] = v_var_per_axis;
  }

  return true;
}

}  // namespace pvt

// NAV-DOP parser (UBX-NAV-DOP, class=0x01 id=0x04)
namespace nav_dop {

constexpr int MIN_LEN      = 14;  // need through hDOP at offset 12
constexpr int OFFSET_GDOP  = 4;   // U2, ×0.01
constexpr int OFFSET_PDOP  = 6;
constexpr int OFFSET_TDOP  = 8;
constexpr int OFFSET_VDOP  = 10;
constexpr int OFFSET_HDOP  = 12;

template <typename T>
inline T read_le(const uint8_t* p) { T v; std::memcpy(&v, p, sizeof(T)); return v; }

/// Parse UBX-NAV-DOP payload, filling gdop/pdop/hdop/vdop in GnssSolution.
inline bool parseNavDop(const uint8_t* p, size_t len,
                        gnss_ros_standardization::msg::GnssSolution& out) {
  if (len < static_cast<size_t>(MIN_LEN)) return false;
  out.gdop = static_cast<float>(read_le<uint16_t>(p + OFFSET_GDOP)) * 0.01f;
  out.pdop = static_cast<float>(read_le<uint16_t>(p + OFFSET_PDOP)) * 0.01f;
  out.hdop = static_cast<float>(read_le<uint16_t>(p + OFFSET_HDOP)) * 0.01f;
  out.vdop = static_cast<float>(read_le<uint16_t>(p + OFFSET_VDOP)) * 0.01f;
  return true;
}

}  // namespace nav_dop

// NAV-COV parser (UBX-NAV-COV, class=0x01 id=0x36)
namespace nav_cov {

constexpr int MIN_LEN              = 64;
constexpr int OFFSET_POS_COV_VALID = 5;   // U1: 1 = posCov valid
constexpr int OFFSET_VEL_COV_VALID = 6;   // U1: 1 = velCov valid
// posCov upper triangle (NED frame, f4): NN/NE/ND/EE/ED/DD
constexpr int OFFSET_POS_NN = 16;
constexpr int OFFSET_POS_NE = 20;
constexpr int OFFSET_POS_ND = 24;
constexpr int OFFSET_POS_EE = 28;
constexpr int OFFSET_POS_ED = 32;
constexpr int OFFSET_POS_DD = 36;
// velCov upper triangle (NED frame, f4): same layout
constexpr int OFFSET_VEL_NN = 40;
constexpr int OFFSET_VEL_NE = 44;
constexpr int OFFSET_VEL_ND = 48;
constexpr int OFFSET_VEL_EE = 52;
constexpr int OFFSET_VEL_ED = 56;
constexpr int OFFSET_VEL_DD = 60;

template <typename T>
inline T read_le(const uint8_t* p) { T v; std::memcpy(&v, p, sizeof(T)); return v; }

/// Parse UBX-NAV-COV payload, filling pos_enu_cov / vel_enu_cov (full 3x3).
/// NED→ENU axis permutation: sign flip on cross-terms involving D axis.
/// Overwrites pos_enu_cov only when posCovValid=1, vel_enu_cov only when velCovValid=1.
inline bool parseNavCov(const uint8_t* p, size_t len,
                        gnss_ros_standardization::msg::GnssSolution& out) {
  if (len < static_cast<size_t>(MIN_LEN)) return false;
  const bool pos_valid = p[OFFSET_POS_COV_VALID] != 0;
  const bool vel_valid = p[OFFSET_VEL_COV_VALID] != 0;
  if (!pos_valid && !vel_valid) return false;

  if (pos_valid) {
    const float nn = read_le<float>(p + OFFSET_POS_NN);
    const float ne = read_le<float>(p + OFFSET_POS_NE);
    const float nd = read_le<float>(p + OFFSET_POS_ND);
    const float ee = read_le<float>(p + OFFSET_POS_EE);
    const float ed = read_le<float>(p + OFFSET_POS_ED);
    const float dd = read_le<float>(p + OFFSET_POS_DD);
    out.pos_enu_cov[0] =  static_cast<double>(ee);
    out.pos_enu_cov[1] =  static_cast<double>(ne);
    out.pos_enu_cov[2] = -static_cast<double>(ed);
    out.pos_enu_cov[3] =  static_cast<double>(ne);
    out.pos_enu_cov[4] =  static_cast<double>(nn);
    out.pos_enu_cov[5] = -static_cast<double>(nd);
    out.pos_enu_cov[6] = -static_cast<double>(ed);
    out.pos_enu_cov[7] = -static_cast<double>(nd);
    out.pos_enu_cov[8] =  static_cast<double>(dd);
  }
  if (vel_valid) {
    const float nn = read_le<float>(p + OFFSET_VEL_NN);
    const float ne = read_le<float>(p + OFFSET_VEL_NE);
    const float nd = read_le<float>(p + OFFSET_VEL_ND);
    const float ee = read_le<float>(p + OFFSET_VEL_EE);
    const float ed = read_le<float>(p + OFFSET_VEL_ED);
    const float dd = read_le<float>(p + OFFSET_VEL_DD);
    out.vel_enu_cov[0] =  static_cast<double>(ee);
    out.vel_enu_cov[1] =  static_cast<double>(ne);
    out.vel_enu_cov[2] = -static_cast<double>(ed);
    out.vel_enu_cov[3] =  static_cast<double>(ne);
    out.vel_enu_cov[4] =  static_cast<double>(nn);
    out.vel_enu_cov[5] = -static_cast<double>(nd);
    out.vel_enu_cov[6] = -static_cast<double>(ed);
    out.vel_enu_cov[7] = -static_cast<double>(nd);
    out.vel_enu_cov[8] =  static_cast<double>(dd);
  }
  return true;
}

}  // namespace nav_cov

// NAV-POSECEF parser (UBX-NAV-POSECEF, class=0x01 id=0x01)
// Payload: iTOW(U4) + ecefX/Y/Z(I4, cm) + pAcc(U4, cm) = 20 bytes
namespace nav_posecef {

constexpr int MIN_LEN     = 20;
constexpr int OFFSET_ITOW = 0;
constexpr int OFFSET_X    = 4;
constexpr int OFFSET_Y    = 8;
constexpr int OFFSET_Z    = 12;
constexpr int OFFSET_PACC = 16;

template <typename T>
inline T read_le(const uint8_t* p) { T v; std::memcpy(&v, p, sizeof(T)); return v; }

inline bool parseNavPosEcef(const uint8_t* p, size_t len,
                            gnss_ros_standardization::msg::GnssSolution& out) {
  if (len < static_cast<size_t>(MIN_LEN)) return false;
  out.pos_ecef.x = static_cast<double>(read_le<int32_t>(p + OFFSET_X)) * 0.01;
  out.pos_ecef.y = static_cast<double>(read_le<int32_t>(p + OFFSET_Y)) * 0.01;
  out.pos_ecef.z = static_cast<double>(read_le<int32_t>(p + OFFSET_Z)) * 0.01;
  return true;
}

}  // namespace nav_posecef

// NAV-VELECEF parser (UBX-NAV-VELECEF, class=0x01 id=0x11)
// Payload: iTOW(U4) + ecefVX/VY/VZ(I4, cm/s) + sAcc(U4, cm/s) = 20 bytes
namespace nav_velecef {

constexpr int MIN_LEN     = 20;
constexpr int OFFSET_ITOW = 0;
constexpr int OFFSET_VX   = 4;
constexpr int OFFSET_VY   = 8;
constexpr int OFFSET_VZ   = 12;
constexpr int OFFSET_SACC = 16;

template <typename T>
inline T read_le(const uint8_t* p) { T v; std::memcpy(&v, p, sizeof(T)); return v; }

inline bool parseNavVelEcef(const uint8_t* p, size_t len,
                            gnss_ros_standardization::msg::GnssSolution& out) {
  if (len < static_cast<size_t>(MIN_LEN)) return false;
  out.vel_ecef.x = static_cast<double>(read_le<int32_t>(p + OFFSET_VX)) * 0.01;
  out.vel_ecef.y = static_cast<double>(read_le<int32_t>(p + OFFSET_VY)) * 0.01;
  out.vel_ecef.z = static_cast<double>(read_le<int32_t>(p + OFFSET_VZ)) * 0.01;
  return true;
}

}  // namespace nav_velecef

}  // namespace ubx
}  // namespace gnss_ros_standardization