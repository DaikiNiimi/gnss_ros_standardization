#pragma once
#include "gnss_ros_standardization/receiver_diagnostics.h"
#include "gnss_ros_standardization/measurement_validation.hpp"
/// @file ubx_protocol.hpp
/// @brief u-blox UBX protocol constants, CFG-VALSET keys, and utility types.
///
/// This header centralizes all UBX protocol definitions used by the driver node,
/// eliminating hardcoded magic numbers from the implementation.

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <string>
#include <string_view>
#include <stdexcept>
#include <vector>

#include "gnss_ros_standardization/msg/gnss_solution.hpp"

namespace gnss_ros_standardization {
namespace ubx {
// DATA0 has no version byte; DATA1 before protocol27 has no signal ID.
// They cannot be inferred from arbitrary Reserved values in a passive stream.
inline const char* rawxLayoutOption(const std::string& layout) {
  if (layout == "auto") return "";
  if (layout == "data0") return " -RAWX0";
  if (layout == "data1_pre27") return " -RAWX1L";
  throw std::invalid_argument("rawx_layout must be auto, data0 or data1_pre27");
}

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
constexpr uint8_t ID_NAV_PVT       = 0x07;
constexpr uint8_t ID_NAV_DOP       = 0x04;
constexpr uint8_t ID_NAV_COV       = 0x36;
constexpr uint8_t ID_NAV_POSECEF   = 0x01;
constexpr uint8_t ID_NAV_VELECEF   = 0x11;
constexpr uint8_t ID_NAV_HPPOSECEF = 0x13;
constexpr uint8_t ID_NAV_HPPOSLLH  = 0x14;
constexpr uint8_t ID_NAV_CLOCK     = 0x22;

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

  // sTtag LSB in seconds: 39.0625 us, i.e. 256 ticks = 10 ms.
  //
  // NOT a documented constant. The ZED-F9R Interface Description's ESF-RAW
  // payload table leaves both Scale and Unit blank for sTtag ("sensor time
  // tag") and refers to "Raw sensor data output" in the Integration manual,
  // which does not give a unit either; gpsd's ubxtool documents it only as
  // "in unrelated units, but at least always increases". So this value is an
  // assumption that a firmware or product change could invalidate silently,
  // which is why ins_utils::SensorTickClock measures its drift against the
  // host clock and says so in the log.
  //
  // Used as the tick of that clock, which maps the whole ESF-RAW stream onto
  // one monotone timeline - not, as before, only to space samples within a
  // single batch.
  constexpr double STTAG_TICK_S = 39.0625e-6;

  // One IMU sample set reassembled from an ESF-RAW message. ESF-RAW batches
  // N × { data(u4) + sTtag(u4) } blocks; blocks sharing an sTtag belong to
  // the same sample instant.
  struct Sample {
    double   accel[3]{0.0, 0.0, 0.0};  // m/s²
    double   gyro[3]{0.0, 0.0, 0.0};   // rad/s
    bool     has_accel[3]{false, false, false};
    bool     has_gyro[3]{false, false, false};
    uint32_t sttag{0};

    bool complete() const {
      return has_accel[0] && has_accel[1] && has_accel[2] &&
             has_gyro[0] && has_gyro[1] && has_gyro[2];
    }
    bool any() const {
      return has_accel[0] || has_accel[1] || has_accel[2] ||
             has_gyro[0]  || has_gyro[1]  || has_gyro[2];
    }
  };

  /// Sign-extend the lower 24 bits of a uint32 into an int32.
  inline int32_t signExtend24(uint32_t v) {
    return (v & 0x00800000u) ? static_cast<int32_t>(v | 0xFF000000u)
                             : static_cast<int32_t>(v & 0x00FFFFFFu);
  }

  /// Parse an ESF-RAW payload (4-byte reserved header + N × { data(u4) +
  /// sTtag(u4) }; data low 24 bits = signed measurement, high 8 bits = type)
  /// into per-sample sets, splitting on sTtag changes. Gyro is converted to
  /// rad/s, accel to m/s². Samples with no recognized axis are dropped.
  inline std::vector<Sample> parseEsfRawSamples(const std::vector<uint8_t>& payload) {
    constexpr int kHeaderLen = 4;
    constexpr int kBlockLen  = 8;
    constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

    std::vector<Sample> samples;
    const int n = (static_cast<int>(payload.size()) - kHeaderLen) / kBlockLen;
    if (n <= 0 || (payload.size() - kHeaderLen) % kBlockLen != 0) return samples;

    Sample cur;
    bool cur_started = false;
    auto flush = [&]() {
      if (cur_started && cur.any()) samples.push_back(cur);
      cur = Sample{};
      cur_started = false;
    };

    for (int i = 0; i < n; ++i) {
      const uint8_t* p = payload.data() + kHeaderLen + i * kBlockLen;
      uint32_t data = 0, sttag = 0;
      std::memcpy(&data,  p,     4);
      std::memcpy(&sttag, p + 4, 4);
      // ESF-RAW uses eight type bits (U2 pp281-282, U3 p73), unlike
      // ESF-MEAS. Masking to six bits aliases unrelated sensor types to IMU.
      const uint8_t type = static_cast<uint8_t>(data >> 24);
      const int32_t val  = signExtend24(data);

      if (cur_started && sttag != cur.sttag) flush();
      cur.sttag   = sttag;
      cur_started = true;

      switch (type) {
        case TYPE_GYRO_X:  cur.gyro[0]  = val * GYRO_SCALE * kDeg2Rad; cur.has_gyro[0]  = true; break;
        case TYPE_GYRO_Y:  cur.gyro[1]  = val * GYRO_SCALE * kDeg2Rad; cur.has_gyro[1]  = true; break;
        case TYPE_GYRO_Z:  cur.gyro[2]  = val * GYRO_SCALE * kDeg2Rad; cur.has_gyro[2]  = true; break;
        case TYPE_ACCEL_X: cur.accel[0] = val * ACCEL_SCALE;           cur.has_accel[0] = true; break;
        case TYPE_ACCEL_Y: cur.accel[1] = val * ACCEL_SCALE;           cur.has_accel[1] = true; break;
        case TYPE_ACCEL_Z: cur.accel[2] = val * ACCEL_SCALE;           cur.has_accel[2] = true; break;
        default: GNSS_RX_EVENT(GNSS_RX_SELECTION,1); break;  // ignore other sensor types (temp, wheel speed, etc.)
      }
    }
    flush();
    return samples;
  }
}

// Assemble axes split across messages without inventing zero measurements.
class EsfRawAssembler {
 public:
  std::vector<esf_raw::Sample> push(const std::vector<uint8_t>& payload) {
    std::vector<esf_raw::Sample> ready;
    for (const auto& sample : esf_raw::parseEsfRawSamples(payload)) {
      if (sent_ && sample.sttag == last_sent_) { GNSS_RX_EVENT(GNSS_RX_DUPLICATE_INPUT,1); continue; }
      if (pending_.any() && pending_.sttag != sample.sttag) GNSS_RX_EVENT(GNSS_RX_ASSEMBLY_REPLACED,1);
      if (!pending_.any() || pending_.sttag != sample.sttag) pending_ = {};
      pending_.sttag = sample.sttag;
      for (int i = 0; i < 3; ++i) {
        if (sample.has_accel[i]) { pending_.accel[i] = sample.accel[i]; pending_.has_accel[i] = true; }
        if (sample.has_gyro[i]) { pending_.gyro[i] = sample.gyro[i]; pending_.has_gyro[i] = true; }
      }
      if (pending_.complete()) {
        ready.push_back(pending_); last_sent_ = pending_.sttag; sent_ = true; pending_ = {};
      }
    }
    return ready;
  }
  bool pending() const { return pending_.any(); }
  void reset() { pending_ = {}; sent_ = false; }
 private:
  esf_raw::Sample pending_{};
  uint32_t last_sent_{0};
  bool sent_{false};
};

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
// Note: X20P supports B1C/B2A/B3I, but the RTKLIB decoder does not.
constexpr uint32_t CFG_SIGNAL_BDS_ENA     = 0x10310022;
constexpr uint32_t CFG_SIGNAL_BDS_B1I_ENA = 0x1031000D;  // B1I (F9P)
constexpr uint32_t CFG_SIGNAL_BDS_B1C_ENA = 0x1031000F;  // B1C (X20P)
constexpr uint32_t CFG_SIGNAL_BDS_B2I_ENA = 0x1031000E;  // B2I (F9P)
constexpr uint32_t CFG_SIGNAL_BDS_B2A_ENA = 0x10310028;  // B2a (X20P)

// GLONASS
constexpr uint32_t CFG_SIGNAL_GLO_ENA    = 0x10310025;
constexpr uint32_t CFG_SIGNAL_GLO_L1_ENA = 0x10310018;
constexpr uint32_t CFG_SIGNAL_GLO_L2_ENA = 0x1031001A;  // G2C/A (GLO L2 OF)

// NavIC (X20 HPG configuration interface)
constexpr uint32_t CFG_SIGNAL_NAVIC_ENA = 0x10310026;
constexpr uint32_t CFG_SIGNAL_NAVIC_L5_ENA = 0x1031001D;

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
constexpr uint32_t CFG_MSGOUT_UBX_NAV_CLOCK_I2C   = 0x20910065;
constexpr uint32_t CFG_MSGOUT_UBX_NAV_POSECEF_I2C = 0x20910024;
constexpr uint32_t CFG_MSGOUT_UBX_NAV_VELECEF_I2C = 0x2091003D;
constexpr uint32_t CFG_MSGOUT_UBX_NAV_HPPOSECEF_I2C = 0x2091002E;
constexpr uint32_t CFG_MSGOUT_UBX_NAV_HPPOSLLH_I2C  = 0x20910033;

constexpr uint32_t CFG_MSGOUT_NMEA_ID_GGA_I2C   = 0x209100BA;
constexpr uint32_t CFG_MSGOUT_NMEA_ID_GLL_I2C   = 0x209100C9;
constexpr uint32_t CFG_MSGOUT_NMEA_ID_GSA_I2C   = 0x209100BF;
constexpr uint32_t CFG_MSGOUT_NMEA_ID_GSV_I2C   = 0x209100C4;
constexpr uint32_t CFG_MSGOUT_NMEA_ID_RMC_I2C   = 0x209100AB;
constexpr uint32_t CFG_MSGOUT_NMEA_ID_VTG_I2C   = 0x209100B0;
constexpr uint32_t CFG_MSGOUT_NMEA_ID_GST_I2C   = 0x209100D3;
constexpr uint32_t CFG_MSGOUT_NMEA_ID_ZDA_I2C   = 0x209100D8;

// --- IMU Output ---
// ESF-INS (calibrated angular rate + acceleration): base I2C key, +port offset for UART1/2/USB
constexpr uint32_t CFG_MSGOUT_UBX_ESF_INS_I2C  = 0x20910114;
// ESF-RAW (uncalibrated raw IMU measurements): base I2C key
constexpr uint32_t CFG_MSGOUT_UBX_ESF_RAW_I2C  = 0x2091029F;

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

/// Supported stream type prefixes (RTKLIB stream types)
constexpr StreamTypeDef kStreamTypes[] = {
    {"tcpcli://", STR_TCPCLI},
    {"tcpsvr://", STR_TCPSVR},
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
// `valid` bitfield (U1). The Integration manual warns that "iTOW values may
// not be valid ... it is not recommended to use the iTOW field for any other
// purpose", and points at NAV-PVT precisely because it carries these flags.
// Anything that dates a measurement from iTOW must gate on them.
constexpr int NAV_PVT_OFFSET_VALID   = 11;
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
constexpr uint8_t VALID_DATE       = 1u << 0;
constexpr uint8_t VALID_TIME       = 1u << 1;
constexpr uint8_t VALID_FULLY_RESOLVED = 1u << 2;

/// True when NAV-PVT's own flags say the epoch's time is trustworthy enough
/// to date a measurement from. Date + time + fully resolved; magnetic
/// declination validity (bit 3) is irrelevant here.
inline bool navPvtTimeIsValid(const uint8_t* p, size_t len) {
  if (len < static_cast<size_t>(NAV_PVT_OFFSET_VALID + 1)) return GNSS_RX_REJECT(GNSS_RX_LENGTH);
  const uint8_t v = p[NAV_PVT_OFFSET_VALID];
  return (v & (VALID_DATE | VALID_TIME | VALID_FULLY_RESOLVED)) ==
         (VALID_DATE | VALID_TIME | VALID_FULLY_RESOLVED);
}

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
/// Covariance contract: explicit per-epoch NAV-COV validity flags preserve the
/// full covariance. Otherwise hAcc/vAcc/sAcc provide diagonal fallback values;
/// zero off-diagonal entries do not imply missing NAV-COV.
inline bool parseNavPvt(const uint8_t* p, size_t len,
                        gnss_ros_standardization::msg::GnssSolution& out,
                        bool pos_cov_from_nav_cov = false,
                        bool vel_cov_from_nav_cov = false) {
  if (len < static_cast<size_t>(NAV_PVT_MIN_LEN)) return GNSS_RX_REJECT(GNSS_RX_LENGTH);

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

  if (itow_ms >= 604800000u || std::abs(static_cast<double>(lat_e7)) > 900000000.0 ||
      std::abs(static_cast<double>(lon_e7)) > 1800000000.0) return GNSS_RX_REJECT(itow_ms>=604800000u ? GNSS_RX_TIME : GNSS_RX_VALUE);
  // Map UBX fix type/flags to GnssSolution status
  uint8_t status = gnss_ros_standardization::msg::GnssSolution::STATUS_NONE;
  const bool fix_ok = (flags & FLAG_GNSSFIXOK) != 0;
  if (fix_ok && fix_type == FIX_DR_ONLY) {
    status = gnss_ros_standardization::msg::GnssSolution::STATUS_DR;
  } else if (fix_ok && (fix_type == FIX_2D || fix_type == FIX_3D || fix_type == FIX_GNSS_DR)) {
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
  if (!pos_cov_from_nav_cov) {
    out.pos_enu_cov[0] = h_var_per_axis;  // EE
    out.pos_enu_cov[4] = h_var_per_axis;  // NN
    out.pos_enu_cov[8] = v_var;           // UU
  }

  // Velocity covariance diagonal (sAcc is 3D speed 1-sigma; split across 3 axes)
  const double s_sigma_mps = static_cast<double>(s_acc_mms) * 1e-3;
  const double v_var_per_axis = (s_sigma_mps * s_sigma_mps) / 3.0;
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
  if (len < static_cast<size_t>(MIN_LEN)) return GNSS_RX_REJECT(GNSS_RX_LENGTH);
  out.gdop = static_cast<float>(read_le<uint16_t>(p + OFFSET_GDOP)) * 0.01f;
  out.pdop = static_cast<float>(read_le<uint16_t>(p + OFFSET_PDOP)) * 0.01f;
  out.hdop = static_cast<float>(read_le<uint16_t>(p + OFFSET_HDOP)) * 0.01f;
  out.vdop = static_cast<float>(read_le<uint16_t>(p + OFFSET_VDOP)) * 0.01f;
  return true;
}

}  // namespace nav_dop

// NAV-CLOCK parser (UBX-NAV-CLOCK, class=0x01 id=0x22).
//
// Carries the receiver's own clock solution: iTOW plus clkB, the receiver
// clock's bias against GPS time. That makes it the best available anchor for
// tying a free-running sensor counter (ESF-RAW sTtag) to GNSS time - better
// than NAV-PVT, which gives the epoch time but not the clock model. Note what
// it does NOT remove: clkB is the clock bias, not the receiver's output
// latency, so a residual remains either way.
namespace nav_clock {

constexpr int MIN_LEN      = 20;
constexpr int OFFSET_ITOW  = 0;   // U4, ms
constexpr int OFFSET_CLKB  = 4;   // I4, ns   - clock bias
constexpr int OFFSET_CLKD  = 8;   // I4, ns/s - clock drift
constexpr int OFFSET_TACC  = 12;  // U4, ns   - time accuracy estimate

struct Result {
  uint32_t itow_ms{0};
  double   clk_bias_s{0.0};
  double   clk_drift_s_per_s{0.0};
  double   time_acc_s{0.0};
};

inline bool parseNavClock(const uint8_t* p, size_t len, Result& out) {
  if (len < static_cast<size_t>(MIN_LEN)) return GNSS_RX_REJECT(GNSS_RX_LENGTH);
  out.itow_ms           = nav_dop::read_le<uint32_t>(p + OFFSET_ITOW);
  out.clk_bias_s        = nav_dop::read_le<int32_t>(p + OFFSET_CLKB) * 1e-9;
  out.clk_drift_s_per_s = nav_dop::read_le<int32_t>(p + OFFSET_CLKD) * 1e-9;
  out.time_acc_s        = nav_dop::read_le<uint32_t>(p + OFFSET_TACC) * 1e-9;
  return true;
}
}  // namespace nav_clock

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
  if (len < static_cast<size_t>(MIN_LEN)) return GNSS_RX_REJECT(GNSS_RX_LENGTH);
  if (p[4] != 0) return GNSS_RX_REJECT(GNSS_RX_VERSION);
  const bool pos_valid = p[OFFSET_POS_COV_VALID] == 1;
  const bool vel_valid = p[OFFSET_VEL_COV_VALID] == 1;
  if (!pos_valid && !vel_valid) return GNSS_RX_REJECT(GNSS_RX_QUALITY);
  for (int base : {OFFSET_POS_NN, OFFSET_VEL_NN}) {
    if ((base == OFFSET_POS_NN && !pos_valid) || (base == OFFSET_VEL_NN && !vel_valid)) continue;
    double c[6];
    for (int i = 0; i < 6; ++i) { c[i] = read_le<float>(p + base + 4*i); if (!std::isfinite(c[i])) return GNSS_RX_REJECT(GNSS_RX_VALUE); }
    if (!validation::covariance(c[0], c[3], c[5], c[1], c[2], c[4])) return GNSS_RX_REJECT(GNSS_RX_VALUE);
  }


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
  if (len < static_cast<size_t>(MIN_LEN)) return GNSS_RX_REJECT(GNSS_RX_LENGTH);
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
  if (len < static_cast<size_t>(MIN_LEN)) return GNSS_RX_REJECT(GNSS_RX_LENGTH);
  out.vel_ecef.x = static_cast<double>(read_le<int32_t>(p + OFFSET_VX)) * 0.01;
  out.vel_ecef.y = static_cast<double>(read_le<int32_t>(p + OFFSET_VY)) * 0.01;
  out.vel_ecef.z = static_cast<double>(read_le<int32_t>(p + OFFSET_VZ)) * 0.01;
  return true;
}

}  // namespace nav_velecef

// NAV-HPPOSECEF parser (UBX-NAV-HPPOSECEF, class=0x01 id=0x13)
// Payload: version(U1) + reserved(3) + iTOW(U4) + ecefX/Y/Z(I4, cm)
//          + ecefXHp/YHp/ZHp(I1, 0.1 mm) + flags(U1, bit0=invalidEcef)
//          + pAcc(U4, 0.1 mm) = 28 bytes
// NOTE: iTOW is at offset 4, unlike most NAV-* messages (offset 0).
namespace nav_hpposecef {

constexpr int MIN_LEN      = 28;
constexpr int OFFSET_ITOW  = 4;
constexpr int OFFSET_X     = 8;
constexpr int OFFSET_Y     = 12;
constexpr int OFFSET_Z     = 16;
constexpr int OFFSET_XHP   = 20;
constexpr int OFFSET_YHP   = 21;
constexpr int OFFSET_ZHP   = 22;
constexpr int OFFSET_FLAGS = 23;
constexpr int OFFSET_PACC  = 24;
constexpr uint8_t FLAG_INVALID_ECEF = 1u << 0;

template <typename T>
inline T read_le(const uint8_t* p) { T v; std::memcpy(&v, p, sizeof(T)); return v; }

/// Parse UBX-NAV-HPPOSECEF payload into a high-precision ECEF position [m].
/// Combines the cm-resolution base and the 0.1 mm Hp correction in double,
/// preserving the full 0.1 mm resolution. Returns false if payload too short
/// or invalidEcef flag is set.
inline bool parseNavHpPosEcef(const uint8_t* p, size_t len, double out_xyz[3]) {
  if (len < static_cast<size_t>(MIN_LEN)) return GNSS_RX_REJECT(GNSS_RX_LENGTH);
  if (p[0] != 0 || read_le<uint32_t>(p + OFFSET_ITOW) >= 604800000) return GNSS_RX_REJECT(p[0]!=0 ? GNSS_RX_VERSION : GNSS_RX_TIME);
  if (p[OFFSET_FLAGS] & FLAG_INVALID_ECEF) return GNSS_RX_REJECT(GNSS_RX_QUALITY);
  out_xyz[0] = static_cast<double>(read_le<int32_t>(p + OFFSET_X)) * 0.01 +
               static_cast<double>(read_le<int8_t>(p + OFFSET_XHP)) * 1e-4;
  out_xyz[1] = static_cast<double>(read_le<int32_t>(p + OFFSET_Y)) * 0.01 +
               static_cast<double>(read_le<int8_t>(p + OFFSET_YHP)) * 1e-4;
  out_xyz[2] = static_cast<double>(read_le<int32_t>(p + OFFSET_Z)) * 0.01 +
               static_cast<double>(read_le<int8_t>(p + OFFSET_ZHP)) * 1e-4;
  return true;
}

}  // namespace nav_hpposecef

// NAV-HPPOSLLH parser (UBX-NAV-HPPOSLLH, class=0x01 id=0x14)
// Payload: version(U1) + reserved(2) + flags(U1, bit0=invalidLlh) + iTOW(U4)
//          + lon/lat(I4, 1e-7 deg) + height/hMSL(I4, mm)
//          + lonHp/latHp(I1, 1e-9 deg) + heightHp/hMSLHp(I1, 0.1 mm)
//          + hAcc/vAcc(U4, 0.1 mm) = 36 bytes
// NOTE: iTOW is at offset 4, unlike most NAV-* messages (offset 0).
namespace nav_hpposllh {

constexpr int MIN_LEN         = 36;
constexpr int OFFSET_FLAGS    = 3;
constexpr int OFFSET_ITOW     = 4;
constexpr int OFFSET_LON      = 8;
constexpr int OFFSET_LAT      = 12;
constexpr int OFFSET_HEIGHT   = 16;
constexpr int OFFSET_LONHP    = 24;
constexpr int OFFSET_LATHP    = 25;
constexpr int OFFSET_HEIGHTHP = 26;
constexpr int OFFSET_HACC     = 28;
constexpr int OFFSET_VACC     = 32;
constexpr uint8_t FLAG_INVALID_LLH = 1u << 0;

template <typename T>
inline T read_le(const uint8_t* p) { T v; std::memcpy(&v, p, sizeof(T)); return v; }

/// High-precision geodetic position parsed from NAV-HPPOSLLH. Kept out of
/// GnssSolution on purpose: NAV-PVT writes latitude/longitude/altitude
/// unconditionally, so the HP values are applied at epoch-flush time to stay
/// independent of in-epoch message arrival order.
struct Result {
  double lat_deg;   ///< geodetic latitude  [deg], 1e-9 deg resolution
  double lon_deg;   ///< geodetic longitude [deg], 1e-9 deg resolution
  double alt_m;     ///< ellipsoidal height [m], 0.1 mm resolution
  double hacc_m;    ///< horizontal accuracy 1-sigma [m]
  double vacc_m;    ///< vertical accuracy 1-sigma [m]
};

/// Parse UBX-NAV-HPPOSLLH payload. Base (1e-7 deg / mm) and Hp correction
/// (1e-9 deg / 0.1 mm) are combined in double in a single expression — double
/// carries ~15-16 significant digits, ample for 1e-9 deg at |lat| <= 90.
/// Returns false if payload too short or invalidLlh flag is set.
inline bool parseNavHpPosLlh(const uint8_t* p, size_t len, Result& out) {
  if (len < static_cast<size_t>(MIN_LEN)) return GNSS_RX_REJECT(GNSS_RX_LENGTH);
  if (p[0] != 0 || read_le<uint32_t>(p + OFFSET_ITOW) >= 604800000) return GNSS_RX_REJECT(p[0]!=0 ? GNSS_RX_VERSION : GNSS_RX_TIME);
  if (p[OFFSET_FLAGS] & FLAG_INVALID_LLH) return GNSS_RX_REJECT(GNSS_RX_QUALITY);
  const double lat = read_le<int32_t>(p + OFFSET_LAT) * 1e-7 + read_le<int8_t>(p + OFFSET_LATHP) * 1e-9;
  const double lon = read_le<int32_t>(p + OFFSET_LON) * 1e-7 + read_le<int8_t>(p + OFFSET_LONHP) * 1e-9;
  if (std::abs(lat) > 90 || std::abs(lon) > 180) return GNSS_RX_REJECT(GNSS_RX_VALUE);
  out.lon_deg = static_cast<double>(read_le<int32_t>(p + OFFSET_LON)) * 1e-7 +
                static_cast<double>(read_le<int8_t>(p + OFFSET_LONHP)) * 1e-9;
  out.lat_deg = static_cast<double>(read_le<int32_t>(p + OFFSET_LAT)) * 1e-7 +
                static_cast<double>(read_le<int8_t>(p + OFFSET_LATHP)) * 1e-9;
  out.alt_m   = static_cast<double>(read_le<int32_t>(p + OFFSET_HEIGHT)) * 1e-3 +
                static_cast<double>(read_le<int8_t>(p + OFFSET_HEIGHTHP)) * 1e-4;
  out.hacc_m  = static_cast<double>(read_le<uint32_t>(p + OFFSET_HACC)) * 1e-4;
  out.vacc_m  = static_cast<double>(read_le<uint32_t>(p + OFFSET_VACC)) * 1e-4;
  return true;
}

}  // namespace nav_hpposllh

}  // namespace ubx
}  // namespace gnss_ros_standardization
