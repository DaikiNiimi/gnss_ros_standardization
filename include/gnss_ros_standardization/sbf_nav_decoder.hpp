#pragma once
/// @file sbf_nav_decoder.hpp
/// @brief Parses Septentrio SBF decoded *Nav block bodies into RTKLIB eph_t/geph_t.
///
/// Reference: Septentrio mosaic-X5 Firmware Reference Manual v4.x
/// Body = bytes after the 8-byte SBF block header stripped by parseSbfByte().
///
/// All multi-byte reads use memcpy (handles misaligned f8 fields safely).

#include <cmath>
#include <cstring>
#include <cstdint>
#include <vector>

#include "rtklib.h"

namespace gnss_ros_standardization {
namespace sbf {
namespace nav {

// ---- little-endian field readers ----
inline uint8_t  ru1(const uint8_t* p) { return *p; }
inline uint16_t ru2(const uint8_t* p) { uint16_t v; std::memcpy(&v, p, 2); return v; }
inline uint32_t ru4(const uint8_t* p) { uint32_t v; std::memcpy(&v, p, 4); return v; }
inline float    rf4(const uint8_t* p) { float    v; std::memcpy(&v, p, 4); return v; }
inline double   rf8(const uint8_t* p) { double   v; std::memcpy(&v, p, 8); return v; }

// Septentrio SVID → RTKLIB sat — mirrors svid2sat() in septentrio.c
inline int svidToSat(int svid) {
    if (svid <=  37) return satno(SYS_GPS, svid);
    if (svid <=  61) return satno(SYS_GLO, svid - 37);
    if (svid ==  62) return 0;
    if (svid <=  68) return satno(SYS_GLO, svid - 38);
    if (svid <=  70) return 0;
    if (svid <= 106) return satno(SYS_GAL, svid - 70);
    if (svid <= 119) return 0;
    if (svid <= 140) return satno(SYS_SBS, svid);
    if (svid <= 180) return satno(SYS_CMP, svid - 140);
    if (svid <= 187) return satno(SYS_QZS, svid - 180 + 192);
    if (svid <= 190) return 0;
    if (svid <= 197) return satno(SYS_IRN, svid - 190);
    if (svid <= 215) return satno(SYS_SBS, svid - 57);
    if (svid <= 222) return satno(SYS_IRN, svid - 208);
    if (svid <= 245) return satno(SYS_CMP, svid - 182);
    return 0;
}

// =============================================================================
// GPS / QZS / NavIC Nav — GPS LNAV Keplerian format
// Offsets are body-relative (after 8-byte SBF header)
// =============================================================================
namespace gps_off {
    constexpr int TOW    =   0;  // u4 current GPS time-of-week (ms)
    constexpr int WNC    =   4;  // u2 current GPS week
    constexpr int SVID   =   6;  // u1 Septentrio satellite ID
    // byte 7: reserved
    constexpr int WN     =   8;  // u2 GPS week of nav message
    constexpr int CA_P   =  10;  // u1 code on L2
    constexpr int URA    =  11;  // u1 URA index (0-15)
    constexpr int HEALTH =  12;  // u2 SV health
    constexpr int IODC   =  14;  // u2 issue of data clock (10-bit)
    constexpr int IODE   =  16;  // u1 issue of data ephemeris
    // byte 17: FitIntervalFlag, byte 18: L2Pflag, byte 19: reserved
    constexpr int TGD    =  20;  // f4 group delay (s)
    constexpr int TOC    =  24;  // u4 clock epoch (s)
    constexpr int AF2    =  28;  // f4 clock drift rate (s/s²)
    constexpr int AF1    =  32;  // f4 clock drift (s/s)
    constexpr int AF0    =  36;  // f4 clock bias (s)
    constexpr int CRS    =  40;  // f4 (m)
    constexpr int DELN   =  44;  // f4 mean motion correction (rad/s)
    constexpr int M0     =  48;  // f8 mean anomaly (rad)
    constexpr int CUC    =  56;  // f4 (rad)
    constexpr int ECC    =  60;  // f8 eccentricity
    constexpr int CUS    =  68;  // f4 (rad)
    constexpr int SQRTA  =  72;  // f8 sqrt(semi-major axis) (m^0.5)
    constexpr int TOE    =  80;  // u4 reference time ephemeris (s)
    constexpr int CIC    =  84;  // f4 (rad)
    constexpr int OMG0   =  88;  // f8 longitude of ascending node (rad)
    constexpr int CIS    =  96;  // f4 (rad)
    constexpr int I0     = 100;  // f8 inclination (rad)
    constexpr int CRC    = 108;  // f4 (m)
    constexpr int OMG    = 112;  // f8 argument of perigee (rad)
    constexpr int OMGD   = 120;  // f8 rate of right ascension (rad/s)
    constexpr int IDOT   = 128;  // f4 rate of inclination (rad/s)
    constexpr int MIN_LEN = 132;
}

inline bool parseGPSLike(const std::vector<uint8_t>& body, eph_t& eph, int sys) {
    using namespace gps_off;
    if (static_cast<int>(body.size()) < MIN_LEN) return false;
    const uint8_t* p = body.data();

    const int svid = ru1(p + SVID);
    const int sat  = svidToSat(svid);
    if (!sat) return false;
    int prn = 0;
    if (satsys(sat, &prn) != sys) return false;

    // Health: lower 6 bits all set = do-not-use (IS-GPS-200 §20.3.3.5.1.3)
    const uint16_t hlth = ru2(p + HEALTH);
    if ((hlth & 0x003F) == 0x003F) return false;

    const int    wn    = static_cast<int>(ru2(p + WN));
    const double toc_s = static_cast<double>(ru4(p + TOC));
    const double toe_s = static_cast<double>(ru4(p + TOE));
    const double sqrtA = rf8(p + SQRTA);

    // Sanity: eccentricity and sqrt_A
    const double ecc = rf8(p + ECC);
    if (ecc < 0.0 || ecc >= 1.0 || sqrtA <= 0.0) return false;

    std::memset(&eph, 0, sizeof(eph_t));
    eph.sat   = sat;
    eph.week  = wn;
    eph.code  = static_cast<int>(ru1(p + CA_P));
    eph.sva   = static_cast<int>(ru1(p + URA));
    eph.svh   = static_cast<int>(hlth & 0x003F);
    eph.iodc  = static_cast<int>(ru2(p + IODC));
    eph.iode  = static_cast<int>(ru1(p + IODE));
    eph.tgd[0] = static_cast<double>(rf4(p + TGD));
    eph.f2    = static_cast<double>(rf4(p + AF2));
    eph.f1    = static_cast<double>(rf4(p + AF1));
    eph.f0    = static_cast<double>(rf4(p + AF0));
    eph.crs   = static_cast<double>(rf4(p + CRS));
    eph.deln  = static_cast<double>(rf4(p + DELN));
    eph.M0    = rf8(p + M0);
    eph.cuc   = static_cast<double>(rf4(p + CUC));
    eph.e     = ecc;
    eph.cus   = static_cast<double>(rf4(p + CUS));
    eph.A     = sqrtA * sqrtA;
    eph.toes  = toe_s;
    eph.cic   = static_cast<double>(rf4(p + CIC));
    eph.OMG0  = rf8(p + OMG0);
    eph.cis   = static_cast<double>(rf4(p + CIS));
    eph.i0    = rf8(p + I0);
    eph.crc   = static_cast<double>(rf4(p + CRC));
    eph.omg   = rf8(p + OMG);
    eph.OMGd  = rf8(p + OMGD);
    eph.idot  = static_cast<double>(rf4(p + IDOT));

    eph.toe = gpst2time(wn, toe_s);
    eph.toc = gpst2time(wn, toc_s);
    eph.ttr = gpst2time(static_cast<int>(ru2(p + WNC)), ru4(p + TOW) / 1000.0);

    return true;
}

inline bool parseGPSNav(const std::vector<uint8_t>& body, eph_t& eph) {
    return parseGPSLike(body, eph, SYS_GPS);
}

inline bool parseQZSNav(const std::vector<uint8_t>& body, eph_t& eph) {
    return parseGPSLike(body, eph, SYS_QZS);
}

inline bool parseNavICNav(const std::vector<uint8_t>& body, eph_t& eph) {
    return parseGPSLike(body, eph, SYS_IRN);
}

// =============================================================================
// GLO Nav (ID 4004) — GLONASS state-vector ephemeris
// =============================================================================
namespace glo_off {
    constexpr int TOW      =  0;  // u4 current GPS TOW (ms)
    constexpr int WNC      =  4;  // u2 current GPS week
    constexpr int SVID     =  6;  // u1
    constexpr int FREQNR   =  7;  // u1 FCN+8 → FCN = FreqNr - 8
    // 8-9: SatSigMask (u2), 10-11: reserved
    constexpr int TB_MIN   = 12;  // u2 t_b in minutes within Moscow day (0-1440)
    constexpr int EN       = 14;  // u2 age of operation (days)
    constexpr int HEALTH   = 16;  // u1 Cn (0 = healthy)
    // 17: P1, 18: P2, 19: P3, 20: P4, 21: M, 22-23: reserved
    constexpr int TAU_N    = 24;  // f8 clock bias (s) — negative of tau_c in ICD
    constexpr int GAMMA_N  = 32;  // f4 relative frequency bias
    constexpr int DTAU_N   = 36;  // f4 L2-L1 time difference (s)
    // 40-43: reserved (u4 padding)
    constexpr int X        = 44;  // f8 position X (km)
    constexpr int Y        = 52;  // f8 position Y (km)
    constexpr int Z        = 60;  // f8 position Z (km)
    constexpr int Xdot     = 68;  // f4 velocity X (km/s)
    constexpr int Ydot     = 72;  // f4 velocity Y (km/s)
    constexpr int Zdot     = 76;  // f4 velocity Z (km/s)
    constexpr int Xdotdot  = 80;  // f4 acceleration X (km/s²)
    constexpr int Ydotdot  = 84;  // f4 acceleration Y (km/s²)
    constexpr int Zdotdot  = 88;  // f4 acceleration Z (km/s²)
    constexpr int MIN_LEN  = 92;
}

inline bool parseGLONav(const std::vector<uint8_t>& body, geph_t& geph) {
    using namespace glo_off;
    if (static_cast<int>(body.size()) < MIN_LEN) return false;
    const uint8_t* p = body.data();

    const int svid = ru1(p + SVID);
    const int sat  = svidToSat(svid);
    if (!sat) return false;
    int prn = 0;
    if (satsys(sat, &prn) != SYS_GLO) return false;

    if (ru1(p + HEALTH) != 0) return false;  // Cn=0 means healthy

    const int    wnc    = static_cast<int>(ru2(p + WNC));
    const double tow_s  = ru4(p + TOW) / 1000.0;
    const int    fcn    = static_cast<int>(ru1(p + FREQNR)) - 8;
    const int    tb_min = static_cast<int>(ru2(p + TB_MIN));
    const int    en     = static_cast<int>(ru2(p + EN));

    const double x = rf8(p + X);
    const double y = rf8(p + Y);
    const double z = rf8(p + Z);
    // Sanity: GLONASS orbit radius ≈ 25510 km from Earth center
    if (std::abs(x) > 35000.0 || std::abs(y) > 35000.0 || std::abs(z) > 35000.0) return false;

    std::memset(&geph, 0, sizeof(geph_t));
    geph.sat   = sat;
    geph.frq   = fcn;
    geph.svh   = 0;
    geph.age   = en;
    geph.iode  = tb_min / 15;  // 7-bit index: 15-min slot (0-95)
    geph.taun  = rf8(p + TAU_N);
    geph.gamn  = static_cast<double>(rf4(p + GAMMA_N));
    geph.dtaun = static_cast<double>(rf4(p + DTAU_N));

    geph.pos[0] = x * 1e3;                             // km → m
    geph.pos[1] = y * 1e3;
    geph.pos[2] = z * 1e3;
    geph.vel[0] = static_cast<double>(rf4(p + Xdot)) * 1e3;   // km/s → m/s
    geph.vel[1] = static_cast<double>(rf4(p + Ydot)) * 1e3;
    geph.vel[2] = static_cast<double>(rf4(p + Zdot)) * 1e3;
    geph.acc[0] = static_cast<double>(rf4(p + Xdotdot)) * 1e3;  // km/s² → m/s²
    geph.acc[1] = static_cast<double>(rf4(p + Ydotdot)) * 1e3;
    geph.acc[2] = static_cast<double>(rf4(p + Zdotdot)) * 1e3;

    geph.tof = gpst2time(wnc, tow_s);

    // Construct toe: t_b is minutes from Moscow midnight (UTC+3)
    // Moscow midnight in UTC = UTC day start - 3 h
    const gtime_t utc_now = gpst2utc(geph.tof);
    double ep[6];
    time2epoch(utc_now, ep);
    ep[3] = ep[4] = ep[5] = 0.0;
    const gtime_t utc_day_start = epoch2time(ep);
    gtime_t toe_utc = timeadd(timeadd(utc_day_start, -3.0 * 3600.0),
                              static_cast<double>(tb_min) * 60.0);
    // Adjust across midnight if toe is >12 h away from current time
    const double dt = timediff(toe_utc, utc_now);
    if      (dt >  43200.0) toe_utc = timeadd(toe_utc, -86400.0);
    else if (dt < -43200.0) toe_utc = timeadd(toe_utc,  86400.0);
    geph.toe = utc2gpst(toe_utc);

    return true;
}

// =============================================================================
// GAL Nav (ID 4002) — Galileo I/NAV decoded ephemeris
// Week is GST week; convert to GPS week by adding 1024.
// =============================================================================
namespace gal_off {
    constexpr int TOW     =   0;  // u4 current GPST TOW (ms)
    constexpr int WNC     =   4;  // u2 current GPS week
    constexpr int SVID    =   6;  // u1
    constexpr int SOURCE  =   7;  // u1 data source bitmask (bit0=INAV, bit1=FNAV)
    constexpr int WN_GAL  =   8;  // u2 Galileo/GST week
    constexpr int SISA    =  10;  // u1 SISA accuracy index
    constexpr int HEALTH  =  11;  // u1 SV health (E5b/E1 combined)
    constexpr int TOC     =  12;  // u4 clock ref time (GST seconds)
    constexpr int IOD     =  16;  // u2 IODnav (10-bit, used as iode & iodc)
    // 18-19: DVS/reserved
    constexpr int AF2     =  20;  // f4 clock drift rate (s/s²)
    constexpr int AF1     =  24;  // f4 clock drift (s/s)
    constexpr int AF0     =  28;  // f8 clock bias (s) — 31-bit precision
    constexpr int CRS     =  36;  // f4 (m)
    constexpr int DELN    =  40;  // f4 mean motion correction (rad/s)
    constexpr int M0      =  44;  // f8 (rad)
    constexpr int CUC     =  52;  // f4 (rad)
    constexpr int ECC     =  56;  // f8 eccentricity
    constexpr int CUS     =  64;  // f4 (rad)
    constexpr int SQRTA   =  68;  // f8 (m^0.5)
    constexpr int TOE     =  76;  // u4 ephemeris ref time (GST seconds)
    constexpr int CIC     =  80;  // f4 (rad)
    constexpr int OMG0    =  84;  // f8 (rad)
    constexpr int CIS     =  92;  // f4 (rad)
    constexpr int I0      =  96;  // f8 (rad)
    constexpr int CRC     = 104;  // f4 (m)
    constexpr int OMG     = 108;  // f8 (rad)
    constexpr int OMGD    = 116;  // f8 (rad/s)
    constexpr int IDOT    = 124;  // f4 (rad/s)
    constexpr int BGD_E1E5A = 128;  // f4 group delay E1-E5a (s)
    constexpr int BGD_E1E5B = 132;  // f4 group delay E1-E5b (s)
    constexpr int MIN_LEN = 136;
}

inline bool parseGALNav(const std::vector<uint8_t>& body, eph_t& eph) {
    using namespace gal_off;
    if (static_cast<int>(body.size()) < MIN_LEN) return false;
    const uint8_t* p = body.data();

    const int svid = ru1(p + SVID);
    const int sat  = svidToSat(svid);
    if (!sat) return false;
    int prn = 0;
    if (satsys(sat, &prn) != SYS_GAL) return false;

    const int    src    = static_cast<int>(ru1(p + SOURCE));
    const int    wn_gal = static_cast<int>(ru2(p + WN_GAL));
    const int    iod    = static_cast<int>(ru2(p + IOD));
    const double toc_s  = static_cast<double>(ru4(p + TOC));
    const double toe_s  = static_cast<double>(ru4(p + TOE));
    const double sqrtA  = rf8(p + SQRTA);
    const double ecc    = rf8(p + ECC);

    if (ecc < 0.0 || ecc >= 1.0 || sqrtA <= 0.0) return false;

    std::memset(&eph, 0, sizeof(eph_t));
    eph.sat   = sat;
    eph.week  = wn_gal + 1024;  // GST week → GPS week
    eph.sva   = static_cast<int>(ru1(p + SISA));
    eph.svh   = static_cast<int>(ru1(p + HEALTH));
    eph.iode  = iod;
    eph.iodc  = iod;
    eph.tgd[0] = static_cast<double>(rf4(p + BGD_E1E5A));  // BGD E1-E5a
    eph.tgd[1] = static_cast<double>(rf4(p + BGD_E1E5B));  // BGD E1-E5b
    eph.f2    = static_cast<double>(rf4(p + AF2));
    eph.f1    = static_cast<double>(rf4(p + AF1));
    eph.f0    = rf8(p + AF0);
    eph.crs   = static_cast<double>(rf4(p + CRS));
    eph.deln  = static_cast<double>(rf4(p + DELN));
    eph.M0    = rf8(p + M0);
    eph.cuc   = static_cast<double>(rf4(p + CUC));
    eph.e     = ecc;
    eph.cus   = static_cast<double>(rf4(p + CUS));
    eph.A     = sqrtA * sqrtA;
    eph.toes  = toe_s;
    eph.cic   = static_cast<double>(rf4(p + CIC));
    eph.OMG0  = rf8(p + OMG0);
    eph.cis   = static_cast<double>(rf4(p + CIS));
    eph.i0    = rf8(p + I0);
    eph.crc   = static_cast<double>(rf4(p + CRC));
    eph.omg   = rf8(p + OMG);
    eph.OMGd  = rf8(p + OMGD);
    eph.idot  = static_cast<double>(rf4(p + IDOT));

    // data source → RTKLIB code convention (RINEX 3.03)
    eph.code = (src & 0x01) ? (1 << 9) : (1 << 8);  // INAV: bit9, FNAV: bit8

    eph.toe = gst2time(wn_gal, toe_s);
    eph.toc = gst2time(wn_gal, toc_s);
    eph.ttr = gpst2time(static_cast<int>(ru2(p + WNC)), ru4(p + TOW) / 1000.0);

    return true;
}

// =============================================================================
// BDS Nav (ID 4081) — BeiDou decoded ephemeris
// Week is BDT week; convert to GPST via bdt2gpst(bdt2time(...)).
// =============================================================================
namespace bds_off {
    constexpr int TOW     =   0;  // u4 current GPST TOW (ms)
    constexpr int WNC     =   4;  // u2 current GPS week
    constexpr int SVID    =   6;  // u1
    // byte 7: reserved
    constexpr int WN_BDS  =   8;  // u2 BDT week
    constexpr int URAI    =  10;  // u1 URA index
    constexpr int HEALTH  =  11;  // u1 SV health
    constexpr int IODC    =  12;  // u2 AODC (Age of Data Clock)
    constexpr int IODE    =  14;  // u2 AODE (Age of Data Ephemeris)
    constexpr int TGD1    =  16;  // f4 TGD B1I-B3I (s)
    constexpr int TGD2    =  20;  // f4 TGD B2I-B3I (s)
    constexpr int TOC     =  24;  // u4 clock ref time in BDT (s)
    constexpr int AF2     =  28;  // f4 (s/s²)
    constexpr int AF1     =  32;  // f4 (s/s)
    constexpr int AF0     =  36;  // f4 (s)
    constexpr int CRS     =  40;  // f4 (m)
    constexpr int DELN    =  44;  // f4 (rad/s)
    constexpr int M0      =  48;  // f8 (rad)
    constexpr int CUC     =  56;  // f4 (rad)
    constexpr int ECC     =  60;  // f8 eccentricity
    constexpr int CUS     =  68;  // f4 (rad)
    constexpr int SQRTA   =  72;  // f8 (m^0.5)
    constexpr int TOE     =  80;  // u4 ephemeris ref time in BDT (s)
    constexpr int CIC     =  84;  // f4 (rad)
    constexpr int OMG0    =  88;  // f8 (rad)
    constexpr int CIS     =  96;  // f4 (rad)
    constexpr int I0      = 100;  // f8 (rad)
    constexpr int CRC     = 108;  // f4 (m)
    constexpr int OMG     = 112;  // f8 (rad)
    constexpr int OMGD    = 120;  // f8 (rad/s)
    constexpr int IDOT    = 128;  // f4 (rad/s)
    constexpr int MIN_LEN = 132;
}

inline bool parseBDSNav(const std::vector<uint8_t>& body, eph_t& eph) {
    using namespace bds_off;
    if (static_cast<int>(body.size()) < MIN_LEN) return false;
    const uint8_t* p = body.data();

    const int svid = ru1(p + SVID);
    const int sat  = svidToSat(svid);
    if (!sat) return false;
    int prn = 0;
    if (satsys(sat, &prn) != SYS_CMP) return false;

    const int    wn_bds = static_cast<int>(ru2(p + WN_BDS));
    const double toc_s  = static_cast<double>(ru4(p + TOC));
    const double toe_s  = static_cast<double>(ru4(p + TOE));
    const double sqrtA  = rf8(p + SQRTA);
    const double ecc    = rf8(p + ECC);

    if (ecc < 0.0 || ecc >= 1.0 || sqrtA <= 0.0) return false;

    std::memset(&eph, 0, sizeof(eph_t));
    eph.sat   = sat;
    eph.sva   = static_cast<int>(ru1(p + URAI));
    eph.svh   = static_cast<int>(ru1(p + HEALTH));
    eph.iodc  = static_cast<int>(ru2(p + IODC));
    eph.iode  = static_cast<int>(ru2(p + IODE));
    eph.tgd[0] = static_cast<double>(rf4(p + TGD1));
    eph.tgd[1] = static_cast<double>(rf4(p + TGD2));
    eph.f2    = static_cast<double>(rf4(p + AF2));
    eph.f1    = static_cast<double>(rf4(p + AF1));
    eph.f0    = static_cast<double>(rf4(p + AF0));
    eph.crs   = static_cast<double>(rf4(p + CRS));
    eph.deln  = static_cast<double>(rf4(p + DELN));
    eph.M0    = rf8(p + M0);
    eph.cuc   = static_cast<double>(rf4(p + CUC));
    eph.e     = ecc;
    eph.cus   = static_cast<double>(rf4(p + CUS));
    eph.A     = sqrtA * sqrtA;
    eph.toes  = toe_s;
    eph.cic   = static_cast<double>(rf4(p + CIC));
    eph.OMG0  = rf8(p + OMG0);
    eph.cis   = static_cast<double>(rf4(p + CIS));
    eph.i0    = rf8(p + I0);
    eph.crc   = static_cast<double>(rf4(p + CRC));
    eph.omg   = rf8(p + OMG);
    eph.OMGd  = rf8(p + OMGD);
    eph.idot  = static_cast<double>(rf4(p + IDOT));

    // BDT → GPST conversion
    eph.toe = bdt2gpst(bdt2time(wn_bds, toe_s));
    eph.toc = bdt2gpst(bdt2time(wn_bds, toc_s));
    eph.ttr = gpst2time(static_cast<int>(ru2(p + WNC)), ru4(p + TOW) / 1000.0);

    // Store GPS week equivalent
    int gps_wk = 0;
    time2gpst(eph.toe, &gps_wk);
    eph.week = gps_wk;

    return true;
}

}  // namespace nav
}  // namespace sbf
}  // namespace gnss_ros_standardization
