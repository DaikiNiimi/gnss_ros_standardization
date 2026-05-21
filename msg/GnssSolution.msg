std_msgs/Header header  # stamp = ROS Time; frame_id = receiver label.
                        # The stamp value is set by the producer:
                        #   (a) PC time  — default for driver/decoder nodes (for sensor sync);
                        #   (b) GPST-derived ROS Time — optional in driver/decoder, and always
                        #       used by RINEX/POS converters (no PC time available).
                        # For canonical GNSS time, always use time_week / time_tow (GPST).

# GNSS Time (GPS Time System)
uint32 time_week        # GPS week number (uint32 to cover RTKLIB's extended week counter past the 1024-week rollover)
float64 time_tow        # time of week [s]

# Solution Origin (where this solution — including its time — came from)
uint8 solution_source
uint8 SOLUTION_SOURCE_UNKNOWN=0   # unset / not determined
uint8 SOLUTION_SOURCE_BINARY=1    # receiver binary PVT message (BESTPOS / NAV-PVT / PVTGeodetic ...)
uint8 SOLUTION_SOURCE_NMEA=2      # receiver NMEA sentence (GGA / RMC ...)
uint8 SOLUTION_SOURCE_COMPUTED=3  # locally computed from RAW obs + ephemerides (SPP / RTK)

# Solution Status
uint8 status
uint8 STATUS_NONE=0     # no solution
uint8 STATUS_FIX=1      # fixed ambiguity (RTK)
uint8 STATUS_FLOAT=2    # float ambiguity (RTK)
uint8 STATUS_SBAS=3     # SBAS-corrected
uint8 STATUS_DGPS=4     # differential GNSS
uint8 STATUS_SINGLE=5   # standalone GNSS
uint8 STATUS_PPP=6      # precise point positioning
uint8 STATUS_EKF=7      # GNSS/IMU EKF fused solution

uint8 num_sats          # number of satellites used
float32 ratio           # AR ratio: reliability of integer ambiguity resolution
float32 age_diff        # age of differential corrections [s]
# DOP values (dilution of precision). Population rules:
#  * Binary path (SBF DOP, NovAtel PSRDOP, UBX NAV-DOP): the most-recently-parsed
#    DOP block is cached across PVT epochs. At publish time, the cached DOP is
#    written into this message only when 0 <= (PVT_tow - DOP_tow) <= PVT_period,
#    i.e. the DOP belongs to this PVT epoch (arrived before PVT in the same frame)
#    or to the immediately-prior PVT epoch (arrived just after the previous PVT
#    flush — common SBF/Septentrio block ordering). The PVT period is auto-
#    detected from consecutive PVT timestamps. On the very first epoch the period
#    is unknown so DOP fields are NaN. The receiver-side staleness is at most one
#    PVT cycle; "future" DOP (dt < 0) and >1-cycle-stale DOP are rejected.
#  * NMEA path: hdop is taken from GGA field 8 (always finite when GGA fix has
#    quality > 0); pdop/vdop come from GSA fields 15/17 (cached across resets,
#    invalidated after 2 consecutive flushes without a fresh GSA). gdop is NaN
#    (GSA lacks tdop, so geometric DOP is not derivable).
# In both paths, check with std::isnan() before use.
float32 gdop            # geometric DOP (NaN when DOP unavailable for this epoch)
float32 pdop            # position DOP  (NaN when DOP unavailable for this epoch)
float32 hdop            # horizontal DOP (NaN when DOP unavailable for this epoch)
float32 vdop            # vertical DOP  (NaN when DOP unavailable for this epoch)

# Global Position — WGS84 ellipsoidal coordinates
float64 latitude        # geodetic latitude  [deg], positive North
float64 longitude       # geodetic longitude [deg], positive East
float64 altitude        # ellipsoidal height [m]

# Global Position — Earth-Centered Earth-Fixed (ECEF, WGS84)
geometry_msgs/Point pos_ecef    # position [m] (X, Y, Z)
float64[9] pos_cov_ecef         # 3×3 position covariance, row-major [m²]

# Global Velocity — ECEF
geometry_msgs/Vector3 vel_ecef  # velocity [m/s] (X, Y, Z)
float64[9] vel_cov_ecef         # 3×3 velocity covariance, row-major [(m/s)²]

# Local Coordinate System Origin — ECEF
geometry_msgs/Point pos_enu_org_ecef    # ENU origin [m] (X, Y, Z); set once at first fix

# Local Position — East-North-Up (ENU) relative to pos_enu_org_ecef
# Frame: axes aligned with the tangent plane at pos_enu_org_ecef (anchored at first fix).
geometry_msgs/Point pos_enu     # position [m] (East, North, Up)

# Position covariance.
# Frame: tangent plane at the CURRENT receiver position — NOT at pos_enu_org_ecef.
# This matches the receiver convention (NAV-COV NED, BESTPOS lat/lon/hgt stddev,
# PosCovGeodetic) and RTKLIB .pos sdn/sde/sdu. For sub-km baselines this is
# equivalent to pos_enu_org_ecef-anchored cov within ~baseline/R_earth (~1e-4 rad).
# For strict frame consistency or long baselines, use pos_cov_ecef.
float64[9] pos_enu_cov          # 3×3 position covariance, row-major [m²]

# Local Velocity — ENU
# Frame: tangent plane at the CURRENT receiver position.
geometry_msgs/Vector3 vel_enu   # velocity [m/s] (East, North, Up)

# Velocity covariance.
# Frame: tangent plane at the CURRENT receiver position.
float64[9] vel_enu_cov          # 3×3 velocity covariance, row-major [(m/s)²]
