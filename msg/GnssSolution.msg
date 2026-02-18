std_msgs/Header header

# GNSS Time
uint16 time_week
float64 time_tow

# Solution Status
uint8 status
uint8 STATUS_NONE=0
uint8 STATUS_FIX=1
uint8 STATUS_FLOAT=2
uint8 STATUS_SBAS=3
uint8 STATUS_DGPS=4
uint8 STATUS_SINGLE=5
uint8 STATUS_PPP=6

uint8 num_sats
float32 ratio       # AR Ratio: Reliability of the ambiguity resolution
float32 age_diff    # Age of differential corrections
float32 gdop
float32 pdop
float32 hdop
float32 vdop

# Global Position (WGS84 Ellipsoidal)
float64 latitude    # degrees
float64 longitude   # degrees
float64 altitude    # meters


# Global Position (ECEF)
geometry_msgs/Point pos_ecef # m (X, Y, Z)
float64[9] pos_cov_ecef

# Global Velocity (ECEF)
geometry_msgs/Vector3 vel_ecef # m/s (X, Y, Z)
float64[9] vel_cov_ecef

# Origin of the Local Coordinate System (ECEF)
geometry_msgs/Point org_ecef # m (X, Y, Z)

# Local Position (ENU)
geometry_msgs/Point pos_enu   # m (East, North, Up)
float64[9] pos_enu_cov

# Local Velocity (ENU)
geometry_msgs/Vector3 vel_enu # m/s (East, North, Up)
float64[9] vel_enu_cov
