std_msgs/Header header  # stamp = solution time (GPST); frame_id = receiver label

# GNSS Time (GPS Time System)
uint16 time_week        # GPS week number
float64 time_tow        # time of week [s]

# Solution Status
uint8 status
uint8 STATUS_NONE=0     # no solution
uint8 STATUS_FIX=1      # fixed ambiguity (RTK)
uint8 STATUS_FLOAT=2    # float ambiguity (RTK)
uint8 STATUS_SBAS=3     # SBAS-corrected
uint8 STATUS_DGPS=4     # differential GNSS
uint8 STATUS_SINGLE=5   # standalone GNSS
uint8 STATUS_PPP=6      # precise point positioning

uint8 num_sats          # number of satellites used
float32 ratio           # AR ratio: reliability of integer ambiguity resolution
float32 age_diff        # age of differential corrections [s]
float32 gdop            # geometric DOP
float32 pdop            # position DOP
float32 hdop            # horizontal DOP
float32 vdop            # vertical DOP

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
geometry_msgs/Point org_ecef    # ENU origin [m] (X, Y, Z); set once at first fix

# Local Position — East-North-Up (ENU) relative to org_ecef
geometry_msgs/Point pos_enu     # position [m] (East, North, Up)
float64[9] pos_enu_cov          # 3×3 position covariance, row-major [m²]

# Local Velocity — ENU
geometry_msgs/Vector3 vel_enu   # velocity [m/s] (East, North, Up)
float64[9] vel_enu_cov          # 3×3 velocity covariance, row-major [(m/s)²]
