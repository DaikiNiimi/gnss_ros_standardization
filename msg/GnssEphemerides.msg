std_msgs/Header header              # stamp = ROS Time: publication time of this snapshot
GnssEphemeris[] gnss_ephemeris      # GPS / Galileo / QZSS / BeiDou / NavIC
GlonassEphemeris[] glonass_ephemeris  # GLONASS

# Broadcast ionosphere model parameters (RTKLIB nav_t order). All zero = not received.
float64[8] ion_gps                  # GPS Klobuchar {a0,a1,a2,a3,b0,b1,b2,b3}
float64[4] ion_gal                  # Galileo NeQuick {ai0,ai1,ai2,0}
float64[8] ion_qzs                  # QZSS Klobuchar {a0,a1,a2,a3,b0,b1,b2,b3}
float64[8] ion_bds                  # BeiDou Klobuchar {a0,a1,a2,a3,b0,b1,b2,b3}
float64[8] ion_irn                  # NavIC Klobuchar {a0,a1,a2,a3,b0,b1,b2,b3}
