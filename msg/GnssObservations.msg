std_msgs/Header header  # stamp = receiver time (GPST); frame_id = receiver label

# GNSS time of the observation epoch
uint16 week             # GPS week number
float64 tow             # time of week [s]

GnssObservation[] observations
