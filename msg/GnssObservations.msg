std_msgs/Header header  # stamp = ROS Time; frame_id = receiver label.
                        # Set by producer: PC time (driver/decoder default) or
                        # GPST-derived ROS Time (driver/decoder option, and always
                        # used by rinex_to_rosbag — no PC time available).
                        # For canonical GNSS time, use week / tow (GPST).

# GNSS time of the observation epoch
uint16 week             # GPS week number
float64 tow             # time of week [s]

GnssObservation[] observations
