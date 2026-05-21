std_msgs/Header header              # stamp = ROS Time. From driver/decoder: time of reception
                                    # (PC time or GPST-derived). From rinex_to_rosbag: the
                                    # corresponding obs epoch stamp (GPST-derived). RINEX nav
                                    # does not preserve broadcast arrival time; --eph-mode
                                    # selects per-epoch republish vs on-change publishing.
GnssEphemeris[] gnss_ephemeris      # GPS / Galileo / QZSS / BeiDou / NavIC / SBAS
GlonassEphemeris[] glonass_ephemeris  # GLONASS
