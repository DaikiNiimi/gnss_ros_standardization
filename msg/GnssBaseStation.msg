std_msgs/Header header  # stamp = ROS Time of publication; frame_id = receiver label.

# One configuration of an RTCM reference station: what RTCM 1005/1006, 1007/1008,
# 1033 and 1230 announce for it. A stream that changes its base station (VRS,
# services that pick the base from the rover position) publishes a new record,
# and the record is published before the first observation epoch it applies to.
#
# An observation epoch at GPST t on the same stream belongs to the record with the
# latest valid_from <= t. A record for a configuration whose position has not yet
# arrived is published with position_valid=false and republished, with the same
# valid_from, once it arrives.

uint16 station_id           # reference station ID (DF003)

# GPST of the first observation epoch of this configuration.
uint32 valid_from_week      # GPS week number
float64 valid_from_tow      # time of week [s]

# Antenna reference point (1005/1006, DF025-DF027), ECEF (WGS84) [m]. This is the
# point the base observations refer to; it is not lowered to the marker.
bool position_valid         # false until 1005/1006 for this configuration has arrived
geometry_msgs/Point arp_ecef
float64 antenna_height      # ARP height above the marker (1006 DF028) [m]; NaN without 1006
uint8 itrf_realization_year # DF021 (0 = not specified)

# Antenna (1007/1008/1033) and receiver (1033) descriptors; empty = not received.
string antenna_descriptor   # IGS antenna name, e.g. "TRM59800.00     NONE"
uint8 antenna_setup_id      # DF031
string antenna_serial
string receiver_type
string receiver_firmware
string receiver_serial

# GLONASS code-phase biases (1230).
bool glonass_code_phase_aligned
float64[4] glonass_code_phase_bias  # {L1 C/A, L1 P, L2 C/A, L2 P} [m]; NaN = not transmitted
