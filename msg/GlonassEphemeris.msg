# GLONASS broadcast ephemeris.
# Uses the PZ-90.11 state vector format (position, velocity, acceleration in ECEF).
# Field names follow RTKLIB's geph_t structure.

string system     # always "R" (GLONASS)
uint16 prn        # slot number (1–27)
uint32 week       # GPS week number of toe (uint32 to cover RTKLIB's extended week counter past the 1024-week rollover)
string satid      # combined ID string, e.g. "R05"
int8 frq          # frequency channel number (-7 to +13)
uint8 iode        # backend issue of data (tb-derived; not a frame number)
uint8 svh         # SV health flag (0 = healthy)
uint8 age         # age of operation data [days]

float64 tof       # time of frame, GPST seconds of week; nearest week relative to toe
float64 toe       # ephemeris reference time, GPST seconds of week

# State vectors at toe in PZ-90.11 ECEF frame
float64[] pos     # position [m]  (X, Y, Z), length 3
float64[] vel     # velocity [m/s] (Ẋ, Ẏ, Ż), length 3
float64[] acc     # lunar/solar acceleration [m/s²] (Ẍ, Ÿ, Z̈), length 3

float64 gamn      # relative frequency deviation (SV clock frequency bias) [-]
float64 taun      # SV clock bias [s]
float64 dtaun     # inter-frequency bias (G1–G2) [s]

# Preserve backend metadata without unsigned narrowing. Availability masks are not represented.
int32 flags           # backend status bits: M[8:7], P4[6], P3[5], P2[4], P1[3:2], P[1:0]
int32 sva             # GLONASS accuracy index; -1 if not representable
