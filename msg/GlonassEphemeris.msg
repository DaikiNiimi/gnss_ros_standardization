# GLONASS broadcast ephemeris.
# Uses the PZ-90.11 state vector format (position, velocity, acceleration in ECEF).
# Field names follow RTKLIB's geph_t structure.

string system     # always "R" (GLONASS)
uint16 prn        # slot number (1–27)
uint32 week       # GPS week number of transmission (uint32 to cover RTKLIB's extended week counter past the 1024-week rollover)
string satid      # combined ID string, e.g. "R05"
int8 frq          # frequency channel number (-7 to +13)
uint8 iode        # issue of data (frame number 1–5)
uint8 svh         # SV health flag (0 = healthy)
uint8 age         # age of operation data [days]

float64 tof       # time of frame (GPST) [s]
float64 toe       # ephemeris reference time (GPST) [s]

# State vectors at toe in PZ-90.11 ECEF frame
float64[] pos     # position [m]  (X, Y, Z), length 3
float64[] vel     # velocity [m/s] (Ẋ, Ẏ, Ż), length 3
float64[] acc     # lunar/solar acceleration [m/s²] (Ẍ, Ÿ, Z̈), length 3

float64 gamn      # relative frequency deviation (SV clock frequency bias) [-]
float64 taun      # SV clock bias [s]
float64 dtaun     # inter-frequency bias (G1–G2) [s]
