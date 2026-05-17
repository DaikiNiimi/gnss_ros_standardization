# Keplerian broadcast ephemeris (GPS, Galileo, QZSS, BeiDou, NavIC, SBAS).
# Field names and definitions follow RTKLIB's eph_t structure.

string system        # constellation: "G"=GPS, "E"=Galileo, "J"=QZSS, "C"=BeiDou, "I"=NavIC, "S"=SBAS
uint16 prn           # satellite PRN number within the constellation
string satid         # combined ID string, e.g. "G01", "E05"
uint16 week          # GPS week number of toe

uint8  sva           # SV accuracy index (URA index)
uint16 code          # GPS: L2 code flag; Galileo: signal source
uint8  iode          # issue of data ephemeris
uint16 iodc          # issue of data clock
uint16 svh           # SV health flag (0 = healthy; Galileo packs up to 9 bits)

# Reference times (seconds since GPS epoch, i.e. GPST)
float64 toc          # clock reference time [s]
float64 ttr          # transmission time [s]
float64 toe          # ephemeris reference time [s]
float64 toes         # toe within current week [s]

# Keplerian orbital elements
float64 a            # semi-major axis [m]
float64 e            # eccentricity [-]
float64 i0           # inclination angle at reference time [rad]
float64 omg0         # longitude of ascending node at weekly epoch [rad]
float64 omg          # argument of perigee [rad]
float64 m0           # mean anomaly at reference time [rad]
float64 deln         # mean motion difference from computed value [rad/s]
float64 omgd         # rate of right ascension [rad/s]
float64 idot         # rate of inclination angle [rad/s]

# Harmonic correction terms
float64 crc          # amplitude of cosine harmonic correction to orbit radius [m]
float64 crs          # amplitude of sine harmonic correction to orbit radius [m]
float64 cuc          # amplitude of cosine harmonic correction to argument of latitude [rad]
float64 cus          # amplitude of sine harmonic correction to argument of latitude [rad]
float64 cic          # amplitude of cosine harmonic correction to inclination [rad]
float64 cis          # amplitude of sine harmonic correction to inclination [rad]

# Clock correction coefficients
float64 f0           # SV clock bias [s]
float64 f1           # SV clock drift [s/s]
float64 f2           # SV clock drift rate [s/s²]

# Group delay
float64[] tgd        # group delay differential [s] (array length depends on constellation)

# Receiver/format-specific flags (round-trip with RTKLIB eph_t)
uint8 flag           # GPS/QZS: L2 P-data flag; other systems: unused
uint8 fit            # GPS/QZS: fit interval [hours] (typically 4 or 6); 0 elsewhere
