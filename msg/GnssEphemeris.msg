string system        # "G"=GPS, "E"=Galileo, "J"=QZSS, "C"=BeiDou, "I"=NavIC, "S"=SBAS
uint16 prn
string satid
uint16 week

uint8  sva           # eph.sva
uint16 code          # eph.code
uint8  iode          # eph.iode
uint16 iodc          # eph.iodc
uint8  svh           # eph.svh

float64 toc          # eph.toc
float64 ttr          # eph.ttr
float64 toe          # eph.toe
float64 toes         # eph.toes

float64 a            # eph.A
float64 e            # eph.e
float64 i0           # eph.i0         [rad]
float64 omg0         # eph.OMG0       [rad]
float64 omg          # eph.omg        [rad]
float64 m0           # eph.M0         [rad]
float64 deln         # eph.deln       [rad/s]
float64 omgd         # eph.OMGdot     [rad/s]
float64 idot         # eph.idot       [rad/s]
float64 crc          # eph.crc        [m]
float64 crs          # eph.crs        [m]
float64 cuc          # eph.cuc        [rad]
float64 cus          # eph.cus        [rad]
float64 cic          # eph.cic        [rad]
float64 cis          # eph.cis        [rad]


float64 f0           # [s]
float64 f1           # [s/s]
float64 f2           # [s/s^2]
float64[] tgd        # eph.tgd[]

