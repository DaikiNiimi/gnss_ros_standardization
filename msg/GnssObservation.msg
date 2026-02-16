# These fields follow RTKLIB's observation data structure directly.
uint8 sat           # satellite number (1-MAXSAT)
uint8 code          # code indicator (CODE_? ? ?)
float64 p           # observation data pseudorange (m) [P]
float64 l           # observation data carrier-phase (cycle) [L]
float64 d           # observation data doppler frequency (Hz) [D]
float32 snr         # signal strength (0.001 dBHz) [SNR]
uint8 lli           # loss of lock indicator [LLI]

# These fields are derived from the core RTKLIB fields for easier usage in ROS.
string system       # system code (e.g. "G", "R", "E", "J", "C", "I", "S")
uint16 prn          # satellite prn number
string satid        # satellite ID (e.g. "G01", "R01")
string code_str     # code indicator string (e.g. "1C", "2W")
