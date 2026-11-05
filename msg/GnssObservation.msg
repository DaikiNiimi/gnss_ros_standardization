# Missing or invalid P/L/D/SNR are NaN where source validity is known.
# Valid numeric zero phase/Doppler/CN0 remains zero; zero pseudorange is not a
# usable range. Some backend paths retain ambiguous zero missing sentinels;
# the ROS converter cannot reconstruct information already lost upstream.
# snr is dB-Hz; lli preserves the RTKLIB bit mask. See OBSERVATIONS.md.
# These fields follow RTKLIB's observation data structure directly.
uint8 sat           # satellite number (1-MAXSAT)
uint8 freq 255      # GLONASS frequency channel + 7 (0-13), as in RTKLIB obsd_t.freq; 255 = unknown/not GLONASS
uint8 code          # code indicator (CODE_? ? ?)
float64 p           # observation data pseudorange (m) [P]
float64 l           # observation data carrier-phase (cycle) [L]
float64 d           # observation data doppler frequency (Hz) [D]
float32 snr         # signal strength [SNR]
uint8 lli           # loss of lock indicator [LLI]

# These fields are derived from the core RTKLIB fields for easier usage in ROS.
string system       # system code (e.g. "G", "R", "E", "J", "C", "I", "S")
uint16 prn          # satellite prn number
string satid        # RTKLIB ID (e.g. "G01", "R01", SBAS "129"); identify by system + prn
string code_str     # code indicator string (e.g. "1C", "2W")
