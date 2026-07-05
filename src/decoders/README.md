# Decoders

Decoder nodes receive a **binary GNSS byte stream** (via NTRIP, TCP, or Serial), parse the protocol using the embedded **RTKLIB (rtklibexplorer fork)** library, and publish standardized ROS 2 messages.

Unlike the drivers in [`../drivers/`](../drivers/), decoders are **passive and do not configure the receiver** — this also holds when the RTCM relay below is enabled: correction bytes are forwarded verbatim. The single exception is the NovAtel decoder's dedicated correction port, which receives one `INTERFACEMODE` command over that port itself (never over the decoded-log port); see the relay section.

## Supported Decoders

| Node | Executable | Input Protocol / Format | Published data |
|---|---|---|---|
| **RTCM3 Decoder** | `rtcm_decoder_node` | RTCM 3.x (MSM4/MSM7 + Ephemeris) | Observations, Ephemerides |
| **u-blox Decoder** | `ubx_decoder_node` | UBX (u-blox proprietary binary) | Observations, Ephemerides, PVT, IMU |
| **Septentrio Decoder** | `sbf_decoder_node` | SBF (Septentrio Binary Format) | Observations, Ephemerides, PVT, IMU |
| **NovAtel Decoder** | `novatel_decoder_node` | OEM4 / OEM6 / OEM7 binary | Observations, Ephemerides, PVT, IMU |

---

## Stream Connection & URI

All decoders accept a `stream_path` parameter specifying the source stream. The connection is managed and opened via RTKLIB's stream interface (`stropen`).

### Supported URI Schemes
| Scheme | Format / Example | Description |
|---|---|---|
| **NTRIP** | `ntrip://user:pass@caster.example.com:2101/MOUNT` | Connects to an NTRIP caster mountpoint. |
| **TCP Client** | `tcpcli://192.168.1.100:9000` | Connects to a TCP server broadcasting GNSS data. |
| **TCP Server** | `tcpsvr://:5556` | Hosts a TCP server and accepts pushed data. |
| **Serial** | `serial:///dev/ttyUSB0:115200` | Connects to a local serial port. |
| **File** | `file:///path/to/log.bin` | Replays a recorded raw log. |

---

### Common Parameters
| Parameter | Type | Default | Description |
|---|---|---|---|
| `stream_path` | `string` | (Varies) | Connection URI for the input GNSS stream. |
| `frame_id` | `string` | `"gnss_link"` | ROS TF frame ID attached to the published message headers. |
| `observation_topic` | `string` | `/gnss/observation` | Topic for raw GNSS observations (`GnssObservations`). |
| `ephemeris_topic` | `string` | `/gnss/ephemeris` | Topic for GNSS ephemerides (`GnssEphemerides`). Uses `transient_local` durability QoS. |
| `solution_topic` | `string` | `/gnss/nmea_solution` | Topic for receiver position solutions from binary PVT or NMEA (`GnssSolution`). *Not used by RTCM3 decoder.* |
| `imu_raw_topic` | `string` | `/gnss/imu/data_raw` | Topic for uncalibrated raw IMU measurements (`sensor_msgs/Imu`). *Not used by RTCM3 decoder.* |
| `imu_topic` | `string` | `/gnss/imu/data` | Topic for calibrated IMU measurements (`sensor_msgs/Imu`). *Used only by UBX/NovAtel decoders.* |
| `ephemeris.snapshot_period_s` | `double` | `30.0` | Period (in seconds) to republish the current ephemeris cache as a heartbeat. |
| `ephemeris.max_age_s` | `double` | `0.0` | Maximum age of ephemeris to keep in cache (seconds). `0.0` keeps all received ephemerides. |
| `use_gps_timestamp` | `bool` | `false` | If `true`, the ROS message headers will use GPS/GPST time instead of the ROS system clock. |
| `origin` | `double[]` | `[0.0, 0.0, 0.0]` | Fixed ECEF/ENU local origin `[Latitude (deg), Longitude (deg), Altitude (m)]` for ENU local coordinates. |
| `rtcm_relay` | `string` | `""` (disabled) | Listen URI for incoming RTCM corrections, written back down the receiver stream for on-chip RTK (see below). On `rtcm_decoder_node` the direction is reversed: it is the *output* URI the decoded source bytes are forwarded to. |
| `rtcm_relay_output` | `string` | `""` | *NovAtel decoder only.* OS device of the dedicated RTCMV3 correction port (e.g. `serial:///dev/ttyUSB2:230400`). Empty writes corrections to the main stream (not supported on NovAtel hardware). |
| `rtcm_relay_correction_port` | `string` | `"THISPORT"` | *NovAtel decoder only.* `INTERFACEMODE` target, sent over `rtcm_relay_output` itself — leave `THISPORT`. |

---

## RTCM correction relay (receiver on-chip RTK)

A decoder holds the receiver port exclusively, so nothing else can feed
RTK corrections to the receiver — on single-port connections (a u-blox USB
cable, a lone UART to an OEM board) there would be no correction path at all.
The `rtcm_relay` parameter solves this RTKLIB-STRSVR-style: the decoder hosts a
TCP server, and raw RTCM bytes pushed there are written back down the stream it
already owns. The receiver then computes RTK on-chip and its Fix appears in the
decoded PVT output as usual.

```
NTRIP caster / base ──> rtcm_decoder_node ──> tcpcli ──> tcpsvr ──> *_decoder ──> receiver port ──> on-chip RTK
                        -p rtcm_relay:="tcpcli://127.0.0.1:5556"    -p rtcm_relay:="tcpsvr://:5556"
```

```bash
# Rover decoder (u-blox example) — hosts the correction server
ros2 run gnss_ros_standardization ubx_decoder_node --ros-args \
  -p stream_path:="serial:///dev/ttyACM0:115200" \
  -p rtcm_relay:="tcpsvr://:5556"

# Correction source — decodes the base stream to topics AND forwards it
ros2 run gnss_ros_standardization rtcm_decoder_node --ros-args \
  -p stream_path:="ntrip://user:pass@caster:2101/MOUNT" \
  -p rtcm_relay:="tcpcli://127.0.0.1:5556"
```

Recommended ports match the drivers so the `rtcm_decoder_node` command line is
identical in driver and decoder operation: **ubx = 5556, sbf = 5557,
novatel = 5558**.

**NovAtel needs a dedicated correction port**, same as the driver:
`INTERFACEMODE` is per-direction and a port receiving RTCMV3 stops interpreting
NOVATEL input, so corrections cannot share the decoded-log port. A single USB
cable exposes three `/dev` devices (receiver USB1/2/3) — point
`rtcm_relay_output` at a spare one and the decoder configures it by sending
`INTERFACEMODE THISPORT RTCMV3 NONE OFF` over that port itself (the main log
port is never touched):

```bash
ros2 run gnss_ros_standardization novatel_decoder_node --ros-args \
  -p stream_path:="serial:///dev/ttyUSB1:230400" \
  -p rtcm_relay:="tcpsvr://:5558" \
  -p rtcm_relay_output:="serial:///dev/ttyUSB2:230400"
```

For u-blox and Septentrio the decoder never sends receiver commands, so the
receiver port must be set up **once beforehand** to accept RTCMv3 input
alongside its binary output:

| Receiver | One-time input setup | Note |
|---|---|---|
| u-blox | u-center: include RTCM3 in the port's `inProtoMask` | F9P default already includes RTCM3 |
| Septentrio | `setDataInOut, COM1, RTCMv3, SBF` (WebUI or RxTools/terminal) | Input and output types are independent per port |
| NovAtel | none (decoder sends `INTERFACEMODE` over `rtcm_relay_output`) | Requires the dedicated port above; single-UART-only wiring cannot receive corrections (receiver limitation) |

Alternatives when a spare receiver port is available (multi-port USB
connections): point `rtcm_decoder_node`'s `rtcm_relay` directly at it, e.g.
`-p rtcm_relay:="serial:///dev/ttyUSB2:230400"` — no decoder-side relay needed.
The drivers offer the same relay for configured operation
(`rtcm_relay.enabled` + `rtcm_relay.listen`, with `configure_on_startup:=false`
for pre-configured receivers); see [`../drivers/`](../drivers/) and
`config/*_driver.yaml`.

---

## Detailed Node Reference

### `rtcm_decoder_node`

```bash
ros2 run gnss_ros_standardization rtcm_decoder_node --ros-args \
  -p stream_path:="ntrip://user:password@host:port/mountpoint"
```

### Decoded RTCM messages

| Group | RTCM message types |
|---|---|
| MSM4 observations | 1074 (GPS), 1084 (GLO), 1094 (GAL), 1104 (SBAS), 1114 (QZS), 1124 (BDS), 1134 (NavIC) |
| MSM7 observations | 1077 (GPS), 1087 (GLO), 1097 (GAL), 1107 (SBAS), 1117 (QZS), 1127 (BDS), 1137 (NavIC) |
| GPS ephemeris | 1019 |
| GLONASS ephemeris | 1020 |
| Galileo ephemeris | 1045 (F/NAV), 1046 (I/NAV) |
| BeiDou ephemeris | 1042 |
| QZSS ephemeris | 1044 |
| NavIC ephemeris | 1041 |
| Station info | 1005, 1006, 1007, 1008, 1033 (parsed but not republished) |

---

### `ubx_decoder_node`

```bash
ros2 run gnss_ros_standardization ubx_decoder_node --ros-args \
  -p stream_path:="serial:///dev/ttyACM0:115200"
```

### Decoded UBX messages

`/gnss/observation`:

| Message | ID | Contents | Notes |
|---|---|---|---|
| `UBX-RXM-RAWX` | `0x02 / 0x15` | Raw pseudorange, carrier phase, Doppler, and SNR for all tracked signals |  |

`/gnss/ephemeris`:

| Message | ID | Contents | Notes |
|---|---|---|---|
| `UBX-RXM-SFRBX` | `0x02 / 0x13` | Raw navigation subframes for GPS/GLO/GAL/BDS/QZS/NavIC/SBAS | Assembled into ephemerides by RTKLIB |

`/gnss/nmea_solution`:

| Message | ID | Contents | Notes |
|---|---|---|---|
| `UBX-NAV-PVT` | `0x01 / 0x07` | Receiver PVT solution | Binary solution source when present during startup grace |
| `UBX-NAV-DOP` | `0x01 / 0x04` | GDOP, PDOP, TDOP, VDOP, HDOP, NDOP, and EDOP | Applied when fresh for the PVT epoch |
| `UBX-NAV-COV` | `0x01 / 0x36` | Position and velocity covariance in the receiver's navigation frame | Refines covariance when emitted by the receiver |
| `UBX-NAV-POSECEF` | `0x01 / 0x01` | Receiver ECEF position | Optional native ECEF position source |
| `UBX-NAV-VELECEF` | `0x01 / 0x11` | Receiver ECEF velocity | Optional native ECEF velocity source |
| `UBX-NAV-HPPOSLLH` | `0x01 / 0x14` | High-precision geodetic position (1e-9 deg / 0.1 mm) | Highest-priority LLH source; overrides NAV-PVT position |
| `UBX-NAV-HPPOSECEF` | `0x01 / 0x13` | High-precision ECEF position (0.1 mm) | Highest-priority ECEF source; overrides NAV-POSECEF |
| NMEA `GGA/RMC/GSA/GST` | — | NMEA position, velocity, DOP, and covariance fallback | Used only when no binary PVT is selected |

`/gnss/imu/data_raw`:

| Message | ID | Contents | Requirement |
|---|---|---|---|
| `UBX-ESF-RAW` | `0x10 / 0x03` | Raw IMU sensor samples (accel + gyro, internal scale) | u-blox receiver with ESF raw output |

`/gnss/imu/data`:

| Message | ID | Contents | Requirement |
|---|---|---|---|
| `UBX-ESF-INS` | `0x10 / 0x15` | Receiver-calibrated angular rate [rad/s] and linear acceleration [m/s²] | u-blox receiver with ESF INS output |

---

### `sbf_decoder_node`

```bash
ros2 run gnss_ros_standardization sbf_decoder_node --ros-args \
  -p stream_path:="serial:///dev/ttyUSB0:115200"
```

### Decoded SBF blocks

`/gnss/observation`:

| Block | ID | Contents | Notes |
|---|---|---|---|
| `MeasEpoch` | 4027 | Raw carrier phase, code range, Doppler, and SNR for all signals |  |

`/gnss/ephemeris` (receiver-assembled — single block = complete ephemeris):

| Block | ID | Contents | Notes |
|---|---|---|---|
| `GPSNav` | 5891 | GPS Keplerian ephemeris |  |
| `GLONav` | 4004 | GLONASS state-vector ephemeris |  |
| `GALNav` | 4002 | Galileo Keplerian ephemeris |  |
| `BDSNav` | 4081 | BeiDou Keplerian ephemeris |  |
| `QZSNav` | 4095 | QZSS Keplerian ephemeris |  |
| `NavICNav` | 4099 | NavIC Keplerian ephemeris |  |

`/gnss/ephemeris` (raw subframe blocks — decoded by RTKLIB, duplicates filtered by IODE/IODC):

| Block | ID | Contents | Notes |
|---|---|---|---|
| `GPSRawCA` | 4017 | GPS L1 C/A navigation subframe |  |
| `GLORawCA` | 4026 | GLONASS L1 C/A navigation string |  |
| `GALRawINAV` | 4023 | Galileo I/NAV page |  |
| `GALRawFNAV` | 4022 | Galileo F/NAV page |  |
| `BDSRaw` | 4047 | BeiDou navigation message |  |
| `QZSRawL1CA` | 4066 | QZSS L1 C/A subframe |  |
| `NAVICRaw` | 4093 | NavIC navigation message |  |

`/gnss/nmea_solution`:

| Block | ID | Contents | Notes |
|---|---|---|---|
| `PVTGeodetic` | 4007 | Receiver LLH position and ENU velocity | Binary solution source when present during startup grace |
| `PVTCartesian` | 4006 | Receiver ECEF position and velocity | Binary solution source when present during startup grace |
| `PosCovGeodetic` | 5906 | Position covariance in geodetic/local navigation frame | Used with `PVTGeodetic` |
| `VelCovGeodetic` | 5908 | Velocity covariance in geodetic/local navigation frame | Used with `PVTGeodetic` |
| `PosCovCartesian` | 5905 | Position covariance in ECEF | Used with `PVTCartesian` |
| `VelCovCartesian` | 5907 | Velocity covariance in ECEF | Used with `PVTCartesian` |
| `DOP` | 4001 | GDOP, PDOP, TDOP, HDOP, VDOP, and related DOP values | Applied when fresh for the PVT epoch |
| NMEA `GGA/RMC/GSA/GST` | — | NMEA position, velocity, DOP, and covariance fallback | Used only when no binary PVT is selected |

`/gnss/imu/data_raw`:

| Block | ID | Contents | Requirement |
|---|---|---|---|
| `ExtSensorMeas` | 4050 | Linear acceleration [m/s²] (type 0) and angular velocity [rad/s] (type 1) | AsteRx-i / mosaic-X5 with IMU |

---

### `novatel_decoder_node`

```bash
ros2 run gnss_ros_standardization novatel_decoder_node --ros-args \
  -p stream_path:="serial:///dev/ttyUSB0:115200"
```

### Decoded NovAtel logs

`/gnss/observation`:

| Log | ID | Contents | Notes |
|---|---|---|---|
| `RANGECMPB` | 140 | Compressed pseudorange, carrier phase, Doppler, and SNR | Recommended |
| `RANGEB` | 43 | Uncompressed pseudorange, carrier phase, Doppler, and SNR |  |

`/gnss/ephemeris`:

| Log | ID | Contents | Notes |
|---|---|---|---|
| `RAWEPHEMB` | 41 | GPS raw ephemeris | RTKLIB does not decode `GPSEPHEMB` ID 7 |
| `GLOEPHEMERISB` | 723 | GLONASS ephemeris |  |
| `GALEPHEMERISB` | 1122 | Galileo ephemeris | Contains both I/NAV and F/NAV |
| `BDSEPHEMERISB` | 1696 | BeiDou ephemeris |  |
| `QZSSRAWEPHEMB` | 1331 | QZSS raw ephemeris | RTKLIB does not decode `QZSSEPHEMERISB` ID 1336 |
| `NAVICEPHEMERISB` | 2123 | NavIC ephemeris |  |
| `IONUTCB` | 8 | GPS ionosphere and UTC parameters |  |
| `GALIONOB` | 1127 | Galileo ionosphere parameters |  |
| `QZSSIONUTCB` | 1347 | QZSS ionosphere and UTC parameters |  |

`/gnss/nmea_solution`:

| Log | ID | Contents | Notes |
|---|---|---|---|
| `BESTPOSB` | 42 | Receiver LLH position, solution status, satellite count, differential age, and position sigma | Binary solution source when present during startup grace |
| `BESTVELB` | 99 | Receiver horizontal speed, track over ground, and vertical speed | Complements `BESTPOSB` velocity fields |
| `BESTXYZB` | 241 | Receiver ECEF position/velocity and covariance diagonals | Optional native ECEF source for the same solution |
| `PSRDOPB` | 174 | GDOP, PDOP, HDOP, HTDOP, TDOP, and cutoff angle | Applied when fresh for the PVT epoch |
| NMEA `GGA/RMC/GSA/GST` | — | NMEA position, velocity, DOP, and covariance fallback | Used only when no binary PVT is selected |

`/gnss/imu/data_raw`:

| Log | ID | Contents | Requirement |
|---|---|---|---|
| `RAWIMUSXB` | 1462 | Raw IMU delta-velocity and delta-angle counts with IMU-type-specific scaling | NovAtel receiver + IMU |

`/gnss/imu/data`:

| Log | ID | Contents | Requirement |
|---|---|---|---|
| `CORRIMUDATAB` | 812 | Bias-, gravity-, and earth-rate-corrected linear acceleration [m/s²] and angular velocity [rad/s] | SPAN-capable receiver + IMU |
