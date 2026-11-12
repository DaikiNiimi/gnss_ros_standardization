# Decoders

Decoders read a receiver byte stream that is already configured and publish the
common GNSS messages. They never send commands to the receiver; the only bytes a decoder
writes are RTCM corrections when `rtcm_input` is enabled (see [drivers](../drivers/README.md)).

| Executable | Input |
|---|---|
| `ubx_decoder_node` | u-blox UBX (+ NMEA) |
| `sbf_decoder_node` | Septentrio SBF (+ NMEA) |
| `novatel_decoder_node` | NovAtel OEM4/6/7 binary (+ NMEA) |
| `rtcm_decoder_node` | RTCM 3 |

```bash
ros2 run gnss_ros_standardization ubx_decoder_node --ros-args \
  -p stream_path:=serial:///dev/ttyACM0:115200
```

## Stream URI (`stream_path`)

| Source | Example |
|---|---|
| Serial | `serial:///dev/ttyUSB0:115200` |
| TCP client | `tcpcli://192.168.1.100:9000` |
| TCP server | `tcpsvr://:5556` |
| NTRIP | `ntrip://user:pass@caster.example.com:2101/MOUNT` |
| File | `file:///path/to/log.ubx` |

`${VAR}` and `$VAR` in a URI are resolved from the node's environment when the stream
is opened, so credentials do not appear in ROS parameters or on the command line
(quote the value so the shell does not expand it):

```bash
export NTRIP_USER=... NTRIP_PASSWORD=...
ros2 run gnss_ros_standardization rtcm_decoder_node --ros-args \
  -p stream_path:='ntrip://${NTRIP_USER}:${NTRIP_PASSWORD}@caster:2101/MOUNT'
```

## Topics

| Topic | Type | Content |
|---|---|---|
| `/gnss/observation` | `GnssObservations` | raw observations, one message per epoch |
| `/gnss/ephemeris` | `GnssEphemerides` | broadcast ephemerides (`transient_local`) |
| `/gnss/nmea_solution` | `GnssSolution` | receiver solution (binary PVT, otherwise NMEA) |
| `/gnss/imu/data_raw` | `sensor_msgs/Imu` | raw IMU (UBX ESF-RAW, SBF ExtSensorMeas, NovAtel RAWIMUSX) |
| `/gnss/imu/data` | `sensor_msgs/Imu` | receiver-corrected IMU (UBX ESF-INS, NovAtel CORRIMUDATA) |

The RTCM decoder publishes observations, ephemerides and `/gnss/station`
(`GnssBaseStation`: base position from 1005/1006, antenna and receiver from
1007/1008/1033, GLONASS biases from 1230; one record per station, see the
[topic contract](../../docs/TOPIC_DATA_CONTRACT.md#base-station-gnssbasestation)).

## Parameters

| Parameter | Default | Nodes | Meaning |
|---|---|---|---|
| `stream_path` | node specific | all | input stream URI |
| `frame_id` | `gnss_link` | UBX, SBF, NovAtel | header frame |
| `topics.observation`, `topics.ephemeris` | `/gnss/observation`, `/gnss/ephemeris` | all | output topics |
| `topics.station` | `/gnss/station` | RTCM | output topic |
| `topics.solution`, `topics.imu_raw` | `/gnss/nmea_solution`, `/gnss/imu/data_raw` | UBX, SBF, NovAtel | output topics |
| `topics.imu` | `/gnss/imu/data` | UBX, NovAtel | output topic |
| `use_gps_timestamp` | `false` | all | header stamp: `false` = host receive time, `true` = GNSS time converted to UTC |
| `ephemeris.snapshot_period_s` | `30.0` | all | republish the full ephemeris set every N s |
| `ephemeris.max_age_s` | `0.0` | all | drop ephemerides older than N s (`0` keeps all) |
| `local_origin.mode`, `.pos`, `.postype` | `auto`, `[0,0,0]`, `llh` | UBX, SBF, NovAtel | ENU origin of the solution: `auto` = first valid solution, `fixed` = `pos` ([details](../../docs/PARAMETER_NAMES.md#enu-origin)) |
| `nmea.reference_date` | `[]` | UBX, SBF, NovAtel | `[year, month, day]` for a GGA-only NMEA stream |
| `rtcm_input.listen` | `""` | UBX, SBF, NovAtel | listen URI for RTCM corrections written to the receiver (see below) |
| `rtcm_input.receiver_stream`, `rtcm_input.receiver_port` | `""`, `THISPORT` | NovAtel | dedicated correction port (see below) |
| `rawx_layout` | `auto` | UBX | RXM-RAWX layout: `auto`, `data0`, `data1_pre27` |
| `imu.sttag_tick_s` | `39.0625e-6` | UBX | nominal ESF-RAW time-tag tick; the actual tick is measured from live input, and from the GNSS epochs of a replayed log when `use_gps_timestamp` is true. Otherwise a replay uses this value |
| `imu_scale_override.accel`, `.gyro` | `0.0` | NovAtel | RAWIMUSX scale factors; `0` selects them from the IMU type |
| `reference_week`, `reference_tow` | `0`, `0.0` | RTCM | GPS week/TOW near the start of a recorded RTCM stream (`0` = from the stream or host date) |
| `station_id` | `-1` | RTCM | accept only this station ID (`-1` = any) |
| `observation.excluded_systems` | `[]` | RTCM | constellations to drop: `G R E C J I S` |
| `rtcm_output.destination` | `""` | RTCM | forward the input bytes to this URI |
| `gga.solution_topic` | `""` | RTCM | send the latest position from this `GnssSolution` topic to the NTRIP caster as GGA every 5 s (`ntrip://` only, see below) |

## Decoded messages

### u-blox UBX

| Topic | Messages |
|---|---|
| observation | RXM-RAWX (and RXM-RAW) |
| ephemeris | RXM-SFRBX: GPS/QZSS LNAV, GLONASS, Galileo I/NAV and F/NAV, BeiDou D1/D2 |
| solution | NAV-PVT, NAV-HPPOSLLH, NAV-HPPOSECEF, NAV-POSECEF, NAV-VELECEF, NAV-COV, NAV-DOP; NMEA GGA/RMC/GSA/GST |
| imu_raw / imu | ESF-RAW / ESF-INS |

### Septentrio SBF

| Topic | Blocks |
|---|---|
| observation | MeasEpoch (4027), MeasExtra, Meas3 |
| ephemeris | GPSNav 5891, GLONav 4004, GALNav 4002, BDSNav 4081, QZSNav 4095; GPSRawCA 4017, GLORawCA 4026, GALRawINAV 4023, GALRawFNAV 4022, BDSRaw 4047, QZSRawL1CA 4066, NAVICRaw 4093 |
| solution | PVTGeodetic 4007, PVTCartesian 4006, PosCovGeodetic 5906, VelCovGeodetic 5908, PosCovCartesian 5905, VelCovCartesian 5907, DOP 4001; NMEA GGA/RMC/GSA/GST |
| imu_raw | ExtSensorMeas 4050 |

The cycle-slip flag (LLI bit 0) can be set on a continuously tracked signal when the receiver
moves that signal between the master and slave MeasEpoch sub-blocks, typically while the
first-frequency signal of the same satellite is weak; consumers treat it as a slip.

### NovAtel OEM4/6/7

| Topic | Logs |
|---|---|
| observation | RANGECMPB 140, RANGEB 43 |
| ephemeris | RAWEPHEMB 41, GLOEPHEMERISB 723, GALEPHEMERISB 1122, BDSEPHEMERISB 1696, QZSSRAWEPHEMB 1331, NAVICEPHEMERISB 2123, IONUTCB 8, GALIONOB 1127, QZSSIONUTCB 1347 |
| solution | BESTPOSB 42, BESTVELB 99, BESTXYZB 241, PSRDOPB 174; NMEA GGA/RMC/GSA/GST |
| imu_raw / imu | RAWIMUSXB 1462 / CORRIMUDATAB 812 |

NovAtel IMU messages keep the receiver axes (X lateral, Y longitudinal, Z up).

### RTCM 3

| Topic | Message types |
|---|---|
| observation | MSM4–7 (GPS 1074–1077, GLONASS 1084–1087, Galileo 1094–1097, SBAS 1104–1107, QZSS 1114–1117, BeiDou 1124–1127, NavIC 1134–1137); 1002, 1004, 1010, 1012 |
| ephemeris | 1019 GPS, 1020 GLONASS, 1041 NavIC, 1042 BeiDou, 1044 QZSS, 1045 Galileo F/NAV, 1046 Galileo I/NAV |

## Time stamps

| `use_gps_timestamp` | Observations / solutions | IMU |
|---|---|---|
| `false` | host receive time | host receive time |
| `true` | GNSS epoch as UTC | measurement time as UTC (UBX ESF-RAW: receiver time tag mapped onto GNSS time) |

With `true`, a message without valid GNSS time is not published. The GNSS epoch is
always in the `week` / `tow` fields.

A recorded RTCM stream without a full GPS week needs `reference_week` and
`reference_tow` set to a time within half a week of the recording.

## RTCM corrections to the receiver (on-chip RTK)

`rtcm_input.listen` makes the decoder host a TCP server and write the RTCM bytes
it receives to the receiver, so the receiver can compute RTK itself:

```bash
ros2 run gnss_ros_standardization ubx_decoder_node --ros-args \
  -p stream_path:=serial:///dev/ttyACM0:115200 -p rtcm_input.listen:=tcpsvr://:5556
ros2 run gnss_ros_standardization rtcm_decoder_node --ros-args \
  -p stream_path:=ntrip://user:pass@caster:2101/MOUNT \
  -p rtcm_output.destination:=tcpcli://127.0.0.1:5556
```

| Receiver | Port | Receiver setup |
|---|---|---|
| u-blox | 5556 | RTCM3 enabled in the port's `inProtoMask` (F9P default) |
| Septentrio | 5557 | `setDataInOut, COM1, RTCMv3, +SBF` |
| NovAtel | 5558 | a second port: `rtcm_input.receiver_stream:=serial:///dev/ttyUSB2:230400`; the decoder sends `INTERFACEMODE THISPORT RTCMV3 NONE OFF` on it |

The [`onboard_rtk` pipeline](../../launch/README.md#on-chip-rtk-through-the-rtcm-relay)
starts both nodes together.

## NTRIP services that need the rover position (GGA)

Services that choose or generate the base from the rover position (ichimill,
VRS) send corrections only after receiving a GGA. `gga.solution_topic` sends
the latest position on that topic every 5 s, starting with the first valid
position:

```bash
ros2 run gnss_ros_standardization rtcm_decoder_node --ros-args \
  -p stream_path:=ntrip://user:pass@caster:2101/MOUNT \
  -p gga.solution_topic:=/gnss/nmea_solution \
  -p rtcm_output.destination:=tcpcli://127.0.0.1:5556
```

Keep `station_id` at `-1`: these services change the base station during a
session.

Field definitions of the published messages: [topic contract](../../docs/TOPIC_DATA_CONTRACT.md).
