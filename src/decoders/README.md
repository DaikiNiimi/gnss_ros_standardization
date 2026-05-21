# Decoders

Decoder nodes consume a **binary GNSS byte stream** (NTRIP / TCP / Serial) and
publish ROS 2 messages. Unlike the drivers in [`../drivers/`](../drivers/),
decoders do **not** configure the receiver — they only parse. Use them when:

- You consume a third-party stream (NTRIP caster output, log replay).
- The receiver is configured by some other means (vendor utility, persistent flash).
- You want to embed RTKLIB-grade decoding into a custom ROS pipeline.

All binary decoding for raw observations / ephemeris is delegated to the embedded
[RTKLIB (rtklibexplorer fork)](https://github.com/rtklibexplorer/RTKLIB).
IMU/INS messages that RTKLIB does not handle are decoded by a parallel
mini-framer inside each decoder node.

| Node | Executable | Source | Input format |
|---|---|---|---|
| RTCM3 decoder | `rtcm_decoder_node` | [`rtcm_decoder_node.cpp`](rtcm_decoder_node.cpp) | RTCM 3.x (MSM + ephemeris) |
| u-blox decoder | `ubx_decoder_node` | [`ubx_decoder_node.cpp`](ubx_decoder_node.cpp) | UBX |
| Septentrio decoder | `sbf_decoder_node` | [`sbf_decoder_node.cpp`](sbf_decoder_node.cpp) | SBF |
| NovAtel decoder | `novatel_decoder_node` | [`novatel_decoder_node.cpp`](novatel_decoder_node.cpp) | OEM4 / OEM6 / OEM7 binary |

---

## Stream URI

All decoders accept the same `stream_path` parameter, parsed via RTKLIB's
`strsvr`. Supported schemes:

| Scheme | Example |
|---|---|
| NTRIP | `ntrip://user:pass@example.com:2101/MOUNT` |
| TCP client | `tcpcli://192.168.1.100:9000` |
| Serial | `serial:///dev/ttyUSB0:115200` |
| File replay | `file:///path/to/log.bin` |

---

## Published topics (common)

| Topic (parameter) | Default | Type |
|---|---|---|
| `observation_topic` | `/gnss/observation` | `gnss_ros_standardization/GnssObservations` |
| `ephemeris_topic` | `/gnss/ephemeris` | `gnss_ros_standardization/GnssEphemerides` (QoS: transient_local, depth 100) |
| `solution_topic` | `/gnss/nmea_solution` | `gnss_ros_standardization/GnssSolution` (UBX/SBF/OEM4 only) |
| `imu_topic` | `/gnss/imu/data` | `sensor_msgs/Imu` (UBX/OEM4 only) |
| `imu_raw_topic` | `/gnss/imu/data_raw` | `sensor_msgs/Imu` (UBX/SBF/OEM4) |

For the message definitions see [`../../msg/README.md`](../../msg/README.md).

---

## `rtcm_decoder_node`

```bash
ros2 run gnss_ros_standardization rtcm_decoder_node --ros-args \
  -p stream_path:="ntrip://user:password@host:port/mountpoint"
```

### Decoded RTCM messages

Performed by RTKLIB `input_rtcm3()`. Typical caster streams contain:

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

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `stream_path` | `""` | Stream URI (required) |
| `observation_topic` | `/gnss/observation` | Output topic |
| `ephemeris_topic` | `/gnss/ephemeris` | Output topic |
| `assemble_delay_ms` | `200` | Reserved |

---

## `ubx_decoder_node`

```bash
ros2 run gnss_ros_standardization ubx_decoder_node --ros-args \
  -p stream_path:="serial:///dev/ttyACM0:115200"
```

RTKLIB decoder: `input_ubx()` / `STRFMT_UBX`.

### Decoded UBX messages

| UBX Message | Class/ID | Output topic | Description |
|---|---|---|---|
| `UBX-RXM-RAWX` | `0x02 / 0x15` | `/gnss/observation` | Raw pseudorange, carrier phase, Doppler, SNR for all tracked signals |
| `UBX-RXM-SFRBX` | `0x02 / 0x13` | `/gnss/ephemeris` | Raw navigation subframe (GPS/GLO/GAL/BDS/QZS/NavIC/SBAS) — assembled to ephemeris by RTKLIB |
| `UBX-NAV-PVT` | `0x01 / 0x07` | `/gnss/nmea_solution` | Receiver PVT solution |
| `UBX-ESF-RAW` | `0x10 / 0x03` | `/gnss/imu/data_raw` | Raw IMU sensor samples (accel + gyro, internal scale) — requires ZED-F9R class |
| `UBX-ESF-INS` | `0x10 / 0x15` | `/gnss/imu/data` | Receiver-calibrated angular rate [rad/s] + linear accel [m/s²] — requires ZED-F9R class |
| NMEA `GGA/RMC/...` | — | `/gnss/nmea_solution` | Fallback solution when NAV-PVT is not output |

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `stream_path` | `serial:///dev/ttyACM0:115200` | Stream URI |
| `frame_id` | `gnss_link` | `header.frame_id` on published messages |
| `observation_topic` | `/gnss/observation` | |
| `ephemeris_topic` | `/gnss/ephemeris` | |
| `solution_topic` | `/gnss/nmea_solution` | |
| `imu_topic` | `/gnss/imu/data` | |
| `imu_raw_topic` | `/gnss/imu/data_raw` | |

---

## `sbf_decoder_node`

```bash
ros2 run gnss_ros_standardization sbf_decoder_node --ros-args \
  -p stream_path:="serial:///dev/ttyUSB0:115200"
```

RTKLIB decoder: `input_sbf()` / `STRFMT_SEPT`.

### Decoded SBF blocks

`/gnss/observation`:

| SBF Block | ID | Description |
|---|---|---|
| `MeasEpoch` | 4027 | Raw carrier phase, code range, Doppler, SNR for all signals |

`/gnss/ephemeris` (receiver-assembled — single block = complete ephemeris):

| SBF Block | ID | Contents |
|---|---|---|
| `GPSNav` | 5891 | GPS Keplerian ephemeris |
| `GLONav` | 4004 | GLONASS state-vector ephemeris |
| `GALNav` | 4002 | Galileo Keplerian ephemeris |
| `BDSNav` | 4081 | BeiDou Keplerian ephemeris |
| `QZSNav` | 4095 | QZSS Keplerian ephemeris |
| `NavICNav` | 4099 | NavIC Keplerian ephemeris |

`/gnss/ephemeris` (raw subframe blocks — decoded by RTKLIB, duplicates filtered by IODE/IODC):

| SBF Block | ID | Contents |
|---|---|---|
| `GPSRawCA` | 4017 | GPS L1 C/A navigation subframe |
| `GLORawCA` | 4026 | GLONASS L1 C/A navigation string |
| `GALRawINAV` | 4023 | Galileo I/NAV page |
| `GALRawFNAV` | 4022 | Galileo F/NAV page |
| `BDSRaw` | 4047 | BeiDou navigation message |
| `QZSRawL1CA` | 4066 | QZSS L1 C/A subframe |
| `NAVICRaw` | 4093 | NavIC navigation message |

`/gnss/imu/data_raw`:

| SBF Block | ID | Description | Requirement |
|---|---|---|---|
| `ExtSensorMeas` | 4050 | Linear accel [m/s²] (type 0) + angular velocity [rad/s] (type 1) | AsteRx-i / mosaic-X5 with IMU |

> Septentrio receivers do not output a separate calibrated IMU stream, so the
> SBF decoder does **not** publish `/gnss/imu/data`.

`/gnss/nmea_solution`: NMEA (GGA/RMC/GSA/GST) when emitted by the receiver.

### Parameters

Same as `ubx_decoder_node` plus no `imu_topic` (calibrated IMU not provided).

---

## `novatel_decoder_node`

```bash
ros2 run gnss_ros_standardization novatel_decoder_node --ros-args \
  -p stream_path:="serial:///dev/ttyUSB0:115200"
```

RTKLIB decoder: `input_oem4()` / `STRFMT_OEM4`.
IMU/INS messages are decoded by a parallel OEM4 mini-framer (RTKLIB does not
handle them).

### Decoded NovAtel logs

`/gnss/observation`:

| Log | Message ID | Description |
|---|---|---|
| `RANGECMPB` | 140 | Compressed range — pseudorange, carrier phase, Doppler, SNR (recommended) |
| `RANGEB` | 43 | Uncompressed range (alternative) |

`/gnss/ephemeris`:

| Log | Message ID | Contents |
|---|---|---|
| `GPSEPHEMB` | 7 | GPS ephemeris |
| `GLOEPHEMERISB` | 723 | GLONASS ephemeris |
| `GALEPHEMERISB` | 1122 | Galileo F/NAV |
| `GALINAVEPHEMERISB` | 1309 | Galileo I/NAV |
| `BDSEPHEMERISB` | 1696 | BeiDou |
| `QZSSEPHEMERISB` | 1336 | QZSS |
| `NAVICEPHEMERISB` | 2123 | NavIC |
| `IONUTCB` | 8 | GPS iono + UTC |
| `GALIONOB` | 1127 | Galileo iono |
| `QZSSIONUTCB` | 1347 | QZSS iono + UTC |

`/gnss/imu/data_raw`:

| Log | Message ID | Description | Requirement |
|---|---|---|---|
| `RAWIMUSXB` | 1462 | Raw IMU delta-velocity / delta-angle counts × IMU-type-specific scale | NovAtel receiver with IMU connected |

`/gnss/imu/data`:

| Log | Message ID | Description | Requirement |
|---|---|---|---|
| `CORRIMUDATAB` | 812 | Bias / gravity / earth-rate corrected accel [m/s²] + angular velocity [rad/s] | SPAN-capable receiver + IMU |

`/gnss/nmea_solution`: BESTPOS/BESTVEL or NMEA, whichever is logged.

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `stream_path` | `serial:///dev/ttyUSB0:115200` | Stream URI |
| `format` | `oem4` | Binary format selector |
| `frame_id` | `gnss_link` | |
| `observation_topic` | `/gnss/observation` | |
| `ephemeris_topic` | `/gnss/ephemeris` | |
| `solution_topic` | `/gnss/nmea_solution` | |
| `imu_topic` | `/gnss/imu/data` | |
| `imu_raw_topic` | `/gnss/imu/data_raw` | |
| `imu_scale_override.accel` | `0.0` (disabled) | Per-sample accel scale → m/s — overrides built-in table |
| `imu_scale_override.gyro` | `0.0` (disabled) | Per-sample gyro scale → rad — overrides built-in table |

For the RAWIMUSX per-IMU scale-factor table and how to extend it, see
[`../drivers/README.md`](../drivers/README.md#supported-novatel-imus-rawimusx-scale-table).

---

## Frame conventions for `GnssSolution`

Decoder output follows the same message-level frame conventions as everything else
in this package — see
[`../../msg/README.md`](../../msg/README.md#frame-conventions-important)
for the canonical definition (pos_enu anchoring, current-tangent covariance,
ECEF derivation). Decoders do not configure the receiver, so what gets aggregated
depends on which blocks are present in the incoming stream.

---

## Troubleshooting

**NTRIP connection fails**
- Verify credentials and mountpoint with an independent NTRIP client
  (e.g. `STRSVR` from RTKLIB).
- Check that `stream_path` matches the exact `ntrip://user:pass@host:port/MOUNT` format.

**No data on `/gnss/observation`**
- For RTCM: confirm the caster outputs MSM4/MSM7 (legacy 1001–1012 are not decoded here).
- For UBX: ensure `UBX-RXM-RAWX` and `UBX-RXM-SFRBX` are enabled in the receiver's persistent configuration.
- For SBF: ensure `MeasEpoch` is being streamed on the connected port.
- For OEM4: ensure `RANGECMPB ONTIME 1` (or `RANGEB`) is logged.

**No `/gnss/imu/data` from Septentrio**
- This is by design — see the note in the SBF table above.

**RAWIMUSX gives zero or nonsensical rates (NovAtel)**
- Confirm the IMU type byte against the table in
  [`../drivers/README.md`](../drivers/README.md#supported-novatel-imus-rawimusx-scale-table).
- If your IMU is not in the table, set `imu_scale_override.{accel,gyro}` to the
  per-sample scale factors from the IMU datasheet.
