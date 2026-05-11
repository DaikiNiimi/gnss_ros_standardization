# gnss_ros_standardization

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-brightgreen)](https://docs.ros.org/en/humble/)

## Overview

**gnss_ros_standardization** is an open-source project that standardizes GNSS raw data handling for robotics and autonomous systems using ROS 2.

The goal is to make GNSS raw data more accessible and usable in real-time experiments, particularly for **tight coupling GNSS/IMU methods** and **multi-sensor fusion frameworks**.

By providing standardized ROS 2 / RTKLIB-based tools, this project enables developers and researchers to:

- Use GNSS raw observations seamlessly in robotics applications
- Perform real-time RTK/PPP with custom GNSS messages
- Support diverse GNSS receivers and formats (u-blox, Septentrio, NovAtel, etc.)
- Reproduce experiments across platforms with consistent interfaces

---

## Scope

This package focuses on **GNSS raw data standardization across receiver brands**.
A minimal IMU passthrough is included for timestamp consistency with GNSS messages,
limited to two ROS-standard topics:

- `/gnss/imu/data_raw` — uncalibrated raw IMU (`sensor_msgs/Imu`)
- `/gnss/imu/data` — calibrated IMU when the receiver provides it (`sensor_msgs/Imu`)

**Out of scope**: orientation/attitude topics, INS solutions, sensor fusion,
`nav_msgs/Odometry` outputs. For full IMU/INS support including attitude
estimation, please use the receiver-vendor's dedicated drivers:

- u-blox: [`ublox_gps`](https://github.com/KumarRobotics/ublox)
- Septentrio: [`septentrio_gnss_driver`](https://github.com/septentrio-gnss/septentrio_gnss_driver)
- NovAtel: [`novatel_oem7_driver`](https://github.com/novatel/novatel_oem7_driver)

These can be run alongside this package — the GNSS topics from here will align
in time with the IMU topics from the vendor drivers, and ROS-standard fusion
nodes (`imu_filter_madgwick`, `robot_localization`) work on the topics from
either source.

---

## Supported ROS 2 Distributions

| Distribution | Ubuntu | Status |
|---|---|---|
| Humble Hawksbill (LTS) | 22.04 | Supported |
| Jazzy Jalisco (LTS) | 24.04 | Supported |
---

## Supported Receivers & Features

| Receiver | Raw Observations | Navigation / Ephemeris | GNSS Solution (NMEA) | IMU Topics |
|---|---|---|---|---|
| u-blox (UBX) | GnssObservations | GnssEphemerides | GnssSolution | `/gnss/imu/data_raw` (ESF-RAW), `/gnss/imu/data` (ESF-INS) |
| Septentrio (SBF) | GnssObservations | GnssEphemerides | GnssSolution | `/gnss/imu/data_raw` (ExtSensorMeas) |
| NovAtel (OEM) | GnssObservations | GnssEphemerides | GnssSolution | `/gnss/imu/data_raw` (RAWIMUSX), `/gnss/imu/data` (CORRIMUDATA, SPAN required) |
| RTCM3 (input) | GnssObservations | GnssEphemerides | — | — |

---

## Protocol Decoding Reference

This section documents which proprietary receiver messages are decoded to produce each ROS 2 topic.
All binary decoding (observation / ephemeris) is performed by the embedded [MALIB](https://github.com/DaikiNiimi/MALIB) (RTKLIB fork) decoder.
IMU/INS messages that RTKLIB does not handle are decoded by a parallel mini-framer inside each driver/decoder node.

### u-blox (UBX protocol)

RTKLIB decoder: `input_ubx()` / `STRFMT_UBX`

#### `/gnss/observation` — Raw Observations

| UBX Message | Class/ID | Description |
|---|---|---|
| `UBX-RXM-RAWX` | `0x02` / `0x15` | Raw pseudorange, carrier phase, Doppler, SNR for all tracked signals and frequencies |

#### `/gnss/ephemeris` — Navigation Messages

| UBX Message | Class/ID | Contents |
|---|---|---|
| `UBX-RXM-SFRBX` | `0x02` / `0x13` | Raw navigation subframe/string/page for GPS, GLONASS, Galileo, BeiDou, QZSS, NavIC, SBAS |

#### `/gnss/imu/data_raw` — Uncalibrated raw IMU

| UBX Message | Class/ID | Description | Requirement |
|---|---|---|---|
| `UBX-ESF-RAW` | `0x10` / `0x03` | Raw IMU sensor measurements (accel + gyro, IMU-internal scale) | ZED-F9R or IMU-enabled module |

#### `/gnss/imu/data` — Calibrated IMU

| UBX Message | Class/ID | Description | Requirement |
|---|---|---|---|
| `UBX-ESF-INS` | `0x10` / `0x15` | Receiver-calibrated angular rate [rad/s] + linear acceleration [m/s²] | ZED-F9R or IMU-enabled module |

---

### Septentrio (SBF protocol)

RTKLIB decoder: `input_sbf()` / `STRFMT_SEPT`

#### `/gnss/observation` — Raw Observations

| SBF Block | ID | Description |
|---|---|---|
| `MeasEpoch` | 4027 | Raw carrier phase, code range, Doppler, SNR for all signals |

#### `/gnss/ephemeris` — Navigation Messages

**Decoded blocks** (receiver-assembled; single block = complete ephemeris):

| SBF Block | ID | Contents |
|---|---|---|
| `GPSNav` | 5891 | GPS complete Keplerian ephemeris |
| `GLONav` | 4004 | GLONASS complete state-vector ephemeris |
| `GALNav` | 4002 | Galileo complete Keplerian ephemeris |
| `BDSNav` | 4081 | BeiDou complete Keplerian ephemeris |
| `QZSNav` | 4095 | QZSS complete Keplerian ephemeris |
| `NavICNav` | 4099 | NavIC complete Keplerian ephemeris |

**Raw subframe blocks** (decoded by RTKLIB `input_sbf()`; duplicates filtered by IODE/IODC):

| SBF Block | ID | Contents |
|---|---|---|
| `GPSRawCA` | 4017 | GPS L1 C/A navigation subframe |
| `GLORawCA` | 4026 | GLONASS L1 C/A navigation string |
| `GALRawINAV` | 4023 | Galileo I/NAV navigation page |
| `GALRawFNAV` | 4022 | Galileo F/NAV navigation page |
| `BDSRaw` | 4047 | BeiDou navigation message |
| `QZSRawL1CA` | 4066 | QZSS L1 C/A navigation subframe |
| `NAVICRaw` | 4093 | NavIC navigation message |

#### `/gnss/imu/data_raw` — Uncalibrated raw IMU

| SBF Block | ID | Description | Requirement |
|---|---|---|---|
| `ExtSensorMeas` | 4050 | Linear acceleration [m/s²] (Type 0) and angular velocity [rad/s] (Type 1) as 3-axis vectors | AsteRx-i / mosaic-X5 with integrated IMU |

> Septentrio does not provide a separate calibrated IMU output — `/gnss/imu/data` is not advertised by the SBF driver/decoder.

---

### NovAtel (OEM4/6/7 binary protocol)

RTKLIB decoder: `input_oem4()` / `STRFMT_OEM4` (or `input_oem3()` for legacy OEM3 format)
IMU/INS messages decoded by a parallel OEM4 mini-framer (RTKLIB does not handle these).

#### `/gnss/observation` — Raw Observations

| NovAtel Log | Message ID | Description |
|---|---|---|
| `RANGECMPB` | 140 | Compressed range measurements — pseudorange, carrier phase, Doppler, SNR (recommended) |
| `RANGEB` | 43 | Uncompressed range measurements (alternative) |

#### `/gnss/ephemeris` — Navigation Messages

| NovAtel Log | Message ID | Contents |
|---|---|---|
| `GPSEPHEMB` | 7 | GPS ephemeris (Keplerian elements) |
| `GLOEPHEMERISB` | 723 | GLONASS ephemeris (state vector) |
| `GALEPHEMERISB` | 1122 | Galileo F/NAV ephemeris |
| `GALINAVEPHEMERISB` | 1309 | Galileo I/NAV ephemeris |
| `BDSEPHEMERISB` | 1696 | BeiDou ephemeris |
| `QZSSEPHEMERISB` | 1336 | QZSS ephemeris |
| `NAVICEPHEMERISB` | 2123 | NavIC ephemeris |
| `IONUTCB` | 8 | GPS ionosphere + UTC parameters |
| `GALIONOB` | 1127 | Galileo ionosphere parameters |
| `QZSSIONUTCB` | 1347 | QZSS ionosphere + UTC parameters |

#### `/gnss/imu/data_raw` — Uncalibrated raw IMU

| NovAtel Log | Message ID | Description | Requirement |
|---|---|---|---|
| `RAWIMUSXB` | 1462 | Raw IMU delta-velocity / delta-angle counts × IMU-type-specific scale factor (ONNEW) | NovAtel receiver with IMU connection |

#### `/gnss/imu/data` — Calibrated IMU

| NovAtel Log | Message ID | Description | Requirement |
|---|---|---|---|
| `CORRIMUDATAB` | 812 | Bias/gravity/earth-rate corrected accel [m/s²] and angular velocity [rad/s] at full IMU rate (ONNEW) | SPAN-capable receiver + IMU |

> **Note on RAWIMUSX scale factors**: The receiver embeds an `IMUType` byte in each
> message identifying the connected IMU. The driver / decoder looks up the per-sample
> count→SI scale factor (delta-velocity in m/s, delta-angle in rad) from a table in
> [`novatel_imu_scales.hpp`](include/gnss_ros_standardization/novatel_imu_scales.hpp)
> and divides by the inter-sample `dt` (from successive `Seconds` fields) to recover
> rates and accelerations. See *Supported NovAtel IMUs* below.

> **Note on CORRIMUDATA**: The log outputs SI-unit *increments* accumulated over one IMU sampling period.
> The driver/decoder divides each increment by the interval between successive timestamps (`dt`) to recover instantaneous rates and accelerations.

#### Supported NovAtel IMUs (RAWIMUSX scale table)

| IMUType | IMU Model | accel scale (counts → m/s) | gyro scale (counts → rad) |
|---------|-----------|---------------------------|---------------------------|
| 4 | Honeywell HG1700-AG11 | 2⁻²² | 2⁻³³ |
| 5 | Honeywell HG1700-AG17 | 2⁻²² | 2⁻³³ |
| 8 | Honeywell HG1700-AG58 | 2⁻²² | 2⁻³³ |
| 11 | Honeywell HG1700-AG62 | 2⁻²² | 2⁻³³ |
| 13 | Honeywell HG1900-CA50 | 2⁻²¹ | 2⁻³³ |
| 16 | Northrop Grumman LN200 | 2⁻¹⁴ | 2⁻¹⁹ |
| 19 | Honeywell HG1930 | 2⁻²² | 2⁻³³ |
| 26, 28 | Analog Devices ADIS16488 | 2⁻¹² | 2⁻²¹ |
| 31 | Sensonor STIM 300 | 2⁻²¹ | 2⁻²⁵ |
| 41 | Epson G320N | 2⁻¹⁵ | 2⁻²¹ |
| 56 | KVH 1750 | 2⁻¹⁵ | 2⁻²¹ |
| 58, 68, 69 | Epson G370N / G382PR | 2⁻¹⁵ | 2⁻²¹ |

If your IMU type is not in the table, set `imu_scale_override.{accel,gyro}` in
`config/novatel_driver.yaml` to the per-sample SI scale factors from the IMU
datasheet. To contribute permanent support for a new IMU, add a `case` branch
to `getImuScale()` in
[`novatel_imu_scales.hpp`](include/gnss_ros_standardization/novatel_imu_scales.hpp)
and update this table — please open a PR.

---

## System Requirements

- **OS**: Ubuntu 22.04 (Humble) or Ubuntu 24.04 (Jazzy)
- **ROS 2**: Humble or Jazzy
- **Build tool**: colcon
- **Dependencies** (installed via rosdep):
  - `rclcpp`, `rcutils`
  - `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `builtin_interfaces`
  - `rosbag2_cpp`, `rosbag2_storage`
  - `Eigen3` (≥ 3.3)
  - `cv_bridge`, `image_transport`
- **Third-party** (included as git submodule):
  - [MALIB](https://github.com/DaikiNiimi/MALIB) — RTKLIB-based GNSS library (BSD 2-Clause)

---

## Goals

- Enable **easy and broad utilization of GNSS raw data** in robotics
- Facilitate **development and testing of tight coupling GNSS positioning methods** with other sensors
- Provide tools to support **real-time, reproducible experiments**

---

## Installation

```bash
# Clone repository with submodules
git clone --recursive https://github.com/DaikiNiimi/gnss_ros_standardization.git
cd gnss_ros_standardization

# Install ROS 2 dependencies
rosdep install --from-paths . --ignore-src -r -y

# Build
colcon build
source install/setup.bash
```

---

## Usage

### RTCM3 Decoder — Publish Raw GNSS Data from NTRIP / TCP / Serial

```bash
ros2 run gnss_ros_standardization rtcm_decoder_node --ros-args \
  -p stream_path:="ntrip://user:password@host:port/mountpoint"
```

Supported stream URI schemes:

| Scheme | Example |
|---|---|
| NTRIP | `ntrip://user:pass@example.com:2101/MOUNT` |
| TCP client | `tcpcli://192.168.1.100:9000` |
| Serial | `serial://dev/ttyUSB0:115200` |

Published topics:

| Topic | Type | Description |
|---|---|---|
| `/gnss/observation` | `GnssObservations` | Raw pseudorange / carrier-phase / Doppler |
| `/gnss/ephemeris` | `GnssEphemerides` | Satellite navigation messages |

---

### u-blox Driver — Direct Serial Connection

```bash
# Copy and edit the sample config
cp config/ubx_driver.yaml my_ubx.yaml
# Edit: serial_port, baud_rate, enable flags as needed

ros2 run gnss_ros_standardization ubx_driver_node --ros-args \
  --params-file my_ubx.yaml
```

---

### Septentrio Driver

```bash
ros2 run gnss_ros_standardization sbf_driver_node --ros-args \
  --params-file config/sbf_driver.yaml
```

---

### NovAtel Driver

```bash
ros2 run gnss_ros_standardization novatel_driver_node --ros-args \
  --params-file config/novatel_driver.yaml
```

---

### ROS 2 Bag → RINEX Converter

Convert a recorded ROS 2 bag file to RINEX observation and navigation files:

```bash
ros2 run gnss_ros_standardization rosbag_to_rinex \
  --bag bag.db3 \
  --topic-obs /gnss/observation \
  --topic-nav /gnss/ephemeris \
  --obs output.obs \
  --nav output.nav \
  --rnx-version 3.04 \
  --nav-systems "GREJC"
```

**Supported RINEX versions (write):** 3.00 – 3.05. Values outside this range
are rejected by `--rnx-version`. RINEX 2.xx and 4.xx are not supported on the
write path.

### RINEX → ROS 2 Bag Converter

Convert RINEX observation/navigation files to a ROS 2 bag with
`/gnss/observation` and `/gnss/ephemeris` topics:

```bash
ros2 run gnss_ros_standardization rinex_to_rosbag \
  --obs input.obs \
  --nav input.nav \
  --out my_bag
```

**Supported RINEX versions (read):** 2.10 – 3.05. RINEX 4.xx is not supported; decompress with
`crx2rnx` before conversion.

### ROS 2 Bag ↔ RTKLIB .pos Solution Converter

Convert between `/gnss/solution` (`GnssSolution`) and the RTKLIB `.pos`
solution format produced by `rnx2rtkp` / `rtkpost`:

```bash
# rosbag -> .pos
ros2 run gnss_ros_standardization rosbag_to_pos \
  --bag bag.db3 --topic /gnss/solution --out output.pos

# .pos -> rosbag
ros2 run gnss_ros_standardization pos_to_rosbag \
  --pos input.pos --out my_bag --topic /gnss/solution
```

Both LLH (`lat/lon/height`) and ECEF (`x/y/z`) `.pos` formats are supported.

---

## Topic Reference

Default topic names (all can be overridden via YAML parameters):

| Topic | Type | Source |
|---|---|---|
| `/gnss/observation` | `GnssObservations` | All drivers / RTCM3 decoder |
| `/gnss/ephemeris` | `GnssEphemerides` | All drivers / RTCM3 decoder |
| `/gnss/solution` | `GnssSolution` | u-blox, Septentrio, NovAtel drivers (via NMEA) |
| `/gnss/nmea_solution` | `GnssSolution` | Decoder nodes (raw stream decoding) |
| `/gnss/imu/data_raw` | `sensor_msgs/Imu` | Uncalibrated raw IMU — u-blox (ESF-RAW), Septentrio (ExtSensorMeas), NovAtel (RAWIMUSX) |
| `/gnss/imu/data` | `sensor_msgs/Imu` | Calibrated IMU — u-blox (ESF-INS), NovAtel (CORRIMUDATA, SPAN required); Septentrio: not provided |

## Message Reference

| Message | Package | Description |
|---|---|---|
| `GnssObservation` | `gnss_ros_standardization` | Single satellite observation (pseudorange, phase, Doppler, SNR) |
| `GnssObservations` | `gnss_ros_standardization` | Array of `GnssObservation` with GNSS time stamp |
| `GnssEphemeris` | `gnss_ros_standardization` | Keplerian ephemeris (GPS/Galileo/QZSS/BeiDou/NavIC/SBAS) |
| `GlonassEphemeris` | `gnss_ros_standardization` | GLONASS ephemeris (state vector) |
| `GnssEphemerides` | `gnss_ros_standardization` | Combined ephemeris message (GNSS + GLONASS arrays) |
| `GnssSolution` | `gnss_ros_standardization` | Receiver positioning solution (LLH, ECEF, ENU, velocity, covariance) |

---

## Roadmap

- **v0.1** (current): Raw observation/ephemeris output for all three receivers; RINEX conversion; RTCM3 input
- **v0.2** (current): GnssSolution via NMEA for Septentrio & NovAtel; minimal IMU passthrough (`sensor_msgs/Imu`) on `/gnss/imu/data_raw` and `/gnss/imu/data`
- **v1.0** (planned): Launch file templates, CI-tested multi-distro Docker images, extended RTK/PPP tooling

> Attitude / INS / sensor-fusion outputs are intentionally out of scope — see *Scope* above.

See [CONTRIBUTING.md](CONTRIBUTING.md) if you would like to contribute.

---

## Troubleshooting

**Build fails: `Eigen3` not found**
```bash
sudo apt install libeigen3-dev
```

**Serial port permission denied**
```bash
sudo usermod -aG dialout $USER
# Log out and back in
```

**NTRIP connection fails**
- Verify credentials and mountpoint with an independent NTRIP client (e.g., STRSVR from RTKLIB).
- Check that `stream_path` matches the exact URI format shown in the Usage section.

**No data on `/gnss/observation`**
- Confirm the receiver outputs RTCM3 MSM7 messages.
- For u-blox: ensure `UBX-RXM-RAWX` and `UBX-RXM-SFRBX` are enabled in the receiver configuration.

---

## Acknowledgements

This project uses [MALIB](https://github.com/DaikiNiimi/MALIB), a fork of [RTKLIB](https://github.com/tomojitakasu/RTKLIB) by Tomoji Takasu, licensed under the BSD 2-Clause License.

---

## License

This project is released under the [MIT License](LICENSE).

---

## Contributing

Contributions are welcome. Please read [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines on reporting issues, submitting pull requests, and the development workflow.

---

## Contact

Maintainer: Daiki Niimi (dai12.gnss@gmail.com)
