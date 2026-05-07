# gnss_ros_standardization

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy%20%7C%20Rolling-brightgreen)](https://docs.ros.org/en/humble/)

## Overview

**gnss_ros_standardization** is an open-source project that standardizes GNSS raw data handling for robotics and autonomous systems using ROS 2.

The goal is to make GNSS raw data more accessible and usable in real-time experiments, particularly for **tight coupling GNSS/IMU methods** and **multi-sensor fusion frameworks**.

By providing standardized ROS 2 / RTKLIB-based tools, this project enables developers and researchers to:

- Use GNSS raw observations seamlessly in robotics applications
- Perform real-time RTK/PPP with custom GNSS messages
- Support diverse GNSS receivers and formats (u-blox, Septentrio, NovAtel, etc.)
- Reproduce experiments across platforms with consistent interfaces

---

## Supported ROS 2 Distributions

| Distribution | Ubuntu | Status |
|---|---|---|
| Humble Hawksbill (LTS) | 22.04 | Supported |
| Jazzy Jalisco (LTS) | 24.04 | Supported |
| Rolling Ridley | 22.04 / 24.04 | Supported |

---

## Supported Receivers & Features

| Receiver | Raw Observations | Navigation / Ephemeris | GNSS Solution (NMEA) | IMU / INS Topics |
|---|---|---|---|---|
| u-blox (UBX) | GnssObservations | GnssEphemerides | GnssSolution | sensor_msgs/Imu (ESF-INS, NAV-ATT) |
| Septentrio (SBF) | GnssObservations | GnssEphemerides | GnssSolution | sensor_msgs/Imu (AttEuler, ExtSensorMeas) |
| NovAtel (OEM) | GnssObservations | GnssEphemerides | GnssSolution | sensor_msgs/Imu (INSPVA, CORRIMUDATA) |
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

#### `/gnss/imu/attitude` — Attitude (orientation quaternion)

| UBX Message | Class/ID | Description | Requirement |
|---|---|---|---|
| `UBX-NAV-ATT` | `0x01` / `0x05` | Roll, pitch, heading → orientation quaternion | ZED-F9R or IMU-enabled module |

#### `/gnss/imu/data_raw` — Raw IMU (accel + gyro)

| UBX Message | Class/ID | Description | Requirement |
|---|---|---|---|
| `UBX-ESF-INS` | `0x10` / `0x15` | Calibrated angular rate [rad/s] + linear acceleration [m/s²] | ZED-F9R or IMU-enabled module |

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

#### `/gnss/imu/attitude` — Attitude (orientation quaternion)

| SBF Block | ID | Description | Requirement |
|---|---|---|---|
| `AttEuler` | 5938 | Heading, pitch, roll → orientation quaternion | AsteRx-i / mosaic-X5 with dual-antenna or IMU |

#### `/gnss/imu/data_raw` — Raw IMU (accel + gyro)

| SBF Block | ID | Description | Requirement |
|---|---|---|---|
| `ExtSensorMeas` | 4050 | Linear acceleration [m/s²] (Type 0) and angular velocity [rad/s] (Type 1) as 3-axis vectors | AsteRx-i / mosaic-X5 with integrated IMU |

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

#### `/gnss/imu/attitude` — Attitude (orientation quaternion)

| NovAtel Log | Message ID | Description | Requirement |
|---|---|---|---|
| `INSPVAB` | 507 | INS position, velocity, attitude — roll/pitch/azimuth → orientation quaternion, output at 10 Hz | SPAN-capable receiver + IMU |

#### `/gnss/imu/data_raw` — Raw IMU (accel + gyro)

| NovAtel Log | Message ID | Description | Requirement |
|---|---|---|---|
| `CORRIMUDATAB` | 812 | Bias/gravity/earth-rate corrected accel [m/s²] and angular velocity [rad/s] at full IMU rate (ONNEW) | SPAN-capable receiver + IMU |

> **Note on CORRIMUDATA**: The log outputs SI-unit *increments* accumulated over one IMU sampling period.
> The driver/decoder divides each increment by the interval between successive timestamps (`dt`) to recover instantaneous rates and accelerations.

---

## System Requirements

- **OS**: Ubuntu 22.04 (Humble) or Ubuntu 24.04 (Jazzy)
- **ROS 2**: Humble, Jazzy, or Rolling
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

Convert a recorded ROS 2 bag file to RINEX 3.x observation and navigation files:

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

---

## Topic Reference

Default topic names (all can be overridden via YAML parameters):

| Topic | Type | Source |
|---|---|---|
| `/gnss/observation` | `GnssObservations` | All drivers / RTCM3 decoder |
| `/gnss/ephemeris` | `GnssEphemerides` | All drivers / RTCM3 decoder |
| `/gnss/solution` | `GnssSolution` | u-blox, Septentrio, NovAtel drivers (via NMEA) |
| `/gnss/nmea_solution` | `GnssSolution` | Decoder nodes (raw stream decoding) |
| `/gnss/imu/attitude` | `sensor_msgs/Imu` | Attitude quaternion — u-blox (NAV-ATT), Septentrio (AttEuler), NovAtel (INSPVA) |
| `/gnss/imu/data_raw` | `sensor_msgs/Imu` | Raw accel + gyro — u-blox (ESF-INS), Septentrio (ExtSensorMeas), NovAtel (CORRIMUDATA) |

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
- **v0.2** (current): GnssSolution via NMEA for Septentrio & NovAtel; IMU/INS topic output (`sensor_msgs/Imu`) for all receivers
- **v0.3** (planned): `nav_msgs/Odometry` from full INS solutions (INSPVA, INSNavGeod, ESF-INS fusion)
- **v1.0** (planned): Launch file templates, CI-tested multi-distro Docker images, extended RTK/PPP tooling

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
