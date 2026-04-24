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
| Septentrio (SBF) | GnssObservations | GnssEphemerides | GnssSolution | sensor_msgs/Imu (AttEuler) |
| NovAtel (OEM) | GnssObservations | GnssEphemerides | GnssSolution | sensor_msgs/Imu (INSPVA attitude) |
| RTCM3 (input) | GnssObservations | GnssEphemerides | — | — |

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
| `/gnss/imu/data` | `sensor_msgs/Imu` | u-blox (ESF-INS/NAV-ATT), Septentrio (AttEuler), NovAtel (INSPVA) |
| `/gnss/imu/data_raw` | `sensor_msgs/Imu` | u-blox driver (ESF-INS raw) |

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
