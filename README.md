# gnss_ros_standardization

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-brightgreen)](https://docs.ros.org/en/humble/)

## Overview

![Repository overview and scope](fig/overview.svg)

**gnss_ros_standardization** is an open-source ROS 2 package that standardizes
GNSS raw-data handling for robotics and autonomous systems.

The goal is to make GNSS raw observations and ephemeris uniformly accessible
across receiver brands so that tight-coupling GNSS/IMU methods and multi-sensor
fusion frameworks can be developed once and reused everywhere.

- Use GNSS raw observations seamlessly in robotics applications
- Perform real-time SPP / RTK with a unified message contract
- Support diverse GNSS receivers and formats (u-blox, Septentrio, NovAtel, RTCM3)
- Reproduce experiments across platforms with consistent interfaces

## Demo

Real-time RTK positioning produced by this package:

![RTK positioning demo](fig/rtk_demo.gif)

Hardware and ROS 2 node configuration used in the demo above:

![Demo setup](fig/demo_setup.svg)

## Scope

This package focuses on **GNSS raw-data standardization across receiver brands**.
A minimal IMU passthrough is included for timestamp consistency with GNSS
messages, limited to two ROS-standard topics:

- `/gnss/imu/data_raw` — uncalibrated raw IMU (`sensor_msgs/Imu`)
- `/gnss/imu/data` — calibrated IMU when the receiver provides it (`sensor_msgs/Imu`)

**Out of scope**: orientation/attitude topics, INS solutions, full sensor
fusion, `nav_msgs/Odometry` outputs. For attitude estimation, use the vendor
drivers ([`ublox_gps`](https://github.com/KumarRobotics/ublox),
[`septentrio_gnss_driver`](https://github.com/septentrio-gnss/septentrio_gnss_driver),
[`novatel_oem7_driver`](https://github.com/novatel/novatel_oem7_driver))
alongside this package — GNSS topics here remain time-aligned with vendor IMU
topics.

## Supported ROS 2 distributions

| Distribution | Ubuntu | Status |
|---|---|---|
| Humble Hawksbill (LTS) | 22.04 | Supported |
| Jazzy Jalisco (LTS) | 24.04 | Supported |

## Supported receivers (summary)

| Receiver | Observations | Ephemeris | NMEA Solution | IMU topics |
|---|---|---|---|---|
| u-blox (UBX) | ✓ | ✓ | ✓ | `/gnss/imu/data_raw`, `/gnss/imu/data` (ZED-F9R) |
| Septentrio (SBF) | ✓ | ✓ | ✓ | `/gnss/imu/data_raw` (AsteRx-i / mosaic-X5) |
| NovAtel (OEM4/6/7) | ✓ | ✓ | ✓ | `/gnss/imu/data_raw`, `/gnss/imu/data` (SPAN) |
| RTCM3 (input) | ✓ | ✓ | — | — |

Per-receiver protocol message tables (UBX message IDs, SBF block IDs,
NovAtel log IDs, RTCM types) live in the component READMEs linked below.

## Dependencies

- **OS**: Ubuntu 22.04 (Humble) or 24.04 (Jazzy)
- **ROS 2**: Humble or Jazzy
- **Build tool**: `colcon`
- **ROS packages** (installed via `rosdep`):
  `rclcpp`, `rcutils`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`,
  `builtin_interfaces`, `rosbag2_cpp`, `rosbag2_storage`, `cv_bridge`,
  `image_transport`
- **System libraries**: Eigen3 ≥ 3.3 (`sudo apt install libeigen3-dev`)
- **Third-party** (vendored as a git submodule):
  [RTKLIB (rtklibexplorer fork)](https://github.com/rtklibexplorer/RTKLIB)
  — RTKLIB by T. Takasu, demo5 fork by T. Everett (BSD 2-Clause)

## Installation

```bash
git clone --recursive https://github.com/DaikiNiimi/gnss_ros_standardization.git
cd gnss_ros_standardization
rosdep install --from-paths . --ignore-src -r -y
colcon build
source install/setup.bash
```

If you cloned without `--recursive`, run `git submodule update --init --recursive`.

## Components

Each component has its own README with the full message table, parameter list,
and `ros2 run` examples.

| Component | Purpose | README |
|---|---|---|
| Decoders | Stream-only: NTRIP / TCP / Serial → ROS topics | [src/decoders/README.md](src/decoders/README.md) |
| Drivers | Connect to receiver, configure outputs, decode | [src/drivers/README.md](src/drivers/README.md) |
| Converters | RINEX ↔ rosbag and RTKLIB `.pos` ↔ rosbag | [src/converter/README.md](src/converter/README.md) |
| Positioning | SPP, RTK, loose-coupled GNSS/IMU EKF | [src/positioning/README.md](src/positioning/README.md) |
| Messages | Public ROS message contract | [msg/README.md](msg/README.md) |

## Topic reference

Default topic names — all overridable via YAML parameters. These names form the
common interface that every component in this package publishes to or
subscribes from.

| Topic | Type | Source |
|---|---|---|
| `/gnss/observation` | `GnssObservations` | All drivers / decoders |
| `/gnss/ephemeris` | `GnssEphemerides` | All drivers / decoders — snapshot-on-change + 30 s heartbeat (single topic for both RINEX conversion and live positioning) |
| `/gnss/nmea_solution` | `GnssSolution` | Drivers / decoders — receiver-side fix. **Source is locked at startup**: if any binary PVT message is enabled (SBF `pvt_geodetic`/`pvt_cartesian`, NovAtel `bestpos`, UBX `nav_pvt`), the binary path is used (higher precision, full 3×3 covariance). Otherwise NMEA-parsed fix is used. No mid-session switching. |
| `/gnss/solution` | `GnssSolution` | Positioning nodes (SPP / RTK / GNSS-IMU EKF) — computed in this package |
| `/gnss/imu/data_raw` | `sensor_msgs/Imu` | u-blox (ESF-RAW), Septentrio (ExtSensorMeas), NovAtel (RAWIMUSX) |
| `/gnss/imu/data` | `sensor_msgs/Imu` | u-blox (ESF-INS), NovAtel (CORRIMUDATA, SPAN required) |

## Message reference

| Message | Description |
|---|---|
| `GnssObservation` | Single satellite observation (pseudorange, phase, Doppler, SNR) |
| `GnssObservations` | Array of `GnssObservation` with GNSS time stamp |
| `GnssEphemeris` | Keplerian ephemeris (GPS / Galileo / QZSS / BeiDou / NavIC / SBAS) |
| `GlonassEphemeris` | GLONASS ephemeris (PZ-90.11 state vector) |
| `GnssEphemerides` | Combined ephemeris message (GNSS + GLONASS arrays) |
| `GnssSolution` | Receiver positioning solution (LLH, ECEF, ENU, velocity, covariance) |

Field-level details (units, frames, enum values): [msg/README.md](msg/README.md).

## Goals

- Enable **easy and broad utilization of GNSS raw data** in robotics
- Facilitate **development and testing of tight-coupling GNSS positioning** with other sensors
- Provide tools for **real-time, reproducible experiments**

## Roadmap

- **v0.1**: Raw observation/ephemeris output for all three receivers; RINEX conversion; RTCM3 input
- **v0.2** (current): `GnssSolution` via NMEA for Septentrio & NovAtel; minimal IMU passthrough
- **v1.0** (planned): Launch file templates, CI-tested multi-distro Docker images, extended RTK/PPP tooling

Attitude / INS / sensor-fusion outputs are intentionally out of scope — see *Scope*.

## References

- RTKLIB Manual: <https://www.rtklib.com/>
- RTKLIB source (rtklibexplorer fork): <https://github.com/rtklibexplorer/RTKLIB>
- RINEX 3.x format: <https://files.igs.org/pub/data/format/rinex_4.00.pdf>
- RTCM 10403.x standards: <https://www.rtcm.org/>
- u-blox UBX protocol: <https://www.u-blox.com/en/docs>
- Septentrio SBF reference guide: <https://www.septentrio.com/en/support/documentation>
- NovAtel OEM7 command/log reference: <https://docs.novatel.com/OEM7/Content/Home.htm>

## Troubleshooting

**Build fails: `Eigen3` not found** — `sudo apt install libeigen3-dev`

For component-specific troubleshooting (serial permissions, NTRIP connectivity,
NovAtel IMU scale factors, etc.), see the respective component README.

## Acknowledgements

This project uses [RTKLIB (rtklibexplorer fork)](https://github.com/rtklibexplorer/RTKLIB),
maintained by Tim Everett, based on [RTKLIB](https://github.com/tomojitakasu/RTKLIB)
by Tomoji Takasu, licensed under BSD 2-Clause.

## License

Released under the [MIT License](LICENSE).

## Contributing

Contributions are welcome. Please read [CONTRIBUTING.md](CONTRIBUTING.md) for
guidelines on reporting issues, submitting pull requests, and the development
workflow.

## Contact

Maintainer: Daiki Niimi (dai12.gnss@gmail.com)
