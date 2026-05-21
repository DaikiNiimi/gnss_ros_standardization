# gnss_ros_standardization

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-brightgreen)](https://docs.ros.org/en/humble/)

## Overview

<p align="center">
  <img src="fig/overview.svg" alt="Repository overview and scope" width="850">
</p>

**gnss_ros_standardization** is an open-source ROS 2 package that standardizes
GNSS raw-data handling for robotics and autonomous systems.

The goal is to make GNSS raw observations and ephemeris uniformly accessible
across receiver brands so that tight-coupling GNSS/IMU methods and multi-sensor
fusion frameworks can be developed once and reused everywhere.

- Use GNSS raw observations seamlessly in robotics applications
- Support diverse GNSS receivers and formats (u-blox, Septentrio, NovAtel, RTCM3)
- Reproduce experiments across platforms with consistent interfaces

## Demo

Hardware and ROS 2 node configuration used in the demo:

<p align="center">
  <img src="fig/demo_setup.svg" alt="Hardware and ROS 2 node configuration" width="850">
</p>

Demo of real-time kinematic (RTK) positioning using this ROS 2 package:

<p align="center">
  <img src="fig/rtk_demo.gif" alt="RTK positioning demo" width="800">
</p>

## Supported ROS 2 distributions

| Distribution | Ubuntu | Status |
|---|---|---|
| Humble Hawksbill (LTS) | 22.04 | Supported |
| Jazzy Jalisco (LTS) | 24.04 | Supported |

## Supported receivers (summary)

| Receiver | Observations | Ephemeris | NMEA Solution | IMU measurement |
|---|---|---|---|---|
| u-blox (UBX) | ✓ | ✓ | ✓ | ✓ |
| Septentrio (SBF) | ✓ | ✓ | ✓ | ✓ |
| NovAtel (OEM4/6/7) | ✓ | ✓ | ✓ | ✓ |
| RTCM3 (MSM4/7) | ✓ | ✓ | — | — | — |

Per-receiver protocol message tables (UBX message IDs, SBF block IDs,
NovAtel log IDs, RTCM types) are shown in the component READMEs.

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

If you cloned the repository without `--recursive`, make sure to run `git submodule update --init --recursive`.

## Components

Each component has its own README with detailed information including message
tables, parameter lists, and `ros2 run` examples.

| Component | Purpose | README |
|---|---|---|
| Decoders | Stream-only: NTRIP / TCP / Serial → ROS topics | [src/decoders/README.md](src/decoders/README.md) |
| Drivers | Connect to receiver, configure outputs, decode | [src/drivers/README.md](src/drivers/README.md) |
| Converters | RINEX ↔ rosbag and RTKLIB `.pos` ↔ rosbag | [src/converter/README.md](src/converter/README.md) |
| Positioning | SPP, RTK, loose-coupled GNSS/IMU EKF | [src/positioning/README.md](src/positioning/README.md) |
| Messages | Public ROS message contract | [msg/README.md](msg/README.md) |

## Acknowledgements

This project uses [RTKLIB (rtklibexplorer fork)](https://github.com/rtklibexplorer/RTKLIB),
maintained by Tim Everett, based on [RTKLIB](https://github.com/tomojitakasu/RTKLIB)
by Tomoji Takasu, licensed under BSD 2-Clause.

## License

Released under the [MIT License](LICENSE).

## Contact

Maintainer: Daiki Niimi (daiki.niimi@ruri.waseda.jp)
