# GNSS_ROS_Standardization

## Overview
**GNSS_ROS_Standardization** is an open-source project to **standardize GNSS raw data handling** in the context of robotics and autonomous systems.  
The goal is to make GNSS raw data more accessible and usable in real-time experiments, particularly for **tight coupling GNSS/IMU methods** and **multi-sensor fusion frameworks**.  

By providing standardized ROS/RTKLIB-based tools, this project enables developers and researchers to:
- Use GNSS raw observations seamlessly in robotics applications
- Perform real-time RTK/PPP with custom GNSS messages
- Support diverse GNSS receivers and formats (u-blox, Septentrio, NovAtel, etc.)
- Reproduce experiments across platforms with consistent interfaces  

---

## Goals
- Enable **easy and broad utilization of GNSS raw data** in robotics fields  
- Facilitate **development and testing of tight coupling GNSS positioning methods** with other sensors  
- Provide tools to support **real-time, reproducible experiments**  

---

## Roadmap / To Do
- **Survey Existing Implementations**
  - [ublox](https://github.com/KumarRobotics/ublox)
  - [septentrio_gnss_driver](https://github.com/septentrio-gnss/septentrio_gnss_driver)
  - [novatel_oem7_driver](https://github.com/novatel/novatel_oem7_driver)
  - [novatel_gps_driver](https://github.com/swri-robotics/novatel_gps_driver)
  - [GICI-LIB](https://github.com/chichengcn/gici-open)  
  - [ublox_driver](https://github.com/HKUST-Aerial-Robotics/ublox_driver)
  - [gnss_comm](https://github.com/HKUST-Aerial-Robotics/gnss_comm)  

- **Repository & Management**  
  - Manage with Git (submodules, similar to MatRTKLIB structure)  

- **Message Definitions**  
  - Define baseband and raw observation message types  
  - Implement `TimeStamp`-based synchronization  
  - Implement `RINEX obs/nav` equivalent messages  
  - Provide universal solution format (NavSatFix equivalent, absolute/relative coordinates, UTM, ENU, etc.)  

- **Message Support**  
  - Support **RTCM3 MSM7** input/output  
  - Support receiver-specific raw messages:  
    - u-blox: RAWX, SFRBX  
    - Septentrio: MeasEpoch, satellite NAV  
    - NovAtel: OEM messages  

- **RTKLIB Integration**  
  - Support standard RTKLIB I/O  
  - Add custom drivers for raw data export in RTCM + custom message format  

- **Example Applications**  
  - Real-time RTK positioning using RTKLIB with custom ROS messages  
  - Conversion tools (e.g., `rosbag` ↔ RINEX)  

- **Tools**  
  - RINEX ⇔ ROS message converters  
  - Replay functionality from ROS messages  

---

## First Milestones
- Target **u-blox F9P** as initial receiver  
- Support **RTCM MSM7** input/output  
- Verify **real-time functionality** (positioning + reproducibility)  
- Add support for **custom messages**  
- Extend to **Septentrio** and **NovAtel**  

---

## Installation
```bash
# Clone repository with submodules
git clone --recursive https://github.com/your-org/GNSS_ROS_Standardization.git](https://github.com/DaikiNiimi/gnss_ros_standardization.git
cd gnss_ros_standardization

# Build with colcon
colcon build
source install/setup.bash

# usage

