---
title: 'gnss_ros_standardization: A ROS 2 framework for receiver-independent handling of raw GNSS observations'
tags:
  - C++
  - ROS 2
  - GNSS
  - RTKLIB
  - robotics
  - sensor fusion
  - factor graph optimization
authors:
  - name: Daiki Niimi
    orcid: 0009-0007-0185-9430
    corresponding: true
    affiliation: 1
  - name: Taro Suzuki
    orcid: 0000-0001-9722-9415
    affiliation: 2
  - name: Junichi Meguro
    orcid: 0009-0008-5992-6453
    affiliation: 3
  - name: Yoshiharu Amano
    orcid: 0000-0003-3480-6777
    affiliation: 1
affiliations:
  - name: Department of Applied Mechanics and Aerospace Engineering, Waseda University, Japan
    index: 1
    ror: 00ntfnx83
  - name: Department of Advanced Robotics, Chiba Institute of Technology, Japan
    index: 2
    ror: 00qwnam72
  - name: Department of Mechatronics Engineering, Meijo University, Japan
    index: 3
    ror: 04h42fc75
date: 15 November 2026
bibliography: paper.bib
---

# Summary

Global navigation satellite systems (GNSS), including GPS, enable receivers to estimate their positions on Earth. Rather than measuring its position directly, a GNSS receiver estimates it from *raw observations*, which are distance-related measurements derived from signals transmitted by multiple satellites. These raw observations are often used for robust GNSS positioning and tightly coupled fusion of GNSS with other sensors, especially in challenging outdoor environments. However, integrating raw GNSS observations into robotic systems built on the Robot Operating System 2 (ROS 2) [@macenski2022] remains challenging because GNSS receivers use manufacturer-specific data formats, and ROS 2 lacks a widely adopted, receiver-independent interface for these raw observations.

`gnss_ros_standardization` provides a receiver-independent framework for handling raw GNSS observations in ROS 2. It converts data from receivers of different manufacturers into common ROS 2 messages, so that positioning and sensor-fusion algorithms developed for one receiver can be reused with others. It also converts in both directions between ROS 2 recordings and RINEX, the standard GNSS file format, allowing existing GNSS datasets to be replayed in ROS 2 and ROS 2 recordings to be processed by conventional GNSS software.

# Statement of need

Robust positioning in challenging environments, such as dense urban areas, increasingly relies on methods that process raw GNSS observations directly and tightly integrate them with inertial measurement unit (IMU), visual, or light detection and ranging (LiDAR) measurements [@cao2022gvins; @wen2022fgo; @liu2024glio]. However, developing and evaluating such methods in ROS 2 presents three main challenges.

**No standard interface for raw GNSS observations:** The standard ROS 2 message type for GNSS, `sensor_msgs/msg/NavSatFix`, represents only position estimates and their associated status and uncertainty. ROS 2 provides no standard message type for raw GNSS observations such as pseudorange, carrier phase, and Doppler. Consequently, applications often depend on driver-specific message definitions, limiting the reuse of positioning algorithms across different receivers.

**Receiver-specific binary protocols:** Even with a common message definition, raw GNSS observations must be decoded from receiver-specific binary formats, such as u-blox UBX, Septentrio SBF, and NovAtel OEM, whose different message structures and signal identifiers add development and maintenance effort for each receiver.

**Limited interoperability with existing GNSS datasets:** GNSS datasets are commonly distributed in RINEX, a standard file format for post-processing, whereas ROS 2 algorithms process time-stamped message streams, so RINEX datasets and ROS 2 bags cannot be used interchangeably without conversion. This hinders dataset reuse and algorithm evaluation across live and recorded data.

`gnss_ros_standardization` addresses these challenges through three complementary features:

- Common ROS 2 message structures are defined based on those of RTKLIB [@takasu2009; @rtklibex2026], a widely used open-source GNSS processing library. These messages provide a receiver-independent representation of raw GNSS observations and facilitate the reuse of RTKLIB's existing functions for positioning and data processing.

- Decoder and driver nodes convert binary data from u-blox, Septentrio, and NovAtel receivers, as well as the RTCM3 format, into the common ROS 2 messages with unified signal identifiers, so downstream modules need not handle receiver-specific formats and signal mappings.

- Converters translate RINEX files into ROS 2 bags containing the same messages as live receiver data, and ROS 2 bags back into RINEX, so that existing datasets can share ROS 2 pipelines with live observations and ROS 2 recordings can be analyzed with conventional GNSS software.

`gnss_ros_standardization` targets researchers in the GNSS and robotics communities developing positioning and multi-sensor localization algorithms in ROS 2, as well as research groups collecting and sharing GNSS datasets.

# State of the field

In the GNSS community, RTKLIB [@takasu2009; @takasu2020rtklib] and its fork RTKLIB-EX [@rtklibex2026; @everett2022] are widely used open-source GNSS processing libraries. They support various receiver protocols, RINEX input and output, and standard positioning algorithms, but do not provide a native ROS interface.

In robotics, several open-source ROS packages integrate raw GNSS observations into robotic systems. Vendor-specific drivers [@dome2026; @novatel2026] and academic or community-developed packages [@meyer2025; @hortovanyi2026; @reed2026; @cao2024; @cao2021] primarily support individual receiver manufacturers and use different message interfaces. In contrast, `gici-open`, the open-source implementation of GICI-LIB [@chi2023], supports multiple GNSS data formats but was designed as an integrated navigation system that fuses GNSS, IMU, and camera measurements rather than as a general-purpose ROS interface for raw GNSS data. Furthermore, it supports only ROS 1, which reached end-of-life in May 2025.

The capabilities of these packages are compared in \autoref{tab:comparison}. Among the existing packages reviewed, none combines support for multiple GNSS receiver formats, ROS 2, and bidirectional RINEX–ROS 2 bag conversion within a single package. Extending a receiver-specific driver would require replacing its manufacturer-specific messages, on which its existing users depend, and adapting `gici-open` would require porting it to ROS 2 and separating its GNSS interface from its estimator. We therefore developed `gnss_ros_standardization` as a standalone ROS 2 package that reuses RTKLIB for decoding, RINEX handling, and positioning.

: Comparison of representative open-source ROS software for raw GNSS observation handling and RINEX interoperability. \label{tab:comparison}

| Software | ROS | Supported formats | Bag → RINEX | RINEX → Bag |
|:-----------------------------------|:-------:|:-------------------------|:------------:|:------------:|
| `septentrio_gnss_driver` [@dome2026] | 1 / 2 | SBF | N | N |
| `novatel_oem7_driver` [@novatel2026] | 1 / 2 | OEM7 | N | N |
| `ublox` [@meyer2025] | 1 / 2 | UBX | N | N |
| `ublox_dgnss` [@hortovanyi2026] | 2 | UBX | N | N |
| `ublox_driver` & `gnss_comm` [@cao2024; @cao2021] | 1 | UBX | N | N |
| `novatel_gps_driver` [@reed2026] | 1 / 2 | OEM4/6/7 | N | N |
| `gici-open` [@chi2023] | 1 | RTCM2/3, UBX, SBF, NovAtel, Tersus | N | Y |
| `gnss_ros_standardization` | 2 | RTCM3, UBX, SBF, OEM4/6/7 | Y | Y |

# Software design

`gnss_ros_standardization` is implemented in C++ and tested through continuous integration on ROS 2 Humble and Jazzy. It builds on RTKLIB-EX [@rtklibex2026], an actively maintained fork of RTKLIB, whose last official release dates from 2020 [@takasu2020rtklib]. Optional baseline implementations of factor graph optimization (FGO)-based positioning, including GNSS/IMU estimation with IMU preintegration [@forster2015], require GTSAM [@dellaert2012; @gtsam2026].

As shown in \autoref{fig:architecture}, the architecture separates receiver-specific data processing from downstream applications through common ROS 2 messages. Decoder nodes read receiver data streams, live or recorded, without sending configuration commands, whereas driver nodes also configure receiver outputs at startup. RINEX files are converted into ROS 2 bags containing the same messages, enabling recorded and live observations to share the same processing pipelines. New receiver formats can therefore be supported without modifying downstream algorithms.

![Architecture of `gnss_ros_standardization`. \label{fig:architecture}](architecture.png)

Three design choices shape the framework:

**Building on RTKLIB:** Rather than defining independent GNSS data structures and decoders, the framework aligns ROS 2 messages for observations, ephemerides, and solutions with RTKLIB and reuses RTKLIB-EX decoding functions. This facilitates integration with RTKLIB-based positioning algorithms and reduces the need to maintain separate decoders, but inherits RTKLIB's satellite and signal identifiers and supported protocols. Necessary fixes and extensions are applied as build-time patches to a staged copy of the source, preserving the upstream submodule while requiring patch maintenance across upgrades.

**Recorded data as live streams:** The converter writes RINEX observations and ephemerides as streams on the live timeline, placing each ephemeris at the transmission time recorded in RINEX, so that positioning nodes can use an ephemeris only after it has been broadcast, as in live operation. This lets offline evaluations follow the same causality as real-time processing, at the cost of relying on these recorded times, which mark transmission rather than reception and often come from a receiver other than the rover. To accommodate this limitation, an option allows all ephemerides to be published before the first observation epoch.

**IMU data integration:** Tightly coupled GNSS/IMU estimation motivates support for GNSS receivers that output inertial measurements in proprietary binary formats. Unlike GNSS observation decoding, IMU decoding is implemented independently of RTKLIB-EX, and the measurements are published as standard ROS 2 `sensor_msgs/msg/Imu` messages, which can also be time-stamped from GNSS time in the same way as the observations. This allows existing sensor fusion software to use them without receiver-specific interfaces, at the cost of a separate decoder for each supported IMU format.

# Research impact statement

The authors use `gnss_ros_standardization` in GNSS positioning research. Its converter was used to release PPC-Dataset-ROS2 [@niimi2026ppcros2], a ROS 2 version of the six vehicle runs of the PPC-Dataset [@suzuki2025ppc] recorded in Nagoya and Tokyo, Japan. Each bag contains rover and base station observations, ephemerides, IMU measurements, and the reference trajectory, using this framework's messages for GNSS data and standard ROS 2 messages for the IMU and reference attitude. All 70,380 epochs and 8,345,592 signal observations, as well as every published ephemeris, IMU, and reference record, were verified against the source files with tools included in this software's repository, so the dataset can be replayed directly through ROS 2 pipelines.

The dataset repository also provides results of the RTK, GNSS-only FGO, and GNSS/IMU FGO examples on all six runs, together with those of the RTKLIB-EX post-processing program `rnx2rtkp` under the same settings, scored against the reference trajectory by fix rate, miss-fix rate, and 3D error. Because the solutions, scoring scripts, and all settings are published, new methods can be compared with these baselines under identical conditions.

A paper describing the framework was presented at the 1st Workshop on Robot Meets GNSS and Ranging for Seamless Autonomy at ICRA 2026 [@niimi2026rmr], where it received the workshop's Best Presentation Award.

# AI usage disclosure

The concept and architecture of the software were developed by the human authors, and the initial prototype was implemented by the first author. Generative AI, primarily Claude Opus 4.7 to 5.5 (Anthropic, via Claude Code) and GPT-5.5 to 6.1 (OpenAI, via Codex), was then used to revise the code, implement fixes and improvements, and draft the documentation, tests, and continuous integration configuration. Before the repository was made public in May 2026, the first author validated the code with physical u-blox, Septentrio, and NovAtel receivers and data collected outdoors; later changes were validated with physical receivers, public datasets, automated tests, and comparisons with RTKLIB and receiver-vendor tools. Claude Opus 5.5 was used for proofreading and suggesting revised wording for this paper. The human authors reviewed, edited, and validated all AI-assisted outputs, made all core design decisions, and take full responsibility for the software and this paper.

# Author contributions

Daiki Niimi developed the detailed design of the software, implemented and maintains it, validated it with physical receivers and public datasets, and led the drafting of this paper. Taro Suzuki conceived the basic design and architecture of the software, provided measurement equipment and data used to validate it, tested the software and provided feedback, and directed the revision of this paper. Junichi Meguro advised on development planning, the required functionality, and ephemeris handling, and provided measurement equipment and data used to validate the software. Yoshiharu Amano advised on the conformance of the software to communication protocol specifications and on its practical applications, and provided measurement equipment used to validate the software. All authors reviewed and approved the final manuscript.

# Acknowledgements

This work received no specific funding. We acknowledge RTKLIB [@takasu2009; @takasu2020rtklib] and its RTKLIB-EX fork maintained by Tim Everett [@rtklibex2026], and GTSAM [@dellaert2012; @gtsam2026], on which this package builds. The FGO baselines are derived in part from `tightly-coupled-gnss-imu-fgo` [@inuex35fgo].

# References
