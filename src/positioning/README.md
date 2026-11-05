# Positioning

Positioning nodes that read the common GNSS topics, from any supported receiver,
live or from a bag. The [pipelines](../../launch/README.md) start them together with
the receiver nodes.

| Executable | Method | Inputs | Output | Guide |
|---|---|---|---|---|
| `single_point_positioning` | SPP (RTKLIB `pntpos`) | rover observations, ephemeris | `/gnss/solution` | [spp](spp/README.md) |
| `real_time_kinematic` | RTK (RTKLIB `rtkpos`) | rover and base observations, ephemeris | `/gnss/solution` | [rtk](rtk/README.md) |
| `gnss_imu_kalman_filter` | error-state EKF, loosely coupled | GNSS solution, IMU, wheel speed | `/gnss/fusion/ekf_solution` | [ekf](ekf/README.md) |
| `gnss_fgo` | factor graph, tightly coupled GNSS | rover and base observations, ephemeris | `/gnss/fgo/solution` | [fgo](fgo/README.md) |
| `gnss_imu_fgo` | factor graph, tightly coupled GNSS/IMU | rover and base observations, ephemeris, IMU | `/gnss/imu_fgo/solution` | [fgo](fgo/README.md) |

Configuration files are in [config/](../../config/); [config/ppc/](../../config/ppc/)
holds the settings for the PPC dataset routes.

All nodes publish `GnssSolution` with the rules in the
[topic contract](../../docs/TOPIC_DATA_CONTRACT.md), and set the ENU origin with
`local_origin.*` ([parameter names](../../docs/PARAMETER_NAMES.md#enu-origin)). A
rover, base or IMU clock that steps back more than 5 s (a bag loop, a receiver
restart) starts a new time line; no restart of the node is needed.
