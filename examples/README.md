# Examples

Positioning and sensor-fusion examples built on top of the standardized GNSS
topics published by this package (`/gnss/observation`, `/gnss/ephemeris`,
`/gnss/imu/data_raw`, ...). Every example is receiver-agnostic: it works with
any of the supported decoders/drivers (u-blox, Septentrio, NovAtel, RTCM3) and
equally with live streams or rosbag playback.

| Example | Method | Coupling | Extra dependency | README |
|---|---|---|---|---|
| [`single_point_positioning`](single_point_positioning/) | SPP (RTKLIB `pntpos`) | — | — | [README](single_point_positioning/README.md) |
| [`real_time_kinematic`](real_time_kinematic/) | RTK (RTKLIB `rtkpos`) | — | — | [README](real_time_kinematic/README.md) |
| [`gnss_imu_kalman_filter`](gnss_imu_kalman_filter/) | Error-state EKF | Loose (GNSS solution + IMU) | — | [README](gnss_imu_kalman_filter/README.md) |

Example configs live in the package-level [config/](../config/) directory
(installed to `share/gnss_ros_standardization/config/`), alongside the driver
configs.

## Inputs and outputs

| Direction | Topic (default) | Type | Used by |
|---|---|---|---|
| Sub | `/gnss/observation` | `GnssObservations` | all examples |
| Sub | `/base/gnss/observation` | `GnssObservations` | RTK |
| Sub | `/gnss/ephemeris` | `GnssEphemerides` | all examples |
| Sub | `/gnss/imu/data_raw` | `sensor_msgs/Imu` | EKF |
| Pub | `/gnss/solution` (or example-specific) | `GnssSolution` | all examples |
