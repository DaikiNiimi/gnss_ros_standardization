# `gnss_imu_kalman_filter`

Loosely-coupled GNSS/IMU extended Kalman filter: a 15-state error-state EKF
(position, velocity, attitude, accel/gyro bias) in a local ENU frame that fuses a
`GnssSolution` (from SPP / RTK or the receiver) with raw IMU, and optionally wheel
speed, ZUPT, and accelerometer leveling to stay bounded through GNSS gaps. Late
GNSS solutions are applied at their true epoch via store-and-rewind time
alignment, and the published output stays strictly causal.

```bash
ros2 run gnss_ros_standardization gnss_imu_kalman_filter --ros-args \
  --params-file config/gnss_imu_kalman_filter.yaml
```

## Topics

| Direction | Default | Type |
|---|---|---|
| Sub | `topics.gnss_solution` (`/gnss/solution`) | `GnssSolution` (remap to `/gnss/nmea_solution` for the receiver-side solution) |
| Sub | `topics.imu_raw` (`/gnss/imu/data_raw`) | `sensor_msgs/Imu` |
| Sub | `topics.wheel_speed` (`/can_twist`) | `geometry_msgs/TwistWithCovarianceStamped` (when `use_wheel_speed: true`) |
| Pub | `topics.solution` (`/gnss/fusion/ekf_solution`) | `GnssSolution` |

For best accuracy drive it with a 10 Hz RTK stream from
[`real_time_kinematic`](../real_time_kinematic/). Update modes (`fix_only` /
`fix_float` / `all`) and all filter tuning live in
[the config](../../config/gnss_imu_kalman_filter.yaml).
