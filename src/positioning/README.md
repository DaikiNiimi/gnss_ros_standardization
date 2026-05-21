# Positioning

Example online positioning nodes that subscribe to the standardized GNSS topics
(`/gnss/observation`, `/gnss/ephemeris`, `/gnss/imu/data_raw`, ...) and publishes
`/gnss/solution` (`GnssSolution`). 

| Algorithm | Source | Sample config |
|---|---|---|
| Single-Point Positioning | [`single_point_positioning.cpp`](single_point_positioning.cpp) | [`config/single_point_positioning.yaml`](../../config/single_point_positioning.yaml) |
| Real-Time Kinematic | [`real_time_kinematic.cpp`](real_time_kinematic.cpp) | [`config/real_time_kinematic.yaml`](../../config/real_time_kinematic.yaml) |
| Loose-coupled GNSS/IMU EKF | [`gnss_imu_kalman_filter.cpp`](gnss_imu_kalman_filter.cpp) | [`config/gnss_imu_kalman_filter.yaml`](../../config/gnss_imu_kalman_filter.yaml) |

---

## `single_point_positioning`

Standalone GNSS pseudorange positioning.

```bash
ros2 run gnss_ros_standardization single_point_positioning --ros-args \
  --params-file config/single_point_positioning.yaml
```

### Topics

| Direction | Default | Type |
|---|---|---|
| Sub | `topics.observation` (`/gnss/observation`) | `GnssObservations` |
| Sub | `topics.ephemeris` (`/gnss/ephemeris`) | `GnssEphemerides` |
| Pub | `topics.solution` (`/gnss/solution`) | `GnssSolution` |

---

## `real_time_kinematic`

RTK positioning using rover and base observations with broadcast ephemeris.

```bash
ros2 run gnss_ros_standardization real_time_kinematic --ros-args \
  --params-file config/real_time_kinematic.yaml
```

### Topics

| Direction | Default | Type |
|---|---|---|
| Sub | `topics.rover_observation` (`/gnss/observation`) | `GnssObservations` |
| Sub | `topics.base_observation` (`/base/gnss/observation`) | `GnssObservations` |
| Sub | `topics.ephemeris` (`/gnss/ephemeris`) | `GnssEphemerides` |
| Pub | `topics.solution` (`/gnss/solution`) | `GnssSolution` |

---

## `gnss_imu_kalman_filter`

Loosely-coupled GNSS/IMU extended Kalman filter using gnss solution, imu raw data and optional wheel speed sensor data.

```bash
ros2 run gnss_ros_standardization gnss_imu_kalman_filter --ros-args \
  --params-file config/gnss_imu_kalman_filter.yaml
```

### Topics

| Direction | Default | Type |
|---|---|---|
| Sub | `topics.gnss_solution` (`/gnss/solution`) | `GnssSolution` (from SPP / RTK; remap to `/gnss/nmea_solution` to use the receiver-side solution instead) |
| Sub | `topics.imu_raw` (`/gnss/imu/data_raw`) | `sensor_msgs/Imu`|
| Sub | `topics.wheel_speed` (`/can_twist`) | `geometry_msgs/TwistWithCovarianceStamped` (when `use_wheel_speed: true`) |
| Pub | `topics.solution` (`/gnss/fusion/ekf_solution`) | `GnssSolution` |
