# `gnss_imu_kalman_filter`

Loosely-coupled GNSS/IMU extended Kalman filter using a GNSS solution, raw IMU
data, and optional wheel-speed sensor data.

It is a 15-state error-state Kalman filter (position, velocity, attitude,
accelerometer bias, gyro bias) in a **local ENU navigation frame**. The attitude
error is a **local (body-frame) multiplicative error** with right-quaternion
injection, following J. Solà, *Quaternion kinematics for the error-state Kalman
filter* (2017, [arXiv:1711.02508](https://arxiv.org/abs/1711.02508)) — see the
convention block at the top of
[`gnss_imu_kalman_filter.cpp`](gnss_imu_kalman_filter.cpp), which all Jacobians
follow consistently.

Measurement updates (each optional/gated, all fused through a shared
Joseph-form update with an ESKF attitude-covariance reset):

- **GNSS position** (+ lever arm) and **GNSS velocity** from the `GnssSolution`.
- **GNSS Doppler heading** above a configurable speed threshold.
- **Accelerometer leveling** — at quasi-static instants the measured
  specific-force direction observes roll/pitch directly. This keeps attitude
  bounded on slow platforms and through GNSS gaps, where GNSS-derived attitude
  observability alone is too weak.
- **ZUPT** (zero-velocity update) — IMU-variance stationarity detection plus an
  estimated-speed gate; pins velocity while standing still.
- **Wheel speed** (longitudinal or 3D, optional NHC).

GNSS solutions arrive tens–hundreds of ms after their measurement epoch (RTK
matching + solve time), so observation updates are applied at the correct epoch
via **store-and-rewind time alignment**: the filter rewinds its state/covariance
to the observation timestamp, updates, and re-propagates buffered IMU samples.
The published output remains strictly causal (past outputs are never rewritten).

```bash
ros2 run gnss_ros_standardization gnss_imu_kalman_filter --ros-args \
  --params-file config/gnss_imu_kalman_filter.yaml
```

## Topics

| Direction | Default | Type |
|---|---|---|
| Sub | `topics.gnss_solution` (`/gnss/solution`) | `GnssSolution` (from SPP / RTK; remap to `/gnss/nmea_solution` to use the receiver-side solution instead) |
| Sub | `topics.imu_raw` (`/gnss/imu/data_raw`) | `sensor_msgs/Imu` |
| Sub | `topics.wheel_speed` (`/can_twist`) | `geometry_msgs/TwistWithCovarianceStamped` (when `use_wheel_speed: true`) |
| Pub | `topics.solution` (`/gnss/fusion/ekf_solution`) | `GnssSolution` |

## GNSS aiding

The EKF consumes a `GnssSolution`, so it is agnostic to how that solution was
produced. For the best accuracy, drive it with an **RTK** stream from the
[`real_time_kinematic`](../real_time_kinematic/) example (10 Hz solutions give
5× the aiding density of a typical 2 Hz receiver NMEA stream). The example
config uses `gnss_update_mode: fix_float` so FLOAT solutions bridge the gaps
while ambiguities are being resolved; set `fix_only` to apply Fix solutions
only:

```
rover/base obs + ephemeris ──▶ real_time_kinematic ──▶ /gnss/solution (RTK Fix)
                                                              │
                          /gnss/imu/data_raw ────────────────┼──▶ gnss_imu_kalman_filter
                                                              │         │
                                                              ▼         ▼
                                                   /gnss/fusion/ekf_solution
```

To replay a rosbag that carries raw observations (e.g. the `20260624_toyama_park`
bags), run the two nodes and play the bag:

```bash
# terminal 1 — RTK, produces /gnss/solution
ros2 run gnss_ros_standardization real_time_kinematic --ros-args \
  --params-file config/real_time_kinematic.yaml

# terminal 2 — EKF
ros2 run gnss_ros_standardization gnss_imu_kalman_filter --ros-args \
  --params-file config/gnss_imu_kalman_filter.yaml

# terminal 3 — playback
ros2 bag play 20260624_toyama_park/rosbag2_2026_06_24-18_00_21
```

For position/velocity update modes (`fix_only` / `fix_float` / `all`) see
`gnss_update_mode` in the config.

## Inspecting accuracy

With `csv.sensors_log_enabled` / `csv.state_log_enabled` (both on by default) the
node writes `ekf_sensors_log.csv` (time-synchronized raw IMU/GNSS/wheel at IMU
rate) and `ekf_state_log.csv` (estimated state + diagonal covariance). Comparing
`ekf_*` against the `gnss_*` columns is the quickest way to check that the fused
velocity tracks the GNSS Doppler and the trajectory does not zigzag between
fixes.
