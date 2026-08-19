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

## Parameters

| Key | Default | Meaning |
|---|---|---|
| `topics.gnss_solution` / `topics.imu_raw` / `topics.wheel_speed` / `topics.solution` | see YAML | inputs and output |
| `frames.world` / `frames.child` | see YAML | TF frame names on the published Odometry |
| `output_reference_frame` | see YAML | frame the solution is expressed in |
| `local_origin.mode` / `local_origin.pos` | see YAML | ENU origin: first fix or a fixed coordinate |
| `lever_arm` | `[0,0,0]` | IMU body (FRD) to GNSS antenna, metres |
| `imu_orientation` | see YAML | IMU mounting convention |
| **Process noise** | | |
| `ekf.sigma_acc` / `ekf.sigma_gyr` | 0.3 / 0.01 | accelerometer / gyroscope noise density, per axis |
| `ekf.sigma_acc_bias` / `ekf.sigma_gyr_bias` | 1e-4 / 1e-5 | bias random walk, per axis |
| **Initial uncertainty** | | |
| `ekf.init_pos_std` | `[5,5,10]` | initial position sigma, metres |
| `ekf.init_vel_std` | `[0.3,0.3,0.3]` | initial velocity sigma, m/s |
| `ekf.init_att_std` | `[0.1,0.1,pi]` | initial roll/pitch/yaw sigma, rad (yaw unknown) |
| `ekf.init_acc_bias_std` / `ekf.init_gyr_bias_std` | 0.1 / 0.01 | initial bias sigma, per axis |
| `init_imu_duration` | 1.0 | stationary IMU averaging at startup, seconds |
| `use_init_yaw` / `init_yaw_deg` | `false` / 0.0 | seed the yaw instead of leaving it unknown |
| **GNSS update** | | |
| `gnss_update_mode` | `fix_only` | which solution statuses are used |
| `gnss.pos_cov_source` / `gnss.vel_cov_source` | see YAML | take covariance from the message or from the defaults below |
| `gnss.pos_sigma_default_fix` / `gnss.pos_sigma_default_float` / `gnss.pos_sigma_default_other` | see YAML | fallback position sigma by solution status, metres |
| `gnss.vel_sigma_default` | 0.2 | fallback velocity sigma, m/s |
| `gnss.cov_min_var` | 1e-6 | floor applied to any supplied covariance, m² |
| `use_doppler_heading` | `false` | derive heading from the GNSS velocity |
| `gnss_heading_speed_threshold` | 0.5 | speed above which that heading is trusted, m/s |
| **Time alignment** | | |
| `ekf.time_align_to_gnss` | see YAML | replay IMU to the GNSS epoch before updating |
| `ekf.gnss_time_source` | see YAML | `header` or `tow_auto_offset` |
| `ekf.gnss_offset_window_s` | 60.0 | sliding window of the latency estimate, seconds |
| `ekf.imu_buffer_duration` | 2.0 | IMU history kept for out-of-sequence replay, seconds |
| **Wheel speed** | | |
| `use_wheel_speed` | see YAML | use the wheel-speed topic |
| `wheel_speed_topic_type` | `twist_with_covariance` | message type on that topic |
| `wheel_speed_mode` | `longitudinal_only` | how the measurement is applied |
| `wheel_speed_sigma` | 0.1 | wheel-speed sigma, m/s |
| `wheel_nhc_enable` | `false` | non-holonomic constraint alongside wheel speed |
| `wheel_nhc_sigma_lateral` / `wheel_nhc_sigma_vertical` | 0.3 / 0.3 | NHC sigma, m/s |
| **Leveling and ZUPT** | | |
| `leveling.enable` | `true` | roll/pitch from the accelerometer while quasi-static |
| `leveling.window` | 0.3 | averaging window, seconds |
| `leveling.sigma_min_deg` | 1.0 | floor on the leveling sigma, degrees |
| `leveling.acc_gain` / `leveling.gyr_gain` | 15.0 / 30.0 | how fast the sigma grows with motion |
| `leveling.max_acc` | 0.5 | specific-force deviation above which leveling stops, m/s² |
| `zupt.enable` | `true` | zero-velocity update at standstill |
| `zupt.window` | 0.5 | stillness detector window, seconds |
| `zupt.acc_std_thresh` / `zupt.gyr_std_thresh` | 0.15 / 0.02 | stillness thresholds, m/s² and rad/s |
| `zupt.acc_mean_thresh` / `zupt.gyr_mean_thresh` | 0.5 / 0.05 | stillness mean thresholds |
| `zupt.sigma` | 0.05 | ZUPT velocity sigma, m/s |
| `zupt.min_interval` | 0.2 | minimum spacing between ZUPTs, seconds |
| `zupt.speed_thresh` / `zupt.speed_timeout` | 0.5 / 1.5 | external speed evidence gate, m/s and seconds |
| `zupt.allow_imu_only` | `false` | allow ZUPT with no external speed evidence |
| **Debug** | | |
| `csv.dir` | see YAML | directory for the CSV logs |
| `csv.state_log_enabled` / `csv.state_log_filename` | see YAML | per-epoch state log; grows without bound, leave disabled in production |
| `csv.sensors_log_enabled` / `csv.sensors_log_filename` | see YAML | per-sample sensor log; grows without bound, leave disabled in production |
