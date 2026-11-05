# `gnss_imu_kalman_filter`

Loosely coupled GNSS/IMU error-state EKF (position, velocity, attitude, accelerometer
and gyro bias in local ENU). It fuses a `GnssSolution` (RTK, SPP or the receiver's own
solution) with raw IMU, and optionally wheel speed, ZUPT and accelerometer leveling.
GNSS solutions that arrive late are applied at their own epoch.

```bash
ros2 run gnss_ros_standardization gnss_imu_kalman_filter --ros-args \
  --params-file config/gnss_imu_kalman_filter.yaml
```

## Topics

| Direction | Default | Type |
|---|---|---|
| Sub | `topics.gnss_solution` (`/gnss/solution`) | `GnssSolution` (remap to `/gnss/nmea_solution` for the receiver-side solution) |
| Sub | `topics.imu_raw` (`/gnss/imu/data_raw`) | `sensor_msgs/Imu` |
| Sub | `topics.wheel_speed` (`/can_twist`) | `geometry_msgs/TwistWithCovarianceStamped` (when `wheel.enable: true`) |
| Pub | `topics.solution` (`/gnss/fusion/ekf_solution`) | `GnssSolution` |
| Pub | `<topics.solution>_odom` | `nav_msgs/Odometry` |

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `topics.gnss_solution` / `topics.imu_raw` / `topics.wheel_speed` / `topics.solution` | see YAML | inputs and output |
| `frames.world` / `frames.child` | see YAML | TF frame names on the published Odometry |
| `output_reference_frame` | see YAML | frame the solution is expressed in |
| `local_origin.mode` / `.postype` / `.pos` | `auto` / `llh` / `[0,0,0]` | ENU origin: `auto` = first usable GNSS solution, `fixed` = `pos` |
| `lever_arm` | `[0,0,0]` | body origin to GNSS antenna in IMU message axes after `imu_orientation`, metres; used only with `output_reference_frame=imu` |
| `imu_orientation` | `[0,0,0]` | IMU sensor to body rotation, roll/pitch/yaw in radians |
| **Process noise** | | |
| `ekf.sigma_acc` / `ekf.sigma_gyr` | 0.3 / 0.01 | accelerometer / gyroscope noise density, per axis |
| `ekf.sigma_acc_bias` / `ekf.sigma_gyr_bias` | 1e-4 / 1e-5 | bias random walk, per axis |
| **Initial uncertainty** | | |
| `ekf.init_pos_std` | `[5,5,10]` | initial position sigma, metres |
| `ekf.init_vel_std` | `[0.3,0.3,0.3]` | initial velocity sigma, m/s |
| `ekf.init_att_std` | `[0.1,0.1,pi]` | initial roll/pitch/yaw sigma, rad (yaw unknown) |
| `ekf.init_acc_bias_std` / `ekf.init_gyr_bias_std` | 0.1 / 0.01 | initial bias sigma, per axis |
| `init.imu_duration_s` | 1.0 | stationary IMU averaging at startup, seconds |
| `init.use_yaw` / `init.yaw_deg` | `false` / 0.0 | seed the yaw instead of leaving it unknown |
| **GNSS update** | | |
| `gnss.update_mode` | `fix_only` | solution statuses used: `fix_only`, `fix_float`, `all` |
| `gnss.pos_cov_source` / `gnss.vel_cov_source` | see YAML | take covariance from the message or from the defaults below |
| `gnss.pos_sigma_default_fix` / `gnss.pos_sigma_default_float` / `gnss.pos_sigma_default_other` | see YAML | fallback position sigma by solution status, metres |
| `gnss.vel_sigma_default` | 0.2 | fallback velocity sigma, m/s |
| `gnss.cov_min_var` | 1e-6 | floor applied to any supplied covariance, m² |
| `gnss.use_doppler_heading` | `false` | derive heading from the GNSS velocity |
| `gnss.heading_min_speed_mps` | 0.5 | speed above which that heading is trusted, m/s |
| **Time alignment** | | |
| `ekf.time_align_to_gnss` | see YAML | replay IMU to the GNSS epoch before updating |
| `time.gnss_epoch_source` | `header` | GNSS epoch time: `header` (stamp) or `tow_auto_offset` (week/TOW plus an estimated arrival offset, for host-stamped input) |
| `ekf.gnss_offset_window_s` | 60.0 | sliding window of the latency estimate, seconds |
| `ekf.imu_buffer_duration_s` | 2.0 | IMU history kept for out-of-sequence replay, seconds |
| **Wheel speed** | | |
| `wheel.enable` | see YAML | use the wheel-speed topic |
| `wheel.topic_type` | `twist_with_covariance` | `twist_with_covariance` or `twist` |
| `wheel.mode` | `longitudinal_only` | `longitudinal_only` or `3d` |
| `wheel.sigma_mps` | 0.1 | wheel-speed sigma, m/s |
| `wheel.nhc.enable` | `false` | non-holonomic constraint alongside wheel speed |
| `wheel.nhc.sigma_lat_mps` / `wheel.nhc.sigma_vert_mps` | 0.3 / 0.3 | NHC sigma, m/s |
| **Leveling and ZUPT** | | |
| `leveling.enable` | `true` | roll/pitch from the accelerometer while quasi-static |
| `leveling.window_s` | 0.3 | averaging window, seconds |
| `leveling.sigma_min_deg` | 1.0 | floor on the leveling sigma, degrees |
| `leveling.acc_gain` / `leveling.gyr_gain` | 15.0 / 30.0 | how fast the sigma grows with motion |
| `leveling.max_acc` | 0.5 | specific-force deviation above which leveling stops, m/s² |
| `zupt.enable` | `true` | zero-velocity update at standstill |
| `zupt.window_s` | 0.5 | stillness detector window, seconds |
| `zupt.max_acc_std` / `zupt.max_gyr_std` | 0.15 / 0.02 | stillness thresholds, m/s² and rad/s |
| `zupt.max_acc_mean_mps2` / `zupt.max_gyr_mean_rps` | 0.5 / 0.05 | stillness mean thresholds |
| `zupt.sigma_mps` | 0.05 | ZUPT velocity sigma, m/s |
| `zupt.min_interval_s` | 0.2 | minimum spacing between ZUPTs, seconds |
| `zupt.max_speed_mps` / `zupt.speed_timeout_s` | 0.5 / 1.5 | external speed evidence gate, m/s and seconds |
| `zupt.allow_imu_only` | `false` | allow ZUPT with no external speed evidence |
| **Debug** | | |
| `csv.dir` | see YAML | directory for the CSV logs |
| `csv.state_log_enabled` / `csv.state_log_filename` | see YAML | per-epoch state CSV |
| `csv.sensors_log_enabled` / `csv.sensors_log_filename` | see YAML | per-sample sensor CSV |

The lever arm is given in the IMU message axes (after `imu_orientation`), the same
convention as `gnss_imu_fgo`.
