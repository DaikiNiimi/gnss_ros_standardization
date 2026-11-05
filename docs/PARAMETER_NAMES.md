# Parameter naming

Every node follows the same naming rules, in code, YAML and launch. There are no
aliases: a key that a node does not declare is ignored by ROS 2, so update older
local parameter files with the table below. `test/test_parameter_names.py` checks
that every key of the shipped YAML files and every parameter the launch files pass
is declared by its node.

## Rules

| Rule | Examples |
|---|---|
| Topics are `topics.<stream>` | `topics.observation`, `topics.ephemeris`, `topics.solution`, `topics.imu`, `topics.imu_raw`, `topics.rover_observation`, `topics.base_observation`, `topics.gnss_solution`, `topics.wheel_speed`, `topics.image` |
| A physical quantity carries its unit | `_s`, `_hz`, `_deg`, `_m`, `_mps`, `_mps2`, `_rps` (rad/s): `rate_hz`, `zupt.window_s`, `wheel.sigma_mps` |
| A feature switch is `<group>.enable` | `zupt.enable`, `leveling.enable`, `wheel.enable`, `wheel.nhc.enable`, `rtcm_input.enable`, `snrmask.enable` |
| Related settings share a group | `wheel.*`, `init.*`, `gnss.*`, `zupt.*`, `leveling.*`, `ekf.*` |

Kept on purpose:
- `real_time_kinematic` uses RTKLIB's configuration names (`pos1.*`, `pos2.*`,
  `rtk_stats.*`, `ant2.*`), so an RTKLIB setting maps one to one.
- Receiver drivers name their output messages after the receiver protocol (`messages.*`).
- The FGO nodes describe the receiver signals as `signals.<rover|base>.<system>.<P|L|D|S>`.

## Renamed before the first release

| Node(s) | Former name | Current name |
|---|---|---|
| decoders, drivers | `observation_topic`, `ephemeris_topic`, `solution_topic`, `imu_topic`, `imu_raw_topic` | `topics.observation`, `topics.ephemeris`, `topics.solution`, `topics.imu`, `topics.imu_raw` |
| `gnss_visualizer` | `obs_topic`, `nav_topic`, `sol_topic`, `image_topic` | `topics.observation`, `topics.ephemeris`, `topics.solution`, `topics.image` |
| `gnss_visualizer` | `recent_window_sec`, `decimate_old_dt_sec` | `recent_window_s`, `decimate_old_dt_s` |
| `gnss_visualizer` | `fixed_latitude`, `fixed_longitude`, `fixed_altitude` | `manual_position.pos` (+ `manual_position.postype`) |
| `sbf_driver_node`, `novatel_driver_node` | `publish_rate` | `rate_hz` (as `ubx_driver_node`) |
| drivers | `rtcm_input.enabled` | `rtcm_input.enable` |
| `gnss_imu_kalman_filter` | `ekf.imu_buffer_duration`, `init.imu_duration`, `leveling.window` | `ekf.imu_buffer_duration_s`, `init.imu_duration_s`, `leveling.window_s` |
| `gnss_imu_kalman_filter` | `zupt.window`, `zupt.min_interval`, `zupt.speed_timeout` | `zupt.window_s`, `zupt.min_interval_s`, `zupt.speed_timeout_s` |
| `gnss_imu_kalman_filter` | `zupt.gyr_mean_thresh`, `zupt.acc_mean_thresh` | `zupt.max_gyr_mean_rps`, `zupt.max_acc_mean_mps2` |
| `gnss_imu_kalman_filter` | `use_wheel_speed`, `wheel_speed_topic_type`, `wheel_speed_mode`, `wheel_speed_sigma` | `wheel.enable`, `wheel.topic_type`, `wheel.mode`, `wheel.sigma_mps` |
| `gnss_imu_kalman_filter` | `wheel_nhc_enable`, `wheel_nhc_sigma_lateral`, `wheel_nhc_sigma_vertical` | `wheel.nhc.enable`, `wheel.nhc.sigma_lat_mps`, `wheel.nhc.sigma_vert_mps` |
| `gnss_imu_kalman_filter` | `use_init_yaw`, `init_yaw_deg` | `init.use_yaw`, `init.yaw_deg` |
| `gnss_imu_kalman_filter` | `gnss_update_mode`, `gnss_heading_speed_threshold`, `use_doppler_heading` | `gnss.update_mode`, `gnss.heading_min_speed_mps`, `gnss.use_doppler_heading` |
| decoders, drivers | `auto_origin`, `origin` | `local_origin.mode`, `local_origin.pos` (+ `local_origin.postype`) |
| `single_point_positioning`, `real_time_kinematic` | `fixed_origin.pos`, `fixed_origin.postype` (all-zero = automatic) | `local_origin.mode`, `local_origin.pos`, `local_origin.postype` |
| `gnss_imu_kalman_filter` | `local_origin.mode: gnss_fix / manual` | `local_origin.mode: auto / fixed` (+ `local_origin.postype`) |

CMake options were renamed at the same time: `BUILD_SPP_EXAMPLE` → `BUILD_SPP_NODE`,
`BUILD_RTK_EXAMPLE` → `BUILD_RTK_NODE`, `BUILD_GNSS_IMU_KALMAN_FILTER` → `BUILD_EKF_NODE`,
`BUILD_GTSAM_FGO_EXAMPLES` → `BUILD_FGO_NODES`.

## ENU origin

Every node that publishes a local ENU position (`pos_enu`, `pos_enu_org_ecef`) sets
its origin with the same three parameters, declared by one shared function:

| Parameter | Default | Meaning |
|---|---|---|
| `local_origin.mode` | `auto` | `auto`: the node's automatic origin (below); `fixed`: `local_origin.pos` |
| `local_origin.pos` | `[0, 0, 0]` | `[lat_deg, lon_deg, h_m]` for `llh`, `[x, y, z]` m for `ecef` |
| `local_origin.postype` | `llh` | `llh` or `ecef` |

The automatic origin is the first valid receiver solution (decoders, drivers), the
first valid solution (`single_point_positioning`), the base station position
(`real_time_kinematic`, `gnss_fgo`, `gnss_imu_fgo`; for FGO also the origin of the
`_odom` pose) or the first usable GNSS solution (`gnss_imu_kalman_filter`).
A `pos` other than `[0, 0, 0]` with mode `auto` is rejected at startup, so a position
is never ignored silently. With mode `fixed`, `[0, 0, 0]` in `llh` is the point
0°N 0°E at height 0; in `ecef` it is rejected.

## Shared meanings

- `imu_orientation` is RPY in radians for the IMU-sensor to vehicle-body rotation
  in both `gnss_imu_kalman_filter` and `gnss_imu_fgo`.
- Rates and base reuse are derived from observed timestamps, not from a nominal
  frequency parameter.
- Defaults need not match between estimators; units and meanings do.
