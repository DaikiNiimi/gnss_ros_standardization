# Positioning

Online positioning nodes that consume the standardized GNSS topics
(`/gnss/observation`, `/gnss/ephemeris`, `/gnss/imu/data_raw`, ...) and publish
`/gnss/solution` (`GnssSolution`). All algorithms run on top of the embedded
RTKLIB.

| Node | Executable | Source | Sample config |
|---|---|---|---|
| Single-Point Positioning | `single_point_positioning` | [`single_point_positioning.cpp`](single_point_positioning.cpp) | [`config/single_point_positioning.yaml`](../../config/single_point_positioning.yaml) |
| Real-Time Kinematic | `real_time_kinematic` | [`real_time_kinematic.cpp`](real_time_kinematic.cpp) | [`config/real_time_kinematic.yaml`](../../config/real_time_kinematic.yaml) |
| Loose-coupled GNSS/IMU EKF | `gnss_imu_kalman_filter` | [`gnss_imu_kalman_filter.cpp`](gnss_imu_kalman_filter.cpp) | [`config/gnss_imu_kalman_filter.yaml`](../../config/gnss_imu_kalman_filter.yaml) |

For the message definitions, see [`../../msg/README.md`](../../msg/README.md).

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

### Key parameters

| Parameter | Description |
|---|---|
| `nav_systems.{gps,glo,gal,bds,qzs,irn,sbs}` | Enable per-constellation |
| `frequencies.{enable_l1,enable_l2,enable_l5}` | Frequency selection |
| `elevation_mask_deg` | Minimum elevation [deg] |
| `snrmask.{enable,l1,l2,l5}` | SNR mask per band |

See [`config/single_point_positioning.yaml`](../../config/single_point_positioning.yaml) for the full set.

---

## `real_time_kinematic`

Differential RTK (rover + base observations + broadcast ephemeris). Base
observations are accepted on the ROS topic `topics.base_observation`. The
`tcp_port` setting exposes an outbound NMEA stream (GGA / RMC) for tools
such as RTKPLOT.

```bash
ros2 run gnss_ros_standardization real_time_kinematic --ros-args \
  --params-file config/real_time_kinematic.yaml
```

### Topics

| Direction | Default | Type |
|---|---|---|
| Sub | `topics.rover_observation` | `GnssObservations` |
| Sub | `topics.base_observation` | `GnssObservations` |
| Sub | `topics.ephemeris` | `GnssEphemerides` |
| Pub | `topics.solution` (`/gnss/solution`) | `GnssSolution` |

### Key parameter groups

| Group | Purpose |
|---|---|
| `frequencies.{enable_l1,enable_l2,enable_l5}` | Frequency selection (shared style with SPP node) |
| `pos1.*` | Elevation mask / dynamics / iono / tropo / sat-eph / nav-sys / SNR mask |
| `pos2.*` | AR mode and thresholds (`armode`, `arthres`, `arlockcnt`, `arelmask`, ...) |
| `rtk_stats.*` | Process / observation noise model (`eratio*`, `prn*`, `std*`) |
| `ant2.{postype,pos}` | Base antenna position (`llh`, `xyz`, ...) |
| `fixed_origin.{postype,pos}` | Fixed ENU local origin (optional) |
| `excluded_satellites` | Satellite IDs to exclude (e.g. `["G05", "E11"]`) |
| `tcp_port` | TCP port for outbound NMEA stream (GGA / RMC) for RTKPLOT-style clients |

See [`config/real_time_kinematic.yaml`](../../config/real_time_kinematic.yaml) for all keys (mirrors RTKLIB `rtk.conf`).

---

## `gnss_imu_kalman_filter`

Loose-coupled GNSS/IMU extended Kalman filter. Fuses `/gnss/solution` (from
SPP or RTK) with `/gnss/imu/data_raw` (and optionally wheel speed) to produce a
higher-rate solution. GNSS / wheel-speed observations are time-aligned to the
state by re-integrating buffered IMU samples up to the measurement timestamp
(`ekf.time_align_to_gnss`, `ekf.imu_buffer_duration`).

```bash
ros2 run gnss_ros_standardization gnss_imu_kalman_filter --ros-args \
  --params-file config/gnss_imu_kalman_filter.yaml
```

### Topics

| Direction | Default | Type |
|---|---|---|
| Sub | `topics.gnss_solution` | `GnssSolution` (e.g. from SPP / RTK) |
| Sub | `topics.imu_raw` | `sensor_msgs/Imu` |
| Sub | `topics.wheel_speed` | wheel speed (when `use_wheel_speed: true`) |
| Pub | `topics.solution` | `GnssSolution` |

### Key parameters

| Parameter | Description |
|---|---|
| `coordinate_frame` | Output frame (`enu` / `ecef`) |
| `local_origin.{mode,pos}` | Local ENU origin: `auto` or fixed `[lat, lon, alt]` |
| `ekf.sigma_{acc,gyr,acc_bias,gyr_bias}` | Process noise σ |
| `ekf.init_{pos,vel,att,acc_bias,gyr_bias}_std` | Initial covariance σ |
| `ekf.time_align_to_gnss`, `ekf.imu_buffer_duration` | IMU-buffer time alignment of observations |
| `gnss_update_mode` | `fix_only` / `fix_and_float` / `all` |
| `gnss_heading_speed_threshold` | Min speed [m/s] for GNSS-velocity-derived heading |
| `use_wheel_speed`, `wheel_speed_{topic_type,mode,sigma}` | Wheel-speed update |
| `init_imu_duration`, `init_yaw_deg` | Static-init duration + optional yaw seed |
| `output_reference_frame` | Output frame_id |
| `lever_arm` | IMU → GNSS antenna lever arm `[x, y, z]` (body frame) [m] |
| `imu_orientation` | IMU mounting Euler `[roll, pitch, yaw]` [deg] |
| `csv.output_path` | Optional CSV log of full state |

See [`config/gnss_imu_kalman_filter.yaml`](../../config/gnss_imu_kalman_filter.yaml) for the full schema.
