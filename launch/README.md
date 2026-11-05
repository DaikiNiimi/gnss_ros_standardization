# Pipelines

One launch file starts the receiver nodes, an estimator and the recorder. What runs
and on which hardware is set in [`config/site.yaml`](../config/site.yaml):

```bash
ros2 launch gnss_ros_standardization pipeline.launch.py
```

| `run.pipeline` | What runs | Output |
|---|---|---|
| [`logging`](#logging) | one or several receivers, a base and IMU, recorded to a bag | bag |
| [`rtk`](#rtk-and-gnss_fgo) | `real_time_kinematic` | `/gnss/solution` |
| [`gnss_fgo`](#rtk-and-gnss_fgo) | `gnss_fgo` (needs the FGO build) | `/gnss/solution` |
| [`gnss_imu_fgo`](#gnss_imu_fgo-and-ekf) | `gnss_imu_fgo` (needs the FGO build) | `/gnss/solution` |
| [`ekf`](#gnss_imu_fgo-and-ekf) | `gnss_imu_kalman_filter` on the RTK or receiver solution | `/gnss/fusion/ekf_solution` |
| [`onboard_rtk`](#on-chip-rtk-through-the-rtcm-relay) | the receiver's own RTK, corrections relayed from the base stream | `/gnss/nmea_solution` |

`Ctrl-C` stops every node and closes the bag.

## Site file

```yaml
run:
  pipeline: rtk
rover:
  receiver: ubx
  mode: decoder
  source: "serial:///dev/ttyACM0:115200"
base:
  receiver: rtcm
  source: "ntrip://${NTRIP_USER}:${NTRIP_PASSWORD}@caster.example.com:2101/MOUNT"
```

- `${VAR}` / `$VAR` are taken from the environment; an unset variable is an error.
- `config/site.local.yaml` (git-ignored) is used instead of `config/site.yaml` when
  it exists. Other setups go in `config/site_<name>.yaml` and are selected with
  `site:=<name>`.
- Precedence: command line > site file > estimator / driver YAML > node default.
  An empty value means "not set".
- On startup the launch prints every resolved setting and where it came from
  (NTRIP credentials masked), and refuses a configuration that cannot work.

## Receivers

`rover.mode` / `base.mode`: `decoder` reads the receiver as configured, `driver`
configures it from [`config/<receiver>_driver.yaml`](../config/).

| `receiver` | Executables | Example `source` |
|---|---|---|
| `ubx` | `ubx_decoder_node` / `ubx_driver_node` | `serial:///dev/ttyACM0:115200` |
| `sbf` | `sbf_decoder_node` / `sbf_driver_node` | `serial:///dev/ttyUSB0:115200` |
| `novatel` | `novatel_decoder_node` / `novatel_driver_node` | `serial:///dev/ttyUSB1:230400` |
| `rtcm` | `rtcm_decoder_node` (decoder only) | `ntrip://user:pass@caster:2101/MOUNT` |

Sources are stream URIs (`serial://`, `tcpcli://`, `tcpsvr://`, `ntrip://`, `file://`),
see [decoders](../src/decoders/README.md#stream-uri-stream_path). A `file://` source
replays a recorded receiver log.

## Topics

| Role | Observations | Ephemerides | Receiver solution | Raw IMU |
|---|---|---|---|---|
| Rover | `/gnss/observation` | `/gnss/ephemeris` | `/gnss/nmea_solution` | `/gnss/imu/data_raw` |
| Base | `/base/gnss/observation` | `/base/gnss/ephemeris` | `/base/gnss/nmea_solution` | — |

Base ephemerides are recorded but not used; `base.merge_ephemeris: true` publishes
them on `/gnss/ephemeris` (for a rover without navigation data). The estimator
output is `/gnss/solution` (EKF: `/gnss/fusion/ekf_solution`).

## Arguments

Each argument overrides one site key for a single run (`--show-args` lists them).

| Argument | Site key | Meaning |
|---|---|---|
| `site` | — | site file name |
| `pipeline` | `run.pipeline` | `logging`, `rtk`, `gnss_fgo`, `gnss_imu_fgo`, `ekf`, `onboard_rtk` |
| `ekf_solution_source` | `run.ekf_solution_source` | `rtk` (default) or `receiver` |
| `replay` | `run.replay` | start no receiver nodes; feed the estimator with `ros2 bag play` |
| `visualize` | `run.visualize` | start `gnss_visualizer` |
| `rover_receiver`, `rover_mode`, `rover_source`, `rover_params_file` | `rover.*` | rover receiver |
| `base_receiver`, `base_source` | `base.*` | base receiver (no base node when `source` is empty) |
| `base_reference_week`, `base_reference_tow` | `base.reference_week`, `base.reference_tow` | GPS week/TOW for a recorded RTCM base |
| `merge_base_ephemeris` | `base.merge_ephemeris` | publish base ephemerides on `/gnss/ephemeris` |
| `use_imu` | `imu.enabled` | publish `/gnss/imu/data_raw` |
| `use_gps_timestamp` | `imu.use_gps_timestamp` | GNSS time (UTC) header stamps |
| `params_file` | `estimator.params_file` | estimator YAML (default: the packaged one) |
| `base_position`, `base_position_type` | `estimator.base_position`, `estimator.base_position_type` | base position `"lat,lon,height"` (`llh`) or ECEF (`ecef`) |
| `lever_arm` | `estimator.lever_arm` | IMU to antenna, metres |
| `relay_port` | `relay.port` | on-chip RTK correction port (`0`: 5556 ubx, 5557 sbf, 5558 novatel) |
| `novatel_relay_output` | `relay.novatel_output` | NovAtel dedicated correction port device |
| `record` | `record.enabled` | record a bag |
| `bag_out` | `record.output` | bag path (default `gnss_<pipeline>_<time>`) |
| `bag_storage` | `record.storage` | `mcap` (default) or `sqlite3` |
| `bag_compress` | `record.compress` | zstd compression |

`imu.enabled` and `imu.use_gps_timestamp` default to on for `gnss_imu_fgo` and `ekf`;
`record.enabled` defaults to on for `logging`.

## Logging

```bash
ros2 launch gnss_ros_standardization pipeline.launch.py pipeline:=logging
ros2 launch gnss_ros_standardization pipeline.launch.py pipeline:=logging \
    rover_source:=file:///path/to/rover.ubx use_imu:=true
```

Recorded topics: rover observation, ephemeris and receiver solution; the base topics
when `base.source` is set; `/gnss/imu/data_raw` when the IMU is enabled.

### Several receivers

List receivers under `rovers:` instead of `rover:` (logging only):

```yaml
run:
  pipeline: logging
rovers:
  - name: f9p
    receiver: ubx
    source: "serial:///dev/ttyACM0:115200"
  - name: mosaic
    receiver: sbf
    source: "serial:///dev/ttyUSB0:115200"
    imu: true
```

| Key | Meaning |
|---|---|
| `name` | required; letters, digits, `_`; becomes the namespace (`base` is reserved) |
| `receiver`, `source` | required |
| `mode`, `params_file` | as for `rover:` |
| `imu` | IMU for this receiver (default: `imu.enabled`) |

Each receiver publishes under its name (`/f9p/gnss/observation`, ...) with
`frame_id` `f9p_gnss_link`. To feed one of them to an estimator, remap at playback:

```bash
ros2 bag play <bag> --remap /f9p/gnss/observation:=/gnss/observation \
                            /f9p/gnss/ephemeris:=/gnss/ephemeris
```

## RTK and `gnss_fgo`

```bash
ros2 launch gnss_ros_standardization pipeline.launch.py pipeline:=rtk
ros2 launch gnss_ros_standardization pipeline.launch.py pipeline:=gnss_fgo
```

Both need `base.source` and a base position (from the estimator YAML or
`estimator.base_position`). The [config/ppc/](../config/ppc/) files reproduce the
PPC dataset settings.

## `gnss_imu_fgo` and EKF

```bash
ros2 launch gnss_ros_standardization pipeline.launch.py pipeline:=gnss_imu_fgo
ros2 launch gnss_ros_standardization pipeline.launch.py pipeline:=ekf
ros2 launch gnss_ros_standardization pipeline.launch.py pipeline:=ekf ekf_solution_source:=receiver
```

`ekf_solution_source: rtk` runs RTK and feeds its solution to the EKF;
`receiver` uses the receiver's own solution and needs no base.

Raw IMU messages enabled on the receiver (`driver` mode does this automatically):

| Receiver | Raw IMU | Also needed |
|---|---|---|
| u-blox | `messages.esf_raw` | `messages.nav_clock` |
| Septentrio | `messages.ext_sensor_meas` | — |
| NovAtel | `messages.rawimusx` | — |

## Replay

```bash
ros2 launch gnss_ros_standardization pipeline.launch.py pipeline:=rtk replay:=true &
sleep 5
ros2 bag play <bag>
```

Play without `--clock`. Convert the result with
[`rosbag_to_pos`](../src/converter/README.md#rosbag_to_pos).

## On-chip RTK through the RTCM relay

```bash
ros2 launch gnss_ros_standardization pipeline.launch.py pipeline:=onboard_rtk
```

The rover node hosts the correction server (`relay.port`), `rtcm_decoder_node`
reads `base.source` and forwards it, and the receiver solution
(`/gnss/nmea_solution`) reaches FIX. NovAtel needs `relay.novatel_output`.
Receiver setup: [decoders](../src/decoders/README.md#rtcm-corrections-to-the-receiver-on-chip-rtk).

Check:

```bash
ss -tnp | grep 5556                    # correction source connected (ESTAB)
ros2 topic echo /gnss/nmea_solution    # status reaches FIX
```
