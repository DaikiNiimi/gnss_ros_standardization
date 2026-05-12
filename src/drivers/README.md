# Drivers

Driver nodes are full receiver clients: they **connect to a receiver, configure
its output messages on startup (optional), and decode the resulting stream**
into ROS 2 messages. Unlike the decoder nodes in [`../decoders/`](../decoders/),
drivers know how to talk back to the receiver (UBX-CFG, ASCII SBF commands,
NovAtel `log ... ontime ...` etc.), so they are the recommended choice when the
receiver is directly attached.

| Node | Executable | Source | Sample config |
|---|---|---|---|
| u-blox | `ubx_driver_node` | [`ubx_driver_node.cpp`](ubx_driver_node.cpp) | [`config/ubx_driver.yaml`](../../config/ubx_driver.yaml) |
| Septentrio | `sbf_driver_node` | [`sbf_driver_node.cpp`](sbf_driver_node.cpp) | [`config/sbf_driver.yaml`](../../config/sbf_driver.yaml) |
| NovAtel | `novatel_driver_node` | [`novatel_driver_node.cpp`](novatel_driver_node.cpp) | [`config/novatel_driver.yaml`](../../config/novatel_driver.yaml) |

For the underlying binary message decoders (UBX / SBF / OEM4 message IDs and
contents), see [`../decoders/README.md`](../decoders/README.md) — drivers reuse
the same decoder layer.

---

## `ubx_driver_node`

```bash
cp config/ubx_driver.yaml my_ubx.yaml
# edit serial_port, baud_rate, enable flags
ros2 run gnss_ros_standardization ubx_driver_node --ros-args \
  --params-file my_ubx.yaml
```

When `configure_on_startup: true`, the driver sends UBX-CFG-VALSET to enable the
requested message rates and the requested GNSS constellations.

### Parameters

| Parameter | Default (sample) | Description |
|---|---|---|
| `stream_path` | `serial:///dev/ttyACM0:115200` | UBX stream URI |
| `rate_hz` | 5 | Measurement rate |
| `configure_on_startup` | `true` | Send UBX-CFG to the receiver |
| `dynamic_model` | `automotive` | UBX-CFG-NAV5 dynamic model |
| `generation` | `F9` | Receiver generation (`M8`/`F9`/`X20`) |
| `frame_id` | `gnss_link` | ROS frame_id |
| `messages.rawx` / `.sfrbx` | `true` | Enable RXM-RAWX / RXM-SFRBX |
| `messages.nav_pvt` | `true` | Enable NAV-PVT for `/gnss/nmea_solution` |
| `messages.nmea_gga|rmc|gsa|gst` | mixed | NMEA fallback |
| `messages.nmea_high_precision` | `false` | Enable extended NMEA precision |
| `messages.esf_raw` / `.esf_ins` | `false` | Raw / calibrated IMU (ZED-F9R required) |
| `gnss.gps|glonass|galileo|beidou|qzss|navic|sbas` | mixed | Enable constellations |
| `observation_topic` / `ephemeris_topic` / `solution_topic` | defaults | Topic overrides |
| `imu_topic` / `imu_raw_topic` | defaults | IMU topic overrides |
| `auto_origin`, `origin.latitude/longitude/altitude` | — | Local ENU origin policy |

---

## `sbf_driver_node`

```bash
ros2 run gnss_ros_standardization sbf_driver_node --ros-args \
  --params-file config/sbf_driver.yaml
```

When `configure_on_startup: true`, the driver issues `setSBFOutput` /
`setNMEAOutput` ASCII commands on the specified `receiver_port` (e.g. `USB1`)
to enable the requested SBF blocks at the requested rate.

### Parameters

| Parameter | Description |
|---|---|
| `stream_path` | Stream URI |
| `frame_id`, `publish_rate`, `receiver_port`, `configure_on_startup` | Driver-level settings |
| `messages.meas_epoch` | Enable MeasEpoch (4027) → `/gnss/observation` |
| `messages.{gps,glo,gal,bds,qzs,navic}_nav` | Enable receiver-assembled nav blocks |
| `messages.{gps,glo,gal,bds,qzs,navic}_nav_raw` | Enable raw subframe blocks |
| `messages.pvt_geodetic` / `pos_cov_geodetic` | Geodetic PVT + covariance |
| `messages.pvt_cartesian` / `pos_cov_cartesian` | ECEF PVT + covariance |
| `messages.ext_sensor_meas` | Enable ExtSensorMeas (4050) → `/gnss/imu/data_raw` |
| `messages.receiver_status`, `messages.quality_ind` | Diagnostics |
| `messages.nmea_{gga,rmc,gsa,gst}` | NMEA fallback |
| `observation_topic` / `ephemeris_topic` / `solution_topic` / `imu_raw_topic` | Topic overrides |

Septentrio does not provide calibrated IMU — there is no `imu_topic`.

---

## `novatel_driver_node`

```bash
ros2 run gnss_ros_standardization novatel_driver_node --ros-args \
  --params-file config/novatel_driver.yaml
```

When `configure_on_startup: true`, the driver issues `log ... ontime` / `onnew`
commands on the configured `receiver_port` (e.g. `COM1`, `USB1`).

### Parameters

| Parameter | Description |
|---|---|
| `stream_path` | Stream URI |
| `frame_id`, `publish_rate`, `receiver_port`, `configure_on_startup` | Driver-level settings |
| `format` | `oem4` (default) |
| `messages.rangecmp` / `messages.range` | RANGECMPB (140) / RANGEB (43) → `/gnss/observation` |
| `messages.bestpos` / `bestvel` | BESTPOS / BESTVEL → `/gnss/nmea_solution` |
| `messages.{gps,glo,gal,bds,qzs,navic}_ephem`, `ionutc` | Ephemeris + iono |
| `messages.nmea_{gpgga,gprmc,gpgsa,gpgst}` | NMEA fallback |
| `messages.rawimusx` | RAWIMUSXB (1462) → `/gnss/imu/data_raw` (ONNEW) |
| `messages.corrimudata` | CORRIMUDATAB (812) → `/gnss/imu/data` (ONNEW; SPAN required) |
| `imu_scale_override.accel` / `.gyro` | Override per-sample IMU scale factors |
| `observation_topic` / `ephemeris_topic` / `solution_topic` / `imu_topic` / `imu_raw_topic` | Topic overrides |

### Note on RAWIMUSX scale factors

`RAWIMUSXB` carries an `IMUType` byte. The decoder looks up the per-sample
count → SI scale factor (delta-velocity in m/s, delta-angle in rad) from a
table in
[`include/gnss_ros_standardization/novatel_imu_scales.hpp`](../../include/gnss_ros_standardization/novatel_imu_scales.hpp)
and divides by the inter-sample `dt` (successive `Seconds` fields) to recover
rates and accelerations.

### Note on CORRIMUDATA

The log carries SI-unit *increments* over one IMU sampling period. The decoder
divides each increment by the timestamp interval `dt` to recover instantaneous
rates and accelerations.

---

## Supported NovAtel IMUs (RAWIMUSX scale table)

IDs follow the official [CONNECTIMU table](https://docs.novatel.com/OEM7/Content/SPAN_Commands/CONNECTIMU.htm).
Scale factors are taken verbatim from the
[RAWIMUSX page](https://docs.novatel.com/OEM7/Content/SPAN_Logs/RAWIMUSX.htm)
and converted to SI (ft/s → m/s, deg → rad) at compile time.

| IMUType | IMU model | gyro (rad/LSB·sample) | accel (m/s/LSB·sample) |
|---|---|---|---|
| 5  | HG1900-CA29                  | 2⁻³³ | 2⁻²⁷ × 0.3048 |
| 8  | LN-200                       | 2⁻¹⁹ | 2⁻¹⁴ |
| 11 | HG1700-AG58                  | 2⁻³³ | 2⁻²⁷ × 0.3048 |
| 12 | HG1700-AG62                  | 2⁻³³ | 2⁻²⁶ × 0.3048 |
| 20 | HG1930-AA99                  | 2⁻³³ | 2⁻²⁷ × 0.3048 |
| 26 | ISA-100C / µIMU-IC           | 1.0e-9 | 2.0e-8 |
| 27 | HG1900-CA50                  | 2⁻³³ | 2⁻²⁷ × 0.3048 |
| 28 | HG1930-CA50                  | 2⁻³³ | 2⁻²⁷ × 0.3048 |
| 32 | OEM-IMU-STIM300              | 2⁻²¹ × π/180 | 2⁻²² |
| 41 | OEM-IMU-EG320N (125 Hz)      | (0.008/65536/125) × π/180 | (0.200/65536/125) × g/1000 |
| 56 | OEM-IMU-STIM300D             | 2⁻²¹ × π/180 | 2⁻²² |
| 58 | HG4930-AN01 (also CPT7/CPT7700) | 2⁻³³ | 2⁻²⁹ |
| 61 | OEM-IMU-EG370N (200 Hz)      | (0.0151515/65536/200) × π/180 | (0.400/65536/200) × g/1000 |
| 62 | OEM-IMU-EG320N (200 Hz)      | (0.008/65536/200) × π/180 | (0.200/65536/200) × g/1000 |
| 68 | HG4930-AN04                  | 2⁻³³ | 2⁻²⁹ |
| 69 | HG4930-AN04 (400 Hz)         | 2⁻³³ | 2⁻²⁹ |

For any IMU not listed above, set `imu_scale_override.{accel,gyro}` in
`config/novatel_driver.yaml` to per-sample SI scale factors from the IMU
datasheet.

**EPSON G320N data-rate caveat**: the published scale depends on the IMU's
sample rate. ID 41 assumes 125 Hz and ID 62 assumes 200 Hz; if you run the IMU
at any other rate, use `imu_scale_override.*` instead.

---

## Troubleshooting

**Serial port permission denied**
```bash
sudo usermod -aG dialout $USER   # log out and back in
```

**`configure_on_startup` has no effect**
- Confirm `receiver_port` matches the port the receiver sees you on
  (e.g. `USB1`, `COM2`). The configuration command goes via that port name —
  not via the OS device path.
- For NovAtel, some logs (e.g. `RAWIMUSXB`) only respond to `ONNEW`, not `ONTIME`.

**Wrong IMU rates / accelerations (NovAtel)**
- See *Supported NovAtel IMUs* above and set `imu_scale_override.*` if needed.
