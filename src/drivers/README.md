# Drivers

Drivers connect to a receiver, configure its output messages on startup, and then
decode the stream exactly like the [decoders](../decoders/README.md) (same topics,
same message tables, same common parameters).

| Executable | Receiver | Configuration commands | Configuration file |
|---|---|---|---|
| `ubx_driver_node` | u-blox | UBX-CFG (`G9`) or UBX-CFG-VALSET (`G10`) | [`config/ubx_driver.yaml`](../../config/ubx_driver.yaml) |
| `sbf_driver_node` | Septentrio | `setSBFOutput` / `setNMEAOutput` | [`config/sbf_driver.yaml`](../../config/sbf_driver.yaml) |
| `novatel_driver_node` | NovAtel OEM | `log ... ontime` / `onnew` | [`config/novatel_driver.yaml`](../../config/novatel_driver.yaml) |

```bash
ros2 run gnss_ros_standardization ubx_driver_node --ros-args \
  --params-file config/ubx_driver.yaml
```

## Parameters

In addition to the [decoder parameters](../decoders/README.md#parameters):

| Parameter | Default (YAML) | Nodes | Meaning |
|---|---|---|---|
| `configure_on_startup` | `true` | all | send the configuration commands; `false` = decode only |
| `rate_hz` | `10` | all | measurement / log rate in Hz (u-blox and NovAtel 1–20, Septentrio 1–200) |
| `receiver_port` | `USB1` / `THISPORT` | SBF, NovAtel | receiver port the output is configured on; must be the connected port |
| `generation` | `G9` | UBX | `G9` (F9P, M8: UBX-CFG) or `G10` (X20P: UBX-CFG-VALSET) |
| `dynamic_model` | `portable` | UBX | `portable`, `stationary`, `pedestrian`, `automotive`, `sea`, `airborne_1g`, `airborne_2g`, `airborne_4g`, `watch` |
| `navsys.gps`, `.glo`, `.gal`, `.bds`, `.qzs`, `.irn`, `.sbs` | see YAML | UBX | constellations enabled on the receiver |
| `rtcm_input.enable` | `false` | all | host the RTCM correction server (`rtcm_input.listen`) |
| `messages.*` | see YAML | all | output messages (tables below) |

## Output messages

### `ubx_driver_node`

| `messages.` | UBX message | Topic |
|---|---|---|
| `rawx` | RXM-RAWX | observation |
| `sfrbx` | RXM-SFRBX | ephemeris |
| `nav_pvt`, `nav_dop`, `nav_cov` | NAV-PVT, NAV-DOP, NAV-COV | solution |
| `nav_posecef`, `nav_velecef` | NAV-POSECEF, NAV-VELECEF | solution |
| `nav_hpposllh`, `nav_hpposecef` | NAV-HPPOSLLH, NAV-HPPOSECEF (with `nav_pvt`) | solution |
| `nmea_gga`, `nmea_rmc`, `nmea_gsa`, `nmea_gst` | NMEA | solution (when no binary PVT) |
| `nmea_high_precision` | NMEA high-precision mode | — |
| `nav_clock` | NAV-CLOCK | used for the IMU time stamps, not published |
| `esf_raw` | ESF-RAW | imu_raw |
| `esf_ins` | ESF-INS | imu |

### `sbf_driver_node`

| `messages.` | SBF block | Topic |
|---|---|---|
| `meas_epoch` | MeasEpoch, EndOfMeas | observation |
| `gps_nav`, `glo_nav`, `gal_nav`, `bds_nav`, `qzs_nav` | GPSNav, GLONav, GALNav, BDSNav, QZSNav | ephemeris |
| `gps_nav_raw`, `glo_nav_raw`, `gal_nav_raw`, `bds_nav_raw`, `qzs_nav_raw`, `navic_nav_raw` | GPSRawCA, GLORawCA, GALRawINAV + GALRawFNAV, BDSRaw, QZSRawL1CA, NAVICRaw | ephemeris |
| `pvt_geodetic`, `pos_cov_geodetic`, `vel_cov_geodetic` | PVTGeodetic, PosCovGeodetic, VelCovGeodetic | solution |
| `pvt_cartesian`, `pos_cov_cartesian`, `vel_cov_cartesian` | PVTCartesian, PosCovCartesian, VelCovCartesian | solution |
| `dop` | DOP | solution |
| `nmea_gga`, `nmea_rmc`, `nmea_gsa`, `nmea_gst` | NMEA | solution (when no binary PVT) |
| `ext_sensor_meas` | ExtSensorMeas | imu_raw |

Use either the geodetic or the cartesian PVT/covariance set.

### `novatel_driver_node`

| `messages.` | NovAtel log | Topic |
|---|---|---|
| `rangecmp`, `range` | RANGECMPB, RANGEB | observation |
| `gps_ephem`, `glo_ephem`, `gal_ephem`, `bds_ephem`, `qzs_ephem`, `navic_ephem` | RAWEPHEMB, GLOEPHEMERISB, GALEPHEMERISB, BDSEPHEMERISB, QZSSRAWEPHEMB, NAVICEPHEMERISB | ephemeris |
| `ionutc` | IONUTCB, GALIONOB, QZSSIONUTCB | ephemeris |
| `bestpos`, `bestvel`, `bestxyz`, `psrdop` | BESTPOSB, BESTVELB, BESTXYZB, PSRDOPB | solution |
| `nmea_gpgga`, `nmea_gprmc`, `nmea_gpgsa`, `nmea_gpgst` | NMEA | solution (when no binary PVT) |
| `rawimusx` | RAWIMUSXB (`onnew`) | imu_raw |
| `corrimudata` | CORRIMUDATAB | imu |

## NovAtel IMU scale factors

RAWIMUSX counts are converted with the scale factors of the IMU type in the log:

| IMUType | IMU | gyro (rad/LSB) | accel (m/s/LSB) |
|---|---|---|---|
| 5 | HG1900-CA29 | 2⁻³³ | 2⁻²⁷ × 0.3048 |
| 8 | LN-200 | 2⁻¹⁹ | 2⁻¹⁴ |
| 11 | HG1700-AG58 | 2⁻³³ | 2⁻²⁷ × 0.3048 |
| 12 | HG1700-AG62 | 2⁻³³ | 2⁻²⁶ × 0.3048 |
| 20 | HG1930-AA99 | 2⁻³³ | 2⁻²⁷ × 0.3048 |
| 26 | ISA-100C / µIMU-IC | 1.0e-9 | 2.0e-8 |
| 27 | HG1900-CA50 | 2⁻³³ | 2⁻²⁷ × 0.3048 |
| 28 | HG1930-CA50 | 2⁻³³ | 2⁻²⁷ × 0.3048 |
| 32 | OEM-IMU-STIM300 | 2⁻²¹ × π/180 | 2⁻²² |
| 41 | OEM-IMU-EG320N (125 Hz) | (0.008/65536/125) × π/180 | (0.200/65536/125) × g/1000 |
| 56 | OEM-IMU-STIM300D | 2⁻²¹ × π/180 | 2⁻²² |
| 58 | HG4930-AN01 (CPT7/CPT7700) | 2⁻³³ | 2⁻²⁹ |
| 61 | OEM-IMU-EG370N (200 Hz) | (0.0151515/65536/200) × π/180 | (0.400/65536/200) × g/1000 |
| 62 | OEM-IMU-EG320N (200 Hz) | (0.008/65536/200) × π/180 | (0.200/65536/200) × g/1000 |
| 68 | HG4930-AN04 | 2⁻³³ | 2⁻²⁹ |
| 69 | HG4930-AN04 (400 Hz) | 2⁻³³ | 2⁻²⁹ |

For another IMU set `imu_scale_override.accel` and `imu_scale_override.gyro`.

## RTCM corrections to the receiver (on-chip RTK)

Set `rtcm_input.enable: true` and `rtcm_input.listen` (recommended ports: u-blox
5556, Septentrio 5557, NovAtel 5558). NovAtel also needs `rtcm_input.receiver_stream`
(a second port). Receiver setup and an example: [decoders](../decoders/README.md#rtcm-corrections-to-the-receiver-on-chip-rtk).
The [`onboard_rtk` pipeline](../../launch/README.md#on-chip-rtk-through-the-rtcm-relay)
starts the driver and the correction source together.
