# Drivers

Driver nodes connect to a receiver, configure
its output messages on startup (optional), and decode the resulting stream
into ROS 2 messages.

| Node | Executable | Configuration method | Sample config |
|---|---|---|---|
| u-blox | `ubx_driver_node` | UBX-CFG | [`config/ubx_driver.yaml`](../../config/ubx_driver.yaml) |
| Septentrio | `sbf_driver_node` | SBF/NMEA ASCII commands | [`config/sbf_driver.yaml`](../../config/sbf_driver.yaml) |
| NovAtel | `novatel_driver_node` | `log ... ontime` / `onnew` commands | [`config/novatel_driver.yaml`](../../config/novatel_driver.yaml) |

The decoder layer is shared with [`../decoders/`](../decoders/) — refer to
[`../decoders/README.md`](../decoders/README.md) for the contents of each wire
message listed below. Message-level semantics (frames, NaN convention,
covariance frame) live in [`../../msg/README.md`](../../msg/README.md).

## RTCM relay (receiver on-chip RTK)

Every driver can host a TCP server (`rtcm_relay.listen`) and forward RTCM
corrections pushed there down to the receiver for on-chip RTK. Feed it from
`rtcm_decoder_node` or any RTKLIB `str2str`/STRSVR.

| YAML key | Applies to | Meaning |
|---|---|---|
| `rtcm_relay.enabled` | all | `false` (default) disables the relay |
| `rtcm_relay.listen` | all | Listen URI; recommended `tcpsvr://:5556` (ubx) / `:5557` (sbf) / `:5558` (novatel) |
| `rtcm_relay.output` | NovAtel | Dedicated RTCMV3 port device (e.g. `serial:///dev/ttyUSB2:230400`); must differ from `stream_path` |
| `rtcm_relay.correction_port` | NovAtel | `INTERFACEMODE` target sent over `output` — leave `THISPORT` |

Per-receiver receiver-side setup (u-blox `inProtoMask`, Septentrio
`setDataInOut`, NovAtel dedicated port) is in
[`../decoders/README.md`](../decoders/README.md#rtcm-correction-relay-receiver-on-chip-rtk).

---

## `ubx_driver_node`

```bash
ros2 run gnss_ros_standardization ubx_driver_node --ros-args \
  --params-file config/ubx_driver.yaml
```

When `configure_on_startup: true`, the driver sends UBX-CFG-VALSET to enable the
requested message rates and constellations.

### Configured outputs

| YAML key | Wire message | Output topic |
|---|---|---|
| `messages.rawx` | `UBX-RXM-RAWX` | `/gnss/observation` |
| `messages.sfrbx` | `UBX-RXM-SFRBX` | `/gnss/ephemeris` |
| `messages.nav_pvt` | `UBX-NAV-PVT` | `/gnss/nmea_solution` |
| `messages.nav_dop` | `UBX-NAV-DOP` | `/gnss/nmea_solution` |
| `messages.nav_cov` | `UBX-NAV-COV` | `/gnss/nmea_solution` |
| `messages.nav_posecef` | `UBX-NAV-POSECEF` | `/gnss/nmea_solution` (optional native ECEF) |
| `messages.nav_velecef` | `UBX-NAV-VELECEF` | `/gnss/nmea_solution` (optional native ECEF) |
| `messages.nav_hpposllh` | `UBX-NAV-HPPOSLLH` | `/gnss/nmea_solution` (high-precision LLH; overrides NAV-PVT position; requires `nav_pvt: true`) |
| `messages.nav_hpposecef` | `UBX-NAV-HPPOSECEF` | `/gnss/nmea_solution` (high-precision ECEF; overrides NAV-POSECEF; requires `nav_pvt: true`) |
| `messages.nmea_gga` / `.nmea_rmc` / `.nmea_gsa` / `.nmea_gst` | NMEA `GGA/RMC/GSA/GST` | `/gnss/nmea_solution` (fallback) |
| `messages.nmea_high_precision` | NMEA high precision | extra lat/lon decimal places |
| `messages.esf_raw` | `UBX-ESF-RAW` | `/gnss/imu/data_raw` (IMU-enabled receiver) |
| `messages.esf_ins` | `UBX-ESF-INS` | `/gnss/imu/data` (IMU-enabled receiver) |

Enable `messages.nav_posecef` / `.nav_velecef` only when the receiver-provided
ECEF position/velocity is preferred over values derived from LLH.

---

## `sbf_driver_node`

```bash
ros2 run gnss_ros_standardization sbf_driver_node --ros-args \
  --params-file config/sbf_driver.yaml
```

When `configure_on_startup: true`, the driver issues `setSBFOutput` /
`setNMEAOutput` ASCII commands on `receiver_port` to enable the requested SBF
blocks at the requested rate.

### Configured outputs

| YAML key | SBF block | Output topic |
|---|---|---|
| `messages.meas_epoch` | `MeasEpoch` | `/gnss/observation` |
| `messages.gps_nav` | `GPSNav` | `/gnss/ephemeris` |
| `messages.glo_nav` | `GLONav` | `/gnss/ephemeris` |
| `messages.gal_nav` | `GALNav` | `/gnss/ephemeris` |
| `messages.bds_nav` | `BDSNav` | `/gnss/ephemeris` |
| `messages.qzs_nav` | `QZSNav` | `/gnss/ephemeris` |
| `messages.navic_nav` | `NavICLNav` | `/gnss/ephemeris` |
| `messages.gps_nav_raw` | `GPSRawCA` | `/gnss/ephemeris` (raw subframe) |
| `messages.glo_nav_raw` | `GLORawCA` | `/gnss/ephemeris` (raw subframe) |
| `messages.gal_nav_raw` | `GALRawINAV` + `GALRawFNAV` | `/gnss/ephemeris` (raw subframe) |
| `messages.bds_nav_raw` | `BDSRaw` | `/gnss/ephemeris` (raw subframe) |
| `messages.qzs_nav_raw` | `QZSRawL1CA` | `/gnss/ephemeris` (raw subframe) |
| `messages.navic_nav_raw` | `NAVICRaw` | `/gnss/ephemeris` (raw subframe) |
| `messages.pvt_geodetic` | `PVTGeodetic` | `/gnss/nmea_solution` |
| `messages.pos_cov_geodetic` | `PosCovGeodetic` | `/gnss/nmea_solution` |
| `messages.vel_cov_geodetic` | `VelCovGeodetic` | `/gnss/nmea_solution` |
| `messages.pvt_cartesian` | `PVTCartesian` | `/gnss/nmea_solution` (optional native ECEF) |
| `messages.pos_cov_cartesian` | `PosCovCartesian` | `/gnss/nmea_solution` (optional native ECEF) |
| `messages.vel_cov_cartesian` | `VelCovCartesian` | `/gnss/nmea_solution` (optional native ECEF) |
| `messages.dop` | `DOP` | `/gnss/nmea_solution` |
| `messages.nmea_gga` / `.nmea_rmc` / `.nmea_gsa` / `.nmea_gst` | NMEA `GGA/RMC/GSA/GST` | `/gnss/nmea_solution` (fallback) |
| `messages.ext_sensor_meas` | `ExtSensorMeas` | `/gnss/imu/data_raw` (AsteRx-i / mosaic-X5) |

**Recommended:** `PVTGeodetic` + `PosCovGeodetic` + `VelCovGeodetic` alone are
sufficient. Enabling the `*Cartesian` covariance blocks *in addition* only
sharpens `pos_ecef` / `pos_cov_ecef` slightly versus LLH-derived values — for
most users it is not worth the extra receiver bandwidth. The driver emits a
runtime warning if both `*Geodetic` and `*Cartesian` covariance blocks are
enabled simultaneously.

---

## `novatel_driver_node`

```bash
ros2 run gnss_ros_standardization novatel_driver_node --ros-args \
  --params-file config/novatel_driver.yaml
```

When `configure_on_startup: true`, the driver issues `log ... ontime` / `onnew`
commands on `receiver_port` (e.g. `COM1`, `USB1`, `THISPORT`).

### Configured outputs

| YAML key | NovAtel log | Output topic |
|---|---|---|
| `messages.rangecmp` | `RANGECMPB` | `/gnss/observation` (recommended) |
| `messages.range` | `RANGEB` | `/gnss/observation` |
| `messages.gps_ephem` | `RAWEPHEMB` | `/gnss/ephemeris` |
| `messages.glo_ephem` | `GLOEPHEMERISB` | `/gnss/ephemeris` |
| `messages.gal_ephem` | `GALEPHEMERISB` | `/gnss/ephemeris` |
| `messages.bds_ephem` | `BDSEPHEMERISB` | `/gnss/ephemeris` |
| `messages.qzs_ephem` | `QZSSRAWEPHEMB` | `/gnss/ephemeris` |
| `messages.navic_ephem` | `NAVICEPHEMERISB` | `/gnss/ephemeris` |
| `messages.ionutc` | `IONUTCB` / `GALIONOB` / `QZSSIONUTCB` | `/gnss/ephemeris` |
| `messages.bestpos` | `BESTPOSB` | `/gnss/nmea_solution` |
| `messages.bestvel` | `BESTVELB` | `/gnss/nmea_solution` |
| `messages.bestxyz` | `BESTXYZB` | `/gnss/nmea_solution` (optional native ECEF) |
| `messages.psrdop` | `PSRDOPB` | `/gnss/nmea_solution` |
| `messages.nmea_gpgga` / `.nmea_gprmc` / `.nmea_gpgsa` / `.nmea_gpgst` | NMEA `GGA/RMC/GSA/GST` | `/gnss/nmea_solution` (fallback) |
| `messages.rawimusx` | `RAWIMUSXB` | `/gnss/imu/data_raw` (logged via `ONNEW`, not `ONTIME`) |
| `messages.corrimudata` | `CORRIMUDATAB` | `/gnss/imu/data` (SPAN-capable receiver) |

### Note on RAWIMUSX / CORRIMUDATA scaling

`RAWIMUSXB` carries an `IMUType` byte; the decoder looks up the per-sample
count → SI scale factor from a table in
[`include/gnss_ros_standardization/novatel_imu_scales.hpp`](../../include/gnss_ros_standardization/novatel_imu_scales.hpp)
and divides by the inter-sample `dt` to recover rates and accelerations.
`CORRIMUDATAB` carries SI-unit *increments* over one IMU sampling period; the
decoder divides each increment by `dt` likewise.

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
