# Topic data contract

Rules shared by every node that publishes or reads the six message types.

## Time and frame

| Field | Meaning |
|---|---|
| `header.stamp` | `use_gps_timestamp: false`: host receive time. `true`: the GNSS epoch converted to UTC. Converted bags: UTC by default, GPST with `--timestamp-scale gpst` (recorded in `gnss_time_basis.yaml`) |
| `week`, `tow` / `time_week`, `time_tow` | GNSS epoch in GPS time, always |
| `header.frame_id` | receiver label: `gnss_link`, or `<name>_gnss_link` for a named receiver |

Use `week`/`tow` for the GNSS epoch; the header stamp can be the arrival time.
Week 0 is invalid.

## Observations (`GnssObservations`)

One message per epoch; one `GnssObservation` per satellite and signal.

| Field | Rule |
|---|---|
| `system`, `prn`, `satid` | satellite identity (`satid` e.g. `G01`, `R01`, SBAS `129`); identify a satellite by `system` + `prn` |
| `code`, `code_str` | RINEX signal code (`code_str` e.g. `1C`, `2W`, `5Q`) |
| `p` | pseudorange, m |
| `l` | carrier phase, cycles |
| `d` | Doppler, Hz |
| `snr` | C/N0, dB-Hz |
| `lli` | loss-of-lock byte as RTKLIB `obsd_t.LLI`: `0x01` cycle slip, `0x02` half cycle not resolved, `0x04` BOC tracking, `0x40` half cycle added, `0x80` half cycle subtracted; test individual bits |
| `freq` | GLONASS FDMA channel + 7 (0–13); 255 = unknown or not GLONASS |
| `sat` | RTKLIB internal satellite number (depends on build options; do not store) |

- NaN marks a component the receiver reported invalid or did not measure; each of
  `p`, `l`, `d`, `snr` is independent. A finite zero (e.g. zero Doppler) is a valid value.
- Negative pseudorange or C/N0 is published as NaN.
- RTCM observations are published when the last message of an epoch arrives
  (multiple-message bit 0). Incomplete epochs are never published.
- SBF observations are published at EndOfMeas, at the next epoch, or 0.2 s after
  the last block of the epoch.

## Ephemerides (`GnssEphemerides`)

- From a receiver node, each message is a complete snapshot of its ephemeris store
  (up to 8 records per satellite and the latest ionosphere parameters), published
  when something is added or changes and every `ephemeris.snapshot_period_s`.
  `rinex_to_rosbag` writes each record at its transmission time.
- `ion_gps`, `ion_gal`, `ion_qzs`, `ion_bds`, `ion_irn`: broadcast ionosphere model
  parameters in RTKLIB `nav_t` order (Klobuchar `a0..a3, b0..b3`; Galileo NeQuick
  `ai0..ai2`). All zero = not received (RTCM carries none). SPP and RTK use them with
  the broadcast ionosphere option.
- QoS: reliable, transient local, depth 256. A late subscriber receives the latest
  snapshot. For playback use the same profile:

```yaml
# eph_qos.yaml (ros2 bag play --qos-profile-overrides-path eph_qos.yaml)
/gnss/ephemeris:
  history: keep_last
  depth: 1
  reliability: reliable
  durability: transient_local
```

- Fields follow RTKLIB `eph_t` / `geph_t`. GLONASS `frq` is the signed channel
  number (−7 to 6), unlike the observation `freq` (+7).
- Header stamp: publication time. The broadcast times are in `week`, `toe`, `toc`,
  `ttr` (GLONASS `tof`).

## Solutions (`GnssSolution`)

| `status` | Value | Producers |
|---|---|---|
| `STATUS_NONE` | 0 | no solution: position, velocity, covariances and DOP are NaN, `num_sats` 0 |
| `STATUS_FIX` | 1 | RTK / FGO fixed, receiver RTK fixed |
| `STATUS_FLOAT` | 2 | RTK / FGO float |
| `STATUS_SBAS` | 3 | receiver SBAS |
| `STATUS_DGPS` | 4 | RTK DGPS, receiver DGPS |
| `STATUS_SINGLE` | 5 | SPP, receiver single |
| `STATUS_PPP` | 6 | receiver PPP |
| `STATUS_DR` | 7 | dead reckoning: FGO epochs without a GNSS update, EKF when the next GNSS update is overdue, receiver dead reckoning |

| Field | Rule |
|---|---|
| `solution_source` | `BINARY` / `NMEA` (receiver), `COMPUTED` (SPP, RTK, EKF, FGO) |
| `latitude`, `longitude`, `altitude` | degrees, ellipsoidal height in m |
| `pos_ecef`, `vel_ecef`, `pos_cov_ecef`, `vel_cov_ecef` | ECEF, covariances row-major |
| `pos_enu`, `pos_enu_org_ecef` | ENU position relative to the origin set by `local_origin.*` ([parameter names](PARAMETER_NAMES.md#enu-origin)); NaN until the origin is known |
| `vel_enu`, `pos_enu_cov`, `vel_enu_cov` | in the ENU frame at the current position |
| `num_sats` | satellites used (RTK: satellites with a valid carrier double difference; EKF: 0) |
| `ratio` | AR ratio |
| `age_diff`, DOP | NaN when not available |

An unknown velocity is NaN together with its covariance.
SPP and RTK report the RTKLIB solution time (the receiver epoch corrected by the
estimated receiver clock bias, within about 1 ms of the observation epoch); EKF and
FGO report the observation epoch.

## IMU (`sensor_msgs/Imu`)

| Topic | Content |
|---|---|
| `/gnss/imu/data_raw` | raw specific force (m/s²) and angular rate (rad/s) in the sensor axes; `orientation_covariance[0] = -1` (no orientation, REP-145) |
| `/gnss/imu/data` | receiver-corrected IMU (UBX ESF-INS, NovAtel CORRIMUDATA) |

With `use_gps_timestamp: true` IMU stamps are on the same GNSS-time (UTC) clock as
the observations. Use `/gnss/imu/data_raw` as input to an estimator that also uses
the GNSS measurements.

## Compatibility

Build publishers and subscribers from the same message definitions (same
installation); bags recorded with other definitions of these types cannot be read.
See [installed API](INSTALLED_API.md).
