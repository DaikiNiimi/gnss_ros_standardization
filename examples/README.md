# Examples

Positioning and sensor-fusion examples built on top of the standardized GNSS
topics published by this package (`/gnss/observation`, `/gnss/ephemeris`,
`/gnss/imu/data_raw`, ...). Every example is receiver-agnostic: it works with
any of the supported decoders/drivers (u-blox, Septentrio, NovAtel, RTCM3) and
equally with live streams or rosbag playback.

| Example | Method | Coupling | Extra dependency | README |
|---|---|---|---|---|
| [`single_point_positioning`](single_point_positioning/) | SPP (RTKLIB `pntpos`) | — | — | [README](single_point_positioning/README.md) |
| [`real_time_kinematic`](real_time_kinematic/) | RTK (RTKLIB `rtkpos`) | — | — | [README](real_time_kinematic/README.md) |
| [`gnss_imu_kalman_filter`](gnss_imu_kalman_filter/) | Error-state EKF | Loose (GNSS solution + IMU) | — | [README](gnss_imu_kalman_filter/README.md) |
| [`tightly_coupled_gnss`](tightly_coupled_gnss/) | Factor graph optimization (ISAM2) | Tight (DD pseudorange + carrier phase) | GTSAM (develop) | [README](tightly_coupled_gnss/README.md) |
| [`tightly_coupled_gnss_imu`](tightly_coupled_gnss_imu/) | Factor graph optimization (ISAM2) | Tight (DD factors + IMU preintegration) | GTSAM (develop) | [README](tightly_coupled_gnss_imu/README.md) |

Example configs live in the package-level [config/](../config/) directory
(installed to `share/gnss_ros_standardization/config/`), alongside the driver
configs.

## Inputs and outputs

| Direction | Topic (default) | Type | Used by |
|---|---|---|---|
| Sub | `/gnss/observation` | `GnssObservations` | all examples |
| Sub | `/base/gnss/observation` | `GnssObservations` | RTK, tightly-coupled examples |
| Sub | `/gnss/ephemeris` | `GnssEphemerides` | all examples |
| Sub | `/gnss/imu/data_raw` | `sensor_msgs/Imu` | EKF, `tightly_coupled_gnss_imu` |
| Pub | `/gnss/solution` (or example-specific) | `GnssSolution` | all examples |

## Tightly-coupled FGO examples

The two `tightly_coupled_*` examples demonstrate the core goal of this
repository: this package handles the GNSS-domain work (rover/base epoch
matching, satellite position computation, double-difference pairing, masks,
wavelengths) through the `GnssPreprocessor` facade, so feeding GTSAM's official
GNSS factors ([introduced June 2026](https://gtsam.org/2026/06/10/rtk-gnss-double-difference.html))
takes only a few lines per epoch:

```
receiver / rosbag (any brand)
        │  driver / decoder
        ▼
/gnss/observation   /base/gnss/observation   /gnss/ephemeris
        │                   │                     │
        ▼                   ▼                     ▼
        GnssPreprocessor  (this package, no GTSAM dependency)
        epoch matching · satposs · masks · ref-sat selection · DD pairing
        │
        ▼  PreprocessedEpoch (sat positions, DD observables, wavelengths, ...)
        GTSAM factor construction (the example, a few lines)
        DoubleDifferencePseudorangeFactor / DoubleDifferenceCarrierPhaseFactor
        │
        ▼
        ISAM2 incremental optimization → /gnss/fgo/solution
```

They are opt-in because the GNSS factors are not yet in a GTSAM release tag —
see the [repository README](../README.md#optional-tightly-coupled-fgo-examples-gtsam)
for the GTSAM build, then `colcon build --cmake-args -DBUILD_GTSAM_FGO_EXAMPLES=ON`.
