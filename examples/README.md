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
| [`gnss_fgo`](tightly_coupled_fgo/) | Factor graph optimization (ISAM2) | Tight (DD pseudorange + carrier phase) | GTSAM (develop) | [README](tightly_coupled_fgo/README.md) |
| [`gnss_imu_fgo`](tightly_coupled_fgo/) | Factor graph optimization (ISAM2) | Tight (DD factors + IMU preintegration) | GTSAM (develop) | [README](tightly_coupled_fgo/README.md) |

Example configs live in the package-level [config/](../config/) directory.

The tightly-coupled FGO examples are opt-in (they need GTSAM) — see the
[repository README](../README.md#optional-tightly-coupled-fgo-examples-gtsam) for
the GTSAM build.
