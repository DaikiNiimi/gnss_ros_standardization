# Tightly-coupled GNSS / GNSS-IMU factor-graph optimization

RTK positioning expressed as incremental factor-graph optimization. Raw
double-difference (DD) pseudorange and carrier-phase measurements are fed
directly into [GTSAM](https://gtsam.org)'s official GNSS factors
([introduced June 2026](https://gtsam.org/2026/06/10/rtk-gnss-double-difference.html))
and optimized with ISAM2; integer ambiguities are resolved on the graph
posterior with RTKLIB's `lambda()` and a ratio test, producing **FIX** solutions.

Two nodes share the same factor construction in
[factor_adapters.hpp](factor_adapters.hpp): `gnss_fgo` (GNSS only) and
`gnss_imu_fgo` (adds IMU preintegration). `GnssPreprocessor` does all the
GNSS-domain work — epoch matching, satellite positions, masks, DD pairing — with
no GTSAM dependency.

These are **examples**: they show how to represent the RTK problem in a factor
graph, not a replacement for a dedicated RTK engine. The graph consumes the
*same* observations as a classical RTK filter, so it adds no information — the
goal is to match a well-tuned filter, not to beat it. Report fix rate and
accuracy against external truth, never against another estimator.

The opt-in GTSAM build is described in the
[repository README](../../README.md#optional-tightly-coupled-fgo-examples-gtsam).
**Set the base-station position in the YAML** — an error there translates the
whole trajectory.

## How ambiguities are resolved

One single-difference carrier ambiguity per `(satellite, band)` is a graph state,
carried across epochs and re-keyed only on a cycle slip or outage, so the carrier
phase accumulates and σ(N) falls over the arc. Each epoch:

1. Motion / IMU factors and this epoch's code + carrier DD blocks are added.
   Each block is one grouped factor with the full covariance of the DDs sharing
   a reference satellite, so optimization and AR use one likelihood.
2. The joint posterior `[position, N...]` (or `[pose, velocity, bias, N...]`) is
   read from the graph; LAMBDA gets `a_float` and `Qa` projected from it. There
   is no second float estimator.
3. A pre-fit innovation test excludes outliers, then LAMBDA and the ratio /
   position gates select integers and the state is conditioned once. Accepted
   integers are held as gauge-free constraints after per-pair confirmation and
   cycle-closure checks; a mismatch rolls the affected component back.

Dual-frequency (L1+L2) makes slip detection and re-locking far more reliable and
is recommended.

## `gnss_fgo` — GNSS only

<p align="center"><img src="../../fig/gnss_fgo_graph.svg" alt="gnss_fgo factor graph" width="720"></p>

The rover position is carried across epochs by a Doppler-derived motion factor
(a `BetweenFactor<Point3>`, like RTKLIB's EKF dynamics).

```bash
ros2 run gnss_ros_standardization gnss_fgo --ros-args \
  --params-file config/gnss_fgo.yaml
```

| Direction | Default | Type |
|---|---|---|
| Sub | `/gnss/observation` | `GnssObservations` |
| Sub | `/base/gnss/observation` | `GnssObservations` |
| Sub | `/gnss/ephemeris` | `GnssEphemerides` |
| Pub | `/gnss/fgo/solution` | `GnssSolution` |

It publishes **no** solution on an epoch with no double differences.
Set `ambiguity.resolution: false` for a **FLOAT-only** run.

## `gnss_imu_fgo` — GNSS + IMU

<p align="center"><img src="../../fig/gnss_imu_fgo_graph.svg" alt="gnss_imu_fgo factor graph" width="720"></p>

Adds GTSAM IMU preintegration (`CombinedImuFactor`, which models bias
random-walk evolution and the bias/measurement cross-correlation); the DD factors
attach to the body pose through the "Arm" factor variants (lever arm +
`ecef_T_nav`). The platform is assumed static for `init.imu_duration` seconds to
initialize attitude; yaw converges once it moves.

```bash
ros2 run gnss_ros_standardization gnss_imu_fgo --ros-args \
  --params-file config/gnss_imu_fgo.yaml
```

| Direction | Default | Type |
|---|---|---|
| Sub | `/gnss/observation` | `GnssObservations` |
| Sub | `/base/gnss/observation` | `GnssObservations` |
| Sub | `/gnss/ephemeris` | `GnssEphemerides` |
| Sub | `/gnss/imu/data_raw` | `sensor_msgs/Imu` |
| Pub | `/gnss/imu_fgo/solution` (+ `_odom`) | `GnssSolution` / `nav_msgs/Odometry` |

`GnssSolution` reports the **antenna** phase centre; `Odometry` reports the
**body** pose (FLU, REP-105). Both describe the same state, so mixing them
without applying `lever_arm` introduces a lever-arm-sized error.

### Vehicle-motion constraints

This example excludes wheel odometry, so it recovers most of that information
from pure-kinematic pseudo-measurements, all stock GTSAM factors in
[imu_factors.hpp](imu_factors.hpp).

| Constraint | Default | What it says |
|---|---|---|
| `attitude.velocity_aiding` | **on** | body +x is parallel to the Doppler velocity. The only constraint tying heading to direction of travel |
| `zupt.enable` | **on** | velocity is exactly zero while stationary. Evidence-gated by `zupt.max_speed_mps`, since the IMU alone cannot tell rest from traffic creep |
| `nhc.enable` | off | body-lateral and vertical velocity are ≈ 0. Gated to straight driving by `nhc.max_yaw_rate_rps`, where the `ω × r` term vanishes and the constraint is exact at the IMU regardless of the lever |

> **Platform assumption.** The attitude aiding is **on by default** and requires a
> **wheeled vehicle**: small side-slip, body +x along the direction of travel, no
> sustained reverse. Set `attitude.velocity_aiding: false` for aircraft, boats or
> handheld use, where the velocity direction says nothing about where the body
> points.

## Parameter reference

Keys marked **IMU** exist only in `gnss_imu_fgo`.

### Topics and frames

| Key | Default | Meaning |
|---|---|---|
| `topics.rover_observation` | `/gnss/observation` | rover observation input |
| `topics.base_observation` | `/base/gnss/observation` | base observation input |
| `topics.ephemeris` | `/gnss/ephemeris` | broadcast ephemeris input |
| `topics.solution` | `/gnss/fgo/solution` | solution output |
| `topics.imu` **IMU** | `/gnss/imu/data_raw` | IMU input |
| `base_position.postype` / `.pos` | `llh` / `[0,0,0]` | base antenna position; `llh` is deg/deg/m, `xyz` is ECEF metres |
| `fixed_origin.postype` / `.pos` | `llh` / `[0,0,0]` | ENU origin for the published local frame |
| `local_origin.mode` **IMU** | `gnss_fix` | ENU origin source: `gnss_fix` or `fixed` |
| `lever_arm` **IMU** | `[0,0,0]` | IMU body (FLU) to antenna, metres |

### Constellations, signals and masks

| Key | Default | Meaning |
|---|---|---|
| `navsys.gps` / `.gal` / `.bds` / `.qzs` | `true` | use this constellation |
| `navsys.glo` | `false` | use GLONASS |
| `navsys.glo_undifferenced_only` | `false` | keep GLONASS for the code solution but exclude it from AR |
| `frequencies.enable_l1` / `.enable_l2` | `true` | use this band |
| `frequencies.enable_l5` | `false` | use L5 / E5a / B2a |
| `frequencies.cross_code_pairing` | `true` | allow rover and base to use different tracking codes on a band |
| `masks.elevation_deg` | 15.0 | elevation mask, degrees |
| `masks.snr_dbhz` | 0.0 | carrier-to-noise mask, dB-Hz; 0 disables |
| `excluded_satellites` | `[]` | satellite IDs to drop, e.g. `["G05"]` |

### Measurement model and noise

| Key | Default | Meaning |
|---|---|---|
| `noise.pr_sigma_m` | 0.5 | pseudorange sigma, metres (zenith; elevation-weighted) |
| `noise.cp_sigma_m` | 0.005 | carrier-phase sigma, metres |
| `noise.pr_innov_gate_m` | 0.0 | pre-fit pseudorange innovation gate, metres; 0 disables |
| `noise.doppler_max_res_m` | 2.0 | epoch-level Doppler residual rejection, m/s |
| `noise.doppler_max_nsigma` | 4.0 | per-satellite Baarda w-test in the Doppler solve; 0 disables |
| `noise.doppler_sigma_mps` | 0.1 | a-priori zenith range-rate sigma, m/s |
| `noise.doppler_corr_time_s` **IMU** | 2.0 | Doppler error correlation time, seconds |
| `measurement.base_correlation_model` | `variance_inflation` | covariance treatment for a reused base epoch |
| `measurement.base_reuse_factor` | 1 | rover epochs sharing one base observation (use ~5 for 1 Hz base / 5 Hz rover) |
| `motion.accel_sigma_mps2` | 3.0 | motion-factor process noise, m/s² |
| `max_age_s` | 5.0 | oldest base epoch accepted for a DD, seconds. Also the epoch matcher's pairing window |
| `cycle_slip.gf_threshold_m` | 0.05 | geometry-free slip threshold, metres |
| `cycle_slip.max_gap_s` | 2.0 | carrier gap that forces a slip, seconds |

### Ambiguity resolution

| Key | Default | Meaning |
|---|---|---|
| `ambiguity.resolution` | `true` | resolve integers; `false` publishes FLOAT only |
| `ambiguity.ratio_threshold` | 3.0 | LAMBDA ratio test threshold |
| `ambiguity.min_fix` | 4 | minimum ambiguities to attempt a fix |
| `ambiguity.min_lock` | 5 | epochs an arc is carried before it may be fixed |
| `ambiguity.min_fix_to_hold` | 10 | consecutive fixes before an integer is held |
| `ambiguity.hold_refresh_s` | 30.0 | re-key an arc at this age; 0 never |
| `ambiguity.max_outage_s` | 5.0 | carrier outage after which arcs are dropped, seconds |
| `ambiguity.max_pos_var_m2` | 0.25 | skip AR above this float position variance, m² |
| `ambiguity.partial_max_drop` | 5 | subset-retry budget for partial AR |
| `ambiguity.fde_max_exclude` | 2 | maximum DDs the pre-fit test may exclude per epoch |

### IMU, attitude and vehicle constraints (`gnss_imu_fgo` only)

| Key | Default | Meaning |
|---|---|---|
| `imu.sigma_acc` / `.sigma_gyr` | 0.3 / 0.01 | accel / gyro noise density, m/s²/√Hz and rad/s/√Hz |
| `imu.sigma_acc_bias` / `.sigma_gyr_bias` | 1e-5 / 1e-6 | bias random walk, m/s²/√s and rad/s/√s |
| `imu.sigma_integration` | 0.01 | preintegration position-integration noise |
| `imu.max_wait_s` | 0.2 | how long an epoch waits for IMU coverage, seconds |
| `imu.queue_depth` | 2000 | IMU subscription queue depth |
| `attitude.velocity_aiding` | `true` | constrain heading to the direction of travel |
| `attitude.min_speed_mps` | 3.0 | speed above which velocity aiding applies, m/s |
| `attitude.sideslip_deg` | 2.0 | assumed side-slip, degrees |
| `attitude.max_misalign_deg` | 90.0 | heading disagreement above which aiding is refused |
| `nhc.enable` | `false` | non-holonomic constraint |
| `nhc.sigma_lat_mps` / `.sigma_vert_mps` | 0.3 / 0.2 | lateral / vertical constraint sigma, m/s |
| `nhc.min_speed_mps` | 0.0 | speed above which NHC applies, m/s |
| `nhc.max_yaw_rate_rps` | 0.02 | yaw rate above which NHC is skipped, rad/s |
| `nhc.lever_frd` | `[0,0,0]` | IMU to constraint point (FRD), metres |
| `zupt.enable` | `true` | zero-velocity update at standstill |
| `zupt.max_speed_mps` | 1.0 | speed below which ZUPT may apply, m/s |
| `zupt.max_acc_std` / `.max_gyr_std` | 0.55 / 0.030 | stillness thresholds, m/s² and rad/s |
| `zupt.max_gyr_median` | 0.020 | stillness median gyro threshold, rad/s |
| `zupt.min_samples` | 5 | IMU samples required to declare stillness |
| `zupt.sigma_mps` / `.sigma_vert_mps` | 0.5 / 0.5 | horizontal / vertical ZUPT sigma, m/s |
| `gap.max_coast_s` | 30.0 | longest dead-reckoned stretch during a GNSS outage, seconds; 0 disables the bound |

### Solver and debug

| Key | Default | Meaning |
|---|---|---|
| `graph.lag_s` | 25.0 | fixed-lag smoother window, seconds; <= 0 keeps full history |
| `time.gnss_epoch_source` **IMU** | `header` | `header` or `tow_auto_offset` |
| `debug.ar_dump_dir` | `""` | AR diagnostic CSV directory; empty disables |
