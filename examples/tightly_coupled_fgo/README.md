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
is recommended. CMC only initializes a new ambiguity value; one data-independent
gauge constraint per connected component makes the graph observable, so the CMC
initializer cannot act as an extra measurement.

## Real-time

The estimator is a `gtsam::IncrementalFixedLagSmoother` — ISAM2 plus
marginalization of variables older than `graph.lag_s`. Marginalizing a position
folds its ambiguity-constraining information into a prior, so the carrier keeps
accumulating and the current-epoch AR is unchanged. The retained graph uses
**QR** factorization, not Cholesky, which squares the condition number of the
ill-conditioned carrier graph. `graph.lag_s <= 0` keeps full history (offline
only; per-epoch cost then grows without bound).

**`graph.lag_s` is an accuracy-versus-latency knob, and the latency half is
steep.** Solve cost is near-linear in the number of poses in the window: at a
fixed DD count, ISAM2's `update()` costs 12 ms at 6 % window occupancy and
237 ms at 100 %. Past the epoch period the node cannot recover — one dense-sky
stretch builds a backlog lasting minutes. Measured on `gnss_imu_fgo`,
PPC nagoya_run1:

| `graph.lag_s` | fix | p05 | epochs solved | latency median | p90 | p99 |
|---|--:|--:|--:|--:|--:|--:|
| 25 (default) | 60.35 % | 71.74 | **7596** | 0.41 s | **0.58 s** | **1.89 s** |
| 40 | 62.18 % | 73.01 | 7520 | 0.47 s | 32.4 s | **58.8 s** |

The extra 1.8 points of fix rate costs 76 epochs that never get a solution and
puts more than a tenth of the output over half a minute late — while the median
stays healthy, so only the upper percentiles reveal it. Raise it only if your
application genuinely prefers a late, better answer.

`gnss_imu_fgo` runs observation intake, IMU intake and the solver on three
separate callback groups, with the solve on a timer and a bounded number of
epochs per tick. Intake therefore keeps recording arrivals during a solve, which
is what lets the node tell *"no observation exists"* from *"the observation is
queued behind me"*. Gaps are bridged from two independent triggers, so
availability does not depend on the node keeping up: the rover going silent in
**arrival**, and a hole in the **emitted epoch stream**. Synthesized slots are
published in time order ahead of the epoch that revealed the hole, and never take
a slot already published.

Run each node alone — concurrent nodes contend for the CPU, and a node that falls
behind loses base epochs to the matcher queue.

## AR acceptance settings

Lowering these trades integrity for FIX rate; record them with any result.

| Setting | Value | Meaning |
|---|---|---|
| `ambiguity.ratio_threshold` | 3.0 | LAMBDA ratio test |
| `ambiguity.min_lock` | 5 | epochs an arc is carried before it may be fixed |
| `ambiguity.min_fix_to_hold` | 10 | consecutive fixes before an integer is held |
| `ambiguity.max_pos_var_m2` | 0.25 m² | skip AR while the float has not converged |
| `ambiguity.hold_refresh_s` | 30 s | re-key an arc at this age, so an unverified hold cannot persist |
| `fde_nsigma` / `fde_max_exclude` | 4 / 2 | pre-fit innovation test and its exclusion budget |
| `hold_sigma_cycles` | 0.03 | held integer constraint, applied once per arc |
| escape gate 1 | 0.08 m / 5 epochs | reject a fix whose code DDs disagree with the integer correction |
| escape gate 2 (IMU) | 2.5 m / 3 epochs | reject when the float itself sits where the code says the antenna is not |
| Huber kernel | k = 1.345, per group | one weight per grouped DD factor |
| GLONASS carrier DD | off | FDMA biases do not cancel across heterogeneous receivers |

Post-fit FIX validation evaluates residuals at the integer-conditioned position,
which was estimated from the rows being tested, so the covariance shrinks below
the measurement noise and is rank-deficient by construction. It is inverted on
its column space and compared against a χ² quantile at that rank, and fails
closed if the robust weights the graph applied cannot be reproduced.

## Inspecting the ambiguity resolution

Set `debug.ar_dump_dir` (empty = off) and both nodes write, per epoch, the state
LAMBDA decided on: float antenna position and covariance, `a_float`, `Qa`, the
ratio and the integers. `gnss_imu_fgo` also writes IMU diagnostics and solve
times. Leave it empty in production — the files grow without bound.

A wrong fix has two causes the ratio test cannot separate, since it is computed
from the same covariance: a **biased float** or an **over-confident covariance**.
With a truth trajectory they do separate — recover each DD's true integer, form
`eps = a_float − N_true`, and test `eps' Qa⁻¹ eps` against χ². A median `chi2/k`
near 1 means `Qa` is honest.

Every position in the dump except two is a function of the graph state, so a
wrong fix is self-consistent with all of them. Two columns break out of that
loop because no carrier ambiguity enters either:

| Column | Source |
|---|---|
| `spp_*` | standalone single-point fix |
| `ddwls_*` | code double-difference WLS (`solveCodeDdWls`), fed the **raw** epoch |

`ddwls` is the tighter yardstick: differencing against the base cancels
ionosphere, troposphere, orbit and clock error. **`rover_ecef_apriori` is NOT
independent** — both nodes pass their own last estimate to `drainEpochs`, so it
returns as a copy of the previous graph float.

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

It publishes **no** solution on an epoch with no double differences: this is a
purely relative method, so "no DD" means "no solution", and emitting the code-only
SPP instead would mislead a consumer into treating a metre-level absolute fix as
the FGO output. Availability is therefore reported against the reference epoch
count, not flattered by a fallback.

Set `ambiguity.resolution: false` for a **FLOAT-only** run: LAMBDA never runs and
no integer is held. The float carries the position whenever AR fails, so scoring
it on its own is the honest way to compare estimators. The same switch exists on
`gnss_imu_fgo`; `real_time_kinematic` has `pos2.armode: 0`.

## `gnss_imu_fgo` — GNSS + IMU (experimental)

<p align="center"><img src="../../fig/gnss_imu_fgo_graph.svg" alt="gnss_imu_fgo factor graph" width="720"></p>

Adds GTSAM IMU preintegration (`CombinedImuFactor`, which models bias
random-walk evolution and the bias/measurement cross-correlation); the DD factors
attach to the body pose through the "Arm" factor variants (lever arm +
`ecef_T_nav`). The platform is assumed static for `init.imu_duration` seconds to
initialize attitude; yaw converges once it moves.

`imu.sigma_acc` / `imu.sigma_gyr` are deliberately **inflated** over the sensor
datasheet. That is standard for a road vehicle: the effective process noise must
absorb vibration and unmodeled dynamics, and a tightly-coupled solve is
destabilized by an over-confident IMU. Only the bias random walks are
datasheet-derived.

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

### Vehicle-motion constraints

This example excludes wheel odometry, so it recovers most of that information
from pure-kinematic pseudo-measurements, all stock GTSAM factors in
[imu_factors.hpp](imu_factors.hpp).

| Constraint | Default | What it says |
|---|---|---|
| `attitude.velocity_aiding` | **on** | body +x is parallel to the Doppler velocity. The only constraint tying heading to direction of travel |
| `zupt.enable` | **on** | velocity is exactly zero while stationary. Evidence-gated by `zupt.max_speed_mps`, since the IMU alone cannot tell rest from traffic creep |
| `nhc.enable` | off | body-lateral and vertical velocity are ≈ 0. Gated to straight driving by `nhc.max_yaw_rate_rps`, where the `ω × r` term vanishes and the constraint is exact at the IMU regardless of the lever |

> **Platform assumption.** The attitude aiding requires a **wheeled vehicle**:
> small side-slip, body +x along the direction of travel, no sustained reverse.
> Set `attitude.velocity_aiding: false` for aircraft, boats or handheld use.

> **Prefer the attitude aiding over the NHC.** The NHC states the same physics as
> a *velocity* constraint, so it collides with ZUPT at a standstill and with the
> Doppler prior while moving, and makes the velocity over-confident. The attitude
> factor carries no velocity information, so it cannot do that.

The first heading of a run is set by `alignYawToCourse`, which replaces yaw
outright from the course over ground once several consecutive candidates agree on
the *offset* `course − yaw`. Voting on the offset keeps the test usable while
manoeuvring. A reversing vehicle reads exactly 180° repeatably, so an agreed
offset beyond 90° is applied only at or above 3 m/s.

### Before relying on it

**Experimental.** It carries extra pose / velocity / bias states, emits no
solution until the static IMU window is complete, and its per-epoch solve is
heavier. A poor attitude init or a wrong lever arm biases every DD factor and can
produce false fixes. Set **`lever_arm`** to the real body→antenna offset, verify
the IMU axis / scale / bias, and confirm time sync.

With `time.gnss_epoch_source: tow_auto_offset` the node estimates the GNSS↔IMU
clock offset as a sliding-window **minimum** of `stamp − utc(week,tow)`. That
strips the variable part of the receiver latency but leaves the constant minimum
in place, which cannot be observed from the stream alone. For tight sync feed a
hardware-synchronized time (PPS/PTP) or a measured `latency`.

Gap dead-reckoning reports an approximate covariance (last posterior plus
preintegration noise and a velocity-over-horizon term), so it is a lower bound
that grows with the outage — adequate for short bridging, not an INS covariance
engine.

`GnssSolution` reports the **antenna** phase centre; `Odometry` reports the
**body** pose (FLU, REP-105). At a FIX both are the same integer-conditioned
state, so `antenna(Odometry pose) == GnssSolution.pos` by construction. Because
`Odometry` has no status field its pose steps at a FIX↔FLOAT transition, with the
covariance tightening accordingly.

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
