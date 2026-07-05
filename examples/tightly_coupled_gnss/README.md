# `gnss_fgo` — Tightly-coupled GNSS factor-graph optimization

RTK-style positioning as incremental factor-graph optimization: raw
double-difference (DD) pseudorange and carrier-phase measurements are fed
directly into [GTSAM](https://gtsam.org)'s official GNSS factors
([introduced June 2026](https://gtsam.org/2026/06/10/rtk-gnss-double-difference.html))
and optimized with ISAM2. Integer ambiguities are resolved with RTKLIB's
`lambda()` (vendored in this repository), producing **FIX** solutions.

The point of this example: **this package already does all the GNSS-domain
work** — the actual tight coupling is ~30 lines of factor construction
([factor_adapters.hpp](../fgo_common/factor_adapters.hpp)).

```
receiver (any brand) / rosbag
        │ driver / decoder
        ▼
/gnss/observation   /base/gnss/observation   /gnss/ephemeris
        │                   │                     │
        ▼                   ▼                     ▼
   GnssPreprocessor  ──  this package, no GTSAM dependency
   · rover/base epoch matching        · satellite pos/clock at transmission
   · elevation/SNR/system masks         time (rover AND base epochs)
   · reference satellite selection    · DD pairing, cycles → meters
        │
        ▼  PreprocessedEpoch (DdSignal = one factor's arguments)
   GTSAM factor construction  ──  this example
   · DoubleDifferencePseudorangeFactor    (key: rover Point3, ECEF)
   · DoubleDifferenceCarrierPhaseFactor   (keys: position + 2 ambiguities)
        │
        ▼
   ISAM2 incremental optimization ── LAMBDA integer ambiguity resolution
        │
        ▼
   /gnss/fgo/solution (GnssSolution, STATUS_FIX / STATUS_FLOAT)
```

## Building

### 1. Build GTSAM (develop branch)

The GNSS factors are not yet in a GTSAM release tag, so build from source:

```bash
git clone https://github.com/borglab/gtsam.git && cd gtsam
git checkout develop
cmake -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_PYTHON=OFF
cmake --build build -j$(nproc)
sudo cmake --install build       # or -DCMAKE_INSTALL_PREFIX=$HOME/gtsam-install
```

`-DGTSAM_USE_SYSTEM_EIGEN=ON` and `-DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF` are
**required**: this package compiles against the system Eigen, and mixing
GTSAM's bundled Eigen or native alignment flags causes silent memory
corruption at the ABI boundary.

### 2. Build the examples (opt-in)

```bash
colcon build --cmake-args -DBUILD_GTSAM_FGO_EXAMPLES=ON
# with a non-system GTSAM prefix:
colcon build --cmake-args -DBUILD_GTSAM_FGO_EXAMPLES=ON \
  -DGTSAM_DIR=$HOME/gtsam-install/lib/cmake/GTSAM
```

The default build (`BUILD_GTSAM_FGO_EXAMPLES=OFF`) does not need GTSAM at all.

## Running

Set the base station position in
[config/gnss_fgo.yaml](../../config/gnss_fgo.yaml) (required — an error here
translates the whole trajectory), then:

```bash
ros2 run gnss_ros_standardization gnss_fgo --ros-args \
  --params-file config/gnss_fgo.yaml
```

Feed it from any source that publishes the standardized topics:

- **Live**: run any driver/decoder for the rover, and a second one (e.g.
  `rtcm_decoder_node` with an NTRIP caster) remapped to
  `/base/gnss/observation` for the base — exactly like the
  [`real_time_kinematic`](../real_time_kinematic/) example.
- **Rosbag**: play back a recording of the same topics.
- **Public RINEX data** (e.g. [PPC-Dataset](https://github.com/taroz/PPC-Dataset)
  or two nearby CORS stations): convert to bags with `rinex_to_rosbag`,
  remapping the base bag's topics.

### Topics

| Direction | Default | Type |
|---|---|---|
| Sub | `topics.rover_observation` (`/gnss/observation`) | `GnssObservations` |
| Sub | `topics.base_observation` (`/base/gnss/observation`) | `GnssObservations` |
| Sub | `topics.ephemeris` (`/gnss/ephemeris`) | `GnssEphemerides` |
| Pub | `topics.solution` (`/gnss/fgo/solution`) | `GnssSolution` |

The solution topic deliberately differs from the RTK node's
`/gnss/solution`, so both can run side by side on the same data:

```bash
ros2 run gnss_ros_standardization real_time_kinematic --ros-args \
  --params-file config/real_time_kinematic.yaml &
ros2 run gnss_ros_standardization gnss_fgo --ros-args \
  --params-file config/gnss_fgo.yaml
# record both, then compare:
ros2 run gnss_ros_standardization rosbag_to_pos --bag out --topic /gnss/solution     --out rtk.pos
ros2 run gnss_ros_standardization rosbag_to_pos --bag out --topic /gnss/fgo/solution --out fgo.pos
```

## Graph structure

The rover **position is continuous across epochs** (a constant-velocity motion
model), while the carrier **ambiguities are per-epoch / instantaneous**. The
velocity that drives the motion model is **estimated in the preprocessor** (from
raw Doppler) and is **not a graph state** — the only states are positions and
ambiguities. This mirrors how RTKLIB works in *every* `armode`: the EKF always
propagates the position (`dynamics`), with the velocity as an input; only the
ambiguity *resolution* is per-epoch in instantaneous mode. See
[the factor-graph diagram](../../fig/gnss_fgo_graph.svg).

```
 X_{k-1} ──[motion: X_k − X_{k-1} = ½(v_{k-1}+v_k)·dt]── X_k (Point3, ECEF)
                                                          │
                        [DD pseudorange]──┤├──[DD carrier]── B_k^1 .. B_k^j
                                                          (fresh this epoch)
```

- **States**: `X(k)` rover ECEF position (`gtsam::Point3`); carrier ambiguities
  `B(k)^i` in cycles, **fresh every epoch**.
- **Motion factor** (the key to fixing): a built-in `BetweenFactor<Point3>` ties
  `X(k)` to `X(k-1)` by the **Doppler-derived displacement** `½(v_{k-1}+v_k)·dt`.
  The rover velocity `v` is estimated from the raw Doppler in the preprocessor
  (undifferenced range-rate least squares, RTKLIB `estvel`-style). This smooths
  the float position to **decimetre** level (vs metre for single-epoch code), so
  the ambiguities land within ~1 cycle of truth. The first epoch / after a gap is
  anchored instead by a loose `PriorFactor<Point3>` (`position_prior.std_m`).
- **DD factors (float trajectory)**: GTSAM's official
  `DoubleDifferencePseudorangeFactor` / `DoubleDifferenceCarrierPhaseFactor`. The
  carrier factor uses `(N_ref, N_tar)` keys; the per-group reference gets a tight
  prior (`ambiguity.ref_prior_sigma_cycles`) that fixes the 1-D gauge, targets a
  loose prior at their single-difference code-minus-carrier estimate. Noise is
  derived from **undifferenced** sigmas (`noise.{pr,cp}_sigma_m`) by error
  propagation, so the float factors and the AR covariance share one stochastic
  model. Robust Huber noise (`robust.*`) down-weights multipath.
- **Ambiguity resolution** runs a **dedicated, textbook joint estimator** — *not*
  the ISAM2 graph posterior. Every DD in a (system, band) group shares the
  reference satellite, so the correctly-correlated DD covariance (RTKLIB `ddcov`)
  has off-diagonal `Var(SD_ref)`; GTSAM's independent per-pair factor noise would
  give the diagonal-only shape and an **uncalibrated** ratio test. So AR solves
  its own small per-epoch system (motion-predicted position prior + this epoch's
  correlated DDs) and reads the joint `Q_xx / Q_xa / Q_a` from one inverse — the
  rigorous LAMBDA construction. Reading them from the graph would *reintroduce*
  the uncalibrated covariance, so the split is deliberate. LDLT throughout, with
  **signed-pivot** positive-definite and conditioning guards (a negative pivot is
  an indefinite, degenerate epoch — rejected, never published). Then the
  **canonical LAMBDA + ratio test** (RTKLIB `resamb_LAMBDA`): cycle-slipped /
  half-cycle carriers and pairs below `el_mask_deg` are excluded up front, one
  `lambda()` solve over the remaining ≥ `min_fix` DDs, accept if the ratio ≥
  threshold. The accepted integers condition the published position
  `x_fix = x − Q_xa Q_a⁻¹(a − ǎ)`; the graph stays float, so a wrong fix never
  corrupts the trajectory.
- **Fault detection & exclusion** (`ambiguity_resolution.fde_*`): after fixing,
  the post-fix carrier residuals `r_i = obs_i − geom_i(x_fix) − λ_i ǎ_i` are
  checked. A correct fix leaves cm-level residuals; a wrong integer on one DD
  (undetected multipath / half-cycle) leaves `≥ λ/2 ≈ 0.1 m`. If the worst
  exceeds `fde_threshold_m`, that satellite is excluded and the epoch re-fixed,
  up to `fde_max_exclude` times. A fix is published only with **clean residuals
  AND** a passing ratio, so removing an outlier can **salvage a float** (the
  outlier had spoiled the ratio) and a **wrong fix is rejected** — the fix rate
  rises while the fix error stays cm-level. (This residual test is the right tool
  for the precise-but-wrong outliers that a success-rate / partial-AR criterion
  cannot catch.)
- **Masks** (shared with the [`real_time_kinematic`](../real_time_kinematic/)
  example): an elevation mask plus an optional **elevation-dependent SNR mask**
  (`masks.snrmask`, RTKLIB `testsnr` — nine per-band thresholds at 5…85°). With
  `masks.snrmask.enable: false` the flat `masks.snr_dbhz` threshold is used.

## Performance

On an open-sky multi-GNSS (GPS/GAL/BDS/QZS) L1+L2 rover/base field recording,
errors vs an RTKLIB fix-and-hold reference:

| | Fix rate | FIX 3D-RMS | FIX 3D-max | all-epoch 3D-RMS |
|---|---|---|---|---|
| RTKLIB instantaneous (`armode=2`) | 27 % | 1.1 cm | 12 cm | 0.64 m |
| **this example** | **74 %** | **2.0 cm** | **7 cm** | **0.28 m** |

The high, correct fix rate comes from the Doppler-aided motion model (a far
better float) and the correlated DD covariance (a calibrated ratio test).
`min_fix` is the main quality knob — a larger required fixed subset means
stronger geometry and a more accurate (especially vertical) fix, at a slightly
lower fix rate.

## Limitations / future work

- **Per-epoch (instantaneous) ambiguities** — integers are not carried across
  epochs (no fix-and-hold); only the position is carried across epochs (by the
  motion model), with the Doppler velocity as its input.
- **AR outlier handling** — the float graph factors are Huber-robust, and
  carrier outliers are caught *after* fixing by the post-fix residual FDE above.
  The AR *float* solve itself is still plain weighted least-squares (no robust
  kernel), so a gross pseudorange outlier could bias the float position before
  FDE runs; an IRLS/Huber float solve is future work.
- **Two-estimator AR** — AR keeps its own correctly-correlated estimator instead
  of the graph posterior (see *Graph structure*). Fully unifying them would need a
  correlation-aware grouped DD factor (full `R_DD` per (system, band)) in the
  graph; until then the published FLOAT (graph) and FIX (AR) come from different
  solvers.
- **No ionosphere / troposphere states** — short-baseline assumption.
- **Mismatched-code DD pairs are dropped** — when rover and base track different
  signals on a band with no code in common (e.g. rover L2L only, base L2W only),
  the pair is excluded because the inter-signal / differential code bias
  (ISB/DCB) is not corrected. RTKLIB uses such pairs *with* an ISB correction;
  adding one is future work. (The shared converter already selects, per band, the
  highest-priority code common to both receivers, RTKLIB `getcodepri`-style, so
  only truly disjoint code sets are lost.)
- **ISAM2 grows with session length** — a production node would use
  `gtsam::IncrementalFixedLagSmoother`.
- **GLONASS** excluded by default (FDMA inter-frequency phase biases do not
  cancel across heterogeneous receivers).

The whole chain (preprocessor → factors → motion → correlated-covariance AR →
LAMBDA) is validated on synthetic multi-GNSS dual-frequency data: the recovered
integers match the ground-truth ambiguities exactly and the FIX trajectory is
centimetre-accurate.
