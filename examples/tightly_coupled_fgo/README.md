# Tightly-coupled GNSS / GNSS-IMU factor-graph optimization

RTK-style positioning as incremental factor-graph optimization. Raw
double-difference (DD) pseudorange and carrier-phase measurements are fed
directly into [GTSAM](https://gtsam.org)'s official GNSS factors
([introduced June 2026](https://gtsam.org/2026/06/10/rtk-gnss-double-difference.html))
and optimized with ISAM2; integer ambiguities are resolved with RTKLIB's
`lambda()` (LAMBDA) and a ratio test, producing **FIX** solutions.

`GnssPreprocessor` (this package, no GTSAM dependency) does all the GNSS-domain
work — rover/base epoch matching, satellite positions, masks, DD pairing — so the
actual tight coupling is ~30 lines of factor construction in
[factor_adapters.hpp](factor_adapters.hpp). The two nodes here share it:
`gnss_fgo` (GNSS only) and `gnss_imu_fgo` (adds IMU preintegration).

The opt-in GTSAM build is described in the
[repository README](../../README.md#optional-tightly-coupled-fgo-examples-gtsam).
Set the base-station position in the YAML — it is **required**; an error there
translates the whole estimated trajectory.

### Ambiguity mode (`ambiguity.mode`)

How carrier ambiguities are treated over time (mirrors RTKLIB's `armode`) — the
main accuracy lever. All three modes resolve integers with the same single-epoch
analytical AR (`resolveAmbiguitiesDd`); they differ only in whether the ambiguity
**keys** persist across epochs, which sets how good the **float** gets.

| Mode | Ambiguities | Fix feedback | Accuracy / risk |
|---|---|---|---|
| `instantaneous` (default) | fresh per epoch | none | float ≈ metre; instantaneous fixes only. Conservative. |
| `continuous` | carried across epochs, re-keyed on cycle slip / outage | none (published only) | float → decimetre/cm, reliable fixes. |
| `fix_and_hold` | carried | accepted integer DDs injected as tight gauge-free relative constraints (`BetweenFactor` on N_ref−N_tar) | held fixes stay cm until the next slip; a wrong fix corrupts the graph until re-keyed. |

The carried modes rely on cycle-slip detection (`cycle_slip.*`) to protect the
persistent keys, so dual-frequency (L1+L2) is recommended whenever one is used.

## `gnss_fgo` — GNSS only

<p align="center"><img src="../../fig/gnss_fgo_graph.svg" alt="gnss_fgo factor graph" width="720"></p>

Carrier ambiguities follow `ambiguity.mode` (default `instantaneous`; see above);
the rover position is carried across epochs by a Doppler-derived motion factor,
which sharpens the float so LAMBDA can fix.

```bash
ros2 run gnss_ros_standardization gnss_fgo --ros-args \
  --params-file config/gnss_fgo.yaml
```

| Direction | Default topic | Type |
|---|---|---|
| Sub | `/gnss/observation` | `GnssObservations` |
| Sub | `/base/gnss/observation` | `GnssObservations` |
| Sub | `/gnss/ephemeris` | `GnssEphemerides` |
| Pub | `/gnss/fgo/solution` | `GnssSolution` |

## `gnss_imu_fgo` — GNSS + IMU

<p align="center"><img src="../../fig/gnss_imu_fgo_graph.svg" alt="gnss_imu_fgo factor graph" width="720"></p>

Adds GTSAM IMU preintegration (`CombinedImuFactor`); the DD factors attach to the
body pose through the "Arm" factor variants (lever arm + `ecef_T_nav`). The
platform is assumed static for `init.imu_duration` seconds to initialize attitude
(yaw converges once it moves).

```bash
ros2 run gnss_ros_standardization gnss_imu_fgo --ros-args \
  --params-file config/gnss_imu_fgo.yaml
```

| Direction | Default topic | Type |
|---|---|---|
| Sub | `/gnss/observation` | `GnssObservations` |
| Sub | `/base/gnss/observation` | `GnssObservations` |
| Sub | `/gnss/ephemeris` | `GnssEphemerides` |
| Sub | `/gnss/imu/data_raw` | `sensor_msgs/Imu` |
| Pub | `/gnss/fgo/solution` (+ `_odom` full pose/attitude) | `GnssSolution` / `nav_msgs/Odometry` |

## Scope & limitations

These are **educational / research reference implementations** of an RTK-style
factor graph — a clear, readable mapping from raw DD observations onto GTSAM's
official GNSS factors, resolved with RTKLIB's `lambda()`. They are **not** a
claim of full RTKLIB compatibility or a production-grade RTK engine, and they do
not guarantee cm-level accuracy across arbitrary receivers or baseline lengths.

- **Graph growth.** Plain `ISAM2` retains the full epoch history, so memory and
  marginal-computation cost grow without bound. This is fine for an example or
  an hours-scale session; a production node should use
  `gtsam::IncrementalFixedLagSmoother` (or similar) instead. A fixed-lag variant
  of these examples may be added separately in the future.
- **FLOAT graph DD correlation.** GTSAM's scalar DD factors use independent
  per-pair noise, so the FLOAT graph's marginal covariance does not fully
  capture the correlation that DD pairs sharing a reference satellite actually
  have. The analytical AR (`resolveAmbiguitiesDd`) uses the correctly-correlated
  RTKLIB-style covariance for its ratio test regardless, but a fully-calibrated
  joint-marginal AR from the graph itself would need a correlated vector DD
  factor.
- **Corrections not modelled.** Satellite/receiver TGD/DCB and other
  inter-code biases, baseline-length-dependent tropospheric/ionospheric models,
  GLONASS inter-frequency bias, antenna phase-center variation, and a base-side
  SNR mask (only the rover stream is masked) are all out of scope. These
  examples target short baselines with synchronized-enough rover/base streams,
  primarily GPS/Galileo/BeiDou/QZSS.
- **Mode guidance.** `continuous` is a reasonable default for normal operation
  (good balance of reliability and accuracy); `fix_and_hold` gives the highest
  accuracy but a wrong fix pollutes the graph until the satellite is re-keyed;
  `instantaneous` is the conservative fallback / comparison baseline.
- Any accuracy figures quoted against a receiver's own NMEA/RTK output should be
  read as **an external reference solution**, not ground truth — it comes from
  the same antenna/receiver and, if compared only over its own FIX intervals,
  carries a selection bias. Surveyed static points or independent post-processed
  (PPK) or independent-receiver solutions are preferable where available.
