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
