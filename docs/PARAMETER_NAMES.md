# Parameter naming

Every node follows the same naming rules, in code, YAML and launch. There are no
aliases: a key that a node does not declare is ignored by ROS 2 (`gnss_fgo` and
`gnss_imu_fgo` stop at startup instead). `test/test_parameter_names.py` checks
that every key of the shipped YAML files and every parameter the launch files pass
is declared by its node.

## Rules

| Rule | Examples |
|---|---|
| Topics are `topics.<stream>` | `topics.observation`, `topics.ephemeris`, `topics.solution`, `topics.imu`, `topics.imu_raw`, `topics.rover_observation`, `topics.base_observation`, `topics.gnss_solution`, `topics.wheel_speed` |
| A physical quantity carries its unit | `_s`, `_hz`, `_deg`, `_m`, `_mps`, `_mps2`, `_rps` (rad/s): `rate_hz`, `zupt.window_s`, `wheel.sigma_mps` |
| A feature switch is `<group>.enable` | `zupt.enable`, `leveling.enable`, `wheel.enable`, `wheel.nhc.enable`, `rtcm_input.enable`, `snrmask.enable` |
| Related settings share a group | `wheel.*`, `init.*`, `gnss.*`, `zupt.*`, `leveling.*`, `ekf.*` |

Kept on purpose:
- `real_time_kinematic` uses RTKLIB's configuration names (`pos1.*`, `pos2.*`,
  `rtk_stats.*`, `ant2.*`), so an RTKLIB setting maps one to one.
- Receiver drivers name their output messages after the receiver protocol (`messages.*`).
- The FGO nodes use the RTK names (`frequencies.*`, `pos1.*`, `pos2.*`, `excluded_satellites`) for the settings they share with `real_time_kinematic`; `pos2.armode` takes only `0` and `3`, and a satellite with a masked or missing band is not used ([FGO](../src/positioning/fgo/README.md#parameters)).

## ENU origin

Every node that publishes a local ENU position (`pos_enu`, `pos_enu_org_ecef`) sets
its origin with the same three parameters, declared by one shared function:

| Parameter | Default | Meaning |
|---|---|---|
| `local_origin.mode` | `auto` | `auto`: the node's automatic origin (below); `fixed`: `local_origin.pos` |
| `local_origin.pos` | `[0, 0, 0]` | `[lat_deg, lon_deg, h_m]` for `llh`, `[x, y, z]` m for `ecef` |
| `local_origin.postype` | `llh` | `llh` or `ecef` |

The automatic origin is the first valid receiver solution (decoders, drivers), the
first valid solution (`single_point_positioning`), the base station position
(`real_time_kinematic`, `gnss_fgo`, `gnss_imu_fgo`; for `gnss_imu_fgo` also the origin
of the `_odom` pose) or the first usable GNSS solution (`gnss_imu_kalman_filter`).
A `pos` other than `[0, 0, 0]` with mode `auto` is rejected at startup, so a position
is never ignored silently. With mode `fixed`, `[0, 0, 0]` in `llh` is the point
0°N 0°E at height 0; in `ecef` it is rejected.

## Shared meanings

- `imu_orientation` is RPY in radians for the IMU-sensor to vehicle-body rotation
  in both `gnss_imu_kalman_filter` and `gnss_imu_fgo`.
- Rates and base reuse are derived from observed timestamps, not from a nominal
  frequency parameter.
- Defaults need not match between estimators; units and meanings do.
