# `real_time_kinematic`

RTK positioning using rover and base observations with broadcast ephemeris
(RTKLIB `rtkpos`).

```bash
ros2 run gnss_ros_standardization real_time_kinematic --ros-args \
  --params-file config/real_time_kinematic.yaml
```

## Topics

| Direction | Default | Type |
|---|---|---|
| Sub | `topics.rover_observation` (`/gnss/observation`) | `GnssObservations` |
| Sub | `topics.base_observation` (`/base/gnss/observation`) | `GnssObservations` |
| Sub | `topics.ephemeris` (`/gnss/ephemeris`) | `GnssEphemerides` |
| Pub | `topics.solution` (`/gnss/solution`) | `GnssSolution` |

## Parameters

`pos1.*`, `pos2.*`, `ant2.*` and `rtk_stats.*` are RTKLIB `prcopt_t` /
`solopt_t` fields passed straight through; see the RTKLIB manual for their
meaning and ranges. The keys below are this package's own.

| Key | Default | Meaning |
|---|---|---|
| `topics.rover_observation` | `/gnss/observation` | rover observation input |
| `topics.base_observation` | `/base/gnss/observation` | base observation input |
| `topics.ephemeris` | `/gnss/ephemeris` | broadcast ephemeris input |
| `topics.solution` | `/gnss/solution` | solution output |
| `tcp_port` | 8002 | NMEA output port; 0 disables |
| `frequencies.enable_l1` / `.enable_l2` | `true` | use this band |
| `frequencies.enable_l5` | `false` | use L5 / E5a / B2a |
| `excluded_satellites` | `[]` | satellite IDs to drop, e.g. `["G05"]` |
| `fixed_origin.postype` / `.pos` | `llh` / `[0,0,0]` | ENU origin for the published local frame |
| `debug.solstatus_path` | `""` | RTKLIB solution-status file; empty disables |
