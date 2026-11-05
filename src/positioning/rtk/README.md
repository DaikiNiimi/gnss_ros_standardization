# `real_time_kinematic`

RTK positioning with RTKLIB `rtkpos` from rover and base observations.

```bash
ros2 run gnss_ros_standardization real_time_kinematic --ros-args \
  --params-file config/real_time_kinematic.yaml \
  -p ant2.postype:=llh -p "ant2.pos:=[35.0, 139.0, 50.0]"
```

`ant2.pos` (the surveyed base position) must be set; the node does not start
with the default `[0, 0, 0]`.

## Topics

| Direction | Parameter | Default | Type |
|---|---|---|---|
| sub | `topics.rover_observation` | `/gnss/observation` | `GnssObservations` |
| sub | `topics.base_observation` | `/base/gnss/observation` | `GnssObservations` |
| sub | `topics.ephemeris` | `/gnss/ephemeris` | `GnssEphemerides` |
| pub | `topics.solution` | `/gnss/solution` | `GnssSolution` |

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `ant2.postype`, `ant2.pos` | `llh`, `[0,0,0]` | base position (`llh`: deg, deg, m; `ecef`: m) |
| `pos1.*`, `pos2.*`, `rtk_stats.*` | see YAML | RTKLIB processing options, same names as the RTKLIB configuration file |
| `frequencies.enable_l1`, `.enable_l2`, `.enable_l5` | `true`, `true`, `false` | bands used |
| `excluded_satellites` | `[]` | satellites not used, e.g. `["G05", "C12"]` |
| `local_origin.mode`, `.postype`, `.pos` | `auto`, `llh`, `[0,0,0]` | ENU origin: `auto` = base position, `fixed` = `pos` |
| `debug.solstatus_path` | `""` | write the RTKLIB solution status file |
| `tcp_port` | `8002` | NMEA output over TCP; `0` disables |

Each rover epoch is paired with the base epoch of the same time, or with the latest
older base epoch (up to `pos2.maxage` seconds).

The NMEA TCP server listens on all network interfaces; set `tcp_port: 0` when it is
not needed.
