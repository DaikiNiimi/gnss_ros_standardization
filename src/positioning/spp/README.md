# `single_point_positioning`

Single point positioning with RTKLIB `pntpos`.

```bash
ros2 run gnss_ros_standardization single_point_positioning --ros-args \
  --params-file config/single_point_positioning.yaml
```

## Topics

| Direction | Parameter | Default | Type |
|---|---|---|---|
| sub | `topics.observation` | `/gnss/observation` | `GnssObservations` |
| sub | `topics.ephemeris` | `/gnss/ephemeris` | `GnssEphemerides` |
| pub | `topics.solution` | `/gnss/solution` | `GnssSolution` |

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `masks.elevation_deg` | `15.0` | elevation mask, degrees |
| `navsys.gps`, `.glo`, `.gal`, `.bds`, `.qzs`, `.irn`, `.sbs` | see YAML | constellations used |
| `frequencies.enable_l1`, `.enable_l2`, `.enable_l5` | see YAML | bands used |
| `snrmask.enable` | `false` | apply the C/N0 mask |
| `snrmask.l1`, `.l2`, `.l5` | see YAML | C/N0 mask per band, dB-Hz |
| `excluded_satellites` | `[]` | satellites not used, e.g. `["G05", "C12"]` |
| `local_origin.mode`, `.postype`, `.pos` | `auto`, `llh`, `[0,0,0]` | ENU origin: `auto` = first solution, `fixed` = `pos` |
| `tcp_port` | `8001` | NMEA output over TCP; `0` disables |

The NMEA TCP server listens on all network interfaces; set `tcp_port: 0` when it is
not needed.
