# `single_point_positioning`

Standalone GNSS pseudorange positioning (RTKLIB `pntpos`).

```bash
ros2 run gnss_ros_standardization single_point_positioning --ros-args \
  --params-file config/single_point_positioning.yaml
```

## Topics

| Direction | Default | Type |
|---|---|---|
| Sub | `topics.observation` (`/gnss/observation`) | `GnssObservations` |
| Sub | `topics.ephemeris` (`/gnss/ephemeris`) | `GnssEphemerides` |
| Pub | `topics.solution` (`/gnss/solution`) | `GnssSolution` |

## Parameters

| Key | Default | Meaning |
|---|---|---|
| `topics.observation` | `/gnss/observation` | observation input |
| `topics.ephemeris` | `/gnss/ephemeris` | broadcast ephemeris input |
| `topics.solution` | `/gnss/solution` | solution output |
| `tcp_port` | 8001 | NMEA output port; 0 disables |
| `elevation_mask_deg` | 15.0 | elevation mask, degrees |
| `nav_systems.gps` / `.gal` / `.bds` / `.qzs` / `.glo` / `.irn` / `.sbs` | see YAML | use this constellation |
| `frequencies.enable_l1` / `.enable_l2` / `.enable_l5` | see YAML | use this band |
| `snrmask.enable` | `false` | apply the carrier-to-noise mask |
| `snrmask.l1` / `.l2` / `.l5` | see YAML | per-band C/N0 mask, dB-Hz |
| `excluded_satellites` | `[]` | satellite IDs to drop, e.g. `["G05"]` |
| `fixed_origin.postype` / `.pos` | `llh` / `[0,0,0]` | ENU origin for the published local frame |
