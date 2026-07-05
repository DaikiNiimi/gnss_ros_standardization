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
