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
