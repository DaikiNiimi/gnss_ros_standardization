# gnss_visualizer

Live dashboard for one receiver: status, skyplot, ENU trajectory and per-satellite
elevation/SNR. Built when OpenCV, `cv_bridge` and `image_transport` are installed.

| Panel | Content |
|---|---|
| Status | week/TOW, solution status, latitude/longitude/height, ENU velocity, satellites per constellation, DOP, AR ratio, differential age |
| Skyplot | satellite azimuth/elevation by constellation |
| Position (ENU) | trajectory colored by solution status, 1-σ ellipse, scale bar |
| Elevation / SNR | per-satellite elevation and L1/L2/L5 SNR |

```bash
ros2 run gnss_ros_standardization gnss_visualizer --ros-args \
  -p topics.solution:=/gnss/solution -p view_mode:=follow
```

## Topics

| Direction | Topic (parameter) | Default | Type |
|---|---|---|---|
| sub | `topics.observation` | `/gnss/observation` | `GnssObservations` |
| sub | `topics.ephemeris` | `/gnss/ephemeris` | `GnssEphemerides` |
| sub | `topics.solution` | `/gnss/nmea_solution` | `GnssSolution` |
| pub | `topics.image` | `gnss_visualization/dashboard` | `sensor_msgs/Image` (with `publish_image:=true`) |

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `use_gui` | `true` | open a window |
| `publish_image` | `false` | publish the dashboard image |
| `image_width`, `image_height` | `1280`, `720` | image size |
| `font_scale` | `1.0` | text size |
| `render_scale` | `2` | supersampling factor (1–4) |
| `view_mode` | `fixed` | position panel: `fixed`, `fit`, `follow` |
| `zoom_level` | `0` | position panel zoom (−7 to 10) |
| `recent_window_s` | `60.0` | trajectory kept at full rate |
| `decimate_old_dt_s` | `1.0` | spacing of older trajectory points |
| `max_history` | `50000` | maximum trajectory points |
| `manual_position.pos`, `.postype` | `[0,0,0]`, `llh` | position shown and used for the skyplot while there is no solution (`llh`: deg, deg, m; `ecef`: m; `[0,0,0]` = none) |

## Window controls

| Input | Action |
|---|---|
| `+` / `−` buttons, mouse wheel | zoom |
| mode button, `v` | cycle `fixed` / `fit` / `follow` |
| `CLR` button, `c` | clear the trajectory |
