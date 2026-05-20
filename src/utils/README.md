# gnss_visualizer

A ROS 2 node that renders a four-panel live GNSS dashboard using OpenCV:

| Panel | Content |
|-------|---------|
| Status | Week / TOW, solution status badge (FIX / FLOAT / SINGLE / DGPS / SBAS / PPP / MANUAL), Lat/Lon/Alt, ENU velocity, satellite counts per constellation, GDOP/PDOP/HDOP/VDOP, AR ratio, age of differential |
| Skyplot | Per-satellite azimuth/elevation circles colored by constellation, with overlap-aware satellite-ID labels |
| Position (ENU) | Trajectory polyline colored by solution status, current-position marker, 1-σ covariance ellipse, scale bar and zoom indicator |
| Elevation / SNR | Per-satellite bars for elevation and L1 / L2 / L5 SNR, with 20 / 35 dB-Hz reference lines |

The composed dashboard is also published as `sensor_msgs/Image` so it can be viewed remotely or recorded into a bag without a display attached.

## Build

The node is built only when `OpenCV`, `cv_bridge`, and `image_transport` are all detected by `find_package`. If any of them is missing, CMake prints a `STATUS` message and skips this target; the rest of the package builds normally.

```bash
colcon build --packages-select gnss_ros_standardization
```

## Run

```bash
ros2 run gnss_ros_standardization gnss_visualizer
```

The node subscribes to three topics and publishes one image stream:

| Direction | Default topic | Type | Parameter |
|-----------|---------------|------|-----------|
| sub | `/gnss/observation` | `gnss_ros_standardization/msg/GnssObservations` | `obs_topic` |
| sub | `/gnss/ephemeris` | `gnss_ros_standardization/msg/GnssEphemerides` | `nav_topic` |
| sub | `/gnss/nmea_solution` | `gnss_ros_standardization/msg/GnssSolution` | `sol_topic` |
| pub | `gnss_visualization/dashboard` | `sensor_msgs/Image` | `image_topic` (only when `publish_image:=true`) |

Override any of them on the command line (this is the easiest way to run multiple instances or attach to a non-default namespace):

```bash
ros2 run gnss_ros_standardization gnss_visualizer --ros-args \
  -p obs_topic:=/rover/gnss/observation \
  -p nav_topic:=/rover/gnss/ephemeris \
  -p sol_topic:=/rover/gnss/solution \
  -p view_mode:=follow
```

## Parameters

### Topics
- `obs_topic` (string, default `/gnss/observation`)
- `nav_topic` (string, default `/gnss/ephemeris`)
- `sol_topic` (string, default `/gnss/nmea_solution`)
- `image_topic` (string, default `gnss_visualization/dashboard`)

### Window
- `image_width`  (int, default `1280`)
- `image_height` (int, default `720`)
- `font_scale` (double, default `1.0`) — global multiplier for all text scales. The defaults are tuned for a 1280×720 canvas; bump to `1.2`–`1.5` if you also enlarge `image_width`/`image_height` (e.g. `1920×1080` or `2560×1440` for 4K screens).
- `use_gui` (bool, default `true`) — set `false` for headless use
- `publish_image` (bool, default `false`) — when `true`, the composed dashboard is published as `sensor_msgs/Image` on `image_topic`. Off by default because the 1280×720 BGR stream at ~100 Hz adds ~270 MB/s if recorded with `ros2 bag record -a`. Enable only when you need a remote dashboard or want to record the image into a bag. Can also be toggled at runtime via `ros2 param set /gnss_visualizer publish_image true`.
- `zoom_level` (int, default `0`, range `[-5, 10]`) — initial zoom for the Position panel

### Position view
- `view_mode` (string, default `"fixed"`): one of
  - `"fixed"` — origin (0,0 ENU) is pinned to the panel center; manual zoom only. **Default; matches pre-existing behavior.**
  - `"fit"` — autoscale so the entire trajectory always fits with ~10 % margin; manual zoom is ignored.
  - `"follow"` — the latest fix is kept at the panel center; manual zoom applies.
- `fixed_latitude`, `fixed_longitude`, `fixed_altitude` (double, default `0.0`) — optional fallback origin in degrees / meters. When no live solution is available, skyplot and the Status panel use these as the receiver position so the skyplot still renders meaningfully.

### Trajectory history
The trail is stored as a `std::deque` and kept compact by time-based decimation. Recent points are retained at the full solution rate; older points are thinned out so that long bag playback does not erase the start of the track.

- `recent_window_sec`   (double, default `60.0`) — points within this age from the latest fix are kept at full rate.
- `decimate_old_dt_sec` (double, default `1.0`)  — minimum time gap between retained older points.
- `max_history`         (int,    default `50000`) — hard upper cap; oldest points drop first.

## On-window controls (GUI mode)

| Action | Effect |
|--------|--------|
| Mouse wheel over canvas | Zoom in / out (Position panel) |
| Click `+` / `−` buttons in Position panel header | Zoom in / out |
| Click `[FIX] / [FIT] / [FOL]` button | Cycle the position view mode (also updates the `view_mode` ROS parameter) |
| Click `[CLR]` button | Clear the trajectory trail (current position marker and skyplot keep updating) |
| Press `v` | Same as clicking the mode button |
| Press `c` | Same as clicking the CLR button |

The active view mode is always shown in the Position panel header (e.g. `[FOL] x4`).

## Post-analysis with rtkplot

For offline trajectory inspection you can convert a recorded bag to RTKLIB's `.pos` format with `src/converter/rosbag_to_pos.cpp` and open it in **rtkplot** from the RTKLIB submodule under `third_party/RTKLIB/app/qtapp/rtkplot_qt/`. rtkplot is intentionally not built by this package (it adds a Qt5/6 build dependency); build it separately when needed.

## Headless / remote use

Pass both `use_gui:=false` (suppress the local OpenCV window) and `publish_image:=true` (enable the image stream). The dashboard is then available on `image_topic` for `rqt_image_view`, `ros2 bag record`, or `image_transport` plugins (e.g. `compressed`) for remote dashboards.

```bash
ros2 run gnss_ros_standardization gnss_visualizer --ros-args \
  -p use_gui:=false -p publish_image:=true
```

## Note on default behavior change

Prior to the addition of `publish_image`, the dashboard image was published unconditionally. It now defaults to **off** so that `ros2 bag record -a` does not silently inflate bags by hundreds of MB per second. If you previously relied on the image topic being available without flags, add `-p publish_image:=true` explicitly.
