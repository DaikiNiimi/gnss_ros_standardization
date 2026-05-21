# utils

This directory hosts the `gnss_visualizer` node (described below) and
`gnss_utils.cpp`, a small shared helper library used across other components
(time conversions, ENU/ECEF rotations, status-string formatting).

## `gnss_visualizer`

A ROS 2 node that renders a four-panel live GNSS dashboard:

| Panel | Content |
|-------|---------|
| Status | Week / TOW, solution status badge (FIX / FLOAT / SINGLE / DGPS / SBAS / PPP / MANUAL), Lat/Lon/Alt, ENU velocity, satellite counts per constellation, GDOP/PDOP/HDOP/VDOP, AR ratio, age of differential |
| Skyplot | Per-satellite azimuth/elevation circles colored by constellation, with overlap-aware satellite-ID labels |
| Position (ENU) | Trajectory polyline colored by solution status, current-position marker, 1-σ covariance ellipse, scale bar and zoom indicator |
| Elevation / SNR | Per-satellite bars for elevation and L1 / L2 / L5 SNR, with 20 / 35 dB-Hz reference lines |

The node is built only when `OpenCV`, `cv_bridge`, and `image_transport` are all detected by `find_package`. If any of them is missing, CMake prints a `STATUS` message and skips this target.

## Run

```bash
ros2 run gnss_ros_standardization gnss_visualizer
```

If override any of them on the command line:

```bash
ros2 run gnss_ros_standardization gnss_visualizer --ros-args \
  -p obs_topic:=/rover/gnss/observation \
  -p nav_topic:=/rover/gnss/ephemeris \
  -p sol_topic:=/rover/gnss/solution \
  -p view_mode:=follow
```

The node subscribes to three topics and publishes one image stream:

| Direction | Default topic | Type | Parameter |
|-----------|---------------|------|-----------|
| sub | `/gnss/observation` | `gnss_ros_standardization/msg/GnssObservations` | `obs_topic` |
| sub | `/gnss/ephemeris` | `gnss_ros_standardization/msg/GnssEphemerides` | `nav_topic` |
| sub | `/gnss/nmea_solution` | `gnss_ros_standardization/msg/GnssSolution` | `sol_topic` |
| pub | `gnss_visualization/dashboard` | `sensor_msgs/Image` | `image_topic` (only when `publish_image:=true`) |

## On-window controls (GUI mode)

| Action | Effect |
|--------|--------|
| Click `+` / `−` buttons in Position panel header | Zoom in / out |
| Click `[FIX] / [FIT] / [FOL]` button | Cycle the position view mode |
| Press `v` | Same as clicking the mode button |
| Click `[CLR]` button | Clear the trajectory trail (current position marker and skyplot keep updating) |
| Press `c` | Same as clicking the CLR button |
