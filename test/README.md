# Tests

## Run

```bash
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select gnss_ros_standardization
colcon test-result --verbose
```

Add `-DBUILD_FGO_NODES=ON` to include the FGO tests. The tests use local files, TCP
sockets and pseudo terminals; no receiver is needed. A single Python test file can
be run directly, e.g. `python3 test/test_pvt_specification.py --bin-dir build/gnss_ros_standardization`.

## Contents

| Area | Tests |
|---|---|
| Receiver messages (UBX, SBF, NovAtel, RTCM, NMEA): observations, navigation, solution, IMU, time | `test_*_specification.py`, `test_rtcm_*.py`, `test_meas3_reference.py`, `test_ssr_output_policy.py`, `test_auxiliary_specification.py`, `test_decoder_boundaries.py` |
| Receiver nodes end to end (decoders and drivers, serial/TCP/file, reconnect) | `test_receiver_streams.py`, `test_receiver_observations.py`, `test_receiver_sessions.py`, `test_receiver_diagnostics.py` |
| Driver configuration against a fake receiver (commands, ACK/NAK, timeout) | `test_driver_configuration.py`, `test_rtcm_relay_names.py` |
| Framing, payloads, NMEA, observation and ephemeris rules | `test_binary_framer.cpp`, `test_receiver_protocol.cpp`, `test_nmea_parser.cpp`, `test_raw_observations.cpp`, `test_observation_validation.cpp`, `test_ephemeris_contract.cpp`, `test_navigation_record_key.cpp`, `test_stream_transport.cpp`, `test_receiver_health.cpp`, `test_tcp_nmea_server.cpp` |
| Converters (RINEX ⇄ bag, POS ⇄ bag) | `test_converter_roundtrip.py`, `test_rinex_audit.py` |
| Positioning (SPP, RTK, EKF, matcher, preprocessing) | `test_spp_node.cpp`, `test_rtk_node.cpp`, `test_gnss_imu_ekf.cpp`, `test_epoch_matcher.cpp`, `test_gnss_preprocessor.cpp`, `test_ins_tick_clock.cpp` |
| FGO | `fgo/` (`test_fgo_*`) |
| Launch, parameter names, release files | `test_launch_settings.py`, `test_parameter_names.py`, `test_release.py`, `check_diagnostic_*.py`, `test_visualizer_release.py` |

## Installed package

```bash
bash test/check_install.sh "$PWD/install/gnss_ros_standardization"
```

Builds an external C++ package against the installed targets and exchanges all six
message types with Python.

## Bag checks

[`tools/`](tools/) has read-only checks for a converted or recorded bag:

```bash
python3 test/tools/verify_bag_obs.py --bag <bag> --obs <rover.obs>
python3 test/tools/verify_bag_eph.py --bag <bag> --nav <nav.rnx>
python3 test/tools/verify_bag_imu.py --bag <bag> --reference <reference.csv>
python3 test/tools/verify_bag_timeline.py --bag <bag>
python3 test/tools/verify_bag_reference.py --bag <bag> --reference <reference.csv>
python3 test/tools/check_ppc_playback.py --help
```

Each exits non-zero and prints the first disagreement.
