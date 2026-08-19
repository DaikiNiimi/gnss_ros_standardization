# test

Unit tests for the shared GNSS code and standalone checks for the rosbags this
package produces and consumes. Both are part of the release: the tests are how a
change is shown not to have broken anything, and the bag checks are how a
converted dataset is shown to be faithful before any estimator runs on it.

## Unit tests

Built and run by `colcon test` when GTest is available. `CMakeLists.txt` guards
on this directory, so a checkout without it still configures and builds.

| file | covers |
|---|---|
| `test_epoch_matcher.cpp` | rover/base epoch pairing: ordering, reorder window, outage settling, queue overflow |
| `test_gnss_preprocessor.cpp` | masks, satellite positions, double-difference pairing |
| `test_gnss_fgo.cpp` | factor adapters, ambiguity resolution gates, gap-bridging predicates |

```bash
colcon test --packages-select gnss_ros_standardization && colcon test-result --verbose
```

## Bag verification

Plain `python3` scripts taking `--bag` plus the source file it should agree
with. Topic names are arguments, not assumptions, so they work on any bag with
the expected message types. Run them after converting a dataset: a conversion
that silently drops satellites, mislabels an ephemeris type or applies a frame
conversion twice otherwise looks like an estimator problem.

| script | checks |
|---|---|
| `verify_bag_obs.py` | observation epochs, satellites and signals against the source RINEX |
| `verify_bag_eph.py` | ephemeris coverage against the satellites the observations actually use |
| `verify_bag_imu.py` | IMU axes and signs against a reference trajectory, using velocity only so no Euler convention can defeat the test |
| `verify_bag_reference.py` | reference/truth solution against its source, including attitude sign conventions |

```bash
source install/setup.bash
python3 test/verify_bag_obs.py --bag <bag> --obs <rover.obs>
python3 test/verify_bag_eph.py --bag <bag> --nav <nav.rnx>
python3 test/verify_bag_imu.py --bag <bag> --reference <reference.csv>
python3 test/verify_bag_reference.py --bag <bag> --reference <reference.csv>
```

Each exits non-zero and prints the first disagreement it finds.
