# Converters

Offline conversion tools between ROS 2 bag files and the standard GNSS file
formats consumed by post-processing tools (RTKLIB `rnx2rtkp`, RTKPOST,
teqc, etc.).

## ROS 2 bag storage formats (Humble vs Jazzy)

ROS 2 distributions differ in the default rosbag2 storage backend:

| Distro | Default storage | Typical file |
|---|---|---|
| Humble | `sqlite3` | `*.db3` inside the bag directory |
| Jazzy  | `mcap`    | `*.mcap` inside the bag directory |

The writer tools (`rinex_to_rosbag`, `pos_to_rosbag`) accept a `--storage` flag
(`mcap` or `sqlite3`). When omitted the storage is chosen from the `--out`
extension (`.mcap` → mcap, `.db3` → sqlite3). otherwise the distro default is
used. 

The reader tools (`rosbag_to_rinex`, `rosbag_to_pos`) auto-detect the
storage backend from the bag's `metadata.yaml` and need no flag.

| Tool | Conversion | Source file |
|---|---|---|
| `rosbag_to_rinex` | rosbag (`/gnss/observation`, `/gnss/ephemeris`) → RINEX OBS + NAV | [`rosbag_to_rinex.cpp`](rosbag_to_rinex.cpp) |
| `rinex_to_rosbag` | RINEX OBS + NAV → rosbag | [`rinex_to_rosbag.cpp`](rinex_to_rosbag.cpp) |
| `rosbag_to_pos` | rosbag (`/gnss/solution`) → RTKLIB `.pos` | [`rosbag_to_pos.cpp`](rosbag_to_pos.cpp) |
| `pos_to_rosbag` | RTKLIB `.pos` → rosbag (`/gnss/solution`) | [`pos_to_rosbag.cpp`](pos_to_rosbag.cpp) |
| `synthetic_scenario_to_rosbag` | Simulated multi-GNSS + IMU scenario → rosbag (+ ground truth) | [`synthetic_scenario_to_rosbag.cpp`](synthetic_scenario_to_rosbag.cpp) |

---

## `rosbag_to_rinex`

```bash
ros2 run gnss_ros_standardization rosbag_to_rinex \
  --bag bag.db3 \
  --topic-obs /gnss/observation \
  --topic-nav /gnss/ephemeris \
  --obs output.obs \
  --nav output.nav \
  --rnx-version 3.04 \
  --nav-systems "GREJC"
```

| Flag | Description |
|---|---|
| `--bag` | Bag directory or `.db3` file |
| `--obs` | Output RINEX OBS path |
| `--nav` | Output RINEX NAV path |
| `--topic-obs` | Topic to read observations from |
| `--topic-nav` | Topic to read ephemeris from |
| `--rnx-version` | Output RINEX version (see below) |
| `--nav-systems` | Subset of `G`/`R`/`E`/`J`/`C`/`I`/`S` to include (default `GREJCIS`) |
| `--no-flush` | Buffer writes (default: flush per epoch) |
| `--pgm` | "PROGRAM" header field (default `rosbag_to_rinex`) |
| `--runby` | "RUN BY" header field (default: `$USER` → `$LOGNAME` → `"user"`) |
| `--help` / `--version` | Print usage / package version and exit |

**Supported RINEX write versions**: 3.00, 3.01, 3.02, 3.03, 3.04, 3.05.
RINEX 2.xx and 4.xx are not supported on the write path now.

## `rinex_to_rosbag`

```bash
ros2 run gnss_ros_standardization rinex_to_rosbag \
  --obs input.obs \
  --nav input.nav \
  --out my_bag.mcap
```

| Flag | Description |
|---|---|
| `--obs` | Input OBS file (required) |
| `--nav` | Input NAV file |
| `--out` | Output bag path (extension `.mcap` / `.db3` selects storage; otherwise distro default) |
| `--storage` | Force `mcap` or `sqlite3` (overrides extension auto-detection) |
| `--eph-mode` | `per-epoch`: republish every visible ephemeris at every obs epoch. `on-change` (default): per-sat, publish only when the selected nav index changes (large bag-size win on long sessions). |
| `--max-dtoe-gnss` | Max \|t_obs − toe\| for GNSS (GPS/Gal/QZS/BDS/IRN) ephemeris selection [sec]. Default `7200`. |
| `--max-dtoe-glo` | Max \|t_obs − toe\| for GLONASS ephemeris selection [sec]. Default `1800`. |
| `--pos` | Optional RTKLIB `.pos` file. When provided, `/gnss/solution` is also written into the bag.

Message timestamps (`header.stamp`) are GPST-based.

**Ephemeris-mode note**:
- `on-change`: Publishes only when the ephemeris changes (reduces bag size).
- `per-epoch`: Publishes at every epoch (recommended if playback starts from arbitrary times).

**Supported RINEX read versions**: 2.10 – 3.05 (via RTKLIB).

## `rosbag_to_pos`

```bash
ros2 run gnss_ros_standardization rosbag_to_pos \
  --bag bag.db3 --topic /gnss/solution --out output.pos
```

| Flag | Description |
|---|---|
| `--bag` | Bag directory or `.db3` file |
| `--topic` | `GnssSolution` topic to read (default `/gnss/solution`) |
| `--out` | Output `.pos` path (default: auto-derived as `<bag-stem>.pos`) |
| `--vel` | Append velocity columns (vn/ve/vu and their std-devs) |
| `--pgm` | `% program` header value (default `rosbag_to_pos`) |
| `--help` / `--version` | Print usage / package version and exit |

## `pos_to_rosbag`

```bash
ros2 run gnss_ros_standardization pos_to_rosbag \
  --pos input.pos --out my_bag.mcap --topic /gnss/solution
```

| Flag | Description |
|---|---|
| `--pos` | Input `.pos` file (required) |
| `--out` | Output bag path (extension `.mcap` / `.db3` selects storage; otherwise distro default) |
| `--topic` | `GnssSolution` topic to publish into the bag (default `/gnss/solution`) |
| `--storage` | Force `mcap` or `sqlite3` (overrides extension auto-detection) |

## `synthetic_scenario_to_rosbag`

Generate a fully synthetic GNSS/IMU scenario and write it to a bag, for
validating the positioning examples against a known ground truth **before
real-hardware testing**. A multi-GNSS constellation (GPS + Galileo + BeiDou +
QZSS) is built from synthetic ephemerides, and rover/base observations are
produced with RTKLIB's own models along a closed-form trajectory (static →
accelerate → turn). A matching IMU stream and the ground-truth antenna
solution are written too; measurement noise, a cycle slip and a GNSS outage
can be injected.

```bash
ros2 run gnss_ros_standardization synthetic_scenario_to_rosbag \
  --out scenario.mcap --gnss-rate 1 --imu-rate 100 \
  --noise on --slip G5:18 --outage 12,5
```

| Flag | Description |
|---|---|
| `--out` | Output bag path (default `scenario.mcap`; extension selects storage) |
| `--gnss-rate` | GNSS epoch rate [Hz] (default `1`) |
| `--imu-rate` | IMU sample rate [Hz] (default `100`) |
| `--noise` | `on` (default) / `off` — measurement + IMU noise and IMU bias |
| `--slip` | Inject a rover cycle slip: `<prn>:<t>` (optional leading system letter), e.g. `G5:18` = +N cycles on GPS PRN 5 at trajectory time 18 s |
| `--slip-cycles` | Slip magnitude in cycles (default `3`) |
| `--outage` | GNSS outage `<start>,<dur>` in trajectory seconds, e.g. `12,5` |
| `--base-east` | Base station offset east of the site [m] (default `8`) |
| `--storage` | Force `mcap` or `sqlite3` |

Topics written: `/gnss/observation`, `/base/gnss/observation`,
`/gnss/ephemeris`, `/gnss/imu/data_raw`, and `/ground_truth` (`GnssSolution`
at the true antenna position). The tool prints the base ECEF coordinate it
used — pass it to the positioning nodes.

### Validating an example against ground truth

```bash
# 1. Generate a scenario (note the base ECEF it prints).
ros2 run gnss_ros_standardization synthetic_scenario_to_rosbag \
  --out scenario.mcap --noise on --slip G5:18 --outage 12,5

# 2. Run a positioning node, e.g. the tightly-coupled FGO (needs GTSAM build).
ros2 run gnss_ros_standardization gnss_fgo --ros-args \
  --params-file examples/tightly_coupled_gnss/config/gnss_fgo.yaml \
  -p base_position.postype:=ecef \
  -p "base_position.pos:=[<X>, <Y>, <Z>]"

# 3. Record the solution and ground truth, then play the scenario.
ros2 bag record -o out /gnss/fgo/solution /ground_truth
ros2 bag play scenario.mcap

# 4. Convert both to .pos and compare (FIX epochs agree at cm level).
ros2 run gnss_ros_standardization rosbag_to_pos --bag out --topic /ground_truth     --out gt.pos
ros2 run gnss_ros_standardization rosbag_to_pos --bag out --topic /gnss/fgo/solution --out fgo.pos
```

The same scenario drives `gnss_imu_fgo` (it consumes `/gnss/imu/data_raw` and
dead-reckons through the injected GNSS outage) and the RTK / SPP examples.
