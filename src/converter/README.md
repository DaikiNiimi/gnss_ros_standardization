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
