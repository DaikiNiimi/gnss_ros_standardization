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
extension (`.mcap` → mcap, `.db3` → sqlite3); otherwise the distro default is
used. The reader tools (`rosbag_to_rinex`, `rosbag_to_pos`) auto-detect the
storage backend from the bag's `metadata.yaml` and need no flag.

Running mcap on Humble requires the mcap storage plugin
(`apt install ros-humble-rosbag2-storage-mcap`); on Jazzy it is part of the
default install.

| Executable | Direction | Source |
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
| `--nav-systems` | Subset of `G`/`R`/`E`/`J`/`C`/`I`/`S` to include |
| `--no-flush` | Buffer writes (default: flush per epoch) |
| `--pgm` | "PROGRAM" header field |
| `--runby` | "RUN BY" header field |

**Supported RINEX write versions**: 3.00, 3.01, 3.02, 3.03, 3.04, 3.05.
Values outside `[3.00, 3.05]` are rejected. RINEX 2.xx and 4.xx are not
supported on the write path.

## `rinex_to_rosbag`

```bash
ros2 run gnss_ros_standardization rinex_to_rosbag \
  --obs input.obs \
  --nav input.nav \
  --out my_bag
```

| Flag | Description |
|---|---|
| `--obs` | Input OBS file (required) |
| `--nav` | Input NAV file (may repeat for multi-constellation NAV split files) |
| `--out` | Output bag path (extension `.mcap` / `.db3` selects storage; otherwise distro default) |
| `--storage` | Force `mcap` or `sqlite3` (overrides extension auto-detection) |

Publishes `/gnss/observation` and `/gnss/ephemeris` into the bag.

**Supported RINEX read versions**: 2.10 – 3.05 (via RTKLIB).
For Hatanaka-compressed inputs, decompress with `crx2rnx` first. RINEX 4.xx is
not supported.

## `rosbag_to_pos`

```bash
ros2 run gnss_ros_standardization rosbag_to_pos \
  --bag bag.db3 --topic /gnss/solution --out output.pos
```

| Flag | Description |
|---|---|
| `--bag` | Bag directory or `.db3` file |
| `--topic` | `GnssSolution` topic to read (default `/gnss/solution`) |
| `--out` | Output `.pos` path |

Both LLH (`lat/lon/height`) and ECEF (`x/y/z`) `.pos` formats are emitted based
on the solution payload.

## `pos_to_rosbag`

```bash
ros2 run gnss_ros_standardization pos_to_rosbag \
  --pos input.pos --out my_bag --topic /gnss/solution
```

| Flag | Description |
|---|---|
| `--pos` | Input `.pos` file (required) |
| `--out` | Output bag path (extension `.mcap` / `.db3` selects storage; otherwise distro default) |
| `--topic` | `GnssSolution` topic to publish into the bag |
| `--storage` | Force `mcap` or `sqlite3` (overrides extension auto-detection) |

Both LLH and ECEF `.pos` formats are accepted.

---

For the structure of the bagged messages, see
[`../../msg/README.md`](../../msg/README.md).
