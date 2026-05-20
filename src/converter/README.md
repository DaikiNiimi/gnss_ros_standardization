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
| `--nav-systems` | Subset of `G`/`R`/`E`/`J`/`C`/`I`/`S` to include (default `GREJCIS`) |
| `--no-flush` | Buffer writes (default: flush per epoch) |
| `--pgm` | "PROGRAM" header field (default `rosbag_to_rinex`) |
| `--runby` | "RUN BY" header field (default: `$USER` → `$LOGNAME` → `"user"`) |
| `--help` / `--version` | Print usage / package version and exit |

If both `--obs` and `--nav` are omitted, output paths are auto-derived as
`<bag-stem>.obs` / `<bag-stem>.nav` next to the bag.

**Supported RINEX write versions**: 3.00, 3.01, 3.02, 3.03, 3.04, 3.05.
Values outside `[3.00, 3.05]` are rejected. RINEX 2.xx and 4.xx are not
supported on the write path.

### Design (two-pass)

The RINEX OBS header must declare every observation column up-front, so the
bag is read twice:

1. **Pass 1 — Scanner**: walks the bag once to determine the union of
   observation types per GNSS system, the timespan, and GLONASS FCN
   assignments. The header is built from this scan.
2. **Pass 2 — Writers**: walks the bag again. `ObsWriter` buffers epochs
   through a short reorder window (fixed 3 s) to tolerate slight
   out-of-order delivery, then emits RINEX OBS records. `NavWriter`
   deduplicates ephemerides by `(sat, IODE, IODC, code)`.

Galileo band digit `'9'` is intentionally rejected (not in RINEX 3.0x spec
for E). IRNSS S-band `'9'` is accepted.

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
| `--nav` | Input NAV file (may repeat for multi-constellation NAV split files; omit to write an empty ephemeris topic) |
| `--out` | Output bag path (extension `.mcap` / `.db3` selects storage; otherwise distro default) |
| `--storage` | Force `mcap` or `sqlite3` (overrides extension auto-detection) |
| `--eph-mode` | `per-epoch`: republish every visible ephemeris at every obs epoch. `on-change` (default): per-sat, publish only when the selected nav index changes (large bag-size win on long sessions). |
| `--max-dtoe-gnss` | Max \|t_obs − toe\| for GNSS (GPS/Gal/QZS/BDS/IRN) ephemeris selection [sec]. Default `7200`. |
| `--max-dtoe-glo` | Max \|t_obs − toe\| for GLONASS ephemeris selection [sec]. Default `1800`. |
| `--pos` | Optional RTKLIB `.pos` file. When provided, `/gnss/solution` is also written into the bag (same format support as `pos_to_rosbag`: LLH/ECEF, GPST/UTC, velocity columns, ENU origin). Enables a single `ros2 bag play` session with all three topics for `gnss_visualizer` or positioning nodes. |

Publishes `/gnss/observation` and `/gnss/ephemeris` into the bag; also `/gnss/solution` when `--pos` is given. The
`/gnss/ephemeris` topic is always declared, even with no `--nav` (it just
stays empty). `header.stamp` is a GPST-derived `rclcpp::Time` because the
input file has no PC-clock context; canonical GNSS time is also exposed in
`week`/`tow` (obs) and per-eph fields (`toes`, `toc`, …).

**Ephemeris-mode caveat (`on-change`)**: RINEX nav files do not record
broadcast-arrival time. `on-change` simply detects "the index returned by
the best-match selector differs from the previously published one for this
satellite", which is a *file-state* change, not an actual broadcast event.
Switching to `per-epoch` is the right choice when downstream consumers may
start playback from an arbitrary epoch and need ephemerides immediately.

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
| `--topic` | `GnssSolution` topic to read (default `/gnss/nmea_solution`) |
| `--out` | Output `.pos` path (default: auto-derived as `<bag-stem>.pos`) |
| `--vel` | Append velocity columns (vn/ve/vu and their std-devs) |
| `--pgm` | `% program` header value (default `rosbag_to_pos`) |
| `--help` / `--version` | Print usage / package version and exit |

Output rules:
- Q codes: `1=fix, 2=float, 3=sbas, 4=dgps, 5=single, 6=ppp`.
- Rows with `status` mapping to `Q=0`, or with `time_week==0 && time_tow==0`,
  are dropped (uninitialized messages).
- Position covariance is row-major **E-N-U** in the message; it is remapped
  to RTKLIB's **N-E-U** column order in `.pos`.
- With `--vel`, if any velocity component is non-finite (some decoders emit
  `quiet_NaN` when origin is unavailable), velocity columns are **omitted
  for that row only**; position columns are still written.

Both LLH (`lat/lon/height`) and ECEF (`x/y/z`) `.pos` formats are emitted based
on the solution payload.

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

### Input handling

- **Position format**: LLH (`lat/lon/height`) and ECEF (`x/y/z-ecef`) `.pos`
  files are both accepted. The format is detected from the header
  (`% (lat/lon/height ...)`, `% (x/y/z-ecef ...)`). If neither keyword is
  found, it is inferred from the first data row's magnitude
  (\|a\|≤180 ∧ \|b\|≤180 → LLH, else ECEF) and a notice is printed.
- **Velocity columns**: optional RTKLIB `--output-vel` output is detected via
  `vx-ecef` / `ve(m/s)` / `vn(m/s)` in the header. When present, the trailing
  3 velocity + 6 velocity-σ columns are read into `vel_ecef` / `vel_enu` and
  the matching cov fields. Without these columns the velocity fields stay
  NaN (see below).
- **Time system**: `% time sys` header is honored. The default is GPST; if
  the header explicitly declares UTC, each row time is converted via
  `utc2gpst`. RTKLIB's default output is GPST.
- **Skipped rows**: malformed rows are skipped with a per-row warning on
  stderr (up to 5; further warnings are suppressed). A final
  `wrote N epochs; skipped M rows` summary is printed. Exit code is non-zero
  if no epochs were written.

### Unset fields (NaN convention)

`.pos` carries position + covariance + Q + ns (+ optional velocity). All
other `GnssSolution` fields are populated with `NaN`:

| Field | When set |
|---|---|
| `pos_ecef`, `latitude`/`longitude`/`altitude`, `pos_cov_ecef` (ECEF input) or `pos_enu_cov` (LLH input) | always (from file) |
| `vel_ecef` + `vel_cov_ecef` (ECEF input with `--output-vel`) | when velocity columns present |
| `vel_enu` + `vel_enu_cov` (LLH input with `--output-vel`) | when velocity columns present |
| `ratio`, `age_diff` | when columns 12/13 are present (RTKLIB default yes) |
| `gdop`/`pdop`/`hdop`/`vdop` | never (not in `.pos`) — kept NaN |
| `pos_enu`, `pos_enu_org_ecef` | never — kept NaN |

**Caveat on RTKLIB zero-fill**: `.pos` itself zero-fills some unknowns
(e.g. `age=0.0` may mean "no diff corrections" or "diff just arrived").
Downstream consumers cannot distinguish "true 0" from "unknown" for those
fields after RTKLIB serialization. Treat 0 as informational, not as a
definitive measurement of zero.

---

## Tests

```bash
colcon test --packages-select gnss_ros_standardization
colcon test-result --verbose
```

Current coverage: unit tests for the `gnss_converter_io` helpers in
[`../../include/gnss_ros_standardization/bag_io_utils.hpp`](../../include/gnss_ros_standardization/bag_io_utils.hpp):
path normalization, output-path derivation, safe parent climb,
`resolveStorageId` priority, `toRosTimeGpst` carry/clamp behavior, and the
ECEF / ENU row-major covariance layout written by `pos_to_rosbag`.

---

For the structure of the bagged messages, see
[`../../msg/README.md`](../../msg/README.md).
