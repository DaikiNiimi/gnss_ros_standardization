# Converters

| Tool | Conversion |
|---|---|
| `rinex_to_rosbag` | RINEX OBS + NAV (and optional `.pos`, IMU CSV, reference CSV) → rosbag |
| `rosbag_to_rinex` | rosbag (`/gnss/observation`, `/gnss/ephemeris`) → RINEX OBS + NAV |
| `pos_to_rosbag` | RTKLIB `.pos` → rosbag (`/gnss/solution`) |
| `rosbag_to_pos` | rosbag (`GnssSolution`) → RTKLIB `.pos` |

Bags are written as `mcap` or `sqlite3`, chosen by `--storage` or by the `--out`
extension (`.mcap`, `.db3`). Readers detect the storage from the bag.

## `rinex_to_rosbag`

```bash
ros2 run gnss_ros_standardization rinex_to_rosbag \
  --obs rover.obs --obs base=base.obs --nav brdc.nav --out run.mcap
```

| Option | Default | Meaning |
|---|---|---|
| `--obs [name=]FILE` | required | observation file, repeatable; `name=` writes `/<name>/observation` |
| `--nav FILE` | — | navigation file, repeatable |
| `--out PATH` | from the OBS name | output bag |
| `--storage mcap\|sqlite3` | from `--out` | bag storage |
| `--timestamp-scale utc\|gpst` | `utc` | header and bag time stamps (`week`/`tow` are always GPST) |
| `--eph-mode transmission\|all` | `transmission` | `transmission`: each NAV record at its transmission time, records sent before the first epoch 1 s before it (as a receiver). `all`: every record 1 s before the first epoch (as `rnx2rtkp`) |
| `--max-dtoe-glo SEC` | `1800` | GLONASS ephemeris window for filling a missing frequency channel |
| `--topic-obs [name=]TOPIC`, `--topic-nav`, `--topic-pos` | `/gnss/observation`, `/gnss/ephemeris`, `/gnss/solution` | topic names |
| `--pos [name=]FILE` | — | RTKLIB `.pos` written as `GnssSolution` |
| `--imu FILE` | — | IMU CSV (GPS TOW, week, acc XYZ m/s², gyro XYZ deg/s) |
| `--topic-imu TOPIC` | `/gnss/imu/data_raw` | IMU topic |
| `--imu-frame raw\|frd2flu\|frd2flu_grav` | `raw` | IMU axes: `raw` = already FLU; `frd2flu` = FRD specific force; `frd2flu_grav` = FRD gravity direction |
| `--reference FILE` | — | ground-truth CSV (PPC `reference.csv` format) |
| `--topic-reference TOPIC` | `/gnss/reference/solution` | reference position/velocity; attitude on `<topic>_odom` |

Use the default `--eph-mode transmission` with the positioning nodes of this package.
The bag contains `gnss_time_basis.yaml` with the time scale used.

## `rosbag_to_rinex`

```bash
ros2 run gnss_ros_standardization rosbag_to_rinex \
  --bag run.mcap --obs out.obs --nav out.nav
```

| Option | Default | Meaning |
|---|---|---|
| `--bag PATH` | required | bag directory or file |
| `--obs PATH`, `--nav PATH` | — | output RINEX files |
| `--topic-obs`, `--topic-nav` | `/gnss/observation`, `/gnss/ephemeris` | input topics |
| `--rnx-version X.YY` | `3.05` | RINEX 3.00–3.05 |
| `--nav-systems` | `GREJCIS` | constellations written to NAV |
| `--nav-all-transmissions` | off | keep every transmission of the same record |
| `--legacy-glonass-time` | off | remove the extra leap second from GLONASS toe/tof of older bags |
| `--no-flush` | off | buffer writes |
| `--pgm`, `--runby` | `rosbag_to_rinex`, `$USER` | RINEX header fields |

A conversion that cannot be written without losing a value stops with an error and
leaves existing output files unchanged.

## `rosbag_to_pos`

```bash
ros2 run gnss_ros_standardization rosbag_to_pos --bag run.mcap --topic /gnss/solution --out out.pos
```

| Option | Default | Meaning |
|---|---|---|
| `--bag PATH` | required | bag directory or file |
| `--topic TOPIC` | `/gnss/solution` | `GnssSolution` topic |
| `--out PATH` | `<bag>.pos` | output file |
| `--vel` | off | add velocity columns |
| `--pgm NAME` | `rosbag_to_pos` | header program name |

## `pos_to_rosbag`

```bash
ros2 run gnss_ros_standardization pos_to_rosbag --pos in.pos --out run.mcap
```

| Option | Default | Meaning |
|---|---|---|
| `--pos FILE` | required | RTKLIB `.pos` (decimal degrees or ECEF) |
| `--out PATH` | from the POS name | output bag |
| `--topic TOPIC` | `/gnss/solution` | output topic |
| `--storage mcap\|sqlite3` | from `--out` | bag storage |
| `--timestamp-scale utc\|gpst` | `utc` | header and bag time stamps |

## Checking a converted bag

[`test/tools`](../../test/tools/) contains read-only checks for a converted bag:
`verify_bag_obs.py` (observations against the RINEX), `verify_bag_eph.py`
(ephemeris stream), `verify_bag_imu.py` (IMU axes), `verify_bag_timeline.py` (IMU
time line), `verify_bag_reference.py` (reference topics) and `check_ppc_playback.py`
(playback completeness). Run each with `--help` for its arguments.
