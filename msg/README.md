# Message Reference

This directory defines the ROS 2 messages that form the public interface of
`gnss_ros_standardization`. All field names and semantics follow the
corresponding [RTKLIB](https://github.com/rtklibexplorer/RTKLIB) C structures
(`obsd_t`, `eph_t`, `geph_t`) so that decoders, converters and positioning
nodes can map directly to/from RTKLIB internals.

| Message | Underlying RTKLIB type | Purpose |
|---|---|---|
| [`GnssObservation.msg`](GnssObservation.msg) | `obsd_t` (single epoch / single satellite) | One pseudorange / carrier-phase / Doppler / SNR sample |
| [`GnssObservations.msg`](GnssObservations.msg) | array of `obsd_t` at one epoch | All satellites at one receiver epoch |
| [`GnssEphemeris.msg`](GnssEphemeris.msg) | `eph_t` | Keplerian ephemeris (GPS / Galileo / QZSS / BeiDou / NavIC / SBAS) |
| [`GlonassEphemeris.msg`](GlonassEphemeris.msg) | `geph_t` | GLONASS state-vector ephemeris (PZ-90.11) |
| [`GnssEphemerides.msg`](GnssEphemerides.msg) | wrapper around both above | Mixed-constellation navigation message bundle |
| [`GnssSolution.msg`](GnssSolution.msg) | RTKLIB `sol_t` + ENU extensions | Receiver positioning solution (LLH / ECEF / ENU + covariances) |

---

## `header.stamp` convention (applies to all messages here)

`header.stamp` is a `rclcpp::Time` value. Two conventions are in use across
producers; the field itself does not encode which:

| Producer | Default `stamp` source |
|---|---|
| `*_driver_node` (UBX / SBF / NovAtel) | PC time at packet ingest — preferred for cross-sensor sync |
| `*_decoder_node` | Same as the driver that fed them (PC time by default; optional GPST→ROS-Time conversion) |
| `rinex_to_rosbag`, `pos_to_rosbag` | GPST seconds packed into `rclcpp::Time` — input file has no PC clock |
| `rosbag_to_rinex`, `rosbag_to_pos` | n/a (reader; uses input `stamp` as-is) |

For canonical, unambiguous GNSS time, **always use the explicit
GPS-week / time-of-week fields** (`week`/`tow` in `GnssObservations`,
`time_week`/`time_tow` in `GnssSolution`, `toes`/`toc` in `GnssEphemeris`,
…) rather than `stamp`. Two consumers receiving the same bag will agree on
`week`/`tow` regardless of how the producer chose to fill `stamp`.

The `stamp` field is the right thing to use only when you want to align
GNSS messages with other ROS sensors recorded in the same bag.

## NaN convention for unset numeric fields

When a producer cannot fill a numeric field, it sets `NaN` rather than `0`:

- DOP fields (`gdop`, `pdop`, `hdop`, `vdop`) when no DOP is available for
  the epoch (see staleness rules in [`GnssSolution.msg`](GnssSolution.msg)).
- `ratio`, `age_diff` when not provided by the source.
- Velocity fields (`vel_ecef`, `vel_cov_ecef`, `vel_enu`, `vel_enu_cov`)
  when the source does not provide velocity (e.g. `.pos` without
  `--output-vel`).
- ENU position fields (`pos_enu`, `pos_enu_org_ecef`) when no ENU origin
  has been established.

Consumers must check with `std::isnan()` before use. The `0` value is
*never* used as a "missing" sentinel for these fields. The single exception
is `.pos` round-trip: RTKLIB's own `.pos` format zero-fills some unknowns
in the file, so `pos_to_rosbag` cannot losslessly recover NaN intent for
fields like `age_diff` — see
[`src/converter/README.md`](../src/converter/README.md#unset-fields-nan-convention).

---

## `GnssObservation.msg`

Single satellite observation at one epoch. Fields prefixed by **(RTKLIB)** mirror
the corresponding member in `obsd_t`; the remaining fields are derived for
ergonomic ROS use.

| Field | Type | Unit / Range | Notes |
|---|---|---|---|
| `sat` | `uint8` | 1 – `MAXSAT` | RTKLIB satellite number |
| `code` | `uint8` | RTKLIB `CODE_*` enum | Signal/code indicator |
| `p` | `float64` | m | Pseudorange |
| `l` | `float64` | cycle | Carrier phase |
| `d` | `float64` | Hz | Doppler |
| `snr` | `float32` | dB-Hz | Signal strength |
| `lli` | `uint8` | bitmask | Loss-of-lock indicator |
| `system` | `string` | `G`/`R`/`E`/`J`/`C`/`I`/`S` | GNSS constellation code |
| `prn` | `uint16` | per-constellation | Satellite PRN |
| `satid` | `string` | e.g. `"G01"`, `"R01"` | Human-readable ID |
| `code_str` | `string` | e.g. `"1C"`, `"2W"` | RINEX 3 observation code |

## `GnssObservations.msg`

One epoch of observations for all tracked satellites.

| Field | Type | Notes |
|---|---|---|
| `header` | `std_msgs/Header` | `stamp` = ROS Time (see [header.stamp convention](#headerstamp-convention-applies-to-all-messages-here)); `frame_id` = receiver label |
| `week` | `uint32` | GPS week of the epoch (extended week — survives 1024-week rollover) |
| `tow` | `float64` | Time of week [s] |
| `observations` | `GnssObservation[]` | One element per (satellite, signal) |

## `GnssEphemeris.msg`

Keplerian broadcast ephemeris (`eph_t`). See [`GnssEphemeris.msg`](GnssEphemeris.msg)
for the full field list including:

- Identification: `system` (`G`/`E`/`J`/`C`/`I`/`S`), `prn`, `satid`, `week`
- Health/quality: `sva`, `code`, `iode`, `iodc`, `svh`
- Reference times: `toc`, `ttr`, `toe`, `toes` (all GPST seconds)
- Keplerian elements: `a`, `e`, `i0`, `omg0`, `omg`, `m0`, `deln`, `omgd`, `idot`
- Harmonic corrections: `crc`, `crs`, `cuc`, `cus`, `cic`, `cis`
- Clock: `f0`, `f1`, `f2`
- Group delay: `tgd[]` (length depends on constellation)

## `GlonassEphemeris.msg`

GLONASS state-vector ephemeris (`geph_t`). Position / velocity / acceleration
are expressed in the **PZ-90.11 ECEF** frame.

| Field | Type | Unit | Notes |
|---|---|---|---|
| `system` | `string` | — | Always `"R"` |
| `prn` | `uint16` | 1 – 27 | Slot number |
| `frq` | `int8` | −7 – +13 | FDMA channel |
| `iode`, `svh`, `age` | uint8 | — | Frame number, health, age (days) |
| `tof`, `toe` | `float64` | s (GPST) | Frame time, ephemeris reference time |
| `pos[3]` | `float64[]` | m | PZ-90.11 ECEF position |
| `vel[3]` | `float64[]` | m/s | PZ-90.11 ECEF velocity |
| `acc[3]` | `float64[]` | m/s² | Lunar/solar acceleration |
| `gamn`, `taun`, `dtaun` | `float64` | — / s / s | Clock frequency bias, clock bias, inter-frequency bias |

## `GnssEphemerides.msg`

Combined ephemeris message. Every published message is a **complete snapshot**
of all currently-valid ephemerides held by the publisher — there are no
delta-only messages on the topic. Publishers emit whenever the store changes
(new toe arrival) and additionally on a heartbeat interval (default 30 s) so
that late-joining subscribers and publisher restarts are covered.

This means logging the single `/gnss/ephemeris` topic is sufficient for both
RINEX NAV conversion (`rosbag_to_rinex` deduplicates by `(sat, iode, iodc, code)`)
and real-time positioning (consumers can subscribe with `transient_local`
depth=1 and receive the full state immediately).

| Field | Type | Notes |
|---|---|---|
| `header` | `std_msgs/Header` | `stamp` = ROS Time (see [header.stamp convention](#headerstamp-convention-applies-to-all-messages-here)) |
| `gnss_ephemeris` | `GnssEphemeris[]` | All currently-valid Keplerian entries (GPS/Galileo/QZSS/BeiDou/NavIC/SBAS) |
| `glonass_ephemeris` | `GlonassEphemeris[]` | All currently-valid GLONASS entries |

## `GnssSolution.msg`

Receiver-side or post-processed positioning solution. Provides the same fix
in three representations (LLH, ECEF, local ENU) with covariances.

**Receiver-side fix sources** (published on `/gnss/nmea_solution`): each driver
locks its source at startup based on YAML flags:

| Receiver | BINARY source (enable flag) | NMEA fallback |
|---|---|---|
| Septentrio SBF | `messages.pvt_geodetic` or `messages.pvt_cartesian` (with the matching `pos_cov_*` block) | NMEA GGA/RMC/GSA/GST |
| NovAtel OEM7   | `messages.bestpos` (+ optional `bestvel`) | NMEA |
| u-blox UBX     | `messages.nav_pvt` | NMEA |

BINARY sources produce full-precision values with proper 3×3 covariance
matrices. NMEA fallback fills LLH/ECEF/ENU but typically only diagonal
covariance from GST. No mid-session switching: if the binary stream stops,
`/gnss/nmea_solution` publication pauses rather than falling back to NMEA.

### Time and quality

| Field | Type | Unit | Notes |
|---|---|---|---|
| `header` | `std_msgs/Header` | — | `stamp` = ROS Time (see [header.stamp convention](#headerstamp-convention-applies-to-all-messages-here)) |
| `time_week` | `uint32` | week | GPS week (extended week — survives 1024-week rollover) |
| `time_tow` | `float64` | s | Time of week |
| `status` | `uint8` | enum | See enum below |
| `num_sats` | `uint8` | — | Satellites used |
| `ratio` | `float32` | — | AR ratio (RTK ambiguity reliability) |
| `age_diff` | `float32` | s | Age of differential corrections |
| `gdop` / `pdop` / `hdop` / `vdop` | `float32` | — | DOP values; **`NaN` when no fresh DOP is available for this epoch** (see "DOP semantics" below) |

`status` constants:

| Value | Symbol | Meaning |
|---|---|---|
| 0 | `STATUS_NONE` | No solution |
| 1 | `STATUS_FIX` | Fixed ambiguity (RTK fix) |
| 2 | `STATUS_FLOAT` | Float ambiguity (RTK float) |
| 3 | `STATUS_SBAS` | SBAS-corrected |
| 4 | `STATUS_DGPS` | Differential GNSS |
| 5 | `STATUS_SINGLE` | Standalone GNSS (SPP) |
| 6 | `STATUS_PPP` | Precise Point Positioning |

### Global position / velocity

| Field | Type | Unit | Frame |
|---|---|---|---|
| `latitude` / `longitude` / `altitude` | `float64` | deg / deg / m | WGS84 ellipsoidal |
| `pos_ecef` | `geometry_msgs/Point` | m | WGS84 ECEF |
| `pos_cov_ecef` | `float64[9]` | m² | 3×3 row-major |
| `vel_ecef` | `geometry_msgs/Vector3` | m/s | WGS84 ECEF |
| `vel_cov_ecef` | `float64[9]` | (m/s)² | 3×3 row-major |

### Local ENU

The first solution sets `pos_enu_org_ecef` (subsequent messages keep that origin). All
subsequent `pos_enu` / `vel_enu` are relative to this origin.

| Field | Type | Unit |
|---|---|---|
| `pos_enu_org_ecef` | `geometry_msgs/Point` | m (ECEF) |
| `pos_enu` | `geometry_msgs/Point` | m (East, North, Up) |
| `pos_enu_cov` | `float64[9]` | m² |
| `vel_enu` | `geometry_msgs/Vector3` | m/s |
| `vel_enu_cov` | `float64[9]` | (m/s)² |

#### Frame conventions (important)

Position and covariance/velocity use **different reference frames**:

- **`pos_enu`** is anchored at `pos_enu_org_ecef`: axes aligned with the tangent plane at
  the initial fix. The vector is a displacement from `pos_enu_org_ecef`.
- **`pos_enu_cov`, `vel_enu`, `vel_enu_cov`** are reported in the **tangent plane
  at the current receiver position** (not at `pos_enu_org_ecef`). This matches the
  convention used by every receiver this library decodes (u-blox NAV-COV's NED
  frame, Septentrio PosCovGeodetic's lat/lon/height stddev, NovAtel BESTPOS's
  lat/lon/hgt stddev) and by RTKLIB's `.pos` `sdn/sde/sdu` columns.

For sub-km baselines the two frames are equivalent within ~baseline/R_earth
(≈ 10⁻⁴ rad). For strict frame consistency or long baselines, prefer the ECEF
fields (`pos_cov_ecef`, `vel_cov_ecef`) which are frame-invariant.

When the receiver outputs ECEF directly (Septentrio PVTCartesian/PosCovCartesian,
u-blox NAV-POSECEF/NAV-VELECEF, NovAtel BESTXYZ), those values are used as the
ECEF truth source. Otherwise ECEF is derived from ENU by rotation at the current
solution's lat/lon.

---

### DOP semantics

**Binary path** (SBF `DOP`, NovAtel `PSRDOP`, u-blox `NAV-DOP`): the most
recently parsed DOP block is cached across PVT epochs. At publish time, the
cached DOP is written into the message only when

```
0 <= (PVT_tow_ms - DOP_tow_ms) <= PVT_period_ms
```

That is, the cached DOP must belong to **this PVT epoch** (DOP arrived before
PVT in the same receiver frame, `dt = 0`) or to **the immediately-prior PVT
epoch** (DOP arrived just after the previous PVT's flush and is now attached
to the current publish, `dt = period`). The PVT period is auto-detected from
consecutive PVT timestamps:

- 1 Hz PVT → accept `dt` in [0, 1000] ms.
- 10 Hz PVT → accept `dt` in [0, 100] ms.
- 0.2 Hz PVT → accept `dt` in [0, 5000] ms.

On the very first PVT epoch the period is not yet known, so DOP fields are
`NaN`. From the second epoch onward, DOP is populated whenever the cache is
fresh and stays `NaN` whenever it is more than one period stale, comes from a
"future" epoch, or is absent. The receiver-side staleness is bounded at one
PVT cycle.

**NMEA path**: `hdop` is taken directly from GGA field 8 (always finite when
GGA reports a valid fix). `pdop` and `vdop` come from GSA fields 15 / 17 via a
persistent cache that survives one cycle of "no GSA arrival" before
invalidating to `NaN` (see the NMEA aggregator section below for details).
`gdop` is always `NaN` (GSA has no `tdop`, so geometric DOP is not derivable).
GSA does not gate flush timing, so this DOP path adds zero publish latency.

This convention guarantees:

- **Zero publish latency**: PVT is published immediately when complete; the
  DOP handler only writes to a persistent cache and never gates the flush.
- **No future-direction bleed-through**: a published PVT epoch never receives
  a DOP that belongs to a *later* receiver epoch. Receiver-side staleness is
  bounded at one PVT cycle.
- **Clean stop on disable**: when the receiver stops emitting the DOP block,
  the cache ages out within one cycle and all subsequent epochs return `NaN`.
- **Explicit absence**: downstream code can detect "no DOP for this epoch"
  with `std::isnan(msg->gdop)` and decide how to handle it (skip, hold last
  finite value, etc.).

Typical scenarios:

| Receiver config | DOP rate | Field values |
|---|---|---|
| u-blox NAV-PVT + NAV-DOP, same rate | matches PVT | finite every epoch (after the first) |
| Septentrio PVT 5 Hz + DOP 1 Hz | < PVT | finite once every 5 epochs (the one adjacent to a DOP arrival); **NaN** elsewhere |
| NovAtel BESTPOS + PSRDOP, same rate | matches PVT | finite every epoch (after the first) |
| First epoch (period not yet detected) | — | **NaN** |
| Receiver stops emitting DOP | — | **NaN** within one cycle |
| No DOP block enabled | — | always **NaN** |

### NMEA path: per-epoch aggregator

NMEA sentences are aggregated into one solution per UTC epoch. The parser
maintains a pending buffer keyed by seconds-of-day; flushes happen either
eagerly (when every sentence type ever seen this session has arrived for the
current epoch) or on the boundary (when a sentence with a new sod arrives).
Flush is gated only on GGA/RMC/GST — GSA is processed but never gates flushing,
so it cannot introduce publish latency.

- **GGA** (position, time, hdop): drives `latitude`/`longitude`/`altitude`,
  `pos_ecef`, `time_week`/`time_tow`, `status`, `num_sats`, `age_diff`, `hdop`.
- **RMC** (velocity): speed (knots → m/s) and course (deg true → rad) fill
  `vel_enu.x/y/z` directly and `vel_ecef.x/y/z` via ENU→ECEF rotation using
  GGA's lat/lon. The A/V flag is **ignored** — velocity is populated whenever
  the numeric fields are finite, since many receivers report V even with
  usable speed/course. Downstream consumers can gate on `solution.status`
  (driven by GGA quality) for nav-validity.
- **GST** (covariance): std_lat/std_lon/std_alt populate `pos_enu_cov` (diag)
  and `pos_cov_ecef` via ENU→ECEF rotation.
- **GSA** (PDOP/VDOP): cached in the parser across resets; applied at flush
  if a fresh GSA arrived within the last cycle. After 2 consecutive flushes
  without a fresh GSA, the cache invalidates and `pdop`/`vdop` go to `NaN`.
  `gdop` stays `NaN` (GSA has no `tdop`, so geometric DOP is not derivable).
  `hdop` is sourced from GGA (not GSA) for consistency.
- **Velocity covariance**: NMEA has no source for `vel_enu_cov` / `vel_cov_ecef`
  (GST provides position stddev only). Both arrays are always `NaN`.

The aggregator learns the receiver's per-cycle sentence set on the first
boundary flush; subsequent epochs eager-flush as soon as that set is complete,
so steady-state publish latency is bounded by the time between the *last*
expected sentence of the cycle and the next sentence. GGA-first or GGA-last
ordering both work — no receiver-side reconfiguration is required.

## RINEX / RTKLIB correspondence

- `GnssObservation` <-> RINEX OBS observation record (one satellite, one epoch)
- `GnssEphemeris`   <-> RINEX NAV record for GPS / Galileo / QZSS / BeiDou / NavIC / SBAS
- `GlonassEphemeris` <-> RINEX NAV record `R##` with state-vector fields
- `GnssSolution`    <-> RTKLIB `.pos` row (LLH or ECEF; see [`src/converter/`](../src/converter/))

For the conversion CLI (RINEX/`.pos` <-> ROS 2 bag), see
[`src/converter/README.md`](../src/converter/README.md).
