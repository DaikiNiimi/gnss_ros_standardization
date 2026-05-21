# Message Reference

ROS 2 messages that form the public interface of `gnss_ros_standardization`.
Field names and semantics follow [RTKLIB](https://github.com/rtklibexplorer/RTKLIB)
C structures (`obsd_t`, `eph_t`, `geph_t`) so decoders, converters and
positioning nodes map directly to/from RTKLIB internals.

| Message | RTKLIB struct type | Description |
|---|---|---|
| [`GnssObservation.msg`](GnssObservation.msg) | `obsd_t` (one satellite at one epoch) | Single pseudorange / carrier-phase / Doppler / SNR sample |
| [`GnssObservations.msg`](GnssObservations.msg) | array of `obsd_t` at one epoch | All satellite observations at a single epoch |
| [`GnssEphemeris.msg`](GnssEphemeris.msg) | `eph_t` | Keplerian ephemeris (GPS / Galileo / QZSS / BeiDou / NavIC / SBAS) |
| [`GlonassEphemeris.msg`](GlonassEphemeris.msg) | `geph_t` | GLONASS state-vector ephemeris |
| [`GnssEphemerides.msg`](GnssEphemerides.msg) | array of `eph_t` and `geph_t` | All satellite ephemerides |
| [`GnssSolution.msg`](GnssSolution.msg) | RTKLIB `sol_t` + ENU extensions | Position and velocity solution (LLH / ECEF / ENU + covariances) |

Per-field semantics (units, enums, frames, NaN/staleness rules) live inline
in each `.msg` file. Open the file linked in the table above for the full
spec.
