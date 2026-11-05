# Messages

| Message | Content | RTKLIB equivalent |
|---|---|---|
| [`GnssObservation`](GnssObservation.msg) | one signal of one satellite | one signal slot of `obsd_t` |
| [`GnssObservations`](GnssObservations.msg) | all signals of all satellites at one epoch | `obsd_t[]` |
| [`GnssEphemeris`](GnssEphemeris.msg) | Kepler ephemeris (GPS, Galileo, QZSS, BeiDou, NavIC) | `eph_t` |
| [`GlonassEphemeris`](GlonassEphemeris.msg) | GLONASS ephemeris | `geph_t` |
| [`GnssEphemerides`](GnssEphemerides.msg) | all retained ephemerides and ionosphere parameters | `eph_t[]`, `geph_t[]`, `nav_t.ion_*` |
| [`GnssSolution`](GnssSolution.msg) | position, velocity, covariances in LLH, ECEF and ENU | `sol_t` |

Units, frames and enumerations are documented in each `.msg` file. Time, missing
values and per-field rules: [topic contract](../docs/TOPIC_DATA_CONTRACT.md).
