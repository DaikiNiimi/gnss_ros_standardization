# `gnss_imu_fgo` — Tightly-coupled GNSS/IMU factor-graph optimization

Extends [`tightly_coupled_gnss`](../tightly_coupled_gnss/) with GTSAM's IMU
preintegration: between consecutive GNSS epochs the raw IMU samples are
integrated into one `CombinedImuFactor`, and the DD pseudorange/carrier-phase
measurements attach to the same body pose through the **Arm** factor variants
(`DoubleDifferencePseudorangeFactorArm` / `DoubleDifferenceCarrierPhaseFactorArm`).
Read the GNSS-only example's README first — GTSAM build, base position, AR,
and limitations all apply here too.

## The frame bridge (core of this example)

IMU preintegration needs a **gravity-aligned navigation frame**; DD factors
are inherently **ECEF**. The Arm factor variants take a constant `ecef_T_nav`
transform plus a body-frame lever arm and convert internally:

```
antenna_ecef = ecef_T_nav * ( X(k) * lever_arm )
```

so the body poses `X(k)` can live in a local ENU frame (anchored at the first
GNSS fix) where `CombinedImuFactor` is valid — one graph couples both sensors
with no manual frame juggling.

```
/gnss/observation  /base/gnss/observation  /gnss/ephemeris   /gnss/imu/data_raw
        │                  │                    │                  │
        ▼                  ▼                    ▼                  ▼
          GnssPreprocessor (this package)             IMU buffer
        │                                                  │
        ▼  PreprocessedEpoch                               ▼ samples in (t_k-1, t_k]
   DD Arm factors on X(k)  ◄──── one graph ────►  CombinedImuFactor
   (lever arm + ecef_T_nav)                       X,V,B (k-1) → X,V,B (k)
        │
        ▼
   ISAM2 + LAMBDA  →  /gnss/fgo/solution
```

**States** per epoch: `X(k)` body `Pose3` (ENU), `V(k)` velocity (ENU),
`B(k)` IMU bias, plus **per-epoch** carrier ambiguities `B(k)^i` (cycles, fresh
each epoch — as in [`tightly_coupled_gnss`](../tightly_coupled_gnss/README.md)).
The [reference diagram](../../fig/gnss_imu_fgo_graph.svg) draws the IMU
link as a separate pre-integration factor plus an IMU-bias between-factor; this
node uses GTSAM's `CombinedImuFactor`, which **bundles both** (pre-integration +
bias random walk) into one factor — the more rigorous, officially-recommended
form, equivalent in information.

The DD carrier on the body pose is only a tight centimeter constraint once its
integer ambiguities are **fixed**; with the per-epoch float it is meter-level,
so reliable per-epoch FIX (dual frequency, enabled by default) is what gives
the centimeter pose and lets the IMU observe attitude and gyro bias.

## Running

```bash
ros2 run gnss_ros_standardization gnss_imu_fgo --ros-args \
  --params-file config/gnss_imu_fgo.yaml
```

| Direction | Default | Type |
|---|---|---|
| Sub | `topics.rover_observation` (`/gnss/observation`) | `GnssObservations` |
| Sub | `topics.base_observation` (`/base/gnss/observation`) | `GnssObservations` |
| Sub | `topics.ephemeris` (`/gnss/ephemeris`) | `GnssEphemerides` |
| Sub | `topics.imu` (`/gnss/imu/data_raw`) | `sensor_msgs/Imu` |
| Pub | `topics.solution` (`/gnss/fgo/solution`) | `GnssSolution` |

The IMU topic matches what this package's drivers publish for IMU-equipped
receivers (u-blox ADR, Septentrio INS, NovAtel SPAN).

Key extra parameters ([config/gnss_imu_fgo.yaml](../../config/gnss_imu_fgo.yaml)):

- `lever_arm` — body-frame translation from the body/IMU origin to the GNSS
  antenna phase center [m].
- `imu.sigma_*` — noise densities (defaults follow
  `gnss_imu_kalman_filter.yaml`).
- `init.*` — static initialization settings (below).
- `local_origin` — ENU anchor (`gnss_fix` = first GNSS a-priori, or `manual`).

## Initialization and observability

The platform is assumed **stationary** for the first `init.imu_duration`
seconds: roll/pitch come from the averaged accelerometer (gravity direction),
the gyro bias from the averaged gyro. **Yaw is unobservable at rest** — it
starts with a π prior and converges from the DD+IMU coupling once the platform
accelerates or turns. A permanently static platform never observes yaw (the
position estimate is unaffected; only the attitude stays degenerate).

## Time bases (accepted approximation)

GNSS epochs are GPST (`week`/`tow`); IMU samples carry ROS stamps. They are
paired via the rover observation's ROS header stamp (PC arrival time), like
the loose-coupled EKF example does. Receiver output latency (typically tens of
ms) therefore shifts the IMU integration boundaries by the same amount. For
publication-grade results, timestamp the rover observations with
`use_gps_timestamp` (driver option) and hardware-synchronized IMU stamps.

The math of the Arm-factor / ENU-bridge / preintegration chain is validated on a
static-platform synthetic scenario with a perfect IMU: the integers are
recovered exactly and the FIX is centimeter-accurate.
