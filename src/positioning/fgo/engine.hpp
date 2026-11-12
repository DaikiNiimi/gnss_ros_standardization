#pragma once
// Fixed-lag smoother engine shared by gnss_fgo and gnss_imu_fgo.
#include "ambiguity.hpp"
#include "model.hpp"
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <cstdint>
#include <functional>
#include <optional>

namespace gnss_fgo {
struct ArDecision {
  int nb = 0;
  double s0 = 0, s1 = 0;
  Vec3 fixed_position = Vec3::Zero();
  // Antenna position covariance in the local frame, conditioned on the fixed
  // integer combinations (NaN when it could not be formed).
  Mat3 fixed_covariance =
      Mat3::Constant(std::numeric_limits<double>::quiet_NaN());
  std::map<SF, double> fixed;
  std::set<SF> allowed;
  // (reference, target) of every accepted DD integer N_ref - N_target =
  // fixed[ref] - fixed[target]; Engine::hold constrains these pairs.
  std::vector<std::pair<SF, SF>> fixed_pairs;
};
// One DD factor appended by Engine::buildDd. The official GTSAM DD factors
// carry no satellite ids, so the readers (FDE, residuals) look them up here;
// `index` pairs the row with its factor in the graph buildDd appended to.
struct DdRow {
  std::size_t index;
  bool carrier;
  int ref_sat, target_sat, band;
};
using DdRows = std::vector<DdRow>;
struct DdPosition {
  std::optional<Vec3> position;
  int active = 0;
  double rms = std::numeric_limits<double>::infinity();
};
// The linear factor `f` (a marginal factor of the fixed-lag smoother) with the
// variables in `drop` integrated out (Schur complement; directions of them `f`
// does not determine carry no information and are skipped), linearized at the
// kept keys' part of f's linearization point. Null when `f` has none of them
// or nothing else.
gtsam::NonlinearFactor::shared_ptr
marginalWithout(const gtsam::LinearContainerFactor &f,
                const std::set<Key> &drop);
class Engine {
public:
  Engine(const Config &config, const Vec3 &initial, const Vec3 &base,
         const Vec3 &lever);
  void addImu(const ImuSample &sample);
  Solution process(const ObservationEpoch &observation);
  // Read-only views of the estimator state (tests and the ROS node).
  const SatelliteStates &satelliteStates() const { return sats_; }
  const EpochState &epochState() const { return current_; }
  Vec3 floatPosition() const { return nav_position_; }
  int graphEpoch() const { return graph_epoch_; }
  // IMU samples dropped because too many were waiting unconsumed (no GNSS
  // epoch arrived to consume them): once more than kImuBacklog +
  // kImuBacklogBlock wait, the oldest are dropped down to kImuBacklog.
  std::uint64_t imuBacklogDropped() const { return imu_backlog_dropped_; }
  static constexpr std::size_t kImuBacklog = 120000;
  static constexpr std::size_t kImuBacklogBlock = kImuBacklog / 10;
  using GraphObserver = std::function<void(
      int, const gtsam::NonlinearFactorGraph &, const gtsam::Values &)>;
  void observeGraph(GraphObserver observer) {
    graph_observer_ = std::move(observer);
  }

private:
  friend struct EngineTestAccess;
  // Regression tests inject failures before/after a real graph mutation.
  // There is no ROS or runtime parameter exposing this hook.
  std::function<void(const char *, bool)> update_probe_;
  void updateProbe(const char *operation, bool after) {
    if (update_probe_)
      update_probe_(operation, after);
  }
  // Regression tests switch the ambiguity gauge prior (buildDd) off to compare
  // with the ungauged graph. No ROS or runtime parameter exposes it.
  bool ambiguity_gauge_ = true;
  // Regression tests switch the Doppler-only switch gate off to show that a
  // fixture needs it. No ROS or runtime parameter exposes it.
  bool doppler_gate_ = true;
  // Sigma [cycles] of that gauge prior.
  static constexpr double kAmbiguityGaugeSigma = .01;
  // Regression tests switch the plausibility guard (phase2) off to show that a
  // fixture needs it. No ROS or runtime parameter exposes it.
  bool plausibility_guard_ = true;
  // Plausibility guard: a graph posterior is not committed when its antenna
  // is farther from the last committed antenna position than kMaxSpeed [m/s]
  // times the elapsed time plus kMaxStepMargin [m], or its speed exceeds
  // kMaxSpeed. Fastest vehicle speed:
  static constexpr double kMaxSpeed = 150.;
  // Estimation error allowed on top of the travelled distance (a regular
  // update can move the estimate by several hundred metres in one epoch):
  static constexpr double kMaxStepMargin = 1000.;
  // ZUPT speed gate [m/s] on the previous posterior velocity: the IMU
  // statistics alone also pass at steady driving speed.
  static constexpr double kZuptMaxSpeed = .5;
  // Switch gate (updateSwitchWindow): the Doppler-only horizontal velocity
  // must agree with the posterior velocity of the motion graph.
  // Largest direction difference [deg].
  static constexpr double kSwitchMaxHeadingDiffDeg = 20.;
  // Speed difference allowed: this many m/s ...
  static constexpr double kSwitchSpeedTolMps = .5;
  // ... or this fraction of the posterior speed, whichever is larger.
  static constexpr double kSwitchSpeedTolFrac = .3;
  // Largest yaw rate [deg/s] of a window epoch: the antenna velocity
  // direction is not the body heading while turning.
  static constexpr double kSwitchMaxYawRateDps = 3.;
  // Smallest window speed change [m/s], GNSS and IMU, that tells forward
  // from reverse motion.
  static constexpr double kSwitchMinSpeedChange = .5;
  // Speed [m/s] at which forward motion is assumed when the speed change
  // does not tell (a road vehicle does not reverse that fast).
  static constexpr double kSwitchForwardSpeed = 5.;
  // Time [s] the switch window spans (at least 2 epochs); the direction test
  // integrates over it.
  static constexpr double kSwitchWindowTime = 1.;
  // IMU-graph integrity: largest angle [deg] between the Doppler-only
  // velocity and the body x axis (both horizontal) at kSwitchForwardSpeed or
  // more.
  static constexpr double kIntegrityMaxHeadingDiffDeg = 45.;
  // Time [s] a violation (heading, or a sanity recovery in every GNSS epoch)
  // must last before the IMU graph is declared invalid; a longer gap between
  // counting epochs restarts it.
  static constexpr double kIntegrityTime = 2.;
  using Smoother = gtsam::IncrementalFixedLagSmoother;
  using Graph = gtsam::NonlinearFactorGraph;
  using Values = gtsam::Values;
  Config cfg_;
  GraphObserver graph_observer_;
  // Outcome of the last runAr call (read by the regression tests): Zero = the
  // integer search fixed nothing (also when it could not be posed), MinPairs =
  // only one DD pair could be formed, NotCalled = neither (AR off, an
  // exception, or a fix).
  enum class ArOutcome { NotCalled, MinPairs, Zero };
  ArOutcome ar_outcome_ = ArOutcome::NotCalled;
  Vec3 base_, lever_, nav_position_;
  Mat3 R_;
  gtsam::Pose3 frame_;
  SatelliteStates sats_;
  EpochState current_;
  // The graph in use: the motion graph (antenna state, MotionFactor) in the
  // motion stage, the IMU graph after the switch (gnss_imu_fgo only).
  std::unique_ptr<Smoother> p2_;
  // gnss_fgo: always 2 (its only graph, the motion graph). gnss_imu_fgo: 1 in
  // the motion stage, 2 after the switch to the IMU graph.
  int phase_ = 1, input_epoch_ = 0, graph_epoch_ = -1, cp_hold_ = 0;
  bool healthy_ = true;
  double graph_time_ = 0, dt_ = .2, current_gpst_ = 0;
  std::optional<double> last_gpst_, graph_gpst_;
  // True while the graph in use is the motion graph: always for gnss_fgo, for
  // gnss_imu_fgo until the switch. The graph-describing branches follow it.
  bool motionStage() const { return cfg_.gnss_motion || phase_ == 1; }
  // Lever arm of the graph in use: the motion state is the antenna itself.
  Vec3 stageLever() const { return motionStage() ? Vec3::Zero() : lever_; }
  std::vector<ImuSample> imu_;
  std::size_t imu_index_ = 0;
  std::uint64_t imu_backlog_dropped_ = 0;
  void pruneImu();
  void fillCovariance(Solution &out) const;
  std::map<int, double> per_sat_res_;
  std::map<int, double> ar_per_sat_res_;
  double main_res_ = 0, actual_lag_ = 1;
  int bad_count_ = 0;
  bool pim_break_ = false;
  gtsam::imuBias::ConstantBias bias_, initial_bias_;
  std::optional<gtsam::Pose3> zupt_anchor_;
  // Switch window (gnss_imu_fgo motion stage): the last consecutive epochs
  // that satisfy the switch condition (updateSwitchWindow), oldest first.
  struct WindowEpoch {
    double time;
    // Output antenna position (ECEF) and the GPSFactorArm sigma [m] for it.
    Vec3 position;
    double position_sigma;
    bool fixed;
    // Posterior antenna velocity and its covariance, local ENU.
    Vec3 velocity;
    Mat3 velocity_covariance;
    // Ambiguity keys of the motion graph after the epoch: transition() uses an
    // epoch's carrier phase only on the arcs that last to the window end.
    AmbMap keys;
    // IMU samples after the previous epoch up to this one.
    std::vector<ImuSample> imu;
    ObservationEpoch observation;
  };
  std::vector<WindowEpoch> window_;
  // Set by phase2 when the epoch was a regular GNSS update of the motion
  // graph with at least min_dd_for_solve DD rows; reset by process().
  bool motion_update_ = false;
  // gnss_imu_fgo motion stage: the epoch's observation (valid during
  // process()) and the IMU samples it consumed.
  const ObservationEpoch *epoch_observation_ = nullptr;
  std::vector<ImuSample> epoch_imu_;
  // Between-satellite Doppler rows of an epoch after addDoppler's screening:
  // the rows (`sat` points into the epoch's observation), their sigmas, the
  // reference row and the receiver position of the line-of-sight geometry.
  struct DopplerRow {
    const SatelliteObservation *sat;
    double d, lam, snr;
  };
  struct DopplerRows {
    std::vector<DopplerRow> rows;
    std::vector<double> sigmas;
    std::size_t ref = 0;
    Vec3 receiver = Vec3::Zero();
  };
  // gnss_imu_fgo: the rows addDoppler used this epoch (empty when it added
  // none), for the switch gate (motion stage) and integrity check 1 (IMU
  // stage); valid during process().
  std::optional<DopplerRows> epoch_doppler_;
  // Why the last epoch did not qualify for the switch window (the codes in
  // updateSwitchWindow; 0 = qualified, -1 = not evaluated). Read by the
  // regression tests.
  int switch_reject_ = -1;
  // The hold factor of each held pair of ambiguity keys (reference, target):
  // its integer N_target - N_ref and its slot in the smoother hold() added it
  // to. At most one factor per key pair; cleared with every smoother
  // replacement and every removal of ambiguity factors.
  struct HoldFactor {
    double difference;
    std::size_t slot;
  };
  std::map<std::pair<Key, Key>, HoldFactor> holds_;
  // `qr`: QR factorization (the motion graph), else ISAM2's default.
  static std::unique_ptr<Smoother> makeSmoother(double lag,
                                                const Config &config, bool qr);
  void slips(const ObservationEpoch &obs);
  int reference(const ObservationEpoch &obs, const std::vector<int> &indices);
  // Appends the DD factors to `graph` and one row per factor to `rows`. A
  // satellite/band with a key reuses it; one without gets a new key with its
  // prior. A (system, band) group that starts in this call also gets one gauge
  // prior on its reference key (not a DD row).
  int buildDd(Graph &graph, DdRows &rows, Values &values,
              const ObservationEpoch &obs, Key pose, const Vec3 &lever,
              bool skip_cp = false);
  // Arc bookkeeping before and after buildDd in phase2.
  void continueArcs(const Values &estimate);
  void endUnusedArcs();
  DdPosition ddprPosition(const ObservationEpoch &obs,
                          const gtsam::Pose3 &initial, const Vec3 &lever);
  // `slots` are the smoother slots of `graph`'s factors, in order (from the
  // ISAM2 result of the update that added them; slots are reused).
  // `rows` are the DD rows buildDd produced for `graph`.
  Values fde(Smoother &sm, const Graph &graph, const DdRows &rows,
             const gtsam::FactorIndices &slots, Values estimate, Key pose);
  ArDecision resolve(Smoother &sm, const Values &estimate, Key pose,
                     const ObservationEpoch &obs);
  ArDecision runAr(Smoother &sm, const Values &estimate, Key pose,
                   const ObservationEpoch &obs);
  void hold(Smoother &sm, const ArDecision &ar);
  // gnss_imu_fgo motion stage, once per epoch from finalize: extends or
  // restarts the switch window with this epoch and calls transition() when it
  // spans kSwitchWindowTime and the direction of travel is known. A failed
  // transition leaves the motion stage as it was and restarts the window.
  void updateSwitchWindow(const Solution &solution);
  // Initial IMU bias (stationaryBias) and levelling rotation
  // attitude(roll, pitch, 0) from the accelerometer, as transition() forms
  // them from the samples consumed so far; `level` is false (identity) when
  // no IMU has been consumed.
  struct LevelImu {
    gtsam::imuBias::ConstantBias b0;
    Mat3 rotation = Mat3::Identity();
    double roll = 0, pitch = 0;
    bool level = false;
  };
  // After the first switch: the levelling it used (the samples it came from
  // are pruned in the IMU stage), with b0 replaced at a fallback by the last
  // bias the IMU graph estimated while both integrity checks were clean.
  std::optional<LevelImu> saved_level_;
  std::optional<gtsam::imuBias::ConstantBias> good_bias_;
  LevelImu levelImu() const;
  // A run of consecutive violating epochs (integrity checks).
  struct IntegrityTimer {
    std::optional<double> start, last;
    // A violating epoch at t: extends the run, or starts one when there is
    // none or the previous counting epoch is more than `gap` back. Returns
    // the run length.
    double violate(double t, double gap) {
      if (!start || t - *last > gap)
        start = t;
      last = t;
      return t - *start;
    }
    void clear() { start.reset(), last.reset(); }
    double length() const { return start ? *last - *start : 0.; }
  };
  IntegrityTimer heading_timer_, recovery_timer_;
  // IMU stage: phase2 completed a regular update (not a reset) this epoch;
  // sanity() returned a recovery this epoch; the angular rate the Doppler
  // factors used.
  bool integrity_epoch_ = false, sanity_recovery_ = false;
  Vec3 epoch_omega_ = Vec3::Zero();
  // Fallbacks so far and the cause of the last one (1 heading, 2 recovery).
  int n_fallback_ = 0, fallback_cause_ = 0;
  // IMU stage, after phase2: the two integrity checks; the fallback to a
  // fresh motion stage when either fails (cause 1 heading, 2 recovery).
  int integrity(bool gnss);
  Solution fallback(const ObservationEpoch &obs, bool gnss, int cause);
  // Builds the IMU graph from window_ and switches to it; on failure nothing
  // is committed and the exception propagates. `reverse`: the body x axis
  // points against the velocity (heading = velocity direction + 180 deg).
  void transition(bool reverse = false);
  Solution phase2(const ObservationEpoch &obs, bool gnss);
  std::vector<ImuSample> collectImu(double until);
  void addPseudo(Graph &graph, int index, const gtsam::Pose3 &previous,
                 const Vec3 &velocity, const std::vector<ImuSample> &samples,
                 const Vec3 &gyro);
  // The Doppler rows addDoppler turns into factors (none when fewer than 2).
  std::optional<DopplerRows> dopplerRows(const ObservationEpoch &obs,
                                         const gtsam::NavState &pred);
  // Least-squares receiver velocity (local ENU) from the between-satellite
  // Doppler rows alone (unknowns: the 3 velocity components); none with
  // fewer than 4 differenced rows or a rank-deficient / nonfinite solve.
  std::optional<Vec3> dopplerVelocity(const DopplerRows &rows) const;
  void addDoppler(Graph &graph, int index, const ObservationEpoch &obs,
                  const gtsam::NavState &pred, const Vec3 &gyro, int nv);
  void addImuChain(Graph &graph, Values &values, int index,
                   const IntegratedImu &imu, const gtsam::Pose3 &previous,
                   const Vec3 &velocity);
  gtsam::FactorIndices flsUpdate(const Graph &graph, const Values &values,
                                 int index, const std::vector<Key> &keep);
  void warmReset(const Vec3 &body_position, const gtsam::Rot3 &rotation);
  void resetAmbiguities();
  std::pair<double, std::map<int, double>>
  residuals(const Graph &graph, const DdRows &rows, const Values &values) const;
  std::optional<Solution> sanity(const Graph &graph, const DdRows &rows,
                                 const gtsam::Pose3 &pose,
                                 const gtsam::NavState &pred, int index);
  Vec3 antenna(const gtsam::Pose3 &pose, const Vec3 &lever) const {
    return base_ +
           R_ * (pose.translation() + pose.rotation().matrix() * lever);
  }
  // Antenna of a state of the graph in use.
  Vec3 antenna(const gtsam::Pose3 &pose) const {
    return antenna(pose, stageLever());
  }
  Solution finalize(Solution solution);
};
} // namespace gnss_fgo

namespace gnss_fgo {
// Equality can release a rover before the same-time base callback is admitted
// on another ROS subscription. Crossing the epoch provides a stream-time
// barrier. The later IMU value is NOT authorized for integration or
// initialization: Engine::collectImu still restricts values to the epoch.
// At explicit EOF, all admitted streams are complete and equality can drain.
inline bool epochReady(std::int64_t rover_ms,
                       std::optional<std::int64_t> imu_watermark_ms,
                       bool eof = false) {
  return imu_watermark_ms &&
         (*imu_watermark_ms > rover_ms ||
          (eof && *imu_watermark_ms == rover_ms));
}
} // namespace gnss_fgo
