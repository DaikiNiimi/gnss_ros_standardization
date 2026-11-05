#pragma once
// Fixed-lag smoother engine shared by gnss_fgo and gnss_imu_fgo.
#include "model.hpp"
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <cstdint>
#include <functional>
#include <optional>

namespace gnss_fgo {
struct ArDecision {
  int nb = 0, dropped = 0;
  double s0 = 0, s1 = 0;
  Vec3 fixed_position = Vec3::Zero();
  // Antenna position covariance in the local frame, conditioned on the fixed
  // integer combinations (NaN when it could not be formed).
  Mat3 fixed_covariance = Mat3::Constant(std::numeric_limits<double>::quiet_NaN());
  std::map<SF, double> fixed;
  std::set<SF> allowed;
};
struct DdPosition {
  std::optional<Vec3> position;
  int active = 0;
  double rms = std::numeric_limits<double>::infinity();
};
class Engine {
public:
  Engine(const Config &config, const Vec3 &initial, const Vec3 &base,
         const Vec3 &lever);
  void addImu(const ImuSample &sample);
  Solution process(const ObservationEpoch &observation);
  // Read-only checkpoints for differential verification; not solver inputs.
  const SatelliteStates &satelliteStates() const { return sats_; }
  const EpochState &epochState() const { return current_; }
  int phase() const { return phase_; }
  Vec3 floatPosition() const { return nav_position_; }
  int graphEpoch() const { return graph_epoch_; }
  // IMU samples dropped because more than kImuBacklog were waiting unconsumed
  // (no GNSS epoch arrived to consume them).
  std::uint64_t imuBacklogDropped() const { return imu_backlog_dropped_; }
  static constexpr std::size_t kImuBacklog = 120000;
  int cpHoldRemaining() const { return cp_hold_; }
  int starvationCount() const { return starvation_; }
  using ArObserver =
      std::function<void(int, bool, const Eigen::VectorXd &,
                         const Eigen::MatrixXd &, int, double, double, int)>;
  void observeAr(ArObserver observer) { ar_observer_ = std::move(observer); }
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
  using Smoother = gtsam::IncrementalFixedLagSmoother;
  using Graph = gtsam::NonlinearFactorGraph;
  using Values = gtsam::Values;
  Config cfg_;
  ArObserver ar_observer_;
  GraphObserver graph_observer_;
  enum class ArOutcome {
    NotCalled,
    Entered,
    Unposed,
    MinPairs,
    MinSatellites,
    Exception,
    Zero,
    Success
  };
  ArOutcome ar_outcome_ = ArOutcome::NotCalled;
  Vec3 base_, lever_, nav_position_;
  Mat3 R_;
  gtsam::Pose3 frame_;
  SatelliteStates sats_;
  EpochState current_;
  std::unique_ptr<Smoother> p1_, p2_;
  int phase_ = 1, input_epoch_ = 0, graph_epoch_ = 0, previous_mode_ = 5,
      cp_hold_ = 0, starvation_ = 0;
  bool healthy_ = true;
  double phase1_time_ = 0, graph_time_ = 0, dt_ = .2, current_gpst_ = 0;
  std::optional<double> last_gpst_, graph_gpst_;
  std::set<Key> p1_inserted_;
  std::vector<ImuSample> imu_;
  std::size_t imu_index_ = 0;
  std::uint64_t imu_backlog_dropped_ = 0;
  void pruneImu();
  void fillCovariance(Solution &out) const;
  std::map<int, double> per_sat_res_;
  std::map<int, double> ar_per_sat_res_;
  std::vector<int> residual_sat_order_;
  double main_res_ = 0, actual_lag_ = 1;
  int bad_count_ = 0;
  bool collecting_ = false, pim_break_ = false;
  gtsam::imuBias::ConstantBias bias_, initial_bias_;
  std::optional<gtsam::Pose3> zupt_anchor_;
  std::vector<std::pair<double, Vec3>> fix_history_;
  struct CollectedFix {
    double time;
    Vec3 position, velocity;
    std::map<SF, int> arcs;
    std::vector<ImuSample> imu;
    ObservationEpoch observation;
  };
  std::vector<CollectedFix> collection_;
  static std::unique_ptr<Smoother> makeSmoother(double lag,
                                                const Config &config);
  void slips(const ObservationEpoch &obs);
  int reference(const ObservationEpoch &obs, const std::vector<int> &indices);
  int buildDd(Graph &graph, Values &values, const ObservationEpoch &obs,
              Key pose, const Vec3 &lever, int dd_epoch = 0,
              const std::map<SF, std::pair<Key, double>> &previous = {},
              bool skip_cp = false);
  DdPosition ddprPosition(const ObservationEpoch &obs,
                          const gtsam::Pose3 &initial, const Vec3 &lever);
  // `slots` are the smoother slots of `graph`'s factors, in order (from the
  // ISAM2 result of the update that added them; slots are reused).
  Values fde(Smoother &sm, const Graph &graph, const gtsam::FactorIndices &slots,
             Values estimate, Key pose, int nv = -1);
  ArDecision resolve(Smoother &sm, const Values &estimate, Key pose,
                     const ObservationEpoch &obs,
                     const std::set<int> &exclude = {}, bool partial = false,
                     std::optional<gtsam::JointMarginal> *covariance_cache = nullptr);
  ArDecision runAr(Smoother &sm, const Values &estimate, Key pose,
                   const ObservationEpoch &obs);
  void hold(Smoother &sm, const ArDecision &ar);
  void restartPhase1();
  Solution phase1(const ObservationEpoch &obs);
  void collect(const ObservationEpoch &obs, const Solution &solution);
  void transition();
  Solution phase2(const ObservationEpoch &obs, bool gnss);
  std::vector<ImuSample> collectImu(double until);
  void addPseudo(Graph &graph, int index, const gtsam::Pose3 &previous,
                 const Vec3 &velocity, const std::vector<ImuSample> &samples,
                 const Vec3 &gyro);
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
  residuals(const Graph &graph, const Values &values) const;
  std::optional<Solution> sanity(const Graph &graph, const gtsam::Pose3 &pose,
                                 const gtsam::NavState &pred, int index);
  Vec3 antenna(const gtsam::Pose3 &pose) const {
    return base_ +
           R_ * (pose.translation() + pose.rotation().matrix() * lever_);
  }
  Solution finalize(Solution solution);
};
} // namespace gnss_fgo

namespace gnss_fgo {
// Equality can release a rover before the same-time base callback is admitted
// on another ROS subscription. Crossing the epoch provides a stream-time
// barrier. The later IMU value is NOT authorized for integration/initialization:
// Engine::collectImu and BroadcastStore still restrict values to the epoch.
// At explicit EOF, all admitted streams are complete and equality can drain.
inline bool epochReady(std::int64_t rover_ms,
                       std::optional<std::int64_t> imu_watermark_ms,
                       bool eof = false) {
  return imu_watermark_ms &&
         (*imu_watermark_ms > rover_ms ||
          (eof && *imu_watermark_ms == rover_ms));
}
} // namespace gnss_fgo
