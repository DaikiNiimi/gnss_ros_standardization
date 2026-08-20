// SPDX-License-Identifier: MIT
//
// Tightly-coupled GNSS/IMU factor-graph optimization example.
//
// Extends the gnss_fgo example with GTSAM's IMU preintegration:
// between consecutive GNSS epochs the raw IMU samples are integrated into a
// PreintegratedCombinedMeasurements and added as one CombinedImuFactor; the
// DD pseudorange / carrier-phase measurements attach to the same body pose
// through the "Arm" factor variants (lever arm + ecef_T_nav).
//
// Frame design (the core of this example):
//   - The IMU factor needs a gravity-aligned navigation frame. Poses X(k)
//     therefore live in a local ENU frame anchored at the first GNSS fix.
//   - The DD factors are inherently ECEF. The Arm variants take the constant
//     ecef_T_nav transform and convert internally - exactly the bridge needed
//     to couple both sensor types in one graph.
//
// State per epoch k: X(k) body Pose3 (ENU), V(k) velocity (ENU),
//                    B(k) IMU bias; N(j) carrier ambiguities in cycles.
//
// Time bases: GNSS epochs are paired with IMU samples on the ROS stamp time
// base; time.gnss_epoch_source picks "header" (stamp as-is) or
// "tow_auto_offset" (rebuild from week/tow, strip receiver latency). See
// gnss_utils::GnssEpochAligner.
//
// Initialization: the platform is assumed STATIC for the first
// init_imu_duration seconds. Roll/pitch come from the averaged accelerometer,
// the gyro bias from the averaged gyro.
//
// Yaw is three separate problems:
//   - Stationary it is UNOBSERVABLE. The initial attitude rotates mean specific
//     force onto +Z, which has zero yaw by construction, so body +x points EAST
//     whatever way the vehicle faces; the pi-sigma yaw prior on X(0) says so.
//   - It is constant while stationary, but what drifts is the GAUGE - a common
//     rotation of the whole window about the vertical - which no per-epoch rate
//     constraint can hold.
//   - Its absolute value is set at the first confident motion from the course
//     over ground, once several consecutive epochs AGREE on the offset
//     (attitude.align_* / YawAlignConfig).
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include "ar_debug_dump.hpp"
#include "factor_adapters.hpp"
#include "imu_factors.hpp"
#include "gnss_ros_standardization/gnss_preprocessor.hpp"
#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"

extern "C" {
#include "rtklib.h"
}

namespace grs = gnss_ros_standardization::msg;
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

namespace {
constexpr int kPositionLogThrottleMs = 1000;
constexpr int kMinDdForAr = 3;
// Observation buffering, applied to the subscriptions AND the epoch matcher.
// Both must express the same capacity or the smaller one silently governs.
constexpr int kObsQueueDepth = 512;  // ~100 s at 5 Hz
// Rover silence, in epochs, before the live inertial bridge may claim a slot.
constexpr double kOutageEpochs = 3.0;
// How far behind the inertial front the LIVE bridge stops. Must exceed rover
// delivery lateness; too large only delays slots the data-domain trigger fills
// anyway, so err large.
constexpr double kLiveStandoffEpochs = 10.0;
// Epochs one worker tick may process before returning to the executor, so the
// solver lock is not held for the length of a backlog.
constexpr std::size_t kMaxEpochsPerBatch = 8;
constexpr double kGravity = 9.80665;
// Static-window gates for the accelerometer/gyro averaging initialization.
constexpr double kStaticGyroRadps = 0.05;
constexpr double kStaticAccStdMps2 = 0.5;
constexpr double kStaticAccNormTolMps2 = 0.3;  // |mean acc| must be within this of g
constexpr double kStaticMaxSpeedMps = 0.3;     // GNSS Doppler speed gate for static
// Fixed constants. Held-back GNSS epoch wait,
// tow_auto_offset estimation window, Doppler velocity-factor sigma floor, and
// the initial-state standard deviations.
// PPC IMU is 100 Hz. A gap beyond 100 ms is an outage, not a sample interval to
// clamp and silently integrate; use the explicit loose fallback link instead.
constexpr double kImuMaxSampleGapS = 0.1;
constexpr double kOffsetWindowS = 60.0;
constexpr double kMotionVelSigmaMps = 0.1;
// Conservative floors used ONLY when dead reckoning cannot obtain the joint
// posterior covariance to propagate. Deliberately pessimistic: an outage is the
// wrong place to understate uncertainty, and the previous fallback was the
// optimistic sum it now replaces.
constexpr double kPredictFallbackPosStdM = 5.0;
constexpr double kPredictFallbackVelStdMps = 1.0;
constexpr double kInitPosStd = 10.0;      // [m]
constexpr double kInitVelStd = 0.3;       // [m/s] stationary start
constexpr double kInitAttStdRp = 0.1;     // [rad] roll/pitch
constexpr double kInitAttStdYaw = 3.14159;  // [rad] yaw unknown
constexpr double kInitAccBiasStd = 0.1;   // [m/s^2]
constexpr double kInitGyrBiasStd = 0.01;  // [rad/s]
constexpr double kEpochBudgetMs = 100.0;  // 10 Hz real-time budget per epoch
// Re-anchor gates, mirroring gnss_fgo's loose absolute prior at the code
// a-priori when the motion chain is broken. Without it, a long IMU-only coast
// drifts far enough that every DD fails the pre-fit test against the predicted
// state, which removes the only measurement that could correct it - a state the
// node never leaves. The loose prior restores the observation path without
// discarding the converged attitude / bias (unlike resetGraph + tryInitialize).
constexpr double kReanchorGapS = 5.0;         // GNSS gap that forces a re-anchor
// 100 m, and deliberately NOT the 10 m that gnss_fgo uses for the same-named
// constant. There the prior is the only thing holding the state, so anchoring it
// at the single-point solution's real accuracy helps. Here the IMU chain already
// holds a state far better than SPP, so a 10 m pull to SPP injects a jump the
// inertial prediction then disagrees with - which raises the next
// prediction_fault, which re-anchors again - a loop that can consume most of a
// run. The same constant is deliberately LOOSER here than in gnss_fgo, because
// the state it corrects is of a different quality.
constexpr double kReanchorPosStdM = 100.0;    // translation sigma [m]
constexpr double kReanchorRotStdRad = 10.0;   // rotation left unconstrained
constexpr double kReanchorVelStdMps = 5.0;    // used when Doppler is unavailable
}  // namespace

class GnssImuFgoNode : public rclcpp::Node {
 public:
  GnssImuFgoNode() : Node("gnss_imu_fgo") {
    declareParameters();

    preprocessor_ = std::make_unique<gnss_utils::GnssPreprocessor>(makePreprocessorConfig());
    adapter_cfg_ = makeAdapterConfig();
    amb_mgr_ = std::make_unique<gnss_fgo::PersistentAmbiguities>(next_amb_id_);
    amb_mgr_->setHoldRefresh(get_parameter("ambiguity.hold_refresh_s").as_double());
    amb_max_outage_ = get_parameter("ambiguity.max_outage_s").as_double();
    max_coast_s_ = get_parameter("gap.max_coast_s").as_double();
    smoother_lag_ = smootherLag(get_parameter("graph.lag_s").as_double());

    smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(
        smoother_lag_, makeIsamParams());

    const auto lever = get_parameter("lever_arm").as_double_array();
    if (lever.size() == 3) lever_arm_ = gtsam::Point3(lever[0], lever[1], lever[2]);

    imu_max_wait_s_ =
        std::max(0.0, get_parameter("imu.max_wait_s").as_double());
    imu_queue_depth_ = std::max(
        1, static_cast<int>(get_parameter("imu.queue_depth").as_int()));
    solver_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // Observation intake, separate from the solve so a long solve cannot stall
    // arrival bookkeeping and hide the node's own backlog.
    intake_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // MutuallyExclusive, not Reentrant. The point of a SEPARATE group is that a
    // long solve cannot delay IMU intake and manufacture a fake sensor outage -
    // that is achieved by the group being distinct from solver_group_, not by
    // reentrancy. Reentrancy would additionally let two IMU callbacks run
    // concurrently on the 2-thread executor, and although imu_mtx_ keeps the
    // deque itself safe, the two samples could then be appended OUT OF ORDER.
    // A non-monotonic buffer yields dt < 0 in the preintegration loop, which
    // silently invalidates the interval and drops the epoch to the constant-
    // state fallback. Serial intake costs nothing here (the callback only
    // timestamps and enqueues) and removes that failure mode entirely.
    imu_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions intake_options;
    intake_options.callback_group = intake_group_;
    rclcpp::SubscriptionOptions imu_options;
    imu_options.callback_group = imu_group_;
    imu_options.event_callbacks.message_lost_callback =
        [this](rclcpp::QOSMessageLostInfo&) { ++n_imu_message_lost_; };

    // The IMU subscription already reports lost messages; the observation ones
    // did not, so an epoch that never arrived was indistinguishable from one
    // with no usable satellites, and availability computed over "epochs
    // received" reports ~100 % while epochs are going missing.
    intake_options.event_callbacks.message_lost_callback =
        [this](rclcpp::QOSMessageLostInfo& info) {
          n_obs_message_lost_ += info.total_count_change;
          last_obs_loss_ctow_.store(
              last_epoch_ctow_.load(std::memory_order_relaxed),
              std::memory_order_relaxed);
        };

    // Deep, because a solve slower than the input period must not make the
    // middleware drop epochs the node never decided about. The matcher queue
    // uses the same depth (makePreprocessorConfig).
    rover_obs_sub_ = create_subscription<grs::GnssObservations>(
        get_parameter("topics.rover_observation").as_string(),
        rclcpp::QoS(kObsQueueDepth),
        std::bind(&GnssImuFgoNode::onRoverObs, this, std::placeholders::_1),
        intake_options);
    base_obs_sub_ = create_subscription<grs::GnssObservations>(
        get_parameter("topics.base_observation").as_string(),
        rclcpp::QoS(kObsQueueDepth),
        std::bind(&GnssImuFgoNode::onBaseObs, this, std::placeholders::_1),
        intake_options);
    nav_sub_ = create_subscription<grs::GnssEphemerides>(
        get_parameter("topics.ephemeris").as_string(),
        // depth 256 (not 1): ephemeris arrives as many per-satellite messages;
        // a depth-1 transient_local latch keeps only the last satellite, so a
        // subscriber matching after the initial burst would starve for DDs.
        rclcpp::QoS(256).transient_local(),
        std::bind(&GnssImuFgoNode::onNav, this, std::placeholders::_1),
        intake_options);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        get_parameter("topics.imu").as_string(),
        rclcpp::QoS(rclcpp::KeepLast(imu_queue_depth_)),
        std::bind(&GnssImuFgoNode::onImu, this, std::placeholders::_1),
        imu_options);
    sol_pub_ = create_publisher<grs::GnssSolution>(
        get_parameter("topics.solution").as_string(), 10);
    // Full navigation state (position + attitude + velocity) - the tightly-
    // coupled formulation estimates attitude that GnssSolution cannot carry.
    // Same convention as the GNSS/IMU EKF example's <solution>_odom topic.
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        get_parameter("topics.solution").as_string() + "_odom", 10);
    // The worker: every solve happens here. Also drains epochs waiting on IMU
    // coverage, so a dead IMU stream cannot stall the queue.
    pending_timer_ = create_wall_timer(
        std::chrono::milliseconds(10), [this] {
          const auto t_batch = std::chrono::steady_clock::now();
          std::lock_guard<std::mutex> lk(mtx_);
          const std::uint64_t before = n_epochs_in_;
          batch_processed_ = 0;
          processEpochs();
          // How long one tick owns mtx_, and how many epochs it swallowed.
          const double held = std::chrono::duration<double>(
              std::chrono::steady_clock::now() - t_batch).count();
          const std::uint64_t n = n_epochs_in_ - before;
          batch_epochs_max_ = std::max(batch_epochs_max_, n);
          if (held > batch_hold_max_s_) {
            batch_hold_max_s_ = held;
            batch_hold_max_epochs_ = n;
          }
        }, solver_group_);

    const std::string debug_dir =
        get_parameter("debug.ar_dump_dir").as_string();
    ar_dump_ = std::make_unique<gnss_fgo::ArDebugDumper>(
        debug_dir, "gnss_imu_fgo");
    if (!debug_dir.empty()) {
      const std::string trace = debug_dir + "/gnss_imu_fgo_base_arrival.csv";
      base_trace_ = std::fopen(trace.c_str(), "w");
      if (base_trace_) {
        std::fprintf(base_trace_,
                     "t_arrival_s,week,tow,pending_base_after_push,t_pushed_s\n");
      }
    }
    if (!debug_dir.empty()) {
      imu_diag_.open(debug_dir + "/gnss_imu_fgo_imu_diagnostics.csv");
      if (imu_diag_.is_open())
        // The bias and attitude columns are the ones that decide whether a
        // vertical/horizontal float error is the IMU model's fault: the
        // ADIS16505-2 in-run stability is 26-43 um/s^2 and 2.2-2.7 deg/h, so a
        // bias state that wanders far beyond that is acting as a dump for state
        // error rather than representing the sensor. roll/pitch/yaw go against
        // the reference trajectory's own Roll/Pitch/Heading.
        imu_diag_ << "week,tow,use_imu,fallback_total,max_imu_gap_s,"
                     "dds_lost_total,imu_received_total,solve_ms,"
                     "ms_imu_stage,ms_gate,ms_gnss_stage,ms_isam_update,ms_estimate,"
                     "ms_ar_posterior,ms_marginal,ms_ar,ms_hold,"
                     "ms_hold_update,ms_hold_refetch,"
                     "ba_x,ba_y,ba_z,bg_x,bg_y,bg_z,"
                     "roll_deg,pitch_deg,yaw_deg,speed_mps,dT_s\n";
    }
    if (ar_dump_->enabled()) {
      RCLCPP_INFO(get_logger(), "AR debug dump -> %s",
                  get_parameter("debug.ar_dump_dir").as_string().c_str());
    }

    RCLCPP_INFO(get_logger(),
                "Tightly-coupled GNSS/IMU FGO started (%s, lever "
                "arm %.3f %.3f %.3f, lag %.1fs).",
                ar_enabled_ ? "fix-and-hold"
                            : "FLOAT ONLY - ambiguity.resolution is false",
                lever_arm_.x(), lever_arm_.y(), lever_arm_.z(), smoother_lag_);
  }

  ~GnssImuFgoNode() override {
    if (base_trace_) { std::fclose(base_trace_); base_trace_ = nullptr; }
    // Availability report: an epoch reaching processEpoch should always yield a
    // solution (IMU-only FLOAT when GNSS is blocked), so a shortfall here is the
    // number of epochs lost to initialization / degenerate-solve paths.
    if (n_epochs_in_ > 0) {
      RCLCPP_INFO(get_logger(),
                  "Availability: %llu / %llu epochs published (%.2f%%), plus "
                  "%llu IMU-predicted epochs during GNSS gaps.",
                  static_cast<unsigned long long>(n_published_ - n_predicted_pub_),
                  static_cast<unsigned long long>(n_epochs_in_),
                  100.0 * static_cast<double>(n_published_ - n_predicted_pub_) /
                      static_cast<double>(n_epochs_in_),
                  static_cast<unsigned long long>(n_predicted_pub_));
    }
    // Why the inertial bridge did or did not run. Read this together with the
    // Availability line: a genuine GNSS outage that is NOT bridged shows up as
    // published < reference with predicted == 0.
    RCLCPP_INFO(get_logger(),
                "Gap publisher: %llu calls -> exits: disabled %llu, not-ready "
                "%llu, pending %llu, no-interval %llu, matcher still holds a "
                "rover %llu, stream alive %llu, backlog %llu (worst %.2f s); "
                "reached the slot loop %llu, ran out of IMU %llu; longest "
                "silence acted on %.2f s (threshold %.2f s); slots "
                "synthesized %llu.",
                static_cast<unsigned long long>(gap_calls_),
                static_cast<unsigned long long>(gap_exit_disabled_),
                static_cast<unsigned long long>(gap_exit_not_ready_),
                static_cast<unsigned long long>(gap_exit_pending_),
                static_cast<unsigned long long>(gap_exit_no_interval_),
                static_cast<unsigned long long>(gap_exit_matcher_holds_),
                static_cast<unsigned long long>(gap_exit_stream_alive_),
                static_cast<unsigned long long>(gap_exit_backlog_),
                gap_backlog_max_s_,
                static_cast<unsigned long long>(gap_reached_loop_),
                static_cast<unsigned long long>(gap_break_no_imu_),
                gap_silence_max_s_, kOutageEpochs * gnss_interval_s_,
                static_cast<unsigned long long>(n_predicted_pub_));
    RCLCPP_INFO(get_logger(),
                "Worker batching: longest rover silence OBSERVED %.2f s; "
                "biggest batch %llu epochs; longest tick held the solver lock "
                "%.2f s (%llu epochs). A tick longer than an outage hides that "
                "outage from the gap publisher.",
                gap_silence_seen_max_s_,
                static_cast<unsigned long long>(batch_epochs_max_),
                batch_hold_max_s_,
                static_cast<unsigned long long>(batch_hold_max_epochs_));
    if (slots_.suppressed() > 0) {
      RCLCPP_WARN(get_logger(),
                  "%llu epoch(s) arrived after their slot had been published "
                  "and were folded into the graph without re-publishing. Those "
                  "slots carry a dead-reckoned solution; the node is not "
                  "keeping up with the observation rate.",
                  static_cast<unsigned long long>(slots_.suppressed()));
    }
    if (n_coast_truncated_ > 0) {
      RCLCPP_WARN(get_logger(),
                  "Dead reckoning was truncated %llu time(s) at the "
                  "gap.max_coast_s = %.1f s limit: the node stopped publishing "
                  "rather than extend an inertial-only solution further.",
                  static_cast<unsigned long long>(n_coast_truncated_),
                  max_coast_s_);
    }
    if (n_reanchor_ > 0) {
      RCLCPP_INFO(get_logger(),
                  "Re-anchors: %llu, of which %llu had an estimator-independent "
                  "target (code-DD WLS / SPP).",
                  static_cast<unsigned long long>(n_reanchor_),
                  static_cast<unsigned long long>(n_reanchor_independent_));
    }
    RCLCPP_INFO(
        get_logger(),
        "IMU diagnostics: received=%llu, DDS-lost=%llu, fallback=%llu, "
        "max-gap=%.3fs.",
        static_cast<unsigned long long>(n_imu_received_.load()),
        static_cast<unsigned long long>(n_imu_message_lost_.load()),
        static_cast<unsigned long long>(n_imu_fallback_), max_imu_gap_s_);
    // The availability figure above is "of the epochs that ARRIVED". Anything
    // lost in transport never reached it, so report that separately rather than
    // letting a perfect-looking percentage stand for perfect coverage.
    const auto obs_lost = n_obs_message_lost_.load();
    if (obs_lost > 0) {
      RCLCPP_WARN(get_logger(),
                  "%llu observation message(s) were LOST in transport before "
                  "reaching this node, so the availability above is measured "
                  "against a short denominator. Epochs that never arrive cannot "
                  "be solved; raise the subscription depth or reduce the load.",
                  static_cast<unsigned long long>(obs_lost));
    } else {
      RCLCPP_INFO(get_logger(), "No observation messages lost in transport.");
    }
    const std::uint64_t n_rover_msgs =
        n_rover_msgs_received_.load(std::memory_order_relaxed);
    RCLCPP_INFO(get_logger(),
                "Epoch accounting: %llu rover message(s) received -> %llu "
                "reached the estimator (%llu produced no epoch in the "
                "preprocessor).",
                static_cast<unsigned long long>(n_rover_msgs),
                static_cast<unsigned long long>(n_epochs_in_),
                static_cast<unsigned long long>(
                    n_rover_msgs >= n_epochs_in_
                        ? n_rover_msgs - n_epochs_in_ : 0));
    {
      using M = gnss_utils::RoverBaseEpochMatcher;
      const auto& c = preprocessor_->matcherCounters();
      std::ostringstream os;
      for (int r = 0; r < M::kNumReasons; ++r) {
        const auto v = c.dropped(M::Stream::Rover, static_cast<M::DropReason>(r));
        if (v) os << " " << M::dropReasonName(static_cast<M::DropReason>(r))
                  << "=" << v;
      }
      RCLCPP_INFO(get_logger(),
                  "Matcher rover accounting: pushed=%llu emitted=%llu "
                  "queued=%zu drops:%s | emitted_unmatched=%llu | "
                  "events_overwritten=%llu",
                  static_cast<unsigned long long>(c.pushed_rover),
                  static_cast<unsigned long long>(c.emitted_rover),
                  preprocessor_->pendingRoverCount(),
                  os.str().empty() ? " none" : os.str().c_str(),
                  static_cast<unsigned long long>(c.emitted_unmatched),
                  static_cast<unsigned long long>(c.drop_events_overwritten));
    }
    RCLCPP_INFO(get_logger(),
                "Queue residue at shutdown: matcher holds %zu rover epoch(s), "
                "pending IMU-coverage queue holds %zu, epochs emitted by the "
                "preprocessor %llu.",
                preprocessor_->pendingRoverCount(), pending_epochs_.size(),
                static_cast<unsigned long long>(n_epochs_emitted_));
    {
      const auto& rej = preprocessor_->epochRejectCounters();
      if (rej.emitted_without_satellites > 0) {
        RCLCPP_INFO(get_logger(),
                    "%llu epoch(s) had no usable satellite after the masks and "
                    "were carried by the IMU alone (real sky outage).",
                    static_cast<unsigned long long>(
                        rej.emitted_without_satellites));
      }
      if (rej.total() > 0) {
        RCLCPP_WARN(get_logger(),
                    "Preprocessor produced NO epoch for %llu rover message(s): "
                    "no_observations=%llu, no_rover_entries=%llu, "
                    "no_apriori=%llu. These never reach the estimator.",
                    static_cast<unsigned long long>(rej.total()),
                    static_cast<unsigned long long>(rej.no_observations),
                    static_cast<unsigned long long>(rej.no_rover_entries),
                    static_cast<unsigned long long>(rej.no_apriori));
      }
    }
    if (n_epochs_dropped_ > 0) {
      // Do NOT tell the reader to raise the queue limit: a larger queue
      // silences the warning and recovers nothing, because a base epoch later
      // than max_tdiff_s is unusable however the queue is bounded. The overflow
      // is a symptom of a stalled subscription, so point at the stall.
      // Pinned by EnlargingTheQueueDoesNotRecoverStarvedEpochs.
      RCLCPP_WARN(get_logger(),
                  "%llu epoch(s) were DROPPED by the rover/base matcher and "
                  "never reached the estimator. See the per-reason warnings "
                  "above. queue_overflow does NOT mean the queue is too small: "
                  "it means one stream stopped being serviced long enough for "
                  "its backlog to age past max_age_s, so check per-epoch solve "
                  "time against the observation rate first.",
                  static_cast<unsigned long long>(n_epochs_dropped_));
    }
  }

  // ISAM2 with QR numerical factorization (see gnss_fgo.cpp): the graph with an
  // IMU chain plus tightly-held integers is ill-conditioned for CHOLESKY, which
  // factors J^T J and squares the condition number.
  //
  // relinearizeSkip stays at 1 here, unlike gnss_fgo. The DD model depends on
  // ATTITUDE through the lever arm and the preintegration carries its own
  // bias-correction nonlinearity, so a stale linearization point biases the
  // float exactly where integer resolution is most fragile.
  //
  // The threshold is per-symbol: a single scalar imposes 0.01 rad/s on the gyro
  // bias, ~100x its own magnitude, so the bias variables would never be
  // relinearized at all.
  static gtsam::ISAM2Params makeIsamParams() {
    gtsam::ISAM2Params p;
    gtsam::FastMap<char, gtsam::Vector> thresholds;
    // Pose3 tangent ordering: [rot(3), trans(3)].
    thresholds['x'] = (gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2)
                          .finished();
    thresholds['v'] = gtsam::Vector3::Constant(1e-2);
    // ConstantBias tangent ordering: [accel(3), gyro(3)].
    thresholds['b'] =
        (gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-4, 1e-4, 1e-4).finished();
    thresholds['n'] = (gtsam::Vector(1) << 1e-2).finished();  // ambiguity [cyc]
    p.relinearizeThreshold = thresholds;
    // skip=1, not gnss_fgo's 10: this graph is materially more nonlinear, and
    // relinearizing every update buys ~3 points of fix rate and ~2 of p05 for
    // +9 % solve time. Do not raise it without re-measuring both.
    p.relinearizeSkip = 1;
    // QR, and it is not negotiable on numerics. The NavState graph carries
    // held integers (sigma 0.03 cycle ~ 5.7 mm) alongside metre-scale pose
    // priors: an information-matrix dynamic range of ~3e8, which CHOLESKY
    // squares to ~1e17, at the edge of double precision. It then resets the
    // graph tens of times per run. QR eliminates in Jacobian space and does not
    // square it, at ~4.8x the solve time (25 -> 122 ms median) - which is where
    // this node's real-time budget goes. Any attempt to revisit CHOLESKY must
    // be tested on a WEAK geometry (a canyon course with ~9 DDs/epoch), not
    // only on open sky, because that is where the squared system goes singular.
    p.factorization = gtsam::ISAM2Params::QR;
    return p;
  }

  // <= 0 means full history (unbounded), realised as a lag longer than any
  // session; see gnss_fgo.cpp.
  static double smootherLag(double lag_s) {
    return lag_s > 0.0 ? lag_s : 1.0e9;
  }

 private:
  struct ImuSample {
    double t;  // ROS time [s]
    Eigen::Vector3d acc;
    Eigen::Vector3d gyr;
  };

  // A matched GNSS epoch waiting for its IMU coverage (see drainPendingEpochs).
  struct PendingEpoch {
    gnss_utils::PreprocessedEpoch ep;
    rclcpp::Time t_epoch;  // aligned epoch time (IMU stamp time base)
    double imu_watermark_at_enqueue{-std::numeric_limits<double>::infinity()};
  };

  void declareParameters() {
    declare_parameter("topics.rover_observation", std::string("/gnss/observation"));
    declare_parameter("topics.base_observation", std::string("/base/gnss/observation"));
    declare_parameter("topics.ephemeris", std::string("/gnss/ephemeris"));
    declare_parameter("topics.imu", std::string("/gnss/imu/data_raw"));
    declare_parameter("topics.solution", std::string("/gnss/imu_fgo/solution"));

    declare_parameter("base_position.postype", std::string("llh"));
    declare_parameter("base_position.pos", std::vector<double>{0.0, 0.0, 0.0});

    // Body-frame translation from the body/IMU origin to the GNSS antenna
    // phase center [m].
    declare_parameter("lever_arm", std::vector<double>{0.0, 0.0, 0.0});

    // ENU anchor: "gnss_fix" anchors at the first GNSS a-priori position,
    // "manual" uses local_origin.pos.
    declare_parameter("local_origin.mode", std::string("gnss_fix"));
    declare_parameter("local_origin.postype", std::string("llh"));
    declare_parameter("local_origin.pos", std::vector<double>{0.0, 0.0, 0.0});

    declare_parameter("masks.elevation_deg", 15.0);
    declare_parameter("masks.snr_dbhz", 0.0);

    declare_parameter("navsys.gps", true);
    declare_parameter("navsys.glo", false);
    // Use GLONASS for the undifferenced product (availability, SPP, Doppler
    // velocity) while keeping it OUT of double differencing and therefore out
    // of AR. Requires navsys.glo:=true. Default false, so the two existing
    // choices (glo off / glo fully on) behave exactly as before.
    //
    // The FDMA inter-frequency bias that motivates excluding GLONASS is a
    // carrier-DD problem; it is not a reason to drop GLONASS pseudoranges from
    // the position fix that keeps the node alive in a canyon.
    declare_parameter("navsys.glo_undifferenced_only", false);
    declare_parameter("navsys.gal", true);
    declare_parameter("navsys.bds", true);
    declare_parameter("navsys.qzs", true);

    declare_parameter("frequencies.enable_l1", true);
    declare_parameter("frequencies.enable_l2", true);
    declare_parameter("frequencies.enable_l5", false);
    declare_parameter("frequencies.cross_code_pairing", true);

    declare_parameter("max_age_s", 5.0);
    declare_parameter("measurement.iono_model", std::string("brdc"));
    declare_parameter("measurement.trop_model", std::string("saas"));
    declare_parameter("measurement.base_correlation_model",
                      std::string("variance_inflation"));
    declare_parameter("measurement.base_reuse_factor", 1.0);

    // Rover-base cycle-slip detection (see gnss_fgo / the README); only the two
    // data-dependent thresholds are exposed, the rest are fixed constants.
    declare_parameter("cycle_slip.gf_threshold_m", 0.05);
    declare_parameter("cycle_slip.max_gap_s", 2.0);

    // Undifferenced (zenith) measurement sigmas; DD noise is derived from these.
    declare_parameter("noise.pr_sigma_m", 0.5);
    declare_parameter("noise.cp_sigma_m", 0.005);
    declare_parameter("noise.pr_innov_gate_m", 0.0);

    // Integer ambiguity resolution on the graph posterior; see gnss_fgo.yaml.
    // false makes this a pure FLOAT estimator: LAMBDA is never run, no fix is
    // ever published, and no integer is ever held. See gnss_fgo.cpp.
    declare_parameter("ambiguity.resolution", true);
    // How many DDs the pre-fit innovation test may exclude before it judges the
    // whole epoch dirty (see ArOptions::fde_max_exclude).
    declare_parameter("ambiguity.fde_max_exclude", 2);
    declare_parameter("ambiguity.ratio_threshold", 3.0);
    declare_parameter("ambiguity.min_fix", 4);
    declare_parameter("ambiguity.min_lock", 5);
    declare_parameter("ambiguity.min_fix_to_hold", 10);
    declare_parameter("ambiguity.partial_max_drop", 5);
    // Absolute code-DD residual RMS at the FLOAT [m]: the float itself is in the
    // wrong place. Complementary to the growth test above, which is blind to it
    // (a confident IMU drags the float along with the fix, so the increment it
    // scores vanishes). Firing drops the carried ambiguities, since they are what
    // put the state there.
    declare_parameter("ambiguity.max_pos_var_m2", 0.25);
    declare_parameter("ambiguity.max_outage_s", 5.0);
    // Longest stretch the node will dead-reckon through a GNSS outage before
    // it stops publishing. Beyond this the solution is inertial drift wearing
    // a position's clothes, and emitting it is worse than emitting nothing.
    // 0 disables dead reckoning entirely (publish only measured epochs), which
    // is what an evaluation against methods that never coast wants.
    declare_parameter("gap.max_coast_s", 30.0);
    // Age-based re-acquisition [s] (0 disables); see gnss_fgo.cpp.
    declare_parameter("ambiguity.hold_refresh_s", 30.0);
    // Cap scope: "all" (default; doubles as fading memory on accumulated
    // model optimism - measured better on urban full courses) or "held"
    // (preserve float arcs until slip/outage; arcs then live minutes).

    // Directory for the ambiguity-resolution dump (empty = off); see gnss_fgo.
    declare_parameter("debug.ar_dump_dir", std::string(""));

    // Fixed-lag window [s]: bounds the per-epoch solve cost (real-time). <= 0
    // selects full-history ISAM2 (unbounded). Shorter than gnss_fgo's window:
    // the IMU factors constrain the poses strongly, so a pose marginalizes
    // consistently sooner, and the heavier Pose/Vel/Bias state needs the smaller
    // window to stay real-time.
    //
    // This is an accuracy-versus-LATENCY knob, and the latency half is steep:
    // cost is near-linear in the number of poses in the window, so a longer one
    // buys a little accuracy and quickly stops meeting the epoch period. See
    // the README before raising it.
    declare_parameter("graph.lag_s", 25.0);

    // ENU output origin: fixed_origin (configured) takes priority over the base
    // station; mirrors gnss_fgo's publishSolution.
    declare_parameter("fixed_origin.postype", std::string("llh"));
    declare_parameter("fixed_origin.pos", std::vector<double>{0.0, 0.0, 0.0});

    // IMU noise densities, deliberately INFLATED over the ADIS16505-2 datasheet
    // and not a mistake: the effective process noise must absorb engine/road
    // vibration, mount flexure and unmodeled dynamics, and a tightly-coupled
    // GNSS solve is destabilized by an over-confident IMU. The bias random
    // walks, by contrast, ARE datasheet-derived.
    declare_parameter("imu.sigma_acc", 0.3);        // [m/s^2/sqrt(Hz)]
    declare_parameter("imu.sigma_gyr", 0.01);       // [rad/s/sqrt(Hz)]
    declare_parameter("imu.sigma_acc_bias", 1.0e-5);  // [m/s^2/sqrt(s)]
    declare_parameter("imu.sigma_gyr_bias", 1.0e-6);  // [rad/s/sqrt(s)]
    // Position-integration uncertainty (continuous-time). GTSAM's own examples
    // and the inuex35 reference use ~1e-4..1e-3; the previous 1e-8 was
    // unjustifiably tight (it under-modeled the discretization error of the
    // zero-order-hold IMU integration between 100 Hz samples).
    declare_parameter("imu.sigma_integration", 0.01);  // sqrt of cov (-> 1e-4)
    declare_parameter("imu.max_wait_s", 0.2);
    declare_parameter("imu.queue_depth", 2000);
    // Model the Earth's rotation in the local ENU navigation frame (see
    // makePimParams). ON by default: at 15 deg/h the Earth rate is several
    // times the sensor's own bias stability, so ignoring it turns the gyro bias
    // state into a proxy for a known deterministic quantity.

    // Vehicle-motion pseudo-measurements (see imu_factors.hpp). They replace
    // what wheel odometry would give and are datasheet-independent kinematics.
    //
    // Velocity-direction attitude aiding: body +x points along the GNSS
    // velocity. ON by default because it supplies the ONLY constraint in this
    // graph that ties the heading to the direction of travel - without it the
    // yaw wanders over tens of degrees and the horizontal lever arm turns that
    // straight into a float-position error. It does assume a WHEELED VEHICLE
    // (small side-slip, no sustained reverse); set false for aircraft, boats or
    // handheld use, where the velocity direction says nothing about the body.
    declare_parameter("attitude.velocity_aiding", true);
    declare_parameter("attitude.min_speed_mps", 3.0);
    declare_parameter("attitude.sideslip_deg", 2.0);
    declare_parameter("attitude.max_misalign_deg", 90.0);
    // NHC: body-lateral / body-vertical velocity ~ 0 (car does not slip/hop).
    declare_parameter("nhc.enable", false);
    declare_parameter("nhc.sigma_lat_mps", 0.3);
    declare_parameter("nhc.sigma_vert_mps", 0.2);
    declare_parameter("nhc.min_speed_mps", 0.0);
    declare_parameter("nhc.lever_frd", std::vector<double>{0.0, 0.0, 0.0});
    // Yaw-rate gate [rad/s]: apply NHC only when driving roughly straight, so it
    // is kinematically exact at the IMU without a measured rear-axle lever.
    declare_parameter("nhc.max_yaw_rate_rps", 0.02);
    // ZUPT: detect a stationary window and pin velocity to 0.
    declare_parameter("zupt.enable", true);
    declare_parameter("zupt.max_acc_std", 0.55);
    declare_parameter("zupt.max_gyr_std", 0.030);
    declare_parameter("zupt.max_gyr_median", 0.020);
    declare_parameter("zupt.min_samples", 5);
    declare_parameter("zupt.max_speed_mps", 1.0);
    declare_parameter("zupt.sigma_mps", 0.5);
    // Vertical ZUPT sigma; see ZuptConfig. Loose by default so a stop cannot
    // freeze an existing vertical position error in place.
    declare_parameter("zupt.sigma_vert_mps", 0.5);
    // Reject the Doppler velocity aiding when its post-fit LS residual RMS
    // exceeds this (m/s): a mismodeled / NLOS Doppler set would otherwise pull
    // the velocity. 0 disables the gate.
    declare_parameter("noise.doppler_max_res_m", 2.0);
    // Correlation time [s] of the GNSS Doppler velocity error (0 = treat as
    // white). The velocity prior is applied once per epoch as an independent
    // absolute observation; with 5 Hz GNSS and a 2 s correlation time that
    // over-counts the information by sqrt(2*5) ~ 3.2x unless the sigma is
    // inflated by the same factor. See the use site in processEpoch.
    declare_parameter("noise.doppler_corr_time_s", 2.0);
    // Per-satellite fault detection inside the Doppler solve (studentized
    // residual threshold; 0 disables). The residual-RMS gate above can only
    // reject the whole epoch, so one NLOS range-rate either survives and biases
    // the velocity, or costs every satellite in the set.
    declare_parameter("noise.doppler_max_nsigma", 4.0);
    // A-priori zenith range-rate sigma [m/s]; the per-satellite variance is
    // doppler_sigma_mps^2 / sin^2(el). Sets the absolute scale of the reported
    // velocity covariance AND the reference the w-test judges against.
    declare_parameter("noise.doppler_sigma_mps", 0.1);

    declare_parameter("init.imu_duration", 1.0);    // [s] static init window

    // GNSS epoch time source for IMU pairing: "header" (trust header.stamp,
    // legacy) or "tow_auto_offset" (utc(week,tow) + estimated clock offset;
    // removes the receiver output latency even on an unsynced PC clock). See
    // gnss_utils::GnssEpochAligner.
    declare_parameter("time.gnss_epoch_source", std::string("header"));

    gnss_utils::GnssEpochAligner::Config tcfg;
    const std::string src = get_parameter("time.gnss_epoch_source").as_string();
    if (!gnss_utils::GnssEpochAligner::parseSource(src, tcfg.source)) {
      RCLCPP_WARN(get_logger(),
                  "Unknown time.gnss_epoch_source '%s' - using 'header'.",
                  src.c_str());
    }
    tcfg.offset_window_s = kOffsetWindowS;
    epoch_aligner_ = gnss_utils::GnssEpochAligner(tcfg);

    ar_enabled_ = get_parameter("ambiguity.resolution").as_bool();
    ar_opt_.fde_max_exclude =
        static_cast<int>(get_parameter("ambiguity.fde_max_exclude").as_int());
    ar_opt_.ratio_threshold = get_parameter("ambiguity.ratio_threshold").as_double();
    ar_opt_.el_mask_rad = get_parameter("masks.elevation_deg").as_double() * D2R;
    ar_opt_.min_fix = static_cast<int>(get_parameter("ambiguity.min_fix").as_int());
    ar_opt_.min_lock =
        static_cast<int>(get_parameter("ambiguity.min_lock").as_int());
    ar_opt_.partial_max_drop =
        static_cast<int>(get_parameter("ambiguity.partial_max_drop").as_int());
    ar_opt_.max_pos_var_m2 =
        get_parameter("ambiguity.max_pos_var_m2").as_double();
    init_imu_duration_ = get_parameter("init.imu_duration").as_double();
    min_fix_to_hold_ =
        static_cast<int>(get_parameter("ambiguity.min_fix_to_hold").as_int());
    vatt_cfg_.enable = get_parameter("attitude.velocity_aiding").as_bool();
    vatt_cfg_.min_speed_mps = get_parameter("attitude.min_speed_mps").as_double();
    vatt_cfg_.sideslip_deg = get_parameter("attitude.sideslip_deg").as_double();
    vatt_cfg_.max_misalign_deg =
        get_parameter("attitude.max_misalign_deg").as_double();
    // Say it once at startup: this one silently assumes a platform.
    if (vatt_cfg_.enable) {
      RCLCPP_INFO(get_logger(),
          "Velocity-direction attitude aiding ON above %.1f m/s (side-slip "
          "sigma %.1f deg). Assumes a WHEELED vehicle - set "
          "attitude.velocity_aiding:=false for aircraft/boat/handheld. First "
          "heading taken from the course once %d epochs agree to %.0f deg with "
          "a course sigma under %.0f deg.",
          vatt_cfg_.min_speed_mps, vatt_cfg_.sideslip_deg,
          yaw_align_cfg_.agree_epochs, yaw_align_cfg_.agree_deg,
          yaw_align_cfg_.max_sigma_deg);
    }
    nhc_cfg_.enable = get_parameter("nhc.enable").as_bool();
    nhc_cfg_.sigma_lat_mps = get_parameter("nhc.sigma_lat_mps").as_double();
    nhc_cfg_.sigma_vert_mps = get_parameter("nhc.sigma_vert_mps").as_double();
    nhc_cfg_.min_speed_mps = get_parameter("nhc.min_speed_mps").as_double();
    {
      const auto l = get_parameter("nhc.lever_frd").as_double_array();
      if (l.size() == 3) nhc_cfg_.lever_frd = Eigen::Vector3d(l[0], l[1], l[2]);
    }
    nhc_cfg_.max_yaw_rate_rps = get_parameter("nhc.max_yaw_rate_rps").as_double();
    zupt_cfg_.enable = get_parameter("zupt.enable").as_bool();
    zupt_cfg_.max_acc_std = get_parameter("zupt.max_acc_std").as_double();
    zupt_cfg_.max_gyr_std = get_parameter("zupt.max_gyr_std").as_double();
    zupt_cfg_.max_gyr_median = get_parameter("zupt.max_gyr_median").as_double();
    zupt_cfg_.min_samples =
        static_cast<int>(get_parameter("zupt.min_samples").as_int());
    zupt_cfg_.max_speed_mps = get_parameter("zupt.max_speed_mps").as_double();
    zupt_cfg_.sigma_mps = get_parameter("zupt.sigma_mps").as_double();
    zupt_cfg_.sigma_vert_mps =
        get_parameter("zupt.sigma_vert_mps").as_double();
    doppler_max_res_m_ = get_parameter("noise.doppler_max_res_m").as_double();
    doppler_corr_time_s_ =
        get_parameter("noise.doppler_corr_time_s").as_double();
    imu_sigma_gyr_ = get_parameter("imu.sigma_gyr").as_double();
    cacheFixedOriginParam();
  }

  void cacheFixedOriginParam() {
    const std::string type = get_parameter("fixed_origin.postype").as_string();
    const auto pos = get_parameter("fixed_origin.pos").as_double_array();
    if (pos.size() == 3 && norm(pos.data(), 3) > 0.0) {
      if (type == "llh") {
        const double llh[3] = {pos[0] * D2R, pos[1] * D2R, pos[2]};
        pos2ecef(llh, fixed_origin_ecef_);
      } else {
        std::copy_n(pos.data(), 3, fixed_origin_ecef_);
      }
      fixed_origin_valid_ = true;
    }
  }

  gnss_utils::GnssPreprocessor::Config makePreprocessorConfig() {
    gnss_utils::GnssPreprocessor::Config cfg;
    cfg.el_mask_rad = get_parameter("masks.elevation_deg").as_double() * D2R;
    cfg.snr_mask_dbhz = get_parameter("masks.snr_dbhz").as_double();
    cfg.navsys = 0;
    if (get_parameter("navsys.gps").as_bool()) cfg.navsys |= SYS_GPS;
    if (get_parameter("navsys.glo").as_bool()) {
      if (get_parameter("navsys.glo_undifferenced_only").as_bool()) {
        cfg.navsys_undifferenced_only |= SYS_GLO;
      } else {
        cfg.navsys |= SYS_GLO;
      }
    }
    if (get_parameter("navsys.gal").as_bool()) cfg.navsys |= SYS_GAL;
    if (get_parameter("navsys.bds").as_bool()) cfg.navsys |= SYS_CMP;
    if (get_parameter("navsys.qzs").as_bool()) cfg.navsys |= SYS_QZS;
    cfg.glonass_carrier_dd = false;  // FDMA biases don't cancel in DD (fixed).
    cfg.cross_code_carrier =
        get_parameter("frequencies.cross_code_pairing").as_bool();
    cfg.bands.clear();
    if (get_parameter("frequencies.enable_l1").as_bool()) cfg.bands.push_back(0);
    if (get_parameter("frequencies.enable_l2").as_bool()) cfg.bands.push_back(1);
    if (get_parameter("frequencies.enable_l5").as_bool()) cfg.bands.push_back(2);
    if (cfg.bands.empty()) cfg.bands.push_back(0);
    cfg.max_age_s = get_parameter("max_age_s").as_double();
    // Match the subscription depth: intake runs off the solver thread, so a
    // backlog accumulates here rather than in the middleware queue.
    cfg.matcher_queue_limit = kObsQueueDepth;
    const std::string iono =
        get_parameter("measurement.iono_model").as_string();
    const std::string trop =
        get_parameter("measurement.trop_model").as_string();
    cfg.ionoopt = (iono == "off") ? IONOOPT_OFF : IONOOPT_BRDC;
    cfg.tropopt = (trop == "off") ? TROPOPT_OFF : TROPOPT_SAAS;
    cfg.detect_slip_gf = true;
    cfg.slip_gf_threshold_m = get_parameter("cycle_slip.gf_threshold_m").as_double();
    cfg.detect_slip_cmc = true;
    cfg.slip_cmc_threshold_m = 3.0;
    cfg.detect_slip_dop = true;
    cfg.slip_dop_threshold_cyc = 1.0;
    cfg.slip_max_gap_s = get_parameter("cycle_slip.max_gap_s").as_double();
    cfg.doppler_max_nsigma =
        get_parameter("noise.doppler_max_nsigma").as_double();
    cfg.doppler_sigma_mps =
        get_parameter("noise.doppler_sigma_mps").as_double();

    const std::string postype = get_parameter("base_position.postype").as_string();
    const auto pos = get_parameter("base_position.pos").as_double_array();
    if (pos.size() == 3 && norm(pos.data(), 3) > 0.0) {
      double ecef[3];
      if (postype == "llh") {
        const double llh[3] = {pos[0] * D2R, pos[1] * D2R, pos[2]};
        pos2ecef(llh, ecef);
      } else {
        std::copy_n(pos.data(), 3, ecef);
      }
      cfg.base_ecef = Eigen::Vector3d(ecef[0], ecef[1], ecef[2]);
    } else {
      RCLCPP_ERROR(get_logger(),
                   "base_position is not set - no DD pairs will be formed.");
    }
    return cfg;
  }

  // Only the two undifferenced sigmas are exposed; the rest are fixed constants
  // (see factor_adapters.hpp / README).
  gnss_fgo::AdapterConfig makeAdapterConfig() {
    gnss_fgo::AdapterConfig cfg;
    cfg.pr_sigma_m = get_parameter("noise.pr_sigma_m").as_double();
    cfg.cp_sigma_m = get_parameter("noise.cp_sigma_m").as_double();
    cfg.pr_innov_gate_m = get_parameter("noise.pr_innov_gate_m").as_double();
    const std::string base_model =
        get_parameter("measurement.base_correlation_model").as_string();
    cfg.base_reuse_factor = (base_model == "independent")
                                ? 1.0
                                : std::max(1.0, get_parameter(
                                      "measurement.base_reuse_factor").as_double());
    return cfg;
  }

  // Earth rotation rate expressed in the local ENU navigation frame at the
  // anchor: omega_ie^n = omega_e * (0, cos phi, sin phi).
  //
  // ONE definition, used by BOTH places that need it - the preintegration
  // params (Coriolis) and the static initialisation of the gyro bias. They must
  // agree: a static gyro reads omega_ib^b = b_g + R_bn * omega_ie^n, so if the
  // raw average is assigned to the bias while NavState::coriolis ALSO removes
  // the Earth rate, the term is removed twice. Keeping the two derivations in
  // separate places is exactly how that inconsistency was introduced.
  // Returns false when the nav frame is not anchored yet.
  bool earthRateNav(gtsam::Vector3& omega_ie_nav) const {
    if (!nav_frame_valid_) return false;
    double llh[3];
    const double e[3] = {ecef_T_nav_.translation().x(),
                         ecef_T_nav_.translation().y(),
                         ecef_T_nav_.translation().z()};
    ecef2pos(e, llh);
    omega_ie_nav = gnss_fgo::earthRateEnu(llh[0]);
    return true;
  }

  // WGS84 normal gravity at geodetic latitude phi [rad] and ellipsoidal height
  // h [m]: Somigliana on the ellipsoid plus the free-air term.
  //
  // The node used the STANDARD value 9.80665 m/s^2 everywhere. That is not the
  // gravity anywhere in particular - at Nagoya (phi 35.17 deg, h ~45 m) normal
  // gravity is 9.7973, a systematic 0.0093 m/s^2 too large. The accelerometer
  // bias is the only state that can absorb it, and its random walk is
  // 1e-5 m/s^2/sqrt(s): over a 25 s window the bias can move 5e-5, so it CANNOT
  // track the error - it just sits displaced, and the residual shows up as a
  // metre-scale float vertical error at the epochs where AR fails.
  static double normalGravity(double phi, double h) {
    constexpr double kGammaE = 9.7803253359;      // equatorial normal gravity
    constexpr double kSomigliana = 0.00193185265241;
    constexpr double kE2 = 0.00669437999013;      // WGS84 first eccentricity^2
    const double s2 = std::sin(phi) * std::sin(phi);
    const double g0 = kGammaE * (1.0 + kSomigliana * s2) / std::sqrt(1.0 - kE2 * s2);
    // Free-air correction (height above the ellipsoid).
    return g0 - (3.0877e-6 - 4.3e-9 * s2) * h + 0.72e-12 * h * h;
  }

  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> makePimParams() {
    // ENU nav frame: gravity along -Z handled by MakeSharedU (z-up). Use the
    // LOCAL normal gravity at the anchor, not the standard constant.
    double g_local = kGravity;
    if (nav_frame_valid_) {
      double llh[3];
      const double e[3] = {ecef_T_nav_.translation().x(),
                           ecef_T_nav_.translation().y(),
                           ecef_T_nav_.translation().z()};
      ecef2pos(e, llh);
      g_local = normalGravity(llh[0], llh[2]);
      RCLCPP_INFO(get_logger(),
                  "Local normal gravity %.5f m/s^2 at lat %.4f deg, h %.1f m "
                  "(standard 9.80665, difference %+.5f).",
                  g_local, llh[0] * R2D, llh[2], g_local - kGravity);
    }
    auto p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(g_local);
    const double sa = get_parameter("imu.sigma_acc").as_double();
    const double sg = get_parameter("imu.sigma_gyr").as_double();
    const double sab = get_parameter("imu.sigma_acc_bias").as_double();
    const double sgb = get_parameter("imu.sigma_gyr_bias").as_double();
    const double si = get_parameter("imu.sigma_integration").as_double();
    p->accelerometerCovariance = gtsam::I_3x3 * sa * sa;
    p->gyroscopeCovariance = gtsam::I_3x3 * sg * sg;
    p->biasAccCovariance = gtsam::I_3x3 * sab * sab;
    p->biasOmegaCovariance = gtsam::I_3x3 * sgb * sgb;
    p->integrationCovariance = gtsam::I_3x3 * si * si;
    // NOTE: biasAccOmegaInt is @deprecated in this GTSAM (no longer used by the
    // preintegration; the initial-bias uncertainty is carried by the prior on
    // B(0) instead), so it is intentionally left at its default and not set.

    // Earth rotation. The local ENU frame is NOT inertial: it rotates at
    // omega_e = 7.292115e-5 rad/s (15.04 deg/h), which is several times the
    // sensor's in-run bias stability, so leaving it unmodelled would let a
    // deterministic effect dominate the gyro bias state. In ENU at the anchor
    // latitude phi the Earth rate is omega_e * (0, cos phi, sin phi).
    //
    // The transport rate (~3 % of the Earth rate) is NOT included: it depends
    // on the current velocity, while these params are fixed at initialization.
    gtsam::Vector3 omega_ie_nav;
    if (earthRateNav(omega_ie_nav)) {
      p->setOmegaCoriolis(omega_ie_nav);
      RCLCPP_INFO(get_logger(),
                  "Earth rotation modelled: omega_ie^ENU = (0, %.3e, %.3e) "
                  "rad/s (the static gyro average is corrected by R_bn*omega_ie "
                  "so the term is not removed twice).",
                  omega_ie_nav.y(), omega_ie_nav.z());
    }
    return p;
  }

  void onNav(const grs::GnssEphemerides::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(intake_mtx_);
    preprocessor_->pushEphemerides(*msg);
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    ImuSample s;
    s.t = rclcpp::Time(msg->header.stamp).seconds();
    s.acc = Eigen::Vector3d(msg->linear_acceleration.x,
                            msg->linear_acceleration.y,
                            msg->linear_acceleration.z);
    s.gyr = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y,
                            msg->angular_velocity.z);
    std::lock_guard<std::mutex> lk(imu_mtx_);
    imu_buffer_.push_back(s);
    last_imu_arrival_wall_ = std::chrono::steady_clock::now();
    ++n_imu_received_;
    // Bounded integration history: roughly ten minutes at 200 Hz.
    if (imu_buffer_.size() > 120000) imu_buffer_.pop_front();
  }

  double imuWatermark() {
    std::lock_guard<std::mutex> lk(imu_mtx_);
    return imu_buffer_.empty()
               ? -std::numeric_limits<double>::infinity()
               : imu_buffer_.back().t;
  }

  // Observation intake. Enqueue and return; the solve runs on the worker timer.
  // Staying cheap is what lets latest_rover_ctow_ track ARRIVAL rather than
  // progress, so the gap publisher can tell "no epoch exists" from "an epoch is
  // queued behind a slow solve".
  void onRoverObs(const grs::GnssObservations::SharedPtr msg) {
    // Arrival on the IMU time base, for the outage verdict. Sampled BEFORE the
    // lock, so imu_mtx_ is never held together with intake_mtx_.
    const double arrival_imu_t = imuWatermark();
    std::lock_guard<std::mutex> lk(intake_mtx_);
    ++n_rover_msgs_received_;
    // Continuous GPST of the newest rover observation RECEIVED (not yet
    // necessarily processed): the decisive gate for the gap publisher - a grid
    // slot at or before this time has a real observation somewhere in the
    // pipeline, so synthesizing it would race (and duplicate) the real epoch.
    latest_rover_ctow_.store(
        std::max(latest_rover_ctow_.load(std::memory_order_relaxed),
                 msg->week * 604800.0 + msg->tow),
        std::memory_order_relaxed);
    last_rover_arrival_imu_t_ = arrival_imu_t;
    preprocessor_->pushRoverObs(msg);
  }

  void onBaseObs(const grs::GnssObservations::SharedPtr msg) {
    // Arrival instrumentation: written only when debug.ar_dump_dir is set, and
    // taken BEFORE the lock so it records when the message reached the node,
    // not when the node got round to it. This is what distinguishes "the base
    // stream arrives late" from "it arrives on time and then sits in a queue",
    // which the drop counters alone cannot separate.
    const double t_wall =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    std::lock_guard<std::mutex> lk(intake_mtx_);
    preprocessor_->pushBaseObs(msg);
    if (base_trace_) {
      std::fprintf(base_trace_, "%.6f,%u,%.3f,%zu,%.6f\n", t_wall, msg->week,
                   msg->tow, preprocessor_->pendingBaseCount(),
                   std::chrono::duration<double>(
                       std::chrono::steady_clock::now().time_since_epoch())
                       .count());
    }
  }

  // Drain what intake has queued and advance the graph. Caller holds mtx_;
  // intake_mtx_ is taken only for the queue handoffs, never across the solve.
  void processEpochs() {
    std::vector<gnss_utils::PreprocessedEpoch> epochs;
    {
      std::lock_guard<std::mutex> intake_lk(intake_mtx_);
      epochs = preprocessor_->drainEpochs(
          last_antenna_ecef_valid_ ? &last_antenna_ecef_ : nullptr);
    }
    for (auto& ep : epochs) {
      // A DD-less epoch is NOT skipped: this is exactly the GNSS outage the
      // inertial chain exists to bridge. Once initialized, processEpoch still
      // runs the IMU stage and publishes the propagated FLOAT (no AR, since
      // there are no ambiguities to fix). Only a cold start genuinely needs
      // DDs, and tryInitialize gates on that.
      if (ep.dd.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Epoch tow %.3f has no DD pairs (check base stream / base_position)"
            "%s.", ep.tow,
            initialized_ ? " - coasting on IMU" : " - cannot initialize yet");
      }
      // GNSS epoch time on the IMU stamp time base (see
      // time.gnss_epoch_source). Everything downstream - IMU integration
      // boundaries, initialization window, published stamps - uses this
      // aligned time.
      std::string align_warn;
      const rclcpp::Time t_epoch = epoch_aligner_.align(
          ep.week, ep.tow, rclcpp::Time(ep.stamp), &align_warn);
      if (!align_warn.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s",
                             align_warn.c_str());
      }
      ++n_epochs_emitted_;
      pending_epochs_.push_back(
          PendingEpoch{std::move(ep), t_epoch, imuWatermark()});
    }
    drainPendingEpochs();
    // takeDropped() is the LEGACY view and only describes one reason
    // (no base within the window). The matcher reason-tags every drop, and the
    // reasons it does not cover are counted and then thrown away, so epochs
    // vanish here with nothing logged and read downstream as a GNSS outage,
    // re-keying every carried arc. Consume the full event list instead.
    std::vector<gnss_utils::RoverBaseEpochMatcher::DropEvent> drop_events;
    {
      std::lock_guard<std::mutex> intake_lk(intake_mtx_);
      drop_events = preprocessor_->takeDropEvents();
      preprocessor_->takeDropped();  // keep the legacy queue from growing
    }
    for (const auto& e : drop_events) {
      ++n_epochs_dropped_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Matcher dropped a %s epoch (tow %.3f): %s. This epoch can never "
          "produce a solution.",
          gnss_utils::RoverBaseEpochMatcher::streamName(e.stream),
          e.tow,
          gnss_utils::RoverBaseEpochMatcher::dropReasonName(e.reason));
      if (e.stream == gnss_utils::RoverBaseEpochMatcher::Stream::Rover)
        last_epoch_drop_ctow_ = e.week * 604800.0 + e.tow;
    }
  }

  // Process pending GNSS epochs whose IMU coverage has arrived. GNSS and IMU
  // messages are delivered independently (with jitter, e.g. under bag replay),
  // so an epoch is held back until the IMU stream reaches its time - otherwise
  // the preintegration interval is empty and the chain degrades to the
  // constant-state fallback for no physical reason. A true IMU outage still
  // advances after imu.max_wait_s through that fallback. Caller holds mtx_.
  void drainPendingEpochs() {
    while (!pending_epochs_.empty()) {
      PendingEpoch& pe = pending_epochs_.front();
      const double t_ros = pe.t_epoch.seconds();
      double watermark;
      double live_silence;
      {
        std::lock_guard<std::mutex> imu_lk(imu_mtx_);
        watermark = imu_buffer_.empty()
                        ? -std::numeric_limits<double>::infinity()
                        : imu_buffer_.back().t;
        live_silence = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_imu_arrival_wall_).count();
      }
      const bool covered = watermark >= t_ros;
      if (!covered) {
        const bool sensor_wait_expired =
            std::isfinite(pe.imu_watermark_at_enqueue) &&
            watermark - pe.imu_watermark_at_enqueue >= imu_max_wait_s_;
        const bool live_outage = live_silence >= imu_max_wait_s_;
        if (!sensor_wait_expired && !live_outage) return;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "IMU watermark %.3f did not cover epoch %.3f (max wait %.2fs, "
            "live silence %.2fs) - processing with fallback.",
            watermark, t_ros, imu_max_wait_s_, live_silence);
      }
      const PendingEpoch cur = std::move(pe);
      pending_epochs_.pop_front();
      // Bridge whatever the epoch stream skipped BEFORE this epoch enters the
      // graph, so synthesized slots leave in time order ahead of it and still
      // see the previous posterior.
      if (initialized_ && last_imu_valid_ && gnss_interval_pinned_)
        publishPredictedGaps(cur.t_epoch.seconds() - 0.5 * gnss_interval_s_,
                             /*data_domain=*/true);
      processEpoch(cur.ep, cur.t_epoch);
      // Give the live publisher a chance between epochs: called only at the end
      // of a batch, it cannot see an outage shorter than the batch.
      publishPredictedGaps();
      if (++batch_processed_ >= kMaxEpochsPerBatch) return;
    }
    publishPredictedGaps();
  }

  // IMU-only dead reckoning across GNSS gaps: synthesize FLOAT solutions on the
  // rover epoch grid by preintegrating from the last posterior. Prediction only
  // - the graph is untouched and the IMU buffer is iterated, not consumed - so
  // a returning GNSS epoch continues from the same state, and the published
  // covariance grows with the preintegration. Caller holds mtx_.
  //
  // `horizon_t_ros` bounds the march; `data_domain` selects the trigger:
  //   - live (default): the rover has gone silent in ARRIVAL. Needs the silence
  //     verdict and the backlog guard, and stays clear of the inertial front.
  //   - data domain: a hole in the EMITTED epoch stream. The matcher emits in
  //     time order and drops anything below its floor, so nothing can still
  //     arrive for those slots; no verdict or guard is needed and it works
  //     while the node is behind.
  void publishPredictedGaps(
      double horizon_t_ros = std::numeric_limits<double>::infinity(),
      bool data_domain = false) {
    ++gap_calls_;
    if (max_coast_s_ <= 0.0) { ++gap_exit_disabled_; return; }
    if (!initialized_ || !last_imu_valid_) { ++gap_exit_not_ready_; return; }
    if (!data_domain && !pending_epochs_.empty()) {
      ++gap_exit_pending_;
      return;
    }

    // Decide WHETHER to synthesize before copying the IMU snapshot below (up to
    // 120 000 samples), since the common case is "no gap, nothing to do".
    //
    // THE CONTRACT, exclusive with processEpoch: synthesize ONLY grid slots for
    // which no rover epoch arrived at all. Every rover epoch that reaches the
    // node publishes exactly one solution, including a DD-less one. A base
    // outage is therefore NOT this publisher's business.
    if (!gnss_interval_pinned_) { ++gap_exit_no_interval_; return; }
    // ONE snapshot of the arrival front: read per use, the backlog guard and
    // the slot loop below could arbitrate against different fronts.
    const double latest_rover =
        latest_rover_ctow_.load(std::memory_order_relaxed);
    if (!data_domain) {
      // Never coast past an epoch the pipeline is still holding: a non-empty
      // matcher queue means a real solution for an unpublished slot is still
      // coming, and publishing ahead of it would push it behind the output
      // high-water mark.
      std::size_t held_by_matcher;
      {
        std::lock_guard<std::mutex> intake_lk(intake_mtx_);
        held_by_matcher = preprocessor_->pendingRoverCount();
      }
      if (held_by_matcher > 0) { ++gap_exit_matcher_holds_; return; }
      const double imu_now = imuWatermark();
      double last_arrival;
      {
        std::lock_guard<std::mutex> intake_lk(intake_mtx_);
        last_arrival = last_rover_arrival_imu_t_;
      }
      // Record what the publisher SAW, not only what it acted on, so "never
      // fired" is still diagnosable.
      const double silence = imu_now - last_arrival;
      if (std::isfinite(silence))
        gap_silence_seen_max_s_ = std::max(gap_silence_seen_max_s_, silence);
      if (!gnss_fgo::gapSlotIsUnobserved(imu_now, last_arrival,
                                         gnss_interval_s_, kOutageEpochs)) {
        ++gap_exit_stream_alive_;
        return;
      }
      gap_silence_max_s_ = std::max(gap_silence_max_s_, silence);
    }
    // Backlog guard: received rover past the processed front means the node is
    // behind on a stream that IS arriving, so the graph will publish these
    // slots itself. The data-domain trigger skips it - there the slots are
    // known to have no observation, backlog or not.
    if (!data_domain && latest_rover >
        last_week_ * 604800.0 + last_tow_ + 1.5 * gnss_interval_s_) {
      ++gap_exit_backlog_;
      gap_backlog_max_s_ = std::max(
          gap_backlog_max_s_,
          latest_rover - (last_week_ * 604800.0 + last_tow_));
      return;
    }
    ++gap_reached_loop_;

    // Only now copy, and only the part that can be integrated: the loop below
    // starts at prev_t_ros_ and never looks further back, so everything older
    // is dead weight. The buffer holds ~20 minutes at 100 Hz while a coast is
    // seconds long.
    std::deque<ImuSample> imu_snapshot;
    {
      std::lock_guard<std::mutex> imu_lk(imu_mtx_);
      if (imu_buffer_.empty()) return;
      auto first = std::lower_bound(
          imu_buffer_.begin(), imu_buffer_.end(), prev_t_ros_,
          [](const ImuSample& s, double t) { return s.t < t; });
      // Keep one sample before the start so the zero-order hold has its
      // left-hand value, exactly as the full copy did.
      if (first != imu_buffer_.begin()) --first;
      imu_snapshot.assign(first, imu_buffer_.end());
    }
    if (imu_snapshot.empty()) return;

    for (;;) {
      const double t_pred =
          prev_t_ros_ + (n_pred_since_epoch_ + 1) * gnss_interval_s_;
      // Stay the silence threshold behind the inertial front. A slot newer than
      // that could still have an observation in flight - the arrival anchor
      // cannot yet have shown it - and claiming it would publish a coasted
      // solution for an epoch the graph is about to solve properly. Costs the
      // last three slots of every gap and is what keeps the re-publish count
      // at zero. Same constant as the verdict: one quantity, expressed once.
      if (t_pred > horizon_t_ros) break;
      // The live trigger stays clear of the inertial front so a slot whose
      // epoch is still in flight is never taken; the data-domain trigger has
      // its horizon from the epoch that revealed the hole.
      if (!data_domain &&
          imu_snapshot.back().t - t_pred <
              kLiveStandoffEpochs * gnss_interval_s_) {
        ++gap_break_no_imu_;
        break;
      }
      if (imu_snapshot.back().t < t_pred) { ++gap_break_no_imu_; break; }

      // Preintegrate (prev_t_ros_, t_pred] with the same causal zero-order
      // hold as processEpoch. Rebuilt from the posterior each time so repeated
      // predictions do not compound integration state.
      gtsam::PreintegratedCombinedMeasurements pim(pim_params_, prev_bias_);
      double t_cursor = prev_t_ros_;
      ImuSample hold = last_imu_;
      bool gap_ok = true;  // no IMU sample gap exceeded the ZOH validity limit
      for (const auto& s : imu_snapshot) {
        if (s.t > t_pred) break;
        if (s.t <= t_cursor) { hold = s; continue; }
        const double dt = s.t - t_cursor;
        if (dt > kImuMaxSampleGapS) gap_ok = false;
        if (dt > 1e-6) pim.integrateMeasurement(hold.acc, hold.gyr, dt);
        t_cursor = s.t;
        hold = s;
      }
      if (t_pred > t_cursor + 1e-6) {
        const double dt = t_pred - t_cursor;
        if (dt > kImuMaxSampleGapS) gap_ok = false;
        pim.integrateMeasurement(hold.acc, hold.gyr, dt);
      }
      // Integration must exactly cover (prev_t_ros_, t_pred]; a sample gap beyond
      // the ZOH validity limit makes the dead-reckoned state and its output stamp
      // inconsistent, so skip this slot rather than publish a state whose
      // integration time is shorter than the horizon it claims (same discipline
      // as processEpoch's use_imu gate). Advance the grid so we do not busy-loop.
      if (!gap_ok ||
          std::abs(pim.deltaTij() - (t_pred - prev_t_ros_)) > 1e-4) {
        ++n_pred_since_epoch_;
        continue;
      }
      const gtsam::NavState pred = pim.predict(prev_state_, prev_bias_);

      // Antenna position/covariance: last posterior pose covariance through
      // the lever-arm Jacobian, plus the preintegrated position uncertainty
      // (delta-position covariance lives in the departure body frame;
      // preintMeasCov ordering is [theta(0:2), pos(3:5), vel(6:8), ba, bg]).
      gtsam::Matrix36 H_pose;
      const gtsam::Point3 ant_nav = pred.pose().transformFrom(lever_arm_, H_pose);
      const gtsam::Point3 ant_ecef = ecef_T_nav_.transformFrom(ant_nav);
      const Eigen::Matrix3d R_e_n = ecef_T_nav_.rotation().matrix();
      const Eigen::Matrix3d R_att = prev_state_.attitude().matrix();

      // Full 15-state propagation, the same one the pre-fit gate uses: X/V/B
      // cross-covariances and bias uncertainty included. The predecessor here
      // summed a few blocks and conceded in its own comment that the result was
      // a lower bound - during an outage, which is exactly when a consumer is
      // relying on this number to decide whether to trust the position.
      Eigen::Matrix<double, 9, 9> pred9;
      const bool have_pred9 = predictedNavCov9(pim, pred9);

      // Antenna position and body velocity out of the NavState chart.
      // J9: the pose block is transformFrom's own Jacobian (identical chart);
      // velocity does not move the antenna. Both are then rotated to ECEF.
      Eigen::Matrix<double, 3, 9> J9 = Eigen::Matrix<double, 3, 9>::Zero();
      J9.leftCols<6>() = H_pose;
      const Eigen::Matrix<double, 3, 9> J = R_e_n * J9;
      const Eigen::Matrix3d R_e_b = R_e_n * R_att;

      Eigen::Matrix3d cov;
      Eigen::Matrix3d vel_cov;  // ENU - see publishSolution / publishOdometry
      if (have_pred9) {
        cov = J * pred9 * J.transpose();
        // ENU, NOT ECEF. publishSolution takes an ENU velocity covariance and
        // applies R_e_n itself; handing it an ECEF one rotates twice. The
        // chart's velocity block is BODY-frame, so exactly one rotation is
        // needed here - by the PREDICTED attitude, since pred9 is expressed
        // about the predicted state.
        vel_cov = gnss_fgo::gapVelocityCovarianceEnu(
            pred9.bottomRightCorner<3, 3>(), pred.attitude());
      } else {
        // No joint posterior to propagate. Publish a deliberately CONSERVATIVE
        // bound rather than an optimistic sum: an outage is the wrong place
        // to understate uncertainty, and a consumer can always ignore a large
        // covariance while it cannot recover from a small wrong one.
        const double h = pim.deltaTij();
        const Eigen::Matrix3d pos_floor =
            Eigen::Matrix3d::Identity() *
            (kPredictFallbackPosStdM * kPredictFallbackPosStdM * (1.0 + h * h));
        cov = J.leftCols<6>() * last_pose_cov_ * J.leftCols<6>().transpose() +
              R_e_b * pim.preintMeasCov().block<3, 3>(3, 3) * R_e_b.transpose() +
              (h * h) * R_e_n * last_vel_cov_ * R_e_n.transpose() + pos_floor;
        // Same frame as the full path: ENU. last_vel_cov_ is already the graph's
        // ENU velocity covariance; the preintegrated velocity block is BODY, so
        // it takes the departure attitude and nothing else.
        vel_cov = gnss_fgo::gapVelocityCovarianceEnuFallback(
            last_vel_cov_, pim.preintMeasCov().block<3, 3>(6, 6),
            prev_state_.attitude(), kPredictFallbackVelStdMps);
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Dead reckoning without a joint posterior covariance: publishing a "
            "conservative bound, not a propagated one.");
      }
      cov = 0.5 * (cov + cov.transpose());
      vel_cov = 0.5 * (vel_cov + vel_cov.transpose());

      ++n_pred_since_epoch_;
      // Synthesized epoch on the rover grid (evaluation tools match on tow).
      double tow = last_tow_ + n_pred_since_epoch_ * gnss_interval_s_;
      std::uint32_t week = last_week_;
      if (tow >= 604800.0) {
        tow -= 604800.0;
        ++week;
      }
      const double slot_ctow = week * 604800.0 + tow;
      // Stop coasting once the dead-reckoned solution has outlived its value.
      // An unbounded coast is not a service: it emits a confident-looking
      // position that is only inertial drift, and in live operation (no bag to
      // end the run) it would do so for as long as the observations stay away.
      //
      // BEFORE the claim: claiming commits the high-water mark, so breaking out
      // afterwards would consume a slot without publishing it and silence the
      // real epoch that later arrives for it.
      if (last_dd_ctow_ > 0.0 && slot_ctow - last_dd_ctow_ > max_coast_s_) {
        ++n_coast_truncated_;
        break;
      }
      // The slot must have no owner: never re-emit an epoch already published,
      // and never take a slot a received rover epoch will publish for. Without
      // the first test the loop re-marches the whole span from the last real
      // epoch on every call (measured once: 261 231 solutions over 17 263
      // distinct epochs, one slot emitted 108 times, a 331 MB output bag).
      // `latest_rover` (the snapshot taken once at the top of this call) is
      // the live path's "an epoch may still be in the pipeline" test; -1.0
      // disables just that clause for the data-domain path, which already has
      // direct evidence.
      if (!slots_.claim(slot_ctow, data_domain ? -1.0 : latest_rover))
        continue;
      gnss_utils::PreprocessedEpoch fake;
      fake.week = week;
      fake.tow = tow;
      fake.age_s = 0.0;
      fake.base_ecef = last_base_ecef_;

      const rclcpp::Time stamp(static_cast<int64_t>(t_pred * 1e9),
                               epoch_clock_type_);
      const Eigen::Vector3d pos(ant_ecef.x(), ant_ecef.y(), ant_ecef.z());
      // Odometry with the coasted pose; tangent covariance approximated by the
      // last posterior plus the preintegrated rotation/position blocks.
      Eigen::MatrixXd pose_cov(6, 6);
      if (have_pred9) {
        pose_cov = pred9.topLeftCorner<6, 6>();
      } else {
        pose_cov = last_pose_cov_;
        pose_cov.block<3, 3>(0, 0) += pim.preintMeasCov().block<3, 3>(0, 0);
        pose_cov.block<3, 3>(3, 3) +=
            pim.preintMeasCov().block<3, 3>(3, 3) +
            Eigen::Matrix3d::Identity() *
                (kPredictFallbackPosStdM * kPredictFallbackPosStdM);
      }
      // Dead reckoning has no joint posterior over the PREDICTED state (pred9
      // is 9x9 and carries no bias block), so the antenna velocity covariance
      // takes the block-diagonal fallback here.
      gnss_fgo::AntennaVelocityCov avc;
      avc.vel_nav = vel_cov;
      avc.rot = pose_cov.topLeftCorner<3, 3>();
      if (last_state_cov15_ok_)
        avc.bias_gyro = last_state_cov15_.block<3, 3>(12, 12);
      avc.sigma_gyr = last_imu_valid_ ? imu_sigma_gyr_ : 0.0;
      const gnss_fgo::AntennaVelocity ant_vel = gnss_fgo::antennaVelocityNav(
          pred.pose().rotation(), pred.velocity(), omegaBody(), lever_arm_, avc);
      publishSolution(fake, stamp, pos, cov, ant_vel.v_nav, ant_vel.cov_nav,
                      grs::GnssSolution::STATUS_FLOAT, 0.0);
      publishOdometry(stamp, pred.pose(), pose_cov, pred.velocity(), vel_cov);
      ++n_predicted_pub_;
      ++n_published_;
    }
  }

  // 9x9 NavState-chart covariance of the state `pim` predicts from the last
  // posterior, ordering [rot(0:2), pos(3:5), vel(6:8)] with the velocity in the
  // BODY frame (the NavState chart's own convention). ONE implementation, used
  // by both the pre-fit gate and the dead-reckoning publisher, so neither can
  // drift into publishing an optimistic lower bound.
  //
  // Returns false when the joint 15x15 posterior was never captured, so the
  // caller can choose a conservative fallback rather than inherit an optimistic
  // one silently.
  bool predictedNavCov9(const gtsam::PreintegratedCombinedMeasurements& pim,
                        Eigen::Matrix<double, 9, 9>& pred9) const {
    if (!last_state_cov15_ok_) return false;
    // Graph charts -> NavState chart, about the PREVIOUS state.
    const Eigen::Matrix3d R_prev = prev_state_.pose().rotation().matrix();
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(15, 15);
    T.block<3, 3>(6, 6) = R_prev.transpose();
    const Eigen::MatrixXd P = T * last_state_cov15_ * T.transpose();

    gtsam::Matrix99 F;
    gtsam::Matrix96 G;
    pim.predict(prev_state_, prev_bias_, F, G);
    const Eigen::Matrix<double, 9, 9> Pxx = P.topLeftCorner<9, 9>();
    const Eigen::Matrix<double, 9, 6> Pxb = P.topRightCorner<9, 6>();
    const Eigen::Matrix<double, 6, 6> Pbb = P.bottomRightCorner<6, 6>();
    const Eigen::Matrix<double, 9, 9> cross = F * Pxb * G.transpose();
    pred9 = F * Pxx * F.transpose() + cross + cross.transpose() +
            G * Pbb * G.transpose() + pim.preintMeasCov().topLeftCorner<9, 9>();
    pred9 = 0.5 * (pred9 + pred9.transpose());
    if (!pred9.allFinite()) return false;
    // A covariance that is not PSD is not a covariance. Small negative
    // eigenvalues are numerical; anything larger means the propagation and the
    // stored posterior disagree, and the caller must not publish it.
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9>> es(pred9);
    if (es.info() != Eigen::Success) return false;
    const double lmax = es.eigenvalues().maxCoeff();
    if (!(lmax > 0.0)) return false;
    return es.eigenvalues().minCoeff() >= -1e-9 * lmax;
  }

  void processEpoch(const gnss_utils::PreprocessedEpoch& ep,
                    const rclcpp::Time& t_epoch) {
    double t_ros = t_epoch.seconds();
    epoch_clock_type_ = t_epoch.get_clock_type();
    ++n_epochs_in_;

    if (!initialized_) {
      tryInitialize(ep, t_ros);
      return;
    }
    const auto solve_t0 = std::chrono::steady_clock::now();
    // Per-stage timing (W4). Optimising the solve without knowing which stage
    // dominates is guesswork; these feed the debug CSV.
    auto tick = [] { return std::chrono::steady_clock::now(); };
    auto ms_since = [](std::chrono::steady_clock::time_point t0) {
      return std::chrono::duration<double, std::milli>(
                 std::chrono::steady_clock::now() - t0).count();
    };
    double ms_imu_stage = 0.0, ms_gate = 0.0, ms_gnss_stage = 0.0;
    double ms_isam_update = 0.0, ms_estimate = 0.0, ms_ar_posterior = 0.0,
           ms_marginal = 0.0;
    double ms_ar = 0.0, ms_hold = 0.0;
    // ms_hold covers BOTH the applyHolds update and the state re-fetch that
    // follows it (3 calculateEstimate + 1 jointMarginalCovariance). Split them,
    // because which one dominates decides whether merging the update into
    // the next epoch is worth anything at all - and because the re-fetch now
    // runs on FIX epochs too, which is where holds actually change.
    double ms_hold_update = 0.0, ms_hold_refetch = 0.0;
    // The auto-offset estimate can step down between epochs; keep the epoch
    // chain strictly monotonic so the integration interval stays valid.
    t_ros = std::max(t_ros, prev_t_ros_ + 1e-4);

    // Integrate the IMU samples in (t_{k-1}, t_k] with causal zero-order hold.
    // A sample stamped t_i describes the interval beginning at t_i; the previous
    // code integrated sample i over (t_{i-1},t_i], advancing the signal by one
    // sample. It also clamped long gaps but still treated the partial interval as
    // a valid CombinedImuFactor.
    gtsam::PreintegratedCombinedMeasurements pim(pim_params_, prev_bias_);
    double t_cursor = prev_t_ros_;
    // Raw samples integrated over this epoch, for the ZUPT stationarity test.
    std::vector<gnss_fgo::ImuStat> zupt_win;
    bool imu_interval_valid =
        last_imu_valid_ && last_imu_.t <= prev_t_ros_ + 1e-6 &&
        prev_t_ros_ - last_imu_.t <= kImuMaxSampleGapS;
    std::deque<ImuSample> epoch_imu;
    {
      std::lock_guard<std::mutex> imu_lk(imu_mtx_);
      while (!imu_buffer_.empty() && imu_buffer_.front().t <= t_ros) {
        epoch_imu.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
      }
    }
    while (!epoch_imu.empty()) {
      const ImuSample s = epoch_imu.front();
      epoch_imu.pop_front();
      if (s.t <= prev_t_ros_) {
        if (!last_imu_valid_ || s.t >= last_imu_.t) {
          last_imu_ = s;
          last_imu_valid_ = true;
          // AND, never assign: a gap already flagged earlier in this interval
          // must not be cleared by a later (still pre-interval) sample.
          imu_interval_valid =
              imu_interval_valid && prev_t_ros_ - s.t <= kImuMaxSampleGapS;
        }
        continue;
      }
      const double dt = s.t - t_cursor;
      max_imu_gap_s_ = std::max(max_imu_gap_s_, std::max(dt, 0.0));
      if (!last_imu_valid_ || dt < 0.0 || dt > kImuMaxSampleGapS) {
        imu_interval_valid = false;
      } else if (imu_interval_valid && dt > 1e-6) {
        pim.integrateMeasurement(last_imu_.acc, last_imu_.gyr, dt);
        zupt_win.push_back({last_imu_.acc, last_imu_.gyr});
      }
      t_cursor = s.t;
      last_imu_ = s;
      last_imu_valid_ = true;
    }
    // Integrate the residual (t_last, t_k] with a zero-order hold of the last
    // IMU sample so the preintegration interval exactly covers (t_{k-1}, t_k].
    // Otherwise the sub-sample motion between the last IMU stamp and the GNSS
    // epoch is dropped and the chain silently loses time every epoch.
    if (last_imu_valid_ && t_cursor < t_ros) {
      const double dt = t_ros - t_cursor;
      max_imu_gap_s_ = std::max(max_imu_gap_s_, std::max(dt, 0.0));
      if (dt > kImuMaxSampleGapS) {
        imu_interval_valid = false;
      } else if (imu_interval_valid && dt > 1e-6) {
        pim.integrateMeasurement(last_imu_.acc, last_imu_.gyr, dt);
      }
    }
    const double expected_imu_dt = t_ros - prev_t_ros_;
    const bool use_imu =
        imu_interval_valid && expected_imu_dt > 0.0 &&
        std::abs(pim.deltaTij() - expected_imu_dt) < 1e-4;

    const gtsam::Key xk = X(epoch_index_);
    const gtsam::Key vk = V(epoch_index_);
    const gtsam::Key bk = B(epoch_index_);

    const Eigen::Matrix3d R_e_n = ecef_T_nav_.rotation().matrix();
    // Antenna ECEF position + 3x3 covariance from a body pose and its 6x6 tangent
    // covariance, using the FULL antenna-vs-pose Jacobian (exact for any lever
    // arm; the translation block alone is exact only at zero lever arm).
    auto antennaFromPose = [&](const gtsam::Pose3& p, const Eigen::MatrixXd& p_cov,
                               Eigen::Vector3d& pos, Eigen::Matrix3d& cov) {
      gtsam::Matrix36 H_pose;
      const gtsam::Point3 ant_nav = p.transformFrom(lever_arm_, H_pose);
      const gtsam::Point3 ant_ecef = ecef_T_nav_.transformFrom(ant_nav);
      pos = Eigen::Vector3d(ant_ecef.x(), ant_ecef.y(), ant_ecef.z());
      const Eigen::Matrix<double, 3, 6> J = R_e_n * H_pose;
      cov = J * p_cov * J.transpose();
    };

    // Two-stage ISAM2 update. Stage 1 adds ONLY the IMU factor (or gap fallback),
    // so the X(k) marginal is the IMU PREDICTION (pre-GNSS) - the correct AR prior,
    // free of this epoch's DD (no double-count). GTSAM propagates the covariance,
    // pose/velocity/bias correlations and tangent-space transforms internally
    // (which a manual predicted covariance would get wrong).
    gtsam::NonlinearFactorGraph imu_graph;
    gtsam::Values imu_values;
    if (use_imu) {
      // Robust (Huber) inter-epoch link, matching how gnss_fgo treats ITS
      // motion factor. The inertial chain is several times stiffer per epoch, so
      // without this a disturbed state cannot be pulled back by the DDs and the
      // float stays wrong. Loosening the IMU globally would do it too, but at
      // the cost of false fixes; the kernel yields only where the residual is
      // genuinely large. CombinedImuFactor builds its own Gaussian model from
      // preintMeasCov(), so re-wrap that model rather than rebuilding the
      // factor.
      const gtsam::CombinedImuFactor imu_factor(
          X(epoch_index_ - 1), V(epoch_index_ - 1), xk, vk,
          B(epoch_index_ - 1), bk, pim);
      imu_graph.add(imu_factor);
    } else {
      ++n_imu_fallback_;
      // IMU gap: keep the chain connected with a loose kinematic link. The
      // measurement must be the DOPPLER-derived displacement, not identity: a
      // zero-displacement mean asserts the vehicle did not move, which at
      // 15 m/s over 0.2 s is a systematic 3 m pull-back applied precisely on
      // the epochs where the state is least constrained. gnss_fgo's motion
      // factor uses the same 0.5*(v_{k-1}+v_k)*dt displacement.
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Incomplete IMU coverage in (%.3f, %.3f] - falling back to a "
          "Doppler/kinematic link.",
          prev_t_ros_, t_ros);
      const double dt = std::max(t_ros - prev_t_ros_, 0.1);
      // Displacement in the PREVIOUS body frame (BetweenFactor<Pose3>'s
      // translation is expressed there), from the nav-frame mean velocity.
      const gtsam::Vector3 v_prev = prev_state_.velocity();
      const gtsam::Vector3 v_now =
          (ep.rover_vel_valid &&
           (doppler_max_res_m_ <= 0.0 || ep.rover_vel_res <= doppler_max_res_m_))
              ? gtsam::Vector3(R_e_n.transpose() * ep.rover_vel_ecef)
              : v_prev;
      const gtsam::Vector3 d_nav = 0.5 * (v_prev + v_now) * dt;
      const gtsam::Vector3 d_body =
          prev_state_.attitude().matrix().transpose() * d_nav;
      // Translation sigma covers the unmodelled acceleration over the gap plus
      // the Doppler velocity error; rotation is left effectively free.
      const double trans_sigma = std::max(1.0 * dt * dt + 0.3 * dt, 0.05);
      imu_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
          X(epoch_index_ - 1), xk, gtsam::Pose3(gtsam::Rot3(), d_body),
          gtsam::noiseModel::Diagonal::Sigmas(
              (gtsam::Vector(6) << 10.0 * dt, 10.0 * dt, 10.0 * dt,
               trans_sigma, trans_sigma, trans_sigma).finished())));
      imu_graph.add(gtsam::BetweenFactor<gtsam::Vector3>(
          V(epoch_index_ - 1), vk, gtsam::Vector3(v_now - v_prev),
          gtsam::noiseModel::Isotropic::Sigma(3, 1.0 * dt)));
      imu_graph.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
          B(epoch_index_ - 1), bk, gtsam::imuBias::ConstantBias(),
          gtsam::noiseModel::Isotropic::Sigma(6, 1e-3)));
    }
    const gtsam::NavState predicted =
        use_imu ? pim.predict(prev_state_, prev_bias_) : prev_state_;
    imu_values.insert(xk, predicted.pose());
    imu_values.insert(vk, predicted.velocity());
    imu_values.insert(bk, prev_bias_);
    // Fixed-lag timestamps: the new nav state at this epoch's time, and a
    // refresh of every live ambiguity key so a carried arc / hold is never
    // marginalized while still tracked.
    gtsam::FixedLagSmoother::KeyTimestampMap ts_imu;
    ts_imu[xk] = t_ros;
    ts_imu[vk] = t_ros;
    ts_imu[bk] = t_ros;
    for (const gtsam::Key k : amb_mgr_->liveKeys()) ts_imu[k] = t_ros;
    const auto t_imu_stage = tick();
    // ONE ISAM2 update per epoch, not two. Splitting it into an inertial-only
    // stage and a GNSS stage would give a DD-free X(k) marginal, but nothing
    // needs one: AR reads the stage-2 posterior (buildGraphPosteriorNav), and
    // the pre-fit gate's DD-free reference is pim.predict (see gate_ant below),
    // which costs no solver work at all. The second update would cost ~40 % of
    // the epoch budget, and with QR this node has none to spare.
    amb_mgr_->retireStale(ep.week * 604800.0 + ep.tow, amb_max_outage_);

    gtsam::NonlinearFactorGraph gnss_graph;
    gtsam::Values gnss_values;
    // Gross-error innovation gate against the IMU-predicted antenna position
    // (from the post-stage-1 pose); the preintegrated prediction is reliable, so
    // a DD whose predicted pseudorange residual is grossly off is NLOS.
    Eigen::Vector3d gate_ant;
    Eigen::Matrix3d gate_cov = Eigen::Matrix3d::Identity() * 100.0;
    bool gate_ok = false;
    int n_gated = 0;
    bool prediction_fault = false;
    gnss_utils::PreprocessedEpoch gnss_ep = ep;
    ms_imu_stage = ms_since(t_imu_stage);
    const auto t_gate = tick();
    // Gate reference: the ANALYTIC inertial prediction, not a solver marginal.
    // pim.predict(prev_state_, prev_bias_) propagates the previous POSTERIOR
    // through this epoch's preintegration, so it is DD-free by construction -
    // free of this epoch's DDs without needing a solve.
    //
    // Its covariance is PROPAGATED, not assembled from the marginals:
    //
    //   P_pred = F * P_prev * F' + G * P_bias * G' + (cross) + Q_preint
    //
    // with F = d(predict)/d(state) and G = d(predict)/d(bias), both supplied by
    // pim.predict(), and P_prev the JOINT posterior. Dropping the cross terms is
    // NOT conservative - the pose-velocity correlation is strongly positive, so
    // omitting it makes the gate over-confident while the state is drifting.
    //
    // Chart bookkeeping, which is where this is easy to get quietly wrong:
    //   - preintMeasCov is [theta, pos, vel, b_acc, b_gyr] and pim.predict's
    //     Jacobians are in the NavState tangent [dR, dP, dV].
    //   - Pose3::retract and NavState::retract BOTH rotate the translation
    //     increment by R, so the graph's 6x6 pose block maps into [dR, dP]
    //     unchanged.
    //   - the graph's V(k) is a plain Vector3: its increment is in the nav
    //     frame, while NavState's dV is rotated by R. That block needs
    //     R' * P_vv * R, and the corresponding cross blocks a one-sided R.
    // Verified by Monte-Carlo in test_gnss_fgo (GateCovarianceMatchesSampling).
    {
      gtsam::Matrix36 H_pose;
      const gtsam::Point3 ant_nav =
          predicted.pose().transformFrom(lever_arm_, H_pose);
      gate_ant = ecef_T_nav_.transformFrom(ant_nav);
      // 3x9 antenna Jacobian in the NavState tangent: the pose part is exactly
      // transformFrom's (identical chart), velocity does not move the antenna.
      Eigen::Matrix<double, 3, 9> J9 = Eigen::Matrix<double, 3, 9>::Zero();
      J9.leftCols<6>() = H_pose;
      const Eigen::Matrix<double, 3, 9> J = R_e_n * J9;

      Eigen::Matrix<double, 9, 9> pred9;
      bool propagated = false;
      if (use_imu) propagated = predictedNavCov9(pim, pred9);
      if (!propagated) {
        // Either the joint marginal was unavailable or there is no inertial
        // link this epoch. Fall back to the additive assembly - and with no IMU
        // at all the prediction is only a constant-state guess, so do not let
        // the gate act on it confidently.
        pred9.setZero();
        pred9.topLeftCorner<6, 6>() = last_pose_cov_;
        pred9.bottomRightCorner<3, 3>() = last_vel_cov_;
        if (use_imu) {
          pred9.topLeftCorner<6, 6>() +=
              pim.preintMeasCov().topLeftCorner<6, 6>();
          const double dt_h = pim.deltaTij();
          pred9.block<3, 3>(3, 3) += (dt_h * dt_h) * last_vel_cov_;
        } else {
          pred9.topLeftCorner<6, 6>() +=
              Eigen::MatrixXd::Identity(6, 6) * (kReanchorPosStdM * 0.1);
        }
      }
      gate_cov = J * pred9 * J.transpose();
      gate_cov = 0.5 * (gate_cov + gate_cov.transpose());
      gnss_ep = gnss_fgo::filterDdPreFitInnovation(
          ep, gate_ant, gate_cov, adapter_cfg_, ar_opt_, &n_gated,
          &prediction_fault);
      gate_ok = true;
    }

    ms_gate = ms_since(t_gate);

    // Re-anchor decision (see kReanchorGapS). Two triggers, both meaning "the
    // inertial chain has drifted somewhere the GNSS observations cannot reach":
    //   - the pre-fit test found the observations self-consistent at the code
    //     a-priori but not at the predicted state (prediction_fault), or
    //   - GNSS has been unusable long enough that the coast error is unbounded.
    const double ctow = ep.week * 604800.0 + ep.tow;
    last_epoch_ctow_.store(ctow, std::memory_order_relaxed);
    // ...and a third state that is NEITHER: the observations were fine but the
    // middleware dropped them. The coast error is then bounded by the real
    // elapsed time, the sky never went away, and the carried arcs are still
    // valid - so re-anchoring would destroy the carrier history for nothing.
    // Without this guard a handful of lost messages can account for the large
    // majority of a run's re-anchors, each one wiping every arc (`Arcs: 0`).
    // "Delivery gap" covers BOTH ways an epoch can fail to arrive - the
    // middleware losing it and the matcher discarding it - because either one
    // means the sky was fine.
    const double last_delivery_gap_ctow =
        std::max(last_obs_loss_ctow_.load(std::memory_order_relaxed),
                 last_epoch_drop_ctow_);
    const bool recent_obs_loss =
        last_delivery_gap_ctow > 0.0 &&
        (ctow - last_delivery_gap_ctow) <= kReanchorGapS;
    // LATCHED: one outage = one re-anchor. A re-anchor does not advance
    // last_dd_ctow_ - only a DD-bearing epoch does - so without the latch this
    // fires on every DD-less epoch of the outage, and each one wipes the carried
    // arcs and hold candidates. A LATER outage re-arms it, because the DDs that
    // ended the first one advance last_dd_ctow_ again.
    const bool gnss_returned = last_dd_ctow_ > 0.0 &&
                               (ctow - last_dd_ctow_) > kReanchorGapS &&
                               !recent_obs_loss && !gap_latch_.latched();
    if (last_dd_ctow_ > 0.0 && (ctow - last_dd_ctow_) > kReanchorGapS &&
        recent_obs_loss) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "%.1f s without a usable DD, but %llu observation message(s) were "
          "LOST in transport - treating this as a delivery gap, not a GNSS "
          "outage, and keeping the carried arcs.",
          ctow - last_dd_ctow_,
          static_cast<unsigned long long>(n_obs_message_lost_.load()));
    }
    // Deliberately NOT conditioned on surviving DDs. When the pre-fit filter
    // judges the whole epoch dirty it empties the DD set and leaves
    // prediction_fault false, which is exactly the state this recovery exists
    // for; its target comes from raw observations upstream of any gating.
    //
    // That target is independentCodePosition (code-DD WLS, then SPP), NOT
    // ep.rover_ecef_apriori - a copy of this node's own last estimate, i.e. the
    // drifted state the re-anchor exists to leave. With no satellites at all
    // independentCodePosition falls back to exactly that a-priori, so the GAP
    // trigger demands an independent target: anchoring the state to itself
    // moves nothing while still re-keying every arc and dropping the holds.
    // prediction_fault keeps the weaker test - there the DDs have already
    // rejected the state, and breaking that lock is worth a stale target.
    const gnss_fgo::IndependentCodePosition anchor =
        gnss_fgo::independentCodePosition(ep, adapter_cfg_);
    const bool reanchor =
        ecef_T_nav_.translation().norm() > 0.0 && anchor.ok &&
        (prediction_fault || (gnss_returned && anchor.independent()));
    if (reanchor) {
      ++n_reanchor_;
      if (anchor.independent()) ++n_reanchor_independent_;
      // The carried ambiguities absorbed the drifted state, so their floats are
      // stale by tens of cycles; re-key them (before the DD factors are built)
      // and drop the fix-and-hold progress that was certified against it.
      amb_mgr_->invalidateAll();
      hold_candidates_.clear();
      code_growth_.reset();
      code_rms_.reset();
      nfix_ = 0;
      // Arm the latch only for the GAP trigger. prediction_fault is a
      // per-epoch measurement verdict and must stay free to fire again.
      if (gnss_returned) gap_latch_.arm();
      RCLCPP_WARN(get_logger(),
          "Re-anchoring at %s (%s; %.1f s since the last usable GNSS epoch) - "
          "carried arcs re-keyed.",
          anchor.sourceName(),
          prediction_fault ? "predicted state rejected by its own DDs"
                           : "GNSS gap",
          last_dd_ctow_ > 0.0 ? ctow - last_dd_ctow_ : 0.0);
    }

    // See DdFactorLayout: the post-fit FIX validation reproduces the graph's
    // robust weights from what the factors actually contain, not from ep.dd.
    gnss_fgo::DdFactorLayout dd_layout;
    int n_bad_cov = 0;
    const auto pairs = gnss_fgo::addGroupedDdFactorsArm(
        gnss_ep, xk, lever_arm_, ecef_T_nav_, adapter_cfg_, gnss_graph,
        gnss_values, *amb_mgr_, gate_ok ? &gate_ant : nullptr, &n_gated,
        &dd_layout, &n_bad_cov);
    if (n_bad_cov > 0) {
      // groupedDdCovariance sums positive variances, so an unusable R means a
      // defect upstream, not unusual data. The group was dropped and the epoch's
      // post-fit FIX validation will refuse a verdict; say so rather than let it
      // look like a quiet epoch.
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
          "%d DD group(s) had an unusable covariance and were dropped; the FIX "
          "validation for this epoch cannot reach a verdict.", n_bad_cov);
    }

    // A loose ABSOLUTE anchor whenever this epoch contributes none of its own.
    // The IMU factor is a relative constraint, so a run of fully-gated epochs
    // chains poses by relative terms only, and the whole chain floats once the
    // last DD-anchored pose leaves the fixed-lag window ("singular near x<k>").
    //
    // The prior SIGMA is kReanchorPosStdM, not the anchor's own covariance: the
    // covariance says how well the code position is known, this sigma says how
    // hard to pull the graph towards it.
    if (reanchor || gnss_ep.dd.empty()) {
      // Loose absolute prior on the body pose: translation from this epoch's
      // independent code antenna position through the current attitude,
      // rotation left free (attitude and biases are worth keeping).
      const Eigen::Vector3d anchor_pos =
          anchor.ok ? anchor.pos : ep.rover_ecef_apriori;
      const gtsam::Point3 ant_nav =
          ecef_T_nav_.transformTo(gtsam::Point3(anchor_pos));
      const gtsam::Rot3 R0 = predicted.pose().rotation();
      gnss_graph.add(gtsam::PriorFactor<gtsam::Pose3>(
          xk, gtsam::Pose3(R0, ant_nav - R0.rotate(lever_arm_)),
          gtsam::noiseModel::Diagonal::Sigmas(
              (gtsam::Vector(6) << kReanchorRotStdRad, kReanchorRotStdRad,
               kReanchorRotStdRad, kReanchorPosStdM, kReanchorPosStdM,
               kReanchorPosStdM).finished())));
    }
    const bool doppler_ok =
        ep.rover_vel_valid &&
        (doppler_max_res_m_ <= 0.0 || ep.rover_vel_res <= doppler_max_res_m_);
    // This epoch's lever-corrected Doppler velocity in the nav frame, kept for
    // the attitude aiding below (which must not re-derive it or diverge from the
    // validity gating applied here).
    Eigen::Vector3d doppler_vel_nav = Eigen::Vector3d::Zero();
    double doppler_sigma_vel = 0.0;
    if (doppler_ok) {
      // GNSS Doppler velocity factor on V(k): an absolute-velocity observation.
      // Doppler measures the ANTENNA velocity; remove the lever-arm rotation
      // term R*(w x l) to observe the body-origin velocity V(k).
      gtsam::Vector3 vel_enu = R_e_n.transpose() * ep.rover_vel_ecef;
      if (lever_arm_.norm() > 0.0 && last_imu_valid_) {
        const Eigen::Vector3d w = omegaBody();
        vel_enu -= predicted.pose().rotation().matrix() *
                   w.cross(Eigen::Vector3d(lever_arm_));
      }
      // Doppler velocity errors are NOT white: satellite geometry, the receiver
      // clock-drift model and multipath persist for seconds, so an independent
      // ABSOLUTE prior once per epoch over-counts the information by ~sqrt(tau*f)
      // and inflates the ambiguity covariance Qa.
      //
      // Used instead in the form where the correlated part CANCELS - a
      // BetweenFactor on consecutive velocities, as gnss_fgo does with the same
      // Doppler on position increments. Its sigma needs only the white part,
      // scaled by sqrt(2) for the difference of two epochs. A loose absolute
      // prior is retained alongside to anchor the velocity LEVEL, weak enough
      // not to re-introduce the over-counting.
      const double sigma_white =
          std::max(std::sqrt(std::max(ep.rover_vel_var, 0.0)), kMotionVelSigmaMps);
      doppler_vel_nav = vel_enu;
      doppler_sigma_vel = sigma_white;
      // Nav-frame (ENU) velocity covariance. vel_enu = R_e_n' * vel_ecef, so the
      // covariance rotates the same way. The per-axis floor is kept: the LS
      // covariance describes the range-rate geometry, not multipath on the
      // range rates themselves, so it can be optimistic in absolute terms even
      // when its SHAPE is right - and the shape is what this change is for.
      const double floor_var = kMotionVelSigmaMps * kMotionVelSigmaMps;
      Eigen::Matrix3d vel_cov_enu =
          Eigen::Matrix3d::Identity() * (sigma_white * sigma_white);
      if (ep.rover_vel_cov.allFinite() &&
          ep.rover_vel_cov.diagonal().sum() > 0.0) {
        vel_cov_enu = R_e_n.transpose() * ep.rover_vel_cov * R_e_n;
        for (int i = 0; i < 3; ++i)
          vel_cov_enu(i, i) = std::max(vel_cov_enu(i, i), floor_var);
      }
      const double f_gnss = (gnss_interval_s_ > 1e-3) ? 1.0 / gnss_interval_s_ : 5.0;
      const double corr_inflation =
          std::sqrt(std::max(1.0, doppler_corr_time_s_ * f_gnss));
      auto robust = [&](const Eigen::Matrix3d& cov) {
        return gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(adapter_cfg_.huber_k),
            gtsam::noiseModel::Gaussian::Covariance(cov));
      };
      // Sigmas scale the covariance by their square.
      const Eigen::Matrix3d cov_abs =
          vel_cov_enu * (corr_inflation * corr_inflation);
      gnss_graph.add(
          gtsam::PriorFactor<gtsam::Vector3>(vk, vel_enu, robust(cov_abs)));
    } else if (reanchor) {
      // Re-anchoring without Doppler: the coasted velocity is as unreliable as
      // the coasted position, so replace it with a loose zero rather than let
      // it propagate through the IMU chain unchallenged.
      gnss_graph.add(gtsam::PriorFactor<gtsam::Vector3>(
          vk, gtsam::Vector3::Zero(),
          gtsam::noiseModel::Isotropic::Sigma(3, kReanchorVelStdMps)));
    }

    // Vehicle-motion pseudo-measurements (attitude aiding + NHC + ZUPT). They
    // need the body attitude, so they are only meaningful when the IMU chain is
    // valid this epoch (use_imu) and the attitude has not just been re-anchored.
    if (use_imu && !reanchor) {
      const double speed = predicted.velocity().norm();
      const Eigen::Vector3d omega_body = omegaBody();
      // Velocity-direction attitude aiding. Uses the Doppler velocity gated
      // above, so an epoch whose Doppler was rejected supplies no heading
      // either. The IMU chain constrains the yaw RATE, not the absolute yaw, so
      // the aiding factors over the window can rotate the whole window's heading
      // together and no state surgery is needed for a small offset.
      //
      // A LARGE first offset is handled separately: gtsam's Unit3 error is the
      // chordal sin(theta), which DECREASES past 90 deg and vanishes at 180, so
      // the optimizer would settle at the reversed heading. Initialization runs
      // on a static window with yaw = 0, so a big first offset is normal here.
      if (doppler_ok && vatt_cfg_.enable && doppler_vel_nav.norm() > 1e-6) {
        const gtsam::Rot3 nRb = predicted.pose().rotation();
        const double misalign =
            gnss_fgo::velocityAttitudeMisalign(nRb, doppler_vel_nav);
        const double course_sigma = gnss_fgo::velocityAttitudeSigma(
            doppler_vel_nav.norm(), doppler_sigma_vel,
            vatt_cfg_.sideslip_deg * D2R);
        // While the run has no heading yet, decide from an AGREEMENT of several
        // consecutive epochs rather than from one course. See YawAlignConfig:
        // the replacement is outright, and a single low-speed course is not
        // trustworthy enough to make it on.
        if (!yaw_aligned_) {
          double dpsi = 0.0, spread = 0.0;
          const bool agreed = yaw_align_vote_.update(
              std::atan2(doppler_vel_nav.y(), doppler_vel_nav.x()),
              nRb.rpy().z(), course_sigma, doppler_vel_nav.head<2>().norm(),
              yaw_align_cfg_, &dpsi, &spread);
          if (agreed && std::fabs(dpsi) > attitude_align_reanchor_rad_) {
            // Re-anchor instead: tryInitialize's in-motion branch takes the
            // heading from the course exactly, and discarding the window is
            // right because every pose in it was built on the wrong heading.
            RCLCPP_INFO(get_logger(),
                "Heading is %.1f deg off the course over ground (%d epochs "
                "agreeing to %.1f deg, course sigma %.2f deg) - re-anchoring on "
                "the course.", dpsi * R2D, yaw_align_cfg_.agree_epochs,
                spread * R2D, course_sigma * R2D);
            yaw_align_vote_.reset();
            resetGraph();
            tryInitialize(ep, t_ros);
            return;
          }
          if (agreed) {
            // Agreed and already small: nothing to replace, and "the heading has
            // agreed once" is exactly what the promotion below records.
            yaw_aligned_ = true;
            RCLCPP_INFO(get_logger(),
                "Heading agrees with the course to %.2f deg over %d epochs "
                "(spread %.1f deg) - attitude aiding is now robust and guarded.",
                dpsi * R2D, yaw_align_cfg_.agree_epochs, spread * R2D);
          }
        }
        // Not robust until the heading has agreed once: Huber would cap exactly
        // the residual that still has to be worked off. The factor applies its
        // own min_speed_mps gate, which stays at its (higher) value because a
        // per-epoch soft factor needs a course sigma the alignment does not.
        const bool added = gnss_fgo::addVelocityAttitudeFactor(
            gnss_graph, xk, nRb, doppler_vel_nav, doppler_sigma_vel, vatt_cfg_,
            yaw_aligned_ ? adapter_cfg_.huber_k : 0.0);
        if (added && !yaw_aligned_ && std::isfinite(misalign) &&
            misalign < 3.0 * course_sigma) {
          yaw_aligned_ = true;
          RCLCPP_INFO(get_logger(),
              "Heading agrees with the course to %.2f deg - attitude aiding is "
              "now robust and guarded.", misalign * R2D);
        }
      } else if (!yaw_aligned_) {
        yaw_align_vote_.reset();
      }
      gnss_fgo::addNhcFactor(gnss_graph, xk, vk, nhc_cfg_, omega_body, speed);
      if (gnss_fgo::zuptDetect(zupt_win, zupt_cfg_, prev_bias_.gyroscope(),
                               speed)) {
        gnss_fgo::addZuptFactor(gnss_graph, vk, zupt_cfg_);
      }
    }

    // One graph, one update: the inertial factor and this epoch's DDs together.
    gnss_graph.push_back(imu_graph);
    gnss_values.insert(imu_values);
    gtsam::FixedLagSmoother::KeyTimestampMap ts_gnss = ts_imu;
    for (const gtsam::Key k : gnss_values.keys()) ts_gnss[k] = t_ros;

    // Non-const: re-fetched after injecting the held integers below. The update
    // AND the estimate/marginal queries are all guarded together, so a
    // degenerate epoch (marginal failure) resets rather than crashing the node.
    // Per-key calculateEstimate: a full calculateEstimate() copies EVERY value
    // in the retained graph each epoch.
    gtsam::Pose3 pose;
    gtsam::Vector3 vel;
    Eigen::MatrixXd pose_cov;   // 6x6 tangent
    Eigen::Matrix3d vel_cov;
    // Joint posterior over [Pose3(6), Velocity(3), Bias(6)] in GRAPH charts.
    // Carried to the next epoch as the departure covariance of the pre-fit
    // gate's prediction (see the gate block in the next processEpoch).
    Eigen::MatrixXd state_cov15;
    bool state_cov15_ok = false;
    // The bias belonging to the SAME posterior as pose/vel/state_cov15. Filled
    // from ONE query in the hold block below, so the state carried forward can
    // never pair a pre-hold pose with a post-hold bias.
    gtsam::imuBias::ConstantBias epoch_bias;
    bool epoch_bias_ok = false;
    // Built here (not below) because its joint marginal already CONTAINS the
    // X and V marginal blocks: taking them from it removes two per-epoch
    // marginalCovariance() solves, each of which repeats the same clique-tree
    // shortcut work. AR consumes the same object further down.
    gnss_fgo::GraphArPosterior posterior;
    bool posterior_ok = false;
    std::string posterior_err;
    const auto t_gnss_stage = tick();
    try {
      const auto t_isam = tick();
      smoother_->update(gnss_graph, gnss_values, ts_gnss);
      if (reanchor) {
        // ISAM2 flags a variable for relinearization on the update that FINDS
        // the large delta, and applies it on the next one. A re-anchor moves
        // the pose by metres, so without this extra (empty) pass the epoch
        // would be published from a linearization point the DDs already
        // rejected. One extra pass only, and only on a re-anchor epoch.
        smoother_->update();
      }
      ms_isam_update = ms_since(t_isam);
      const auto t_est = tick();
      pose = smoother_->calculateEstimate<gtsam::Pose3>(xk);
      vel = smoother_->calculateEstimate<gtsam::Vector3>(vk);
      ms_estimate = ms_since(t_est);
      const auto t_post = tick();
      if (static_cast<int>(pairs.size()) >= kMinDdForAr) {
        posterior_ok = gnss_fgo::buildGraphPosteriorNav(
            smoother_->getISAM2(), xk, vk, bk, lever_arm_, ecef_T_nav_, pairs,
            posterior, &posterior_err);
      }
      ms_ar_posterior = ms_since(t_post);
      const auto t_marg = tick();
      if (posterior_ok) {
        // full_cov layout is [Pose3(6), Velocity(3), Bias(6)].
        pose_cov = posterior.full_cov.topLeftCorner<6, 6>();
        vel_cov = posterior.full_cov.block<3, 3>(6, 6);
        // The whole joint block, which the next epoch's gate needs: it must
        // propagate pose, velocity and bias TOGETHER, and the cross terms only
        // exist in a joint marginal. Free here - AR already paid for it.
        state_cov15 = posterior.full_cov;
        state_cov15_ok = true;
      } else {
        // No AR this epoch. One joint query instead of two separate marginals:
        // same clique-tree shortcut work, and it yields the cross terms as well
        // as the diagonal blocks the publisher needs.
        try {
          const gtsam::JointMarginal jm =
              smoother_->getISAM2().jointMarginalCovariance({xk, vk, bk});
          const std::array<gtsam::Key, 3> sk{xk, vk, bk};
          const std::array<int, 3> dim{6, 3, 6};
          const std::array<int, 3> off{0, 6, 9};
          state_cov15 = Eigen::MatrixXd::Zero(15, 15);
          for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
              state_cov15.block(off[i], off[j], dim[i], dim[j]) =
                  jm.at(sk[i], sk[j]);
          pose_cov = state_cov15.topLeftCorner<6, 6>();
          vel_cov = state_cov15.block<3, 3>(6, 6);
          state_cov15_ok = state_cov15.allFinite();
        } catch (const std::exception&) {
          // Joint marginals can fail where single ones still succeed; fall back
          // rather than lose the epoch. The gate then propagates without cross
          // terms next epoch, which is the pre-fix behaviour, not a new risk.
          pose_cov = smoother_->marginalCovariance(xk);
          vel_cov = smoother_->marginalCovariance(vk);
          state_cov15_ok = false;
        }
      }
      ms_marginal = ms_since(t_marg);
    } catch (const gtsam::IndeterminantLinearSystemException& e) {
      RCLCPP_ERROR(get_logger(),
                   "GNSS-stage update/marginal failed (%s) - reinitializing.",
                   e.what());
      dumpKeyDiagnostics(e.nearbyVariable());
      resetGraph();
      tryInitialize(ep, t_ros);  // re-anchor on THIS epoch (warm: no wait)
      return;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(),
                   "GNSS-stage update/marginal failed (%s) - reinitializing.",
                   e.what());
      resetGraph();
      tryInitialize(ep, t_ros);
      return;
    }

    ms_gnss_stage = ms_since(t_gnss_stage);

    Eigen::Vector3d pub_pos;
    Eigen::Matrix3d pub_cov;
    antennaFromPose(pose, pose_cov, pub_pos, pub_cov);
    const Eigen::Vector3d graph_float_ant = pub_pos;  // graph (stage-2) float (A2)

    uint8_t status = grs::GnssSolution::STATUS_FLOAT;
    double ratio = 0.0;
    gtsam::Pose3 fixed_pose = pose;
    gtsam::Vector3 fixed_vel = vel;
    Eigen::MatrixXd fixed_pose_cov = pose_cov;
    Eigen::Matrix3d fixed_vel_cov = vel_cov;

    // The joint stage-2 marginal over Pose, Velocity, Bias and all ambiguity
    // keys was extracted above. LAMBDA and the published FIX state use that
    // same posterior; no GNSS observation is applied a second time.
    //
    // This block only DECIDES; it must not publish. `status` / `pub_*` /
    // `fixed_*` are committed once, after every escape gate below, so the
    // published state can never disagree with `ar.fixed`. Committing here and
    // letting a gate flip `ar.fixed` afterwards would publish STATUS_FIX with
    // the integer-conditioned position the node had just decided not to trust -
    // and, since `status` also selects the Odometry pose and suppresses the
    // post-hold FLOAT refresh, the rejected integers would reach attitude and
    // velocity as well.
    const auto t_ar = tick();
    gnss_fgo::ArResult ar;
    gnss_fgo::ConditionedNavState conditioned;
    if (ar_enabled_ && static_cast<int>(pairs.size()) >= kMinDdForAr) {
      if (posterior_ok) {
        ar = gnss_fgo::resolveAmbiguitiesPosterior(
            gnss_ep, pairs, posterior, dd_layout, adapter_cfg_, ar_opt_);
        ratio = ar.ratio;
        conditioned = gnss_fgo::makeConditionedNavState(
            pose, vel, lever_arm_, ecef_T_nav_, ar);
        // makeConditionedNavState returns ok=false whenever !ar.fixed, so this
        // is the genuine "fixed, but the conditioned state does not reproduce
        // the analytical antenna" case.
        if (ar.fixed && !conditioned.ok) {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "Rejected FIX publish: conditioned navigation state does not "
              "reproduce analytical antenna within 1 mm.");
          ar.fixed = false;
        }
      } else {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Stage-2 graph AR posterior unavailable - skipping AR (%s).",
            posterior_err.c_str());
      }
    }
    // Escape gate 1: reject the fix when the pseudoranges have disagreed with
    // the integer CORRECTION for several consecutive epochs. See
    // CodeGrowthMonitor - one epoch of disagreement is a false alarm.
    if (ar.fixed &&
        code_growth_.reject(ar.code_resid_growth,
                            ar_opt_.max_code_resid_growth_m,
                            ar_opt_.code_resid_persist)) {
      ar.fixed = false;
      ar.fail = gnss_fgo::ArFail::CodeGrowthReject;
      code_growth_.reset();
    }
    // Escape gate 2: the FLOAT is in the wrong place, which gate 1 cannot see.
    // Refusing the fix alone would leave it there, so drop the carried
    // ambiguities too - they are what dragged the state, and re-keying is the
    // same recovery the re-anchor path uses.
    if (code_rms_.reject(ar.code_resid_rms, ar_opt_.max_code_resid_m,
                         ar_opt_.code_resid_rms_persist)) {
      RCLCPP_WARN(get_logger(),
          "Code DDs put the float %.2f m off (> %.2f m for %d epochs) - dropping "
          "the carried ambiguities.", ar.code_resid_rms,
          ar_opt_.max_code_resid_m, ar_opt_.code_resid_rms_persist);
      ar.fixed = false;
      ar.fail = gnss_fgo::ArFail::CodeGrowthReject;
      amb_mgr_->invalidateAll();
      hold_candidates_.clear();
      code_growth_.reset();
      code_rms_.reset();
      nfix_ = 0;
    }
    // Single commit point: every gate has now had its say. `conditioned.ok`
    // implied ar.fixed when it was computed, and the gates only ever clear
    // ar.fixed, so this condition alone is sufficient.
    if (ar.fixed) {
      fixed_pose = conditioned.pose;
      fixed_vel = conditioned.velocity;
      fixed_pose_cov = conditioned.pose_cov;
      fixed_vel_cov = conditioned.velocity_cov;
      pub_pos = ar.fixed_pos;
      pub_cov = ar.state_cov;
      status = grs::GnssSolution::STATUS_FIX;
    }
    nfix_ = ar.fixed ? nfix_ + 1 : 0;
    // Code-only DD position for the dump. Deliberately built from the RAW `ep`,
    // not the pre-fit-filtered `gnss_ep`: the filter gates against the
    // predicted state, and a yardstick derived from the state it is meant to
    // check is no yardstick at all. This is the one position in the dump that
    // the IMU chain and the held integers cannot move. Only recorded, never
    // fed back.
    gnss_fgo::CodeDdSolution ddwls;
    if (ar_dump_->enabled()) {
      ddwls = gnss_fgo::solveCodeDdWls(ep, adapter_cfg_, ep.rover_ecef_apriori);
    }
    // Record the pre-LAMBDA float and its covariance next to the decision they
    // produced (no-op unless debug.ar_dump_dir is set).
    ar_dump_->writeEpoch(ep.week, ep.tow, gnss_ep, pairs, ar, held_dd_.size(), nfix_,
                         graph_float_ant, n_gated, ddwls);
    ms_ar = ms_since(t_ar);
    const auto t_hold = tick();
    {
      // Hold accepted integers as two-variable DD constraints (added once,
      // removed on re-key); runs every epoch to clean stale holds. Re-fetch the
      // now-tightened state to carry forward and to refresh a FLOAT publish.
      // Holds start only after min_fix_to_hold consecutive fixes (RTKLIB
      // minfix); the empty-spec call still cleans up stale holds.
      const auto observed_specs = gnss_fgo::collectHoldSpecs(
          *amb_mgr_, gnss_ep, ar, 0.5, ar_opt_.min_lock);
      const auto specs = gnss_fgo::confirmHoldSpecs(
          hold_candidates_, observed_specs, min_fix_to_hold_);
      const auto t_hold_upd = tick();
      const auto hr = gnss_fgo::applyHolds(*smoother_, *amb_mgr_, held_dd_, specs,
                                           adapter_cfg_.hold_sigma_cycles);
      ms_hold_update = ms_since(t_hold_upd);
      const auto t_refetch = tick();
      if (hr == gnss_fgo::HoldResult::Failure) {
        RCLCPP_WARN(get_logger(), "Hold update failed - reinitializing.");
        resetGraph();
        tryInitialize(ep, t_ros);  // re-anchor on THIS epoch (warm: no wait)
        return;
      }
      // Re-read the state whenever the hold update MOVED the graph (Success;
      // NoChange is its own value), on FIX epochs TOO. The published FIX stays
      // the analytical integer-conditioned position - pub_pos/pub_cov are not
      // touched on that path - but everything CARRIED FORWARD must belong to
      // ONE posterior. Skipping FIX, where holds almost always change, left
      // prev_state_ pre-hold beside a separately re-queried post-hold bias.
      //
      // ONE jointMarginalCovariance rather than two marginalCovariance calls:
      // the same clique-tree shortcut work, and it also yields the 15x15 the
      // next epoch's gate needs, which neither branch used to refresh at all.
      if (hr == gnss_fgo::HoldResult::Success) {
        try {
          const gtsam::Pose3 post_pose =
              smoother_->calculateEstimate<gtsam::Pose3>(xk);
          const gtsam::Vector3 post_vel =
              smoother_->calculateEstimate<gtsam::Vector3>(vk);
          const auto post_bias =
              smoother_->calculateEstimate<gtsam::imuBias::ConstantBias>(bk);
          const gtsam::JointMarginal jm =
              smoother_->getISAM2().jointMarginalCovariance({xk, vk, bk});
          const std::array<gtsam::Key, 3> sk{xk, vk, bk};
          const std::array<int, 3> dim{6, 3, 6};
          const std::array<int, 3> off{0, 6, 9};
          Eigen::MatrixXd post_cov15 = Eigen::MatrixXd::Zero(15, 15);
          for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
              post_cov15.block(off[i], off[j], dim[i], dim[j]) =
                  jm.at(sk[i], sk[j]);
          if (!post_cov15.allFinite())
            throw std::runtime_error("non-finite post-hold joint marginal");
          // Commit as a unit - either the whole posterior is replaced or none
          // of it is, so a throw halfway cannot leave a mixed state behind.
          pose = post_pose;
          vel = post_vel;
          epoch_bias = post_bias;
          epoch_bias_ok = true;
          pose_cov = post_cov15.topLeftCorner<6, 6>();
          vel_cov = post_cov15.block<3, 3>(6, 6);
          state_cov15 = post_cov15;
          state_cov15_ok = true;
          // A FIX publishes the analytical integer-conditioned antenna, so only
          // a FLOAT refreshes the published position from the tightened graph.
          if (status != grs::GnssSolution::STATUS_FIX)
            antennaFromPose(pose, pose_cov, pub_pos, pub_cov);
        } catch (const gtsam::IndeterminantLinearSystemException& e) {
          // The hold UPDATE succeeded; only this refresh QUERY failed
          // (numerically extreme conditioning on a long arc can break the
          // marginal shortcut computation). Reinitializing would discard every
          // carried arc over a read-only failure, and a genuinely corrupted
          // graph still resets at the next epoch's update - so keep the
          // pre-hold estimates, which are ONE consistent posterior, and
          // invalidate the caches that would otherwise claim to be post-hold.
          RCLCPP_WARN(get_logger(),
                      "Post-hold estimate failed (%s) - keeping pre-hold "
                      "estimates.", e.what());
          dumpKeyDiagnostics(e.nearbyVariable());
          state_cov15_ok = false;
        } catch (const std::exception& e) {
          RCLCPP_WARN(get_logger(),
                      "Post-hold estimate failed (%s) - keeping pre-hold "
                      "estimates.", e.what());
          state_cov15_ok = false;
        }
        // epoch_bias_ok stays false: querying here would hand the POST-hold
        // bias to the PRE-hold pose, the mixture this block exists to prevent.
        // The previous epoch's bias is carried instead - stale, but a value of
        // the same slowly-varying random-walk state.
      } else {
        // NoChange: the graph has not moved since pose/vel/state_cov15 were
        // read above, so one bias query completes that same posterior.
        try {
          epoch_bias =
              smoother_->calculateEstimate<gtsam::imuBias::ConstantBias>(bk);
          epoch_bias_ok = true;
        } catch (const std::exception&) {
          // Keep the previous bias estimate on a degenerate query.
        }
      }
      ms_hold_refetch = ms_since(t_refetch);
      // Scope the hold-refresh age cap to the pairs that actually carry a hold
      // now (the cap re-verifies HELD integers; a float arc has nothing to
      // re-verify and must live until a real slip/outage).
      std::set<std::pair<int, int>> held_sats;
      for (const auto& kv : held_dd_) {
        held_sats.insert({std::get<0>(kv.first), std::get<2>(kv.first)});
        held_sats.insert({std::get<1>(kv.first), std::get<2>(kv.first)});
      }
      amb_mgr_->setHeld(std::move(held_sats));
    }
    ms_hold = ms_since(t_hold);

    const rclcpp::Time stamp_pub(static_cast<int64_t>(t_ros * 1e9),
                                 t_epoch.get_clock_type());
    // The graph/DR seed below remains float (possibly hold-conditioned). A
    // current FIX publish uses the one integer-conditioned joint posterior.
    gtsam::Pose3 odom_pose = pose;
    Eigen::MatrixXd odom_pose_cov = pose_cov;
    gtsam::Vector3 odom_vel = vel;
    Eigen::Matrix3d odom_vel_cov = vel_cov;
    if (status == grs::GnssSolution::STATUS_FIX) {
      odom_pose = fixed_pose;
      odom_pose_cov = fixed_pose_cov;
      odom_vel = fixed_vel;
      odom_vel_cov = fixed_vel_cov;
    }
    // GnssSolution names the ANTENNA phase center, Odometry the body origin:
    // one state, two physical points, so the velocity is converted for the
    // former and passed through for the latter.
    gnss_fgo::AntennaVelocityCov avc;
    avc.vel_nav = odom_vel_cov;
    avc.rot = odom_pose_cov.topLeftCorner<3, 3>();
    if (state_cov15_ok) avc.bias_gyro = state_cov15.block<3, 3>(12, 12);
    avc.sigma_gyr = last_imu_valid_ ? imu_sigma_gyr_ : 0.0;
    // The joint posterior describes the FLOAT state; a FIX publishes the
    // integer-conditioned velocity, whose posterior is 3x3, so it takes the
    // block-diagonal fallback. The gyro-bias block stays the float's - AR
    // conditions the ambiguities, not the IMU bias.
    if (status != grs::GnssSolution::STATUS_FIX) {
      avc.joint = state_cov15;
      avc.joint_ok = state_cov15_ok;
    }
    const gnss_fgo::AntennaVelocity ant_vel = gnss_fgo::antennaVelocityNav(
        odom_pose.rotation(), odom_vel, omegaBody(), lever_arm_, avc);

    // One decision for the whole epoch. A slot the gap publisher already filled
    // is not re-published on EITHER topic - the epoch has still entered the
    // graph above and improves every later epoch, it just produces no output.
    if (slots_.claim(ctow)) {
      publishSolution(ep, stamp_pub, pub_pos, pub_cov, ant_vel.v_nav,
                      ant_vel.cov_nav, status, ratio);
      publishOdometry(stamp_pub, odom_pose, odom_pose_cov, odom_vel,
                      odom_vel_cov);
      ++n_published_;
    } else {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "Epoch GPST %.3f arrived after its slot was already published "
          "(%llu so far); folded into the graph but not re-published. A rising "
          "count means the node is not keeping up with the observation rate.",
          ctow, static_cast<unsigned long long>(slots_.suppressed()));
    }

    // Health summary on a throttle as well as at shutdown. A node fed by a
    // live decoder may run for weeks and never reach its destructor, so a
    // report that only exists there cannot warn anyone about a coast that has
    // taken over the output.
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 60000,
        "Health: %llu rover msg received, %llu epoch(s) solved, %llu "
        "dead-reckoned (%.1f%% of output), %llu coast truncation(s), "
        "%llu late epoch(s) not re-published.",
        static_cast<unsigned long long>(
            n_rover_msgs_received_.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(n_published_ - n_predicted_pub_),
        static_cast<unsigned long long>(n_predicted_pub_),
        n_published_ > 0 ? 100.0 * static_cast<double>(n_predicted_pub_) /
                               static_cast<double>(n_published_)
                         : 0.0,
        static_cast<unsigned long long>(n_coast_truncated_),
        static_cast<unsigned long long>(slots_.suppressed()));

    // Context for the gap dead-reckoning publisher: rover grid interval,
    // week/tow anchor, base for the ENU origin, and the posterior pose cov.
    // Rover grid interval, pinned ONCE from the observation stream itself (the
    // median of the first kIntervalSamples spacings) rather than tracked as a
    // running average of the inertial dt. The dead-reckoning grid has to land on
    // the real epoch times, and an average keeps moving; the median is immune to
    // the gaps that motivate the publisher in the first place. These epochs are
    // estimated normally - only the interval measurement is collected here.
    if (!gnss_interval_pinned_) {
      if (expected_imu_dt > 0.05 && expected_imu_dt < 0.5)
        interval_samples_.push_back(expected_imu_dt);
      const double pinned = gnss_fgo::medianEpochInterval(interval_samples_);
      if (pinned > 0.0) {
        gnss_interval_s_ = pinned;
        gnss_interval_pinned_ = true;
        interval_samples_.clear();
        interval_samples_.shrink_to_fit();
        RCLCPP_INFO(get_logger(),
                    "Rover epoch interval pinned at %.3f s (%.1f Hz) from the "
                    "observation stream.", gnss_interval_s_,
                    1.0 / gnss_interval_s_);
      }
    }
    last_week_ = ep.week;
    last_tow_ = ep.tow;
    // Last epoch whose DDs actually reached the graph. Drives the re-anchor gap
    // test, so a long run of gated-out epochs counts as a GNSS outage - which
    // is exactly what it is from the estimator's point of view.
    if (!gnss_ep.dd.empty()) {
      last_dd_ctow_ = ctow;
      gap_latch_.onUsableGnss();
    }
    if (ep.base_ecef.norm() > 0.0) last_base_ecef_ = ep.base_ecef;
    last_pose_cov_ = pose_cov;
    last_vel_cov_ = vel_cov;
    last_state_cov15_ok_ = state_cov15_ok;
    if (state_cov15_ok) last_state_cov15_ = state_cov15;
    n_pred_since_epoch_ = 0;

    // pose, vel, epoch_bias, pose_cov, vel_cov and state_cov15 above all come
    // from ONE posterior (see the hold block) - never a pre-hold pose beside a
    // post-hold bias.
    prev_state_ = gtsam::NavState(pose, vel);
    if (epoch_bias_ok) prev_bias_ = epoch_bias;
    prev_t_ros_ = t_ros;
    // Keep the analytical fixed position out of the next epoch's elevation
    // mask / linearization prior. A wrong integer hypothesis would otherwise
    // feed back through preprocessing even before it became a held graph
    // constraint. The final pose below is the float (possibly hold-tightened)
    // state, projected to the antenna phase center.
    Eigen::Matrix3d apriori_cov;
    antennaFromPose(pose, pose_cov, last_antenna_ecef_, apriori_cov);
    last_antenna_ecef_valid_ = true;
    ++epoch_index_;

    // Real-time budget (see gnss_fgo.cpp): with the fixed-lag window the
    // per-epoch solve time stays flat vs. epoch index; full history grows it.
    const double solve_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - solve_t0)
                                .count();
    solve_ms_max_ = std::max(solve_ms_max_, solve_ms);
    if (imu_diag_.is_open()) {
      // fixed6: ostream's default 6 SIGNIFICANT digits quantises tow to whole
      // seconds (550381.2 -> "550381"), which collapses the 5 epochs in each
      // second onto one key and destroys the pairing with the reference.
      imu_diag_ << ep.week << ',' << gnss_fgo::fixed6(ep.tow) << ','
                << (use_imu ? 1 : 0)
                << ',' << n_imu_fallback_ << ',' << max_imu_gap_s_
                << ',' << n_imu_message_lost_.load() << ','
                << n_imu_received_.load() << ',' << solve_ms
                << ',' << ms_imu_stage << ',' << ms_gate << ',' << ms_gnss_stage
                << ',' << ms_isam_update << ',' << ms_estimate
                << ',' << ms_ar_posterior << ',' << ms_marginal
                << ',' << ms_ar << ',' << ms_hold
                << ',' << ms_hold_update << ',' << ms_hold_refetch;
      // prev_bias_/prev_state_ were just refreshed from this epoch's posterior.
      const gtsam::Vector3 ba = prev_bias_.accelerometer();
      const gtsam::Vector3 bg = prev_bias_.gyroscope();
      const gtsam::Vector3 rpy = prev_state_.attitude().rpy();
      imu_diag_ << ',' << ba.x() << ',' << ba.y() << ',' << ba.z()
                << ',' << bg.x() << ',' << bg.y() << ',' << bg.z()
                << ',' << rpy.x() * R2D << ',' << rpy.y() * R2D
                << ',' << rpy.z() * R2D
                << ',' << prev_state_.velocity().norm()
                << ',' << expected_imu_dt << '\n';
      imu_diag_.flush();
    }
    if (solve_ms > kEpochBudgetMs) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Epoch solve %.0f ms exceeds the %.0f ms real-time budget - graph too "
          "large; reduce graph.lag_s.", solve_ms, kEpochBudgetMs);
    }

    double llh[3];
    const double e[3] = {last_antenna_ecef_.x(), last_antenna_ecef_.y(),
                         last_antenna_ecef_.z()};
    ecef2pos(e, llh);
    // Arc diagnostics: short mean arc age / high re-key counts mean the
    // carried ambiguities never accumulate and the float stays at code level.
    // solve ms (this / max) exposes the real-time headroom.
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), kPositionLogThrottleMs,
        "FGO %s | LLH: %.8f %.8f %.3f | Ratio: %.1f | DD: %zu (gated %d) | "
        "dT_imu: %.2f | "
        "Arcs: %zu (fresh %zu, rekey %llu, mean %.0fs) | hold: %zu | nfix: %d | "
        "solve: %.0f/%.0f ms",
        status == grs::GnssSolution::STATUS_FIX ? "FIX" : "FLOAT",
        llh[0] * R2D, llh[1] * R2D, llh[2], ratio, ep.dd.size(), n_gated,
        use_imu ? pim.deltaTij() : 0.0,
        amb_mgr_->liveCount(), amb_mgr_->freshCount(),
        static_cast<unsigned long long>(amb_mgr_->totalRekeys()),
        amb_mgr_->meanArcAge(ep.week * 604800.0 + ep.tow), held_dd_.size(),
        nfix_, solve_ms, solve_ms_max_);
  }

  // Static initialization: average accelerometer -> roll/pitch, average gyro
  // -> gyro bias, first GNSS a-priori -> ENU anchor and initial position.
  // A mid-run reinitialization PRESERVES the nav frame (the anchor's gravity
  // direction is off by only ~0.16 mrad per km travelled, absorbed by the
  // accelerometer bias), so the output/odom frame never jumps; if the platform
  // is moving, the last good attitude/bias are reused instead of the static
  // averages (which would be corrupted by motion).
  void tryInitialize(const gnss_utils::PreprocessedEpoch& ep, double t_ros) {
    // A cold start has no state at all, so it needs GNSS to place the anchor.
    // (A WARM restart does not: it reuses the existing nav frame and the last
    // attitude/bias, so it can re-anchor on a DD-less epoch too.)
    if (!nav_frame_valid_ && ep.dd.empty()) return;

    std::vector<ImuSample> window;
    {
      std::lock_guard<std::mutex> imu_lk(imu_mtx_);
      for (const auto& s : imu_buffer_) {
        if (s.t >= t_ros - init_imu_duration_ && s.t <= t_ros)
          window.push_back(s);
      }
    }
    // The window must be COMPLETE in sensor time, not merely long enough.
    // Samples are selected by timestamp, so the set is a function of the data
    // only once they have all been DELIVERED; accepting a partially delivered
    // span makes the initial attitude and gyro bias depend on arrival timing,
    // which a fix-and-hold estimator amplifies into a different trajectory.
    // Waiting costs at most one epoch - the IMU stream is already ahead.
    const double window_span =
        window.size() >= 2 ? window.back().t - window.front().t : 0.0;
    const double sample_dt = window.size() >= 2
        ? window_span / static_cast<double>(window.size() - 1)
        : 0.0;
    const bool window_ok =
        window.size() >= 10 &&
        window.front().t <= t_ros - init_imu_duration_ + sample_dt &&
        window.back().t >= t_ros - sample_dt;
    // The window exists only to derive attitude/bias from a STATIC average. A
    // warm restart already has better values (the last converged estimate), so
    // requiring the window there would drop epochs for nothing - the epochs
    // spent rebuilding it are exactly the availability gap vs. gnss_fgo.
    if (!window_ok && !nav_frame_valid_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "Waiting for a COMPLETE IMU initialization window (%zu samples, "
          "%.2f/%.2f s covered).",
          window.size(), window_span, init_imu_duration_);
      return;
    }

    Eigen::Vector3d acc_mean = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyr_mean = Eigen::Vector3d::Zero();
    double acc_std = 0.0;
    bool is_static = false;
    if (window_ok) {
      for (const auto& s : window) {
        acc_mean += s.acc;
        gyr_mean += s.gyr;
      }
      acc_mean /= static_cast<double>(window.size());
      gyr_mean /= static_cast<double>(window.size());
      double acc_var = 0.0;
      for (const auto& s : window) acc_var += (s.acc - acc_mean).squaredNorm();
      acc_std = std::sqrt(acc_var / static_cast<double>(window.size()));
      gnss_fgo::StaticInitConfig scfg;
      scfg.max_gyro_radps = kStaticGyroRadps;
      scfg.max_acc_std_mps2 = kStaticAccStdMps2;
      scfg.acc_norm_tol_mps2 = kStaticAccNormTolMps2;
      scfg.max_speed_mps = kStaticMaxSpeedMps;
      scfg.gravity_mps2 = kGravity;
      is_static = gnss_fgo::staticInitDetect(
          acc_mean, acc_std, gyr_mean, ep.rover_vel_valid,
          ep.rover_vel_ecef.norm(), scfg);
    }
    if (!is_static && !nav_frame_valid_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Platform is moving (|gyro| %.3f rad/s, acc std %.2f m/s^2) - waiting "
          "for a static window to initialize.", gyr_mean.norm(), acc_std);
      return;
    }

    // ENU anchor: set ONCE per session (a reinitialization keeps the frame).
    if (!nav_frame_valid_) {
      Eigen::Vector3d anchor = ep.rover_ecef_apriori;
      if (get_parameter("local_origin.mode").as_string() == "manual") {
        const auto pos = get_parameter("local_origin.pos").as_double_array();
        if (pos.size() == 3 && norm(pos.data(), 3) > 0.0) {
          if (get_parameter("local_origin.postype").as_string() == "llh") {
            const double llh[3] = {pos[0] * D2R, pos[1] * D2R, pos[2]};
            double e[3];
            pos2ecef(llh, e);
            anchor = Eigen::Vector3d(e[0], e[1], e[2]);
          } else {
            anchor = Eigen::Vector3d(pos[0], pos[1], pos[2]);
          }
        }
      }
      double anchor_llh[3];
      const double anchor_e[3] = {anchor.x(), anchor.y(), anchor.z()};
      ecef2pos(anchor_e, anchor_llh);
      double E[9];  // rows e,n,u: enu = E * d_ecef
      xyz2enu(anchor_llh, E);
      Eigen::Matrix3d R_enu_ecef;
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) R_enu_ecef(r, c) = E[r + c * 3];
      }
      ecef_T_nav_ = gtsam::Pose3(gtsam::Rot3(R_enu_ecef.transpose()),
                                 gtsam::Point3(anchor));
      nav_frame_valid_ = true;
    }

    // Attitude/bias: static averages, or the last good estimate when
    // reinitializing in motion.
    gtsam::Rot3 R0;
    gtsam::imuBias::ConstantBias bias0;
    if (is_static) {
      // Roll/pitch: rotate the measured specific-force direction onto +Z (up).
      const Eigen::Vector3d a_hat = acc_mean.normalized();
      const Eigen::Vector3d z(0.0, 0.0, 1.0);
      const Eigen::Vector3d axis = a_hat.cross(z);
      const double angle = std::atan2(axis.norm(), a_hat.dot(z));
      R0 = axis.norm() < 1e-9
               ? gtsam::Rot3()
               : gtsam::Rot3::AxisAngle(gtsam::Unit3(axis), angle);
      // A static gyro reads omega_ib^b = b_g + R_bn * omega_ie^n, so the raw
      // average is NOT the bias - it contains the Earth rate. Assigning it
      // wholesale while NavState::coriolis also removes the Earth rate would
      // remove the term TWICE. Subtract the modelled Earth rate here.
      //
      // Caveat: yaw is unobservable at initialisation, so only the vertical
      // component (omega*sin phi, yaw-invariant) is exact; the horizontal split
      // uses R0's yaw, which is zero by construction. The residual is well
      // inside kInitGyrBiasStd.
      gtsam::Vector3 omega_ie_nav = gtsam::Vector3::Zero();
      if (!earthRateNav(omega_ie_nav)) omega_ie_nav.setZero();
      bias0 = gtsam::imuBias::ConstantBias(
          Eigen::Vector3d::Zero(),
          gnss_fgo::staticInitGyroBias(gyr_mean, R0, omega_ie_nav));
      // Yaw is 0 by construction here, so the velocity-direction aiding has to
      // re-converge from whatever the true heading is: withdraw the promotion to
      // the robust+guarded model until it agrees once again.
      yaw_aligned_ = false;
    } else {
      R0 = prev_state_.attitude();
      bias0 = prev_bias_;
      // Re-anchoring in MOTION keeps the coasted attitude, whose yaw is exactly
      // the part that drifts during an outage (nothing observes it while the
      // graph is coasting). When this epoch's Doppler is trustworthy and fast
      // enough for the course to mean something, take the heading from it and
      // keep only the roll/pitch, which the accelerometer does hold.
      if (vatt_cfg_.enable && ep.rover_vel_valid &&
          (doppler_max_res_m_ <= 0.0 || ep.rover_vel_res <= doppler_max_res_m_)) {
        const Eigen::Vector3d v_nav =
            ecef_T_nav_.rotation().unrotate(gtsam::Point3(ep.rover_vel_ecef));
        // Gated on the COURSE UNCERTAINTY rather than on the aiding factor's
        // speed threshold: this is a one-shot heading replacement, not a
        // per-epoch soft factor, so what it needs is a course whose sigma is
        // small next to attitude.align_reanchor_deg. The caller reaching here
        // through the misalignment escape has already established agreement
        // across several epochs, INCLUDING that the offset it agreed on is one
        // the speed permits (see YawAlignConfig on reverse driving).
        const double course_sigma = gnss_fgo::velocityAttitudeSigma(
            v_nav.norm(),
            std::max(std::sqrt(std::max(ep.rover_vel_var, 0.0)),
                     kMotionVelSigmaMps),
            vatt_cfg_.sideslip_deg * D2R);
        // Re-check the reverse-driving bound here too: this branch is also
        // reached from a plain outage re-anchor, which has taken no vote.
        const double course_offset = std::fabs(gnss_fgo::YawAlignmentVote::wrapPi(
            std::atan2(v_nav.y(), v_nav.x()) - R0.rpy().z()));
        const bool reverse_ok =
            course_offset <= yaw_align_cfg_.reverse_ambiguous_deg * D2R ||
            v_nav.head<2>().norm() >= yaw_align_cfg_.reverse_speed_mps;
        if (course_sigma <= yaw_align_cfg_.max_sigma_deg * D2R && reverse_ok) {
          const gtsam::Rot3 aligned = gnss_fgo::alignYawToCourse(R0, v_nav);
          RCLCPP_INFO(get_logger(),
              "Re-anchoring in motion: heading taken from the course "
              "(%.1f -> %.1f deg, course sigma %.2f deg), roll/pitch kept.",
              R0.rpy().z() * R2D, aligned.rpy().z() * R2D, course_sigma * R2D);
          R0 = aligned;
          yaw_aligned_ = true;
          yaw_align_vote_.reset();
        }
      }
    }

    // Antenna at the current a-priori (in the preserved nav frame); body
    // origin = antenna - R0 * lever_arm.
    const gtsam::Point3 ant_nav =
        ecef_T_nav_.transformTo(gtsam::Point3(ep.rover_ecef_apriori));
    const gtsam::Point3 body0 = ant_nav - R0.rotate(lever_arm_);
    const gtsam::Pose3 pose0(R0, body0);

    pim_params_ = makePimParams();

    const double pos_std = kInitPosStd;
    const double rp_std = kInitAttStdRp;
    const double yaw_std = kInitAttStdYaw;
    const double vel_std = kInitVelStd;
    const double ab_std = kInitAccBiasStd;
    const double gb_std = kInitGyrBiasStd;

    // Initial velocity: zero when static; Doppler (or a loose zero) in motion.
    gtsam::Vector3 v0 = gtsam::Vector3::Zero();
    double v0_std = vel_std;
    if (!is_static) {
      if (ep.rover_vel_valid) {
        v0 = ecef_T_nav_.rotation().matrix().transpose() * ep.rover_vel_ecef;
        v0_std = std::max(std::sqrt(std::max(ep.rover_vel_var, 0.0)),
                          kMotionVelSigmaMps);
      } else {
        v0_std = 5.0;
      }
    }

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    values.insert(X(0), pose0);
    values.insert(V(0), v0);
    values.insert(B(0), bias0);
    // Pose3 tangent ordering: [rot(3), trans(3)]; yaw = rotation about up.
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(
        X(0), pose0,
        gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << rp_std, rp_std, yaw_std,
             pos_std, pos_std, pos_std).finished())));
    graph.add(gtsam::PriorFactor<gtsam::Vector3>(
        V(0), v0, gtsam::noiseModel::Isotropic::Sigma(3, v0_std)));
    graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
        B(0), bias0,
        gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << ab_std, ab_std, ab_std,
             gb_std, gb_std, gb_std).finished())));

    // Apply the SAME pre-fit outlier test every other epoch gets. The anchor
    // epoch is the most fragile one in the run - everything downstream is
    // linearized around it - so exempting it was backwards. The reference is
    // this epoch's code a-priori (the anchor itself), at single-point accuracy.
    int n_gated_init = 0;
    const gnss_utils::PreprocessedEpoch init_ep =
        gnss_fgo::filterDdPreFitInnovation(
            ep, ep.rover_ecef_apriori,
            Eigen::Matrix3d::Identity() * (kInitPosStd * kInitPosStd),
            adapter_cfg_, ar_opt_, &n_gated_init);
    gnss_fgo::addGroupedDdFactorsArm(init_ep, X(0), lever_arm_, ecef_T_nav_,
                                     adapter_cfg_, graph, values, *amb_mgr_);

    // All initial variables enter the window at t_ros.
    gtsam::FixedLagSmoother::KeyTimestampMap ts0;
    for (const gtsam::Key k : values.keys()) ts0[k] = t_ros;
    try {
      smoother_->update(graph, values, ts0);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Initialization update failed: %s", e.what());
      resetGraph();
      return;
    }

    // Read the initialization POSTERIOR once and hand the SAME state/covariance
    // to both the carried filter state and the published solution, so the next
    // epoch's IMU prediction and the gap dead-reckoning start from what was
    // published - not from the pre-optimization prior (pose0/v0/bias0), which
    // ignores this epoch's DD correction (P0-7).
    gtsam::Pose3 post_pose = pose0;
    gtsam::Vector3 post_vel = v0;
    gtsam::imuBias::ConstantBias post_bias = bias0;
    Eigen::MatrixXd post_pose_cov =
        Eigen::MatrixXd::Identity(6, 6) * (kInitPosStd * kInitPosStd);
    Eigen::Matrix3d post_vel_cov =
        Eigen::Matrix3d::Identity() * (kInitVelStd * kInitVelStd);
    try {
      post_pose = smoother_->calculateEstimate<gtsam::Pose3>(X(0));
      post_vel = smoother_->calculateEstimate<gtsam::Vector3>(V(0));
      post_bias =
          smoother_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(0));
      post_pose_cov = smoother_->marginalCovariance(X(0));
      post_vel_cov = smoother_->marginalCovariance(V(0));
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(),
          "Initialization posterior query failed (%s) - carrying the prior.",
          e.what());
    }

    prev_state_ = gtsam::NavState(post_pose, post_vel);
    prev_bias_ = post_bias;
    prev_t_ros_ = t_ros;
    {
      gtsam::Matrix36 H_pose;
      const gtsam::Point3 ant_nav = post_pose.transformFrom(lever_arm_, H_pose);
      const gtsam::Point3 ant_ecef = ecef_T_nav_.transformFrom(ant_nav);
      last_antenna_ecef_ =
          Eigen::Vector3d(ant_ecef.x(), ant_ecef.y(), ant_ecef.z());
    }
    last_antenna_ecef_valid_ = true;
    epoch_index_ = 1;
    initialized_ = true;
    // Anchor the gap dead-reckoning grid at this (re)initialization epoch.
    last_week_ = ep.week;
    last_tow_ = ep.tow;
    if (!init_ep.dd.empty()) last_dd_ctow_ = ep.week * 604800.0 + ep.tow;
    if (ep.base_ecef.norm() > 0.0) last_base_ecef_ = ep.base_ecef;
    last_pose_cov_ = post_pose_cov;
    last_vel_cov_ = post_vel_cov;
    // Seed the joint 15x15 as well, so a GNSS gap arriving immediately after
    // (re)initialization has something to propagate instead of falling to the
    // conservative fallback. Block-diagonal is EXACT here rather than an
    // approximation: at this instant the pose, velocity and bias states carry
    // only their own independent priors, and no measurement has yet correlated
    // them. The velocity block is stored in the GRAPH chart, which is what
    // predictedNavCov9 expects to convert.
    last_state_cov15_ = Eigen::MatrixXd::Zero(15, 15);
    last_state_cov15_.topLeftCorner<6, 6>() = post_pose_cov;
    last_state_cov15_.block<3, 3>(6, 6) = post_vel_cov;
    last_state_cov15_.block<3, 3>(9, 9) =
        Eigen::Matrix3d::Identity() * (kInitAccBiasStd * kInitAccBiasStd);
    last_state_cov15_.block<3, 3>(12, 12) =
        Eigen::Matrix3d::Identity() * (kInitGyrBiasStd * kInitGyrBiasStd);
    last_state_cov15_ok_ = last_state_cov15_.allFinite();
    n_pred_since_epoch_ = 0;

    // Drop IMU samples up to the anchor epoch, remembering the last one so the
    // first inter-epoch integration can hold it up to the next GNSS stamp.
    {
      std::lock_guard<std::mutex> imu_lk(imu_mtx_);
      while (!imu_buffer_.empty() && imu_buffer_.front().t <= t_ros) {
        last_imu_ = imu_buffer_.front();
        last_imu_valid_ = true;
        imu_buffer_.pop_front();
      }
    }

    // Publish the initialization epoch too. The state is only prior-quality
    // (position ~kInitPosStd), but it is a real, covariance-honest estimate of
    // this epoch - dropping it silently loses an epoch of availability on every
    // (re)initialization.
    publishInitialSolution(ep, t_ros, post_pose, post_vel, post_pose_cov,
                           post_vel_cov);

    double anchor_llh[3];
    const double anchor_e[3] = {ecef_T_nav_.translation().x(),
                                ecef_T_nav_.translation().y(),
                                ecef_T_nav_.translation().z()};
    ecef2pos(anchor_e, anchor_llh);
    RCLCPP_INFO(get_logger(),
        "Initialized%s: anchor LLH %.8f %.8f %.3f | %zu IMU samples "
        "| DD %zu (gated %d) | yaw prior sigma %.2f rad.",
        is_static ? "" : " (in motion - reused last attitude/bias)",
        anchor_llh[0] * R2D, anchor_llh[1] * R2D, anchor_llh[2], window.size(),
        init_ep.dd.size(), n_gated_init, yaw_std);
  }

  // FLOAT output for the epoch that (re)initialized the graph, from the SAME
  // initialization posterior handed to the carried state (see tryInitialize) -
  // so the published epoch and the next epoch's prediction start consistent.
  void publishInitialSolution(const gnss_utils::PreprocessedEpoch& ep,
                              double t_ros, const gtsam::Pose3& pose,
                              const gtsam::Vector3& vel,
                              const Eigen::MatrixXd& pose_cov,
                              const Eigen::Matrix3d& vel_cov) {
    gtsam::Matrix36 H_pose;
    const gtsam::Point3 ant_nav = pose.transformFrom(lever_arm_, H_pose);
    const gtsam::Point3 ant_ecef = ecef_T_nav_.transformFrom(ant_nav);
    const Eigen::Matrix<double, 3, 6> J = ecef_T_nav_.rotation().matrix() * H_pose;
    const Eigen::Vector3d pos(ant_ecef.x(), ant_ecef.y(), ant_ecef.z());
    const Eigen::Matrix3d cov = J * pose_cov * J.transpose();

    const rclcpp::Time stamp(static_cast<int64_t>(t_ros * 1e9),
                             epoch_clock_type_);
    // Initialization posterior: separate marginals only, so the antenna
    // velocity covariance takes the block-diagonal fallback.
    gnss_fgo::AntennaVelocityCov avc;
    avc.vel_nav = vel_cov;
    avc.rot = pose_cov.topLeftCorner<3, 3>();
    if (last_state_cov15_ok_)
      avc.bias_gyro = last_state_cov15_.block<3, 3>(12, 12);
    avc.sigma_gyr = last_imu_valid_ ? imu_sigma_gyr_ : 0.0;
    const gnss_fgo::AntennaVelocity ant_vel = gnss_fgo::antennaVelocityNav(
        pose.rotation(), vel, omegaBody(), lever_arm_, avc);
    if (slots_.claim(ep.week * 604800.0 + ep.tow)) {
      publishSolution(ep, stamp, pos, cov, ant_vel.v_nav, ant_vel.cov_nav,
                      grs::GnssSolution::STATUS_FLOAT, 0.0);
      publishOdometry(stamp, pose, pose_cov, vel, vel_cov);
      ++n_published_;
    }
  }

  // Diagnostic dump for an IndeterminantLinearSystemException: list every live
  // factor touching the offending key with its error at the current
  // linearization point; see gnss_fgo.cpp.
  void dumpKeyDiagnostics(gtsam::Key key) {
    const gtsam::Symbol sym(key);
    std::ostringstream os;
    os << "singular near " << static_cast<char>(sym.chr()) << sym.index()
       << "; touching factors:";
    int n = 0;
    const gtsam::Values& lin = smoother_->getISAM2().getLinearizationPoint();
    for (const auto& f : smoother_->getISAM2().getFactorsUnsafe()) {
      if (!f) continue;
      if (std::find(f->keys().begin(), f->keys().end(), key) == f->keys().end())
        continue;
      os << " {";
      for (const gtsam::Key k : f->keys()) {
        const gtsam::Symbol s(k);
        os << static_cast<char>(s.chr()) << s.index() << " ";
      }
      double err = std::numeric_limits<double>::quiet_NaN();
      try {
        err = f->error(lin);
      } catch (const std::exception&) {
      }
      os << typeid(*f).name() << " e=" << err << "}";
      ++n;
    }
    os << " count=" << n;
    RCLCPP_ERROR(get_logger(), "%s", os.str().c_str());
  }

  void resetGraph() {
    smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(
        smoother_lag_, makeIsamParams());
    epoch_index_ = 0;
    next_amb_id_ = 0;
    last_antenna_ecef_valid_ = false;
    initialized_ = false;
    nfix_ = 0;
    last_dd_ctow_ = -1.0;
    if (amb_mgr_) amb_mgr_->resetAll();
    held_dd_.clear();
    hold_candidates_.clear();
    code_growth_.reset();
    code_rms_.reset();
    gap_latch_.reset();
    // The votes measured an offset against a heading this reset discards.
    yaw_align_vote_.reset();
  }

  // FRAMES AND PHYSICAL POINTS OF THE ARGUMENTS, because they differ and the
  // difference has already produced one double-rotation bug:
  //   pos_ecef / pos_cov_ecef       : ANTENNA phase center, ECEF, published
  //                                   as-is; the ENU forms are derived here.
  //   vel_ant_nav / vel_cov_ant_nav : ANTENNA phase center, NAV frame. This
  //                                   function applies R_e_n itself, so an
  //                                   ECEF argument would be rotated twice.
  // Callers build the antenna velocity with gnss_fgo::antennaVelocityNav: the
  // graph's V(k) is the BODY origin and belongs in Odometry, not here.
  // Pure message formatter: it does NOT decide whether to publish. Slot
  // ownership is settled by slots_.claim() in the caller, once, before any of
  // this epoch's outputs are sent - so GnssSolution, Odometry and the publish
  // counters can never disagree about whether the epoch produced output.
  void publishSolution(const gnss_utils::PreprocessedEpoch& ep,
                       const rclcpp::Time& stamp,
                       const Eigen::Vector3d& pos_ecef,
                       const Eigen::Matrix3d& pos_cov_ecef,
                       const gtsam::Vector3& vel_ant_nav,
                       const Eigen::Matrix3d& vel_cov_ant_nav, uint8_t status,
                       double ratio) {
    auto msg = std::make_unique<grs::GnssSolution>();
    // The aligned observation epoch time (not now()): keeps the solution's
    // header time tied to the measurement epoch, so a downstream time-aligned
    // consumer (e.g. the loose EKF in "header" mode) pairs it correctly.
    msg->header.stamp = stamp;
    msg->header.frame_id = "gnss_link";
    msg->time_week = ep.week;
    msg->time_tow = ep.tow;
    msg->solution_source = grs::GnssSolution::SOLUTION_SOURCE_COMPUTED;
    msg->status = status;
    msg->ratio = static_cast<float>(ratio);
    msg->age_diff = static_cast<float>(ep.age_s);

    std::set<int> sats;
    for (const auto& d : ep.dd) {
      sats.insert(d.sat_ref);
      sats.insert(d.sat_tar);
    }
    msg->num_sats = static_cast<uint8_t>(sats.size());

    const float nan = std::numeric_limits<float>::quiet_NaN();
    msg->gdop = nan;
    msg->pdop = nan;
    msg->hdop = nan;
    msg->vdop = nan;

    // Antenna position is supplied directly in ECEF (FLOAT: pose + lever arm;
    // FIX: the analytical AR conditioned position).
    const double ecef[3] = {pos_ecef.x(), pos_ecef.y(), pos_ecef.z()};
    double llh[3];
    ecef2pos(ecef, llh);
    msg->latitude = llh[0] * R2D;
    msg->longitude = llh[1] * R2D;
    msg->altitude = llh[2];
    msg->pos_ecef.x = pos_ecef.x();
    msg->pos_ecef.y = pos_ecef.y();
    msg->pos_ecef.z = pos_ecef.z();
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) msg->pos_cov_ecef[3 * r + c] = pos_cov_ecef(r, c);
    }

    // ENU origin priority: fixed_origin (configured) > base station > local nav
    // anchor - the same convention as gnss_fgo and the RTK example, so the
    // pos_enu trajectories of all nodes share one origin.
    double origin[3] = {0.0, 0.0, 0.0};
    if (fixed_origin_valid_) {
      std::copy_n(fixed_origin_ecef_, 3, origin);
    } else if (ep.base_ecef.norm() > 0.0) {
      origin[0] = ep.base_ecef.x();
      origin[1] = ep.base_ecef.y();
      origin[2] = ep.base_ecef.z();
    } else if (ecef_T_nav_.translation().norm() > 0.0) {
      origin[0] = ecef_T_nav_.translation().x();
      origin[1] = ecef_T_nav_.translation().y();
      origin[2] = ecef_T_nav_.translation().z();
    }
    msg->pos_enu_org_ecef.x = origin[0];
    msg->pos_enu_org_ecef.y = origin[1];
    msg->pos_enu_org_ecef.z = origin[2];
    // The ENU position MEAN is anchored at the origin; every covariance and the
    // velocity use the tangent plane at the CURRENT antenna position. Those
    // references are deliberately different - see gnss_fgo::EnuFrames for the
    // contract and msg/GnssSolution.msg:71-86. Do not "unify" them.
    const gnss_fgo::EnuFrames enu = gnss_fgo::enuFrames(llh, origin);
    if (enu.has_origin) {
      const double origin_llh[3] = {enu.origin_lat, enu.origin_lon, 0.0};
      const double d_ecef[3] = {ecef[0] - origin[0], ecef[1] - origin[1],
                                ecef[2] - origin[2]};
      double enu_pos[3];
      ecef2enu(origin_llh, d_ecef, enu_pos);
      msg->pos_enu.x = enu_pos[0];
      msg->pos_enu.y = enu_pos[1];
      msg->pos_enu.z = enu_pos[2];
    }

    double q_ecef[9];
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) q_ecef[3 * r + c] = pos_cov_ecef(r, c);
    }
    double q_enu[9];
    gnss_utils::rotateCovariance(q_ecef, enu.cur_lat, enu.cur_lon, q_enu);
    for (int i = 0; i < 9; ++i) msg->pos_enu_cov[i] = q_enu[i];

    // The antenna velocity arrives in the nav frame (ENU at the anchor). ECEF
    // uses the anchor rotation; vel_enu / vel_enu_cov are then BOTH taken at
    // the current antenna position, per the message contract, so the published
    // velocity and its covariance describe one set of axes.
    const Eigen::Matrix3d R_e_n = ecef_T_nav_.rotation().matrix();
    const Eigen::Vector3d vel_ecef = R_e_n * Eigen::Vector3d(vel_ant_nav);
    msg->vel_ecef.x = vel_ecef.x();
    msg->vel_ecef.y = vel_ecef.y();
    msg->vel_ecef.z = vel_ecef.z();
    const Eigen::Matrix3d vel_cov_ecef =
        R_e_n * vel_cov_ant_nav * R_e_n.transpose();
    double qv_ecef[9];
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        msg->vel_cov_ecef[3 * r + c] = vel_cov_ecef(r, c);
        qv_ecef[3 * r + c] = vel_cov_ecef(r, c);
      }
    }
    const double v_ecef[3] = {vel_ecef.x(), vel_ecef.y(), vel_ecef.z()};
    double v_enu[3];
    ecef2enu(llh, v_ecef, v_enu);
    msg->vel_enu.x = v_enu[0];
    msg->vel_enu.y = v_enu[1];
    msg->vel_enu.z = v_enu[2];
    double qv_enu[9];
    gnss_utils::rotateCovariance(qv_ecef, enu.cur_lat, enu.cur_lon, qv_enu);
    for (int i = 0; i < 9; ++i) msg->vel_enu_cov[i] = qv_enu[i];

    sol_pub_->publish(std::move(msg));
  }

  // Full navigation state as nav_msgs/Odometry (body pose + attitude + velocity),
  // same convention as the GNSS/IMU EKF example: frame "odom" (local ENU nav),
  // child "imu_link" (body); twist is expressed in the body frame (REP-105).
  //
  // `vel_cov_enu` is ENU here TOO - this function rotates it into the body frame
  // itself. Both publishers therefore take ENU, and a caller that has a body- or
  // ECEF-frame covariance must convert once, on its own side.
  void publishOdometry(const rclcpp::Time& stamp,
                       const gtsam::Pose3& pose, const Eigen::MatrixXd& pose_cov,
                       const gtsam::Vector3& vel_enu,
                       const Eigen::Matrix3d& vel_cov_enu) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "imu_link";

    const gtsam::Point3 p = pose.translation();
    odom.pose.pose.position.x = p.x();
    odom.pose.pose.position.y = p.y();
    odom.pose.pose.position.z = p.z();
    const gtsam::Quaternion q = pose.rotation().toQuaternion();
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();

    // Reorder GTSAM Pose3 tangent covariance [rot(0-2), trans(3-5)] into the
    // Odometry [pos(0-2), att(3-5)] layout. The translation tangent is a
    // body-frame perturbation, so rotate its blocks into the nav (ENU) frame.
    const Eigen::Matrix3d R = pose.rotation().matrix();
    const Eigen::Matrix3d Prr = pose_cov.topLeftCorner<3, 3>();      // rot-rot
    const Eigen::Matrix3d Ptt = pose_cov.bottomRightCorner<3, 3>();  // trans-trans
    const Eigen::Matrix3d Ptr = pose_cov.bottomLeftCorner<3, 3>();   // trans-rot
    const Eigen::Matrix3d Cpp = R * Ptt * R.transpose();  // pos-pos (nav)
    const Eigen::Matrix3d Cpa = R * Ptr;                  // pos-att
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        odom.pose.covariance[r * 6 + c] = Cpp(r, c);              // pos-pos
        odom.pose.covariance[(r + 3) * 6 + (c + 3)] = Prr(r, c);  // att-att
        odom.pose.covariance[r * 6 + (c + 3)] = Cpa(r, c);        // pos-att
        odom.pose.covariance[(r + 3) * 6 + c] = Cpa(c, r);        // att-pos
      }
    }

    // Twist in the body/child frame (REP-105).
    const Eigen::Vector3d vel_body = R.transpose() * Eigen::Vector3d(vel_enu);
    odom.twist.twist.linear.x = vel_body.x();
    odom.twist.twist.linear.y = vel_body.y();
    odom.twist.twist.linear.z = vel_body.z();
    const Eigen::Matrix3d vel_cov_body = R.transpose() * vel_cov_enu * R;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        odom.twist.covariance[r * 6 + c] = vel_cov_body(r, c);
      }
    }
    // Angular velocity (body frame): the bias-corrected latest gyro, with the
    // gyro noise density as its covariance. Without this the twist.angular reads
    // as an authoritative zero with zero covariance, which a consumer would trust.
    const Eigen::Vector3d omega_body = omegaBody();
    odom.twist.twist.angular.x = omega_body.x();
    odom.twist.twist.angular.y = omega_body.y();
    odom.twist.twist.angular.z = omega_body.z();
    const double gyr_var =
        last_imu_valid_ ? imu_sigma_gyr_ * imu_sigma_gyr_ : 1.0e6;  // 1e6=unknown
    for (int i = 0; i < 3; ++i) odom.twist.covariance[(i + 3) * 6 + (i + 3)] = gyr_var;

    odom_pub_->publish(odom);
  }

  // Bias-corrected body angular rate. ONE definition, shared by the Doppler
  // lever-arm removal, the motion pseudo-measurements, the published antenna
  // velocity and the Odometry twist.angular - so those four cannot disagree
  // about how fast the body is turning. Solver-owned state; no lock needed.
  Eigen::Vector3d omegaBody() const {
    return last_imu_valid_
               ? Eigen::Vector3d(last_imu_.gyr - prev_bias_.gyroscope())
               : Eigen::Vector3d::Zero();
  }

  // Lock order where both are needed: mtx_ then intake_mtx_. Intake takes
  // intake_mtx_ ONLY, so a solve holding mtx_ never blocks it.
  std::mutex mtx_;         // estimator state (smoother_, prev_*, pending_epochs_)
  // preprocessor_ queues, and the read-modify-write of the atomic
  // latest_rover_ctow_ / last_rover_arrival_imu_t_ pair against other intake.
  std::mutex intake_mtx_;
  std::mutex imu_mtx_;
  // Debug-only base-arrival trace (see onBaseObs); null unless
  // debug.ar_dump_dir is set.
  std::FILE* base_trace_{nullptr};
  // Slot ownership for BOTH publishers - the one place that decides whether an
  // epoch produces output. Also the dead-reckoning high-water mark (see
  // publishPredictedGaps).
  gnss_fgo::EpochSlotArbiter slots_;
  double max_coast_s_{30.0};    // gap.max_coast_s

  std::uint64_t n_coast_truncated_{0};
  rclcpp::CallbackGroup::SharedPtr solver_group_;
  rclcpp::CallbackGroup::SharedPtr intake_group_;
  rclcpp::CallbackGroup::SharedPtr imu_group_;
  std::unique_ptr<gnss_utils::GnssPreprocessor> preprocessor_;
  gnss_utils::GnssEpochAligner epoch_aligner_;
  std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_;
  double smoother_lag_{1.0e9};
  double solve_ms_max_{0.0};  // worst per-epoch solve time (real-time headroom)
  gnss_fgo::AdapterConfig adapter_cfg_;
  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> pim_params_;

  std::deque<ImuSample> imu_buffer_;
  double imu_max_wait_s_{0.2};
  int imu_queue_depth_{2000};
  std::chrono::steady_clock::time_point last_imu_arrival_wall_{
      std::chrono::steady_clock::now()};
  std::atomic<std::uint64_t> n_imu_received_{0};
  std::atomic<std::uint64_t> n_imu_message_lost_{0};
  std::uint64_t n_imu_fallback_{0};
  double max_imu_gap_s_{0.0};
  ImuSample last_imu_{};        // most recent integrated IMU sample (for ZOH)
  bool last_imu_valid_{false};

  bool initialized_{false};
  gnss_fgo::VelocityAttitudeConfig vatt_cfg_;
  bool yaw_aligned_{false};   // heading has agreed with the course at least once
  double attitude_align_reanchor_rad_{20.0 * D2R};
  gnss_fgo::YawAlignConfig yaw_align_cfg_;
  gnss_fgo::YawAlignmentVote yaw_align_vote_;  // consecutive-agreement accumulator
  gnss_fgo::NhcConfig nhc_cfg_;
  gnss_fgo::ZuptConfig zupt_cfg_;
  double doppler_max_res_m_{2.0};
  double doppler_corr_time_s_{2.0};  // Doppler error correlation time [s]
  double imu_sigma_gyr_{0.01};  // cached for the Odometry angular-velocity cov
  int epoch_index_{0};
  std::uint64_t next_amb_id_{0};  // globally-unique ambiguity keys
  // Carried-across-epochs ambiguity keys (one SD ambiguity per (sat,band)).
  std::unique_ptr<gnss_fgo::PersistentAmbiguities> amb_mgr_;
  double amb_max_outage_{5.0};
  gnss_fgo::HeldDdMap held_dd_;
  gnss_fgo::HoldCandidateMap hold_candidates_;  // held DD constraints (by sat pair)
  gnss_fgo::CodeGrowthMonitor code_growth_;  // escape gate: bad AR increment
  gnss_fgo::CodeGrowthMonitor code_rms_;     // escape gate: bad float position
  int min_fix_to_hold_{10};  // consecutive fixes before holding (RTKLIB minfix)
  int nfix_{0};              // current consecutive-fix count
  // Continuous GPST of the last epoch whose DDs reached the graph (-1 = none).
  // A gap longer than kReanchorGapS forces the re-anchor path.
  double last_dd_ctow_{-1.0};
  // Observation messages the middleware dropped before this node saw them, plus
  // the epoch time of the most recent loss. The latter separates a TRANSPORT
  // gap from a real GNSS outage in the starvation trigger above.
  std::atomic<std::uint64_t> n_obs_message_lost_{0};
  // These cross the intake/solver boundary in OPPOSITE directions: the
  // message-lost event reads last_epoch_ctow_, which processEpoch writes, and
  // last_obs_loss_ctow_ travels back to gate the re-anchor. Atomic rather than
  // mutex-guarded because the solver reads them while holding mtx_, and taking
  // intake_mtx_ there would let a long solve block intake.
  std::atomic<double> last_obs_loss_ctow_{-1.0};
  std::atomic<double> last_epoch_ctow_{-1.0};
  // Epochs the MATCHER discarded (queue overflow, late arrival, ...). These
  // never reach processEpoch, so they are the other way an epoch can go
  // missing without the middleware reporting anything lost.
  std::uint64_t n_epochs_dropped_{0};
  double last_epoch_drop_ctow_{-1.0};
  // Rover observation messages that actually reached the callback. Compared
  // against the epochs that reached processEpoch, this separates "the message
  // never arrived" from "the preprocessor did not emit an epoch for it" -
  // which are different bugs with different fixes.
  // Written on intake_group_, read on solver_group_ by the health log.
  std::atomic<std::uint64_t> n_rover_msgs_received_{0};
  std::uint64_t n_epochs_emitted_{0};  // PreprocessedEpochs out of drainEpochs
  // One gap-triggered re-anchor per outage; cleared when DDs reach the graph.
  gnss_fgo::GapReanchorLatch gap_latch_;
  // Re-anchors, and how many of them had a genuinely independent target. When
  // the second lags the first the escape is degenerating into the old
  // anchor-to-yourself behaviour and its verdicts should not be trusted.
  std::uint64_t n_reanchor_{0};
  std::uint64_t n_reanchor_independent_{0};
  double prev_t_ros_{0.0};
  gtsam::NavState prev_state_;
  gtsam::imuBias::ConstantBias prev_bias_;
  gtsam::Pose3 ecef_T_nav_;
  bool nav_frame_valid_{false};  // anchored once; kept across reinitializations
  gtsam::Point3 lever_arm_{0.0, 0.0, 0.0};

  Eigen::Vector3d last_antenna_ecef_{Eigen::Vector3d::Zero()};
  bool last_antenna_ecef_valid_{false};

  gnss_fgo::ArOptions ar_opt_;
  bool ar_enabled_{true};  // ambiguity.resolution; false = FLOAT-only estimator
  std::unique_ptr<gnss_fgo::ArDebugDumper> ar_dump_;
  std::ofstream imu_diag_;
  // Availability accounting: every epoch handed to processEpoch must end in a
  // published solution (the inertial chain can always produce one), so a gap
  // between these two is a bug, not a data limitation.
  std::uint64_t n_epochs_in_{0};
  std::uint64_t n_published_{0};
  rcl_clock_type_t epoch_clock_type_{RCL_ROS_TIME};
  // GNSS-gap dead reckoning (publishPredictedGaps): rover epoch grid estimate
  // and the last posterior context needed to synthesize/stamp predictions.
  double gnss_interval_s_{0.2};
  bool gnss_interval_pinned_{false};
  std::vector<double> interval_samples_;
  // Why the gap publisher did or did not synthesize. Diagnostics only: none of
  // these influence the estimate or what is published.
  std::uint64_t gap_calls_{0}, gap_exit_disabled_{0}, gap_exit_not_ready_{0},
      gap_exit_pending_{0}, gap_exit_no_interval_{0}, gap_exit_stream_alive_{0},
      gap_exit_matcher_holds_{0}, gap_exit_backlog_{0}, gap_reached_loop_{0},
      gap_break_no_imu_{0};
  double gap_backlog_max_s_{0.0};  // worst received-vs-processed gap [s]
  double gap_silence_max_s_{0.0};  // longest rover silence acted on [s]
  double gap_silence_seen_max_s_{0.0};  // longest rover silence ever OBSERVED
  // One worker tick: epochs swallowed and how long it held mtx_.
  std::uint64_t batch_epochs_max_{0}, batch_hold_max_epochs_{0};
  double batch_hold_max_s_{0.0};
  std::size_t batch_processed_{0};  // epochs handled by the current tick
  // Newest RECEIVED rover obs [GPST cont. s]. Written on intake_group_ under
  // intake_mtx_ (which serialises the read-modify-write), read on solver_group_
  // without it. Gates gap slot ownership, so a stale read lets the gap
  // publisher claim a slot the real epoch is about to fill.
  std::atomic<double> latest_rover_ctow_{-1.0};
  // IMU watermark when the newest rover observation arrived; the anchor the
  // outage verdict measures silence from (see gapSlotIsUnobserved).
  double last_rover_arrival_imu_t_{
      -std::numeric_limits<double>::infinity()};
  int n_pred_since_epoch_{0};
  std::uint64_t n_predicted_pub_{0};
  std::uint32_t last_week_{0};
  double last_tow_{0.0};
  Eigen::Vector3d last_base_ecef_{Eigen::Vector3d::Zero()};
  Eigen::MatrixXd last_pose_cov_{Eigen::MatrixXd::Identity(6, 6) * 100.0};
  // Last posterior velocity covariance, propagated into the gap-prediction
  // position uncertainty (the velocity-over-horizon term preintMeasCov omits).
  Eigen::Matrix3d last_vel_cov_{Eigen::Matrix3d::Identity() * 1.0};
  // Full joint posterior [Pose3(6), Velocity(3), Bias(6)] of the PREVIOUS
  // epoch, in graph charts. The pre-fit gate propagates this through the
  // preintegration Jacobians; the two blocks above cannot substitute for it
  // because the terms that matter most over one interval - pose-velocity and
  // velocity-bias correlation - live only in the off-diagonal blocks.
  Eigen::MatrixXd last_state_cov15_{Eigen::MatrixXd::Zero(15, 15)};
  bool last_state_cov15_ok_{false};
  double init_imu_duration_{1.0};
  // GNSS epochs waiting for IMU coverage (see drainPendingEpochs).
  std::deque<PendingEpoch> pending_epochs_;
  rclcpp::TimerBase::SharedPtr pending_timer_;

  double fixed_origin_ecef_[3]{0.0, 0.0, 0.0};
  bool fixed_origin_valid_{false};

  rclcpp::Subscription<grs::GnssObservations>::SharedPtr rover_obs_sub_;
  rclcpp::Subscription<grs::GnssObservations>::SharedPtr base_obs_sub_;
  rclcpp::Subscription<grs::GnssEphemerides>::SharedPtr nav_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<grs::GnssSolution>::SharedPtr sol_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GnssImuFgoNode>();
  // One thread per mutually-exclusive group: observation intake, IMU intake,
  // and the solver. Fewer would let a long solve hold off an intake callback,
  // which is exactly the coupling the group split exists to remove.
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
