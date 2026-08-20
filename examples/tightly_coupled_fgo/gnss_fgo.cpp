// SPDX-License-Identifier: MIT
//
// Tightly-coupled GNSS factor-graph optimization example.
//
// Pipeline: the standardized topics (/gnss/observation, /base/gnss/observation,
// /gnss/ephemeris) are pushed into gnss_utils::GnssPreprocessor, which does all
// GNSS-domain work (epoch matching, satellite positions at transmission time,
// masks, reference-satellite selection, DD pairing, cycles->meters, and the
// undifferenced Doppler rover velocity). PreprocessedEpochs feed GTSAM's
// official GNSS factors (DoubleDifferencePseudorange/CarrierPhaseFactor, added
// to GTSAM in June 2026), optimized incrementally with ISAM2.
//
// Structure: the rover position is tied across epochs by a Doppler-derived
// displacement (a BetweenFactor<Point3>). This plays the role RTKLIB's EKF
// dynamics do - keeping position continuous rather than re-estimating it per
// epoch - but is NOT the same model: RTKLIB propagates a position/velocity(/
// acceleration) state with its full transition covariance, whereas this factor
// only constrains the position increment by the measured Doppler velocity (no
// velocity/acceleration state, no state-transition covariance propagation). One
// SD carrier ambiguity per (sat,band) is carried across epochs and re-keyed on
// cycle slip / outage, so the carrier phase accumulates (fix-and-hold; see
// gnss_fgo::PersistentAmbiguities).
//   State per epoch k:  X(k) rover ECEF position (gtsam::Point3)
//   Per (sat, band):    N(k) carrier ambiguity in cycles.
//   Inter-epoch:        BetweenFactor<Point3> = Doppler-derived displacement.
// Ambiguity resolution mirrors RTKLIB resamb_LAMBDA: the integers are searched
// on the stage-2 graph posterior after grouped code/carrier DD factors with the
// full shared-reference covariance have been inserted. The same posterior mean,
// Qa and state/ambiguity cross covariance feed RTKLIB's lambda(), full-state
// integer conditioning, validation and partial AR
// (resolveAmbiguitiesPosterior). Accepted integers are held as gauge-free
// BetweenFactor constraints (applyHolds).
//
// Real-time: the estimator is a gtsam::IncrementalFixedLagSmoother (ISAM2 plus
// marginalization of variables older than graph.lag_s), so the per-epoch cost
// is BOUNDED - marginalizing an old position folds its ambiguity-constraining
// information into a prior on the retained ambiguities, so the carrier keeps
// accumulating and the current-epoch AR is unchanged. graph.lag_s <= 0 selects
// full-history ISAM2 (unbounded, offline / max accuracy). The linear solver is
// QR - see makeIsamParams.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include "ar_debug_dump.hpp"
#include "factor_adapters.hpp"
#include "gnss_ros_standardization/gnss_preprocessor.hpp"
#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"

extern "C" {
#include "rtklib.h"
}

namespace grs = gnss_ros_standardization::msg;
using gtsam::symbol_shorthand::X;

namespace {
constexpr int kPositionLogThrottleMs = 1000;
constexpr int kMinDdForAr = 3;
constexpr double kEpochBudgetMs = 100.0;  // 10 Hz real-time budget per epoch
// Fixed motion / prior constants.
// GNSS gap (s) with no DD reaching the graph that forces a re-anchor.
constexpr double kReanchorGapS = 5.0;
// Sigma of the position prior used when RE-ANCHORING on the code a-priori.
//
// Deliberately NOT kPositionPriorStdM: that is a COLD-START prior, where 100 m
// only has to keep the first epoch finite. A re-anchor is a different claim -
// "single-point positioning says the antenna is here" - and the pre-fit test in
// factor_adapters.hpp models that same quantity at 10 m, and the two must
// agree: in a canyon with ~9 DDs the geometry barely observes the vertical, so
// this prior is the only thing holding it. At 100 m it loses to the DD factors
// and the re-anchor cannot pull the state back at all.
constexpr double kReanchorPosStdM = 10.0;
constexpr double kPositionPriorStdM = 100.0;  // first epoch / after a motion gap
constexpr double kMotionVelSigmaMps = 0.1;    // floor on the Doppler vel sigma
constexpr double kMotionMaxDtS = 5.0;         // beyond this gap, re-anchor
}  // namespace

class GnssFgoNode : public rclcpp::Node {
 public:
  GnssFgoNode() : Node("gnss_fgo") {
    declareParameters();

    preprocessor_ = std::make_unique<gnss_utils::GnssPreprocessor>(makePreprocessorConfig());
    adapter_cfg_ = makeAdapterConfig();
    amb_mgr_ = std::make_unique<gnss_fgo::PersistentAmbiguities>(next_amb_id_);
    amb_mgr_->setHoldRefresh(get_parameter("ambiguity.hold_refresh_s").as_double());
    amb_max_outage_ = get_parameter("ambiguity.max_outage_s").as_double();
    smoother_lag_ = smootherLag(get_parameter("graph.lag_s").as_double());

    smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(
        smoother_lag_, makeIsamParams());

    ar_dump_ = std::make_unique<gnss_fgo::ArDebugDumper>(
        get_parameter("debug.ar_dump_dir").as_string(), "gnss_fgo");
    if (ar_dump_->enabled()) {
      RCLCPP_INFO(get_logger(), "AR debug dump -> %s",
                  get_parameter("debug.ar_dump_dir").as_string().c_str());
    }

    cacheFixedOriginParam();

    // A LOST observation is silent unless you ask for it. Without these
    // callbacks an epoch that never reaches onRoverObs is indistinguishable
    // from an epoch that carried no usable satellites, and both nodes then
    // report a per-received availability that looks perfect while epochs are
    // going missing in the middleware.
    rclcpp::SubscriptionOptions obs_options;
    obs_options.event_callbacks.message_lost_callback =
        [this](rclcpp::QOSMessageLostInfo& info) {
          n_obs_message_lost_ += info.total_count_change;
          last_obs_loss_ctow_ = last_epoch_ctow_;
        };

    // Depth 512, not 50. At a 5 Hz rover a 50-deep queue is 10 s of slack,
    // and the measured solve time spikes to 0.8 s; a burst during replay (or a
    // busy CPU in the field) overruns it and the epochs are dropped by the
    // middleware, not by any decision this node made.
    rover_obs_sub_ = create_subscription<grs::GnssObservations>(
        get_parameter("topics.rover_observation").as_string(), rclcpp::QoS(512),
        std::bind(&GnssFgoNode::onRoverObs, this, std::placeholders::_1),
        obs_options);
    base_obs_sub_ = create_subscription<grs::GnssObservations>(
        get_parameter("topics.base_observation").as_string(), rclcpp::QoS(512),
        std::bind(&GnssFgoNode::onBaseObs, this, std::placeholders::_1),
        obs_options);
    nav_sub_ = create_subscription<grs::GnssEphemerides>(
        get_parameter("topics.ephemeris").as_string(),
        // depth 256 (not 1): ephemeris arrives as many per-satellite messages;
        // a depth-1 transient_local latch keeps only the last satellite, so a
        // subscriber matching after the initial burst would starve for DDs.
        rclcpp::QoS(256).transient_local(),
        std::bind(&GnssFgoNode::onNav, this, std::placeholders::_1),
        obs_options);
    sol_pub_ = create_publisher<grs::GnssSolution>(
        get_parameter("topics.solution").as_string(), 10);

    if (ar_enabled_) {
      RCLCPP_INFO(get_logger(),
                  "Tightly-coupled GNSS FGO started (fix-and-hold, ratio %.1f, "
                  "lag %.1fs).",
                  ar_opt_.ratio_threshold, smoother_lag_);
    } else {
      RCLCPP_INFO(get_logger(),
                  "Tightly-coupled GNSS FGO started (FLOAT ONLY - "
                  "ambiguity.resolution is false, lag %.1fs).",
                  smoother_lag_);
    }
  }

  ~GnssFgoNode() override {
    // Report delivery, not just work done. A node that never sees an epoch
    // cannot solve it, and a per-received availability figure hides that
    // completely - which is how 98 lost epochs per run went unnoticed.
    const auto lost = n_obs_message_lost_.load();
    if (lost > 0) {
      RCLCPP_WARN(get_logger(),
                  "%llu observation message(s) were LOST in transport before "
                  "reaching this node. Epochs that never arrive cannot be "
                  "solved; raise the subscription depth or reduce the load.",
                  static_cast<unsigned long long>(lost));
    } else {
      RCLCPP_INFO(get_logger(), "No observation messages lost in transport.");
    }
    if (n_reanchor_ > 0) {
      RCLCPP_INFO(get_logger(),
                  "Re-anchors: %llu, of which %llu had an estimator-independent "
                  "target (code-DD WLS / SPP).",
                  static_cast<unsigned long long>(n_reanchor_),
                  static_cast<unsigned long long>(n_reanchor_independent_));
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

  // ISAM2 with QR factorization, on BOTH nodes.
  //
  // CHOLESKY is ~4.8x faster and NOT usable here: it squares the information
  // matrix, and a canyon epoch with a median of 9 DDs is conditioned badly
  // enough that the squared system goes singular and resets the whole graph.
  // Open sky cannot show this, so do not revisit the choice without testing the
  // WEAKEST geometry in the set. gnss_imu_fgo needs QR independently - its
  // NavState graph mixes held integers (5.7 mm) with metre-scale priors.
  // QR costs roughly p95 49 -> 69 ms here, still inside the budget.
  static gtsam::ISAM2Params makeIsamParams() {
    gtsam::ISAM2Params p;
    p.relinearizeThreshold = 0.01;
    // Relinearize at most every 10th update. The DD factors are only mildly
    // nonlinear (short-baseline geometry), so this keeps the node real-time on
    // the retained graph with no measurable accuracy cost.
    p.relinearizeSkip = 10;
    p.factorization = gtsam::ISAM2Params::QR;
    return p;
  }

  // Map the configured window to the smoother lag: <= 0 means full history
  // (unbounded), realised as a lag longer than any session.
  static double smootherLag(double lag_s) {
    return lag_s > 0.0 ? lag_s : 1.0e9;
  }

 private:
  void declareParameters() {
    declare_parameter("topics.rover_observation", std::string("/gnss/observation"));
    declare_parameter("topics.base_observation", std::string("/base/gnss/observation"));
    declare_parameter("topics.ephemeris", std::string("/gnss/ephemeris"));
    declare_parameter("topics.solution", std::string("/gnss/fgo/solution"));

    // Base station antenna position. REQUIRED for DD: an error here shifts
    // the whole estimated trajectory by roughly the same amount.
    declare_parameter("base_position.postype", std::string("llh"));
    declare_parameter("base_position.pos", std::vector<double>{0.0, 0.0, 0.0});

    declare_parameter("fixed_origin.postype", std::string("llh"));
    declare_parameter("fixed_origin.pos", std::vector<double>{0.0, 0.0, 0.0});

    declare_parameter("masks.elevation_deg", 15.0);
    declare_parameter("masks.snr_dbhz", 0.0);  // flat SNR mask [dBHz]; 0 disables

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

    // Rover-base cycle-slip detection. Always-on checks (LLI cycle-slip bit,
    // half-cycle-bit transition, observed-code change, SD code-minus-carrier
    // gross error, Doppler-phase) protect the carried ambiguities; only the two
    // data-dependent thresholds are exposed.
    declare_parameter("cycle_slip.gf_threshold_m", 0.05);  // SD geometry-free
    declare_parameter("cycle_slip.max_gap_s", 2.0);        // carrier outage [s]

    // Undifferenced (zenith) measurement sigmas; DD noise is derived from these.
    declare_parameter("noise.pr_sigma_m", 0.5);
    declare_parameter("noise.cp_sigma_m", 0.005);
    declare_parameter("noise.pr_innov_gate_m", 0.0);
    // Reject the Doppler velocity used by the motion factor when its post-fit LS
    // residual RMS exceeds this (m/s); 0 disables the gate.
    declare_parameter("noise.doppler_max_res_m", 2.0);
    // Per-satellite fault detection inside the Doppler solve (studentized
    // residual threshold; 0 disables). The residual-RMS gate above can only
    // reject the whole epoch, so a single NLOS range-rate either survives or
    // costs the entire velocity observation.
    declare_parameter("noise.doppler_max_nsigma", 4.0);
    // A-priori zenith range-rate sigma [m/s]; the per-satellite variance is
    // doppler_sigma_mps^2 / sin^2(el). Sets the absolute scale of the reported
    // velocity covariance AND the reference the w-test judges against.
    declare_parameter("noise.doppler_sigma_mps", 0.1);

    // Integer ambiguity resolution on the graph posterior (RTKLIB resamb_LAMBDA
    // equivalent): the joint [position, carried ambiguity] prior from ISAM2 plus
    // this epoch's DDs under the correctly-correlated DD covariance, then LAMBDA
    // + ratio test + sigma-normalized FDE + partial AR. Accepted integers are
    // held as gauge-free BetweenFactor constraints (fix-and-hold). All safety
    // parameters (FDE sigma, partial-AR budget, hold sigma, init sigma) are
    // fixed constants (see README); only these decision knobs are exposed.
    // Resolve integer ambiguities at all. false makes this a pure FLOAT
    // estimator: LAMBDA is never run, no fix is ever published, and no integer
    // is ever held. Provided so the float solution can be evaluated on its own
    // - it is the half of the estimator that carries the position when AR
    // fails, and its quality is otherwise masked by the fixed epochs.
    declare_parameter("ambiguity.resolution", true);
    // How many DDs the pre-fit innovation test may exclude before it judges the
    // whole epoch dirty (see ArOptions::fde_max_exclude).
    declare_parameter("ambiguity.fde_max_exclude", 2);
    declare_parameter("ambiguity.ratio_threshold", 3.0);
    declare_parameter("ambiguity.min_fix", 4);
    declare_parameter("ambiguity.min_lock", 5);
    // Consecutive FIX epochs required before holding (RTKLIB minfix), so a
    // single false fix cannot corrupt the graph.
    declare_parameter("ambiguity.min_fix_to_hold", 10);
    declare_parameter("ambiguity.partial_max_drop", 5);
    // Skip AR when the float position variance exceeds this [m^2] (RTKLIB
    // pos2-arthres1): an unconverged float biases the integer search. 0 = off.
    declare_parameter("ambiguity.max_pos_var_m2", 0.25);
    // Re-key a carried ambiguity after an outage this long [s].
    declare_parameter("ambiguity.max_outage_s", 5.0);
    // Age-based re-acquisition [s] (0 disables): bounds how long a held integer
    // may persist without being re-verified from a fresh loose prior. Without
    // it a subtly-wrong hold biases the graph posterior, which biases the AR
    // prior, which lets the ratio test keep confirming the same wrong integer -
    // measured to raise the false-fix rate materially.
    declare_parameter("ambiguity.hold_refresh_s", 30.0);
    // Cap scope: "all" (default; doubles as fading memory on accumulated
    // model optimism - measured better on urban full courses) or "held"
    // (preserve float arcs until slip/outage; arcs then live minutes).

    // Directory for the ambiguity-resolution dump (empty = off): the float
    // solution and its covariance as handed to LAMBDA, for offline analysis.
    declare_parameter("debug.ar_dump_dir", std::string(""));

    // Inter-epoch motion model: a BetweenFactor<Point3> ties X(k) to X(k-1) by
    // the Doppler-derived displacement (mean = 0.5*(v_{k-1}+v_k)*dt), like
    // RTKLIB's EKF dynamics. The factor sigma is
    // sqrt((vel_sigma*dt)^2 + (0.5*accel_sigma*dt^2)^2); only the unmodelled
    // acceleration (process noise) is exposed.
    declare_parameter("motion.accel_sigma_mps2", 3.0);

    // Fixed-lag window [s]: variables older than this are marginalized, so the
    // per-epoch solve cost is bounded (real-time). A position is only
    // marginalized once its ambiguities are firmly held, so the marginal prior
    // is consistent (too short a window marginalizes not-yet-converged states,
    // over-confidently, which admits false fixes); ~60 s is the floor for this
    // fix-and-hold settling. <= 0 selects full-history ISAM2 (unbounded;
    // offline / max accuracy).
    declare_parameter("graph.lag_s", 60.0);

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
    min_fix_to_hold_ =
        static_cast<int>(get_parameter("ambiguity.min_fix_to_hold").as_int());
    motion_accel_sigma_ = get_parameter("motion.accel_sigma_mps2").as_double();
    doppler_max_res_m_ = get_parameter("noise.doppler_max_res_m").as_double();
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
    // GLONASS carrier DD is off: its FDMA inter-frequency biases do not cancel
    // in DD across heterogeneous receivers (fixed; see README).
    cfg.glonass_carrier_dd = false;
    cfg.cross_code_carrier =
        get_parameter("frequencies.cross_code_pairing").as_bool();
    cfg.bands.clear();
    if (get_parameter("frequencies.enable_l1").as_bool()) cfg.bands.push_back(0);
    if (get_parameter("frequencies.enable_l2").as_bool()) cfg.bands.push_back(1);
    if (get_parameter("frequencies.enable_l5").as_bool()) cfg.bands.push_back(2);
    if (cfg.bands.empty()) cfg.bands.push_back(0);
    cfg.max_age_s = get_parameter("max_age_s").as_double();
    const std::string iono =
        get_parameter("measurement.iono_model").as_string();
    const std::string trop =
        get_parameter("measurement.trop_model").as_string();
    cfg.ionoopt = (iono == "off") ? IONOOPT_OFF : IONOOPT_BRDC;
    cfg.tropopt = (trop == "off") ? TROPOPT_OFF : TROPOPT_SAAS;
    // Cycle-slip checks: all on; only the two data-dependent thresholds are
    // exposed (the rest are fixed constants, see README).
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
      RCLCPP_INFO(get_logger(), "Base position: ECEF %.3f %.3f %.3f",
                  ecef[0], ecef[1], ecef[2]);
    } else {
      RCLCPP_ERROR(get_logger(),
                   "base_position is not set - no DD pairs will be formed.");
    }
    return cfg;
  }

  // Only the two undifferenced sigmas are exposed; the rest of AdapterConfig
  // (elevation weighting, Huber, init/hold sigmas) are fixed constants defined
  // by the struct's defaults (see factor_adapters.hpp / README).
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

  void onNav(const grs::GnssEphemerides::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    preprocessor_->pushEphemerides(*msg);
  }

  void onRoverObs(const grs::GnssObservations::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    preprocessor_->pushRoverObs(msg);
    processEpochs();
  }

  void onBaseObs(const grs::GnssObservations::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    preprocessor_->pushBaseObs(msg);
    processEpochs();
  }

  void processEpochs() {
    const auto epochs = preprocessor_->drainEpochs(
        last_estimate_valid_ ? &last_estimate_ : nullptr);
    for (const auto& ep : epochs) processEpoch(ep);
    // takeDropped() is the LEGACY view and only describes one reason
    // (no base within the window). The matcher reason-tags every drop, and the
    // reasons it does not cover are counted and then thrown away, so epochs
    // vanish here with nothing logged and read downstream as a GNSS outage,
    // re-keying every carried arc. Consume the full event list instead.
    for (const auto& e : preprocessor_->takeDropEvents()) {
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
    preprocessor_->takeDropped();  // keep the legacy queue from growing
  }

  void processEpoch(const gnss_utils::PreprocessedEpoch& ep) {
    if (ep.dd.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Epoch tow %.3f has no DD pairs (check base stream / base_position).",
          ep.tow);
      return;
    }
    const auto solve_t0 = std::chrono::steady_clock::now();

    const gtsam::Key xk = X(epoch_index_);
    const gtsam::Point3 apriori(ep.rover_ecef_apriori);

    // Motion prediction: connect this epoch's position to the previous one by
    // the Doppler-derived displacement (mean = 0.5*(v_{k-1}+v_k)*dt) - the
    // position-continuity role RTKLIB's EKF dynamics play, though only a
    // displacement constraint, not a full pos/vel/acc state transition.
    // Continuous GPST seconds (week rollover safe) for all time differencing;
    // the message time_tow stays the raw tow.
    const double t_gnss = ep.week * 604800.0 + ep.tow;
    const double dt = last_float_valid_ ? (t_gnss - prev_tow_) : 0.0;
    const bool continuous = last_float_valid_ && dt > 1e-3 && dt <= kMotionMaxDtS;

    // Reject a grossly mismodeled / NLOS Doppler set (see noise.doppler_max_res_m).
    const bool doppler_ok =
        ep.rover_vel_valid &&
        (doppler_max_res_m_ <= 0.0 || ep.rover_vel_res <= doppler_max_res_m_);
    Eigen::Vector3d delta = Eigen::Vector3d::Zero();
    Eigen::Matrix3d motion_cov =
        Eigen::Matrix3d::Identity() * (kPositionPriorStdM * kPositionPriorStdM);
    Eigen::Vector3d x_init = apriori;
    if (continuous) {
      const Eigen::Vector3d v_now =
          doppler_ok ? ep.rover_vel_ecef
          : (prev_vel_valid_ ? prev_vel_ : Eigen::Vector3d::Zero());
      const Eigen::Vector3d v_prev = prev_vel_valid_ ? prev_vel_ : v_now;
      delta = 0.5 * (v_prev + v_now) * dt;
      // Displacement covariance = (velocity covariance)*dt^2 + the unmodelled
      // acceleration over the interval. The velocity term is the Doppler
      // solve's real ECEF 3x3, anisotropic in the same direction the satellite
      // geometry is weak, so the motion factor does not claim to constrain the
      // vertical increment as well as the horizontal one.
      const double floor_var = kMotionVelSigmaMps * kMotionVelSigmaMps;
      Eigen::Matrix3d vel_cov = Eigen::Matrix3d::Identity() * floor_var;
      if (doppler_ok) {
        if (ep.rover_vel_cov.allFinite() &&
            ep.rover_vel_cov.diagonal().sum() > 0.0) {
          vel_cov = ep.rover_vel_cov;
          // Keep the historical floor as a floor on each axis, not a
          // replacement: it exists because the LS covariance says nothing about
          // multipath on the range rates.
          for (int i = 0; i < 3; ++i)
            vel_cov(i, i) = std::max(vel_cov(i, i), floor_var);
        } else {
          vel_cov = Eigen::Matrix3d::Identity() *
                    std::max(ep.rover_vel_var, floor_var);
        }
      }
      const double accel_var = std::pow(0.5 * motion_accel_sigma_ * dt * dt, 2);
      motion_cov = vel_cov * (dt * dt) +
                   Eigen::Matrix3d::Identity() * std::max(accel_var, 1e-6);
      x_init = prev_float_pos_ + delta;
    }

    // Two-stage ISAM2 update. Stage 1 adds ONLY the motion factor (or the
    // anchor prior), so the marginals afterwards are the PREDICTED prior -
    // free of this epoch's DD, hence usable as the AR prior without double-
    // counting. Stage 2 then adds the DD factors (the posterior float).
    gtsam::NonlinearFactorGraph motion_graph;
    gtsam::Values motion_values;
    motion_values.insert(xk, gtsam::Point3(x_init));
    if (continuous) {
      // Robust (Huber) motion model: Doppler velocity degrades at low speed
      // (range-rate noise / clock-drift artefacts dominate), so down-weight
      // increments that disagree with the rest of the graph instead of letting
      // a noisy low-speed Doppler pull the float trajectory.
      gtsam::SharedNoiseModel mnoise =
          gtsam::noiseModel::Gaussian::Covariance(motion_cov);
      mnoise = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(adapter_cfg_.huber_k),
          mnoise);
      motion_graph.add(gtsam::BetweenFactor<gtsam::Point3>(
          X(epoch_index_ - 1), xk, gtsam::Point3(delta), mnoise));
    } else {
      motion_graph.add(gtsam::PriorFactor<gtsam::Point3>(
          xk, apriori,
          gtsam::noiseModel::Isotropic::Sigma(3, kPositionPriorStdM)));
    }

    // Timestamps for the fixed-lag window: the new position at this epoch's
    // time, and a refresh of every live ambiguity key so a carried arc (and any
    // hold on it) is never marginalized out from under us while still tracked.
    gtsam::FixedLagSmoother::KeyTimestampMap ts_motion;
    ts_motion[xk] = t_gnss;
    for (const gtsam::Key k : amb_mgr_->liveKeys()) ts_motion[k] = t_gnss;
    try {
      smoother_->update(motion_graph, motion_values, ts_motion);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Motion-stage update failed (%s) - resetting.",
                  e.what());
      resetGraph();
      return;
    }

    // Retire outage-stale carried ambiguities AFTER the update (original order):
    // retiring BEFORE the window refresh marginalizes a still-held key one epoch
    // early and was measured to raise the false-fix rate on the IMU node; the
    // graph-bloat it saves is bounded by the fixed-lag window anyway.
    amb_mgr_->retireStale(t_gnss, amb_max_outage_);

    gtsam::NonlinearFactorGraph dd_graph;
    gtsam::Values dd_values;
    // Pre-fit FDE runs on the stage-1 motion prediction with the complete
    // shared-reference innovation covariance. Its filtered set is the only set
    // consumed by both the graph and AR.
    int n_gated = 0;
    gnss_utils::PreprocessedEpoch gnss_ep = ep;
    Eigen::Vector3d predicted_pos = x_init;
    Eigen::Matrix3d predicted_cov =
        Eigen::Matrix3d::Identity() * kPositionPriorStdM * kPositionPriorStdM;
    bool prediction_fault = false;
    try {
      predicted_pos = smoother_->calculateEstimate<gtsam::Point3>(xk);
      predicted_cov = smoother_->marginalCovariance(xk);
      gnss_ep = gnss_fgo::filterDdPreFitInnovation(
          ep, predicted_pos, predicted_cov, adapter_cfg_, ar_opt_, &n_gated,
          &prediction_fault);
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Pre-fit innovation marginal unavailable - retaining DD set (%s).",
          e.what());
    }

    // Re-anchor when this epoch's own DDs agree with each other at an
    // independent code position but not with the graph's predicted state.
    // Fix-and-hold can walk the estimate somewhere the observations do not
    // support - a subtly wrong held integer biases the float, which biases the
    // next AR prior - and the carried arcs then keep that error self-consistent.
    //
    // The trigger must NOT require surviving DDs: when the pre-fit filter judges
    // the whole epoch dirty it empties the DD set AND leaves prediction_fault
    // false, so neither trigger would fire and no measurement could ever correct
    // the state. The gap trigger below covers exactly that case, and anchoring
    // needs only `ep`'s code a-priori, which is computed upstream of gating.
    const double ctow = ep.week * 604800.0 + ep.tow;
    last_epoch_ctow_ = ctow;
    // A gap only counts as starvation if the observations really stopped. A
    // DELIVERY gap - the middleware losing an epoch, or the matcher discarding
    // one - means the sky was fine and the carrier arcs are still valid, so the
    // trigger is suppressed for one gap-length after any such loss.
    const double last_delivery_gap_ctow =
        std::max(last_obs_loss_ctow_, last_epoch_drop_ctow_);
    const bool recent_obs_loss =
        last_delivery_gap_ctow > 0.0 &&
        (ctow - last_delivery_gap_ctow) <= kReanchorGapS;
    // One outage = one re-anchor. Without the latch the gap condition stays
    // true for EVERY epoch of the outage - last_dd_ctow_ only advances when a
    // DD exists - so the re-anchor re-fires each epoch and invalidateAll()
    // below destroys every carried arc again. Measured on the PPC set before
    // this latch: 1.8 to 203 gap re-anchors per outage EVENT, against 0.0-0.09
    // for the node that already had it, at essentially the same number of
    // outages. A LATER outage re-arms it, because the DDs that ended the first
    // one advance last_dd_ctow_ again.
    const bool gnss_starved = last_dd_ctow_ > 0.0 &&
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
    // The anchor is resolved BEFORE the trigger is evaluated, because "is there
    // somewhere independent to go?" is part of the decision: re-keying every
    // carried arc to move onto the previous float is a cost with no benefit.
    const gnss_fgo::IndependentCodePosition anchor =
        gnss_fgo::independentCodePosition(ep, adapter_cfg_);
    // prediction_fault keeps the weaker test - there the DDs have already
    // rejected the state, and breaking that lock is worth a stale target.
    const bool reanchor =
        anchor.ok &&
        (prediction_fault || (gnss_starved && anchor.independent()));
    if (reanchor) {
      ++n_reanchor_;
      if (anchor.independent()) ++n_reanchor_independent_;
      // The carried ambiguities absorbed the drifted position, so their floats
      // are stale by tens of cycles; re-key them before the DD factors are
      // built and drop the fix-and-hold progress certified against that state.
      amb_mgr_->invalidateAll();
      hold_candidates_.clear();
      code_growth_.reset();
      // Arm the latch only for the GAP trigger. prediction_fault is a
      // per-epoch measurement verdict and must stay free to fire again.
      if (gnss_starved) gap_latch_.arm();
      nfix_ = 0;
      // Say WHICH trigger fired: a starvation re-anchor is not a measurement
      // disagreement, and reporting it as one sends the reader to the wrong
      // half of the pipeline.
      RCLCPP_WARN(get_logger(),
                  "Re-anchoring at %s (%s) - carried arcs re-keyed.",
                  anchor.sourceName(),
                  prediction_fault
                      ? "predicted state rejected by its own DDs"
                      : "GNSS gap: no usable DD reached the graph");
    }

    const Eigen::Vector3d* gate_pos =
        (continuous && !reanchor) ? &predicted_pos : nullptr;
    // The layout records the rows and covariance each DD factor was actually
    // built with; the post-fit FIX validation reproduces the graph's robust
    // weights from it rather than re-deriving them (see DdFactorLayout).
    gnss_fgo::DdFactorLayout dd_layout;
    int n_bad_cov = 0;
    const auto pairs = gnss_fgo::addGroupedDdFactors(
        gnss_ep, xk, adapter_cfg_, dd_graph, dd_values, *amb_mgr_, gate_pos,
        &n_gated, &dd_layout, &n_bad_cov);
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
    // The motion factor is a BetweenFactor, so a run of fully-gated epochs
    // chains poses by RELATIVE constraints only; once the last DD-anchored pose
    // leaves the fixed-lag window the whole chain is free and ISAM2 throws
    // "singular near x<k>". A 100 m sigma cannot bias a well-observed epoch,
    // and there is nothing to bias in one that has no observations left.
    //
    // The sigma is kReanchorPosStdM, NOT the anchor's own covariance: the
    // covariance says how well the code solution is known, this sigma says how
    // hard to pull the graph towards it, and pulling as hard as a metre-level
    // code solution deserves would make the escape override the carrier phase.
    if (reanchor || gnss_ep.dd.empty()) {
      const Eigen::Vector3d anchor_pos =
          anchor.ok ? anchor.pos : ep.rover_ecef_apriori;
      dd_graph.add(gtsam::PriorFactor<gtsam::Point3>(
          xk, gtsam::Point3(anchor_pos),
          gtsam::noiseModel::Isotropic::Sigma(3, kReanchorPosStdM)));
    }
    // Refresh EVERY live ambiguity, not just the ones inserted this epoch.
    //
    // The fixed-lag smoother marginalizes by last-seen timestamp, so a carried
    // arc whose timestamp is never refreshed is marginalized out once it is
    // older than graph.lag_s - and its GAUGE PRIOR, a unary factor on that key
    // alone, goes with it. `anchored_` still records the key as gauged, so the
    // group silently loses rank and the next update throws
    // IndeterminantLinearSystemException.
    gtsam::FixedLagSmoother::KeyTimestampMap ts_dd;
    for (const gtsam::Key k : dd_values.keys()) ts_dd[k] = t_gnss;
    for (const gtsam::Key k : amb_mgr_->liveKeys()) ts_dd[k] = t_gnss;

    // The ISAM2 update and the marginal/estimate queries can all throw an
    // IndeterminantLinearSystemException on a degenerate epoch (e.g. an
    // under-constrained ambiguity); guard them together and reset on failure.
    gtsam::Point3 float_pos;
    Eigen::Matrix3d float_cov;
    try {
      smoother_->update(dd_graph, dd_values, ts_dd);
      float_pos = smoother_->calculateEstimate<gtsam::Point3>(xk);
      float_cov = smoother_->marginalCovariance(xk);
    } catch (const gtsam::IndeterminantLinearSystemException& e) {
      RCLCPP_WARN(get_logger(), "ISAM2 solve failed (%s) - resetting graph.",
                  e.what());
      dumpKeyDiagnostics(e.nearbyVariable());
      resetGraph();
      return;
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "ISAM2 solve failed (%s) - resetting graph.",
                  e.what());
      resetGraph();
      return;
    }

    Eigen::Vector3d pub_pos = float_pos;
    Eigen::Matrix3d pub_cov = float_cov;
    uint8_t status = grs::GnssSolution::STATUS_FLOAT;
    double ratio = 0.0;

    // Integer resolution is a pure projection/conditioning of the optimized
    // stage-2 graph posterior. No DD is applied a second time in a private GLS.
    //
    // This block only DECIDES; it must not publish. `status` / `pub_*` are
    // committed once, after every escape gate below, so the published state can
    // never disagree with `ar.fixed`. Setting them here and letting a gate flip
    // `ar.fixed` afterwards would publish STATUS_FIX with
    // the very integer-conditioned position it had just decided not to trust,
    // and would skip the post-hold FLOAT refresh, which is keyed on `status`.
    gnss_fgo::ArResult ar;
    if (ar_enabled_ && static_cast<int>(pairs.size()) >= kMinDdForAr) {
      gnss_fgo::GraphArPosterior posterior;
      std::string posterior_err;
      if (gnss_fgo::buildGraphPosteriorPoint(
              smoother_->getISAM2(), xk, pairs, posterior, &posterior_err)) {
        ar = gnss_fgo::resolveAmbiguitiesPosterior(
            gnss_ep, pairs, posterior, dd_layout, adapter_cfg_, ar_opt_);
        ratio = ar.ratio;
      } else {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Stage-2 graph AR posterior unavailable - skipping AR (%s).",
            posterior_err.c_str());
      }
    }
    // Escape gate: reject the fix only when the pseudoranges have disagreed
    // with the integer correction for several consecutive epochs. See
    // CodeGrowthMonitor - one epoch of disagreement is a false alarm.
    if (ar.fixed &&
        code_growth_.reject(ar.code_resid_growth,
                            ar_opt_.max_code_resid_growth_m,
                            ar_opt_.code_resid_persist)) {
      ar.fixed = false;
      ar.fail = gnss_fgo::ArFail::CodeGrowthReject;
      code_growth_.reset();
    }
    // Single commit point: every gate has now had its say.
    if (ar.fixed) {
      pub_pos = ar.fixed_pos;
      pub_cov = ar.state_cov;
      status = grs::GnssSolution::STATUS_FIX;
    }
    // Last epoch whose DDs actually reached the graph. Drives the starvation
    // trigger above, so a long run of fully-gated epochs counts as a GNSS
    // outage - which is exactly what it is from the estimator's point of view.
    // Seed on the first epoch so the gap is measured from a real anchor rather
    // than never arming (a node that has never had a usable DD has no state to
    // rescue, but one that loses them immediately after the first must recover).
    if (!gnss_ep.dd.empty() || last_dd_ctow_ < 0.0) last_dd_ctow_ = ctow;
    // Usable GNSS is back: re-arm for the next outage.
    if (!gnss_ep.dd.empty()) gap_latch_.onUsableGnss();
    nfix_ = ar.fixed ? nfix_ + 1 : 0;
    // Code-only DD position for the dump. Deliberately built from the RAW `ep`,
    // not the pre-fit-filtered `gnss_ep`: the filter gates against the
    // predicted state, and a yardstick derived from the state it is meant to
    // check is no yardstick at all. Only recorded, never fed back.
    gnss_fgo::CodeDdSolution ddwls;
    if (ar_dump_->enabled()) {
      ddwls = gnss_fgo::solveCodeDdWls(ep, adapter_cfg_, ep.rover_ecef_apriori);
    }
    // Record the pre-LAMBDA float and its covariance next to the decision they
    // produced (no-op unless debug.ar_dump_dir is set).
    ar_dump_->writeEpoch(ep.week, ep.tow, gnss_ep, pairs, ar, held_dd_.size(), nfix_,
                         float_pos, n_gated, ddwls);
    {
      // Hold accepted integers as two-variable DD constraints on the carried
      // keys (added once, removed on re-key). Runs every epoch so stale holds
      // are cleaned even when this epoch produced no fix. The published FIX
      // stays the analytical ar.fixed_pos; carry the now-tightened float forward.
      // Holds start only after min_fix_to_hold consecutive fixes (RTKLIB
      // minfix); the empty-spec call still cleans up stale holds.
      const auto observed_specs = gnss_fgo::collectHoldSpecs(
          *amb_mgr_, gnss_ep, ar, 0.5, ar_opt_.min_lock);
      const auto specs = gnss_fgo::confirmHoldSpecs(
          hold_candidates_, observed_specs, min_fix_to_hold_);
      const auto hr = gnss_fgo::applyHolds(*smoother_, *amb_mgr_, held_dd_, specs,
                                           adapter_cfg_.hold_sigma_cycles);
      if (hr == gnss_fgo::HoldResult::Failure) {
        // ISAM2 may be inconsistent after a thrown hold update; reset and skip.
        RCLCPP_WARN(get_logger(), "Hold update failed - resetting graph.");
        resetGraph();
        return;
      }
      if (hr == gnss_fgo::HoldResult::Success) {
        try {
          float_pos = smoother_->calculateEstimate<gtsam::Point3>(xk);
          float_cov = smoother_->marginalCovariance(xk);
          // Publish the now-tightened FLOAT this epoch too (a new FIX keeps the
          // analytical ar.fixed_pos); otherwise the held cm-level constraint
          // would not reach the current output.
          if (status != grs::GnssSolution::STATUS_FIX) {
            pub_pos = float_pos;
            pub_cov = float_cov;
          }
        } catch (const gtsam::IndeterminantLinearSystemException& e) {
          // The hold UPDATE succeeded; only this refresh QUERY failed
          // (numerically extreme conditioning on a long arc can break the
          // marginal shortcut computation). Keep the pre-hold estimates and
          // continue - resetting would discard every carried arc over a
          // read-only failure. A genuinely corrupted graph still resets at
          // the next epoch's update.
          RCLCPP_WARN(get_logger(),
                      "Post-hold estimate failed (%s) - keeping pre-hold "
                      "estimates.", e.what());
          dumpKeyDiagnostics(e.nearbyVariable());
        } catch (const std::exception& e) {
          RCLCPP_WARN(get_logger(),
                      "Post-hold estimate failed (%s) - keeping pre-hold "
                      "estimates.", e.what());
        }
      }
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

    publishSolution(ep, pub_pos, pub_cov, status, ratio);

    // Real-time budget: the per-epoch solve time must stay bounded and inside
    // the input period. With the fixed-lag window it is flat vs. epoch index;
    // with full history (graph.lag_s <= 0) it grows and eventually exceeds the
    // budget, dropping epochs. Warn so a user on constrained hardware notices.
    const double solve_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - solve_t0)
                                .count();
    solve_ms_max_ = std::max(solve_ms_max_, solve_ms);
    if (solve_ms > kEpochBudgetMs) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Epoch solve %.0f ms exceeds the %.0f ms real-time budget - graph too "
          "large; reduce graph.lag_s.", solve_ms, kEpochBudgetMs);
    }

    double llh[3];
    const double ecef[3] = {pub_pos.x(), pub_pos.y(), pub_pos.z()};
    ecef2pos(ecef, llh);
    // Arc diagnostics: short mean arc age / high re-key counts mean the
    // carried ambiguities never accumulate and the float stays at code level.
    // solve ms (this / max) exposes the real-time headroom.
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), kPositionLogThrottleMs,
        "FGO %s | LLH: %.8f %.8f %.3f | Ratio: %.1f | DD: %zu (gated %d) | "
        "Age: %.1f | "
        "Arcs: %zu (fresh %zu, rekey %llu, mean %.0fs) | hold: %zu | nfix: %d | "
        "solve: %.0f/%.0f ms",
        status == grs::GnssSolution::STATUS_FIX ? "FIX" : "FLOAT",
        llh[0] * R2D, llh[1] * R2D, llh[2], ratio, ep.dd.size(), n_gated,
        ep.age_s,
        amb_mgr_->liveCount(), amb_mgr_->freshCount(),
        static_cast<unsigned long long>(amb_mgr_->totalRekeys()),
        amb_mgr_->meanArcAge(t_gnss), held_dd_.size(), nfix_, solve_ms,
        solve_ms_max_);

    // Carry the (hold-tightened) FLOAT estimate forward; the published FIX
    // position itself is never fed back into the graph (only the accepted
    // integers are, as gauge-free holds).
    prev_float_pos_ = float_pos;
    last_float_valid_ = true;
    prev_tow_ = t_gnss;  // continuous GPST seconds
    if (doppler_ok) {
      prev_vel_ = ep.rover_vel_ecef;
      prev_vel_valid_ = true;
    }
    // The analytical FIX is an output hypothesis, not a safe geometry prior.
    // Feeding a wrong fix back into the elevation mask can remove good
    // satellites on the next epoch and turn one mis-fix into a persistent
    // failure. Use only the graph's float/hold-tightened state here.
    last_estimate_ = float_pos;
    last_estimate_valid_ = true;
    ++epoch_index_;
  }

  // Diagnostic dump for an IndeterminantLinearSystemException: list every live
  // factor touching the offending key with its error at the current
  // linearization point (NaN / huge errors discriminate a poisoned residual
  // from robust-weight collapse), so the degenerate structure can be
  // identified from the log.
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
    dumpGaugeAudit(key);
  }

  // Ground truth for a rank deficiency in the ambiguities: for every carrier
  // group touching `key`, count the unary priors that actually EXIST in the
  // smoother right now. `anchored_` records intent; this reports reality, and
  // the two diverging is exactly how a group loses rank without anyone noticing.
  void dumpGaugeAudit(gtsam::Key key) {
    const auto& factors = smoother_->getISAM2().getFactorsUnsafe();
    // 1. Ambiguity keys reachable from `key` through the factors that touch it.
    std::set<gtsam::Key> amb;
    for (const auto& f : factors) {
      if (!f) continue;
      if (std::find(f->keys().begin(), f->keys().end(), key) == f->keys().end())
        continue;
      for (const gtsam::Key k : f->keys())
        if (gtsam::Symbol(k).chr() == 'n') amb.insert(k);
    }
    if (amb.empty()) return;
    // 2. Union-find over every factor that links two ambiguities.
    std::map<gtsam::Key, gtsam::Key> parent;
    auto find = [&parent](gtsam::Key k) {
      auto it = parent.find(k);
      if (it == parent.end()) { parent[k] = k; return k; }
      while (parent[k] != k) k = parent[k];
      return k;
    };
    for (const gtsam::Key k : amb) find(k);
    for (const auto& f : factors) {
      if (!f) continue;
      gtsam::Key prev = 0; bool have = false;
      for (const gtsam::Key k : f->keys()) {
        if (gtsam::Symbol(k).chr() != 'n' || !amb.count(k)) continue;
        if (have) { const gtsam::Key a = find(prev), b = find(k);
                    if (a != b) parent[a] = b; }
        prev = k; have = true;
      }
    }
    // 3. Unary factors on an ambiguity = its gauge prior.
    std::map<gtsam::Key, int> gauges;  // component root -> prior count
    for (const gtsam::Key k : amb) gauges[find(k)] += 0;
    for (const auto& f : factors) {
      if (!f || f->keys().size() != 1) continue;
      const gtsam::Key k = f->keys().front();
      if (gtsam::Symbol(k).chr() != 'n' || !amb.count(k)) continue;
      gauges[find(k)] += 1;
    }
    std::ostringstream os;
    os << "gauge audit: " << amb.size() << " ambiguities in " << gauges.size()
       << " component(s);";
    int starved = 0;
    for (const auto& g : gauges) {
      std::size_t sz = 0;
      for (const gtsam::Key k : amb) if (find(k) == g.first) ++sz;
      os << " [n" << gtsam::Symbol(g.first).index() << " size=" << sz
         << " priors=" << g.second << "]";
      if (g.second == 0) ++starved;
    }
    os << " UNGAUGED_COMPONENTS=" << starved;
    RCLCPP_ERROR(get_logger(), "%s", os.str().c_str());
  }

  void resetGraph() {
    smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(
        smoother_lag_, makeIsamParams());
    epoch_index_ = 0;
    next_amb_id_ = 0;
    last_estimate_valid_ = false;
    last_float_valid_ = false;
    prev_vel_valid_ = false;
    nfix_ = 0;
    if (amb_mgr_) amb_mgr_->resetAll();
    held_dd_.clear();
    hold_candidates_.clear();
    code_growth_.reset();
    gap_latch_.reset();
  }

  void publishSolution(const gnss_utils::PreprocessedEpoch& ep,
                       const Eigen::Vector3d& pos, const Eigen::Matrix3d& cov,
                       uint8_t status, double ratio) {
    auto msg = std::make_unique<grs::GnssSolution>();
    // Inherit the observation epoch stamp (not now()): keeps the solution's
    // header time tied to the input, so a downstream time-aligned consumer does
    // not fold FGO processing / bag-replay wall-clock into the measurement time.
    msg->header.stamp = ep.stamp;
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

    const double ecef[3] = {pos.x(), pos.y(), pos.z()};
    double llh[3];
    ecef2pos(ecef, llh);
    msg->latitude = llh[0] * R2D;
    msg->longitude = llh[1] * R2D;
    msg->altitude = llh[2];
    msg->pos_ecef.x = pos.x();
    msg->pos_ecef.y = pos.y();
    msg->pos_ecef.z = pos.z();
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) msg->pos_cov_ecef[3 * r + c] = cov(r, c);
    }

    // ENU origin priority: fixed_origin (configured) > base station.
    double origin[3] = {0.0, 0.0, 0.0};
    if (fixed_origin_valid_) {
      std::copy_n(fixed_origin_ecef_, 3, origin);
    } else if (ep.base_ecef.norm() > 0.0) {
      origin[0] = ep.base_ecef.x();
      origin[1] = ep.base_ecef.y();
      origin[2] = ep.base_ecef.z();
    }
    msg->pos_enu_org_ecef.x = origin[0];
    msg->pos_enu_org_ecef.y = origin[1];
    msg->pos_enu_org_ecef.z = origin[2];
    // ENU reference lat/lon: the origin's when an origin is set, else the current
    // position. The ENU displacement mean and its covariance MUST share this one
    // reference so they describe the same axes (was: mean at origin, cov at the
    // current position - a small but real inconsistency for a public API).
    double enu_ref_lat = llh[0], enu_ref_lon = llh[1];
    if (norm(origin, 3) > 0.0) {
      double origin_llh[3];
      ecef2pos(origin, origin_llh);
      enu_ref_lat = origin_llh[0];
      enu_ref_lon = origin_llh[1];
      const double d_ecef[3] = {ecef[0] - origin[0], ecef[1] - origin[1],
                                ecef[2] - origin[2]};
      double enu[3];
      ecef2enu(origin_llh, d_ecef, enu);
      msg->pos_enu.x = enu[0];
      msg->pos_enu.y = enu[1];
      msg->pos_enu.z = enu[2];
    }

    double q_ecef[9];
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) q_ecef[3 * r + c] = cov(r, c);
    }
    double q_enu[9];
    gnss_utils::rotateCovariance(q_ecef, enu_ref_lat, enu_ref_lon, q_enu);
    for (int i = 0; i < 9; ++i) msg->pos_enu_cov[i] = q_enu[i];

    // Doppler-derived rover velocity (ECEF + ENU), the same quantity that drives
    // the inter-epoch motion factor.
    if (ep.rover_vel_valid) {
      msg->vel_ecef.x = ep.rover_vel_ecef.x();
      msg->vel_ecef.y = ep.rover_vel_ecef.y();
      msg->vel_ecef.z = ep.rover_vel_ecef.z();
      const double vel_ecef[3] = {ep.rover_vel_ecef.x(), ep.rover_vel_ecef.y(),
                                  ep.rover_vel_ecef.z()};
      double vel_enu[3];
      ecef2enu(llh, vel_ecef, vel_enu);
      msg->vel_enu.x = vel_enu[0];
      msg->vel_enu.y = vel_enu[1];
      msg->vel_enu.z = vel_enu[2];
      // The real 3x3, not a diagonal of the averaged variance: Doppler geometry
      // is markedly weaker vertically, and a consumer weighting this velocity
      // needs to see that rather than an isotropic claim.
      double qv_ecef[9];
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) qv_ecef[3 * r + c] = ep.rover_vel_cov(r, c);
      }
      double qv_enu[9];
      gnss_utils::rotateCovariance(qv_ecef, enu_ref_lat, enu_ref_lon, qv_enu);
      for (int i = 0; i < 9; ++i) {
        msg->vel_cov_ecef[i] = qv_ecef[i];
        msg->vel_enu_cov[i] = qv_enu[i];
      }
    }

    sol_pub_->publish(std::move(msg));
  }

  std::mutex mtx_;
  std::unique_ptr<gnss_utils::GnssPreprocessor> preprocessor_;
  std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_;
  double smoother_lag_{1.0e9};
  double solve_ms_max_{0.0};  // worst per-epoch solve time (real-time headroom)
  gnss_fgo::AdapterConfig adapter_cfg_;

  int epoch_index_{0};
  std::uint64_t next_amb_id_{0};  // globally-unique ambiguity keys
  // Carried-across-epochs ambiguity keys (one SD ambiguity per (sat,band)).
  std::unique_ptr<gnss_fgo::PersistentAmbiguities> amb_mgr_;
  double amb_max_outage_{5.0};
  gnss_fgo::HeldDdMap held_dd_;
  gnss_fgo::HoldCandidateMap hold_candidates_;  // held DD constraints (by sat pair)
  gnss_fgo::CodeGrowthMonitor code_growth_;  // escape gate state
  int min_fix_to_hold_{10};  // consecutive fixes before holding (RTKLIB minfix)
  int nfix_{0};              // current consecutive-fix count
  // Continuous TOW of the last epoch whose DDs actually reached the graph. A
  // longer gap than kReanchorGapS forces the re-anchor even with no usable DDs
  // this epoch - see the re-anchor comment for why that case is the important
  // one.
  double last_dd_ctow_{-1.0};
  gnss_fgo::GapReanchorLatch gap_latch_;
  // Re-anchors, and how many of them had a genuinely independent target. When
  // the second lags the first the escape is degenerating into the old
  // anchor-to-yourself behaviour and its verdicts should not be trusted.
  std::uint64_t n_reanchor_{0};
  std::uint64_t n_reanchor_independent_{0};
  // Observation messages the middleware dropped before this node saw them, and
  // the epoch time at which the most recent loss was noticed. The second is
  // what keeps a TRANSPORT gap from being mistaken for a GNSS outage: the
  // starvation re-anchor exists for a sky that stopped delivering, and it
  // re-keys every carried arc, so firing it because messages went missing
  // destroys the carrier history for no reason.
  std::atomic<std::uint64_t> n_obs_message_lost_{0};
  double last_obs_loss_ctow_{-1.0};
  double last_epoch_ctow_{-1.0};
  // Epochs the MATCHER discarded (queue overflow, late arrival, ...). These
  // never reach processEpoch, so they are the other way an epoch can go
  // missing without the middleware reporting anything lost.
  std::uint64_t n_epochs_dropped_{0};
  double last_epoch_drop_ctow_{-1.0};
  Eigen::Vector3d last_estimate_{Eigen::Vector3d::Zero()};
  bool last_estimate_valid_{false};

  // Filter state carried between epochs for the motion model.
  double prev_tow_{0.0};
  Eigen::Vector3d prev_vel_{Eigen::Vector3d::Zero()};
  bool prev_vel_valid_{false};
  Eigen::Vector3d prev_float_pos_{Eigen::Vector3d::Zero()};
  bool last_float_valid_{false};

  gnss_fgo::ArOptions ar_opt_;
  bool ar_enabled_{true};  // ambiguity.resolution; false = FLOAT-only estimator
  double motion_accel_sigma_{3.0};
  double doppler_max_res_m_{2.0};
  std::unique_ptr<gnss_fgo::ArDebugDumper> ar_dump_;

  double fixed_origin_ecef_[3]{0.0, 0.0, 0.0};
  bool fixed_origin_valid_{false};

  rclcpp::Subscription<grs::GnssObservations>::SharedPtr rover_obs_sub_;
  rclcpp::Subscription<grs::GnssObservations>::SharedPtr base_obs_sub_;
  rclcpp::Subscription<grs::GnssEphemerides>::SharedPtr nav_sub_;
  rclcpp::Publisher<grs::GnssSolution>::SharedPtr sol_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssFgoNode>());
  rclcpp::shutdown();
  return 0;
}
