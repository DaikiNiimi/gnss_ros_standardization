// SPDX-License-Identifier: MIT
#ifndef GNSS_ROS_STANDARDIZATION_GNSS_PREPROCESSOR_HPP
#define GNSS_ROS_STANDARDIZATION_GNSS_PREPROCESSOR_HPP

#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "gnss_ros_standardization/ephemeris_store.hpp"
#include "gnss_ros_standardization/epoch_matcher.hpp"
#include "gnss_ros_standardization/msg/gnss_ephemerides.hpp"
#include "gnss_ros_standardization/msg/gnss_observations.hpp"
#include "gnss_ros_standardization/obs_converter.hpp"

extern "C" {
#include "rtklib.h"
}

namespace gnss_utils {

// One undifferenced rover observation (one satellite, one band), with the
// satellite state already computed. Carries everything an undifferenced GNSS
// factor (e.g. gtsam::PseudorangeFactor / CarrierPhaseFactor) needs.
struct SatObs {
  int sat{0};            // RTKLIB satellite number
  std::string satid;     // e.g. "G05"
  int sys{0};            // SYS_GPS | SYS_GLO | ...
  int band{0};           // RTKLIB frequency index: 0=L1, 1=L2, 2=L5
  uint8_t code{0};       // CODE_* actually observed on this band

  // Satellite ECEF position [m] at signal transmission time and satellite
  // clock bias [s]. Raw broadcast values: no Sagnac pre-rotation is applied
  // and the clock bias is NOT subtracted from the pseudorange (downstream
  // factors handle both; see e.g. gtsam::gnss::geodist).
  Eigen::Vector3d sat_pos{0, 0, 0};
  double sat_clk{0.0};
  // Satellite ECEF velocity [m/s] and clock drift [s/s] at transmission time
  // (raw broadcast). Used for the Doppler-based rover velocity estimate.
  Eigen::Vector3d sat_vel{0, 0, 0};
  double sat_clk_drift{0.0};

  double pr{0.0};        // pseudorange [m] (0 when absent)
  double cp_m{0.0};      // carrier phase in METERS (= lam * cycles, 0 when absent)
  double doppler{0.0};   // [Hz]
  double snr{0.0};       // [dBHz]
  double lam{0.0};       // carrier wavelength [m]
  double el{0.0};        // elevation at the rover a-priori position [rad]
  double az{0.0};        // azimuth [rad]
  bool slip{false};      // continuity break on this band -> re-key the carried
                         // ambiguity (Continuous / FixAndHold). Does NOT remove
                         // the carrier from this epoch's DD or AR: it is still a
                         // valid measurement (the first sample of the new arc).
  bool half_cycle{false};  // half-cycle-ambiguity present -> a half-integer, so
                           // exclude from integer AR (but keep in the float DD)
  bool cp_excluded{false};  // carrier corrupt this epoch (CMC gross error) ->
                            // drop it from the DD entirely, and re-key
};

// One double-difference pair (reference satellite minus target satellite,
// rover minus base). Field-for-field what the GTSAM DD factors take:
// the four raw observables, the four satellite positions (computed separately
// at rover and base observation times -- satellites move ~3 km/s, so reusing
// rover-time positions for an aged base epoch causes ~100 m DD errors), the
// known base position, and the wavelength.
struct DdSignal {
  int sat_ref{0}, sat_tar{0};
  std::string satid_ref, satid_tar;
  int sys{0};
  int band{0};
  // Tracking codes at each receiver. Cross-receiver pairing may deliberately
  // use different codes (for example Galileo Q at the rover and X at the base),
  // but both codes must remain common across a DD reference/target pair so each
  // receiver's inter-code bias cancels.
  uint8_t code_ref{0}, code_tar{0};
  uint8_t code_base_ref{0}, code_base_tar{0};
  double lam{0.0};       // [m]; identical for ref/tar (enforced; GLONASS FDMA
                         // pairs with different FCN wavelengths drop carrier)

  bool has_pr{false};    // all four pseudoranges present
  bool has_cp{false};    // all four carrier phases present

  // Observables [m], with satellite clock (+c*dts) applied at each receiver's
  // own reception epoch (see sat_ref_rov below). Carrier phase already
  // converted to meters.
  double pr_rov_ref{0}, pr_base_ref{0}, pr_rov_tar{0}, pr_base_tar{0};
  double cp_rov_ref{0}, cp_base_ref{0}, cp_rov_tar{0}, cp_base_tar{0};

  // Satellite ECEF positions [m] at signal transmission time, per receiver
  // epoch. No Sagnac pre-rotation applied (cancels in DD). The pr/cp_* above
  // DO have satellite clock (+c*dts) applied, since it does NOT cancel across
  // asynchronous rover/base epochs.
  Eigen::Vector3d sat_ref_rov{0, 0, 0}, sat_tar_rov{0, 0, 0};
  Eigen::Vector3d sat_ref_base{0, 0, 0}, sat_tar_base{0, 0, 0};

  double el_ref{0.0}, el_tar{0.0};   // rover elevation [rad]
  double el_base_ref{0.0}, el_base_tar{0.0};  // base elevation [rad]
  // Residual variance of atmospheric corrections in one rover-base single
  // difference [m^2], retained per satellite for grouped DD covariance.
  double model_var_ref_sd{0.0}, model_var_tar_sd{0.0};
  double snr_ref{0.0}, snr_tar{0.0}; // rover SNR [dBHz]
  bool slip_ref{false}, slip_tar{false};        // continuity break -> re-key
  bool half_cycle_ref{false}, half_cycle_tar{false};  // half-integer -> AR-exclude
};

// One matched rover/base epoch after preprocessing. Emitted epochs always
// carry a valid rover a-priori position (SPP, or the caller-provided
// approximation when SPP fails).
struct PreprocessedEpoch {
  uint32_t week{0};
  double tow{0.0};       // rover epoch (GPST)
  // Base epoch identity + age. During a base outage the rover epoch is still
  // emitted (rover-only): has_base is false and base_tow / age_s are NaN.
  bool has_base{false};
  uint32_t base_week{0};
  double base_tow{std::numeric_limits<double>::quiet_NaN()};
  double age_s{0.0};     // rover minus base epoch difference (NaN if no base)
  // Rover message header stamp (PC arrival time by default). This is what
  // consumers should pair against other ROS-stamped sensors (e.g. IMU);
  // receiver output latency then shows up as a small pairing error.
  builtin_interfaces::msg::Time stamp;

  Eigen::Vector3d base_ecef{0, 0, 0};
  // Linearization point / mask evaluation point. When the caller supplies an
  // approximate rover position to drainEpochs this is that hint (the caller's
  // own filter state is far less noisy than SPP, which is what masks want).
  // It is therefore NOT an independent measurement - see rover_ecef_spp.
  Eigen::Vector3d rover_ecef_apriori{0, 0, 0};

  // Standalone single-point (code) solution, computed from THIS epoch's
  // observations only and never from the caller's hint (Config::compute_spp).
  //
  // Exists so a consumer has one position that its own estimator cannot move.
  // Downstream filters/optimizers feed their state back in as the a-priori
  // hint above, which makes rover_ecef_apriori a copy of that state and
  // useless as a cross-check; this field is the independent alternative,
  // usable for fault detection or for re-anchoring an estimate that has
  // drifted. Accuracy is ordinary SPP (metre level). For a short baseline
  // where a base station is available, a code double-difference solution is
  // several times better and equally independent.
  Eigen::Vector3d rover_ecef_spp{0, 0, 0};
  bool rover_spp_valid{false};
  Eigen::Matrix3d rover_spp_cov{Eigen::Matrix3d::Zero()};  // ECEF [m^2]
  int rover_spp_nsat{0};

  // Rover ECEF velocity [m/s] from undifferenced Doppler (RTKLIB estvel-style
  // least squares: receiver velocity + clock drift). Used to constrain
  // consecutive position states with a loosely-coupled motion factor.
  Eigen::Vector3d rover_vel_ecef{0, 0, 0};
  bool rover_vel_valid{false};
  // Full 3x3 ECEF velocity covariance [(m/s)^2]: sigma0^2 * (H'WH)^-1 over the
  // velocity block. The Doppler geometry is the SAME geometry that makes a
  // vertical position weak, so this matrix is strongly anisotropic - typically
  // 2-4x larger up than horizontal - and it is not diagonal in ECEF at all.
  // Collapsing it to a scalar (which this used to do, and which rover_vel_var
  // still reports for readers that want one number) tells a consumer the
  // vertical channel is better than it is and the horizontal worse, and hides
  // the correlation entirely.
  Eigen::Matrix3d rover_vel_cov{Eigen::Matrix3d::Zero()};
  double rover_vel_var{0.0};  // trace(rover_vel_cov)/3 [(m/s)^2]
  double rover_vel_res{0.0};  // post-fit Doppler residual RMS [m/s] (outlier gate)
  int rover_vel_nsat{0};      // satellites in the accepted Doppler solve
  int rover_vel_excluded{0};  // satellites dropped by the studentized-residual FDE

  std::vector<SatObs> rover_sats;  // undifferenced, masks applied
  std::vector<DdSignal> dd;        // empty when age_s exceeds max_age_s
};

// Undifferenced Doppler velocity solution (see estimateDopplerVelocity).
struct DopplerVelocitySolution {
  Eigen::Vector3d vel{Eigen::Vector3d::Zero()};  // rover ECEF velocity [m/s]
  Eigen::Matrix3d cov{Eigen::Matrix3d::Zero()};  // ECEF [(m/s)^2]
  double res{0.0};   // unweighted post-fit residual RMS [m/s]
  int nsat{0};       // satellites in the accepted solve
  int excluded{0};   // satellites dropped by the studentized-residual FDE
};

// Rover velocity + receiver clock drift from undifferenced Doppler, RTKLIB
// estvel/resdop-style weighted least squares, with per-satellite fault
// detection. Exposed (rather than kept private to the preprocessor) because it
// is a self-contained GNSS routine and because the FDE and the covariance
// SHAPE it returns are worth testing directly - a wrong 3x3 is invisible in a
// trajectory plot but silently mis-weights every consumer of the velocity.
//
// max_nsigma <= 0 disables the rejection loop. Returns false when fewer than 4
// usable Doppler observations exist or the solve is degenerate.
bool estimateDopplerVelocity(const std::vector<SatObs>& sats,
                             const Eigen::Vector3d& rover_ecef,
                             double sigma_mps, double max_nsigma,
                             int max_exclude, DopplerVelocitySolution& out);

// Facade bundling every GNSS-domain step needed to feed raw observations
// into a factor-graph optimizer: ephemeris bookkeeping, rover/base epoch
// matching, satellite position/clock computation at transmission time
// (RTKLIB satposs), SPP a-priori (pntpos), elevation/SNR/system masks,
// per-system reference satellite selection, and double-difference pairing
// with cycles-to-meters conversion.
//
// Deliberately GTSAM-free: outputs are plain Eigen/std types
// (gtsam::Point3 is a typedef of Eigen::Vector3d, so they map 1:1).
//
// Not thread-safe: the caller serializes push*/drainEpochs (hold one mutex
// across ROS callbacks, like the RTK node does).
class GnssPreprocessor {
 public:
  using ObsMsg = gnss_ros_standardization::msg::GnssObservations;
  using EphMsg = gnss_ros_standardization::msg::GnssEphemerides;

  struct Config {
    double el_mask_rad{15.0 * D2R};
    // Flat SNR mask [dBHz]; 0 disables. Applied to BOTH receivers: a double
    // difference inherits the noise of its worst input, so masking the rover
    // alone would admit clean-rover/weak-base pairs that look trustworthy by
    // rover SNR but carry the base's noise into the ambiguity.
    double snr_mask_dbhz{0.0};
    // Elevation-dependent SNR mask (RTKLIB testsnr). When snr_mask.ena[0] is set
    // it takes precedence over snr_mask_dbhz; same model as the RTK example.
    snrmask_t snr_mask{};
    // GLONASS is excluded by default: its inter-frequency biases do not
    // cancel in DD across heterogeneous receivers.
    int navsys{SYS_GPS | SYS_GAL | SYS_CMP | SYS_QZS};
    std::vector<int> bands{0};        // RTKLIB freq indices; default L1-only
    // Allow GLONASS carrier-phase DD (only sensible when rover and base are
    // the same receiver model AND the FCN wavelengths match per pair).
    bool glonass_carrier_dd{false};
    // Systems admitted to the UNDIFFERENCED product (rover_sats, the SPP fix,
    // the Doppler velocity) but excluded from double differencing and hence
    // from AR. Default 0 = no such split, so navsys alone decides everything
    // and existing behaviour is unchanged.
    //
    // Exists because the two questions are physically different. GLONASS FDMA
    // inter-frequency bias does not cancel in a DD across heterogeneous
    // receivers - that is a CARRIER DD problem. It says nothing about whether
    // GLONASS pseudoranges should count toward availability, the single-point
    // fix, or the Doppler velocity. Folding both into navsys meant a canyon
    // epoch whose only satellites were GLONASS produced nothing at all: on PPC
    // nagoya tow 550595.0 the raw sky was 1 BDS + 1 GLONASS, and the exclusion
    // is part of why the masks left zero usable satellites.
    int navsys_undifferenced_only{0};
    // Allow DD across DIFFERENT tracking codes on the same band where it is
    // carrier-integer-safe (Galileo E-band pilot/combined; L5/E5a Q vs X). The
    // per-receiver inter-code bias is common to a group's reference and target
    // (all rover sats share the rover code, all base sats the base code), so it
    // cancels in the DD; only half-cycle-ambiguous mixes (GPS/QZS L2 codeless-
    // P(Y) 2W vs L2C) are excluded. Recovers Galileo / L5 when the rover and
    // base receivers track different components (e.g. Septentrio Q vs combined X).
    bool cross_code_carrier{false};
    // Atmospheric corrections applied to each undifferenced observable before
    // differencing. Values are RTKLIB IONOOPT_* / TROPOPT_* constants.
    int ionoopt{IONOOPT_BRDC};
    int tropopt{TROPOPT_SAAS};
    // Oldest base epoch a rover epoch may be paired with. ONE quantity: the
    // matcher's pairing window IS this, because a pair the DD stage will refuse
    // is not a pair. Keeping a separate, looser matcher window let the matcher
    // hand back epochs whose base was up to that window old; the DD stage then
    // emptied them (age_s > max_age_s), and - worse - the unusable base stayed
    // at the head of the queue and was re-selected for every following rover,
    // so a single backlog turned into tens of seconds with no double
    // differences at all.
    double max_age_s{1.0};
    double match_tol_s{0.001};
    // Epoch-matcher robustness (see RoverBaseEpochMatcher::Options). The
    // decision horizon is drop_ahead_s; duplicate_tol_s must be <= match_tol_s.
    double matcher_decision_horizon_s{2.0};
    double matcher_reorder_window_s{0.05};
    double matcher_duplicate_tol_s{1e-6};
    std::size_t matcher_queue_limit{50};
    Eigen::Vector3d base_ecef{0, 0, 0};  // required for DD output
    // Always run the standalone SPP solve and publish it as
    // PreprocessedEpoch::rover_ecef_spp, even when the caller supplies an
    // a-priori hint. Without this the hint short-circuits pntpos() and no
    // caller-independent position exists anywhere in the output. Costs one
    // pntpos() per epoch (a few ms - the same call the RTK node makes
    // unconditionally); set false to skip it if that matters.
    bool compute_spp{true};
    // Per-satellite fault detection inside the Doppler velocity solve. The
    // global post-fit RMS gate a consumer can apply to rover_vel_res only sees
    // the SUM of the residuals, so one NLOS range-rate hides inside a large
    // healthy set and biases the velocity while leaving the RMS respectable.
    // This drops the largest studentized residual and re-solves while it
    // exceeds doppler_max_nsigma and redundancy remains. Studentized, not raw:
    // a satellite with high leverage pulls the solution towards itself, which
    // SHRINKS its own residual, so a raw-residual test systematically misses
    // exactly the outliers that do the most damage. 0 disables the loop.
    double doppler_max_nsigma{4.0};
    // A-priori zenith range-rate sigma [m/s]; the per-satellite variance is
    // doppler_sigma_mps^2 / sin^2(elevation). Sets the absolute scale of
    // PreprocessedEpoch::rover_vel_cov and the reference the blunder test is
    // judged against, so it must be a real noise figure rather than a relative
    // weight. 0.1 m/s matches the floor both FGO nodes already apply.
    double doppler_sigma_mps{0.1};
    // Cap on the drops, so a genuinely degraded epoch reports a large residual
    // and a large covariance instead of being trimmed into false confidence.
    int doppler_max_exclude{2};
    // Cycle-slip detection for the carried ambiguities of the FGO continuous /
    // fix_and_hold modes (an undetected slip corrupts a carried ambiguity). All
    // checks are on the rover-base pair (both receivers), complementing the LLI
    // cycle-slip bit and half-cycle-bit transition and observed-code change that
    // are always monitored:
    //   - SD geometry-free (dual-freq): jump in (Lrov1-Lbas1)*lam1 -
    //     (Lrov2-Lbas2)*lam2 beyond slip_gf_threshold_m -> slip on both bands.
    //   - SD code-minus-carrier: a large jump in (Prov-Pbas) - (Lrov-Lbas)*lam
    //     (gross carrier error / big slip) -> slip AND drop the corrupt carrier
    //     from this epoch's DD (cp_excluded); threshold well above code noise.
    //   - Doppler-phase (RTKLIB detslp_dop, single-frequency safeguard): the
    //     per-sat residual dL + D*dt with the common receiver-clock component
    //     (median) removed, beyond slip_dop_threshold_cyc -> slip.
    //   - Carrier outage: a carrier gap longer than slip_max_gap_s [s] -> slip
    //     on resume (unit matches the FGO's PersistentAmbiguities max_outage_s).
    bool detect_slip_gf{true};
    double slip_gf_threshold_m{0.05};
    bool detect_slip_cmc{true};
    double slip_cmc_threshold_m{3.0};
    bool detect_slip_dop{true};
    double slip_dop_threshold_cyc{1.0};
    double slip_max_gap_s{2.0};
  };

  explicit GnssPreprocessor(const Config& config);
  ~GnssPreprocessor();

  GnssPreprocessor(const GnssPreprocessor&) = delete;
  GnssPreprocessor& operator=(const GnssPreprocessor&) = delete;

  void setBasePosition(const Eigen::Vector3d& ecef) { config_.base_ecef = ecef; }

  void pushEphemerides(const EphMsg& msg);
  void pushRoverObs(ObsMsg::ConstSharedPtr msg);
  void pushBaseObs(ObsMsg::ConstSharedPtr msg);

  // Process every rover/base epoch that can be decided now. approx_rover is
  // the fallback rover position (e.g. the optimizer's last estimate) used for
  // masks and as the published a-priori when SPP fails for an epoch.
  std::vector<PreprocessedEpoch> drainEpochs(
      const Eigen::Vector3d* approx_rover = nullptr);

  // End-of-stream: also finalize the newest buffered rover epoch. If ephemeris
  // has not arrived yet (an adversarial callback order where EOF precedes the
  // transient-local nav snapshot) the request is DEFERRED, not applied, so no
  // observation is erased; the deferred flush runs on the next drain once nav
  // is available.
  std::vector<PreprocessedEpoch> flushEpochs(
      const Eigen::Vector3d* approx_rover = nullptr);

  std::size_t pendingRoverCount() const { return matcher_.pendingRoverCount(); }
  std::size_t pendingBaseCount() const { return matcher_.pendingBaseCount(); }
  const RoverBaseEpochMatcher::Counters& matcherCounters() const {
    return matcher_.counters();
  }
  std::vector<RoverBaseEpochMatcher::DropEvent> takeDropEvents() {
    return matcher_.takeDropEvents();
  }

  // Why a matched rover epoch produced no PreprocessedEpoch. Without this a
  // consumer can only observe that an epoch went missing, not why - and the
  // three causes need completely different responses (a real sky outage wants
  // dead reckoning, a mask/decoding problem wants fixing).
  struct EpochRejectCounters {
    std::uint64_t no_observations{0};   // nothing survived the frequency mask
    std::uint64_t no_rover_entries{0};  // mask left the rover with no satellite
    std::uint64_t no_apriori{0};        // no SPP, no hint, no previous, no base
    // NOT a rejection: the masks left no usable rover satellite, so the epoch
    // is emitted with rover_sats and dd both empty. Counted separately because
    // it is the signature of a real sky outage, and a consumer that can
    // dead-reckon wants to know how much of the run it had to carry.
    std::uint64_t emitted_without_satellites{0};
    // Epochs actually discarded (no epoch reached the caller at all).
    std::uint64_t total() const {
      return no_observations + no_rover_entries + no_apriori;
    }
  };
  const EpochRejectCounters& epochRejectCounters() const {
    return epoch_reject_;
  }

  // Rover epochs dropped by the matcher since the last call.
  // Newest buffered base epoch [GPST continuous s], or -1 (base-outage detection).
  double newestBaseCtow() const { return matcher_.newestBaseCtow(); }

  std::vector<RoverBaseEpochMatcher::Dropped> takeDropped() {
    return matcher_.takeDropped();
  }

  bool hasEphemeris() const { return nav_.n > 0 || nav_.ng > 0; }

 private:
  // `base` is null during a base outage (rover-only epoch): the rover a-priori,
  // Doppler velocity and rover_sats are still populated, dd is left empty, and
  // has_base / base_tow / age_s are cleared.
  EpochRejectCounters epoch_reject_;

  bool buildEpoch(const ObsMsg& rover, const ObsMsg* base, double age_s,
                  const Eigen::Vector3d* approx_rover, PreprocessedEpoch& out);

  // Build a PreprocessedEpoch for each matcher pair (rover-only when no base)
  // and append the successful ones to `out`.
  void buildEpochs(const std::vector<RoverBaseEpochMatcher::Pair>& pairs,
                   const Eigen::Vector3d* approx_rover,
                   std::vector<PreprocessedEpoch>& out);

  // Rover-base cycle-slip detection: sets slip / cp_excluded on out.rover_sats
  // (see Config::detect_slip_*). obs holds rover (rcv=1) then base (rcv=2)
  // entries; rover_idx/base_idx map satellite -> obs index for each receiver.
  // `tow` is CONTINUOUS GPST seconds (week*604800+tow), so outage/Doppler time
  // differences stay valid across the week rollover.
  void detectSlips(const std::vector<obsd_t>& obs,
                   const std::map<int, int>& rover_idx,
                   const std::map<int, int>& base_idx, double tow,
                   PreprocessedEpoch& out);

  Config config_;
  FrequencyMask freq_mask_;
  prcopt_t spp_opt_;

  EphemerisStore eph_store_;
  nav_t nav_{};
  RoverBaseEpochMatcher matcher_;

  Eigen::Vector3d last_apriori_{0, 0, 0};
  bool last_apriori_valid_{false};
  bool flush_pending_{false};  // EOF flush requested before ephemeris arrived

  // Previous-epoch carrier-tracking state per satellite, for cycle-slip
  // detection (detectSlips). Arrays indexed by RTKLIB band (0=L1,1=L2,2=L5).
  struct SlipTrack {
    double lrov[NFREQ]{};           // rover carrier phase [cycles]
    double cmc[NFREQ]{};            // SD code-minus-carrier [m]
    double last_seen[NFREQ]{};      // last time the ROVER carrier was present [s]
    double base_tow[NFREQ]{};       // BASE epoch's own time when last present
                                    // [GPST cont. s] (not the rover clock, so the
                                    // 1 Hz base reuse across rover epochs and real
                                    // base outages are both measured correctly)
    std::uint8_t code_rov[NFREQ]{}, code_base[NFREQ]{}, lli_rov[NFREQ]{},
        lli_base[NFREQ]{};
    bool lval[NFREQ]{}, cval[NFREQ]{}, codeval[NFREQ]{};
    bool lval_base[NFREQ]{};        // a base carrier has been seen before
    double sdgf{0.0};
    bool sdgf_val{false};
  };
  std::map<int, SlipTrack> slip_track_;
};

}  // namespace gnss_utils

#endif  // GNSS_ROS_STANDARDIZATION_GNSS_PREPROCESSOR_HPP
