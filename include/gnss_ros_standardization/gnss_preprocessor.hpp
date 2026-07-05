// SPDX-License-Identifier: MIT
#ifndef GNSS_ROS_STANDARDIZATION_GNSS_PREPROCESSOR_HPP
#define GNSS_ROS_STANDARDIZATION_GNSS_PREPROCESSOR_HPP

#include <cstdint>
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
  bool slip{false};      // LLI & 0x3 on this band -> re-key the ambiguity
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
  uint8_t code_ref{0}, code_tar{0};
  double lam{0.0};       // [m]; identical for ref/tar (enforced; GLONASS FDMA
                         // pairs with different FCN wavelengths drop carrier)

  bool has_pr{false};    // all four pseudoranges present
  bool has_cp{false};    // all four carrier phases present

  // Raw observables [m]. Carrier phase already converted to meters.
  double pr_rov_ref{0}, pr_base_ref{0}, pr_rov_tar{0}, pr_base_tar{0};
  double cp_rov_ref{0}, cp_base_ref{0}, cp_rov_tar{0}, cp_base_tar{0};

  // Satellite ECEF positions [m] at signal transmission time, per receiver
  // epoch. No Sagnac pre-rotation, no clock correction applied (cancels in DD).
  Eigen::Vector3d sat_ref_rov{0, 0, 0}, sat_tar_rov{0, 0, 0};
  Eigen::Vector3d sat_ref_base{0, 0, 0}, sat_tar_base{0, 0, 0};

  double el_ref{0.0}, el_tar{0.0};   // rover elevation [rad]
  double snr_ref{0.0}, snr_tar{0.0}; // rover SNR [dBHz]
  bool slip_ref{false}, slip_tar{false};  // LLI on either receiver
};

// One matched rover/base epoch after preprocessing. Emitted epochs always
// carry a valid rover a-priori position (SPP, or the caller-provided
// approximation when SPP fails).
struct PreprocessedEpoch {
  uint32_t week{0};
  double tow{0.0};       // rover epoch (GPST)
  double age_s{0.0};     // rover minus base epoch difference
  // Rover message header stamp (PC arrival time by default). This is what
  // consumers should pair against other ROS-stamped sensors (e.g. IMU);
  // receiver output latency then shows up as a small pairing error.
  builtin_interfaces::msg::Time stamp;

  Eigen::Vector3d base_ecef{0, 0, 0};
  Eigen::Vector3d rover_ecef_apriori{0, 0, 0};

  // Rover ECEF velocity [m/s] from undifferenced Doppler (RTKLIB estvel-style
  // least squares: receiver velocity + clock drift). Used to constrain
  // consecutive position states with a loosely-coupled motion factor.
  Eigen::Vector3d rover_vel_ecef{0, 0, 0};
  bool rover_vel_valid{false};
  double rover_vel_var{0.0};  // isotropic velocity variance [(m/s)^2]

  std::vector<SatObs> rover_sats;  // undifferenced, masks applied
  std::vector<DdSignal> dd;        // empty when age_s exceeds max_age_s
};

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
    double snr_mask_dbhz{0.0};        // 0 disables the flat SNR mask
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
    double max_age_s{1.0};            // DD pairs only when age <= this
    double max_tdiff_s{30.0};         // epoch matcher window
    double match_tol_s{0.001};
    Eigen::Vector3d base_ecef{0, 0, 0};  // required for DD output
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

  // Rover epochs dropped by the matcher since the last call.
  std::vector<RoverBaseEpochMatcher::Dropped> takeDropped() {
    return matcher_.takeDropped();
  }

  bool hasEphemeris() const { return nav_.n > 0 || nav_.ng > 0; }

 private:
  bool buildEpoch(const ObsMsg& rover, const ObsMsg& base, double age_s,
                  const Eigen::Vector3d* approx_rover, PreprocessedEpoch& out);

  Config config_;
  FrequencyMask freq_mask_;
  prcopt_t spp_opt_;

  EphemerisStore eph_store_;
  nav_t nav_{};
  RoverBaseEpochMatcher matcher_;

  Eigen::Vector3d last_apriori_{0, 0, 0};
  bool last_apriori_valid_{false};
};

}  // namespace gnss_utils

#endif  // GNSS_ROS_STANDARDIZATION_GNSS_PREPROCESSOR_HPP
