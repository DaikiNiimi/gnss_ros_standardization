// SPDX-License-Identifier: MIT
//
// Offline-analysis dump of the ambiguity-resolution state, written as it is at
// the moment LAMBDA is called: the FLOAT solution (antenna position + its
// covariance, the float DD ambiguities and their covariance Qa) alongside the
// integer decision that followed. This is the ground on which an AR failure is
// diagnosed - a wrong fix is either a biased float or an over-confident
// covariance, and the two are only separable when both are recorded.
//
// With the truth trajectory the dump supports the standard covariance
// consistency check: recover the true integers from the truth position, form
// eps = a_float - N_true, and compare eps^T Qa^-1 eps against chi-square(k). A
// median chi2/k far above 1 means Qa is optimistic and the ratio test is
// deciding on a covariance the estimator has not earned.
//
// Disabled unless a directory is configured (debug.ar_dump_dir), so it costs
// nothing in normal operation.
#ifndef EXAMPLES_TIGHTLY_COUPLED_FGO_AR_DEBUG_DUMP_HPP
#define EXAMPLES_TIGHTLY_COUPLED_FGO_AR_DEBUG_DUMP_HPP

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <fstream>
#include <string>
#include <vector>

#include "factor_adapters.hpp"
#include "gnss_ros_standardization/gnss_preprocessor.hpp"

namespace gnss_fgo {

// Fixed-decimal CSV formatting, shared by this dump and the node's IMU
// diagnostics. NOT optional for GPS time-of-week: ostream's default is 6
// SIGNIFICANT digits, so a tow of 550381.2 prints as "550381" and every epoch
// inside a second collapses onto one key. That silently destroyed the 1:1
// pairing with the reference trajectory in the IMU diagnostics until it was
// caught; keeping one helper for both writers is what prevents a repeat.
inline std::string fixedN(double v, int n) {
  if (!std::isfinite(v)) return "";
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.*f", n, v);
  return buf;
}
inline std::string fixed1(double v) { return fixedN(v, 1); }
inline std::string fixed3(double v) { return fixedN(v, 3); }
inline std::string fixed4(double v) { return fixedN(v, 4); }
inline std::string fixed6(double v) { return fixedN(v, 6); }


// Writes three epoch-aligned CSVs (join on week+tow):
//   <prefix>_ar_epochs.csv  one row per epoch: float position + covariance,
//                           the ratio, and the decision
//   <prefix>_ar_dd.csv      one row per DD in the LAMBDA set: identity,
//                           a_float, sqrt(Qa_ii), a_fix, and the geometry
//                           needed to re-linearize at the truth position
//   <prefix>_ar_qa.csv      one row per epoch: the full Qa lower triangle
class ArDebugDumper {
 public:
  // `dir` empty -> disabled. `prefix` separates the two nodes' files.
  ArDebugDumper(const std::string& dir, const std::string& prefix) {
    if (dir.empty()) return;
    epochs_.open(dir + "/" + prefix + "_ar_epochs.csv");
    dd_.open(dir + "/" + prefix + "_ar_dd.csv");
    qa_.open(dir + "/" + prefix + "_ar_qa.csv");
    enabled_ = epochs_.is_open() && dd_.is_open() && qa_.is_open();
    if (!enabled_) return;
    epochs_ << "week,tow,n_dd,n_lambda,ratio,candidate,fixed,hold_conditioned_float,"
               "float_x,float_y,float_z,"
               "cov_xx,cov_xy,cov_xz,cov_yy,cov_yz,cov_zz,"
               "fix_x,fix_y,fix_z,graph_x,graph_y,graph_z,n_holds,nfix,"
               // Why this epoch did not fix, and the quantities that decide it.
               // ratio = s1/s0: s0 large means the float sits far from ANY
               // lattice point (float quality); s1 small means a competing
               // candidate is nearly as good (geometry). n_gated is the pre-fit
               // FDE's rejection count for the epoch.
               "fail,n_eligible,n_excl_half,n_excl_lock,n_excl_nokey,"
               "n_partial_dropped,pos_var,s0,s1,n_gated,"
               // Post-fit FIX validation statistic (see the validation block in
               // resolveAmbiguitiesPosterior). Recorded whether or not it
               // rejected, and with its own DOF, so the threshold can be swept
               // against truth offline instead of by re-running the course.
               "postfit_chi2,postfit_dof,postfit_max_std,"
               // Positions no carrier ambiguity can move: the standalone SPP
               // fix and the code-DD weighted least squares. Every other
               // position here is a function of the graph state, so a wrong
               // fix is self-consistent with all of them; these two are the
               // only columns that can contradict it. Required for any offline
               // study of an escape statistic.
               "spp_x,spp_y,spp_z,spp_valid,spp_nsat,"
               "ddwls_x,ddwls_y,ddwls_z,ddwls_valid,ddwls_ndd,"
               "ddwls_nrej,ddwls_resid_rms\n";
    dd_ << "week,tow,i,satid_ref,satid_tar,sys,band,code_ref,code_tar,"
           "el_ref_deg,el_tar_deg,snr_ref,snr_tar,lock,fresh,half_cycle,lam,"
           "a_float,qa_sigma,a_fix,resid_cp_float_m,resid_pr_float_m,hx,hy,hz\n";
    qa_ << "week,tow,k,qa_lower\n";
  }

  bool enabled() const { return enabled_; }

  static const char* arFailName(ArFail f) {
    switch (f) {
      case ArFail::None: return "none";
      case ArFail::NoPairs: return "no_pairs";
      case ArFail::TooFewEligible: return "too_few_eligible";
      case ArFail::PosVarGate: return "pos_var_gate";
      case ArFail::ElevMaskTooFew: return "elev_mask_too_few";
      case ArFail::LambdaNumeric: return "lambda_numeric";
      case ArFail::QaNotPd: return "qa_not_pd";
      case ArFail::RatioBelowThreshold: return "ratio_below";
      case ArFail::ConditioningFailed: return "conditioning_failed";
      case ArFail::FdeReject: return "fde_reject";
      case ArFail::CodeGrowthReject: return "code_growth_reject";
    }
    return "unknown";
  }

  void writeEpoch(std::uint32_t week, double tow,
                  const gnss_utils::PreprocessedEpoch& ep,
                  const std::vector<DdAmbiguityPair>& pairs,
                  const ArResult& res, std::size_t n_holds, int nfix,
                  const Eigen::Vector3d& graph_ant = Eigen::Vector3d::Zero(),
                  int n_gated = 0,
                  const CodeDdSolution& ddwls = CodeDdSolution{}) {
    if (!enabled_) return;
    const int k = static_cast<int>(res.a_float.size());

    epochs_ << week << ',' << fixed6(tow) << ',' << ep.dd.size() << ',' << k
            << ',' << res.ratio << ',' << (res.candidate ? 1 : 0) << ','
            << (res.fixed ? 1 : 0) << ','
            << (!res.fixed && n_holds > 0 ? 1 : 0) << ',';
    if (res.float_valid) {
      epochs_ << fixed4(res.float_ant_pos.x()) << ','
              << fixed4(res.float_ant_pos.y()) << ','
              << fixed4(res.float_ant_pos.z()) << ','
              << res.float_ant_cov(0, 0) << ',' << res.float_ant_cov(0, 1)
              << ',' << res.float_ant_cov(0, 2) << ','
              << res.float_ant_cov(1, 1) << ',' << res.float_ant_cov(1, 2)
              << ',' << res.float_ant_cov(2, 2) << ',';
    } else {
      epochs_ << ",,,,,,,,,";
    }
    if (res.candidate) {
      epochs_ << fixed4(res.fixed_pos.x()) << ',' << fixed4(res.fixed_pos.y())
              << ',' << fixed4(res.fixed_pos.z()) << ',';
    } else {
      epochs_ << ",,,";
    }
    // Graph (stage-2) float antenna, for the graph-vs-AR float divergence (A2).
    epochs_ << fixed4(graph_ant.x()) << ',' << fixed4(graph_ant.y()) << ','
            << fixed4(graph_ant.z()) << ',';
    epochs_ << n_holds << ',' << nfix << ',' << arFailName(res.fail)
            << ',' << res.n_eligible << ',' << res.n_excl_half
            << ',' << res.n_excl_lock << ',' << res.n_excl_nokey
            << ',' << res.n_partial_dropped << ',' << res.pos_var
            << ',' << res.s0 << ',' << res.s1 << ',' << n_gated << ','
            << res.postfit_chi2 << ',' << res.postfit_dof << ','
            << res.postfit_max_standard << ',';
    // The two graph-independent positions (blank when unavailable).
    if (ep.rover_spp_valid) {
      epochs_ << fixed4(ep.rover_ecef_spp.x()) << ','
              << fixed4(ep.rover_ecef_spp.y()) << ','
              << fixed4(ep.rover_ecef_spp.z()) << ",1,"
              << ep.rover_spp_nsat << ',';
    } else {
      epochs_ << ",,,0,0,";
    }
    if (ddwls.ok) {
      epochs_ << fixed4(ddwls.pos.x()) << ',' << fixed4(ddwls.pos.y()) << ','
              << fixed4(ddwls.pos.z()) << ",1," << ddwls.n_dd << ','
              << ddwls.n_rejected << ',' << ddwls.resid_rms << '\n';
    } else {
      epochs_ << ",,,0,0,0,\n";
    }
    epochs_.flush();

    if (k == 0) return;

    // Per-DD rows. The line-of-sight difference (hx,hy,hz) and the residual at
    // the float position let an offline tool re-evaluate each DD at the truth
    // position without re-implementing the geometry:
    //   a_true = (resid_cp_float_m - h . (X_true - X_float)) / lam
    for (int i = 0; i < k; ++i) {
      if (i >= static_cast<int>(res.cp_dd_index.size())) break;
      const int di = res.cp_dd_index[i];
      if (di < 0 || di >= static_cast<int>(ep.dd.size())) continue;
      const gnss_utils::DdSignal& d = ep.dd[di];

      // Pair-level bookkeeping (lock / fresh) lives in `pairs`, keyed by the
      // same ep.dd index.
      int lock = -1;
      int fresh = -1;
      for (const auto& p : pairs) {
        if (p.dd_index == di) {
          lock = p.lock;
          fresh = p.fresh ? 1 : 0;
          break;
        }
      }

      Eigen::Vector3d e_ref, e_tar, etmp;
      const Eigen::Vector3d& x =
          res.float_valid ? res.float_ant_pos : ep.rover_ecef_apriori;
      detail::geodistSagnac(d.sat_ref_rov, x, &e_ref);
      detail::geodistSagnac(d.sat_tar_rov, x, &e_tar);
      detail::geodistSagnac(d.sat_ref_base, ep.base_ecef, &etmp);
      const Eigen::Vector3d h = e_tar - e_ref;
      // Zero-ambiguity carrier residual at the float position.
      const double resid = detail::ddCarrierResidual(d, ep.base_ecef, x, 0.0);
      // Code DD residual at the same position. Dumped because it is the only
      // observable that an ambiguity error CANNOT hide in: re-evaluated at the
      // fixed position (subtract h . (X_fix - X_float)) it is the one statistic
      // that stays sensitive to a wrong integer after the state has absorbed
      // it. The carrier post-fit residual does not: its sense is INVERTED,
      // because a wrong integer over-fits and drives that residual DOWN.
      const double resid_pr =
          d.has_pr ? detail::ddPseudorangeResidual(d, ep.base_ecef, x)
                   : std::numeric_limits<double>::quiet_NaN();

      const double qa_sig =
          (i < res.Qa.rows() && res.Qa(i, i) >= 0.0) ? std::sqrt(res.Qa(i, i))
                                                     : -1.0;
      dd_ << week << ',' << fixed6(tow) << ',' << i << ',' << d.satid_ref << ','
          << d.satid_tar << ',' << d.sys << ',' << d.band << ','
          << static_cast<int>(d.code_ref) << ','
          << static_cast<int>(d.code_tar) << ',' << fixed3(d.el_ref * R2D)
          << ',' << fixed3(d.el_tar * R2D) << ',' << fixed1(d.snr_ref) << ','
          << fixed1(d.snr_tar) << ',' << lock << ',' << fresh << ','
          << ((d.half_cycle_ref || d.half_cycle_tar) ? 1 : 0) << ','
          << fixed6(d.lam) << ',' << fixed6(res.a_float(i)) << ','
          << fixed6(qa_sig) << ',';
      if (i < res.a_fix.size() && std::isfinite(res.a_fix(i))) {
        dd_ << fixed6(res.a_fix(i));
      }
      dd_ << ',' << fixed6(resid) << ',';
      if (std::isfinite(resid_pr)) dd_ << fixed6(resid_pr);
      dd_ << ',' << fixed6(h.x()) << ',' << fixed6(h.y()) << ','
          << fixed6(h.z()) << '\n';
    }
    dd_.flush();

    // Qa lower triangle, semicolon-separated in one field (row-major by row i,
    // columns j <= i), so the epoch stays one line regardless of k.
    qa_ << week << ',' << fixed6(tow) << ',' << k << ',';
    for (int i = 0; i < k; ++i) {
      for (int j = 0; j <= i; ++j) {
        if (i || j) qa_ << ';';
        qa_ << res.Qa(i, j);
      }
    }
    qa_ << '\n';
    qa_.flush();
  }

 private:
  bool enabled_{false};
  std::ofstream epochs_, dd_, qa_;
};

}  // namespace gnss_fgo

#endif  // EXAMPLES_TIGHTLY_COUPLED_FGO_AR_DEBUG_DUMP_HPP
