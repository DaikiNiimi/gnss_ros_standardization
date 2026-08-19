// SPDX-License-Identifier: MIT
//
// Unit tests for the carried-ambiguity / fix-and-hold machinery in
// examples/tightly_coupled_fgo/factor_adapters.hpp. These exercise the
// state-transition and transaction logic that a bag replay cannot deterministically
// reproduce: per-epoch re-key de-duplication, hold-pair normalization and sign,
// incremental (non-stacking) hold management, the integer-mismatch transaction,
// and rollback on a failed ISAM2 update.
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>

#include "factor_adapters.hpp"
#include "imu_factors.hpp"

namespace gf = gnss_fgo;
namespace gu = gnss_utils;

namespace {

// Obtain a carried ambiguity key, inserting its initial value into v. The gauge
// prior is NOT added here - addGroupedDdFactorsImpl owns it (see the gauge tests
// below), because only there is a DD group's complete key set known.
gtsam::Key obtain(gf::PersistentAmbiguities& amb, int sat, int band, bool slip,
                  double tow, gtsam::Values& v) {
  return amb.obtain(sat, band, 0.0, slip, tow, v);
}

// Count the live (non-removed) factors in an ISAM2.
std::size_t liveFactors(const gtsam::ISAM2& isam) {
  std::size_t n = 0;
  for (const auto& f : isam.getFactorsUnsafe())
    if (f) ++n;
  return n;
}

}  // namespace

// --- PersistentAmbiguities: per-epoch re-key de-duplication (fix #1/#2) -------

TEST(PersistentAmbiguities, ReuseAcrossEpochsWithoutSlip) {
  std::uint64_t next = 0;
  gf::PersistentAmbiguities amb(next);
  gtsam::NonlinearFactorGraph g;
  gtsam::Values v;
  amb.beginEpoch();
  const gtsam::Key k1 = obtain(amb, 5, 0, /*slip=*/false, 100.0, v);
  gtsam::NonlinearFactorGraph g2;
  gtsam::Values v2;
  amb.beginEpoch();
  const gtsam::Key k2 = obtain(amb, 5, 0, /*slip=*/false, 101.0, v2);
  EXPECT_EQ(k1, k2);
  EXPECT_EQ(v2.size(), 0u);  // no new value/prior on reuse
}

TEST(PersistentAmbiguities, ReferenceReKeyedOncePerEpoch) {
  // The reference satellite appears in several DDs (obtain called repeatedly with
  // slip=true). It must re-key at most ONCE per epoch, sharing one key.
  std::uint64_t next = 0;
  gf::PersistentAmbiguities amb(next);
  gtsam::NonlinearFactorGraph g0;
  gtsam::Values v0;
  amb.beginEpoch();
  const gtsam::Key k0 = obtain(amb, 5, 0, /*slip=*/false, 100.0, v0);

  gtsam::NonlinearFactorGraph g1;
  gtsam::Values v1;
  amb.beginEpoch();
  const gtsam::Key a = obtain(amb, 5, 0, /*slip=*/true, 101.0, v1);
  const gtsam::Key b = obtain(amb, 5, 0, /*slip=*/true, 101.0, v1);
  const gtsam::Key c = obtain(amb, 5, 0, /*slip=*/true, 101.0, v1);
  EXPECT_EQ(a, b);
  EXPECT_EQ(b, c);
  EXPECT_NE(a, k0);          // re-keyed exactly once
  EXPECT_EQ(v1.size(), 1u);  // one new value inserted, not three
}

TEST(PersistentAmbiguities, NoReKeyWhenSlipStaysUnset) {
  // Mirrors a persistent half-cycle bit that (fix #1) no longer sets slip: the
  // key must persist across many epochs.
  std::uint64_t next = 0;
  gf::PersistentAmbiguities amb(next);
  gtsam::NonlinearFactorGraph g;
  gtsam::Values v;
  amb.beginEpoch();
  const gtsam::Key k0 = obtain(amb, 9, 0, /*slip=*/false, 0.0, v);
  for (int e = 1; e < 20; ++e) {
    gtsam::NonlinearFactorGraph ge;
    gtsam::Values ve;
    amb.beginEpoch();
    EXPECT_EQ(obtain(amb, 9, 0, /*slip=*/false, e, ve), k0);
  }
}

TEST(PersistentAmbiguities, InvalidateForcesReKey) {
  std::uint64_t next = 0;
  gf::PersistentAmbiguities amb(next);
  gtsam::NonlinearFactorGraph g;
  gtsam::Values v;
  amb.beginEpoch();
  const gtsam::Key k1 = obtain(amb, 7, 1, /*slip=*/false, 1.0, v);
  amb.invalidate(7, 1);
  gtsam::Key probe;
  EXPECT_FALSE(amb.tryKey(7, 1, probe));  // entry gone
  amb.beginEpoch();
  const gtsam::Key k2 = obtain(amb, 7, 1, /*slip=*/false, 2.0, v);
  EXPECT_NE(k1, k2);
}

// --- collectHoldSpecs: pair normalization + BetweenFactor sign ---------------

namespace {
// Build a one-carrier-DD epoch and an AR result that fixed it to `integer`
// (= N_ref - N_tar).
gu::PreprocessedEpoch oneDdEpoch(int sat_ref, int sat_tar, int band) {
  gu::PreprocessedEpoch ep;
  gu::DdSignal d;
  d.sat_ref = sat_ref;
  d.sat_tar = sat_tar;
  d.band = band;
  d.has_cp = true;
  ep.dd.push_back(d);
  return ep;
}
gf::ArResult fixedRes(long integer) {
  gf::ArResult r;
  r.fixed = true;
  r.cp_dd_index = {0};
  r.a_fix.resize(1);
  r.a_fix(0) = static_cast<double>(integer);
  return r;
}
}  // namespace

TEST(CollectHoldSpecs, NormalizesPairAndSign) {
  std::uint64_t next = 0;
  gf::PersistentAmbiguities amb(next);
  gtsam::NonlinearFactorGraph g;
  gtsam::Values v;
  amb.beginEpoch();
  const gtsam::Key k8 = obtain(amb, 8, 0, false, 0.0, v);
  const gtsam::Key k3 = obtain(amb, 3, 0, false, 0.0, v);

  // DD ref=8, tar=3, N_8 - N_3 = 4.
  const auto specs =
      gf::collectHoldSpecs(amb, oneDdEpoch(8, 3, 0), fixedRes(4));
  ASSERT_EQ(specs.size(), 1u);
  EXPECT_EQ(specs[0].sat_min, 3);
  EXPECT_EQ(specs[0].sat_max, 8);
  EXPECT_EQ(specs[0].k_min, k3);
  EXPECT_EQ(specs[0].k_max, k8);
  // n_min_minus_max = N_3 - N_8 = -4.
  EXPECT_EQ(specs[0].integer, -4);
}

TEST(CollectHoldSpecs, ReversedSatOrderGivesSameNormalizedEntry) {
  std::uint64_t next = 0;
  gf::PersistentAmbiguities amb(next);
  gtsam::NonlinearFactorGraph g;
  gtsam::Values v;
  amb.beginEpoch();
  obtain(amb, 8, 0, false, 0.0, v);
  obtain(amb, 3, 0, false, 0.0, v);
  // (ref=3, tar=8, N_3 - N_8 = -4) must normalize identically to (8,3,+4).
  const auto s1 = gf::collectHoldSpecs(amb, oneDdEpoch(8, 3, 0), fixedRes(4));
  const auto s2 = gf::collectHoldSpecs(amb, oneDdEpoch(3, 8, 0), fixedRes(-4));
  ASSERT_EQ(s1.size(), 1u);
  ASSERT_EQ(s2.size(), 1u);
  EXPECT_EQ(s1[0].sat_min, s2[0].sat_min);
  EXPECT_EQ(s1[0].sat_max, s2[0].sat_max);
  EXPECT_EQ(s1[0].integer, s2[0].integer);
}

TEST(CollectHoldSpecs, RequiresPairSpecificLockBeforeHold) {
  std::uint64_t next = 0;
  gf::PersistentAmbiguities amb(next);
  gtsam::NonlinearFactorGraph g;
  gtsam::Values v;
  amb.beginEpoch();
  obtain(amb, 8, 0, false, 0.0, v);
  obtain(amb, 3, 0, false, 0.0, v);

  // A global FIX streak may already be long, but a pair first seen this epoch
  // must not immediately inherit that history and receive a tight hold.
  EXPECT_TRUE(gf::collectHoldSpecs(
      amb, oneDdEpoch(8, 3, 0), fixedRes(4), 0.5, 1).empty());

  gtsam::NonlinearFactorGraph g1;
  gtsam::Values v1;
  amb.beginEpoch();
  obtain(amb, 8, 0, false, 1.0, v1);
  obtain(amb, 3, 0, false, 1.0, v1);
  EXPECT_EQ(gf::collectHoldSpecs(
                amb, oneDdEpoch(8, 3, 0), fixedRes(4), 0.5, 1)
                .size(),
            1u);
}

// --- applyHolds: incremental / transactional behaviour -----------------------

namespace {
// A tiny fixture: a fixed-lag smoother (huge lag = full history, so nothing is
// marginalized in these unit tests) holding ambiguity keys for a set of
// (sat,band=0), plus the manager that owns them.
struct HoldFixture {
  std::uint64_t next = 0;
  gf::PersistentAmbiguities amb{next};
  gtsam::IncrementalFixedLagSmoother isam{1.0e9};
  std::map<int, gtsam::Key> key;

  explicit HoldFixture(const std::vector<int>& sats) {
    gtsam::NonlinearFactorGraph g;
    gtsam::Values v;
    amb.beginEpoch();
    for (int s : sats) key[s] = obtain(amb, s, 0, false, 0.0, v);
    // These unit tests exercise hold bookkeeping in isolation, with no DD
    // factors, so every ambiguity here is unobservable. Pin each one directly
    // (the production gauge - exactly one prior per DD group - is built by
    // addGroupedDdFactorsImpl and is covered by the GaugeInvariant tests).
    for (const auto& kv : key) {
      g.add(gtsam::PriorFactor<double>(
          kv.second, 0.0, gtsam::noiseModel::Isotropic::Sigma(1, 20.0)));
    }
    gtsam::FixedLagSmoother::KeyTimestampMap ts;
    for (const gtsam::Key k : v.keys()) ts[k] = 0.0;
    isam.update(g, v, ts);
  }
  gf::HoldSpec spec(int a, int b, long n_min_minus_max) {
    return gf::HoldSpec{0, a, b, key.at(a), key.at(b), n_min_minus_max};
  }
};
}  // namespace

TEST(ApplyHolds, AddsOnceAndDoesNotStack) {
  HoldFixture fx({3, 8});
  gf::HeldDdMap held;
  const std::vector<gf::HoldSpec> specs{fx.spec(3, 8, -4)};

  EXPECT_EQ(gf::applyHolds(fx.isam, fx.amb, held, specs, 0.03),
            gf::HoldResult::Success);
  ASSERT_EQ(held.size(), 1u);
  const std::size_t after_add = liveFactors(fx.isam.getISAM2());
  const std::size_t idx = held.begin()->second.factor_index;

  // Re-applying the SAME (survivor) spec must not add another factor (NoChange).
  for (int i = 0; i < 5; ++i)
    EXPECT_EQ(gf::applyHolds(fx.isam, fx.amb, held, specs, 0.03),
              gf::HoldResult::NoChange);
  EXPECT_EQ(held.size(), 1u);
  EXPECT_EQ(liveFactors(fx.isam.getISAM2()), after_add);        // no stacking
  EXPECT_EQ(held.begin()->second.factor_index, idx);  // same factor kept
}

TEST(ApplyHolds, ReKeyRemovesStaleHold) {
  HoldFixture fx({3, 8});
  gf::HeldDdMap held;
  gf::applyHolds(fx.isam, fx.amb, held, {fx.spec(3, 8, -4)}, 0.03);
  ASSERT_EQ(held.size(), 1u);

  // Satellite 8 re-keys (slip); the stale hold must be removed on the next call.
  fx.amb.invalidate(8, 0);
  gf::applyHolds(fx.isam, fx.amb, held, {}, 0.03);
  EXPECT_TRUE(held.empty());
}

TEST(ApplyHolds, IntegerMismatchRemovesAllSuspectHoldsAndReKeys) {
  HoldFixture fx({3, 8, 9});
  gf::HeldDdMap held;
  // Two holds that share satellite 8.
  gf::applyHolds(fx.isam, fx.amb, held,
                 {fx.spec(3, 8, -4), fx.spec(8, 9, 2)}, 0.03);
  ASSERT_EQ(held.size(), 2u);

  // AR now reports a DIFFERENT integer for (3,8): an anomaly. Both holds that
  // touch a suspect satellite (3,8 -> suspects {3,8}; also 8-9) must be removed,
  // and satellites 3 and 8 re-keyed. The new integer is NOT adopted.
  gf::applyHolds(fx.isam, fx.amb, held, {fx.spec(3, 8, -5)}, 0.03);
  EXPECT_TRUE(held.empty());  // (3,8) and (8,9) both gone
  gtsam::Key probe;
  EXPECT_FALSE(fx.amb.tryKey(3, 0, probe));  // re-keyed
  EXPECT_FALSE(fx.amb.tryKey(8, 0, probe));
  EXPECT_TRUE(fx.amb.tryKey(9, 0, probe));   // untouched
}

TEST(HoldConfirmation, RequiresSamePairIntegerAndGenerationConsecutively) {
  gf::HoldCandidateMap candidates;
  const gf::HoldSpec a{0, 3, 8, gtsam::symbol_shorthand::N(1),
                       gtsam::symbol_shorthand::N(2), -4};
  EXPECT_TRUE(gf::confirmHoldSpecs(candidates, {a}, 3).empty());
  EXPECT_TRUE(gf::confirmHoldSpecs(candidates, {a}, 3).empty());
  EXPECT_EQ(gf::confirmHoldSpecs(candidates, {a}, 3).size(), 1u);

  gf::HoldSpec changed = a;
  changed.integer = -5;
  EXPECT_TRUE(gf::confirmHoldSpecs(candidates, {changed}, 3).empty());
  changed.integer = -4;
  changed.k_max = gtsam::symbol_shorthand::N(20);
  EXPECT_TRUE(gf::confirmHoldSpecs(candidates, {changed}, 3).empty());
  EXPECT_TRUE(gf::confirmHoldSpecs(candidates, {}, 3).empty());
  EXPECT_TRUE(candidates.empty());
}

TEST(ApplyHolds, RejectsInconsistentIntegerCycleAndRollsBackComponent) {
  HoldFixture fx({1, 2, 3});
  gf::HeldDdMap held;
  ASSERT_EQ(gf::applyHolds(
                fx.isam, fx.amb, held,
                {fx.spec(1, 2, 3), fx.spec(2, 3, 4)}, 0.03),
            gf::HoldResult::Success);
  ASSERT_EQ(held.size(), 2u);

  // Existing path implies N1-N3=7. The consistent closing edge is redundant.
  EXPECT_EQ(gf::applyHolds(fx.isam, fx.amb, held, {fx.spec(1, 3, 7)}, 0.03),
            gf::HoldResult::NoChange);
  EXPECT_EQ(held.size(), 2u);

  // A contradictory closure invalidates and removes the full connected cycle.
  EXPECT_EQ(gf::applyHolds(fx.isam, fx.amb, held, {fx.spec(1, 3, 8)}, 0.03),
            gf::HoldResult::Success);
  EXPECT_TRUE(held.empty());
  gtsam::Key probe;
  EXPECT_FALSE(fx.amb.tryKey(1, 0, probe));
  EXPECT_FALSE(fx.amb.tryKey(2, 0, probe));
  EXPECT_FALSE(fx.amb.tryKey(3, 0, probe));
}

TEST(PersistentAmbiguities, ObtainInsertsOnlyTheInitialValueNotAGaugePrior) {
  std::uint64_t next = 0;
  gf::PersistentAmbiguities amb(next);
  gtsam::Values values;
  amb.beginEpoch();
  const gtsam::Key k =
      amb.obtain(5, 0, 123.0, false, 0.0, values,
                 gf::AmbiguityGroupId{SYS_GPS, 0, CODE_L1C, CODE_L1C});
  // CMC supplies the nonlinear starting point only.
  EXPECT_EQ(values.size(), 1u);
  EXPECT_DOUBLE_EQ(values.at<double>(k), 123.0);
  EXPECT_FALSE(amb.isAnchored(k));  // the gauge is decided at factor build time
}

TEST(GroupedDdCovariance, RetainsReferenceCorrelationAndBaseReuseInflation) {
  gu::DdSignal a, b;
  a.el_ref = b.el_ref = 0.8;
  a.el_base_ref = b.el_base_ref = 0.7;
  a.el_tar = 0.6;
  b.el_tar = 0.5;
  a.el_base_tar = 0.55;
  b.el_base_tar = 0.45;
  a.model_var_ref_sd = b.model_var_ref_sd = 0.04;
  a.model_var_tar_sd = 0.01;
  b.model_var_tar_sd = 0.02;
  gf::AdapterConfig cfg;
  cfg.elevation_weighting = true;
  cfg.base_reuse_factor = 5.0;
  cfg.robust = false;
  const Eigen::MatrixXd R = gf::groupedDdCovariance({a, b}, 0.5, cfg);
  const double vref = gf::ddSingleDifferenceVar(a, true, 0.5, cfg);
  EXPECT_NEAR(R(0, 1), vref, 1e-12);
  EXPECT_NEAR(R(1, 0), vref, 1e-12);
  EXPECT_NEAR(R(0, 0),
              vref + gf::ddSingleDifferenceVar(a, false, 0.5, cfg), 1e-12);
  EXPECT_GT(R(1, 1), R(0, 1));
}

TEST(ResolveAmbiguitiesPosterior, FullStateConditioningMatchesBatchFormula) {
  gf::GraphArPosterior p;
  constexpr int m = 5;
  p.ant = Eigen::Vector3d(10.0, -20.0, 30.0);
  p.n.resize(m);
  p.n << 10.04, 7.02, 4.01, 1.03, -1.02;
  p.n_cov = Eigen::MatrixXd::Identity(m, m) * 0.002;
  p.n_cov.array() += 0.0005;  // common SD gauge component
  p.full_cov = Eigen::MatrixXd::Identity(15, 15) * 4.0;
  p.full_n_cross = Eigen::MatrixXd::Zero(15, m);
  for (int r = 0; r < 15; ++r)
    for (int c = 0; c < m; ++c)
      p.full_n_cross(r, c) = 1e-3 * (r + 1) * (c - 2);
  p.ant_cov = p.full_cov.topLeftCorner<3, 3>();
  p.ant_n_cross = p.full_n_cross.topRows(3);

  std::vector<gf::DdAmbiguityPair> pairs;
  gu::PreprocessedEpoch ep;
  ep.dd.resize(m - 1);
  const gtsam::Key ref = gtsam::symbol_shorthand::N(100);
  p.col[ref] = 0;
  for (int i = 1; i < m; ++i) {
    const gtsam::Key tar = gtsam::symbol_shorthand::N(100 + i);
    p.col[tar] = i;
    gf::DdAmbiguityPair pair{ref, tar, 0.19, 0.9};
    pair.dd_index = i - 1;
    pair.lock = 10;
    pairs.push_back(pair);
  }

  gf::ArOptions opt;
  opt.min_fix = 4;
  opt.min_lock = 0;
  opt.el_mask_rad = 0.0;
  opt.max_pos_var_m2 = 0.0;
  opt.partial_ar = false;
  opt.fde_enable = false;
  gf::AdapterConfig cfg;
  // fde_enable is false here, so the layout is never consulted.
  const gf::DdFactorLayout layout;
  const gf::ArResult r =
      gf::resolveAmbiguitiesPosterior(ep, pairs, p, layout, cfg, opt);
  ASSERT_TRUE(r.fixed);
  ASSERT_EQ(r.full_state_delta.size(), 15);
  EXPECT_LT((r.float_ant_pos - p.ant).norm(), 1e-12);
  EXPECT_LT((r.float_ant_cov - p.ant_cov).norm(), 1e-12);

  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(m - 1, m);
  for (int i = 0; i < m - 1; ++i) {
    D(i, 0) = 1.0;
    D(i, i + 1) = -1.0;
  }
  const Eigen::VectorXd a = D * p.n;
  const Eigen::MatrixXd Qa = D * p.n_cov * D.transpose();
  EXPECT_LT((r.a_float - a).norm(), 1e-12);
  EXPECT_LT((r.Qa - Qa).norm(), 1e-12);
  const Eigen::MatrixXd Qza = p.full_n_cross * D.transpose();
  const Eigen::VectorXd expected_delta =
      -Qza * Qa.ldlt().solve(a - r.a_fix);
  const Eigen::MatrixXd expected_cov =
      p.full_cov - Qza * Qa.ldlt().solve(Qza.transpose());
  EXPECT_LT((r.full_state_delta - expected_delta).norm(), 1e-10);
  EXPECT_LT((r.full_state_cov - expected_cov).norm(), 1e-10);
  EXPECT_LT((r.fixed_pos -
             (p.ant + expected_delta.head<3>())).norm(), 1e-10);
}
TEST(ConditionedNavState, PublishedPoseReproducesFixedAntennaWithinOneMm) {
  const gtsam::Pose3 ecef_T_nav(
      gtsam::Rot3::Ypr(0.4, -0.2, 0.1),
      gtsam::Point3(3.8e6, 1.4e6, 5.0e6));
  const gtsam::Pose3 pose(
      gtsam::Rot3::Ypr(-0.8, 0.25, -0.15),
      gtsam::Point3(12.0, -7.0, 1.5));
  const gtsam::Point3 lever(1.2, -0.4, 1.8);
  const gtsam::Vector3 velocity(8.0, -0.5, 0.2);
  gf::ArResult ar;
  ar.fixed = true;
  ar.full_state_delta = Eigen::VectorXd::Zero(15);
  ar.full_state_delta.head<6>() <<
      0.35, -0.20, 0.15, 0.8, -0.4, 0.3;
  ar.full_state_delta.segment<3>(6) << 0.4, -0.2, 0.1;
  ar.full_state_cov = Eigen::MatrixXd::Identity(15, 15);
  const gtsam::Pose3 provisional =
      pose.retract(ar.full_state_delta.head<6>());
  const gtsam::Point3 provisional_ant =
      ecef_T_nav.transformFrom(provisional.transformFrom(lever));
  ar.fixed_pos =
      Eigen::Vector3d(provisional_ant.x(), provisional_ant.y(),
                      provisional_ant.z()) +
      Eigen::Vector3d(0.25, -0.18, 0.12);

  const gf::ConditionedNavState fixed = gf::makeConditionedNavState(
      pose, velocity, lever, ecef_T_nav, ar);
  ASSERT_TRUE(fixed.ok);
  const gtsam::Point3 published_ant =
      ecef_T_nav.transformFrom(fixed.pose.transformFrom(lever));
  EXPECT_LT((Eigen::Vector3d(published_ant.x(), published_ant.y(),
                            published_ant.z()) -
             ar.fixed_pos).norm(),
            1e-3);
  EXPECT_LT((fixed.velocity -
             (velocity + ar.full_state_delta.segment<3>(6))).norm(),
            1e-12);
  EXPECT_LT((fixed.pose_cov - ar.full_state_cov.topLeftCorner<6, 6>()).norm(),
            1e-12);
  EXPECT_LT((fixed.velocity_cov -
             ar.full_state_cov.block<3, 3>(6, 6)).norm(),
            1e-12);
}
// --- PreFitInnovationFde: synthetic DD geometry ----------

namespace {

// Sagnac-corrected range, identical to the model inside the AR.
double rangeSagnac(const Eigen::Vector3d& sat, const Eigen::Vector3d& rcv) {
  return (sat - rcv).norm() +
         OMGE * (sat.x() * rcv.y() - sat.y() * rcv.x()) / CLIGHT;
}

// Noise-free short-baseline scenario: one reference satellite + n_tar targets,
// used to exercise the correlated pre-fit innovation subset exclusion.
struct SyntheticDdScenario {
  Eigen::Vector3d base, rover;

  gu::PreprocessedEpoch ep;
};

SyntheticDdScenario makeSyntheticDdScenario(int n_tar) {
  SyntheticDdScenario sc;
  const double llh[3] = {35.7 * D2R, 139.72 * D2R, 74.0};
  double be[3];
  pos2ecef(llh, be);
  sc.base = Eigen::Vector3d(be[0], be[1], be[2]);
  sc.rover = sc.base + Eigen::Vector3d(5.0, 3.0, 1.0);

  double E[9];  // enu = E * d_ecef (column-major, RTKLIB layout)
  xyz2enu(llh, E);
  Eigen::Matrix3d R_enu_ecef;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) R_enu_ecef(r, c) = E[r + c * 3];
  auto sat_at = [&](double az, double el) {
    const Eigen::Vector3d enu(std::cos(el) * std::sin(az),
                              std::cos(el) * std::cos(az), std::sin(el));
    return Eigen::Vector3d(sc.rover + R_enu_ecef.transpose() * enu * 2.2e7);
  };

  const double lam = CLIGHT / FREQL1;
  const double el_ref = 80.0 * D2R;
  const Eigen::Vector3d s_ref = sat_at(0.0, el_ref);
  const double n_sd_ref = 100.0;  // SD ambiguity of the reference [cycles]

  sc.ep.week = 2200;
  sc.ep.tow = 100.0;
  sc.ep.base_ecef = sc.base;
  sc.ep.rover_ecef_apriori = sc.rover + Eigen::Vector3d(0.3, -0.2, 0.4);

  for (int i = 0; i < n_tar; ++i) {
    const double az = 2.0 * M_PI * (i + 1) / (n_tar + 1);
    const double el = (30.0 + 30.0 * i / std::max(n_tar - 1, 1)) * D2R;
    const Eigen::Vector3d s_tar = sat_at(az, el);
    const long dd = static_cast<long>(i) - 2;  // mixed-sign integers
    const double n_sd_tar = n_sd_ref - static_cast<double>(dd);

    gu::DdSignal d;
    d.sat_ref = 1;
    d.sat_tar = 2 + i;
    d.sys = SYS_GPS;
    d.band = 0;
    d.lam = lam;
    d.has_pr = true;
    d.has_cp = true;
    d.el_ref = el_ref;
    d.el_tar = el;
    d.sat_ref_rov = d.sat_ref_base = s_ref;
    d.sat_tar_rov = d.sat_tar_base = s_tar;
    d.pr_rov_ref = rangeSagnac(s_ref, sc.rover);
    d.pr_base_ref = rangeSagnac(s_ref, sc.base);
    d.pr_rov_tar = rangeSagnac(s_tar, sc.rover);
    d.pr_base_tar = rangeSagnac(s_tar, sc.base);
    d.cp_rov_ref = d.pr_rov_ref + lam * n_sd_ref;
    d.cp_base_ref = d.pr_base_ref;
    d.cp_rov_tar = d.pr_rov_tar + lam * n_sd_tar;
    d.cp_base_tar = d.pr_base_tar;
    sc.ep.dd.push_back(d);
  }
  return sc;
}

gf::ArOptions fdeOptions() {
  gf::ArOptions opt;
  opt.fde_enable = true;
  opt.fde_nsigma = 4.0;
  opt.fde_max_exclude = 2;
  return opt;
}
}  // namespace
TEST(PreFitInnovationFde, ExcludesCorruptDdUsingCorrelatedGlobalTest) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(8);
  for (auto& d : sc.ep.dd) {
    d.el_base_ref = d.el_ref;
    d.el_base_tar = d.el_tar;
  }
  const int bad_sat = sc.ep.dd[3].sat_tar;
  sc.ep.dd[3].pr_rov_tar += 30.0;
  gf::AdapterConfig cfg;
  cfg.robust = false;
  gf::ArOptions opt = fdeOptions();
  opt.fde_max_exclude = 2;
  int excluded = 0;
  const gu::PreprocessedEpoch filtered = gf::filterDdPreFitInnovation(
      sc.ep, sc.rover, Eigen::Matrix3d::Identity() * 0.01, cfg, opt,
      &excluded);
  EXPECT_EQ(excluded, 1);
  EXPECT_EQ(filtered.dd.size(), sc.ep.dd.size() - 1);
  for (const auto& d : filtered.dd) EXPECT_NE(d.sat_tar, bad_sat);
  // Clean observations seen from a good state are never a "prediction fault".
  bool fault = false;
  int e2 = 0;
  gf::filterDdPreFitInnovation(sc.ep, sc.rover,
                               Eigen::Matrix3d::Identity() * 0.01, cfg, opt,
                               &e2, &fault);
  EXPECT_FALSE(fault);
}

// The regression this whole change exists for. A multi-second IMU-only coast
// leaves the PREDICTED state metres away while the observations themselves are
// perfectly clean. The old code spent its 2-exclusion budget, concluded the
// epoch was dirty and dropped ALL DDs - which removed the only measurement that
// could correct the state, so the next epoch predicted from the same wrong
// place and dropped everything again (measured: 519 s stuck at zero carrier
// arcs). The set must survive, flagged as a prediction fault.
TEST(PreFitInnovationFde, KeepsCleanDdsWhenThePredictedStateIsTheFaultySide) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(8);
  for (auto& d : sc.ep.dd) {
    d.el_base_ref = d.el_ref;
    d.el_base_tar = d.el_tar;
  }
  // Code a-priori at the truth (single-point quality); the graph's prediction
  // has coasted 17 m up, with an over-confident covariance.
  sc.ep.rover_ecef_apriori = sc.rover;
  const Eigen::Vector3d up = sc.rover.normalized();
  const Eigen::Vector3d drifted = sc.rover + 17.0 * up;

  gf::AdapterConfig cfg;
  cfg.robust = false;
  const gf::ArOptions opt = fdeOptions();
  int excluded = 0;
  bool fault = false;
  const gu::PreprocessedEpoch filtered = gf::filterDdPreFitInnovation(
      sc.ep, drifted, Eigen::Matrix3d::Identity() * 0.01, cfg, opt, &excluded,
      &fault);
  EXPECT_TRUE(fault);
  EXPECT_EQ(excluded, 0);
  EXPECT_EQ(filtered.dd.size(), sc.ep.dd.size());
}

// The escape hatch must not become a blanket amnesty: when the observations are
// dirty at the independent reference too, the epoch is still rejected.
TEST(PreFitInnovationFde, StillRejectsDirtySetsAtTheIndependentReference) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(8);
  for (auto& d : sc.ep.dd) {
    d.el_base_ref = d.el_ref;
    d.el_base_tar = d.el_tar;
  }
  sc.ep.rover_ecef_apriori = sc.rover;
  // More gross errors than the exclusion budget can absorb.
  sc.ep.dd[1].pr_rov_tar += 40.0;
  sc.ep.dd[3].pr_rov_tar -= 55.0;
  sc.ep.dd[5].pr_rov_tar += 70.0;
  sc.ep.dd[6].pr_rov_tar -= 35.0;

  gf::AdapterConfig cfg;
  cfg.robust = false;
  const gf::ArOptions opt = fdeOptions();
  int excluded = 0;
  bool fault = false;
  const gu::PreprocessedEpoch filtered = gf::filterDdPreFitInnovation(
      sc.ep, sc.rover, Eigen::Matrix3d::Identity() * 0.01, cfg, opt, &excluded,
      &fault);
  EXPECT_FALSE(fault);
  EXPECT_TRUE(filtered.dd.empty());
}

// --- Gauge invariant: exactly one live prior per carrier DD group ------------
//
// A carrier group constrains only DIFFERENCES of its ambiguities (m rows over
// m+1 keys), so one loose prior must pin the common gauge. The old code decided
// this inside obtain() by asking "does any entry belong to this group?" BEFORE
// the re-key replaced the entry - so a member being re-keyed saw its own stale
// entry, concluded the group was anchored, and left the new key ungauged while
// the prior stayed on the abandoned one. With a 30 s arc re-key every group
// lost its gauge routinely, throwing IndeterminantLinearSystemException and
// forcing a full graph reset (measured 7-18 per run on both nodes).
namespace {
// Add one epoch of a synthetic carrier-only DD group and report how many gauge
// priors (PriorFactor<double>) the call added.
std::size_t addEpochCountingGaugePriors(gf::PersistentAmbiguities& amb,
                                        const gu::PreprocessedEpoch& ep,
                                        gtsam::NonlinearFactorGraph& graph,
                                        gtsam::Values& values) {
  const std::size_t before = graph.size();
  gf::AdapterConfig cfg;
  cfg.robust = false;
  gf::addGroupedDdFactors(ep, gtsam::symbol_shorthand::X(0), cfg, graph, values,
                          amb);
  std::size_t priors = 0;
  for (std::size_t i = before; i < graph.size(); ++i) {
    if (std::dynamic_pointer_cast<gtsam::PriorFactor<double>>(graph.at(i)))
      ++priors;
  }
  return priors;
}

// A carrier-only epoch (no pseudorange) - the configuration that actually went
// singular in the logs, because without code DDs nothing else pins the gauge.
gu::PreprocessedEpoch carrierOnlyEpoch(bool slip) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(4);
  for (auto& d : sc.ep.dd) {
    d.has_pr = false;
    d.el_base_ref = d.el_ref;
    d.el_base_tar = d.el_tar;
    d.slip_ref = slip;
    d.slip_tar = slip;
  }
  return sc.ep;
}
}  // namespace

TEST(GaugeInvariant, FirstEpochAnchorsTheGroupExactlyOnce) {
  std::uint64_t next = 0;
  gf::PersistentAmbiguities amb(next);
  gtsam::NonlinearFactorGraph g;
  gtsam::Values v;
  EXPECT_EQ(addEpochCountingGaugePriors(amb, carrierOnlyEpoch(false), g, v), 1u);
}

TEST(GaugeInvariant, AnchoredGroupIsNotAnchoredTwice) {
  std::uint64_t next = 0;
  gf::PersistentAmbiguities amb(next);
  gtsam::NonlinearFactorGraph g;
  gtsam::Values v;
  addEpochCountingGaugePriors(amb, carrierOnlyEpoch(false), g, v);
  // Second epoch, same arcs carried: the live prior still gauges the group.
  EXPECT_EQ(addEpochCountingGaugePriors(amb, carrierOnlyEpoch(false), g, v), 0u);
}

// The regression. Every member re-keys (slip), so the old anchor key is
// abandoned - the group MUST be re-gauged or it goes singular.
TEST(GaugeInvariant, FullReKeyReAnchorsTheGroup) {
  std::uint64_t next = 0;
  gf::PersistentAmbiguities amb(next);
  gtsam::NonlinearFactorGraph g;
  gtsam::Values v;
  addEpochCountingGaugePriors(amb, carrierOnlyEpoch(false), g, v);
  EXPECT_EQ(addEpochCountingGaugePriors(amb, carrierOnlyEpoch(true), g, v), 1u);
}

TEST(GaugeInvariant, RetiringTheAnchorReAnchorsTheGroup) {
  std::uint64_t next = 0;
  gf::PersistentAmbiguities amb(next);
  gtsam::NonlinearFactorGraph g;
  gtsam::Values v;
  const gu::PreprocessedEpoch ep = carrierOnlyEpoch(false);
  addEpochCountingGaugePriors(amb, ep, g, v);
  // Everything goes stale (the epoch's own continuous GPST plus a long outage).
  amb.retireStale(ep.week * 604800.0 + ep.tow + 100.0, /*max_outage_s=*/1.0);
  EXPECT_EQ(amb.liveCount(), 0u);
  EXPECT_EQ(addEpochCountingGaugePriors(amb, carrierOnlyEpoch(false), g, v), 1u);
}

// End-to-end: a carrier-only group whose members all re-key must still yield a
// NON-SINGULAR system. This is the exact failure the logs showed
// ("singular near n13; touching factors: {x1 n13 n14 n15 GroupedDdFactor}").
TEST(GaugeInvariant, CarrierOnlyGroupStaysSolvableThroughReKeyCycles) {
  std::uint64_t next = 0;
  gf::PersistentAmbiguities amb(next);
  gtsam::NonlinearFactorGraph g;
  gtsam::Values v;
  const SyntheticDdScenario sc = makeSyntheticDdScenario(4);
  v.insert(gtsam::symbol_shorthand::X(0), gtsam::Point3(sc.rover));
  // Pin the position; the ambiguities must be determined by DD + gauge alone.
  g.add(gtsam::PriorFactor<gtsam::Point3>(
      gtsam::symbol_shorthand::X(0), gtsam::Point3(sc.rover),
      gtsam::noiseModel::Isotropic::Sigma(3, 0.01)));
  for (int epoch = 0; epoch < 6; ++epoch) {
    // Re-key every arc on every other epoch, as the 30 s age cap does.
    addEpochCountingGaugePriors(amb, carrierOnlyEpoch(epoch % 2 == 1), g, v);
  }
  // Would throw IndeterminantLinearSystemException if any group lost its gauge.
  EXPECT_NO_THROW({
    const gtsam::Matrix h = g.linearize(v)->hessian().first;
    const Eigen::LDLT<gtsam::Matrix> ldlt(h);
    ASSERT_EQ(ldlt.info(), Eigen::Success);
    EXPECT_GT(ldlt.vectorD().minCoeff(), 0.0);
  });
}

// invalidateAll re-keys every carried arc (used by the re-anchor path) while
// leaving the held-set bookkeeping for applyHolds to reconcile.
TEST(PersistentAmbiguities, InvalidateAllForcesReKeyOfEveryArc) {
  std::uint64_t next = 0;
  gf::PersistentAmbiguities amb(next);
  gtsam::NonlinearFactorGraph g;
  gtsam::Values v;
  amb.beginEpoch();
  const gtsam::Key a1 = obtain(amb, 5, 0, /*slip=*/false, 100.0, v);
  const gtsam::Key b1 = obtain(amb, 6, 0, /*slip=*/false, 100.0, v);
  EXPECT_EQ(amb.liveCount(), 2u);

  amb.invalidateAll();
  EXPECT_EQ(amb.liveCount(), 0u);

  gtsam::NonlinearFactorGraph g2;
  gtsam::Values v2;
  amb.beginEpoch();
  const gtsam::Key a2 = obtain(amb, 5, 0, /*slip=*/false, 100.2, v2);
  const gtsam::Key b2 = obtain(amb, 6, 0, /*slip=*/false, 100.2, v2);
  EXPECT_NE(a1, a2);
  EXPECT_NE(b1, b2);
  EXPECT_EQ(amb.lockCount(5, 0), 0);  // arc restarts, so AR's min_lock applies
}


TEST(ApplyHolds, FailedUpdateLeavesStateUnchanged) {
  HoldFixture fx({3, 8});
  gf::HeldDdMap held;
  gf::applyHolds(fx.isam, fx.amb, held, {fx.spec(3, 8, -4)}, 0.03);
  ASSERT_EQ(held.size(), 1u);
  const gf::HeldDd saved = held.begin()->second;

  // A spec referencing keys that do not exist in ISAM2 makes the update throw.
  const gtsam::Key bogus_a = gtsam::symbol_shorthand::N(90000);
  const gtsam::Key bogus_b = gtsam::symbol_shorthand::N(90001);
  const std::vector<gf::HoldSpec> bad{gf::HoldSpec{0, 100, 101, bogus_a, bogus_b, 1}};
  EXPECT_EQ(gf::applyHolds(fx.isam, fx.amb, held, bad, 0.03),
            gf::HoldResult::Failure);

  // Rollback: the existing hold map is untouched (no partial commit).
  ASSERT_EQ(held.size(), 1u);
  EXPECT_EQ(held.begin()->second.factor_index, saved.factor_index);
  EXPECT_EQ(held.begin()->second.integer, saved.integer);
}

// --- D1: hold-refresh age cap applies ONLY to held arcs ----------------------

TEST(PersistentAmbiguities, AgeCapDoesNotReKeyUnheldArcsInHeldScope) {
  std::uint64_t next = 0;
  gf::PersistentAmbiguities amb(next);
  amb.setHoldRefresh(30.0);
  amb.setRefreshHeldOnly(true);  // "held" scope
  gtsam::NonlinearFactorGraph g;
  gtsam::Values v;
  amb.beginEpoch();
  const gtsam::Key k0 = obtain(amb, 5, 0, /*slip=*/false, 0.0, v);
  // Far past the cap (jitter range is [0.8, 1.2]*30 = [24, 36] s), no hold:
  // the float arc must survive - killing it was the measured defect (mean arc
  // age pinned at ~cap/2 regardless of sky conditions).
  for (double tow : {50.0, 100.0, 500.0}) {
    gtsam::NonlinearFactorGraph ge;
    gtsam::Values ve;
    amb.beginEpoch();
    EXPECT_EQ(obtain(amb, 5, 0, /*slip=*/false, tow, ve), k0)
        << "unheld arc re-keyed by the age cap at tow " << tow;
  }
}

TEST(PersistentAmbiguities, DefaultScopeReKeysAllArcsAtCap) {
  // "all" scope (default): the cap doubles as fading memory - an unheld arc
  // re-keys at the cap age (measured better on urban full courses than
  // preserving it; see setRefreshHeldOnly).
  std::uint64_t next = 0;
  gf::PersistentAmbiguities amb(next);
  amb.setHoldRefresh(30.0);
  gtsam::NonlinearFactorGraph g0;
  gtsam::Values v0;
  amb.beginEpoch();
  const gtsam::Key a0 = obtain(amb, 6, 0, /*slip=*/false, 0.0, v0);
  gtsam::NonlinearFactorGraph g1;
  gtsam::Values v1;
  amb.beginEpoch();
  EXPECT_NE(obtain(amb, 6, 0, /*slip=*/false, 100.0, v1), a0);
}

TEST(PersistentAmbiguities, HeldScopeReKeysHeldArcsAtCap) {
  std::uint64_t next = 0;
  gf::PersistentAmbiguities amb(next);
  amb.setHoldRefresh(30.0);
  amb.setRefreshHeldOnly(true);
  gtsam::NonlinearFactorGraph g;
  gtsam::Values v;
  amb.beginEpoch();
  const gtsam::Key k0 = obtain(amb, 5, 0, /*slip=*/false, 0.0, v);
  amb.setHeld({{5, 0}});
  // Before the cap window ([24,36] s): held arc persists.
  {
    gtsam::NonlinearFactorGraph ge;
    gtsam::Values ve;
    amb.beginEpoch();
    EXPECT_EQ(obtain(amb, 5, 0, /*slip=*/false, 20.0, ve), k0);
  }
  // Beyond the cap window: held arc must re-key (bounds how long an
  // unverified held integer can persist).
  {
    gtsam::NonlinearFactorGraph ge;
    gtsam::Values ve;
    amb.beginEpoch();
    EXPECT_NE(obtain(amb, 5, 0, /*slip=*/false, 100.0, ve), k0);
  }
}

// --- NHC: body lateral/vertical velocity constrained, forward free ----------

namespace {
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

// Linearized error + whitened Jacobian of the single NHC factor in a graph.
gtsam::Vector nhcWhitenedError(const gnss_fgo::NhcConfig& cfg,
                               const gtsam::Pose3& pose, const gtsam::Vector3& vel,
                               const Eigen::Vector3d& omega, double speed) {
  gtsam::NonlinearFactorGraph g;
  EXPECT_TRUE(gnss_fgo::addNhcFactor(g, X(0), V(0), cfg, omega, speed));
  gtsam::Values v;
  v.insert(X(0), pose);
  v.insert(V(0), vel);
  const auto nmf =
      std::dynamic_pointer_cast<gtsam::NoiseModelFactor>(g.at(0));
  return nmf->unwhitenedError(v);
}
}  // namespace

TEST(Nhc, ConstrainsLateralAndVerticalNotForward) {
  gnss_fgo::NhcConfig cfg;  // sigma_lat 0.3, sigma_vert 0.2, forward free
  cfg.enable = true;  // off by default; the math tests exercise it enabled
  // Identity attitude: body == nav. A purely forward velocity satisfies NHC.
  const gtsam::Vector e_fwd = nhcWhitenedError(
      cfg, gtsam::Pose3(), gtsam::Vector3(7.0, 0.0, 0.0),
      Eigen::Vector3d::Zero(), 7.0);
  ASSERT_EQ(e_fwd.size(), 3);
  EXPECT_NEAR(e_fwd[1], 0.0, 1e-9);  // lateral error 0
  EXPECT_NEAR(e_fwd[2], 0.0, 1e-9);  // vertical error 0
  // A lateral velocity component shows up as a lateral error of that magnitude.
  const gtsam::Vector e_lat = nhcWhitenedError(
      cfg, gtsam::Pose3(), gtsam::Vector3(7.0, 0.5, 0.0),
      Eigen::Vector3d::Zero(), 7.0);
  EXPECT_NEAR(e_lat[1], 0.5, 1e-9);
  EXPECT_NEAR(e_lat[2], 0.0, 1e-9);
}

TEST(Nhc, ForwardComponentIsEffectivelyUnconstrained) {
  gnss_fgo::NhcConfig cfg;
  cfg.enable = true;
  gtsam::NonlinearFactorGraph g;
  ASSERT_TRUE(gnss_fgo::addNhcFactor(g, X(0), V(0), cfg, Eigen::Vector3d::Zero(),
                                     7.0));
  gtsam::Values v;
  v.insert(X(0), gtsam::Pose3());
  v.insert(V(0), gtsam::Vector3(7.0, 0.0, 0.0));
  // The whitened (information-weighted) error must be ~0: the forward speed of
  // 7 m/s is divided by the 1e3 forward sigma, the lateral/vertical are 0.
  const double wnorm = g.at(0)->error(v);  // 0.5 * ||whitened||^2
  EXPECT_LT(wnorm, 1e-3);
}

TEST(Nhc, RotatesWithAttitude) {
  gnss_fgo::NhcConfig cfg;
  cfg.enable = true;
  // Yaw +90 deg: nav-frame East velocity is purely forward in the body frame.
  const gtsam::Pose3 pose(gtsam::Rot3::Yaw(M_PI / 2.0), gtsam::Point3(0, 0, 0));
  const gtsam::Vector e = nhcWhitenedError(
      cfg, pose, gtsam::Vector3(0.0, 7.0, 0.0), Eigen::Vector3d::Zero(), 7.0);
  EXPECT_NEAR(e[1], 0.0, 1e-9);  // no lateral error: it is forward in body frame
  EXPECT_NEAR(e[2], 0.0, 1e-9);
}

TEST(Nhc, YawRateGateSkipsWhileTurning) {
  gnss_fgo::NhcConfig cfg;
  cfg.enable = true;
  cfg.max_yaw_rate_rps = 0.02;
  gtsam::NonlinearFactorGraph g;
  // Straight (yaw rate below gate): NHC is added.
  EXPECT_TRUE(gnss_fgo::addNhcFactor(g, X(0), V(0), cfg,
                                     Eigen::Vector3d(0, 0, 0.005), 7.0));
  // Turning (yaw rate above gate): NHC is skipped.
  EXPECT_FALSE(gnss_fgo::addNhcFactor(g, X(0), V(0), cfg,
                                      Eigen::Vector3d(0, 0, 0.2), 7.0));
  EXPECT_EQ(g.size(), 1u);  // only the straight-driving one was added
}

TEST(Nhc, DisabledOrBelowMinSpeedAddsNothing) {
  gnss_fgo::NhcConfig off;
  off.enable = false;
  gtsam::NonlinearFactorGraph g;
  EXPECT_FALSE(gnss_fgo::addNhcFactor(g, X(0), V(0), off,
                                      Eigen::Vector3d::Zero(), 7.0));
  gnss_fgo::NhcConfig gated;
  gated.enable = true;  // enabled, but the prior speed is below min_speed
  gated.min_speed_mps = 2.0;
  EXPECT_FALSE(gnss_fgo::addNhcFactor(g, X(0), V(0), gated,
                                      Eigen::Vector3d::Zero(), 1.0));
  EXPECT_EQ(g.size(), 0u);
}

// --- static-init detection gates --------------------------------------------

TEST(StaticInit, AcceptsGenuineStillWindow) {
  gnss_fgo::StaticInitConfig cfg;
  EXPECT_TRUE(gnss_fgo::staticInitDetect(
      Eigen::Vector3d(0, 0, 9.80665), 0.05, Eigen::Vector3d::Zero(),
      /*doppler_valid=*/true, /*speed=*/0.02, cfg));
}

TEST(StaticInit, RejectsGravityNormDeviation) {
  gnss_fgo::StaticInitConfig cfg;
  // Quiet gyro + low dispersion, but a scale error / vertical bias pushes the
  // specific-force norm ~0.5 m/s^2 off g: must not initialize as static.
  EXPECT_FALSE(gnss_fgo::staticInitDetect(
      Eigen::Vector3d(0, 0, 10.3), 0.05, Eigen::Vector3d::Zero(), false, 0.0,
      cfg));
}

TEST(StaticInit, RejectsConstantVelocityDriveViaDoppler) {
  gnss_fgo::StaticInitConfig cfg;
  // A constant-velocity drive: gyro quiet, dispersion low, norm at g - only the
  // GNSS Doppler speed reveals the motion.
  EXPECT_FALSE(gnss_fgo::staticInitDetect(
      Eigen::Vector3d(0, 0, 9.80665), 0.05, Eigen::Vector3d::Zero(),
      /*doppler_valid=*/true, /*speed=*/8.0, cfg));
  // Without a Doppler reference the same window is (unavoidably) accepted.
  EXPECT_TRUE(gnss_fgo::staticInitDetect(
      Eigen::Vector3d(0, 0, 9.80665), 0.05, Eigen::Vector3d::Zero(),
      /*doppler_valid=*/false, /*speed=*/8.0, cfg));
}

// --- ZUPT: stationarity detection gates -------------------------------------

namespace {
std::vector<gnss_fgo::ImuStat> quietWindow(int n, double g = 9.80665) {
  std::vector<gnss_fgo::ImuStat> w;
  for (int i = 0; i < n; ++i)
    w.push_back({Eigen::Vector3d(0, 0, g), Eigen::Vector3d::Zero()});
  return w;
}
}  // namespace

// --- Earth rotation in the static initialisation ----------------------------
//
// A stationary gyro reads b_g + R_bn*omega_ie^n. Assigning the raw average to
// the bias while GTSAM's Coriolis term also removes the Earth rate takes it out
// twice (a spurious 15.04 deg/h). These pin the corrected initialisation.

TEST(EarthRate, EnuComponentsMatchLatitude) {
  const double phi = 35.1652 * M_PI / 180.0;   // PPC nagoya anchor
  const gtsam::Vector3 w = gf::earthRateEnu(phi);
  EXPECT_NEAR(w.x(), 0.0, 1e-15);              // no East component, by definition
  EXPECT_NEAR(w.y(), 5.9603e-5, 1e-8);         // omega * cos(phi)
  EXPECT_NEAR(w.z(), 4.1993e-5, 1e-8);         // omega * sin(phi)
  EXPECT_NEAR(w.norm(), 7.2921151467e-5, 1e-12);
}

TEST(EarthRate, StaticInitRecoversTheSensorBiasNotTheEarthRate) {
  const double phi = 35.1652 * M_PI / 180.0;
  const gtsam::Vector3 w_nav = gf::earthRateEnu(phi);
  // Level platform yawed 40 deg: attitude nav <- body.
  const gtsam::Rot3 nRb = gtsam::Rot3::Yaw(40.0 * M_PI / 180.0);
  const gtsam::Vector3 true_bias(1.0e-4, -2.0e-4, 3.0e-4);
  // What a stationary gyro would actually average out.
  const gtsam::Vector3 gyro_mean = true_bias + nRb.transpose() * w_nav;

  const gtsam::Vector3 got = gf::staticInitGyroBias(gyro_mean, nRb, w_nav);
  EXPECT_TRUE(got.isApprox(true_bias, 1e-12));
  // Without the correction the Earth rate lands in the bias, and it is not
  // small: 7.3e-5 rad/s is 15 deg/h against an in-run stability of ~2.5 deg/h.
  const gtsam::Vector3 uncorrected =
      gf::staticInitGyroBias(gyro_mean, nRb, gtsam::Vector3::Zero());
  EXPECT_GT((uncorrected - true_bias).norm(), 7.0e-5);
}

TEST(EarthRate, ZeroRateReproducesTheUncorrectedBehaviour) {
  const gtsam::Vector3 gyro_mean(1.0e-3, 2.0e-3, -5.0e-4);
  const gtsam::Rot3 nRb = gtsam::Rot3::Yaw(1.1);
  EXPECT_TRUE(gf::staticInitGyroBias(gyro_mean, nRb, gtsam::Vector3::Zero())
                  .isApprox(gyro_mean, 1e-15));
}

TEST(Zupt, FiresOnQuietWindow) {
  gnss_fgo::ZuptConfig cfg;
  EXPECT_TRUE(gnss_fgo::zuptDetect(quietWindow(20), cfg,
                                   Eigen::Vector3d::Zero(), 0.0));
}

TEST(Zupt, RejectsHighAccelDispersion) {
  gnss_fgo::ZuptConfig cfg;
  auto w = quietWindow(20);
  // Inject a large alternating specific-force disturbance (> max_acc_std).
  for (size_t i = 0; i < w.size(); ++i)
    w[i].acc.x() = (i % 2 == 0) ? 1.0 : -1.0;
  EXPECT_FALSE(gnss_fgo::zuptDetect(w, cfg, Eigen::Vector3d::Zero(), 0.0));
}

TEST(Zupt, RejectsSustainedRotation) {
  gnss_fgo::ZuptConfig cfg;
  auto w = quietWindow(20);
  // A constant turn: low dispersion but high bias-corrected rate magnitude.
  for (auto& s : w) s.gyr = Eigen::Vector3d(0, 0, 0.1);  // > max_gyr_median
  EXPECT_FALSE(gnss_fgo::zuptDetect(w, cfg, Eigen::Vector3d::Zero(), 0.0));
}

TEST(Zupt, VelocityGateVetoesWhenMoving) {
  gnss_fgo::ZuptConfig cfg;
  cfg.max_speed_mps = 1.0;
  // A perfectly quiet window but the prior says 5 m/s: must not fire.
  EXPECT_FALSE(gnss_fgo::zuptDetect(quietWindow(20), cfg,
                                    Eigen::Vector3d::Zero(), 5.0));
  EXPECT_TRUE(gnss_fgo::zuptDetect(quietWindow(20), cfg,
                                   Eigen::Vector3d::Zero(), 0.2));
}

TEST(Zupt, RejectsShortWindow) {
  gnss_fgo::ZuptConfig cfg;
  EXPECT_FALSE(gnss_fgo::zuptDetect(quietWindow(3), cfg,
                                    Eigen::Vector3d::Zero(), 0.0));
}

// The escape statistic for a marginally wrong fix. Three cheaper candidates
// were measured on the PPC dumps and all failed, the post-fit CARRIER residual
// most instructively: on nagoya_run2 it is INVERTED, because the state absorbs
// a wrong integer and drives the carrier residual below the measurement noise.
// The pseudorange has no ambiguity to absorb anything with, so it keeps its
// opinion - and taking the DIFFERENCE between two candidate positions cancels
// the metres of common multipath bias that make its absolute value useless.
TEST(CodeResidualGrowth, RisesWhenTheCorrectionMovesAwayFromTruth) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(8);
  int n_used = 0;
  // Moving the antenna off the truth must cost, in every direction.
  for (const Eigen::Vector3d& off :
       {Eigen::Vector3d(0.8, 0.0, 0.0), Eigen::Vector3d(0.0, -0.9, 0.0),
        Eigen::Vector3d(0.0, 0.0, 1.1)}) {
    const double g = gf::codeResidualGrowth(sc.ep, sc.rover, sc.rover + off,
                                            /*min_dd=*/6, &n_used);
    EXPECT_EQ(n_used, static_cast<int>(sc.ep.dd.size()));
    EXPECT_GT(g, 0.1) << "offset " << off.transpose();
  }
}

TEST(CodeResidualGrowth, IsNegativeWhenTheCorrectionMovesTowardTruth) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(8);
  const Eigen::Vector3d wrong = sc.rover + Eigen::Vector3d(1.0, 0.5, -0.7);
  EXPECT_LT(gf::codeResidualGrowth(sc.ep, wrong, sc.rover, 6), -0.1);
}

TEST(CodeResidualGrowth, IsZeroForTheIdenticalPosition) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(8);
  EXPECT_NEAR(gf::codeResidualGrowth(sc.ep, sc.rover, sc.rover, 6), 0.0, 1e-9);
}

// Below the DD minimum the RMS is too noisy to act on, so the helper must
// report "no opinion" (0) rather than a number the caller would gate on.
TEST(CodeResidualGrowth, ReturnsNoOpinionBelowTheDdMinimum) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(8);
  const Eigen::Vector3d wrong = sc.rover + Eigen::Vector3d(0.0, 0.0, 2.0);
  EXPECT_GT(gf::codeResidualGrowth(sc.ep, sc.rover, wrong, 6), 0.0);
  EXPECT_EQ(gf::codeResidualGrowth(sc.ep, sc.rover, wrong, 99), 0.0);
}

// Carrier-only DDs carry no pseudorange, so they must not enter the statistic.
TEST(CodeResidualGrowth, IgnoresDdsWithoutPseudorange) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(8);
  const Eigen::Vector3d wrong = sc.rover + Eigen::Vector3d(0.0, 0.0, 1.5);
  const double all = gf::codeResidualGrowth(sc.ep, sc.rover, wrong, 1);
  for (std::size_t i = 0; i < sc.ep.dd.size(); i += 2) sc.ep.dd[i].has_pr = false;
  int n_used = 0;
  const double half = gf::codeResidualGrowth(sc.ep, sc.rover, wrong, 1, &n_used);
  EXPECT_EQ(n_used, static_cast<int>(sc.ep.dd.size()) / 2);
  EXPECT_GT(half, 0.0);
  EXPECT_NE(all, half);
}

// The persistence wrapper is what turns the code test from an entry gate into
// an escape gate. Measured motivation: on nagoya_run1 a single epoch of code
// disagreement fires 55 times and NONE of those epochs is a wrong fix, while a
// runaway on nagoya_run2 disagrees for its whole 32-133 epoch duration.
TEST(CodeGrowthMonitor, IsolatedDisagreementNeverRejects) {
  gf::CodeGrowthMonitor m;
  for (int i = 0; i < 20; ++i) {
    EXPECT_FALSE(m.reject(0.9, 0.15, 5)) << "epoch " << i;  // over threshold
    EXPECT_FALSE(m.reject(0.0, 0.15, 5));                   // but not sustained
  }
}

TEST(CodeGrowthMonitor, RejectsOnlyAfterThePersistenceCount) {
  gf::CodeGrowthMonitor m;
  for (int i = 1; i < 5; ++i) EXPECT_FALSE(m.reject(0.2, 0.15, 5)) << i;
  EXPECT_TRUE(m.reject(0.2, 0.15, 5));
}

TEST(CodeGrowthMonitor, DisabledThresholdNeverRejectsAndKeepsNoState) {
  gf::CodeGrowthMonitor m;
  for (int i = 0; i < 50; ++i) EXPECT_FALSE(m.reject(10.0, 0.0, 1));
  EXPECT_EQ(m.run(), 0);
}

// A count accumulated against integers that have since been thrown away must
// not carry over, or the next fix inherits a rejection it did not earn.
TEST(CodeGrowthMonitor, ResetClearsAccumulatedEvidence) {
  gf::CodeGrowthMonitor m;
  for (int i = 1; i < 5; ++i) EXPECT_FALSE(m.reject(0.2, 0.15, 5));
  m.reset();
  for (int i = 1; i < 5; ++i) EXPECT_FALSE(m.reject(0.2, 0.15, 5)) << i;
  EXPECT_TRUE(m.reject(0.2, 0.15, 5));
}

TEST(CodeGrowthMonitor, PersistOneReactsImmediately) {
  gf::CodeGrowthMonitor m;
  EXPECT_TRUE(m.reject(0.16, 0.15, 1));
  EXPECT_TRUE(m.reject(0.16, 0.15, 0));  // clamped to 1, not "always reject"
  EXPECT_FALSE(m.reject(0.14, 0.15, 0));
}

// --- Velocity-direction attitude aiding -------------------------------------
// The root treatment for the wandering yaw. The point of these tests is the
// KIND of constraint: it must inform the rotation and nothing else, which is
// exactly what distinguishes it from the NHC (a velocity constraint saying the
// same physics, measured to make wrong fixes survive 4x longer).

namespace {
// nav <- body with the given ENU yaw (CCW from East) and no roll/pitch.
gtsam::Rot3 yawOnly(double deg) {
  return gtsam::Rot3::RzRyRx(0.0, 0.0, deg * M_PI / 180.0);
}
// The factor the node builds, for direct inspection.
gtsam::NonlinearFactorGraph attitudeAidingGraph(const gtsam::Rot3& nRb,
                                                const Eigen::Vector3d& v_nav,
                                                double sigma_vel,
                                                const gf::VelocityAttitudeConfig& cfg,
                                                bool* added = nullptr) {
  gtsam::NonlinearFactorGraph g;
  const bool ok = gf::addVelocityAttitudeFactor(
      g, gtsam::symbol_shorthand::X(0), nRb, v_nav, sigma_vel, cfg, 0.0);
  if (added) *added = ok;
  return g;
}
}  // namespace

TEST(VelocityAttitude, ErrorVanishesWhenTheBodyPointsAlongTheVelocity) {
  gf::VelocityAttitudeConfig cfg;
  const Eigen::Vector3d v(10.0, 0.0, 0.0);  // due East at 10 m/s
  auto g = attitudeAidingGraph(yawOnly(0.0), v, 0.1, cfg);
  ASSERT_EQ(g.size(), 1u);
  const auto nmf = std::dynamic_pointer_cast<gtsam::NoiseModelFactor>(g[0]);
  ASSERT_TRUE(nmf != nullptr);
  gtsam::Values val;
  val.insert(gtsam::symbol_shorthand::X(0),
             gtsam::Pose3(yawOnly(0.0), gtsam::Point3(1.0, 2.0, 3.0)));
  EXPECT_NEAR(nmf->unwhitenedError(val).norm(), 0.0, 1e-9);
  // Unit3's error is the CHORDAL measure sin(theta), not theta - so a sigma in
  // radians is the small-angle reading, and the error is non-monotone past
  // 90 deg and zero at 180 deg. The node relies on that: a large initial offset
  // is fixed by re-anchoring the heading, never by pulling through the antipode.
  val.update(gtsam::symbol_shorthand::X(0),
             gtsam::Pose3(yawOnly(20.0), gtsam::Point3(1.0, 2.0, 3.0)));
  EXPECT_NEAR(nmf->unwhitenedError(val).norm(), std::sin(20.0 * M_PI / 180.0),
              1e-9);
  val.update(gtsam::symbol_shorthand::X(0),
             gtsam::Pose3(yawOnly(180.0), gtsam::Point3(1.0, 2.0, 3.0)));
  EXPECT_NEAR(nmf->unwhitenedError(val).norm(), 0.0, 1e-9)
      << "the antipode is a stationary point, hence the re-anchor path";
}

// The whole design argument in one test: the factor touches the pose only, so it
// cannot make the velocity over-confident the way the NHC does.
TEST(VelocityAttitude, ConstrainsTheRotationAndNothingElse) {
  gf::VelocityAttitudeConfig cfg;
  auto g = attitudeAidingGraph(yawOnly(10.0), Eigen::Vector3d(10.0, 0.0, 0.0),
                               0.1, cfg);
  ASSERT_EQ(g.size(), 1u);
  EXPECT_EQ(g[0]->size(), 1u) << "must be a unary factor on the pose";
  EXPECT_EQ(g[0]->keys().front(), gtsam::symbol_shorthand::X(0));
  gtsam::Values val;
  val.insert(gtsam::symbol_shorthand::X(0),
             gtsam::Pose3(yawOnly(10.0), gtsam::Point3(5.0, -5.0, 2.0)));
  const auto lin = g[0]->linearize(val);
  const gtsam::Matrix H = lin->jacobian().first;
  ASSERT_EQ(H.cols(), 6);  // Pose3 tangent: [rot(3), trans(3)]
  EXPECT_GT(H.leftCols<3>().norm(), 1e-6) << "rotation block must be non-zero";
  EXPECT_NEAR(H.rightCols<3>().norm(), 0.0, 1e-12)
      << "translation block must be exactly zero";
}

TEST(VelocityAttitude, SigmaFollowsTheVelocityUncertaintyAndFloorsAtSideslip) {
  const double floor = 2.0 * M_PI / 180.0;
  // At high speed the direction is well determined and the floor dominates.
  EXPECT_NEAR(gf::velocityAttitudeSigma(20.0, 0.1, floor),
              std::hypot(0.005, floor), 1e-12);
  // Halving the speed doubles the velocity-derived term.
  const double s10 = gf::velocityAttitudeSigma(10.0, 0.1, 0.0);
  const double s5 = gf::velocityAttitudeSigma(5.0, 0.1, 0.0);
  EXPECT_NEAR(s5 / s10, 2.0, 1e-9);
  // Never returns 0 or a non-finite value, even at a standstill.
  EXPECT_GT(gf::velocityAttitudeSigma(0.0, 0.1, floor), 0.0);
  EXPECT_TRUE(std::isfinite(gf::velocityAttitudeSigma(0.0, 0.1, floor)));
}

TEST(VelocityAttitude, SkipsBelowMinSpeedWhereTheCourseIsNoise) {
  gf::VelocityAttitudeConfig cfg;
  cfg.min_speed_mps = 3.0;
  bool added = true;
  attitudeAidingGraph(yawOnly(0.0), Eigen::Vector3d(1.0, 0.0, 0.0), 0.1, cfg,
                      &added);
  EXPECT_FALSE(added);
  attitudeAidingGraph(yawOnly(0.0), Eigen::Vector3d(4.0, 0.0, 0.0), 0.1, cfg,
                      &added);
  EXPECT_TRUE(added);
}

// Reverse driving reads ~180 deg. Accepting it would lock the heading exactly
// backwards, which is worse than having no aiding at all.
TEST(VelocityAttitude, RejectsAReversedHeadingButAcceptsALargeHonestError) {
  gf::VelocityAttitudeConfig cfg;
  cfg.max_misalign_deg = 90.0;
  const Eigen::Vector3d v(10.0, 0.0, 0.0);
  bool added = true;
  attitudeAidingGraph(yawOnly(180.0), v, 0.1, cfg, &added);
  EXPECT_FALSE(added) << "driving in reverse must not be treated as heading";
  attitudeAidingGraph(yawOnly(80.0), v, 0.1, cfg, &added);
  EXPECT_TRUE(added);
  // Suspending the guard (what the node does until the heading first agrees)
  // lets an initial offset beyond 90 deg be corrected instead of locked out.
  cfg.max_misalign_deg = 180.0;
  attitudeAidingGraph(yawOnly(150.0), v, 0.1, cfg, &added);
  EXPECT_TRUE(added);
}

TEST(VelocityAttitude, MisalignIsMeasuredHorizontallySoPitchDoesNotTripTheGuard) {
  const Eigen::Vector3d v(10.0, 0.0, 0.0);
  // 40 deg of pitch, heading correct: the horizontal angle is still ~0.
  const gtsam::Rot3 pitched = gtsam::Rot3::RzRyRx(0.0, 40.0 * M_PI / 180.0, 0.0);
  EXPECT_NEAR(gf::velocityAttitudeMisalign(pitched, v), 0.0, 1e-9);
  EXPECT_NEAR(gf::velocityAttitudeMisalign(yawOnly(30.0), v),
              30.0 * M_PI / 180.0, 1e-9);
}

TEST(VelocityAttitude, CourseAlignmentReplacesYawAndKeepsRollAndPitch) {
  const gtsam::Rot3 nRb = gtsam::Rot3::RzRyRx(0.07, -0.13, 1.2);
  // Heading North-East at 10 m/s -> ENU yaw = 45 deg.
  const gtsam::Rot3 out =
      gf::alignYawToCourse(nRb, Eigen::Vector3d(7.07, 7.07, 0.0));
  EXPECT_NEAR(out.rpy().x(), 0.07, 1e-9);
  EXPECT_NEAR(out.rpy().y(), -0.13, 1e-9);
  EXPECT_NEAR(out.rpy().z(), M_PI / 4.0, 1e-6);
  // A degenerate horizontal velocity leaves the attitude untouched.
  EXPECT_TRUE(gf::alignYawToCourse(nRb, Eigen::Vector3d(0.0, 0.0, 5.0))
                  .equals(nRb, 1e-12));
}

TEST(VelocityAttitude, DisabledAddsNothing) {
  gf::VelocityAttitudeConfig cfg;
  cfg.enable = false;
  bool added = true;
  auto g = attitudeAidingGraph(yawOnly(30.0), Eigen::Vector3d(10.0, 0.0, 0.0),
                               0.1, cfg, &added);
  EXPECT_FALSE(added);
  EXPECT_EQ(g.size(), 0u);
}

// The absolute companion to CodeResidualGrowth, and the reason it exists: the
// growth test scores `fix - float`, so when a confident IMU chain drags the
// FLOAT to the wrong place along with the fix, the increment vanishes and the
// test goes quiet (measured: tightening it changed nagoya_run2's false fixes
// from 268 to 269). The pseudorange is not a function of the carrier
// ambiguities, so it keeps reporting where the antenna is not.
TEST(CodeResidualRms, GrowsWithDistanceFromTheTruePosition) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(8);
  int n = 0;
  const double at_truth = gf::codeResidualRms(sc.ep, sc.rover, 6, &n);
  EXPECT_EQ(n, static_cast<int>(sc.ep.dd.size()));
  EXPECT_NEAR(at_truth, 0.0, 1e-6);
  double prev = at_truth;
  for (const double d : {0.5, 1.0, 2.0, 4.0}) {
    const double r = gf::codeResidualRms(
        sc.ep, sc.rover + Eigen::Vector3d(d, 0.0, 0.0), 6);
    EXPECT_GT(r, prev) << "offset " << d;
    prev = r;
  }
}

// Unlike the growth test, this one does not care how the state got there - only
// where it is. Two positions equally wrong must score equally.
TEST(CodeResidualRms, IsAboutThePositionNotThePathToIt) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(8);
  const Eigen::Vector3d off(0.0, 0.0, 3.0);
  EXPECT_NEAR(gf::codeResidualRms(sc.ep, sc.rover + off, 6),
              gf::codeResidualRms(sc.ep, sc.rover + off, 6), 1e-12);
  // A wrong position reached with NO increment at all still scores - which is
  // exactly the case the growth test misses.
  EXPECT_NEAR(gf::codeResidualGrowth(sc.ep, sc.rover + off, sc.rover + off, 6),
              0.0, 1e-12);
  EXPECT_GT(gf::codeResidualRms(sc.ep, sc.rover + off, 6), 1.0);
}

TEST(CodeResidualRms, ReturnsNoOpinionBelowTheDdMinimum) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(8);
  const Eigen::Vector3d wrong = sc.rover + Eigen::Vector3d(0.0, 0.0, 5.0);
  EXPECT_GT(gf::codeResidualRms(sc.ep, wrong, 6), 0.0);
  EXPECT_EQ(gf::codeResidualRms(sc.ep, wrong, 99), 0.0);
}

TEST(CodeResidualRms, IgnoresDdsWithoutPseudorange) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(8);
  for (std::size_t i = 0; i < sc.ep.dd.size(); i += 2) sc.ep.dd[i].has_pr = false;
  int n = 0;
  gf::codeResidualRms(sc.ep, sc.rover, 1, &n);
  EXPECT_EQ(n, static_cast<int>(sc.ep.dd.size()) / 2);
}

// The precondition of a wedge that cost a full evaluation batch: when the
// pre-fit filter cannot clear an epoch even at the graph-independent code
// a-priori, it rejects EVERY DD and leaves prediction_fault false. Both nodes
// used to gate their re-anchor on `prediction_fault && !dd.empty()`, so in this
// exact state neither term held - no measurement could correct the state and
// the next epoch was judged identically. Observed on PPC tokyo_run1 after a
// singular update: -273 m height with "DD: 38 (gated 38)" for the rest of the
// run. This test pins the filter's half of the contract; the nodes now also
// re-anchor on a starvation timer, which needs no surviving DDs.
TEST(PreFitInnovationFde, DirtyEpochYieldsAnEmptySetAndNoPredictionFault) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(8);
  for (auto& d : sc.ep.dd) {
    d.el_base_ref = d.el_ref;
    d.el_base_tar = d.el_tar;
  }
  // Corrupt more pseudoranges than the exclusion budget can absorb, in a way
  // that is inconsistent at ANY position - so the a-priori retest fails too.
  sc.ep.dd[1].pr_rov_tar += 40.0;
  sc.ep.dd[3].pr_rov_tar -= 55.0;
  sc.ep.dd[5].pr_rov_tar += 70.0;
  sc.ep.dd[6].pr_rov_tar -= 35.0;
  gf::AdapterConfig cfg;
  cfg.robust = false;
  gf::ArOptions opt = fdeOptions();
  opt.fde_max_exclude = 2;
  int excluded = 0;
  bool fault = true;  // must be cleared by the filter
  const gu::PreprocessedEpoch filtered = gf::filterDdPreFitInnovation(
      sc.ep, sc.rover, Eigen::Matrix3d::Identity() * 0.01, cfg, opt, &excluded,
      &fault);
  EXPECT_TRUE(filtered.dd.empty()) << "a dirty epoch is rejected whole";
  EXPECT_FALSE(fault) << "not a prediction fault - the DATA is what failed";
  EXPECT_EQ(excluded, static_cast<int>(sc.ep.dd.size()));
}

namespace {
// A carrier-only epoch whose DDs form TWO disconnected components inside ONE
// (sys, band, code) group: satellites {0,1} share one reference and {2,3}
// share another. Nothing in the pipeline forces a group's rows to share a
// reference, so this shape is reachable on real data.
gu::PreprocessedEpoch twoComponentEpoch() {
  SyntheticDdScenario sc = makeSyntheticDdScenario(5);
  gu::PreprocessedEpoch ep = sc.ep;
  ep.dd.resize(4);
  for (auto& d : ep.dd) {
    d.has_pr = false;
    d.el_base_ref = d.el_ref;
    d.el_base_tar = d.el_tar;
  }
  // Component A: ref 101 -> {102, 103}.  Component B: ref 104 -> {105}.
  ep.dd[0].sat_ref = 101; ep.dd[0].sat_tar = 102;
  ep.dd[1].sat_ref = 101; ep.dd[1].sat_tar = 103;
  ep.dd[2].sat_ref = 104; ep.dd[2].sat_tar = 105;
  ep.dd.resize(3);
  return ep;
}
}  // namespace

// The gauge freedom is per CONNECTED COMPONENT, not per group: a DD row
// constrains only the DIFFERENCE of its two ambiguities. Gauging per group left
// every component but one unpinned, which is a rank deficiency - on tokyo_run1
// it surfaced as "singular near x4897" and cost the whole graph.
TEST(GaugeInvariant, EachConnectedComponentOfAGroupIsGaugedSeparately) {
  std::uint64_t next = 0;
  gf::PersistentAmbiguities amb(next);
  gtsam::NonlinearFactorGraph g;
  gtsam::Values v;
  const gu::PreprocessedEpoch ep = twoComponentEpoch();
  // All three rows live in one (sys, band, code) group but form two components.
  EXPECT_EQ(addEpochCountingGaugePriors(amb, ep, g, v), 2u);
  // Carried forward, both stay gauged and neither is gauged twice.
  EXPECT_EQ(addEpochCountingGaugePriors(amb, ep, g, v), 0u);
}

// And the system that results must actually be solvable: 3 ambiguity keys per
// component minus the DD rank, pinned by one prior each.
TEST(GaugeInvariant, TwoComponentGroupYieldsANonSingularSystem) {
  std::uint64_t next = 0;
  gf::PersistentAmbiguities amb(next);
  gtsam::NonlinearFactorGraph g;
  gtsam::Values v;
  const gu::PreprocessedEpoch ep = twoComponentEpoch();
  v.insert(gtsam::symbol_shorthand::X(0), gtsam::Point3(ep.rover_ecef_apriori));
  g.add(gtsam::PriorFactor<gtsam::Point3>(
      gtsam::symbol_shorthand::X(0), gtsam::Point3(ep.rover_ecef_apriori),
      gtsam::noiseModel::Isotropic::Sigma(3, 0.01)));
  addEpochCountingGaugePriors(amb, ep, g, v);
  EXPECT_NO_THROW({
    const gtsam::GaussianFactorGraph::shared_ptr lin = g.linearize(v);
    lin->optimize();
  });
}

// ---------------------------------------------------------------------------
// Code-DD weighted least squares: the one position in these nodes that the
// estimator cannot move.
//
// Every other integrity statistic here is computed from the graph posterior,
// which puts it inside the loop it is meant to police: a confident IMU drags
// the float, the float biases the AR prior, accepted integers are held, the
// holds drag the graph. These tests pin the two properties that make this
// solution a way out of that loop - it is decided by the observations (not by
// where the search started) and no carrier ambiguity can touch it.
// ---------------------------------------------------------------------------

TEST(CodeDdWls, RecoversTheRoverFromCodeDdsAlone) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(8);
  gf::AdapterConfig cfg;
  const gf::CodeDdSolution s =
      gf::solveCodeDdWls(sc.ep, cfg, sc.ep.rover_ecef_apriori);
  ASSERT_TRUE(s.ok);
  EXPECT_EQ(s.n_dd, static_cast<int>(sc.ep.dd.size()));
  EXPECT_EQ(s.n_rejected, 0);
  EXPECT_LT((s.pos - sc.rover).norm(), 1e-3);
  EXPECT_NEAR(s.resid_rms, 0.0, 1e-6);
  EXPECT_TRUE(s.cov.allFinite());
}

// THE independence proof. If the answer depended on the seed, then feeding it
// the estimator's own state would quietly hand back a copy of that state -
// which is exactly the defect this function exists to avoid (the a-priori
// carried in PreprocessedEpoch has precisely that problem, because the nodes
// pass their last estimate in as the hint). Convergence from wildly different
// starts to the same point is what makes it a measurement.
TEST(CodeDdWls, IsDeterminedByObservationsNotBySeed) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(8);
  gf::AdapterConfig cfg;
  const gf::CodeDdSolution ref =
      gf::solveCodeDdWls(sc.ep, cfg, sc.ep.rover_ecef_apriori);
  ASSERT_TRUE(ref.ok);
  for (const Eigen::Vector3d& off :
       {Eigen::Vector3d(50.0, 0.0, 0.0), Eigen::Vector3d(0.0, -50.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 50.0), Eigen::Vector3d(-30.0, 40.0, 20.0)}) {
    const gf::CodeDdSolution s = gf::solveCodeDdWls(sc.ep, cfg, sc.rover + off);
    ASSERT_TRUE(s.ok) << "seed offset " << off.transpose();
    EXPECT_LT((s.pos - ref.pos).norm(), 1e-3) << "seed offset " << off.transpose();
  }
}

// The property the escape design rests on: a wrong integer moves the carrier
// observable by whole wavelengths, and this solution does not notice, because
// no ambiguity term appears in a code double difference at all.
TEST(CodeDdWls, NoCarrierAmbiguityCanMoveIt) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(8);
  gf::AdapterConfig cfg;
  const gf::CodeDdSolution before =
      gf::solveCodeDdWls(sc.ep, cfg, sc.ep.rover_ecef_apriori);
  ASSERT_TRUE(before.ok);
  // Corrupt every carrier phase by tens of cycles - a catastrophic wrong fix.
  for (auto& d : sc.ep.dd) {
    d.cp_rov_ref += 37.0 * d.lam;
    d.cp_rov_tar -= 91.0 * d.lam;
  }
  const gf::CodeDdSolution after =
      gf::solveCodeDdWls(sc.ep, cfg, sc.ep.rover_ecef_apriori);
  ASSERT_TRUE(after.ok);
  EXPECT_LT((after.pos - before.pos).norm(), 1e-9);
}

// It cannot lean on the graph's pre-fit gating (that gating is judged against
// the suspect state), so it has to screen its own blunders.
TEST(CodeDdWls, RejectsAGrossOutlierOnItsOwn) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(8);
  gf::AdapterConfig cfg;
  sc.ep.dd[3].pr_rov_tar += 30.0;  // 30 m blunder on one target pseudorange
  const gf::CodeDdSolution s =
      gf::solveCodeDdWls(sc.ep, cfg, sc.ep.rover_ecef_apriori);
  ASSERT_TRUE(s.ok);
  EXPECT_GE(s.n_rejected, 1);
  EXPECT_EQ(s.n_dd, static_cast<int>(sc.ep.dd.size()) - s.n_rejected);
  EXPECT_LT((s.pos - sc.rover).norm(), 1e-2) << "the blunder was not removed";
}

TEST(CodeDdWls, ReportsFailureWhenUnderdetermined) {
  gf::AdapterConfig cfg;
  SyntheticDdScenario sc = makeSyntheticDdScenario(8);
  sc.ep.dd.resize(2);  // 2 DDs cannot determine 3 unknowns with redundancy
  EXPECT_FALSE(gf::solveCodeDdWls(sc.ep, cfg, sc.rover).ok);
  // No base position means no double differences exist at all.
  SyntheticDdScenario sc2 = makeSyntheticDdScenario(8);
  sc2.ep.base_ecef = Eigen::Vector3d::Zero();
  EXPECT_FALSE(gf::solveCodeDdWls(sc2.ep, cfg, sc2.rover).ok);
}

TEST(CodeDdWls, IgnoresDdsThatCarryNoPseudorange) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(8);
  gf::AdapterConfig cfg;
  for (std::size_t i = 0; i < sc.ep.dd.size(); i += 2) sc.ep.dd[i].has_pr = false;
  const gf::CodeDdSolution s =
      gf::solveCodeDdWls(sc.ep, cfg, sc.ep.rover_ecef_apriori);
  ASSERT_TRUE(s.ok);
  EXPECT_EQ(s.n_dd, static_cast<int>(sc.ep.dd.size()) / 2);
  EXPECT_LT((s.pos - sc.rover).norm(), 1e-3);
}

// ---------------------------------------------------------------------------
// independentCodePosition: the escape target must be a MEASUREMENT.
//
// The defect being guarded against is subtle enough to have survived in the
// code with a comment acknowledging it: both nodes feed their own estimate to
// drainEpochs, so ep.rover_ecef_apriori comes back as a copy of that estimate,
// and everything that treated it as an independent code position was comparing
// the state against itself.

TEST(IndependentCodePosition, PrefersTheCodeDdSolutionWhenABaseIsAvailable) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(8);
  gf::AdapterConfig cfg;
  const gf::IndependentCodePosition a = gf::independentCodePosition(sc.ep, cfg);
  ASSERT_TRUE(a.ok);
  EXPECT_EQ(a.source, gf::IndependentCodePosition::Source::DdWls);
  EXPECT_TRUE(a.independent());
  EXPECT_LT((a.pos - sc.rover).norm(), 1e-3);
  // The floor must be applied: a formal covariance of a noiseless synthetic
  // solve is essentially zero, and an anchor that claims millimetres would
  // override the carrier phase it is meant to arbitrate.
  for (int i = 0; i < 3; ++i) EXPECT_GE(a.cov(i, i), 1.0);
}

// The whole point, stated as a test: feeding in a badly wrong "previous
// estimate" must not move the answer.
TEST(IndependentCodePosition, IsNotMovedByAWrongApriori) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(8);
  gf::AdapterConfig cfg;
  const gf::IndependentCodePosition ref =
      gf::independentCodePosition(sc.ep, cfg);
  ASSERT_TRUE(ref.ok);
  // A runaway state: 60 m off, the kind of drift the re-anchor exists for.
  sc.ep.rover_ecef_apriori = sc.rover + Eigen::Vector3d(40.0, -30.0, 20.0);
  const gf::IndependentCodePosition a = gf::independentCodePosition(sc.ep, cfg);
  ASSERT_TRUE(a.ok);
  EXPECT_TRUE(a.independent());
  EXPECT_LT((a.pos - ref.pos).norm(), 1e-3);
  // ...whereas the value it replaced follows the runaway exactly.
  EXPECT_GT((sc.ep.rover_ecef_apriori - sc.rover).norm(), 50.0);
}

TEST(IndependentCodePosition, FallsBackToSppThenApriori) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(8);
  gf::AdapterConfig cfg;
  const Eigen::Vector3d spp = sc.rover + Eigen::Vector3d(1.5, -0.8, 2.0);
  sc.ep.dd.clear();  // no base: the code-DD solve cannot run
  sc.ep.rover_spp_valid = true;
  sc.ep.rover_ecef_spp = spp;
  sc.ep.rover_spp_cov = Eigen::Matrix3d::Identity() * 0.04;
  const gf::IndependentCodePosition s = gf::independentCodePosition(sc.ep, cfg);
  ASSERT_TRUE(s.ok);
  EXPECT_EQ(s.source, gf::IndependentCodePosition::Source::Spp);
  EXPECT_TRUE(s.independent());
  EXPECT_LT((s.pos - spp).norm(), 1e-9);
  for (int i = 0; i < 3; ++i) EXPECT_GE(s.cov(i, i), 9.0);  // 3 m floor

  sc.ep.rover_spp_valid = false;
  const gf::IndependentCodePosition f = gf::independentCodePosition(sc.ep, cfg);
  ASSERT_TRUE(f.ok);
  EXPECT_EQ(f.source, gf::IndependentCodePosition::Source::Apriori);
  // Reported honestly as NOT independent, so a caller counting escapes can see
  // when the mechanism has quietly degraded to the old behaviour.
  EXPECT_FALSE(f.independent());
}

// ---------------------------------------------------------------------------
// Post-fit FIX validation: the residual covariance must SHRINK, not grow.

// The identity the fix rests on, checked by sampling rather than by repeating
// the algebra: for the linear-Gaussian posterior mean, Cov(z - H x_hat+) is
// R - H P+ H', which is SMALLER than R - while the pre-fit form R + H P H' is
// larger. Getting the sign wrong therefore does not merely mis-scale the test,
// it moves it to the wrong side of the measurement noise.
TEST(PostFitResidualCovariance, SubtractiveFormMatchesSampling) {
  const int n = 7;
  std::mt19937 rng(11);
  std::normal_distribution<double> g(0.0, 1.0);
  Eigen::MatrixXd H(n, 3);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < 3; ++j) H(i, j) = g(rng);
  Eigen::MatrixXd Rc(n, n);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) Rc(i, j) = g(rng);
  const Eigen::MatrixXd R =
      Rc * Rc.transpose() * 0.01 + Eigen::MatrixXd::Identity(n, n) * 0.25;
  const Eigen::Matrix3d P0 = Eigen::Matrix3d::Identity() * 4.0;  // prior

  // Posterior of x given z: P+ = (P0^-1 + H' R^-1 H)^-1.
  const Eigen::Matrix3d Pp =
      (P0.inverse() + H.transpose() * R.inverse() * H).inverse();
  const Eigen::MatrixXd K_gain = Pp * H.transpose() * R.inverse();  // 3 x n

  const Eigen::MatrixXd Lr = Eigen::LLT<Eigen::MatrixXd>(R).matrixL();
  const Eigen::Matrix3d Lp = Eigen::LLT<Eigen::Matrix3d>(P0).matrixL();
  const int N = 200000;
  Eigen::MatrixXd acc = Eigen::MatrixXd::Zero(n, n);
  for (int s = 0; s < N; ++s) {
    Eigen::Vector3d zx;
    for (int i = 0; i < 3; ++i) zx(i) = g(rng);
    const Eigen::Vector3d x = Lp * zx;  // truth about a zero prior mean
    Eigen::VectorXd zr(n);
    for (int i = 0; i < n; ++i) zr(i) = g(rng);
    const Eigen::VectorXd z = H * x + Lr * zr;
    const Eigen::Vector3d xhat = K_gain * z;   // posterior mean (prior mean 0)
    const Eigen::VectorXd v = z - H * xhat;    // POST-fit residual
    acc += v * v.transpose();
  }
  acc /= N;

  const Eigen::MatrixXd subtractive = R - H * Pp * H.transpose();
  const Eigen::MatrixXd additive = R + H * Pp * H.transpose();
  EXPECT_LT((acc - subtractive).norm() / subtractive.norm(), 0.02);
  // And the form the code used to have is not merely imprecise, it is larger
  // than the truth by a wide margin - which is why the test under-rejected.
  EXPECT_GT(additive.norm(), 1.5 * subtractive.norm());
}

// A clean fix must survive the stricter test, or the correction is worthless
// however sound the statistics.
//
// Note what is NOT tested here, because it would be testing the wrong thing: a
// pure position offset. When the posterior is formed from these same rows, a
// position shift is precisely what the 3-parameter fit absorbs - the residuals
// barely move, by construction, and no post-fit test can or should flag it.
// What a post-fit test detects is INCONSISTENCY BETWEEN rows, which is also
// what a wrong integer produces: the carrier rows move by whole wavelengths
// while the code rows do not. A single-row blunder is the same geometry and is
// what is injected below.
TEST(PostFitValidation, AcceptsAConsistentFixAndRejectsAnInconsistentRow) {
  gf::AdapterConfig cfg;
  gf::ArOptions opt;

  auto validate = [&]() {
    SyntheticDdScenario sc = makeSyntheticDdScenario(9);
    // The BASE elevations must be set. They default to 0, and the
    // elevation-dependent variance model divides by sin^2(el), so leaving them
    // makes every sigma astronomically large and no residual can ever fail a
    // test - which is how this test first "passed" a 1.7 m position error.
    for (auto& d : sc.ep.dd) {
      d.el_base_ref = d.el_ref;
      d.el_base_tar = d.el_tar;
    }
    return sc;
  };
  // Exercised through the public entry point in the ValidationRows sense is not
  // possible without a full ISAM2 posterior, so drive the statistic directly on
  // the same construction the validation block uses.
  // Mirrors the validation block: the same H, R and residuals, evaluated at the
  // weighted-least-squares position implied by the rows themselves - which is
  // what a graph posterior conditioned on those rows converges to, and the only
  // setting in which R - H P+ H' is the correct covariance.
  //
  // `subtractive=false` reproduces the legacy R + H P+ H' so the two can be
  // compared on identical data.
  auto chi2_of = [&](const gu::PreprocessedEpoch& ep, bool subtractive,
                     int* dof_out) {
    const int nr = static_cast<int>(ep.dd.size());
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(nr, nr);
    for (int i = 0; i < nr; ++i)
      R(i, i) = gf::ddSingleDifferenceVar(ep.dd[i], true, cfg.pr_sigma_m, cfg) +
                gf::ddSingleDifferenceVar(ep.dd[i], false, cfg.pr_sigma_m, cfg);

    // Converge to the GLS position under R, then linearize there.
    const gf::CodeDdSolution wls =
        gf::solveCodeDdWls(ep, cfg, ep.rover_ecef_apriori);
    const Eigen::Vector3d x = wls.ok ? wls.pos : ep.rover_ecef_apriori;

    Eigen::VectorXd residual(nr);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(nr, 3);
    for (int i = 0; i < nr; ++i) {
      const auto& d = ep.dd[i];
      Eigen::Vector3d er, et;
      gf::detail::geodistSagnac(d.sat_ref_rov, x, &er);
      gf::detail::geodistSagnac(d.sat_tar_rov, x, &et);
      H.row(i) = (et - er).transpose();
      residual(i) = gf::detail::ddPseudorangeResidual(d, ep.base_ecef, x);
    }
    // The posterior these rows produce. Using anything else would be testing a
    // formula against a covariance that does not belong to it.
    const Eigen::Matrix3d Pp =
        (H.transpose() * R.inverse() * H).inverse();

    Eigen::MatrixXd S = subtractive ? Eigen::MatrixXd(R - H * Pp * H.transpose())
                                    : Eigen::MatrixXd(R + H * Pp * H.transpose());
    S = 0.5 * (S + S.transpose());
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(S);
    const double lmax = es.eigenvalues().maxCoeff();
    double chi2 = 0.0;
    int dof = 0;
    for (int i = 0; i < nr; ++i) {
      if (es.eigenvalues()(i) <= 1e-9 * lmax) continue;
      const double proj = es.eigenvectors().col(i).dot(residual);
      chi2 += proj * proj / es.eigenvalues()(i);
      ++dof;
    }
    *dof_out = dof;
    return chi2;
  };

  // Largest |residual| / sqrt(S_ii), the other half of the validation verdict.
  auto max_standardized = [&](const gu::PreprocessedEpoch& ep,
                              bool subtractive) {
    const int n = static_cast<int>(ep.dd.size());
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(n, n);
    for (int i = 0; i < n; ++i)
      R(i, i) = gf::ddSingleDifferenceVar(ep.dd[i], true, cfg.pr_sigma_m, cfg) +
                gf::ddSingleDifferenceVar(ep.dd[i], false, cfg.pr_sigma_m, cfg);
    const gf::CodeDdSolution wls =
        gf::solveCodeDdWls(ep, cfg, ep.rover_ecef_apriori);
    const Eigen::Vector3d x = wls.ok ? wls.pos : ep.rover_ecef_apriori;
    Eigen::VectorXd residual(n);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n, 3);
    for (int i = 0; i < n; ++i) {
      const auto& d = ep.dd[i];
      Eigen::Vector3d er, et;
      gf::detail::geodistSagnac(d.sat_ref_rov, x, &er);
      gf::detail::geodistSagnac(d.sat_tar_rov, x, &et);
      H.row(i) = (et - er).transpose();
      residual(i) = gf::detail::ddPseudorangeResidual(d, ep.base_ecef, x);
    }
    const Eigen::Matrix3d Pp = (H.transpose() * R.inverse() * H).inverse();
    const Eigen::MatrixXd HPH = H * Pp * H.transpose();
    const Eigen::MatrixXd S = subtractive ? Eigen::MatrixXd(R - HPH)
                                          : Eigen::MatrixXd(R + HPH);
    double z = 0.0;
    for (int i = 0; i < n; ++i)
      if (S(i, i) > 1e-12)
        z = std::max(z, std::fabs(residual(i)) / std::sqrt(S(i, i)));
    return z;
  };

  // Clean data passes, and the rank is the REDUNDANCY - exactly nr - 3, because
  // the fit consumed the three position directions. That is the property which
  // makes a chi-square quantile meaningful; the old chi2/nr threshold silently
  // assumed nr.
  SyntheticDdScenario clean = validate();
  const int nr = static_cast<int>(clean.ep.dd.size());
  int dof = 0;
  const double chi2_clean = chi2_of(clean.ep, true, &dof);
  EXPECT_EQ(dof, nr - 3);
  EXPECT_LT(chi2_clean, gf::chi2CriticalValue(dof, opt.fde_nsigma));

  // One inconsistent row - the geometry a wrong integer creates - must fail.
  // 25 m is an NLOS-scale blunder; the DD code sigmas here are metre-level
  // after elevation weighting, so a few metres is genuinely NOT significant at
  // 4 sigma across 9 rows and it would be dishonest to assert otherwise.
  SyntheticDdScenario bad = validate();
  bad.ep.dd[3].pr_rov_tar += 25.0;
  int dof_bad = 0;
  const double chi2_bad = chi2_of(bad.ep, true, &dof_bad);
  EXPECT_EQ(dof_bad, nr - 3);
  EXPECT_GT(chi2_bad, gf::chi2CriticalValue(dof_bad, opt.fde_nsigma));

  // The legacy form is the strictly less sensitive test, and the clean way to
  // see it is per row: S_sub = R - H P H' is smaller than S_add = R + H P H' on
  // every diagonal entry, so every standardized residual is larger under the
  // correct form. (The two chi^2 values are NOT directly comparable - the
  // subtractive form is rank nr-3 while the additive one is full rank, so they
  // are quantiles at different dof.)
  const double z_sub = max_standardized(bad.ep, true);
  const double z_add = max_standardized(bad.ep, false);
  EXPECT_GT(z_sub, z_add);
  EXPECT_GT(z_sub, opt.fde_nsigma) << "the correct form must flag this row";
}

// ---------------------------------------------------------------------------
// Pre-fit gate covariance: propagation, not summation.
//
// Mirrors the assembly in gnss_imu_fgo's gate block exactly. The check is by
// SAMPLING - drawing departure states from the graph posterior, pushing each
// through the real pim.predict(), and comparing the covariance of the resulting
// ANTENNA position. A hand-derived agreement would not catch the two things
// that are actually easy to get wrong here: the NavState-vs-Pose3/Vector3 chart
// mismatch on the velocity block, and the sign of the bias cross term.
TEST(GateCovariance, PropagationMatchesSampling) {
  const gtsam::Rot3 R0 = gtsam::Rot3::RzRyRx(0.05, -0.11, 0.9);
  const gtsam::NavState prev(R0, gtsam::Point3(12.0, -3.0, 4.0),
                             gtsam::Vector3(7.0, 1.5, -0.2));
  const gtsam::imuBias::ConstantBias bias0(
      (gtsam::Vector3() << 0.02, -0.01, 0.03).finished(),
      (gtsam::Vector3() << 1e-3, -2e-3, 5e-4).finished());
  const gtsam::Point3 lever(0.593, 0.670, 1.216);

  auto params = gtsam::PreintegrationCombinedParams::MakeSharedU(9.81);
  params->setAccelerometerCovariance(gtsam::I_3x3 * 0.09);
  params->setGyroscopeCovariance(gtsam::I_3x3 * 1e-4);
  params->setIntegrationCovariance(gtsam::I_3x3 * 1e-8);
  params->biasAccCovariance = gtsam::I_3x3 * 1e-10;
  params->biasOmegaCovariance = gtsam::I_3x3 * 1e-12;
  gtsam::PreintegratedCombinedMeasurements pim(params, bias0);
  for (int i = 0; i < 40; ++i)
    pim.integrateMeasurement(gtsam::Vector3(0.4, -0.2, 9.9),
                             gtsam::Vector3(0.01, -0.02, 0.15), 0.005);

  // A departure covariance in GRAPH charts with real pose-velocity and
  // velocity-bias correlation - the blocks the additive form threw away.
  std::mt19937 rng(7);
  std::normal_distribution<double> g(0.0, 1.0);
  Eigen::MatrixXd A(15, 15);
  for (int i = 0; i < 15; ++i)
    for (int j = 0; j < 15; ++j) A(i, j) = g(rng);
  const Eigen::MatrixXd P = A * A.transpose() * 1e-4 +
                            Eigen::MatrixXd::Identity(15, 15) * 1e-5;

  // --- the code under test, transcribed from the gate block -----------------
  Eigen::MatrixXd T = Eigen::MatrixXd::Identity(15, 15);
  T.block<3, 3>(6, 6) = prev.pose().rotation().matrix().transpose();
  const Eigen::MatrixXd Pn = T * P * T.transpose();
  gtsam::Matrix99 F;
  gtsam::Matrix96 G;
  const gtsam::NavState predicted = pim.predict(prev, bias0, F, G);
  const Eigen::Matrix<double, 9, 9> cross =
      F * Pn.topRightCorner<9, 6>() * G.transpose();
  const Eigen::Matrix<double, 9, 9> pred9 =
      F * Pn.topLeftCorner<9, 9>() * F.transpose() + cross + cross.transpose() +
      G * Pn.bottomRightCorner<6, 6>() * G.transpose();  // Q_preint excluded

  gtsam::Matrix36 H_pose;
  const gtsam::Point3 ant0 = predicted.pose().transformFrom(lever, H_pose);
  Eigen::Matrix<double, 3, 9> J9 = Eigen::Matrix<double, 3, 9>::Zero();
  J9.leftCols<6>() = H_pose;
  const Eigen::Matrix3d analytic = J9 * pred9 * J9.transpose();

  // --- sampling -------------------------------------------------------------
  const Eigen::MatrixXd L = Eigen::LLT<Eigen::MatrixXd>(P).matrixL();
  const int N = 60000;
  std::vector<Eigen::Vector3d> e;
  e.reserve(N);
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (int n = 0; n < N; ++n) {
    Eigen::VectorXd z(15);
    for (int i = 0; i < 15; ++i) z(i) = g(rng);
    const Eigen::VectorXd d = L * z;
    const gtsam::NavState s(prev.pose().retract(d.head<6>()),
                            prev.velocity() + d.segment<3>(6));
    const gtsam::NavState pred_s =
        pim.predict(s, bias0.retract(d.segment<6>(9)));
    const Eigen::Vector3d dv =
        pred_s.pose().transformFrom(lever) - ant0;
    e.push_back(dv);
    mean += dv;
  }
  mean /= N;
  Eigen::Matrix3d sampled = Eigen::Matrix3d::Zero();
  for (const auto& v : e) sampled += (v - mean) * (v - mean).transpose();
  sampled /= (N - 1);

  EXPECT_LT((sampled - analytic).norm() / analytic.norm(), 0.03)
      << "analytic:\n" << analytic << "\nsampled:\n" << sampled;

  // The additive form the fix replaces is not a conservative approximation of
  // this: with positively-correlated pose and velocity it comes out SMALLER,
  // i.e. the old gate was over-confident and rejected good DDs.
  Eigen::Matrix<double, 9, 9> additive = Eigen::Matrix<double, 9, 9>::Zero();
  additive.topLeftCorner<6, 6>() = P.topLeftCorner<6, 6>();
  additive.bottomRightCorner<3, 3>() = P.block<3, 3>(6, 6);
  const double dt_h = pim.deltaTij();
  additive.block<3, 3>(3, 3) += (dt_h * dt_h) * P.block<3, 3>(6, 6);
  const Eigen::Matrix3d old_cov = J9 * additive * J9.transpose();
  EXPECT_LT(old_cov.diagonal().sum(), analytic.diagonal().sum())
      << "old " << old_cov.diagonal().sum() << " vs propagated " << analytic.diagonal().sum();
}

TEST(Chi2CriticalValue, IsCloseToTheTabulatedQuantiles) {
  // Wilson-Hilferty against textbook chi-square tables. z = 1.645 is the 95th
  // percentile of the standard normal, z = 2.326 the 99th.
  struct Case { int dof; double z; double expected; };
  const Case cases[] = {
      {5, 1.645, 11.07}, {10, 1.645, 18.31}, {20, 1.645, 31.41},
      {5, 2.326, 15.09}, {10, 2.326, 23.21}, {20, 2.326, 37.57},
  };
  for (const auto& c : cases) {
    const double got = gf::chi2CriticalValue(c.dof, c.z);
    EXPECT_NEAR(got, c.expected, 0.02 * c.expected)
        << "dof=" << c.dof << " z=" << c.z;
  }
  EXPECT_EQ(gf::chi2CriticalValue(0, 4.0), 0.0);
  // Monotone in both arguments, which the threshold logic silently relies on.
  EXPECT_GT(gf::chi2CriticalValue(10, 4.0), gf::chi2CriticalValue(10, 2.0));
  EXPECT_GT(gf::chi2CriticalValue(20, 3.0), gf::chi2CriticalValue(10, 3.0));
}

// ---------------------------------------------------------------------------
// AUDIT WP2.1 - finite-difference verification of the ONE custom factor.
//
// Everything else in these nodes uses stock GTSAM factors (AttitudeFactor,
// ExpressionFactor, PriorFactor), whose Jacobians GTSAM verifies itself.
// GroupedDdFactor is the only hand-written one, every GNSS measurement in both
// nodes flows through it, and it had no derivative test at all. The Pose3
// instantiation is the one that matters most: its J_ant = R_ecef_nav * H_pose
// carries the lever-arm rotation, which is where a frame or sign slip would sit
// and still look plausible in a trajectory plot.
namespace {

// A DD geometry that is well conditioned but not axis-aligned, so a transposed
// or permuted Jacobian cannot pass by symmetry.
gu::DdSignal fdDd(int sat_ref, int sat_tar, const Eigen::Vector3d& rover,
                  const Eigen::Vector3d& base, bool carrier) {
  gu::DdSignal d;
  d.sat_ref = sat_ref;
  d.sat_tar = sat_tar;
  d.band = 0;
  d.lam = 0.19029367279836487;  // GPS L1
  d.has_pr = true;
  d.has_cp = carrier;
  const double kR = 2.0e7;
  auto sat = [&](double a, double b) {
    return Eigen::Vector3d(kR * std::cos(a) * std::cos(b),
                           kR * std::sin(a) * std::cos(b), kR * std::sin(b));
  };
  d.sat_ref_rov = sat(0.3 + 0.11 * sat_ref, 0.7 - 0.03 * sat_ref);
  d.sat_tar_rov = sat(1.9 + 0.11 * sat_tar, 0.2 + 0.05 * sat_tar);
  d.sat_ref_base = d.sat_ref_rov + Eigen::Vector3d(11.0, -7.0, 5.0);
  d.sat_tar_base = d.sat_tar_rov + Eigen::Vector3d(-9.0, 13.0, -3.0);
  // Observations consistent with a slightly different position, so the residual
  // is non-zero (a zero residual can mask a Jacobian error in some factors).
  const Eigen::Vector3d truth = rover + Eigen::Vector3d(0.37, -0.21, 0.14);
  Eigen::Vector3d e;
  const double model =
      (gf::detail::geodistSagnac(d.sat_ref_rov, truth, &e) -
       gf::detail::geodistSagnac(d.sat_ref_base, base, &e)) -
      (gf::detail::geodistSagnac(d.sat_tar_rov, truth, &e) -
       gf::detail::geodistSagnac(d.sat_tar_base, base, &e));
  d.pr_rov_ref = model;  // the factor only uses the DD combination of these
  d.pr_base_ref = 0.0;
  d.pr_rov_tar = 0.0;
  d.pr_base_tar = 0.0;
  d.cp_rov_ref = model;
  d.cp_base_ref = 0.0;
  d.cp_rov_tar = 0.0;
  d.cp_base_tar = 0.0;
  return d;
}

// Central-difference Jacobian of unwhitenedError w.r.t. one key's tangent
// space, computed through Values so the factor is exercised exactly as GTSAM
// exercises it.
//
// The step is PER COLUMN and deliberately coarse for the metre axes. ECEF
// coordinates are ~2e7 m, so a central difference of a range has a roundoff
// floor of eps*|f|/h; at h = 1e-2 that is ~1e-7 and swamps the answer. The
// optimum for a range is h ~ (eps*|f|*r^2)^(1/3) ~ 0.5 m, which puts both the
// roundoff and the truncation term near 1e-8. Angles carry no such magnitude
// and want a small step. Getting this wrong makes a correct Jacobian look
// broken - which is exactly what the first run of these tests reported.
template <typename T>
gtsam::Matrix fdBlock(const gtsam::NoiseModelFactor& f,
                      const gtsam::Values& v0, gtsam::Key key,
                      const gtsam::Vector& steps, int rows) {
  const int dim = static_cast<int>(steps.size());
  gtsam::Matrix J(rows, dim);
  for (int c = 0; c < dim; ++c) {
    const double h = steps(c);
    gtsam::Vector delta = gtsam::Vector::Zero(dim);
    delta(c) = h;
    gtsam::Values vp = v0, vm = v0;
    vp.update(key, gtsam::traits<T>::Retract(v0.at<T>(key), delta));
    vm.update(key, gtsam::traits<T>::Retract(v0.at<T>(key), -delta));
    J.col(c) = (f.unwhitenedError(vp) - f.unwhitenedError(vm)) / (2.0 * h);
  }
  return J;
}

// Steps sized for the units of each tangent axis.
const gtsam::Vector3 kStepMetres = gtsam::Vector3::Constant(0.5);
const gtsam::Vector kStepPose3 =
    (gtsam::Vector(6) << 3e-3, 3e-3, 3e-3, 0.5, 0.5, 0.5).finished();

// GroupedDdFactor builds its state Jacobian from the line-of-sight unit vectors
// that detail::geodistSagnac returns, which are d(range)/d(rcv) WITHOUT the
// Sagnac term that the same function adds to its value. detail::
// geodistSagnacGradient - the derivative the code-DD WLS uses - does include it.
// The two are therefore inconsistent by OMGE*|sat|/CLIGHT ~ 5e-6 per rover
// satellite (AUDIT A2-1). That is a 7 ppm Jacobian error: it perturbs the
// Gauss-Newton step, never the residual, so the converged solution is
// unaffected - but it must be pinned, not left as an unexplained FD mismatch.
Eigen::RowVector3d sagnacPart(const Eigen::Vector3d& sat) {
  return Eigen::RowVector3d(-OMGE * sat.y() / CLIGHT, OMGE * sat.x() / CLIGHT,
                            0.0);
}
gtsam::Matrix sagnacCorrection(const std::vector<gf::GroupedDdRow>& rows) {
  gtsam::Matrix c(static_cast<int>(rows.size()), 3);
  for (std::size_t i = 0; i < rows.size(); ++i)
    c.row(static_cast<int>(i)) =
        sagnacPart(rows[i].dd.sat_ref_rov) - sagnacPart(rows[i].dd.sat_tar_rov);
  return c;
}

const gtsam::Point3 kBase(-3959000.0, 3353000.0, 3697000.0);
const gtsam::Point3 kRover(-3958900.0, 3353100.0, 3697050.0);

}  // namespace

TEST(GroupedDdFactorJacobian, Point3CodeMatchesFiniteDifference) {
  std::vector<gf::GroupedDdRow> rows;
  for (int i = 0; i < 4; ++i)
    rows.push_back({fdDd(1 + i, 11 + i, kRover, kBase, false), 0, 0});
  const gtsam::Key xk = gtsam::symbol_shorthand::X(0);
  gf::GroupedDdFactor<gtsam::Point3> f(
      xk, rows, false, kBase, gtsam::Point3(0, 0, 0), gtsam::Pose3(),
      gtsam::noiseModel::Isotropic::Sigma(4, 1.0));
  gtsam::Values v;
  v.insert(xk, kRover);

  std::vector<gtsam::Matrix> H;
  const gtsam::Vector e = f.unwhitenedError(v, &H);
  ASSERT_EQ(e.size(), 4);
  ASSERT_EQ(H.size(), 1U);
  EXPECT_GT(e.cwiseAbs().maxCoeff(), 1e-3) << "residual must not be trivially 0";
  const gtsam::Matrix fd = fdBlock<gtsam::Point3>(f, v, xk, kStepMetres, 4);
  // Agrees to 7 ppm as shipped, and EXACTLY once the omitted Sagnac gradient is
  // restored - which is what identifies the residue as A2-1 and not an error.
  EXPECT_LT((H[0] - fd).cwiseAbs().maxCoeff(), 2e-5)
      << "analytic\n" << H[0] << "\nfinite difference\n" << fd;
  EXPECT_LT((H[0] + sagnacCorrection(rows) - fd).cwiseAbs().maxCoeff(), 1e-8)
      << "the residue is not the Sagnac gradient - a real Jacobian error";
}

TEST(GroupedDdFactorJacobian, Point3CarrierMatchesFiniteDifference) {
  std::uint64_t next = 0;
  gf::PersistentAmbiguities amb(next);
  gtsam::Values v;
  const gtsam::Key xk = gtsam::symbol_shorthand::X(0);
  v.insert(xk, kRover);
  std::vector<gf::GroupedDdRow> rows;
  for (int i = 0; i < 3; ++i) {
    const gtsam::Key kr = obtain(amb, 1 + i, 0, false, 100.0 + i, v);
    const gtsam::Key kt = obtain(amb, 11 + i, 0, false, 100.0 + i, v);
    rows.push_back({fdDd(1 + i, 11 + i, kRover, kBase, true), kr, kt});
  }
  gf::GroupedDdFactor<gtsam::Point3> f(
      xk, rows, true, kBase, gtsam::Point3(0, 0, 0), gtsam::Pose3(),
      gtsam::noiseModel::Isotropic::Sigma(3, 1.0));

  std::vector<gtsam::Matrix> H;
  const gtsam::Vector e = f.unwhitenedError(v, &H);
  ASSERT_EQ(H.size(), f.keys().size());
  EXPECT_GT(e.cwiseAbs().maxCoeff(), 1e-3);
  const gtsam::Matrix fd_x = fdBlock<gtsam::Point3>(f, v, xk, kStepMetres, 3);
  EXPECT_LT((H[0] - fd_x).cwiseAbs().maxCoeff(), 2e-5);
  EXPECT_LT((H[0] + sagnacCorrection(rows) - fd_x).cwiseAbs().maxCoeff(), 1e-8);
  // Every ambiguity column too: a swapped ref/tar sign is invisible in the
  // state block but flips the integer the search converges on.
  for (std::size_t j = 1; j < f.keys().size(); ++j) {
    const gtsam::Matrix fd = fdBlock<double>(
        f, v, f.keys()[j], gtsam::Vector::Constant(1, 1e-3), 3);
    EXPECT_LT((H[j] - fd).cwiseAbs().maxCoeff(), 1e-9)
        << "ambiguity key index " << j;
  }
}

TEST(GroupedDdFactorJacobian, Pose3WithLeverArmMatchesFiniteDifference) {
  // Non-trivial attitude AND a non-zero lever, so the rotation part of J_ant is
  // genuinely exercised. ecef_T_nav is a real ENU->ECEF rotation, not identity.
  const gtsam::Rot3 R_nav_body =
      gtsam::Rot3::RzRyRx(0.07, -0.11, 1.31);          // roll, pitch, yaw
  const gtsam::Rot3 R_ecef_nav = gtsam::Rot3::RzRyRx(0.0, -1.02, 2.44);
  const gtsam::Pose3 ecef_T_nav(R_ecef_nav, kRover);
  const gtsam::Point3 lever(0.31, 0.0, -0.55);          // tokyo PPC lever
  const gtsam::Pose3 pose(R_nav_body, gtsam::Point3(3.0, -2.0, 1.0));
  const gtsam::Point3 ant_ecef = ecef_T_nav.transformFrom(
      pose.transformFrom(lever));

  std::vector<gf::GroupedDdRow> rows;
  for (int i = 0; i < 5; ++i)
    rows.push_back({fdDd(1 + i, 11 + i, ant_ecef, kBase, false), 0, 0});
  const gtsam::Key xk = gtsam::symbol_shorthand::X(0);
  gf::GroupedDdFactor<gtsam::Pose3> f(
      xk, rows, false, kBase, lever, ecef_T_nav,
      gtsam::noiseModel::Isotropic::Sigma(5, 1.0));
  gtsam::Values v;
  v.insert(xk, pose);

  std::vector<gtsam::Matrix> H;
  const gtsam::Vector e = f.unwhitenedError(v, &H);
  ASSERT_EQ(H.size(), 1U);
  ASSERT_EQ(H[0].cols(), 6);
  EXPECT_GT(e.cwiseAbs().maxCoeff(), 1e-3);
  const gtsam::Matrix fd = fdBlock<gtsam::Pose3>(f, v, xk, kStepPose3, 5);
  // Same 7 ppm Sagnac residue, mapped through J_ant. A genuine frame or sign
  // error in the lever-arm rotation would be O(0.1-2) here, five orders above.
  EXPECT_LT((H[0] - fd).cwiseAbs().maxCoeff(), 1e-4)
      << "analytic\n" << H[0] << "\nfinite difference\n" << fd;
  const gtsam::Matrix J_ant =
      ecef_T_nav.rotation().matrix() *
      [&] { gtsam::Matrix36 Hp; pose.transformFrom(lever, Hp); return Hp; }();
  EXPECT_LT((H[0] + sagnacCorrection(rows) * J_ant - fd).cwiseAbs().maxCoeff(),
            1e-5)
      << "the residue is not the Sagnac gradient - a real Jacobian error";
  // The rotation columns must actually carry signal, or the test would pass on
  // a Jacobian that simply ignored attitude.
  EXPECT_GT(H[0].leftCols<3>().cwiseAbs().maxCoeff(), 1e-3);
}

TEST(GroupedDdFactorJacobian, SagnacGradientMatchesFiniteDifference) {
  // detail::geodistSagnacGradient is the code-DD WLS's derivative of
  // detail::geodistSagnac. They must agree, including the Sagnac term.
  const Eigen::Vector3d sat(1.4e7, -1.9e7, 8.0e6);
  const Eigen::Vector3d rcv = kRover;
  const Eigen::Vector3d g = gf::detail::geodistSagnacGradient(sat, rcv);
  Eigen::Vector3d fd;
  const double h = 0.5;  // see fdBlock: the roundoff floor at ECEF magnitudes
  for (int c = 0; c < 3; ++c) {
    Eigen::Vector3d dp = Eigen::Vector3d::Zero();
    dp(c) = h;
    fd(c) = (gf::detail::geodistSagnac(sat, rcv + dp, nullptr) -
             gf::detail::geodistSagnac(sat, rcv - dp, nullptr)) / (2.0 * h);
  }
  EXPECT_LT((g - fd).cwiseAbs().maxCoeff(), 1e-8)
      << "analytic " << g.transpose() << " fd " << fd.transpose();
}

// One outage = one re-anchor. This latch lived in one node and not the other,
// and the node without it re-fired the gap re-anchor 1.8 to 203 times per
// outage EVENT (against 0.0-0.09 for the node with it) at essentially the same
// number of outages - destroying every carried carrier arc each time. It is
// shared now so the two nodes cannot drift apart again.
TEST(GapReanchorLatch, OneOutageFiresOnce) {
  gf::GapReanchorLatch latch;
  // A 200-epoch outage: the caller's starvation test is true throughout.
  int fired = 0;
  for (int i = 0; i < 200; ++i) {
    if (latch.shouldFire(true)) { ++fired; latch.arm(); }
  }
  EXPECT_EQ(fired, 1) << "the gap re-anchor re-fired within a single outage";
}

TEST(GapReanchorLatch, ALaterOutageFiresAgain) {
  gf::GapReanchorLatch latch;
  auto outage = [&latch](int epochs) {
    int fired = 0;
    for (int i = 0; i < epochs; ++i)
      if (latch.shouldFire(true)) { ++fired; latch.arm(); }
    return fired;
  };
  EXPECT_EQ(outage(50), 1);
  latch.onUsableGnss();          // DDs came back
  EXPECT_EQ(outage(50), 1) << "the latch did not re-arm after usable GNSS";
}

TEST(GapReanchorLatch, NeverFiresWhileTheCallerSaysNotStarved) {
  gf::GapReanchorLatch latch;
  for (int i = 0; i < 100; ++i) EXPECT_FALSE(latch.shouldFire(false));
  EXPECT_FALSE(latch.latched());
  // reset() is what a graph reset uses; it must leave the latch disarmed.
  latch.arm();
  latch.reset();
  EXPECT_TRUE(latch.shouldFire(true));
}

// --- Per-row Huber on a grouped DD factor -----------------------------------
//
// The defect these pin: gtsam::noiseModel::Robust reweights the BLOCK. It forms
// one w = min(1, k/||z||) from the whole group's whitened norm and multiplies it
// into every row, so a single bad satellite drags its groupmates down with it.
// Measured on the nagoya_run1 / tokyo_run1 code DDs, that is 8.76 % / 7.57 % of
// all DD rows down-weighted purely as collateral against 2.16 % / 1.30 %
// genuine outliers.
namespace {

// A DD group covariance with the shared-reference structure: vref everywhere,
// each target's own variance added on the diagonal.
Eigen::MatrixXd ddGroupCovariance(int n, double vref, double vtar) {
  Eigen::MatrixXd R = Eigen::MatrixXd::Constant(n, n, vref);
  for (int i = 0; i < n; ++i) R(i, i) += vtar;
  return R;
}

}  // namespace

// --- One-shot heading alignment ---------------------------------------------
//
// The defect these pin: alignYawToCourse replaces the heading OUTRIGHT, and the
// old gate was a bare speed threshold. Measured against truth on nagoya_run1 /
// tokyo_run1, 4 % of epochs at 1-1.5 m/s carry a course error beyond 20 deg
// (16-20 % below 0.2 m/s), so no speed threshold makes a single-epoch decision
// safe.
namespace {

constexpr double kDeg = M_PI / 180.0;

// A course sigma comfortably inside the default 10 deg gate.
constexpr double kGoodSigma = 3.0 * kDeg;
// A speed above reverse_speed_mps, so the reverse-driving bound never binds
// except in the test that is about it.
constexpr double kGoodSpeed = 5.0;

}  // namespace

TEST(YawAlignmentVote, RequiresConsecutiveAgreementBeforeItFires) {
  gf::YawAlignConfig cfg;  // 3 epochs, 20 deg spread, 10 deg sigma
  gf::YawAlignmentVote vote;
  double dpsi = 0.0, spread = 0.0;
  // A constant 80 deg offset, seen twice: not yet enough.
  EXPECT_FALSE(vote.update(80.0 * kDeg, 0.0, kGoodSigma, kGoodSpeed, cfg, &dpsi, &spread));
  EXPECT_FALSE(vote.update(81.0 * kDeg, 0.0, kGoodSigma, kGoodSpeed, cfg, &dpsi, &spread));
  EXPECT_TRUE(vote.update(79.0 * kDeg, 0.0, kGoodSigma, kGoodSpeed, cfg, &dpsi, &spread));
  EXPECT_NEAR(dpsi, 80.0 * kDeg, 1e-9);
  EXPECT_NEAR(spread, 2.0 * kDeg, 1e-9);
}

TEST(YawAlignmentVote, HoldsThroughATurnBecauseItVotesOnTheOFFSET) {
  // The property that makes the test usable at all: while the heading is merely
  // offset by a constant, the offset stays constant through a turn because the
  // yaw is carried by the gyro. Voting on the course itself would never agree.
  gf::YawAlignConfig cfg;
  gf::YawAlignmentVote vote;
  double dpsi = 0.0;
  bool fired = false;
  for (int i = 0; i < 3; ++i) {
    const double yaw = (10.0 + 25.0 * i) * kDeg;   // turning hard
    fired = vote.update(yaw + 90.0 * kDeg, yaw, kGoodSigma, kGoodSpeed, cfg, &dpsi);
  }
  EXPECT_TRUE(fired);
  EXPECT_NEAR(dpsi, 90.0 * kDeg, 1e-9);
}

TEST(YawAlignmentVote, AnIsolatedBlunderCannotSetTheHeading) {
  gf::YawAlignConfig cfg;
  gf::YawAlignmentVote vote;
  double dpsi = 0.0, spread = 0.0;
  vote.update(80.0 * kDeg, 0.0, kGoodSigma, kGoodSpeed, cfg);
  vote.update(120.0 * kDeg, 0.0, kGoodSigma, kGoodSpeed, cfg);  // the 4 %-of-epochs blunder
  // Spread is 40 deg > agree_deg, so nothing fires while it is in the window -
  // and it stays in for agree_epochs epochs, which is the point: a blunder
  // DELAYS the alignment rather than making a wrong one.
  EXPECT_FALSE(vote.update(81.0 * kDeg, 0.0, kGoodSigma, kGoodSpeed, cfg, &dpsi, &spread));
  EXPECT_GT(spread, cfg.agree_deg * kDeg);
  EXPECT_FALSE(vote.update(79.0 * kDeg, 0.0, kGoodSigma, kGoodSpeed, cfg, &dpsi, &spread));
  // Once it has aged out, the honest votes agree again.
  EXPECT_TRUE(vote.update(80.5 * kDeg, 0.0, kGoodSigma, kGoodSpeed, cfg, &dpsi, &spread));
  EXPECT_NEAR(dpsi, 80.5 * kDeg, 1e-9);
  EXPECT_LT(spread, cfg.agree_deg * kDeg);
}

TEST(YawAlignmentVote, BlunderInsideTheToleranceIsOutvotedByTheMedian) {
  gf::YawAlignConfig cfg;
  cfg.agree_deg = 45.0;  // deliberately loose so the outlier stays in
  gf::YawAlignmentVote vote;
  double dpsi = 0.0;
  vote.update(80.0 * kDeg, 0.0, kGoodSigma, kGoodSpeed, cfg);
  vote.update(115.0 * kDeg, 0.0, kGoodSigma, kGoodSpeed, cfg);
  EXPECT_TRUE(vote.update(82.0 * kDeg, 0.0, kGoodSigma, kGoodSpeed, cfg, &dpsi));
  EXPECT_NEAR(dpsi, 82.0 * kDeg, 1e-9) << "the median, not the mean";
}

TEST(YawAlignmentVote, PoorCourseQualityProducesNoCandidateAtAll) {
  gf::YawAlignConfig cfg;
  gf::YawAlignmentVote vote;
  const double bad_sigma = 25.0 * kDeg;  // e.g. 0.1 m/s with sigma_vel 0.1 m/s
  for (int i = 0; i < 10; ++i)
    EXPECT_FALSE(vote.update(80.0 * kDeg, 0.0, bad_sigma, kGoodSpeed, cfg));
  EXPECT_EQ(vote.size(), 0);
  // ... and a bad epoch clears the votes either side of it, because the test is
  // about CONSECUTIVE agreement.
  vote.update(80.0 * kDeg, 0.0, kGoodSigma, kGoodSpeed, cfg);
  vote.update(80.0 * kDeg, 0.0, kGoodSigma, kGoodSpeed, cfg);
  vote.update(80.0 * kDeg, 0.0, bad_sigma, kGoodSpeed, cfg);
  EXPECT_EQ(vote.size(), 0);
  EXPECT_FALSE(vote.update(80.0 * kDeg, 0.0, kGoodSigma, kGoodSpeed, cfg));
}

TEST(YawAlignmentVote, AgreesAcrossTheWrapBoundary) {
  gf::YawAlignConfig cfg;
  gf::YawAlignmentVote vote;
  double dpsi = 0.0, spread = 0.0;
  // Offsets of +179, -179 and +180 deg are 2 deg apart, not 358.
  vote.update(179.0 * kDeg, 0.0, kGoodSigma, kGoodSpeed, cfg);
  vote.update(-179.0 * kDeg, 0.0, kGoodSigma, kGoodSpeed, cfg);
  EXPECT_TRUE(vote.update(180.0 * kDeg, 0.0, kGoodSigma, kGoodSpeed, cfg, &dpsi, &spread));
  EXPECT_NEAR(spread, 2.0 * kDeg, 1e-9);
  EXPECT_NEAR(std::fabs(dpsi), 180.0 * kDeg, 1e-9);
}

TEST(YawAlignmentVote, TheCourseSigmaGateCorrespondsToTheDocumentedSpeed) {
  // The gate is stated in degrees so it compares against align_reanchor_deg,
  // but it has to remain readable as a speed. With the sigma_vel floor of
  // 0.1 m/s the default 10 deg gate opens at ~0.57 m/s.
  gf::YawAlignConfig cfg;
  const double sideslip = 2.0 * kDeg;
  EXPECT_GT(gf::velocityAttitudeSigma(0.5, 0.1, sideslip), cfg.max_sigma_deg * kDeg);
  EXPECT_LT(gf::velocityAttitudeSigma(0.6, 0.1, sideslip), cfg.max_sigma_deg * kDeg);
  // A poor Doppler epoch is rejected even when it is fast.
  EXPECT_GT(gf::velocityAttitudeSigma(5.0, 1.5, sideslip), cfg.max_sigma_deg * kDeg);
}

TEST(YawAlignmentVote, WillNotTakeAReversingVehiclesCourseAsTheHeading) {
  // The trap a course-QUALITY gate cannot see. A vehicle backing out produces a
  // course exactly 180 deg from its heading, precisely and repeatably, so it
  // passes both the sigma gate and the agreement test with room to spare.
  // Measured on the truth trajectories: nagoya_run1 reverses out of a parking
  // space over t = 48.0-56.2 s at up to 1.18 m/s - which is exactly the window
  // in which that run's heading first becomes observable - and none of its 133
  // reversing epochs exceed 3.0 m/s.
  gf::YawAlignConfig cfg;
  gf::YawAlignmentVote vote;
  double dpsi = 0.0, spread = 0.0;
  const double crawl = 1.18;  // the measured peak reversing speed
  bool fired = false;
  for (int i = 0; i < 5; ++i)
    fired = vote.update(94.0 * kDeg, -85.0 * kDeg, kGoodSigma, crawl, cfg,
                        &dpsi, &spread);
  EXPECT_FALSE(fired) << "a 179 deg offset at a crawl is reverse driving";
  EXPECT_LT(spread, cfg.agree_deg * kDeg)
      << "and it AGREED - only the speed bound rejects it";

  // Above the bound, sustained reverse driving is ruled out, so the same offset
  // is believed: that is the case of a genuinely unknown initial heading.
  gf::YawAlignmentVote fast;
  for (int i = 0; i < 3; ++i)
    fired = fast.update(94.0 * kDeg, -85.0 * kDeg, kGoodSigma,
                        cfg.reverse_speed_mps + 0.1, cfg, &dpsi);
  EXPECT_TRUE(fired);
  EXPECT_NEAR(std::fabs(dpsi), 179.0 * kDeg, 1e-9);
}

TEST(YawAlignmentVote, AnUnambiguousOffsetIsStillTakenAtLowSpeed) {
  // The bound must not cost the case the whole mechanism exists for. The
  // measured initial offsets are 78 deg (nagoya_run1) and 137 deg (tokyo_run1);
  // the first is unambiguous and must be usable as soon as the course is good,
  // the second is not and has to wait for speed.
  gf::YawAlignConfig cfg;
  double dpsi = 0.0;
  gf::YawAlignmentVote slow;
  bool fired = false;
  for (int i = 0; i < 3; ++i)
    fired = slow.update(78.0 * kDeg, 0.0, kGoodSigma, 0.6, cfg, &dpsi);
  EXPECT_TRUE(fired) << "78 deg cannot be reverse driving";
  EXPECT_NEAR(dpsi, 78.0 * kDeg, 1e-9);

  gf::YawAlignmentVote tokyo;
  for (int i = 0; i < 3; ++i)
    fired = tokyo.update(137.0 * kDeg, 0.0, kGoodSigma, 0.6, cfg, &dpsi);
  EXPECT_FALSE(fired) << "137 deg at a crawl is not distinguishable";
  for (int i = 0; i < 3; ++i)
    fired = tokyo.update(137.0 * kDeg, 0.0, kGoodSigma, 3.6, cfg, &dpsi);
  EXPECT_TRUE(fired) << "... and is taken once the speed rules reversing out";
  EXPECT_NEAR(dpsi, 137.0 * kDeg, 1e-9);
}

TEST(YawAlignmentVote, EveryCandidateMustClearTheReverseBoundNotJustTheMedian) {
  // A window straddling the moment the vehicle pulls away must not be admitted
  // on the strength of its faster epochs alone.
  gf::YawAlignConfig cfg;
  gf::YawAlignmentVote vote;
  double dpsi = 0.0;
  vote.update(94.0 * kDeg, -85.0 * kDeg, kGoodSigma, 0.9, cfg);
  vote.update(94.0 * kDeg, -85.0 * kDeg, kGoodSigma, 4.0, cfg);
  EXPECT_FALSE(
      vote.update(94.0 * kDeg, -85.0 * kDeg, kGoodSigma, 4.0, cfg, &dpsi));
  // Once the crawling epoch ages out, the window is uniformly fast.
  EXPECT_TRUE(
      vote.update(94.0 * kDeg, -85.0 * kDeg, kGoodSigma, 4.0, cfg, &dpsi));
}

// --- Doppler velocity: redundancy, re-test and conditioning ------------------
//
// The defects these pin, all from the Review.txt pass:
//   - four satellites against four unknowns is zero redundancy, so the residuals
//     are identically zero and out.res - the caller's gross-outlier gate - reads
//     a perfect 0 however bad a range rate is. A blunder does not merely escape
//     detection, it looks ideal.
//   - the exclusion loop stopped at five rows WITHOUT re-testing, so "excluded a
//     satellite and the fault is still there" was reported as a clean velocity.
//   - N = H'WH squares the condition number and N.inverse() returns a huge but
//     finite matrix on a degenerate geometry, which allFinite() waves through.
namespace {

// A well-conditioned constellation: satellites spread over the sky above the
// rover, with a known true velocity and clock drift imprinted on the Doppler.
std::vector<gu::SatObs> dopplerScenario(int n, const Eigen::Vector3d& v_true,
                                        double drift_true,
                                        const Eigen::Vector3d& rr,
                                        double spread = 1.0) {
  std::vector<gu::SatObs> sats;
  const double lam = 0.19029367;
  for (int i = 0; i < n; ++i) {
    gu::SatObs s;
    s.sat = i + 1;
    s.sys = SYS_GPS;
    s.band = 0;
    s.lam = lam;
    const double az = 2.0 * M_PI * i / n;
    const double el = (20.0 + 55.0 * ((i % 3) / 2.0) * spread) * M_PI / 180.0;
    const Eigen::Vector3d up = rr.normalized();
    Eigen::Vector3d east = up.cross(Eigen::Vector3d::UnitZ()).normalized();
    Eigen::Vector3d north = up.cross(east);
    const Eigen::Vector3d e =
        (std::cos(el) * (std::cos(az) * north + std::sin(az) * east) +
         std::sin(el) * up).normalized();
    s.sat_pos = rr + e * 2.2e7;
    s.sat_vel = Eigen::Vector3d(1000.0 * std::cos(az), 1000.0 * std::sin(az),
                                800.0);
    s.sat_clk_drift = 0.0;
    s.el = el;
    // Invert the model in estimateDopplerVelocity so the solve recovers exactly.
    const double omge_c = OMGE / CLIGHT;
    const double sagnac =
        omge_c * (s.sat_vel.y() * rr.x() - s.sat_vel.x() * rr.y());
    const double model = e.dot(s.sat_vel - v_true) + sagnac + drift_true;
    s.doppler = -model / lam;
    sats.push_back(s);
  }
  return sats;
}

}  // namespace

TEST(DopplerVelocity, FourSatellitesAreRefusedBecauseTheyCannotBeChecked) {
  const Eigen::Vector3d rr(-3810241.0, 3567865.0, 3652890.0);
  const Eigen::Vector3d v_true(3.0, -1.0, 0.5);
  gu::DopplerVelocitySolution out;
  // Four rows: exactly determined, residuals identically zero. Refused.
  EXPECT_FALSE(estimateDopplerVelocity(dopplerScenario(4, v_true, 12.0, rr), rr,
                                       0.1, 4.0, 2, out));
  // Five rows: one degree of freedom, enough to notice a fault. Accepted.
  EXPECT_TRUE(estimateDopplerVelocity(dopplerScenario(5, v_true, 12.0, rr), rr,
                                      0.1, 4.0, 2, out));
  // 1e-4 relative, not 1e-12: the design matrix carries the Sagnac terms
  // exactly while this scenario imprints them as a constant, so the recovery is
  // physically exact rather than algebraically so (measured ~5e-6 m/s).
  EXPECT_TRUE(out.vel.isApprox(v_true, 1e-4)) << out.vel.transpose();
  EXPECT_EQ(out.nsat, 5);
}

TEST(DopplerVelocity, AFaultThatSurvivesExclusionRejectsTheEpoch) {
  const Eigen::Vector3d rr(-3810241.0, 3567865.0, 3652890.0);
  const Eigen::Vector3d v_true(3.0, -1.0, 0.5);
  gu::DopplerVelocitySolution out;

  // One blunder among seven: excluded, the rest agree, velocity recovered.
  auto one_bad = dopplerScenario(7, v_true, 12.0, rr);
  one_bad[2].doppler += 40.0 / one_bad[2].lam;  // 40 m/s range-rate blunder
  ASSERT_TRUE(estimateDopplerVelocity(one_bad, rr, 0.1, 4.0, 2, out));
  EXPECT_EQ(out.excluded, 1);
  EXPECT_TRUE(out.vel.isApprox(v_true, 1e-4)) << out.vel.transpose();

  // More blunders than the budget allows: the loop cannot clear the fault, so
  // the EPOCH is rejected rather than a still-faulty velocity being published.
  auto many_bad = dopplerScenario(7, v_true, 12.0, rr);
  many_bad[1].doppler += 40.0 / many_bad[1].lam;
  many_bad[3].doppler -= 55.0 / many_bad[3].lam;
  many_bad[5].doppler += 70.0 / many_bad[5].lam;
  EXPECT_FALSE(estimateDopplerVelocity(many_bad, rr, 0.1, 4.0, 1, out))
      << "budget 1 cannot clear three blunders; the epoch must be refused";
}

TEST(DopplerVelocity, FiveSatellitesWithAFaultRejectTheEpochRatherThanTrim) {
  const Eigen::Vector3d rr(-3810241.0, 3567865.0, 3652890.0);
  const Eigen::Vector3d v_true(3.0, -1.0, 0.5);
  auto bad = dopplerScenario(5, v_true, 12.0, rr);
  bad[0].doppler += 50.0 / bad[0].lam;
  gu::DopplerVelocitySolution out;
  // At one degree of freedom the fault is detectable but not isolatable.
  // Trimming to four would manufacture confidence, so the answer is no velocity.
  EXPECT_FALSE(estimateDopplerVelocity(bad, rr, 0.1, 4.0, 2, out));
}

TEST(DopplerVelocity, ADegenerateGeometryIsRejectedNotSilentlyInverted) {
  const Eigen::Vector3d rr(-3810241.0, 3567865.0, 3652890.0);
  const Eigen::Vector3d v_true(3.0, -1.0, 0.5);
  // Collapse the constellation into a tight cluster: the four unknowns are no
  // longer separable. N.inverse() would return a huge but finite matrix.
  auto sats = dopplerScenario(8, v_true, 12.0, rr);
  const Eigen::Vector3d e0 = (sats[0].sat_pos - rr).normalized();
  for (auto& s : sats) {
    const Eigen::Vector3d e = (s.sat_pos - rr).normalized();
    s.sat_pos = rr + (e0 + 1e-7 * (e - e0)).normalized() * 2.2e7;
    s.el = 1.4;
  }
  gu::DopplerVelocitySolution out;
  EXPECT_FALSE(estimateDopplerVelocity(sats, rr, 0.1, 4.0, 2, out));
}

// --- Pre-fit global test: chi-square quantile, not chi2/n --------------------

TEST(PreFitGlobalTest, ChiSquareQuantileCatchesWhatChiSquareOverNMisses) {
  // The reduced form chi2/n > nsigma^2 does not scale with n, so it grows
  // steadily more permissive as satellites are added - which is exactly
  // backwards. Several MODERATE faults, each below the per-row w-test, are what
  // an urban multipath epoch looks like, and they are what it lets through.
  const double nsigma = 4.0;
  for (const int n : {10, 20, 30}) {
    const double reduced_bar = n * nsigma * nsigma;      // old: chi2 > n*16
    const double quantile_bar = gf::chi2CriticalValue(n, nsigma);
    EXPECT_LT(quantile_bar, reduced_bar)
        << "at n=" << n << " the reduced form is looser";
    // A chi2 landing between the two is a set the old test passed and the new
    // one rejects. At n = 20 that band is roughly 45 .. 320.
    const double between = 0.5 * (quantile_bar + reduced_bar);
    EXPECT_GT(between, quantile_bar);
    EXPECT_LT(between, reduced_bar);
  }
  // The quantile tracks the degrees of freedom; the reduced bar's ratio to it
  // widens with n, which is the mis-calibration itself.
  const double r10 = (10 * 16.0) / gf::chi2CriticalValue(10, 4.0);
  const double r30 = (30 * 16.0) / gf::chi2CriticalValue(30, 4.0);
  EXPECT_GT(r30, r10);
}

// --- Dead-reckoning covariance propagation -----------------------------------
//
// The review's criterion, and it is the right one: "larger than the old
// approximation" proves nothing, because any inflation passes it. What has to
// hold is agreement with the distribution the propagation claims to describe.
// These check the 9x9 NavState-chart propagation
//   P+ = F Pxx F' + F Pxb G' + (.)' + G Pbb G' + preintMeasCov
// which is what predictedNavCov9 assembles, against Monte Carlo, PSD, and
// continuity at the start of a gap.
namespace {

gtsam::PreintegratedCombinedMeasurements makePim(double dt, int steps,
                                                 const gtsam::imuBias::ConstantBias& b) {
  auto p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(9.80665);
  p->setAccelerometerCovariance(gtsam::I_3x3 * 1e-4);
  p->setGyroscopeCovariance(gtsam::I_3x3 * 1e-6);
  p->setIntegrationCovariance(gtsam::I_3x3 * 1e-8);
  p->setBiasAccCovariance(gtsam::I_3x3 * 1e-10);
  p->setBiasOmegaCovariance(gtsam::I_3x3 * 1e-12);
  p->setBiasAccOmegaInit(gtsam::I_6x6 * 1e-10);
  gtsam::PreintegratedCombinedMeasurements pim(p, b);
  const gtsam::Vector3 acc(0.2, -0.1, 9.80665), gyr(0.01, -0.02, 0.005);
  for (int i = 0; i < steps; ++i) pim.integrateMeasurement(acc, gyr, dt);
  return pim;
}

// The propagation under test, written out independently of the node so the test
// does not merely restate the implementation.
Eigen::Matrix<double, 9, 9> propagate9(
    const gtsam::PreintegratedCombinedMeasurements& pim,
    const gtsam::NavState& prev, const gtsam::imuBias::ConstantBias& bias,
    const Eigen::MatrixXd& P15) {
  gtsam::Matrix99 F;
  gtsam::Matrix96 G;
  pim.predict(prev, bias, F, G);
  const Eigen::Matrix<double, 9, 9> Pxx = P15.topLeftCorner<9, 9>();
  const Eigen::Matrix<double, 9, 6> Pxb = P15.topRightCorner<9, 6>();
  const Eigen::Matrix<double, 6, 6> Pbb = P15.bottomRightCorner<6, 6>();
  const Eigen::Matrix<double, 9, 9> cross = F * Pxb * G.transpose();
  Eigen::Matrix<double, 9, 9> out = F * Pxx * F.transpose() + cross +
                                    cross.transpose() + G * Pbb * G.transpose() +
                                    pim.preintMeasCov().topLeftCorner<9, 9>();
  return 0.5 * (out + out.transpose());
}

}  // namespace

TEST(PredictedCovariance, MatchesMonteCarloOverTheStateAndBiasUncertainty) {
  const gtsam::NavState prev(gtsam::Rot3::RzRyRx(0.02, -0.01, 0.5),
                             gtsam::Point3(10.0, -5.0, 2.0),
                             gtsam::Vector3(4.0, 1.0, -0.2));
  const gtsam::imuBias::ConstantBias bias(gtsam::Vector3(0.01, -0.02, 0.005),
                                          gtsam::Vector3(1e-3, -2e-3, 5e-4));
  const auto pim = makePim(0.01, 40, bias);

  // A posterior with genuine X/V/B CROSS terms - the part the old dead-reckoning
  // covariance dropped entirely, and the part a block-diagonal test would miss.
  //
  // Scaled to a REALISTIC posterior (mrad attitude, cm-dm position, cm/s
  // velocity). That is not cosmetic: the propagation is a first-order form, so
  // comparing it to Monte Carlo only tests what it claims in the regime where
  // linearization holds. At 0.1 rad of attitude uncertainty the second-order
  // terms are worth ~13 % on their own and the test would be measuring the
  // linearization error, not the propagation.
  Eigen::MatrixXd A = Eigen::MatrixXd::Random(15, 15);
  Eigen::MatrixXd P15 = A * A.transpose() + Eigen::MatrixXd::Identity(15, 15);
  Eigen::VectorXd scale(15);
  scale << 1e-3, 1e-3, 1e-3,      // attitude [rad]
      5e-2, 5e-2, 5e-2,           // position [m]
      2e-2, 2e-2, 2e-2,           // velocity [m/s]
      1e-3, 1e-3, 1e-3,           // accel bias [m/s^2]
      1e-4, 1e-4, 1e-4;           // gyro bias [rad/s]
  P15 = scale.asDiagonal() * P15 * scale.asDiagonal();
  const Eigen::Matrix<double, 9, 9> pred = propagate9(pim, prev, bias, P15);

  // Sample the SAME uncertainty and push it through the nonlinear prediction.
  const Eigen::LLT<Eigen::MatrixXd> llt(P15);
  ASSERT_EQ(llt.info(), Eigen::Success);
  std::mt19937 rng(20260812);
  std::normal_distribution<double> nd(0.0, 1.0);
  const int N = 40000;
  Eigen::MatrixXd samples(9, N);
  const gtsam::NavState mean_pred = pim.predict(prev, bias);
  for (int s = 0; s < N; ++s) {
    Eigen::VectorXd g(15);
    for (int i = 0; i < 15; ++i) g(i) = nd(rng);
    const Eigen::VectorXd d = llt.matrixL() * g;
    // Perturb through NavState's OWN retraction. Its chart applies the position
    // and velocity increments in the BODY frame, which is the chart F from
    // pim.predict() is expressed in; adding them globally instead samples a
    // different chart and the comparison is then meaningless (measured: 27 %
    // mismatch from this alone, and it does not shrink with the uncertainty).
    const gtsam::NavState prev_s = prev.retract(d.head<9>());
    const gtsam::imuBias::ConstantBias bias_s(
        bias.accelerometer() + d.segment<3>(9),
        bias.gyroscope() + d.segment<3>(12));
    samples.col(s) = mean_pred.localCoordinates(pim.predict(prev_s, bias_s));
  }
  const Eigen::VectorXd mu = samples.rowwise().mean();
  const Eigen::MatrixXd centred = samples.colwise() - mu;
  // The sample covariance carries only the STATE uncertainty; preintMeasCov is
  // the IMU-noise term the propagation adds on top, so subtract it to compare.
  const Eigen::Matrix<double, 9, 9> analytic =
      pred - pim.preintMeasCov().topLeftCorner<9, 9>();
  const Eigen::MatrixXd mc = centred * centred.transpose() / (N - 1);

  // Relative agreement on the diagonal. The Monte Carlo standard error on a
  // variance is sqrt(2/N) ~ 0.7 % at N = 40000; allow 5 % for the nonlinearity
  // the analytic form linearizes away.
  for (int i = 0; i < 9; ++i) {
    EXPECT_NEAR(mc(i, i), analytic(i, i), 0.05 * std::fabs(analytic(i, i)))
        << "diagonal " << i;
  }
  EXPECT_LT((mc - analytic).norm() / analytic.norm(), 0.05)
      << "\nanalytic:\n" << analytic << "\nmonte carlo:\n" << mc;
}

TEST(PredictedCovariance, IsPsdAndContinuousAtTheStartOfAGap) {
  const gtsam::NavState prev(gtsam::Rot3::RzRyRx(0.02, -0.01, 0.5),
                             gtsam::Point3(10.0, -5.0, 2.0),
                             gtsam::Vector3(4.0, 1.0, -0.2));
  const gtsam::imuBias::ConstantBias bias(gtsam::Vector3(0.01, -0.02, 0.005),
                                          gtsam::Vector3(1e-3, -2e-3, 5e-4));
  Eigen::MatrixXd A = Eigen::MatrixXd::Random(15, 15);
  const Eigen::MatrixXd P15 = A * A.transpose() * 1e-4 +
                              Eigen::MatrixXd::Identity(15, 15) * 1e-5;

  // Continuity: as the horizon goes to zero the propagated covariance must
  // return the posterior it started from, in the NavState chart. A propagation
  // that jumped here would be publishing a discontinuity at every gap onset.
  const auto pim0 = makePim(0.01, 1, bias);
  const Eigen::Matrix<double, 9, 9> near0 = propagate9(pim0, prev, bias, P15);
  const Eigen::MatrixXd P9 = P15.topLeftCorner(9, 9);
  // Continuity means the drift VANISHES with the horizon, not that it is under
  // some fixed bar: one 10 ms step already carries real preintegration noise.
  double prev_drift = 1.0;
  for (const int steps : {8, 4, 2, 1}) {
    const auto pim_s = makePim(0.01, steps, bias);
    const Eigen::Matrix<double, 9, 9> P = propagate9(pim_s, prev, bias, P15);
    const double drift = (P - P9).norm() / P9.norm();
    EXPECT_LT(drift, prev_drift) << "drift did not shrink at " << steps
                                 << " steps";
    prev_drift = drift;
  }
  EXPECT_LT(prev_drift, 0.10) << "one step already jumps away from the posterior";
  (void)near0;

  // PSD, and monotonically growing with the horizon: dead reckoning cannot
  // become more certain by coasting for longer.
  double prev_trace = 0.0;
  for (const int steps : {1, 10, 50, 200}) {
    const auto pim = makePim(0.01, steps, bias);
    const Eigen::Matrix<double, 9, 9> P = propagate9(pim, prev, bias, P15);
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9>> es(P);
    ASSERT_EQ(es.info(), Eigen::Success);
    EXPECT_GE(es.eigenvalues().minCoeff(), -1e-9 * es.eigenvalues().maxCoeff())
        << "not PSD at " << steps << " steps";
    // NOTE: rtklib.h #defines trace(), so Eigen's member cannot be called here.
    const double tr = P.diagonal().sum();
    EXPECT_GT(tr, prev_trace) << "shrank at " << steps << " steps";
    prev_trace = tr;
  }
}

// --- Post-fit FDE: the covariance the posterior was ACTUALLY formed under ----
//
// Pinned here: the test must use the weights the graph applied (R_eff = R / w),
// the covariance recorded in the layout rather than a re-derived one, and a
// single fail-closed behaviour for every "cannot decide" path.
namespace {

// A layout whose group mirrors what a factor would have been built with.
gf::DdFactorLayout layoutFor(const gu::PreprocessedEpoch& ep, bool carrier,
                             const gf::AdapterConfig& cfg) {
  gf::DdFactorLayout out;
  gf::DdFactorLayout::Group g;
  g.carrier = carrier;
  std::vector<gu::DdSignal> signals;
  for (int i = 0; i < static_cast<int>(ep.dd.size()); ++i) {
    g.dd_index.push_back(i);
    signals.push_back(ep.dd[i]);
  }
  g.nominal_cov = gf::groupedDdCovariance(
      signals, carrier ? cfg.cp_sigma_m : cfg.pr_sigma_m, cfg);
  out.groups.push_back(std::move(g));
  out.valid = true;
  return out;
}

}  // namespace

TEST(PostFitWeights, BlockScopeIsReproducedNotLeftOnTheNominalR) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(6);
  gf::AdapterConfig cfg;
  cfg.robust = true;
  const gf::DdFactorLayout layout = layoutFor(sc.ep, false, cfg);

  // Far enough off that the group trips the loss. It has to be large: a DD
  // residual is differential, so a position error common to both satellites
  // largely cancels and only the geometry difference survives.
  gf::GraphArPosterior post;
  post.lin_ant = sc.rover + Eigen::Vector3d(60.0, 25.0, -10.0);
  post.lin_valid = true;

  const gf::FrontEndRobustWeights few =
      gf::frontEndRobustWeights(sc.ep, {}, post, layout, cfg);
  ASSERT_TRUE(few.valid) << "Block must be reproducible, not skipped";

  // Recompute the expected Block weight independently: gtsam whitens with
  // chol(R)^-1 and forms ONE weight from the norm of the whole group.
  const Eigen::MatrixXd& R = layout.groups[0].nominal_cov;
  const int n = static_cast<int>(R.rows());
  Eigen::VectorXd r(n);
  for (int i = 0; i < n; ++i)
    r(i) = gf::detail::ddPseudorangeResidual(sc.ep.dd[i], sc.ep.base_ecef,
                                             post.lin_ant);
  const Eigen::MatrixXd L = Eigen::LLT<Eigen::MatrixXd>(R).matrixL();
  const double nz = L.triangularView<Eigen::Lower>().solve(r).norm();
  ASSERT_GT(nz, cfg.huber_k) << "scenario must actually trip the loss";
  const double w = cfg.huber_k / nz;

  int row = 0;
  const Eigen::MatrixXd* blk = few.block(0, false, &row);
  ASSERT_NE(blk, nullptr);
  EXPECT_TRUE(blk->isApprox(R / w, 1e-9))
      << "Block R_eff must be R / w, not the nominal R";
  // The regression itself: it must NOT be the nominal R.
  EXPECT_FALSE(blk->isApprox(R, 1e-6));
}

TEST(PostFitWeights, WeightsFollowTheLinearizationPointNotTheFixResidual) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(6);
  gf::AdapterConfig cfg;
  cfg.robust = true;
  const gf::DdFactorLayout layout = layoutFor(sc.ep, false, cfg);

  gf::GraphArPosterior near, far;
  near.lin_ant = sc.rover;
  near.lin_valid = true;
  far.lin_ant = sc.rover + Eigen::Vector3d(60.0, 25.0, -10.0);
  far.lin_valid = true;

  const auto a = gf::frontEndRobustWeights(sc.ep, {}, near, layout, cfg);
  const auto b = gf::frontEndRobustWeights(sc.ep, {}, far, layout, cfg);
  ASSERT_TRUE(a.valid && b.valid);
  int r0 = 0, r1 = 0;
  // Moving the linearization point must move R_eff. If the weights were being
  // recomputed from the FIX residuals instead, this would not react.
  EXPECT_FALSE(a.block(0, false, &r0)->isApprox(*b.block(0, false, &r1), 1e-6));
}

TEST(PostFitWeights, NoLinearizationPointOrNoLayoutMeansNoWeights) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(5);
  gf::AdapterConfig cfg;
  cfg.robust = true;
  const gf::DdFactorLayout good = layoutFor(sc.ep, false, cfg);

  gf::GraphArPosterior no_lin;  // lin_valid stays false
  EXPECT_FALSE(gf::frontEndRobustWeights(sc.ep, {}, no_lin, good, cfg).valid);

  gf::GraphArPosterior post;
  post.lin_ant = sc.rover;
  post.lin_valid = true;
  const gf::DdFactorLayout no_layout;  // valid stays false
  EXPECT_FALSE(
      gf::frontEndRobustWeights(sc.ep, {}, post, no_layout, cfg).valid);
}

TEST(PostFitWeights, LayoutCarriesTheFactorsOwnCovarianceNotARederivedOne) {
  // Row indices alone cannot pin R: it is built from cfg/sigma at add time. A
  // layout that stored only indices would silently agree with a DIFFERENT cfg.
  SyntheticDdScenario sc = makeSyntheticDdScenario(5);
  gf::AdapterConfig built;
  built.robust = false;
  built.base_reuse_factor = 5.0;
  const gf::DdFactorLayout layout = layoutFor(sc.ep, false, built);

  gf::AdapterConfig other = built;
  other.base_reuse_factor = 1.0;  // a cfg the validation might be handed later
  std::vector<gu::DdSignal> signals(sc.ep.dd.begin(), sc.ep.dd.end());
  const Eigen::MatrixXd rederived =
      gf::groupedDdCovariance(signals, other.pr_sigma_m, other);
  EXPECT_FALSE(layout.groups[0].nominal_cov.isApprox(rederived, 1e-9))
      << "the two must differ, or this test proves nothing";

  // The stored matrix is the one the factor used, whatever cfg arrives later.
  gf::GraphArPosterior post;
  post.lin_ant = sc.rover;
  post.lin_valid = true;
  const auto few = gf::frontEndRobustWeights(sc.ep, {}, post, layout, other);
  ASSERT_TRUE(few.valid);
  int row = 0;
  EXPECT_TRUE(few.block(0, false, &row)->isApprox(
      layout.groups[0].nominal_cov, 1e-12))
      << "robust=false must reproduce the STORED R, not a re-derived one";
}

// --- The FDE exit itself, and the paths that feed it -------------------------
//
// Everything above tests the pieces. These test the OUTPUT: that
// resolveAmbiguitiesPosterior actually refuses the fix. Until now the suite
// verified frontEndRobustWeights().valid and stopped there, so the one behaviour
// the whole post-fit test exists for - saying no - had never been exercised.

TEST(FdeVerdict, EveryUndecidablePathRejectsTheFix) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(8);
  gf::AdapterConfig cfg;
  gf::ArOptions opt;
  opt.min_fix = 4;
  opt.min_lock = 0;
  opt.el_mask_rad = 0.0;
  opt.max_pos_var_m2 = 0.0;
  opt.partial_ar = false;
  opt.fde_enable = true;

  gf::GraphArPosterior post;  // lin_valid == false: cannot reproduce weights
  const gf::DdFactorLayout empty_layout;
  const std::vector<gf::DdAmbiguityPair> no_pairs;
  const gf::ArResult r =
      gf::resolveAmbiguitiesPosterior(sc.ep, no_pairs, post, empty_layout, cfg,
                                      opt);
  // With no ambiguities there is nothing to fix, which is the trivially safe
  // outcome; the point of the assertion is that it is never a silent FIX.
  EXPECT_FALSE(r.fixed);
}

TEST(DdFactorLayout, RecordsTheRowsTheGateLetThroughNotTheWholeEpoch) {
  // pr_innov_gate_m drops DD rows at factor-build time. The layout must contain
  // what the FACTOR contains, or the post-fit weights are formed over a row set
  // the graph never saw.
  SyntheticDdScenario sc = makeSyntheticDdScenario(8);
  gf::AdapterConfig cfg;
  cfg.robust = false;
  cfg.pr_innov_gate_m = 0.5;  // tight enough that a displaced gate drops rows

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  std::uint64_t next_id = 0;
  gf::PersistentAmbiguities amb(next_id);
  gf::DdFactorLayout layout;
  int n_gated = 0;
  // A gate position well away from the truth, so the innovation gate fires.
  const Eigen::Vector3d gate_pos = sc.rover + Eigen::Vector3d(40.0, 0.0, 0.0);
  gf::addGroupedDdFactors(sc.ep, gtsam::Symbol('x', 0), cfg, graph, values, amb,
                          &gate_pos, &n_gated, &layout);

  ASSERT_TRUE(layout.valid);
  ASSERT_GT(n_gated, 0) << "the scenario must actually trip the gate";

  size_t code_rows = 0;
  for (const auto& g : layout.groups)
    if (!g.carrier) code_rows += g.dd_index.size();
  size_t epoch_pr_rows = 0;
  for (const auto& d : sc.ep.dd)
    if (d.has_pr) ++epoch_pr_rows;

  EXPECT_LT(code_rows, epoch_pr_rows)
      << "the layout must NOT contain the rows the gate removed";
  EXPECT_EQ(code_rows + static_cast<size_t>(n_gated), epoch_pr_rows);
  // ... and each recorded group's covariance is sized to its own rows.
  for (const auto& g : layout.groups)
    EXPECT_EQ(g.nominal_cov.rows(), static_cast<int>(g.dd_index.size()));
}

// --- Dead-reckoning velocity covariance frames -------------------------------

TEST(GapVelocityCovariance, BothPathsReturnEnuSoThePublisherRotatesItOnce) {
  // The bug this pins: an ECEF covariance handed to publishSolution, which
  // applies R_e_n itself, was rotated twice. Both paths must return ENU.
  const gtsam::Rot3 nRb_pred = gtsam::Rot3::RzRyRx(0.03, -0.02, 1.1);
  const gtsam::Rot3 nRb_prev = gtsam::Rot3::RzRyRx(0.02, -0.01, 1.0);
  Eigen::Matrix3d Pvv_body;
  Pvv_body << 0.04, 0.01, 0.0, 0.01, 0.09, 0.02, 0.0, 0.02, 0.16;

  const Eigen::Matrix3d full = gf::gapVelocityCovarianceEnu(Pvv_body, nRb_pred);
  EXPECT_TRUE(full.isApprox(
      nRb_pred.matrix() * Pvv_body * nRb_pred.matrix().transpose(), 1e-12));

  // A body-frame covariance rotated ONCE changes; rotating twice would not
  // match, which is what makes this a real check of the frame.
  EXPECT_FALSE(full.isApprox(Pvv_body, 1e-6));

  const Eigen::Matrix3d last_enu = Eigen::Matrix3d::Identity() * 0.02;
  const double floor = 1.0;
  const Eigen::Matrix3d fb = gf::gapVelocityCovarianceEnuFallback(
      last_enu, Pvv_body, nRb_prev, floor);
  EXPECT_TRUE(fb.isApprox(
      last_enu + nRb_prev.matrix() * Pvv_body * nRb_prev.matrix().transpose() +
          Eigen::Matrix3d::Identity() * (floor * floor),
      1e-12));

  // Both symmetric, both PSD, and the fallback is the more conservative of the
  // two - an outage must not report tighter uncertainty than the propagated one.
  for (const auto& M : {full, fb}) {
    EXPECT_TRUE(M.isApprox(M.transpose(), 1e-12));
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(M);
    EXPECT_GT(es.eigenvalues().minCoeff(), 0.0);
  }
  EXPECT_GT(fb.diagonal().sum(), full.diagonal().sum());
}

// --- Robust weight edge cases the fourth review round found ------------------

TEST(RobustWeights, BlockMatchesGtsamExactlyWithNoFloor) {
  // The reproduction path must equal gtsam::mEstimator::Huber::weight for every
  // input, including ones where a 1e-6 floor would have diverged. A different
  // formula - however rarely it differs - defeats the purpose of reproducing.
  const double k = 1.345;
  for (const double nz : {0.1, 1.0, 1.345, 2.0, 1.0e5, 1.0e7, 1.0e9}) {
    const double gtsam_w = (nz <= k) ? 1.0 : k / nz;
    const double ours = (nz > k) ? k / nz : 1.0;  // no floor
    EXPECT_DOUBLE_EQ(ours, gtsam_w) << "at ||z|| = " << nz;
  }
}

TEST(RobustWeights, AnUndecomposableCovarianceIsReportedNotRepaired) {
  // The previous version floored the eigenvalues, silently turning a singular or
  // indefinite covariance into an invertible one.
  Eigen::MatrixXd S;
  Eigen::MatrixXd singular = Eigen::MatrixXd::Ones(3, 3);  // rank 1
  EXPECT_FALSE(gf::inverseSymmetricSqrt(singular, S));

  Eigen::MatrixXd indefinite = Eigen::MatrixXd::Identity(3, 3);
  indefinite(2, 2) = -1.0;
  EXPECT_FALSE(gf::inverseSymmetricSqrt(indefinite, S));

  Eigen::MatrixXd nan_mat = Eigen::MatrixXd::Identity(3, 3);
  nan_mat(0, 0) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(gf::inverseSymmetricSqrt(nan_mat, S));

  // A well-formed one still works, and is a genuine inverse square root.
  const Eigen::MatrixXd R = ddGroupCovariance(4, 0.30, 0.20);
  ASSERT_TRUE(gf::inverseSymmetricSqrt(R, S));
  EXPECT_TRUE((S * R * S.transpose())
                  .isApprox(Eigen::MatrixXd::Identity(4, 4), 1e-9));
  EXPECT_TRUE(S.isApprox(S.transpose(), 1e-12)) << "must be the SYMMETRIC root";

  gf::AdapterConfig cfg;
  cfg.robust = true;
  EXPECT_EQ(gf::groupedDdNoise(singular, cfg), nullptr);
}

// gtsam::noiseModel::Gaussian::Covariance is NOT a safety net, and this pins
// that so the assumption cannot be written into the code a second time. It
// validates only that the matrix is square; a singular one goes through
// inverse() and Information() without any exception, producing a model whose
// whitening is NaN. groupedDdNoise therefore refuses such an R outright rather
// than falling back to it.
TEST(RobustWeights, GaussianCovarianceSilentlyAcceptsASingularMatrix) {
  const Eigen::MatrixXd singular = Eigen::MatrixXd::Ones(3, 3);  // rank 1
  auto model = gtsam::noiseModel::Gaussian::Covariance(singular, false);
  ASSERT_NE(model, nullptr) << "it does not throw - that is the point";
  const gtsam::Vector v = (gtsam::Vector(3) << 1.0, 2.0, 3.0).finished();
  EXPECT_FALSE(model->whiten(v).allFinite())
      << "and the model it returns whitens to NaN/Inf";
}

TEST(GroupedDdNoise, RefusesAnUnusableCovarianceRobustOrNot) {
  const Eigen::MatrixXd good = ddGroupCovariance(4, 0.30, 0.20);
  const Eigen::MatrixXd singular = Eigen::MatrixXd::Ones(4, 4);
  gf::AdapterConfig cfg;
  for (const bool robust : {false, true}) {
    cfg.robust = robust;
    EXPECT_NE(gf::groupedDdNoise(good, cfg), nullptr);
    EXPECT_EQ(gf::groupedDdNoise(singular, cfg), nullptr)
        << "robust=" << robust;
  }
}

TEST(GroupedDdNoise, ABadCovarianceDropsTheGroupAndInvalidatesTheLayout) {
  // Force an unusable R by driving a modelled variance to a non-finite value;
  // the factor must not be built, and the layout must be marked invalid so the
  // post-fit validation refuses a verdict for the epoch.
  SyntheticDdScenario sc = makeSyntheticDdScenario(5);
  for (auto& d : sc.ep.dd) {
    d.has_cp = false;
    d.model_var_ref_sd = std::numeric_limits<double>::quiet_NaN();
  }
  gf::AdapterConfig cfg;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  std::uint64_t next_id = 0;
  gf::PersistentAmbiguities amb(next_id);
  gf::DdFactorLayout layout;
  int n_gated = 0, n_bad_cov = 0;
  gf::addGroupedDdFactors(sc.ep, gtsam::Symbol('x', 0), cfg, graph, values, amb,
                          nullptr, &n_gated, &layout, &n_bad_cov);
  EXPECT_GT(n_bad_cov, 0) << "the bad covariance must be reported";
  EXPECT_FALSE(layout.valid) << "and the epoch must fail closed downstream";
  for (const auto& fac : graph) {
    ASSERT_NE(fac, nullptr);
    EXPECT_EQ(std::dynamic_pointer_cast<gf::GroupedDdFactor<gtsam::Point3>>(fac),
              nullptr)
        << "no DD factor may be built around an unusable covariance";
  }
}

// Regression: a DD that pr_innov_gate_m removed at factor-build time is not in
// the layout, but the post-fit validation still walks every ep.dd row. Looking
// its weight up returns null, which the fail-closed policy turns into
// FdeReject - so ONE gated satellite rejected the whole epoch's FIX.
//
// Rows the graph never saw must be EXCLUDED from the validation instead. They
// cannot be repaired into it either: for a row the fit never consumed, the
// residual covariance is the PRE-fit R + H*P+*H', not the post-fit
// R - H*P+*H' this matrix is built from, so mixing the two forms in one matrix
// would be wrong in the other direction.
TEST(FdeVerdict, AGatedRowMustNotRejectTheWholeEpoch) {
  SyntheticDdScenario sc = makeSyntheticDdScenario(8);
  // Code-only, so the layout has no carrier groups and this test isolates the
  // gated CODE row it is about (a carrier group would additionally need float
  // ambiguities, which is a separate lookup).
  for (auto& d : sc.ep.dd) d.has_cp = false;
  gf::AdapterConfig cfg;
  cfg.robust = true;
  // Chosen so the gate removes SOME rows and keeps others: the partial case is
  // the one that matters, because it is the one where a good FIX exists and the
  // lookup gap throws it away.
  cfg.pr_innov_gate_m = 8.0;

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  std::uint64_t next_id = 0;
  gf::PersistentAmbiguities amb(next_id);
  gf::DdFactorLayout layout;
  int n_gated = 0;
  const Eigen::Vector3d gate_pos = sc.rover + Eigen::Vector3d(30.0, 12.0, 0.0);
  gf::addGroupedDdFactors(sc.ep, gtsam::Symbol('x', 0), cfg, graph, values, amb,
                          &gate_pos, &n_gated, &layout);
  ASSERT_GT(n_gated, 0) << "the scenario must actually gate a row";
  ASSERT_TRUE(layout.valid);

  // Every code row the layout DOES carry must be resolvable; the gated ones
  // must simply be absent rather than poisoning the lookup.
  gf::GraphArPosterior post;
  post.lin_ant = sc.rover;
  post.lin_valid = true;
  const gf::FrontEndRobustWeights few =
      gf::frontEndRobustWeights(sc.ep, {}, post, layout, cfg);
  ASSERT_TRUE(few.valid);

  int missing = 0, present = 0, row = 0;
  for (int di = 0; di < static_cast<int>(sc.ep.dd.size()); ++di) {
    if (!sc.ep.dd[di].has_pr) continue;
    (few.block(di, false, &row) == nullptr) ? ++missing : ++present;
  }
  EXPECT_EQ(missing, n_gated)
      << "gated rows are absent from the layout, by construction";
  EXPECT_GT(present, 0) << "and some rows must survive, or this proves nothing";

  // The behaviour that matters: the validation must SKIP the gated rows and
  // still reach a verdict on the rest, rather than refusing the epoch because a
  // lookup came up empty.
  gf::ArOptions opt;
  opt.min_fix = 4;
  opt.min_lock = 0;
  opt.el_mask_rad = 0.0;
  opt.max_pos_var_m2 = 0.0;
  opt.partial_ar = false;
  opt.fde_enable = true;

  gf::GraphArPosterior p2 = post;
  p2.ant = sc.rover;
  p2.ant_cov = Eigen::Matrix3d::Identity() * 1e-4;
  p2.full_cov = p2.ant_cov;
  const std::vector<gf::DdAmbiguityPair> no_pairs;
  const gf::ArResult r = gf::resolveAmbiguitiesPosterior(
      sc.ep, no_pairs, p2, layout, cfg, opt);
  // No ambiguities here, so there is no FIX to validate - the assertion is that
  // the gated row did not turn into an FdeReject on the way through.
  EXPECT_NE(r.fail, gf::ArFail::FdeReject)
      << "a gated satellite must not reject the epoch";
}

// --- Post-fit residual covariance under robust weights -----------------------
//
// The old expression R_eff - H P+ H' is the residual covariance of an EFFICIENT
// estimator (W = R^-1). A robust kernel applies W = R_eff^-1 while the noise is
// still R, so the estimator is not efficient and two terms no longer cancel:
//
//     Cov(v) = R - S_hat R - R S_hat' + H P+ H',   S_hat = H P+ H' R_eff^-1
//
// These pin the algebra directly, since the code path needs a full AR posterior
// to reach end-to-end.
namespace {

// R_eff for an arbitrary per-row weight vector, used only to feed the sandwich
// formula an R_eff that is NOT proportional to R: R_eff = S^-1 W^-1 S^-T with
// S the inverse symmetric square root.
Eigen::MatrixXd downWeightedCov(const Eigen::MatrixXd& R,
                                const Eigen::VectorXd& w) {
  Eigen::MatrixXd S;
  EXPECT_TRUE(gf::inverseSymmetricSqrt(R, S));
  const Eigen::MatrixXd S_inv = S.inverse();
  return S_inv * w.cwiseInverse().asDiagonal() * S_inv.transpose();
}

Eigen::MatrixXd residualCovSandwich(const Eigen::MatrixXd& R,
                                    const Eigen::MatrixXd& R_eff,
                                    const Eigen::MatrixXd& H,
                                    const Eigen::MatrixXd& P) {
  const Eigen::MatrixXd HPHt = H * P * H.transpose();
  const Eigen::MatrixXd Shat_R = HPHt * R_eff.ldlt().solve(R);
  const Eigen::MatrixXd S = R - Shat_R - Shat_R.transpose() + HPHt;
  return 0.5 * (S + S.transpose());
}

}  // namespace

TEST(PostFitResidualCov, ReducesExactlyToTheOldFormWhenNothingIsDownWeighted) {
  const int n = 6;
  const Eigen::MatrixXd R = ddGroupCovariance(n, 0.30, 0.20);
  Eigen::MatrixXd H = Eigen::MatrixXd::Random(n, 3);
  Eigen::MatrixXd A = Eigen::MatrixXd::Random(3, 3);
  const Eigen::MatrixXd P = A * A.transpose() * 1e-3;

  // R_eff == R is exactly what "no row was down-weighted" produces.
  const Eigen::MatrixXd got = residualCovSandwich(R, R, H, P);
  const Eigen::MatrixXd old = R - H * P * H.transpose();
  EXPECT_TRUE(got.isApprox(0.5 * (old + old.transpose()), 1e-10))
      << "the generalisation must not change the efficient case";
}

TEST(PostFitResidualCov, DiffersOnceAnyRowIsDownWeighted) {
  const int n = 6;
  const Eigen::MatrixXd R = ddGroupCovariance(n, 0.30, 0.20);
  Eigen::MatrixXd H = Eigen::MatrixXd::Random(n, 3);
  Eigen::MatrixXd A = Eigen::MatrixXd::Random(3, 3);
  const Eigen::MatrixXd P = A * A.transpose() * 1e-3;

  Eigen::VectorXd w = Eigen::VectorXd::Ones(n);
  w(0) = 0.25;
  const Eigen::MatrixXd R_eff = downWeightedCov(R, w);

  const Eigen::MatrixXd sandwich = residualCovSandwich(R, R_eff, H, P);
  const Eigen::MatrixXd conflated = R_eff - H * P * H.transpose();
  EXPECT_FALSE(sandwich.isApprox(conflated, 1e-6))
      << "conflating R_eff with the true R must be observably different";

  // Symmetric, and still a sensible residual covariance.
  EXPECT_TRUE(sandwich.isApprox(sandwich.transpose(), 1e-12));
  // The efficient-case identity must NOT be assumed: S_hat R != H P+ H' here.
  const Eigen::MatrixXd HPHt = H * P * H.transpose();
  EXPECT_FALSE((HPHt * R_eff.ldlt().solve(R)).isApprox(HPHt, 1e-6));
}

TEST(PostFitResidualCov, IsInvariantToTheOrderOfTheRows) {
  const int n = 5;
  const Eigen::MatrixXd R = ddGroupCovariance(n, 0.30, 0.20);
  Eigen::MatrixXd H = Eigen::MatrixXd::Random(n, 3);
  Eigen::MatrixXd A = Eigen::MatrixXd::Random(3, 3);
  const Eigen::MatrixXd P = A * A.transpose() * 1e-3;
  Eigen::VectorXd w = Eigen::VectorXd::Ones(n);
  w(1) = 0.3;
  const Eigen::MatrixXd R_eff = downWeightedCov(R, w);

  Eigen::PermutationMatrix<Eigen::Dynamic> Pm(n);
  Pm.indices() << 3, 0, 4, 1, 2;
  const Eigen::MatrixXd got = residualCovSandwich(R, R_eff, H, P);
  const Eigen::MatrixXd permuted = residualCovSandwich(
      Pm * R * Pm.transpose(), Pm * R_eff * Pm.transpose(), Pm * H, P);
  EXPECT_TRUE(permuted.isApprox(Pm * got * Pm.transpose(), 1e-9));
}

// --- Gap publisher / epoch pipeline must cover DISJOINT epochs ---------------
//
// The defect this pins: a `base_outage` branch disabled the slot-ownership
// guard on the premise that a DD-less rover epoch "does not own the slot
// because the graph stops advancing". The graph does NOT stop - processEpoch
// bridges a DD-less epoch with the inertial chain and publishes - so the
// publisher emitted a second, worse solution for the same GPST, and which one
// a consumer saw depended on delivery order. Measured on PPC nagoya_run1:
// 8870 solutions for 7651 truth epochs, 1228 of them unmatched.

TEST(GapSlotOwnership, AReceivedRoverEpochAlwaysOwnsItsSlot) {
  const double rover = 1000.0;   // newest RECEIVED rover epoch
  const double pub = 999.0;      // newest published
  EXPECT_FALSE(gf::gapSlotIsFree(999.8, rover, pub)) << "before the rover front";
  EXPECT_FALSE(gf::gapSlotIsFree(1000.0, rover, pub)) << "at the rover front";
  EXPECT_TRUE(gf::gapSlotIsFree(1000.2, rover, pub)) << "beyond it: free";
}

TEST(GapSlotOwnership, NeverReEmitsAnAlreadyPublishedEpoch) {
  EXPECT_FALSE(gf::gapSlotIsFree(1200.0, -1.0, 1200.0));
  EXPECT_FALSE(gf::gapSlotIsFree(1200.0, -1.0, 1250.0));
  EXPECT_TRUE(gf::gapSlotIsFree(1250.2, -1.0, 1250.0));
}

TEST(GapSlotOwnership, ColdStartHasNoOwners) {
  EXPECT_TRUE(gf::gapSlotIsFree(1000.0, -1.0, -1.0));
}

TEST(GapSlotOwnership, DataDomainBridgingIgnoresTheReceptionFront) {
  // A hole in the EMITTED epoch stream proves no observation exists for the
  // slots inside it, so the "an epoch may still be in the pipeline" clause must
  // not apply - while the node is behind, the reception front sits far past the
  // hole and would veto every slot in it. Passing -1.0 disables just that
  // clause; never re-publishing an owned slot still holds.
  const double reception_front = 1000.0;  // far ahead: the node is behind
  const double slot = 900.0;              // inside the hole
  EXPECT_FALSE(gf::gapSlotIsFree(slot, reception_front, 800.0))
      << "the live path must defer to a possibly-queued epoch";
  EXPECT_TRUE(gf::gapSlotIsFree(slot, -1.0, 800.0))
      << "the data-domain path has direct evidence and may claim it";
  EXPECT_FALSE(gf::gapSlotIsFree(slot, -1.0, 900.0))
      << "an already-published slot is still owned";
}

TEST(GapSlotOwnership, NonFiniteSlotFailsClosed) {
  EXPECT_FALSE(gf::gapSlotIsFree(std::numeric_limits<double>::quiet_NaN(),
                                 -1.0, -1.0));
}

// --- Rover epoch grid: interval pinning and the outage bound ----------------
//
// The defect these pin: the publisher decided "no observation exists for this
// slot" from WALL-CLOCK silence, which cannot be separated from "not delivered
// yet". Measured on PPC nagoya_run1, that claimed the slot of a real epoch
// still in flight (tow 550597.200, immediately before the bag's only genuine
// 10 s hole) and published a second solution for a GPST processEpoch then
// published too - making which one the evaluator scored depend on delivery
// order. The verdict is now taken in the data domain instead.

TEST(EpochInterval, MedianIsImmuneToGapsInTheSpacings) {
  std::vector<double> nominal(20, 0.2);
  EXPECT_DOUBLE_EQ(gf::medianEpochInterval(nominal), 0.2);
  // One 10 s hole (the nagoya_run1 case) must not move the estimate.
  std::vector<double> with_gap = nominal;
  with_gap[7] = 10.0;
  EXPECT_DOUBLE_EQ(gf::medianEpochInterval(with_gap), 0.2);
  with_gap[3] = 2.0;
  with_gap[11] = 4.0;
  EXPECT_DOUBLE_EQ(gf::medianEpochInterval(with_gap), 0.2);
}

TEST(EpochInterval, UndeterminedUntilThereIsEnoughEvidence) {
  EXPECT_DOUBLE_EQ(gf::medianEpochInterval({}), 0.0);
  EXPECT_DOUBLE_EQ(gf::medianEpochInterval(std::vector<double>(9, 0.2)), 0.0);
  EXPECT_DOUBLE_EQ(gf::medianEpochInterval(std::vector<double>(10, 0.2)), 0.2);
  // Non-finite and non-positive spacings are DISCARDED before the count test,
  // so they cannot pad the sample size either.
  std::vector<double> dirty(13, 0.2);
  dirty[0] = std::numeric_limits<double>::quiet_NaN();
  dirty[1] = -1.0;
  dirty[2] = 0.0;
  EXPECT_DOUBLE_EQ(gf::medianEpochInterval(dirty), 0.2) << "10 valid samples";
  dirty.pop_back();
  EXPECT_DOUBLE_EQ(gf::medianEpochInterval(dirty), 0.0) << "only 9 valid";
}

// Arguments are (imu_now, imu_watermark_when_last_rover_arrived, interval).
TEST(GapOutageBound, NormalDeliveryNeverClaimsASlot) {
  EXPECT_FALSE(gf::gapSlotIsUnobserved(100.2, 100.0, 0.2)) << "one interval";
  EXPECT_FALSE(gf::gapSlotIsUnobserved(100.6, 100.0, 0.2)) << "exactly 3";
  EXPECT_FALSE(gf::gapSlotIsUnobserved(100.59, 100.0, 0.2));
}

TEST(GapOutageBound, FiresAfterThreeSilentEpochs) {
  EXPECT_TRUE(gf::gapSlotIsUnobserved(100.61, 100.0, 0.2));
  EXPECT_TRUE(gf::gapSlotIsUnobserved(110.0, 100.0, 0.2)) << "a 10 s hole";
}

TEST(GapOutageBound, ABusyNodeIsNotAnOutage) {
  // The defect this replaced: the bound was the running maximum of the very
  // quantity being tested, so it could only be exceeded by setting a new
  // record and never fired. Here the anchor is refreshed by arrival, not by
  // progress, so a node 68 s behind on a stream that IS arriving stays silent.
  EXPECT_FALSE(gf::gapSlotIsUnobserved(1068.0, 1067.9, 0.2)) << "behind, not out";
  EXPECT_TRUE(gf::gapSlotIsUnobserved(1068.0, 1057.9, 0.2)) << "genuinely silent";
}

TEST(GapOutageBound, FailsClosedOnUnusableInputs) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(gf::gapSlotIsUnobserved(nan, 100.0, 0.2));
  EXPECT_FALSE(gf::gapSlotIsUnobserved(110.0, nan, 0.2));
  EXPECT_FALSE(gf::gapSlotIsUnobserved(110.0, -inf, 0.2)) << "no rover yet";
  EXPECT_FALSE(gf::gapSlotIsUnobserved(110.0, 100.0, 0.0)) << "interval not pinned";
  EXPECT_FALSE(gf::gapSlotIsUnobserved(110.0, 100.0, -0.2));
}

// --- One solution per slot, whoever gets there first -------------------------
//
// The output is a function of the SLOT, not of the event that filled it. The
// same predicate that stops the gap publisher from taking an owned slot also
// stops a late observation from re-publishing one the consumer already has -
// the message contract offers no way to retract a solution.
//
// The defect this pins: both paths published the same GPST, so which solution
// the evaluator scored depended on delivery order. Measured on PPC nagoya_run1,
// that alone moved the fix rate across a 14.5-point band between runs of the
// same build.

TEST(SlotOwnership, ALateEpochDoesNotRePublishAnOwnedSlot) {
  const double published = 1000.0;
  // The coaster reached 1000.0 first; an observation for it arriving later must
  // not be published again (it is still fed to the graph by the caller).
  EXPECT_FALSE(gf::gapSlotIsFree(1000.0, -1.0, published));
  EXPECT_FALSE(gf::gapSlotIsFree(999.6, -1.0, published)) << "older still";
  // The next slot is untouched and remains publishable.
  EXPECT_TRUE(gf::gapSlotIsFree(1000.2, -1.0, published));
}

TEST(SlotOwnership, TheSamePredicateGovernsBothPublishers) {
  // Publisher side (last_pub_ctow only) and gap side (also the rover front)
  // must agree on who owns a slot, or the invariant has two definitions.
  const double pub = 500.0, rover = 500.4;
  EXPECT_FALSE(gf::gapSlotIsFree(500.0, rover, pub)) << "already published";
  EXPECT_FALSE(gf::gapSlotIsFree(500.2, rover, pub)) << "rover front owns it";
  EXPECT_FALSE(gf::gapSlotIsFree(500.4, rover, pub)) << "at the rover front";
  EXPECT_TRUE(gf::gapSlotIsFree(500.6, rover, pub)) << "beyond both: free";
}
