// SPDX-License-Identifier: MIT
//
// Unit tests for the publish-side contracts of
// examples/tightly_coupled_fgo/gnss_imu_fgo.cpp, factored into the pure helpers
// in imu_factors.hpp so they can be exercised without a running node:
//
//   * antennaVelocityNav  - GnssSolution names the ANTENNA phase center, while
//     the graph estimates the BODY origin. A rotating platform with a non-zero
//     lever arm makes those two different velocities.
//   * EpochSlotArbiter    - one epoch produces one output SLOT, and both the
//     solution and the odometry publisher must agree about whether it did.
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include <Eigen/Cholesky>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

#include "imu_factors.hpp"

namespace gf = gnss_fgo;

namespace {

constexpr double kTol = 1e-9;

// A generic, well-conditioned 15x15 joint posterior with non-zero cross terms,
// so a propagation that silently drops them cannot match a full one.
Eigen::MatrixXd makeJoint(double scale = 1.0) {
  Eigen::MatrixXd A(15, 15);
  for (int i = 0; i < 15; ++i)
    for (int j = 0; j < 15; ++j)
      A(i, j) = std::sin(0.7 * i + 1.3 * j + 0.2) * 0.05;
  Eigen::MatrixXd P = A * A.transpose();
  P += Eigen::MatrixXd::Identity(15, 15) * 0.01;
  return P * scale;
}

// Monte-Carlo the covariance of v_ant through the exact nonlinear map, for the
// finite-difference style check the Jacobian propagation has to reproduce.
Eigen::Matrix3d sampledCov(const gtsam::Rot3& nRb, const Eigen::Vector3d& v,
                           const Eigen::Vector3d& omega,
                           const gtsam::Point3& lever,
                           const Eigen::MatrixXd& P15, int n = 200000) {
  // Cholesky of the 9 dimensions that matter: rot(0:2), vel(6:8), bg(12:14).
  Eigen::Matrix<double, 9, 9> P9;
  const int idx[9] = {0, 1, 2, 6, 7, 8, 12, 13, 14};
  for (int i = 0; i < 9; ++i)
    for (int j = 0; j < 9; ++j) P9(i, j) = P15(idx[i], idx[j]);
  const Eigen::LLT<Eigen::Matrix<double, 9, 9>> llt(P9);
  const Eigen::Matrix<double, 9, 9> L = llt.matrixL();

  std::mt19937 rng(12345);
  std::normal_distribution<double> g(0.0, 1.0);
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  Eigen::Matrix3d m2 = Eigen::Matrix3d::Zero();
  for (int k = 0; k < n; ++k) {
    Eigen::Matrix<double, 9, 1> z;
    for (int i = 0; i < 9; ++i) z(i) = g(rng);
    const Eigen::Matrix<double, 9, 1> d = L * z;
    const gtsam::Rot3 R = nRb * gtsam::Rot3::Expmap(d.head<3>());
    const Eigen::Vector3d vv = v + d.segment<3>(3);
    const Eigen::Vector3d w = omega - d.tail<3>();  // omega = meas - b_g
    const Eigen::Vector3d s =
        vv + R.matrix() * w.cross(Eigen::Vector3d(lever));
    mean += s;
    m2 += s * s.transpose();
  }
  mean /= n;
  m2 /= n;
  return m2 - mean * mean.transpose();
}

}  // namespace

// --- antennaVelocityNav: the mean -------------------------------------------

TEST(AntennaVelocity, ZeroLeverArmLeavesTheBodyVelocityUntouched) {
  const gtsam::Rot3 R = gtsam::Rot3::RzRyRx(0.3, -0.2, 1.1);
  const Eigen::Vector3d v(3.0, -1.0, 0.2);
  const Eigen::Vector3d w(0.05, -0.02, 0.4);
  gf::AntennaVelocityCov cov;
  cov.vel_nav = Eigen::Matrix3d::Identity() * 0.04;
  const auto out =
      gf::antennaVelocityNav(R, v, w, gtsam::Point3(0, 0, 0), cov);
  EXPECT_NEAR((out.v_nav - v).norm(), 0.0, kTol);
  // With no lever arm the extra terms all vanish and the velocity covariance
  // passes through unchanged - including the gyro noise term.
  cov.sigma_gyr = 0.01;
  const auto out2 =
      gf::antennaVelocityNav(R, v, w, gtsam::Point3(0, 0, 0), cov);
  EXPECT_NEAR((out2.cov_nav - cov.vel_nav).norm(), 0.0, kTol);
}

// Acceptance criterion 3: rotating in place with a non-zero lever arm, the body
// translational velocity is zero but the antenna is still moving.
TEST(AntennaVelocity, PureYawWithLeverArmGivesTangentialAntennaVelocity) {
  const double yaw_rate = 0.5;                    // rad/s
  const gtsam::Point3 lever(2.0, 0.0, 0.0);       // 2 m forward of the IMU
  const Eigen::Vector3d w(0.0, 0.0, yaw_rate);
  const Eigen::Vector3d v_body = Eigen::Vector3d::Zero();
  gf::AntennaVelocityCov cov;

  // Body aligned with nav: the antenna sweeps along +y (nav North) at w * |l|.
  const auto out = gf::antennaVelocityNav(gtsam::Rot3::Identity(), v_body, w,
                                          lever, cov);
  EXPECT_NEAR(out.v_nav.x(), 0.0, kTol);
  EXPECT_NEAR(out.v_nav.y(), yaw_rate * lever.x(), kTol);
  EXPECT_NEAR(out.v_nav.z(), 0.0, kTol);
  EXPECT_NEAR(out.v_nav.norm(), yaw_rate * lever.norm(), kTol);

  // The speed is frame-independent: yaw the body 90 deg and it is unchanged,
  // only redirected.
  const auto turned = gf::antennaVelocityNav(
      gtsam::Rot3::Yaw(M_PI / 2), v_body, w, lever, cov);
  EXPECT_NEAR(turned.v_nav.norm(), yaw_rate * lever.norm(), kTol);
  EXPECT_NEAR(turned.v_nav.x(), -yaw_rate * lever.x(), 1e-9);
}

// The published antenna velocity must be the exact inverse of the lever-arm
// term the Doppler factor removes to observe V(k) - same convention, opposite
// sign - or the node would publish a velocity its own estimator disagrees with.
TEST(AntennaVelocity, InvertsTheDopplerLeverArmRemoval) {
  const gtsam::Rot3 R = gtsam::Rot3::RzRyRx(0.1, 0.25, -0.8);
  const gtsam::Point3 lever(1.2, -0.4, 0.9);
  const Eigen::Vector3d w(0.11, -0.07, 0.35);
  const Eigen::Vector3d v_antenna_truth(4.0, 1.5, -0.3);

  // What the Doppler path does: antenna -> body.
  const Eigen::Vector3d v_body =
      v_antenna_truth - R.matrix() * w.cross(Eigen::Vector3d(lever));
  // What the publisher does: body -> antenna.
  const auto out =
      gf::antennaVelocityNav(R, v_body, w, lever, gf::AntennaVelocityCov{});
  EXPECT_NEAR((out.v_nav - v_antenna_truth).norm(), 0.0, kTol);
}

// --- antennaVelocityNav: the covariance --------------------------------------

TEST(AntennaVelocityCovariance, JointPropagationMatchesMonteCarlo) {
  const gtsam::Rot3 R = gtsam::Rot3::RzRyRx(0.2, -0.15, 0.9);
  const Eigen::Vector3d v(2.0, 0.5, -0.1);
  const Eigen::Vector3d w(0.08, -0.05, 0.30);
  const gtsam::Point3 lever(1.5, 0.3, -0.6);

  gf::AntennaVelocityCov cov;
  cov.joint = makeJoint(0.02);
  cov.joint_ok = true;
  cov.sigma_gyr = 0.0;  // the sampler has no measurement-noise channel

  const auto out = gf::antennaVelocityNav(R, v, w, lever, cov);
  const Eigen::Matrix3d mc = sampledCov(R, v, w, lever, cov.joint);
  // Monte-Carlo at 2e5 draws; the linearization itself is exact to second order
  // here, so a few percent is the sampling error, not a modelling gap.
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      EXPECT_NEAR(out.cov_nav(i, j), mc(i, j), 0.03 * mc.diagonal().maxCoeff())
          << "block (" << i << "," << j << ")";
}

TEST(AntennaVelocityCovariance, JacobianBlocksMatchFiniteDifference) {
  const gtsam::Rot3 R = gtsam::Rot3::RzRyRx(-0.4, 0.2, 2.0);
  const Eigen::Vector3d v(1.0, -2.0, 0.4);
  const Eigen::Vector3d w(0.2, 0.1, -0.15);
  const gtsam::Point3 lever(0.8, 1.1, -0.25);
  const double h = 1e-6;

  const auto f = [&](const Eigen::Vector3d& dtheta, const Eigen::Vector3d& dv,
                     const Eigen::Vector3d& dbg) {
    const gtsam::Rot3 Rp = R * gtsam::Rot3::Expmap(dtheta);
    return Eigen::Vector3d((v + dv) +
                           Rp.matrix() *
                               (w - dbg).cross(Eigen::Vector3d(lever)));
  };
  const Eigen::Vector3d f0 = f(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                               Eigen::Vector3d::Zero());

  // Analytic blocks, as antennaVelocityNav builds them.
  const Eigen::Vector3d wxl = w.cross(Eigen::Vector3d(lever));
  const Eigen::Matrix3d J_theta = -R.matrix() * gf::skew3(wxl);
  const Eigen::Matrix3d J_bg = R.matrix() * gf::skew3(Eigen::Vector3d(lever));

  for (int k = 0; k < 3; ++k) {
    Eigen::Vector3d e = Eigen::Vector3d::Zero();
    e(k) = h;
    const Eigen::Vector3d d_theta =
        (f(e, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()) - f0) / h;
    const Eigen::Vector3d d_v =
        (f(Eigen::Vector3d::Zero(), e, Eigen::Vector3d::Zero()) - f0) / h;
    const Eigen::Vector3d d_bg =
        (f(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), e) - f0) / h;
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(J_theta(i, k), d_theta(i), 1e-5) << "d/dtheta " << i << k;
      EXPECT_NEAR(Eigen::Matrix3d::Identity()(i, k), d_v(i), 1e-5);
      EXPECT_NEAR(J_bg(i, k), d_bg(i), 1e-5) << "d/dbg " << i << k;
    }
  }
}

// The gyro white noise has no graph state, so it is in no posterior block and
// must be added on top of BOTH the joint and the fallback paths.
TEST(AntennaVelocityCovariance, GyroWhiteNoiseAlwaysContributes) {
  const gtsam::Rot3 R = gtsam::Rot3::Yaw(0.6);
  const Eigen::Vector3d v(1.0, 0.0, 0.0);
  const Eigen::Vector3d w(0.0, 0.0, 0.2);
  const gtsam::Point3 lever(1.0, 0.0, 0.0);

  gf::AntennaVelocityCov quiet;
  quiet.joint = makeJoint(0.01);
  quiet.joint_ok = true;
  gf::AntennaVelocityCov noisy = quiet;
  noisy.sigma_gyr = 0.05;

  const auto a = gf::antennaVelocityNav(R, v, w, lever, quiet);
  const auto b = gf::antennaVelocityNav(R, v, w, lever, noisy);
  const Eigen::Matrix3d d = b.cov_nav - a.cov_nav;
  // Positive semi-definite increment, and not the zero matrix.
  EXPECT_GT(d.trace(), 0.0);
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(d);
  EXPECT_GE(es.eigenvalues().minCoeff(), -1e-12);

  // Same on the block-diagonal path.
  gf::AntennaVelocityCov fq;
  fq.vel_nav = Eigen::Matrix3d::Identity() * 0.02;
  gf::AntennaVelocityCov fn = fq;
  fn.sigma_gyr = 0.05;
  EXPECT_GT((gf::antennaVelocityNav(R, v, w, lever, fn).cov_nav -
             gf::antennaVelocityNav(R, v, w, lever, fq).cov_nav)
                .trace(),
            0.0);
}

TEST(AntennaVelocityCovariance, FallsBackToBlockDiagonalWithoutTheJoint) {
  const gtsam::Rot3 R = gtsam::Rot3::RzRyRx(0.1, 0.1, 0.1);
  const Eigen::Vector3d v(1.0, 2.0, 3.0);
  const Eigen::Vector3d w(0.1, 0.2, 0.3);
  const gtsam::Point3 lever(0.5, -0.5, 0.25);

  const Eigen::MatrixXd P = makeJoint(0.02);
  gf::AntennaVelocityCov fb;
  fb.vel_nav = P.block<3, 3>(6, 6);
  fb.rot = P.block<3, 3>(0, 0);
  fb.bias_gyro = P.block<3, 3>(12, 12);
  const auto out = gf::antennaVelocityNav(R, v, w, lever, fb);

  const Eigen::Vector3d wxl = w.cross(Eigen::Vector3d(lever));
  const Eigen::Matrix3d Jt = -R.matrix() * gf::skew3(wxl);
  const Eigen::Matrix3d M = R.matrix() * gf::skew3(Eigen::Vector3d(lever));
  const Eigen::Matrix3d expect = fb.vel_nav + Jt * fb.rot * Jt.transpose() +
                                 M * fb.bias_gyro * M.transpose();
  EXPECT_NEAR((out.cov_nav - expect).norm(), 0.0, 1e-12);

  // A malformed joint must not be trusted: it takes the same fallback.
  gf::AntennaVelocityCov bad = fb;
  bad.joint = Eigen::MatrixXd::Identity(9, 9);
  bad.joint_ok = true;
  EXPECT_NEAR(
      (gf::antennaVelocityNav(R, v, w, lever, bad).cov_nav - expect).norm(),
      0.0, 1e-12);
}

TEST(AntennaVelocityCovariance, IsSymmetricAndPositiveSemiDefinite) {
  const gtsam::Rot3 R = gtsam::Rot3::RzRyRx(1.0, -0.5, 0.25);
  const Eigen::Vector3d v(0.0, 0.0, 0.0);
  const Eigen::Vector3d w(0.4, -0.3, 0.9);
  const gtsam::Point3 lever(1.0, 2.0, 3.0);
  gf::AntennaVelocityCov cov;
  cov.joint = makeJoint(0.05);
  cov.joint_ok = true;
  cov.sigma_gyr = 0.02;
  const auto out = gf::antennaVelocityNav(R, v, w, lever, cov);
  EXPECT_NEAR((out.cov_nav - out.cov_nav.transpose()).norm(), 0.0, 1e-15);
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(out.cov_nav);
  EXPECT_GE(es.eigenvalues().minCoeff(), -1e-12);
}

// --- EpochSlotArbiter --------------------------------------------------------

// Acceptance criterion 5: the gap publisher and a late real epoch cannot both
// produce output for one slot.
TEST(EpochSlotArbiter, OnlyTheFirstClaimOnASlotSucceeds) {
  gf::EpochSlotArbiter slots;
  EXPECT_TRUE(slots.claim(1000.0));
  EXPECT_DOUBLE_EQ(slots.lastPublished(), 1000.0);
  // The same slot, reached by the other publisher.
  EXPECT_FALSE(slots.claim(1000.0));
  // And anything at or behind the high-water mark.
  EXPECT_FALSE(slots.claim(999.8));
  EXPECT_FALSE(slots.claim(1000.0005));  // inside the 1 ms tolerance
  EXPECT_TRUE(slots.claim(1000.2));
}

// Acceptance criterion 6: a suppressed epoch publishes NOTHING - so the caller
// must be able to see that from the return value, not from a side effect
// buried in one of the two publishers.
TEST(EpochSlotArbiter, ASuppressedEpochIsCountedAndProducesNoOutput) {
  gf::EpochSlotArbiter slots;
  int solutions = 0, odometries = 0;
  const auto publish = [&](double ctow) {
    if (slots.claim(ctow)) {
      ++solutions;
      ++odometries;
    }
  };
  publish(500.0);          // gap prediction fills the slot
  publish(500.0);          // the real epoch arrives late for the same slot
  publish(500.2);          // the next slot is free
  EXPECT_EQ(solutions, 2);
  EXPECT_EQ(odometries, 2);
  EXPECT_EQ(solutions, odometries) << "the two topics must never disagree";
  EXPECT_EQ(slots.suppressed(), 1u);
}

// Yielding to a rover epoch still in the pipeline is the gate working, not the
// node falling behind: it must not inflate the "arrived too late" count.
TEST(EpochSlotArbiter, YieldingToAPendingRoverEpochIsNotSuppression) {
  gf::EpochSlotArbiter slots;
  EXPECT_FALSE(slots.claim(800.0, /*latest_rover_ctow=*/900.0));
  EXPECT_EQ(slots.suppressed(), 0u);
  EXPECT_DOUBLE_EQ(slots.lastPublished(), -1.0);
  // Past the arrival front it is free again.
  EXPECT_TRUE(slots.claim(900.2, /*latest_rover_ctow=*/900.0));
  EXPECT_EQ(slots.suppressed(), 0u);
}

TEST(EpochSlotArbiter, MatchesGapSlotIsFreeOnEveryDecision) {
  gf::EpochSlotArbiter slots;
  const double rover = 1000.0;
  for (double slot : {999.8, 1000.0, 1000.2, 1000.4, 1000.4, 1000.1}) {
    const double before = slots.lastPublished();
    const bool expected = gf::gapSlotIsFree(slot, rover, before);
    EXPECT_EQ(slots.claim(slot, rover), expected) << "slot " << slot;
  }
}

TEST(EpochSlotArbiter, RejectsNonFiniteSlots) {
  gf::EpochSlotArbiter slots;
  EXPECT_FALSE(slots.claim(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(slots.claim(std::numeric_limits<double>::infinity()));
  EXPECT_DOUBLE_EQ(slots.lastPublished(), -1.0);
  EXPECT_TRUE(slots.claim(1.0));
}
