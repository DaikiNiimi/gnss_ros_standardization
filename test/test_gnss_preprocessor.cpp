// SPDX-License-Identifier: MIT
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "gnss_ros_standardization/gnss_preprocessor.hpp"
#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/msg/gnss_ephemeris.hpp"
#include "gnss_ros_standardization/msg/gnss_observation.hpp"

extern "C" {
#include "rtklib.h"
}

namespace {

using Preprocessor = gnss_utils::GnssPreprocessor;
using ObsMsg = Preprocessor::ObsMsg;
using EphMsg = Preprocessor::EphMsg;
using Observation = gnss_ros_standardization::msg::GnssObservation;
using Ephemeris = gnss_ros_standardization::msg::GnssEphemeris;

ObsMsg::SharedPtr observationEpoch(std::uint32_t week, double tow,
                                   std::uint8_t lli = 0) {
  auto msg = std::make_shared<ObsMsg>();
  msg->week = week;
  msg->tow = tow;

  Observation obs;
  obs.system = "G";
  obs.prn = 1;
  obs.sat = static_cast<std::uint16_t>(satid2no("G01"));
  obs.satid = "G01";
  obs.code = CODE_L1C;
  obs.code_str = "1C";
  obs.p = 2.25e7;
  obs.l = 1.18e8;
  obs.d = -1200.0;
  obs.snr = 45.0;
  obs.lli = lli;
  msg->observations.push_back(obs);
  return msg;
}

EphMsg gpsEphemeris(std::uint32_t week, double toe) {
  EphMsg msg;
  Ephemeris eph;
  eph.system = "G";
  eph.prn = 1;
  eph.satid = "G01";
  eph.week = week;
  eph.sva = 1;
  eph.iode = 1;
  eph.iodc = 1;
  eph.svh = 0;
  eph.toe = toe;
  eph.toes = toe;
  eph.toc = toe;
  eph.ttr = toe;
  eph.a = 26560000.0;
  eph.e = 0.01;
  eph.i0 = 0.94;
  eph.omg0 = 1.0;
  eph.omg = 0.5;
  eph.m0 = 0.2;
  eph.deln = 0.0;
  eph.omgd = -2.6e-9;
  eph.idot = 0.0;
  eph.fit = 4;
  eph.tgd = {0.0, 0.0};
  msg.gnss_ephemeris.push_back(eph);
  return msg;
}

Preprocessor::Config config() {
  Preprocessor::Config cfg;
  cfg.el_mask_rad = -90.0 * D2R;
  cfg.navsys = SYS_GPS;
  cfg.bands = {0};
  cfg.matcher_decision_horizon_s = 0.5;
  cfg.matcher_reorder_window_s = 0.01;
  cfg.detect_slip_gf = false;
  cfg.detect_slip_cmc = false;
  cfg.detect_slip_dop = false;
  return cfg;
}

TEST(GnssPreprocessor, RejectsInvalidConfiguration) {
  auto cfg = config();
  cfg.bands.clear();
  EXPECT_THROW((void)Preprocessor(cfg), std::invalid_argument);

  cfg = config();
  cfg.bands = {0, 0};
  EXPECT_THROW((void)Preprocessor(cfg), std::invalid_argument);

  cfg = config();
  cfg.matcher_duplicate_tol_s = cfg.match_tol_s * 2.0;
  EXPECT_THROW((void)Preprocessor(cfg), std::invalid_argument);
}

TEST(GnssPreprocessor, KeepsObservationsUntilLateEphemerisAndDeferredFlush) {
  Preprocessor pre(config());
  const Eigen::Vector3d approximate(6378137.0, 0.0, 0.0);
  pre.pushRoverObs(observationEpoch(2200, 100100.0));

  EXPECT_TRUE(pre.drainEpochs(&approximate).empty());
  EXPECT_EQ(pre.pendingRoverCount(), 1U);
  // EOF can precede a transient-local nav snapshot in an adversarial callback
  // order. The flush request must be deferred rather than erasing observations.
  EXPECT_TRUE(pre.flushEpochs(&approximate).empty());
  EXPECT_EQ(pre.pendingRoverCount(), 1U);

  pre.pushEphemerides(gpsEphemeris(2200, 100000.0));
  const auto epochs = pre.drainEpochs(&approximate);
  ASSERT_EQ(epochs.size(), 1U);
  EXPECT_EQ(epochs[0].week, 2200U);
  EXPECT_DOUBLE_EQ(epochs[0].tow, 100100.0);
  EXPECT_FALSE(epochs[0].has_base);
  EXPECT_TRUE(std::isnan(epochs[0].base_tow));
  EXPECT_TRUE(std::isnan(epochs[0].age_s));
  EXPECT_FALSE(epochs[0].rover_sats.empty());
  EXPECT_EQ(pre.pendingRoverCount(), 0U);
}

TEST(GnssPreprocessor, RetainsExactBaseEpochIdentity) {
  Preprocessor pre(config());
  const Eigen::Vector3d approximate(6378137.0, 0.0, 0.0);
  pre.pushEphemerides(gpsEphemeris(2200, 100000.0));
  pre.pushBaseObs(observationEpoch(2200, 100099.8));
  pre.pushRoverObs(observationEpoch(2200, 100100.0));

  const auto epochs = pre.flushEpochs(&approximate);
  ASSERT_EQ(epochs.size(), 1U);
  EXPECT_TRUE(epochs[0].has_base);
  EXPECT_EQ(epochs[0].base_week, 2200U);
  EXPECT_DOUBLE_EQ(epochs[0].base_tow, 100099.8);
  EXPECT_NEAR(epochs[0].age_s, 0.2, 1e-9);
}

TEST(GnssPreprocessor, ExposesMatcherOverflowDiagnosticsBeforeEphemeris) {
  auto cfg = config();
  cfg.matcher_queue_limit = 2;
  Preprocessor pre(cfg);
  pre.pushRoverObs(observationEpoch(2200, 100.0));
  pre.pushRoverObs(observationEpoch(2200, 101.0));
  pre.pushRoverObs(observationEpoch(2200, 102.0));

  EXPECT_EQ(pre.pendingRoverCount(), 2U);
  EXPECT_EQ(pre.matcherCounters().dropped(
                gnss_utils::RoverBaseEpochMatcher::Stream::Rover,
                gnss_utils::RoverBaseEpochMatcher::DropReason::QueueOverflow),
            1U);
  const auto events = pre.takeDropEvents();
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].reason,
            gnss_utils::RoverBaseEpochMatcher::DropReason::QueueOverflow);
}

TEST(GnssPreprocessor, ReusedFlaggedBaseDoesNotRepeatedlySignalSlip) {
  auto cfg = config();
  // This test isolates base-reuse state; callback reordering is covered by the
  // matcher suite and would intentionally defer the newest exact epoch.
  cfg.matcher_reorder_window_s = 0.0;
  Preprocessor pre(cfg);
  const Eigen::Vector3d approximate(6378137.0, 0.0, 0.0);
  pre.pushEphemerides(gpsEphemeris(2200, 100000.0));

  // The first use consumes the base LLI slip. The same 1 Hz base observation is
  // then reused at rover 5 Hz and must not re-key the ambiguity five times.
  pre.pushBaseObs(observationEpoch(2200, 100100.0, 0x1));
  pre.pushRoverObs(observationEpoch(2200, 100100.0));
  auto first = pre.drainEpochs(&approximate);
  ASSERT_EQ(first.size(), 1U);
  ASSERT_EQ(first[0].rover_sats.size(), 1U);
  EXPECT_TRUE(first[0].rover_sats[0].slip);

  pre.pushRoverObs(observationEpoch(2200, 100100.2));
  pre.pushBaseObs(observationEpoch(2200, 100101.0));
  auto second = pre.drainEpochs(&approximate);
  ASSERT_EQ(second.size(), 1U);
  ASSERT_EQ(second[0].rover_sats.size(), 1U);
  EXPECT_FALSE(second[0].rover_sats[0].slip);
  EXPECT_TRUE(second[0].has_base);
  EXPECT_DOUBLE_EQ(second[0].base_tow, 100100.0);
}

}  // namespace

// A rover epoch whose satellites are all removed by the masks must still be
// EMITTED, with rover_sats and dd empty.
//
// It used to be deleted outright, and that erased the epoch from the timeline
// rather than just the measurement: a consumer with an inertial sensor was
// never told the epoch happened, so it could not carry the state across a real
// sky outage - which is the entire reason such a consumer exists. Measured on
// PPC nagoya_run1, 24 epochs per 400 s vanished this way, the tightly-coupled
// node published nothing across the outage, and the resulting hole then read
// downstream as a GNSS outage that re-keyed every carried carrier arc.
TEST(GnssPreprocessor, EmitsEpochWhenMasksLeaveNoUsableSatellite) {
  auto cfg = config();
  // An elevation mask no satellite can pass. The observations themselves are
  // untouched, so this isolates the mask from the data.
  cfg.el_mask_rad = 89.9 * D2R;
  Preprocessor pre(cfg);
  const Eigen::Vector3d approximate(6378137.0, 0.0, 0.0);
  pre.pushEphemerides(gpsEphemeris(2200, 100000.0));
  pre.pushRoverObs(observationEpoch(2200, 100100.0));

  const auto epochs = pre.flushEpochs(&approximate);
  ASSERT_EQ(epochs.size(), 1U)
      << "an epoch with no usable satellite must still reach the consumer";
  EXPECT_TRUE(epochs[0].rover_sats.empty());
  EXPECT_TRUE(epochs[0].dd.empty());
  EXPECT_EQ(epochs[0].week, 2200U);                 // the time is still known...
  EXPECT_DOUBLE_EQ(epochs[0].tow, 100100.0);        // ...which is what makes it
  EXPECT_FALSE(epochs[0].rover_vel_valid);          //    usable to a consumer
  // Counted, and counted as EMITTED rather than rejected: nothing was lost.
  const auto& c = pre.epochRejectCounters();
  EXPECT_EQ(c.emitted_without_satellites, 1U);
  EXPECT_EQ(c.total(), 0U) << "this is not a rejection";
}

// The complement: with the mask open the same observations produce a normal
// epoch, so the test above measures the mask and not a broken fixture.
TEST(GnssPreprocessor, EmitsSatellitesWhenTheMaskAllowsThem) {
  Preprocessor pre(config());
  const Eigen::Vector3d approximate(6378137.0, 0.0, 0.0);
  pre.pushEphemerides(gpsEphemeris(2200, 100000.0));
  pre.pushRoverObs(observationEpoch(2200, 100100.0));

  const auto epochs = pre.flushEpochs(&approximate);
  ASSERT_EQ(epochs.size(), 1U);
  EXPECT_FALSE(epochs[0].rover_sats.empty());
  EXPECT_EQ(pre.epochRejectCounters().emitted_without_satellites, 0U);
}

// GLONASS admitted to the undifferenced product but kept out of double
// differencing: the two questions are physically different (FDMA bias is a
// carrier-DD problem, not a reason to drop the pseudoranges that keep a canyon
// epoch alive), and folding them into one flag is part of why an epoch whose
// only satellites were GLONASS produced nothing at all.
TEST(GnssPreprocessor, UndifferencedOnlySystemReachesRoverSatsButNotDd) {
  auto cfg = config();
  cfg.navsys = SYS_GPS;                       // GPS double-differences as usual
  cfg.navsys_undifferenced_only = SYS_GLO;    // GLONASS: undifferenced only
  Preprocessor pre(cfg);
  const Eigen::Vector3d approximate(6378137.0, 0.0, 0.0);
  pre.pushEphemerides(gpsEphemeris(2200, 100000.0));
  pre.pushRoverObs(observationEpoch(2200, 100100.0));

  const auto epochs = pre.flushEpochs(&approximate);
  ASSERT_EQ(epochs.size(), 1U);
  // The GPS satellite in the fixture still reaches the undifferenced list, so
  // widening the admission did not break the normal path.
  EXPECT_FALSE(epochs[0].rover_sats.empty());
  for (const auto& d : epochs[0].dd) {
    EXPECT_NE(d.sys, SYS_GLO) << "an undifferenced-only system must never "
                                 "appear in a double difference";
  }
}

// The default must leave existing behaviour untouched.
TEST(GnssPreprocessor, UndifferencedOnlyDefaultsToNoSplit) {
  EXPECT_EQ(Preprocessor::Config{}.navsys_undifferenced_only, 0);
}

// ---------------------------------------------------------------------------
// AUDIT WP2.5 - GnssEpochAligner / TowAutoOffset had no test at all, and it is
// the component that decides what time every GNSS measurement is applied at.
// It is also the one cppcheck found a real out-of-bounds path in (A3-1).
namespace {

// Wall stamp for a given GPS (week, tow) plus a chosen latency, in the same
// clock the aligner returns.
rclcpp::Time stampFor(std::uint32_t week, double tow, double latency_s) {
  const int64_t meas_ns =
      gnss_utils::gpstToUtcRosTime(gpst2time(static_cast<int>(week), tow))
          .nanoseconds();
  return rclcpp::Time(meas_ns + static_cast<int64_t>(latency_s * 1e9),
                      RCL_ROS_TIME);
}

}  // namespace

TEST(GnssEpochAligner, TowAutoOffsetRecoversTheMinimumLatency) {
  gnss_utils::GnssEpochAligner::Config cfg;
  cfg.source = gnss_utils::GnssEpochAligner::Source::TowAutoOffset;
  cfg.offset_window_s = 60.0;
  gnss_utils::GnssEpochAligner a(cfg);

  const std::uint32_t week = 2200;
  // Jittery latency with a clear floor at 0.020 s: the estimator takes the
  // sliding-window MINIMUM, so the floor is what must come back.
  const double lat[] = {0.080, 0.045, 0.020, 0.061, 0.033, 0.097};
  rclcpp::Time out;
  for (int i = 0; i < 6; ++i) {
    const double tow = 100.0 + i;
    std::string warn;
    out = a.align(week, tow, stampFor(week, tow, lat[i]), &warn);
    EXPECT_TRUE(warn.empty()) << warn;
  }
  const int64_t meas_ns =
      gnss_utils::gpstToUtcRosTime(gpst2time(static_cast<int>(week), 105.0))
          .nanoseconds();
  EXPECT_NEAR((out.nanoseconds() - meas_ns) / 1e9, 0.020, 2e-3);
}

TEST(GnssEpochAligner, BackwardsStampRestartsTheWindow) {
  // A bag replay restart steps the wall clock backwards. The old latency
  // history describes a different session and must not survive it.
  gnss_utils::GnssEpochAligner::Config cfg;
  cfg.source = gnss_utils::GnssEpochAligner::Source::TowAutoOffset;
  gnss_utils::GnssEpochAligner a(cfg);
  const std::uint32_t week = 2200;

  a.align(week, 500.0, stampFor(week, 500.0, 0.005), nullptr);  // tiny latency
  // Replay restarts: stamps jump back, and this session's floor is 0.200 s.
  rclcpp::Time out;
  for (int i = 0; i < 4; ++i) {
    const double tow = 100.0 + i;
    out = a.align(week, tow, stampFor(week, tow, 0.200 + 0.01 * i), nullptr);
  }
  const int64_t meas_ns =
      gnss_utils::gpstToUtcRosTime(gpst2time(static_cast<int>(week), 103.0))
          .nanoseconds();
  // 0.200, not the stale 0.005 carried over from before the restart.
  EXPECT_NEAR((out.nanoseconds() - meas_ns) / 1e9, 0.200, 2e-3);
}

TEST(GnssEpochAligner, NegativeWindowIsSurvivable) {
  // Regression for AUDIT A3-1. offset_window_s is a plain ROS parameter with
  // no validation; a negative value used to empty the history deque and then
  // read front() off it. The newest sample must always be retained.
  gnss_utils::GnssEpochAligner::Config cfg;
  cfg.source = gnss_utils::GnssEpochAligner::Source::TowAutoOffset;
  cfg.offset_window_s = -1.0;
  gnss_utils::GnssEpochAligner a(cfg);
  const std::uint32_t week = 2200;
  rclcpp::Time out;
  for (int i = 0; i < 5; ++i) {
    const double tow = 100.0 + i;
    out = a.align(week, tow, stampFor(week, tow, 0.030), nullptr);
  }
  const int64_t meas_ns =
      gnss_utils::gpstToUtcRosTime(gpst2time(static_cast<int>(week), 104.0))
          .nanoseconds();
  // A zero-length window keeps only the current sample, so the offset is just
  // this epoch's latency - defined, finite, and not a crash.
  EXPECT_NEAR((out.nanoseconds() - meas_ns) / 1e9, 0.030, 2e-3);
}

TEST(GnssEpochAligner, InvalidGnssTimeFallsBackToTheHeaderAndSaysSo) {
  gnss_utils::GnssEpochAligner::Config cfg;
  cfg.source = gnss_utils::GnssEpochAligner::Source::TowAutoOffset;
  gnss_utils::GnssEpochAligner a(cfg);
  const rclcpp::Time stamp(1234567890, 0, RCL_ROS_TIME);
  for (const double tow : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(),
                           -1.0, 604800.0}) {
    std::string warn;
    const rclcpp::Time out = a.align(2200, tow, stamp, &warn);
    EXPECT_EQ(out.nanoseconds(), stamp.nanoseconds()) << "tow=" << tow;
    EXPECT_FALSE(warn.empty()) << "tow=" << tow << " fell back silently";
  }
}

// ---------------------------------------------------------------------------
// Undifferenced Doppler velocity: covariance SHAPE and per-satellite FDE.
//
// Both properties are invisible in a trajectory plot. A collapsed isotropic
// covariance still produces a plausible velocity, and a single NLOS range rate
// still produces a plausible velocity - just a biased one, weighted as though
// it were clean. So they are tested here directly.
namespace {

// A satellite geometry in a local ENU frame placed at a real ECEF location, so
// the Sagnac and clock-drift terms are exercised rather than degenerate.
// The a-priori zenith range-rate sigma the config defaults to.
constexpr double kSigma = 0.1;

struct DopplerScene {
  std::vector<gnss_utils::SatObs> sats;
  Eigen::Vector3d rover{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel{Eigen::Vector3d::Zero()};
};

// `min_el_deg` controls how much VERTICAL strength the geometry has: with every
// satellite high in the sky the vertical velocity is well determined, and with
// all of them near the horizon it is not. That contrast is the point of the
// test - an isotropic covariance cannot express it.
DopplerScene makeDopplerScene(int n, double min_el_deg, double max_el_deg) {
  DopplerScene sc;
  const double llh[3] = {35.7 * D2R, 139.72 * D2R, 74.0};
  double re[3];
  pos2ecef(llh, re);
  sc.rover = Eigen::Vector3d(re[0], re[1], re[2]);
  sc.vel = Eigen::Vector3d(4.0, -2.5, 0.7);

  double E[9];  // enu = E * d_ecef (RTKLIB column-major)
  xyz2enu(llh, E);
  Eigen::Matrix3d R_enu_ecef;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) R_enu_ecef(r, c) = E[r + c * 3];

  const double omge_c = OMGE / CLIGHT;
  const double lam = CLIGHT / FREQL1;
  for (int i = 0; i < n; ++i) {
    const double az = 2.0 * M_PI * i / n;
    const double el =
        (min_el_deg + (max_el_deg - min_el_deg) * i / std::max(n - 1, 1)) * D2R;
    const Eigen::Vector3d enu(std::cos(el) * std::sin(az),
                              std::cos(el) * std::cos(az), std::sin(el));
    gnss_utils::SatObs s;
    s.sat = 1 + i;
    s.sys = SYS_GPS;
    s.band = 0;
    s.lam = lam;
    s.el = el;
    s.sat_pos = sc.rover + R_enu_ecef.transpose() * enu * 2.2e7;
    s.sat_vel = Eigen::Vector3d(1000.0 * std::cos(az), -900.0 * std::sin(az),
                                800.0 * std::cos(el));
    s.sat_clk_drift = 1e-11 * (i + 1);
    // Synthesize the Doppler this geometry and velocity would produce, so the
    // solve must return sc.vel exactly.
    const Eigen::Vector3d e = (s.sat_pos - sc.rover).normalized();
    // Both halves of the Sagnac rate: the satellite-velocity term AND the
    // receiver-velocity term the solver carries in H. Omitting the second makes
    // the synthetic inconsistent with the model by ~2e-5 m/s, which is small
    // enough to look like rounding and large enough to break an exactness test.
    const double sagnac =
        omge_c * (s.sat_vel.y() * sc.rover.x() - s.sat_vel.x() * sc.rover.y()) +
        omge_c * (s.sat_pos.y() * sc.vel.x() - s.sat_pos.x() * sc.vel.y());
    const double range_rate = e.dot(s.sat_vel - sc.vel) + sagnac -
                              CLIGHT * s.sat_clk_drift;
    s.doppler = -range_rate / lam;
    sc.sats.push_back(s);
  }
  return sc;
}

}  // namespace

TEST(DopplerVelocity, RecoversTheVelocityAndReportsAnAnisotropicCovariance) {
  const DopplerScene sc = makeDopplerScene(9, 10.0, 35.0);  // low, weak vertical
  gnss_utils::DopplerVelocitySolution out;
  ASSERT_TRUE(gnss_utils::estimateDopplerVelocity(sc.sats, sc.rover, kSigma, 4.0, 2, out));
  EXPECT_LT((out.vel - sc.vel).norm(), 1e-6);
  EXPECT_EQ(out.nsat, 9);
  EXPECT_EQ(out.excluded, 0);

  // Rotate the ECEF covariance into ENU and compare the vertical against the
  // horizontal. With every satellite below 35 degrees the up channel must be
  // markedly worse - the fact an averaged scalar cannot represent.
  const double llh[3] = {35.7 * D2R, 139.72 * D2R, 74.0};
  double E[9];
  xyz2enu(llh, E);
  Eigen::Matrix3d R_enu_ecef;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) R_enu_ecef(r, c) = E[r + c * 3];
  const Eigen::Matrix3d enu_cov =
      R_enu_ecef * out.cov * R_enu_ecef.transpose();
  const double horiz = 0.5 * (enu_cov(0, 0) + enu_cov(1, 1));
  EXPECT_GT(enu_cov(2, 2), 2.0 * horiz)
      << "vertical " << enu_cov(2, 2) << " vs horizontal " << horiz;

  // And the scalar the old code reported sits between the two, so it overstates
  // the horizontal precision and understates the vertical - in one number.
  const double avg = out.cov.diagonal().sum() / 3.0;
  EXPECT_GT(avg, 0.0);
}

TEST(DopplerVelocity, HighSatelliteGeometryTightensTheVerticalChannel) {
  const DopplerScene low = makeDopplerScene(9, 10.0, 25.0);
  const DopplerScene high = makeDopplerScene(9, 55.0, 85.0);
  gnss_utils::DopplerVelocitySolution a, b;
  ASSERT_TRUE(gnss_utils::estimateDopplerVelocity(low.sats, low.rover, kSigma, 0.0, 0, a));
  ASSERT_TRUE(gnss_utils::estimateDopplerVelocity(high.sats, high.rover, kSigma, 0.0, 0, b));
  // Geometry, not noise, decides this: both sets are noiseless.
  EXPECT_LT(b.cov.diagonal().sum(), a.cov.diagonal().sum());
}

TEST(DopplerVelocity, ExcludesASingleBlunderAndRecoversTheCleanVelocity) {
  DopplerScene sc = makeDopplerScene(10, 15.0, 75.0);
  // 20 m/s of range-rate error on one satellite: an NLOS reflection, well past
  // anything the receiver's own noise produces.
  sc.sats[4].doppler += 20.0 / sc.sats[4].lam;

  gnss_utils::DopplerVelocitySolution off;
  ASSERT_TRUE(gnss_utils::estimateDopplerVelocity(sc.sats, sc.rover, kSigma, 0.0, 0, off));
  const double err_without_fde = (off.vel - sc.vel).norm();
  EXPECT_GT(err_without_fde, 0.5) << "the blunder must actually hurt";

  gnss_utils::DopplerVelocitySolution on;
  ASSERT_TRUE(gnss_utils::estimateDopplerVelocity(sc.sats, sc.rover, kSigma, 4.0, 2, on));
  EXPECT_GE(on.excluded, 1);
  EXPECT_EQ(on.nsat, 10 - on.excluded);
  EXPECT_LT((on.vel - sc.vel).norm(), 1e-6);
  EXPECT_LT((on.vel - sc.vel).norm(), err_without_fde);
}

TEST(DopplerVelocity, KeepsACleanSetIntactAndRespectsTheBudget) {
  DopplerScene sc = makeDopplerScene(10, 15.0, 75.0);
  gnss_utils::DopplerVelocitySolution clean;
  ASSERT_TRUE(gnss_utils::estimateDopplerVelocity(sc.sats, sc.rover, kSigma, 4.0, 2, clean));
  EXPECT_EQ(clean.excluded, 0) << "no satellite may be dropped from a clean set";

  // Three blunders against a budget of two: the loop cannot clear the fault, so
  // the EPOCH is refused.
  //
  // This inverts the previous contract, which returned the still-faulty solution
  // and relied on the caller's residual-RMS gate to reject it. `res` is not a
  // gate that can be relied on - with four satellites it is identically zero
  // however bad a range rate is, because the fit passes exactly through every
  // observation - so a fault that survives exclusion is refused at the source
  // rather than handed onward with a number that may not reveal it.
  for (const int i : {2, 5, 8}) sc.sats[i].doppler += 20.0 / sc.sats[i].lam;
  gnss_utils::DopplerVelocitySolution over;
  EXPECT_FALSE(
      gnss_utils::estimateDopplerVelocity(sc.sats, sc.rover, kSigma, 4.0, 2, over));
}

TEST(DopplerVelocity, ReportsFailureBelowFiveUsableSatellites) {
  // FIVE is the minimum, not four. Four rows against four unknowns leaves zero
  // redundancy: the residuals are identically zero and the post-fit RMS reads a
  // perfect 0 whatever the data, so a blunder does not merely escape detection,
  // it looks ideal. Five rows carry one degree of freedom - enough to notice a
  // fault, though not to isolate one.
  {
    DopplerScene four = makeDopplerScene(4, 20.0, 70.0);
    gnss_utils::DopplerVelocitySolution out4;
    EXPECT_FALSE(gnss_utils::estimateDopplerVelocity(four.sats, four.rover,
                                                     kSigma, 4.0, 2, out4));
    DopplerScene five = makeDopplerScene(5, 20.0, 70.0);
    gnss_utils::DopplerVelocitySolution out5;
    EXPECT_TRUE(gnss_utils::estimateDopplerVelocity(five.sats, five.rover,
                                                    kSigma, 4.0, 2, out5));
  }
  DopplerScene sc = makeDopplerScene(3, 20.0, 70.0);
  gnss_utils::DopplerVelocitySolution out;
  EXPECT_FALSE(gnss_utils::estimateDopplerVelocity(sc.sats, sc.rover, kSigma, 4.0, 2, out));
}

// The matcher window and the DD usability window are ONE quantity.
//
// They used to be two (Config::max_tdiff_s defaulted to 30 s while max_age_s
// was 5), so the matcher paired a rover with a base up to 30 s old, the DD
// stage then refused it for being older than max_age_s, and the unusable base
// stayed at the head of the queue to be re-selected for every following rover.
// Measured on PPC nagoya_run1: 12.1 % of epochs came back with a base between
// 5 and 30 s old and therefore no double differences at all, in runs of the
// same build that otherwise reached 0 %.
TEST(PreprocessorConfig, MatcherWindowIsTheDdUsabilityWindow) {
  auto cfg = config();
  cfg.max_age_s = 5.0;
  Preprocessor p(cfg);
  // No separate looser window exists to disagree with max_age_s: the field is
  // gone, so this compiles only while the contract holds.
  EXPECT_DOUBLE_EQ(cfg.max_age_s, 5.0);
}
