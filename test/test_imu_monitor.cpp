#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>

#include "imu_monitor.hpp"

using gnss_visualizer::ImuMonitor;
using gnss_visualizer::ImuSample;

namespace {

constexpr double kG = ImuMonitor::kGravity;
constexpr int64_t kMs = 1'000'000;
constexpr int64_t kStart = 1'000'000'000'000;
constexpr double kPi = 3.14159265358979323846;

using Vector = std::array<double, 3>;

// Feeds a 100 Hz stream and closes a bin every 200 ms, like the node does.
class Feeder {
 public:
  explicit Feeder(ImuMonitor& monitor) : monitor_(monitor) {}

  void run(double seconds, const std::function<void(int, Vector&, Vector&)>& motion) {
    const int steps = static_cast<int>(std::lround(seconds * 100.0));
    for (int i = 0; i < steps; ++i) {
      Vector accel{}, gyro{};
      motion(i, accel, gyro);
      sample(accel, gyro);
      advance(10 * kMs);
    }
  }
  void hold(double seconds, const Vector& accel, const Vector& gyro = {}) {
    run(seconds, [&](int, Vector& a, Vector& w) {
      a = accel;
      w = gyro;
    });
  }
  void sample(const Vector& accel, const Vector& gyro) {
    ImuSample s;
    s.stamp_ns = stamp_;
    s.arrival_ns = stamp_ + 5 * kMs;
    s.accel = accel;
    s.gyro = gyro;
    monitor_.addSample(s);
    if (++since_close_ == 20) {
      since_close_ = 0;
      monitor_.closeBin(stamp_);
    }
  }
  void advance(int64_t ns) { stamp_ += ns; }
  int64_t stamp() const { return stamp_; }

 private:
  ImuMonitor& monitor_;
  int64_t stamp_{kStart};
  int since_close_{0};
};

double upAngleDeg(const ImuMonitor& monitor, const Vector& expected) {
  const auto& [w, x, y, z] = monitor.status().attitude;
  const Vector up = {2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)};
  const double cosine = up[0] * expected[0] + up[1] * expected[1] + up[2] * expected[2];
  return std::acos(std::min(1.0, std::max(-1.0, cosine))) * 180.0 / kPi;
}

}  // namespace

TEST(ImuMonitorAttitude, LevelSensorReportsPlusZ) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  feeder.hold(2.0, {0.0, 0.0, kG});
  const auto& status = monitor.status();
  ASSERT_TRUE(status.attitude_ready);
  EXPECT_EQ(status.up_axis, 2);
  EXPECT_EQ(status.up_sign, 1);
  EXPECT_LT(status.tilt_deg, 0.1);
}

TEST(ImuMonitorAttitude, TiltedSensorReportsItsTilt) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  const double tilt = 30.0 * kPi / 180.0;
  feeder.hold(2.0, {kG * std::sin(tilt), 0.0, kG * std::cos(tilt)});
  EXPECT_EQ(monitor.status().up_axis, 2);
  EXPECT_NEAR(monitor.status().tilt_deg, 30.0, 0.2);
}

TEST(ImuMonitorAttitude, UpsideDownSensorReportsMinusZ) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  feeder.hold(2.0, {0.0, 0.0, -kG});
  const auto& status = monitor.status();
  ASSERT_TRUE(status.attitude_ready);
  for (const double value : status.attitude) EXPECT_TRUE(std::isfinite(value));
  EXPECT_EQ(status.up_axis, 2);
  EXPECT_EQ(status.up_sign, -1);
  EXPECT_LT(status.tilt_deg, 0.1);
}

TEST(ImuMonitorAttitude, FollowsARotationAboutX) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  feeder.hold(2.0, {0.0, 0.0, kG});
  // A positive quarter turn about +X brings +Y up.
  feeder.run(2.0, [](int i, Vector& a, Vector& w) {
    const double angle = std::min(i * 0.01, kPi / 2.0);
    a = {0.0, kG * std::sin(angle), kG * std::cos(angle)};
    w = {angle < kPi / 2.0 ? 1.0 : 0.0, 0.0, 0.0};
  });
  EXPECT_EQ(monitor.status().up_axis, 1);
  EXPECT_EQ(monitor.status().up_sign, 1);
  EXPECT_LT(upAngleDeg(monitor, {0.0, 1.0, 0.0}), 3.0);
}

TEST(ImuMonitorAttitude, SustainedTurnDoesNotPullTiltToTheSpecificForce) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  feeder.hold(2.0, {0.0, 0.0, kG});
  feeder.hold(10.0, {0.0, 0.3 * kG, kG}, {0.0, 0.0, 0.3});
  EXPECT_FALSE(monitor.status().still);
  EXPECT_LT(upAngleDeg(monitor, {0.0, 0.0, 1.0}), 9.0);  // 16.7 deg if the force were trusted
}

TEST(ImuMonitorAttitude, SlowMotionHoldsTheTiltDespiteAnUnknownGyroBias) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  // Hand-held: too much gyro scatter to ever be still, with 1 deg/s of bias.
  feeder.run(60.0, [](int i, Vector& a, Vector& w) {
    a = {0.0, 0.0, kG};
    w = {0.0175 + (i % 2 ? 0.03 : -0.03), 0.0, 0.0};
  });
  ASSERT_TRUE(monitor.status().attitude_ready);
  EXPECT_FALSE(monitor.status().have_still);
  EXPECT_LT(upAngleDeg(monitor, {0.0, 0.0, 1.0}), 3.0);  // about 20 deg with the moving gain alone
}

TEST(ImuMonitorAttitude, LongGapWaitsForANewTilt) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  feeder.hold(2.0, {0.0, 0.0, kG});
  feeder.advance(800 * kMs);
  feeder.hold(0.2, {kG, 0.0, 0.0});
  EXPECT_FALSE(monitor.status().attitude_ready);
  feeder.hold(1.0, {kG, 0.0, 0.0});
  EXPECT_TRUE(monitor.status().attitude_ready);
  EXPECT_EQ(monitor.status().up_axis, 0);
}

TEST(ImuMonitorAttitude, DoesNotInitializeFromAnAccelerationThatIsNotGravity) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  feeder.hold(2.0, {0.0, 0.0, 1.5 * kG});
  EXPECT_FALSE(monitor.status().attitude_ready);
}

TEST(ImuMonitorAttitude, GravityFreeTopicIsReportedWithoutAttitude) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  feeder.hold(2.0, {0.05, 0.0, 0.02}, {0.0, 0.0, 0.01});
  const auto& status = monitor.status();
  EXPECT_TRUE(status.gravity_free);
  EXPECT_FALSE(status.attitude_ready);
  EXPECT_TRUE(std::isfinite(monitor.bins().back().accel[0].high));
}

TEST(ImuMonitorStill, UsesTheLimitsOfTheEstimators) {
  const auto still = [](double accel_scatter, double gyro_scatter) {
    ImuMonitor monitor;
    Feeder feeder(monitor);
    feeder.run(3.0, [&](int i, Vector& a, Vector& w) {
      const double sign = i % 2 ? 1.0 : -1.0;
      a = {sign * accel_scatter, 0.0, kG};
      w = {sign * gyro_scatter, 0.0, 0.0};
    });
    return monitor.status().still;
  };
  EXPECT_TRUE(still(0.10, 0.005));
  EXPECT_FALSE(still(0.20, 0.005));
  EXPECT_FALSE(still(0.10, 0.020));
}

TEST(ImuMonitorStill, SteadyTurnIsNotStill) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  feeder.hold(3.0, {0.0, 0.0, kG}, {0.0, 0.0, 0.2});
  EXPECT_FALSE(monitor.status().still);
}

TEST(ImuMonitorStill, AccelerometerOffsetDoesNotPreventStillness) {
  for (const double offset : {-0.4, 0.4}) {
    ImuMonitor monitor;
    Feeder feeder(monitor);
    feeder.hold(3.0, {0.0, 0.0, kG + offset});
    EXPECT_TRUE(monitor.status().still) << offset;
    EXPECT_NEAR(monitor.status().accel_norm, kG + offset, 1e-6);
  }
}

TEST(ImuMonitorStill, RecoversAfterTheSensorIsPlacedDifferently) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  feeder.hold(3.0, {0.0, 0.0, kG + 0.4});
  ASSERT_TRUE(monitor.status().still);
  feeder.hold(2.0, {0.0, 0.0, kG}, {0.5, 0.0, 0.0});
  ASSERT_FALSE(monitor.status().still);
  feeder.hold(3.0, {0.0, kG - 0.4, 0.0});
  EXPECT_TRUE(monitor.status().still);
}

TEST(ImuMonitorStill, SteadySpecificForceThatIsNotGravityIsNotStill) {
  for (const double scale : {0.8, 1.2}) {
    ImuMonitor monitor;
    Feeder feeder(monitor);
    feeder.hold(3.0, {0.0, 0.0, scale * kG});
    EXPECT_FALSE(monitor.status().still) << scale;
  }
}

TEST(ImuMonitorStill, ReportsBiasAndKeepsItWhileMoving) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  feeder.hold(3.0, {0.0, 0.0, kG}, {0.01, -0.02, 0.0});
  EXPECT_TRUE(monitor.status().still);
  EXPECT_NEAR(monitor.status().gyro_bias[0], 0.01 * 180.0 / kPi, 1e-6);
  EXPECT_NEAR(monitor.status().accel_norm, kG, 1e-6);
  feeder.hold(2.0, {0.0, 0.0, kG}, {0.5, 0.0, 0.0});
  EXPECT_FALSE(monitor.status().still);
  EXPECT_TRUE(monitor.status().have_still);
  EXPECT_NEAR(monitor.status().gyro_bias[0], 0.01 * 180.0 / kPi, 1e-6);
}

namespace {

// Sensor rocking about X by +-40 deg: gravity turns as right-handed axes predict.
void rock(Feeder& feeder, double seconds, const Vector& accel_sign, double gyro_sign) {
  static double t = 0.0;
  feeder.run(seconds, [&](int, Vector& a, Vector& w) {
    constexpr double amplitude = 40.0 * kPi / 180.0, omega = 2.0 * kPi / 8.0;
    const double angle = amplitude * std::sin(omega * t);
    a = {0.0, accel_sign[1] * kG * std::sin(angle), accel_sign[2] * kG * std::cos(angle)};
    w = {gyro_sign * amplitude * omega * std::cos(omega * t), 0.0, 0.0};
    t += 0.01;
  });
}

}  // namespace

TEST(ImuMonitorFrame, RightHandedAxesAreAccepted) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  rock(feeder, 60.0, {1.0, 1.0, 1.0}, 1.0);
  EXPECT_FALSE(monitor.status().frame_mismatch);
}

TEST(ImuMonitorFrame, MirroredAxesAreReported) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  rock(feeder, 60.0, {1.0, -1.0, 1.0}, 1.0);
  EXPECT_TRUE(monitor.status().frame_mismatch);
}

TEST(ImuMonitorFrame, ReversedGyroIsReported) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  rock(feeder, 60.0, {1.0, 1.0, 1.0}, -1.0);
  EXPECT_TRUE(monitor.status().frame_mismatch);
}

TEST(ImuMonitorFrame, GravitySignAloneIsNotAMismatch) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  rock(feeder, 60.0, {1.0, -1.0, -1.0}, 1.0);
  EXPECT_FALSE(monitor.status().frame_mismatch);
}

TEST(ImuMonitorFrame, RotationAboutGravityGivesNoVerdict) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  feeder.hold(60.0, {0.0, 0.0, kG}, {0.0, 0.0, -0.5});
  EXPECT_FALSE(monitor.status().frame_mismatch);
}

TEST(ImuMonitorFrame, WarningClearsOnceTheAxesAreCorrected) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  rock(feeder, 60.0, {1.0, -1.0, 1.0}, 1.0);
  ASSERT_TRUE(monitor.status().frame_mismatch);
  rock(feeder, 100.0, {1.0, 1.0, 1.0}, 1.0);
  EXPECT_FALSE(monitor.status().frame_mismatch);
}

TEST(ImuMonitorFrame, WarningStaysWhileNothingTilts) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  rock(feeder, 60.0, {1.0, -1.0, 1.0}, 1.0);
  ASSERT_TRUE(monitor.status().frame_mismatch);
  feeder.hold(30.0, {0.0, 0.0, kG});
  EXPECT_TRUE(monitor.status().frame_mismatch);
}

TEST(ImuMonitorTiming, ReportsRate) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  feeder.hold(6.0, {0.0, 0.0, kG});
  EXPECT_NEAR(monitor.status().rate_hz, 100.0, 0.5);
  EXPECT_EQ(monitor.status().gaps, 0U);
  EXPECT_NEAR(monitor.bins().back().latency_ms.low, 5.0, 1e-6);
}

TEST(ImuMonitorTiming, CountsAGap) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  feeder.hold(3.0, {0.0, 0.0, kG});
  feeder.advance(50 * kMs);
  feeder.hold(1.0, {0.0, 0.0, kG});
  EXPECT_EQ(monitor.status().gaps, 1U);
  EXPECT_NEAR(monitor.status().max_gap_ms, 60.0, 1e-6);
}

TEST(ImuMonitorTiming, BurstDeliveryIsNotAGap) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  for (int burst = 0; burst < 100; ++burst) {
    for (int i = 0; i < 10; ++i) {
      feeder.sample({0.0, 0.0, kG}, {});
      feeder.advance(1000);
    }
    feeder.advance(100 * kMs - 10000);
  }
  EXPECT_EQ(monitor.status().gaps, 0U);
  EXPECT_NEAR(monitor.status().rate_hz, 100.0, 15.0);
}

TEST(ImuMonitorTiming, RepeatedDropoutsAreStillCounted) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  for (int i = 0; i < 1000; ++i) {
    feeder.sample({0.0, 0.0, kG}, {});
    feeder.advance((i % 15 == 14 ? 30 : 10) * kMs);
  }
  EXPECT_GT(monitor.status().gaps, 30U);
}

TEST(ImuMonitorTiming, CountsOutOfOrderAndDuplicateStamps) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  feeder.sample({0.0, 0.0, kG}, {});
  feeder.sample({0.0, 0.0, kG}, {});  // same stamp
  feeder.advance(-5 * kMs);
  feeder.sample({0.0, 0.0, kG}, {});
  monitor.closeBin(feeder.stamp());
  EXPECT_EQ(monitor.status().duplicates, 1U);
  EXPECT_EQ(monitor.status().out_of_order, 1U);
  EXPECT_LT(monitor.bins().back().dt_ms.low, 0.0);
}

TEST(ImuMonitorTiming, LargeBackwardJumpRestartsEverything) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  feeder.hold(2.0, {0.0, 0.0, kG});
  ASSERT_TRUE(monitor.status().attitude_ready);
  EXPECT_FALSE(monitor.takeRestarted());
  feeder.advance(-3000 * kMs);
  feeder.sample({0.0, 0.0, kG}, {});
  EXPECT_TRUE(monitor.takeRestarted());
  EXPECT_FALSE(monitor.takeRestarted());
  EXPECT_TRUE(monitor.bins().empty());
  EXPECT_EQ(monitor.status().samples, 1U);
  EXPECT_EQ(monitor.status().out_of_order, 0U);
}

TEST(ImuMonitorTiming, ReportsTheOffsetToTheGnssObservations) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  monitor.noteObservation(kStart);
  monitor.closeBin(kStart);
  EXPECT_FALSE(std::isfinite(monitor.status().imu_minus_observation_s));
  feeder.sample({0.0, 0.0, kG}, {});
  monitor.noteObservation(kStart - 18'000 * kMs);
  monitor.closeBin(kStart);
  EXPECT_NEAR(monitor.status().imu_minus_observation_s, 18.0, 1e-9);
}

TEST(ImuMonitorInput, RejectsSamplesThatAreNotFinite) {
  ImuMonitor monitor;
  Feeder feeder(monitor);
  feeder.hold(1.0, {0.0, 0.0, kG});
  const auto before = monitor.status().samples;
  feeder.sample({std::numeric_limits<double>::quiet_NaN(), 0.0, kG}, {});
  feeder.sample({0.0, 0.0, kG}, {std::numeric_limits<double>::infinity(), 0.0, 0.0});
  EXPECT_EQ(monitor.status().rejected, 2U);
  EXPECT_EQ(monitor.status().samples, before);
  for (const double value : monitor.status().attitude) EXPECT_TRUE(std::isfinite(value));
}

TEST(ImuMonitorBins, KeepALimitedHistoryAndNeverReuseANumber) {
  ImuMonitor monitor;
  for (int i = 0; i < 700; ++i) monitor.closeBin(kStart + i * 200 * kMs);
  ASSERT_EQ(monitor.bins().size(), ImuMonitor::kHistoryBins);
  EXPECT_EQ(monitor.bins().back().seq, 699U);
  EXPECT_FALSE(std::isfinite(monitor.bins().back().dt_ms.high));
  monitor.reset();
  EXPECT_TRUE(monitor.bins().empty());
  monitor.closeBin(kStart);
  EXPECT_EQ(monitor.bins().back().seq, 700U);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
