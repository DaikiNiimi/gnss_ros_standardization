//
// ins_utils::SensorTickClock - maps a receiver sample counter (u-blox
// UBX-ESF-RAW sTtag) onto a monotone ROS timeline.
//
// The defect these tests pin: handleEsfRaw() used to stamp each batch from
// `now()` and spread the samples BACKWARDS by their sTtag deltas, so the
// receiver clock was discarded between batches. Frame k's last sample landed
// at anchor_k while frame k+1's first sample landed at anchor_{k+1} - span,
// which is earlier whenever the arrival gap is shorter than the batch span -
// routine with a 10 ms poll timer and ~10 ms batches, and guaranteed when two
// frames arrive in one read. Downstream, the FGO drops the whole IMU factor
// for such an epoch and its imu_buffer_ stops being sorted.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "gnss_ros_standardization/ins_utils.hpp"

namespace {

namespace ins = gnss_ros_standardization::ins_utils;

constexpr double kSttagTick = 39.0625e-6;  // ubx::esf_raw::STTAG_TICK_S
constexpr std::int64_t kTicksPer10ms = 256;

rclcpp::Time at(double seconds) {
  return rclcpp::Time(static_cast<std::int64_t>(seconds * 1e9), RCL_ROS_TIME);
}

// Arrival times are produced by calling now() in read order, so a real stream
// can NEVER hand back an earlier arrival than the one before. That bounds how
// fast transport latency may fall: at most one sample interval per sample.
// Scenarios that ignore this are unphysical, and the clock rightly treats
// them as a stream restart - which silently turns a latency test into a
// re-seed test. This guard keeps that mistake out of the suite.
class ArrivalSequence {
 public:
  rclcpp::Time next(double arrival_s) {
    EXPECT_GE(arrival_s, last_) << "unphysical scenario: arrival went backwards";
    last_ = arrival_s;
    return at(arrival_s);
  }

 private:
  double last_{-std::numeric_limits<double>::infinity()};
};

ins::SensorTickClock::Config sttagConfig() {
  ins::SensorTickClock::Config c;
  c.tick_s = kSttagTick;
  c.modulus = 1ull << 32;
  c.window_s = 20.0;
  return c;
}

}  // namespace

// THE headline regression. Two batches handed over with the SAME arrival
// stamp to the nanosecond - what a single strread() of 4096 bytes produces.
// The old code gave both the same anchor, so batch 2's first sample was
// stamped before batch 1's last.
TEST(SensorTickClock, MonotoneAcrossFramesDeliveredTogether) {
  ins::SensorTickClock clock(sttagConfig());
  const rclcpp::Time same_arrival = at(1000.0);

  std::vector<rclcpp::Time> out;
  std::uint64_t tick = 500000;
  for (int frame = 0; frame < 2; ++frame) {
    for (int i = 0; i < 4; ++i) {
      out.push_back(clock.stamp(tick, same_arrival));
      tick += kTicksPer10ms;
    }
  }

  ASSERT_EQ(out.size(), 8u);
  for (size_t i = 1; i < out.size(); ++i) {
    EXPECT_GT(out[i].nanoseconds(), out[i - 1].nanoseconds())
        << "sample " << i << " is not after sample " << i - 1;
  }
}

// The test that separates this design from the naive one. A stall followed by
// a burst is how a latency floor actually collapses on a real link: samples
// pile up in the receiver/USB buffer (their latency grows), then the whole
// backlog is handed over in one read, so the newest of them has almost no
// latency at all. The sliding-window minimum therefore drops by the length of
// the stall in a single step.
//
// Note arrival is NON-DECREASING throughout, which it must be: the node calls
// now() in read order, so a real stream can never hand back an earlier arrival
// than the one before. (A genuinely backwards arrival means bag replay
// restarted, and is handled by re-seeding - see
// BackwardsArrivalStampRestartsTheWindow.)
//
// Taking the new minimum directly would move the timeline back by nearly the
// stall length. The slew limit caps |db| at max_slew * gap, so the output dt
// stays positive by construction.
TEST(SensorTickClock, SlewLimitSurvivesALatencyFloorDrop) {
  ins::SensorTickClock clock(sttagConfig());

  const std::int64_t ticks_per_5ms = 128;  // 200 Hz
  std::uint64_t tick = 10000;
  double t = 100.0;
  std::vector<rclcpp::Time> out;

  // Steady state: 60 ms transport latency, arrival tracking the counter.
  for (int i = 0; i < 200; ++i) {
    out.push_back(clock.stamp(tick, at(t + 0.060)));
    tick += ticks_per_5ms;
    t += 0.005;
  }

  // The link stalls for 200 ms; 40 samples accumulate and are then delivered
  // together at one arrival instant. The last of them has ~0 latency, so the
  // window minimum collapses by ~200 ms in one step.
  const double burst_arrival = t + 0.200;
  for (int i = 0; i < 40; ++i) {
    out.push_back(clock.stamp(tick, at(burst_arrival)));
    tick += ticks_per_5ms;
    t += 0.005;
  }

  // Back to steady state at the new, much lower floor.
  for (int i = 0; i < 200; ++i) {
    out.push_back(clock.stamp(tick, at(t + 0.005)));
    tick += ticks_per_5ms;
    t += 0.005;
  }

  for (size_t i = 1; i < out.size(); ++i) {
    EXPECT_GT(out[i].nanoseconds(), out[i - 1].nanoseconds())
        << "backwards stamp at sample " << i;
  }
}

// Mirror of GnssEpochAligner's TowAutoOffsetRecoversTheMinimumLatency: the
// estimator converges on the FLOOR of the jittery transport delay.
//
// The latency model is a sawtooth, which is what transport jitter physically
// looks like: the link stalls (latency jumps up), then the backlog drains at
// up to one sample interval per sample (latency decays). Decaying 7 ms per
// 10 ms sample keeps arrival increasing, as it must.
//
// What survives is exactly that floor. Removing the minimum latency as well
// would need an external reference (PPS, or the GNSS epoch anchor) - the same
// residual the aligner documents as "residual ~= the min latency". So the
// contract is: every sample lands at its true instant plus ONE constant, with
// the per-sample jitter gone.
TEST(SensorTickClock, RecoversTheMinimumLatency) {
  auto cfg = sttagConfig();
  cfg.warmup_slew = 0.5;
  ins::SensorTickClock clock(cfg);
  ArrivalSequence arrivals;

  const double floor_latency = 0.020;
  auto latency = [floor_latency](int i) {
    return floor_latency + 0.007 * (9 - (i % 10));
  };

  std::uint64_t tick = 7000;
  double t = 50.0;
  for (int i = 0; i < 600; ++i) {
    clock.stamp(tick, arrivals.next(t + latency(i)));
    tick += kTicksPer10ms;
    t += 0.010;
  }

  // Every subsequent sample maps to (true instant + floor), whatever latency
  // that individual sample happened to suffer. That constancy is the point:
  // the jitter is what corrupts dt, and it is gone.
  for (int i = 600; i < 610; ++i) {
    const rclcpp::Time mapped = clock.stamp(tick, arrivals.next(t + latency(i)));
    EXPECT_NEAR(mapped.nanoseconds() / 1e9, t + floor_latency, 1e-3)
        << "sample " << i << " with latency " << latency(i);
    tick += kTicksPer10ms;
    t += 0.010;
  }
}

// The actual claim of the fix: output spacing follows the COUNTER, not the
// arrival jitter it replaces.
TEST(SensorTickClock, SpacingTracksTheCounterNotTheArrival) {
  ins::SensorTickClock clock(sttagConfig());

  const double jitter[] = {0.000, 0.005, -0.003, 0.004, -0.002, 0.005};
  std::uint64_t tick = 900000;
  double t = 10.0;
  std::vector<rclcpp::Time> out;
  for (int i = 0; i < 300; ++i) {
    out.push_back(clock.stamp(tick, at(t + 0.050 + jitter[i % 6])));
    tick += kTicksPer10ms;
    t += 0.010;
  }

  // Skip the warm-up, then every dt must equal the counter spacing.
  const double expected_dt = kTicksPer10ms * kSttagTick;
  for (size_t i = 200; i < out.size(); ++i) {
    const double dt = (out[i].nanoseconds() - out[i - 1].nanoseconds()) / 1e9;
    EXPECT_NEAR(dt, expected_dt, 1e-5) << "at sample " << i;
  }
}

TEST(SensorTickClock, Uint32WrapIsTransparent) {
  ins::SensorTickClock clock(sttagConfig());

  std::uint64_t tick = 0xFFFFFF00ull;
  double t = 500.0;
  std::vector<rclcpp::Time> out;
  for (int i = 0; i < 20; ++i) {
    out.push_back(clock.stamp(tick & 0xFFFFFFFFull, at(t + 0.030)));
    tick += kTicksPer10ms;  // crosses 2^32 partway through
    t += 0.010;
  }

  for (size_t i = 1; i < out.size(); ++i) {
    const double dt = (out[i].nanoseconds() - out[i - 1].nanoseconds()) / 1e9;
    EXPECT_NEAR(dt, kTicksPer10ms * kSttagTick, 1e-5)
        << "wrap broke the spacing at sample " << i;
  }
}

// modulus must be honored for a non-power-of-two wrap too: an iTOW week
// rollover steps by -604800000, which FITS in int32_t and so would be read as
// a receiver reset by generic 32-bit unwrap logic.
TEST(SensorTickClock, ItowWeekRolloverIsTransparent) {
  ins::SensorTickClock::Config cfg;
  cfg.tick_s = 1e-3;
  cfg.modulus = 604800000ull;
  cfg.window_s = 20.0;
  ins::SensorTickClock clock(cfg);

  std::uint64_t itow = 604799800ull;  // 200 ms before the rollover
  double t = 900.0;
  std::vector<rclcpp::Time> out;
  for (int i = 0; i < 20; ++i) {
    out.push_back(clock.stamp(itow % 604800000ull, at(t + 0.030)));
    itow += 50;  // 50 ms steps, crossing the week boundary
    t += 0.050;
  }

  for (size_t i = 1; i < out.size(); ++i) {
    const double dt = (out[i].nanoseconds() - out[i - 1].nanoseconds()) / 1e9;
    EXPECT_NEAR(dt, 0.050, 1e-5)
        << "week rollover broke the spacing at sample " << i;
  }
}

TEST(SensorTickClock, ReceiverResetReSeedsAtArrival) {
  ins::SensorTickClock clock(sttagConfig());

  std::uint64_t tick = 800000;
  double t = 200.0;
  for (int i = 0; i < 50; ++i) {
    clock.stamp(tick, at(t + 0.040));
    tick += kTicksPer10ms;
    t += 0.010;
  }

  // Receiver restarts: sTtag returns to near zero.
  std::string warn;
  const rclcpp::Time after = clock.stamp(12, at(t + 0.040), &warn);
  EXPECT_FALSE(warn.empty()) << "a re-seed must be reported to the caller";
  EXPECT_NEAR(after.nanoseconds() / 1e9, t + 0.040, 1e-9)
      << "re-seed must land exactly on the arrival time";
}

TEST(SensorTickClock, BackwardsArrivalStampRestartsTheWindow) {
  ins::SensorTickClock clock(sttagConfig());

  std::uint64_t tick = 300000;
  double t = 700.0;
  for (int i = 0; i < 50; ++i) {
    clock.stamp(tick, at(t + 0.040));
    tick += kTicksPer10ms;
    t += 0.010;
  }

  // Bag replay restarts: arrival jumps backwards.
  std::string warn;
  const rclcpp::Time after = clock.stamp(tick, at(10.0), &warn);
  EXPECT_FALSE(warn.empty());
  EXPECT_NEAR(after.nanoseconds() / 1e9, 10.0, 1e-9)
      << "the persistent offset and unwrap state must both be discarded";
}

// Without the divergence guard a +1 s host clock step would take
// 1.0 / 2e-4 = 5000 s of receiver time to absorb.
TEST(SensorTickClock, HostClockStepTriggersTheDivergenceReset) {
  ins::SensorTickClock clock(sttagConfig());

  std::uint64_t tick = 400000;
  double t = 300.0;
  for (int i = 0; i < 100; ++i) {
    clock.stamp(tick, at(t + 0.030));
    tick += kTicksPer10ms;
    t += 0.010;
  }

  std::string warn;
  tick += kTicksPer10ms;
  t += 0.010;
  const rclcpp::Time after = clock.stamp(tick, at(t + 5.0), &warn);
  EXPECT_FALSE(warn.empty());
  EXPECT_NEAR(after.nanoseconds() / 1e9, t + 5.0, 1e-9);
}

// Offline reprocessing: a recorded UBX log replayed through file:// hands
// over 120 s of receiver time in a second or two, so the counter advances far
// faster than the host clock. Nothing is wrong - the counter is the clock
// worth following - but an accumulated-divergence guard would reset over and
// over. This is how that bug was found: a real file:// replay produced
// periodic -1 s jumps and a median dt 10 % short.
TEST(SensorTickClock, FastReplayDoesNotThrash) {
  ins::SensorTickClock clock(sttagConfig());
  ArrivalSequence arrivals;

  std::uint64_t tick = 1000;
  double counter_t = 0.0;   // receiver time: 10 ms per sample
  double wall_t = 900.0;    // host time: 80x faster replay
  std::vector<rclcpp::Time> out;
  std::string warn;
  int reseeds = 0;

  for (int i = 0; i < 4000; ++i) {
    out.push_back(clock.stamp(tick, arrivals.next(wall_t), &warn));
    if (!warn.empty()) ++reseeds;
    tick += kTicksPer10ms;
    counter_t += 0.010;
    wall_t += 0.010 / 80.0;
  }

  EXPECT_EQ(reseeds, 0) << "a sustained rate difference is not a fault";
  for (size_t i = 1; i < out.size(); ++i) {
    EXPECT_GT(out[i].nanoseconds(), out[i - 1].nanoseconds())
        << "backwards stamp at sample " << i;
  }
  // Once the window has filled the steady-state slew is 2e-4, so the output
  // spacing tracks the counter grid to better than 0.1 %.
  const double expected_dt = kTicksPer10ms * kSttagTick;
  for (size_t i = 3000; i < out.size(); ++i) {
    const double dt = (out[i].nanoseconds() - out[i - 1].nanoseconds()) / 1e9;
    EXPECT_NEAR(dt, expected_dt, expected_dt * 1e-3) << "at sample " << i;
  }
}

// warmup_slew is the worst-case relative dt distortion while converging. It
// must stay small enough that a downstream gap check never sees a surprise.
//
// The latency drains at 8 ms per 10 ms sample - just under the physical
// maximum, since draining at exactly the sample interval means arrival stands
// still (a burst) and draining faster is impossible.
TEST(SensorTickClock, WarmupDistortsDtByAtMostTheWarmupSlew) {
  auto cfg = sttagConfig();
  ins::SensorTickClock clock(cfg);
  ArrivalSequence arrivals;

  std::uint64_t tick = 2000;
  double t = 10.0;
  double latency = 0.200;  // seeded far above the floor: worst case for b
  std::vector<rclcpp::Time> out;
  for (int i = 0; i < 500; ++i) {
    out.push_back(clock.stamp(tick, arrivals.next(t + latency)));
    latency = std::max(0.010, latency - 0.008);
    tick += kTicksPer10ms;
    t += 0.010;
  }

  const double expected_dt = kTicksPer10ms * kSttagTick;
  for (size_t i = 1; i < out.size(); ++i) {
    const double dt = (out[i].nanoseconds() - out[i - 1].nanoseconds()) / 1e9;
    EXPECT_GT(dt, 0.0) << "at sample " << i;
    EXPECT_LE(std::fabs(dt - expected_dt), expected_dt * cfg.warmup_slew * 1.05)
        << "warm-up distorted dt beyond warmup_slew at sample " << i;
  }
}

// The no-added-latency contract: nothing is buffered to fill a window, so the
// very first stamp is what the legacy code would have produced.
TEST(SensorTickClock, FirstStampEqualsArrival) {
  ins::SensorTickClock clock(sttagConfig());
  const rclcpp::Time arrival = at(1234.567);
  EXPECT_EQ(clock.stamp(999, arrival).nanoseconds(), arrival.nanoseconds());
}

// Direct mirror of GnssEpochAligner's NegativeWindowIsSurvivable: window_s is
// an unvalidated parameter, and emptying the deque before reading front()
// would dereference nothing.
TEST(SensorTickClock, NonPositiveWindowIsSurvivable) {
  for (double w : {0.0, -1.0}) {
    auto cfg = sttagConfig();
    cfg.window_s = w;
    ins::SensorTickClock clock(cfg);
    std::uint64_t tick = 100;
    double t = 5.0;
    for (int i = 0; i < 20; ++i) {
      EXPECT_NO_FATAL_FAILURE(clock.stamp(tick, at(t + 0.020)));
      tick += kTicksPer10ms;
      t += 0.010;
    }
  }
}

TEST(SensorTickClock, ZeroTickIsPassthrough) {
  ins::SensorTickClock::Config cfg;  // tick_s defaults to 0
  ins::SensorTickClock clock(cfg);
  for (double t : {1.0, 2.0, 3.0}) {
    EXPECT_EQ(clock.stamp(42, at(t)).nanoseconds(), at(t).nanoseconds());
  }
}

// A wrong STTAG_TICK_S is invisible without this: the constant is not in the
// ZED-F9R Interface Description, so drift against the host is the only signal.
TEST(SensorTickClock, DriftRateDetectsAWrongTickConstant) {
  auto cfg = sttagConfig();
  ins::SensorTickClock clock(cfg);

  // The receiver's true tick is 1% larger than the configured nominal, so
  // real time advances faster than counter*tick and the offset must climb.
  const double true_tick = kSttagTick * 1.01;
  std::uint64_t tick = 1000;
  double t = 0.0;
  for (int i = 0; i < 4000; ++i) {
    clock.stamp(tick, at(t + 0.030));
    tick += kTicksPer10ms;
    t += kTicksPer10ms * true_tick;
  }
  EXPECT_GT(clock.driftRate(), 1e-3)
      << "a 1% tick error must show up as a large drift rate";
}

// Measured on a real ZED-F9R (HPS 1.40): the sTtag counter runs on the IMU's
// own oscillator, 38.7447 us per tick instead of the nominal 39.0625 us. With
// the tick taken as a constant, the stamps ran ahead of real time by 7.9 ms
// every second. The clock must measure the rate and stay on real time.
namespace {
constexpr double kF9rTrueTick = 38.7447e-6;

struct LiveRun {
  std::vector<double> stamp, truth;
};

// 100 Hz samples at the true tick; arrival = truth + 1 ms + jitter in [0, 4 ms)
// plus an optional backlog that drains (arrival never goes backwards).
LiveRun runLive(ins::SensorTickClock& clock, int samples, double backlog_s = 0.0,
                double drain_s = 1.0) {
  LiveRun run;
  ArrivalSequence arrivals;
  std::uint64_t tick = 7000;
  std::uint32_t lcg = 12345;
  double last_arrival = -1.0;
  for (int i = 0; i < samples; ++i) {
    const double truth = 500.0 + i * kTicksPer10ms * kF9rTrueTick;
    lcg = lcg * 1664525u + 1013904223u;
    const double jitter = 0.004 * (lcg >> 8) / double(1u << 24);
    const double backlog = std::max(0.0, backlog_s * (1.0 - (truth - 500.0) / drain_s));
    double arrival = truth + 0.001 + jitter + backlog;
    arrival = std::max(arrival, last_arrival);
    last_arrival = arrival;
    run.stamp.push_back(clock.stamp(tick, arrivals.next(arrival)).nanoseconds() * 1e-9);
    run.truth.push_back(truth);
    tick = (tick + kTicksPer10ms) & 0xffffffffu;
  }
  return run;
}

void expectOnRealTime(const ins::SensorTickClock& clock, const LiveRun& run, size_t from) {
  for (size_t i = 1; i < run.stamp.size(); ++i)
    ASSERT_GT(run.stamp[i], run.stamp[i - 1]) << "backwards stamp at sample " << i;
  double worst = 0.0;
  for (size_t i = from; i < run.stamp.size(); ++i)
    worst = std::max(worst, std::fabs(run.stamp[i] - (run.truth[i] + 0.001)));
  EXPECT_LT(worst, 0.003) << "stamps left real time by " << worst << " s";
  const size_t n = run.stamp.size() - 1;
  const double dt = (run.stamp[n] - run.stamp[from]) / double(n - from);
  EXPECT_NEAR(dt / (kTicksPer10ms * kF9rTrueTick), 1.0, 1e-4) << "sample spacing";
  EXPECT_EQ(clock.rateState(), ins::SensorTickClock::RateState::Tracking);
  EXPECT_NEAR(clock.appliedTick() / kF9rTrueTick, 1.0, 5e-5);
}
}  // namespace

TEST(SensorTickClock, FollowsAMeasuredTickOffTheNominalValue) {
  ins::SensorTickClock clock(sttagConfig());
  const auto run = runLive(clock, 60000);  // 10 min
  expectOnRealTime(clock, run, 12000);     // from 2 min on
}

// A driver connecting to a receiver that has been streaming meets a burst of
// buffered output first: its delayed arrivals must not bias the rate.
TEST(SensorTickClock, StartupBacklogDoesNotBiasTheRate) {
  ins::SensorTickClock clock(sttagConfig());
  const auto run = runLive(clock, 60000, /*backlog_s=*/0.4, /*drain_s=*/6.0);
  expectOnRealTime(clock, run, 12000);
}

TEST(SensorTickClock, ReplayIsReportedUnpacedAndKeepsTheNominalTick) {
  ins::SensorTickClock clock(sttagConfig());
  std::uint64_t tick = 1000;
  for (int i = 0; i < 6000; ++i) {
    clock.stamp(tick, at(900.0 + i * 0.010 / 80.0));
    tick += kTicksPer10ms;
  }
  EXPECT_EQ(clock.rateState(), ins::SensorTickClock::RateState::Unpaced);
  EXPECT_DOUBLE_EQ(clock.appliedTick(), kSttagTick);
}

// ---- GnssTimeAnchor ------------------------------------------------------
namespace {
using Anchor = ins::GnssTimeAnchor;
std::int64_t ns(double seconds) { return static_cast<std::int64_t>(std::llround(seconds * 1e9)); }

// 10 Hz epochs: latency = floor + jitter in [0, 10 ms).
struct LatencyGen {
  std::uint32_t lcg{4242};
  double next() {
    lcg = lcg * 1664525u + 1013904223u;
    return 0.050 + 0.010 * (lcg >> 8) / double(1u << 24);
  }
};
}  // namespace

// Measured on a ZED-F9R: one RXM-RAWX in 6000 arrived 23 ms early and a plain
// 60 s minimum then held the whole IMU timeline 20 ms off for 60 s.
TEST(GnssTimeAnchor, OneEarlyEpochDoesNotMoveTheOffset) {
  Anchor anchor;
  LatencyGen latency;
  std::deque<std::pair<double, double>> window;  // the previous estimator, for contrast
  double worst_anchor = 0.0, worst_window_min = 0.0;
  for (int i = 0; i < 3000; ++i) {
    const double gnss = 1000.0 + i * 0.1;
    double y = latency.next();
    if (i == 1500) y -= 0.023;
    anchor.update(Anchor::Source::Observation, ns(gnss + y), ns(gnss));
    window.emplace_back(gnss, y);
    while (gnss - window.front().first > 60.0) window.pop_front();
    if (i < 700) continue;
    double min = window.front().second;
    for (const auto& w : window) min = std::min(min, w.second);
    worst_anchor = std::max(worst_anchor, std::fabs(anchor.offset() - 0.050));
    worst_window_min = std::max(worst_window_min, std::fabs(min - 0.050));
  }
  EXPECT_LT(worst_anchor, 0.002);
  EXPECT_GT(worst_window_min, 0.015) << "scenario must reproduce the defect";
}

// RXM-RAWX is output about 30 ms earlier in an epoch without a navigation
// solution. Those epochs must not pull the offset, whatever their share.
TEST(GnssTimeAnchor, EpochsWithoutNavigationSolutionDoNotMoveTheOffset) {
  Anchor anchor;
  for (int i = 0; i < 1200; ++i) {
    const double gnss = 100.0 + i * 0.1;
    const bool with_nav = i % 2 == 0;
    anchor.update(Anchor::Source::Observation, ns(gnss + (with_nav ? 0.060 : 0.030)), ns(gnss));
    if (with_nav) anchor.update(Anchor::Source::Solution, ns(gnss + 0.100), ns(gnss));
  }
  EXPECT_NEAR(anchor.offset(), 0.060, 1e-6) << "no-NAV epochs must be excluded";
}

TEST(GnssTimeAnchor, ObservationWaitsForItsSolution) {
  Anchor anchor;
  anchor.update(Anchor::Source::Observation, ns(100.05), ns(100.0));
  anchor.update(Anchor::Source::Solution, ns(100.09), ns(100.007));
  ASSERT_TRUE(anchor.valid());
  EXPECT_NEAR(anchor.offset(), 0.05, 1e-6);

  // Pending until the solution of the same epoch: its arrival time is the one used.
  anchor.update(Anchor::Source::Observation, ns(100.17), ns(100.1));
  EXPECT_NEAR(anchor.offset(), 0.05, 1e-6);
  anchor.update(Anchor::Source::Solution, ns(100.25), ns(100.107));
  EXPECT_NEAR(anchor.offset(), 0.07, 1e-6);  // the first solution restarted the window

  // A solution of another epoch does not release a pending observation.
  anchor.update(Anchor::Source::Observation, ns(100.31), ns(100.3));
  anchor.update(Anchor::Source::Solution, ns(100.45), ns(100.207));
  EXPECT_NEAR(anchor.offset(), 0.07, 1e-6);
}

TEST(GnssTimeAnchor, SolutionBeforeItsObservationIsAlsoMatched) {
  Anchor anchor;
  anchor.update(Anchor::Source::Observation, ns(100.05), ns(100.0));
  anchor.update(Anchor::Source::Solution, ns(100.09), ns(100.0));
  anchor.update(Anchor::Source::Solution, ns(102.09), ns(102.0));
  anchor.update(Anchor::Source::Observation, ns(102.15), ns(102.007));
  EXPECT_GT(anchor.offset(), 0.05);  // the 0.143 s sample entered the median
}

TEST(GnssTimeAnchor, EitherMessageAloneStillAnchors) {
  Anchor observations;
  observations.update(Anchor::Source::Observation, ns(10.03), ns(10.0));
  EXPECT_NEAR(observations.offset(), 0.03, 1e-6);

  Anchor solutions;
  EXPECT_FALSE(solutions.valid());
  solutions.update(Anchor::Source::Solution, ns(20.1), ns(20.0));
  EXPECT_NEAR(solutions.offset(), 0.1, 1e-6);
  // The first observation takes over once its solution confirms the epoch.
  solutions.update(Anchor::Source::Observation, ns(20.13), ns(20.1));
  EXPECT_NEAR(solutions.offset(), 0.1, 1e-6);
  solutions.update(Anchor::Source::Solution, ns(20.2), ns(20.1));
  EXPECT_NEAR(solutions.offset(), 0.03, 1e-6);
}

// file:// replay hands over GNSS epochs 80x faster than the host clock, so y
// falls continuously. The offset stays with it, without steps the nodes would
// treat as a discontinuity (> 1 s).
TEST(GnssTimeAnchor, FasterThanRealTimeReplayStaysWithTheStream) {
  Anchor anchor;
  double previous = 0.0;
  for (int i = 0; i < 20000; ++i) {
    const double gnss = 5000.0 + i * 0.1;
    const double arrival = 900.0 + i * 0.1 / 80.0;
    anchor.update(Anchor::Source::Observation, ns(arrival), ns(gnss));
    const double y = arrival - gnss;
    EXPECT_LT(std::fabs(anchor.offset() - y), 0.7) << "epoch " << i;
    if (i) { EXPECT_LT(std::fabs(anchor.offset() - previous), 1.0) << "epoch " << i; }
    previous = anchor.offset();
  }
}

TEST(GnssTimeAnchor, HostClockStepDownIsFollowedAndAStallIsIgnored) {
  Anchor anchor;
  double gnss = 2000.0;
  for (int i = 0; i < 600; ++i, gnss += 0.1) anchor.update(Anchor::Source::Observation, ns(gnss + 0.05), ns(gnss));
  // A stalled delivery raises one sample by 0.8 s: not a clock step.
  anchor.update(Anchor::Source::Observation, ns(gnss + 0.85), ns(gnss));
  EXPECT_NEAR(anchor.offset(), 0.05, 1e-6);
  gnss += 0.1;
  // The host clock steps back 2 s.
  anchor.update(Anchor::Source::Observation, ns(gnss - 1.95), ns(gnss));
  EXPECT_NEAR(anchor.offset(), -1.95, 1e-6);
}

TEST(GnssTimeAnchor, ResetForgetsTheObservationReference) {
  Anchor anchor;
  anchor.update(Anchor::Source::Observation, ns(10.03), ns(10.0));
  anchor.reset();
  EXPECT_FALSE(anchor.valid());
  anchor.update(Anchor::Source::Solution, ns(20.1), ns(20.0));
  EXPECT_NEAR(anchor.offset(), 0.1, 1e-6);
}

// ---- StreamPacing --------------------------------------------------------
using Pacing = ins::StreamPacing;

TEST(StreamPacing, FastLogIsReplayAndRealTimeIsLive) {
  Pacing replay;
  std::uint32_t c = 1000;
  for (int i = 0; i < 400; ++i, c += kTicksPer10ms) replay.update(c, 50.0 + i * 0.010 / 80.0);
  EXPECT_EQ(replay.state(), Pacing::State::Replay);

  Pacing live;
  c = 1000;
  for (int i = 0; i < 400; ++i, c += kTicksPer10ms) live.update(c, 50.0 + i * 0.010);
  EXPECT_EQ(live.state(), Pacing::State::Live);
}

// A connection meets the receiver's buffered output first: a burst of buffered
// samples is not a replay.
TEST(StreamPacing, StartupBurstIsNotAReplay) {
  Pacing pacing;
  std::uint32_t c = 1000;
  double arrival = 50.0;
  for (int i = 0; i < 400; ++i, c += kTicksPer10ms) {
    pacing.update(c, arrival);
    arrival += i < 50 ? 0.0005 : 0.010;  // 0.5 s of buffered samples at once
  }
  EXPECT_EQ(pacing.state(), Pacing::State::Live);
}

TEST(StreamPacing, CounterRestartBeginsANewMeasurementAndDecisionIsLatched) {
  Pacing pacing;
  std::uint32_t c = 1000;
  for (int i = 0; i < 100; ++i, c += kTicksPer10ms) pacing.update(c, 50.0 + i * 0.010 / 80.0);
  EXPECT_EQ(pacing.state(), Pacing::State::Unknown);
  c = 500;  // receiver restarted its counter
  for (int i = 0; i < 400; ++i, c += kTicksPer10ms) pacing.update(c, 60.0 + i * 0.010);
  EXPECT_EQ(pacing.state(), Pacing::State::Live);
  // Latched: later delivery changes do not switch the timeline.
  for (int i = 0; i < 400; ++i, c += kTicksPer10ms) pacing.update(c, 70.0);
  EXPECT_EQ(pacing.state(), Pacing::State::Live);
}

struct FakeSample { std::uint32_t sttag; };

TEST(StampBatch, ReturnsOneStampPerSampleInOrder) {
  ins::SensorTickClock clock(sttagConfig());
  std::vector<FakeSample> samples;
  for (int i = 0; i < 4; ++i) {
    samples.push_back({static_cast<std::uint32_t>(5000 + i * kTicksPer10ms)});
  }

  const auto out = ins::stampBatch(
      clock, samples, [](const FakeSample& s) { return s.sttag; }, at(77.0));

  ASSERT_EQ(out.size(), samples.size());
  for (size_t i = 1; i < out.size(); ++i) {
    EXPECT_GT(out[i].nanoseconds(), out[i - 1].nanoseconds());
  }
}

TEST(StampBatch, EmptyBatchIsSafe) {
  ins::SensorTickClock clock(sttagConfig());
  const std::vector<FakeSample> none;
  EXPECT_TRUE(ins::stampBatch(
      clock, none, [](const FakeSample& s) { return s.sttag; }, at(1.0))
                  .empty());
}

TEST(SensorTickClock, RejectedBackstepDoesNotLengthenNextInterval) {
  gnss_ros_standardization::ins_utils::SensorTickClock::Config cfg;
  cfg.tick_s=39.0625e-6; cfg.max_slew=0; cfg.warmup_slew=0;
  gnss_ros_standardization::ins_utils::SensorTickClock clock(cfg);
  clock.stamp(1000,rclcpp::Time(100,0,RCL_ROS_TIME));
  auto before=clock.stamp(1256,rclcpp::Time(100,10000000,RCL_ROS_TIME));
  clock.stamp(1200,rclcpp::Time(100,11000000,RCL_ROS_TIME));
  auto after=clock.stamp(1512,rclcpp::Time(100,20000000,RCL_ROS_TIME));
  EXPECT_NEAR((after-before).seconds(),0.01,1e-8);
}

TEST(SensorTickClock, Simulated25HourInputRemainsBoundedAndMonotonic) {
  using gnss_ros_standardization::ins_utils::SensorTickClock;
  SensorTickClock::Config cfg;cfg.tick_s=1e-3;cfg.window_s=2;
  SensorTickClock clock(cfg);
  int64_t previous=-1;
  const uint64_t start=(1ull<<32)-1000;
  for(uint64_t i=0;i<900000;++i){
    const auto out=clock.stamp((start+i*100)&0xffffffffu,
        rclcpp::Time(1700000000000000000LL+int64_t(i)*100000000LL,RCL_ROS_TIME));
    ASSERT_GT(out.nanoseconds(),previous);previous=out.nanoseconds();
  }
}
TEST(SensorTickClock, AnchorRefinementDoesNotInvertTime) {
  gnss_ros_standardization::ins_utils::SlewedTimeOffset offset;
  const auto first=offset.map(at(100),10.0);
  const auto second=offset.map(at(100.01),10.5);
  EXPECT_GT(second.nanoseconds(),first.nanoseconds());
  EXPECT_NEAR((second.nanoseconds()-first.nanoseconds())*1e-9,0.0098,1e-6);
  offset.reset();
  EXPECT_NEAR(offset.map(at(10),1).seconds(),9,1e-9);
}
