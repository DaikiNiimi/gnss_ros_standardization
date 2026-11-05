#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "gnss_ros_standardization/epoch_matcher.hpp"

namespace {

using Matcher = gnss_utils::RoverBaseEpochMatcher;
using ObsMsg = Matcher::ObsMsg;

ObsMsg::SharedPtr epoch(std::uint32_t week, double tow) {
  auto msg = std::make_shared<ObsMsg>();
  msg->week = week;
  msg->tow = tow;
  return msg;
}

TEST(ReleaseN2, WeekZeroIsInvalidOnBothStreams) {
  Matcher matcher;
  matcher.pushRover(epoch(0, 100));
  matcher.pushBase(epoch(0, 100));
  EXPECT_TRUE(matcher.flush().empty());
  for (auto stream : {Matcher::Stream::Rover, Matcher::Stream::Base})
    EXPECT_EQ(matcher.counters().dropped(stream, Matcher::DropReason::InvalidTimestamp), 1U);
}

TEST(EpochMatcher, RejectsInvalidConfiguration) {
  Matcher::Options opt;
  opt.queue_limit = 0;
  EXPECT_THROW((void)Matcher(opt), std::invalid_argument);

  opt = Matcher::Options{};
  opt.match_tol_s = -1.0;
  EXPECT_THROW((void)Matcher(opt), std::invalid_argument);

  opt = Matcher::Options{};
  opt.duplicate_tol_s = opt.match_tol_s * 2.0;
  EXPECT_THROW((void)Matcher(opt), std::invalid_argument);
}

TEST(EpochMatcher, SortsBothStreamsAndDeduplicates) {
  Matcher matcher;
  matcher.pushRover(epoch(2200, 100.4));
  matcher.pushRover(epoch(2200, 100.0));
  matcher.pushRover(epoch(2200, 100.2));
  matcher.pushRover(epoch(2200, 100.2));  // duplicate
  matcher.pushBase(epoch(2200, 101.0));
  matcher.pushBase(epoch(2200, 100.0));   // out of callback order

  const auto pairs = matcher.flush();
  ASSERT_EQ(pairs.size(), 3U);
  EXPECT_DOUBLE_EQ(pairs[0].rover->tow, 100.0);
  EXPECT_DOUBLE_EQ(pairs[1].rover->tow, 100.2);
  EXPECT_DOUBLE_EQ(pairs[2].rover->tow, 100.4);
  EXPECT_EQ(pairs[0].kind, Matcher::PairKind::Exact);
  EXPECT_EQ(pairs[1].kind, Matcher::PairKind::Aged);
  EXPECT_EQ(pairs[2].kind, Matcher::PairKind::Aged);
  EXPECT_DOUBLE_EQ(pairs[1].base->tow, 100.0);
  EXPECT_NEAR(pairs[1].age_s, 0.2, 1e-9);
  EXPECT_EQ(matcher.counters().dropped(
                Matcher::Stream::Rover, Matcher::DropReason::Duplicate),
            1U);
}

TEST(EpochMatcher, ExactPairIsDecidedAtOnceAndAnOlderLateRoverIsCounted) {
  Matcher::Options opt;
  opt.reorder_window_s = 0.05;
  Matcher matcher(opt);

  // Simulate the real callback pattern: every push is followed by drain.
  matcher.pushRover(epoch(2200, 100.04));
  EXPECT_TRUE(matcher.drainMatches().empty());
  matcher.pushBase(epoch(2200, 100.04));
  const auto pairs = matcher.drainMatches();
  ASSERT_EQ(pairs.size(), 1U) << "nothing better than an exact base can arrive";
  EXPECT_EQ(pairs[0].kind, Matcher::PairKind::Exact);

  // A rover epoch older than one already decided cannot be published in order.
  matcher.pushRover(epoch(2200, 100.00));
  EXPECT_TRUE(matcher.drainMatches().empty());
  EXPECT_EQ(matcher.counters().dropped(
                Matcher::Stream::Rover, Matcher::DropReason::LateArrival),
            1U);
}

TEST(EpochMatcher, AWaitingEpochHoldsEveryLaterOne) {
  Matcher matcher;
  for (double t : {95.0, 96.0, 97.0, 98.0, 99.0}) matcher.pushBase(epoch(2200, t));
  matcher.pushRover(epoch(2200, 100.0));   // base(100) is due: waits
  matcher.pushBase(epoch(2200, 101.0));    // exact for the NEXT epoch
  EXPECT_TRUE(matcher.drainMatches().empty());
  matcher.pushRover(epoch(2200, 101.0));
  const auto pairs = matcher.drainMatches();
  ASSERT_EQ(pairs.size(), 2U);
  EXPECT_DOUBLE_EQ(pairs[0].rover->tow, 100.0);
  EXPECT_EQ(pairs[0].kind, Matcher::PairKind::Aged);
  EXPECT_DOUBLE_EQ(pairs[1].rover->tow, 101.0);
  EXPECT_EQ(pairs[1].kind, Matcher::PairKind::Exact);
}

TEST(EpochMatcher, BaseCadenceDecidesAnAgedNewestEpochAtOnce) {
  // A 5 Hz rover between 1 Hz base epochs: the next base is due after the
  // rover epoch, so no closer base can come and nothing needs to wait.
  Matcher matcher;
  matcher.pushBase(epoch(2200, 99.0));
  matcher.pushRover(epoch(2200, 99.4));
  EXPECT_TRUE(matcher.drainMatches().empty()) << "one base: cadence unknown";
  matcher.pushBase(epoch(2200, 100.0));
  auto pairs = matcher.drainMatches();
  ASSERT_EQ(pairs.size(), 1U) << "cadence 1 s: base(99.4) cannot exist";
  EXPECT_DOUBLE_EQ(pairs[0].base->tow, 99.0);
  matcher.pushRover(epoch(2200, 100.2));
  pairs = matcher.drainMatches();
  ASSERT_EQ(pairs.size(), 1U);
  EXPECT_EQ(pairs[0].kind, Matcher::PairKind::Aged);
  EXPECT_DOUBLE_EQ(pairs[0].base->tow, 100.0);
  matcher.pushRover(epoch(2200, 101.0));   // on the base grid: base(101) due
  EXPECT_TRUE(matcher.drainMatches().empty());
  matcher.pushBase(epoch(2200, 101.0));
  pairs = matcher.drainMatches();
  ASSERT_EQ(pairs.size(), 1U);
  EXPECT_EQ(pairs[0].kind, Matcher::PairKind::Exact);
}

TEST(EpochMatcher, UsesRobustMedianBaseCadence) {
  Matcher matcher;
  matcher.pushBase(epoch(2200, 110.0));
  matcher.pushBase(epoch(2200, 101.0));
  matcher.pushBase(epoch(2200, 100.0));
  matcher.pushBase(epoch(2200, 102.0));

  // Sorted adjacent intervals are 1, 1, 8 seconds. A minimum happened to work
  // for this data but was unstable to duplicates; the estimator is now median.
  EXPECT_NEAR(matcher.estimatedBaseInterval(), 1.0, 1e-12);
}

TEST(EpochMatcher, MatchesAcrossGpsWeekRollover) {
  Matcher matcher;
  matcher.pushBase(epoch(2200, 604799.8));
  matcher.pushRover(epoch(2201, 0.1));

  const auto pairs = matcher.flush();
  ASSERT_EQ(pairs.size(), 1U);
  ASSERT_TRUE(pairs[0].hasBase());
  EXPECT_EQ(pairs[0].base->week, 2200U);
  EXPECT_NEAR(pairs[0].age_s, 0.3, 1e-9);
}

TEST(EpochMatcher, FlushFinalizesNewestEpochAtEndOfStream) {
  Matcher matcher;
  matcher.pushBase(epoch(2200, 100.0));
  matcher.pushRover(epoch(2200, 100.2));

  EXPECT_TRUE(matcher.drainMatches().empty());
  EXPECT_EQ(matcher.pendingRoverCount(), 1U);

  const auto pairs = matcher.flush();
  ASSERT_EQ(pairs.size(), 1U);
  EXPECT_EQ(pairs[0].kind, Matcher::PairKind::Aged);
  EXPECT_NEAR(pairs[0].age_s, 0.2, 1e-9);
  EXPECT_EQ(matcher.pendingRoverCount(), 0U);
  EXPECT_EQ(matcher.pendingBaseCount(), 0U);
}

TEST(EpochMatcher, EmitsEveryRoverEpochDuringBaseOutageWhenEnabled) {
  Matcher::Options opt;
  opt.emit_unmatched_rover = true;
  opt.drop_ahead_s = 1.0;
  Matcher matcher(opt);

  matcher.pushRover(epoch(2200, 100.0));
  matcher.pushRover(epoch(2200, 101.0));
  matcher.pushRover(epoch(2200, 102.0));

  auto first = matcher.drainMatches();
  ASSERT_EQ(first.size(), 2U);
  for (const auto& p : first) {
    EXPECT_FALSE(p.hasBase());
    EXPECT_EQ(p.kind, Matcher::PairKind::Unmatched);
    EXPECT_TRUE(std::isnan(p.age_s));
  }
  auto last = matcher.flush();
  ASSERT_EQ(last.size(), 1U);
  EXPECT_EQ(last[0].kind, Matcher::PairKind::Unmatched);
  EXPECT_EQ(matcher.counters().emitted_unmatched, 3U);
  EXPECT_TRUE(matcher.takeDropped().empty());
}

TEST(EpochMatcher, ReportsBaseOutageDropsByReasonWhenUnmatchedDisabled) {
  Matcher::Options opt;
  opt.drop_ahead_s = 1.0;
  Matcher matcher(opt);
  matcher.pushRover(epoch(2200, 100.0));
  matcher.pushRover(epoch(2200, 101.0));
  matcher.pushRover(epoch(2200, 102.0));

  EXPECT_TRUE(matcher.drainMatches().empty());
  EXPECT_TRUE(matcher.flush().empty());
  EXPECT_EQ(matcher.counters().dropped(
                Matcher::Stream::Rover,
                Matcher::DropReason::NoBaseWithinWindow),
            2U);
  EXPECT_EQ(matcher.counters().dropped(
                Matcher::Stream::Rover,
                Matcher::DropReason::EndOfStreamNoBase),
            1U);
}

TEST(EpochMatcher, QueueOverflowAndLateInputsAreNeverSilent) {
  Matcher::Options opt;
  // P1-1: exercise deferred pre-navigation storage explicitly. Live intake
  // decides on push; later base arrivals must not revise released pairs.
  opt.decide_on_push = false;
  opt.queue_limit = 2;
  Matcher matcher(opt);

  matcher.pushRover(epoch(2200, 100.0));
  matcher.pushRover(epoch(2200, 101.0));
  matcher.pushRover(epoch(2200, 102.0));  // evicts rover 100
  matcher.pushRover(epoch(2200, 99.0));   // older than terminal epoch
  matcher.pushRover(epoch(2200, 102.0));  // queued duplicate

  // The base queue is sized from the observed cadence (drop_ahead_s 2 s at
  // 1 Hz needs 4 entries), not only queue_limit. Overflow evicts the front;
  // the newest base is never the victim.
  for (int i = 0; i <= 4; ++i) matcher.pushBase(epoch(2200, 200.0 + i));  // evicts 200
  matcher.pushBase(epoch(2200, 201.0));   // still queued: duplicate
  matcher.pushBase(epoch(2200, 200.0));   // behind the evicted front: late

  EXPECT_EQ(matcher.counters().dropped(
                Matcher::Stream::Rover, Matcher::DropReason::QueueOverflow),
            1U);
  EXPECT_EQ(matcher.counters().dropped(
                Matcher::Stream::Rover, Matcher::DropReason::LateArrival),
            1U);
  EXPECT_EQ(matcher.counters().dropped(
                Matcher::Stream::Rover, Matcher::DropReason::Duplicate),
            1U);
  EXPECT_EQ(matcher.counters().dropped(
                Matcher::Stream::Base, Matcher::DropReason::QueueOverflow),
            1U);
  EXPECT_EQ(matcher.counters().dropped(
                Matcher::Stream::Base, Matcher::DropReason::Duplicate),
            1U);
  EXPECT_EQ(matcher.counters().dropped(
                Matcher::Stream::Base, Matcher::DropReason::LateArrival),
            1U);

  const auto events = matcher.takeDropEvents();
  EXPECT_EQ(events.size(), 6U);
  EXPECT_TRUE(matcher.takeDropEvents().empty());
  EXPECT_EQ(matcher.pendingRoverCount(), 2U);
  EXPECT_EQ(matcher.pendingBaseCount(), 4U);
}

TEST(EpochMatcher, RejectsNonCanonicalOrNonFiniteTow) {
  Matcher matcher;
  matcher.pushRover(epoch(2200, -0.1));
  matcher.pushRover(epoch(2200, 604800.0));
  matcher.pushBase(
      epoch(2200, std::numeric_limits<double>::quiet_NaN()));

  EXPECT_EQ(matcher.counters().dropped(
                Matcher::Stream::Rover,
                Matcher::DropReason::InvalidTimestamp),
            2U);
  EXPECT_EQ(matcher.counters().dropped(
                Matcher::Stream::Base,
                Matcher::DropReason::InvalidTimestamp),
            1U);
  EXPECT_EQ(matcher.pendingRoverCount(), 0U);
  EXPECT_EQ(matcher.pendingBaseCount(), 0U);
}

TEST(EpochMatcher, ConsumedBaseHistoryDoesNotCountAsQueueLoss) {
  Matcher::Options opt;
  opt.reorder_window_s = 0.0;
  opt.queue_limit = 50;
  opt.max_tdiff_s = 30.0;
  Matcher matcher(opt);

  for (int i = 0; i < 100; ++i) {
    matcher.pushBase(epoch(2200, 100.0 + i));
    matcher.pushRover(epoch(2200, 100.0 + i));
    const auto pairs = matcher.drainMatches();
    ASSERT_EQ(pairs.size(), 1U);
  }

  EXPECT_EQ(matcher.counters().dropped(
                Matcher::Stream::Base, Matcher::DropReason::QueueOverflow),
            0U);
  EXPECT_LE(matcher.pendingBaseCount(), 31U);
}

TEST(EpochMatcher, DiagnosticQueuesAreBoundedButCountersRemainLossless) {
  Matcher::Options opt;
  opt.queue_limit = 1;
  Matcher matcher(opt);
  for (int i = 0; i < 100; ++i) matcher.pushBase(epoch(2200, -1.0));

  EXPECT_EQ(matcher.counters().dropped(
                Matcher::Stream::Base,
                Matcher::DropReason::InvalidTimestamp),
            100U);
  EXPECT_EQ(matcher.counters().drop_events_overwritten, 36U);
  EXPECT_EQ(matcher.takeDropEvents().size(), 64U);
  EXPECT_STREQ(Matcher::dropReasonName(Matcher::DropReason::QueueOverflow),
               "queue_overflow");
  EXPECT_STREQ(Matcher::streamName(Matcher::Stream::Base), "base");
}

TEST(EpochMatcher, OldBaseBeyondMaxDifferenceDoesNotMasqueradeAsPair) {
  Matcher::Options opt;
  opt.max_tdiff_s = 1.0;
  opt.emit_unmatched_rover = true;
  Matcher matcher(opt);
  matcher.pushBase(epoch(2200, 100.0));
  matcher.pushRover(epoch(2200, 102.0));

  const auto pairs = matcher.flush();
  ASSERT_EQ(pairs.size(), 1U);
  EXPECT_EQ(pairs[0].kind, Matcher::PairKind::Unmatched);
  EXPECT_FALSE(pairs[0].hasBase());
  EXPECT_TRUE(std::isnan(pairs[0].age_s));
}

// A STARVED-DELIVERY pattern: the base subscription is not serviced for a
// while, then its whole backlog arrives at once. Worth pinning in its own
// right - a consumer really can be starved this way - but note what it is NOT.
//
// This is not the mechanism behind the base-epoch loss that was actually
// observed on a 5 Hz rover course. Instrumenting the node showed the base
// stream arriving exactly on time (wall-vs-tow slope 0.9998, callback lock wait
// 1e-5 s) while the ROVER processing lagged, which fills the queue with base
// AHEAD of the rover rather than behind it. That case is covered by
// OverflowKeepsTheBaseTheLaggingRoverStillNeeds, and unlike this one it IS
// sensitive to the eviction policy. Do not use the result below to argue about
// queue sizing for a lagging consumer - the two patterns behave oppositely.
struct DeliveryResult {
  std::uint64_t unmatched{0};
  std::uint64_t base_overflow{0};
  std::uint64_t exact_pairs{0};
};

// rover at 5 Hz, base at 1 Hz over `span_s`. The base subscription is serviced
// only every `service_period_s`, and then drains its whole backlog back to back
// before the next drainMatches - the shape a starved callback actually has.
// service_period_s <= the base interval means "serviced promptly".
DeliveryResult runDelivery(double span_s, double service_period_s,
                           std::size_t queue_limit) {
  Matcher::Options opt;
  opt.emit_unmatched_rover = true;  // as configured by GnssPreprocessor
  opt.queue_limit = queue_limit;
  Matcher matcher(opt);
  const std::uint32_t week = 2200;
  const double t0 = 100.0;
  std::vector<double> base_pending;
  for (double t = t0; t < t0 + span_s; t += 1.0) base_pending.push_back(t);
  std::size_t next_base = 0;
  double next_service = t0;  // the first service happens with the first rover
  DeliveryResult r;

  for (double rt = t0; rt < t0 + span_s; rt += 0.2) {
    if (rt + 1e-9 >= next_service) {
      while (next_base < base_pending.size() &&
             base_pending[next_base] <= rt + 1e-9) {
        matcher.pushBase(epoch(week, base_pending[next_base]));
        ++next_base;
      }
      next_service += service_period_s;
    }
    matcher.pushRover(epoch(week, rt));
    for (const auto& p : matcher.drainMatches()) {
      if (p.kind == Matcher::PairKind::Exact) ++r.exact_pairs;
    }
  }
  for (const auto& p : matcher.flush()) {
    if (p.kind == Matcher::PairKind::Exact) ++r.exact_pairs;
  }
  r.unmatched = matcher.counters().emitted_unmatched;
  r.base_overflow = matcher.counters().dropped(
      Matcher::Stream::Base, Matcher::DropReason::QueueOverflow);
  return r;
}

TEST(EpochMatcher, PromptBaseDeliveryLosesNothing) {
  const DeliveryResult r = runDelivery(300.0, 1.0, 50);
  EXPECT_EQ(r.unmatched, 0U);
  EXPECT_EQ(r.base_overflow, 0U);
  EXPECT_EQ(r.exact_pairs, 300U);  // one exact pair per base epoch
}

TEST(EpochMatcher, StarvedBaseCallbackCostsRoverEpochs) {
  // 50 s between services, past max_tdiff_s (30 s): the backlog arrives too old
  // to pair, and the rover epochs it should have served are emitted bare.
  const DeliveryResult r = runDelivery(300.0, 50.0, 50);
  EXPECT_GT(r.unmatched, 500U);
  EXPECT_LT(r.exact_pairs, 20U);
}

TEST(EpochMatcher, EnlargingTheQueueDoesNotRecoverStarvedEpochsWhenBaseIsLate) {
  // For THIS pattern (base genuinely late), a queue 12x larger only makes the
  // queue_overflow warnings disappear and recovers nothing, because an epoch
  // past max_tdiff_s is unusable however the queue is bounded. The opposite
  // holds when base is early and the rover is late - see the eviction-policy
  // tests below.
  for (const double service_s : {10.0, 30.0, 50.0, 60.0}) {
    const DeliveryResult small = runDelivery(300.0, service_s, 50);
    const DeliveryResult large = runDelivery(300.0, service_s, 600);
    EXPECT_EQ(small.unmatched, large.unmatched) << "service_s=" << service_s;
    EXPECT_EQ(small.exact_pairs, large.exact_pairs) << "service_s=" << service_s;
    EXPECT_EQ(large.base_overflow, 0U) << "service_s=" << service_s;
  }
}

}  // namespace

// AUDIT A1-6, the actual mechanism. Instrumenting the node showed the base
// stream arriving exactly on time (wall-vs-tow slope 0.9998, callback lock wait
// 1e-5 s) while pendingBaseCount climbed to the queue limit and pinned there.
// A consumer that cannot keep up works on rover epochs behind the wall clock,
// so the queue fills with base AHEAD of the rover - and the old policy evicted
// the FRONT, which is the epoch that lagging rover is about to need.
TEST(EpochMatcher, OverflowKeepsTheBaseTheLaggingRoverStillNeeds) {
  Matcher::Options opt;
  opt.emit_unmatched_rover = true;
  opt.queue_limit = 8;          // small, so the situation is reachable quickly
  opt.max_tdiff_s = 30.0;
  Matcher matcher(opt);
  const std::uint32_t week = 2200;

  // The rover waits at 100.5 with only base 100.0; base keeps arriving.
  matcher.pushBase(epoch(week, 100.0));
  matcher.pushRover(epoch(week, 100.5));
  (void)matcher.drainMatches();
  for (int i = 1; i <= 20; ++i) matcher.pushBase(epoch(week, 100.0 + i));

  // Overflow happened, and it did NOT throw away base 100.0.
  EXPECT_GT(matcher.counters().dropped(Matcher::Stream::Base,
                                       Matcher::DropReason::QueueOverflow),
            0U);
  const auto pairs = matcher.flush();
  ASSERT_EQ(pairs.size(), 1U);
  EXPECT_EQ(pairs[0].kind, Matcher::PairKind::Aged)
      << "the base epoch the waiting rover needs was evicted";
  ASSERT_TRUE(pairs[0].hasBase());
  EXPECT_DOUBLE_EQ(pairs[0].base->tow, 100.0);
}

TEST(EpochMatcher, TheBaseStreamSettlesTheLastRoverBeforeAnOutage) {
  // During a rover outage the last rover epoch IS the newest, so the
  // rover-only reorder rule never settles a waiting (aged) epoch and it would
  // be held for the whole outage. The base stream keeps arriving and proves
  // time has moved on.
  Matcher::Options opt;
  opt.emit_unmatched_rover = true;
  opt.drop_ahead_s = 2.0;
  Matcher matcher(opt);
  const std::uint32_t week = 2200;

  for (double t : {97.0, 98.0, 99.0}) matcher.pushBase(epoch(week, t));
  matcher.pushRover(epoch(week, 100.0));
  EXPECT_TRUE(matcher.drainMatches().empty()) << "base(100) is due: waits";

  // Base(100) is lost; the base stream goes on, the rover stream has stopped.
  matcher.pushBase(epoch(week, 101.0));
  EXPECT_TRUE(matcher.drainMatches().empty()) << "still inside drop_ahead_s";
  matcher.pushBase(epoch(week, 102.0));
  const auto pairs = matcher.drainMatches();
  ASSERT_EQ(pairs.size(), 1U) << "held for the whole outage";
  EXPECT_DOUBLE_EQ(pairs[0].rover->tow, 100.0);
  EXPECT_EQ(pairs[0].kind, Matcher::PairKind::Aged);
  EXPECT_DOUBLE_EQ(pairs[0].base->tow, 99.0);
}

TEST(EpochMatcher, ADeepRoverQueueDoesNotPruneTheBaseItStillNeeds) {
  // A backlogged node holds many rover epochs at once. Pruning base relative to
  // the NEWEST queued rover then discards the base the queued rovers are about
  // to be matched against, and it gets worse the deeper the queue - invisible
  // while the node keeps up, severe once it does not.
  Matcher::Options opt;
  // P1-1: exercise deferred pre-navigation storage explicitly. Live intake
  // decides on push; later base arrivals must not revise released pairs.
  opt.decide_on_push = false;
  opt.emit_unmatched_rover = true;
  opt.queue_limit = 512;
  opt.max_tdiff_s = 5.0;
  Matcher matcher(opt);
  const std::uint32_t week = 2200;

  // 100 s of 5 Hz rover and 1 Hz base, all pushed before anything is drained.
  for (int i = 0; i < 500; ++i) matcher.pushRover(epoch(week, 100.0 + 0.2 * i));
  for (int i = 0; i < 100; ++i) matcher.pushBase(epoch(week, 100.0 + 1.0 * i));

  const auto pairs = matcher.drainMatches();
  ASSERT_FALSE(pairs.empty());
  std::size_t unmatched = 0;
  for (const auto& p : pairs) {
    if (!p.hasBase()) ++unmatched;
  }
  EXPECT_EQ(unmatched, 0U)
      << "base pruned against the newest rover instead of the oldest";
  for (const auto& p : pairs) {
    if (p.hasBase()) {
      EXPECT_LE(p.age_s, 1.0) << "paired with a needlessly old base";
    }
  }
}

TEST(EpochMatcher, OverflowStillDropsBaseTheRoverHasLeftBehind) {
  // The other end of the same policy: when the front is genuinely past the
  // matching window it is the useless one, and it must still be the victim so
  // the queue cannot be pinned full by stale history.
  Matcher::Options opt;
  opt.emit_unmatched_rover = true;
  opt.queue_limit = 8;
  opt.max_tdiff_s = 2.0;        // everything older than 2 s is unusable
  Matcher matcher(opt);
  const std::uint32_t week = 2200;

  for (int i = 0; i <= 20; ++i) matcher.pushBase(epoch(week, 100.0 + i));
  matcher.pushRover(epoch(week, 130.0));   // far ahead of the oldest base
  (void)matcher.drainMatches();
  for (int i = 21; i <= 30; ++i) matcher.pushBase(epoch(week, 100.0 + i));

  // The newest base (130) survived, so the rover can still be served.
  const auto pairs = matcher.flush();
  ASSERT_FALSE(pairs.empty());
  EXPECT_EQ(pairs.back().kind, Matcher::PairKind::Exact);
  EXPECT_DOUBLE_EQ(pairs.back().base->tow, 130.0);
}

// The cadence estimate must not get more expensive the longer the node runs.
// It is called from the rover matching loop and sorts its history twice, so an
// unbounded history turns uptime into per-epoch cost: invisible in a replay of
// a few thousand epochs, unbounded in continuous operation.
TEST(EpochMatcher, CadenceEstimateCostDoesNotGrowWithUptime) {
  Matcher matcher;
  const std::uint32_t week = 2200;
  for (int i = 0; i < 100000; ++i)
    matcher.pushBase(epoch(week, 100.0 + i * 1.0));
  // 1e-6, not tighter: base_times_ holds absolute GPS-epoch seconds (~1.4e9),
  // where a double resolves ~2e-7, so differencing them cannot do better.
  EXPECT_NEAR(matcher.estimatedBaseInterval(), 1.0, 1e-6);

  // The retained history is bounded, so the queue cannot be the thing that
  // grows either. queue_limit (50 by default) bounds the pending queue; the
  // cadence window is separate and also bounded.
  EXPECT_LE(matcher.pendingBaseCount(), 50U);

  // And it still tracks a CHANGE in cadence rather than being anchored to the
  // whole session - the point of a window.
  for (int i = 0; i < 200; ++i)
    matcher.pushBase(epoch(week, 100100.0 + i * 0.2));
  EXPECT_NEAR(matcher.estimatedBaseInterval(), 0.2, 1e-6);
}

// R8-1: with history kept for max_tdiff_s, base_rate x max_tdiff_s above the
// queue limit made overflow evict the NEWEST base, pairing rover epochs with a
// base seconds older than the one that had arrived. Every pair must use the
// newest base that had arrived when the epoch was decided.
TEST(EpochMatcher, AnyBaseRateAndAgeLimitPairsWithTheNewestArrivedBase) {
  const std::uint32_t week = 2200;
  for (const double rover_hz : {1.0, 10.0}) {
    for (const double base_hz : {1.0, 5.0, 10.0, 20.0}) {
      for (const double latency : {0.0, 0.1, 0.5}) {
        for (const double max_tdiff : {5.0, 10.0, 30.0}) {
          Matcher::Options opt;
          opt.emit_unmatched_rover = true;
          opt.max_tdiff_s = max_tdiff;
          Matcher matcher(opt);
          // Arrival order: base(t) arrives at t + latency, rover(t) at t.
          struct Ev { double arrival; bool base; double t; };
          std::vector<Ev> ev;
          for (int k = 0; k <= static_cast<int>(60 * rover_hz); ++k)
            ev.push_back({100.0 + k / rover_hz, false, 100.0 + k / rover_hz});
          for (int j = 0; j <= static_cast<int>(60 * base_hz); ++j)
            ev.push_back({100.0 + j / base_hz + latency, true, 100.0 + j / base_hz});
          std::stable_sort(ev.begin(), ev.end(),
                           [](const Ev& a, const Ev& b) { return a.arrival < b.arrival; });
          std::vector<Matcher::Pair> pairs;
          for (const auto& e : ev) {
            if (e.base) matcher.pushBase(epoch(week, e.t));
            else matcher.pushRover(epoch(week, e.t));
            for (auto& p : matcher.drainMatches()) pairs.push_back(std::move(p));
          }
          for (auto& p : matcher.flush()) pairs.push_back(std::move(p));

          const std::string where = "rover " + std::to_string(rover_hz) + " Hz, base " +
                                    std::to_string(base_hz) + " Hz, latency " +
                                    std::to_string(latency) + ", max_tdiff " +
                                    std::to_string(max_tdiff);
          EXPECT_EQ(matcher.counters().dropped(Matcher::Stream::Base,
                                               Matcher::DropReason::QueueOverflow),
                    0U) << where;
          ASSERT_EQ(pairs.size(), static_cast<std::size_t>(60 * rover_hz) + 1) << where;
          for (const auto& p : pairs) {
            // Only epochs decided before the first base arrived may lack one.
            if (!p.hasBase()) {
              EXPECT_LT(p.rover->tow + 1.0 / rover_hz, 100.0 + latency + 1e-6)
                  << where << " tow " << p.rover->tow;
              continue;
            }
            const double base_age_bound = latency + 1.0 / base_hz + 1e-6;
            EXPECT_LE(p.age_s, base_age_bound) << where << " tow " << p.rover->tow;
            if (latency == 0.0 && std::fabs(p.rover->tow * base_hz -
                                            std::round(p.rover->tow * base_hz)) < 1e-6) {
              EXPECT_EQ(p.kind, Matcher::PairKind::Exact) << where << " tow " << p.rover->tow;
            }
          }
        }
      }
    }
  }
}

// Q-A1: an exact pair must not wait for the next rover epoch (one rover
// interval of latency for nothing). Only an epoch with an older base waits.
TEST(EpochMatcher, ExactPairIsReleasedWithoutWaitingForTheNextRover) {
  Matcher matcher;
  const std::uint32_t week = 2200;
  matcher.pushBase(epoch(week, 100.0));
  matcher.pushRover(epoch(week, 100.0));
  const auto first = matcher.drainMatches();
  ASSERT_EQ(first.size(), 1U);
  EXPECT_EQ(first[0].kind, Matcher::PairKind::Exact);

  matcher.pushRover(epoch(week, 101.0));   // base 101 not yet here
  EXPECT_TRUE(matcher.drainMatches().empty());
  matcher.pushBase(epoch(week, 101.0));
  const auto second = matcher.drainMatches();
  ASSERT_EQ(second.size(), 1U);
  EXPECT_EQ(second[0].kind, Matcher::PairKind::Exact);
}

// A-4: at 20 Hz the GPST difference of adjacent epochs rounds to just below
// 0.05 s, so whether the reorder window was reached depended on the epoch.
TEST(EpochMatcher, TwentyHertzSettlesAfterOneRoverInterval) {
  Matcher matcher;   // reorder_window_s 0.05
  const std::uint32_t week = 2200;
  matcher.pushBase(epoch(week, 100.0));
  std::size_t released = 0;
  for (int k = 1; k <= 40; ++k) {
    matcher.pushRover(epoch(week, 100.0 + k / 20.0));
    released += matcher.drainMatches().size();
    EXPECT_EQ(released, static_cast<std::size_t>(k - 1)) << "epoch " << k;
  }
}

TEST(EpochMatcher, ExplicitResetAllowsEarlierReplaySession) {
  Matcher matcher;
  matcher.pushRover(epoch(2300,100));matcher.pushBase(epoch(2300,100));
  ASSERT_EQ(matcher.flush().size(),1u);
  matcher.reset();
  matcher.pushRover(epoch(2300,10));matcher.pushBase(epoch(2300,10));
  const auto pairs=matcher.flush();ASSERT_EQ(pairs.size(),1u);
  EXPECT_DOUBLE_EQ(pairs.front().rover->tow,10);
}

#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
// Test harness worker: one solver owner woken by intake (not part of the package API).
namespace gnss_utils {
// One solver owner, woken only by intake. Notifications during a solve remain
// pending; stopping wakes an idle worker and joins an in-flight solve.
class EventWorker {
public:
  EventWorker() = default;
  EventWorker(const EventWorker&) = delete;
  EventWorker& operator=(const EventWorker&) = delete;
  ~EventWorker() { stop(); }
  void start(std::function<void()> work) {
    thread_ = std::thread([this, work=std::move(work)] {
      std::unique_lock<std::mutex> lock(mutex_);
      while (true) {
        cv_.wait(lock, [this]{return pending_ || stopping_;});
        if (stopping_) return;
        pending_=false;
        lock.unlock(); work(); lock.lock();
      }
    });
  }
  void notify() {
    { std::lock_guard<std::mutex> lock(mutex_); pending_=true; }
    cv_.notify_one();
  }
  void stop() {
    { std::lock_guard<std::mutex> lock(mutex_); stopping_=true; }
    cv_.notify_one();
    if (thread_.joinable()) thread_.join();
  }
private:
  std::mutex mutex_;
  std::condition_variable cv_;
  bool pending_{false}, stopping_{false};
  std::thread thread_;
};
} // namespace gnss_utils
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

TEST(ReleaseG2, SlowSolverDoesNotBlockIntakeOrChangeEpochPrior) {
  using namespace std::chrono_literals;
  auto replay=[](double hz, bool slow) {
    Matcher::Options options;
    options.emit_unmatched_rover=true;
    Matcher matcher(options);
    gnss_utils::EventWorker worker;
    std::mutex intake, output_mutex;
    std::condition_variable done;
    std::vector<std::pair<double,double>> outputs;
    std::deque<Matcher::Pair> tail;
    double previous=1.;
    const int count=static_cast<int>(2*hz)+1; // Includes three 1 Hz base-grid epochs.
    worker.start([&] {
      std::vector<Matcher::Pair> pairs;
      {
        std::lock_guard<std::mutex> lock(intake);
        pairs=matcher.drainMatches();
        while (!tail.empty()) { pairs.push_back(tail.front());tail.pop_front(); }
      }
      for (const auto& pair:pairs) {
        // Synthetic state-dependent preprocessing immediately before each solve.
        // Batching all aprioris at drain time would fail this equality test.
        const double measurement=pair.rover->tow+previous;
        if (slow) std::this_thread::sleep_for(300ms);
        previous=measurement*.5;
        { std::lock_guard<std::mutex> lock(output_mutex);
          outputs.emplace_back(pair.age_s,previous); }
        done.notify_all();
      }
    });
    for (int i=0;i<count;++i) {
      { std::lock_guard<std::mutex> lock(intake);
        const double tow=100.+i/hz;
        if (std::abs(tow-std::round(tow))<1e-8) matcher.pushBase(epoch(2300,tow));
        matcher.pushRover(epoch(2300,tow)); }
      worker.notify();
      // Intake keeps running during the slow solve.
      std::this_thread::sleep_for(5ms);
    }
    { std::lock_guard<std::mutex> lock(intake);
      for (auto& pair:matcher.flush()) tail.push_back(std::move(pair)); }
    worker.notify();
    { std::unique_lock<std::mutex> lock(output_mutex);
      EXPECT_TRUE(done.wait_for(lock,30s,[&]{return outputs.size()==static_cast<size_t>(count);})); }
    worker.stop();
    for (int i=0;i<int(outputs.size());++i)
      if (std::abs(i/hz-std::round(i/hz))<1e-8) { EXPECT_DOUBLE_EQ(outputs[i].first,0.); }
    return outputs;
  };
  for (double hz : {1.,5.,10.,20.}) EXPECT_EQ(replay(hz,false),replay(hz,true));
}
