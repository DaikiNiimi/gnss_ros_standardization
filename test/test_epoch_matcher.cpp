// SPDX-License-Identifier: MIT
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
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

TEST(EpochMatcher, BoundedCallbackReorderCannotPublishNewerExactFirst) {
  Matcher::Options opt;
  opt.reorder_window_s = 0.05;
  Matcher matcher(opt);

  // Simulate the real callback pattern: every push is followed by drain.
  matcher.pushRover(epoch(2200, 100.04));
  EXPECT_TRUE(matcher.drainMatches().empty());
  matcher.pushBase(epoch(2200, 100.04));
  EXPECT_TRUE(matcher.drainMatches().empty());

  // An older rover/base pair arrives 40 ms late, still inside the configured
  // reorder window. Neither exact pair may publish out of order.
  matcher.pushRover(epoch(2200, 100.00));
  EXPECT_TRUE(matcher.drainMatches().empty());
  matcher.pushBase(epoch(2200, 100.00));
  EXPECT_TRUE(matcher.drainMatches().empty());

  matcher.pushRover(epoch(2200, 100.10));  // advances rover watermark
  const auto pairs = matcher.drainMatches();
  ASSERT_EQ(pairs.size(), 2U);
  EXPECT_DOUBLE_EQ(pairs[0].rover->tow, 100.00);
  EXPECT_DOUBLE_EQ(pairs[1].rover->tow, 100.04);
  EXPECT_EQ(pairs[0].kind, Matcher::PairKind::Exact);
  EXPECT_EQ(pairs[1].kind, Matcher::PairKind::Exact);
  EXPECT_EQ(matcher.pendingRoverCount(), 1U);
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
  opt.queue_limit = 2;
  Matcher matcher(opt);

  matcher.pushRover(epoch(2200, 100.0));
  matcher.pushRover(epoch(2200, 101.0));
  matcher.pushRover(epoch(2200, 102.0));  // evicts rover 100
  matcher.pushRover(epoch(2200, 99.0));   // older than terminal epoch
  matcher.pushRover(epoch(2200, 102.0));  // queued duplicate

  // Base overflow evicts the end the rover cannot use. Here the rover queue
  // holds 101/102 and the base is far AHEAD of it, so the front (200) is the
  // one the rover will reach first and the BACK is evicted instead - see
  // OverflowKeepsTheBaseTheLaggingRoverStillNeeds. Base 200 therefore survives
  // and re-pushing it is a duplicate, not a late arrival.
  matcher.pushBase(epoch(2200, 200.0));
  matcher.pushBase(epoch(2200, 201.0));
  matcher.pushBase(epoch(2200, 202.0));   // evicts base 202 (the back)
  matcher.pushBase(epoch(2200, 200.0));   // still queued: duplicate

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

  const auto events = matcher.takeDropEvents();
  EXPECT_EQ(events.size(), 5U);
  EXPECT_TRUE(matcher.takeDropEvents().empty());
  EXPECT_EQ(matcher.pendingRoverCount(), 2U);
  EXPECT_EQ(matcher.pendingBaseCount(), 2U);
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

  // The rover is stuck at 100.0 (the node is busy); base keeps arriving.
  matcher.pushRover(epoch(week, 100.0));
  (void)matcher.drainMatches();
  for (int i = 0; i <= 20; ++i) matcher.pushBase(epoch(week, 100.0 + i));

  // Overflow happened, and it did NOT throw away base 100.0.
  EXPECT_GT(matcher.counters().dropped(Matcher::Stream::Base,
                                       Matcher::DropReason::QueueOverflow),
            0U);
  const auto pairs = matcher.flush();
  ASSERT_EQ(pairs.size(), 1U);
  EXPECT_EQ(pairs[0].kind, Matcher::PairKind::Exact)
      << "the base epoch matching the lagging rover was evicted";
  ASSERT_TRUE(pairs[0].hasBase());
  EXPECT_DOUBLE_EQ(pairs[0].base->tow, 100.0);
}

TEST(EpochMatcher, TheBaseStreamSettlesTheLastRoverBeforeAnOutage) {
  // During a rover outage the last rover epoch IS the newest, so the
  // rover-only reorder rule never settles it and it would be held for the whole
  // outage. The base stream keeps arriving and proves time has moved on.
  Matcher::Options opt;
  opt.emit_unmatched_rover = true;
  opt.drop_ahead_s = 2.0;
  Matcher matcher(opt);
  const std::uint32_t week = 2200;

  matcher.pushRover(epoch(week, 100.0));
  matcher.pushBase(epoch(week, 100.0));
  EXPECT_TRUE(matcher.drainMatches().empty()) << "nothing proves time moved yet";

  // Base keeps coming; the rover stream has stopped.
  matcher.pushBase(epoch(week, 101.0));
  EXPECT_TRUE(matcher.drainMatches().empty()) << "only 1 s past - still inside";
  matcher.pushBase(epoch(week, 102.0));
  const auto pairs = matcher.drainMatches();
  ASSERT_EQ(pairs.size(), 1U) << "held for the whole outage";
  EXPECT_DOUBLE_EQ(pairs[0].rover->tow, 100.0);
  EXPECT_EQ(pairs[0].kind, Matcher::PairKind::Exact);
}

TEST(EpochMatcher, ADeepRoverQueueDoesNotPruneTheBaseItStillNeeds) {
  // A backlogged node holds many rover epochs at once. Pruning base relative to
  // the NEWEST queued rover then discards the base the queued rovers are about
  // to be matched against, and it gets worse the deeper the queue - invisible
  // while the node keeps up, severe once it does not.
  Matcher::Options opt;
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
