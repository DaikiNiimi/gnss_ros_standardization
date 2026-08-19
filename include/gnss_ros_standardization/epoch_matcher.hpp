// SPDX-License-Identifier: MIT
#ifndef GNSS_ROS_STANDARDIZATION_EPOCH_MATCHER_HPP
#define GNSS_ROS_STANDARDIZATION_EPOCH_MATCHER_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <vector>

#include "gnss_ros_standardization/msg/gnss_observations.hpp"

extern "C" {
#include "rtklib.h"
}

namespace gnss_utils {

// Rover/base observation epoch matcher shared by the RTK node and the
// tightly-coupled FGO examples. Pairs each rover epoch with a base epoch:
// EXACT within match_tol_s when available, otherwise the most recent base
// epoch up to max_tdiff_s old (AGED); with emit_unmatched_rover, a rover epoch
// with no usable base is emitted UNMATCHED (base outage) rather than dropped.
//
// Losslessness: both streams are kept in full-GPST order (callback order does
// not change the result), duplicates / out-of-order / overflow / invalid-time
// inputs are reason-tagged and counted (never silently dropped), and flush()
// finalizes the newest buffered epoch at end-of-stream so nothing is left
// unprocessed. Bounded reorder (reorder_window_s) prevents publishing a newer
// exact pair before an older one that is still within the window.
//
// Not thread-safe: the caller serializes pushRover/pushBase/drainMatches/flush.
class RoverBaseEpochMatcher {
 public:
  using ObsMsg = gnss_ros_standardization::msg::GnssObservations;

  enum class PairKind { Exact, Aged, Unmatched };
  enum class Stream { Rover, Base };
  enum class DropReason {
    InvalidTimestamp, Duplicate, LateArrival, QueueOverflow,
    NoBaseWithinWindow, EndOfStreamNoBase
  };
  static constexpr int kNumReasons = 6;
  static constexpr int kNumStreams = 2;

  struct Options {
    double max_tdiff_s{30.0};    // max accepted rover-base age (RTKLIB maxtdiff)
    double match_tol_s{0.001};   // exact-match tolerance
    double duplicate_tol_s{1e-6};  // same-epoch tolerance (<= match_tol_s)
    double reorder_window_s{0.05};  // hold a rover until the watermark advances
    double drop_ahead_s{2.0};    // base this far ahead -> the rover is decidable
    std::size_t queue_limit{50};
    bool emit_unmatched_rover{false};  // emit rover-only epochs on base outage
  };

  struct Pair {
    ObsMsg::ConstSharedPtr rover;
    ObsMsg::ConstSharedPtr base;
    PairKind kind{PairKind::Unmatched};
    double age_s{0.0};  // rover epoch minus base epoch (NaN when unmatched)
    bool hasBase() const { return static_cast<bool>(base); }
  };

  struct DropEvent {
    Stream stream;
    DropReason reason;
    std::uint32_t week{0};
    double tow{0.0};
  };

  // Legacy diagnostic view kept for the RTK node's existing log.
  struct Dropped {
    double rover_tow{0.0};
    double newest_base_tow{0.0};
  };

  struct Counters {
    std::array<std::array<std::uint64_t, kNumReasons>, kNumStreams> counts{};
    std::uint64_t pushed_rover{0};   // messages handed to pushRover
    std::uint64_t emitted_rover{0};  // pairs returned to the caller
    std::uint64_t emitted_unmatched{0};
    std::uint64_t drop_events_overwritten{0};
    std::uint64_t legacy_drop_views_overwritten{0};
    std::uint64_t dropped(Stream s, DropReason r) const {
      return counts[static_cast<int>(s)][static_cast<int>(r)];
    }
  };

  RoverBaseEpochMatcher() : RoverBaseEpochMatcher(Options{}) {}
  explicit RoverBaseEpochMatcher(const Options& opt) : opt_(opt) {
    if (opt_.queue_limit == 0)
      throw std::invalid_argument("queue_limit must be positive");
    if (!(std::isfinite(opt_.match_tol_s) && opt_.match_tol_s >= 0.0))
      throw std::invalid_argument("match_tol_s must be finite and non-negative");
    if (!(std::isfinite(opt_.duplicate_tol_s) && opt_.duplicate_tol_s >= 0.0 &&
          opt_.duplicate_tol_s <= opt_.match_tol_s))
      throw std::invalid_argument("duplicate_tol_s must be in [0, match_tol_s]");
    if (!(std::isfinite(opt_.reorder_window_s) && opt_.reorder_window_s >= 0.0))
      throw std::invalid_argument("reorder_window_s must be finite non-negative");
    if (!(std::isfinite(opt_.max_tdiff_s) && opt_.max_tdiff_s >= 0.0))
      throw std::invalid_argument("max_tdiff_s must be finite and non-negative");
    if (!(std::isfinite(opt_.drop_ahead_s) && opt_.drop_ahead_s >= 0.0))
      throw std::invalid_argument("drop_ahead_s must be finite and non-negative");
    event_limit_ = std::max<std::size_t>(opt_.queue_limit, 64);
  }

  void pushRover(ObsMsg::ConstSharedPtr msg) {
    ++counters_.pushed_rover;
    insert(Stream::Rover, rover_queue_, std::move(msg));
  }

  void pushBase(ObsMsg::ConstSharedPtr msg) {
    insert(Stream::Base, base_queue_, std::move(msg));
  }

  // Decide every rover epoch that can be decided now; hold the rest.
  std::vector<Pair> drainMatches() { return process(false); }

  // End-of-stream: finalize the newest epoch too, so nothing is left pending.
  std::vector<Pair> flush() { return process(true); }

  const Counters& counters() const { return counters_; }
  std::size_t pendingRoverCount() const { return rover_queue_.size(); }
  std::size_t pendingBaseCount() const { return base_queue_.size(); }

  // Median of the sorted adjacent base intervals (robust to duplicates); <= 0
  // until at least two base epochs have arrived.
  // Median adjacent interval over a BOUNDED window of recent base epochs.
  //
  // The window matters for more than tidiness: this is called from the rover
  // matching loop, and it copies and sorts its history twice. With an unbounded
  // history that cost grows with UPTIME - invisible in a 50-minute replay
  // (~3000 samples) and unbounded in continuous operation (86 400/day at 1 Hz),
  // where it eventually stops the consumer keeping up with its own input.
  // A cadence estimate wants recent samples anyway, not the whole session.
  double estimatedBaseInterval() const {
    if (base_times_.size() < 2) return 0.0;
    std::vector<double> sorted(base_times_.begin(), base_times_.end());
    std::sort(sorted.begin(), sorted.end());
    std::vector<double> gaps;
    gaps.reserve(sorted.size() - 1);
    for (std::size_t i = 1; i < sorted.size(); ++i) {
      const double g = sorted[i] - sorted[i - 1];
      if (g > 0.0) gaps.push_back(g);
    }
    if (gaps.empty()) return 0.0;
    std::sort(gaps.begin(), gaps.end());
    return gaps[gaps.size() / 2];
  }

  double newestBaseCtow() const {
    return base_queue_.empty()
               ? -1.0
               : base_queue_.back()->week * 604800.0 + base_queue_.back()->tow;
  }

  std::vector<Dropped> takeDropped() {
    std::vector<Dropped> out;
    out.swap(dropped_);
    return out;
  }

  std::vector<DropEvent> takeDropEvents() {
    std::vector<DropEvent> out;
    out.swap(drop_events_);
    return out;
  }

  static const char* dropReasonName(DropReason r) {
    switch (r) {
      case DropReason::InvalidTimestamp: return "invalid_timestamp";
      case DropReason::Duplicate: return "duplicate";
      case DropReason::LateArrival: return "late_arrival";
      case DropReason::QueueOverflow: return "queue_overflow";
      case DropReason::NoBaseWithinWindow: return "no_base_within_window";
      case DropReason::EndOfStreamNoBase: return "end_of_stream_no_base";
    }
    return "unknown";
  }

  static const char* streamName(Stream s) {
    return s == Stream::Rover ? "rover" : "base";
  }

 private:
  using Queue = std::deque<ObsMsg::ConstSharedPtr>;

  static gtime_t msgTime(const ObsMsg& m) {
    return gpst2time(static_cast<int>(m.week), m.tow);
  }
  static bool validTow(double tow) {
    return std::isfinite(tow) && tow >= 0.0 && tow < 604800.0;
  }

  void recordDrop(Stream s, DropReason r, const ObsMsg& m) {
    ++counters_.counts[static_cast<int>(s)][static_cast<int>(r)];
    if (drop_events_.size() >= event_limit_) {
      drop_events_.erase(drop_events_.begin());
      ++counters_.drop_events_overwritten;
    }
    drop_events_.push_back({s, r, m.week, m.tow});
  }

  // Sorted, deduplicated, late-tagged insertion with bounded overflow.
  void insert(Stream stream, Queue& q, ObsMsg::ConstSharedPtr msg) {
    if (!validTow(msg->tow)) { recordDrop(stream, DropReason::InvalidTimestamp, *msg); return; }
    const gtime_t t = msgTime(*msg);
    for (const auto& e : q) {
      if (std::fabs(timediff(msgTime(*e), t)) <= opt_.duplicate_tol_s) {
        recordDrop(stream, DropReason::Duplicate, *msg);
        return;
      }
    }
    const bool have_floor = (stream == Stream::Rover) ? have_rover_floor_
                                                      : have_base_floor_;
    const gtime_t& floor = (stream == Stream::Rover) ? rover_floor_ : base_floor_;
    if (have_floor && timediff(t, floor) <= 0.0) {
      recordDrop(stream, DropReason::LateArrival, *msg);
      return;
    }
    // Sorted insert (ascending GPST).
    auto pos = q.begin();
    while (pos != q.end() && timediff(msgTime(**pos), t) < 0.0) ++pos;
    q.insert(pos, std::move(msg));
    if (stream == Stream::Base) {
      base_times_.push_back(t.time + t.sec);
      if (base_times_.size() > kCadenceWindow)
        base_times_.erase(base_times_.begin(),
                          base_times_.end() - kCadenceWindow);
    }
    while (q.size() > opt_.queue_limit) {
      // Evict the end that is USELESS to the rover being matched, which is not
      // always the oldest. A consumer that cannot keep up processes rover
      // epochs behind the wall clock, so the base queue fills with epochs
      // AHEAD of the rover; its front is then the epoch the lagging rover is
      // about to need, and dropping it is the one thing that must not happen.
      // Drop the back instead whenever the front is still inside the matching
      // window. (Instrumented on a 5 Hz rover / 1 Hz base course: the base
      // stream arrived exactly on time while pendingBaseCount climbed to the
      // limit and pinned there, evicting the front on every subsequent push.)
      bool drop_front = true;
      if (stream == Stream::Base && !rover_queue_.empty()) {
        const gtime_t oldest_rover = msgTime(*rover_queue_.front());
        drop_front =
            timediff(oldest_rover, msgTime(*q.front())) > opt_.max_tdiff_s;
      }
      const ObsMsg& victim = drop_front ? *q.front() : *q.back();
      if (drop_front) setFloor(stream, msgTime(victim));
      recordDrop(stream, DropReason::QueueOverflow, victim);
      if (drop_front) q.pop_front(); else q.pop_back();
    }
  }

  void setFloor(Stream stream, const gtime_t& t) {
    if (stream == Stream::Rover) {
      if (!have_rover_floor_ || timediff(t, rover_floor_) > 0.0) rover_floor_ = t;
      have_rover_floor_ = true;
    } else {
      if (!have_base_floor_ || timediff(t, base_floor_) > 0.0) base_floor_ = t;
      have_base_floor_ = true;
    }
  }

  std::vector<Pair> process(bool end_of_stream) {
    std::vector<Pair> out;
    // Prune base epochs no pending rover can still use. The reference is the
    // OLDEST queued rover - the next one to be decided - not the newest: a base
    // is only dead once the rover at the head of the queue has outrun it by
    // max_tdiff. Measuring from the newest discards base history the rovers
    // behind it still need, in proportion to how deep the queue is.
    if (!rover_queue_.empty() && base_queue_.size() > 1) {
      const gtime_t oldest_rover = msgTime(*rover_queue_.front());
      while (base_queue_.size() > 1 &&
             timediff(oldest_rover, msgTime(*base_queue_.front())) > opt_.max_tdiff_s) {
        setFloor(Stream::Base, msgTime(*base_queue_.front()));
        base_queue_.pop_front();
      }
    }

    const bool have_newest = !rover_queue_.empty();
    const gtime_t newest_rover =
        have_newest ? msgTime(*rover_queue_.back()) : gtime_t{};

    auto it = rover_queue_.begin();
    while (it != rover_queue_.end()) {
      const ObsMsg::ConstSharedPtr rover_msg = *it;
      const gtime_t rover_time = msgTime(*rover_msg);
      const bool is_newest = (std::next(it) == rover_queue_.end());
      // Bounded reorder: hold a rover until the watermark is reorder_window past
      // it, unless we are flushing. The base stream is an equally good witness
      // that time has moved on, and the only one available during a rover
      // outage - there the newest rover IS this one, so `behind` stays 0 and
      // the epoch would be held for the whole outage.
      const double behind = timediff(newest_rover, rover_time);
      bool settled = end_of_stream || (behind >= opt_.reorder_window_s);
      if (!settled && !base_queue_.empty() &&
          timediff(msgTime(*base_queue_.back()), rover_time) >=
              opt_.drop_ahead_s)
        settled = true;

      // Exact base within match_tol.
      ObsMsg::ConstSharedPtr exact;
      for (const auto& b : base_queue_)
        if (std::fabs(timediff(msgTime(*b), rover_time)) <= opt_.match_tol_s) {
          exact = b;
          break;
        }
      if (exact) {
        if (!settled) { ++it; continue; }
        setFloor(Stream::Rover, rover_time);
        ++counters_.emitted_rover;
        out.push_back({rover_msg, exact, PairKind::Exact,
                       timediff(rover_time, msgTime(*exact))});
        it = rover_queue_.erase(it);
        continue;
      }

      // Aged base: newest base not after the rover, within max_tdiff.
      ObsMsg::ConstSharedPtr aged;
      double best_age = 1e18;
      for (const auto& b : base_queue_) {
        const double age = timediff(rover_time, msgTime(*b));
        if (age >= -opt_.match_tol_s && age <= opt_.max_tdiff_s && age < best_age) {
          best_age = age;
          aged = b;
        }
      }
      if (aged) {
        // Wait for a closer base only while unsettled and a newer one is due.
        bool wait = !settled;
        const double base_interval = estimatedBaseInterval();
        if (wait && !is_newest && base_interval > 0.0 &&
            best_age < base_interval - 0.01)
          wait = false;  // the base before it is already the best
        if (wait) { ++it; continue; }
        setFloor(Stream::Rover, rover_time);
        ++counters_.emitted_rover;
        out.push_back({rover_msg, aged, PairKind::Aged, best_age});
        it = rover_queue_.erase(it);
        continue;
      }

      // No usable base. Decide only when settled (or at EOF).
      if (!settled) { ++it; continue; }
      if (opt_.emit_unmatched_rover) {
        setFloor(Stream::Rover, rover_time);
        ++counters_.emitted_unmatched;
        ++counters_.emitted_rover;
        out.push_back({rover_msg, nullptr, PairKind::Unmatched,
                       std::numeric_limits<double>::quiet_NaN()});
        it = rover_queue_.erase(it);
        continue;
      }
      // Drop with the right reason: the newest epoch at EOF is EndOfStreamNoBase,
      // anything else that outran its base window is NoBaseWithinWindow.
      const bool base_far_ahead =
          !base_queue_.empty() &&
          timediff(msgTime(*base_queue_.back()), rover_time) > opt_.drop_ahead_s;
      if (end_of_stream || base_far_ahead) {
        const DropReason reason =
            (end_of_stream && is_newest) ? DropReason::EndOfStreamNoBase
                                         : DropReason::NoBaseWithinWindow;
        setFloor(Stream::Rover, rover_time);
        dropped_.push_back({rover_msg->tow,
                            base_queue_.empty() ? 0.0 : base_queue_.back()->tow});
        recordDrop(Stream::Rover, reason, *rover_msg);
        it = rover_queue_.erase(it);
        continue;
      }
      ++it;
    }
    // End-of-stream: no further rover can arrive, so leftover base history is
    // no longer needed. Retire it so nothing stays pending (not a data loss:
    // any base that could pair already has).
    if (end_of_stream) {
      for (const auto& b : base_queue_) setFloor(Stream::Base, msgTime(*b));
      base_queue_.clear();
    }
    return out;
  }

  Options opt_;
  Queue rover_queue_;
  Queue base_queue_;
  std::vector<Dropped> dropped_;
  std::vector<DropEvent> drop_events_;
  // Recent base epoch times for the median cadence estimate. Bounded: see
  // estimatedBaseInterval.
  static constexpr std::size_t kCadenceWindow = 64;
  std::vector<double> base_times_;
  Counters counters_;
  std::size_t event_limit_{64};
  gtime_t rover_floor_{};
  gtime_t base_floor_{};
  bool have_rover_floor_{false};
  bool have_base_floor_{false};
};

}  // namespace gnss_utils

#endif  // GNSS_ROS_STANDARDIZATION_EPOCH_MATCHER_HPP
