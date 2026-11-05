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

#include "gnss_ros_standardization/rtklib_compat.hpp"

namespace gnss_utils {

// Rover/base observation epoch matcher shared by the RTK node and the
// tightly-coupled FGO nodes. Pairs each rover epoch with a base epoch:
// EXACT within match_tol_s when available, otherwise the most recent base
// epoch up to max_tdiff_s old (AGED); with emit_unmatched_rover, a rover epoch
// with no usable base is emitted UNMATCHED (base outage) rather than dropped.
//
// Both streams are kept in full-GPST order and rover epochs are decided in that
// order. A rover epoch whose base(T) is present is decided at once; one that
// only has an older base is decided at once when the observed base cadence
// rules out a closer base, and otherwise waits until a later rover epoch
// arrives, so base(T) may still arrive within one rover interval. A rover epoch arriving after a
// newer one was decided is a counted LateArrival. Duplicates / out-of-order /
// overflow / invalid-time inputs are reason-tagged and counted (never silently
// dropped), and flush() finalizes the newest buffered epoch at end-of-stream so
// nothing is left unprocessed.
//
// Base history is pruned by usefulness: once every undecided rover epoch is
// later than T, only the newest base not after T and newer bases can still be
// selected, so older ones are retired. The newest base is never evicted.
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
    bool decide_on_push{true};  // decisions follow intake order, not consumer cadence
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

  // Explicit session boundary (bag loop/seek or receiver clock reset).
  // Keep lifetime counters, discard queues and timestamp floors together.
  void reset() {
    rover_queue_.clear(); base_queue_.clear(); base_times_.clear(); ready_.clear();
    dropped_.clear(); drop_events_.clear();
    have_rover_floor_ = have_base_floor_ = false;
  }

  void pushRover(ObsMsg::ConstSharedPtr msg) {
    ++counters_.pushed_rover;
    insert(Stream::Rover, rover_queue_, std::move(msg));
    if (opt_.decide_on_push) collect(false);
  }

  void pushBase(ObsMsg::ConstSharedPtr msg) {
    insert(Stream::Base, base_queue_, std::move(msg));
    if (opt_.decide_on_push) collect(false);
  }

  // Decide every rover epoch that can be decided now; hold the rest.
  std::vector<Pair> drainMatches() { collect(false); return takeReady(); }

  void setDecideOnPush(bool enabled) { opt_.decide_on_push = enabled; }

  // End-of-stream: finalize the newest epoch too, so nothing is left pending.
  std::vector<Pair> flush() { collect(true); return takeReady(); }

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
    if (msg->week == 0 || !validTow(msg->tow)) { recordDrop(stream, DropReason::InvalidTimestamp, *msg); return; }
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
    // Decisions are taken at intake, so a lagging consumer cannot fill these
    // queues; overflow means a stream outran the other. Evict from the front
    // only: the newest epoch is the one the next decision needs.
    const std::size_t limit = stream == Stream::Base ? baseQueueLimit() : opt_.queue_limit;
    while (q.size() > limit) {
      const ObsMsg& victim = *q.front();
      setFloor(stream, msgTime(victim));
      recordDrop(stream, DropReason::QueueOverflow, victim);
      q.pop_front();
    }
  }

  // The base queue must hold every base up to drop_ahead_s ahead of the oldest
  // undecided rover; size that from the observed base cadence.
  std::size_t baseQueueLimit() const {
    const double interval = estimatedBaseInterval();
    if (!(interval > 0.0)) return opt_.queue_limit;
    const double needed = std::ceil(opt_.drop_ahead_s / interval) + 2.0;
    return std::max(opt_.queue_limit, static_cast<std::size_t>(std::min(needed, 1e6)));
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
    const bool have_newest = !rover_queue_.empty();
    const gtime_t newest_rover =
        have_newest ? msgTime(*rover_queue_.back()) : gtime_t{};

    // Rover epochs are decided strictly in time order: an epoch that has to
    // wait holds every later one, so output order never depends on which epoch
    // happened to get its base first.
    auto it = rover_queue_.begin();
    while (it != rover_queue_.end()) {
      const ObsMsg::ConstSharedPtr rover_msg = *it;
      const gtime_t rover_time = msgTime(*rover_msg);
      const bool is_newest = (std::next(it) == rover_queue_.end());
      // A waiting epoch is settled once a later rover epoch reaches the reorder
      // window (1 us tolerance: GPST seconds near 1.4e9 resolve only ~2e-7 s, and
      // a rover interval equal to the window must still settle), at flush, or
      // when the base stream is drop_ahead_s past it - the only witness during
      // a rover outage, where the newest rover IS this one.
      const double behind = timediff(newest_rover, rover_time);
      bool settled = end_of_stream || (behind + 1e-6 >= opt_.reorder_window_s);
      if (!settled && !base_queue_.empty() &&
          timediff(msgTime(*base_queue_.back()), rover_time) >=
              opt_.drop_ahead_s)
        settled = true;

      // Exact base within match_tol: nothing better can arrive, decide now.
      ObsMsg::ConstSharedPtr exact;
      for (const auto& b : base_queue_)
        if (std::fabs(timediff(msgTime(*b), rover_time)) <= opt_.match_tol_s) {
          exact = b;
          break;
        }
      if (exact) {
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
        // With the observed base cadence, the next base is due one interval
        // after the aged one; if that is after this rover epoch, no closer
        // base can come (e.g. a 5 Hz rover between 1 Hz base epochs).
        bool wait = !settled;
        const double base_interval = estimatedBaseInterval();
        if (wait && base_interval > 0.0 && best_age < base_interval - 0.01)
          wait = false;  // the base before it is already the best
        if (wait) break;
        setFloor(Stream::Rover, rover_time);
        ++counters_.emitted_rover;
        out.push_back({rover_msg, aged, PairKind::Aged, best_age});
        it = rover_queue_.erase(it);
        continue;
      }

      // No usable base. Decide only when settled (or at EOF).
      if (!settled) break;
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
      break;
    }
    pruneBaseHistory();
    // End-of-stream: no further rover can arrive, so leftover base history is
    // no longer needed. Retire it so nothing stays pending (not a data loss:
    // any base that could pair already has).
    if (end_of_stream) {
      for (const auto& b : base_queue_) setFloor(Stream::Base, msgTime(*b));
      base_queue_.clear();
    }
    return out;
  }

  // Every rover epoch still to be decided is later than T = min(rover floor,
  // oldest pending rover). Such an epoch selects either an exact base (within
  // match_tol of a time after T) or the newest base not after it, which is at
  // least the newest base not after T. Older bases can never be selected again.
  void pruneBaseHistory() {
    bool have_t = !rover_queue_.empty();
    gtime_t t = have_t ? msgTime(*rover_queue_.front()) : gtime_t{};
    if (have_rover_floor_ && (!have_t || timediff(rover_floor_, t) < 0.0)) {
      t = rover_floor_;
      have_t = true;
    }
    if (!have_t || base_queue_.size() < 2) return;
    bool found = false;
    gtime_t keep{};
    for (const auto& b : base_queue_) {
      if (timediff(msgTime(*b), t) > opt_.match_tol_s) break;
      keep = msgTime(*b);
      found = true;
    }
    if (!found) return;
    while (base_queue_.size() > 1) {
      const gtime_t front = msgTime(*base_queue_.front());
      if (timediff(front, keep) >= 0.0 || timediff(front, t) >= -opt_.match_tol_s) break;
      setFloor(Stream::Base, front);
      base_queue_.pop_front();
    }
  }

  void collect(bool eof) {
    for (auto& pair : process(eof)) {
      ready_.push_back(std::move(pair));
      if (ready_.size() > opt_.queue_limit) {
        recordDrop(Stream::Rover, DropReason::QueueOverflow, *ready_.front().rover);
        --counters_.emitted_rover;
        if (ready_.front().kind == PairKind::Unmatched) --counters_.emitted_unmatched;
        ready_.pop_front();
      }
    }
  }
  std::vector<Pair> takeReady() {
    std::vector<Pair> out(std::make_move_iterator(ready_.begin()),
                          std::make_move_iterator(ready_.end()));
    ready_.clear(); return out;
  }
  std::deque<Pair> ready_;
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
