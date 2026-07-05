// SPDX-License-Identifier: MIT
#ifndef GNSS_ROS_STANDARDIZATION_EPOCH_MATCHER_HPP
#define GNSS_ROS_STANDARDIZATION_EPOCH_MATCHER_HPP

#include <cmath>
#include <cstddef>
#include <deque>
#include <iterator>
#include <vector>

#include "gnss_ros_standardization/msg/gnss_observations.hpp"

extern "C" {
#include "rtklib.h"
}

namespace gnss_utils {

// Rover/base observation epoch matcher shared by the RTK node and the
// tightly-coupled FGO examples. Pairs each rover epoch with a base epoch:
// exact match within match_tol_s when available, otherwise the most recent
// base epoch up to max_tdiff_s old ("aged base"). When a newer base epoch may
// still arrive (estimated from the observed base interval) the rover epoch is
// kept queued instead of being paired early, so a 5 Hz rover / 1 Hz base setup
// pairs against the best base epoch rather than the first one seen.
//
// Epochs are compared on full GPST (week, tow) via timediff(), not tow alone,
// so pairing keeps working across the GPS week boundary.
//
// Not thread-safe: the caller serializes pushRover/pushBase/drainMatches
// (the RTK node holds its observation mutex across all three).
class RoverBaseEpochMatcher {
 public:
  using ObsMsg = gnss_ros_standardization::msg::GnssObservations;

  struct Options {
    double max_tdiff_s{30.0};   // max accepted rover-base age (RTKLIB maxtdiff)
    double match_tol_s{0.001};  // exact-match tolerance
    std::size_t queue_limit{50};
    double drop_ahead_s{2.0};   // drop a rover epoch once base is this far ahead
  };

  struct Pair {
    ObsMsg::ConstSharedPtr rover;
    ObsMsg::ConstSharedPtr base;
    double age_s{0.0};  // rover epoch minus base epoch
  };

  // Rover epoch dropped because no base epoch within max_tdiff_s can match it
  // anymore. Reported so the caller can log it.
  struct Dropped {
    double rover_tow{0.0};
    double newest_base_tow{0.0};
  };

  RoverBaseEpochMatcher() = default;
  explicit RoverBaseEpochMatcher(const Options& opt) : opt_(opt) {}

  void pushRover(ObsMsg::ConstSharedPtr msg) {
    rover_queue_.push_back(std::move(msg));
    if (rover_queue_.size() > opt_.queue_limit) rover_queue_.pop_front();
  }

  void pushBase(ObsMsg::ConstSharedPtr msg) {
    if (!base_queue_.empty()) {
      const double interval = timediff(msgTime(*msg), msgTime(*base_queue_.back()));
      if (interval > 0.01) {
        if (estimated_base_interval_ <= 0.0) {
          estimated_base_interval_ = interval;
        } else {
          estimated_base_interval_ = std::min(estimated_base_interval_, interval);
        }
      }
    }
    base_queue_.push_back(std::move(msg));
    if (base_queue_.size() > opt_.queue_limit) base_queue_.pop_front();
  }

  // Decide every rover epoch that can be decided now and return the resulting
  // pairs in rover-epoch order. Undecidable rover epochs (a better base epoch
  // may still arrive) stay queued for the next call.
  std::vector<Pair> drainMatches() {
    std::vector<Pair> out;

    if (!rover_queue_.empty() && !base_queue_.empty()) {
      const gtime_t newest_rover = msgTime(*rover_queue_.back());
      while (base_queue_.size() > 1 &&
             timediff(newest_rover, msgTime(*base_queue_.front())) > opt_.max_tdiff_s) {
        base_queue_.pop_front();
      }
    }

    auto it_rover = rover_queue_.begin();
    while (it_rover != rover_queue_.end()) {
      const ObsMsg::ConstSharedPtr& rover_msg = *it_rover;
      const gtime_t rover_time = msgTime(*rover_msg);

      ObsMsg::ConstSharedPtr exact_base = nullptr;
      for (const auto& base_msg : base_queue_) {
        if (std::fabs(timediff(msgTime(*base_msg), rover_time)) < opt_.match_tol_s) {
          exact_base = base_msg;
          break;
        }
      }
      if (exact_base) {
        out.push_back({rover_msg, exact_base,
                       timediff(rover_time, msgTime(*exact_base))});
        it_rover = rover_queue_.erase(it_rover);
        continue;
      }

      ObsMsg::ConstSharedPtr best_base = nullptr;
      double best_age = 1e9;
      for (const auto& base_msg : base_queue_) {
        const double age = timediff(rover_time, msgTime(*base_msg));
        if (age >= -opt_.match_tol_s && age < best_age) {
          best_age = age;
          best_base = base_msg;
        }
      }

      const bool is_newest_rover = (std::next(it_rover) == rover_queue_.end());

      if (best_base && best_age <= opt_.max_tdiff_s) {
        bool should_wait = is_newest_rover;
        if (!should_wait && estimated_base_interval_ > 0.0) {
          if (best_age >= estimated_base_interval_ - 0.01) {
            const double queue_span =
                timediff(msgTime(*rover_queue_.back()), rover_time);
            if (queue_span <= estimated_base_interval_ + 0.1) should_wait = true;
          }
        }
        if (should_wait) {
          ++it_rover;
        } else {
          out.push_back({rover_msg, best_base, best_age});
          it_rover = rover_queue_.erase(it_rover);
        }
        continue;
      }

      if (!base_queue_.empty()) {
        const gtime_t newest_base = msgTime(*base_queue_.back());
        if (timediff(newest_base, rover_time) > opt_.drop_ahead_s) {
          dropped_.push_back({rover_msg->tow, base_queue_.back()->tow});
          it_rover = rover_queue_.erase(it_rover);
          continue;
        }
      }

      ++it_rover;
    }
    return out;
  }

  // Rover epochs dropped since the last call (cleared on return).
  std::vector<Dropped> takeDropped() {
    std::vector<Dropped> out;
    out.swap(dropped_);
    return out;
  }

  double estimatedBaseInterval() const { return estimated_base_interval_; }

 private:
  static gtime_t msgTime(const ObsMsg& m) {
    return gpst2time(static_cast<int>(m.week), m.tow);
  }

  Options opt_;
  std::deque<ObsMsg::ConstSharedPtr> rover_queue_;
  std::deque<ObsMsg::ConstSharedPtr> base_queue_;
  std::vector<Dropped> dropped_;
  double estimated_base_interval_{0.0};
};

}  // namespace gnss_utils

#endif  // GNSS_ROS_STANDARDIZATION_EPOCH_MATCHER_HPP
