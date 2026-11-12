#pragma once
/// @file ins_utils.hpp
/// @brief Covariance and timing helpers for sensor_msgs/Imu used by IMU
///        decoder/driver nodes.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/time.hpp>

namespace gnss_ros_standardization {
namespace ins_utils {

/// Return a 3×3 row-major covariance array indicating the MEASUREMENT ITSELF
/// is not provided (-1 at [0,0]), per sensor_msgs/Imu semantics. Use this only
/// when the field has no data (e.g. orientation is not estimated). If the
/// field DOES have a value but its covariance is simply unknown, use
/// makeZeroCovariance() instead — conflating the two misreports "no data" as
/// "data with a real (if unknown) uncertainty".
inline std::array<double, 9> makeUnknownCovariance()
{
    std::array<double, 9> cov{};
    cov[0] = -1.0;
    return cov;
}

/// Return an all-zero 3×3 covariance array: per sensor_msgs/Imu semantics,
/// this means "the measurement IS provided but its covariance is unknown"
/// (as opposed to makeUnknownCovariance()'s -1, which means the field is not
/// provided at all).
inline std::array<double, 9> makeZeroCovariance()
{
    return std::array<double, 9>{};
}

/// Return a 3×3 diagonal covariance array from per-axis standard deviations.
/// Off-diagonal terms are zero (no cross-correlation assumed).
inline std::array<double, 9> makeDiagCovariance(
    double sigma_x, double sigma_y, double sigma_z)
{
    std::array<double, 9> cov{};
    cov[0] = sigma_x * sigma_x;
    cov[4] = sigma_y * sigma_y;
    cov[8] = sigma_z * sigma_z;
    return cov;
}


/// Map a free-running receiver sample counter onto a monotone ROS timeline.
///
///     t_ros(c) = tau(c) + b,   tau advances by (counter delta) * tick_s * rate
///     rate     -> measured counter rate / nominal rate (see below)
///     b        -> sliding-window minimum of (arrival - tau), approached under
///                 a slew limit.
///
/// THE COUNTER RATE IS MEASURED, NOT ASSUMED. On a real ZED-F9R the sTtag
/// counter runs on the IMU's own oscillator: one unit measured 38.7447 us per
/// tick against the nominal 39.0625 us (-8135 ppm, stable to +-15 ppm over
/// 10 min). With a fixed tick the counter time runs 0.8 % fast, b can follow
/// only max_slew of it, and the stamps ran ahead of real time by 7.9 ms every
/// second (4 s after 9 min). The rate is therefore fitted to the arrival
/// lower envelope - the minima of (arrival - counter * tick_s) in rate_bin_s
/// bins over rate_window_s, Theil-Sen slope so a startup backlog or a few
/// stalled bins cannot bias it - and accepted within max_rate_error. A larger
/// rate difference is not an oscillator: the stream is not paced by real time
/// (file:// replay of a log), and tick_s is used as is.
///
/// Why a counter at all: u-blox UBX-ESF-RAW carries only `sTtag`, a receiver
/// sample counter with no GNSS time and - per the ZED-F9R Interface
/// Description, which leaves both Scale and Unit blank - no documented unit
/// either. gpsd's ubxtool notes independently that "the UBX-ESF-RAW sTtag is
/// in unrelated units, but at least always increases". That monotonicity is
/// the whole basis of this class: the counter is a good clock of unknown
/// offset, while host arrival time is a bad clock of known offset. Stamping
/// from arrival (the previous behavior) quantizes the IMU timeline to the
/// poll period and USB burst structure, and emits out-of-order stamps
/// whenever the arrival gap between two frames is shorter than the span the
/// second frame covers.
///
/// The offset estimator is the one already proven in
/// gnss_utils::GnssEpochAligner: the per-sample offset d_k = arrival - c*T is
/// a constant (clock difference) plus a strictly non-negative, jittery
/// transport delay, so the sliding-window MINIMUM of d_k is the best
/// latency-free estimate of the constant.
///
/// THE SLEW LIMIT IS NOT OPTIONAL. Taking that minimum directly is NOT
/// monotone at IMU rates:
///
///     t_k - t_{k-1} = (c_k - c_{k-1})*T - (b_{k-1} - b_k)
///
/// so any drop of the latency floor larger than the inter-sample interval
/// emits a backwards stamp. At 100 Hz that interval is 10 ms - exactly the
/// poll quantization this class exists to remove - so the naive version
/// would reintroduce the very defect it fixes, in a new form. Constraining
/// |db| <= max_slew * (elapsed receiver time) with max_slew < 1 makes the
/// output strictly increasing by construction. GnssEpochAligner gets away
/// without it only because its epochs are 1 s apart while its latency steps
/// are ~100 ms.
///
/// ARRIVAL IS ASSUMED NON-DECREASING. Callers obtain it by calling now() in
/// read order, so a live stream cannot produce an earlier arrival than the
/// one before. That bounds how fast the transport latency can fall - at most
/// one sample interval per sample - which is why a latency floor collapses
/// through a stall-then-burst, not through a step. A genuinely backwards
/// arrival therefore means the stream restarted (bag replay, or a host clock
/// step), and is handled by re-seeding.
///
/// Monotonicity of the output holds within a seeded run. A re-seed
/// (backwards arrival, receiver counter reset, corrupt tag, host clock step)
/// deliberately breaks it: after a bag replay restart the timeline must
/// follow the new time rather than the old one, and downstream has to reset
/// regardless. Every re-seed sets `warn`, so a caller that cares can see it.
class SensorTickClock {
 public:
  struct Config {
    /// Counter LSB [s]. 0 disables the mapping: every stamp equals `arrival`,
    /// restoring the legacy behavior without a branch at the call site.
    double tick_s{0.0};
    /// Counter wrap point. 2^32 for a u4 sTtag; 604800000 for iTOW in ms.
    /// Not cosmetic: an iTOW week rollover steps by -604800000, which fits in
    /// int32_t and so would be misread as a receiver reset by generic 32-bit
    /// unwrap logic.
    std::uint64_t modulus{1ull << 32};
    /// Sliding window for the minimum-latency floor [s]. Shorter than the
    /// aligner's 60 s because the window length is what absorbs clock drift:
    /// at 100 Hz, 20 s is still 2000 samples to find the floor with, while
    /// cutting the drift-staleness bias threefold.
    double window_s{20.0};
    /// Steady-state slew limit for b [s per s of receiver time].
    double max_slew{2e-4};
    /// Faster slew until the window has filled once, so a cold start
    /// converges in seconds instead of minutes. Still < 1, so still monotone.
    ///
    /// This is a direct trade against dt fidelity: every change to b is taken
    /// out of one interval, so warmup_slew IS the worst-case relative dt
    /// error during convergence. At 0.02 a 10 ms interval is distorted by at
    /// most 200 us while an initial offset error of 50 ms is absorbed in
    /// ~250 samples (2.5 s at 100 Hz). The integrated correction is the same
    /// whatever the rate; only the per-sample distortion differs, and a small
    /// one is far less likely to trip a downstream gap check.
    double warmup_slew{0.02};
    /// A one-sample STEP in (t_ros - arrival) beyond this re-seeds. Catches
    /// an NTP step on the host clock, which max_slew would otherwise take
    /// hours to absorb.
    ///
    /// Deliberately a step and not an accumulated difference: a SUSTAINED
    /// rate difference between the counter and the host clock is not a fault.
    /// Replaying a recorded UBX log through file:// - offline reprocessing,
    /// a normal workflow - feeds 120 s of receiver time in a second or two,
    /// so (t_ros - arrival) ramps without bound while nothing is wrong. An
    /// accumulated-difference guard resets continuously there, which is how
    /// this was found: a file:// replay showed periodic -1 s jumps. A step
    /// guard distinguishes the two, and the ramp costs nothing because the
    /// output follows the counter, which is the clock worth following.
    double max_divergence_s{1.0};
    /// A backwards counter step at least this large means the receiver reset
    /// its counter rather than delivering something slightly out of order.
    std::int64_t max_backstep_ticks{256};
    /// A forward counter jump beyond this means a corrupt tag, or an outage
    /// long enough that the old offset estimate is not worth keeping.
    double max_forward_gap_s{300.0};
    /// Measure the counter rate (see the class note); false trusts tick_s.
    bool estimate_rate{true};
    /// Bin width of the arrival lower envelope [s of counter time].
    double rate_bin_s{2.0};
    /// Span of bins the rate is fitted over [s of counter time].
    double rate_window_s{60.0};
    /// Closed bins needed before the first rate estimate.
    int rate_min_bins{10};
    /// Largest accepted |measured / nominal rate - 1|. Sensor oscillators are
    /// within about 1 %; a file:// replay is off by orders of magnitude.
    double max_rate_error{0.03};
  };

  /// Pending: not enough data yet. Tracking: measured rate applied.
  /// Unpaced: the stream does not follow real time; tick_s is used as is.
  enum class RateState { Pending, Tracking, Unpaced };

  SensorTickClock() = default;
  explicit SensorTickClock(const Config& cfg) : cfg_(cfg) {}

  void configure(const Config& cfg) { cfg_ = cfg; reset(); }
  const Config& config() const { return cfg_; }

  /// Map one raw counter value observed at `arrival` onto the output
  /// timeline. Guarantees, while `tick_s > 0`: strictly increasing across
  /// calls, and spacing equal to the counter delta to within the slew limit.
  ///
  /// On any re-seed or refusal `warn` (when non-null) is set to a reason
  /// string and the returned time equals `arrival`; callers throttle-log it.
  /// The result carries `arrival`'s clock type - mixing clock types in
  /// rclcpp::Time arithmetic throws, which is why all of this is done in
  /// plain seconds and converted back only at the end.
  rclcpp::Time stamp(std::uint64_t raw_counter, const rclcpp::Time& arrival,
                     std::string* warn = nullptr) {
    if (warn) warn->clear();
    const double arrival_s = arrival.nanoseconds() / 1e9;
    if (!(cfg_.tick_s > 0.0)) return arrival;

    // A backwards arrival stamp means the host clock or the stream restarted
    // (bag replay). Same reasoning as GnssEpochAligner clearing its deque,
    // except here the persistent offset and unwrap state must go too.
    if (seeded_ && arrival_s < last_arrival_s_) {
      reset();
      if (warn) *warn = "arrival stamp went backwards - re-seeding";
    }

    if (!seeded_) {
      seed(raw_counter, arrival_s);
      return toTime(last_t_ros_, arrival);
    }

    const std::int64_t d = signedDelta(raw_counter);
    const double gap_s = static_cast<double>(d) * cfg_.tick_s;

    if (d <= -cfg_.max_backstep_ticks) {
      reset();
      seed(raw_counter, arrival_s);
      if (warn) *warn = "counter stepped backwards - receiver reset assumed";
      return toTime(last_t_ros_, arrival);
    }
    if (gap_s > cfg_.max_forward_gap_s) {
      reset();
      seed(raw_counter, arrival_s);
      if (warn) *warn = "counter jumped forward - corrupt tag or long outage";
      return toTime(last_t_ros_, arrival);
    }

    if (d == 0) {
      // Same sample instant split across frames: re-emit, do not advance.
      last_arrival_s_ = arrival_s;
      return toTime(last_t_ros_, arrival);
    }
    if (d < 0) {
      // Small backstep. parseEsfRawSamples delivers stream order, so this
      // should not happen; clamp rather than corrupt the timeline.
      if (warn) *warn = "counter stepped back slightly - clamped";
      last_arrival_s_ = arrival_s;
      return toTime(last_t_ros_, arrival);
    }

    last_raw_ = raw_counter;
    unwrapped_ += d;
    const double nominal_s = static_cast<double>(unwrapped_) * cfg_.tick_s;
    if (cfg_.estimate_rate) updateRate(nominal_s, arrival_s);
    tau_ += static_cast<double>(d) * cfg_.tick_s * rate_;
    const double counter_s = tau_;

    // Sliding-window minimum of the offset, over receiver time so the window
    // does not stretch when arrivals are bursty.
    const double d_k = arrival_s - counter_s;
    if (!floor_.empty() && counter_s < floor_.back().first) floor_.clear();
    floor_.emplace_back(counter_s, d_k);
    // size() > 1, not !empty(): the sample just pushed has age 0, so a
    // non-negative window stops the loop on its own - but window_s is a
    // plain parameter with no validation, and a negative value would empty
    // the deque and leave front() below dereferencing nothing. Keeping the
    // newest sample makes that impossible whatever the configuration says.
    while (floor_.size() > 1 &&
           counter_s - floor_.front().first > cfg_.window_s) {
      floor_.pop_front();
      window_filled_ = true;
    }
    double target = floor_.front().second;
    for (const auto& s : floor_) target = std::min(target, s.second);

    // Slew-limited approach to the target. This is what keeps the output
    // monotone; see the class note. The fast warm-up slew lasts until the
    // rate is settled too: whatever offset error built up while the rate was
    // still unknown must be absorbed in seconds, not at max_slew.
    const bool rate_settled = !cfg_.estimate_rate || rate_state_ == RateState::Unpaced ||
        (rate_state_ == RateState::Tracking && counter_s - tracking_since_ >= cfg_.window_s);
    const double slew = window_filled_ && rate_settled ? cfg_.max_slew : cfg_.warmup_slew;
    const double max_step = slew * gap_s;
    const double want = target - b_;
    b_ += std::max(-max_step, std::min(max_step, want));

    double t = counter_s + b_;

    // Host-clock step (NTP): the estimate can no longer be reconciled by
    // slewing, so re-seed rather than strand the stream for hours. Measured
    // as a ONE-SAMPLE step so that a sustained counter/host rate difference
    // (a fast file:// replay) is not mistaken for a fault - see
    // Config::max_divergence_s.
    const double skew = t - arrival_s;
    if (std::fabs(skew - last_skew_) > cfg_.max_divergence_s) {
      reset();
      seed(raw_counter, arrival_s);
      if (warn) *warn = "arrival stepped relative to the counter - host clock step assumed";
      return toTime(last_t_ros_, arrival);
    }
    last_skew_ = skew;

    // Monotonicity is guaranteed by the slew limit; this is a cheap assertion
    // of the contract for the degenerate configs (max_slew >= 1) rather than
    // a load-bearing correction.
    if (t <= last_t_ros_) t = last_t_ros_ + cfg_.tick_s;

    last_t_ros_ = t;
    last_arrival_s_ = arrival_s;
    return toTime(t, arrival);
  }

  /// Current offset estimate [s].
  double offset() const { return b_; }

  /// Last measured (true tick / tick_s - 1), whether or not it was accepted;
  /// 0 while the rate is still Pending.
  double driftRate() const { return measured_rate_error_; }
  RateState rateState() const { return rate_state_; }
  /// Counter tick currently applied [s].
  double appliedTick() const { return cfg_.tick_s * rate_; }

  bool seeded() const { return seeded_; }
  bool windowFilled() const { return window_filled_; }

  void reset() {
    seeded_ = false;
    window_filled_ = false;
    last_raw_ = 0;
    unwrapped_ = 0;
    b_ = 0.0;
    last_t_ros_ = -std::numeric_limits<double>::infinity();
    last_arrival_s_ = -std::numeric_limits<double>::infinity();
    last_skew_ = 0.0;
    floor_.clear();
    resetRate();
  }

 private:
  struct RateBin { double start, x, y; };  // y = min(arrival - nominal counter s) at x

  void resetRate() {
    tau_ = 0.0;
    rate_ = 1.0;
    rate_state_ = RateState::Pending;
    measured_rate_error_ = 0.0;
    tracking_since_ = 0.0;
    bins_.clear();
  }

  /// Lower-envelope rate fit; see the class note. Re-fitted when a bin closes.
  void updateRate(double nominal_s, double arrival_s) {
    const double y = arrival_s - nominal_s;
    bool closed = false;
    if (bins_.empty() || nominal_s - bins_.back().start >= cfg_.rate_bin_s) {
      closed = !bins_.empty();
      bins_.push_back({nominal_s, nominal_s, y});
    } else if (y < bins_.back().y) {
      bins_.back().x = nominal_s;
      bins_.back().y = y;
    }
    while (bins_.size() > 1 && nominal_s - bins_.front().start > cfg_.rate_window_s)
      bins_.pop_front();
    const size_t n = bins_.size() - 1;  // closed bins; the last one is still open
    if (!closed || static_cast<int>(n) < std::max(2, cfg_.rate_min_bins)) return;
    std::vector<double> slopes;
    slopes.reserve(n * (n - 1) / 2);
    for (size_t i = 0; i < n; ++i)
      for (size_t j = i + 1; j < n; ++j)
        if (bins_[j].x > bins_[i].x)
          slopes.push_back((bins_[j].y - bins_[i].y) / (bins_[j].x - bins_[i].x));
    if (slopes.empty()) return;
    auto mid = slopes.begin() + slopes.size() / 2;
    std::nth_element(slopes.begin(), mid, slopes.end());
    const double error = *mid;
    if (!std::isfinite(error)) return;
    measured_rate_error_ = error;
    if (std::fabs(error) <= cfg_.max_rate_error) {
      if (rate_state_ != RateState::Tracking) tracking_since_ = tau_;
      rate_state_ = RateState::Tracking;
      rate_ = 1.0 + error;
    } else {
      rate_state_ = RateState::Unpaced;
      rate_ = 1.0;
    }
  }

  void seed(std::uint64_t raw, double arrival_s) {
    seeded_ = true;
    window_filled_ = false;
    last_raw_ = raw;
    unwrapped_ = 0;
    // Seed the offset exactly, so the FIRST stamp equals the arrival time -
    // bit-identical to the legacy behavior. Never buffer to fill a window:
    // that would add latency to a real-time topic and, once recorded, make a
    // bag replay deliver a burst that never happened on the wire.
    b_ = arrival_s;
    last_t_ros_ = arrival_s;
    last_arrival_s_ = arrival_s;
    last_skew_ = 0.0;  // the seed puts t exactly on arrival
    floor_.clear();
    floor_.emplace_back(0.0, arrival_s);
    resetRate();
    if (cfg_.estimate_rate) updateRate(0.0, arrival_s);
  }

  /// Counter delta in (-modulus/2, +modulus/2], so a legitimate wrap reads as
  /// a small positive step. Correct for both the 2^32 sTtag wrap and the
  /// 604800000 ms iTOW rollover; a plain int32_t cast is correct only for the
  /// former.
  std::int64_t signedDelta(std::uint64_t raw) {
    const std::int64_t m = static_cast<std::int64_t>(cfg_.modulus);
    std::int64_t d = static_cast<std::int64_t>(raw) -
                     static_cast<std::int64_t>(last_raw_);
    if (m > 0) {
      if (d >  m / 2) d -= m;
      if (d < -m / 2) d += m;
    }
    return d;
  }

  static rclcpp::Time toTime(double t_s, const rclcpp::Time& like) {
    return rclcpp::Time(static_cast<std::int64_t>(t_s * 1e9),
                        like.get_clock_type());
  }

  Config cfg_;
  bool   seeded_{false};
  bool   window_filled_{false};
  std::uint64_t last_raw_{0};
  std::int64_t  unwrapped_{0};
  double b_{0.0};
  double last_t_ros_{-std::numeric_limits<double>::infinity()};
  double last_arrival_s_{-std::numeric_limits<double>::infinity()};
  double last_skew_{0.0};
  std::deque<std::pair<double, double>> floor_;
  double tau_{0.0};       // counter time at the applied rate [s]
  double rate_{1.0};      // applied tick / tick_s
  RateState rate_state_{RateState::Pending};
  double measured_rate_error_{0.0};
  double tracking_since_{0.0};
  std::deque<RateBin> bins_;
};

// Smooth anchor refinements without introducing timestamp inversions. An
// explicit reset is required at a clock/session discontinuity.
class SlewedTimeOffset {
 public:
  void reset() { seeded_ = false; }
  rclcpp::Time map(const rclcpp::Time& input, double target_offset_s) {
    const int64_t ns = input.nanoseconds();
    if (!seeded_) { offset_s_ = target_offset_s; seeded_ = true; }
    else {
      const double dt = std::max(0.0, (ns - last_ns_) * 1e-9);
      offset_s_ += std::clamp(target_offset_s - offset_s_, -0.02*dt, 0.02*dt);
    }
    last_ns_ = ns;
    return rclcpp::Time(ns - static_cast<int64_t>(offset_s_*1e9), input.get_clock_type());
  }
 private:
  bool seeded_{false};
  int64_t last_ns_{0};
  double offset_s_{0};
};

/// Host-clock minus GNSS-time offset for placing an IMU timeline on GNSS time.
///
/// Each GNSS epoch gives y = host arrival - GNSS time, which is the clock
/// difference plus that epoch's output latency. The offset is the MEDIAN of the
/// per-bin minima of y over the window, because a plain window minimum is held
/// for the whole window by one early epoch.
///
/// Which epochs: on a ZED-F9R an RXM-RAWX leaves the receiver about 30 ms earlier
/// when no navigation solution is computed for that epoch, and how often that
/// happens depends on the navigation state (sensor fusion lowers the NAV-PVT
/// rate). Only RXM-RAWX epochs that have a NAV-PVT are therefore used, so the
/// reference does not move with the navigation rate. RXM-RAWX comes first, so
/// it waits here for its NAV-PVT. Without one of the two message types, the
/// other is used alone.
class GnssTimeAnchor {
 public:
  enum class Source { Observation, Solution };

  void reset() {
    observation_seen_ = false;
    solution_seen_ = false;
    pending_valid_ = false;
    valid_ = false;
    offset_s_ = 0.0;
    last_arrival_s_ = 0.0;
    last_gnss_s_ = 0.0;
    last_solution_ns_ = -1;
    bins_.clear();
  }

  /// Times are nanoseconds on the host clock and on the GNSS-derived timeline.
  void update(Source source, std::int64_t arrival_ns, std::int64_t gnss_ns) {
    if (source == Source::Observation) {
      if (!observation_seen_) {
        observation_seen_ = true;
        bins_.clear();
      }
      if (!solution_seen_ || sameEpoch(last_solution_ns_, gnss_ns)) {
        accept(arrival_ns, gnss_ns);
        pending_valid_ = false;
      } else {
        pending_arrival_ns_ = arrival_ns;
        pending_gnss_ns_ = gnss_ns;
        pending_valid_ = true;
      }
      return;
    }
    if (!observation_seen_) {
      solution_seen_ = true;
      accept(arrival_ns, gnss_ns);
      return;
    }
    if (!solution_seen_) {
      solution_seen_ = true;
      bins_.clear();
    }
    last_solution_ns_ = gnss_ns;
    if (pending_valid_ && sameEpoch(pending_gnss_ns_, gnss_ns)) {
      accept(pending_arrival_ns_, pending_gnss_ns_);
      pending_valid_ = false;
    }
  }

  double offset() const { return offset_s_; }
  bool valid() const { return valid_; }

 private:
  static constexpr double kWindowS = 60.0;
  static constexpr double kBinS = 2.0;
  static constexpr double kDownStepS = 0.5;
  static constexpr double kRewindS = 5.0;
  static constexpr std::int64_t kSameEpochNs = 50000000;  // half a 10 Hz epoch
  struct Bin { double start, min; };

  static bool sameEpoch(std::int64_t a_ns, std::int64_t b_ns) {
    return a_ns >= 0 && std::llabs(a_ns - b_ns) < kSameEpochNs;
  }

  void accept(std::int64_t arrival_ns, std::int64_t gnss_ns) {
    const double arrival_s = arrival_ns * 1e-9;
    const double gnss_s = gnss_ns * 1e-9;
    const double y = (arrival_ns - gnss_ns) * 1e-9;
    // Stream restart, or a downward step no latency can explain: a host clock
    // step, or a log replayed faster than real time (y then falls continuously
    // and the offset has to stay with it). A rise is left to the window: it can
    // also be a stalled delivery, which must not move the offset.
    if (valid_ && (arrival_s < last_arrival_s_ || gnss_s < last_gnss_s_ - kRewindS ||
                   y < offset_s_ - kDownStepS))
      bins_.clear();
    last_arrival_s_ = arrival_s;
    last_gnss_s_ = gnss_s;

    if (bins_.empty() || arrival_s - bins_.back().start >= kBinS)
      bins_.push_back({arrival_s, y});
    else
      bins_.back().min = std::min(bins_.back().min, y);
    while (bins_.size() > 1 && arrival_s - bins_.front().start > kWindowS) bins_.pop_front();

    std::vector<double> minima;
    minima.reserve(bins_.size());
    for (const auto& bin : bins_) minima.push_back(bin.min);
    std::sort(minima.begin(), minima.end());
    const size_t n = minima.size();
    offset_s_ = n % 2 ? minima[n / 2] : 0.5 * (minima[n / 2 - 1] + minima[n / 2]);
    valid_ = true;
  }

  bool observation_seen_{false};
  bool solution_seen_{false};
  bool pending_valid_{false};
  bool valid_{false};
  double offset_s_{0.0};
  double last_arrival_s_{0.0};
  double last_gnss_s_{0.0};
  std::int64_t last_solution_ns_{-1};
  std::int64_t pending_arrival_ns_{0};
  std::int64_t pending_gnss_ns_{0};
  std::deque<Bin> bins_;
};

/// Is the receiver stream arriving in real time, or is a recorded log being
/// replayed through it? file:// reads and the launch playback (which reaches the
/// decoder as tcpsvr://) both deliver at the speed the node reads, so this is
/// decided from the data: seconds of IMU counter against seconds of arrival.
/// Latched once decided. The receiver's transmit buffer holds well under a
/// second of output, so a live stream cannot deliver 2 s of samples in 1 s.
class StreamPacing {
 public:
  enum class State { Unknown, Live, Replay };

  void configure(double tick_s) { tick_s_ = tick_s; }
  State state() const { return state_; }

  State update(std::uint32_t counter, double arrival_s) {
    if (state_ != State::Unknown) return state_;
    if (!started_) { restart(counter, arrival_s); return state_; }
    const auto delta = static_cast<std::int32_t>(counter - last_counter_);
    // A counter step back means the receiver restarted its counter.
    if (delta <= 0) { if (delta < 0) restart(counter, arrival_s); return state_; }
    last_counter_ = counter;
    elapsed_ticks_ += delta;
    const double counter_s = static_cast<double>(elapsed_ticks_) * tick_s_;
    if (counter_s >= kDecideAfterS)
      state_ = arrival_s - start_arrival_s_ < kReplayRatio * counter_s ? State::Replay : State::Live;
    return state_;
  }

 private:
  static constexpr double kDecideAfterS = 2.0;
  static constexpr double kReplayRatio = 0.5;

  void restart(std::uint32_t counter, double arrival_s) {
    started_ = true;
    last_counter_ = counter;
    elapsed_ticks_ = 0;
    start_arrival_s_ = arrival_s;
  }

  State state_{State::Unknown};
  double tick_s_{39.0625e-6};
  bool started_{false};
  std::uint32_t last_counter_{0};
  std::int64_t elapsed_ticks_{0};
  double start_arrival_s_{0.0};
};

/// Log line when the counter-rate state changes (once per state), else "".
inline std::string tickRateReport(const SensorTickClock& clock,
                                  SensorTickClock::RateState& reported) {
  using State = SensorTickClock::RateState;
  const State state = clock.rateState();
  if (state == State::Pending || state == reported) return {};
  reported = state;
  char line[256];
  if (state == State::Unpaced)
    std::snprintf(line, sizeof(line),
        "ESF-RAW stream is not real-time paced (%.0f %% off wall speed): offline replay "
        "assumed; using imu.sttag_tick_s = %.9g s", clock.driftRate() * 100.0,
        clock.config().tick_s);
  else
    std::snprintf(line, sizeof(line),
        "ESF-RAW sTtag tick measured %.6f us (%+.0f ppm vs imu.sttag_tick_s), applied",
        clock.appliedTick() * 1e6, clock.driftRate() * 1e6);
  return line;
}

/// Stamp a whole batch of samples that arrived together. `samples` must be in
/// stream order (ubx::esf_raw::parseEsfRawSamples guarantees this);
/// `get_counter` extracts the raw counter from one sample. Returns one stamp
/// per sample, in the same order.
template <class SampleT, class GetCounter>
inline std::vector<rclcpp::Time> stampBatch(SensorTickClock& clock,
                                            const std::vector<SampleT>& samples,
                                            GetCounter get_counter,
                                            const rclcpp::Time& arrival,
                                            std::string* warn = nullptr) {
  std::vector<rclcpp::Time> out;
  out.reserve(samples.size());
  std::string one;
  for (const auto& s : samples) {
    out.push_back(clock.stamp(get_counter(s), arrival, warn ? &one : nullptr));
    if (warn && warn->empty() && !one.empty()) *warn = one;
  }
  return out;
}

}  // namespace ins_utils
}  // namespace gnss_ros_standardization