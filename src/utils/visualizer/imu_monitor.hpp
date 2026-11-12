// Full-rate IMU analysis for the gnss_visualizer page. It uses no ROS types, so
// it can be tested alone. Every sample is processed; closeBin() is called once
// per page update (200 ms) and turns what arrived since the last call into one
// bin, so nothing here assumes a sensor rate.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <utility>
#include <vector>

namespace gnss_visualizer {

struct ImuSample {
  int64_t stamp_ns{0}, arrival_ns{0};
  std::array<double, 3> accel{};  // m/s^2
  std::array<double, 3> gyro{};   // rad/s
};

// Lowest and highest value of a bin. Not finite while empty.
struct ImuRange {
  double low{std::numeric_limits<double>::infinity()};
  double high{-std::numeric_limits<double>::infinity()};
  void add(double value) {
    low = std::min(low, value);
    high = std::max(high, value);
  }
};

struct ImuBin {
  uint64_t seq{0};
  double time_s{0.0};  // when the bin was closed, on the arrival clock
  int count{0};
  bool still{false};
  std::array<ImuRange, 3> accel;  // m/s^2
  std::array<ImuRange, 3> gyro;   // deg/s
  ImuRange dt_ms, latency_ms;  // latency: arrival minus stamp, on whatever clocks those are
};

struct ImuStatus {
  uint64_t samples{0}, gaps{0}, out_of_order{0}, duplicates{0}, rejected{0};
  double age_s{std::numeric_limits<double>::quiet_NaN()};
  double rate_hz{std::numeric_limits<double>::quiet_NaN()};
  double period_ms{std::numeric_limits<double>::quiet_NaN()};
  double max_gap_ms{std::numeric_limits<double>::quiet_NaN()};
  double imu_minus_observation_s{std::numeric_limits<double>::quiet_NaN()};
  bool still{false}, have_still{false}, gravity_free{false}, attitude_ready{false};
  bool frame_mismatch{false};             // gravity turns opposite to what right-handed axes predict
  // Values of the most recent still interval.
  double accel_norm{0.0};                 // m/s^2
  double accel_std{0.0}, gyro_std{0.0};   // m/s^2, deg/s; worst axis
  std::array<double, 3> gyro_bias{};      // deg/s
  std::array<double, 4> attitude{1.0, 0.0, 0.0, 0.0};  // w x y z, body to level; heading is arbitrary
  int up_axis{2}, up_sign{1};             // sensor axis closest to up
  double tilt_deg{0.0};                   // angle between that axis and up
};

class ImuMonitor {
 public:
  static constexpr double kGravity = 9.80665;
  static constexpr size_t kHistoryBins = 600;  // 120 s of 200 ms bins

  void addSample(const ImuSample& sample) {
    if (!finite(sample.accel) || !finite(sample.gyro)) {
      ++status_.rejected;
      return;
    }
    if (have_stamp_ && sample.stamp_ns - last_stamp_ns_ < -kRestartBackwardNs) {
      reset();
      restarted_ = true;
    }
    addTiming(sample);
    open_.count += 1;
    double norm2 = 0.0;
    for (size_t i = 0; i < 3; ++i) {
      open_.accel[i].add(sample.accel[i]);
      open_.gyro[i].add(sample.gyro[i] * kRadToDeg);
      norm2 += sample.accel[i] * sample.accel[i];
    }
    open_timing_.accel.add(sample.accel);
    open_timing_.gyro.add(sample.gyro);
    open_timing_.norm_sum += std::sqrt(norm2);
    updateAttitude(sample, std::sqrt(norm2));
    updateFrameCheck(sample);
  }

  void noteObservation(int64_t stamp_ns) {
    if (have_stamp_) status_.imu_minus_observation_s = static_cast<double>(last_stamp_ns_ - stamp_ns) * 1e-9;
  }

  void closeBin(int64_t now_ns) {
    open_.seq = next_seq_++;
    open_.time_s = static_cast<double>(now_ns) * 1e-9;
    recent_.push_back(open_timing_);
    if (recent_.size() > kRateBins) recent_.pop_front();
    updateRate();
    updateStillness();
    open_.still = status_.still;
    bins_.push_back(open_);
    if (bins_.size() > kHistoryBins) bins_.pop_front();
    updateStatus(now_ns);
    open_ = ImuBin{};
    open_timing_ = BinTiming{};
  }

  // Forgets everything except the bin numbering.
  void reset() {
    const auto rejected = status_.rejected;
    open_ = ImuBin{};
    open_timing_ = BinTiming{};
    bins_.clear();
    recent_.clear();
    status_ = ImuStatus{};
    status_.rejected = rejected;
    have_stamp_ = false;
    reference_gap_ns_ = 0.0;
    gyro_bias_rad_ = {};
    restartAttitude();
    frame_ = FrameWindow{};
    frame_votes_ = 0;
    frame_mirrored_ = 0.0;
  }

  // True once after the stamps jumped back and everything was reset, as when a bag restarts.
  bool takeRestarted() { return std::exchange(restarted_, false); }

  const std::deque<ImuBin>& bins() const { return bins_; }
  const ImuStatus& status() const { return status_; }

 private:
  static constexpr double kPi = 3.14159265358979323846;
  static constexpr double kRadToDeg = 180.0 / kPi;
  static constexpr int64_t kRestartBackwardNs = 1'000'000'000;
  static constexpr size_t kRateBins = 25;   // 5 s
  static constexpr size_t kStillBins = 5;   // 1 s
  static constexpr int kMinReferenceBins = 5;
  static constexpr double kGapFactor = 2.0;
  static constexpr double kBurstFactor = 0.25;
  // Same limits as gnss_fgo::leadingStillSamples, which cannot be included without GTSAM.
  static constexpr double kStillAccelStd = 0.15, kStillGyroStd = 0.01;
  static constexpr int kStillMinSamples = 10;
  static constexpr double kStillMinSpan = 0.8;
  // A steady turn also has little scatter, but it is not stillness.
  static constexpr double kStillMaxGyroMean = 0.05;
  // Stillness is known only per bin, so each sample must also agree with it.
  static constexpr double kStillMaxAccelDeviation = 3.0 * kStillAccelStd;
  static constexpr double kGravityFreeNorm = 0.5 * kGravity;
  static constexpr double kInitSpan = 0.5, kInitMinNorm = 0.7 * kGravity, kInitMaxNorm = 1.3 * kGravity;
  static constexpr int kInitMinSamples = 5;
  static constexpr double kMaxAttitudeGap = 0.5;
  static constexpr double kCorrectionMinNorm = 0.85 * kGravity, kCorrectionMaxNorm = 1.15 * kGravity;
  // Gyro bias offsets the tilt by bias / gain, and a bias is unknown until the
  // first still interval, so slow motion needs more than the moving gain.
  static constexpr double kStillGain = 2.0, kSlowGain = 0.5, kMovingGain = 0.05;
  static constexpr double kSlowMaxNormError = 0.2, kSlowMaxRate = 0.1;
  // Windows too short or turning too little cannot tell the two predictions apart
  // from vehicle or hand acceleration.
  static constexpr double kFrameWindow = 2.0, kFrameAverage = 0.1, kFrameMaxGap = 0.5;
  static constexpr double kFrameMaxAgreement = 0.9848;  // cos(10 deg) between the two predictions
  static constexpr int kFrameMemory = 20, kFrameMinVotes = 8;
  static constexpr double kFrameSetShare = 0.7, kFrameClearShare = 0.3;

  using Vector = std::array<double, 3>;

  // Count, mean and squared deviation per axis; adding two needs no samples.
  struct Moments {
    int n{0};
    Vector mean{}, m2{};
    void add(const Vector& v) {
      ++n;
      for (size_t i = 0; i < 3; ++i) {
        const double delta = v[i] - mean[i];
        mean[i] += delta / n;
        m2[i] += delta * (v[i] - mean[i]);
      }
    }
    void merge(const Moments& other) {
      if (other.n == 0) return;
      const int total = n + other.n;
      for (size_t i = 0; i < 3; ++i) {
        const double delta = other.mean[i] - mean[i];
        m2[i] += other.m2[i] + delta * delta * n * other.n / total;
        mean[i] += delta * other.n / total;
      }
      n = total;
    }
    double worstStd() const {
      if (n == 0) return std::numeric_limits<double>::quiet_NaN();
      return std::sqrt(*std::max_element(m2.begin(), m2.end()) / n);
    }
  };

  struct BinTiming {
    int samples{0}, intervals{0};
    int64_t first_ns{0}, last_ns{0};
    double dt_min_ns{std::numeric_limits<double>::infinity()};
    double dt_max_ns{-std::numeric_limits<double>::infinity()};
    Moments accel, gyro;
    double norm_sum{0.0};
  };

  struct FrameWindow {
    bool open{false}, turning{false};
    int64_t first_ns{0}, last_ns{0};
    Vector start_sum{}, end_sum{};
    int start_count{0}, end_count{0};
    Vector right{}, mirrored{};  // unit gravity propagated with the gyro and with its opposite
  };

  static bool finite(const Vector& v) {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
  }
  static double dot(const Vector& a, const Vector& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  static Vector cross(const Vector& a, const Vector& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
  }
  static double median(std::vector<double>& values) {
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2), values.end());
    return values[values.size() / 2];
  }

  void addTiming(const ImuSample& sample) {
    ++status_.samples;
    last_arrival_ns_ = sample.arrival_ns;
    open_.latency_ms.add(static_cast<double>(sample.arrival_ns - sample.stamp_ns) * 1e-6);
    if (open_timing_.samples++ == 0) open_timing_.first_ns = sample.stamp_ns;
    open_timing_.last_ns = sample.stamp_ns;
    if (have_stamp_) {
      const int64_t dt_ns = sample.stamp_ns - last_stamp_ns_;
      if (dt_ns < 0) ++status_.out_of_order;
      else if (dt_ns == 0) ++status_.duplicates;
      else if (reference_gap_ns_ > 0.0 && static_cast<double>(dt_ns) > kGapFactor * reference_gap_ns_) ++status_.gaps;
      const double dt = static_cast<double>(dt_ns);
      open_.dt_ms.add(dt * 1e-6);
      ++open_timing_.intervals;
      open_timing_.dt_min_ns = std::min(open_timing_.dt_min_ns, dt);
      open_timing_.dt_max_ns = std::max(open_timing_.dt_max_ns, dt);
    }
    last_stamp_ns_ = sample.stamp_ns;
    have_stamp_ = true;
  }

  // The normal spacing of the stamps. Receivers stamped with the host clock can
  // deliver samples in bursts: most intervals are then near zero and the spacing
  // of the bursts is normal, so it must not count as a gap.
  void updateRate() {
    int samples = 0;
    int64_t first = 0, last = 0;
    bool any = false;
    std::vector<double> minimums, maximums;
    for (const auto& bin : recent_) {
      if (bin.samples == 0) continue;
      samples += bin.samples;
      if (!any) first = bin.first_ns;
      last = bin.last_ns;
      any = true;
      if (bin.intervals > 0) {
        minimums.push_back(bin.dt_min_ns);
        maximums.push_back(bin.dt_max_ns);
      }
    }
    const double span = static_cast<double>(last - first);
    if (samples < 2 || span <= 0.0) {
      status_.rate_hz = status_.period_ms = std::numeric_limits<double>::quiet_NaN();
      reference_gap_ns_ = 0.0;
      return;
    }
    const double period_ns = span / (samples - 1);
    status_.rate_hz = 1e9 / period_ns;
    status_.period_ms = period_ns * 1e-6;
    if (static_cast<int>(minimums.size()) < kMinReferenceBins) {
      reference_gap_ns_ = 0.0;
      return;
    }
    reference_gap_ns_ = period_ns;
    if (median(minimums) < kBurstFactor * period_ns) reference_gap_ns_ = std::max(period_ns, median(maximums));
  }

  void updateStillness() {
    Moments accel, gyro;
    double norm_sum = 0.0;
    int64_t first = 0, last = 0;
    bool any = false;
    const size_t begin = recent_.size() > kStillBins ? recent_.size() - kStillBins : 0;
    for (size_t i = begin; i < recent_.size(); ++i) {
      const auto& bin = recent_[i];
      if (bin.samples == 0) continue;
      accel.merge(bin.accel);
      gyro.merge(bin.gyro);
      norm_sum += bin.norm_sum;
      if (!any) first = bin.first_ns;
      last = bin.last_ns;
      any = true;
    }
    status_.still = false;
    if (accel.n < kStillMinSamples) return;
    const double norm = norm_sum / accel.n;
    const bool gravity_free = norm < kGravityFreeNorm;
    if (gravity_free && !status_.gravity_free) restartAttitude();
    status_.gravity_free = gravity_free;
    const double span = static_cast<double>(last - first) * 1e-9;
    const double gyro_mean = std::sqrt(dot(gyro.mean, gyro.mean));
    // The accelerometer offset moves |a| by several tenths of m/s^2 depending on
    // how the sensor is placed, so |a| can only rule out what is clearly not gravity.
    const bool plausible_gravity = gravity_free || (norm > kCorrectionMinNorm && norm < kCorrectionMaxNorm);
    status_.still = span >= kStillMinSpan && accel.worstStd() < kStillAccelStd && gyro.worstStd() < kStillGyroStd &&
                    gyro_mean < kStillMaxGyroMean && plausible_gravity;
    if (!status_.still) return;
    status_.have_still = true;
    status_.accel_norm = norm;
    status_.accel_std = accel.worstStd();
    status_.gyro_std = gyro.worstStd() * kRadToDeg;
    gyro_bias_rad_ = gyro.mean;
    still_accel_ = accel.mean;
    for (size_t i = 0; i < 3; ++i) status_.gyro_bias[i] = gyro.mean[i] * kRadToDeg;
  }

  void updateStatus(int64_t now_ns) {
    status_.age_s = have_stamp_ ? static_cast<double>(now_ns - last_arrival_ns_) * 1e-9
                                : std::numeric_limits<double>::quiet_NaN();
    double max_gap = -std::numeric_limits<double>::infinity();
    for (const auto& bin : bins_) max_gap = std::max(max_gap, bin.dt_ms.high);
    status_.max_gap_ms = std::isfinite(max_gap) ? max_gap : std::numeric_limits<double>::quiet_NaN();
    status_.attitude_ready = attitude_ready_;
    if (!attitude_ready_) return;
    status_.attitude = attitude_;
    const Vector u = up();
    int axis = 0;
    for (int i = 1; i < 3; ++i)
      if (std::abs(u[static_cast<size_t>(i)]) > std::abs(u[static_cast<size_t>(axis)])) axis = i;
    const double along = u[static_cast<size_t>(axis)];
    status_.up_axis = axis;
    status_.up_sign = along < 0.0 ? -1 : 1;
    status_.tilt_deg = std::acos(std::min(1.0, std::abs(along))) * kRadToDeg;
  }

  void restartAttitude() {
    attitude_ready_ = false;
    init_count_ = 0;
    init_sum_ = {};
  }

  // Up expressed in sensor axes: the third row of the body-to-level rotation.
  Vector up() const {
    const auto& [w, x, y, z] = attitude_;
    return {2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)};
  }

  // Waits for a span of data whose mean looks like gravity. One sample could
  // be anything, and a sensor already moving would start from a wrong tilt.
  void collectInitialTilt(const ImuSample& sample) {
    if (init_count_ > 0 && (sample.stamp_ns <= init_last_ns_ ||
                            static_cast<double>(sample.stamp_ns - init_last_ns_) * 1e-9 > kMaxAttitudeGap)) {
      init_count_ = 0;
      init_sum_ = {};
    }
    if (init_count_ == 0) init_first_ns_ = sample.stamp_ns;
    ++init_count_;
    init_last_ns_ = sample.stamp_ns;
    for (size_t i = 0; i < 3; ++i) init_sum_[i] += sample.accel[i];
    if (static_cast<double>(sample.stamp_ns - init_first_ns_) * 1e-9 < kInitSpan || init_count_ < kInitMinSamples) return;
    Vector mean = init_sum_;
    for (auto& value : mean) value /= init_count_;
    const double norm = std::sqrt(dot(mean, mean));
    if (norm < kInitMinNorm || norm > kInitMaxNorm) {
      init_count_ = 0;
      init_sum_ = {};
      return;
    }
    const Vector u = {mean[0] / norm, mean[1] / norm, mean[2] / norm};
    // Shortest rotation taking u to the level frame's +Z; half a turn when mounted upside down.
    if (1.0 + u[2] < 1e-6) {
      attitude_ = {0.0, 1.0, 0.0, 0.0};
    } else {
      const double w = 1.0 + u[2], x = u[1], y = -u[0];
      const double length = std::sqrt(w * w + x * x + y * y);
      attitude_ = {w / length, x / length, y / length, 0.0};
    }
    attitude_ready_ = true;
    attitude_stamp_ns_ = sample.stamp_ns;
  }

  // A display-only complementary filter. The specific force pulls the tilt
  // back; heading is free and drifts. Vehicle acceleration also moves the
  // specific force, so the correction weakens as the motion gets stronger.
  void updateAttitude(const ImuSample& sample, double norm) {
    if (status_.gravity_free) return;
    if (!attitude_ready_) {
      collectInitialTilt(sample);
      return;
    }
    const double dt = static_cast<double>(sample.stamp_ns - attitude_stamp_ns_) * 1e-9;
    if (dt > kMaxAttitudeGap) {
      restartAttitude();
      collectInitialTilt(sample);
      return;
    }
    Vector rate = {sample.gyro[0] - gyro_bias_rad_[0], sample.gyro[1] - gyro_bias_rad_[1],
                   sample.gyro[2] - gyro_bias_rad_[2]};
    if (dt > 0.0) {
      attitude_stamp_ns_ = sample.stamp_ns;
      Vector corrected = rate;
      if (norm > kCorrectionMinNorm && norm < kCorrectionMaxNorm) {
        const Vector unit = {sample.accel[0] / norm, sample.accel[1] / norm, sample.accel[2] / norm};
        const Vector error = cross(unit, up());
        const double gain = correctionGain(sample, rate, norm);
        for (size_t i = 0; i < 3; ++i) corrected[i] += gain * error[i];
      }
      integrate(corrected, dt);
    }
  }

  static Vector scaled(const Vector& v, double factor) { return {v[0] * factor, v[1] * factor, v[2] * factor}; }
  static Vector unit(const Vector& v) { return scaled(v, 1.0 / std::sqrt(dot(v, v))); }
  static bool plausibleGravity(const Vector& mean) {
    const double norm = std::sqrt(dot(mean, mean));
    return norm > kCorrectionMinNorm && norm < kCorrectionMaxNorm;
  }

  // Tilting moves gravity in the sensor axes the way the gyro says only when the
  // axes are right-handed. With mirrored axes or a reversed gyro it moves the
  // opposite way. Each window votes for whichever prediction lands nearer the
  // accelerometer; rotation about gravity predicts no change and casts no vote.
  void updateFrameCheck(const ImuSample& sample) {
    if (status_.gravity_free) {
      frame_ = FrameWindow{};
      return;
    }
    const double gap = static_cast<double>(sample.stamp_ns - frame_.last_ns) * 1e-9;
    if (!frame_.open || gap <= 0.0 || gap > kFrameMaxGap) startFrameWindow(sample);
    double elapsed = static_cast<double>(sample.stamp_ns - frame_.first_ns) * 1e-9;
    if (elapsed >= kFrameWindow) {
      voteFrame();
      startFrameWindow(sample);
      elapsed = 0.0;
    }
    frame_.last_ns = sample.stamp_ns;
    if (elapsed < kFrameAverage) {
      for (size_t i = 0; i < 3; ++i) frame_.start_sum[i] += sample.accel[i];
      ++frame_.start_count;
    } else if (elapsed < kFrameWindow - kFrameAverage) {
      propagateFrame(sample, gap);
    } else {
      for (size_t i = 0; i < 3; ++i) frame_.end_sum[i] += sample.accel[i];
      ++frame_.end_count;
    }
  }

  void startFrameWindow(const ImuSample& sample) {
    frame_ = FrameWindow{};
    frame_.open = true;
    frame_.first_ns = sample.stamp_ns;
  }

  void propagateFrame(const ImuSample& sample, double dt) {
    if (!frame_.turning) {
      const Vector start = scaled(frame_.start_sum, 1.0 / frame_.start_count);
      if (!plausibleGravity(start)) return;
      frame_.right = frame_.mirrored = unit(start);
      frame_.turning = true;
    }
    const Vector angle = {(sample.gyro[0] - gyro_bias_rad_[0]) * dt, (sample.gyro[1] - gyro_bias_rad_[1]) * dt,
                          (sample.gyro[2] - gyro_bias_rad_[2]) * dt};
    const Vector right_step = cross(angle, frame_.right), mirrored_step = cross(angle, frame_.mirrored);
    frame_.right = unit({frame_.right[0] - right_step[0], frame_.right[1] - right_step[1], frame_.right[2] - right_step[2]});
    frame_.mirrored = unit({frame_.mirrored[0] + mirrored_step[0], frame_.mirrored[1] + mirrored_step[1],
                            frame_.mirrored[2] + mirrored_step[2]});
  }

  void voteFrame() {
    if (!frame_.turning || frame_.end_count == 0) return;
    const Vector end = scaled(frame_.end_sum, 1.0 / frame_.end_count);
    if (!plausibleGravity(end) || dot(frame_.right, frame_.mirrored) > kFrameMaxAgreement) return;
    const Vector measured = unit(end);
    const bool mirrored_nearer = dot(frame_.mirrored, measured) > dot(frame_.right, measured);
    frame_votes_ = std::min(frame_votes_ + 1, kFrameMemory);
    frame_mirrored_ += ((mirrored_nearer ? 1.0 : 0.0) - frame_mirrored_) / frame_votes_;
    if (frame_votes_ < kFrameMinVotes) return;
    if (frame_mirrored_ >= kFrameSetShare) status_.frame_mismatch = true;
    else if (frame_mirrored_ <= kFrameClearShare) status_.frame_mismatch = false;
  }

  double correctionGain(const ImuSample& sample, const Vector& rate, double norm) const {
    const double rate2 = dot(rate, rate);
    if (status_.still) {
      const Vector deviation = {sample.accel[0] - still_accel_[0], sample.accel[1] - still_accel_[1],
                                sample.accel[2] - still_accel_[2]};
      if (dot(deviation, deviation) < kStillMaxAccelDeviation * kStillMaxAccelDeviation &&
          rate2 < kStillMaxGyroMean * kStillMaxGyroMean)
        return kStillGain;
    }
    const bool slow = std::abs(norm - kGravity) < kSlowMaxNormError && rate2 < kSlowMaxRate * kSlowMaxRate;
    return slow ? kSlowGain : kMovingGain;
  }

  void integrate(const Vector& rate, double dt) {
    auto [w, x, y, z] = attitude_;
    const double h = 0.5 * dt;
    const std::array<double, 4> next = {
        w + h * (-x * rate[0] - y * rate[1] - z * rate[2]),
        x + h * (w * rate[0] + y * rate[2] - z * rate[1]),
        y + h * (w * rate[1] - x * rate[2] + z * rate[0]),
        z + h * (w * rate[2] + x * rate[1] - y * rate[0]),
    };
    const double length = std::sqrt(next[0] * next[0] + next[1] * next[1] + next[2] * next[2] + next[3] * next[3]);
    attitude_ = {next[0] / length, next[1] / length, next[2] / length, next[3] / length};
  }

  ImuBin open_;
  BinTiming open_timing_;
  std::deque<ImuBin> bins_;
  std::deque<BinTiming> recent_;
  ImuStatus status_;
  uint64_t next_seq_{0};
  bool restarted_{false};

  bool have_stamp_{false};
  int64_t last_stamp_ns_{0}, last_arrival_ns_{0};
  double reference_gap_ns_{0.0};

  bool attitude_ready_{false};
  std::array<double, 4> attitude_{1.0, 0.0, 0.0, 0.0};
  int64_t attitude_stamp_ns_{0};
  Vector gyro_bias_rad_{}, still_accel_{};
  int init_count_{0};
  Vector init_sum_{};
  int64_t init_first_ns_{0}, init_last_ns_{0};

  FrameWindow frame_;
  int frame_votes_{0};
  double frame_mirrored_{0.0};  // share of recent votes for the mirrored prediction
};

}  // namespace gnss_visualizer
