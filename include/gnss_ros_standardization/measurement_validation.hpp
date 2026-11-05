#pragma once
#include <algorithm>
#include <cmath>

namespace gnss_ros_standardization::validation {
// All principal minors of a symmetric 3x3 covariance must be nonnegative.
// Normalize first to keep the tolerance relative and avoid overflow. The small
// tolerance covers float32 rounding at a semidefinite boundary.
inline bool covariance(double xx, double yy, double zz, double xy, double xz, double yz) {
  for (double v : {xx, yy, zz, xy, xz, yz}) if (!std::isfinite(v)) return false;
  if (xx < 0 || yy < 0 || zz < 0) return false;
  const double scale = std::max({xx, yy, zz, std::abs(xy), std::abs(xz), std::abs(yz)});
  if (scale == 0) return true;
  xx /= scale; yy /= scale; zz /= scale;
  xy /= scale; xz /= scale; yz /= scale;
  constexpr double tolerance = 1e-7;
  if ((xx == 0 && (xy != 0 || xz != 0)) || (yy == 0 && (xy != 0 || yz != 0)) ||
      (zz == 0 && (xz != 0 || yz != 0))) return false;
  return xx*yy - xy*xy >= -tolerance && xx*zz - xz*xz >= -tolerance &&
         yy*zz - yz*yz >= -tolerance &&
         xx*yy*zz + 2*xy*xz*yz - xx*yz*yz - yy*xz*xz - zz*xy*xy >= -tolerance;
}
} // namespace gnss_ros_standardization::validation

// ---- observed interval ----
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace gnss_utils {
// Pin the grid from the first ten positive finite measured spacings. A
// minority of missing samples does not bias the median. Zero means unknown.
class ObservedInterval {
public:
  void observe(double time) {
    if (!std::isfinite(time) || (have_previous_ && time <= previous_)) return;
    if (have_previous_ && interval_ == 0) {
      samples_.push_back(time - previous_);
      if (samples_.size() == 10) {
        std::nth_element(samples_.begin(), samples_.begin()+5, samples_.end());
        interval_ = samples_[5];
        samples_.clear();
      }
    }
    previous_ = time; have_previous_ = true;
  }
  double seconds() const { return interval_; }
  bool known() const { return interval_ > 0; }
  bool covers(double spacing) const {
    return known() && std::isfinite(spacing) && spacing >= 0 && spacing <= 2.5*interval_;
  }
  void reset() { *this = ObservedInterval{}; }
private:
  double previous_{0}, interval_{0};
  bool have_previous_{false};
  std::vector<double> samples_;
};
} // namespace gnss_utils
