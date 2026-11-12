// Series statistics and differences following RTKPLOT (calcStats, TIMEPOS).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "plot_data.hpp"

namespace rtk_bag_plot {

inline constexpr double kMaxDifferentialTime = 60.0;  // RTKPLOT MAXTDIFF (s)
inline constexpr double kTimeTolerance = 0.025;       // RTKLIB DTTOL (s)

inline AxisStats computeAxisStats(const std::vector<double>& x) {
  AxisStats out;
  out.n = x.size();
  if (x.empty()) return out;
  double sum = 0.0, sum_sq = 0.0;
  for (const double v : x) {
    sum += v;
    sum_sq += v * v;
  }
  const double n = static_cast<double>(x.size());
  out.ave = sum / n;
  out.std = x.size() > 1
      ? std::sqrt(std::max(0.0, (sum_sq - 2.0 * sum * out.ave + out.ave * out.ave * n) / (n - 1.0)))
      : 0.0;
  out.rms = std::sqrt(sum_sq / n);
  return out;
}

// Statistics of all rows (index 0) and of each quality flag (1-7).
inline StatsTable computeStatsTable(const TimePos& series) {
  StatsTable table{};
  for (size_t q = 0; q < table.size(); ++q) {
    std::array<std::vector<double>, 3> axes;
    for (size_t i = 0; i < series.size(); ++i) {
      if (q != 0 && series.q[i] != static_cast<int>(q)) continue;
      axes[0].push_back(series.x[i]);
      axes[1].push_back(series.y[i]);
      axes[2].push_back(series.z[i]);
    }
    for (size_t a = 0; a < 3; ++a) table[q][a] = computeAxisStats(axes[a]);
  }
  return table;
}

// Numeric time derivative placed at interval midpoints (TIMEPOS::tdiff).
inline TimePos timeDerivative(const TimePos& in) {
  TimePos out;
  for (size_t i = 0; i + 1 < in.size(); ++i) {
    const double dt = in.t[i + 1] - in.t[i];
    if (dt == 0.0 || std::fabs(dt) > kMaxDifferentialTime) continue;
    out.t.push_back(in.t[i] + dt / 2.0);
    out.x.push_back((in.x[i + 1] - in.x[i]) / dt);
    out.y.push_back((in.y[i + 1] - in.y[i]) / dt);
    out.z.push_back((in.z[i + 1] - in.z[i]) / dt);
    out.q.push_back(std::max(in.q[i], in.q[i + 1]));
  }
  return out;
}

// Difference of two time-sorted series matched within DTTOL (TIMEPOS::diff).
inline TimePos seriesDifference(const TimePos& a, const TimePos& b) {
  TimePos out;
  size_t i = 0, j = 0;
  while (i < a.size() && j < b.size()) {
    const double dt = a.t[i] - b.t[j];
    if (dt < -kTimeTolerance) { ++i; continue; }
    if (dt > kTimeTolerance) { ++j; continue; }
    out.t.push_back(a.t[i]);
    out.x.push_back(a.x[i] - b.x[j]);
    out.y.push_back(a.y[i] - b.y[j]);
    out.z.push_back(a.z[i] - b.z[j]);
    out.q.push_back(std::max(a.q[i], b.q[j]));
    ++i;
    ++j;
  }
  return out;
}

}  // namespace rtk_bag_plot
