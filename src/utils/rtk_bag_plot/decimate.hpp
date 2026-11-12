// Index selection that bounds embedded data while keeping what a plot must show.
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

namespace rtk_bag_plot {

// A numeric channel over row indices; `valid` excludes rows from the extrema.
struct DecimateChannel {
  std::function<double(size_t)> value;
  std::function<bool(size_t)> valid;
};

// Sorted row indices to retain from `count` rows split into `buckets` equal
// bins: each bin keeps its first and last row and the extrema of every channel.
// Rows for which `must_keep` holds are retained too, at most
// `max_events_per_bucket` per bin when that limit is non-zero; the limit stops
// receivers that flag every epoch from defeating the budget.
inline std::vector<uint32_t> selectRows(size_t count, size_t buckets,
                                        const std::vector<DecimateChannel>& channels,
                                        const std::function<bool(size_t)>& must_keep,
                                        size_t max_events_per_bucket = 0) {
  std::vector<uint32_t> keep;
  if (count == 0) return keep;
  buckets = std::clamp<size_t>(buckets, 1, count);
  keep.reserve(std::min(count, buckets * (2 + channels.size() * 2)));
  for (size_t b = 0; b < buckets; ++b) {
    const size_t begin = b * count / buckets;
    const size_t end = (b + 1) * count / buckets;
    if (begin >= end) continue;
    keep.push_back(static_cast<uint32_t>(begin));
    keep.push_back(static_cast<uint32_t>(end - 1));
    for (const auto& channel : channels) {
      size_t low = end, high = end;
      double low_value = std::numeric_limits<double>::infinity();
      double high_value = -std::numeric_limits<double>::infinity();
      for (size_t i = begin; i < end; ++i) {
        if (channel.valid && !channel.valid(i)) continue;
        const double v = channel.value(i);
        if (v < low_value) { low_value = v; low = i; }
        if (v > high_value) { high_value = v; high = i; }
      }
      if (low != end) keep.push_back(static_cast<uint32_t>(low));
      if (high != end) keep.push_back(static_cast<uint32_t>(high));
    }
    if (must_keep) {
      size_t events = 0;
      for (size_t i = begin; i < end; ++i) {
        if (!must_keep(i)) continue;
        keep.push_back(static_cast<uint32_t>(i));
        if (max_events_per_bucket != 0 && ++events >= max_events_per_bucket) break;
      }
    }
  }
  std::sort(keep.begin(), keep.end());
  keep.erase(std::unique(keep.begin(), keep.end()), keep.end());
  return keep;
}

template <typename T>
void applySelection(std::vector<T>& values, const std::vector<uint32_t>& keep) {
  std::vector<T> out;
  out.reserve(keep.size());
  for (const auto i : keep) out.push_back(values[i]);
  values.swap(out);
}

}  // namespace rtk_bag_plot
