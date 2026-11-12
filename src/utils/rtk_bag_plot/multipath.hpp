// Observation combinations following RTKPLOT (updateMp, updateIono,
// generateObservationSlips) and its colour scales (snrColor, mpColor, ionoColor).
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace rtk_bag_plot {

inline constexpr double kSpeedOfLight = 299792458.0;
inline constexpr double kLgJumpThreshold = 0.1;     // RTKPLOT THRESLIP (m)
inline constexpr double kMultipathArcJump = 5.0;    // RTKPLOT updateMp (m)

// One signal of a satellite at one epoch. Zero means "not observed".
struct SignalSample {
  double p{0.0}, l{0.0}, frequency{0.0};
};

// Ionosphere term from the first two distinct frequencies with carrier phase.
// Returns false for single-frequency epochs.
inline bool referenceIonosphere(const std::vector<SignalSample>& samples,
                                double& reference_frequency, double& ionosphere) {
  for (size_t j = 0; j < samples.size(); ++j) {
    if (samples[j].l == 0.0 || samples[j].frequency == 0.0) continue;
    const double f1 = samples[j].frequency;
    for (size_t k = j + 1; k < samples.size(); ++k) {
      const double f2 = samples[k].frequency;
      if (samples[k].l == 0.0 || f2 == 0.0 || f1 == f2) continue;
      const double ratio = f1 / f2;
      ionosphere = -kSpeedOfLight * (samples[j].l / f1 - samples[k].l / f2) / (1.0 - ratio * ratio);
      reference_frequency = f1;
      return true;
    }
    return false;
  }
  return false;
}

// Code-minus-carrier multipath before arc-mean removal; zero when unavailable.
inline double rawMultipath(const SignalSample& s, double reference_frequency, double ionosphere) {
  if (s.p == 0.0 || s.l == 0.0 || s.frequency == 0.0) return 0.0;
  const double ratio = reference_frequency / s.frequency;
  return s.p - kSpeedOfLight * s.l / s.frequency - 2.0 * ratio * ratio * ionosphere;
}

// Remove the running arc mean in place. `mp` holds one value per epoch of the
// satellite (zero = no value); an arc restarts on `reset` or a jump above 5 m.
inline void removeArcMean(std::vector<double>& mp, const std::vector<uint8_t>& reset) {
  double mean = 0.0;
  size_t arc_start = 0;
  int count = 0;
  const auto subtract = [&](size_t end) {
    for (size_t k = arc_start; k < end; ++k)
      if (mp[k] != 0.0) mp[k] -= mean;
  };
  for (size_t i = 0; i < mp.size(); ++i) {
    if (reset[i] || std::fabs(mp[i] - mean) > kMultipathArcJump) {
      subtract(i);
      count = 0;
      arc_start = i;
      mean = 0.0;
    }
    if (mp[i] != 0.0) mean += (mp[i] - mean) / ++count;
  }
  subtract(mp.size());
}

// Geometry-free combination (m) between a reference and a target signal.
inline double geometryFree(double l_ref, double f_ref, double l, double f) {
  return kSpeedOfLight * (l_ref / f_ref - l / f);
}

// Slant ionosphere in TECU from two pseudoranges; returns NaN when unavailable.
inline double slantIonosphere(double p1, double p2, double f1, double f2) {
  if (p1 == 0.0 || p2 == 0.0 || f1 == 0.0 || f2 == 0.0 || f1 == f2) return std::nan("");
  const double scale = 1.0 / 40.308 * f1 * f1 * f2 * f2 / (f1 * f1 - f2 * f2) * 1e-16;
  return scale * (p2 - p1);
}

struct Rgb {
  uint8_t r{0}, g{0}, b{0};
  bool operator==(const Rgb& o) const { return r == o.r && g == o.g && b == o.b; }
};

// RTKPLOT's default marker colours.
inline const std::array<Rgb, 8>& markerColors() {
  static const std::array<Rgb, 8> colors{{{0xc0, 0xc0, 0xc0}, {0x00, 0x80, 0x00},
                                          {0xff, 0xaa, 0x00}, {0xff, 0x00, 0xff},
                                          {0x00, 0x00, 0x80}, {0xff, 0x00, 0x00},
                                          {0x80, 0x80, 0x00}, {0xa0, 0xa0, 0xa4}}};
  return colors;
}

// RTKPLOT's default marker colours of the second solution.
inline const std::array<Rgb, 8>& secondMarkerColors() {
  static const std::array<Rgb, 8> colors{{{0xc0, 0xc0, 0xc0}, {0x00, 0x40, 0x80},
                                          {0x80, 0x80, 0x00}, {0x80, 0x00, 0xff},
                                          {0x00, 0x80, 0xff}, {0xff, 0x80, 0x80},
                                          {0x80, 0x80, 0xff}, {0xa0, 0xa0, 0xa4}}};
  return colors;
}

inline Rgb mixColor(const Rgb& a, const Rgb& b, double weight_a) {
  const auto mix = [&](uint8_t x, uint8_t y) {
    return static_cast<uint8_t>(static_cast<unsigned>(weight_a * x + (1.0 - weight_a) * y) & 0xFF);
  };
  return {mix(a.r, b.r), mix(a.g, b.g), mix(a.b, b.b)};
}

inline Rgb snrColor(double snr) {
  const auto& c = markerColors();
  if (snr < 25.0) return c[7];
  if (snr < 27.5) return c[5];
  if (snr > 47.5) return c[1];
  const double a = (snr - 27.5) / 5.0;
  const int i = static_cast<int>(a);
  return mixColor(c[4 - i], c[5 - i], a - i);
}

inline Rgb mpColor(double mp) {
  const auto& c = markerColors();
  const std::array<Rgb, 5> scale{c[1], c[2], c[3], c[4], c[5]};
  if (mp >= 0.6) return scale[4];
  if (mp <= -0.6) return scale[0];
  const double a = (mp + 0.8) / 0.4;
  const int i = static_cast<int>(a);
  return mixColor(scale[i + 1], scale[i], a - i);
}

inline Rgb ionoColor(double iono) {
  const auto& c = markerColors();
  const std::array<Rgb, 7> scale{c[1], c[2], c[3], c[4], c[5], c[6], c[7]};
  if (std::isnan(iono)) return c[0];
  if (iono >= 70.0) return scale[4];
  if (iono <= 0.0) return scale[0];
  const double a = iono / 10.0;
  const int i = static_cast<int>(a);
  if (i >= 6) return scale[6];
  return mixColor(scale[i + 1], scale[i], a - i);
}

}  // namespace rtk_bag_plot
