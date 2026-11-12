#include "geometry.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/msg/gnss_observations.hpp"
#include "gnss_ros_standardization/obs_converter.hpp"

#include "multipath.hpp"

namespace rtk_bag_plot {
namespace {

using gnss_ros_standardization::msg::GnssObservation;
using gnss_ros_standardization::msg::GnssObservations;

constexpr double kGpsWeekSeconds = 604800.0;
constexpr double kGlonassViewRefreshSeconds = 30.0;

gtime_t toGtime(double gpst) {
  const int week = static_cast<int>(gpst / kGpsWeekSeconds);
  return gpst2time(week, gpst - week * kGpsWeekSeconds);
}

int bandOrder(int band) { return band < 0 ? 1000 : band; }

uint16_t encodeAngle(double rad) {
  const double deg = rad * R2D;
  return static_cast<uint16_t>(std::clamp(std::lround(deg * 100.0), 0L, 65534L));
}

uint16_t encodeDop(double value) {
  if (!std::isfinite(value) || value <= 0.0) return 0;
  return static_cast<uint16_t>(std::min(65535L, std::lround(value * 100.0)));
}

uint8_t encodeSnr(float snr) {
  if (!std::isfinite(snr) || snr <= 0.0F) return 0;
  return static_cast<uint8_t>(std::clamp(std::lround(snr / kSnrScale), 1L, 255L));
}

int16_t encodeMultipath(double mp) {
  if (mp == 0.0 || !std::isfinite(mp)) return kMultipathInvalid;
  const long counts = std::lround(mp / kMultipathScale);
  return static_cast<int16_t>(std::clamp(counts, -32767L, 32767L));
}

// Carrier frequency of one row, resolving the GLONASS channel from the
// observation or, when the receiver did not report it, from the ephemeris.
double signalFrequency(const SignalInfo& info, const nav_t* nav, gtime_t time) {
  int channel = 0;
  if (info.sys == SYS_GLO) {
    if (info.channel <= 13) {
      channel = static_cast<int>(info.channel) - 7;
    } else if (nav) {
      const auto* g = gnss_utils::selectGlonassEphemeris(*nav, info.sat, time);
      if (!g) return 0.0;
      channel = g->frq;
    } else {
      return 0.0;
    }
  }
  return code2freq(info.sys, info.code, channel);
}

// Rebuild the receiver epoch for RTKLIB's single point positioning.
GnssObservations epochMessage(const RawObservationTopic& raw, size_t begin, size_t end, gtime_t t) {
  GnssObservations message;
  int week = 0;
  message.tow = time2gpst(t, &week);
  message.week = static_cast<uint32_t>(week);
  message.observations.reserve(end - begin);
  for (size_t r = begin; r < end; ++r) {
    const auto& row = raw.rows[r];
    const auto& info = raw.signals[row.signal];
    GnssObservation o;
    o.satid = info.satid;
    o.system = info.system;
    o.prn = static_cast<decltype(o.prn)>(info.prn);
    o.code = info.code;
    o.code_str = info.code_str;
    o.freq = info.channel;
    o.p = row.p;
    o.l = row.l;
    o.snr = row.snr;
    o.lli = row.lli;
    message.observations.push_back(std::move(o));
  }
  return message;
}

class ReceiverGeometry {
 public:
  explicit ReceiverGeometry(const nav_t* nav) : nav_(nav) {
    options_ = prcopt_default;
    options_.err[0] = 900.0;  // RTKPLOT updateObservation
  }

  // True when a receiver position is available for this epoch.
  bool update(const RawObservationTopic& raw, size_t begin, size_t end, gtime_t time) {
    if (!nav_) return false;
    if (!view_ || timediff(time, view_time_) >= kGlonassViewRefreshSeconds ||
        timediff(time, view_time_) < 0.0) {
      view_ = std::make_unique<gnss_utils::GlonassNavigationView>(*nav_, time);
      view_time_ = time;
    }
    gnss_utils::FrequencyMask mask;
    mask.l5 = true;
    auto obs = gnss_utils::buildObsEpoch(epochMessage(raw, begin, end, time), nullptr, mask);
    gnss_utils::ObservationRejectCounters counters;
    view_->filter(obs, counters);
    valid_ = false;
    if (static_cast<int>(obs.size()) > MAXOBS) obs.resize(MAXOBS);
    if (obs.empty()) return false;
    sol_t sol{};
    char message[128] = "";
    if (!pntpos(obs.data(), static_cast<int>(obs.size()), &view_->get(), &options_, &sol,
                nullptr, nullptr, message) ||
        sol.stat == SOLQ_NONE) {
      return false;
    }
    matcpy(rr_, sol.rr, 3, 1);
    ecef2pos(rr_, pos_);
    valid_ = norm(rr_, 3) > 0.0;
    return valid_;
  }

  bool satelliteAzEl(int sat, gtime_t time, double* azel) const {
    if (!valid_ || !view_) return false;
    double rs[6], dts[2], var, e[3];
    int svh = 0;
    if (!satpos(time, time, sat, EPHOPT_BRDC, &view_->get(), rs, dts, &var, &svh)) return false;
    if (geodist(rs, rr_, e) <= 0.0) return false;
    satazel(pos_, e, azel);
    if (azel[0] < 0.0) azel[0] += 2.0 * PI;
    return true;
  }

 private:
  const nav_t* nav_;
  prcopt_t options_{};
  std::unique_ptr<gnss_utils::GlonassNavigationView> view_;
  gtime_t view_time_{};
  double rr_[3]{}, pos_[3]{};
  bool valid_{false};
};

}  // namespace

int maskedDop(const std::vector<double>& azel, int mask_deg, double* values) {
  std::vector<double> in_mask;
  in_mask.reserve(azel.size());
  const double mask_rad = mask_deg * D2R;
  for (size_t i = 0; i + 1 < azel.size(); i += 2) {
    if (azel[i + 1] < mask_rad) continue;
    in_mask.push_back(azel[i]);
    in_mask.push_back(azel[i + 1]);
  }
  const int ns = static_cast<int>(in_mask.size() / 2);
  if (ns > 0) dops(ns, in_mask.data(), mask_rad, values);
  return ns;
}

size_t normalizeObservationTopic(RawObservationTopic& raw) {
  // Signals: satellite, then band, then descending RINEX code priority.
  std::vector<uint32_t> signal_order(raw.signals.size());
  std::iota(signal_order.begin(), signal_order.end(), 0U);
  std::sort(signal_order.begin(), signal_order.end(), [&](uint32_t a, uint32_t b) {
    const auto& x = raw.signals[a];
    const auto& y = raw.signals[b];
    if (x.sat != y.sat) return x.sat < y.sat;
    if (x.band != y.band) return bandOrder(x.band) < bandOrder(y.band);
    const int px = getcodepri(x.sys, x.code, "");
    const int py = getcodepri(y.sys, y.code, "");
    if (px != py) return px > py;
    return x.code < y.code;
  });
  std::vector<uint32_t> signal_map(raw.signals.size());
  std::vector<SignalInfo> sorted_signals;
  sorted_signals.reserve(raw.signals.size());
  for (uint32_t i = 0; i < signal_order.size(); ++i) {
    signal_map[signal_order[i]] = i;
    sorted_signals.push_back(raw.signals[signal_order[i]]);
  }
  raw.signals.swap(sorted_signals);

  // Epochs: ascending GPST, keeping the first arrival of a duplicated epoch.
  std::vector<uint32_t> epoch_order(raw.epoch_gpst.size());
  std::iota(epoch_order.begin(), epoch_order.end(), 0U);
  std::stable_sort(epoch_order.begin(), epoch_order.end(), [&](uint32_t a, uint32_t b) {
    return raw.epoch_gpst[a] < raw.epoch_gpst[b];
  });
  constexpr uint32_t kDropped = 0xFFFFFFFFU;
  std::vector<uint32_t> epoch_map(raw.epoch_gpst.size(), kDropped);
  std::vector<double> epochs;
  epochs.reserve(raw.epoch_gpst.size());
  size_t duplicates = 0;
  for (const auto index : epoch_order) {
    if (!epochs.empty() && std::fabs(raw.epoch_gpst[index] - epochs.back()) < 1e-6) {
      ++duplicates;
      continue;
    }
    epoch_map[index] = static_cast<uint32_t>(epochs.size());
    epochs.push_back(raw.epoch_gpst[index]);
  }
  raw.epoch_gpst.swap(epochs);

  size_t out = 0;
  for (auto& row : raw.rows) {
    if (epoch_map[row.epoch] == kDropped) continue;
    row.epoch = epoch_map[row.epoch];
    row.signal = signal_map[row.signal];
    raw.rows[out++] = row;
  }
  raw.rows.resize(out);
  std::sort(raw.rows.begin(), raw.rows.end(), [](const RawRow& a, const RawRow& b) {
    return a.epoch != b.epoch ? a.epoch < b.epoch : a.signal < b.signal;
  });
  return duplicates;
}

ObservationResult processObservationTopic(RawObservationTopic& raw, const nav_t* nav,
                                          double origin_gpst) {
  ObservationResult out;
  out.topic = raw.topic;
  out.has_geometry = nav != nullptr;
  out.total_rows = raw.rows.size();
  out.t.reserve(raw.epoch_gpst.size());
  for (const double t : raw.epoch_gpst) out.t.push_back(t - origin_gpst);

  // Series indexed by satellite and by signal.
  std::vector<int> satellite_index(MAXSAT + 1, -1);
  out.signals.resize(raw.signals.size());
  for (size_t i = 0; i < raw.signals.size(); ++i) {
    const auto& info = raw.signals[i];
    if (satellite_index[info.sat] < 0) {
      SatelliteSeries series;
      char id[16] = "";
      satno2id(info.sat, id);
      series.id = id;
      series.sat = info.sat;
      series.sys = info.sys;
      satellite_index[info.sat] = static_cast<int>(out.sats.size());
      out.sats.push_back(std::move(series));
    }
    auto& signal = out.signals[i];
    signal.sat_index = static_cast<uint32_t>(satellite_index[info.sat]);
    signal.code = info.code_str;
    signal.band = info.band;
  }

  for (const int mask : kElevationMasks) {
    DopSeries dop;
    dop.mask = mask;
    if (nav) {
      dop.gdop.resize(raw.epoch_gpst.size());
      dop.pdop.resize(raw.epoch_gpst.size());
      dop.hdop.resize(raw.epoch_gpst.size());
      dop.vdop.resize(raw.epoch_gpst.size());
      dop.ns.resize(raw.epoch_gpst.size());
    }
    out.dops.push_back(std::move(dop));
  }

  // Multipath before arc-mean removal, row by row and per signal.
  std::vector<double> raw_mp(raw.rows.size(), 0.0);
  std::vector<double> previous_gf(raw.signals.size(), 0.0);
  ReceiverGeometry geometry(nav);

  size_t row = 0;
  for (uint32_t epoch = 0; epoch < raw.epoch_gpst.size(); ++epoch) {
    const size_t epoch_begin = row;
    while (row < raw.rows.size() && raw.rows[row].epoch == epoch) ++row;
    const size_t epoch_end = row;
    const gtime_t time = toGtime(raw.epoch_gpst[epoch]);
    const bool have_position = geometry.update(raw, epoch_begin, epoch_end, time);
    if (nav && !have_position) ++out.failed_position_epochs;

    std::vector<double> azel_list;
    for (size_t a = epoch_begin; a < epoch_end;) {
      const int sat = raw.signals[raw.rows[a].signal].sat;
      size_t b = a;
      while (b < epoch_end && raw.signals[raw.rows[b].signal].sat == sat) ++b;
      auto& series = out.sats[satellite_index[sat]];

      std::vector<SignalSample> samples;
      samples.reserve(b - a);
      for (size_t r = a; r < b; ++r) {
        const auto& rr = raw.rows[r];
        samples.push_back({rr.p, rr.l, signalFrequency(raw.signals[rr.signal], nav, time)});
      }

      double reference_frequency = 0.0, ionosphere = 0.0;
      if (referenceIonosphere(samples, reference_frequency, ionosphere)) {
        for (size_t r = a; r < b; ++r)
          raw_mp[r] = rawMultipath(samples[r - a], reference_frequency, ionosphere);
      }

      // First (highest-priority) row of the two reference bands.
      size_t first_band[2] = {b, b};
      for (size_t r = a; r < b; ++r) {
        const int band = raw.signals[raw.rows[r].signal].band;
        if ((band == 0 || band == 1) && first_band[band] == b) first_band[band] = r;
      }
      const auto referenceSlip = [&](int band) {
        return first_band[band] != b && (raw.rows[first_band[band]].lli & 1);
      };

      double iono = std::nan("");
      if (first_band[0] != b && first_band[1] != b) {
        const auto& s1 = samples[first_band[0] - a];
        const auto& s2 = samples[first_band[1] - a];
        iono = slantIonosphere(s1.p, s2.p, s1.frequency, s2.frequency);
      }

      // Geometry-free slip flag against band 0 (band 1 for band-0 signals).
      for (size_t r = a; r < b; ++r) {
        const int band = raw.signals[raw.rows[r].signal].band;
        if (band < 0) continue;
        const size_t ref = first_band[band == 0 ? 1 : 0];
        if (ref == b) continue;
        const auto& target = samples[r - a];
        const auto& reference = samples[ref - a];
        if (target.frequency == 0.0 || reference.frequency == 0.0) continue;
        const double gf = geometryFree(reference.l, reference.frequency, target.l, target.frequency);
        auto& prev = previous_gf[raw.rows[r].signal];
        if (std::fabs(prev - gf) > kLgJumpThreshold) raw.rows[r].lli |= 0x80;
        prev = gf;
      }

      double azel[2] = {0.0, 0.0};
      const bool have_angle = have_position && geometry.satelliteAzEl(sat, time, azel);
      series.epoch.push_back(epoch);
      series.az.push_back(have_angle ? encodeAngle(azel[0]) : kAngleInvalid);
      series.el.push_back(have_angle && azel[1] >= 0.0 ? encodeAngle(azel[1]) : kAngleInvalid);
      series.iono.push_back(static_cast<float>(iono));
      series.slip_reference.push_back(referenceSlip(0) || referenceSlip(1) ? 1 : 0);
      if (have_angle) {
        azel_list.push_back(azel[0]);
        azel_list.push_back(azel[1]);
      }
      a = b;
    }

    if (nav) {
      for (auto& dop : out.dops) {
        double values[4] = {0.0, 0.0, 0.0, 0.0};
        const int ns = maskedDop(azel_list, dop.mask, values);
        dop.gdop[epoch] = encodeDop(values[0]);
        dop.pdop[epoch] = encodeDop(values[1]);
        dop.hdop[epoch] = encodeDop(values[2]);
        dop.vdop[epoch] = encodeDop(values[3]);
        dop.ns[epoch] = static_cast<uint8_t>(std::min(ns, 255));
      }
    }
  }

  // Rows grouped by signal, in epoch order.
  std::vector<std::vector<uint32_t>> signal_rows(raw.signals.size());
  for (uint32_t r = 0; r < raw.rows.size(); ++r) signal_rows[raw.rows[r].signal].push_back(r);

  std::vector<double> aligned;
  std::vector<uint8_t> reset;
  for (size_t s = 0; s < signal_rows.size(); ++s) {
    const auto& rows = signal_rows[s];
    auto& signal = out.signals[s];
    const auto& series = out.sats[signal.sat_index];
    aligned.assign(series.epoch.size(), 0.0);
    reset.assign(series.epoch.size(), 0);
    std::vector<uint32_t> position(rows.size());
    size_t cursor = 0;
    for (size_t i = 0; i < rows.size(); ++i) {
      while (series.epoch[cursor] != raw.rows[rows[i]].epoch) ++cursor;
      position[i] = static_cast<uint32_t>(cursor);
      aligned[cursor] = raw_mp[rows[i]];
      reset[cursor] = raw.rows[rows[i]].lli & 1;
    }
    for (size_t i = 0; i < reset.size(); ++i) reset[i] = reset[i] || series.slip_reference[i];
    removeArcMean(aligned, reset);

    signal.epoch.reserve(rows.size());
    signal.snr.reserve(rows.size());
    signal.mp.reserve(rows.size());
    signal.flags.reserve(rows.size());
    for (size_t i = 0; i < rows.size(); ++i) {
      const auto& r = raw.rows[rows[i]];
      uint8_t flags = 0;
      if (r.lli & 1) flags |= kFlagSlip;
      if (r.lli & 2) flags |= kFlagHalf;
      if (r.p != 0.0) flags |= kFlagHasP;
      if (r.l != 0.0) flags |= kFlagHasL;
      if (r.has_d) flags |= kFlagHasD;
      if (r.lli & 0x80) flags |= kFlagLgJump;
      signal.epoch.push_back(r.epoch);
      signal.snr.push_back(encodeSnr(r.snr));
      signal.mp.push_back(encodeMultipath(aligned[position[i]]));
      signal.flags.push_back(flags);
    }
  }
  for (auto& series : out.sats) std::vector<uint8_t>().swap(series.slip_reference);
  out.kept_rows = out.total_rows;
  return out;
}

}  // namespace rtk_bag_plot
