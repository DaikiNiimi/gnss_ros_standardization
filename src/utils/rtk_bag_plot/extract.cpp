#include "extract.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/msg/gnss_ephemerides.hpp"
#include "gnss_ros_standardization/msg/gnss_observations.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"
#include "gnss_ros_standardization/pos_reader.hpp"

#include "decimate.hpp"
#include "geometry.hpp"
#include "stats.hpp"

namespace rtk_bag_plot {
namespace {

using gnss_ros_standardization::msg::GnssEphemerides;
using gnss_ros_standardization::msg::GnssObservations;
using gnss_ros_standardization::msg::GnssSolution;
using sensor_msgs::msg::Imu;

constexpr double kGpsWeekSeconds = 604800.0;
constexpr size_t kMaxGapsPerTopic = 200;
constexpr size_t kMaxSolutionRows = 250000;
constexpr size_t kMaxObservationRows = 5000000;
constexpr size_t kImuBuckets = 20000;
constexpr size_t kMaxResidualRows = 2000000;
constexpr size_t kMaxTimeOffsetSamples = 20000;
constexpr size_t kMaxEventsPerBucket = 8;
constexpr size_t kMaxDifferenceTopics = 8;

constexpr const char* kTypeObservations = "gnss_ros_standardization/msg/GnssObservations";
constexpr const char* kTypeSolution = "gnss_ros_standardization/msg/GnssSolution";
constexpr const char* kTypeEphemerides = "gnss_ros_standardization/msg/GnssEphemerides";
constexpr const char* kTypeBaseStation = "gnss_ros_standardization/msg/GnssBaseStation";
constexpr const char* kTypeImu = "sensor_msgs/msg/Imu";

template <typename T>
bool deserialize(const rosbag2_storage::SerializedBagMessage& in, rclcpp::Serialization<T>& ser,
                 T& out) {
  if (!in.serialized_data || in.serialized_data->buffer_length == 0) return false;
  try {
    rclcpp::SerializedMessage message(*in.serialized_data);
    ser.deserialize_message(&message, &out);
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool validGpst(uint32_t week, double tow) {
  return week != 0 && week <= static_cast<uint32_t>(std::numeric_limits<int>::max()) &&
         std::isfinite(tow) && tow >= 0.0 && tow < kGpsWeekSeconds;
}

float finiteOrNan(double value) {
  return std::isfinite(value) ? static_cast<float>(value) : std::nanf("");
}

float standardDeviation(double variance) {
  return std::isfinite(variance) && variance >= 0.0 ? static_cast<float>(std::sqrt(variance))
                                                    : std::nanf("");
}

void appendObservations(RawObservationTopic& topic, const GnssObservations& message, double gpst) {
  const auto epoch = static_cast<uint32_t>(topic.epoch_gpst.size());
  topic.epoch_gpst.push_back(gpst);
  for (const auto& o : message.observations) {
    const int sat = gnss_utils::resolveSatellite(o.satid, o.system, o.prn);
    const int code = gnss_utils::resolveSignal(o.code_str, o.code);
    if (sat <= 0 || sat > MAXSAT || !code) {
      ++topic.rejected_signals;
      continue;
    }
    const std::string key = std::to_string(sat) + ":" + std::to_string(code);
    auto found = topic.signal_index.find(key);
    if (found == topic.signal_index.end()) {
      SignalInfo info;
      int prn = 0;
      info.sat = sat;
      info.sys = satsys(sat, &prn);
      info.prn = prn;
      info.code = static_cast<uint8_t>(code);
      info.band = code2idx(info.sys, info.code);
      info.satid = o.satid;
      info.system = o.system;
      info.code_str = code2obs(info.code);
      info.channel = o.freq;
      found = topic.signal_index.emplace(key, static_cast<uint32_t>(topic.signals.size())).first;
      topic.signals.push_back(std::move(info));
    } else if (topic.signals[found->second].channel > 13 && o.freq <= 13) {
      topic.signals[found->second].channel = o.freq;
    }
    RawRow row;
    row.epoch = epoch;
    row.signal = found->second;
    row.p = std::isfinite(o.p) && o.p > 0.0 ? o.p : 0.0;
    row.l = std::isfinite(o.l) ? o.l : 0.0;
    row.snr = std::isfinite(o.snr) ? o.snr : 0.0F;
    row.lli = static_cast<uint8_t>(o.lli & 3);
    row.has_d = std::isfinite(o.d) && o.d != 0.0 ? 1 : 0;
    topic.rows.push_back(row);
  }
}

void appendSolution(RawSolutionTopic& topic, const GnssSolution& m, double gpst) {
  RawSolutionRow row;
  row.t = gpst;
  row.status = m.status;
  row.num_sats = m.num_sats;
  row.ratio = m.ratio;
  row.age = m.age_diff;
  row.dop = {finiteOrNan(m.gdop), finiteOrNan(m.pdop), finiteOrNan(m.hdop), finiteOrNan(m.vdop)};
  bool have_ecef = std::isfinite(m.pos_ecef.x) && std::isfinite(m.pos_ecef.y) &&
                   std::isfinite(m.pos_ecef.z) &&
                   (m.pos_ecef.x != 0.0 || m.pos_ecef.y != 0.0 || m.pos_ecef.z != 0.0);
  if (have_ecef) {
    row.ecef = {m.pos_ecef.x, m.pos_ecef.y, m.pos_ecef.z};
  } else if (std::isfinite(m.latitude) && std::isfinite(m.longitude) && std::isfinite(m.altitude)) {
    const double pos[3] = {m.latitude * D2R, m.longitude * D2R, m.altitude};
    pos2ecef(pos, row.ecef.data());
  } else {
    return;
  }
  if (std::sqrt(row.ecef[0] * row.ecef[0] + row.ecef[1] * row.ecef[1] + row.ecef[2] * row.ecef[2]) <
      RE_WGS84 / 2.0) {
    return;  // RTKPLOT drops positions far below the ellipsoid.
  }
  row.sd_enu = {standardDeviation(m.pos_enu_cov[0]), standardDeviation(m.pos_enu_cov[4]),
                standardDeviation(m.pos_enu_cov[8])};
  row.vel_enu = {finiteOrNan(m.vel_enu.x), finiteOrNan(m.vel_enu.y), finiteOrNan(m.vel_enu.z)};
  topic.rows.push_back(row);
}

bool isEventDriven(const std::string& type) {
  return type == kTypeEphemerides || type == kTypeBaseStation;
}

double medianOf(std::vector<double> values) {
  if (values.empty()) return 0.0;
  const auto mid = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), mid, values.end());
  return *mid;
}

}  // namespace

BagContents readBag(BagSession& session) {
  BagContents contents;
  contents.ephemerides.setHistoryPerSatellite(0);

  std::unordered_map<std::string, size_t> timeline_of, observation_of, solution_of, imu_of;
  std::unordered_map<std::string, std::string> type_of;
  auto topics = session.topics();
  std::sort(topics.begin(), topics.end(), [](const auto& a, const auto& b) { return a.name < b.name; });
  for (const auto& topic : topics) {
    type_of[topic.name] = topic.type;
    timeline_of[topic.name] = contents.timelines.size();
    TopicTimeline timeline;
    timeline.name = topic.name;
    timeline.type = topic.type;
    contents.timelines.push_back(std::move(timeline));
    if (topic.type == kTypeObservations) {
      observation_of[topic.name] = contents.observations.size();
      contents.observations.push_back({});
      contents.observations.back().topic = topic.name;
    } else if (topic.type == kTypeSolution) {
      solution_of[topic.name] = contents.solutions.size();
      contents.solutions.push_back({});
      contents.solutions.back().topic = topic.name;
    } else if (topic.type == kTypeImu) {
      imu_of[topic.name] = contents.imus.size();
      contents.imus.push_back({});
      contents.imus.back().topic = topic.name;
    }
  }

  rclcpp::Serialization<GnssObservations> observation_serializer;
  rclcpp::Serialization<GnssSolution> solution_serializer;
  rclcpp::Serialization<GnssEphemerides> ephemeris_serializer;
  rclcpp::Serialization<Imu> imu_serializer;
  GnssObservations observations;
  GnssSolution solution;
  GnssEphemerides ephemerides;
  Imu imu;

  while (session.hasNext()) {
    if (!rclcpp::ok()) throw std::runtime_error("Interrupted");
    const auto message = session.readNext();
    if (!message) break;
    const auto timeline_it = timeline_of.find(message->topic_name);
    if (timeline_it == timeline_of.end()) continue;
    auto& timeline = contents.timelines[timeline_it->second];
    const int64_t stamp = bagTimestamp(*message);
    timeline.bag_ns.push_back(stamp);
    const std::string& type = type_of[message->topic_name];

    const auto recordEpoch = [&](uint32_t week, double tow) {
      if (!validGpst(week, tow)) {
        ++timeline.invalid;
        return std::numeric_limits<double>::quiet_NaN();
      }
      const double gpst = week * kGpsWeekSeconds + tow;
      timeline.gpst.push_back(gpst);
      if (contents.bag_minus_gpst.size() < kMaxTimeOffsetSamples)
        contents.bag_minus_gpst.push_back(static_cast<double>(stamp) * 1e-9 - gpst);
      return gpst;
    };

    if (type == kTypeObservations) {
      if (!deserialize(*message, observation_serializer, observations)) {
        ++timeline.invalid;
        continue;
      }
      const double gpst = recordEpoch(observations.week, observations.tow);
      if (std::isfinite(gpst))
        appendObservations(contents.observations[observation_of[message->topic_name]], observations, gpst);
    } else if (type == kTypeSolution) {
      if (!deserialize(*message, solution_serializer, solution)) {
        ++timeline.invalid;
        continue;
      }
      const double gpst = recordEpoch(solution.time_week, solution.time_tow);
      if (std::isfinite(gpst) && solution.status != GnssSolution::STATUS_NONE)
        appendSolution(contents.solutions[solution_of[message->topic_name]], solution, gpst);
    } else if (type == kTypeEphemerides) {
      if (!deserialize(*message, ephemeris_serializer, ephemerides)) {
        ++timeline.invalid;
        continue;
      }
      contents.ephemerides.ingestIon(ephemerides);
      for (const auto& e : ephemerides.gnss_ephemeris) {
        const eph_t converted = gnss_utils::msgToEph(e);
        if (gnss_utils::ephemerisMessageError(converted)) {
          ++contents.rejected_ephemerides;
          continue;
        }
        contents.ephemerides.ingestEph(converted);
        contents.has_ephemeris = true;
      }
      for (const auto& g : ephemerides.glonass_ephemeris) {
        const geph_t converted = gnss_utils::msgToGeph(g);
        if (gnss_utils::ephemerisMessageError(converted)) {
          ++contents.rejected_ephemerides;
          continue;
        }
        contents.ephemerides.ingestGeph(converted);
        contents.has_ephemeris = true;
      }
    } else if (type == kTypeImu) {
      if (!deserialize(*message, imu_serializer, imu)) {
        ++timeline.invalid;
        continue;
      }
      RawImuRow row;
      row.t = static_cast<double>(stamp) * 1e-9;
      row.accel = {finiteOrNan(imu.linear_acceleration.x), finiteOrNan(imu.linear_acceleration.y),
                   finiteOrNan(imu.linear_acceleration.z)};
      row.gyro = {finiteOrNan(imu.angular_velocity.x), finiteOrNan(imu.angular_velocity.y),
                  finiteOrNan(imu.angular_velocity.z)};
      contents.imus[imu_of[message->topic_name]].rows.push_back(row);
    }
  }
  return contents;
}

void appendComparisonBag(BagContents& contents, BagSession& session, const std::string& label) {
  std::unordered_map<std::string, size_t> solution_of;
  auto topics = session.topics();
  std::sort(topics.begin(), topics.end(), [](const auto& a, const auto& b) { return a.name < b.name; });
  for (const auto& topic : topics) {
    if (topic.type != kTypeSolution) continue;
    solution_of[topic.name] = contents.solutions.size();
    contents.solutions.push_back({});
    contents.solutions.back().topic = label + ":" + topic.name;
    contents.solutions.back().source = "compare";
  }
  rclcpp::Serialization<GnssSolution> serializer;
  GnssSolution solution;
  size_t added = 0;
  while (session.hasNext()) {
    if (!rclcpp::ok()) throw std::runtime_error("Interrupted");
    const auto message = session.readNext();
    if (!message) break;
    const auto found = solution_of.find(message->topic_name);
    if (found == solution_of.end() || !deserialize(*message, serializer, solution)) continue;
    if (!validGpst(solution.time_week, solution.time_tow) ||
        solution.status == GnssSolution::STATUS_NONE) {
      continue;
    }
    appendSolution(contents.solutions[found->second], solution,
                   solution.time_week * kGpsWeekSeconds + solution.time_tow);
    ++added;
  }
  if (added == 0) contents.comparison_notes.push_back(label + ": no solutions to compare.");
}

void appendPosFile(BagContents& contents, const std::string& path) {
  if (!std::filesystem::is_regular_file(path))
    throw std::runtime_error("Cannot read the position file: " + path);
  const auto parsed = gnss_pos_reader::parsePosFile(path, std::cerr);
  RawSolutionTopic topic;
  topic.topic = "pos:" + std::filesystem::path(path).filename().string();
  topic.source = "pos";
  for (const auto& row : parsed.rows) {
    if (row.sol.status == GnssSolution::STATUS_NONE) continue;
    int week = 0;
    const double tow = time2gpst(row.t, &week);
    appendSolution(topic, row.sol, week * kGpsWeekSeconds + tow);
  }
  if (topic.rows.empty()) throw std::runtime_error("No solutions in the position file: " + path);
  if (parsed.n_skipped > 0) {
    contents.comparison_notes.push_back(topic.topic + ": " + std::to_string(parsed.n_skipped) +
                                        " lines could not be read.");
  }
  contents.solutions.push_back(std::move(topic));
}

TopicSummary summarizeTimeline(const TopicTimeline& timeline, double bag_minus_gpst,
                               double origin_gpst, bool bag_time_axis) {
  TopicSummary summary;
  summary.name = timeline.name;
  summary.type = timeline.type;
  summary.count = timeline.bag_ns.size();
  summary.invalid = timeline.invalid;
  summary.gnss_epochs = !timeline.gpst.empty();

  std::vector<double> times;
  if (summary.gnss_epochs) {
    double latest = -std::numeric_limits<double>::infinity();
    for (const double t : timeline.gpst) {
      if (t < latest) ++summary.out_of_order;
      latest = std::max(latest, t);
    }
    times = timeline.gpst;
    std::sort(times.begin(), times.end());
    for (size_t i = 1; i < times.size(); ++i)
      if (std::fabs(times[i] - times[i - 1]) < 1e-6) ++summary.duplicates;
    times.erase(std::unique(times.begin(), times.end(),
                            [](double a, double b) { return std::fabs(a - b) < 1e-6; }),
                times.end());
    for (auto& t : times) t -= origin_gpst;
  } else {
    times.reserve(timeline.bag_ns.size());
    int64_t latest = std::numeric_limits<int64_t>::min();
    for (const auto ns : timeline.bag_ns) {
      if (ns < latest) ++summary.out_of_order;
      latest = std::max(latest, ns);
      const double t = static_cast<double>(ns) * 1e-9;
      times.push_back((bag_time_axis ? t : t - bag_minus_gpst) - origin_gpst);
    }
    std::sort(times.begin(), times.end());
  }
  if (times.empty()) return summary;

  summary.first = times.front();
  summary.last = times.back();
  std::vector<double> intervals;
  intervals.reserve(times.size());
  for (size_t i = 1; i < times.size(); ++i) intervals.push_back(times[i] - times[i - 1]);
  if (intervals.empty()) return summary;
  const double nominal = medianOf(intervals);
  if (nominal > 0.0) summary.rate_hz = 1.0 / nominal;
  // Navigation and station data arrive on change, so spacing is not a loss.
  if (isEventDriven(summary.type)) return summary;
  const double gap_threshold = nominal * 2.0;
  for (size_t i = 0; i < intervals.size(); ++i) {
    if (intervals[i] <= gap_threshold || gap_threshold <= 0.0) continue;
    ++summary.gap_count;
    summary.max_gap = std::max(summary.max_gap, intervals[i]);
    if (summary.gaps.size() < kMaxGapsPerTopic) summary.gaps.push_back({times[i], intervals[i]});
  }
  return summary;
}

namespace {

SolutionResult processSolution(RawSolutionTopic& raw, const double* origin_pos,
                               const double* origin_ecef, double origin_gpst) {
  std::stable_sort(raw.rows.begin(), raw.rows.end(),
                   [](const RawSolutionRow& a, const RawSolutionRow& b) { return a.t < b.t; });
  raw.rows.erase(std::unique(raw.rows.begin(), raw.rows.end(),
                             [](const RawSolutionRow& a, const RawSolutionRow& b) {
                               return std::fabs(a.t - b.t) < 1e-6;
                             }),
                 raw.rows.end());

  SolutionResult out;
  out.topic = raw.topic;
  out.source = raw.source;
  out.total_rows = raw.rows.size();
  const size_t n = raw.rows.size();
  out.t.reserve(n);
  TimePos position;
  for (const auto& row : raw.rows) {
    const double dr[3] = {row.ecef[0] - origin_ecef[0], row.ecef[1] - origin_ecef[1],
                          row.ecef[2] - origin_ecef[2]};
    double enu[3];
    ecef2enu(origin_pos, dr, enu);
    const double t = row.t - origin_gpst;
    out.t.push_back(t);
    out.e.push_back(static_cast<float>(enu[0]));
    out.n.push_back(static_cast<float>(enu[1]));
    out.u.push_back(static_cast<float>(enu[2]));
    out.q.push_back(row.status);
    out.ns.push_back(row.num_sats);
    out.ratio.push_back(row.ratio);
    out.age.push_back(row.age);
    out.gdop.push_back(row.dop[0]);
    out.pdop.push_back(row.dop[1]);
    out.hdop.push_back(row.dop[2]);
    out.vdop.push_back(row.dop[3]);
    out.sd_e.push_back(row.sd_enu[0]);
    out.sd_n.push_back(row.sd_enu[1]);
    out.sd_u.push_back(row.sd_enu[2]);
    out.vel_e.push_back(row.vel_enu[0]);
    out.vel_n.push_back(row.vel_enu[1]);
    out.vel_u.push_back(row.vel_enu[2]);
    position.t.push_back(t);
    position.x.push_back(enu[0]);
    position.y.push_back(enu[1]);
    position.z.push_back(enu[2]);
    position.q.push_back(row.status);
    if (row.status < out.status_counts.size()) ++out.status_counts[row.status];
  }
  out.pos_stats = computeStatsTable(position);
  out.velocity = timeDerivative(position);
  out.acceleration = timeDerivative(out.velocity);
  out.vel_stats = computeStatsTable(out.velocity);
  out.acc_stats = computeStatsTable(out.acceleration);
  return out;
}

ImuResult processImu(const RawImuTopic& raw, double bag_minus_gpst, double origin_gpst,
                     bool bag_time_axis) {
  ImuResult out;
  out.topic = raw.topic;
  out.total_rows = raw.rows.size();
  if (raw.rows.empty()) return out;

  std::vector<double> times;
  times.reserve(raw.rows.size());
  for (const auto& row : raw.rows)
    times.push_back((bag_time_axis ? row.t : row.t - bag_minus_gpst) - origin_gpst);
  const auto [low, high] = std::minmax_element(times.begin(), times.end());
  const size_t buckets = std::min(kImuBuckets, raw.rows.size());
  const double span = std::max(*high - *low, 1e-9);

  struct Bucket {
    double t{0.0};
    std::array<float, 6> low{}, high{};
    bool filled{false};
  };
  std::vector<Bucket> bins(buckets);
  for (size_t i = 0; i < raw.rows.size(); ++i) {
    const auto index = std::min(buckets - 1, static_cast<size_t>((times[i] - *low) / span * buckets));
    auto& bin = bins[index];
    const auto& row = raw.rows[i];
    const std::array<float, 6> values{row.accel[0], row.accel[1], row.accel[2],
                                      row.gyro[0],  row.gyro[1],  row.gyro[2]};
    if (!bin.filled) {
      bin.t = times[i];
      bin.low = values;
      bin.high = values;
      bin.filled = true;
    } else {
      for (size_t a = 0; a < 6; ++a) {
        bin.low[a] = std::min(bin.low[a], values[a]);
        bin.high[a] = std::max(bin.high[a], values[a]);
      }
    }
  }
  for (const auto& bin : bins) {
    if (!bin.filled) continue;
    out.t.push_back(bin.t);
    for (size_t a = 0; a < 6; ++a) {
      out.min[a].push_back(bin.low[a]);
      out.max[a].push_back(bin.high[a]);
    }
  }
  return out;
}

ResidualResult readResiduals(const std::string& path, double origin_gpst) {
  if (!std::filesystem::is_regular_file(path))
    throw std::runtime_error("Cannot read the solution status file: " + path);
  solstatbuf_t buffer{};
  const char* files[1] = {path.c_str()};
  if (!readsolstat(files, 1, &buffer))
    throw std::runtime_error("No solution status records in: " + path);

  ResidualResult out;
  out.path = path;
  out.total_rows = static_cast<size_t>(buffer.n);
  std::vector<int> sat_index(MAXSAT + 1, -1);
  const size_t stride = std::max<size_t>(1, out.total_rows / kMaxResidualRows + 1);
  const auto toRelative = [&](gtime_t time) {
    int week = 0;
    const double tow = time2gpst(time, &week);
    return week * kGpsWeekSeconds + tow - origin_gpst;
  };

  for (const int mask : kElevationMasks) {
    DopSeries dop;
    dop.mask = mask;
    out.dops.push_back(std::move(dop));
  }

  std::vector<double> azel;
  bool have_time = false;
  gtime_t current{};
  int previous_sat = 0;
  const auto flushEpoch = [&]() {
    if (!have_time || azel.empty()) return;
    out.dop_t.push_back(toRelative(current));
    for (auto& dop : out.dops) {
      double values[4] = {0.0, 0.0, 0.0, 0.0};
      const int ns = maskedDop(azel, dop.mask, values);
      const auto encode = [](double v) {
        return static_cast<uint16_t>(std::clamp(std::lround(v * 100.0), 0L, 65535L));
      };
      dop.gdop.push_back(encode(values[0]));
      dop.pdop.push_back(encode(values[1]));
      dop.hdop.push_back(encode(values[2]));
      dop.vdop.push_back(encode(values[3]));
      dop.ns.push_back(static_cast<uint8_t>(std::min(ns, 255)));
    }
  };

  for (int i = 0; i < buffer.n; ++i) {
    const solstat_t& row = buffer.data[i];
    if (!have_time || timediff(row.time, current) > DTTOL) {
      flushEpoch();
      azel.clear();
      current = row.time;
      have_time = true;
      previous_sat = 0;
    }
    if (row.sat != previous_sat && (row.flag & 0x20) != 0) {
      azel.push_back(row.az);
      azel.push_back(row.el);
      previous_sat = row.sat;
    }
    if (static_cast<size_t>(i) % stride != 0) continue;
    if (sat_index[row.sat] < 0) {
      char id[16] = "";
      satno2id(row.sat, id);
      sat_index[row.sat] = static_cast<int>(out.sat_ids.size());
      out.sat_ids.emplace_back(id);
    }
    out.t.push_back(toRelative(row.time));
    out.sat_index.push_back(static_cast<uint8_t>(sat_index[row.sat]));
    out.frequency.push_back(row.frq);
    out.flags.push_back(row.flag);
    out.az.push_back(static_cast<float>(row.az * R2D));
    out.el.push_back(static_cast<float>(row.el * R2D));
    out.code_residual.push_back(row.resp);
    out.phase_residual.push_back(row.resc);
    out.snr.push_back(row.snr);
  }
  flushEpoch();
  freesolstatbuf(&buffer);
  return out;
}

}  // namespace

PlotModel buildPlotModel(BagContents& contents, const std::string& bag_name,
                         const std::string& stat_path) {
  PlotModel model;
  model.bag = bag_name;
  model.notes = contents.comparison_notes;

  // Time origin: the earliest GNSS epoch or, without GNSS data, the bag time.
  double origin = std::numeric_limits<double>::infinity();
  for (const auto& timeline : contents.timelines)
    for (const double t : timeline.gpst) origin = std::min(origin, t);
  for (const auto& topic : contents.solutions)
    for (const auto& row : topic.rows) origin = std::min(origin, row.t);
  const bool bag_time_axis = !std::isfinite(origin);
  if (bag_time_axis) {
    model.time_basis = "bag";
    for (const auto& timeline : contents.timelines)
      for (const auto ns : timeline.bag_ns) origin = std::min(origin, static_cast<double>(ns) * 1e-9);
    if (!std::isfinite(origin)) origin = 0.0;
    model.notes.push_back("No GNSS epochs were found; the time axis is the bag time.");
  }
  model.origin_gpst = origin;
  if (!bag_time_axis) {
    const int week = static_cast<int>(origin / kGpsWeekSeconds);
    const gtime_t t = gpst2time(week, origin - week * kGpsWeekSeconds);
    model.leap_seconds = timediff(gpst2utc(t), t);
  }
  const double bag_minus_gpst = medianOf(contents.bag_minus_gpst);

  for (const auto& timeline : contents.timelines)
    model.topics.push_back(summarizeTimeline(timeline, bag_minus_gpst, origin, bag_time_axis));

  // Navigation data shared by every observation topic.
  std::unique_ptr<nav_t, void (*)(nav_t*)> nav(nullptr, [](nav_t* n) {
    freenav(n, 0xFF);
    delete n;
  });
  if (contents.has_ephemeris) {
    nav.reset(new nav_t{});
    contents.ephemerides.applyToNav(*nav, true);
    if (nav->n == 0 && nav->ng == 0) {
      nav.reset();
      contents.has_ephemeris = false;
    }
  }
  size_t undecoded_ephemerides = 0;
  for (const auto& timeline : contents.timelines) {
    if (timeline.type == kTypeEphemerides) undecoded_ephemerides += timeline.invalid;
    if (timeline.invalid > 0) {
      model.notes.push_back(timeline.name + ": " + std::to_string(timeline.invalid) +
                            " messages could not be decoded or had an invalid GNSS time.");
    }
  }
  if (!contents.has_ephemeris) {
    model.notes.push_back(undecoded_ephemerides > 0
        ? "The ephemeris messages could not be decoded: Skyplot, DOP and elevation are unavailable."
        : "No ephemeris in this bag: Skyplot, DOP and elevation are unavailable.");
  }
  if (contents.rejected_ephemerides > 0) {
    model.notes.push_back(std::to_string(contents.rejected_ephemerides) +
                          " ephemeris records were rejected as invalid.");
  }

  if (nav) {
    std::vector<int> index_of(MAXSAT + 1, -1);
    const auto satIndex = [&](int sat) {
      if (index_of[sat] < 0) {
        char id[16] = "";
        satno2id(sat, id);
        index_of[sat] = static_cast<int>(model.ephemeris_sats.size());
        model.ephemeris_sats.emplace_back(id);
      }
      return index_of[sat];
    };
    const auto relative = [&](gtime_t time) {
      int week = 0;
      const double tow = time2gpst(time, &week);
      return week * kGpsWeekSeconds + tow - origin;
    };
    for (int i = 0; i < nav->n; ++i) {
      const eph_t& e = nav->eph[i];
      const bool unhealthy = satsys(e.sat, nullptr) == SYS_QZS ? (e.svh & 0xfe) != 0 : e.svh != 0;
      model.ephemerides.push_back({satIndex(e.sat), relative(e.ttr), relative(e.toe), unhealthy});
    }
    for (int i = 0; i < nav->ng; ++i) {
      const geph_t& g = nav->geph[i];
      const bool unhealthy = (g.svh & 9) != 0 || (g.svh & 6) == 4;
      model.ephemerides.push_back({satIndex(g.sat), relative(g.tof), relative(g.toe), unhealthy});
    }
  }

  // Common solution origin: the average position of the longest solution.
  const RawSolutionTopic* primary = nullptr;
  for (const auto& topic : contents.solutions)
    if (!topic.rows.empty() && (!primary || topic.rows.size() > primary->rows.size())) primary = &topic;
  double origin_ecef[3] = {0.0, 0.0, 0.0}, origin_pos[3] = {0.0, 0.0, 0.0};
  if (primary) {
    for (const auto& row : primary->rows)
      for (int a = 0; a < 3; ++a) origin_ecef[a] += row.ecef[a] / static_cast<double>(primary->rows.size());
    ecef2pos(origin_ecef, origin_pos);
    model.solution_origin_topic = primary->topic;
    model.solution_origin_llh = {origin_pos[0] * R2D, origin_pos[1] * R2D, origin_pos[2]};
  } else {
    model.notes.push_back("No solution topics in this bag.");
  }

  std::vector<TimePos> positions;
  for (auto& topic : contents.solutions) {
    if (topic.rows.empty()) continue;
    model.solutions.push_back(processSolution(topic, origin_pos, origin_ecef, origin));
    const auto& solution = model.solutions.back();
    TimePos position;
    for (size_t i = 0; i < solution.t.size(); ++i) {
      position.t.push_back(solution.t[i]);
      position.x.push_back(solution.e[i]);
      position.y.push_back(solution.n[i]);
      position.z.push_back(solution.u[i]);
      position.q.push_back(solution.q[i]);
    }
    positions.push_back(std::move(position));
  }

  // Pairwise differences; large bags compare against the first topic only.
  for (size_t a = 0; a < positions.size(); ++a) {
    for (size_t b = a + 1; b < positions.size(); ++b) {
      if (positions.size() > kMaxDifferenceTopics && a != 0) continue;
      SolutionDifference difference;
      difference.a = a;
      difference.b = b;
      difference.diff = seriesDifference(positions[a], positions[b]);
      if (difference.diff.size() == 0) continue;
      difference.stats = computeStatsTable(difference.diff);
      model.differences.push_back(std::move(difference));
    }
  }

  for (auto& raw : contents.observations) {
    if (raw.epoch_gpst.empty()) continue;
    std::fprintf(stderr, "Processing observations: %s (%zu epochs)\n", raw.topic.c_str(),
                 raw.epoch_gpst.size());
    normalizeObservationTopic(raw);
    model.observations.push_back(processObservationTopic(raw, nav.get(), origin));
    if (!rclcpp::ok()) throw std::runtime_error("Interrupted");
    if (raw.rejected_signals > 0) {
      model.notes.push_back(raw.topic + ": " + std::to_string(raw.rejected_signals) +
                            " signals with an unknown satellite or code were skipped.");
    }
    const auto& result = model.observations.back();
    if (result.failed_position_epochs > 0) {
      model.notes.push_back(raw.topic + ": receiver position unavailable in " +
                            std::to_string(result.failed_position_epochs) +
                            " epochs; satellite angles are missing there.");
    }
    std::vector<RawRow>().swap(raw.rows);
  }

  for (const auto& raw : contents.imus) {
    if (raw.rows.empty()) continue;
    model.imus.push_back(processImu(raw, bag_minus_gpst, origin, bag_time_axis));
  }

  if (!stat_path.empty()) {
    model.residuals = readResiduals(stat_path, origin);
    model.has_residuals = true;
  }
  return model;
}

namespace {

void decimateTimePos(TimePos& series, size_t max_rows) {
  if (series.size() <= max_rows) return;
  const std::vector<DecimateChannel> channels{
      {[&](size_t i) { return series.x[i]; }, {}},
      {[&](size_t i) { return series.y[i]; }, {}},
      {[&](size_t i) { return series.z[i]; }, {}},
  };
  const auto keep = selectRows(series.size(), max_rows / 8, channels, {});
  applySelection(series.t, keep);
  applySelection(series.x, keep);
  applySelection(series.y, keep);
  applySelection(series.z, keep);
  applySelection(series.q, keep);
}

void decimateSolution(SolutionResult& s, size_t max_rows) {
  decimateTimePos(s.velocity, max_rows);
  decimateTimePos(s.acceleration, max_rows);
  if (s.t.size() <= max_rows) return;
  const std::vector<DecimateChannel> channels{
      {[&](size_t i) { return static_cast<double>(s.e[i]); }, {}},
      {[&](size_t i) { return static_cast<double>(s.n[i]); }, {}},
      {[&](size_t i) { return static_cast<double>(s.u[i]); }, {}},
  };
  const auto status_change = [&](size_t i) { return i > 0 && s.q[i] != s.q[i - 1]; };
  const auto keep = selectRows(s.t.size(), max_rows / 8, channels, status_change);
  applySelection(s.t, keep);
  applySelection(s.q, keep);
  applySelection(s.ns, keep);
  for (auto* column : {&s.e, &s.n, &s.u, &s.ratio, &s.age, &s.gdop, &s.pdop, &s.hdop, &s.vdop,
                       &s.sd_e, &s.sd_n, &s.sd_u, &s.vel_e, &s.vel_n, &s.vel_u}) {
    applySelection(*column, keep);
  }
}

void decimateSignal(SignalSeries& signal, double fraction) {
  const size_t n = signal.epoch.size();
  if (n == 0) return;
  const size_t buckets = std::max<size_t>(1, static_cast<size_t>(std::ceil(n * fraction / 6.0)));
  if (buckets * 6 >= n) return;
  const std::vector<DecimateChannel> channels{
      {[&](size_t i) { return static_cast<double>(signal.snr[i]); },
       [&](size_t i) { return signal.snr[i] != 0; }},
      {[&](size_t i) { return static_cast<double>(signal.mp[i]); },
       [&](size_t i) { return signal.mp[i] != kMultipathInvalid; }},
  };
  const auto events = [&](size_t i) {
    return (signal.flags[i] & (kFlagSlip | kFlagLgJump)) != 0;
  };
  const auto keep = selectRows(n, buckets, channels, events, kMaxEventsPerBucket);
  applySelection(signal.epoch, keep);
  applySelection(signal.snr, keep);
  applySelection(signal.mp, keep);
  applySelection(signal.flags, keep);
}

void decimateSatellite(SatelliteSeries& series, double fraction) {
  const size_t n = series.epoch.size();
  const auto target = static_cast<size_t>(std::ceil(n * fraction));
  if (n <= 2 || target >= n) return;
  const auto keep = selectRows(n, std::max<size_t>(1, target / 2), {}, {});
  applySelection(series.epoch, keep);
  applySelection(series.az, keep);
  applySelection(series.el, keep);
  applySelection(series.iono, keep);
}

constexpr size_t kBytesPerRow = 4 + 1 + 2 + 1;           // epoch, snr, mp, flags
constexpr size_t kBytesPerSatelliteEpoch = 4 + 2 + 2 + 4;  // epoch, az, el, iono
constexpr size_t kBytesPerSolutionRow = 8 + 1 + 2 + 15 * 4;
constexpr size_t kBytesPerDerivativeRow = 8 + 3 * 8 + 4;
constexpr size_t kBytesPerImuBucket = 8 + 12 * 4;
constexpr size_t kBytesPerResidualRow = 8 + 3 + 5 * 4;
constexpr size_t kBytesPerDifferenceRow = 8 + 3 * 8 + 4;
constexpr size_t kBytesPerDopEpoch = kElevationMasks.size() * (4 * 2 + 1);

}  // namespace

size_t estimateObservationFixedBytes(const ObservationResult& observation) {
  const bool has_dop = !observation.dops.empty() && !observation.dops.front().ns.empty();
  return observation.t.size() * (8 + (has_dop ? kBytesPerDopEpoch : 0));
}

void applySizeBudget(PlotModel& model, size_t budget_bytes) {
  size_t fixed_bytes = 256 * 1024;
  for (auto& solution : model.solutions) {
    decimateSolution(solution, kMaxSolutionRows);
    fixed_bytes += solution.t.size() * kBytesPerSolutionRow;
    fixed_bytes += (solution.velocity.size() + solution.acceleration.size()) * kBytesPerDerivativeRow;
  }
  for (auto& difference : model.differences) {
    decimateTimePos(difference.diff, kMaxSolutionRows);
    fixed_bytes += difference.diff.size() * kBytesPerDifferenceRow;
  }
  for (const auto& imu : model.imus) fixed_bytes += imu.t.size() * kBytesPerImuBucket;
  if (model.has_residuals) fixed_bytes += model.residuals.t.size() * kBytesPerResidualRow;

  size_t rows = 0, satellite_epochs = 0;
  for (const auto& observation : model.observations) {
    fixed_bytes += estimateObservationFixedBytes(observation);
    for (const auto& signal : observation.signals) rows += signal.epoch.size();
    for (const auto& series : observation.sats) satellite_epochs += series.epoch.size();
  }
  const size_t observation_bytes = rows * kBytesPerRow + satellite_epochs * kBytesPerSatelliteEpoch;
  const size_t available = budget_bytes > fixed_bytes ? budget_bytes - fixed_bytes : budget_bytes / 4;
  double fraction = 1.0;
  if (observation_bytes > available)
    fraction = static_cast<double>(available) / static_cast<double>(observation_bytes);
  if (rows > kMaxObservationRows)
    fraction = std::min(fraction, static_cast<double>(kMaxObservationRows) / static_cast<double>(rows));
  if (fraction >= 1.0) return;

  for (auto& observation : model.observations) {
    size_t kept = 0;
    for (auto& signal : observation.signals) {
      decimateSignal(signal, fraction);
      kept += signal.epoch.size();
    }
    for (auto& series : observation.sats) decimateSatellite(series, fraction);
    observation.kept_rows = kept;
  }
  model.notes.push_back("Observation rows were decimated to fit the page; extrema and slips are kept.");
}

}  // namespace rtk_bag_plot
