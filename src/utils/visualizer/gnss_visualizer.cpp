// gnss_visualizer: live dashboard served to a web browser.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/msg/gnss_ephemerides.hpp"
#include "gnss_ros_standardization/msg/gnss_observations.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"
#include "rtklib.h"
#include "sensor_msgs/msg/imu.hpp"

#include "gnss_visualizer_page_embed.hpp"
#include "imu_monitor.hpp"
#include "sse_server.hpp"

using namespace std::chrono_literals;
using gnss_ros_standardization::msg::GnssEphemerides;
using gnss_ros_standardization::msg::GnssObservations;
using gnss_ros_standardization::msg::GnssSolution;
using sensor_msgs::msg::Imu;

namespace {

constexpr auto kPublishPeriod = 200ms;
constexpr double kAngleRefreshSeconds = 10.0;
constexpr const char* kObservationType = "gnss_ros_standardization/msg/GnssObservations";
constexpr const char* kEphemerisType = "gnss_ros_standardization/msg/GnssEphemerides";
constexpr const char* kSolutionType = "gnss_ros_standardization/msg/GnssSolution";
constexpr const char* kImuType = "sensor_msgs/msg/Imu";
constexpr size_t kSolutionSlots = 2;
constexpr size_t kImuQueueDepth = 1000;  // some receivers deliver IMU samples in batches

std::string number(double value, int decimals) {
  if (!std::isfinite(value)) return "null";
  char text[48];
  std::snprintf(text, sizeof(text), "%.*f", decimals, value);
  return text;
}

std::string quoted(const std::string& value) {
  std::string out = "\"";
  for (const unsigned char c : value)
    if (c >= 0x20 && c != '"' && c != '\\' && c != '<') out += static_cast<char>(c);
  return out + "\"";
}

std::string jsonList(const std::vector<std::string>& values) {
  std::string out = "[";
  for (size_t i = 0; i < values.size(); ++i) out += (i ? "," : "") + quoted(values[i]);
  return out + "]";
}

const char* statusName(uint8_t status) {
  switch (status) {
    case GnssSolution::STATUS_FIX: return "FIX";
    case GnssSolution::STATUS_FLOAT: return "FLOAT";
    case GnssSolution::STATUS_SBAS: return "SBAS";
    case GnssSolution::STATUS_DGPS: return "DGPS";
    case GnssSolution::STATUS_SINGLE: return "SINGLE";
    case GnssSolution::STATUS_PPP: return "PPP";
    case GnssSolution::STATUS_DR: return "DR";
    default: return "NONE";
  }
}

bool contains(const std::vector<std::string>& values, const std::string& value) {
  return std::find(values.begin(), values.end(), value) != values.end();
}

int64_t stampNanoseconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<int64_t>(stamp.sec) * 1'000'000'000 + stamp.nanosec;
}

// Not the node's clock: arrival times must keep advancing during a bag replay.
int64_t wallNanoseconds() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string rangeJson(const gnss_visualizer::ImuRange& range, int decimals) {
  return "[" + number(range.low, decimals) + "," + number(range.high, decimals) + "]";
}

std::string axesJson(const std::array<gnss_visualizer::ImuRange, 3>& axes, int decimals) {
  return "[" + rangeJson(axes[0], decimals) + "," + rangeJson(axes[1], decimals) + "," + rangeJson(axes[2], decimals) + "]";
}

std::string imuBinJson(const gnss_visualizer::ImuBin& bin) {
  return "{\"seq\":" + std::to_string(bin.seq) + ",\"t\":" + number(bin.time_s, 3) + ",\"n\":" + std::to_string(bin.count) +
         ",\"still\":" + (bin.still ? "1" : "0") +
         ",\"dt\":" + rangeJson(bin.dt_ms, 3) + ",\"lat\":" + rangeJson(bin.latency_ms, 3) +
         ",\"a\":" + axesJson(bin.accel, 3) + ",\"g\":" + axesJson(bin.gyro, 3) + "}";
}

std::string imuStatusJson(const gnss_visualizer::ImuStatus& s) {
  const char axis[3] = {s.up_sign < 0 ? '-' : '+', static_cast<char>('X' + s.up_axis), '\0'};
  return "{\"samples\":" + std::to_string(s.samples) + ",\"age\":" + number(s.age_s, 3) + ",\"rate\":" + number(s.rate_hz, 2) +
         ",\"period\":" + number(s.period_ms, 3) + ",\"max_gap\":" + number(s.max_gap_ms, 2) +
         ",\"gaps\":" + std::to_string(s.gaps) + ",\"out_of_order\":" + std::to_string(s.out_of_order) +
         ",\"duplicates\":" + std::to_string(s.duplicates) + ",\"rejected\":" + std::to_string(s.rejected) +
         ",\"still\":" + (s.still ? "true" : "false") + ",\"have_still\":" + (s.have_still ? "true" : "false") +
         ",\"norm\":" + number(s.accel_norm, 3) + ",\"accel_std\":" + number(s.accel_std, 4) + ",\"gyro_std\":" + number(s.gyro_std, 4) +
         ",\"bias\":[" + number(s.gyro_bias[0], 3) + "," + number(s.gyro_bias[1], 3) + "," + number(s.gyro_bias[2], 3) + "]" +
         ",\"imu_minus_obs\":" + number(s.imu_minus_observation_s, 3) +
         ",\"frame_mismatch\":" + (s.frame_mismatch ? "true" : "false") +
         ",\"gravity_free\":" + (s.gravity_free ? "true" : "false") + ",\"ready\":" + (s.attitude_ready ? "true" : "false") +
         ",\"q\":[" + number(s.attitude[0], 5) + "," + number(s.attitude[1], 5) + "," + number(s.attitude[2], 5) + "," +
         number(s.attitude[3], 5) + "],\"up\":\"" + axis + "\",\"tilt\":" + number(s.tilt_deg, 1) + "}";
}

// Lower is preferred: an estimator's solution, then a reference, then the receiver's own.
int solutionPreference(const std::string& topic) {
  if (topic.find("nmea_solution") != std::string::npos) return 2;
  if (topic.find("reference") != std::string::npos) return 1;
  return 0;
}

}  // namespace

class GnssVisualizer : public rclcpp::Node {
 public:
  explicit GnssVisualizer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("gnss_visualizer", options) {
    const auto address = declare_parameter<std::string>("bind_address", "127.0.0.1");
    const auto port = declare_parameter<int>("port", 8765);
    if (port < 0 || port > 65535) throw std::invalid_argument("port must be 0-65535");
    const auto observation_topic = declare_parameter<std::string>("topics.observation", "/gnss/observation");
    const auto ephemeris_topic = declare_parameter<std::string>("topics.ephemeris", "/gnss/ephemeris");
    const auto solution_topic = declare_parameter<std::string>("topics.solution", "/gnss/nmea_solution");
    const auto imu_topic = declare_parameter<std::string>("topics.imu", "/gnss/imu/data_raw");
    recent_window_s_ = declare_parameter("recent_window_s", 60.0);
    decimate_old_dt_s_ = declare_parameter("decimate_old_dt_s", 1.0);
    max_history_ = declare_parameter("max_history", 50000);
    const auto manual = declare_parameter<std::vector<double>>("manual_position.pos", {0.0, 0.0, 0.0});
    const auto manual_type = declare_parameter<std::string>("manual_position.postype", "llh");
    if (manual.size() != 3 || (manual_type != "llh" && manual_type != "ecef"))
      throw std::invalid_argument("manual_position needs three values and postype llh or ecef");
    if (manual[0] != 0.0 || manual[1] != 0.0 || manual[2] != 0.0) {
      have_manual_ = true;
      if (manual_type == "llh") {
        manual_llh_[0] = manual[0] * D2R;
        manual_llh_[1] = manual[1] * D2R;
        manual_llh_[2] = manual[2];
      } else {
        ecef2pos(manual.data(), manual_llh_);
      }
      pos2ecef(manual_llh_, manual_ecef_);
    }

    server_ = std::make_unique<gnss_visualizer::SseServer>(
        address, static_cast<uint16_t>(port),
        std::string_view(reinterpret_cast<const char*>(kVisualizerPage), kVisualizerPageSize));

    subscribeEphemeris(ephemeris_topic);
    selectObservation(observation_topic);
    selectSolution(0, solution_topic);
    selectImu(imu_topic);
    timer_ = create_wall_timer(kPublishPeriod, [this] { publish(); });

    RCLCPP_INFO(get_logger(), "Open http://%s:%u/ in a web browser", address.c_str(), server_->port());
  }

 private:
  struct Point {
    double e, n;
    double t;
    uint8_t status;
  };
  struct SatelliteAngle {
    double az, el;
  };
  struct SolutionTrack {
    std::string topic;
    rclcpp::Subscription<GnssSolution>::SharedPtr subscription;
    GnssSolution::SharedPtr last;
    std::deque<Point> history;
    std::vector<Point> pending;
    double last_compact{-1.0};
    bool sticky{false};  // chosen on the page or already delivering data
  };

  // Ephemerides describe the satellites, not one receiver, so every topic feeds
  // one store. It is never cleared: records arrive only every few minutes.
  void subscribeEphemeris(const std::string& topic) {
    if (ephemeris_subs_.count(topic)) return;
    ephemeris_subs_[topic] = create_subscription<GnssEphemerides>(
        topic, rclcpp::QoS(256).transient_local(), [this](GnssEphemerides::SharedPtr msg) { onEphemeris(*msg); });
  }

  void onEphemeris(const GnssEphemerides& msg) {
    for (const auto& e : msg.gnss_ephemeris) {
      const eph_t eph = gnss_utils::msgToEph(e);
      if (eph.sat <= 0) continue;
      const auto found = eph_.find(eph.sat);
      if (found != eph_.end() && timediff(eph.toe, found->second.toe) < 0.0) continue;
      if (found == eph_.end()) angles_valid_ = false;
      eph_[eph.sat] = eph;
    }
    for (const auto& g : msg.glonass_ephemeris) {
      const geph_t geph = gnss_utils::msgToGeph(g);
      if (geph.sat <= 0) continue;
      const auto found = geph_.find(geph.sat);
      if (found != geph_.end() && timediff(geph.toe, found->second.toe) < 0.0) continue;
      if (found == geph_.end()) angles_valid_ = false;
      geph_[geph.sat] = geph;
    }
  }

  void selectObservation(const std::string& topic) {
    observation_topic_ = topic;
    observation_sticky_ = false;
    last_obs_.reset();
    angles_valid_ = false;
    observation_sub_ = create_subscription<GnssObservations>(topic, 10, [this](GnssObservations::SharedPtr msg) {
      observation_sticky_ = true;
      imu_.noteObservation(stampNanoseconds(msg->header.stamp));
      last_obs_ = std::move(msg);
    });
  }

  // Best effort accepts both reliable and sensor-data publishers.
  void selectImu(const std::string& topic) {
    imu_topic_ = topic;
    imu_sticky_ = false;
    imu_.reset();
    imu_sub_.reset();
    if (topic.empty()) return;
    imu_sub_ = create_subscription<Imu>(topic, rclcpp::QoS(kImuQueueDepth).best_effort(), [this](Imu::SharedPtr msg) {
      imu_sticky_ = true;
      gnss_visualizer::ImuSample sample;
      sample.stamp_ns = stampNanoseconds(msg->header.stamp);
      sample.arrival_ns = wallNanoseconds();
      sample.accel = {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
      sample.gyro = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
      imu_.addSample(sample);
    });
  }

  void selectSolution(size_t slot, const std::string& topic) {
    auto& track = tracks_[slot];
    track.topic = topic;
    track.sticky = false;
    track.last.reset();
    track.history.clear();
    track.pending.clear();
    track.last_compact = -1.0;
    track.subscription.reset();
    angles_valid_ = false;
    if (std::all_of(tracks_.begin(), tracks_.end(), [](const auto& t) { return t.history.empty(); }))
      have_origin_ = false;
    if (topic.empty()) return;
    track.subscription = create_subscription<GnssSolution>(
        topic, 10, [this, slot](GnssSolution::SharedPtr msg) { onSolution(slot, std::move(msg)); });
  }

  // Refreshes the topic lists. Topics without a publisher are ignored, because
  // this node's own subscriptions also appear in the ROS graph.
  void discoverTopics() {
    observation_topics_.clear();
    solution_topics_.clear();
    imu_topics_.clear();
    for (const auto& [name, types] : get_topic_names_and_types()) {
      if (count_publishers(name) == 0) continue;
      for (const auto& type : types) {
        if (type == kObservationType) observation_topics_.push_back(name);
        else if (type == kSolutionType) solution_topics_.push_back(name);
        else if (type == kEphemerisType) subscribeEphemeris(name);
        else if (type == kImuType) imu_topics_.push_back(name);
      }
    }
    std::stable_sort(solution_topics_.begin(), solution_topics_.end(), [](const auto& a, const auto& b) {
      return solutionPreference(a) < solutionPreference(b);
    });
  }

  // Replaces a configured topic that does not exist, unless it was chosen or is delivering data.
  bool autoSelect() {
    bool changed = false;
    if (!observation_sticky_ && !observation_topics_.empty() && !contains(observation_topics_, observation_topic_)) {
      const auto rover = std::find_if(observation_topics_.begin(), observation_topics_.end(),
                                      [](const auto& t) { return t.find("base") == std::string::npos; });
      selectObservation(rover != observation_topics_.end() ? *rover : observation_topics_.front());
      changed = true;
    }
    if (!tracks_[0].sticky && !solution_topics_.empty() && !contains(solution_topics_, tracks_[0].topic)) {
      selectSolution(0, solution_topics_.front());
      changed = true;
    }
    if (!imu_sticky_ && !imu_topics_.empty() && !contains(imu_topics_, imu_topic_)) {
      selectImu(preferredImuTopic());
      changed = true;
    }
    return changed;
  }

  // The rover's raw topic first: only raw data still contains gravity.
  std::string preferredImuTopic() const {
    const auto rover = [](const std::string& t) { return t.find("base") == std::string::npos; };
    const auto raw = std::find_if(imu_topics_.begin(), imu_topics_.end(), [&](const auto& t) {
      return rover(t) && t.find("data_raw") != std::string::npos;
    });
    if (raw != imu_topics_.end()) return *raw;
    const auto any = std::find_if(imu_topics_.begin(), imu_topics_.end(), rover);
    return any != imu_topics_.end() ? *any : imu_topics_.front();
  }

  bool applySelection(const gnss_visualizer::SseServer::Selection& selection) {
    const auto& [slot, topic] = selection;
    if (slot == "obs" && contains(observation_topics_, topic)) {
      if (topic != observation_topic_) selectObservation(topic);
      observation_sticky_ = true;
      return true;
    }
    if (slot == "sol" && contains(solution_topics_, topic)) {
      if (topic != tracks_[0].topic) selectSolution(0, topic);
      tracks_[0].sticky = true;
      return true;
    }
    if (slot == "sol2" && (topic.empty() || contains(solution_topics_, topic))) {
      if (topic != tracks_[1].topic) selectSolution(1, topic);
      tracks_[1].sticky = true;
      return true;
    }
    if (slot == "imu" && contains(imu_topics_, topic)) {
      if (topic != imu_topic_) selectImu(topic);
      imu_sticky_ = true;
      // Only the IMU history changes, so the trajectories need not be sent again.
      server_->broadcast("selection", selectionJson());
      server_->broadcast("imu_init", imuInitJson());
    }
    return false;
  }

  // ENU relative to the first fix. Geodetic position is used because pos_enu
  // has a publisher-specific origin and is absent until that origin is known.
  bool enuOf(const GnssSolution& msg, double& east, double& north) {
    double ecef[3];
    if (std::isfinite(msg.latitude) && std::isfinite(msg.longitude) && std::isfinite(msg.altitude)) {
      const double llh[3] = {msg.latitude * D2R, msg.longitude * D2R, msg.altitude};
      pos2ecef(llh, ecef);
    } else if (std::isfinite(msg.pos_ecef.x) && std::isfinite(msg.pos_ecef.y) && std::isfinite(msg.pos_ecef.z) &&
               std::hypot(msg.pos_ecef.x, msg.pos_ecef.y, msg.pos_ecef.z) > RE_WGS84 / 2.0) {
      ecef[0] = msg.pos_ecef.x;
      ecef[1] = msg.pos_ecef.y;
      ecef[2] = msg.pos_ecef.z;
    } else {
      return false;
    }
    if (!have_origin_) {
      matcpy(origin_ecef_, ecef, 3, 1);
      ecef2pos(origin_ecef_, origin_llh_);
      have_origin_ = true;
    }
    const double delta[3] = {ecef[0] - origin_ecef_[0], ecef[1] - origin_ecef_[1], ecef[2] - origin_ecef_[2]};
    double enu[3];
    ecef2enu(origin_llh_, delta, enu);
    east = enu[0];
    north = enu[1];
    return true;
  }

  void onSolution(size_t slot, GnssSolution::SharedPtr msg) {
    auto& track = tracks_[slot];
    track.sticky = true;
    track.last = msg;
    double east = 0.0, north = 0.0;
    if (msg->status == GnssSolution::STATUS_NONE || !enuOf(*msg, east, north)) return;
    const double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    const Point point{east, north, now, msg->status};
    track.history.push_back(point);
    track.pending.push_back(point);
    while (max_history_ > 0 && static_cast<int>(track.history.size()) > max_history_) track.history.pop_front();
    compactHistory(track, now);
  }

  void compactHistory(SolutionTrack& track, double now) {
    auto& history = track.history;
    if (recent_window_s_ <= 0.0 || decimate_old_dt_s_ <= 0.0 || history.size() < 3) return;
    const double cutoff = now - recent_window_s_;
    if (history.front().t >= cutoff || (track.last_compact > 0.0 && now - track.last_compact < 2.0)) return;
    track.last_compact = now;
    std::deque<Point> compacted;
    compacted.push_back(history.front());
    for (size_t i = 1; i < history.size(); ++i) {
      if (history[i].t < cutoff && history[i].t - compacted.back().t < decimate_old_dt_s_) continue;
      compacted.push_back(history[i]);
    }
    history.swap(compacted);
  }

  void refreshAngles() {
    const auto now = std::chrono::steady_clock::now();
    if (angles_valid_ && now - angles_time_ < std::chrono::duration<double>(kAngleRefreshSeconds)) return;
    double rr[3] = {0.0, 0.0, 0.0}, llh[3] = {0.0, 0.0, 0.0};
    bool have_position = false;
    for (const auto& track : tracks_) {
      const auto& s = track.last;
      if (!s || s->status == GnssSolution::STATUS_NONE || !std::isfinite(s->latitude) ||
          !std::isfinite(s->longitude) || !std::isfinite(s->altitude)) {
        continue;
      }
      llh[0] = s->latitude * D2R;
      llh[1] = s->longitude * D2R;
      llh[2] = s->altitude;
      pos2ecef(llh, rr);
      have_position = true;
      break;
    }
    if (!have_position && have_manual_) {
      matcpy(llh, manual_llh_, 3, 1);
      matcpy(rr, manual_ecef_, 3, 1);
      have_position = true;
    }
    if (!last_obs_ || !have_position) return;

    const gtime_t epoch = gpst2time(static_cast<int>(last_obs_->week), last_obs_->tow);
    std::map<int, SatelliteAngle> angles;
    for (const auto& obs : last_obs_->observations) {
      const int sat = gnss_utils::resolveSatellite(obs.satid, obs.system, obs.prn);
      if (!sat || angles.count(sat)) continue;
      double rs[6], dts[2], var, e[3], azel[2];
      if (satsys(sat, nullptr) == SYS_GLO) {
        const auto found = geph_.find(sat);
        if (found == geph_.end()) continue;
        geph2pos(epoch, &found->second, rs, dts, &var);
      } else {
        const auto found = eph_.find(sat);
        if (found == eph_.end() || !gnss_utils::supportedByLegacyOrbitModel(found->second)) continue;
        eph2pos(epoch, &found->second, rs, dts, &var);
      }
      if (geodist(rs, rr, e) <= 0.0) continue;
      satazel(llh, e, azel);
      if (azel[1] >= 0.0) angles[sat] = {azel[0] * R2D, azel[1] * R2D};
    }
    angles_ = std::move(angles);
    angles_valid_ = true;
    angles_time_ = now;
  }

  std::string statusJson() const {
    std::string out = "{";
    if (tracks_[0].last) {
      const auto& s = *tracks_[0].last;
      out += "\"week\":" + std::to_string(s.time_week) + ",\"tow\":" + number(s.time_tow, 1);
      out += ",\"status\":\"" + std::string(statusName(s.status)) + "\",\"q\":" + std::to_string(s.status);
      out += ",\"lat\":" + number(s.latitude, 8) + ",\"lon\":" + number(s.longitude, 8) + ",\"alt\":" + number(s.altitude, 3);
      out += ",\"ve\":" + number(s.vel_enu.x, 2) + ",\"vn\":" + number(s.vel_enu.y, 2) + ",\"vu\":" + number(s.vel_enu.z, 2);
      out += ",\"ns\":" + std::to_string(s.num_sats) + ",\"gdop\":" + number(s.gdop, 1) + ",\"pdop\":" + number(s.pdop, 1);
      out += ",\"hdop\":" + number(s.hdop, 1) + ",\"vdop\":" + number(s.vdop, 1);
      out += ",\"ratio\":" + number(s.ratio, 1) + ",\"age\":" + number(s.age_diff, 1);
    } else if (have_manual_) {
      out += "\"status\":\"MANUAL\",\"q\":0,\"lat\":" + number(manual_llh_[0] * R2D, 8) +
             ",\"lon\":" + number(manual_llh_[1] * R2D, 8) + ",\"alt\":" + number(manual_llh_[2], 3);
    } else {
      out += "\"status\":\"NONE\",\"q\":0";
    }
    return out + "}";
  }

  std::string satellitesJson() {
    refreshAngles();
    struct Signals {
      std::string id;
      int sys{0};
      double snr[3]{0.0, 0.0, 0.0};
    };
    std::map<int, Signals> satellites;
    size_t signal_count = 0;
    if (last_obs_) {
      for (const auto& obs : last_obs_->observations) {
        const int sat = gnss_utils::resolveSatellite(obs.satid, obs.system, obs.prn);
        if (!sat) continue;
        ++signal_count;
        auto& entry = satellites[sat];
        entry.id = obs.satid;
        entry.sys = satsys(sat, nullptr);
        if (!std::isfinite(obs.snr) || obs.snr < 0 || obs.code_str.empty()) continue;
        const char band = obs.code_str[0];
        const int index = band == '1' ? 0 : band == '2' ? 1 : (band == '5' || band == '6' || band == '7') ? 2 : -1;
        if (index >= 0) entry.snr[index] = std::max(entry.snr[index], static_cast<double>(obs.snr));
      }
    }
    std::string out = "{\"signals\":" + std::to_string(signal_count) + ",\"list\":[";
    bool first = true;
    for (const auto& [sat, entry] : satellites) {
      if (!first) out += ",";
      first = false;
      const auto angle = angles_.find(sat);
      const bool known = angle != angles_.end();
      out += "[" + quoted(entry.id) + "," + quoted(gnss_utils::systemCode(entry.sys)) + "," +
             (known ? number(angle->second.az, 1) : "null") + "," + (known ? number(angle->second.el, 1) : "null") +
             "," + number(entry.snr[0], 1) + "," + number(entry.snr[1], 1) + "," + number(entry.snr[2], 1) + "]";
    }
    return out + "]}";
  }

  template <typename Container>
  static std::string pointsJson(const Container& points) {
    std::string out = "[";
    bool first = true;
    for (const auto& p : points) {
      if (!first) out += ",";
      first = false;
      out += "[" + number(p.e, 3) + "," + number(p.n, 3) + "," + std::to_string(p.status) + "]";
    }
    return out + "]";
  }

  std::string selectionJson() const {
    return "{\"obs\":" + quoted(observation_topic_) + ",\"sol\":" + quoted(tracks_[0].topic) +
           ",\"sol2\":" + quoted(tracks_[1].topic) + ",\"imu\":" + quoted(imu_topic_) + "}";
  }

  std::string tracksJson() const {
    return "{\"max_history\":" + std::to_string(max_history_) + ",\"tracks\":[" + pointsJson(tracks_[0].history) + "," +
           pointsJson(tracks_[1].history) + "]}";
  }

  std::string imuInitJson() const {
    std::string out = "{\"bins\":[";
    bool first = true;
    for (const auto& bin : imu_.bins()) {
      if (!first) out += ",";
      first = false;
      out += imuBinJson(bin);
    }
    return out + "]}";
  }

  std::string imuUpdateJson() const {
    const auto& bins = imu_.bins();
    return "{\"bin\":" + (bins.empty() ? std::string("null") : imuBinJson(bins.back())) +
           ",\"status\":" + imuStatusJson(imu_.status()) + "}";
  }

  // Topics are looked up only when a page connects, that is on opening or reloading it.
  void serveNewStreams() {
    const auto streams = server_->takeNewStreams();
    if (streams.empty()) return;
    discoverTopics();
    if (autoSelect()) broadcastSelection();
    for (const int stream : streams) {
      server_->sendTo({stream}, "topics",
                      "{\"observations\":" + jsonList(observation_topics_) + ",\"solutions\":" + jsonList(solution_topics_) +
                          ",\"imus\":" + jsonList(imu_topics_) +
                          ",\"can_select\":" + (server_->isLocalStream(stream) ? "true" : "false") +
                          ",\"selected\":" + selectionJson() + "}");
    }
    server_->sendTo(streams, "init", tracksJson());
    server_->sendTo(streams, "imu_init", imuInitJson());
  }

  void broadcastSelection() {
    server_->broadcast("selection", selectionJson());
    server_->broadcast("init", tracksJson());
    server_->broadcast("imu_init", imuInitJson());
  }

  void publish() {
    bool changed = false;
    for (const auto& selection : server_->takeSelections()) changed = applySelection(selection) || changed;
    if (changed) broadcastSelection();
    // Before the early return below, so a page opened later still gets the history.
    imu_.closeBin(wallNanoseconds());
    if (imu_.takeRestarted()) server_->broadcast("imu_init", imuInitJson());
    serveNewStreams();
    if (server_->streamCount() == 0) {
      for (auto& track : tracks_) track.pending.clear();
      return;
    }
    server_->broadcast("update", "{\"status\":" + statusJson() + ",\"sats\":" + satellitesJson() + ",\"points\":[" +
                                     pointsJson(tracks_[0].pending) + "," + pointsJson(tracks_[1].pending) +
                                     "],\"imu\":" + imuUpdateJson() + "}");
    for (auto& track : tracks_) track.pending.clear();
  }

  std::unique_ptr<gnss_visualizer::SseServer> server_;
  rclcpp::Subscription<GnssObservations>::SharedPtr observation_sub_;
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  std::map<std::string, rclcpp::Subscription<GnssEphemerides>::SharedPtr> ephemeris_subs_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::string> observation_topics_, solution_topics_, imu_topics_;
  std::string observation_topic_, imu_topic_;
  bool observation_sticky_{false}, imu_sticky_{false};
  std::array<SolutionTrack, kSolutionSlots> tracks_;
  gnss_visualizer::ImuMonitor imu_;

  GnssObservations::SharedPtr last_obs_;
  std::map<int, eph_t> eph_;
  std::map<int, geph_t> geph_;
  std::map<int, SatelliteAngle> angles_;
  bool angles_valid_{false};
  std::chrono::steady_clock::time_point angles_time_{};

  double recent_window_s_{60.0}, decimate_old_dt_s_{1.0};
  int max_history_{50000};
  bool have_manual_{false}, have_origin_{false};
  double manual_llh_[3]{}, manual_ecef_[3]{}, origin_ecef_[3]{}, origin_llh_[3]{};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<GnssVisualizer>());
  } catch (const std::exception& error) {
    std::fprintf(stderr, "gnss_visualizer failed: %s\n", error.what());
    code = 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return code;
}
