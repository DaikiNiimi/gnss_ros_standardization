#pragma once
#include "engine.hpp"
#include "ros_adapter.hpp"
#include "gnss_ros_standardization/ephemeris_store.hpp"
#include "gnss_ros_standardization/epoch_matcher.hpp"
#include <Eigen/Core>
#include <gnss_ros_standardization/msg/gnss_ephemerides.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <mutex>
#include <thread>

namespace gnss_fgo {

// Call once at process startup, before any solver or worker thread starts.
// Eigen normally caches the CPU cache sizes detected on the initializing
// core. On hybrid CPUs those sizes vary between cores, changing matrix-product
// blocking (including GTSAM's QR right-hand side) and floating-point reduction
// order. Small differences can later change integer ambiguity decisions.
// Use Eigen's conservative x86 fallback sizes as a fixed execution policy;
// these are blocking hints, not a requirement on the physical CPU caches.
// This fixes run-to-run cache selection, not cross-platform/compiler equality.
inline void configureDeterministicEigen() {
  Eigen::setCpuCacheSizes(32 * 1024, 256 * 1024, 2 * 1024 * 1024);
}

// ROS intake, solver worker and publication shared by gnss_fgo (GNSS-only
// MotionFactor) and gnss_imu_fgo (GNSS/IMU). The estimator itself is Engine.

struct NodeVariant {
  std::string node_name;      // also the logger name
  std::string solution_topic; // default output topic
  std::string banner;
  bool imu = false;           // GNSS/IMU model (IMU input, lever arm, odometry)
};

// Every input the node did not turn into an estimator input is counted here
// and reported at shutdown; none of them stops the node.
struct IntakeCounters {
  std::atomic<std::uint64_t> rover_received{0}, base_received{0},
      imu_received{0}, nav_received{0};
  std::atomic<std::uint64_t> imu_invalid{0}, imu_reversed{0},
      imu_queue_overflow{0}, epochs_without_imu{0};
  std::atomic<std::uint64_t> nav_records_invalid{0};
  std::atomic<std::uint64_t> unsupported_satellites{0}, duplicate_signals{0};
  std::atomic<std::uint64_t> pairs_queue_overflow{0}, epoch_failures{0};
  std::atomic<std::uint64_t> timeline_resets{0}, after_end_marker{0};
  std::atomic<std::uint64_t> obs_message_lost{0}, imu_message_lost{0},
      nav_message_lost{0};
};

class FgoNode final : public rclcpp::Node {
public:
  // An exact rover/base pair is decided at once; a rover epoch with only an
  // older base waits for the next rover epoch (RoverBaseEpochMatcher). The
  // GNSS/IMU model additionally waits until IMU covers the epoch, or until a
  // rover epoch kImuWaitMs later has arrived (stream time); then the epoch is
  // solved with the IMU that has arrived. The IMU already received is taken
  // before each rover epoch (takeReceivedImu), so the wait judges the IMU
  // stream, not the order in which the executor takes queued messages.
  static constexpr std::int64_t kImuWaitMs = 200;
  // A clock step back of more than this (stream time) starts a new timeline.
  static constexpr std::int64_t kTimelineResetMs = 5000;
  // Solver backlog bounds: pairs (oldest dropped) and IMU samples (oldest
  // dropped). Navigation and timeline events are never dropped.
  static constexpr std::size_t kPairLimit = 4096;
  static constexpr std::size_t kImuLimit = 120000;

  explicit FgoNode(const NodeVariant &variant)
      : Node(variant.node_name), variant_(variant) {
    config_.gnss_motion = !variant_.imu;
    const auto type =
        declare_parameter("base_position.postype", std::string("ecef"));
    base_ = positionParameter(
        declare_parameter("base_position.pos", std::vector<double>{0, 0, 0}),
        type);
    const auto itype =
        declare_parameter("initial_position.postype", std::string("ecef"));
    initial_ = positionParameter(
        declare_parameter("initial_position.pos",
                          std::vector<double>{base_.x(), base_.y(), base_.z()}),
        itype);
    lever_ = Vec3::Zero();
    imu_to_body_ = Mat3::Identity();
    if (variant_.imu) {
      lever_ = vectorParameter("lever_arm");
      const auto rpy = vectorParameter("imu_orientation");
      imu_to_body_ = gtsam::Rot3::RzRyRx(rpy).matrix();
    }
    max_age_ = declare_parameter("max_age_s", 30.);
    if (!std::isfinite(max_age_) || max_age_ < 0)
      throw std::invalid_argument("invalid base age: max_age_s must be finite "
                                  "and >= 0");
    const auto depth = declare_parameter<std::int64_t>("queue_depth", 4096);
    if (depth < 1 || depth > 1000000)
      throw std::invalid_argument("queue_depth must be in [1, 1000000]");
    depth_ = static_cast<std::size_t>(depth);
    trace_dir_ = declare_parameter("debug.trace_directory", std::string{});
    // ENU origin of pos_enu and the odometry (shared local_origin.* schema):
    // mode fixed = local_origin.pos, mode auto = the base station, as RTK.
    const auto local_origin = gnss_utils::declareLocalOrigin(*this);
    origin_ = local_origin.fixed ? Vec3(local_origin.ecef[0],
                                        local_origin.ecef[1],
                                        local_origin.ecef[2])
                                 : base_;
    if (!trace_dir_.empty()) {
      std::filesystem::create_directories(trace_dir_);
      intake_trace_.open(trace_dir_ + "/intake-trace.jsonl");
      if (!intake_trace_)
        throw std::runtime_error("cannot open trace files");
    }
    declareEstimatorParameters();
    engine_ = std::make_unique<Engine>(config_, initial_, base_, lever_);
    frontend_ = std::make_unique<Preprocessor>(config_, base_);

    gnss_utils::RoverBaseEpochMatcher::Options mo;
    mo.max_tdiff_s = max_age_;
    mo.emit_unmatched_rover = true; // base outage: the epoch is still solved
    mo.queue_limit = kPairLimit;
    matcher_ = gnss_utils::RoverBaseEpochMatcher(mo);

    const auto solution_topic =
        declare_parameter("topics.solution", variant_.solution_topic);
    solution_pub_ = create_publisher<rosmsg::GnssSolution>(
        solution_topic, rclcpp::QoS(depth_));
    if (variant_.imu)
      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
          solution_topic + "_odom", rclcpp::QoS(depth_));
    auto topic = [&](const std::string &key, const std::string &fallback) {
      return declare_parameter("topics." + key, fallback);
    };
    rclcpp::SubscriptionOptions obs_options, imu_options, nav_options;
    obs_options.event_callbacks.message_lost_callback =
        [this](rclcpp::QOSMessageLostInfo &info) {
          counters_.obs_message_lost += info.total_count_change;
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "Observation messages lost in transport: %llu",
                               static_cast<unsigned long long>(
                                   counters_.obs_message_lost.load()));
        };
    imu_options.event_callbacks.message_lost_callback =
        [this](rclcpp::QOSMessageLostInfo &info) {
          counters_.imu_message_lost += info.total_count_change;
        };
    nav_options.event_callbacks.message_lost_callback =
        [this](rclcpp::QOSMessageLostInfo &info) {
          counters_.nav_message_lost += info.total_count_change;
        };
    rover_sub_ = create_subscription<rosmsg::GnssObservations>(
        topic("rover_observation", "/gnss/observation"), rclcpp::QoS(depth_),
        [this](rosmsg::GnssObservations::ConstSharedPtr m) { onRover(m); },
        obs_options);
    base_sub_ = create_subscription<rosmsg::GnssObservations>(
        topic("base_observation", "/base/gnss/observation"),
        rclcpp::QoS(depth_),
        [this](rosmsg::GnssObservations::ConstSharedPtr m) { onBase(m); },
        obs_options);
    if (variant_.imu)
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
          topic("imu", "/gnss/imu/data_raw"), rclcpp::QoS(depth_),
          [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { onImu(m); },
          imu_options);
    nav_sub_ = create_subscription<rosmsg::GnssEphemerides>(
        topic("ephemeris", "/gnss/ephemeris"),
        rclcpp::QoS(256).transient_local(),
        [this](rosmsg::GnssEphemerides::ConstSharedPtr m) { onNav(m); },
        nav_options);
    const auto end_topic = declare_parameter("debug.end_topic", std::string{});
    if (!end_topic.empty())
      end_sub_ = create_subscription<std_msgs::msg::String>(
          end_topic, rclcpp::QoS(depth_),
          [this](std_msgs::msg::String::ConstSharedPtr m) { onEnd(*m); });
    for (const auto &[name, value] :
         get_node_parameters_interface()->get_parameter_overrides())
      if (!has_parameter(name))
        throw std::invalid_argument(
            "unsupported parameter: " + name);
    worker_ = std::thread([this] { work(); });
    RCLCPP_INFO(get_logger(), "%s", variant_.banner.c_str());
  }

  ~FgoNode() override {
    stopWorker();
    report();
  }

  // Stops the solver worker; true unless it failed.
  bool stopWorker() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable())
      worker_.join();
    return !failed_.load();
  }

private:
  enum class TaskKind { Pair, Imu, Navigation, Reset, End };
  struct Task {
    TaskKind kind;
    std::function<void()> run;
  };
  using Matcher = gnss_utils::RoverBaseEpochMatcher;
  using ObsPtr = rosmsg::GnssObservations::ConstSharedPtr;

  NodeVariant variant_;
  Config config_;
  Vec3 base_, initial_, lever_;
  Mat3 imu_to_body_;
  // Solver state: touched only by the worker thread (and the constructor).
  std::unique_ptr<Engine> engine_;
  std::unique_ptr<Preprocessor> frontend_;
  gnss_utils::EphemerisStore ephemeris_;
  std::optional<Vec3> origin_;
  SignalPolicy policy_;
  gnss_utils::SignalSelection signal_selection_;
  std::uint64_t reported_imu_backlog_ = 0;
  // Intake state: touched only by the executor thread.
  // Time scale of the header stamps (UTC by the topic contract; GPST in
  // some converted bags), read off the rover observations: header vs
  // week/tow. IMU stamps are converted with it; until it is known, IMU
  // samples wait (bounded).
  enum class Basis { Unknown, Utc, Gpst };
  Basis basis_ = Basis::Unknown;
  std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_waiting_;
  Matcher matcher_;
  std::deque<Matcher::Pair> decided_;
  std::optional<std::int64_t> newest_rover_ms_, newest_base_ms_, last_imu_ns_,
      imu_watermark_ms_;
  double max_age_ = 30.;
  std::size_t depth_ = 4096;
  bool input_closed_ = false;
  std::size_t rover_count_ = 0;
  std::atomic<std::size_t> published_{0};
  std::string trace_dir_;
  std::ofstream intake_trace_;
  IntakeCounters counters_;
  // Worker queue.
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<Task> queue_;
  std::size_t queued_pairs_ = 0, queued_imu_ = 0;
  bool stopping_ = false;
  std::atomic<bool> failed_{false};
  std::thread worker_;
  rclcpp::Publisher<rosmsg::GnssSolution>::SharedPtr solution_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<rosmsg::GnssObservations>::SharedPtr rover_sub_,
      base_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<rosmsg::GnssEphemerides>::SharedPtr nav_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr end_sub_;

  static bool validTime(const rosmsg::GnssObservations &m) {
    return m.week != 0 && std::isfinite(m.tow) && m.tow >= 0 &&
           m.tow < 604800.;
  }
  static std::int64_t epochMs(const rosmsg::GnssObservations &m) {
    return std::int64_t(m.week) * 604800000 + std::llround(m.tow * 1000.);
  }
  Vec3 vectorParameter(const std::string &name) {
    auto p = declare_parameter(name, std::vector<double>{0., 0., 0.});
    if (p.size() != 3 || !std::all_of(p.begin(), p.end(), [](double v) {
          return std::isfinite(v);
        }))
      throw std::invalid_argument(name + " requires three finite numbers");
    return Vec3(p[0], p[1], p[2]);
  }
  // Observation selection uses the parameter names of real_time_kinematic.
  // The SNR mask (pos1.snrmask.*) has the same thresholds but acts on the
  // whole satellite (Preprocessor::edit): a band the receiver has tracked with
  // code and phase that is missing or below the mask in an epoch leaves the
  // satellite out for that receiver in that epoch. Everything else stays an
  // internal constant.
  void declareEstimatorParameters() {
    Config &c = config_;
    const auto number = [this](const std::string &name, double &value) {
      value = declare_parameter(name, value);
    };
    const auto integer = [this](const std::string &name, int &value) {
      value = static_cast<int>(
          declare_parameter<std::int64_t>(name, value));
    };
    const auto flag = [this](const std::string &name, int &value) {
      value = declare_parameter(name, value != 0) ? 1 : 0;
    };
    policy_.frequencies.l1 = declare_parameter("frequencies.enable_l1", true);
    policy_.frequencies.l2 = declare_parameter("frequencies.enable_l2", true);
    policy_.frequencies.l5 = declare_parameter("frequencies.enable_l5", false);
    policy_.systems.clear();
    const auto useSystem = [this](const std::string &name, const char *system) {
      if (declare_parameter(name, true))
        policy_.systems.insert(system);
    };
    useSystem("pos1.navsys.gps", "G");
    useSystem("pos1.navsys.gal", "E");
    useSystem("pos1.navsys.bds", "C");
    useSystem("pos1.navsys.qzs", "J");
    number("pos1.elmask", c.elmin_deg);
    c.snrmask_enable[0] = declare_parameter("pos1.snrmask.rover", false);
    c.snrmask_enable[1] = declare_parameter("pos1.snrmask.base", false);
    const auto snrBand = [this, &c](const std::string &name, int band) {
      const auto values = declare_parameter(name, std::vector<double>(9, 0.));
      if (values.size() != 9)
        throw std::invalid_argument(name + " requires nine values");
      std::copy(values.begin(), values.end(), c.snrmask[band].begin());
    };
    snrBand("pos1.snrmask.l1", 0);
    snrBand("pos1.snrmask.l2", 1);
    snrBand("pos1.snrmask.l5", 2);
    for (const auto &id : declare_parameter(
             "excluded_satellites", std::vector<std::string>{})) {
      if (id.empty())
        continue;
      const int sat = satid2no(id.c_str());
      if (sat <= 0)
        throw std::invalid_argument("Invalid excluded_satellites entry: " + id);
      policy_.excluded.insert(sat);
    }
    integer("pos2.armode", c.ar_mode);
    number("pos2.arthres", c.ar_thresar);
    integer("pos2.minfixsats", c.ar_minfixsats);
    number("pos2.slipthres", c.thres_slip);

    number("noise.doppler_mps", c.doppler_sd_sigma);
    number("graph.lag_s", c.fls_lag);
    number("hold.variance_cycles2", c.varholdamb);
    flag("ambiguity.exclude_bds_geo", c.exclude_bds_geo);
    flag("fde.enable", c.fde_enable);
    number("fde.pseudorange_m", c.fde_pr);
    number("fde.carrier_m", c.fde_cp);

    if (!variant_.imu) {
      number("motion.acceleration_psd", c.motion_acceleration_psd);
      return;
    }
    number("imu.gravity_mps2", c.gravity);
    number("imu.sigma_acc", c.accel_noise);
    number("imu.sigma_gyr", c.gyro_noise);
    number("imu.integration_covariance", c.imu_integ_cov);
    number("imu.sigma_acc_bias", c.accel_bias_sigma);
    number("imu.sigma_gyr_bias", c.gyro_bias_sigma);
    number("imu.bias_prior_sigma_acc_mps2", c.bias_prior_acc_sigma);
    number("imu.bias_prior_sigma_gyr_rps", c.bias_prior_gyro_sigma);
    integer("imu.bias_prior_mode", c.bias_prior_mode);
    flag("nhc.enable", c.nhc_enable);
    number("nhc.sigma_lat_mps", c.nhc_sigma_lat);
    number("nhc.sigma_vert_mps", c.nhc_sigma_vert);
    const Vec3 nhc_lever = vectorParameter("nhc.lever_arm");
    c.nhc_lever = std::to_string(nhc_lever.x()) + "," +
                  std::to_string(nhc_lever.y()) + "," +
                  std::to_string(nhc_lever.z());
    flag("zupt.enable", c.zupt_enable);
    number("zupt.max_acc_std", c.zupt_max_acc_std);
    number("zupt.max_gyr_std", c.zupt_max_gyro_std);
    number("zupt.sigma_mps", c.zupt_sigma_zero_velocity);
    number("init.pitch_deg", c.init_pitch_deg);
  }
  bool closed(const char *what) {
    if (!input_closed_)
      return false;
    ++counters_.after_end_marker;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "%s received after the end marker: ignored", what);
    return true;
  }

  // ---- intake (executor thread) ----
  void onRover(const ObsPtr &m) {
    if (closed("rover observation"))
      return;
    if (variant_.imu)
      takeReceivedImu();
    ++counters_.rover_received;
    if (validTime(*m)) {
      if (variant_.imu)
        learnBasis(*m);
      const auto t = epochMs(*m);
      if (newest_rover_ms_ && t < *newest_rover_ms_ - kTimelineResetMs)
        timelineReset("rover observation");
      if (!newest_rover_ms_ || t > *newest_rover_ms_)
        newest_rover_ms_ = t;
      ++rover_count_;
    }
    matcher_.pushRover(m);
    collectDecided();
    release();
  }
  // After an executor stall the queued messages are taken one per
  // subscription per cycle: a 5 Hz rover stream then runs ahead of a 100 Hz
  // IMU stream in stream time, and a rover epoch kImuWaitMs later would end
  // the wait although the IMU covering the epoch has already arrived.
  void takeReceivedImu() {
    sensor_msgs::msg::Imu msg;
    rclcpp::MessageInfo info;
    while (imu_sub_->take(msg, info))
      onImu(std::make_shared<const sensor_msgs::msg::Imu>(msg));
  }
  void learnBasis(const rosmsg::GnssObservations &m) {
    if (basis_ != Basis::Unknown)
      return;
    const double header = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9;
    gtime_t gpst{};
    gpst.time = static_cast<time_t>(315964800 + std::int64_t(m.week) * 604800 +
                                    std::int64_t(std::floor(m.tow)));
    gpst.sec = m.tow - std::floor(m.tow);
    const gtime_t utc = gpst2utc(gpst);
    const double d_gpst = header - (double(gpst.time) + gpst.sec);
    const double d_utc = header - (double(utc.time) + utc.sec);
    const double half_leap = 0.5 * timediff(gpst, utc);
    if (std::fabs(d_utc) < half_leap)
      basis_ = Basis::Utc;
    else if (std::fabs(d_gpst) < half_leap)
      basis_ = Basis::Gpst;
    if (basis_ == Basis::Unknown) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Rover header stamps are neither UTC nor GPST of "
                           "the epoch (header %.3f, epoch week %u tow %.3f): "
                           "IMU cannot be placed in GPS time yet",
                           header, m.week, m.tow);
      return;
    }
    RCLCPP_INFO(get_logger(), "Header stamps are %s (read off the rover "
                "observations)", basis_ == Basis::Utc ? "UTC" : "GPST");
    while (!imu_waiting_.empty()) {
      auto sample = std::move(imu_waiting_.front());
      imu_waiting_.pop_front();
      ingestImu(*sample);
    }
  }
  void onBase(const ObsPtr &m) {
    if (closed("base observation"))
      return;
    ++counters_.base_received;
    // Any input clock stepping back starts the new timeline, whichever stream
    // arrives first after a loop (otherwise its first bases are "late").
    if (validTime(*m)) {
      const auto t = epochMs(*m);
      if (newest_base_ms_ && t < *newest_base_ms_ - kTimelineResetMs)
        timelineReset("base observation");
      if (!newest_base_ms_ || t > *newest_base_ms_)
        newest_base_ms_ = t;
    }
    matcher_.pushBase(m);
    collectDecided();
    release();
  }
  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr &m) {
    if (closed("IMU sample"))
      return;
    ++counters_.imu_received;
    const Vec3 a(m->linear_acceleration.x, m->linear_acceleration.y,
                 m->linear_acceleration.z),
        g(m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z);
    // covariance[0] < 0 marks a field the sensor did not provide (REP-145).
    if (m->header.stamp.nanosec >= 1000000000u || !a.allFinite() ||
        !g.allFinite() || m->linear_acceleration_covariance[0] < 0 ||
        m->angular_velocity_covariance[0] < 0) {
      ++counters_.imu_invalid;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "IMU sample without a valid stamp, finite "
                           "acceleration and angular rate: dropped (%llu)",
                           static_cast<unsigned long long>(
                               counters_.imu_invalid.load()));
      return;
    }
    if (basis_ == Basis::Unknown) {
      imu_waiting_.push_back(m);
      if (imu_waiting_.size() > kImuLimit) {
        imu_waiting_.pop_front();
        ++counters_.imu_queue_overflow;
      }
      return;
    }
    ingestImu(*m);
  }
  void ingestImu(const sensor_msgs::msg::Imu &msg) {
    const auto *m = &msg;
    const Vec3 a(m->linear_acceleration.x, m->linear_acceleration.y,
                 m->linear_acceleration.z),
        g(m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z);
    const auto stamp = decodeImuTime(m->header.stamp, basis_ == Basis::Utc);
    const std::int64_t ns =
        stamp.seconds * 1000000000 + std::llround(stamp.fraction * 1e9);
    if (last_imu_ns_ && ns < *last_imu_ns_ - kTimelineResetMs * 1000000)
      timelineReset("IMU");
    if (last_imu_ns_ && ns <= *last_imu_ns_) {
      ++counters_.imu_reversed;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "IMU stamp does not advance: dropped (%llu)",
                           static_cast<unsigned long long>(
                               counters_.imu_reversed.load()));
      return;
    }
    last_imu_ns_ = ns;
    ImuSample sample{stamp.absolute(), imu_to_body_ * a, imu_to_body_ * g, {}};
    enqueue(TaskKind::Imu, [this, sample] { engine_->addImu(sample); });
    imu_watermark_ms_ = (ns + 500000) / 1000000;
    release();
  }
  void onNav(const rosmsg::GnssEphemerides::ConstSharedPtr &m) {
    if (closed("navigation message"))
      return;
    ++counters_.nav_received;
    std::vector<eph_t> records;
    for (const auto &e : m->gnss_ephemeris) {
      if (e.system != "G" && e.system != "E" && e.system != "J" &&
          e.system != "C")
        continue;
      try {
        const eph_t record = gnss_utils::msgToEph(e);
        if (record.sat <= 0)
          throw std::invalid_argument("unknown satellite");
        if (!policy_.excluded.count(record.sat))
          records.push_back(record);
      } catch (const std::exception &error) {
        ++counters_.nav_records_invalid;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Ephemeris %s%d ignored: %s", e.system.c_str(),
                             e.prn, error.what());
      }
    }
    if (records.empty())
      return;
    enqueue(TaskKind::Navigation, [this, records = std::move(records)] {
      for (const auto &record : records)
        ephemeris_.ingestEph(record);
      ephemeris_.applyToNav(frontend_->navigation(), true);
    });
  }
  void onEnd(const std_msgs::msg::String &m) {
    if (m.data != "end" || input_closed_) {
      RCLCPP_WARN(get_logger(), "Unexpected end marker ignored");
      return;
    }
    for (auto &pair : matcher_.flush())
      decided_.push_back(std::move(pair));
    noteMatcherDrops();
    release(true);
    input_closed_ = true;
    const auto unresolved = decided_.size() + matcher_.pendingRoverCount();
    intake_trace_.flush();
    enqueue(TaskKind::End, [this, unresolved, rovers = rover_count_] {
      if (!trace_dir_.empty()) {
        std::ofstream(trace_dir_ + "/intake-summary.json")
            << "{\"rover_count\":" << rovers
            << ",\"pending\":" << unresolved
            << ",\"worker_exit\":" << (unresolved ? 1 : 0) << "}";
        std::ofstream(trace_dir_ + "/solver-completed") << published_ << "\n";
      }
      if (unresolved)
        throw std::logic_error("end of input with unreleased rover epochs: " +
                               std::to_string(unresolved));
      rclcpp::shutdown();
    });
  }
  void collectDecided() {
    for (auto &pair : matcher_.drainMatches())
      decided_.push_back(std::move(pair));
    noteMatcherDrops();
  }
  void noteMatcherDrops() {
    for (const auto &e : matcher_.takeDropEvents())
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "%s epoch (week %u tow %.3f) dropped: %s",
                           Matcher::streamName(e.stream), e.week, e.tow,
                           Matcher::dropReasonName(e.reason));
  }
  // Hand decided pairs to the solver in rover order.
  void release(bool eof = false) {
    while (!decided_.empty()) {
      auto &pair = decided_.front();
      const auto t = epochMs(*pair.rover);
      if (variant_.imu && !epochReady(t, imu_watermark_ms_, eof)) {
        const bool waited =
            newest_rover_ms_ && *newest_rover_ms_ - t >= kImuWaitMs;
        if (!waited && !eof)
          break;
        ++counters_.epochs_without_imu;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "IMU does not cover rover epoch tow %.3f: solved "
                             "with the IMU received (%llu epochs)",
                             pair.rover->tow,
                             static_cast<unsigned long long>(
                                 counters_.epochs_without_imu.load()));
      }
      if (intake_trace_.is_open())
        intake_trace_ << "{\"event\":\"base_selected\",\"rover_ms\":" << t
                      << ",\"base_ms\":"
                      << (pair.base ? epochMs(*pair.base) : -1) << "}\n";
      enqueue(TaskKind::Pair, [this, pair = std::move(pair)] { solve(pair); });
      decided_.pop_front();
    }
  }
  // Clock stepped back (bag loop, receiver reset): finish the old timeline,
  // then start the estimator afresh. Navigation is kept.
  void timelineReset(const char *source) {
    for (auto &pair : matcher_.flush())
      decided_.push_back(std::move(pair));
    noteMatcherDrops();
    release(true);
    matcher_.reset();
    newest_rover_ms_.reset();
    newest_base_ms_.reset();
    last_imu_ns_.reset();
    imu_watermark_ms_.reset();
    ++counters_.timeline_resets;
    RCLCPP_WARN(get_logger(),
                "%s clock moved back more than %.1f s: new timeline", source,
                kTimelineResetMs * 1e-3);
    enqueue(TaskKind::Reset, [this] { resetEstimator(); });
  }

  // ---- worker ----
  void enqueue(TaskKind kind, std::function<void()> run) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_)
        return;
      if (kind == TaskKind::Pair && queued_pairs_ >= kPairLimit) {
        dropOldest(TaskKind::Pair);
        ++counters_.pairs_queue_overflow;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Solver behind by %zu epochs: oldest dropped "
                             "(%llu)",
                             kPairLimit,
                             static_cast<unsigned long long>(
                                 counters_.pairs_queue_overflow.load()));
      }
      if (kind == TaskKind::Imu && queued_imu_ >= kImuLimit) {
        dropOldest(TaskKind::Imu);
        ++counters_.imu_queue_overflow;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Solver behind by %zu IMU samples: oldest dropped "
                             "(%llu)",
                             kImuLimit,
                             static_cast<unsigned long long>(
                                 counters_.imu_queue_overflow.load()));
      }
      queued_pairs_ += kind == TaskKind::Pair;
      queued_imu_ += kind == TaskKind::Imu;
      queue_.push_back({kind, std::move(run)});
    }
    cv_.notify_one();
  }
  void dropOldest(TaskKind kind) { // mutex_ held
    for (auto it = queue_.begin(); it != queue_.end(); ++it)
      if (it->kind == kind) {
        queue_.erase(it);
        (kind == TaskKind::Pair ? queued_pairs_ : queued_imu_)--;
        return;
      }
  }
  void work() {
    while (true) {
      Task task;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [&] { return stopping_ || !queue_.empty(); });
        if (stopping_)
          return;
        task = std::move(queue_.front());
        queue_.pop_front();
        queued_pairs_ -= task.kind == TaskKind::Pair;
        queued_imu_ -= task.kind == TaskKind::Imu;
      }
      try {
        task.run();
      } catch (const std::exception &error) {
        if (task.kind == TaskKind::Imu) {
          ++counters_.imu_invalid;
          continue;
        }
        fail(error.what());
        return;
      } catch (...) {
        fail("unknown exception");
        return;
      }
    }
  }
  void fail(const std::string &what) {
    failed_ = true;
    RCLCPP_FATAL(get_logger(), "Solver worker failed: %s", what.c_str());
    rclcpp::shutdown();
  }
  void resetEstimator() {
    engine_ = std::make_unique<Engine>(config_, initial_, base_, lever_);
    frontend_ = std::make_unique<Preprocessor>(config_, base_);
    ephemeris_.applyToNav(frontend_->navigation(), true);
    signal_selection_.reset();
    reported_imu_backlog_ = 0;
  }
  void solve(const Matcher::Pair &pair) {
    Solution sol;
    ObservationEpoch prepared;
    try {
      DecodeStats stats;
      const auto choice = gnss_utils::selectSignals(
          *pair.rover, pair.base.get(), policy_.frequencies, signal_selection_);
      const auto rover =
          decodeObservation(*pair.rover, 1, policy_, choice, &stats);
      std::optional<RawEpoch> base;
      if (pair.base)
        base = decodeObservation(*pair.base, 2, policy_, choice, &stats);
      counters_.unsupported_satellites += stats.unsupported_satellites;
      counters_.duplicate_signals += stats.duplicate_signals;
      prepared = frontend_->prepare(rover, base, engine_->floatPosition());
      sol = engine_->process(prepared);
    } catch (const std::exception &error) {
      // The estimator state may be partially updated: start it afresh. The
      // epoch itself is published without a solution.
      ++counters_.epoch_failures;
      RCLCPP_ERROR(get_logger(),
                   "Epoch week %u tow %.3f failed (%s): estimator restarted",
                   pair.rover->week, pair.rover->tow, error.what());
      resetEstimator();
      sol = Solution{};
      sol.gpst = std::int64_t(pair.rover->week) * 604800 + pair.rover->tow;
      prepared = ObservationEpoch{};
    }
    // The engine caught a failed update of this epoch; the solution is then a
    // warm-reset position or none.
    if (!sol.failure.empty())
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Epoch week %u tow %.3f: estimator update failed "
                           "(%s)",
                           pair.rover->week, pair.rover->tow,
                           sol.failure.c_str());
    if (variant_.imu && engine_->imuBacklogDropped() > reported_imu_backlog_) {
      reported_imu_backlog_ = engine_->imuBacklogDropped();
      RCLCPP_WARN(get_logger(),
                  "%llu IMU samples waited without a GNSS epoch to consume "
                  "them: oldest dropped",
                  static_cast<unsigned long long>(reported_imu_backlog_));
    }
    auto m = solutionMessage(sol, origin_);
    if (m.status == m.STATUS_FIX || m.status == m.STATUS_FLOAT) {
      std::set<int> used;
      for (const auto &s : prepared.sats)
        used.insert(s.sat);
      m.num_sats =
          static_cast<std::uint8_t>(std::min<std::size_t>(255, used.size()));
    }
    if (m.status != m.STATUS_NONE && pair.base)
      m.age_diff = static_cast<float>(pair.age_s);
    // The header stamp is the rover observation's (same clock as the input).
    m.header.stamp = pair.rover->header.stamp;
    solution_pub_->publish(m);
    ++published_;
    if (odom_pub_)
      publishOdometry(sol, m.header.stamp);
  }
  void publishOdometry(const Solution &s,
                       const builtin_interfaces::msg::Time &stamp) {
    if (!s.valid || !origin_ || !s.body_to_ecef.allFinite() ||
        !s.body_velocity.allFinite())
      return;
    nav_msgs::msg::Odometry m;
    m.header.stamp = stamp;
    m.header.frame_id = "map";
    m.child_frame_id = "imu_link";
    const auto E = enuFromEcef(*origin_);
    assignVector(m.pose.pose.position, Vec3(E * (s.body_position - *origin_)));
    const Eigen::Quaterniond q(E * s.body_to_ecef);
    m.pose.pose.orientation.w = q.w();
    m.pose.pose.orientation.x = q.x();
    m.pose.pose.orientation.y = q.y();
    m.pose.pose.orientation.z = q.z();
    assignVector(m.twist.twist.linear,
                 Vec3(s.body_to_ecef.transpose() * s.body_velocity));
    assignVector(m.twist.twist.angular, s.angular_velocity);
    m.pose.covariance.fill(std::numeric_limits<double>::quiet_NaN());
    m.twist.covariance.fill(std::numeric_limits<double>::quiet_NaN());
    odom_pub_->publish(m);
  }
  void report() {
    const auto &c = counters_;
    auto n = [](const std::atomic<std::uint64_t> &v) {
      return static_cast<unsigned long long>(v.load());
    };
    const auto obs_counts = matcher_.counters();
    // Epochs still waiting for a base/IMU decision or for the (stopped)
    // solver when the node stops: the rest of the rover count.
    const auto waiting = static_cast<unsigned long long>(
        decided_.size() + matcher_.pendingRoverCount());
    unsigned long long queued = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      queued = queued_pairs_;
    }
    using R = Matcher::DropReason;
    using S = Matcher::Stream;
    auto dropped = [&](S s, R r) {
      return static_cast<unsigned long long>(obs_counts.dropped(s, r));
    };
    RCLCPP_INFO(
        get_logger(),
        "Input summary: rover %llu (invalid time %llu, duplicate %llu, late "
        "%llu, no base %llu), base %llu (invalid time %llu, duplicate %llu, "
        "late %llu), solutions %zu, epoch failures %llu, timeline resets "
        "%llu, not solved at shutdown %llu (waiting %llu, queued %llu)",
        n(c.rover_received), dropped(S::Rover, R::InvalidTimestamp),
        dropped(S::Rover, R::Duplicate), dropped(S::Rover, R::LateArrival),
        dropped(S::Rover, R::NoBaseWithinWindow) +
            dropped(S::Rover, R::EndOfStreamNoBase),
        n(c.base_received), dropped(S::Base, R::InvalidTimestamp),
        dropped(S::Base, R::Duplicate), dropped(S::Base, R::LateArrival),
        published_.load(), n(c.epoch_failures), n(c.timeline_resets),
        waiting + queued, waiting, queued);
    RCLCPP_INFO(get_logger(),
                "Rows skipped: unsupported satellite %llu, duplicate signal "
                "%llu; navigation: messages %llu, invalid records %llu",
                n(c.unsupported_satellites), n(c.duplicate_signals),
                n(c.nav_received), n(c.nav_records_invalid));
    if (variant_.imu)
      RCLCPP_INFO(get_logger(),
                  "IMU: received %llu, invalid %llu, not advancing %llu, "
                  "queue overflow %llu, backlog dropped %llu, epochs without "
                  "IMU coverage %llu",
                  n(c.imu_received), n(c.imu_invalid), n(c.imu_reversed),
                  n(c.imu_queue_overflow),
                  static_cast<unsigned long long>(
                      engine_ ? engine_->imuBacklogDropped() : 0),
                  n(c.epochs_without_imu));
    if (c.pairs_queue_overflow || c.after_end_marker)
      RCLCPP_WARN(get_logger(),
                  "Epochs dropped by solver backlog %llu; inputs after the "
                  "end marker %llu",
                  n(c.pairs_queue_overflow), n(c.after_end_marker));
    const auto lost = c.obs_message_lost + c.imu_message_lost +
                      c.nav_message_lost;
    if (lost)
      RCLCPP_WARN(get_logger(),
                  "Messages LOST in transport before reaching this node: "
                  "observations %llu, IMU %llu, navigation %llu. Raise "
                  "queue_depth or reduce the load.",
                  n(c.obs_message_lost), n(c.imu_message_lost),
                  n(c.nav_message_lost));
  }
};

// Shared main: deterministic Eigen blocking (hybrid CPUs), startup errors as
// FATAL + exit 1, and the node destroyed before rclcpp::shutdown so a second
// Ctrl-C cannot interrupt the teardown.
inline int runNode(int argc, char **argv, const NodeVariant &variant) {
  gnss_fgo::configureDeterministicEigen();
  rclcpp::init(argc, argv);
  bool ok = false;
  try {
    auto node = std::make_shared<FgoNode>(variant);
    rclcpp::spin(node);
    ok = node->stopWorker();
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger(variant.node_name), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return ok ? 0 : 1;
}

} // namespace gnss_fgo
