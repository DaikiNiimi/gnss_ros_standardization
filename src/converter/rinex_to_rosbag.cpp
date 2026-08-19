// SPDX-License-Identifier: MIT
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/bag_io_utils.hpp"
#include "gnss_ros_standardization/pos_reader.hpp"
// gnss_utils.hpp includes rtklib.h internally

#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <queue>
#include <sstream>
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <limits>

using gnss_ros_standardization::msg::GnssObservation;
using gnss_ros_standardization::msg::GnssObservations;
using gnss_ros_standardization::msg::GnssEphemeris;
using gnss_ros_standardization::msg::GlonassEphemeris;
using gnss_ros_standardization::msg::GnssEphemerides;

// Default ephemeris selection gates (seconds). Configurable via CLI.
//   GNSS  : GPS LNAV TOE validity is ±2 h (7200 s); Galileo I/NAV similar.
//   GLONASS: broadcast ephemeris validity is ±15 min nominal; we use ±30 min
//            (1800 s) to keep the same window RTKLIB uses by default.
constexpr double kDefaultMaxDtoeGnss    = 7200.0;
constexpr double kDefaultMaxDtoeGlonass = 1800.0;

enum class EphMode { PerEpoch, OnChange };

// One RINEX OBS input → one rosbag topic. Multiple sources (e.g. rover/base
// for RTK) can be specified; epochs are written in GPST-time order across
// sources so `ros2 bag play` reproduces the real-time interleave.
struct ObsSource {
  std::string label;  // empty = unlabeled (single-source compat mode)
  std::string path;
  std::string topic;  // resolved after CLI parse
};

// Everything below is private to this translation unit. Each converter is a
// separate executable, but they all declare a file-scope `struct Args`, and
// identical names with different layouts at external linkage are an ODR
// violation the moment anything links two of them (cppcheck flags it as one).
namespace {
struct Args {
  std::vector<ObsSource> obs_sources;
  std::vector<std::string> nav_paths;
  std::string output_bag;
  std::string storage_id;       // empty => auto-detect from --out extension or distro default
  std::string pos_path;         // optional; when set, also writes the solution topic
  std::string pos_label;        // empty = bind to first OBS source
  std::string topic_nav  = "/gnss/ephemeris";
  std::string topic_pos  = "/gnss/solution";
  std::map<std::string, std::string> topic_obs_overrides;  // label -> explicit topic
  EphMode eph_mode = EphMode::OnChange;
  double max_dtoe_gnss    = kDefaultMaxDtoeGnss;
  double max_dtoe_glonass = kDefaultMaxDtoeGlonass;
  // Optional IMU csv -> sensor_msgs/Imu (e.g. PPC-Dataset imu.csv). Columns:
  // GPS TOW, GPS Week, Acc XYZ [m/s^2], Ang Rate XYZ [deg/s] (deg->rad applied).
  std::string imu_path;
  std::string topic_imu = "/gnss/imu/data_raw";
  std::string imu_frame = "raw";  // see applyImuFrame()
  // Optional ground-truth trajectory csv (e.g. PPC-Dataset reference.csv).
  // Written on its OWN topics, never on --topic-pos: a reference trajectory
  // and an estimator's output must not be confusable in a published dataset.
  std::string reference_path;
  std::string topic_reference = "/gnss/reference/solution";
};

// Body-frame convention adapters for the IMU. The FGO node integrates in a
// z-up ENU navigation frame (gtsam MakeSharedU), so the body specific force must
// read +g on the up axis at rest and the frame must be right-handed z-up.
//   raw (default): pass through unchanged - the source is already a z-up
//     specific force in x-forward / y-left / z-up. This is the correct mode
//     for PPC-Dataset (verified on all six runs; see the sign checks below).
//   frd2flu : plain FRD->FLU rotation acc=(ax,-ay,-az), gyr=(gx,-gy,-gz)
//     (source already reports specific force = reaction, +g on up).
//   frd2flu_grav : source is right-handed Forward-Right-Down AND the
//     accelerometer reports the GRAVITY direction rather than specific force.
//     Negate to specific force, then rotate FRD->FLU (180 deg about forward):
//     acc=(-ax, ay, az), gyr=(gx, -gy, -gz).
//
// Identity is the default because a wrong non-identity mode is unrecoverable:
// the accelerometer leg of frd2flu_grav is a reflection (det -1) while its gyro
// leg is a rotation (det +1), so if the mode does not match the source, no
// attitude whatsoever can reconcile the two sensors and the optimiser is handed
// a physically impossible pair.
//
// Verify the mode with two sign checks against a reference trajectory. Both use
// only quantities that carry no Euler-angle convention, so neither can be
// fooled by a roll/pitch/yaw sign disagreement:
//   1. accel x   vs d|v|/dt              -> slope +1 means x is forward
//   2. gyro  z   vs d(atan2(vn,ve))/dt   -> slope +1 means z is up
// A negative slope on either one means the mode is wrong. Matching the node's
// initialized roll/pitch/heading is NOT a sufficient check: with an FLU source
// and frd2flu_grav applied, only accel x and gyro y/z flip, so roll still
// agrees with the reference while pitch and heading are silently corrupted.
struct ImuTriplet { double x, y, z; };
static bool applyImuFrame(const std::string& mode, ImuTriplet& acc,
                          ImuTriplet& gyr) {
  if (mode == "raw") {
    return true;
  } else if (mode == "frd2flu") {
    acc = {acc.x, -acc.y, -acc.z};
    gyr = {gyr.x, -gyr.y, -gyr.z};
    return true;
  } else if (mode == "frd2flu_grav") {
    acc = {-acc.x, acc.y, acc.z};
    gyr = {gyr.x, -gyr.y, -gyr.z};
    return true;
  }
  return false;
}

// RAII wrappers for RTKLIB obs_t / nav_t so memory is released on any exit
// path (writer exception, early return, std::terminate from a downstream
// throw, etc.). RTKLIB frees the dynamic arrays inside the struct; the
// struct itself is local.
struct ObsGuard {
  obs_t obs{};
  ~ObsGuard() { freeobs(&obs); }
};
struct NavGuard {
  nav_t nav{};
  ~NavGuard() { freenav(&nav, 0xFF); }
};

// Split a CLI value at the first '=' into {label, rest}. If no '=' is found,
// label is empty and rest is the whole input. Used for --obs/--topic-obs/--pos.
static std::pair<std::string, std::string> splitLabel(const std::string& s) {
  auto pos = s.find('=');
  if (pos == std::string::npos) return {"", s};
  return {s.substr(0, pos), s.substr(pos + 1)};
}

Args parseArgs(const std::vector<std::string>& args) {
  Args a;
  for (size_t i = 0; i < args.size(); ++i) {
    if (args[i] == "--obs" && i + 1 < args.size()) {
      auto [label, path] = splitLabel(args[++i]);
      a.obs_sources.push_back(ObsSource{label, path, ""});
    } else if (args[i] == "--nav" && i + 1 < args.size()) {
      a.nav_paths.push_back(args[++i]);
    } else if (args[i] == "--topic-obs" && i + 1 < args.size()) {
      auto [label, topic] = splitLabel(args[++i]);
      a.topic_obs_overrides[label] = topic;
    } else if (args[i] == "--topic-nav" && i + 1 < args.size()) {
      a.topic_nav = args[++i];
    } else if (args[i] == "--topic-pos" && i + 1 < args.size()) {
      a.topic_pos = args[++i];
    } else if (args[i] == "--out" && i + 1 < args.size()) {
      a.output_bag = args[++i];
    } else if (args[i] == "--storage" && i + 1 < args.size()) {
      a.storage_id = args[++i];
    } else if (args[i] == "--eph-mode" && i + 1 < args.size()) {
      std::string m = args[++i];
      if (m == "per-epoch")      a.eph_mode = EphMode::PerEpoch;
      else if (m == "on-change") a.eph_mode = EphMode::OnChange;
      else {
        std::cerr << "Error: unknown --eph-mode '" << m
                  << "' (expected per-epoch | on-change)\n";
        std::exit(2);
      }
    } else if (args[i] == "--max-dtoe-gnss" && i + 1 < args.size()) {
      a.max_dtoe_gnss = std::stod(args[++i]);
    } else if (args[i] == "--max-dtoe-glo" && i + 1 < args.size()) {
      a.max_dtoe_glonass = std::stod(args[++i]);
    } else if (args[i] == "--pos" && i + 1 < args.size()) {
      auto [label, path] = splitLabel(args[++i]);
      a.pos_label = label;
      a.pos_path  = path;
    } else if (args[i] == "--imu" && i + 1 < args.size()) {
      a.imu_path = args[++i];
    } else if (args[i] == "--topic-imu" && i + 1 < args.size()) {
      a.topic_imu = args[++i];
    } else if (args[i] == "--imu-frame" && i + 1 < args.size()) {
      a.imu_frame = args[++i];
    } else if (args[i] == "--reference" && i + 1 < args.size()) {
      a.reference_path = args[++i];
    } else if (args[i] == "--topic-reference" && i + 1 < args.size()) {
      a.topic_reference = args[++i];
    }
  }
  return a;
}

// Read a PPC-style IMU csv and write sensor_msgs/Imu to the bag. Header line is
// skipped; each row is "GPS TOW, GPS Week, Acc X,Y,Z [m/s^2], Ang Rate X,Y,Z
// [deg/s]". Gyro is converted deg->rad and both are mapped into the node's z-up
// body frame by applyImuFrame(imu_frame). Timestamps are GPST from week/tow, so
// they share the GNSS time base. Returns the number of samples written (-1 on
// a fatal error).
static long writeImuCsv(rosbag2_cpp::Writer& writer, const std::string& path,
                        const std::string& topic, const std::string& frame,
                        std::ostream& err) {
  std::ifstream ifs(path);
  if (!ifs) { err << "Error: cannot open --imu file '" << path << "'\n"; return -1; }
  std::string line;
  std::getline(ifs, line);  // header
  long n = 0, n_skip = 0;
  const double kDeg2Rad = 3.14159265358979323846 / 180.0;
  while (std::getline(ifs, line)) {
    if (line.empty()) continue;
    for (char& c : line) if (c == ',') c = ' ';
    std::istringstream iss(line);
    double tow; int week;
    ImuTriplet acc{}, gyr{};
    if (!(iss >> tow >> week >> acc.x >> acc.y >> acc.z >> gyr.x >> gyr.y >> gyr.z)) {
      if (++n_skip <= 5) err << "[imu] skipped malformed line: " << line << "\n";
      continue;
    }
    gyr.x *= kDeg2Rad; gyr.y *= kDeg2Rad; gyr.z *= kDeg2Rad;
    if (!applyImuFrame(frame, acc, gyr)) {
      err << "Error: unknown --imu-frame '" << frame
          << "' (expected frd2flu_grav | frd2flu | raw)\n";
      return -1;
    }
    gtime_t t = gpst2time(week, tow);
    rclcpp::Time stamp = gnss_converter_io::toRosTimeGpst(t);
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "imu_link";
    msg.linear_acceleration.x = acc.x;
    msg.linear_acceleration.y = acc.y;
    msg.linear_acceleration.z = acc.z;
    msg.angular_velocity.x = gyr.x;
    msg.angular_velocity.y = gyr.y;
    msg.angular_velocity.z = gyr.z;
    msg.orientation_covariance[0] = -1.0;  // orientation unknown (REP-145)
    writer.write(msg, topic, stamp);
    ++n;
  }
  return n;
}

// Ground-truth trajectory csv -> GnssSolution + nav_msgs/Odometry.
//
// Columns (PPC-Dataset reference.csv):
//   GPS TOW [s], GPS Week, Lat [deg], Lon [deg], Ellipsoid Height [m],
//   ECEF X/Y/Z [m], Roll [deg], Pitch [deg], Heading [deg],
//   East/North/Up Velocity [m/s]
//
// Two topics because one message cannot carry the whole state: GnssSolution has
// no attitude field, and a trajectory reference without attitude cannot check
// an estimator that produces attitude. The Odometry companion follows the same
// "<solution>_odom" convention gnss_imu_fgo already publishes on, so the same
// consumer reads truth and estimate the same way.
//
// HEADING CONVENTION - the one thing here that can be silently wrong.
// The csv Heading is degrees clockwise from North; the ROS/ENU convention is
// yaw counter-clockwise from East, so yaw_enu = 90 - heading. A sign or offset
// error produces a perfectly plausible-looking bag, so it is verified rather
// than asserted: reference.csv also carries East/North velocity, and
// atan2(VN, VE) is the course over ground in the SAME convention the quaternion
// must use. test/ppc_eval/verify_bag_reference.py regresses one against the
// other and requires slope +1, intercept 0. (This is the identical trap that
// let a wrong IMU body frame survive in this converter for months: the check
// that was performed could not have detected it.)
static long writeReferenceCsv(rosbag2_cpp::Writer& writer,
                              const std::string& path, const std::string& topic,
                              std::ostream& err) {
  std::ifstream ifs(path);
  if (!ifs) {
    err << "Error: cannot open --reference file '" << path << "'\n";
    return -1;
  }
  const std::string odom_topic = topic + "_odom";
  std::string line;
  std::getline(ifs, line);  // header
  long n = 0, n_skip = 0;
  const double kDeg2Rad = 3.14159265358979323846 / 180.0;
  while (std::getline(ifs, line)) {
    if (line.empty()) continue;
    for (char& c : line) if (c == ',') c = ' ';
    std::istringstream iss(line);
    double tow, lat, lon, hgt, ex, ey, ez, roll, pitch, heading, ve, vn, vu;
    int week;
    if (!(iss >> tow >> week >> lat >> lon >> hgt >> ex >> ey >> ez >> roll >>
          pitch >> heading >> ve >> vn >> vu)) {
      if (++n_skip <= 5) err << "[reference] skipped malformed line: " << line << "\n";
      continue;
    }
    const gtime_t t = gpst2time(week, tow);
    const rclcpp::Time stamp = gnss_converter_io::toRosTimeGpst(t);

    gnss_ros_standardization::msg::GnssSolution sol;
    sol.header.stamp = stamp;
    sol.header.frame_id = "gnss_link";
    sol.time_week = static_cast<uint32_t>(week);
    sol.time_tow = tow;
    // The PPC reference is a post-processed tightly-coupled GNSS/INS product
    // from the receiver vendor's suite, not something computed here.
    sol.solution_source = gnss_ros_standardization::msg::GnssSolution::SOLUTION_SOURCE_BINARY;
    sol.status = gnss_ros_standardization::msg::GnssSolution::STATUS_FIX;
    sol.latitude = lat;
    sol.longitude = lon;
    sol.altitude = hgt;
    sol.pos_ecef.x = ex;
    sol.pos_ecef.y = ey;
    sol.pos_ecef.z = ez;
    sol.vel_enu.x = ve;
    sol.vel_enu.y = vn;
    sol.vel_enu.z = vu;
    // ENU -> ECEF for the velocity, so both frames agree in one message.
    {
      const double llh[3] = {lat * kDeg2Rad, lon * kDeg2Rad, hgt};
      const double enu[3] = {ve, vn, vu};
      double ecef_v[3];
      enu2ecef(llh, enu, ecef_v);
      sol.vel_ecef.x = ecef_v[0];
      sol.vel_ecef.y = ecef_v[1];
      sol.vel_ecef.z = ecef_v[2];
    }
    const float nan = std::numeric_limits<float>::quiet_NaN();
    sol.gdop = sol.pdop = sol.hdop = sol.vdop = nan;
    writer.write(sol, topic, stamp);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";
    // ENU yaw from a North-referenced clockwise heading (see the note above).
    const double yaw_enu = (90.0 - heading) * kDeg2Rad;
    const double cr = std::cos(roll * kDeg2Rad * 0.5);
    const double sr = std::sin(roll * kDeg2Rad * 0.5);
    const double cp = std::cos(pitch * kDeg2Rad * 0.5);
    const double sp = std::sin(pitch * kDeg2Rad * 0.5);
    const double cy = std::cos(yaw_enu * 0.5);
    const double sy = std::sin(yaw_enu * 0.5);
    odom.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy;
    odom.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy;
    odom.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy;
    odom.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy;
    odom.twist.twist.linear.x = ve;
    odom.twist.twist.linear.y = vn;
    odom.twist.twist.linear.z = vu;
    writer.write(odom, odom_topic, stamp);
    ++n;
  }
  if (n_skip > 5) {
    err << "[reference] " << n_skip << " malformed lines skipped in total\n";
  }
  return n;
}

// Resolve each ObsSource.topic from CLI overrides, label-derived auto names,
// and the single-source compatibility fallback. Returns false on validation
// failure (caller prints + exits).
static bool resolveObsTopics(Args& a, std::ostream& err) {
  // 1. Check for duplicate labels and multiple unlabeled sources.
  std::map<std::string, int> label_counts;
  for (const auto& s : a.obs_sources) label_counts[s.label]++;
  for (const auto& [lbl, n] : label_counts) {
    if (n > 1) {
      if (lbl.empty()) {
        err << "Error: multiple --obs without a name= label. "
               "Use --obs name=path (e.g. --obs rover=rover.obs --obs base=base.obs).\n";
      } else {
        err << "Error: duplicate --obs label '" << lbl << "'.\n";
      }
      return false;
    }
  }

  // 2. Validate that every --topic-obs label corresponds to an --obs source.
  for (const auto& [lbl, topic] : a.topic_obs_overrides) {
    if (label_counts.count(lbl) == 0) {
      err << "Error: --topic-obs label '" << lbl
          << "' does not match any --obs source.\n";
      return false;
    }
  }

  // 3. Assign topics: override > /<label>/observation > /gnss/observation.
  for (auto& src : a.obs_sources) {
    auto it = a.topic_obs_overrides.find(src.label);
    if (it != a.topic_obs_overrides.end()) {
      src.topic = it->second;
    } else if (!src.label.empty()) {
      src.topic = "/" + src.label + "/observation";
    } else {
      src.topic = "/gnss/observation";
    }
  }

  // 4. If --pos has a label, it must match an --obs label.
  if (!a.pos_path.empty() && !a.pos_label.empty() &&
      label_counts.count(a.pos_label) == 0) {
    err << "Error: --pos label '" << a.pos_label
        << "' does not match any --obs source.\n";
    return false;
  }
  return true;
}

// Select the best ephemeris index for the current time. Works for both eph_t
// and geph_t (any RTKLIB type with a `toe` field).
//
// The selection MIRRORS RTKLIB's seleph() (ephemeris.c) so that a node
// consuming the bag makes the same per-epoch choice rnx2rtkp makes reading the
// RINEX directly:
//   - smallest |t_curr - toe| within the per-system window, FUTURE toe
//     included (that is what seleph does for GPS/QZS/BDS/IRN);
//   - EXCEPT Galileo, where seleph requires toe strictly in the PAST
//     ("AOD<=0") - set require_past_toe. Without this, nearest-|toe| selects a
//     future record and the downstream seleph rejects the satellite outright:
//     measured on PPC tokyo_run1, E36/E09 carried a toe ~25 min in the future
//     from epoch 0 and were unusable for the first 1530 s of the replay.
// (A stricter transmission-time causality filter - only records with
// ttr <= t_curr, like a live receiver - was tried and REJECTED: seleph does
// not apply it, so it broke selection parity with the CLI for GPS/BDS/QZS and
// measurably cost satellite availability early in ephemeris windows.)
//
// On exact ties, prefer the later array index - since RTKLIB appends records
// in file order, "later index" means "issued more recently in the source
// RINEX nav file", which is the safer choice when two ephemerides share the
// same TOE.
template <typename T>
int select_best_eph(const std::vector<int>& indices, const T* data_array,
                    gtime_t t_curr, double max_dtoe,
                    bool require_past_toe = false) {
    int best_idx = -1;
    double min_diff = std::numeric_limits<double>::max();
    for (int idx : indices) {
        if (require_past_toe &&
            timediff(data_array[idx].toe, t_curr) >= 0.0) continue;
        double diff = std::fabs(timediff(t_curr, data_array[idx].toe));
        if (diff > max_dtoe) continue;
        if (diff < min_diff) { min_diff = diff; best_idx = idx; }
        else if (diff == min_diff && idx > best_idx) best_idx = idx;
    }
    return best_idx;
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  std::vector<std::string> args_vec;
  for (int i=1; i<argc; ++i) args_vec.push_back(argv[i]);
  Args args = parseArgs(args_vec);

  if (args.output_bag.empty() && !args.obs_sources.empty())
    args.output_bag = gnss_converter_io::deriveOutputPath(args.obs_sources.front().path, "");

  if (args.obs_sources.empty()) {
    std::cerr <<
      "Usage: rinex_to_rosbag --obs [name=]<obs_file> [--obs [name=]<obs_file> ...]\n"
      "                       [--nav <nav_file> ...]\n"
      "                       [--topic-obs [name=]<topic>] [--topic-nav <topic>]\n"
      "                       [--topic-pos <topic>]\n"
      "                       [--out <bag_path>] [--storage mcap|sqlite3]\n"
      "                       [--eph-mode per-epoch|on-change]\n"
      "                       [--max-dtoe-gnss <sec>] [--max-dtoe-glo <sec>]\n"
      "                       [--pos [name=]<pos_file>]\n"
      "                       [--imu <imu_csv>] [--topic-imu <topic>]\n"
      "                       [--reference <csv>] [--topic-reference <topic>]\n"
      "                       [--imu-frame raw|frd2flu|frd2flu_grav]\n"
      "\n"
      "  --obs         Repeatable. Use 'name=path' (e.g. rover=rover.obs) to give the\n"
      "                source a label; topic becomes /<name>/observation. Without a\n"
      "                label the topic is /gnss/observation (single-source mode).\n"
      "                Epochs from all sources are interleaved in GPST time order so\n"
      "                'ros2 bag play' reproduces the real-time arrival pattern.\n"
      "  --topic-obs   Override topic per source: 'name=topic' (or bare 'topic' for the\n"
      "                unlabeled source). Default: /<name>/observation, or /gnss/observation.\n"
      "  --topic-nav   NAV topic name (default /gnss/ephemeris). NAV is shared across\n"
      "                all OBS sources (single ephemeris stream).\n"
      "  --topic-pos   POS topic name (default /gnss/solution).\n"
      "  --pos         Optional RTKLIB .pos file. Use 'name=path' to bind it to a\n"
      "                specific OBS source (e.g. --pos rover=rover.pos). Without a\n"
      "                label, binds to the first --obs source. Solution rows are\n"
      "                interleaved at that source's epoch boundaries.\n"
      "  --eph-mode    per-epoch: republish all visible ephemerides every obs epoch.\n"
      "                on-change (default): publish per-sat only when the selected\n"
      "                     nav index changes (avoids massive duplication; first\n"
      "                     visible epoch of each sat still publishes once).\n"
      "  --max-dtoe-*  max |t_obs - toe| for ephemeris selection [sec]. Defaults:\n"
      "                GNSS=" << kDefaultMaxDtoeGnss
      << ", GLONASS=" << kDefaultMaxDtoeGlonass << ".\n"
      "  --imu         Optional IMU csv (GPS TOW, Week, Acc XYZ [m/s^2], Ang Rate XYZ\n"
      "                [deg/s]; e.g. PPC-Dataset imu.csv). Written as sensor_msgs/Imu\n"
      "                to --topic-imu (default /gnss/imu/data_raw), GPST-stamped.\n"
      "  --imu-frame   Body-frame adapter into the node's z-up (ENU) body frame:\n"
      "                raw (default; source already FLU specific force - correct for\n"
      "                PPC-Dataset), frd2flu (source FRD, accel reports specific\n"
      "                force), frd2flu_grav (source FRD, accel reports gravity dir).\n"
      "                Verify: accel x must correlate +1 with d|v|/dt and gyro z\n"
      "                +1 with the reference course rate; a wrong mode is silent.\n"
      "  --reference   Optional ground-truth trajectory csv (GPS TOW, Week, Lat, Lon,\n"
      "                Height, ECEF XYZ, Roll, Pitch, Heading [deg], E/N/U velocity;\n"
      "                e.g. PPC-Dataset reference.csv). Written GPST-stamped to TWO\n"
      "                topics so the whole state survives: --topic-reference\n"
      "                (default /gnss/reference/solution, GnssSolution: position and\n"
      "                velocity) and <topic>_odom (nav_msgs/Odometry: attitude).\n"
      "                Deliberately separate from --topic-pos so a reference can\n"
      "                never be mistaken for an estimator's own output.\n"
      "                Heading is North-referenced clockwise; the Odometry yaw is\n"
      "                ENU (90-heading). Verify with\n"
      "                test/ppc_eval/verify_bag_reference.py - a wrong convention\n"
      "                produces a plausible-looking bag.\n"
      "  When --nav is omitted, the NAV topic is created but empty.\n";
    return 1;
  }

  if (!resolveObsTopics(args, std::cerr)) return 1;

  // 1. Read each OBS source. unique_ptr<ObsGuard> keeps ObsGuard non-movable
  //    requirements satisfied while letting us hold many in a vector.
  std::vector<std::unique_ptr<ObsGuard>> obs_guards;
  NavGuard nav_tmp_g;  // throwaway nav from OBS reads (a few mixed records)
  for (const auto& src : args.obs_sources) {
    auto og = std::make_unique<ObsGuard>();
    sta_t sta = {};
    int obs_rc = readrnx(src.path.c_str(), 1, "", &og->obs, &nav_tmp_g.nav, &sta);
    if (obs_rc <= 0 || og->obs.n <= 0) {
      std::cerr << "Error: failed to read observations from '" << src.path
                << "' (readrnx rc=" << obs_rc << ", n=" << og->obs.n
                << "). Check file path and RINEX OBS format.\n";
      return 1;
    }
    obs_guards.push_back(std::move(og));
  }

  // 2. Read NAV data into a single accumulating nav_t (shared across sources).
  NavGuard ng;
  size_t nav_ok = 0;
  for (const auto& path : args.nav_paths) {
    ObsGuard dummy_og;
    sta_t dummy_sta = {};
    int nav_rc = readrnx(path.c_str(), 1, "", &dummy_og.obs, &ng.nav, &dummy_sta);
    if (nav_rc <= 0) {
      std::cerr << "Warning: failed to read nav file '" << path
                << "' (readrnx rc=" << nav_rc << "). Skipping.\n";
    } else {
      ++nav_ok;
    }
  }
  if (!args.nav_paths.empty() && nav_ok == 0) {
    std::cerr << "Error: all --nav files failed to load; no ephemerides available.\n";
    return 1;
  }

  // 3. Index nav for fast per-satellite lookup.
  std::map<int, std::vector<int>> sat_eph_indices;
  std::map<int, std::vector<int>> sat_geph_indices;
  for (int i = 0; i < ng.nav.n; ++i) {
    const auto& e = ng.nav.eph[i];
    if (e.toe.time != 0 && e.sat > 0 && satsys(e.sat, NULL) != SYS_NONE) {
      sat_eph_indices[e.sat].push_back(i);
    }
  }
  for (int i = 0; i < ng.nav.ng; ++i) {
    const auto& g = ng.nav.geph[i];
    if (g.toe.time != 0 && g.sat > 0 && satsys(g.sat, NULL) == SYS_GLO) {
      sat_geph_indices[g.sat].push_back(i);
    }
  }

  // 4. Optionally preload .pos rows for solution interleaving, bound to one source.
  gnss_pos_reader::ParseResult pos_result;
  int pos_src_idx = -1;
  if (!args.pos_path.empty()) {
    pos_result = gnss_pos_reader::parsePosFile(args.pos_path, std::cerr);
    if (pos_result.rows.empty()) {
      std::cerr << "Warning: --pos file produced no rows; solution topic will be absent.\n";
    } else {
      // Resolve which OBS source this pos is bound to.
      if (args.pos_label.empty()) {
        pos_src_idx = 0;
      } else {
        for (size_t s = 0; s < args.obs_sources.size(); ++s) {
          if (args.obs_sources[s].label == args.pos_label) { pos_src_idx = (int)s; break; }
        }
      }
      // Sanity check: warn if pos and bound-source obs time ranges don't overlap.
      const auto& og_bound = obs_guards[pos_src_idx]->obs;
      if (og_bound.n > 0) {
        gtime_t obs_first = og_bound.data[0].time;
        gtime_t obs_last  = og_bound.data[og_bound.n - 1].time;
        gtime_t pos_first = pos_result.rows.front().t;
        gtime_t pos_last  = pos_result.rows.back().t;
        if (timediff(pos_last, obs_first) < -1.0 || timediff(pos_first, obs_last) > 1.0) {
          std::cerr << "Warning: --pos time range does not overlap with obs time range for "
                    << "source '" << args.obs_sources[pos_src_idx].label
                    << "'. Verify the .pos file was generated from the same session.\n";
        }
      }
    }
  }

  // 5. Setup bag writer. Scoped so the writer closes before the post-loop
  //    cleanup check (remove_all requires the file handles to be released first).
  size_t n_obs_epochs = 0;
  size_t n_eph_msgs   = 0;
  size_t n_sol_msgs   = 0;
  long   n_imu_msgs   = 0;
  long   n_ref_msgs   = 0;
  {
    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions storage_opts;
    storage_opts.uri = args.output_bag;
    storage_opts.storage_id =
        gnss_converter_io::resolveStorageId(args.storage_id, args.output_bag);

    rosbag2_cpp::ConverterOptions converter_opts;
    converter_opts.input_serialization_format = "cdr";
    converter_opts.output_serialization_format = "cdr";

    try {
      writer.open(storage_opts, converter_opts);
    } catch (const std::exception& e) {
      std::cerr << "Failed to open bag: " << e.what() << "\n";
      rclcpp::shutdown();
      return 1;
    }

    rosbag2_storage::TopicMetadata tm;
    tm.serialization_format = "cdr";

    // Per-source observation topic (one create_topic per source).
    gnss_converter_io::set_qos_profile(tm, gnss_converter_io::defaultConverterQosYaml());
    for (const auto& src : args.obs_sources) {
      tm.name = src.topic;
      tm.type = "gnss_ros_standardization/msg/GnssObservations";
      writer.create_topic(tm);
    }

    // NAV topic is consumed by positioning nodes (SPP, RTK, gnss_visualizer)
    // with transient_local() subscriptions — use matching TRANSIENT_LOCAL durability
    // so late subscribers receive the last published ephemeris immediately.
    gnss_converter_io::set_qos_profile(tm, gnss_converter_io::transientLocalConverterQosYaml());
    tm.name = args.topic_nav;
    tm.type = "gnss_ros_standardization/msg/GnssEphemerides";
    writer.create_topic(tm);

    if (!pos_result.rows.empty()) {
      gnss_converter_io::set_qos_profile(tm, gnss_converter_io::defaultConverterQosYaml());
      tm.name = args.topic_pos;
      tm.type = "gnss_ros_standardization/msg/GnssSolution";
      writer.create_topic(tm);
    }

    if (!args.imu_path.empty()) {
      gnss_converter_io::set_qos_profile(tm, gnss_converter_io::defaultConverterQosYaml());
      tm.name = args.topic_imu;
      tm.type = "sensor_msgs/msg/Imu";
      writer.create_topic(tm);
    }

    if (!args.reference_path.empty()) {
      gnss_converter_io::set_qos_profile(tm, gnss_converter_io::defaultConverterQosYaml());
      tm.name = args.topic_reference;
      tm.type = "gnss_ros_standardization/msg/GnssSolution";
      writer.create_topic(tm);
      tm.name = args.topic_reference + "_odom";
      tm.type = "nav_msgs/msg/Odometry";
      writer.create_topic(tm);
    }

    // "Last published nav index" for --eph-mode on-change, keyed by
    // sat*4 + nav-type group (Galileo I/NAV and F/NAV are independent
    // streams; see the emission loop). Shared across all OBS sources because
    // NAV is shared.
    std::map<int, int> last_eph_idx;
    std::map<int, int> last_geph_idx;

    // Cursor into pos rows for the bound source.
    size_t pos_idx = 0;

    // 6. Time-interleaved epoch processing using a min-heap across sources.
    //    Each heap entry is the next-unprocessed epoch start of one source.
    struct HeapItem {
      gtime_t t;
      int     src_idx;
    };
    struct HeapCmp {
      bool operator()(const HeapItem& a, const HeapItem& b) const {
        double dt = timediff(a.t, b.t);
        if (std::fabs(dt) > 1e-9) return dt > 0.0;   // earlier time first
        return a.src_idx > b.src_idx;                // stable tie-break by src
      }
    };
    std::priority_queue<HeapItem, std::vector<HeapItem>, HeapCmp> heap;
    std::vector<int> cursor(args.obs_sources.size(), 0);
    for (size_t s = 0; s < args.obs_sources.size(); ++s) {
      if (obs_guards[s]->obs.n > 0) {
        heap.push({obs_guards[s]->obs.data[0].time, (int)s});
      }
    }

    constexpr size_t kProgressEvery = 1000;
    while (!heap.empty()) {
      HeapItem top = heap.top(); heap.pop();
      const int src_idx = top.src_idx;
      const auto& obs = obs_guards[src_idx]->obs;
      const auto& src = args.obs_sources[src_idx];

      gtime_t t_curr = top.t;
      rclcpp::Time stamp = gnss_converter_io::toRosTimeGpst(t_curr);

      GnssObservations msg_obs;
      int week = 0;
      msg_obs.tow = time2gpst(t_curr, &week);
      msg_obs.week = week;
      msg_obs.header.stamp = stamp;
      msg_obs.header.frame_id = src.label.empty() ? "gnss_receiver" : src.label;

      GnssEphemerides msg_eph;
      msg_eph.header.stamp = stamp;
      msg_eph.header.frame_id = "gnss_receiver";

      std::vector<int> epoch_sats;

      int i = cursor[src_idx];
      int j = i;
      while (j < obs.n) {
        if (timediff(obs.data[j].time, t_curr) > 1e-3) break;
        const obsd_t& d = obs.data[j];

        bool has_signal = false;
        for (int k = 0; k < NFREQ + NEXOBS; ++k) {
          if (d.code[k] || d.P[k] != 0.0 || d.L[k] != 0.0) {
            msg_obs.observations.push_back(gnss_utils::obsToMsg(d, k));
            has_signal = true;
          }
        }
        if (has_signal) epoch_sats.push_back(d.sat);
        j++;
      }

      std::sort(epoch_sats.begin(), epoch_sats.end());
      epoch_sats.erase(std::unique(epoch_sats.begin(), epoch_sats.end()),
                       epoch_sats.end());

      // Select best ephemeris for each observed satellite.
      // For on-change mode, only emit when the selected nav index changes
      // (dictionaries are shared across sources since NAV is shared).
      for (int sat : epoch_sats) {
        int sys = satsys(sat, NULL);
        if (sys == SYS_GLO) {
          if (sat_geph_indices.count(sat)) {
            int idx = select_best_eph(sat_geph_indices[sat], ng.nav.geph,
                                      t_curr, args.max_dtoe_glonass);
            if (idx == -1) continue;
            if (args.eph_mode == EphMode::OnChange) {
              auto it = last_geph_idx.find(sat);
              if (it != last_geph_idx.end() && it->second == idx) continue;
              last_geph_idx[sat] = idx;
            }
            msg_eph.glonass_ephemeris.push_back(
                gnss_utils::gephToMsg(ng.nav.geph[idx]));
          }
        } else {
          if (sat_eph_indices.count(sat)) {
            // Galileo broadcasts two independent navigation messages (I/NAV on
            // E1B/E5b: eph.code bit 9; F/NAV on E5a: bit 8) and RTKLIB's
            // seleph() selects BY TYPE (default: I/NAV only). Selecting the
            // nearest-toe record across both types therefore lets an F/NAV
            // record shadow its I/NAV twin under on-change emission, and the
            // downstream node then sees NO usable ephemeris for that satellite
            // until the next I/NAV change - measured to remove E36/E09 from
            // half of a 40-minute run. Select and emit the best record PER
            // navigation type.
            std::map<int, std::vector<int>> nav_type_groups;
            for (int i : sat_eph_indices[sat]) {
              int g = 0;
              if (sys == SYS_GAL) {
                g = (ng.nav.eph[i].code & (1 << 9)) ? 1 : 2;  // I/NAV : F/NAV
              }
              nav_type_groups[g].push_back(i);
            }
            // Per-system validity window, as in seleph() (Galileo and BeiDou
            // ephemerides are valid far longer than GPS's 2 h; capping them at
            // the GPS window starves satellites the CLI happily uses).
            double tmax = args.max_dtoe_gnss;
            if (sys == SYS_GAL) tmax = std::max(tmax, double(MAXDTOE_GAL));
            if (sys == SYS_CMP) tmax = std::max(tmax, MAXDTOE_CMP + 1.0);
            for (const auto& kv : nav_type_groups) {
              int idx = select_best_eph(kv.second, ng.nav.eph, t_curr, tmax,
                                        /*require_past_toe=*/sys == SYS_GAL);
              if (idx == -1) continue;
              if (args.eph_mode == EphMode::OnChange) {
                const int key = sat * 4 + kv.first;
                auto it = last_eph_idx.find(key);
                if (it != last_eph_idx.end() && it->second == idx) continue;
                last_eph_idx[key] = idx;
              }
              msg_eph.gnss_ephemeris.push_back(
                  gnss_utils::ephToMsg(ng.nav.eph[idx]));
            }
          }
        }
      }

      writer.write(msg_obs, src.topic, stamp);
      ++n_obs_epochs;

      if (!msg_eph.gnss_ephemeris.empty() || !msg_eph.glonass_ephemeris.empty()) {
        writer.write(msg_eph, args.topic_nav, stamp);
        ++n_eph_msgs;
      }

      // Flush any pos rows with t <= t_curr (only when this is the bound source).
      if (src_idx == pos_src_idx) {
        while (pos_idx < pos_result.rows.size()) {
          if (timediff(pos_result.rows[pos_idx].t, t_curr) > 1e-3) break;
          rclcpp::Time ps = gnss_converter_io::toRosTimeGpst(pos_result.rows[pos_idx].t);
          writer.write(pos_result.rows[pos_idx].sol, args.topic_pos, ps);
          ++pos_idx;
          ++n_sol_msgs;
        }
      }

      if (n_obs_epochs % kProgressEvery == 0) {
        std::cerr << "[rinex_to_rosbag] " << n_obs_epochs << " obs epochs, "
                  << n_eph_msgs << " eph msgs written\n";
      }

      cursor[src_idx] = j;
      if (j < obs.n) heap.push({obs.data[j].time, src_idx});
    }

    // Flush any pos rows that follow the last obs epoch of the bound source.
    while (pos_idx < pos_result.rows.size()) {
      rclcpp::Time ps = gnss_converter_io::toRosTimeGpst(pos_result.rows[pos_idx].t);
      writer.write(pos_result.rows[pos_idx].sol, args.topic_pos, ps);
      ++pos_idx;
      ++n_sol_msgs;
    }

    // IMU is written in its own pass; 'ros2 bag play' replays by timestamp so
    // the higher-rate IMU interleaves correctly with the GNSS epochs regardless
    // of storage order.
    if (!args.reference_path.empty()) {
      n_ref_msgs = writeReferenceCsv(writer, args.reference_path,
                                     args.topic_reference, std::cerr);
      if (n_ref_msgs < 0) return 1;
    }
    if (!args.imu_path.empty()) {
      n_imu_msgs = writeImuCsv(writer, args.imu_path, args.topic_imu,
                               args.imu_frame, std::cerr);
      if (n_imu_msgs < 0) return 1;
    }

    std::cerr << "[rinex_to_rosbag] done: " << n_obs_epochs
              << " obs epochs, " << n_eph_msgs << " eph msgs";
    if (!pos_result.rows.empty())
      std::cerr << ", " << n_sol_msgs << " solution msgs";
    if (!args.imu_path.empty())
      std::cerr << ", " << n_imu_msgs << " imu msgs";
    if (!args.reference_path.empty())
      std::cerr << ", " << n_ref_msgs << " reference msgs (x2 topics)";
    std::cerr << " -> " << args.output_bag << "\n";
  }  // writer closed here

  rclcpp::shutdown();
  if (n_obs_epochs == 0) {
    std::filesystem::remove_all(args.output_bag);
    std::fprintf(stderr, "[rinex_to_rosbag] error: no obs epochs written; removed %s\n",
                 args.output_bag.c_str());
    return 1;
  }
  return 0;
}
