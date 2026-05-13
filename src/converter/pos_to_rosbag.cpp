// Convert an RTKLIB .pos file to a ROS 2 bag of GnssSolution messages.
//
// Supports both lat/lon/height (LLH) and x/y/z-ecef (ECEF) variants, with
// either GPST date-time columns ("yyyy/mm/dd hh:mm:ss.sss") or GPST
// week + seconds-of-week columns. Format is auto-detected from header
// comments and the first data row.

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

using gnss_ros_standardization::msg::GnssSolution;

struct Args {
  std::string pos_path;
  std::string output_bag = "pos_converted";
  std::string topic = "/gnss/solution";
  std::string storage_id;  // empty => auto-detect from --out extension or distro default
};

static Args parseArgs(int argc, char** argv) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    std::string s = argv[i];
    auto next = [&]() {
      if (i + 1 >= argc) throw std::runtime_error("missing value for " + s);
      return std::string(argv[++i]);
    };
    if      (s == "--pos")     a.pos_path    = next();
    else if (s == "--out")     a.output_bag  = next();
    else if (s == "--topic")   a.topic       = next();
    else if (s == "--storage") a.storage_id  = next();
    else std::fprintf(stderr, "[warn] unknown arg %s\n", s.c_str());
  }
  return a;
}

// Resolve rosbag2 storage_id: explicit override > extension auto-detect > "" (distro default).
static std::string resolveStorageId(const std::string& override_id,
                                    const std::string& output_uri) {
  if (!override_id.empty()) return override_id;
  auto pos = output_uri.find_last_of('.');
  if (pos == std::string::npos) return "";
  std::string ext = output_uri.substr(pos);
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c){ return std::tolower(c); });
  if (ext == ".mcap") return "mcap";
  if (ext == ".db3")  return "sqlite3";
  return "";
}

static rclcpp::Time toRosTime(gtime_t t) {
  long sec     = static_cast<long>(t.time);
  long nanosec = static_cast<long>(t.sec * 1e9);
  if (nanosec >= 1000000000) { sec += nanosec / 1000000000; nanosec %= 1000000000; }
  else if (nanosec < 0) {
    long roll = (-nanosec + 999999999) / 1000000000;
    sec -= roll; nanosec += roll * 1000000000;
  }
  if (sec < 0) sec = 0;
  return rclcpp::Time(sec, nanosec);
}

static uint8_t qToStatus(int Q) {
  switch (Q) {
    case 1: return GnssSolution::STATUS_FIX;
    case 2: return GnssSolution::STATUS_FLOAT;
    case 3: return GnssSolution::STATUS_SBAS;
    case 4: return GnssSolution::STATUS_DGPS;
    case 5: return GnssSolution::STATUS_SINGLE;
    case 6: return GnssSolution::STATUS_PPP;
    default: return GnssSolution::STATUS_NONE;
  }
}

static std::vector<std::string> splitTokens(const std::string& s) {
  std::vector<std::string> v;
  std::istringstream iss(s);
  std::string t;
  while (iss >> t) v.push_back(t);
  return v;
}

template <typename T>
static void set_qos_profile(T& tm, const std::string& profile) {
  if constexpr (std::is_same_v<std::decay_t<decltype(tm.offered_qos_profiles)>, std::string>) {
    tm.offered_qos_profiles = profile;
  } else if constexpr (std::is_same_v<std::decay_t<decltype(tm.offered_qos_profiles)>,
                                      std::vector<rclcpp::QoS>>) {
    tm.offered_qos_profiles.push_back(rclcpp::QoS(1).reliable().durability_volatile());
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  Args args = parseArgs(argc, argv);

  if (args.pos_path.empty()) {
    std::fprintf(stderr,
      "Usage: pos_to_rosbag --pos <input.pos> [--out <bag>] "
      "[--topic /gnss/solution] [--storage mcap|sqlite3]\n");
    return 2;
  }

  std::ifstream ifs(args.pos_path);
  if (!ifs) {
    std::perror(args.pos_path.c_str());
    return 1;
  }

  bool is_ecef = false;
  bool format_detected = false;
  std::string line;

  // Parse header (lines starting with '%') to detect ECEF vs LLH.
  std::streampos data_start = 0;
  while (true) {
    data_start = ifs.tellg();
    if (!std::getline(ifs, line)) break;
    if (line.empty()) continue;
    if (line[0] != '%') {
      ifs.seekg(data_start);
      break;
    }
    if (!format_detected) {
      if (line.find("x/y/z-ecef") != std::string::npos ||
          line.find("x-ecef") != std::string::npos) {
        is_ecef = true; format_detected = true;
      } else if (line.find("lat/lon/height") != std::string::npos ||
                 line.find("latitude")       != std::string::npos) {
        is_ecef = false; format_detected = true;
      }
    }
  }

  rosbag2_cpp::Writer writer;
  rosbag2_storage::StorageOptions storage_opts;
  storage_opts.uri = args.output_bag;
  storage_opts.storage_id = resolveStorageId(args.storage_id, args.output_bag);
  rosbag2_cpp::ConverterOptions converter_opts;
  converter_opts.input_serialization_format  = "cdr";
  converter_opts.output_serialization_format = "cdr";

  try {
    writer.open(storage_opts, converter_opts);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "Failed to open bag: %s\n", e.what());
    return 1;
  }

  const std::string qos_profile =
      "- history: 1\n"
      "  depth: 1\n"
      "  reliability: 1\n"
      "  durability: 2\n"
      "  deadline:\n    sec: 0\n    nsec: 0\n"
      "  lifespan:\n    sec: 0\n    nsec: 0\n"
      "  liveliness: 0\n"
      "  liveliness_lease_duration:\n    sec: 0\n    nsec: 0\n"
      "  avoid_ros_namespace_conventions: false";

  rosbag2_storage::TopicMetadata tm;
  tm.serialization_format = "cdr";
  set_qos_profile(tm, qos_profile);
  tm.name = args.topic;
  tm.type = "gnss_ros_standardization/msg/GnssSolution";
  writer.create_topic(tm);

  size_t n_rows = 0;

  while (std::getline(ifs, line)) {
    if (line.empty()) continue;
    if (line[0] == '%') continue;

    auto tok = splitTokens(line);
    if (tok.size() < 8) continue;

    // Time parsing — first token determines format.
    gtime_t t{};
    size_t off = 0;
    if (tok[0].find('/') != std::string::npos) {
      // "yyyy/mm/dd HH:MM:SS.sss"
      std::string ts = tok[0] + " " + tok[1];
      if (str2time(ts.c_str(), 0, (int)ts.size(), &t) != 0) continue;
      off = 2;
    } else {
      // GPST week + seconds-of-week
      try {
        int week = std::stoi(tok[0]);
        double tow = std::stod(tok[1]);
        t = gpst2time(week, tow);
      } catch (...) { continue; }
      off = 2;
    }

    if (tok.size() < off + 6) continue;

    GnssSolution sol;
    rclcpp::Time stamp = toRosTime(t);
    sol.header.stamp = stamp;
    sol.header.frame_id = "gnss_receiver";
    int week = 0;
    sol.time_tow = time2gpst(t, &week);
    sol.time_week = static_cast<uint16_t>(week);

    try {
      const double a = std::stod(tok[off + 0]);
      const double b = std::stod(tok[off + 1]);
      const double c = std::stod(tok[off + 2]);
      const int    Q = std::stoi(tok[off + 3]);
      const int    ns = std::stoi(tok[off + 4]);

      sol.status   = qToStatus(Q);
      sol.num_sats = static_cast<uint8_t>(std::min(255, std::max(0, ns)));

      if (is_ecef) {
        sol.pos_ecef.x = a;
        sol.pos_ecef.y = b;
        sol.pos_ecef.z = c;
        double r[3] = {a, b, c};
        double pos[3] = {0, 0, 0};
        ecef2pos(r, pos);
        sol.latitude  = pos[0] * R2D;
        sol.longitude = pos[1] * R2D;
        sol.altitude  = pos[2];
      } else {
        sol.latitude  = a;
        sol.longitude = b;
        sol.altitude  = c;
        double pos[3] = {a * D2R, b * D2R, c};
        double r[3]   = {0, 0, 0};
        pos2ecef(pos, r);
        sol.pos_ecef.x = r[0];
        sol.pos_ecef.y = r[1];
        sol.pos_ecef.z = r[2];
      }

      // Optional sigma columns. RTKLIB writes 6 sigma values + age + ratio.
      if (tok.size() >= off + 11) {
        const double s0 = std::stod(tok[off + 5]);
        const double s1 = std::stod(tok[off + 6]);
        const double s2 = std::stod(tok[off + 7]);
        const double s3 = std::stod(tok[off + 8]);
        const double s4 = std::stod(tok[off + 9]);
        const double s5 = std::stod(tok[off + 10]);

        // RTKLIB convention: off-diagonal entries are sgn*sqrt(|cov|),
        // so cov = sgn(s)*s*s.
        auto sgnsq = [](double s) { return s >= 0 ? s * s : -s * s; };

        if (is_ecef) {
          // sdx, sdy, sdz, sdxy, sdyz, sdzx
          const double cxx = s0 * s0,    cyy = s1 * s1,    czz = s2 * s2;
          const double cxy = sgnsq(s3),  cyz = sgnsq(s4),  czx = sgnsq(s5);
          sol.pos_cov_ecef = {cxx, cxy, czx,
                              cxy, cyy, cyz,
                              czx, cyz, czz};
        } else {
          // sdn, sde, sdu, sdne, sdeu, sdun  — store as ENU (E,N,U) row-major.
          const double cnn = s0 * s0,    cee = s1 * s1,    cuu = s2 * s2;
          const double cne = sgnsq(s3),  ceu = sgnsq(s4),  cun = sgnsq(s5);
          sol.pos_enu_cov = {cee, cne, ceu,
                             cne, cnn, cun,
                             ceu, cun, cuu};
        }
      }

      if (tok.size() >= off + 12) sol.age_diff = static_cast<float>(std::stod(tok[off + 11]));
      if (tok.size() >= off + 13) sol.ratio    = static_cast<float>(std::stod(tok[off + 12]));
    } catch (...) {
      continue;
    }

    writer.write(sol, args.topic, stamp);
    ++n_rows;
  }

  std::fprintf(stderr, "Wrote %zu epochs (%s) to %s\n",
               n_rows, is_ecef ? "ECEF" : "LLH", args.output_bag.c_str());
  rclcpp::shutdown();
  return 0;
}
