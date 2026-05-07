// Convert a ROS 2 bag containing GnssSolution messages to an RTKLIB .pos file.
//
// Output format: standard RTKLIB lat/lon/height .pos with GPST date column.
// Q codes: 1=fix, 2=float, 3=sbas, 4=dgps, 5=single, 6=ppp.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using gnss_ros_standardization::msg::GnssSolution;

struct Args {
  std::string bag_uri;
  std::string topic = "/gnss/solution";
  std::string out_path;
  std::string program_name = "rosbag_to_pos";
};

static std::string normalizeBagUri(std::string uri) {
  while (!uri.empty() && (uri.back() == '/' || uri.back() == '\\')) uri.pop_back();
  if (uri.size() >= 4) {
    std::string tail = uri.substr(uri.size() - 4);
    for (auto& c : tail) c = (char)std::tolower((unsigned char)c);
    if (tail == ".db3") {
      auto pos = uri.find_last_of("/\\");
      if (pos != std::string::npos) {
        std::string dir = uri.substr(0, pos);
        return dir.empty() ? "." : dir;
      }
    }
  }
  return uri;
}

static int statusToQ(uint8_t status) {
  switch (status) {
    case GnssSolution::STATUS_FIX:    return 1;
    case GnssSolution::STATUS_FLOAT:  return 2;
    case GnssSolution::STATUS_SBAS:   return 3;
    case GnssSolution::STATUS_DGPS:   return 4;
    case GnssSolution::STATUS_SINGLE: return 5;
    case GnssSolution::STATUS_PPP:    return 6;
    default:                          return 0;
  }
}

// RTKLIB convention: signed sqrt of covariance — sgn(c)*sqrt(|c|).
static double signedSqrt(double c) {
  if (c >= 0.0) return std::sqrt(c);
  return -std::sqrt(-c);
}

static Args parseArgs(int argc, char** argv) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    std::string s = argv[i];
    auto next = [&]() {
      if (i + 1 >= argc) throw std::runtime_error("missing value for " + s);
      return std::string(argv[++i]);
    };
    if      (s == "--bag")   a.bag_uri = next();
    else if (s == "--topic") a.topic   = next();
    else if (s == "--out")   a.out_path = next();
    else std::fprintf(stderr, "[warn] unknown arg %s\n", s.c_str());
  }
  return a;
}

static void writeHeader(FILE* fp, const std::string& prog) {
  std::fprintf(fp, "%% program   : %s\n", prog.c_str());
  std::fprintf(fp, "%% pos mode  : (from receiver / GnssSolution.status)\n");
  std::fprintf(fp, "%%\n");
  std::fprintf(fp, "%% (lat/lon/height=WGS84/ellipsoidal,"
                   "Q=1:fix,2:float,3:sbas,4:dgps,5:single,6:ppp,"
                   "ns=# of satellites)\n");
  std::fprintf(fp, "%%  %-23s  %14s %14s %10s %3s %3s "
                   "%8s %8s %8s %8s %8s %8s %6s %6s\n",
               "GPST",
               "latitude(deg)", "longitude(deg)", "height(m)",
               "Q", "ns",
               "sdn(m)", "sde(m)", "sdu(m)",
               "sdne(m)", "sdeu(m)", "sdun(m)",
               "age(s)", "ratio");
}

static void writeEpoch(FILE* fp, const GnssSolution& m) {
  const int Q = statusToQ(m.status);
  if (Q == 0) return;

  gtime_t t = gpst2time((int)m.time_week, m.time_tow);
  char tbuf[64];
  time2str(t, tbuf, 3);

  // ENU covariance: row-major [nn ne nu; en ee eu; un ue uu]
  const double* C = m.pos_enu_cov.data();
  // Note: GnssSolution lays out indices as East-North-Up;
  // pos_enu = (East, North, Up) — see msg comment.
  // Cov layout is row-major over (E,N,U). Convert to RTKLIB's (N,E,U) order:
  //   sdn  = sqrt(C_NN) = sqrt(C[4])
  //   sde  = sqrt(C_EE) = sqrt(C[0])
  //   sdu  = sqrt(C_UU) = sqrt(C[8])
  //   sdne = sgn(C_NE)*sqrt(|C_NE|)  C_NE = C[1] (E-row, N-col) == C[3] (N-row, E-col)
  //   sdeu = sgn(C_EU)*sqrt(|C_EU|)  C_EU = C[2]
  //   sdun = sgn(C_UN)*sqrt(|C_UN|)  C_UN = C[7]
  const double sdn  = std::sqrt(std::max(0.0, C[4]));
  const double sde  = std::sqrt(std::max(0.0, C[0]));
  const double sdu  = std::sqrt(std::max(0.0, C[8]));
  const double sdne = signedSqrt(C[1]);
  const double sdeu = signedSqrt(C[2]);
  const double sdun = signedSqrt(C[7]);

  std::fprintf(fp,
    "%s  %14.9f %14.9f %10.4f %3d %3u "
    "%8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %6.2f %6.1f\n",
    tbuf,
    m.latitude, m.longitude, m.altitude,
    Q, m.num_sats,
    sdn, sde, sdu, sdne, sdeu, sdun,
    (double)m.age_diff, (double)m.ratio);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  Args args = parseArgs(argc, argv);

  if (args.bag_uri.empty() || args.out_path.empty()) {
    std::fprintf(stderr,
      "Usage: rosbag_to_pos --bag <bag_dir_or_db3> --out <out.pos> "
      "[--topic /gnss/solution]\n");
    return 2;
  }

  rosbag2_storage::StorageOptions sopt;
  sopt.uri = normalizeBagUri(args.bag_uri);
  sopt.storage_id = "sqlite3";
  rosbag2_cpp::ConverterOptions copt;

  rosbag2_cpp::Reader reader;
  try {
    reader.open(sopt, copt);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "Failed to open bag: %s\n", e.what());
    return 1;
  }

  FILE* fp = std::fopen(args.out_path.c_str(), "w");
  if (!fp) {
    std::perror(args.out_path.c_str());
    return 1;
  }
  writeHeader(fp, args.program_name);

  rclcpp::Serialization<GnssSolution> ser;
  size_t n_written = 0;

  while (reader.has_next()) {
    auto msg = reader.read_next();
    if (!msg) break;
    if (msg->topic_name != args.topic) continue;
    if (!msg->serialized_data || msg->serialized_data->buffer_length == 0) continue;

    GnssSolution sol;
    try {
      rclcpp::SerializedMessage smsg(*msg->serialized_data);
      ser.deserialize_message(&smsg, &sol);
    } catch (...) {
      continue;
    }
    writeEpoch(fp, sol);
    ++n_written;
  }

  std::fclose(fp);
  std::fprintf(stderr, "Wrote %zu epochs to %s\n", n_written, args.out_path.c_str());
  rclcpp::shutdown();
  return 0;
}
