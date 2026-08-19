// SPDX-License-Identifier: MIT
// Convert a ROS 2 bag containing GnssSolution messages to an RTKLIB .pos file.
//
// Output format: standard RTKLIB lat/lon/height .pos with GPST date column.
// Q codes: 1=fix, 2=float, 3=sbas, 4=dgps, 5=single, 6=ppp.
//
// Design: single-pass streaming. Each GnssSolution message becomes one .pos
// line. Rows where status maps to Q=0 (unknown), or where the time stamp is
// zero, are silently dropped (typically uninitialized messages). When --vel
// is requested but any velocity component is non-finite, the velocity
// columns for that row are omitted so consumer tools (rtkplot) don't choke.
// Position covariance is remapped from the message's row-major E-N-U order
// into RTKLIB's expected N-E-U order. See the writeEpoch comments below.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"
#include "gnss_ros_standardization/bag_io_utils.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

using gnss_ros_standardization::msg::GnssSolution;
using gnss_converter_io::deserializeRos;
using gnss_converter_io::deriveOutputPath;
using gnss_converter_io::normalizeBagUri;

// Everything below is private to this translation unit. Each converter is a
// separate executable, but they all declare a file-scope `struct Args`, and
// identical names with different layouts at external linkage are an ODR
// violation the moment anything links two of them (cppcheck flags it as one).
namespace {
struct Args {
  std::string bag_uri;
  std::string topic = "/gnss/solution";
  std::string out_path;
  std::string program_name = "rosbag_to_pos";
  bool with_velocity = false;
};

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
    else if (s == "--vel")   a.with_velocity = true;
    else if (s == "--pgm")   a.program_name = next();
    else std::fprintf(stderr, "[warn] unknown arg %s\n", s.c_str());
  }
  return a;
}

static void writeHeader(FILE* fp, const std::string& prog, bool with_velocity) {
  std::fprintf(fp, "%% program   : %s\n", prog.c_str());
  std::fprintf(fp, "%% pos mode  : (from receiver / GnssSolution.status)\n");
  std::fprintf(fp, "%%\n");
  std::fprintf(fp, "%% (lat/lon/height=WGS84/ellipsoidal,"
                   "Q=1:fix,2:float,3:sbas,4:dgps,5:single,6:ppp,"
                   "ns=# of satellites)\n");
  std::fprintf(fp, "%%  %-23s  %14s %14s %10s %3s %3s "
                   "%8s %8s %8s %8s %8s %8s %6s %6s",
               "GPST",
               "latitude(deg)", "longitude(deg)", "height(m)",
               "Q", "ns",
               "sdn(m)", "sde(m)", "sdu(m)",
               "sdne(m)", "sdeu(m)", "sdun(m)",
               "age(s)", "ratio");
  if (with_velocity) {
    std::fprintf(fp, " %10s %10s %10s %8s %8s %8s %8s %8s %8s",
                 "vn(m/s)", "ve(m/s)", "vu(m/s)",
                 "sdvn", "sdve", "sdvu",
                 "sdvne", "sdveu", "sdvun");
  }
  std::fprintf(fp, "\n");
}

static bool writeEpoch(FILE* fp, const GnssSolution& m, bool with_velocity) {
  const int Q = statusToQ(m.status);
  if (Q == 0) return false;
  if (m.time_week == 0 && m.time_tow == 0.0) return false;

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
    "%8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %6.2f %6.1f",
    tbuf,
    m.latitude, m.longitude, m.altitude,
    Q, m.num_sats,
    sdn, sde, sdu, sdne, sdeu, sdun,
    (double)m.age_diff, (double)m.ratio);

  if (with_velocity) {
    // Some decoders emit quiet_NaN for vel_enu when origin/velocity is
    // unavailable (see novatel/sbf decoder NaN paths). Skip the velocity
    // columns when any component is non-finite so consumer tools (RTKPLOT,
    // rtkplot) don't choke on "nan" tokens; the row's Q/position/cov stay.
    const double vn = m.vel_enu.y;
    const double ve = m.vel_enu.x;
    const double vu = m.vel_enu.z;
    if (std::isfinite(vn) && std::isfinite(ve) && std::isfinite(vu)) {
      // Same E-N-U -> N-E-U remap as position covariance.
      const double* Cv = m.vel_enu_cov.data();
      const double sdvn  = std::sqrt(std::max(0.0, Cv[4]));
      const double sdve  = std::sqrt(std::max(0.0, Cv[0]));
      const double sdvu  = std::sqrt(std::max(0.0, Cv[8]));
      const double sdvne = signedSqrt(Cv[1]);
      const double sdveu = signedSqrt(Cv[2]);
      const double sdvun = signedSqrt(Cv[7]);
      std::fprintf(fp,
        " %10.5f %10.5f %10.5f %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f",
        vn, ve, vu, sdvn, sdve, sdvu, sdvne, sdveu, sdvun);
    }
  }
  std::fprintf(fp, "\n");
  return true;
}

static void printUsage(FILE* out) {
  std::fprintf(out,
    "Usage: rosbag_to_pos --bag <bag_dir_or_db3> "
    "[--out <out.pos>] [--topic /gnss/solution] [--vel] [--pgm <name>] "
    "[--help] [--version]\n");
}

}  // namespace

int main(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--help" || a == "-h") { printUsage(stdout); return 0; }
    if (a == "--version" || a == "-V") {
#ifdef PACKAGE_VERSION
      std::fprintf(stdout, "rosbag_to_pos %s\n", PACKAGE_VERSION);
#else
      std::fprintf(stdout, "rosbag_to_pos (unknown version)\n");
#endif
      return 0;
    }
  }
  rclcpp::init(argc, argv);
  Args args = parseArgs(argc, argv);

  if (args.bag_uri.empty()) {
    printUsage(stderr);
    return 2;
  }

  if (args.out_path.empty()) {
    args.out_path = deriveOutputPath(args.bag_uri, ".pos");
    std::fprintf(stderr, "Info: output path auto-derived: %s\n", args.out_path.c_str());
  }

  rosbag2_storage::StorageOptions sopt;
  sopt.uri = normalizeBagUri(args.bag_uri);
  sopt.storage_id = "";  // auto-detect from metadata.yaml (sqlite3 / mcap)
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
  writeHeader(fp, args.program_name, args.with_velocity);

  rclcpp::Serialization<GnssSolution> ser;
  size_t n_written = 0;

  while (reader.has_next()) {
    auto msg = reader.read_next();
    if (!msg) break;
    if (msg->topic_name != args.topic) continue;

    GnssSolution sol;
    if (!deserializeRos(*msg, ser, sol)) continue;
    if (writeEpoch(fp, sol, args.with_velocity)) ++n_written;
  }

  std::fclose(fp);
  std::fprintf(stderr, "Wrote %zu epochs to %s\n", n_written, args.out_path.c_str());
  rclcpp::shutdown();
  return 0;
}