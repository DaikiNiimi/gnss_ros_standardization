// SPDX-License-Identifier: MIT
// Convert an RTKLIB .pos file to a ROS 2 bag of GnssSolution messages.
//
// Supports both lat/lon/height (LLH) and x/y/z-ecef (ECEF) variants, with
// either GPST date-time columns ("yyyy/mm/dd hh:mm:ss.sss") or GPST
// week + seconds-of-week columns. Format is auto-detected from header
// comments and the first data row.

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include "gnss_ros_standardization/bag_io_utils.hpp"
#include "gnss_ros_standardization/pos_reader.hpp"

#include <cstdio>
#include <string>

struct Args {
  std::string pos_path;
  std::string output_bag;
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

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  Args args = parseArgs(argc, argv);

  if (args.output_bag.empty() && !args.pos_path.empty())
    args.output_bag = gnss_converter_io::deriveOutputPath(args.pos_path, "");

  if (args.pos_path.empty()) {
    std::fprintf(stderr,
      "Usage: pos_to_rosbag --pos <input.pos> [--out <bag>] "
      "[--topic /gnss/solution] [--storage mcap|sqlite3]\n");
    rclcpp::shutdown();
    return 2;
  }

  auto result = gnss_pos_reader::parsePosFile(args.pos_path, std::cerr);
  if (result.rows.empty()) {
    std::fprintf(stderr, "[pos_to_rosbag] error: no valid rows in %s\n",
                 args.pos_path.c_str());
    rclcpp::shutdown();
    return 1;
  }

  // Scoped writer: bag is closed by the destructor before the cleanup check
  // below, ensuring file handles are released before remove_all is called.
  {
    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions storage_opts;
    storage_opts.uri = args.output_bag;
    storage_opts.storage_id =
        gnss_converter_io::resolveStorageId(args.storage_id, args.output_bag);
    rosbag2_cpp::ConverterOptions converter_opts;
    converter_opts.input_serialization_format  = "cdr";
    converter_opts.output_serialization_format = "cdr";

    try {
      writer.open(storage_opts, converter_opts);
    } catch (const std::exception& e) {
      std::fprintf(stderr, "Failed to open bag: %s\n", e.what());
      rclcpp::shutdown();
      return 1;
    }

    rosbag2_storage::TopicMetadata tm;
    tm.serialization_format = "cdr";
    gnss_converter_io::set_qos_profile(tm, gnss_converter_io::defaultConverterQosYaml());
    tm.name = args.topic;
    tm.type = "gnss_ros_standardization/msg/GnssSolution";
    writer.create_topic(tm);

    for (const auto& row : result.rows) {
      rclcpp::Time stamp = gnss_converter_io::toRosTimeGpst(row.t);
      writer.write(row.sol, args.topic, stamp);
    }
  }  // writer closed here

  std::fprintf(stderr, "[pos_to_rosbag] wrote %zu epochs (%s%s) to %s; skipped %zu rows\n",
               result.rows.size(),
               result.is_ecef ? "ECEF" : "LLH",
               result.has_velocity ? "+vel" : "",
               args.output_bag.c_str(),
               result.n_skipped);

  rclcpp::shutdown();
  return 0;
}
