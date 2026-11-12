// rtk_bag_plot: writes an RTKPLOT-style report of a rosbag2 bag as one HTML file.
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "gnss_ros_standardization/bag_io_utils.hpp"

#include "bag_session.hpp"
#include "encode.hpp"
#include "extract.hpp"
#include "rtk_bag_plot_viewer_embed.hpp"

extern char** environ;

namespace fs = std::filesystem;

namespace {

constexpr size_t kBudgetBytes = 48U * 1024U * 1024U;  // about 64 MB of HTML

struct UsageError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

struct Args {
  std::string bag, out, stat;
  std::vector<std::string> pos, compare;
  bool open_browser{true};
};

void printUsage(FILE* out) {
  std::fprintf(out,
               "Usage: rtk_bag_plot <bag> [--out <file.html>] [--pos <file.pos>]... "
               "[--compare <bag>]... [--stat <file.stat>] [--no-open] [--help] [--version]\n");
}

Args parseArgs(int argc, char** argv) {
  Args args;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto value = [&]() {
      if (i + 1 >= argc) throw UsageError("Missing value for " + arg);
      return std::string(argv[++i]);
    };
    if (arg == "--bag") {
      args.bag = value();
    } else if (arg == "--out") {
      args.out = value();
    } else if (arg == "--stat") {
      args.stat = value();
    } else if (arg == "--pos") {
      args.pos.push_back(value());
    } else if (arg == "--compare") {
      args.compare.push_back(value());
    } else if (arg == "--no-open") {
      args.open_browser = false;
    } else if (!arg.empty() && arg[0] == '-') {
      throw UsageError("Unknown argument: " + arg);
    } else if (args.bag.empty()) {
      args.bag = arg;
    } else {
      throw UsageError("Unexpected argument: " + arg);
    }
  }
  if (args.bag.empty()) throw UsageError("A bag path is required");
  return args;
}

// Starts the desktop's browser without waiting for it. Failure is not an error
// because the report has already been written.
bool openInBrowser(const std::string& path) {
  if (!std::getenv("DISPLAY") && !std::getenv("WAYLAND_DISPLAY")) return false;
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_addopen(&actions, STDOUT_FILENO, "/dev/null", 1, 0);
  posix_spawn_file_actions_addopen(&actions, STDERR_FILENO, "/dev/null", 1, 0);
  std::string program = "xdg-open";
  std::string target = path;
  char* const child_argv[] = {program.data(), target.data(), nullptr};
  pid_t pid = 0;
  const int result = posix_spawnp(&pid, "xdg-open", &actions, nullptr, child_argv, environ);
  posix_spawn_file_actions_destroy(&actions);
  return result == 0;
}

int run(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      printUsage(stdout);
      return 0;
    }
    if (arg == "--version" || arg == "-V") {
#ifdef PACKAGE_VERSION
      std::fprintf(stdout, "rtk_bag_plot %s\n", PACKAGE_VERSION);
#else
      std::fprintf(stdout, "rtk_bag_plot (unknown version)\n");
#endif
      return 0;
    }
  }

  rclcpp::InitOptions init_options;
  init_options.auto_initialize_logging(false);
  rclcpp::init(argc, argv, init_options);
  const auto cleaned = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char*> cleaned_argv;
  for (const auto& argument : cleaned) cleaned_argv.push_back(const_cast<char*>(argument.c_str()));
  Args args = parseArgs(static_cast<int>(cleaned_argv.size()), cleaned_argv.data());

  const std::string uri = gnss_converter_io::normalizeBagUri(args.bag);
  if (args.out.empty()) args.out = gnss_converter_io::deriveOutputPath(uri, ".html");
  const fs::path out_path = fs::absolute(args.out);
  if (!fs::is_directory(out_path.parent_path()))
    throw std::runtime_error("Output directory does not exist: " + out_path.parent_path().string());

  rtk_bag_plot::PlotModel model;
  {
    rtk_bag_plot::BagSession session(uri, out_path.parent_path());
    std::fprintf(stderr, "Reading %s\n", uri.c_str());
    auto contents = rtk_bag_plot::readBag(session);
    for (const auto& other : args.compare) {
      const std::string other_uri = gnss_converter_io::normalizeBagUri(other);
      rtk_bag_plot::BagSession other_session(other_uri, out_path.parent_path());
      std::fprintf(stderr, "Reading %s\n", other_uri.c_str());
      rtk_bag_plot::appendComparisonBag(contents, other_session, fs::path(other_uri).filename().string());
    }
    for (const auto& pos : args.pos) rtk_bag_plot::appendPosFile(contents, pos);
    model = rtk_bag_plot::buildPlotModel(contents, fs::path(uri).filename().string(), args.stat);
  }
  rtk_bag_plot::applySizeBudget(model, kBudgetBytes);

  if (model.topics.empty() || [&]() {
        for (const auto& topic : model.topics)
          if (topic.count > 0) return false;
        return true;
      }()) {
    throw std::runtime_error("The bag contains no messages: " + uri);
  }
  for (const auto& note : model.notes) std::fprintf(stderr, "Note: %s\n", note.c_str());

  gnss_converter_io::OutputTransaction output;
  FILE* fp = output.open(out_path.string());
  rtk_bag_plot::writeHtml(model, rtk_bag_plot::kViewerHtml, rtk_bag_plot::kViewerHtmlSize, fp);
  output.commit();
  std::fprintf(stdout, "%s\n", out_path.c_str());

  if (args.open_browser && !openInBrowser(out_path.string()))
    std::fprintf(stderr, "Open the file above in a web browser.\n");
  rclcpp::shutdown();
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    return run(argc, argv);
  } catch (const UsageError& error) {
    std::fprintf(stderr, "%s\n", error.what());
    printUsage(stderr);
    if (rclcpp::ok()) rclcpp::shutdown();
    return 2;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "rtk_bag_plot failed: %s\n", error.what());
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}
