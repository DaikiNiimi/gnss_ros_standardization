// Shared helpers for rosbag2 ↔ RTKLIB converters.
//   Read side  (bag → RTKLIB):   deserializeRos, normalizeBagUri, deriveOutputPath
//                                — used by rosbag_to_rinex.cpp, rosbag_to_pos.cpp
//   Write side (RTKLIB → bag):   resolveStorageId, toRosTimeGpst,
//                                set_qos_profile, kDefaultQosYaml
//                                — used by rinex_to_rosbag.cpp, pos_to_rosbag.cpp

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

#include "gnss_ros_standardization/rtklib_compat.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace gnss_converter_io {

// Normalize a bag URI: strip trailing slashes and, if a .db3 file path is
// passed, return its parent directory (rosbag2 expects the directory).
inline std::string normalizeBagUri(std::string uri) {
  while (!uri.empty() && (uri.back() == '/' || uri.back() == '\\')) uri.pop_back();
  if (uri.size() >= 4) {
    std::string tail = uri.substr(uri.size() - 4);
    for (auto& c : tail) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
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

// Climb up empty-filename parents safely (no infinite loop at filesystem root).
inline std::filesystem::path safeParentClimb(std::filesystem::path p) {
  while (!p.empty() && !p.has_filename()) {
    auto parent = p.parent_path();
    if (parent == p) break;
    p = parent;
  }
  return p;
}

// Derive an output path "<bag_dir>/<stem><ext>" from the bag URI.
// ext should include the leading dot (e.g. ".obs", ".pos").
inline std::string deriveOutputPath(const std::string& bag_uri, const std::string& ext) {
  std::filesystem::path bag_path(bag_uri);
  bag_path = safeParentClimb(bag_path);
  std::string stem = bag_path.stem().string();
  if (stem.empty()) stem = "output";
  std::string base_dir = bag_path.parent_path().string();
  if (!base_dir.empty()) base_dir += "/";
  return base_dir + stem + ext;
}

// Deserialize a SerializedBagMessage into a typed ROS message.
// Pass a long-lived rclcpp::Serialization<ROSMsg>& so allocations in the
// serializer can be amortized across the entire bag.
template <typename ROSMsg>
inline bool deserializeRos(const rosbag2_storage::SerializedBagMessage& in,
                           rclcpp::Serialization<ROSMsg>& ser, ROSMsg& out) {
  if (!in.serialized_data || in.serialized_data->buffer_length == 0)
    throw std::runtime_error("Rejected 1 message: empty serialized data on " + in.topic_name);
  try {
    rclcpp::SerializedMessage smsg(*in.serialized_data);
    ser.deserialize_message(&smsg, &out);
    return true;
  } catch (const std::exception& e) {
    throw std::runtime_error("Rejected 1 message: Cannot deserialize " + in.topic_name + ": " + e.what());
  }
}

// ---- Write-side helpers (RTKLIB → rosbag2) ----

// Resolve a rosbag2 storage_id given an explicit override and the output URI.
// Priority: explicit override > extension auto-detect (.mcap / .db3) > ""
// (which lets rosbag2 pick the distro default).
inline std::string resolveStorageId(const std::string& override_id,
                                    const std::string& output_uri) {
  if (!override_id.empty()) return override_id;
  auto pos = output_uri.find_last_of('.');
  if (pos == std::string::npos) return "";
  std::string ext = output_uri.substr(pos);
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  if (ext == ".mcap") return "mcap";
  if (ext == ".db3")  return "sqlite3";
  return "";
}

// Convert RTKLIB gtime_t (GPST seconds since the gtime_t epoch) into a
// rclcpp::Time tagged RCL_ROS_TIME. No leap-second conversion is applied —
// the numeric value is GPST. For UTC-converted ROS Time, use
// gnss_utils::gpstToUtcRosTime() instead.
inline rclcpp::Time toRosTimeGpst(gtime_t t) {
  if (t.time < 0 || t.time > std::numeric_limits<int32_t>::max() ||
      !std::isfinite(t.sec) || t.sec < 0 || t.sec >= 1)
    throw std::out_of_range("GNSS epoch outside ROS builtin Time range");
  const int64_t ns=static_cast<int64_t>(t.time)*1000000000LL + std::llround(t.sec*1e9);
  if (ns/1000000000LL > std::numeric_limits<int32_t>::max())
    throw std::out_of_range("GNSS epoch rounds outside ROS builtin Time range");
  return rclcpp::Time(ns, RCL_ROS_TIME);
}

enum class TimeScale { Utc, LegacyGpst };
inline TimeScale parseTimeScale(const std::string& value) {
  if (value == "utc") return TimeScale::Utc;
  if (value == "gpst") return TimeScale::LegacyGpst;
  throw std::invalid_argument("--timestamp-scale must be utc or gpst");
}
inline rclcpp::Time toRosTime(gtime_t t, TimeScale scale = TimeScale::Utc) {
  return toRosTimeGpst(scale == TimeScale::Utc ? gpst2utc(t) : t);
}
inline void writeTimeScaleMetadata(const std::string& bag, TimeScale scale) {
  std::ofstream out(std::filesystem::path(bag) / "gnss_time_basis.yaml");
  out << "header_and_bag_time: " << (scale == TimeScale::Utc ? "utc" : "gpst_legacy")
      << "\ngnss_week_tow: gpst\n";
  out.close();
  if (!out) throw std::runtime_error("Cannot write time basis metadata for " + bag);
}

// Set offered_qos_profiles on a TopicMetadata in a way that compiles on both
// Humble (string YAML) and Jazzy+ (vector<rclcpp::QoS>).
template <typename TopicMetadataT>
inline void set_qos_profile(TopicMetadataT& tm, const std::string& yaml_profile) {
  if constexpr (std::is_same_v<std::decay_t<decltype(tm.offered_qos_profiles)>,
                               std::string>) {
    tm.offered_qos_profiles = yaml_profile;
  } else if constexpr (std::is_same_v<std::decay_t<decltype(tm.offered_qos_profiles)>,
                                      std::vector<rclcpp::QoS>>) {
    const bool transient = (yaml_profile.find("durability: 1") != std::string::npos);
    // Honour the profile's depth. Hardcoding 1 here silently overrode the
    // depth-256 transient_local ephemeris profile, reintroducing exactly the
    // starvation that profile exists to prevent (a depth-1 latch keeps only the
    // last satellite of the per-satellite ephemeris burst).
    std::size_t depth = 1;
    const std::size_t pos = yaml_profile.find("depth:");
    if (pos != std::string::npos) {
      try {
        depth = std::max<std::size_t>(
            1, std::stoul(yaml_profile.substr(pos + 6)));
      } catch (const std::exception&) {
        depth = 1;  // malformed profile: keep the conservative default
      }
    }
    rclcpp::QoS qos{rclcpp::KeepLast(depth)};
    qos.reliable();
    if (transient) {
      qos.transient_local();
    } else {
      qos.durability_volatile();
    }
    tm.offered_qos_profiles.clear();
    tm.offered_qos_profiles.push_back(qos);
  }
}

// Default QoS YAML used by all converter writers: depth=1, RELIABLE, VOLATILE.
inline const std::string& defaultConverterQosYaml() {
  static const std::string kProfile =
      "- history: 1\n"
      "  depth: 1\n"
      "  reliability: 1\n"
      "  durability: 2\n"
      "  deadline:\n    sec: 0\n    nsec: 0\n"
      "  lifespan:\n    sec: 0\n    nsec: 0\n"
      "  liveliness: 0\n"
      "  liveliness_lease_duration:\n    sec: 0\n    nsec: 0\n"
      "  avoid_ros_namespace_conventions: false";
  return kProfile;
}

// QoS YAML for topics consumed by transient_local subscribers (e.g. /gnss/ephemeris).
// All positioning nodes subscribe /gnss/ephemeris with transient_local(); a VOLATILE
// publisher is incompatible with TRANSIENT_LOCAL subscribers under DDS QoS rules.
// Depth is 256, not 1: ephemeris is published as many per-satellite messages on one
// topic, so a depth-1 latch keeps only the LAST satellite. A subscriber that matches
// after the initial burst (e.g. a node started against a replayed bag) would then see
// only one satellite and starve for double-differences. 256 latches a full multi-GNSS
// constellation so any late/racing subscriber receives the complete ephemeris set.
inline const std::string& transientLocalConverterQosYaml() {
  static const std::string kProfile =
      "- history: 1\n"
      "  depth: 256\n"
      "  reliability: 1\n"
      "  durability: 1\n"
      "  deadline:\n    sec: 0\n    nsec: 0\n"
      "  lifespan:\n    sec: 0\n    nsec: 0\n"
      "  liveliness: 0\n"
      "  liveliness_lease_duration:\n    sec: 0\n    nsec: 0\n"
      "  avoid_ros_namespace_conventions: false";
  return kProfile;
}

}  // namespace gnss_converter_io

// ---- output transaction ----
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>
namespace gnss_converter_io {
// Ordinary conversion/write failures leave destinations untouched. The two
// final renames are not a crash-atomic multi-file filesystem transaction.
class OutputTransaction {
  struct File {
    std::filesystem::path target, directory, temporary, backup;
    FILE* stream = nullptr;
    bool saved = false, installed = false;
  };
  std::vector<File> files_;
  bool committed_ = false;
public:
  FILE* open(const std::string& path) {
    namespace fs = std::filesystem;
    const auto target = fs::absolute(path).lexically_normal();
    for (const auto& f : files_)
      if (target == f.target || (fs::exists(target) && fs::exists(f.target) && fs::equivalent(target,f.target)))
        throw std::runtime_error("Output destinations must differ");
    if (fs::exists(target) && !fs::is_regular_file(fs::symlink_status(target)))
      throw std::runtime_error("Output destination must be a regular file: " + path);
    std::string pattern = target.string() + ".tmp-XXXXXX";
    if (!mkdtemp(pattern.data())) throw std::runtime_error("Cannot create output temporary directory: " + path);
    files_.push_back({target,pattern,fs::path(pattern)/"output",fs::path(pattern)/"previous"});
    auto& file = files_.back();
    file.stream = std::fopen(file.temporary.c_str(), "w");
    if (!file.stream) throw std::runtime_error("Cannot open temporary output: " + path);
    // Replacing an existing file must not broaden its access permissions.
    if (fs::exists(target)) fs::permissions(file.temporary,fs::status(target).permissions());
    setvbuf(file.stream, nullptr, _IOFBF, 1<<20);
    return file.stream;
  }
  void commit() {
    for (auto& file : files_) {
      bool failed = std::ferror(file.stream) || std::fflush(file.stream) != 0;
      if (std::fclose(file.stream) != 0) failed = true;
      file.stream = nullptr;
      if (failed) throw std::runtime_error("Output write/close failed");
    }
    for (auto& file : files_) {
      if (std::filesystem::exists(file.target)) {
        std::filesystem::rename(file.target,file.backup); file.saved = true;
      }
      std::filesystem::rename(file.temporary,file.target); file.installed = true;
    }
    committed_ = true;
  }
  ~OutputTransaction() {
    for (auto& file : files_) {
      if (file.stream) std::fclose(file.stream);
      std::error_code error;
      if (!committed_) {
        if (file.installed) std::filesystem::remove(file.target,error);
        if (file.saved) {
          std::filesystem::rename(file.backup,file.target,error);
          if (error) { // Preserve the backup if an external filesystem change prevents rollback.
            std::fprintf(stderr,"Cannot restore output; previous file saved at %s\n",file.backup.c_str());
            continue;
          }
        }
      }
      std::filesystem::remove_all(file.directory,error);
    }
  }
};

// Bag writers require a nonexistent destination. Own a private staging parent
// so every failure removes only files this conversion created.
class BagTransaction {
  std::filesystem::path target_, directory_, staged_;
public:
  explicit BagTransaction(const std::string& path):target_(std::filesystem::absolute(path).lexically_normal()) {
    while (!target_.has_filename() && target_ != target_.parent_path()) target_=target_.parent_path();
    if (std::filesystem::exists(target_)) throw std::runtime_error("Output bag already exists: " + path);
    std::string pattern=target_.string()+".tmp-XXXXXX";
    if (!mkdtemp(pattern.data())) throw std::runtime_error("Cannot stage output bag: " + path);
    directory_=pattern; staged_=directory_/target_.filename();
  }
  std::string path() const {return staged_.string();}
  void commit() {
    if (std::filesystem::exists(target_)) throw std::runtime_error("Output bag appeared during conversion");
    std::filesystem::rename(staged_,target_);
  }
  ~BagTransaction() {std::error_code error;std::filesystem::remove_all(directory_,error);}
};
} // namespace gnss_converter_io
