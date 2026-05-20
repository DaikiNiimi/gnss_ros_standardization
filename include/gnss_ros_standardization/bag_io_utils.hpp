// SPDX-License-Identifier: MIT
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

extern "C" {
#include "rtklib.h"
}

#include <algorithm>
#include <cctype>
#include <filesystem>
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
  if (!in.serialized_data || in.serialized_data->buffer_length == 0) return false;
  try {
    rclcpp::SerializedMessage smsg(*in.serialized_data);
    ser.deserialize_message(&smsg, &out);
    return true;
  } catch (...) {
    return false;
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
  long sec     = static_cast<long>(t.time);
  long nanosec = static_cast<long>(t.sec * 1e9);
  if (nanosec >= 1000000000) {
    sec += nanosec / 1000000000;
    nanosec %= 1000000000;
  } else if (nanosec < 0) {
    long roll = (-nanosec + 999999999) / 1000000000;
    sec -= roll;
    nanosec += roll * 1000000000;
  }
  if (sec < 0) sec = 0;
  return rclcpp::Time(sec, nanosec, RCL_ROS_TIME);
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
    rclcpp::QoS qos = transient
        ? rclcpp::QoS(1).reliable().transient_local()
        : rclcpp::QoS(1).reliable().durability_volatile();
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
inline const std::string& transientLocalConverterQosYaml() {
  static const std::string kProfile =
      "- history: 1\n"
      "  depth: 1\n"
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