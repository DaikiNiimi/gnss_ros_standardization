// Read-only access to a rosbag2 bag, plain or compressed.
#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

namespace rtk_bag_plot {

class BagSession {
 public:
  // Opens `uri`. File-compressed bags are read through a private staging
  // directory created under `staging_parent`, so that decompressed files
  // never appear in the original bag directory.
  BagSession(const std::string& uri, const std::filesystem::path& staging_parent);
  ~BagSession();
  BagSession(const BagSession&) = delete;
  BagSession& operator=(const BagSession&) = delete;

  std::vector<rosbag2_storage::TopicMetadata> topics() const;
  bool hasNext();
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> readNext();

 private:
  std::filesystem::path staging_;
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
};

// Bag timestamp (ns); the field name differs between ROS 2 distributions.
int64_t bagTimestamp(const rosbag2_storage::SerializedBagMessage& message);

}  // namespace rtk_bag_plot
