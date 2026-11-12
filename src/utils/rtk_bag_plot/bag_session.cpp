#include "bag_session.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <stdexcept>
#include <system_error>

#include <rosbag2_compression/sequential_compression_reader.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/metadata_io.hpp>
#include <rosbag2_storage/storage_options.hpp>

namespace fs = std::filesystem;

namespace rtk_bag_plot {
namespace {

template <typename M>
auto stampOf(const M& m, int) -> decltype(m.recv_timestamp) { return m.recv_timestamp; }
template <typename M>
auto stampOf(const M& m, long) -> decltype(m.time_stamp) { return m.time_stamp; }

bool isFileMode(std::string mode) {
  std::transform(mode.begin(), mode.end(), mode.begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return mode == "FILE";
}

fs::path stageCompressedBag(const fs::path& bag, const rosbag2_storage::BagMetadata& metadata,
                            const fs::path& parent) {
  std::string pattern = (parent / ".rtk_bag_plot-XXXXXX").string();
  if (!mkdtemp(pattern.data()))
    throw std::runtime_error("Cannot create a staging directory under " + parent.string());
  const fs::path staging(pattern);
  try {
    fs::copy_file(bag / rosbag2_storage::MetadataIo::metadata_filename,
                  staging / rosbag2_storage::MetadataIo::metadata_filename);
    for (const auto& file : metadata.relative_file_paths) {
      const fs::path source = bag / fs::path(file).filename();
      fs::create_symlink(fs::absolute(source), staging / fs::path(file).filename());
    }
  } catch (...) {
    std::error_code ignored;
    fs::remove_all(staging, ignored);
    throw;
  }
  return staging;
}

}  // namespace

BagSession::BagSession(const std::string& uri, const fs::path& staging_parent) {
  rosbag2_storage::StorageOptions storage;
  storage.uri = uri;
  storage.storage_id = "";

  rosbag2_storage::MetadataIo metadata_io;
  bool compressed = false;
  if (metadata_io.metadata_file_exists(uri)) {
    const auto metadata = metadata_io.read_metadata(uri);
    compressed = !metadata.compression_format.empty();
    if (compressed && isFileMode(metadata.compression_mode)) {
      staging_ = stageCompressedBag(fs::path(uri), metadata, staging_parent);
      storage.uri = staging_.string();
    }
  }

  std::unique_ptr<rosbag2_cpp::reader_interfaces::BaseReaderInterface> implementation;
  if (compressed)
    implementation = std::make_unique<rosbag2_compression::SequentialCompressionReader>();
  else
    implementation = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
  reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(implementation));
  rosbag2_cpp::ConverterOptions converter;
  try {
    reader_->open(storage, converter);
  } catch (...) {
    reader_.reset();
    std::error_code ignored;
    if (!staging_.empty()) fs::remove_all(staging_, ignored);
    throw;
  }
}

BagSession::~BagSession() {
  reader_.reset();
  if (!staging_.empty()) {
    std::error_code ignored;
    fs::remove_all(staging_, ignored);
  }
}

std::vector<rosbag2_storage::TopicMetadata> BagSession::topics() const {
  return reader_->get_all_topics_and_types();
}

bool BagSession::hasNext() { return reader_->has_next(); }

std::shared_ptr<rosbag2_storage::SerializedBagMessage> BagSession::readNext() {
  return reader_->read_next();
}

int64_t bagTimestamp(const rosbag2_storage::SerializedBagMessage& message) {
  return static_cast<int64_t>(stampOf(message, 0));
}

}  // namespace rtk_bag_plot
