#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include "gnss_utils.hpp"
// gnss_utils.hpp includes rtklib.h internally

#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <cmath>
#include <limits>

using gnss_ros_standardization::msg::GnssObservation;
using gnss_ros_standardization::msg::GnssObservations;
using gnss_ros_standardization::msg::GnssEphemeris;
using gnss_ros_standardization::msg::GlonassEphemeris;
using gnss_ros_standardization::msg::GnssEphemerides;

// Constants for selection gates (seconds)
const double MAX_DTOE_GNSS = 7200.0;
const double MAX_DTOE_GLONASS = 1800.0;

struct Args {
  std::string obs_path;
  std::vector<std::string> nav_paths;
  std::string output_bag = "rinex_converted";
};

Args parseArgs(const std::vector<std::string>& args) {
  Args a;
  for (size_t i = 0; i < args.size(); ++i) {
    if (args[i] == "--obs" && i + 1 < args.size()) {
      a.obs_path = args[++i];
    } else if (args[i] == "--nav" && i + 1 < args.size()) {
      a.nav_paths.push_back(args[++i]);
    } else if (args[i] == "--out" && i + 1 < args.size()) {
      a.output_bag = args[++i];
    }
  }
  return a;
}

rclcpp::Time toRosTime(gtime_t t) {
  long sec = t.time;
  long nanosec = static_cast<long>(t.sec * 1e9);
  
  // Normalize nanosec
  if (nanosec >= 1000000000) {
      sec += nanosec / 1000000000;
      nanosec %= 1000000000;
  } else if (nanosec < 0) {
      long roll = (-nanosec + 999999999) / 1000000000;
      sec -= roll;
      nanosec += roll * 1000000000;
  }
  
  if (sec < 0) sec = 0;
  return rclcpp::Time(sec, nanosec);
}

// Template helper to select best ephemeris index
// Works for both eph_t and geph_t as long as they have 'toe'
template <typename T>
int select_best_eph(const std::vector<int>& indices, const T* data_array, gtime_t t_curr, double max_dtoe) {
    int best_idx = -1;
    double min_diff = std::numeric_limits<double>::max();
    
    for (int idx : indices) {
        double diff = fabs(timediff(t_curr, data_array[idx].toe));
        if (diff <= max_dtoe) {
            if (diff < min_diff) {
                min_diff = diff;
                best_idx = idx;
            } else if (diff == min_diff) {
                // Tie-breaker: pick later index (later in file)
                if (idx > best_idx) best_idx = idx;
            }
        }
    }
    return best_idx;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  std::vector<std::string> args_vec;
  for (int i=1; i<argc; ++i) args_vec.push_back(argv[i]);
  Args args = parseArgs(args_vec);

  if (args.obs_path.empty()) {
    std::cerr << "Usage: rinex_to_rosbag --obs <obs_file> [--nav <nav_file> ...] [--out <bag_path>]\n";
    return 1;
  }

  // 1. Read OBS Data
  obs_t obs = {0};
  nav_t nav_tmp = {0};
  sta_t sta = {0};
  
  if (readrnx(args.obs_path.c_str(), 1, "", &obs, &nav_tmp, &sta) == 0) {
      std::cerr << "Warning: No data read from OBS file.\n";
  }
  freenav(&nav_tmp, 0xFF); 

  if (obs.n <= 0) {
      std::cerr << "Error: No observations loaded.\n";
      freeobs(&obs);
      return 1;
  }

  // 2. Read NAV Data
  nav_t nav = {0};
  for (const auto& path : args.nav_paths) {
      obs_t dummy_obs = {0};
      sta_t dummy_sta = {0};
      readrnx(path.c_str(), 1, "", &dummy_obs, &nav, &dummy_sta);
      freeobs(&dummy_obs);
  }

  // 3. Index NAV Data for Fast Lookup
  // Map: SatID -> Vector of indices in nav.eph/geph
  std::map<int, std::vector<int>> sat_eph_indices;
  std::map<int, std::vector<int>> sat_geph_indices;

  for (int i = 0; i < nav.n; ++i) {
      if (nav.eph[i].toe.time != 0) { // Only consider valid TOE
          sat_eph_indices[nav.eph[i].sat].push_back(i);
      }
  }
  for (int i = 0; i < nav.ng; ++i) {
      if (nav.geph[i].toe.time != 0) { // Only consider valid TOE
          sat_geph_indices[nav.geph[i].sat].push_back(i);
      }
  }

  // 4. Setup Bag Writer
  rosbag2_cpp::Writer writer;
  rosbag2_storage::StorageOptions storage_opts;
  storage_opts.uri = args.output_bag;
  storage_opts.storage_id = "sqlite3";
  
  rosbag2_cpp::ConverterOptions converter_opts;
  converter_opts.input_serialization_format = "cdr";
  converter_opts.output_serialization_format = "cdr";

  try {
      writer.open(storage_opts, converter_opts);
  } catch (const std::exception& e) {
      std::cerr << "Failed to open bag: " << e.what() << "\n";
      return 1;
  }

  // QoS profile (RELIABLE, VOLATILE, Default Limits)
  std::string qos_profile = 
      "- history: 1\n"
      "  depth: 1\n"
      "  reliability: 1\n"
      "  durability: 2\n" 
      "  deadline:\n"
      "    sec: 0\n"
      "    nsec: 0\n"
      "  lifespan:\n"
      "    sec: 0\n"
      "    nsec: 0\n"
      "  liveliness: 0\n"
      "  liveliness_lease_duration:\n"
      "    sec: 0\n"
      "    nsec: 0\n"
      "  avoid_ros_namespace_conventions: false";

  rosbag2_storage::TopicMetadata tm;
  tm.serialization_format = "cdr";
  tm.offered_qos_profiles = qos_profile;

  tm.name = "/gnss/observation";
  tm.type = "gnss_ros_standardization/msg/GnssObservations";
  writer.create_topic(tm);

  tm.name = "/gnss/ephemeris";
  tm.type = "gnss_ros_standardization/msg/GnssEphemerides";
  writer.create_topic(tm);

  // 5. Processing Loop (Epoch-by-Epoch)
  int i = 0;
  while (i < obs.n) {
      gtime_t t_curr = obs.data[i].time;
      rclcpp::Time stamp = toRosTime(t_curr);
      
      GnssObservations msg_obs;
      int week=0;
      msg_obs.tow = time2gpst(t_curr, &week);
      msg_obs.week = week;
      msg_obs.header.stamp = stamp;
      msg_obs.header.frame_id = "gnss_receiver";

      GnssEphemerides msg_eph;
      msg_eph.header.stamp = stamp;
      msg_eph.header.frame_id = "gnss_receiver";

      std::vector<int> epoch_sats;

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
          if (has_signal) {
            epoch_sats.push_back(d.sat);
          }
          j++;
      }
      
      std::sort(epoch_sats.begin(), epoch_sats.end());
      epoch_sats.erase(std::unique(epoch_sats.begin(), epoch_sats.end()), epoch_sats.end());

      // Select Best Ephemeris for each observed satellite
      for (int sat : epoch_sats) {
          int sys = satsys(sat, NULL);
          
          if (sys == SYS_GLO) {
              if (sat_geph_indices.count(sat)) {
                  int idx = select_best_eph(sat_geph_indices[sat], nav.geph, t_curr, MAX_DTOE_GLONASS);
                  if (idx != -1) {
                      msg_eph.glonass_ephemeris.push_back(gnss_utils::gephToMsg(nav.geph[idx]));
                  }
              }
          } else {
              if (sat_eph_indices.count(sat)) {
                  int idx = select_best_eph(sat_eph_indices[sat], nav.eph, t_curr, MAX_DTOE_GNSS);
                  if (idx != -1) {
                      msg_eph.gnss_ephemeris.push_back(gnss_utils::ephToMsg(nav.eph[idx]));
                  }
              }
          }
      }

      writer.write(msg_obs, "/gnss/observation", stamp);
      
      if (!msg_eph.gnss_ephemeris.empty() || !msg_eph.glonass_ephemeris.empty()) {
          writer.write(msg_eph, "/gnss/ephemeris", stamp);
      }

      i = j;
  }

  freeobs(&obs);
  freenav(&nav, 0xFF);
  
  std::cout << "Conversion complete -> " << args.output_bag << "\n";

  rclcpp::shutdown();
  return 0;
}
