// SPDX-License-Identifier: MIT
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/bag_io_utils.hpp"
#include "gnss_ros_standardization/pos_reader.hpp"
// gnss_utils.hpp includes rtklib.h internally

#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <limits>

using gnss_ros_standardization::msg::GnssObservation;
using gnss_ros_standardization::msg::GnssObservations;
using gnss_ros_standardization::msg::GnssEphemeris;
using gnss_ros_standardization::msg::GlonassEphemeris;
using gnss_ros_standardization::msg::GnssEphemerides;

// Default ephemeris selection gates (seconds). Configurable via CLI.
//   GNSS  : GPS LNAV TOE validity is ±2 h (7200 s); Galileo I/NAV similar.
//   GLONASS: broadcast ephemeris validity is ±15 min nominal; we use ±30 min
//            (1800 s) to keep the same window RTKLIB uses by default.
constexpr double kDefaultMaxDtoeGnss    = 7200.0;
constexpr double kDefaultMaxDtoeGlonass = 1800.0;

enum class EphMode { PerEpoch, OnChange };

struct Args {
  std::string obs_path;
  std::vector<std::string> nav_paths;
  std::string output_bag;
  std::string storage_id;  // empty => auto-detect from --out extension or distro default
  std::string pos_path;    // optional; when set, also writes /gnss/solution
  EphMode eph_mode = EphMode::OnChange;
  double max_dtoe_gnss    = kDefaultMaxDtoeGnss;
  double max_dtoe_glonass = kDefaultMaxDtoeGlonass;
};

// RAII wrappers for RTKLIB obs_t / nav_t so memory is released on any exit
// path (writer exception, early return, std::terminate from a downstream
// throw, etc.). RTKLIB frees the dynamic arrays inside the struct; the
// struct itself is local.
struct ObsGuard {
  obs_t obs{};
  ~ObsGuard() { freeobs(&obs); }
};
struct NavGuard {
  nav_t nav{};
  ~NavGuard() { freenav(&nav, 0xFF); }
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
    } else if (args[i] == "--storage" && i + 1 < args.size()) {
      a.storage_id = args[++i];
    } else if (args[i] == "--eph-mode" && i + 1 < args.size()) {
      std::string m = args[++i];
      if (m == "per-epoch")      a.eph_mode = EphMode::PerEpoch;
      else if (m == "on-change") a.eph_mode = EphMode::OnChange;
      else {
        std::cerr << "Error: unknown --eph-mode '" << m
                  << "' (expected per-epoch | on-change)\n";
        std::exit(2);
      }
    } else if (args[i] == "--max-dtoe-gnss" && i + 1 < args.size()) {
      a.max_dtoe_gnss = std::stod(args[++i]);
    } else if (args[i] == "--max-dtoe-glo" && i + 1 < args.size()) {
      a.max_dtoe_glonass = std::stod(args[++i]);
    } else if (args[i] == "--pos" && i + 1 < args.size()) {
      a.pos_path = args[++i];
    }
  }
  return a;
}

// Select the best ephemeris index for the current time. Works for both eph_t
// and geph_t (any RTKLIB type with a `toe` field).
// Selection rule: smallest |t_curr - toe| within max_dtoe. On exact ties,
// prefer the later array index — since RTKLIB appends records in file order,
// "later index" means "issued more recently in the source RINEX nav file",
// which is the safer choice when two ephemerides share the same TOE.
template <typename T>
int select_best_eph(const std::vector<int>& indices, const T* data_array,
                    gtime_t t_curr, double max_dtoe) {
    int best_idx = -1;
    double min_diff = std::numeric_limits<double>::max();
    for (int idx : indices) {
        double diff = std::fabs(timediff(t_curr, data_array[idx].toe));
        if (diff > max_dtoe) continue;
        if (diff < min_diff) { min_diff = diff; best_idx = idx; }
        else if (diff == min_diff && idx > best_idx) best_idx = idx;
    }
    return best_idx;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  std::vector<std::string> args_vec;
  for (int i=1; i<argc; ++i) args_vec.push_back(argv[i]);
  Args args = parseArgs(args_vec);

  if (args.output_bag.empty() && !args.obs_path.empty())
    args.output_bag = gnss_converter_io::deriveOutputPath(args.obs_path, "");

  if (args.obs_path.empty()) {
    std::cerr <<
      "Usage: rinex_to_rosbag --obs <obs_file> [--nav <nav_file> ...]\n"
      "                       [--out <bag_path>] [--storage mcap|sqlite3]\n"
      "                       [--eph-mode per-epoch|on-change]\n"
      "                       [--max-dtoe-gnss <sec>] [--max-dtoe-glo <sec>]\n"
      "                       [--pos <pos_file>]\n"
      "\n"
      "  --eph-mode    per-epoch: republish all visible ephemerides every obs epoch.\n"
      "                on-change (default): publish per-sat only when the selected\n"
      "                     nav index changes (avoids massive duplication; first\n"
      "                     visible epoch of each sat still publishes once).\n"
      "  --max-dtoe-*  max |t_obs - toe| for ephemeris selection [sec]. Defaults:\n"
      "                GNSS=" << kDefaultMaxDtoeGnss
      << ", GLONASS=" << kDefaultMaxDtoeGlonass << ".\n"
      "  --pos         optional RTKLIB .pos file; when provided, /gnss/solution is\n"
      "                also written into the bag (same formats as pos_to_rosbag).\n"
      "  When --nav is omitted, the /gnss/ephemeris topic is created but empty.\n";
    return 1;
  }

  // 1. Read OBS data. RAII guard releases obs memory on any exit path.
  ObsGuard og;
  NavGuard nav_tmp_g;  // throwaway nav from OBS read (a few mixed records)
  sta_t sta = {};
  int obs_rc = readrnx(args.obs_path.c_str(), 1, "", &og.obs, &nav_tmp_g.nav, &sta);
  if (obs_rc <= 0 || og.obs.n <= 0) {
      std::cerr << "Error: failed to read observations from '" << args.obs_path
                << "' (readrnx rc=" << obs_rc << ", n=" << og.obs.n
                << "). Check file path and RINEX OBS format.\n";
      return 1;
  }

  // 2. Read NAV data into a single accumulating nav_t.
  NavGuard ng;
  size_t nav_ok = 0;
  for (const auto& path : args.nav_paths) {
      ObsGuard dummy_og;
      sta_t dummy_sta = {};
      int nav_rc = readrnx(path.c_str(), 1, "", &dummy_og.obs, &ng.nav, &dummy_sta);
      if (nav_rc <= 0) {
          std::cerr << "Warning: failed to read nav file '" << path
                    << "' (readrnx rc=" << nav_rc << "). Skipping.\n";
      } else {
          ++nav_ok;
      }
  }
  if (!args.nav_paths.empty() && nav_ok == 0) {
      std::cerr << "Error: all --nav files failed to load; no ephemerides available.\n";
      return 1;
  }

  // 3. Index nav for fast per-satellite lookup.
  std::map<int, std::vector<int>> sat_eph_indices;
  std::map<int, std::vector<int>> sat_geph_indices;
  for (int i = 0; i < ng.nav.n; ++i) {
      const auto& e = ng.nav.eph[i];
      if (e.toe.time != 0 && e.sat > 0 && satsys(e.sat, NULL) != SYS_NONE) {
          sat_eph_indices[e.sat].push_back(i);
      }
  }
  for (int i = 0; i < ng.nav.ng; ++i) {
      const auto& g = ng.nav.geph[i];
      if (g.toe.time != 0 && g.sat > 0 && satsys(g.sat, NULL) == SYS_GLO) {
          sat_geph_indices[g.sat].push_back(i);
      }
  }

  // 4. Optionally preload .pos rows for /gnss/solution interleaving.
  gnss_pos_reader::ParseResult pos_result;
  if (!args.pos_path.empty()) {
    pos_result = gnss_pos_reader::parsePosFile(args.pos_path, std::cerr);
    if (pos_result.rows.empty()) {
      std::cerr << "Warning: --pos file produced no rows; /gnss/solution will be absent.\n";
    }
  }

  // Check that pos and obs time ranges overlap; warn on mismatch (wrong files combined).
  if (!pos_result.rows.empty() && og.obs.n > 0) {
    gtime_t obs_first = og.obs.data[0].time;
    gtime_t obs_last  = og.obs.data[og.obs.n - 1].time;
    gtime_t pos_first = pos_result.rows.front().t;
    gtime_t pos_last  = pos_result.rows.back().t;
    if (timediff(pos_last, obs_first) < -1.0 || timediff(pos_first, obs_last) > 1.0) {
      std::cerr << "Warning: --pos time range does not overlap with obs time range. "
                << "Verify that the .pos file was generated from the same obs session.\n";
    }
  }

  // 5. Setup bag writer. Scoped so the writer closes before the post-loop
  // cleanup check (remove_all requires the file handles to be released first).
  size_t n_obs_epochs = 0;
  size_t n_eph_msgs   = 0;
  {
    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions storage_opts;
    storage_opts.uri = args.output_bag;
    storage_opts.storage_id =
        gnss_converter_io::resolveStorageId(args.storage_id, args.output_bag);

    rosbag2_cpp::ConverterOptions converter_opts;
    converter_opts.input_serialization_format = "cdr";
    converter_opts.output_serialization_format = "cdr";

    try {
        writer.open(storage_opts, converter_opts);
    } catch (const std::exception& e) {
        std::cerr << "Failed to open bag: " << e.what() << "\n";
        rclcpp::shutdown();
        return 1;
    }

    rosbag2_storage::TopicMetadata tm;
    tm.serialization_format = "cdr";

    gnss_converter_io::set_qos_profile(tm, gnss_converter_io::defaultConverterQosYaml());
    tm.name = "/gnss/observation";
    tm.type = "gnss_ros_standardization/msg/GnssObservations";
    writer.create_topic(tm);

    // /gnss/ephemeris is consumed by positioning nodes (SPP, RTK, gnss_visualizer)
    // with transient_local() subscriptions — use matching TRANSIENT_LOCAL durability
    // so late subscribers receive the last published ephemeris immediately.
    gnss_converter_io::set_qos_profile(tm, gnss_converter_io::transientLocalConverterQosYaml());
    tm.name = "/gnss/ephemeris";
    tm.type = "gnss_ros_standardization/msg/GnssEphemerides";
    writer.create_topic(tm);

    if (!pos_result.rows.empty()) {
      gnss_converter_io::set_qos_profile(tm, gnss_converter_io::defaultConverterQosYaml());
      tm.name = "/gnss/solution";
      tm.type = "gnss_ros_standardization/msg/GnssSolution";
      writer.create_topic(tm);
    }

    // Per-satellite "last published nav index" for --eph-mode on-change.
    // -1 means "never published yet for this sat".
    std::map<int, int> last_eph_idx;
    std::map<int, int> last_geph_idx;

    // Cursor into the pre-loaded pos rows for interleaved /gnss/solution writes.
    size_t pos_idx = 0;

    // 6. Processing loop, epoch by epoch.
    constexpr size_t kProgressEvery = 1000;
    int i = 0;
    while (i < og.obs.n) {
        gtime_t t_curr = og.obs.data[i].time;
        rclcpp::Time stamp = gnss_converter_io::toRosTimeGpst(t_curr);

        GnssObservations msg_obs;
        int week = 0;
        msg_obs.tow = time2gpst(t_curr, &week);
        msg_obs.week = week;
        msg_obs.header.stamp = stamp;
        msg_obs.header.frame_id = "gnss_receiver";

        GnssEphemerides msg_eph;
        msg_eph.header.stamp = stamp;
        msg_eph.header.frame_id = "gnss_receiver";

        std::vector<int> epoch_sats;

        int j = i;
        while (j < og.obs.n) {
            if (timediff(og.obs.data[j].time, t_curr) > 1e-3) break;
            const obsd_t& d = og.obs.data[j];

            bool has_signal = false;
            for (int k = 0; k < NFREQ + NEXOBS; ++k) {
                if (d.code[k] || d.P[k] != 0.0 || d.L[k] != 0.0) {
                    msg_obs.observations.push_back(gnss_utils::obsToMsg(d, k));
                    has_signal = true;
                }
            }
            if (has_signal) epoch_sats.push_back(d.sat);
            j++;
        }

        std::sort(epoch_sats.begin(), epoch_sats.end());
        epoch_sats.erase(std::unique(epoch_sats.begin(), epoch_sats.end()),
                         epoch_sats.end());

        // Select best ephemeris for each observed satellite.
        // For on-change mode, only emit when the selected nav index changes.
        for (int sat : epoch_sats) {
            int sys = satsys(sat, NULL);
            if (sys == SYS_GLO) {
                if (sat_geph_indices.count(sat)) {
                    int idx = select_best_eph(sat_geph_indices[sat], ng.nav.geph,
                                              t_curr, args.max_dtoe_glonass);
                    if (idx == -1) continue;
                    if (args.eph_mode == EphMode::OnChange) {
                        auto it = last_geph_idx.find(sat);
                        if (it != last_geph_idx.end() && it->second == idx) continue;
                        last_geph_idx[sat] = idx;
                    }
                    msg_eph.glonass_ephemeris.push_back(
                        gnss_utils::gephToMsg(ng.nav.geph[idx]));
                }
            } else {
                if (sat_eph_indices.count(sat)) {
                    int idx = select_best_eph(sat_eph_indices[sat], ng.nav.eph,
                                              t_curr, args.max_dtoe_gnss);
                    if (idx == -1) continue;
                    if (args.eph_mode == EphMode::OnChange) {
                        auto it = last_eph_idx.find(sat);
                        if (it != last_eph_idx.end() && it->second == idx) continue;
                        last_eph_idx[sat] = idx;
                    }
                    msg_eph.gnss_ephemeris.push_back(
                        gnss_utils::ephToMsg(ng.nav.eph[idx]));
                }
            }
        }

        writer.write(msg_obs, "/gnss/observation", stamp);
        ++n_obs_epochs;

        // Write all pos rows with t <= t_curr (interleaved in timestamp order).
        while (pos_idx < pos_result.rows.size()) {
          if (timediff(pos_result.rows[pos_idx].t, t_curr) > 1e-3) break;
          rclcpp::Time ps = gnss_converter_io::toRosTimeGpst(pos_result.rows[pos_idx].t);
          writer.write(pos_result.rows[pos_idx].sol, "/gnss/solution", ps);
          ++pos_idx;
        }

        if (!msg_eph.gnss_ephemeris.empty() || !msg_eph.glonass_ephemeris.empty()) {
            writer.write(msg_eph, "/gnss/ephemeris", stamp);
            ++n_eph_msgs;
        }

        if (n_obs_epochs % kProgressEvery == 0) {
            std::cerr << "[rinex_to_rosbag] " << n_obs_epochs << " obs epochs, "
                      << n_eph_msgs << " eph msgs written\n";
        }

        i = j;
    }

    // Flush any pos rows that follow the last obs epoch.
    while (pos_idx < pos_result.rows.size()) {
      rclcpp::Time ps = gnss_converter_io::toRosTimeGpst(pos_result.rows[pos_idx].t);
      writer.write(pos_result.rows[pos_idx].sol, "/gnss/solution", ps);
      ++pos_idx;
    }

    std::cerr << "[rinex_to_rosbag] done: " << n_obs_epochs
              << " obs epochs, " << n_eph_msgs << " eph msgs";
    if (!pos_result.rows.empty())
      std::cerr << ", " << pos_result.rows.size() << " solution msgs";
    std::cerr << " -> " << args.output_bag << "\n";
  }  // writer closed here

  rclcpp::shutdown();
  if (n_obs_epochs == 0) {
    std::filesystem::remove_all(args.output_bag);
    std::fprintf(stderr, "[rinex_to_rosbag] error: no obs epochs written; removed %s\n",
                 args.output_bag.c_str());
    return 1;
  }
  return 0;
}