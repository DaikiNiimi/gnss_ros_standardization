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
#include <memory>
#include <queue>
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

// One RINEX OBS input → one rosbag topic. Multiple sources (e.g. rover/base
// for RTK) can be specified; epochs are written in GPST-time order across
// sources so `ros2 bag play` reproduces the real-time interleave.
struct ObsSource {
  std::string label;  // empty = unlabeled (single-source compat mode)
  std::string path;
  std::string topic;  // resolved after CLI parse
};

struct Args {
  std::vector<ObsSource> obs_sources;
  std::vector<std::string> nav_paths;
  std::string output_bag;
  std::string storage_id;       // empty => auto-detect from --out extension or distro default
  std::string pos_path;         // optional; when set, also writes the solution topic
  std::string pos_label;        // empty = bind to first OBS source
  std::string topic_nav  = "/gnss/ephemeris";
  std::string topic_pos  = "/gnss/solution";
  std::map<std::string, std::string> topic_obs_overrides;  // label -> explicit topic
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

// Split a CLI value at the first '=' into {label, rest}. If no '=' is found,
// label is empty and rest is the whole input. Used for --obs/--topic-obs/--pos.
static std::pair<std::string, std::string> splitLabel(const std::string& s) {
  auto pos = s.find('=');
  if (pos == std::string::npos) return {"", s};
  return {s.substr(0, pos), s.substr(pos + 1)};
}

Args parseArgs(const std::vector<std::string>& args) {
  Args a;
  for (size_t i = 0; i < args.size(); ++i) {
    if (args[i] == "--obs" && i + 1 < args.size()) {
      auto [label, path] = splitLabel(args[++i]);
      a.obs_sources.push_back(ObsSource{label, path, ""});
    } else if (args[i] == "--nav" && i + 1 < args.size()) {
      a.nav_paths.push_back(args[++i]);
    } else if (args[i] == "--topic-obs" && i + 1 < args.size()) {
      auto [label, topic] = splitLabel(args[++i]);
      a.topic_obs_overrides[label] = topic;
    } else if (args[i] == "--topic-nav" && i + 1 < args.size()) {
      a.topic_nav = args[++i];
    } else if (args[i] == "--topic-pos" && i + 1 < args.size()) {
      a.topic_pos = args[++i];
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
      auto [label, path] = splitLabel(args[++i]);
      a.pos_label = label;
      a.pos_path  = path;
    }
  }
  return a;
}

// Resolve each ObsSource.topic from CLI overrides, label-derived auto names,
// and the single-source compatibility fallback. Returns false on validation
// failure (caller prints + exits).
static bool resolveObsTopics(Args& a, std::ostream& err) {
  // 1. Check for duplicate labels and multiple unlabeled sources.
  std::map<std::string, int> label_counts;
  for (const auto& s : a.obs_sources) label_counts[s.label]++;
  for (const auto& [lbl, n] : label_counts) {
    if (n > 1) {
      if (lbl.empty()) {
        err << "Error: multiple --obs without a name= label. "
               "Use --obs name=path (e.g. --obs rover=rover.obs --obs base=base.obs).\n";
      } else {
        err << "Error: duplicate --obs label '" << lbl << "'.\n";
      }
      return false;
    }
  }

  // 2. Validate that every --topic-obs label corresponds to an --obs source.
  for (const auto& [lbl, topic] : a.topic_obs_overrides) {
    if (label_counts.count(lbl) == 0) {
      err << "Error: --topic-obs label '" << lbl
          << "' does not match any --obs source.\n";
      return false;
    }
  }

  // 3. Assign topics: override > /<label>/observation > /gnss/observation.
  for (auto& src : a.obs_sources) {
    auto it = a.topic_obs_overrides.find(src.label);
    if (it != a.topic_obs_overrides.end()) {
      src.topic = it->second;
    } else if (!src.label.empty()) {
      src.topic = "/" + src.label + "/observation";
    } else {
      src.topic = "/gnss/observation";
    }
  }

  // 4. If --pos has a label, it must match an --obs label.
  if (!a.pos_path.empty() && !a.pos_label.empty() &&
      label_counts.count(a.pos_label) == 0) {
    err << "Error: --pos label '" << a.pos_label
        << "' does not match any --obs source.\n";
    return false;
  }
  return true;
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

  if (args.output_bag.empty() && !args.obs_sources.empty())
    args.output_bag = gnss_converter_io::deriveOutputPath(args.obs_sources.front().path, "");

  if (args.obs_sources.empty()) {
    std::cerr <<
      "Usage: rinex_to_rosbag --obs [name=]<obs_file> [--obs [name=]<obs_file> ...]\n"
      "                       [--nav <nav_file> ...]\n"
      "                       [--topic-obs [name=]<topic>] [--topic-nav <topic>]\n"
      "                       [--topic-pos <topic>]\n"
      "                       [--out <bag_path>] [--storage mcap|sqlite3]\n"
      "                       [--eph-mode per-epoch|on-change]\n"
      "                       [--max-dtoe-gnss <sec>] [--max-dtoe-glo <sec>]\n"
      "                       [--pos [name=]<pos_file>]\n"
      "\n"
      "  --obs         Repeatable. Use 'name=path' (e.g. rover=rover.obs) to give the\n"
      "                source a label; topic becomes /<name>/observation. Without a\n"
      "                label the topic is /gnss/observation (single-source mode).\n"
      "                Epochs from all sources are interleaved in GPST time order so\n"
      "                'ros2 bag play' reproduces the real-time arrival pattern.\n"
      "  --topic-obs   Override topic per source: 'name=topic' (or bare 'topic' for the\n"
      "                unlabeled source). Default: /<name>/observation, or /gnss/observation.\n"
      "  --topic-nav   NAV topic name (default /gnss/ephemeris). NAV is shared across\n"
      "                all OBS sources (single ephemeris stream).\n"
      "  --topic-pos   POS topic name (default /gnss/solution).\n"
      "  --pos         Optional RTKLIB .pos file. Use 'name=path' to bind it to a\n"
      "                specific OBS source (e.g. --pos rover=rover.pos). Without a\n"
      "                label, binds to the first --obs source. Solution rows are\n"
      "                interleaved at that source's epoch boundaries.\n"
      "  --eph-mode    per-epoch: republish all visible ephemerides every obs epoch.\n"
      "                on-change (default): publish per-sat only when the selected\n"
      "                     nav index changes (avoids massive duplication; first\n"
      "                     visible epoch of each sat still publishes once).\n"
      "  --max-dtoe-*  max |t_obs - toe| for ephemeris selection [sec]. Defaults:\n"
      "                GNSS=" << kDefaultMaxDtoeGnss
      << ", GLONASS=" << kDefaultMaxDtoeGlonass << ".\n"
      "  When --nav is omitted, the NAV topic is created but empty.\n";
    return 1;
  }

  if (!resolveObsTopics(args, std::cerr)) return 1;

  // 1. Read each OBS source. unique_ptr<ObsGuard> keeps ObsGuard non-movable
  //    requirements satisfied while letting us hold many in a vector.
  std::vector<std::unique_ptr<ObsGuard>> obs_guards;
  NavGuard nav_tmp_g;  // throwaway nav from OBS reads (a few mixed records)
  for (const auto& src : args.obs_sources) {
    auto og = std::make_unique<ObsGuard>();
    sta_t sta = {};
    int obs_rc = readrnx(src.path.c_str(), 1, "", &og->obs, &nav_tmp_g.nav, &sta);
    if (obs_rc <= 0 || og->obs.n <= 0) {
      std::cerr << "Error: failed to read observations from '" << src.path
                << "' (readrnx rc=" << obs_rc << ", n=" << og->obs.n
                << "). Check file path and RINEX OBS format.\n";
      return 1;
    }
    obs_guards.push_back(std::move(og));
  }

  // 2. Read NAV data into a single accumulating nav_t (shared across sources).
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

  // 4. Optionally preload .pos rows for solution interleaving, bound to one source.
  gnss_pos_reader::ParseResult pos_result;
  int pos_src_idx = -1;
  if (!args.pos_path.empty()) {
    pos_result = gnss_pos_reader::parsePosFile(args.pos_path, std::cerr);
    if (pos_result.rows.empty()) {
      std::cerr << "Warning: --pos file produced no rows; solution topic will be absent.\n";
    } else {
      // Resolve which OBS source this pos is bound to.
      if (args.pos_label.empty()) {
        pos_src_idx = 0;
      } else {
        for (size_t s = 0; s < args.obs_sources.size(); ++s) {
          if (args.obs_sources[s].label == args.pos_label) { pos_src_idx = (int)s; break; }
        }
      }
      // Sanity check: warn if pos and bound-source obs time ranges don't overlap.
      const auto& og_bound = obs_guards[pos_src_idx]->obs;
      if (og_bound.n > 0) {
        gtime_t obs_first = og_bound.data[0].time;
        gtime_t obs_last  = og_bound.data[og_bound.n - 1].time;
        gtime_t pos_first = pos_result.rows.front().t;
        gtime_t pos_last  = pos_result.rows.back().t;
        if (timediff(pos_last, obs_first) < -1.0 || timediff(pos_first, obs_last) > 1.0) {
          std::cerr << "Warning: --pos time range does not overlap with obs time range for "
                    << "source '" << args.obs_sources[pos_src_idx].label
                    << "'. Verify the .pos file was generated from the same session.\n";
        }
      }
    }
  }

  // 5. Setup bag writer. Scoped so the writer closes before the post-loop
  //    cleanup check (remove_all requires the file handles to be released first).
  size_t n_obs_epochs = 0;
  size_t n_eph_msgs   = 0;
  size_t n_sol_msgs   = 0;
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

    // Per-source observation topic (one create_topic per source).
    gnss_converter_io::set_qos_profile(tm, gnss_converter_io::defaultConverterQosYaml());
    for (const auto& src : args.obs_sources) {
      tm.name = src.topic;
      tm.type = "gnss_ros_standardization/msg/GnssObservations";
      writer.create_topic(tm);
    }

    // NAV topic is consumed by positioning nodes (SPP, RTK, gnss_visualizer)
    // with transient_local() subscriptions — use matching TRANSIENT_LOCAL durability
    // so late subscribers receive the last published ephemeris immediately.
    gnss_converter_io::set_qos_profile(tm, gnss_converter_io::transientLocalConverterQosYaml());
    tm.name = args.topic_nav;
    tm.type = "gnss_ros_standardization/msg/GnssEphemerides";
    writer.create_topic(tm);

    if (!pos_result.rows.empty()) {
      gnss_converter_io::set_qos_profile(tm, gnss_converter_io::defaultConverterQosYaml());
      tm.name = args.topic_pos;
      tm.type = "gnss_ros_standardization/msg/GnssSolution";
      writer.create_topic(tm);
    }

    // Per-satellite "last published nav index" for --eph-mode on-change.
    // Shared across all OBS sources because NAV is shared. -1 means "never published yet".
    std::map<int, int> last_eph_idx;
    std::map<int, int> last_geph_idx;

    // Cursor into pos rows for the bound source.
    size_t pos_idx = 0;

    // 6. Time-interleaved epoch processing using a min-heap across sources.
    //    Each heap entry is the next-unprocessed epoch start of one source.
    struct HeapItem {
      gtime_t t;
      int     src_idx;
    };
    struct HeapCmp {
      bool operator()(const HeapItem& a, const HeapItem& b) const {
        double dt = timediff(a.t, b.t);
        if (std::fabs(dt) > 1e-9) return dt > 0.0;   // earlier time first
        return a.src_idx > b.src_idx;                // stable tie-break by src
      }
    };
    std::priority_queue<HeapItem, std::vector<HeapItem>, HeapCmp> heap;
    std::vector<int> cursor(args.obs_sources.size(), 0);
    for (size_t s = 0; s < args.obs_sources.size(); ++s) {
      if (obs_guards[s]->obs.n > 0) {
        heap.push({obs_guards[s]->obs.data[0].time, (int)s});
      }
    }

    constexpr size_t kProgressEvery = 1000;
    while (!heap.empty()) {
      HeapItem top = heap.top(); heap.pop();
      const int src_idx = top.src_idx;
      const auto& obs = obs_guards[src_idx]->obs;
      const auto& src = args.obs_sources[src_idx];

      gtime_t t_curr = top.t;
      rclcpp::Time stamp = gnss_converter_io::toRosTimeGpst(t_curr);

      GnssObservations msg_obs;
      int week = 0;
      msg_obs.tow = time2gpst(t_curr, &week);
      msg_obs.week = week;
      msg_obs.header.stamp = stamp;
      msg_obs.header.frame_id = src.label.empty() ? "gnss_receiver" : src.label;

      GnssEphemerides msg_eph;
      msg_eph.header.stamp = stamp;
      msg_eph.header.frame_id = "gnss_receiver";

      std::vector<int> epoch_sats;

      int i = cursor[src_idx];
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
        if (has_signal) epoch_sats.push_back(d.sat);
        j++;
      }

      std::sort(epoch_sats.begin(), epoch_sats.end());
      epoch_sats.erase(std::unique(epoch_sats.begin(), epoch_sats.end()),
                       epoch_sats.end());

      // Select best ephemeris for each observed satellite.
      // For on-change mode, only emit when the selected nav index changes
      // (dictionaries are shared across sources since NAV is shared).
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

      writer.write(msg_obs, src.topic, stamp);
      ++n_obs_epochs;

      if (!msg_eph.gnss_ephemeris.empty() || !msg_eph.glonass_ephemeris.empty()) {
        writer.write(msg_eph, args.topic_nav, stamp);
        ++n_eph_msgs;
      }

      // Flush any pos rows with t <= t_curr (only when this is the bound source).
      if (src_idx == pos_src_idx) {
        while (pos_idx < pos_result.rows.size()) {
          if (timediff(pos_result.rows[pos_idx].t, t_curr) > 1e-3) break;
          rclcpp::Time ps = gnss_converter_io::toRosTimeGpst(pos_result.rows[pos_idx].t);
          writer.write(pos_result.rows[pos_idx].sol, args.topic_pos, ps);
          ++pos_idx;
          ++n_sol_msgs;
        }
      }

      if (n_obs_epochs % kProgressEvery == 0) {
        std::cerr << "[rinex_to_rosbag] " << n_obs_epochs << " obs epochs, "
                  << n_eph_msgs << " eph msgs written\n";
      }

      cursor[src_idx] = j;
      if (j < obs.n) heap.push({obs.data[j].time, src_idx});
    }

    // Flush any pos rows that follow the last obs epoch of the bound source.
    while (pos_idx < pos_result.rows.size()) {
      rclcpp::Time ps = gnss_converter_io::toRosTimeGpst(pos_result.rows[pos_idx].t);
      writer.write(pos_result.rows[pos_idx].sol, args.topic_pos, ps);
      ++pos_idx;
      ++n_sol_msgs;
    }

    std::cerr << "[rinex_to_rosbag] done: " << n_obs_epochs
              << " obs epochs, " << n_eph_msgs << " eph msgs";
    if (!pos_result.rows.empty())
      std::cerr << ", " << n_sol_msgs << " solution msgs";
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
