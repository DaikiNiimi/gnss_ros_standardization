// Bag reading and construction of the plot model.
#pragma once

#include <string>
#include <vector>

#include "gnss_ros_standardization/ephemeris_store.hpp"

#include "bag_session.hpp"
#include "plot_data.hpp"

namespace rtk_bag_plot {

struct BagContents {
  std::vector<TopicTimeline> timelines;
  std::vector<RawObservationTopic> observations;
  std::vector<RawSolutionTopic> solutions;
  std::vector<RawImuTopic> imus;
  gnss_utils::EphemerisStore ephemerides;
  std::vector<double> bag_minus_gpst;  // samples for the bag-to-GPST mapping
  std::vector<std::string> comparison_notes;
  bool has_ephemeris{false};
  size_t rejected_ephemerides{0};
};

// Reads every message of the bag. Throws on an interrupted or unreadable bag.
BagContents readBag(BagSession& session);

// Adds the solutions of another bag, labelled "<label>:<topic>", for comparison.
void appendComparisonBag(BagContents& contents, BagSession& session, const std::string& label);

// Adds the solutions of an RTKLIB .pos file for comparison. Throws when the
// file cannot be read or holds no solution.
void appendPosFile(BagContents& contents, const std::string& path);

// Builds all plot series. `stat_path` may be empty.
PlotModel buildPlotModel(BagContents& contents, const std::string& bag_name,
                         const std::string& stat_path);

// Applies the size budget to observation and solution series in place.
void applySizeBudget(PlotModel& model, size_t budget_bytes);

size_t estimateObservationFixedBytes(const ObservationResult& observation);

// Topic statistics from arrival times; exposed for testing.
TopicSummary summarizeTimeline(const TopicTimeline& timeline, double bag_minus_gpst,
                               double origin_gpst, bool bag_time_axis);

}  // namespace rtk_bag_plot
