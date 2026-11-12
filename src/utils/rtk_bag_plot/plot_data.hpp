// Plain data carried from bag extraction through processing to the HTML encoder.
#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace rtk_bag_plot {

// Elevation masks offered by RTKPLOT's options dialog (deg).
inline constexpr std::array<int, 10> kElevationMasks{0, 5, 10, 15, 20, 25, 30, 35, 40, 45};

// Row flags stored per signal observation.
enum RowFlag : uint8_t {
  kFlagSlip = 1 << 0,      // LLI bit 0 (cycle slip)
  kFlagHalf = 1 << 1,      // LLI bit 1 (half-cycle ambiguity)
  kFlagHasP = 1 << 2,
  kFlagHasL = 1 << 3,
  kFlagHasD = 1 << 4,
  kFlagLgJump = 1 << 5,    // geometry-free jump
};

inline constexpr uint16_t kAngleInvalid = 0xFFFF;
inline constexpr int16_t kMultipathInvalid = std::numeric_limits<int16_t>::min();
inline constexpr double kSnrScale = 0.25;       // dB-Hz per count
inline constexpr double kMultipathScale = 0.002;  // m per count

// Identity of one (satellite, signal) pair within an observation topic.
struct SignalInfo {
  int sat{0};
  int sys{0};
  int band{-1};          // RTKLIB code2idx
  uint8_t code{0};
  uint8_t channel{255};  // GLONASS channel + 7, 255 = unknown
  int prn{0};
  std::string satid, system, code_str;
};

// One signal observation as read from the bag, before processing.
struct RawRow {
  uint32_t epoch{0};
  uint32_t signal{0};
  double p{0.0};
  double l{0.0};
  float snr{0.0F};
  uint8_t lli{0};
  uint8_t has_d{0};
};

struct RawObservationTopic {
  std::string topic;
  std::vector<SignalInfo> signals;
  std::map<std::string, uint32_t> signal_index;
  std::vector<double> epoch_gpst;  // seconds since the GPST origin
  std::vector<RawRow> rows;
  size_t rejected_signals{0};
};

struct RawSolutionRow {
  double t{0.0};
  std::array<double, 3> ecef{};
  std::array<float, 3> sd_enu{};
  std::array<float, 3> vel_enu{};
  std::array<float, 4> dop{};  // gdop, pdop, hdop, vdop
  float ratio{0.0F};
  float age{0.0F};
  uint16_t num_sats{0};
  uint8_t status{0};
};

struct RawSolutionTopic {
  std::string topic;
  std::string source{"bag"};  // "bag", "compare" or "pos"
  std::vector<RawSolutionRow> rows;
};

struct RawImuRow {
  double t{0.0};  // bag time (s), mapped to the plot time axis later
  std::array<float, 3> accel{};
  std::array<float, 3> gyro{};
};

struct RawImuTopic {
  std::string topic;
  std::vector<RawImuRow> rows;
};

// Arrival statistics collected for every topic in the bag.
struct TopicTimeline {
  std::string name, type;
  std::vector<int64_t> bag_ns;
  std::vector<double> gpst;  // GNSS topics: message epochs in arrival order
  size_t invalid{0};
};

struct Gap {
  double start{0.0};
  double duration{0.0};
};

struct TopicSummary {
  std::string name, type;
  size_t count{0};
  double first{0.0}, last{0.0}, rate_hz{0.0};
  double max_gap{0.0};
  size_t gap_count{0};
  size_t out_of_order{0};
  size_t duplicates{0};
  size_t invalid{0};
  bool gnss_epochs{false};
  std::vector<Gap> gaps;
};

struct SatelliteSeries {
  std::string id;
  int sat{0};
  int sys{0};
  std::vector<uint32_t> epoch;
  std::vector<uint16_t> az, el;  // 0.01 deg, kAngleInvalid when unknown
  std::vector<float> iono;
  std::vector<uint8_t> slip_reference;  // internal: LLI bit 0 on band 0/1
};

struct SignalSeries {
  uint32_t sat_index{0};
  std::string code;
  int band{-1};
  std::vector<uint32_t> epoch;
  std::vector<uint8_t> snr;
  std::vector<int16_t> mp;
  std::vector<uint8_t> flags;
};

struct DopSeries {
  int mask{0};
  std::vector<uint16_t> gdop, pdop, hdop, vdop;  // 0.01 units, 0 = unavailable
  std::vector<uint8_t> ns;
};

struct ObservationResult {
  std::string topic;
  std::vector<double> t;  // relative to the plot origin
  std::vector<SatelliteSeries> sats;
  std::vector<SignalSeries> signals;
  std::vector<DopSeries> dops;
  bool has_geometry{false};
  size_t kept_rows{0}, total_rows{0};
  size_t failed_position_epochs{0};
};

// One (time, xyz, quality) series, the counterpart of RTKPLOT's TIMEPOS.
struct TimePos {
  std::vector<double> t;
  std::vector<double> x, y, z;
  std::vector<int> q;
  size_t size() const { return t.size(); }
};

struct AxisStats {
  size_t n{0};
  double ave{0.0}, std{0.0}, rms{0.0};
};

// Statistics indexed by quality (0 = all, 1-7 = solution status) and axis.
using StatsTable = std::array<std::array<AxisStats, 3>, 8>;

struct SolutionResult {
  std::string topic;
  std::string source{"bag"};
  std::vector<double> t;
  std::vector<float> e, n, u;
  std::vector<uint8_t> q;
  std::vector<uint16_t> ns;
  std::vector<float> ratio, age, gdop, pdop, hdop, vdop;
  std::vector<float> sd_e, sd_n, sd_u;
  std::vector<float> vel_e, vel_n, vel_u;
  TimePos velocity, acceleration;
  StatsTable pos_stats{}, vel_stats{}, acc_stats{};
  std::array<size_t, 8> status_counts{};
  size_t total_rows{0};
};

struct SolutionDifference {
  size_t a{0}, b{0};
  TimePos diff;
  StatsTable stats{};
};

struct EphemerisMark {
  int sat{0};
  double transmit{0.0}, reference{0.0};  // relative to the plot origin
  bool unhealthy{false};
};

struct ImuResult {
  std::string topic;
  std::vector<double> t;
  std::array<std::vector<float>, 6> min, max;
  size_t total_rows{0};
};

struct ResidualResult {
  std::string path;
  std::vector<double> t;
  std::vector<uint8_t> sat_index, frequency, flags;
  std::vector<float> az, el, code_residual, phase_residual, snr;
  std::vector<std::string> sat_ids;
  std::vector<DopSeries> dops;
  std::vector<double> dop_t;
  size_t total_rows{0};
};

struct PlotModel {
  std::string bag;
  std::string time_basis{"gpst"};
  double origin_gpst{0.0};
  double leap_seconds{0.0};
  std::string solution_origin_topic;
  std::array<double, 3> solution_origin_llh{};
  std::vector<TopicSummary> topics;
  std::vector<SolutionResult> solutions;
  std::vector<SolutionDifference> differences;
  std::vector<ObservationResult> observations;
  std::vector<std::string> ephemeris_sats;
  std::vector<EphemerisMark> ephemerides;
  std::vector<ImuResult> imus;
  bool has_residuals{false};
  ResidualResult residuals;
  std::vector<std::string> notes;
};

}  // namespace rtk_bag_plot
