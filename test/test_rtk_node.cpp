#define RTK_NO_MAIN
#include "real_time_kinematic.cpp"
#include <gtest/gtest.h>
#include "solution_topic_probe.hpp"

// Q-A8: a rover epoch the node cannot solve (here: no navigation yet) is
// published as STATUS_NONE with NaN position and velocity, at the rover time.
TEST(RtkNode, EpochWithoutSolutionIsPublishedAsNone) {
  const char* argv[] = {"test", "--ros-args", "-p", "tcp_port:=0",
                        "-p", "ant2.pos:=[35.0, 139.0, 10.0]",
                        "-p", "topics.rover_observation:=/rtk_test/rover",
                        "-p", "topics.base_observation:=/rtk_test/base",
                        "-p", "topics.solution:=/rtk_test/solution"};
  rclcpp::init(sizeof(argv) / sizeof(argv[0]), argv);
  {
    auto node = std::make_shared<RtkPositionNode>();
    const auto out = solution_probe::collect(node, "/rtk_test/solution", [](rclcpp::Node& io) {
      auto rover = io.create_publisher<solution_probe::Obs>("/rtk_test/rover", 10);
      auto base = io.create_publisher<solution_probe::Obs>("/rtk_test/base", 10);
      solution_probe::waitMatched(rover); solution_probe::waitMatched(base);
      base->publish(solution_probe::epoch(2300, 100.));
      rover->publish(solution_probe::epoch(2300, 100.));
    }, 1);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].status, solution_probe::Sol::STATUS_NONE);
    EXPECT_EQ(out[0].time_week, 2300u);
    EXPECT_DOUBLE_EQ(out[0].time_tow, 100.);
    for (double v : {out[0].latitude, out[0].pos_ecef.x, out[0].pos_cov_ecef[0], out[0].vel_ecef.x})
      EXPECT_TRUE(std::isnan(v));
  }
  rclcpp::shutdown();
}

// A bag loop (or receiver restart) steps the input clocks back: the node starts
// a new timeline instead of dropping every later epoch as a late arrival.
TEST(RtkNode, ClockSteppingBackStartsANewTimeline) {
  const char* argv[] = {"test", "--ros-args", "-p", "tcp_port:=0",
                        "-p", "ant2.pos:=[35.0, 139.0, 10.0]",
                        "-p", "topics.rover_observation:=/rtk_loop/rover",
                        "-p", "topics.base_observation:=/rtk_loop/base",
                        "-p", "topics.solution:=/rtk_loop/solution"};
  rclcpp::init(sizeof(argv) / sizeof(argv[0]), argv);
  {
    auto node = std::make_shared<RtkPositionNode>();
    const auto out = solution_probe::collect(node, "/rtk_loop/solution", [](rclcpp::Node& io) {
      auto rover = io.create_publisher<solution_probe::Obs>("/rtk_loop/rover", 10);
      auto base = io.create_publisher<solution_probe::Obs>("/rtk_loop/base", 10);
      solution_probe::waitMatched(rover); solution_probe::waitMatched(base);
      base->publish(solution_probe::epoch(2300, 300.));
      rover->publish(solution_probe::epoch(2300, 300.));
      base->publish(solution_probe::epoch(2300, 200.));   // loop: 100 s back
      rover->publish(solution_probe::epoch(2300, 200.));
    }, 2);
    ASSERT_EQ(out.size(), 2u);
    EXPECT_DOUBLE_EQ(out[0].time_tow, 300.);
    EXPECT_DOUBLE_EQ(out[1].time_tow, 200.);
  }
  rclcpp::shutdown();
}

namespace {
grs::GnssBaseStation station(uint16_t id, double from_tow, bool valid, double x = 0) {
  grs::GnssBaseStation r;
  r.station_id = id; r.valid_from_week = 2300; r.valid_from_tow = from_tow;
  r.position_valid = valid;
  if (valid) { r.arp_ecef.x = x; r.arp_ecef.y = 3348993.8; r.arp_ecef.z = 3698211.7; }
  return r;
}
grs::GnssObservations baseEpoch(double tow) {
  grs::GnssObservations m; m.week = 2300; m.tow = tow; return m;
}
}  // namespace

TEST(BaseStationTable, EpochBelongsToLatestRecordNotAfterIt) {
  BaseStationTable table;
  table.insert(station(42, 100., true, -3961904.9));
  table.insert(station(77, 102., false));
  EXPECT_EQ(table.lookup(2300 * 604800. + 99.), nullptr);
  EXPECT_EQ(table.lookup(2300 * 604800. + 101.)->station_id, 42);
  EXPECT_FALSE(table.lookup(2300 * 604800. + 102.)->position_valid);
  table.insert(station(77, 102., true, -3962904.9));  // republished with its position
  EXPECT_TRUE(table.lookup(2300 * 604800. + 103.)->position_valid);
  EXPECT_EQ(table.lookup(2300 * 604800. + 101.)->station_id, 42);
  for (int i = 0; i < 20; ++i) table.insert(station(1, 200. + i, true, -1.));
  EXPECT_EQ(table.lookup(2300 * 604800. + 150.), nullptr);  // oldest dropped beyond capacity
  EXPECT_DOUBLE_EQ(table.lookup(2300 * 604800. + 1e6)->valid_from_tow, 219.);
}

struct RtkNodeTestAccess {
  static void station(RtkPositionNode& n, const grs::GnssBaseStation& r) {
    n.onStation(std::make_shared<grs::GnssBaseStation>(r));
  }
  static bool apply(RtkPositionNode& n, double tow) {
    std::lock_guard<std::mutex> lk(n.obs_mtx_);
    return n.applyStation(baseEpoch(tow));
  }
  // rtkpos reloads rtk.rb from opt.rb each epoch, so both must hold the base.
  static double rbx(const RtkPositionNode& n) {
    EXPECT_DOUBLE_EQ(n.rtk_.opt.rb[0], n.rtk_.rb[0]);
    return n.rtk_.rb[0];
  }
  static double originx(const RtkPositionNode& n) { return n.fixed_origin_ecef_[0]; }
  static std::uint64_t late(const RtkPositionNode& n) { return n.station_records_late_; }
};

// ant2.postype rtcm: the base follows the station records, a station change
// restarts the filter at the new ARP, and the ENU origin stays at the first.
TEST(RtkNode, RtcmBasePositionFollowsStationRecords) {
  const char* argv[] = {"test", "--ros-args", "-p", "tcp_port:=0", "-p", "ant2.postype:=rtcm",
                        "-p", "topics.base_station:=/rtk_station/station"};
  rclcpp::init(sizeof(argv) / sizeof(argv[0]), argv);
  {
    RtkPositionNode node;  // no ant2.pos needed
    using A = RtkNodeTestAccess;
    EXPECT_FALSE(A::apply(node, 100.));  // no record yet
    A::station(node, station(42, 100., true, -3961904.9));
    EXPECT_TRUE(A::apply(node, 100.));
    EXPECT_DOUBLE_EQ(A::rbx(node), -3961904.9);
    A::station(node, station(77, 102., false));
    EXPECT_TRUE(A::apply(node, 101.));
    EXPECT_FALSE(A::apply(node, 102.));  // position pending: not solved against 42
    A::station(node, station(77, 102., true, -3962904.9));
    EXPECT_EQ(A::late(node), 0u);  // epoch 102 was never solved
    EXPECT_TRUE(A::apply(node, 103.));
    EXPECT_DOUBLE_EQ(A::rbx(node), -3962904.9);
    EXPECT_DOUBLE_EQ(A::originx(node), -3961904.9);
    // Station 88's record overtaken by its own epochs (solved against 77).
    EXPECT_TRUE(A::apply(node, 105.));
    A::station(node, station(88, 104., true, -3963904.9));
    EXPECT_EQ(A::late(node), 1u);
    EXPECT_TRUE(A::apply(node, 106.));
    EXPECT_DOUBLE_EQ(A::rbx(node), -3963904.9);
    // A republished record of the active base is not a change.
    A::station(node, station(88, 104., true, -3963904.9));
    EXPECT_EQ(A::late(node), 1u);
  }
  rclcpp::shutdown();
}
