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
