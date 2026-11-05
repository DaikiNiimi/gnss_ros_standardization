#define SPP_NO_MAIN
#include "single_point_positioning.cpp"
#include <gtest/gtest.h>
#include "solution_topic_probe.hpp"

// Q-A8: an observation epoch the node cannot solve (here: no navigation yet)
// is published as STATUS_NONE with NaN position and velocity.
TEST(SppNode, EpochWithoutSolutionIsPublishedAsNone) {
  const char* argv[] = {"test", "--ros-args", "-p", "tcp_port:=0",
                        "-p", "topics.observation:=/spp_test/observation",
                        "-p", "topics.solution:=/spp_test/solution"};
  rclcpp::init(sizeof(argv) / sizeof(argv[0]), argv);
  {
    auto node = std::make_shared<SppPntposNode>();
    const auto out = solution_probe::collect(node, "/spp_test/solution", [](rclcpp::Node& io) {
      auto obs = io.create_publisher<solution_probe::Obs>("/spp_test/observation", 10);
      solution_probe::waitMatched(obs);
      obs->publish(solution_probe::epoch(2300, 100.));
    }, 1);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].status, solution_probe::Sol::STATUS_NONE);
    EXPECT_DOUBLE_EQ(out[0].time_tow, 100.);
    for (double v : {out[0].latitude, out[0].pos_ecef.x, out[0].pos_cov_ecef[0], out[0].vel_ecef.x})
      EXPECT_TRUE(std::isnan(v));
  }
  rclcpp::shutdown();
}
