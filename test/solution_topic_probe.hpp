// Black-box I/O for a computed-solution node: publish inputs on its topics and
// collect what it publishes, through real ROS topics.
#pragma once
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "gnss_ros_standardization/msg/gnss_observations.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"

namespace solution_probe {
using Obs = gnss_ros_standardization::msg::GnssObservations;
using Sol = gnss_ros_standardization::msg::GnssSolution;

inline Obs epoch(std::uint32_t week, double tow) {
  Obs m; m.week = week; m.tow = tow;
  m.header.stamp = rclcpp::Time(static_cast<int64_t>(std::llround((1.7e9 + tow) * 1e9)));
  return m;
}

// Waits (up to 3 s) until a publisher reaches its subscriber.
template <class Publisher>
inline void waitMatched(const Publisher& pub) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (pub->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline)
    rclcpp::sleep_for(std::chrono::milliseconds(10));
}

// Spins `node` and a probe node until `count` solutions arrived or 5 s passed.
inline std::vector<Sol> collect(const rclcpp::Node::SharedPtr& node, const std::string& solution_topic,
                                const std::function<void(rclcpp::Node&)>& feed, std::size_t count) {
  auto io = std::make_shared<rclcpp::Node>("solution_probe");
  std::vector<Sol> out;
  auto sub = io->create_subscription<Sol>(solution_topic, rclcpp::QoS(100),
                                          [&](const Sol& m) { out.push_back(m); });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node); exec.add_node(io);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  // Let discovery settle before feeding, so nothing is published unheard.
  while (std::chrono::steady_clock::now() < deadline &&
         (node->count_subscribers(solution_topic) == 0 || io->count_publishers(solution_topic) == 0))
    exec.spin_some(std::chrono::milliseconds(10));
  feed(*io);
  while (out.size() < count && std::chrono::steady_clock::now() < deadline)
    exec.spin_some(std::chrono::milliseconds(10));
  return out;
}
}  // namespace solution_probe
