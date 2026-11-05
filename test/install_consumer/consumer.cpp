#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

#include <gnss_ros_standardization/bag_io_utils.hpp>
#include <gnss_ros_standardization/gnss_imu_kalman_filter.hpp>
#include <gnss_ros_standardization/gnss_preprocessor.hpp>
#include <gnss_ros_standardization/obs_converter.hpp>
#include <rclcpp/rclcpp.hpp>

using Observations = gnss_ros_standardization::msg::GnssObservations;
using Ephemerides = gnss_ros_standardization::msg::GnssEphemerides;

int main(int argc, char** argv) {
  if (Eigen::Matrix3d::Identity().trace() != 3.0) {
    throw std::runtime_error("Eigen public-header compatibility failed");
  }
  rclcpp::init(argc, argv);
  if (MAXPRNCMP != 63 || satno(SYS_CMP, 63) != NSATGPS+NSATGLO+NSATGAL+NSATQZS+63 ||
      satno(SYS_IRN, 1) != NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+1) {
    throw std::runtime_error("installed BeiDou limit/MAXSAT ABI mismatch");
  }
  auto node = std::make_shared<rclcpp::Node>("installed_cpp_consumer");
  const auto qos = rclcpp::QoS(10).reliable();
  auto observations = node->create_publisher<Observations>("echo/observations", qos);
  auto ephemerides = node->create_publisher<Ephemerides>("echo/ephemerides", qos);
  auto obs_sub = node->create_subscription<Observations>(
      "input/observations", qos, [observations](Observations::ConstSharedPtr msg) {
        observations->publish(*msg);
      });
  auto eph_sub = node->create_subscription<Ephemerides>(
      "input/ephemerides", qos, [ephemerides](Ephemerides::ConstSharedPtr msg) {
        ephemerides->publish(*msg);
      });
  // Exercise installed RTKLIB and gnss_utils together. If ABI-affecting options
  // such as NEXOBS/ENAGLO are not exported, these structs/satellite IDs disagree.
  obsd_t raw{};
  raw.time = gpst2time(2400, 123456.0);
  raw.sat = satno(SYS_GLO, 1);
  raw.freq = 13;  // actual channel +6
  raw.code[0] = CODE_L1C;
  raw.P[0] = 21000000.25;
  raw.L[0] = std::numeric_limits<double>::quiet_NaN();
  raw.D[0] = 0.0;
  raw.SNR[0] = 42.5;
  raw.LLI[0] = 0xc3;
  if (!raw.sat || gnss_utils::resolveSatellite("R01", "R", 1) != raw.sat) {
    throw std::runtime_error("installed RTKLIB constellation configuration mismatch");
  }
  Observations probe;
  probe.week = 2400;
  probe.tow = 123456.0;
  probe.observations.push_back(gnss_utils::obsToMsg(raw, 0));
  auto publisher = node->create_publisher<Observations>("cpp/probe", qos);
  auto timer = node->create_wall_timer(std::chrono::milliseconds(100),
                                     [publisher, probe]() { publisher->publish(probe); });
  std::cout << "Installed consumer: NFREQ=" << NFREQ << " NEXOBS=" << NEXOBS
            << " MAXSAT=" << MAXSAT << " sizeof(obsd_t)=" << sizeof(obsd_t) << std::endl;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
