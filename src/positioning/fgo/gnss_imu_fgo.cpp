// gnss_imu_fgo: tightly-coupled GNSS/IMU fixed-lag smoother.
#include "ros_node.hpp"

int main(int argc, char **argv) {
  gnss_fgo::NodeVariant variant;
  variant.node_name = "gnss_imu_fgo";
  variant.solution_topic = "/gnss/solution";
  variant.banner = "GNSS/IMU; UTC/GPST header "
                   "stamps read off the rover observations, no GLONASS RTK";
  variant.imu = true;
  return gnss_fgo::runNode(argc, argv, variant);
}
