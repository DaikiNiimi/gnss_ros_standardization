// gnss_imu_fgo: tightly-coupled GNSS/IMU fixed-lag smoother.
#include "ros_node.hpp"

int main(int argc, char **argv) {
  gnss_fgo::NodeVariant variant;
  variant.node_name = "gnss_imu_fgo";
  variant.profile = "ratio3_direct_v1";
  variant.solution_topic = "/gnss/imu_fgo/solution";
  variant.banner = "Ratio 3.0 direct FIX/hold; UTC/GPST header "
                   "stamps read off the rover observations, no GLONASS RTK";
  variant.imu = true;
  return gnss_fgo::runNode(argc, argv, variant);
}
