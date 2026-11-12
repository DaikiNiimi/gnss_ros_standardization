// gnss_fgo: tightly-coupled GNSS-only fixed-lag smoother with a
// constant-velocity MotionFactor.
#include "ros_node.hpp"

int main(int argc, char **argv) {
  gnss_fgo::NodeVariant variant;
  variant.node_name = "gnss_fgo";
  variant.solution_topic = "/gnss/solution";
  variant.banner = "GNSS-only MotionFactor; "
                   "output stamped with the rover header, no GLONASS RTK";
  variant.imu = false;
  return gnss_fgo::runNode(argc, argv, variant);
}
