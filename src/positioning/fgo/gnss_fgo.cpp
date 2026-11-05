// gnss_fgo: tightly-coupled GNSS-only fixed-lag smoother with a
// constant-velocity MotionFactor.
#include "ros_node.hpp"

int main(int argc, char **argv) {
  gnss_fgo::NodeVariant variant;
  variant.node_name = "gnss_fgo";
  variant.profile = "gnss_motion_ratio3_v1";
  variant.solution_topic = "/gnss/fgo/solution";
  variant.banner = "GNSS-only MotionFactor, Ratio 3.0 direct FIX/hold; "
                   "output stamped with the rover header, no GLONASS RTK";
  variant.imu = false;
  return gnss_fgo::runNode(argc, argv, variant);
}
