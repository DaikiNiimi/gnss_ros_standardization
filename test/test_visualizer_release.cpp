#define GNSS_VISUALIZER_NO_MAIN
#include "../src/utils/gnss_visualizer.cpp"
#include <gtest/gtest.h>
struct VisualizerReleaseAccess {
  static void solution(GnssVisualizer& n,double x,int64_t stamp) {
    auto m=std::make_shared<gnss_ros_standardization::msg::GnssSolution>();
    m->status=m->STATUS_FIX;m->pos_enu.x=x;m->pos_enu.y=2;m->pos_enu.z=3;
    m->header.stamp=rclcpp::Time(stamp);n.solCallback(m);
  }
  static size_t count(const GnssVisualizer& n){return n.pos_history_.size();}
  static double span(const GnssVisualizer& n){return n.pos_history_.back().t_sec-n.pos_history_.front().t_sec;}
  static cv::Mat snr(GnssVisualizer& n,int backend_sat) {
    auto m=std::make_shared<gnss_ros_standardization::msg::GnssObservations>();
    gnss_ros_standardization::msg::GnssObservation o;
    o.sat=backend_sat;o.satid="G01";o.system="G";o.prn=1;o.code=1;o.code_str="1C";o.snr=45;
    m->observations.push_back(o);n.last_obs_=m;
    cv::Mat image(720,1280,CV_8UC3,cv::Scalar(0,0,0));
    n.renderSnrPlot(image,cv::Rect(0,0,1280,720));return image;
  }
};
TEST(ReleaseW1, NonfinitePointsAndDifferentSensorClocksDoNotCorruptHistory) {
  rclcpp::init(0,nullptr);
  {
    rclcpp::NodeOptions o;o.parameter_overrides({rclcpp::Parameter("use_gui",false)});
    GnssVisualizer node(o);using A=VisualizerReleaseAccess;
    A::solution(node,NAN,1000000000);A::solution(node,INFINITY,2000000000);
    EXPECT_EQ(A::count(node),0u);
    A::solution(node,1,1700000000000000000);A::solution(node,2,1000000000);
    EXPECT_EQ(A::count(node),2u);EXPECT_GE(A::span(node),0.);EXPECT_LT(A::span(node),1.);
    auto a=A::snr(node,1),b=A::snr(node,250);
    EXPECT_EQ(cv::norm(a,b,cv::NORM_INF),0.);
    EXPECT_GT(cv::countNonZero(a.reshape(1)),0);
  }
  rclcpp::shutdown();
}
