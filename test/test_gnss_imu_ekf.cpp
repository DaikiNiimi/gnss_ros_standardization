#include "gnss_ros_standardization/gnss_imu_kalman_filter.hpp"
#include <gtest/gtest.h>
namespace gnss_imu_kalman_filter {
// Exercise the real node's input callback/state without depending on DDS
// timing.
struct EkfRegressionAccess {
  static void imu(GnssImuKalmanFilter &node, double t, bool valid = true) {
    auto m = std::make_shared<sensor_msgs::msg::Imu>();
    m->header.stamp = rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME);
    m->linear_acceleration.z = 9.80665;
    if (!valid)
      m->angular_velocity_covariance[0] = -1;
    node.onImu(m);
  }
  static int initialCount(const GnssImuKalmanFilter &n) {
    return n.init_imu_acc_count_;
  }
  static size_t buffered(const GnssImuKalmanFilter &n) {
    return n.imu_buffer_.size();
  }
  // A FIX at 35N 139E for GPST (week, tow), stamped with `header_s`.
  static void gnss(GnssImuKalmanFilter &node, double tow, double header_s) {
    auto m = std::make_shared<gnss_ros_standardization::msg::GnssSolution>();
    m->header.stamp = rclcpp::Time(std::llround(header_s * 1e9), RCL_ROS_TIME);
    m->time_week = 2300; m->time_tow = tow;
    m->status = gnss_ros_standardization::msg::GnssSolution::STATUS_FIX;
    const double llh[3] = {35 * D2R, 139 * D2R, 10};
    double xyz[3]; pos2ecef(llh, xyz);
    m->pos_ecef.x = xyz[0]; m->pos_ecef.y = xyz[1]; m->pos_ecef.z = xyz[2];
    m->latitude = 35; m->longitude = 139; m->altitude = 10;
    for (int i : {0, 4, 8}) { m->pos_cov_ecef[i] = 1e-4; m->vel_cov_ecef[i] = 1e-4; }
    m->vel_ecef.x = m->vel_ecef.y = m->vel_ecef.z = 0;
    node.onGnss(m);
  }
  static bool initialized(const GnssImuKalmanFilter &n) { return n.initialized_; }
  static uint8_t status(const GnssImuKalmanFilter &n, double t) {
    return n.outputStatus(rclcpp::Time(std::llround(t * 1e9), RCL_ROS_TIME));
  }
  static double tow(const GnssImuKalmanFilter &n, double t) {
    uint32_t week = 0; double tow = 0;
    EXPECT_TRUE(n.headerTimeToGpst(rclcpp::Time(std::llround(t * 1e9), RCL_ROS_TIME), week, tow));
    return tow;
  }
  static bool useInitYaw(const GnssImuKalmanFilter &n) { return n.config_.use_init_yaw; }
  // Moves the state away from the ENU origin and returns the output message.
  static gnss_ros_standardization::msg::GnssSolution solutionAt(GnssImuKalmanFilter &n, double t,
      const Eigen::Vector3d &pos_enu, const Eigen::Vector3d &vel_enu,
      const Eigen::Matrix3d &P_pos, const Eigen::Matrix3d &P_vel) {
    n.x_.segment<3>(IDX_POS) = pos_enu; n.x_.segment<3>(IDX_VEL) = vel_enu;
    n.P_.block<3,3>(EIDX_POS, EIDX_POS) = P_pos; n.P_.block<3,3>(EIDX_VEL, EIDX_VEL) = P_vel;
    return n.buildSolution(rclcpp::Time(std::llround(t * 1e9), RCL_ROS_TIME));
  }
};
} // namespace gnss_imu_kalman_filter
TEST(EkfInputRegression, MissingMeasurementRejectedAndReplayReinitializes) {
  rclcpp::init(0, nullptr);
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("init.imu_duration_s", 0.1)});
    gnss_imu_kalman_filter::GnssImuKalmanFilter node(options);
    using A = gnss_imu_kalman_filter::EkfRegressionAccess;
    A::imu(node, 100, false);
    EXPECT_EQ(A::initialCount(node), 0);
    A::imu(node, 100);
    A::imu(node, 100.2);
    A::imu(node, 100.3);
    EXPECT_GT(A::buffered(node), 0u);
    A::imu(node, 10);
    EXPECT_EQ(A::buffered(node), 0u);
    EXPECT_EQ(A::initialCount(node), 1);
    A::imu(node, 10.2);
    A::imu(node, 10.3);
    EXPECT_GT(A::buffered(node), 0u);
    auto count = A::buffered(node);
    A::imu(node, 10.29);
    EXPECT_EQ(A::buffered(node), count);
  }
  rclcpp::shutdown();
}

namespace {
// GPST seconds (RTKLIB time_t reading) of week 2300 + tow, and its UTC reading.
double gpstSeconds(double tow) { const gtime_t t = gpst2time(2300, tow); return t.time + t.sec; }
double utcSeconds(double tow) { const gtime_t t = gpst2utc(gpst2time(2300, tow)); return t.time + t.sec; }

// Runs 100 Hz IMU from tow 100 and 1 Hz FIX solutions until `gnss_end`, IMU to
// `imu_end`; header stamps in UTC or (legacy bags) GPST.
void drive(gnss_imu_kalman_filter::GnssImuKalmanFilter &node, bool gpst_headers,
           double gnss_end, double imu_end) {
  using A = gnss_imu_kalman_filter::EkfRegressionAccess;
  auto header = [&](double tow) { return gpst_headers ? gpstSeconds(tow) : utcSeconds(tow); };
  for (int i = 0; 100. + i * .01 <= imu_end + 1e-9; ++i) {
    const double tow = 100. + i * .01;
    A::imu(node, header(tow));
    if (i % 100 == 0 && tow <= gnss_end + 1e-9) A::gnss(node, tow, header(tow) + .05);
  }
}
rclcpp::NodeOptions ekfOptions() {
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("init.imu_duration_s", 0.1),
                               rclcpp::Parameter("init.use_yaw", true)});
  return options;
}
}  // namespace

// Q-A15: the EKF output carries the status of the GNSS solution behind its last
// update, and STATUS_DR once the next update is overdue (GNSS outage).
TEST(EkfOutputStatus, FollowsTheLastGnssUpdateThenDeadReckons) {
  rclcpp::init(0, nullptr);
  {
    gnss_imu_kalman_filter::GnssImuKalmanFilter node(ekfOptions());
    using A = gnss_imu_kalman_filter::EkfRegressionAccess;
    using S = gnss_ros_standardization::msg::GnssSolution;
    drive(node, false, 115., 120.);
    ASSERT_TRUE(A::initialized(node));
    EXPECT_EQ(A::status(node, utcSeconds(115.9)), S::STATUS_FIX);
    EXPECT_EQ(A::status(node, utcSeconds(117.)), S::STATUS_DR);
  }
  rclcpp::shutdown();
}

// A-23: with legacy GPST header stamps the output week/tow must not be shifted
// by the leap seconds; with UTC stamps they are read as UTC.
TEST(EkfOutputTime, HeaderBasisIsReadOffTheGnssInput) {
  rclcpp::init(0, nullptr);
  for (bool gpst_headers : {false, true}) {
    gnss_imu_kalman_filter::GnssImuKalmanFilter node(ekfOptions());
    using A = gnss_imu_kalman_filter::EkfRegressionAccess;
    drive(node, gpst_headers, 105., 105.);
    const double header = gpst_headers ? gpstSeconds(104.5) : utcSeconds(104.5);
    EXPECT_NEAR(A::tow(node, header), 104.5, 1e-6) << "gpst headers " << gpst_headers;
  }
  rclcpp::shutdown();
}

// A-19: the code default matches the shipped YAML and README (false: wait for
// a Doppler heading instead of assuming a configured yaw).
TEST(EkfDefaults, InitialYawIsNotAssumedByDefault) {
  rclcpp::init(0, nullptr);
  {
    gnss_imu_kalman_filter::GnssImuKalmanFilter node{rclcpp::NodeOptions()};
    EXPECT_FALSE(gnss_imu_kalman_filter::EkfRegressionAccess::useInitYaw(node));
  }
  rclcpp::shutdown();
}

// B7: GnssSolution.msg defines pos_enu in the origin's tangent plane but
// pos_enu_cov, vel_enu and vel_enu_cov in the tangent plane at the CURRENT
// position (as every other node). The EKF state is origin-ENU.
TEST(EkfOutputFrames, LocalVelocityAndCovarianceUseTheCurrentTangentPlane) {
  rclcpp::init(0, nullptr);
  {
    gnss_imu_kalman_filter::GnssImuKalmanFilter node(ekfOptions());
    using A = gnss_imu_kalman_filter::EkfRegressionAccess;
    drive(node, false, 105., 105.);
    ASSERT_TRUE(A::initialized(node));
    Eigen::Matrix3d Pp, Pv;
    Pp << 4, 1, .5, 1, 2, .3, .5, .3, 9;
    Pv << .04, .01, 0, .01, .02, .003, 0, .003, .09;
    const auto m = A::solutionAt(node, utcSeconds(104.5), {20000, 15000, 30}, {10, 5, .5}, Pp, Pv);
    const double cur[3] = {m.latitude * D2R, m.longitude * D2R, m.altitude};
    const double v_ecef[3] = {m.vel_ecef.x, m.vel_ecef.y, m.vel_ecef.z};
    double v_cur[3]; ecef2enu(cur, v_ecef, v_cur);
    EXPECT_NEAR(m.vel_enu.x, v_cur[0], 1e-9); EXPECT_NEAR(m.vel_enu.y, v_cur[1], 1e-9);
    EXPECT_NEAR(m.vel_enu.z, v_cur[2], 1e-9);
    EXPECT_NEAR(std::hypot(std::hypot(m.vel_enu.x, m.vel_enu.y), m.vel_enu.z),
                std::hypot(std::hypot(10., 5.), .5), 1e-9);  // a rotation of the state velocity
    double pcur[9], vcur[9];
    gnss_utils::rotateCovariance(m.pos_cov_ecef.data(), cur[0], cur[1], pcur);
    gnss_utils::rotateCovariance(m.vel_cov_ecef.data(), cur[0], cur[1], vcur);
    for (int i = 0; i < 9; ++i) {
      EXPECT_NEAR(m.pos_enu_cov[i], pcur[i], 1e-9) << i;
      EXPECT_NEAR(m.vel_enu_cov[i], vcur[i], 1e-12) << i;
    }
    // 20 km from the origin the two tangent planes differ by ~3 mrad: the
    // origin-frame values must not pass.
    EXPECT_GT(std::abs(m.vel_enu.z - .5), 1e-3);
  }
  rclcpp::shutdown();
}
