// SPDX-License-Identifier: MIT
#include <algorithm>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "gnss_ros_standardization/ephemeris_store.hpp"
#include "gnss_ros_standardization/epoch_matcher.hpp"
#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"
#include "gnss_ros_standardization/obs_converter.hpp"
#include "gnss_ros_standardization/tcp_nmea_server.hpp"

extern "C" {
#include "rtklib.h"
}

namespace grs = gnss_ros_standardization::msg;

namespace {
constexpr std::size_t kNmeaBufBytes = 256;
constexpr int kPositionLogThrottleMs = 1000;
}  // namespace

class RtkPositionNode : public rclcpp::Node {
 public:
  RtkPositionNode() : Node("real_time_kinematic") {
    initializeParameters();

    rover_obs_sub_ = create_subscription<grs::GnssObservations>(
        get_parameter("topics.rover_observation").as_string(), rclcpp::QoS(50),
        std::bind(&RtkPositionNode::onRoverObs, this, std::placeholders::_1));

    base_obs_sub_ = create_subscription<grs::GnssObservations>(
        get_parameter("topics.base_observation").as_string(), rclcpp::QoS(50),
        std::bind(&RtkPositionNode::onBaseObs, this, std::placeholders::_1));

    nav_sub_ = create_subscription<grs::GnssEphemerides>(
        get_parameter("topics.ephemeris").as_string(), rclcpp::QoS(1).transient_local(),
        std::bind(&RtkPositionNode::onNav, this, std::placeholders::_1));

    initializeRtk();
    cacheFixedOriginParam();

    gnss_utils::RoverBaseEpochMatcher::Options match_opt;
    match_opt.max_tdiff_s = rtk_.opt.maxtdiff;
    matcher_ = gnss_utils::RoverBaseEpochMatcher(match_opt);

    gnss_sol_pub_ = create_publisher<grs::GnssSolution>(
        get_parameter("topics.solution").as_string(), 10);

    const int tcp_port = get_parameter("tcp_port").as_int();
    tcp_server_.start(tcp_port, get_logger());

    RCLCPP_INFO(get_logger(), "RTK node started, waiting for rover/base data.");
  }

  ~RtkPositionNode() override {
    tcp_server_.stop();
    rtkfree(&rtk_);
    std::lock_guard<std::mutex> lk(nav_mtx_);
    freenav(&nav_, 0xFF);
  }

 private:
  void initializeParameters() {
    auto declare_safe = [this](const std::string& name, const auto& default_val) {
      try {
        declare_parameter(name, default_val);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to declare parameter '%s': %s", name.c_str(), e.what());
        throw;
      }
    };

    declare_safe("topics.rover_observation", std::string("/rover/gnss/observation"));
    declare_safe("topics.base_observation",  std::string("/base/gnss/observation"));
    declare_safe("topics.ephemeris",         std::string("/gnss/ephemeris"));
    declare_safe("topics.solution",          std::string("/gnss/solution"));

    declare_safe("tcp_port", 8000);

    declare_safe("ant2.postype", std::string("rtcm"));
    declare_safe("ant2.pos", std::vector<double>{0.0, 0.0, 0.0});

    declare_safe("fixed_origin.postype", std::string("llh"));
    declare_safe("fixed_origin.pos", std::vector<double>{0.0, 0.0, 0.0});

    declare_safe("pos1.elmask", 15.0);
    declare_safe("pos1.dynamics", 0);
    declare_safe("pos1.ionoopt", IONOOPT_BRDC);
    declare_safe("pos1.tropopt", TROPOPT_SAAS);
    declare_safe("pos1.sateph", EPHOPT_BRDC);

    // Frequency selection: bool flags per band (matches SPP node's style).
    declare_safe("frequencies.enable_l1", true);
    declare_safe("frequencies.enable_l2", true);
    declare_safe("frequencies.enable_l5", false);

    declare_safe("pos1.navsys.gps", true);
    declare_safe("pos1.navsys.glo", true);
    declare_safe("pos1.navsys.gal", true);
    declare_safe("pos1.navsys.bds", true);
    declare_safe("pos1.navsys.qzs", true);
    declare_safe("pos1.navsys.irn", true);

    declare_safe("pos1.snrmask.enable", false);
    declare_safe("pos1.snrmask.l1", std::vector<double>(9, 0.0));
    declare_safe("pos1.snrmask.l2", std::vector<double>(9, 0.0));
    declare_safe("pos1.snrmask.l5", std::vector<double>(9, 0.0));

    declare_safe("excluded_satellites", std::vector<std::string>{});

    declare_safe("pos2.armode", ARMODE_CONT);
    declare_safe("pos2.gloarmode", 1);
    declare_safe("pos2.bdsarmode", 1);
    declare_safe("pos2.arthres", 3.0);
    declare_safe("pos2.arlockcnt", 0);
    declare_safe("pos2.arelmask", 0.0);
    declare_safe("pos2.arminfix", 10);
    declare_safe("pos2.armaxiter", 1);
    declare_safe("pos2.elmaskhold", 0.0);
    declare_safe("pos2.aroutcnt", 5);
    declare_safe("pos2.maxage", 30.0);
    declare_safe("pos2.syncsol", 0);
    declare_safe("pos2.slipthres", 0.05);
    declare_safe("pos2.rejionno", 30.0);
    declare_safe("pos2.niter", 1);

    declare_safe("rtk_stats.eratio1", 100.0);
    declare_safe("rtk_stats.eratio2", 100.0);
    declare_safe("rtk_stats.errphase", 0.003);
    declare_safe("rtk_stats.errphaseel", 0.003);
    declare_safe("rtk_stats.errdoppler", 1.0);
    declare_safe("rtk_stats.stdbias", 30.0);
    declare_safe("rtk_stats.stdiono", 0.03);
    declare_safe("rtk_stats.stdtrop", 0.3);
    declare_safe("rtk_stats.prnaccelh", 10.0);
    declare_safe("rtk_stats.prnaccelv", 10.0);
    declare_safe("rtk_stats.prnbias", 0.0001);
    declare_safe("rtk_stats.prniono", 0.001);
    declare_safe("rtk_stats.prntrop", 0.0001);
    declare_safe("rtk_stats.prnpos", 0.0);
  }

  void initializeRtk() {
    prcopt_t opt = prcopt_default;
    opt.mode = PMODE_KINEMA;
    opt.elmin = get_parameter("pos1.elmask").as_double() * D2R;
    opt.dynamics = get_parameter("pos1.dynamics").as_int();
    opt.ionoopt = get_parameter("pos1.ionoopt").as_int();
    opt.tropopt = get_parameter("pos1.tropopt").as_int();
    opt.sateph = get_parameter("pos1.sateph").as_int();

    freq_mask_.l1 = get_parameter("frequencies.enable_l1").as_bool();
    freq_mask_.l2 = get_parameter("frequencies.enable_l2").as_bool();
    freq_mask_.l5 = get_parameter("frequencies.enable_l5").as_bool();
    if (freq_mask_.l5) opt.nf = 3;
    else if (freq_mask_.l2) opt.nf = 2;
    else opt.nf = 1;
    RCLCPP_INFO(get_logger(), "Frequency config: L1=%d L2=%d L5=%d -> nf=%d",
                freq_mask_.l1, freq_mask_.l2, freq_mask_.l5, opt.nf);

    opt.navsys = 0;
    if (get_parameter("pos1.navsys.gps").as_bool()) opt.navsys |= SYS_GPS;
    if (get_parameter("pos1.navsys.glo").as_bool()) opt.navsys |= SYS_GLO;
    if (get_parameter("pos1.navsys.gal").as_bool()) opt.navsys |= SYS_GAL;
    if (get_parameter("pos1.navsys.bds").as_bool()) opt.navsys |= SYS_CMP;
    if (get_parameter("pos1.navsys.qzs").as_bool()) opt.navsys |= SYS_QZS;
    if (get_parameter("pos1.navsys.irn").as_bool()) opt.navsys |= SYS_IRN;

    if (get_parameter("pos1.snrmask.enable").as_bool()) {
      opt.snrmask.ena[0] = 1;
      opt.snrmask.ena[1] = 1;
      const auto l1 = get_parameter("pos1.snrmask.l1").as_double_array();
      const auto l2 = get_parameter("pos1.snrmask.l2").as_double_array();
      const auto l5 = get_parameter("pos1.snrmask.l5").as_double_array();
      for (int i = 0; i < 9; ++i) {
        if (i < static_cast<int>(l1.size())) opt.snrmask.mask[0][i] = l1[i];
        if (i < static_cast<int>(l2.size())) opt.snrmask.mask[1][i] = l2[i];
        if (i < static_cast<int>(l5.size())) opt.snrmask.mask[2][i] = l5[i];
      }
    }

    const auto excluded = get_parameter("excluded_satellites").as_string_array();
    for (const auto& satid : excluded) {
      const int sat = satid2no(satid.c_str());
      if (sat > 0 && sat <= MAXSAT) opt.exsats[sat - 1] = 1;
    }

    opt.modear     = get_parameter("pos2.armode").as_int();
    opt.glomodear  = get_parameter("pos2.gloarmode").as_int();
    opt.bdsmodear  = get_parameter("pos2.bdsarmode").as_int();
    opt.thresar[0] = get_parameter("pos2.arthres").as_double();
    opt.minlock    = get_parameter("pos2.arlockcnt").as_int();
    opt.elmaskar   = get_parameter("pos2.arelmask").as_double() * D2R;
    opt.minfix     = get_parameter("pos2.arminfix").as_int();
    opt.armaxiter  = get_parameter("pos2.armaxiter").as_int();
    opt.elmaskhold = get_parameter("pos2.elmaskhold").as_double() * D2R;
    opt.maxout     = get_parameter("pos2.aroutcnt").as_int();
    opt.maxtdiff   = get_parameter("pos2.maxage").as_double();
    opt.syncsol    = get_parameter("pos2.syncsol").as_int();
    opt.thresslip  = get_parameter("pos2.slipthres").as_double();
    // rtklibexplorer: maxinno is now an array [phase, code]; maxgdop was removed.
    {
      const double rejionno = get_parameter("pos2.rejionno").as_double();
      opt.maxinno[0] = rejionno;
      opt.maxinno[1] = rejionno;
    }
    opt.niter = get_parameter("pos2.niter").as_int();

    opt.eratio[0] = get_parameter("rtk_stats.eratio1").as_double();
    opt.eratio[1] = get_parameter("rtk_stats.eratio2").as_double();
    opt.err[1] = get_parameter("rtk_stats.errphase").as_double();
    opt.err[2] = get_parameter("rtk_stats.errphaseel").as_double();
    opt.err[4] = get_parameter("rtk_stats.errdoppler").as_double();
    opt.std[0] = get_parameter("rtk_stats.stdbias").as_double();
    opt.std[1] = get_parameter("rtk_stats.stdiono").as_double();
    opt.std[2] = get_parameter("rtk_stats.stdtrop").as_double();
    opt.prn[3] = get_parameter("rtk_stats.prnaccelh").as_double();
    opt.prn[4] = get_parameter("rtk_stats.prnaccelv").as_double();
    opt.prn[0] = get_parameter("rtk_stats.prnbias").as_double();
    opt.prn[1] = get_parameter("rtk_stats.prniono").as_double();
    opt.prn[2] = get_parameter("rtk_stats.prntrop").as_double();
    opt.prn[5] = get_parameter("rtk_stats.prnpos").as_double();

    const std::string postype = get_parameter("ant2.postype").as_string();
    const auto pos_vec = get_parameter("ant2.pos").as_double_array();
    if (postype == "ecef") {
      opt.refpos = 0;
      if (pos_vec.size() == 3) std::copy(pos_vec.begin(), pos_vec.end(), opt.rb);
    } else if (postype == "llh") {
      opt.refpos = 0;
      if (pos_vec.size() == 3) {
        double llh[3] = {pos_vec[0] * D2R, pos_vec[1] * D2R, pos_vec[2]};
        pos2ecef(llh, opt.rb);
      }
    } else if (postype == "rtcm") {
      opt.refpos = 4;
    }

    rtkinit(&rtk_, &opt);
    if (opt.refpos == 0) {
      for (int i = 0; i < 3; ++i) rtk_.rb[i] = opt.rb[i];
      RCLCPP_INFO(get_logger(), "RTK init: forced base position to %.3f %.3f %.3f",
                  rtk_.rb[0], rtk_.rb[1], rtk_.rb[2]);
    }

    std::memset(&nav_, 0, sizeof(nav_));
  }

  // ENU origin priority: fixed_origin (configured) > base station (rtk_.rb).
  // Resolved once at startup so per-epoch publishing does not re-read params.
  void cacheFixedOriginParam() {
    const std::string type = get_parameter("fixed_origin.postype").as_string();
    const auto pos = get_parameter("fixed_origin.pos").as_double_array();
    if (pos.size() == 3 && norm(pos.data(), 3) > 0.0) {
      if (type == "llh") {
        double llh[3] = {pos[0] * D2R, pos[1] * D2R, pos[2]};
        pos2ecef(llh, fixed_origin_ecef_);
      } else {
        fixed_origin_ecef_[0] = pos[0];
        fixed_origin_ecef_[1] = pos[1];
        fixed_origin_ecef_[2] = pos[2];
      }
      fixed_origin_valid_ = true;
      RCLCPP_INFO(get_logger(),
                  "ENU origin (config): ECEF %.3f %.3f %.3f",
                  fixed_origin_ecef_[0], fixed_origin_ecef_[1], fixed_origin_ecef_[2]);
    }
  }

  void onNav(const grs::GnssEphemerides::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(nav_mtx_);
    for (const auto& e : msg->gnss_ephemeris) {
      if (!e.satid.empty()) eph_store_.ingestEph(gnss_utils::msgToEph(e));
    }
    for (const auto& g : msg->glonass_ephemeris) {
      if (!g.satid.empty()) eph_store_.ingestGeph(gnss_utils::msgToGeph(g));
    }
    eph_store_.applyToNav(nav_);
  }

  void onBaseObs(const grs::GnssObservations::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(obs_mtx_);
    matcher_.pushBase(msg);
    processMatches();
  }

  void onRoverObs(const grs::GnssObservations::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(obs_mtx_);
    matcher_.pushRover(msg);
    processMatches();
  }

  void processMatches() {
    for (const auto& pair : matcher_.drainMatches()) {
      processRtk(pair.rover, pair.base);
    }
    for (const auto& d : matcher_.takeDropped()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Dropped rover obs (tow %.3f) - no base within maxtdiff (newest base %.3f)",
          d.rover_tow, d.newest_base_tow);
    }
  }

  void processRtk(const grs::GnssObservations::ConstSharedPtr& rover,
                  const grs::GnssObservations::ConstSharedPtr& base) {
    std::lock_guard<std::mutex> lk_nav(nav_mtx_);
    if (nav_.n == 0 && nav_.ng == 0) return;

    std::vector<obsd_t> obs = gnss_utils::buildObsEpoch(*rover, base.get(), freq_mask_);
    if (obs.empty()) return;

    const int stat = rtkpos(&rtk_, obs.data(), static_cast<int>(obs.size()), &nav_);
    if (stat <= 0) return;

    double llh[3];
    ecef2pos(rtk_.sol.rr, llh);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), kPositionLogThrottleMs,
        "RTK %s | LLH: %.8f %.8f %.3f | Ratio: %.1f | Sats: %d | Age: %.1f",
        gnss_utils::solqToString(rtk_.sol.stat).c_str(),
        llh[0] * R2D, llh[1] * R2D, llh[2],
        rtk_.sol.ratio, rtk_.sol.ns, rtk_.sol.age);

    publishSolution(llh, rover->header.stamp);
  }

  // Stamp the solution with the measurement epoch (the rover observation's
  // header.stamp), NOT this->now(): the latter is the processing wall-clock time,
  // which under rosbag replay (without --clock) differs from the recorded IMU /
  // observation timestamps by the replay-vs-recording offset. A wrong stamp breaks
  // downstream IMU/GNSS time alignment (e.g. the EKF predictToTime gap guard).
  void publishSolution(const double llh[3], const builtin_interfaces::msg::Time& stamp) {
    auto sol_msg = std::make_unique<grs::GnssSolution>();
    sol_msg->header.stamp = stamp;
    sol_msg->header.frame_id = "gnss_link";

    int week = 0;
    sol_msg->time_tow = time2gpst(rtk_.sol.time, &week);
    sol_msg->time_week = static_cast<uint32_t>(week);
    sol_msg->solution_source = grs::GnssSolution::SOLUTION_SOURCE_COMPUTED;

    sol_msg->status = rtk_.sol.stat;
    sol_msg->num_sats = rtk_.sol.ns;
    sol_msg->ratio = rtk_.sol.ratio;
    sol_msg->age_diff = rtk_.sol.age;

    const auto dops = gnss_utils::calculateDops(rtk_.ssat, MAXSAT, rtk_.opt.elmin);
    sol_msg->gdop = dops.gdop;
    sol_msg->pdop = dops.pdop;
    sol_msg->hdop = dops.hdop;
    sol_msg->vdop = dops.vdop;

    sol_msg->latitude  = llh[0] * R2D;
    sol_msg->longitude = llh[1] * R2D;
    sol_msg->altitude  = llh[2];

    sol_msg->pos_ecef.x = rtk_.sol.rr[0];
    sol_msg->pos_ecef.y = rtk_.sol.rr[1];
    sol_msg->pos_ecef.z = rtk_.sol.rr[2];

    // sol.qr layout: {xx, yy, zz, xy, yz, zx}
    sol_msg->pos_cov_ecef[0] = rtk_.sol.qr[0];
    sol_msg->pos_cov_ecef[4] = rtk_.sol.qr[1];
    sol_msg->pos_cov_ecef[8] = rtk_.sol.qr[2];
    sol_msg->pos_cov_ecef[1] = rtk_.sol.qr[3];
    sol_msg->pos_cov_ecef[3] = rtk_.sol.qr[3];
    sol_msg->pos_cov_ecef[5] = rtk_.sol.qr[4];
    sol_msg->pos_cov_ecef[7] = rtk_.sol.qr[4];
    sol_msg->pos_cov_ecef[2] = rtk_.sol.qr[5];
    sol_msg->pos_cov_ecef[6] = rtk_.sol.qr[5];

    sol_msg->vel_ecef.x = rtk_.sol.rr[3];
    sol_msg->vel_ecef.y = rtk_.sol.rr[4];
    sol_msg->vel_ecef.z = rtk_.sol.rr[5];
    sol_msg->vel_cov_ecef[0] = rtk_.sol.qv[0];
    sol_msg->vel_cov_ecef[4] = rtk_.sol.qv[1];
    sol_msg->vel_cov_ecef[8] = rtk_.sol.qv[2];
    sol_msg->vel_cov_ecef[1] = rtk_.sol.qv[3];
    sol_msg->vel_cov_ecef[3] = rtk_.sol.qv[3];
    sol_msg->vel_cov_ecef[5] = rtk_.sol.qv[4];
    sol_msg->vel_cov_ecef[7] = rtk_.sol.qv[4];
    sol_msg->vel_cov_ecef[2] = rtk_.sol.qv[5];
    sol_msg->vel_cov_ecef[6] = rtk_.sol.qv[5];

    // ENU origin: fixed_origin (cached) > base station (rtk_.rb) > zero.
    double origin_ecef[3] = {0.0, 0.0, 0.0};
    if (fixed_origin_valid_) {
      std::copy_n(fixed_origin_ecef_, 3, origin_ecef);
    } else if (norm(rtk_.rb, 3) > 0.0) {
      std::copy_n(rtk_.rb, 3, origin_ecef);
    }

    sol_msg->pos_enu_org_ecef.x = origin_ecef[0];
    sol_msg->pos_enu_org_ecef.y = origin_ecef[1];
    sol_msg->pos_enu_org_ecef.z = origin_ecef[2];

    if (norm(origin_ecef, 3) > 0.0) {
      double origin_llh[3];
      ecef2pos(origin_ecef, origin_llh);
      const double d_ecef[3] = {
          rtk_.sol.rr[0] - origin_ecef[0],
          rtk_.sol.rr[1] - origin_ecef[1],
          rtk_.sol.rr[2] - origin_ecef[2],
      };
      double pos_enu[3];
      ecef2enu(origin_llh, d_ecef, pos_enu);
      sol_msg->pos_enu.x = pos_enu[0];
      sol_msg->pos_enu.y = pos_enu[1];
      sol_msg->pos_enu.z = pos_enu[2];
    } else {
      // No origin configured and base station position not yet received.
      // Fall back to (0,0,0) — pos_enu stays at rest until origin resolves.
      sol_msg->pos_enu.x = 0.0;
      sol_msg->pos_enu.y = 0.0;
      sol_msg->pos_enu.z = 0.0;
    }

    double Q_ecef[9] = {
        rtk_.sol.qr[0], rtk_.sol.qr[3], rtk_.sol.qr[5],
        rtk_.sol.qr[3], rtk_.sol.qr[1], rtk_.sol.qr[4],
        rtk_.sol.qr[5], rtk_.sol.qr[4], rtk_.sol.qr[2],
    };
    double Q_enu[9];
    gnss_utils::rotateCovariance(Q_ecef, llh[0], llh[1], Q_enu);
    for (int i = 0; i < 9; ++i) sol_msg->pos_enu_cov[i] = Q_enu[i];

    double vel_ecef[3] = {rtk_.sol.rr[3], rtk_.sol.rr[4], rtk_.sol.rr[5]};
    double vel_enu[3];
    ecef2enu(llh, vel_ecef, vel_enu);
    sol_msg->vel_enu.x = vel_enu[0];
    sol_msg->vel_enu.y = vel_enu[1];
    sol_msg->vel_enu.z = vel_enu[2];

    double Qv_ecef[9] = {
        rtk_.sol.qv[0], rtk_.sol.qv[3], rtk_.sol.qv[5],
        rtk_.sol.qv[3], rtk_.sol.qv[1], rtk_.sol.qv[4],
        rtk_.sol.qv[5], rtk_.sol.qv[4], rtk_.sol.qv[2],
    };
    double Qv_enu[9];
    gnss_utils::rotateCovariance(Qv_ecef, llh[0], llh[1], Qv_enu);
    for (int i = 0; i < 9; ++i) sol_msg->vel_enu_cov[i] = Qv_enu[i];

    gnss_sol_pub_->publish(std::move(sol_msg));

    if (tcp_server_.hasClients()) {
      unsigned char gga[kNmeaBufBytes];
      int n = outnmea_gga(gga, &rtk_.sol);
      if (n > 0) tcp_server_.send(reinterpret_cast<const char*>(gga), n);
      unsigned char rmc[kNmeaBufBytes];
      int m = outnmea_rmc(rmc, &rtk_.sol);
      if (m > 0) tcp_server_.send(reinterpret_cast<const char*>(rmc), m);
    }
  }

  rtk_t rtk_{};
  nav_t nav_{};
  std::mutex nav_mtx_;
  std::mutex obs_mtx_;
  gnss_utils::EphemerisStore eph_store_;

  gnss_utils::RoverBaseEpochMatcher matcher_;

  rclcpp::Subscription<grs::GnssObservations>::SharedPtr rover_obs_sub_;
  rclcpp::Subscription<grs::GnssObservations>::SharedPtr base_obs_sub_;
  rclcpp::Subscription<grs::GnssEphemerides>::SharedPtr nav_sub_;
  rclcpp::Publisher<grs::GnssSolution>::SharedPtr gnss_sol_pub_;

  gnss_utils::TcpNmeaServer tcp_server_;

  gnss_utils::FrequencyMask freq_mask_;

  double fixed_origin_ecef_[3]{0.0, 0.0, 0.0};
  bool fixed_origin_valid_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RtkPositionNode>());
  rclcpp::shutdown();
  return 0;
}
