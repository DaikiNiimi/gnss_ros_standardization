// SPDX-License-Identifier: MIT
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "gnss_ros_standardization/ephemeris_store.hpp"
#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/msg/gnss_ephemerides.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"
#include "gnss_ros_standardization/tcp_nmea_server.hpp"

extern "C" {
#include "rtklib.h"
}

namespace grs = gnss_ros_standardization::msg;

namespace {
constexpr std::size_t kNmeaBufBytes = 256;
constexpr std::size_t kRtkMsgBufBytes = 1024;
constexpr int kPositionLogThrottleMs = 1000;
}  // namespace

class SppPntposNode : public rclcpp::Node {
 public:
  SppPntposNode() : Node("single_point_positioning") {
    initializeParameters();

    obs_sub_ = create_subscription<grs::GnssObservations>(
        get_parameter("topics.observation").as_string(), rclcpp::QoS(50),
        std::bind(&SppPntposNode::onObs, this, std::placeholders::_1));

    nav_sub_ = create_subscription<grs::GnssEphemerides>(
        get_parameter("topics.ephemeris").as_string(), rclcpp::QoS(1).transient_local(),
        std::bind(&SppPntposNode::onNav, this, std::placeholders::_1));

    gnss_sol_pub_ = create_publisher<grs::GnssSolution>(
        get_parameter("topics.solution").as_string(), 10);

    initializeRtklibOptions();

    cacheFixedOriginParam();

    std::memset(&nav_, 0, sizeof(nav_));

    const int tcp_port = get_parameter("tcp_port").as_int();
    tcp_server_.start(tcp_port, get_logger());

    RCLCPP_INFO(get_logger(), "SPP node started, waiting for GNSS data.");
  }

  ~SppPntposNode() override {
    tcp_server_.stop();
    std::lock_guard<std::mutex> lk(nav_mtx_);
    freenav(&nav_, 0xFF);
  }

 private:
  void initializeParameters() {
    declare_parameter<int>("tcp_port", 8000);
    declare_parameter<std::string>("topics.observation", "/gnss/observation");
    declare_parameter<std::string>("topics.ephemeris", "/gnss/ephemeris");
    declare_parameter<std::string>("topics.solution", "/gnss/solution");

    declare_parameter<double>("elevation_mask_deg", 15.0);

    declare_parameter<bool>("snrmask.enable", false);
    declare_parameter<std::vector<double>>("snrmask.l1", std::vector<double>(9, 0.0));
    declare_parameter<std::vector<double>>("snrmask.l2", std::vector<double>(9, 0.0));
    declare_parameter<std::vector<double>>("snrmask.l5", std::vector<double>(9, 0.0));

    declare_parameter<bool>("nav_systems.gps", true);
    declare_parameter<bool>("nav_systems.glo", true);
    declare_parameter<bool>("nav_systems.gal", true);
    declare_parameter<bool>("nav_systems.bds", true);
    declare_parameter<bool>("nav_systems.qzs", true);
    declare_parameter<bool>("nav_systems.irn", true);
    declare_parameter<bool>("nav_systems.sbs", false);

    declare_parameter<bool>("frequencies.enable_l1", true);
    declare_parameter<bool>("frequencies.enable_l2", true);
    declare_parameter<bool>("frequencies.enable_l5", true);

    declare_parameter<std::vector<std::string>>("excluded_satellites", std::vector<std::string>{});

    declare_parameter<std::string>("fixed_origin.postype", "llh");
    declare_parameter<std::vector<double>>("fixed_origin.pos", std::vector<double>{0.0, 0.0, 0.0});
  }

  void initializeRtklibOptions() {
    opt_ = prcopt_default;
    opt_.mode = PMODE_SINGLE;
    opt_.ionoopt = IONOOPT_BRDC;
    opt_.tropopt = TROPOPT_SAAS;
    opt_.sateph = EPHOPT_BRDC;
    opt_.elmin = get_parameter("elevation_mask_deg").as_double() * D2R;

    opt_.navsys = 0;
    if (get_parameter("nav_systems.gps").as_bool()) opt_.navsys |= SYS_GPS;
    if (get_parameter("nav_systems.glo").as_bool()) opt_.navsys |= SYS_GLO;
    if (get_parameter("nav_systems.gal").as_bool()) opt_.navsys |= SYS_GAL;
    if (get_parameter("nav_systems.bds").as_bool()) opt_.navsys |= SYS_CMP;
    if (get_parameter("nav_systems.qzs").as_bool()) opt_.navsys |= SYS_QZS;
    if (get_parameter("nav_systems.irn").as_bool()) opt_.navsys |= SYS_IRN;
    if (get_parameter("nav_systems.sbs").as_bool()) opt_.navsys |= SYS_SBS;

    if (get_parameter("snrmask.enable").as_bool()) {
      opt_.snrmask.ena[0] = 1;  // rover
      const auto l1 = get_parameter("snrmask.l1").as_double_array();
      const auto l2 = get_parameter("snrmask.l2").as_double_array();
      const auto l5 = get_parameter("snrmask.l5").as_double_array();
      for (int i = 0; i < 9; ++i) {
        if (i < static_cast<int>(l1.size())) opt_.snrmask.mask[0][i] = l1[i];
        if (i < static_cast<int>(l2.size())) opt_.snrmask.mask[1][i] = l2[i];
        if (i < static_cast<int>(l5.size())) opt_.snrmask.mask[2][i] = l5[i];
      }
    }

    enable_l1_ = get_parameter("frequencies.enable_l1").as_bool();
    enable_l2_ = get_parameter("frequencies.enable_l2").as_bool();
    enable_l5_ = get_parameter("frequencies.enable_l5").as_bool();

    // nf must cover the highest enabled band; RTKLIB iterates idx in [0, nf).
    if (enable_l5_) opt_.nf = 3;
    else if (enable_l2_) opt_.nf = 2;
    else opt_.nf = 1;

    const auto excluded = get_parameter("excluded_satellites").as_string_array();
    for (const auto& satid : excluded) {
      const int sat = satid2no(satid.c_str());
      if (sat > 0 && sat <= MAXSAT) opt_.exsats[sat - 1] = 1;
    }

    RCLCPP_INFO(get_logger(), "Frequency config: L1=%d L2=%d L5=%d -> nf=%d",
                enable_l1_, enable_l2_, enable_l5_, opt_.nf);
  }

  // Resolve fixed_origin.* from parameters once at startup. ENU origin is
  // either (a) the configured fixed point, or (b) the first successful fix.
  void cacheFixedOriginParam() {
    const std::string type = get_parameter("fixed_origin.postype").as_string();
    const auto pos = get_parameter("fixed_origin.pos").as_double_array();
    if (pos.size() != 3 || norm(pos.data(), 3) <= 0.0) {
      fixed_origin_valid_ = false;
      return;
    }
    if (type == "llh") {
      double llh[3] = {pos[0] * D2R, pos[1] * D2R, pos[2]};
      pos2ecef(llh, fixed_origin_ecef_);
      RCLCPP_INFO(get_logger(),
                  "ENU origin (config, LLH): %.8f %.8f %.3f -> ECEF %.3f %.3f %.3f",
                  pos[0], pos[1], pos[2],
                  fixed_origin_ecef_[0], fixed_origin_ecef_[1], fixed_origin_ecef_[2]);
    } else {
      fixed_origin_ecef_[0] = pos[0];
      fixed_origin_ecef_[1] = pos[1];
      fixed_origin_ecef_[2] = pos[2];
      RCLCPP_INFO(get_logger(),
                  "ENU origin (config, ECEF): %.3f %.3f %.3f",
                  fixed_origin_ecef_[0], fixed_origin_ecef_[1], fixed_origin_ecef_[2]);
    }
    fixed_origin_valid_ = true;
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
    RCLCPP_INFO_ONCE(get_logger(), "Ephemeris received. Ready for positioning.");
  }

  // Guard against frequency collisions in code2idx(). The only true collision
  // is BDS idx=0: B1I (1561 MHz) and B1C (1575 MHz) both map to idx=0. All
  // other systems share a real physical frequency per idx, so accept any code.
  static bool isPrimaryCode(int sys, uint8_t code, int idx) {
    if (sys == SYS_CMP && idx == 0 && code != CODE_L2I) return false;
    return true;
  }

  void onObs(const grs::GnssObservations::SharedPtr in) {
    const gtime_t t = gpst2time(static_cast<int>(in->week), in->tow);

    struct Acc {
      obsd_t o{};
      bool inited{false};
    };
    std::unordered_map<int, Acc> satmap;

    for (const auto& m : in->observations) {
      const int sat = satid2no(m.satid.c_str());
      if (sat <= 0 || sat > MAXSAT) continue;
      const uint8_t code = m.code;
      if (code == CODE_NONE) continue;
      int prn = 0;
      const int sys = satsys(sat, &prn);
      const int idx = code2idx(sys, code);
      if (idx < 0) continue;
      if (!isPrimaryCode(sys, code, idx)) continue;

      // Per-band enable filter. RTKLIB iterates idx in [0, opt_.nf), so a band
      // with no populated P/L is treated as missing — but skipping ingestion
      // is still cheaper and prevents accidental use of disabled bands.
      if (idx == 0 && !enable_l1_) continue;
      if (idx == 1 && !enable_l2_) continue;
      if (idx == 2 && !enable_l5_) continue;
      if (idx > 2) continue;

      auto& acc = satmap[sat];
      if (!acc.inited) {
        acc.o = {};
        acc.o.time = t;
        acc.o.sat = static_cast<uint8_t>(sat);
        acc.o.rcv = 1;
        acc.inited = true;
      }
      acc.o.code[idx] = code;
      if (m.p > 0.0) acc.o.P[idx] = m.p;
      if (m.l != 0.0) acc.o.L[idx] = m.l;
      if (m.d != 0.0) acc.o.D[idx] = static_cast<float>(m.d);
      if (m.snr > 0.0) acc.o.SNR[idx] = static_cast<float>(m.snr);
      acc.o.LLI[idx] = static_cast<uint8_t>(m.lli);
    }

    std::lock_guard<std::mutex> lk(nav_mtx_);
    if (nav_.n == 0 && nav_.ng == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No ephemeris data yet.");
      return;
    }

    std::vector<obsd_t> obs;
    obs.reserve(satmap.size());
    for (auto& kv : satmap) obs.push_back(std::move(kv.second.o));

    if (obs.size() < 4) {
      RCLCPP_WARN(get_logger(),
                  "Not enough satellites observed. (Found %zu, need >= 4)", obs.size());
      return;
    }
    std::sort(obs.begin(), obs.end(),
              [](const obsd_t& a, const obsd_t& b) { return a.sat < b.sat; });

    sol_t sol{};
    ssat_t ssat[MAXSAT]{};
    double azel[MAXSAT * 2]{};
    char msg_buffer[kRtkMsgBufBytes] = "";
    // pntpos() returns 1 on success, 0 on failure (msg_buffer carries the cause).
    const int stat = pntpos(obs.data(), static_cast<int>(obs.size()),
                            &nav_, &opt_, &sol, azel, ssat, msg_buffer);
    if (!stat) {
      RCLCPP_WARN(get_logger(), "SPP failed: %s",
                  std::strlen(msg_buffer) > 0 ? msg_buffer : "pntpos returned 0");
      return;
    }

    publishSolution(obs, sol, ssat);
  }

  void publishSolution(const std::vector<obsd_t>& obs, const sol_t& sol, const ssat_t* ssat) {
    int cnt_sys[7] = {};
    int cnt_frq[NFREQ] = {};
    for (const auto& ob : obs) {
      const int sat_idx = ob.sat - 1;
      if (sat_idx >= 0 && sat_idx < MAXSAT && ssat[sat_idx].vs) {
        for (int j = 0; j < opt_.nf; ++j) {
          if (ob.P[j] != 0.0) cnt_frq[j]++;
        }
      }
    }
    for (int i = 0; i < MAXSAT; ++i) {
      if (!ssat[i].vs) continue;
      const int sat = i + 1;
      const int sys = satsys(sat, nullptr);
      if      (sys == SYS_GPS) cnt_sys[0]++;
      else if (sys == SYS_GLO) cnt_sys[1]++;
      else if (sys == SYS_GAL) cnt_sys[2]++;
      else if (sys == SYS_QZS) cnt_sys[3]++;
      else if (sys == SYS_CMP) cnt_sys[4]++;
      else if (sys == SYS_IRN) cnt_sys[5]++;
      else if (sys == SYS_SBS) cnt_sys[6]++;
    }

    char sat_breakdown[128];
    std::snprintf(sat_breakdown, sizeof(sat_breakdown),
                  "G:%d R:%d E:%d J:%d C:%d I:%d",
                  cnt_sys[0], cnt_sys[1], cnt_sys[2],
                  cnt_sys[3], cnt_sys[4], cnt_sys[5]);
    char frq_breakdown[128] = "";
    if (opt_.nf >= 1)
      std::snprintf(frq_breakdown + std::strlen(frq_breakdown),
                    sizeof(frq_breakdown) - std::strlen(frq_breakdown),
                    "L1:%d", cnt_frq[0]);
    if (opt_.nf >= 2)
      std::snprintf(frq_breakdown + std::strlen(frq_breakdown),
                    sizeof(frq_breakdown) - std::strlen(frq_breakdown),
                    " L2:%d", cnt_frq[1]);
    if (opt_.nf >= 3)
      std::snprintf(frq_breakdown + std::strlen(frq_breakdown),
                    sizeof(frq_breakdown) - std::strlen(frq_breakdown),
                    " L5:%d", cnt_frq[2]);

    double llh[3];
    ecef2pos(sol.rr, llh);

    char tstr[40];
    time2str(sol.time, tstr, 3);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), kPositionLogThrottleMs,
        "SPP OK | %s | LLH: (%.8f, %.8f, %.3f) | Sats: %d (%s) | Freqs: (%s) | %s",
        tstr, llh[0] * R2D, llh[1] * R2D, llh[2], sol.ns,
        sat_breakdown, frq_breakdown, gnss_utils::solqToString(sol.stat).c_str());

    auto sol_msg = std::make_unique<grs::GnssSolution>();
    sol_msg->header.stamp = this->now();
    sol_msg->header.frame_id = "gnss_link";

    int week = 0;
    sol_msg->time_tow = time2gpst(sol.time, &week);
    sol_msg->time_week = static_cast<uint32_t>(week);
    sol_msg->solution_source = grs::GnssSolution::SOLUTION_SOURCE_COMPUTED;

    sol_msg->status = sol.stat;
    sol_msg->num_sats = sol.ns;
    sol_msg->ratio = sol.ratio;
    sol_msg->age_diff = sol.age;

    const auto dops = gnss_utils::calculateDops(ssat, MAXSAT, opt_.elmin);
    sol_msg->gdop = dops.gdop;
    sol_msg->pdop = dops.pdop;
    sol_msg->hdop = dops.hdop;
    sol_msg->vdop = dops.vdop;

    sol_msg->latitude  = llh[0] * R2D;
    sol_msg->longitude = llh[1] * R2D;
    sol_msg->altitude  = llh[2];

    sol_msg->pos_ecef.x = sol.rr[0];
    sol_msg->pos_ecef.y = sol.rr[1];
    sol_msg->pos_ecef.z = sol.rr[2];

    // sol.qr layout: {xx, yy, zz, xy, yz, zx}
    sol_msg->pos_cov_ecef[0] = sol.qr[0];
    sol_msg->pos_cov_ecef[4] = sol.qr[1];
    sol_msg->pos_cov_ecef[8] = sol.qr[2];
    sol_msg->pos_cov_ecef[1] = sol.qr[3];
    sol_msg->pos_cov_ecef[3] = sol.qr[3];
    sol_msg->pos_cov_ecef[5] = sol.qr[4];
    sol_msg->pos_cov_ecef[7] = sol.qr[4];
    sol_msg->pos_cov_ecef[2] = sol.qr[5];
    sol_msg->pos_cov_ecef[6] = sol.qr[5];

    sol_msg->vel_ecef.x = sol.rr[3];
    sol_msg->vel_ecef.y = sol.rr[4];
    sol_msg->vel_ecef.z = sol.rr[5];
    sol_msg->vel_cov_ecef[0] = sol.qv[0];
    sol_msg->vel_cov_ecef[4] = sol.qv[1];
    sol_msg->vel_cov_ecef[8] = sol.qv[2];
    sol_msg->vel_cov_ecef[1] = sol.qv[3];
    sol_msg->vel_cov_ecef[3] = sol.qv[3];
    sol_msg->vel_cov_ecef[5] = sol.qv[4];
    sol_msg->vel_cov_ecef[7] = sol.qv[4];
    sol_msg->vel_cov_ecef[2] = sol.qv[5];
    sol_msg->vel_cov_ecef[6] = sol.qv[5];

    // ENU origin: fixed_origin (cached at startup) > first successful fix.
    if (!origin_set_) {
      if (fixed_origin_valid_) {
        std::copy_n(fixed_origin_ecef_, 3, origin_ecef_);
      } else {
        std::copy_n(sol.rr, 3, origin_ecef_);
        RCLCPP_INFO(get_logger(),
                    "ENU origin set to first fix: %.3f %.3f %.3f",
                    origin_ecef_[0], origin_ecef_[1], origin_ecef_[2]);
      }
      origin_set_ = true;
    }

    sol_msg->pos_enu_org_ecef.x = origin_ecef_[0];
    sol_msg->pos_enu_org_ecef.y = origin_ecef_[1];
    sol_msg->pos_enu_org_ecef.z = origin_ecef_[2];

    double origin_llh[3];
    ecef2pos(origin_ecef_, origin_llh);
    const double d_ecef[3] = {
        sol.rr[0] - origin_ecef_[0],
        sol.rr[1] - origin_ecef_[1],
        sol.rr[2] - origin_ecef_[2],
    };
    double pos_enu[3];
    ecef2enu(origin_llh, d_ecef, pos_enu);
    sol_msg->pos_enu.x = pos_enu[0];
    sol_msg->pos_enu.y = pos_enu[1];
    sol_msg->pos_enu.z = pos_enu[2];

    // Covariances are rotated at the rover's current LLH so the reported
    // E/N/U axes match the rover frame at the solution epoch.
    double Q_ecef[9] = {
        sol.qr[0], sol.qr[3], sol.qr[5],
        sol.qr[3], sol.qr[1], sol.qr[4],
        sol.qr[5], sol.qr[4], sol.qr[2],
    };
    double Q_enu[9];
    gnss_utils::rotateCovariance(Q_ecef, llh[0], llh[1], Q_enu);
    for (int i = 0; i < 9; ++i) sol_msg->pos_enu_cov[i] = Q_enu[i];

    double vel_ecef[3] = {sol.rr[3], sol.rr[4], sol.rr[5]};
    double vel_enu[3];
    ecef2enu(llh, vel_ecef, vel_enu);
    sol_msg->vel_enu.x = vel_enu[0];
    sol_msg->vel_enu.y = vel_enu[1];
    sol_msg->vel_enu.z = vel_enu[2];

    double Qv_ecef[9] = {
        sol.qv[0], sol.qv[3], sol.qv[5],
        sol.qv[3], sol.qv[1], sol.qv[4],
        sol.qv[5], sol.qv[4], sol.qv[2],
    };
    double Qv_enu[9];
    gnss_utils::rotateCovariance(Qv_ecef, llh[0], llh[1], Qv_enu);
    for (int i = 0; i < 9; ++i) sol_msg->vel_enu_cov[i] = Qv_enu[i];

    gnss_sol_pub_->publish(std::move(sol_msg));

    // NMEA fan-out. SPP solutions are always single-fix, but RTKLIB may report
    // a downstream-relevant SOLQ_*; force SOLQ_SINGLE here so RTKPLOT colors
    // the trace as single-point regardless of the upstream label.
    sol_t sol_for_nmea = sol;
    sol_for_nmea.stat = SOLQ_SINGLE;

    unsigned char gga[kNmeaBufBytes];
    int n = outnmea_gga(gga, &sol_for_nmea);
    if (n > 0) {
      if (!(n >= 2 && gga[n - 2] == '\r' && gga[n - 1] == '\n') &&
          n <= static_cast<int>(kNmeaBufBytes) - 2) {
        gga[n++] = '\r';
        gga[n++] = '\n';
      }
      tcp_server_.send(reinterpret_cast<const char*>(gga), n);
    }
    unsigned char rmc[kNmeaBufBytes];
    int m = outnmea_rmc(rmc, &sol_for_nmea);
    if (m > 0) {
      if (!(m >= 2 && rmc[m - 2] == '\r' && rmc[m - 1] == '\n') &&
          m <= static_cast<int>(kNmeaBufBytes) - 2) {
        rmc[m++] = '\r';
        rmc[m++] = '\n';
      }
      tcp_server_.send(reinterpret_cast<const char*>(rmc), m);
    }
  }

  rclcpp::Subscription<grs::GnssObservations>::SharedPtr obs_sub_;
  rclcpp::Subscription<grs::GnssEphemerides>::SharedPtr nav_sub_;
  rclcpp::Publisher<grs::GnssSolution>::SharedPtr gnss_sol_pub_;

  std::mutex nav_mtx_;
  prcopt_t opt_{};
  nav_t nav_{};
  gnss_utils::EphemerisStore eph_store_;

  gnss_utils::TcpNmeaServer tcp_server_;

  bool enable_l1_{true};
  bool enable_l2_{true};
  bool enable_l5_{true};

  double fixed_origin_ecef_[3]{0.0, 0.0, 0.0};
  bool fixed_origin_valid_{false};
  double origin_ecef_[3]{0.0, 0.0, 0.0};
  bool origin_set_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SppPntposNode>());
  rclcpp::shutdown();
  return 0;
}
