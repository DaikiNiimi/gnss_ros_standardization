#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>
#include <cstring>
#include <mutex>
#include <cstdio>
#include <thread>
#include <sys/socket.h>
#include <netinet/in.h>
#include "gnss_ros_standardization/msg/gnss_ephemerides.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"
#include "gnss_ros_standardization/gnss_utils.hpp"

#include <arpa/inet.h> 
#include <unistd.h>


extern "C" {
  #include "rtklib.h"
}

namespace grs = gnss_ros_standardization::msg;

class SppPntposNode : public rclcpp::Node {
public:
  SppPntposNode() : Node("spp_pntpos_node") {
    initializeParameters();

    // Use configured topic names
    obs_sub_ = create_subscription<grs::GnssObservations>(
      get_parameter("topics.observation").as_string(), rclcpp::QoS(50),
      std::bind(&SppPntposNode::onObs, this, std::placeholders::_1));

    nav_sub_ = create_subscription<grs::GnssEphemerides>(
      get_parameter("topics.ephemeris").as_string(), rclcpp::QoS(100).transient_local(),
      std::bind(&SppPntposNode::onNav, this, std::placeholders::_1));

    gnss_sol_pub_ = this->create_publisher<grs::GnssSolution>(
      get_parameter("topics.solution").as_string(), 10);

    opt_ = prcopt_default;
    opt_.mode    = PMODE_SINGLE;
    opt_.nf      = 1;
    
    // Configure Nav Systems
    opt_.navsys = 0;
    if (get_parameter("nav_systems.gps").as_bool()) opt_.navsys |= SYS_GPS;
    if (get_parameter("nav_systems.glo").as_bool()) opt_.navsys |= SYS_GLO;
    if (get_parameter("nav_systems.gal").as_bool()) opt_.navsys |= SYS_GAL;
    if (get_parameter("nav_systems.bds").as_bool()) opt_.navsys |= SYS_CMP;
    if (get_parameter("nav_systems.qzs").as_bool()) opt_.navsys |= SYS_QZS;
    if (get_parameter("nav_systems.irn").as_bool()) opt_.navsys |= SYS_IRN;
    if (get_parameter("nav_systems.sbs").as_bool()) opt_.navsys |= SYS_SBS;

    opt_.ionoopt = IONOOPT_BRDC;
    opt_.tropopt = TROPOPT_SAAS;
    opt_.sateph  = EPHOPT_BRDC;
    
    // Configure Masks
    opt_.elmin = get_parameter("elevation_mask_deg").as_double() * D2R;
    
    
    // Configure SNR Mask
    if (get_parameter("snrmask.enable").as_bool()) {
        opt_.snrmask.ena[0] = 1; // rover
        auto l1 = get_parameter("snrmask.l1").as_double_array();
        auto l2 = get_parameter("snrmask.l2").as_double_array();
        auto l5 = get_parameter("snrmask.l5").as_double_array();
        
        for (int i=0; i<9; ++i) {
            if (i < (int)l1.size()) opt_.snrmask.mask[0][i] = l1[i];
            if (i < (int)l2.size()) opt_.snrmask.mask[1][i] = l2[i];
            if (i < (int)l5.size()) opt_.snrmask.mask[2][i] = l5[i];
        }
    }

    // Configure Frequencies
    enable_l1_ = get_parameter("frequencies.enable_l1").as_bool();
    enable_l2_ = get_parameter("frequencies.enable_l2").as_bool();
    enable_l5_ = get_parameter("frequencies.enable_l5").as_bool();

    // Dynamically set number of frequencies (nf)
    // RTKLIB uses frequencies up to nf. So if L5 is used, nf must be 3.
    if (enable_l5_) {
        opt_.nf = 3;
    } else if (enable_l2_) {
        opt_.nf = 2;
    } else {
        opt_.nf = 1;
    }
    
    // Configure Excluded Satellites
    auto excluded = get_parameter("excluded_satellites").as_string_array();
    for (const auto& satid : excluded) {
        int sat = satid2no(satid.c_str());
        if (sat > 0 && sat <= MAXSAT) {
            opt_.exsats[sat-1] = 1; // 1: excluded
        }
    }
    
    RCLCPP_INFO(get_logger(), "Frequency config: L1=%d L2=%d L5=%d -> nf=%d", 
                enable_l1_, enable_l2_, enable_l5_, opt_.nf);

    std::memset(&nav_, 0, sizeof(nav_));

    // --- TCP Server Setup ---
    // --- TCP Server Setup ---
    int tcp_port = get_parameter("tcp_port").as_int();
    setupTcpServer(tcp_port); // Start TCP server

    RCLCPP_INFO(get_logger(), "SPP Point-Positioning Node has been started. Waiting for GNSS data...");
  }

  ~SppPntposNode() override {
    // --- TCP Server Shutdown ---
    run_server_ = false;
    if (server_socket_ != -1) {
      shutdown(server_socket_, SHUT_RDWR);
      close(server_socket_);
    }
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
    {
      std::lock_guard<std::mutex> lock(client_sockets_mtx_);
      for (int client_socket : client_sockets_) {
        close(client_socket);
      }
      client_sockets_.clear();
    }
    // --- End TCP Server Shutdown ---

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
    
    // SNR Mask
    declare_parameter<bool>("snrmask.enable", false);
    declare_parameter<std::vector<double>>("snrmask.l1", {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0});
    declare_parameter<std::vector<double>>("snrmask.l2", {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0});
    declare_parameter<std::vector<double>>("snrmask.l5", {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0});
    
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
    declare_parameter<std::vector<double>>("fixed_origin.pos", {0.0, 0.0, 0.0});
  }

  // (Helper functions toEph, toGeph, upsertEph, upsertGeph, solstatToString are unchanged)

  void upsertEph(const eph_t &e) {
    if (e.sat <= 0 || e.sat > MAXSAT) return;
    for (int i = 0; i < nav_.n; i++) {
      if (nav_.eph[i].sat == e.sat && nav_.eph[i].code == e.code) { nav_.eph[i] = e; return; }
    }
    if (nav_.n >= nav_.nmax) {
      int newmax = nav_.nmax == 0 ? 8 : nav_.nmax * 2;
      auto *p = (eph_t*)std::realloc(nav_.eph, sizeof(eph_t) * newmax);
      if (!p) { RCLCPP_ERROR(get_logger(), "realloc failed for nav_.eph"); return; }
      nav_.eph = p; nav_.nmax = newmax;
    }
    nav_.eph[nav_.n++] = e;
  }
  void upsertGeph(const geph_t &g) {
    if (g.sat <= 0 || g.sat > MAXSAT) return;
    for (int i = 0; i < nav_.ng; i++) {
      if (nav_.geph[i].sat == g.sat) { nav_.geph[i] = g; return; }
    }
    if (nav_.ng >= nav_.ngmax) {
      int newmax = nav_.ngmax == 0 ? 8 : nav_.ngmax * 2;
      auto *p = (geph_t*)std::realloc(nav_.geph, sizeof(geph_t) * newmax);
      if (!p) { RCLCPP_ERROR(get_logger(), "realloc failed for nav_.geph"); return; }
      nav_.geph = p; nav_.ngmax = newmax;
    }
    nav_.geph[nav_.ng++] = g;
  }
  static std::string solstatToString(int stat) {
      switch (stat) {
          case SOLQ_NONE: return "No Solution";
          case SOLQ_FIX: return "Fix";
          case SOLQ_FLOAT: return "Float";
          case SOLQ_SBAS: return "SBAS";
          case SOLQ_DGPS: return "DGPS";
          case SOLQ_SINGLE: return "Single";
          case SOLQ_PPP: return "PPP";
          case SOLQ_DR: return "DR";
          default: return "Unknown";
      }
  }

  void onNav(const grs::GnssEphemerides::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(nav_mtx_);
    for (const auto &e : msg->gnss_ephemeris) {
      if (!e.satid.empty()) upsertEph(gnss_utils::msgToEph(e));
    }
    for (const auto &g : msg->glonass_ephemeris) {
      if (!g.satid.empty()) upsertGeph(gnss_utils::msgToGeph(g));
    }
    RCLCPP_INFO_ONCE(get_logger(), "Ephemeris received. Ready for positioning.");
  }

  /// Guard against frequency collisions in code2idx().
  /// Only BDS idx=0 has a real collision (B1I 1561 MHz vs B1C 1575 MHz).
  /// All other systems have no collision — accept all codes on any idx.
  static bool isPrimaryCode(int sys, uint8_t code, int idx) {
    if (sys == SYS_CMP && idx == 0 && code != CODE_L2I) {
      return false;  // Reject B1C on idx=0; keep only B1I
    }
    return true;
  }

  void onObs(const grs::GnssObservations::SharedPtr in) {
    // (onObs function is unchanged until after the pntpos call)
    gtime_t t = gpst2time(static_cast<int>(in->week), in->tow);
    struct Acc { obsd_t o{}; bool inited=false; };
    std::unordered_map<int, Acc> satmap;
    for (const auto &m : in->observations) {
      const int sat = satid2no(m.satid.c_str());
      if (sat <= 0 || sat > MAXSAT) continue;
      // Use integer code directly if available, or fall back to string parsing?
      // Since we populate both, we can use integer code directly for efficiency!
      // const uint8_t code = obs2code(m.code_str.c_str());
      // Actually let's use the efficient way:
      const uint8_t code = m.code; 
      if (code == CODE_NONE) continue;
      int prn = 0; const int sys = satsys(sat, &prn);
      const int idx = code2idx(sys, code);
      if (idx < 0) continue;
      if (!isPrimaryCode(sys, code, idx)) continue;

      // Filter by frequency
      // RTKLIB index mapping (roughly):
      // 0 -> L1/E1/B1
      // 1 -> L2/E5b/B2
      // 2 -> L5/E5a/B2a
      if (idx == 0 && !enable_l1_) continue;
      if (idx == 1 && !enable_l2_) continue;
      if (idx == 2 && !enable_l5_) continue;
      if (idx > 2) continue; // Skip unsupported frequencies for now

      auto &acc = satmap[sat];
      if (!acc.inited) {
        acc.o = {}; acc.o.time = t; acc.o.sat = (uint8_t)sat; acc.o.rcv = 1;
        acc.inited = true;
      }
      acc.o.code[idx] = code;
      if (m.p > 0.0) acc.o.P[idx] = m.p;
      if (m.l != 0.0) acc.o.L[idx] = m.l;
      if (m.d != 0.0) acc.o.D[idx] = static_cast<float>(m.d);
      if (m.snr > 0.0) acc.o.SNR[idx] = static_cast<uint16_t>(std::lround(m.snr * 4.0));
      acc.o.LLI[idx] = static_cast<uint8_t>(m.lli);
    }
    std::lock_guard<std::mutex> lk(nav_mtx_);
    if (nav_.n == 0 && nav_.ng == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No ephemeris data...");
      return;
    }
    std::vector<obsd_t> obs;
    obs.reserve(satmap.size());
    for (auto &kv : satmap) {
        obs.push_back(std::move(kv.second.o));
    }
    if (obs.size() < 4) {
      RCLCPP_WARN(get_logger(), "Not enough satellites observed. (Found %zu, Need >= 4)", obs.size());
      return;
    }
    std::sort(obs.begin(), obs.end(), [](const obsd_t& a, const obsd_t& b) {
      return a.sat < b.sat;
    });
    sol_t  sol{};
    ssat_t ssat[MAXSAT]{};
    double azel[MAXSAT*2]{};
    char msg_buffer[1024] = "";
    const int stat = pntpos(obs.data(), static_cast<int>(obs.size()),
                            &nav_, &opt_, &sol, azel, ssat, msg_buffer);

    if (stat) {
      // (Satellite and frequency counting logic is unchanged)
      int cnt_sys[7] = {}; int cnt_frq[NFREQ] = {};
      for (const auto& ob : obs) {
        int sat_idx = ob.sat - 1;
        if (sat_idx >= 0 && sat_idx < MAXSAT && ssat[sat_idx].vs) {
          for (int j = 0; j < opt_.nf; j++) {
            if (ob.P[j] != 0.0) { cnt_frq[j]++; }
          }
        }
      }
      for (int i=0; i<MAXSAT; ++i) {
          if (!ssat[i].vs) continue;
          int sat = i + 1; int sys = satsys(sat, NULL);
          if (sys == SYS_GPS) cnt_sys[0]++; else if (sys == SYS_GLO) cnt_sys[1]++;
          else if (sys == SYS_GAL) cnt_sys[2]++; else if (sys == SYS_QZS) cnt_sys[3]++;
          else if (sys == SYS_CMP) cnt_sys[4]++; else if (sys == SYS_IRN) cnt_sys[5]++;
          else if (sys == SYS_SBS) cnt_sys[6]++;
      }
      char sat_breakdown[128];
      snprintf(sat_breakdown, sizeof(sat_breakdown), "G:%d R:%d E:%d J:%d C:%d I:%d",
              cnt_sys[0], cnt_sys[1], cnt_sys[2], cnt_sys[3], cnt_sys[4], cnt_sys[5]);
      char frq_breakdown[128] = "";
      if (opt_.nf >= 1) snprintf(frq_breakdown + strlen(frq_breakdown), sizeof(frq_breakdown) - strlen(frq_breakdown), "L1:%d", cnt_frq[0]);
      if (opt_.nf >= 2) snprintf(frq_breakdown + strlen(frq_breakdown), sizeof(frq_breakdown) - strlen(frq_breakdown), " L2:%d", cnt_frq[1]);
      if (opt_.nf >= 3) snprintf(frq_breakdown + strlen(frq_breakdown), sizeof(frq_breakdown) - strlen(frq_breakdown), " L5:%d", cnt_frq[2]);
      
      double llh[3];
      ecef2pos(sol.rr, llh);
      
      RCLCPP_INFO(get_logger(),
        "SPP OK | Time: %s | LLH: (%.8f, %.8f, %.3f)m | Sats: %d (%s) | Freqs: (%s) | Sol: %s",
        time_str(sol.time, 3), llh[0] * R2D, llh[1] * R2D, llh[2], sol.ns,
        sat_breakdown, frq_breakdown, solstatToString(sol.stat).c_str());




      // --- Publish GnssSolution ---
      auto sol_msg = std::make_unique<grs::GnssSolution>();
      sol_msg->header.stamp = this->now();
      sol_msg->header.frame_id = "gnss_link";
      
      sol_msg->time_week = sol.time.time / (7*24*3600);
      sol_msg->time_tow = fmod(sol.time.time + sol.time.sec, 7*24*3600);
      
      sol_msg->status = sol.stat;
      sol_msg->num_sats = sol.ns;
      sol_msg->ratio = sol.ratio;
      sol_msg->age_diff = sol.age;
      
      
      // Calculate DOPs using helper (sol_t does not have dop member)
      auto dops = gnss_utils::calculateDops(ssat, MAXSAT, opt_.elmin);
      sol_msg->gdop = dops.gdop;
      sol_msg->pdop = dops.pdop;
      sol_msg->hdop = dops.hdop;
      sol_msg->vdop = dops.vdop;
      
      sol_msg->latitude = llh[0] * R2D;
      sol_msg->longitude = llh[1] * R2D;
      sol_msg->altitude = llh[2];
      
      // Global Position (ECEF)
      sol_msg->pos_ecef.x = sol.rr[0];
      sol_msg->pos_ecef.y = sol.rr[1];
      sol_msg->pos_ecef.z = sol.rr[2];
      
      // Position Covariance (ECEF)
      sol_msg->pos_cov_ecef[0] = sol.qr[0]; // xx
      sol_msg->pos_cov_ecef[4] = sol.qr[1]; // yy
      sol_msg->pos_cov_ecef[8] = sol.qr[2]; // zz
      sol_msg->pos_cov_ecef[1] = sol.qr[3]; // xy
      sol_msg->pos_cov_ecef[3] = sol.qr[3]; // yx
      sol_msg->pos_cov_ecef[5] = sol.qr[4]; // yz
      sol_msg->pos_cov_ecef[7] = sol.qr[4]; // zy
      sol_msg->pos_cov_ecef[2] = sol.qr[5]; // zx
      sol_msg->pos_cov_ecef[6] = sol.qr[5]; // xz

      // Global Velocity (ECEF)
      sol_msg->vel_ecef.x = sol.rr[3];
      sol_msg->vel_ecef.y = sol.rr[4];
      sol_msg->vel_ecef.z = sol.rr[5];
      
      // Velocity Covariance (ECEF)
      sol_msg->vel_cov_ecef[0] = sol.qv[0];
      sol_msg->vel_cov_ecef[4] = sol.qv[1];
      sol_msg->vel_cov_ecef[8] = sol.qv[2];
      sol_msg->vel_cov_ecef[1] = sol.qv[3];
      sol_msg->vel_cov_ecef[3] = sol.qv[3];
      sol_msg->vel_cov_ecef[5] = sol.qv[4];
      sol_msg->vel_cov_ecef[7] = sol.qv[4];
      sol_msg->vel_cov_ecef[2] = sol.qv[5];
      sol_msg->vel_cov_ecef[6] = sol.qv[5];

      // Local Origin (ECEF)
      if (norm(origin_ecef_, 3) <= 0.0) {
          // Check parameters first
          std::string type = get_parameter("fixed_origin.postype").as_string();
          std::vector<double> pos = get_parameter("fixed_origin.pos").as_double_array();
          
          if (pos.size() == 3 && norm(pos.data(), 3) > 0.0) {
              if (type == "llh") {
                  // Convert LLH to ECEF
                  double input_llh[3] = {pos[0] * D2R, pos[1] * D2R, pos[2]};
                  pos2ecef(input_llh, origin_ecef_);
                  RCLCPP_INFO(get_logger(), "ENU Origin set from Config (LLH): %.8f, %.8f, %.3f -> ECEF: %.3f, %.3f, %.3f", 
                              pos[0], pos[1], pos[2], origin_ecef_[0], origin_ecef_[1], origin_ecef_[2]);
              } else {
                  // Assume ECEF
                  origin_ecef_[0] = pos[0];
                  origin_ecef_[1] = pos[1];
                  origin_ecef_[2] = pos[2];
                  RCLCPP_INFO(get_logger(), "ENU Origin set from Config (ECEF): %.3f, %.3f, %.3f", 
                              origin_ecef_[0], origin_ecef_[1], origin_ecef_[2]);
              }
              origin_set_ = true;
          } else if (!origin_set_) {
              // Auto-set from first fix (Datum)
              origin_ecef_[0] = sol.rr[0];
              origin_ecef_[1] = sol.rr[1];
              origin_ecef_[2] = sol.rr[2];
              origin_set_ = true;
              RCLCPP_INFO(get_logger(), "ENU Origin set to First Fix: %.3f, %.3f, %.3f", 
                          origin_ecef_[0], origin_ecef_[1], origin_ecef_[2]);
          }
      }

      sol_msg->org_ecef.x = origin_ecef_[0];
      sol_msg->org_ecef.y = origin_ecef_[1];
      sol_msg->org_ecef.z = origin_ecef_[2];

      // Local Position (ENU)
      if (origin_set_) {
          double origin_llh[3];
          ecef2pos(origin_ecef_, origin_llh);
          
          double d_ecef[3] = {
              sol.rr[0] - origin_ecef_[0],
              sol.rr[1] - origin_ecef_[1],
              sol.rr[2] - origin_ecef_[2]
          };
          double pos_enu[3];
          ecef2enu(origin_llh, d_ecef, pos_enu);
          
          sol_msg->pos_enu.x = pos_enu[0];
          sol_msg->pos_enu.y = pos_enu[1];
          sol_msg->pos_enu.z = pos_enu[2];
          
          // Local Position Covariance (ENU) - Rotated
          // Use current LLH for rotation? Or Origin LLH?
          // Strictly speaking, covariance at current position should use current LLH for ENU directions.
          // But 'local_pos_cov' usually pairs with 'local_position'.
          // If local_position is relative to Origin, its covariance is also in Origin's ENU frame?
          // Usually we want "East/North/Up" at the rover's location.
          // Let's use current LLH for rotation (same as RTK logic).
          double Q_ecef[9] = {
            sol.qr[0], sol.qr[3], sol.qr[5],
            sol.qr[3], sol.qr[1], sol.qr[4],
            sol.qr[5], sol.qr[4], sol.qr[2]
          };
          double Q_enu[9];
          gnss_utils::rotateCovariance(Q_ecef, llh[0], llh[1], Q_enu);
          for(int i=0; i<9; ++i) sol_msg->pos_enu_cov[i] = Q_enu[i];
      } else {
          // Should not happen if logic above handles first fix
          sol_msg->pos_enu.x = 0;
          sol_msg->pos_enu.y = 0;
          sol_msg->pos_enu.z = 0;
      }
      
      // Calculate ENU velocity
      double vel_ecef[3] = {sol.rr[3], sol.rr[4], sol.rr[5]};
      double vel_enu[3];
      ecef2enu(llh, vel_ecef, vel_enu);

      // Local Velocity (ENU)
      sol_msg->vel_enu.x = vel_enu[0];
      sol_msg->vel_enu.y = vel_enu[1];
      sol_msg->vel_enu.z = vel_enu[2];
      
      // Velocity Covariance (ENU) - Rotated from ECEF
      double Qv_ecef[9] = {
        sol.qv[0], sol.qv[3], sol.qv[5],
        sol.qv[3], sol.qv[1], sol.qv[4],
        sol.qv[5], sol.qv[4], sol.qv[2]
      };
      double Qv_enu[9];
      gnss_utils::rotateCovariance(Qv_ecef, llh[0], llh[1], Qv_enu);
      for(int i=0; i<9; ++i) sol_msg->vel_enu_cov[i] = Qv_enu[i];

      gnss_sol_pub_->publish(std::move(sol_msg));


      unsigned char gga[256], rmc[256];
      sol_t sol_for_nmea = sol;
      sol_for_nmea.stat = SOLQ_SINGLE;

      // 1) GGA
      int n = outnmea_gga(gga, &sol_for_nmea);
      if (n > 0) {
        if (!(n >= 2 && gga[n-2] == '\r' && gga[n-1] == '\n')) {
          if (n <= 254) { gga[n++] = '\r'; gga[n++] = '\n'; }
        }
        sendTcpData(reinterpret_cast<const char*>(gga), n);
      }

      int m = outnmea_rmc(rmc, &sol_for_nmea);
      if (m > 0) {
        if (!(m >= 2 && rmc[m-2] == '\r' && rmc[m-1] == '\n')) {
          if (m <= 254) { rmc[m++] = '\r'; rmc[m++] = '\n'; }
        }
        sendTcpData(reinterpret_cast<const char*>(rmc), m);
      }

    } else {
      RCLCPP_ERROR(get_logger(), "SPP Failed | Error: %s", strlen(msg_buffer) > 0 ? msg_buffer : "pntpos returned 0");
    }
  }

  // --- TCP Server Member Functions ---
  void setupTcpServer(int port) {
    server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket_ == -1) {
      RCLCPP_ERROR(get_logger(), "Failed to create server socket");
      return;
    }

    int opt = 1;
    setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);

    if (bind(server_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to bind server socket to port %d", port);
      close(server_socket_);
      server_socket_ = -1;
      return;
    }

    listen(server_socket_, 5);
    run_server_ = true;
    server_thread_ = std::thread(&SppPntposNode::acceptConnections, this);
    RCLCPP_INFO(get_logger(), "TCP server started on port %d, ready for RTKPLOT connections.", port);
  }

  void acceptConnections() {
    while (run_server_ && rclcpp::ok()) {
      sockaddr_in client_addr{};
      socklen_t client_len = sizeof(client_addr);
      int client_socket = accept(server_socket_, (struct sockaddr*)&client_addr, &client_len);

      if (client_socket < 0) {
        if (run_server_) {
          RCLCPP_WARN(get_logger(), "Failed to accept client connection.");
        }
        continue;
      }
      
      char client_ip[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
      RCLCPP_INFO(get_logger(), "Client connected from %s", client_ip);

      std::lock_guard<std::mutex> lock(client_sockets_mtx_);
      client_sockets_.push_back(client_socket);
    }
  }

  void sendTcpData(const char* data, size_t length) {
    std::lock_guard<std::mutex> lock(client_sockets_mtx_);
    auto it = client_sockets_.begin();
    while (it != client_sockets_.end()) {
      int client_socket = *it;
      if (send(client_socket, data, length, MSG_NOSIGNAL) < 0) {
        RCLCPP_INFO(get_logger(), "Client disconnected. Closing socket.");
        close(client_socket);
        it = client_sockets_.erase(it);
      } else {
        ++it;
      }
    }
  }

private:
  rclcpp::Subscription<grs::GnssObservations>::SharedPtr obs_sub_;
  rclcpp::Subscription<grs::GnssEphemerides>::SharedPtr  nav_sub_;
  rclcpp::Publisher<grs::GnssSolution>::SharedPtr gnss_sol_pub_;
  std::mutex nav_mtx_;

  prcopt_t opt_{};
  nav_t    nav_{};

  // --- TCP Server Member Variables ---
  int server_socket_ = -1;
  std::vector<int> client_sockets_;
  std::mutex client_sockets_mtx_;
  std::thread server_thread_;
  bool run_server_ = false;

  // Frequency flags
  bool enable_l1_ = true;
  bool enable_l2_ = true;
  bool enable_l5_ = true;

  // Origin for Local ENU
  double origin_ecef_[3] = {0.0, 0.0, 0.0};
  bool origin_set_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SppPntposNode>());
  rclcpp::shutdown();
  return 0;
}
