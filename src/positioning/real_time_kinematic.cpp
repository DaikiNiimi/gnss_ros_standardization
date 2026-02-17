#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>
#include <cstring>
#include <mutex>
#include <deque>
#include <thread>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "gnss_ros_standardization/gnss_utils.hpp"
#include "gnss_ros_standardization/msg/gnss_solution.hpp"

extern "C" {
  #include "rtklib.h"
}

namespace grs = gnss_ros_standardization::msg;

class RtkPositionNode : public rclcpp::Node {
public:
  RtkPositionNode() : Node("rtk_position_node") {
    std::cerr << "RtkPositionNode constructor start" << std::endl;
    try {
        initializeParameters();
    } catch (const rclcpp::exceptions::InvalidParameterValueException& e) {
        std::cerr << "Invalid parameter value: " << e.what() << std::endl;
        throw;
    } catch (const std::exception& e) {
        std::cerr << "Exception in initializeParameters: " << e.what() << std::endl;
        throw;
    }
    std::cerr << "initializeParameters done" << std::endl;

    const std::string rover_obs_topic = get_parameter("topics.rover_observation").as_string();
    const std::string base_obs_topic = get_parameter("topics.base_observation").as_string();
    const std::string eph_topic = get_parameter("topics.ephemeris").as_string();

    RCLCPP_INFO(get_logger(), "Topics retrieved.");

    rover_obs_sub_ = create_subscription<grs::GnssObservations>(
      rover_obs_topic, rclcpp::QoS(50),
      std::bind(&RtkPositionNode::onRoverObs, this, std::placeholders::_1));

    base_obs_sub_ = create_subscription<grs::GnssObservations>(
      base_obs_topic, rclcpp::QoS(50),
      std::bind(&RtkPositionNode::onBaseObs, this, std::placeholders::_1));

    nav_sub_ = create_subscription<grs::GnssEphemerides>(
      eph_topic, rclcpp::QoS(100).transient_local(),
      std::bind(&RtkPositionNode::onNav, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscriptions created. Initializing RTK...");
    initializeRtk();
    RCLCPP_INFO(get_logger(), "RTK initialized.");

    gnss_sol_pub_ = this->create_publisher<grs::GnssSolution>(
      get_parameter("topics.solution").as_string(), 10);

    int tcp_port = get_parameter("tcp_port").as_int();
    if (tcp_port > 0) {
      setupTcpServer(tcp_port);
    }

    RCLCPP_INFO(get_logger(), "RTK Positioning Node started. Waiting for Rover and Base data...");
  }

  ~RtkPositionNode() override {
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

    // Topics
    declare_safe("topics.rover_observation", std::string("/rover/gnss/observation"));
    declare_safe("topics.base_observation", std::string("/base/gnss/observation"));
    declare_safe("topics.ephemeris", std::string("/gnss/ephemeris"));
    declare_safe("topics.solution", std::string("/gnss/solution"));

    // TCP Server
    declare_safe("tcp_port", 8000);

    // Base Position
    declare_safe("ant2.postype", std::string("rtcm")); // ecef, llh, rtcm
    declare_safe("ant2.pos", std::vector<double>{0.0, 0.0, 0.0});

    // Fixed Origin for Enu
    declare_safe("fixed_origin.postype", std::string("llh"));
    declare_safe("fixed_origin.pos", std::vector<double>{0.0, 0.0, 0.0});

    // pos1 options
    declare_safe("pos1.frequency", 2);
    declare_safe("pos1.elmask", 15.0);
    declare_safe("pos1.dynamics", 0);
    declare_safe("pos1.ionoopt", IONOOPT_BRDC);
    declare_safe("pos1.tropopt", TROPOPT_SAAS);
    declare_safe("pos1.sateph", EPHOPT_BRDC);
    declare_safe("pos1.eratio1", 100.0);
    
    // Navigation Systems
    declare_safe("pos1.navsys.gps", true);
    declare_safe("pos1.navsys.glo", true);
    declare_safe("pos1.navsys.gal", true);
    declare_safe("pos1.navsys.bds", true);
    declare_safe("pos1.navsys.qzs", true);
    declare_safe("pos1.navsys.irn", true);

    // SNR Mask
    declare_safe("pos1.snrmask.enable", false);
    declare_safe("pos1.snrmask.l1", std::vector<double>{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0});
    declare_safe("pos1.snrmask.l2", std::vector<double>{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0});
    declare_safe("pos1.snrmask.l5", std::vector<double>{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0});

    // Excluded Stats
    declare_safe("excluded_satellites", std::vector<std::string>{});

    // pos2 options
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
    declare_safe("pos2.rejgdop", 30.0);
    declare_safe("pos2.niter", 1);

    // stats options
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
    
    RCLCPP_INFO(get_logger(), "Init RTK: getting pos1");
    // pos1
    opt.mode = PMODE_KINEMA; // Fixed to Kinematic
    opt.nf = get_parameter("pos1.frequency").as_int();
    opt.elmin = get_parameter("pos1.elmask").as_double() * D2R;
    opt.dynamics = get_parameter("pos1.dynamics").as_int();
    opt.ionoopt = get_parameter("pos1.ionoopt").as_int();
    opt.tropopt = get_parameter("pos1.tropopt").as_int();
    opt.sateph = get_parameter("pos1.sateph").as_int();
    
    RCLCPP_INFO(get_logger(), "Init RTK: getting navsys");
    // navsys
    opt.navsys = 0;
    if (get_parameter("pos1.navsys.gps").as_bool()) opt.navsys |= SYS_GPS;
    if (get_parameter("pos1.navsys.glo").as_bool()) opt.navsys |= SYS_GLO;
    if (get_parameter("pos1.navsys.gal").as_bool()) opt.navsys |= SYS_GAL;
    if (get_parameter("pos1.navsys.bds").as_bool()) opt.navsys |= SYS_CMP;
    if (get_parameter("pos1.navsys.qzs").as_bool()) opt.navsys |= SYS_QZS;
    if (get_parameter("pos1.navsys.irn").as_bool()) opt.navsys |= SYS_IRN;

    RCLCPP_INFO(get_logger(), "Init RTK: getting snrmask");
    // SNR mask
    if (get_parameter("pos1.snrmask.enable").as_bool()) {
        opt.snrmask.ena[0] = 1; // rover
        opt.snrmask.ena[1] = 1; // base
        auto l1 = get_parameter("pos1.snrmask.l1").as_double_array();
        auto l2 = get_parameter("pos1.snrmask.l2").as_double_array();
        auto l5 = get_parameter("pos1.snrmask.l5").as_double_array();
        
        for (int i=0; i<9; ++i) {
            if (i < (int)l1.size()) opt.snrmask.mask[0][i] = l1[i];
            if (i < (int)l2.size()) opt.snrmask.mask[1][i] = l2[i];
            if (i < (int)l5.size()) opt.snrmask.mask[2][i] = l5[i];
        }
    }

    RCLCPP_INFO(get_logger(), "Init RTK: getting excluded");
    // Excluded satellites
    auto excluded = get_parameter("excluded_satellites").as_string_array();
    for (const auto& satid : excluded) {
        int sat = satid2no(satid.c_str());
        if (sat > 0 && sat <= MAXSAT) {
            opt.exsats[sat-1] = 1; // 1: excluded
        }
    }

    RCLCPP_INFO(get_logger(), "Init RTK: getting pos2");
    // pos2
    opt.modear = get_parameter("pos2.armode").as_int();
    opt.glomodear = get_parameter("pos2.gloarmode").as_int();
    opt.bdsmodear = get_parameter("pos2.bdsarmode").as_int();
    opt.thresar[0] = get_parameter("pos2.arthres").as_double();
    opt.minlock = get_parameter("pos2.arlockcnt").as_int();
    opt.elmaskar = get_parameter("pos2.arelmask").as_double() * D2R;
    opt.minfix = get_parameter("pos2.arminfix").as_int();
    opt.armaxiter = get_parameter("pos2.armaxiter").as_int();
    opt.elmaskhold = get_parameter("pos2.elmaskhold").as_double() * D2R;
    opt.maxout = get_parameter("pos2.aroutcnt").as_int();
    opt.maxtdiff = get_parameter("pos2.maxage").as_double();
    opt.syncsol = get_parameter("pos2.syncsol").as_int();
    opt.thresslip = get_parameter("pos2.slipthres").as_double();
    opt.maxinno = get_parameter("pos2.rejionno").as_double();
    opt.maxgdop = get_parameter("pos2.rejgdop").as_double();
    opt.niter = get_parameter("pos2.niter").as_int();

    RCLCPP_INFO(get_logger(), "Init RTK: getting stats");
    // stats
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

    RCLCPP_INFO(get_logger(), "Init RTK: getting ant2");
    // Base position
    std::string postype = get_parameter("ant2.postype").as_string();
    auto pos_vec = get_parameter("ant2.pos").as_double_array();
    
    if (postype == "ecef") {
      opt.refpos = 0; // pos in prcopt
      if (pos_vec.size() == 3) {
        std::copy(pos_vec.begin(), pos_vec.end(), opt.rb);
      }
    } else if (postype == "llh") {
      opt.refpos = 0;
      if (pos_vec.size() == 3) {
        double llh[3] = {pos_vec[0] * D2R, pos_vec[1] * D2R, pos_vec[2]};
        pos2ecef(llh, opt.rb);
      }
    } else if (postype == "rtcm") {
      opt.refpos = 4; // read from rtcm
    }

    rtkinit(&rtk_, &opt);
    
    if (opt.refpos == 0) {
        for (int i = 0; i < 3; i++) rtk_.rb[i] = opt.rb[i];
        RCLCPP_INFO(get_logger(), "RTK Init: Force set Base Pos to %.3f, %.3f, %.3f", 
            rtk_.rb[0], rtk_.rb[1], rtk_.rb[2]);
    }

    std::memset(&nav_, 0, sizeof(nav_));
  }

  void onNav(const grs::GnssEphemerides::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(nav_mtx_);
    for (const auto &e : msg->gnss_ephemeris) {
      if (!e.satid.empty()) upsertEph(gnss_utils::msgToEph(e));
    }
    for (const auto &g : msg->glonass_ephemeris) {
      if (!g.satid.empty()) upsertGeph(gnss_utils::msgToGeph(g));
    }
  }

  void upsertEph(const eph_t &e) {
    if (e.sat <= 0 || e.sat > MAXSAT) return;
    for (int i = 0; i < nav_.n; i++) {
        if (nav_.eph[i].sat == e.sat && nav_.eph[i].code == e.code) { nav_.eph[i] = e; return; }
    }
    if (nav_.n >= nav_.nmax) {
        int newmax = nav_.nmax == 0 ? 8 : nav_.nmax * 2;
        auto *p = (eph_t*)std::realloc(nav_.eph, sizeof(eph_t) * newmax);
        if (!p) return;
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
        if (!p) return;
        nav_.geph = p; nav_.ngmax = newmax;
    }
    nav_.geph[nav_.ng++] = g;
  }

  void onBaseObs(const grs::GnssObservations::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(obs_mtx_);
    base_obs_queue_.push_back(msg);
    if (base_obs_queue_.size() > 50) base_obs_queue_.pop_front();
    matchAndProcess();
  }

  void onRoverObs(const grs::GnssObservations::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(obs_mtx_);
    rover_obs_queue_.push_back(msg);
    if (rover_obs_queue_.size() > 50) rover_obs_queue_.pop_front();
    matchAndProcess();
  }

  void matchAndProcess() {
    auto it_rover = rover_obs_queue_.begin();
    while (it_rover != rover_obs_queue_.end()) {
      grs::GnssObservations::SharedPtr rover_msg = *it_rover;
      grs::GnssObservations::SharedPtr matched_base = nullptr;
      
      // Find match in base queue
      for (const auto& base_msg : base_obs_queue_) {
           if (std::abs(base_msg->tow - rover_msg->tow) < 0.001) {
               matched_base = base_msg;
               break;
           }
      }
      
      if (matched_base) {
          processRtk(rover_msg, matched_base);
          it_rover = rover_obs_queue_.erase(it_rover);
          continue;
      }
      
      // Timeout check: if rover msg is significantly older than the newest base msg
      if (!base_obs_queue_.empty()) {
          double newest_base_tow = base_obs_queue_.back()->tow;
          // If Rover is older than Base by > 2.0s, drop it (we likely missed the base data)
          if (newest_base_tow > rover_msg->tow + 2.0) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                  "Dropped Rover Obs (TOW: %.3f) - Timeout awaiting Base (Newest Base: %.3f)", 
                  rover_msg->tow, newest_base_tow);
              it_rover = rover_obs_queue_.erase(it_rover);
              continue;
          }
      }
      
      // Keep waiting
      ++it_rover;
    }
  }

  void processRtk(const grs::GnssObservations::SharedPtr rover, const grs::GnssObservations::SharedPtr base) {
    std::lock_guard<std::mutex> lk_nav(nav_mtx_);
    if (nav_.n == 0 && nav_.ng == 0) return;

    std::vector<obsd_t> obs;
    obs.reserve(rover->observations.size() + base->observations.size());

    gtime_t tr = gpst2time(rover->week, rover->tow);
    gtime_t tb = gpst2time(base->week, base->tow);

    char tr_str[64], tb_str[64];
    time2str(tr, tr_str, 3);
    time2str(tb, tb_str, 3);
    
    // Aggregate observations by (Receiver, Satellite) to combine frequencies
    // Key: (rcv << 8) | sat.  (Assumes sat < 256, rcv is 1 or 2)
    std::map<int, obsd_t> obs_map;

    // Helper to merge observations
    auto merge_obs = [&](const grs::GnssObservation& m, gtime_t t, int rcv) {
        obsd_t o = convertObs(m, t, rcv);
        if (o.sat == 0) return;
        
        int key = (rcv << 16) | o.sat;
        if (obs_map.find(key) == obs_map.end()) {
            obs_map[key] = o;
        } else {
            // Merge into existing
            obsd_t& existing = obs_map[key];
            for (int i = 0; i < NFREQ; i++) {
                if (o.P[i] != 0.0) existing.P[i] = o.P[i];
                if (o.L[i] != 0.0) existing.L[i] = o.L[i];
                if (o.D[i] != 0.0) existing.D[i] = o.D[i];
                if (o.SNR[i] != 0) existing.SNR[i] = o.SNR[i];
                if (o.LLI[i] != 0) existing.LLI[i] = o.LLI[i];
                if (o.code[i] != 0) existing.code[i] = o.code[i];
            }
        }
    };

    // Rover (rcv=1)
    for (const auto &m : rover->observations) merge_obs(m, tr, 1);
    // Base (rcv=2)
    for (const auto &m : base->observations) merge_obs(m, tb, 2);

    for (const auto& kv : obs_map) {
        obs.push_back(kv.second);
    }

    if (obs.empty()) return;

    // Sort observations as required by rtkpos
    std::sort(obs.begin(), obs.end(), [](const obsd_t& a, const obsd_t& b) {
      if (a.rcv != b.rcv) return a.rcv < b.rcv;
      return a.sat < b.sat;
    });

    int n_rover = 0, n_base = 0;
    for (const auto& o : obs) {
        if (o.rcv == 1) n_rover++;
        else if (o.rcv == 2) n_base++;
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, 
        "RTK Status: %s | Rover: %d, Base: %d | Base Pos: %.3f, %.3f, %.3f | Ratio: %.1f | Sats: %d", 
        rtk_.sol.stat == SOLQ_FIX ? "FIX" : (rtk_.sol.stat == SOLQ_FLOAT ? "FLOAT" : "NONE"),
        n_rover, n_base, 
        rtk_.rb[0], rtk_.rb[1], rtk_.rb[2], 
        rtk_.sol.ratio,
        rtk_.sol.ns);



    int stat = rtkpos(&rtk_, obs.data(), static_cast<int>(obs.size()), &nav_);

    if (stat > 0) {
      double llh[3];
      ecef2pos(rtk_.sol.rr, llh);
      RCLCPP_INFO(get_logger(), "RTK %s | LLH: %.8f, %.8f, %.3f | Ratio: %.1f | Sats: %d",
                  getStatString(rtk_.sol.stat).c_str(), llh[0]*R2D, llh[1]*R2D, llh[2],
                  rtk_.sol.ratio, rtk_.sol.ns);
      
      // Calculate ENU velocity
      double vel_ecef[3] = {rtk_.sol.rr[3], rtk_.sol.rr[4], rtk_.sol.rr[5]};
      double vel_enu[3];
      ecef2enu(llh, vel_ecef, vel_enu);

      // Calculate Local ENU Position (Relative to Base)
      double pos_enu[3];
      // RTKLIB stores Base ECEF in rtk_.rb
      if (norm(rtk_.rb, 3) > 0.0) {
          double rb_llh[3];
          ecef2pos(rtk_.rb, rb_llh);
          double d_ecef[3] = {
              rtk_.sol.rr[0] - rtk_.rb[0],
              rtk_.sol.rr[1] - rtk_.rb[1],
              rtk_.sol.rr[2] - rtk_.rb[2]
          };
          ecef2enu(rb_llh, d_ecef, pos_enu);
      } else {
          // Fallback if base not set (should not happen in RTK)
          pos_enu[0] = 0; pos_enu[1] = 0; pos_enu[2] = 0;
      }

      // Publish GnssSolution
      auto sol_msg = std::make_unique<grs::GnssSolution>();
      sol_msg->header.stamp = this->now();
      sol_msg->header.frame_id = "gnss_link";
      
      sol_msg->time_week = rtk_.sol.time.time / (7*24*3600); // Approximate week (RTKLIB internal time is GPST)
      sol_msg->time_tow = fmod(rtk_.sol.time.time + rtk_.sol.time.sec, 7*24*3600); 

      sol_msg->status = rtk_.sol.stat;
      sol_msg->num_sats = rtk_.sol.ns;
      sol_msg->ratio = rtk_.sol.ratio;
      sol_msg->age_diff = rtk_.sol.age;
      // Calculate DOPs
      // Using rtk_.opt.elmin which should be populated during configuration
      auto dops = gnss_utils::calculateDops(rtk_.ssat, MAXSAT, rtk_.opt.elmin);
      
      sol_msg->gdop = dops.gdop;
      sol_msg->pdop = dops.pdop;
      sol_msg->hdop = dops.hdop;
      sol_msg->vdop = dops.vdop;

      sol_msg->latitude = llh[0] * R2D;
      sol_msg->longitude = llh[1] * R2D;
      sol_msg->altitude = llh[2];

      // Covariance (position) - RTKLIB gives float QR[6] = {xx,yy,zz,xy,yz,zx}
      // Note: These are in ECEF usually for sol_t. Need rotation to ENU if strictly needed.
      // For simplicity here, we populate diagonal. A full rotation P_enu = R * P_ecef * R' is better.
      // Let's implement a proper rotation later if needed, or use the single point pos function which does it.
      // For now, just fill diagonal with first 3 elements (xx, yy, zz) as approximation or placeholder.
      // Position covariance (ENU)

      // Global Position (ECEF)
      sol_msg->position_ecef.x = rtk_.sol.rr[0];
      sol_msg->position_ecef.y = rtk_.sol.rr[1];
      sol_msg->position_ecef.z = rtk_.sol.rr[2];
      // Note: qr is in ECEF, so we can map it directly to pos_cov_ecef if we treat it as such.
      // RTKLIB sol_t qr is float[6] = {xx, yy, zz, xy, yz, zx} variance-covariance in ECEF?
      // Actually RTKLIB documentation says:
      // "float qr[6]; /* position variance/covariance (m^2) */
      //               /* {c_xx,c_yy,c_zz,c_xy,c_yz,c_zx} or */
      //               /* {c_ee,c_nn,c_uu,c_en,c_nu,c_ue} */"
      // It depends on the mode. For RTK (kinematic), it is typically ECEF.
      sol_msg->pos_cov_ecef[0] = rtk_.sol.qr[0];
      sol_msg->pos_cov_ecef[4] = rtk_.sol.qr[1];
      sol_msg->pos_cov_ecef[8] = rtk_.sol.qr[2];
      sol_msg->pos_cov_ecef[1] = rtk_.sol.qr[3]; // xy
      sol_msg->pos_cov_ecef[3] = rtk_.sol.qr[3]; // yx
      sol_msg->pos_cov_ecef[5] = rtk_.sol.qr[4]; // yz
      sol_msg->pos_cov_ecef[7] = rtk_.sol.qr[4]; // zy
      sol_msg->pos_cov_ecef[2] = rtk_.sol.qr[5]; // zx
      sol_msg->pos_cov_ecef[6] = rtk_.sol.qr[5]; // xz

      // Global Velocity (ECEF)
      sol_msg->velocity_ecef.x = rtk_.sol.rr[3];
      sol_msg->velocity_ecef.y = rtk_.sol.rr[4];
      sol_msg->velocity_ecef.z = rtk_.sol.rr[5];
      // Velocity covariance
      sol_msg->vel_cov_ecef[0] = rtk_.sol.qv[0];
      sol_msg->vel_cov_ecef[4] = rtk_.sol.qv[1];
      sol_msg->vel_cov_ecef[8] = rtk_.sol.qv[2];
      sol_msg->vel_cov_ecef[1] = rtk_.sol.qv[3];
      sol_msg->vel_cov_ecef[3] = rtk_.sol.qv[3];
      sol_msg->vel_cov_ecef[5] = rtk_.sol.qv[4];
      sol_msg->vel_cov_ecef[7] = rtk_.sol.qv[4];
      sol_msg->vel_cov_ecef[2] = rtk_.sol.qv[5];
      sol_msg->vel_cov_ecef[6] = rtk_.sol.qv[5];

      // Local Origin (ECEF)
      double origin_ecef[3] = {0.0, 0.0, 0.0};
      
      // Check fixed_origin parameters first
      std::string fo_type = get_parameter("fixed_origin.postype").as_string();
      std::vector<double> fo_pos = get_parameter("fixed_origin.pos").as_double_array();
      
      bool use_fixed = false;
      if (fo_pos.size() == 3 && norm(fo_pos.data(), 3) > 0.0) {
          if (fo_type == "llh") {
              double input_llh[3] = {fo_pos[0] * D2R, fo_pos[1] * D2R, fo_pos[2]};
              pos2ecef(input_llh, origin_ecef);
              RCLCPP_INFO_ONCE(get_logger(), "RTK ENU Origin set from Config (LLH->ECEF)");
          } else {
              origin_ecef[0] = fo_pos[0];
              origin_ecef[1] = fo_pos[1];
              origin_ecef[2] = fo_pos[2];
              RCLCPP_INFO_ONCE(get_logger(), "RTK ENU Origin set from Config (ECEF)");
          }
          use_fixed = true;
      } 
      
      if (!use_fixed) {
          // Fallback to Base Station Position (rtk_.rb)
          if (norm(rtk_.rb, 3) > 0.0) {
              origin_ecef[0] = rtk_.rb[0];
              origin_ecef[1] = rtk_.rb[1];
              origin_ecef[2] = rtk_.rb[2];
          }
      }

      sol_msg->local_origin_ecef.x = origin_ecef[0];
      sol_msg->local_origin_ecef.y = origin_ecef[1];
      sol_msg->local_origin_ecef.z = origin_ecef[2];
      
      // Calculate Local Position (ENU) relative to chosen origin
      if (norm(origin_ecef, 3) > 0.0) {
          double origin_llh[3];
          ecef2pos(origin_ecef, origin_llh);
          
          double d_ecef[3] = {
              rtk_.sol.rr[0] - origin_ecef[0],
              rtk_.sol.rr[1] - origin_ecef[1],
              rtk_.sol.rr[2] - origin_ecef[2]
          };
          double loc_pos_enu[3];
          ecef2enu(origin_llh, d_ecef, loc_pos_enu);
          
          sol_msg->local_position.x = loc_pos_enu[0];
          sol_msg->local_position.y = loc_pos_enu[1];
          sol_msg->local_position.z = loc_pos_enu[2];
      } else {
           // No origin, no local position
           sol_msg->local_position.x = 0.0;
           sol_msg->local_position.y = 0.0;
           sol_msg->local_position.z = 0.0;
      }

      // Local Position Covariance (ENU) - Rotated from ECEF
      // Re-adding this as per user request for "Local Position and its Covariance"
      double Q_ecef[9] = {
        rtk_.sol.qr[0], rtk_.sol.qr[3], rtk_.sol.qr[5],
        rtk_.sol.qr[3], rtk_.sol.qr[1], rtk_.sol.qr[4],
        rtk_.sol.qr[5], rtk_.sol.qr[4], rtk_.sol.qr[2]
      };
      double Q_enu[9];
      gnss_utils::rotateCovariance(Q_ecef, llh[0], llh[1], Q_enu);
      for(int i=0; i<9; ++i) sol_msg->local_pos_cov[i] = Q_enu[i];

      // Local Velocity (ENU)
      sol_msg->velocity_enu.x = vel_enu[0];
      sol_msg->velocity_enu.y = vel_enu[1];
      sol_msg->velocity_enu.z = vel_enu[2];
      
      // Velocity covariance (ENU) - Rotated from ECEF (rtk_.sol.qv)
      double Qv_ecef[9] = {
        rtk_.sol.qv[0], rtk_.sol.qv[3], rtk_.sol.qv[5],
        rtk_.sol.qv[3], rtk_.sol.qv[1], rtk_.sol.qv[4],
        rtk_.sol.qv[5], rtk_.sol.qv[4], rtk_.sol.qv[2]
      };
      double Qv_enu[9];
      gnss_utils::rotateCovariance(Qv_ecef, llh[0], llh[1], Qv_enu);
      
      for(int i=0; i<9; ++i) sol_msg->vel_cov_enu[i] = Qv_enu[i];

      gnss_sol_pub_->publish(std::move(sol_msg));

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, 
                  "RTK %s | LLH: %.8f, %.8f, %.3f | Ratio: %.1f | Sats: %d",
                  getStatString(rtk_.sol.stat).c_str(), llh[0]*R2D, llh[1]*R2D, llh[2],
                  rtk_.sol.ratio, rtk_.sol.ns);
      
      // Output to TCP server (NMEA)
      if (!client_sockets_.empty()) {
        unsigned char gga[256], rmc[256];
        int n = outnmea_gga(gga, &rtk_.sol);
        if (n > 0) sendTcpData(reinterpret_cast<const char*>(gga), n);
        int m = outnmea_rmc(rmc, &rtk_.sol);
        if (m > 0) sendTcpData(reinterpret_cast<const char*>(rmc), m);
      }
    }
  }

  obsd_t convertObs(const grs::GnssObservation &m, gtime_t t, int rcv) {
    obsd_t o{};
    int sat = satid2no(m.satid.c_str());
    if (sat <= 0 || sat > MAXSAT) return o;

    o.time = t;
    o.sat = static_cast<uint8_t>(sat);
    o.rcv = static_cast<uint8_t>(rcv);

    int prn = 0;
    int sys = satsys(sat, &prn);
    int idx = code2idx(sys, m.code);
    if (idx < 0 || idx >= NFREQ) return o;

    o.P[idx] = m.p;
    o.L[idx] = m.l;
    o.D[idx] = static_cast<float>(m.d);
    // ROS msg is dBHz (float). RTKLIB SNR is 0.001 dBHz (uint16).
    // So 45.0 dBHz -> 45000.
    o.SNR[idx] = static_cast<uint16_t>(m.snr * 1000.0);
    o.LLI[idx] = static_cast<uint8_t>(m.lli);
    o.code[idx] = m.code;

    return o;
  }

  std::string getStatString(int stat) {
    switch (stat) {
      case SOLQ_FIX: return "FIX";
      case SOLQ_FLOAT: return "FLOAT";
      case SOLQ_DGPS: return "DGPS";
      case SOLQ_SINGLE: return "SINGLE";
      default: return "NONE";
    }
  }

  // TCP Server logic
  void setupTcpServer(int port) {
    server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket_ == -1) return;
    int opt = 1;
    setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);
    if (bind(server_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
      close(server_socket_);
      server_socket_ = -1;
      return;
    }
    listen(server_socket_, 5);
    run_server_ = true;
    server_thread_ = std::thread(&RtkPositionNode::acceptConnections, this);
  }

  void acceptConnections() {
    while (run_server_ && rclcpp::ok()) {
      sockaddr_in client_addr{};
      socklen_t client_len = sizeof(client_addr);
      int client_socket = accept(server_socket_, (struct sockaddr*)&client_addr, &client_len);
      if (client_socket < 0) continue;
      std::lock_guard<std::mutex> lock(client_sockets_mtx_);
      client_sockets_.push_back(client_socket);
    }
  }

  void sendTcpData(const char* data, size_t length) {
    std::lock_guard<std::mutex> lock(client_sockets_mtx_);
    auto it = client_sockets_.begin();
    while (it != client_sockets_.end()) {
      if (send(*it, data, length, MSG_NOSIGNAL) < 0) {
        close(*it);
        it = client_sockets_.erase(it);
      } else {
        ++it;
      }
    }
  }

  rtk_t rtk_{};
  nav_t nav_{};
  std::mutex nav_mtx_;
  std::mutex obs_mtx_;

  std::deque<grs::GnssObservations::SharedPtr> base_obs_queue_;
  std::deque<grs::GnssObservations::SharedPtr> rover_obs_queue_;
  rclcpp::Subscription<grs::GnssObservations>::SharedPtr rover_obs_sub_;
  rclcpp::Subscription<grs::GnssObservations>::SharedPtr base_obs_sub_;
  rclcpp::Subscription<grs::GnssEphemerides>::SharedPtr nav_sub_;
  rclcpp::Publisher<grs::GnssSolution>::SharedPtr gnss_sol_pub_;

  // TCP Server
  int server_socket_ = -1;
  std::vector<int> client_sockets_;
  std::mutex client_sockets_mtx_;
  std::thread server_thread_;
  bool run_server_ = false;
};

int main(int argc, char** argv) {
  std::cerr << "Starting main..." << std::endl;
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<RtkPositionNode>());
  } catch (const std::exception& e) {
    std::cerr << "Exception in main: " << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
