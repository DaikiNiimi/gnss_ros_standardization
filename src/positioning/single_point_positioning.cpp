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
#include <arpa/inet.h> 
#include <unistd.h>

#include "gnss_ros_standardization/gnss_utils.hpp"


extern "C" {
  #include "rtklib.h"
}

namespace grs = gnss_ros_standardization::msg;

class SppPntposNode : public rclcpp::Node {
public:
  SppPntposNode() : Node("spp_pntpos_node") {
    obs_sub_ = create_subscription<grs::GnssObservations>(
      "/gnss/observation", rclcpp::QoS(50),
      std::bind(&SppPntposNode::onObs, this, std::placeholders::_1));

    nav_sub_ = create_subscription<grs::GnssEphemerides>(
      "/gnss/ephemeris", rclcpp::QoS(10).reliable(),
      std::bind(&SppPntposNode::onNav, this, std::placeholders::_1));

    opt_ = prcopt_default;
    opt_.mode    = PMODE_SINGLE;
    opt_.nf      = 1;
    opt_.navsys  = SYS_GPS | SYS_GLO | SYS_GAL | SYS_QZS | SYS_CMP;
    opt_.ionoopt = IONOOPT_BRDC;
    opt_.tropopt = TROPOPT_SAAS;
    opt_.sateph  = EPHOPT_BRDC;
    opt_.elmin = 15.0 * D2R;
    opt_.snrmask.ena[0] = 1;
    opt_.snrmask.mask[0][0] = 35.0;
    opt_.snrmask.mask[0][1] = 35.0;
    opt_.snrmask.mask[0][2] = 35.0;

    std::memset(&nav_, 0, sizeof(nav_));

    // --- TCP Server Setup ---
    setupTcpServer(8000); // Start TCP server on port 8000

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
  // (Helper functions toEph, toGeph, upsertEph, upsertGeph, solstatToString are unchanged)

  void upsertEph(const eph_t &e) {
    if (e.sat <= 0 || e.sat > MAXSAT) return;
    for (int i = 0; i < nav_.n; i++) {
      if (nav_.eph[i].sat == e.sat) { nav_.eph[i] = e; return; }
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
      auto &acc = satmap[sat];
      if (!acc.inited) {
        acc.o = {}; acc.o.time = t; acc.o.sat = (uint8_t)sat; acc.o.rcv = 1;
        acc.inited = true;
      }
      acc.o.code[idx] = code;
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
          else if (sys == SYS_CMP) cnt_sys[4]++; else if (sys == SYS_SBS) cnt_sys[6]++;
      }
      char sat_breakdown[128];
      snprintf(sat_breakdown, sizeof(sat_breakdown), "G:%d R:%d E:%d J:%d C:%d",
              cnt_sys[0], cnt_sys[1], cnt_sys[2], cnt_sys[3], cnt_sys[4]);
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
  std::mutex nav_mtx_;

  prcopt_t opt_{};
  nav_t    nav_{};

  // --- TCP Server Member Variables ---
  int server_socket_ = -1;
  std::vector<int> client_sockets_;
  std::mutex client_sockets_mtx_;
  std::thread server_thread_;
  bool run_server_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SppPntposNode>());
  rclcpp::shutdown();
  return 0;
}
