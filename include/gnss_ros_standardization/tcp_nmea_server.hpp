// SPDX-License-Identifier: MIT
#ifndef GNSS_ROS_STANDARDIZATION_TCP_NMEA_SERVER_HPP
#define GNSS_ROS_STANDARDIZATION_TCP_NMEA_SERVER_HPP

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace gnss_utils {

// Minimal TCP push server used by positioning nodes to stream NMEA sentences
// (typically GGA/RMC) to RTKPLOT or similar clients. Single-threaded acceptor
// loop, fan-out broadcast on send(). Dead clients are pruned on the first
// failed send().
class TcpNmeaServer {
 public:
  TcpNmeaServer() = default;
  ~TcpNmeaServer() { stop(); }

  TcpNmeaServer(const TcpNmeaServer&) = delete;
  TcpNmeaServer& operator=(const TcpNmeaServer&) = delete;

  // Start listening on `port`. `logger` is borrowed for diagnostic output.
  // `port <= 0` is treated as "disabled" and returns false without error.
  bool start(int port, rclcpp::Logger logger) {
    if (port <= 0) return false;
    logger_ = std::make_unique<rclcpp::Logger>(logger);

    server_socket_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket_ == -1) {
      RCLCPP_ERROR(*logger_, "TcpNmeaServer: socket() failed");
      return false;
    }

    int opt = 1;
    ::setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (::bind(server_socket_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(*logger_, "TcpNmeaServer: bind() failed on port %d", port);
      ::close(server_socket_);
      server_socket_ = -1;
      return false;
    }

    ::listen(server_socket_, kBacklog);
    run_server_ = true;
    server_thread_ = std::thread(&TcpNmeaServer::acceptLoop, this);
    RCLCPP_INFO(*logger_, "TcpNmeaServer listening on port %d", port);
    return true;
  }

  // Tear down acceptor and close all sockets. Safe to call multiple times.
  void stop() {
    run_server_ = false;
    if (server_socket_ != -1) {
      ::shutdown(server_socket_, SHUT_RDWR);
      ::close(server_socket_);
      server_socket_ = -1;
    }
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
    std::lock_guard<std::mutex> lk(client_sockets_mtx_);
    for (int s : client_sockets_) ::close(s);
    client_sockets_.clear();
  }

  // Broadcast `length` bytes to all connected clients. Disconnected clients
  // are dropped on first failed send(). NMEA sentences are short (<256 B), so
  // a partial write is treated as success here — revisit for larger payloads.
  void send(const char* data, std::size_t length) {
    std::lock_guard<std::mutex> lk(client_sockets_mtx_);
    auto it = client_sockets_.begin();
    while (it != client_sockets_.end()) {
      if (::send(*it, data, length, MSG_NOSIGNAL) < 0) {
        if (logger_) RCLCPP_INFO(*logger_, "TcpNmeaServer: client disconnected");
        ::close(*it);
        it = client_sockets_.erase(it);
      } else {
        ++it;
      }
    }
  }

  bool hasClients() {
    std::lock_guard<std::mutex> lk(client_sockets_mtx_);
    return !client_sockets_.empty();
  }

 private:
  static constexpr int kBacklog = 5;
  static constexpr int kAcceptBackoffMs = 100;

  void acceptLoop() {
    while (run_server_) {
      sockaddr_in client_addr{};
      socklen_t client_len = sizeof(client_addr);
      int client = ::accept(server_socket_, reinterpret_cast<struct sockaddr*>(&client_addr), &client_len);
      if (client < 0) {
        if (!run_server_) break;
        // Persistent accept failure (e.g. fd-table exhausted, EINTR): back off
        // so we don't pin a CPU on a tight failure loop.
        if (logger_) RCLCPP_WARN(*logger_, "TcpNmeaServer: accept() failed: %s", std::strerror(errno));
        std::this_thread::sleep_for(std::chrono::milliseconds(kAcceptBackoffMs));
        continue;
      }
      if (logger_) {
        char ip[INET_ADDRSTRLEN] = {0};
        inet_ntop(AF_INET, &client_addr.sin_addr, ip, sizeof(ip));
        RCLCPP_INFO(*logger_, "TcpNmeaServer: client connected from %s", ip);
      }
      std::lock_guard<std::mutex> lk(client_sockets_mtx_);
      client_sockets_.push_back(client);
    }
  }

  int server_socket_{-1};
  std::vector<int> client_sockets_;
  std::mutex client_sockets_mtx_;
  std::thread server_thread_;
  std::atomic<bool> run_server_{false};
  std::unique_ptr<rclcpp::Logger> logger_;
};

}  // namespace gnss_utils

#endif  // GNSS_ROS_STANDARDIZATION_TCP_NMEA_SERVER_HPP
