#ifndef GNSS_ROS_STANDARDIZATION_TCP_NMEA_SERVER_HPP
#define GNSS_ROS_STANDARDIZATION_TCP_NMEA_SERVER_HPP

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
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
#include <string>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace gnss_utils {

// Minimal TCP push server used by positioning nodes to stream NMEA sentences
// (typically GGA/RMC) to RTKPLOT or similar clients. send() only enqueues;
// socket I/O runs on the worker. Slow clients cannot stop the producer.
class TcpNmeaServer {
 public:
  TcpNmeaServer() = default;
  ~TcpNmeaServer() { stop(); }

  TcpNmeaServer(const TcpNmeaServer&) = delete;
  TcpNmeaServer& operator=(const TcpNmeaServer&) = delete;

  // Start listening on `port`. The logger is copied for diagnostic output.
  // `port <= 0` is treated as "disabled" and returns false without error.
  bool start(int port, rclcpp::Logger logger) {
    if (port <= 0 || port > 65535 || run_server_) return false;
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

    if (::listen(server_socket_, kBacklog) < 0 ||
        ::fcntl(server_socket_, F_SETFL, O_NONBLOCK) < 0) {
      RCLCPP_ERROR(*logger_, "TcpNmeaServer: listen/nonblocking setup failed");
      ::close(server_socket_);
      server_socket_ = -1;
      return false;
    }
    run_server_ = true;
    server_thread_ = std::thread(&TcpNmeaServer::acceptLoop, this, server_socket_);
    RCLCPP_INFO(*logger_, "TcpNmeaServer listening on port %d", port);
    return true;
  }

  // Tear down acceptor and close all sockets. Safe to call multiple times.
  void stop() {
    run_server_ = false;
    if (server_socket_ != -1) {
      ::shutdown(server_socket_, SHUT_RDWR);
    }
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
    if (server_socket_ != -1) {
      ::close(server_socket_);
      server_socket_ = -1;
    }
    std::lock_guard<std::mutex> lk(client_sockets_mtx_);
    for (const auto& client : client_sockets_) ::close(client.fd);
    client_sockets_.clear();
  }

  // At most 16 clients, 64 KiB pending per client. Overflow disconnects that
  // client; never drop a suffix and continue the same corrupted byte stream.
  // start()/stop() are lifecycle operations and must be serialized by the owner.
  void send(const char* data, std::size_t length) {
    if (!data || !length || !run_server_) return;
    std::lock_guard<std::mutex> lk(client_sockets_mtx_);
    auto it = client_sockets_.begin();
    while (it != client_sockets_.end()) {
      if (length > kMaxPending - it->pending.size()) {
        if (logger_) RCLCPP_WARN(*logger_, "TcpNmeaServer: slow client disconnected (64 KiB queue limit)");
        ::close(it->fd);
        it = client_sockets_.erase(it);
      } else {
        if (it->pending.empty()) it->progress = Clock::now();
        it->pending.append(data, length);
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
  static constexpr size_t kMaxClients = 16;
  static constexpr size_t kMaxPending = 64 * 1024;
  using Clock = std::chrono::steady_clock;
  struct Client {
    int fd;
    std::string pending;
    Clock::time_point progress{Clock::now()};
  };

  void acceptLoop(int listener) {
    while (run_server_) {
      pollfd poller{listener, POLLIN, 0};
      ::poll(&poller, 1, 20);
      if (!run_server_) break;
      // Limit accepts and writes per iteration, even under connection floods.
      for (size_t count = 0; count < kMaxClients; ++count) {
      sockaddr_in client_addr{};
      socklen_t client_len = sizeof(client_addr);
      int client = ::accept(listener, reinterpret_cast<struct sockaddr*>(&client_addr), &client_len);
      if (client < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        break;
      }
      std::lock_guard<std::mutex> lk(client_sockets_mtx_);
      if (client_sockets_.size() >= kMaxClients ||
          ::fcntl(client, F_SETFL, O_NONBLOCK) < 0) {
        ::close(client);
      } else {
        int send_buffer = 16 * 1024;
        ::setsockopt(client, SOL_SOCKET, SO_SNDBUF, &send_buffer, sizeof(send_buffer));
        client_sockets_.push_back(Client{client, {}});
      }
      }
      std::lock_guard<std::mutex> lk(client_sockets_mtx_);
      auto it = client_sockets_.begin();
      while (it != client_sockets_.end()) {
        bool failed = false;
        if (!it->pending.empty()) {
          const auto n = ::send(it->fd, it->pending.data(), it->pending.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
          if (n > 0) {
            it->pending.erase(0, static_cast<size_t>(n));
            it->progress = Clock::now();
          } else if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
            failed = true;
          }
          if (!it->pending.empty() && Clock::now() - it->progress >= std::chrono::seconds(5))
            failed = true;
        }
        // Detect idle disconnects as well. This is an output-only service.
        char probe;
        const auto n = ::recv(it->fd, &probe, 1, MSG_PEEK | MSG_DONTWAIT);
        if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR))
          failed = true;
        if (failed) {
          if (logger_) RCLCPP_INFO(*logger_, "TcpNmeaServer: client disconnected or stalled for 5 seconds");
          ::close(it->fd);
          it = client_sockets_.erase(it);
        } else ++it;
      }
    }
  }

  int server_socket_{-1};
  std::vector<Client> client_sockets_;
  std::mutex client_sockets_mtx_;
  std::thread server_thread_;
  std::atomic<bool> run_server_{false};
  std::unique_ptr<rclcpp::Logger> logger_;
};

}  // namespace gnss_utils

#endif  // GNSS_ROS_STANDARDIZATION_TCP_NMEA_SERVER_HPP
