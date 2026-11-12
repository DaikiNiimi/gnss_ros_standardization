// Minimal HTTP server: serves one page and streams Server-Sent Events.
#pragma once

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace gnss_visualizer {

class SseServer {
 public:
  SseServer(const std::string& address, uint16_t port, std::string_view page)
      : address_(address), port_(port), page_(page) {
    in_addr parsed{};
    if (inet_pton(AF_INET, address.c_str(), &parsed) != 1)
      throw std::invalid_argument("bind_address must be an IPv4 address: " + address);
    loopback_only_ = (ntohl(parsed.s_addr) >> 24) == 127;

    listen_fd_ = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (listen_fd_ < 0) throw std::runtime_error("Cannot create a socket");
    const int yes = 1;
    setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    sockaddr_in endpoint{};
    endpoint.sin_family = AF_INET;
    endpoint.sin_port = htons(port);
    endpoint.sin_addr = parsed;
    if (bind(listen_fd_, reinterpret_cast<sockaddr*>(&endpoint), sizeof(endpoint)) != 0 ||
        listen(listen_fd_, 16) != 0) {
      const std::string reason = std::strerror(errno);
      close(listen_fd_);
      throw std::runtime_error("Cannot listen on " + address + ":" + std::to_string(port) + ": " + reason);
    }
    socklen_t size = sizeof(endpoint);
    if (getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&endpoint), &size) == 0)
      port_ = ntohs(endpoint.sin_port);
    if (pipe2(wake_, O_NONBLOCK | O_CLOEXEC) != 0) {
      close(listen_fd_);
      throw std::runtime_error("Cannot create a wake pipe");
    }
    thread_ = std::thread([this] { run(); });
  }

  ~SseServer() {
    running_ = false;
    wake();
    if (thread_.joinable()) thread_.join();
    for (auto& [fd, client] : clients_) close(fd);
    close(listen_fd_);
    close(wake_[0]);
    close(wake_[1]);
  }
  SseServer(const SseServer&) = delete;
  SseServer& operator=(const SseServer&) = delete;

  // One selection requested by a page: slot name and topic.
  using Selection = std::pair<std::string, std::string>;

  uint16_t port() const { return port_; }
  size_t streamCount() const { return streams_.load(); }

  // Streams that connected since the last call and still need a full snapshot.
  std::vector<int> takeNewStreams() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<int> out;
    out.swap(new_streams_);
    return out;
  }

  std::vector<Selection> takeSelections() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Selection> out;
    out.swap(selections_);
    return out;
  }

  // True when the stream's browser runs on this computer.
  bool isLocalStream(int stream) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = clients_.find(stream);
    return found != clients_.end() && found->second.local;
  }

  void sendTo(const std::vector<int>& streams, const char* event, const std::string& data) {
    const std::string frame = makeFrame(event, data);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const int fd : streams) {
        const auto found = clients_.find(fd);
        if (found != clients_.end() && found->second.stream) queue(found->second, frame);
      }
    }
    wake();
  }

  void broadcast(const char* event, const std::string& data) {
    const std::string frame = makeFrame(event, data);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto& [fd, client] : clients_)
        if (client.stream) queue(client, frame);
    }
    wake();
  }

 private:
  static constexpr size_t kMaxClients = 16;
  static constexpr size_t kMaxRequestBytes = 8192;
  static constexpr size_t kMaxQueuedBytes = 4U * 1024U * 1024U;
  static constexpr auto kRequestTimeout = std::chrono::seconds(10);

  struct Client {
    std::string request, output;
    bool stream{false}, close_after_flush{false}, drop{false}, local{false};
    std::chrono::steady_clock::time_point connected;
  };

  static std::string makeFrame(const char* event, const std::string& data) {
    return std::string("event: ") + event + "\ndata: " + data + "\n\n";
  }

  static void queue(Client& client, const std::string& frame) {
    // A stalled browser must not grow the node's memory without bound.
    if (client.output.size() + frame.size() > kMaxQueuedBytes) {
      client.drop = true;
      return;
    }
    client.output += frame;
  }

  void wake() {
    const char byte = 0;
    [[maybe_unused]] const auto ignored = write(wake_[1], &byte, 1);
  }

  // Value of a header, in lower case; empty when absent. `name` is lower case.
  static std::string headerValue(const std::string& request, const std::string& name) {
    std::string head = request.substr(0, request.find("\r\n\r\n") + 2);
    std::transform(head.begin(), head.end(), head.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    const std::string key = "\r\n" + name + ":";
    const auto at = head.find(key);
    if (at == std::string::npos) return "";
    auto begin = at + key.size();
    const auto end = head.find("\r\n", begin);
    while (begin < end && head[begin] == ' ') ++begin;
    return head.substr(begin, end - begin);
  }

  static bool requestComplete(const std::string& request) {
    const auto end = request.find("\r\n\r\n");
    if (end == std::string::npos) return false;
    const size_t body = static_cast<size_t>(std::strtoul(headerValue(request, "content-length").c_str(), nullptr, 10));
    return request.size() >= end + 4 + body;
  }

  // Refuses names other than loopback while bound to loopback, so a web page
  // cannot reach this server through DNS rebinding.
  bool hostAllowed(const std::string& request) const {
    if (!loopback_only_) return true;
    std::string host = headerValue(request, "host");
    if (host.empty()) return false;
    const auto colon = host.rfind(':');
    if (colon != std::string::npos) host.resize(colon);
    return host == "localhost" || host == "127.0.0.1";
  }

  static std::string response(const char* status, const char* type, std::string_view body) {
    std::string out = std::string("HTTP/1.1 ") + status + "\r\nContent-Type: " + type +
                      "\r\nContent-Length: " + std::to_string(body.size()) +
                      "\r\nCache-Control: no-store\r\nX-Content-Type-Options: nosniff"
                      "\r\nConnection: close\r\n\r\n";
    out.append(body);
    return out;
  }

  void handleRequest(int fd, Client& client) {
    client.close_after_flush = true;
    const auto line_end = client.request.find("\r\n");
    const std::string line = client.request.substr(0, line_end);
    if (!hostAllowed(client.request)) {
      client.output = response("403 Forbidden", "text/plain", "Forbidden\n");
      return;
    }
    if (line.rfind("POST /select ", 0) == 0) {
      handleSelect(client);
      return;
    }
    if (line.rfind("GET ", 0) != 0) {
      client.output = response("405 Method Not Allowed", "text/plain", "Method not allowed\n");
      return;
    }
    const auto path_end = line.find(' ', 4);
    const std::string path = line.substr(4, path_end == std::string::npos ? std::string::npos : path_end - 4);
    if (path == "/") {
      client.output = response("200 OK", "text/html; charset=utf-8", page_);
    } else if (path == "/events") {
      client.output =
          "HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\nCache-Control: no-store\r\n"
          "X-Content-Type-Options: nosniff\r\nConnection: keep-alive\r\n\r\n";
      client.stream = true;
      client.close_after_flush = false;
      ++streams_;
      new_streams_.push_back(fd);
    } else {
      client.output = response("404 Not Found", "text/plain", "Not found\n");
    }
  }

  // Only a page on this computer may change what is shown. The custom header
  // and the Origin check keep other web sites from sending this request.
  void handleSelect(Client& client) {
    const std::string& request = client.request;
    const std::string host = headerValue(request, "host");
    if (!client.local || headerValue(request, "x-gnss-visualizer") != "select" ||
        headerValue(request, "origin") != "http://" + host) {
      client.output = response("403 Forbidden", "text/plain", "Forbidden\n");
      return;
    }
    const std::string body = request.substr(request.find("\r\n\r\n") + 4);
    const auto split = body.find('\n');
    if (split == std::string::npos) {
      client.output = response("400 Bad Request", "text/plain", "Bad request\n");
      return;
    }
    selections_.emplace_back(body.substr(0, split), body.substr(split + 1));
    client.output = response("204 No Content", "text/plain", "");
  }

  void remove(std::map<int, Client>::iterator it) {
    if (it->second.stream) --streams_;
    close(it->first);
    clients_.erase(it);
  }

  void run() {
    while (running_) {
      std::vector<pollfd> fds;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        fds.reserve(clients_.size() + 2);
        fds.push_back({wake_[0], POLLIN, 0});
        fds.push_back({listen_fd_, POLLIN, 0});
        for (const auto& [fd, client] : clients_)
          fds.push_back({fd, static_cast<short>(POLLIN | (client.output.empty() ? 0 : POLLOUT)), 0});
      }
      if (poll(fds.data(), fds.size(), 1000) < 0 && errno != EINTR) break;

      if (fds[0].revents & POLLIN) {
        char buffer[256];
        while (read(wake_[0], buffer, sizeof(buffer)) > 0) {}
      }
      std::lock_guard<std::mutex> lock(mutex_);
      if (fds[1].revents & POLLIN) {
        while (true) {
          sockaddr_in peer{};
          socklen_t peer_size = sizeof(peer);
          const int fd = accept4(listen_fd_, reinterpret_cast<sockaddr*>(&peer), &peer_size, SOCK_NONBLOCK | SOCK_CLOEXEC);
          if (fd < 0) break;
          if (clients_.size() >= kMaxClients) {
            close(fd);
            continue;
          }
          clients_[fd].connected = std::chrono::steady_clock::now();
          clients_[fd].local = (ntohl(peer.sin_addr.s_addr) >> 24) == 127;
        }
      }
      for (size_t i = 2; i < fds.size(); ++i) {
        const auto it = clients_.find(fds[i].fd);
        if (it == clients_.end()) continue;
        Client& client = it->second;
        bool closed = (fds[i].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0;
        if (!closed && (fds[i].revents & POLLIN)) {
          char buffer[2048];
          const ssize_t n = recv(fds[i].fd, buffer, sizeof(buffer), 0);
          if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
            closed = true;
          } else if (n > 0 && !client.stream && !client.close_after_flush) {
            client.request.append(buffer, static_cast<size_t>(n));
            if (requestComplete(client.request)) handleRequest(fds[i].fd, client);
            else if (client.request.size() > kMaxRequestBytes) closed = true;
          }
        }
        if (!closed && !client.output.empty() && (fds[i].revents & POLLOUT)) {
          const ssize_t n = send(fds[i].fd, client.output.data(), client.output.size(), MSG_NOSIGNAL);
          if (n > 0) client.output.erase(0, static_cast<size_t>(n));
          else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) closed = true;
        }
        const bool timed_out = !client.stream && !client.close_after_flush &&
            std::chrono::steady_clock::now() - client.connected > kRequestTimeout;
        if (closed || client.drop || timed_out || (client.close_after_flush && client.output.empty()))
          remove(it);
      }
    }
  }

  std::string address_;
  uint16_t port_;
  std::string page_;
  bool loopback_only_{true};
  int listen_fd_{-1};
  int wake_[2]{-1, -1};
  std::atomic<bool> running_{true};
  std::atomic<size_t> streams_{0};
  std::mutex mutex_;
  std::map<int, Client> clients_;
  std::vector<int> new_streams_;
  std::vector<Selection> selections_;
  std::thread thread_;
};

}  // namespace gnss_visualizer
