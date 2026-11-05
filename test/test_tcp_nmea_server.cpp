#include <gtest/gtest.h>
#include "gnss_ros_standardization/tcp_nmea_server.hpp"
#include <functional>

namespace {
using Clock = std::chrono::steady_clock;
bool waitFor(const std::function<bool()>& ready) {
  const auto end = Clock::now() + std::chrono::seconds(2);
  while (Clock::now() < end) {
    if (ready()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return ready();
}
struct Socket {
  int fd{-1};
  ~Socket() { if (fd >= 0) ::close(fd); }
};
int freePort() {
  Socket s{::socket(AF_INET, SOCK_STREAM, 0)};
  sockaddr_in a{}; a.sin_family=AF_INET; a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
  if (::bind(s.fd,reinterpret_cast<sockaddr*>(&a),sizeof(a))) return -1;
  socklen_t n=sizeof(a);
  if (::getsockname(s.fd,reinterpret_cast<sockaddr*>(&a),&n)) return -1;
  return ntohs(a.sin_port);
}
void connectClient(Socket& s,int port) {
  s.fd=::socket(AF_INET,SOCK_STREAM,0);
  int window=1024; ::setsockopt(s.fd,SOL_SOCKET,SO_RCVBUF,&window,sizeof(window));
  timeval timeout{3,0}; ::setsockopt(s.fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
  sockaddr_in a{}; a.sin_family=AF_INET; a.sin_addr.s_addr=htonl(INADDR_LOOPBACK); a.sin_port=htons(port);
  ASSERT_EQ(::connect(s.fd,reinterpret_cast<sockaddr*>(&a),sizeof(a)),0);
}
std::string receive(int fd,size_t wanted) {
  std::string result; char bytes[4096];
  while(result.size()<wanted) {
    const auto n=::recv(fd,bytes,std::min(sizeof(bytes),wanted-result.size()),0);
    if(n<=0) break;
    result.append(bytes,n);
  }
  return result;
}
}

TEST(TcpNmeaServer, OrderedBytesSurviveSmallReceiveWindow) {
  gnss_utils::TcpNmeaServer server; const int port=freePort(); ASSERT_GT(port,0);
  ASSERT_TRUE(server.start(port,rclcpp::get_logger("nmea_test")));
  Socket client; connectClient(client,port); ASSERT_TRUE(waitFor([&]{return server.hasClients();}));
  std::string payload;
  for(size_t i=0;i<60*1024;++i) payload.push_back(static_cast<char>(i%251));
  server.send(payload.data(),payload.size());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(receive(client.fd,payload.size()),payload);
  const std::string next="$GPGGA,following\r\n";
  server.send(next.data(),next.size()); EXPECT_EQ(receive(client.fd,next.size()),next);
}

TEST(TcpNmeaServer, SlowClientCannotBlockProducerOrReconnect) {
  gnss_utils::TcpNmeaServer server; const int port=freePort(); ASSERT_GT(port,0);
  ASSERT_TRUE(server.start(port,rclcpp::get_logger("nmea_test")));
  Socket slow; connectClient(slow,port); ASSERT_TRUE(waitFor([&]{return server.hasClients();}));
  std::string payload(32*1024,'x'); const auto start=Clock::now();
  for(int i=0;i<500 && server.hasClients();++i) {
    server.send(payload.data(),payload.size());
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_FALSE(server.hasClients());
  EXPECT_LT(Clock::now()-start,std::chrono::seconds(2));
  Socket fresh; connectClient(fresh,port); ASSERT_TRUE(waitFor([&]{return server.hasClients();}));
  server.send("new\r\n",5); EXPECT_EQ(receive(fresh.fd,5),"new\r\n");
  const auto stop=Clock::now(); server.stop();
  EXPECT_LT(Clock::now()-stop,std::chrono::seconds(1));
  server.stop();
  ASSERT_TRUE(server.start(port,rclcpp::get_logger("nmea_test")));
  server.stop();
}

TEST(TcpNmeaServer, ValidatesPortAndDoesNotReplaceActiveListener) {
  gnss_utils::TcpNmeaServer server; auto log=rclcpp::get_logger("nmea_test");
  EXPECT_FALSE(server.start(0,log)); EXPECT_FALSE(server.start(65536,log));
  const int port=freePort(); ASSERT_GT(port,0); ASSERT_TRUE(server.start(port,log));
  EXPECT_FALSE(server.start(port,log));
  Socket client; connectClient(client,port); ASSERT_TRUE(waitFor([&]{return server.hasClients();}));
  ::close(client.fd); client.fd=-1;
  EXPECT_TRUE(waitFor([&]{return !server.hasClients();}));
}
