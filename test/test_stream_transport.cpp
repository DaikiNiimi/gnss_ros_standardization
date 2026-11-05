#include <gtest/gtest.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include "gnss_ros_standardization/decoder_common.hpp"

using namespace gnss_ros_standardization;
namespace {
// One-threaded fake caster: RTKLIB progresses on read(), while this fixture
// accepts the request and supplies independently chosen response bytes.
struct Caster {
  int listener{-1},peer{-1}; stream_t stream{}; std::string request;
  Caster() { strinit(&stream); }
  ~Caster() { strclose(&stream); if(peer>=0) ::close(peer); if(listener>=0) ::close(listener); }
  bool start() {
    listener=::socket(AF_INET,SOCK_STREAM,0); if(listener<0) return false;
    sockaddr_in a{}; a.sin_family=AF_INET; a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    if(::bind(listener,reinterpret_cast<sockaddr*>(&a),sizeof(a)) || ::listen(listener,2)) return false;
    socklen_t n=sizeof(a); if(::getsockname(listener,reinterpret_cast<sockaddr*>(&a),&n)) return false;
    ::fcntl(listener,F_SETFL,O_NONBLOCK);
    const auto path="127.0.0.1:"+std::to_string(ntohs(a.sin_port))+"/TEST";
    if(!stropen(&stream,STR_NTRIPCLI,STR_MODE_R,path.c_str())) return false;
    strsettimeout(&stream,0,100);
    return handshake();
  }
  bool handshake() {
    request.clear(); const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(3);
    while(std::chrono::steady_clock::now()<end) {
      uint8_t b[1024]; strread(&stream,b,sizeof(b));
      if(peer<0) { peer=::accept(listener,nullptr,nullptr); if(peer>=0) ::fcntl(peer,F_SETFL,O_NONBLOCK); }
      if(peer>=0) {
        const auto n=::recv(peer,b,sizeof(b),MSG_DONTWAIT);
        if(n>0) request.append(reinterpret_cast<char*>(b),n);
        if(request.find("\r\n\r\n")!=std::string::npos) return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
  }
  std::string exchange(const std::string& response,size_t fragment=4096) {
    std::string out; size_t sent=0;
    const auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(500);
    while(std::chrono::steady_clock::now()<end) {
      if(sent<response.size()) {
        const auto n=::send(peer,response.data()+sent,std::min(fragment,response.size()-sent),MSG_NOSIGNAL|MSG_DONTWAIT);
        if(n>0) sent+=n;
      }
      uint8_t b[4096]; const int n=strread(&stream,b,sizeof(b));
      if(n>0) out.append(reinterpret_cast<char*>(b),n);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return out;
  }
};
}

TEST(StreamTransport, FragmentedIcyPreservesBinaryPayload) {
  Caster c; ASSERT_TRUE(c.start());
  EXPECT_NE(c.request.find("GET /TEST HTTP/1.0\r\n"),std::string::npos);
  const std::string payload("\xd3\0\xff\x80\r\n",6);
  EXPECT_EQ(c.exchange("ICY 200 OK\r\n"+payload,1),payload);
}

TEST(StreamTransport, RejectedAndFullResponsesRecoverWithNewConnection) {
  for(const std::string& response : {
        std::string("HTTP/1.0 401 Unauthorized\r\n\r\n"),
        std::string("HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n"),
        std::string("SOURCETABLE 200 OK\r\n"),
        std::string(32767,'x'),
        std::string("HTTP/1.0 401 ")+std::string(12000,'x')+"\r\n"}) {
    Caster c; ASSERT_TRUE(c.start()); EXPECT_TRUE(c.exchange(response).empty());
    ::close(c.peer); c.peer=-1;
    // No inactivity timeout: this specifically tests response rejection rather
    // than a timeout eventually rescuing a stuck full response buffer.
    ASSERT_TRUE(c.handshake()) << response.substr(0,32);
    EXPECT_EQ(c.exchange("ICY 200 OK\r\nnew"),"new");
  }
}

TEST(StreamTransport, InvalidUrisAreRejectedBeforeBackendOpen) {
  stream_t s{}; const auto log=rclcpp::get_logger("transport_test");
  for(const auto& path : {std::string("ntrips://host/mount"),std::string("ntrip://[::1]/mount"),
       std::string("tcpcli://"),std::string("tcpcli://")+std::string(MAXSTRPATH,'x'),
       std::string("tcpcli://host\r\nINJECT")})
    EXPECT_FALSE(decoder_common::openStreamPath(log,s,path,STR_MODE_R,"test"));
}
