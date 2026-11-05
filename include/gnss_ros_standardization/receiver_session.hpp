#pragma once
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <optional>
#include <stdexcept>
#include "gnss_ros_standardization/binary_framer.hpp"
#include "gnss_ros_standardization/rtklib_compat.hpp"

namespace gnss_ros_standardization::wire {
// Silence/EOF alone is not a new session. RTKLIB reports 2/3 for an established
// connection, 0/1/-1 for closed/wait/error. Observe after every read, before bytes
// reach the framer. This also catches a disconnect discovered by strread itself.
class StreamSession {
 public:
  using Clock=std::chrono::steady_clock;
  bool poll(stream_t& stream, Clock::time_point now=Clock::now()) {
    const int state=strstat(&stream,nullptr);
    const bool boundary=lost(state);
    if (stream.type==STR_SERIAL && state<0 && now>=retry_) {
      const std::string path(stream.path); const int mode=stream.mode;
      strclose(&stream);
      stropen(&stream,STR_SERIAL,mode,path.c_str());
      retry_=now+std::chrono::seconds(1);
    }
    return boundary;
  }
  bool lost(int state) {
    const bool connected=state>=2;
    const bool changed=connected_ && !connected;
    connected_=connected;
    return changed;
  }
 private:
  bool connected_{false};
  std::chrono::steady_clock::time_point retry_{};
};

// A weekless PVT/IMU must not date itself from an arbitrarily old decoder time.
// Only explicitly dated observations or validated UTC PVT refresh this reference.
class FreshGnssReference {
 public:
  using Clock=std::chrono::steady_clock;
  void update(gtime_t time, Clock::time_point now=Clock::now()) {
    time_=time; arrival_=now;
  }
  gtime_t get(Clock::time_point now=Clock::now()) const {
    return time_.time && now>=arrival_ && now-arrival_<=std::chrono::seconds(5) ? time_ : gtime_t{};
  }
  void reset() { time_={}; }
 private:
  gtime_t time_{};
  Clock::time_point arrival_{};
};

inline void resetRaw(raw_t& raw, int format) {
  const std::string options(raw.opt);
  free_raw(&raw);
  raw={};
  if (!init_raw(&raw,format)) throw std::runtime_error("receiver reset: init_raw failed");
  std::strncpy(raw.opt,options.c_str(),sizeof(raw.opt)-1);
}

// Watch primary observation and PVT streams separately: optional/late extras
// must not trigger resets. A rewind > 1 ms begins a new input timeline. Normal
// GPS week rollover is forward in absolute time; UBX PVT's weekless iTOW uses
// the nearest-week rule. A silent reboot with identical timestamps is not
// distinguishable without a connection change or receiver reset indication.
class ReceiverTimeline {
 public:
  bool observe(double epoch, unsigned lane=0, bool weekless=false) {
    if (!std::isfinite(epoch)) return false;
    bool rewind=false;
    if (epochs_[lane]) {
      double delta=epoch-*epochs_[lane];
      if (weekless && delta < -302400.) delta+=604800.;
      rewind=delta < -.001001;
    }
    epochs_[lane]=epoch;
    return rewind;
  }
  void reset() { epochs_={}; }
  bool frame(Protocol protocol,const Frame& f) {
    const auto& p=f.payload;
    double tow=0; int week=0; unsigned lane=0; bool weekless=false,clock_reset=false;
    if (protocol==Protocol::Ubx) {
      if (f.cls==2 && f.id==0x15 && p.size()>=16 && p[13]<=1 && p.size()>=16u+32u*p[11]) {
        std::memcpy(&tow,p.data(),8); week=u16(p.data()+8); clock_reset=(p[12]&2)!=0;
      } else if (f.cls==2 && f.id==0x10 && p.size()>=8 && p.size()>=8u+24u*p[6]) {
        tow=u32(p.data())*.001; week=u16(p.data()+4);
      } else if (f.cls==1 && f.id==7 && (p.size()==84 || p.size()==92)) {
        tow=u32(p.data())*.001; lane=1; weekless=true;
      } else return false;
    } else if (protocol==Protocol::Sbf) {
      if (f.id==4027 && p.size()>=12) {
        if (p[9]&0x80 || p[7]<20) return false;
        size_t offset=12;
        for (unsigned row=0;row<p[6];++row) {
          if (offset+p[7]>p.size()) return false;
          const unsigned children=p[offset+19];
          if (children && p[8]<12) return false;
          offset+=p[7]+p[8]*children;
          if (offset>p.size()) return false;
        }
      } else if (f.id==4109 && f.revision==0 && p.size()>=12 && p[11]<=31 && !(p[6]&0x80)) lane=0;
      else if ((f.id==4007 || f.id==4006) && p.size()>=8) lane=1;
      else return false;
      tow=u32(p.data())*.001; week=u16(p.data()+4);
    } else {
      if (f.id==43 || f.id==140) {
        if (p.size()<4 || u32(p.data())>(p.size()-4)/(f.id==43 ? 44 : 24)) return false;
      }
      else if (f.id==42) lane=1;
      else return false;
      if (f.short_header || (f.time_status!=160 && f.time_status!=170 && f.time_status!=180)) return false;
      tow=f.tow_ms*.001; week=f.week;
    }
    if (!std::isfinite(tow) || tow<0 || tow>=604800 || (!weekless && (week<=0 || week==65535))) return false;
    return observe(weekless ? tow : week*604800.+tow,lane,weekless) || clock_reset;
  }
 private:
  std::array<std::optional<double>,2> epochs_{};
};
} // namespace gnss_ros_standardization::wire
