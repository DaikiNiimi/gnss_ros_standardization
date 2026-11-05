#include <gtest/gtest.h>
#include "gnss_ros_standardization/receiver_diagnostics.h"
#include <algorithm>
#include <iterator>
#include <cstring>
#include <memory>
#include "gnss_ros_standardization/sbf_protocol.hpp"
#include <vector>
#include "gnss_ros_standardization/binary_framer.hpp"
#include "gnss_ros_standardization/gnss_utils.hpp"

namespace {
namespace wire = gnss_ros_standardization::wire;
using Bytes = std::vector<uint8_t>;
// Binary fixture builders for the little-endian wire formats.
template<class T> void put(Bytes& b, size_t at, T value) {
  uint8_t bytes[sizeof(T)]; std::memcpy(bytes, &value, sizeof(T));
  const uint16_t endian = 1;
  for (size_t i=0; i<sizeof(T); ++i)
    b.at(at+i) = bytes[*reinterpret_cast<const uint8_t*>(&endian) ? i : sizeof(T)-1-i];
}
Bytes ubx(Bytes payload, uint8_t cls=2, uint8_t id=0x15) {
  Bytes b{0xb5,0x62,cls,id,0,0}; put<uint16_t>(b,4,payload.size());
  b.insert(b.end(),payload.begin(),payload.end()); uint8_t a=0,c=0;
  for(size_t i=2;i<b.size();++i) { a+=b[i]; c+=a; }
  b.push_back(a);b.push_back(c);return b;
}
Bytes rawx(uint8_t flags=7, uint8_t cpstd=0, double tow=100., uint8_t version=1) {
  Bytes p(48);put<double>(p,0,tow);put<uint16_t>(p,8,2300);
  p[11]=1;p[13]=version;
  put<double>(p,16,21000000.125);put<double>(p,24,-110000000.25);
  put<float>(p,32,0.f);p[37]=1;put<uint16_t>(p,40,1000);p[42]=45;
  p[44]=cpstd;p[46]=flags;return ubx(p);
}
Bytes oem(Bytes payload, uint16_t id=43) {
  Bytes b(28);b[0]=0xaa;b[1]=0x44;b[2]=0x12;b[3]=28;
  put<uint16_t>(b,4,id);put<uint16_t>(b,8,payload.size());b[13]=160;
  put<uint16_t>(b,14,2300);put<uint32_t>(b,16,100000);
  b.insert(b.end(),payload.begin(),payload.end());auto crc=wire::crc32(b.data(),b.size());
  b.resize(b.size()+4);put<uint32_t>(b,b.size()-4,crc);return b;
}
Bytes range(uint32_t flags, float doppler=-1234.f) {
  Bytes p(48);put<uint32_t>(p,0,1);put<uint16_t>(p,4,1);
  put<double>(p,8,21000000.125);put<double>(p,20,110000000.25);
  put<float>(p,32,doppler);put<float>(p,36,45);put<float>(p,40,10);
  put<uint32_t>(p,44,flags|(1u<<11));return oem(p);
}
Bytes sbf(uint16_t id, Bytes p) {
  while(p.size()%4) p.push_back(0);
  Bytes b(8);b[0]='$';b[1]='@';put<uint16_t>(b,4,id);put<uint16_t>(b,6,8+p.size());
  b.insert(b.end(),p.begin(),p.end());put<uint16_t>(b,2,wire::crc16(b.data()+4,b.size()-4));return b;
}
Bytes meas(bool missing=false, bool secondary=false, uint32_t tow=100000) {
  Bytes p(32+(secondary?12:0));put<uint32_t>(p,0,tow);put<uint16_t>(p,4,2300);
  p[6]=1;p[7]=20;p[8]=12;p[14]=1;p[15]=4;put<uint32_t>(p,16,3820130941u);
  put<int32_t>(p,20,missing?INT32_MIN:0);p[26]=missing?128:0;
  p[27]=missing?255:140;put<uint16_t>(p,28,missing?65535:10);p[31]=secondary?1:0;
  if(secondary) { p[32]=2;p[33]=10;p[34]=140;put<uint16_t>(p,42,10000); }
  return sbf(4027,p);
}
Bytes end(uint32_t tow=100000) {
  Bytes p(8);put<uint32_t>(p,0,tow);put<uint16_t>(p,4,2300);return sbf(5922,p);
}
class Decoder {
 public:
  std::unique_ptr<raw_t> raw{new raw_t{}};
  explicit Decoder(int fmt): fmt_(fmt) { if(!init_raw(raw.get(),fmt)) throw std::runtime_error("init_raw"); }
  ~Decoder(){free_raw(raw.get());}
  int feed(const Bytes& b) {
    int status=0;
    for(auto byte:b) { const int r=fmt_==STRFMT_UBX?input_ubx(raw.get(),byte):
        fmt_==STRFMT_OEM4?input_oem4(raw.get(),byte):input_sbf(raw.get(),byte);if(r)status=r; }
    return status;
  }
 private:int fmt_;
};
// R3 IGS SSR1.00: version is part of the envelope, before any state mutation.
TEST(RtcmAuxiliarySpecification, UnknownIgsVersionsCannotChangeState) {
  auto decoder=std::make_unique<rtcm_t>(); ASSERT_TRUE(init_rtcm(decoder.get()));
  decoder->time=gpst2time(2300,100);
  decoder->ssr[0].update=1; decoder->ssr[0].dclk[0]=42;
  const auto initial_time=decoder->time;
  const auto initial_ssr=std::vector<ssr_t>(decoder->ssr,decoder->ssr+MAXSAT);
  for(int base:{20,40,60,80,100,120}) for(int kind=1;kind<=7;++kind)
    for(int version:{0,2,7}) {
      Bytes f(70); f[0]=0xd3;setbitu(f.data(),14,10,64);
      setbitu(f.data(),24,12,4076);setbitu(f.data(),36,3,version);
      setbitu(f.data(),39,8,base+kind);setbitu(f.data(),47,20,500);
      setbitu(f.data(),f.size()*8-24,24,rtk_crc24q(f.data(),f.size()-3));
      int status=0;for(auto b:f){int r=input_rtcm3(decoder.get(),b);if(r)status=r;}
      EXPECT_LT(status,0) << base+kind << "/" << version;
      EXPECT_DOUBLE_EQ(timediff(decoder->time,initial_time),0);
      EXPECT_EQ(std::memcmp(decoder->ssr,initial_ssr.data(),sizeof(decoder->ssr)),0);
    }
  free_rtcm(decoder.get());
}
TEST(RtcmAuxiliarySpecification, DescriptorCountBoundariesAndShortBiases) {
  auto d=std::make_unique<rtcm_t>();ASSERT_TRUE(init_rtcm(d.get()));
  auto frame=[](Bytes payload) {
    Bytes f{0xd3,0,0};setbitu(f.data(),14,10,payload.size());
    f.insert(f.end(),payload.begin(),payload.end());const auto crc=rtk_crc24q(f.data(),f.size());
    f.resize(f.size()+3);setbitu(f.data(),f.size()*8-24,24,crc);return f;
  };
  auto feed=[&](Bytes payload){int status=0;for(auto b:frame(payload)){int r=input_rtcm3(d.get(),b);if(r)status=r;}return status;};
  for(int id:{1007,1008,1033}) for(int n:{0,31,32,63,64,255}) {
    SCOPED_TRACE(std::to_string(id)+"/"+std::to_string(n));
    Bytes p(3);setbitu(p.data(),0,12,id);setbitu(p.data(),12,12,42);
    p.push_back(n);p.insert(p.end(),n,'A');p.push_back(7);
    if(id!=1007){p.push_back(3);p.insert(p.end(),{'S','N','1'});}
    if(id==1033) for(int j=0;j<3;++j){p.push_back(3);p.insert(p.end(),{'A','B','C'});}
    ASSERT_EQ(feed(p),5);
    EXPECT_EQ(std::string(d->sta.antdes),std::string(std::min(n,MAXANT-1),'A'));
    EXPECT_EQ(d->sta.antsetup,7);
    if(id!=1007) { EXPECT_STREQ(d->sta.antsno,"SN1"); }
    if(id==1033){EXPECT_STREQ(d->sta.rectype,"ABC");EXPECT_STREQ(d->sta.recver,"ABC");EXPECT_STREQ(d->sta.recsno,"ABC");}
    const auto saved=d->sta;p.pop_back();EXPECT_LT(feed(p),0);
    EXPECT_EQ(std::memcmp(&saved,&d->sta,sizeof(saved)),0);
  }
  Bytes p(4);setbitu(p.data(),0,12,1230);setbitu(p.data(),12,12,42);
  EXPECT_EQ(feed(p),5);const auto saved=d->sta;
  setbitu(p.data(),28,4,15);EXPECT_LT(feed(p),0);
  EXPECT_EQ(std::memcmp(&saved,&d->sta,sizeof(saved)),0);
  free_rtcm(d.get());
}
TEST(RtcmAuxiliarySpecification, SsrCountsMustFitBeforeStateChanges) {
  auto d=std::make_unique<rtcm_t>();ASSERT_TRUE(init_rtcm(d.get()));
  for(int id=1057;id<=1068;++id) for(int count:{0,63}) {
    d->time=gpst2time(2300,400000);const auto before=d->time;
    Bytes f(15);f[0]=0xd3;setbitu(f.data(),14,10,9);setbitu(f.data(),24,12,id);
    const bool glo=id>=1063;const int kind=(id-1057)%6;
    const int epochbits=glo?17:20;const bool orbit=kind==0||kind==3;
    setbitu(f.data(),36,epochbits,count?100:(glo?86400:604800));
    setbitu(f.data(),36+epochbits+4+1+int(orbit)+4+16+4,6,count);
    setbitu(f.data(),f.size()*8-24,24,rtk_crc24q(f.data(),f.size()-3));
    int status=0;for(auto b:f){int r=input_rtcm3(d.get(),b);if(r)status=r;}
    EXPECT_LT(status,0)<<id;EXPECT_DOUBLE_EQ(timediff(d->time,before),0)<<id;
  }
  free_rtcm(d.get());
}
// S1 pp328-339/381/423: invalid lengths/counts must not consume stale bytes.
TEST(SbfAuxiliarySpecification, TruncatedPayloadsCannotMutateState) {
  for (auto id : {5924,5902,5926,5927,5931,5932,5933}) {
    SCOPED_TRACE(id);
    Decoder d(STRFMT_SEPT);
    Bytes p(id==5902?256:12);
    put<uint32_t>(p,0,100000);put<uint16_t>(p,4,2300);p[6]=120;
    if(id==5926)p[8]=20; // listed PRNs are missing
    if(id==5927){p[7]=2;p[10]=2;p[11]=8;}
    if(id==5931){p[7]=1;p[10]=20;}
    if(id==5932){p[7]=1;p[8]=40;}
    if(id==5933){p[9]=15;p[10]=8;}
    EXPECT_LT(d.feed(sbf(id,p)),0);
    EXPECT_EQ(d.raw->obs.tmcount,0);
    EXPECT_EQ(d.raw->nav.sbssat.nsat,0);
  }
}
TEST(SbfAuxiliarySpecification, ExternalEventRevisionTimeAndSetupBaseLength) {
  Decoder d(STRFMT_SEPT);
  for(int revision : {0,1}) {
    Bytes p(revision?24:20);put<uint32_t>(p,0,100000);put<uint16_t>(p,4,2300);
    p[6]=1;put<float>(p,8,.125f);put<double>(p,12,.0625);
    EXPECT_EQ(d.feed(sbf(5924|(revision<<13),p)),0);
    EXPECT_NEAR(timediff(d.raw->obs.data[0].eventime,gpst2time(2300,100.0625)),0.,1e-12);
  }
  Bytes setup(260);put<uint32_t>(setup,0,100000);put<uint16_t>(setup,4,2300);
  put<float>(setup,248,1.25f);put<float>(setup,252,-2.5f);put<float>(setup,256,3.75f);
  EXPECT_EQ(d.feed(sbf(5902,setup)),5);
  EXPECT_DOUBLE_EQ(d.raw->sta.del[0],-2.5);
  EXPECT_DOUBLE_EQ(d.raw->sta.del[1],3.75);
  EXPECT_DOUBLE_EQ(d.raw->sta.del[2],1.25);
}
TEST(SbfAuxiliarySpecification, OneBasedCorrectionIndexesAndVelocityAxes) {
  Decoder d(STRFMT_SEPT);
  auto header=[](size_t size) {Bytes p(size);put<uint32_t>(p,0,100000);put<uint16_t>(p,4,2300);p[6]=120;return p;};
  auto mask=header(12);mask[8]=2;mask[9]=1;mask[10]=2;
  EXPECT_EQ(d.feed(sbf(5926,mask)),3);
  auto fast=header(20);fast[7]=2;fast[10]=1;fast[11]=8;fast[12]=2;fast[13]=5;put<float>(fast,16,-1.25f);
  EXPECT_EQ(d.feed(sbf(5927,fast)),3);
  EXPECT_DOUBLE_EQ(d.raw->nav.sbssat.sat[1].fcorr.prc,-1.25);
  EXPECT_EQ(d.raw->nav.sbssat.sat[1].fcorr.udre,5);
  auto lon=header(172);lon[7]=4;lon[8]=40;
  for(int i=0;i<4;i++) {int j=12+40*i;lon[j]=1;lon[j+1]=i+1;
    put<float>(lon,j+16,1.25f);put<float>(lon,j+20,-2.5f);put<float>(lon,j+24,3.75f);put<uint32_t>(lon,j+36,100);}
  EXPECT_EQ(d.feed(sbf(5932,lon)),3);
  for(int i=0;i<4;i++) {EXPECT_DOUBLE_EQ(d.raw->nav.sbssat.sat[i].lcorr.dvel[0],1.25);
    EXPECT_DOUBLE_EQ(d.raw->nav.sbssat.sat[i].lcorr.dvel[1],-2.5);
    EXPECT_DOUBLE_EQ(d.raw->nav.sbssat.sat[i].lcorr.dvel[2],3.75);}
  auto ion=header(132);ion[9]=15;ion[10]=8;
  for(int i=0;i<15;i++) {ion[12+8*i]=i==14?201:i+1;ion[13+8*i]=3;put<float>(ion,16+8*i,float(i)+.25f);}
  EXPECT_EQ(d.feed(sbf(5933,ion)),3);
  EXPECT_DOUBLE_EQ(d.raw->nav.sbsion[0].igp[0].delay,.25);
  EXPECT_DOUBLE_EQ(d.raw->nav.sbsion[0].igp[200].delay,14.25);
  auto igp=header(16);igp[7]=1;igp[10]=2;igp[11]=1;igp[12]=2;
  EXPECT_EQ(d.feed(sbf(5931,igp)),3);
  EXPECT_NE(d.raw->nav.sbsion[0].igp[1].lat,0);
}
TEST(OemObservationSpecification, FilteredEpochDoesNotRepublishPreviousEpoch) {
  for (int mid : {43,140}) {
    Decoder d(STRFMT_OEM4);
    ASSERT_EQ(d.feed(range(0x1c00)),1);
    ASSERT_EQ(d.raw->obs.n,1);
    Bytes p(mid==43?48:28);put<uint32_t>(p,0,1);
    put<uint32_t>(p,mid==43?44:4,0x1c00u|(31u<<21));
    auto b=oem(p,mid);put<uint32_t>(b,16,101000);
    put<uint32_t>(b,b.size()-4,wire::crc32(b.data(),b.size()-4));
    EXPECT_EQ(d.feed(b),1);
    EXPECT_EQ(d.raw->obs.n,0);
  }
}
TEST(UbxTimeSpecification, SignedFractionalTimeValidityAndWeekBoundary) {
  // U2 pp386-387 /397-398: I2 week, I4 fractional nanoseconds and validity.
  for (uint8_t id : {6,32}) {
    SCOPED_TRACE(unsigned(id));
    Decoder decoder(STRFMT_UBX);
    Bytes p(id==6 ? 52 : 16);
    put<int16_t>(p,8,2300);
    p[10]=id==6 ? 3 : 18;
    const uint8_t mask=id==6 ? 12 : 3;
    p[11]=mask;
    for (int32_t fraction : {-500000,0,500000}) {
      put<uint32_t>(p,0,0); put<int32_t>(p,4,fraction);
      EXPECT_EQ(decoder.feed(ubx(p,1,id)),0);
      const int64_t epoch=315964800LL+2300LL*604800;
      // gtime_t may carry a negative fractional part; compare physical time
      // without forcing an internal normalization convention or losing ns.
      EXPECT_NEAR((decoder.raw->time.time-epoch)+decoder.raw->time.sec,fraction*1e-9,1e-10);
    }
    const gtime_t accepted=decoder.raw->time;
    put<uint32_t>(p,0,200000);
    for (uint8_t flags : {uint8_t(0),uint8_t(mask&(mask-1)),uint8_t(mask&-mask)}) {
      p[11]=flags;
      EXPECT_EQ(decoder.feed(ubx(p,1,id)),0);
      EXPECT_EQ(decoder.raw->time.time,accepted.time);
      EXPECT_EQ(decoder.raw->time.sec,accepted.sec);
    }
  }
}
TEST(UbxTimeSpecification, InvalidFieldsAndTruncationCannotReplaceAcceptedTime) {
  for (uint8_t id : {6,32}) {
    SCOPED_TRACE(unsigned(id));
    Decoder decoder(STRFMT_UBX);
    Bytes good(id==6 ? 52 : 16);
    put<uint32_t>(good,0,100000); put<int16_t>(good,8,2300);
    good[11]=id==6 ? 12 : 3;
    EXPECT_EQ(decoder.feed(ubx(good,1,id)),0);
    const auto accepted=decoder.raw->time;
    std::vector<Bytes> invalid;
    for (int16_t week : {int16_t(-1),int16_t(-32768),int16_t(0)}) {
      auto p=good;put<int16_t>(p,8,week);invalid.push_back(p);
    }
    for (uint32_t tow : {604800000u,0xffffffffu}) {
      auto p=good;put<uint32_t>(p,0,tow);invalid.push_back(p);
    }
    for (int32_t fraction : {-500001,500001,INT32_MIN,INT32_MAX}) {
      auto p=good;put<int32_t>(p,4,fraction);invalid.push_back(p);
    }
    for (size_t n=0;n<good.size();++n) {
      auto p=good;put<uint32_t>(p,0,200000);p.resize(n);invalid.push_back(p);
    }
    for (size_t n=0;n<invalid.size();++n) {
      SCOPED_TRACE(n);
      decoder.feed(ubx(invalid[n],1,id));
      EXPECT_EQ(decoder.raw->time.time,accepted.time);
      EXPECT_EQ(decoder.raw->time.sec,accepted.sec);
      EXPECT_EQ(decoder.feed(ubx(good,1,id)),0);
    }
  }
}
TEST(UbxTimeSpecification, TimemarkGpsUtcAndShortPayloadState) {
  // U2 pp429-430: timeBase=1 GPS /2 UTC, falling edge bit2, valid bit6.
  Decoder decoder(STRFMT_UBX);
  Bytes p(28);p[1]=0x4c;put<uint16_t>(p,2,7);
  put<uint16_t>(p,4,2300);put<uint16_t>(p,6,2300);
  put<uint32_t>(p,8,100000);put<uint32_t>(p,16,100001);
  put<uint32_t>(p,20,125000);put<uint32_t>(p,24,100);
  for (uint8_t flags : {0x4c,0x74}) {
    p[1]=flags;
    EXPECT_EQ(decoder.feed(ubx(p,13,3)),0);
    EXPECT_EQ(decoder.raw->obs.n,0);  // event is not a measurement epoch
    EXPECT_EQ(decoder.raw->obs.data[0].timevalid,1);
    EXPECT_EQ(decoder.raw->obs.flag,5);
    EXPECT_EQ(decoder.raw->obs.rcvcount,7);
    EXPECT_EQ(decoder.raw->obs.data[0].eventime.time,
              315964800LL+2300LL*604800+100+(flags==0x74 ? 18 : 0));
    EXPECT_NEAR(decoder.raw->obs.data[0].eventime.sec,.001125,1e-10);
  }
  const auto event=decoder.raw->obs.data[0].eventime;
  const int count=decoder.raw->obs.tmcount;
  for (size_t size=0;size<p.size();++size) {
    auto short_p=p;short_p.resize(size);
    decoder.feed(ubx(short_p,13,3));
    EXPECT_EQ(decoder.raw->obs.tmcount,count);
    EXPECT_EQ(decoder.raw->obs.data[0].eventime.time,event.time);
    EXPECT_EQ(decoder.raw->obs.data[0].eventime.sec,event.sec);
  }
}
TEST(RawObservations, SbfMeasExtraModuloCountPreservesLastRowAndRejectsTruncation) {
  // S1 FW4.15.0 pp268-269: N is modulo 256; CN0HighRes is 1/32 dB-Hz.
  for (unsigned count : {1u, 255u, 256u, 257u, 510u}) {
    SCOPED_TRACE(count);
    Decoder decoder(STRFMT_SEPT);
    EXPECT_EQ(decoder.feed(meas()), 0);
    Bytes payload(12 + 16 * count);
    put<uint32_t>(payload, 0, 100000); put<uint16_t>(payload, 4, 2300);
    payload[6] = count % 256; payload[7] = 16;
    for (unsigned row = 0; row < count; ++row) {
      payload[12 + 16 * row] = row == count - 1 ? 0 : 1 + row % 255;
      payload[12 + 16 * row + 15] = 7;
    }
    EXPECT_EQ(decoder.feed(sbf(4000 | (3 << 13), payload)), 0);
    EXPECT_EQ(decoder.feed(end()), 1);
    ASSERT_EQ(decoder.raw->obs.n, 1);
    EXPECT_DOUBLE_EQ(decoder.raw->obs.data[0].SNR[0], 45.21875);
  }
  Decoder decoder(STRFMT_SEPT);
  EXPECT_EQ(decoder.feed(meas()), 0);
  Bytes payload(28);
  put<uint32_t>(payload, 0, 100000); put<uint16_t>(payload, 4, 2300);
  payload[6] = 2; payload[7] = 16; payload[27] = 7;
  EXPECT_EQ(decoder.feed(sbf(4000 | (3 << 13), payload)), -1);
  EXPECT_EQ(decoder.feed(end()), 1);
  ASSERT_EQ(decoder.raw->obs.n, 1);
  EXPECT_DOUBLE_EQ(decoder.raw->obs.data[0].SNR[0], 45.);
}

TEST(RawObservations, UbxMulticodePreservesDataPilotSlotsAcrossRowOrder) {
  // U1 F9 HPG1.50 R01 table4 p20, RAWX pp195-197; RINEX3.04 A2.
  // Explicit code identities and distinct values, not backend-generated data.
  struct Family { int gnss,prn,sys;std::vector<std::pair<int,int>> signals; };
  const std::vector<Family> families{
    {0,1,SYS_GPS,{{0,CODE_L1C},{3,CODE_L2L},{4,CODE_L2S},{6,CODE_L5I},{7,CODE_L5Q}}},
    {2,1,SYS_GAL,{{0,CODE_L1C},{1,CODE_L1B},{3,CODE_L5I},{4,CODE_L5Q},{5,CODE_L7I},{6,CODE_L7Q}}},
    {5,1,SYS_QZS,{{0,CODE_L1C},{1,CODE_L1Z},{4,CODE_L2S},{5,CODE_L2L},{8,CODE_L5I},{9,CODE_L5Q}}},
    {3,20,SYS_CMP,{{0,CODE_L2I},{2,CODE_L7I},{5,CODE_L1P},{6,CODE_L1D},{7,CODE_L5P},{8,CODE_L5D}}}
  };
  for(const auto& family:families) {
    SCOPED_TRACE(family.gnss);Decoder decoder(STRFMT_UBX);
    std::strcpy(decoder.raw->opt,"-MULTICODE");
    for(int reverse:{0,1}) {
      const size_t count=family.signals.size();Bytes p(16+32*count);
      put<double>(p,0,100.+reverse);put<uint16_t>(p,8,2300);p[11]=count;p[13]=1;
      for(size_t row=0;row<count;++row) {
        const size_t i=reverse?count-1-row:row,at=16+32*row;
        put<double>(p,at,21000000.+i*.125);put<double>(p,at+8,100000.+i*.25);
        put<float>(p,at+16,-100.f-i*.5f);p[at+20]=family.gnss;p[at+21]=family.prn;
        p[at+22]=family.signals[i].first;put<uint16_t>(p,at+24,1000+reverse);
        p[at+26]=45;p[at+30]=7;
      }
      ASSERT_EQ(decoder.feed(ubx(p)),1);ASSERT_EQ(decoder.raw->obs.n,1);
      const auto& observation=decoder.raw->obs.data[0];size_t populated=0;
      EXPECT_EQ(observation.sat,satno(family.sys,family.prn+(family.sys==SYS_QZS?192:0)));
      for(int slot=0;slot<NFREQ+NEXOBS;++slot) if(observation.code[slot]) {
        ++populated;
        auto found=std::find_if(family.signals.begin(),family.signals.end(),
            [&](const auto& signal){return signal.second==observation.code[slot];});
        ASSERT_NE(found,family.signals.end());const auto i=std::distance(family.signals.begin(),found);
        EXPECT_DOUBLE_EQ(observation.P[slot],21000000.+i*.125);
        EXPECT_DOUBLE_EQ(observation.L[slot],100000.+i*.25);
        EXPECT_FLOAT_EQ(observation.D[slot],-100.f-i*.5f);
        EXPECT_EQ(observation.LLI[slot],0);
      }
      EXPECT_EQ(populated,count);
    }
  }
}
TEST(RawObservations, ExpandedBeidouPrnsSurviveReceiverDecoders) {
  for(int prn:{50,51,59,60,62,63}) {
    SCOPED_TRACE(prn);
    Decoder u(STRFMT_UBX),n(STRFMT_OEM4),s(STRFMT_SEPT);
    auto bytes=rawx();Bytes p(bytes.begin()+6,bytes.end()-2);
    p[36]=3;p[37]=prn; // RAWX BeiDou B1I
    ASSERT_EQ(u.feed(ubx(p)),1);ASSERT_EQ(u.raw->obs.n,1);
    EXPECT_EQ(u.raw->obs.data[0].sat,satno(SYS_CMP,prn));
    bytes=range((4u<<16)|(1u<<10)|(1u<<12));p.assign(bytes.begin()+28,bytes.end()-4);
    put<uint16_t>(p,4,prn);
    ASSERT_EQ(n.feed(oem(p)),1);ASSERT_EQ(n.raw->obs.n,1);
    EXPECT_EQ(n.raw->obs.data[0].sat,satno(SYS_CMP,prn));
    bytes=meas();p.assign(bytes.begin()+8,bytes.end());
    p[13]=28;p[14]=prn+182; // MeasEpoch B1I, extended SVID range C41..C63
    ASSERT_EQ(s.feed(sbf(4027,p)),0);ASSERT_EQ(s.feed(end()),1);
    ASSERT_EQ(s.raw->obs.n,1);EXPECT_EQ(s.raw->obs.data[0].sat,satno(SYS_CMP,prn));
    for(const auto* d:{&u,&n,&s}) {
      const auto message=gnss_utils::obsToMsg(d->raw->obs.data[0],0);
      EXPECT_EQ(message.system,"C");EXPECT_EQ(message.prn,prn);
      EXPECT_NEAR(message.p,21000000.125,.001);
    }
  }
}
TEST(RawNavigation, RtcmGlonassStatusBitsSurviveEncoderAndDecoder) {
  auto encoder=std::make_unique<rtcm_t>(); auto decoder=std::make_unique<rtcm_t>();
  ASSERT_TRUE(init_rtcm(encoder.get())); ASSERT_TRUE(init_rtcm(decoder.get()));
  std::strcpy(decoder->opt,"-EPHALL");
  for (int week : {2300,2350}) for (int flags : {0,1,4,16,32,64,128,256,511}) {
    encoder->ephsat=satno(SYS_GLO,1);
    auto& g=encoder->nav.geph[0];g={};g.sat=encoder->ephsat;
    g.toe=g.tof=gpst2time(week,18.);g.frq=-7;g.flags=flags;g.sva=7;
    g.svh=8;g.pos[0]=19000000.;
    decoder->time=encoder->time=g.toe;
    ASSERT_TRUE(gen_rtcm3(encoder.get(),1020,0,0));
    // Independent wire assertions: ln after P at payload bit 230;
    // M follows the 11-bit NT field (not its first two bits).
    EXPECT_EQ(getbitu(encoder->buff,24+230,1),1u);
    EXPECT_EQ(getbitu(encoder->buff,24+279,2),static_cast<unsigned>((flags>>7)&3));
    int status=0;
    for (int i=0;i<encoder->nbyte;++i) { int r=input_rtcm3(decoder.get(),encoder->buff[i]);if(r)status=r; }
    ASSERT_EQ(status,2);
    const auto& restored=decoder->nav.geph[0];
    EXPECT_EQ(restored.flags,flags);EXPECT_EQ(restored.sva,7);EXPECT_EQ(restored.svh,8);
    const auto msg=gnss_utils::gephToMsg(restored);
    EXPECT_EQ(msg.flags,flags);EXPECT_EQ(msg.sva,7);
  }
  free_rtcm(encoder.get());free_rtcm(decoder.get());
}
TEST(RawObservations, GlonassInvalidChannelsDoNotBecomeMinusSeven) {
  for (uint8_t encoded : {uint8_t(0), uint8_t(7), uint8_t(13), uint8_t(14), uint8_t(255)}) {
    Decoder d(STRFMT_UBX);
    Bytes p(48); put<double>(p,0,100.); put<uint16_t>(p,8,2300);
    p[11]=1; p[13]=1; put<double>(p,16,21000000.); put<double>(p,24,110000000.);
    p[36]=6; p[37]=1; p[39]=encoded; p[42]=45; p[46]=7;
    ASSERT_EQ(d.feed(ubx(p)),1);
    ASSERT_EQ(d.raw->obs.n,1);
    const auto m=gnss_utils::obsToMsg(d.raw->obs.data[0],0,&d.raw->nav);
    EXPECT_EQ(m.freq,encoded<=13 ? encoded : 255);
  }
  // SBF channel zero on the wire is unavailable (valid FCN+8 is 1..14).
  Decoder sb(STRFMT_SEPT);
  Bytes p(32);put<uint32_t>(p,0,100000);put<uint16_t>(p,4,2300);
  p[6]=1;p[7]=20;p[8]=12;p[13]=8;p[14]=38;p[15]=4;
  put<uint32_t>(p,16,3820130941u);p[27]=140;
  sb.feed(sbf(4027,p));ASSERT_EQ(sb.feed(end()),1);
  ASSERT_EQ(sb.raw->obs.n,1);
  EXPECT_EQ(gnss_utils::obsToMsg(sb.raw->obs.data[0],0,&sb.raw->nav).freq,255);
}
TEST(RawObservations, NovatelRangeCarriesItsOwnChannelWithoutNavigationCache) {
  for (unsigned channel : {0, 1, 7, 13, 14, 65535}) {
    Decoder d(STRFMT_OEM4);
    Bytes p(48); put<uint32_t>(p,0,1); put<uint16_t>(p,4,38);
    put<uint16_t>(p,6,channel);
    put<double>(p,8,21000000.125); put<double>(p,20,110000000.25);
    put<float>(p,32,0.f); put<float>(p,36,45.f); put<float>(p,40,10.f);
    put<uint32_t>(p,44,(1u<<10)|(1u<<11)|(1u<<12)|(1u<<16));
    ASSERT_EQ(d.feed(oem(p)),1); ASSERT_EQ(d.raw->obs.n,1);
    EXPECT_EQ(d.raw->nav.glo_fcn[0],channel<=13 ? channel+1 : 0);
    std::fill(std::begin(d.raw->nav.glo_fcn),std::end(d.raw->nav.glo_fcn),0);
    const auto message=gnss_utils::obsToMsg(d.raw->obs.data[0],0);
    EXPECT_EQ(message.freq,channel<=13 ? channel : 255);
    EXPECT_DOUBLE_EQ(message.p,21000000.125); EXPECT_DOUBLE_EQ(message.d,0.);
  }
}
TEST(RawNavigation, SbfCnavRatesNormalizeToRadiansForGpsAndBeidou) {
  for (int id : {4042,4251,4252,4253}) {
    const bool gps=id==4042;
    Bytes p(gps?164:(id==4253?144:152));put<uint32_t>(p,0,100000);put<uint16_t>(p,4,2300);p[6]=1;
    if (gps) put<uint16_t>(p,8,2300);
    put<uint32_t>(p,gps?12:128,400000);
    put<uint32_t>(p,gps?16:8,300);
    put<double>(p,gps?20:12,26560000.);
    put<double>(p,gps?28:20,-.125);
    put<float>(p,gps?40:32,-1e-12f);
    put<uint32_t>(p,gps?120:108,300);
    if (!gps) { p[137]=9; put<uint16_t>(p,138,1023); }
    if (gps) p[11]=static_cast<uint8_t>(-3);
    Decoder d(STRFMT_SEPT);std::strcpy(d.raw->opt,"-EPHALL");
    ASSERT_EQ(d.feed(sbf(id,p)),2) << id;
    const auto& e=d.raw->nav.eph[d.raw->ephsat-1];
    EXPECT_EQ(e.iode,(gps||id==4253)?0:9); EXPECT_EQ(e.iodc,(gps||id==4253)?0:1023);
    EXPECT_EQ(e.sva,gps?-3:-1);
    EXPECT_DOUBLE_EQ(time2gpst(e.toe,nullptr),gps?300:314);
    EXPECT_DOUBLE_EQ(e.Adot,-.125);
    EXPECT_DOUBLE_EQ(e.ndot,static_cast<double>(-1e-12f)*PI);
    const auto msg=gnss_utils::ephToMsg(e);
    EXPECT_DOUBLE_EQ(msg.adot,e.Adot);EXPECT_DOUBLE_EQ(msg.ndot,e.ndot);
  }
}
TEST(RawObservations, RawxValidityFlagsKeepIndependentComponents) {
  for(uint8_t flags:{uint8_t(7),uint8_t(6),uint8_t(5),uint8_t(4)}) {
    Decoder d(STRFMT_UBX);ASSERT_EQ(d.feed(rawx(flags)),1);ASSERT_EQ(d.raw->obs.n,1);
    const auto out=gnss_utils::obsToMsg(d.raw->obs.data[0],0);
    if(flags&1) EXPECT_DOUBLE_EQ(out.p,21000000.125);else EXPECT_TRUE(std::isnan(out.p));
    if(flags&2) EXPECT_DOUBLE_EQ(out.l,-110000000.25);else EXPECT_TRUE(std::isnan(out.l));
    EXPECT_EQ(out.d,0);EXPECT_EQ(out.snr,45);
  }
}
TEST(RawObservations, RawxPhaseQualityAndBadEpochAreRejected) {
  Decoder d(STRFMT_UBX);ASSERT_EQ(d.feed(rawx(7,15)),1);
  EXPECT_TRUE(std::isnan(d.raw->obs.data[0].L[0]));
  for(double tow:{double(NAN),double(INFINITY),-1.,604800.}) EXPECT_LT(d.feed(rawx(7,0,tow)),0);
  EXPECT_LT(d.feed(rawx(7,0,100.,2)),0);
}
TEST(RawObservations, RangeCodeLockPreservesDopplerWithoutPhaseLock) {
  Decoder d(STRFMT_OEM4);ASSERT_EQ(d.feed(range(1u<<12)),1);
  const auto out=gnss_utils::obsToMsg(d.raw->obs.data[0],0);
  EXPECT_EQ(out.p,21000000.125);EXPECT_TRUE(std::isnan(out.l));EXPECT_EQ(out.d,-1234);
  ASSERT_EQ(d.feed(range(0)),1);
  const auto invalid=gnss_utils::obsToMsg(d.raw->obs.data[0],0);
  EXPECT_TRUE(std::isnan(invalid.p));EXPECT_TRUE(std::isnan(invalid.l));EXPECT_TRUE(std::isnan(invalid.d));
  ASSERT_EQ(d.feed(range((1u<<12)|(1u<<10),0)),1);
  EXPECT_EQ(d.raw->obs.data[0].D[0],0);EXPECT_EQ(d.raw->obs.data[0].L[0],-110000000.25);
}
TEST(RawObservations, SbfMissingValuesAreNanAndZeroDopplerIsValid) {
  Decoder d(STRFMT_SEPT);EXPECT_EQ(d.feed(meas(true)),0);ASSERT_EQ(d.feed(end()),1);
  const auto out=gnss_utils::obsToMsg(d.raw->obs.data[0],0);
  EXPECT_NEAR(out.p,21000000.125,1e-6);EXPECT_TRUE(std::isnan(out.l));
  EXPECT_TRUE(std::isnan(out.d));EXPECT_TRUE(std::isnan(out.snr));
  EXPECT_EQ(d.feed(end()),0);
}
TEST(RawObservations, SbfSecondaryDopplerUsesValidZeroPrimary) {
  Decoder d(STRFMT_SEPT);EXPECT_EQ(d.feed(meas(false,true)),0);ASSERT_EQ(d.feed(end()),1);
  EXPECT_EQ(d.raw->obs.data[0].D[0],0);EXPECT_FLOAT_EQ(d.raw->obs.data[0].D[1],1);
}
TEST(RawObservations, SbfEpochBoundaryWorksInBothTimeDirections) {
  Decoder d(STRFMT_SEPT);EXPECT_EQ(d.feed(meas(false,false,101000)),0);
  ASSERT_EQ(d.feed(meas(false,false,100000)),1);int week;
  EXPECT_DOUBLE_EQ(time2gpst(d.raw->obs.data[0].time,&week),101.);
  ASSERT_EQ(d.feed(end()),1);EXPECT_DOUBLE_EQ(time2gpst(d.raw->obs.data[0].time,&week),100.);
}
TEST(RawObservations, SbfFinalEpochQuietDeadlineAndLateMarker) {
  using Completion=gnss_ros_standardization::SbfObservationCompletion;
  Completion completion;completion.configure(.2);const auto t=Completion::Clock::time_point{};
  Decoder d(STRFMT_SEPT);ASSERT_EQ(d.feed(meas()),0);completion.decoded(4027,*d.raw,t);
  EXPECT_FALSE(completion.flushIfDue(*d.raw,t+std::chrono::milliseconds(199)));
  ASSERT_TRUE(completion.flushIfDue(*d.raw,t+std::chrono::milliseconds(200)));
  EXPECT_EQ(d.raw->obs.n,1);EXPECT_EQ(d.raw->obuf.n,0);
  EXPECT_FALSE(completion.flushIfDue(*d.raw,t+std::chrono::seconds(1)));
  EXPECT_EQ(d.feed(end()),0);
  ASSERT_EQ(d.feed(meas(false,false,101000)),0);
  EXPECT_EQ(d.feed(end()),0); // delayed marker must not flush the new epoch
  ASSERT_EQ(d.feed(end(101000)),1);
  for(double bad:{0.,-1.,6.,double(NAN),double(INFINITY)}) EXPECT_THROW(completion.configure(bad),std::invalid_argument);
}
TEST(RawObservations, MalformedCountsDoNotPublishPartialEpochs) {
  for(uint16_t id:{uint16_t(43),uint16_t(140)}) {
    Decoder d(STRFMT_OEM4);Bytes p(4);put<uint32_t>(p,0,UINT32_MAX);
    EXPECT_LT(d.feed(oem(p,id)),0);EXPECT_EQ(d.raw->obs.n,0);
    EXPECT_LT(d.feed(oem({},id)),0);
  }
  Decoder d(STRFMT_SEPT);auto frame=meas();frame[14]=2; // claims a second absent Type1
  put<uint16_t>(frame,2,wire::crc16(frame.data()+4,frame.size()-4));
  EXPECT_LT(d.feed(frame),0);EXPECT_EQ(d.feed(end()),0);
  frame=meas();frame[15]=0;put<uint16_t>(frame,2,wire::crc16(frame.data()+4,frame.size()-4));
  EXPECT_LT(d.feed(frame),0);
  EXPECT_LT(d.feed(meas(false,false,604800000)),0);
}
TEST(RawObservations, RangeCmpCodeAndPhaseLockAreIndependent) {
  for(uint32_t flags:{1u<<12,0u,(1u<<12)|(1u<<10)}) {
    Bytes p(28);put<uint32_t>(p,0,1);put<uint32_t>(p,4,flags|(1u<<11));
    // GPS L1C, zero Doppler, finite pseudorange/ADR, C/N0 = 20 dB-Hz.
    p[15]=10;p[21]=1;
    Decoder d(STRFMT_OEM4);ASSERT_EQ(d.feed(oem(p,140)),1);ASSERT_EQ(d.raw->obs.n,1);
    const auto& out=d.raw->obs.data[0];
    if(flags&(1u<<12)) { EXPECT_GT(out.P[0],0);EXPECT_EQ(out.D[0],0); }
    else { EXPECT_TRUE(std::isnan(out.P[0]));EXPECT_TRUE(std::isnan(out.D[0])); }
    if(flags&(1u<<10)) EXPECT_TRUE(std::isfinite(out.L[0]));else EXPECT_TRUE(std::isnan(out.L[0]));
  }
}
TEST(RawObservations, RangeCmpPhaseUsesWireChannelRegardlessOfNavigationCache) {
  for (int mode=0;mode<3;++mode) for(unsigned channel:{0u,7u,13u,14u,63u}) {
    Decoder d(STRFMT_OEM4);
    const int sat=satno(SYS_GLO,1);const auto time=gpst2time(2300,100.);
    d.raw->nav.glo_fcn[0]=1;d.raw->nav.ng=mode;
    for(int i=0;i<mode;++i) {
      d.raw->nav.geph[i].sat=sat;d.raw->nav.geph[i].toe=time;
      d.raw->nav.geph[i].frq=i ? 6 : -7; // includes equally dated conflicting records
    }
    Bytes p(28);put<uint32_t>(p,0,1);
    put<uint32_t>(p,4,(1u<<10)|(1u<<11)|(1u<<12)|(1u<<16));
    p[15]=10;p[21]=38;p[25]=channel<<2;
    ASSERT_EQ(d.feed(oem(p,140)),1);ASSERT_EQ(d.raw->obs.n,1);
    const auto& obs=d.raw->obs.data[0];
    EXPECT_GT(obs.P[0],0.);EXPECT_EQ(obs.D[0],0.);
    EXPECT_EQ(obs.freq,channel<=13 ? channel : 255);
    EXPECT_EQ(d.raw->nav.glo_fcn[0],channel<=13 ? channel+1 : 0);
    if(channel>13) EXPECT_TRUE(std::isnan(obs.L[0]));
    else {
      const double frequency=1602000000.+(int(channel)-7)*562500.;
      const double rolls=obs.P[0]*frequency/299792458./8388608.;
      EXPECT_DOUBLE_EQ(obs.L[0],8388608.*std::floor(rolls+0.5));
      const auto message=gnss_utils::obsToMsg(obs,0,&d.raw->nav);
      EXPECT_EQ(message.freq,channel); // own channel wins over even fresh conflicting nav
    }
  }
}
TEST(RawObservations, SbfExtraBoundsAndLateExtraCannotReviveEpoch) {
  Decoder d(STRFMT_SEPT);ASSERT_EQ(d.feed(meas()),0);
  Bytes p(28);put<uint32_t>(p,0,100000);put<uint16_t>(p,4,2300);
  p[6]=1;p[7]=16;p[27]=1;
  ASSERT_EQ(d.feed(sbf(4000|(3<<13),p)),0);
  ASSERT_EQ(d.feed(end()),1);EXPECT_FLOAT_EQ(d.raw->obs.data[0].SNR[0],45.03125f);
  EXPECT_EQ(d.feed(sbf(4000|(3<<13),p)),0);EXPECT_EQ(d.feed(end()),0);
  p[7]=0;EXPECT_LT(d.feed(sbf(4000,p)),0);
  p[7]=16;p[6]=2;EXPECT_LT(d.feed(sbf(4000,p)),0);
  EXPECT_LT(d.feed(sbf(4000,Bytes(8))),0);
}
TEST(RawObservations, SbfSecondaryDnuAndZeroWeekBoundary) {
  Decoder d(STRFMT_SEPT);auto frame=meas(false,true,0);
  frame[8+33]=255; // Type2 lock time unavailable
  frame[8+35]=0x80; // Doppler offset DNU (-16, 0); code offset remains zero
  put<uint16_t>(frame,8+42,0);
  put<uint16_t>(frame,2,wire::crc16(frame.data()+4,frame.size()-4));
  ASSERT_EQ(d.feed(frame),0);ASSERT_EQ(d.feed(end(0)),1);
  const auto& obs=d.raw->obs.data[0];int week;
  EXPECT_EQ(time2gpst(obs.time,&week),0);EXPECT_EQ(week,2300);
  EXPECT_GT(obs.P[1],0);EXPECT_TRUE(std::isnan(obs.L[1]));EXPECT_TRUE(std::isnan(obs.D[1]));
}
} // namespace

TEST(RawNavigation, BeidouCnavWireLayoutAndSameIodClockUpdate) {
  for (int id : {4251,4252,4253}) {
    Bytes p(id==4253?144:152);
    put<uint32_t>(p,0,5000);put<uint16_t>(p,4,2300);p[6]=1;p[7]=3;
    put<uint32_t>(p,8,604700);put<double>(p,12,26560000.);put<double>(p,44,.01);
    put<float>(p,68,-2e-9f);put<double>(p,72,.3);put<float>(p,80,1e-10f);
    for (int i=0;i<6;++i) put<float>(p,84+4*i,float(i+1));
    put<uint32_t>(p,108,604700);put<float>(p,112,1e-20f);put<float>(p,116,1e-12f);
    put<double>(p,120,1e-5);put<uint32_t>(p,128,400000);
    p[132]=30;p[133]=19;p[134]=29;p[135]=7;p[136]=0x81;p[137]=9;put<uint16_t>(p,138,1023);
    put<float>(p,140,2e-9f);
    if(id!=4253) { put<float>(p,144,3e-9f);put<float>(p,148,4e-9f); }
    Decoder d(STRFMT_SEPT);std::strcpy(d.raw->opt,"-EPHALL");
    auto bytes=sbf(id,p);ASSERT_EQ(d.feed(bytes),2);
    auto e=d.raw->nav.eph[d.raw->ephsat-1];
    EXPECT_DOUBLE_EQ(e.OMGd,double(-2e-9f)*PI);EXPECT_DOUBLE_EQ(e.i0,.3*PI);
    EXPECT_EQ(e.cis,1);EXPECT_EQ(e.cic,2);EXPECT_EQ(e.crs,3);EXPECT_EQ(e.crc,4);EXPECT_EQ(e.cus,5);EXPECT_EQ(e.cuc,6);
    EXPECT_DOUBLE_EQ(e.f0,1e-5);EXPECT_EQ(e.week,943);EXPECT_EQ(time2gpst(e.toe,nullptr),604714);
    EXPECT_EQ(e.iode,id==4253?0:9);EXPECT_EQ(e.iodc,id==4253?0:1023);EXPECT_EQ(e.svh,0x81);
    gnss_ros_standardization::wire::Frame frame;frame.id=id;frame.payload=p;frame.bytes=bytes;
    EXPECT_FALSE(gnss_ros_standardization::supportedSbfNavigation(frame));
    put<double>(p,120,2e-5);ASSERT_EQ(d.feed(sbf(id,p)),2);
    EXPECT_DOUBLE_EQ(d.raw->nav.eph[d.raw->ephsat-1].f0,2e-5);
    EXPECT_LT(d.feed(sbf(id|(1<<13),p)),0);
    p.resize(p.size()-4);EXPECT_LT(d.feed(sbf(id,p)),0);
  }
}
TEST(RawNavigation, SbfBdsNavUsesExtendedSvidMappingAndRetainsD1D2Orbit) {
  // S1 mosaic-X5 FW4.15, 4.2.6 pp312-313, SVID table 4.1.9.
  for(int prn:{1,32,33,40,41,50,51,59,60,62,63}) {
    SCOPED_TRACE(prn);
    Bytes p(132);put<uint32_t>(p,0,100000);put<uint16_t>(p,4,2300);
    p[6]=prn+(prn<=40?140:182);put<uint16_t>(p,8,944);
    p[10]=2;p[12]=7;p[13]=8;
    put<float>(p,16,1e-9f);put<float>(p,20,-2e-9f);
    put<uint32_t>(p,24,100);put<float>(p,36,1e-5f);
    put<double>(p,60,.01);put<double>(p,72,5153.5);put<uint32_t>(p,80,100);
    put<uint16_t>(p,128,944);put<uint16_t>(p,130,944);
    Decoder d(STRFMT_SEPT);ASSERT_EQ(d.feed(sbf(4081,p)),2);
    ASSERT_EQ(d.raw->ephsat,satno(SYS_CMP,prn));
    const auto& e=d.raw->nav.eph[d.raw->ephsat-1];
    const auto m=gnss_utils::ephToMsg(e);
    EXPECT_EQ(m.system,"C");EXPECT_EQ(m.prn,prn);
    EXPECT_EQ(m.iode,8);EXPECT_EQ(m.iodc,7);
    EXPECT_DOUBLE_EQ(m.a,5153.5*5153.5);EXPECT_DOUBLE_EQ(m.e,.01);
    EXPECT_NEAR(timediff(e.toe,gpst2time(2300,114)),0,1e-9);
    p[6]=181;EXPECT_LT(d.feed(sbf(4081,p)),0); // QZSS must not masquerade as BDS.
    p[6]=246;EXPECT_LT(d.feed(sbf(4081,p)),0); // outside documented SVID range
  }
}
TEST(RawNavigation, GlonavUnitsFlagsAndRevisionAvailability) {
  for(int revision:{0,1,2,7}) {
    Bytes p(revision?92:88);put<uint32_t>(p,0,100000);put<uint16_t>(p,4,2300);p[6]=38;p[7]=1;
    put<double>(p,8,19000);put<uint32_t>(p,68,18);put<uint16_t>(p,72,2300%1024);
    p[74]=60;p[75]=1;p[77]=4;put<uint16_t>(p,78,180);p[80]=2;p[81]=3;p[82]=1;p[83]=1;
    put<uint16_t>(p,86,250);if(revision)p[88]=1;
    auto bytes=sbf(4004|(revision<<13),p);Decoder d(STRFMT_SEPT);
    ASSERT_EQ(d.feed(bytes),2);const auto& g=d.raw->nav.geph[0];
    EXPECT_EQ(g.flags,351);EXPECT_EQ(g.svh,revision?15:9);EXPECT_EQ(g.sva,2);EXPECT_EQ(g.iode,12);
    wire::Frame f;f.id=4004;f.revision=revision;f.bytes=bytes;f.payload=p;
    EXPECT_TRUE(gnss_ros_standardization::supportedSbfNavigation(f));
    auto truncated=p;truncated.resize(84);
    EXPECT_LT(d.feed(sbf(4004|(revision<<13),truncated)),0);
    put<uint16_t>(p,78,181);EXPECT_LT(d.feed(sbf(4004|(revision<<13),p)),0);
  }
}
TEST(RawNavigation, GalileoUnknownAccuracyAndSameIodUpdate) {
  Bytes p(144);put<uint32_t>(p,0,100000);put<uint16_t>(p,4,2300);p[6]=71;p[7]=2;
  put<double>(p,8,5440);put<uint32_t>(p,92,100);put<uint32_t>(p,96,100);
  put<uint16_t>(p,116,2300);put<uint16_t>(p,118,2300);put<uint16_t>(p,120,999);p[126]=255;
  Decoder d(STRFMT_SEPT);std::strcpy(d.raw->opt,"-EPHALL");ASSERT_EQ(d.feed(sbf(4002,p)),2);
  EXPECT_EQ(d.raw->nav.eph[d.raw->ephsat-1].sva,255);
  put<double>(p,108,1e-4);ASSERT_EQ(d.feed(sbf(4002,p)),2);
  EXPECT_DOUBLE_EQ(d.raw->nav.eph[d.raw->ephsat-1].f0,1e-4);
}

TEST(SbfNavigationAdmission, RejectsModernModelsEvenWithZeroPayload) {
  gnss_ros_standardization::wire::Frame frame{};
  frame.payload.resize(164, 0);
  for (auto id : {4042, 4258, 4251, 4252, 4253}) {
    frame.id = id;
    EXPECT_FALSE(gnss_ros_standardization::supportedSbfNavigation(frame));
  }
  frame.id = 4004;
  frame.payload.resize(88);
  frame.revision = 0;
  EXPECT_TRUE(gnss_ros_standardization::supportedSbfNavigation(frame));
  frame.revision = 1;
  EXPECT_FALSE(gnss_ros_standardization::supportedSbfNavigation(frame));
  frame.payload.resize(92);
  EXPECT_TRUE(gnss_ros_standardization::supportedSbfNavigation(frame));
  frame.revision = 2;
  EXPECT_TRUE(gnss_ros_standardization::supportedSbfNavigation(frame));
  frame.payload.resize(100,0xa5);frame.revision=7;
  EXPECT_TRUE(gnss_ros_standardization::supportedSbfNavigation(frame));
  frame.id = 5891;  // Legacy GPSNav continues to the existing backend checks.
  EXPECT_TRUE(gnss_ros_standardization::supportedSbfNavigation(frame));
}

namespace {
Bytes meas3(uint32_t tow=100000, bool delta=false, uint16_t week=2300) {
  Bytes p(delta ? 17 : 22); put<uint32_t>(p,0,tow); put<uint16_t>(p,4,week);
  put<uint16_t>(p,8,1); p[10]=0x20; // GPS, reference each 1000 ms
  p[12]=delta ? 0 : 1; // reused header or one-byte satellite mask
  if (delta) {
    // Short delta: unchanged pseudorange, carrier and C/N0.
    put<uint32_t>(p,13,2u|(1u<<2)|(8192u<<4)|(8192u<<18));
  } else {
    p[13]=1; // GPS PRN 1, master signal index 0 (L1C)
    put<uint32_t>(p,14,1u|(131072u<<1)|(1u<<20)|(21u<<23));
    put<uint32_t>(p,18,2000000125u); // 19e6 + 2000000.125 m
  }
  return sbf(4109,p);
}
TEST(DecoderBoundaries, Meas3QuietCompletionAndDeltaReference) {
  Decoder d(STRFMT_SEPT);
  using Completion=gnss_ros_standardization::SbfObservationCompletion;
  Completion completion;const auto t=Completion::Clock::time_point{};
  ASSERT_EQ(d.feed(meas3()),0);completion.decoded(4109,*d.raw,t);
  ASSERT_TRUE(completion.flushIfDue(*d.raw,t+std::chrono::seconds(1)));
  ASSERT_EQ(d.raw->obs.n,1);EXPECT_DOUBLE_EQ(d.raw->obs.data[0].P[0],21000000.125);
  EXPECT_EQ(d.raw->obs.data[0].freq,255);
  EXPECT_TRUE(std::isnan(d.raw->obs.data[0].D[0]));
  ASSERT_EQ(d.feed(meas3(100100,true)),0);ASSERT_EQ(d.feed(end(100100)),1);
  EXPECT_DOUBLE_EQ(d.raw->obs.data[0].P[0],21000000.125);
}
TEST(DecoderBoundaries, Meas3DeltaDoesNotRepeatReferenceSlip) {
  for (bool long_delta : {false,true}) {
    for (unsigned lti : {0u,2u}) {
      Decoder d(STRFMT_SEPT);
      ASSERT_EQ(d.feed(meas3()),0);ASSERT_EQ(d.feed(end()),1);
      auto ref=meas3(102000);
      put<uint32_t>(ref,22,1u|(131072u<<1)|(lti<<20)|(21u<<23));
      put<uint16_t>(ref,2,wire::crc16(ref.data()+4,ref.size()-4));
      ASSERT_EQ(d.feed(ref),0);ASSERT_EQ(d.feed(end(102000)),1);
      ASSERT_EQ(d.raw->obs.data[0].LLI[0],lti==0?3:1);
      for (unsigned ms : {102100u,102200u}) {
        auto delta=meas3(ms,true);
        if (long_delta) {
          Bytes p(18);put<uint32_t>(p,0,ms);put<uint16_t>(p,4,2300);
          p[8]=1;p[10]=0x20;p[13]=14|(8<<4);
          put<uint32_t>(p,14,(4u<<13)|(32768u<<16));
          p.resize(20);delta=sbf(4109,p);
        }
        ASSERT_EQ(d.feed(delta),0);ASSERT_EQ(d.feed(end(ms)),1);
        EXPECT_EQ(d.raw->obs.data[0].LLI[0],lti==0?2:0);
        EXPECT_TRUE(std::isfinite(d.raw->obs.data[0].L[0]));
      }
    }
  }
}
TEST(DecoderBoundaries, Meas3TruncatedRangesCannotUseStaleBuffer) {
  Decoder d(STRFMT_SEPT);ASSERT_EQ(d.feed(meas3()),0);ASSERT_EQ(d.feed(end()),1);
  auto frame=meas3(101000);frame.resize(24);put<uint16_t>(frame,6,frame.size());
  put<uint16_t>(frame,2,wire::crc16(frame.data()+4,frame.size()-4));
  EXPECT_LT(d.feed(frame),0);EXPECT_EQ(d.feed(end(101000)),0);
  ASSERT_EQ(d.feed(meas3(102000)),0);ASSERT_EQ(d.feed(end(102000)),1);
  EXPECT_DOUBLE_EQ(d.raw->obs.data[0].P[0],21000000.125);
}
}

namespace {
Bytes ubxMessage(uint8_t cls,uint8_t id,Bytes payload) {
  auto b=ubx(payload);b[2]=cls;b[3]=id;uint8_t a=0,c=0;
  for(size_t i=2;i<b.size()-2;i++){a+=b[i];c+=a;}b[b.size()-2]=a;b.back()=c;return b;
}
Bytes meas3Extra(uint16_t id,Bytes values,uint32_t tow=100000) {
  Bytes p(8);put<uint32_t>(p,0,tow);put<uint16_t>(p,4,2300);
  p.insert(p.end(),values.begin(),values.end());return sbf(id,p);
}
TEST(DecoderBoundaries, Meas3ExtrasAreAtomicAndSameEpochOnly) {
  Decoder d(STRFMT_SEPT);ASSERT_EQ(d.feed(meas3()),0);
  // A long (4-byte) negative rate, 2000 mm/s, and fractional C/N0.
  ASSERT_EQ(d.feed(meas3Extra(4111,{0x0f,0x7d,0,0})),0);
  EXPECT_NEAR(d.raw->obuf.data[0].D[0],2.*FREQL1/CLIGHT,1e-6);
  ASSERT_EQ(d.feed(meas3Extra(4110,{12})),0);
  EXPECT_FLOAT_EQ(d.raw->obuf.data[0].SNR[0],45.25);
  ASSERT_EQ(d.feed(meas3Extra(4110,{12})),0);
  EXPECT_FLOAT_EQ(d.raw->obuf.data[0].SNR[0],45.25);
  EXPECT_EQ(d.feed(meas3Extra(4111,{0},101000)),0);
  EXPECT_EQ(d.raw->obuf.n,1); // Wrong-epoch extra must not flush measurements.
  ASSERT_EQ(d.feed(end()),1);EXPECT_FLOAT_EQ(d.raw->obs.data[0].SNR[0],45.25);
  EXPECT_EQ(d.feed(meas3Extra(4110,{12})),0);EXPECT_EQ(d.feed(end()),0);
  ASSERT_EQ(d.feed(meas3(101000)),0);
  EXPECT_LT(d.feed(meas3Extra(4111,{},101000)),0);
  EXPECT_TRUE(std::isnan(d.raw->obuf.data[0].D[0]));
  ASSERT_EQ(d.feed(end(101000)),1);
}
TEST(DecoderBoundaries, Meas3RejectsMissingWrongWeekAndMalformedReferences) {
  Decoder d(STRFMT_SEPT);EXPECT_LT(d.feed(meas3(100100,true)),0);
  ASSERT_EQ(d.feed(meas3()),0);ASSERT_EQ(d.feed(end()),1);
  EXPECT_LT(d.feed(meas3(100100,true,2301)),0);
  for (int variant=0;variant<4;variant++) {
    auto f=meas3();
    if(variant==0)f[19]=32; // unsupported data version
    if(variant==1)f[14]=128; // scrambled
    if(variant==2)f[17]=128; // unsupported constellation bitmap
    if(variant==3)put<uint16_t>(f,4,4109|(1<<13));
    put<uint16_t>(f,2,wire::crc16(f.data()+4,f.size()-4));
    EXPECT_LT(d.feed(f),0);EXPECT_EQ(d.feed(end()),0);
  }
  ASSERT_EQ(d.feed(meas3(102000)),0);ASSERT_EQ(d.feed(end(102000)),1);
}
TEST(DecoderBoundaries, Meas3UnknownMasterCannotBecomeReferenceSignalZero) {
  Decoder d(STRFMT_SEPT);auto f=meas3();f[20]=1|(15<<3);
  put<uint16_t>(f,2,wire::crc16(f.data()+4,f.size()-4));
  EXPECT_EQ(d.feed(f),0);d.feed(end());
  EXPECT_LT(d.feed(meas3(100100,true)),0);
  ASSERT_EQ(d.feed(meas3(101000)),0);ASSERT_EQ(d.feed(end(101000)),1);
  EXPECT_EQ(d.raw->obs.data[0].code[0],CODE_L1C);
}
TEST(DecoderBoundaries, Meas3MissingCarrierStaysMissingInDelta) {
  Decoder d(STRFMT_SEPT);auto f=meas3();put<uint32_t>(f,22,1u|(1u<<20)|(21u<<23));
  put<uint16_t>(f,2,wire::crc16(f.data()+4,f.size()-4));
  ASSERT_EQ(d.feed(f),0);ASSERT_EQ(d.feed(end()),1);
  EXPECT_TRUE(std::isnan(d.raw->obs.data[0].L[0]));
  ASSERT_EQ(d.feed(meas3(100100,true)),0);ASSERT_EQ(d.feed(end(100100)),1);
  EXPECT_TRUE(std::isnan(d.raw->obs.data[0].L[0]));
}
TEST(DecoderBoundaries, UbxLegacyRawCountAndTrkD5PartialRows) {
  Decoder d(STRFMT_UBX);Bytes raw(32);put<uint32_t>(raw,0,100000);put<uint16_t>(raw,4,2300);raw[6]=1;
  put<double>(raw,8,110000000.);put<double>(raw,16,21000000.);raw[28]=1;raw[29]=6;raw[30]=45;
  ASSERT_EQ(d.feed(ubxMessage(2,0x10,raw)),1);EXPECT_DOUBLE_EQ(d.raw->obs.data[0].P[0],21000000.);
  raw.resize(28);EXPECT_LT(d.feed(ubxMessage(2,0x10,raw)),0);
  EXPECT_LT(d.feed(ubxMessage(2,0x10,{})),0);
  for (int type : {3,6}) {
    const size_t rowlen=type==6?64:56;
    Bytes trk(80+rowlen);trk[0]=type;
    put<int64_t>(trk,80,static_cast<int64_t>(99930.*4294967296.));
    trk[80+41]=6;trk[80+34]=1;trk[80+54]=8;put<uint16_t>(trk,80+32,45*256);
    if(type==6){trk[80+56]=0;trk[80+57]=1;}
    ASSERT_EQ(d.feed(ubxMessage(3,0x0a,trk)),1);ASSERT_EQ(d.raw->obs.n,1);
    EXPECT_EQ(d.raw->obs.data[0].freq,255);
    trk.pop_back();EXPECT_LT(d.feed(ubxMessage(3,0x0a,trk)),0);
  }
  EXPECT_EQ(d.feed(rawx()),1);
}
TEST(RawObservations, UbxLegacyQualityPreservesWireLliAndValidZero) {
  Decoder d(STRFMT_UBX);
  for (int quality : {-128,-1,0,3,4,5,6,127}) {
    SCOPED_TRACE(quality);
    Bytes p(32);put<int32_t>(p,0,100500);put<int16_t>(p,4,2300);p[6]=1;
    put<double>(p,8,-12345.25);put<double>(p,16,21000000.125);put<float>(p,24,-123.5f);
    p[28]=1;p[29]=static_cast<uint8_t>(quality);p[30]=45;p[31]=3;
    ASSERT_EQ(d.feed(ubxMessage(2,0x10,p)),1);ASSERT_EQ(d.raw->obs.n,1);
    const auto& o=d.raw->obs.data[0];
    if(quality<4) {EXPECT_TRUE(std::isnan(o.P[0]));EXPECT_TRUE(std::isnan(o.D[0]));}
    else {EXPECT_DOUBLE_EQ(o.P[0],21000000.125);EXPECT_FLOAT_EQ(o.D[0],-123.5f);}
    if(quality<5) EXPECT_TRUE(std::isnan(o.L[0]));else EXPECT_DOUBLE_EQ(o.L[0],-12345.25);
    EXPECT_EQ(o.LLI[0],3);EXPECT_FLOAT_EQ(o.SNR[0],45.f);
    p[29]=6;put<double>(p,8,0.);put<double>(p,16,0.);put<float>(p,24,0.f);p[30]=0;
    ASSERT_EQ(d.feed(ubxMessage(2,0x10,p)),1);
    EXPECT_DOUBLE_EQ(d.raw->obs.data[0].P[0],0.);EXPECT_DOUBLE_EQ(d.raw->obs.data[0].L[0],0.);
    EXPECT_FLOAT_EQ(d.raw->obs.data[0].D[0],0.f);EXPECT_EQ(d.raw->obs.data[0].code[0],CODE_L1C);
  }
}
TEST(RawObservations, UbxLegacyInvalidSignedWeekCannotChangeAcceptedTime) {
  Decoder d(STRFMT_UBX);
  Bytes p(32);put<int32_t>(p,0,100500);put<int16_t>(p,4,2300);p[6]=1;p[28]=1;p[29]=6;
  ASSERT_EQ(d.feed(ubxMessage(2,0x10,p)),1);
  const auto accepted=d.raw->time;
  for (int week:{0,-1,-32768}) {
    SCOPED_TRACE(week);put<int16_t>(p,4,week);
    EXPECT_LT(d.feed(ubxMessage(2,0x10,p)),0);
    EXPECT_DOUBLE_EQ(timediff(d.raw->time,accepted),0.);
  }
  put<int16_t>(p,4,2300);put<int32_t>(p,0,101500);
  ASSERT_EQ(d.feed(ubxMessage(2,0x10,p)),1);
  EXPECT_DOUBLE_EQ(timediff(d.raw->time,accepted),1.);
}
TEST(DecoderBoundaries, RawxAndOemCountsRecoverToValidObservations) {
  Decoder ub(STRFMT_UBX);auto f=rawx();Bytes p(f.begin()+6,f.end()-2);p[11]=2;
  EXPECT_LT(ub.feed(ubx(p)),0);ASSERT_EQ(ub.feed(rawx(7,0,100.,0)),1);
  for(uint16_t id:{43,140}) {
    Decoder d(STRFMT_OEM4);Bytes p(4);put<uint32_t>(p,0,1);
    EXPECT_LT(d.feed(oem(p,id)),0);ASSERT_EQ(d.feed(range((1u<<10)|(1u<<12))),1);
    EXPECT_DOUBLE_EQ(d.raw->obs.data[0].P[0],21000000.125);
  }
}
TEST(DecoderBoundaries, SbfNavicRawRequiresAllTenWords) {
  Decoder d(STRFMT_SEPT);Bytes p(44);put<uint32_t>(p,0,100000);put<uint16_t>(p,4,2300);
  p[6]=191;p[7]=1;
  EXPECT_LT(d.feed(sbf(4093,p)),0);p.resize(52);
  EXPECT_EQ(d.feed(sbf(4093,p)),0);
  wire::Frame frame{};frame.id=4254;
  EXPECT_FALSE(gnss_ros_standardization::supportedSbfNavigation(frame));
  ASSERT_EQ(d.feed(meas()),0);ASSERT_EQ(d.feed(end()),1);
}
}

namespace {
TEST(DecoderBoundaries, Meas3LongAndSlaveDeltaPreserveMeasurements) {
  Decoder d(STRFMT_SEPT);
  Bytes p(24);put<uint32_t>(p,0,100000);put<uint16_t>(p,4,2300);p[8]=1;p[10]=0x20;p[12]=1;p[13]=1;
  const uint64_t millimetres=21000000125ULL;
  put<uint32_t>(p,14,(4u<<2)|(2097152u<<6)|(1u<<28));
  put<uint32_t>(p,18,millimetres&0xffffffffu);put<uint16_t>(p,22,35|(1<<6));
  ASSERT_EQ(d.feed(sbf(4109,p)),0);ASSERT_EQ(d.feed(end()),1);
  EXPECT_DOUBLE_EQ(d.raw->obs.data[0].P[0],21000000.125);
  EXPECT_NEAR(d.raw->obs.data[0].L[0],21000000.125*FREQL1/CLIGHT,1e-6);
  // Master short with one L1W slave, then a slave delta at the next epoch.
  auto full=meas3();p=Bytes(full.begin()+8,full.begin()+30);p.resize(27);
  put<uint32_t>(p,14,1u|(131072u<<1)|(1u<<20)|(21u<<23)|(4u<<28));
  put<uint32_t>(p,22,1u|(32768u<<1)|(10000u<<17));p[26]=1;
  ASSERT_EQ(d.feed(sbf(4109,p)),0);
  ASSERT_EQ(d.feed(meas3Extra(4110,{0xc8})),0);
  EXPECT_FLOAT_EQ(d.raw->obuf.data[0].SNR[NFREQ],42.25);
  ASSERT_EQ(d.feed(end()),1);EXPECT_EQ(d.raw->obs.data[0].code[NFREQ],CODE_L1W);
  EXPECT_DOUBLE_EQ(d.raw->obs.data[0].P[NFREQ],21000000.125);
  auto delta=meas3(100100,true);p=Bytes(delta.begin()+8,delta.begin()+25);p.resize(20);
  put<uint16_t>(p,17,2|(2048<<2)|(2<<14));p[19]=128;
  ASSERT_EQ(d.feed(sbf(4109,p)),0);ASSERT_EQ(d.feed(end(100100)),1);
  EXPECT_NEAR(d.raw->obs.data[0].P[NFREQ],21000000.125,1e-6);
  EXPECT_EQ(d.raw->obs.data[0].code[NFREQ],CODE_L1W);
}
TEST(DecoderBoundaries, Meas3DeterministicMalformedPayloadsRecover) {
  Decoder d(STRFMT_SEPT);const auto good=meas3();
  for(size_t size=8;size<good.size()-8;size+=4) {
    Bytes p(good.begin()+8,good.begin()+8+size);d.feed(sbf(4109,p));d.feed(end());
    ASSERT_EQ(d.feed(meas3(101000)),0);ASSERT_EQ(d.feed(end(101000)),1);
  }
  for(size_t offset=12;offset<good.size()-8;offset++) for(uint8_t value:{uint8_t(0),uint8_t(0xff)}) {
    Bytes p(good.begin()+8,good.end());p[offset]=value;
    d.feed(sbf(4109,p));d.feed(end());
    ASSERT_EQ(d.feed(meas3(101000)),0);ASSERT_EQ(d.feed(end(101000)),1);
    ASSERT_EQ(d.raw->obs.n,1);EXPECT_DOUBLE_EQ(d.raw->obs.data[0].P[0],21000000.125);
  }
}
TEST(DecoderBoundaries, UbxTrkMeasCountAndMaximumObservationBuffer) {
  Decoder d(STRFMT_UBX);ASSERT_EQ(d.feed(rawx()),1);
  Bytes p(104+56);p[2]=1;
  p[105]=6;p[108]=0;p[109]=1;p[112]=0x20;p[120]=1;p[121]=1;
  put<uint16_t>(p,124,45*256);put<int64_t>(p,128,static_cast<int64_t>(99930.*4294967296.));
  ASSERT_EQ(d.feed(ubxMessage(3,0x10,p)),1);ASSERT_EQ(d.raw->obs.n,1);
  p[2]=2;EXPECT_LT(d.feed(ubxMessage(3,0x10,p)),0);
  const Bytes row(p.begin()+104,p.end());p.resize(104);p[2]=MAXOBS+1;
  for(int i=0;i<MAXOBS+1;i++)p.insert(p.end(),row.begin(),row.end());
  ASSERT_EQ(d.feed(ubxMessage(3,0x10,p)),1);EXPECT_EQ(d.raw->obs.n,MAXOBS);
}
}

namespace {
TEST(DecoderBoundaries, Meas3LongDeltaAndVariableRateWidths) {
  Decoder d(STRFMT_SEPT);ASSERT_EQ(d.feed(meas3()),0);ASSERT_EQ(d.feed(end()),1);
  Bytes p(18);put<uint32_t>(p,0,100100);put<uint16_t>(p,4,2300);p[8]=1;p[10]=0x20;
  p[13]=0x8e;put<uint32_t>(p,14,(4u<<13)|(32768u<<16));
  ASSERT_EQ(d.feed(sbf(4109,p)),0);ASSERT_EQ(d.feed(end(100100)),1);
  EXPECT_DOUBLE_EQ(d.raw->obs.data[0].P[0],21000000.125);
  EXPECT_NEAR(d.raw->obs.data[0].L[0],21000000.125*FREQL1/CLIGHT,1e-6);
  for(unsigned width=1;width<=4;width++)for(unsigned negative=0;negative<2;negative++) {
    ASSERT_EQ(d.feed(meas3()),0);
    unsigned shift=width==1?2:width==2?3:4;
    unsigned tag=width==1?0:width==2?2:width==3?6:14;
    uint32_t encoded=(10u<<shift)|tag|negative;
    Bytes value(width);for(unsigned i=0;i<width;i++)value[i]=(encoded>>(8*i))&255;
    ASSERT_EQ(d.feed(meas3Extra(4111,value)),0);ASSERT_EQ(d.feed(end()),1);
    EXPECT_NEAR(d.raw->obs.data[0].D[0],(negative?1.:-1.)*.01*FREQL1/CLIGHT,1e-7);
  }
  ASSERT_EQ(d.feed(meas3()),0);ASSERT_EQ(d.feed(meas3Extra(4111,{0})),0);
  ASSERT_EQ(d.feed(end()),1);EXPECT_EQ(d.raw->obs.data[0].D[0],0.);
}
TEST(DecoderBoundaries, Meas3TruncatedExtraDoesNotPartiallyModifyFirstSatellite) {
  Decoder d(STRFMT_SEPT);auto f=meas3();Bytes p(f.begin()+8,f.begin()+30);
  p[13]=3;const Bytes row(p.begin()+14,p.end());p.insert(p.end(),row.begin(),row.end());
  ASSERT_EQ(d.feed(sbf(4109,p)),0);ASSERT_EQ(d.raw->obuf.n,2);
  EXPECT_LT(d.feed(meas3Extra(4111,{0x0f,0x7d,0,0})),0);
  EXPECT_TRUE(std::isnan(d.raw->obuf.data[0].D[0]));
  EXPECT_TRUE(std::isnan(d.raw->obuf.data[1].D[0]));
  ASSERT_EQ(d.feed(end()),1);
}
TEST(DecoderBoundaries, RtcmSupportedMsmVariantsRejectShortFinalPayloads) {
  for(int sys:{SYS_GPS,SYS_GAL,SYS_GLO,SYS_CMP,SYS_QZS,SYS_SBS,SYS_IRN})for(int variant:{4,5,6,7}) {
    auto enc=std::make_unique<rtcm_t>();auto dec=std::make_unique<rtcm_t>();
    ASSERT_TRUE(init_rtcm(enc.get()));ASSERT_TRUE(init_rtcm(dec.get()));
    enc->staid=42;enc->time=dec->time=gpst2time(2300,100.);enc->obs.n=1;
    auto& o=enc->obs.data[0];o.time=enc->time;
    o.sat=satno(sys,sys==SYS_QZS?193:sys==SYS_SBS?120:1);
    o.code[0]=sys==SYS_CMP?CODE_L2I:sys==SYS_IRN?CODE_L5A:CODE_L1C;
    o.P[0]=21000000.125;o.SNR[0]=45.;enc->nav.glo_fcn[0]=8;
    const int family=sys==SYS_GPS?1070:sys==SYS_GLO?1080:sys==SYS_GAL?1090:sys==SYS_SBS?1100:sys==SYS_QZS?1110:sys==SYS_CMP?1120:1130;
    ASSERT_TRUE(gen_rtcm3(enc.get(),family+variant,0,0));
    Bytes valid(enc->buff,enc->buff+enc->nbyte);
    auto feed=[&](const Bytes& b){int status=0;for(auto byte:b){const int r=input_rtcm3(dec.get(),byte);if(r)status=r;}return status;};
    ASSERT_EQ(feed(valid),1);ASSERT_EQ(dec->obs.n,1);
    EXPECT_NEAR(dec->obs.data[0].P[0],21000000.125,.02);
    auto broken=valid;broken.resize(broken.size()-8);const unsigned length=broken.size()-6;
    broken[1]=length>>8;broken[2]=length&255;
    const auto crc=rtk_crc24q(broken.data(),broken.size()-3);
    broken[broken.size()-3]=crc>>16;broken[broken.size()-2]=crc>>8;broken.back()=crc;
    EXPECT_LT(feed(broken),0);EXPECT_EQ(dec->obs.n,0);
    EXPECT_EQ(feed(valid),1);EXPECT_EQ(dec->obs.n,1);
    free_rtcm(enc.get());free_rtcm(dec.get());
  }
}
}

namespace {
TEST(DecoderBoundaries, SbfExtrasCannotFlushOrModifyDifferentMeasurementFamily) {
  Decoder d(STRFMT_SEPT);ASSERT_EQ(d.feed(meas()),0);
  Bytes extra(28);put<uint32_t>(extra,0,99000);put<uint16_t>(extra,4,2300);extra[6]=1;extra[7]=16;extra[27]=1;
  EXPECT_EQ(d.feed(sbf(4000|(3<<13),extra)),0);EXPECT_EQ(d.raw->obuf.n,1);
  EXPECT_EQ(d.raw->obs.n,0);
  ASSERT_EQ(d.feed(end()),1);
  ASSERT_EQ(d.feed(meas3()),0);put<uint32_t>(extra,0,100000);
  EXPECT_EQ(d.feed(sbf(4000|(3<<13),extra)),0);
  EXPECT_FLOAT_EQ(d.raw->obuf.data[0].SNR[0],45.);
  // Even an invalid legacy measurement must not disarm a valid Meas3 epoch.
  auto bad=meas();bad[14]=2;put<uint16_t>(bad,2,wire::crc16(bad.data()+4,bad.size()-4));
  EXPECT_LT(d.feed(bad),0);
  ASSERT_EQ(d.feed(meas3Extra(4110,{12})),0);
  EXPECT_FLOAT_EQ(d.raw->obuf.data[0].SNR[0],45.25);
  ASSERT_EQ(d.feed(end()),1);
}
}

namespace {
TEST(DecoderBoundaries, MalformedMeas3DoesNotAlterAcceptedLockHistory) {
  Decoder d(STRFMT_SEPT);auto good=meas3();put<uint32_t>(good,22,1u|(131072u<<1)|(4u<<20)|(21u<<23));
  put<uint16_t>(good,2,wire::crc16(good.data()+4,good.size()-4));
  ASSERT_EQ(d.feed(good),0);ASSERT_EQ(d.feed(end()),1);
  auto bad=meas3();bad[21]=3; // claims a second satellite after changing the first lock time
  put<uint16_t>(bad,2,wire::crc16(bad.data()+4,bad.size()-4));
  EXPECT_LT(d.feed(bad),0);
  ASSERT_EQ(d.feed(good),0);ASSERT_EQ(d.feed(end()),1);
  EXPECT_EQ(d.raw->obs.data[0].LLI[0]&LLI_SLIP,0);
}
}

#include "gnss_ros_standardization/receiver_session.hpp"
TEST(DecoderBoundaries, SessionResetRetiresBackendCachesAndPreservesDecodeOptions) {
  for(int format:{STRFMT_UBX,STRFMT_SEPT,STRFMT_OEM4}) {
    Decoder decoder(format);auto& raw=*decoder.raw;
    std::strcpy(raw.opt,"-EPHALL");
    for(int repeat=0;repeat<4;repeat++) {
      raw.time=gpst2time(2300,100.);raw.nav.glo_fcn[0]=8;
      raw.nav.geph[0].sat=satno(SYS_GLO,1);raw.nav.geph[0].toe=raw.time;
      raw.lockt[0][0]=100;raw.halfc[0][0]=1;raw.obs.n=1;raw.obuf.n=1;
      gnss_ros_standardization::wire::resetRaw(raw,format);
      EXPECT_EQ(raw.time.time,0);EXPECT_EQ(raw.nav.glo_fcn[0],0);
      EXPECT_EQ(raw.nav.geph[0].sat,0);EXPECT_DOUBLE_EQ(raw.lockt[0][0],0);
      EXPECT_EQ(raw.halfc[0][0],0);EXPECT_EQ(raw.obs.n,0);EXPECT_EQ(raw.obuf.n,0);
      EXPECT_STREQ(raw.opt,"-EPHALL");
    }
  }
}

TEST(RawObservations, MeasEpochHalfCycleSurvivesMissingPhaseAndLock) {
  Decoder d(STRFMT_SEPT);
  // Both master and slave have explicit half-cycle ambiguity, unavailable lock/phase.
  Bytes p(44);put<uint32_t>(p,0,100000);put<uint16_t>(p,4,2300);
  p[6]=1;p[7]=20;p[8]=12;p[14]=1;p[15]=4;put<uint32_t>(p,16,3820130941u);
  p[26]=128;p[27]=140;put<uint16_t>(p,28,65535);p[30]=4;p[31]=1;
  p[32]=2;p[33]=255;p[34]=180;p[36]=128;p[37]=4;
  d.feed(sbf(4027,p));ASSERT_EQ(d.feed(end()),1);ASSERT_EQ(d.raw->obs.n,1);
  for(int slot:{0,1}) {
    EXPECT_TRUE(std::isnan(d.raw->obs.data[0].L[slot]));
    EXPECT_EQ(d.raw->obs.data[0].LLI[slot],LLI_HALFC);
  }
}
TEST(DecoderBoundaries, RawxCapacityBoundaryDoesNotCorruptNextEpoch) {
  for(unsigned count:{95u,96u,97u}) {
    Decoder d(STRFMT_UBX);Bytes p(16+32*count);
    gnss_receiver_diagnostics health{};wire::ReceiverDiagnosticScope scope(health);
    put<double>(p,0,100.);put<uint16_t>(p,8,2300);p[11]=count;p[13]=1;
    for(unsigned i=0;i<count;++i) {
      const auto at=16+32*i;
      put<double>(p,at,21000000.+i);put<double>(p,at+8,110000000.);
      p[at+20]=i<32 ? 0 : i<95 ? 3 : 2;
      p[at+21]=i<32 ? i+1 : i<95 ? i-31 : i-94;
      p[at+26]=45;p[at+30]=7;
    }
    ASSERT_EQ(d.feed(ubx(p)),1);ASSERT_EQ(d.raw->obs.n,std::min<int>(count,MAXOBS));
    EXPECT_EQ(health.counts[GNSS_RX_CAPACITY_ROWS],count>MAXOBS ? count-MAXOBS : 0u);
    for(int i=0;i<d.raw->obs.n;++i) EXPECT_DOUBLE_EQ(d.raw->obs.data[i].P[0],21000000.+i);
    ASSERT_EQ(d.feed(rawx(7,0,101.)),1);EXPECT_EQ(d.raw->obs.n,1);
  }
}

TEST(ReceiverDiagnostics, ObservationExclusionsComeFromActualFilterSites) {
  for(int format:{STRFMT_UBX,STRFMT_OEM4,STRFMT_SEPT}) {
    SCOPED_TRACE(format);
    Decoder decoder(format);gnss_receiver_diagnostics health{};
    wire::ReceiverDiagnosticScope scope(health);
    if(format==STRFMT_UBX) {
      std::strcpy(decoder.raw->opt,"-MULTICODE");
      auto b=rawx();Bytes p(b.begin()+6,b.end()-2);
      p[37]=64;decoder.feed(ubx(p)); // invalid GPS satellite
      p[37]=1;p[38]=255;decoder.feed(ubx(p)); // unmapped signal
      p[38]=0;ASSERT_EQ(decoder.feed(ubx(p)),1);
    } else if(format==STRFMT_OEM4) {
      auto b=range((1u<<10)|(1u<<12));Bytes p(b.begin()+28,b.end()-4);
      put<uint16_t>(p,4,64);decoder.feed(oem(p));
      put<uint16_t>(p,4,1);put<uint32_t>(p,44,(31u<<21)|(1u<<10)|(1u<<11)|(1u<<12));
      decoder.feed(oem(p));
      put<uint32_t>(p,44,(1u<<10)|(1u<<11)|(1u<<12));ASSERT_EQ(decoder.feed(oem(p)),1);
    } else {
      auto b=meas();Bytes p(b.begin()+8,b.end());
      p[14]=0;decoder.feed(sbf(4027,p));
      p[14]=1;p[13]=31;p[30]=0;decoder.feed(sbf(4027,p)); // extension32 is Galileo E1 for GPS
      p[13]=0;decoder.feed(sbf(4027,p));ASSERT_EQ(decoder.feed(end()),1);
    }
    EXPECT_EQ(health.counts[GNSS_RX_SATELLITE],1u);
    EXPECT_EQ(health.counts[GNSS_RX_SIGNAL],1u);
    EXPECT_EQ(health.counts[GNSS_RX_CAPACITY_ROWS],0u);
    EXPECT_EQ(health.counts[GNSS_RX_VALIDATION],0u);
    ASSERT_EQ(decoder.raw->obs.n,1);
    EXPECT_EQ(decoder.raw->obs.data[0].sat,satno(SYS_GPS,1));
  }
}
TEST(ReceiverDiagnostics, RejectionSiteAndVersionDifferFromNavigationAssembly) {
  Decoder d(STRFMT_UBX);gnss_receiver_diagnostics health{};
  wire::ReceiverDiagnosticScope scope(health);
  EXPECT_EQ(d.feed(rawx(7,0,100.,2)),-1);
  EXPECT_EQ(health.counts[GNSS_RX_VERSION],1u);
  EXPECT_EQ(health.counts[GNSS_RX_VALIDATION],1u);
  EXPECT_STREQ(health.functions[GNSS_RX_VALIDATION],"decode_rxmrawx");
  // GPS subframe1 is stored, but ephemeris needs later subframes.
  Bytes p(48);p[1]=1;p[4]=10;p[6]=2;
  put<uint32_t>(p,12,1u<<8); // second word: subframe ID bits43..45
  EXPECT_EQ(d.feed(ubxMessage(2,0x13,p)),0);
  EXPECT_EQ(health.counts[GNSS_RX_NAV_WAIT],1u);
  EXPECT_EQ(health.counts[GNSS_RX_NAV_NO_OUTPUT],0u);
  EXPECT_EQ(d.feed(rawx()),1);
}

TEST(ReceiverDiagnostics, RtcmInvalidGlonassSatelliteDoesNotWriteChannelCache) {
  auto encoder=std::make_unique<rtcm_t>();auto decoder=std::make_unique<rtcm_t>();
  ASSERT_TRUE(init_rtcm(encoder.get()));ASSERT_TRUE(init_rtcm(decoder.get()));
  encoder->time=decoder->time=gpst2time(2300,100.);encoder->staid=42;
  encoder->obs.n=1;auto& obs=encoder->obs.data[0];
  obs.sat=satno(SYS_GLO,1);obs.time=encoder->time;obs.code[0]=CODE_L1C;
  obs.P[0]=21000000.;obs.L[0]=1e8;obs.SNR[0]=45;encoder->nav.glo_fcn[0]=8;
  ASSERT_TRUE(gen_rtcm3(encoder.get(),1087,0,0));
  const Bytes valid(encoder->buff,encoder->buff+encoder->nbyte);
  Bytes invalid=valid;
  setbitu(invalid.data(),97,32,0);setbitu(invalid.data(),129,32,1); // GLO slot64
  auto crc=rtk_crc24q(invalid.data(),invalid.size()-3);
  setbitu(invalid.data(),(invalid.size()-3)*8,24,crc);
  const auto* nav_begin=reinterpret_cast<const uint8_t*>(&decoder->nav);
  const Bytes before(nav_begin,nav_begin+sizeof(decoder->nav));
  gnss_receiver_diagnostics health{};
  {
    wire::ReceiverDiagnosticScope scope(health);
    for(auto byte:invalid) input_rtcm3(decoder.get(),byte);
  }
  EXPECT_EQ(std::memcmp(before.data(),&decoder->nav,before.size()),0);
  EXPECT_EQ(decoder->obs.n,0);
  EXPECT_EQ(health.counts[GNSS_RX_SATELLITE],1u);
  for(auto byte:valid) input_rtcm3(decoder.get(),byte);
  EXPECT_EQ(decoder->obs.n,1);EXPECT_EQ(decoder->nav.glo_fcn[0],8);
  // A full epoch rejects insertion of a new satellite without claiming that
  // every full epoch necessarily loses data. The next epoch can recover.
  decoder->obs.n=MAXOBS;decoder->obsflag=0;
  const int excluded=satno(SYS_GLO,1);
  for(int i=0;i<MAXOBS;++i) {
    decoder->obs.data[i]={};decoder->obs.data[i].time=decoder->time;
    decoder->obs.data[i].sat=i+1<excluded ? i+1 : i+2;
  }
  {
    wire::ReceiverDiagnosticScope scope(health);
    for(auto byte:valid) input_rtcm3(decoder.get(),byte);
  }
  EXPECT_EQ(health.counts[GNSS_RX_CAPACITY_ATTEMPTS],1u);
  EXPECT_EQ(health.counts[GNSS_RX_VALIDATION],0u);
  encoder->time=obs.time=gpst2time(2300,101.);
  ASSERT_TRUE(gen_rtcm3(encoder.get(),1087,0,0));
  for(int i=0;i<encoder->nbyte;++i) input_rtcm3(decoder.get(),encoder->buff[i]);
  EXPECT_EQ(decoder->obs.n,1);
  free_rtcm(encoder.get());free_rtcm(decoder.get());
}

TEST(ReceiverDiagnostics, SlotSelectionIsDistinctFromUnknownSignals) {
  for(int format:{STRFMT_UBX,STRFMT_OEM4,STRFMT_SEPT}) {
    SCOPED_TRACE(format);Decoder d(format);gnss_receiver_diagnostics health{};
    wire::ReceiverDiagnosticScope scope(health);
    if(format==STRFMT_UBX) {
      std::strcpy(d.raw->opt,"-MULTICODE");
      auto b=rawx();Bytes p(b.begin()+6,b.end()-2);p[36]=2;p[38]=8; // known E6B, no slot
      d.feed(ubx(p));
    } else if(format==STRFMT_OEM4) {
      std::strcpy(d.raw->opt,"-GL1L");d.feed(range(0x1c00)); // intentional priority selection
    } else {
      std::strcpy(d.raw->opt,"-GL1W");d.feed(meas());
    }
    EXPECT_EQ(health.counts[GNSS_RX_SLOT],1u);
    EXPECT_EQ(health.counts[GNSS_RX_SIGNAL],0u);
    EXPECT_EQ(health.counts[GNSS_RX_VALIDATION],0u);
  }
}

TEST(ReceiverDiagnostics, Meas3MissingReferenceAndMeasEpochCapacityHaveSeparateUnits) {
  Decoder d(STRFMT_SEPT);gnss_receiver_diagnostics health{};
  wire::ReceiverDiagnosticScope scope(health);
  EXPECT_LT(d.feed(meas3(100100,true)),0);
  EXPECT_EQ(health.counts[GNSS_RX_REFERENCE],1u);
  ASSERT_EQ(d.feed(meas3(101000)),0);ASSERT_EQ(d.feed(end(101000)),1);
  auto b=meas(false,false,102000);Bytes p(b.begin()+8,b.end());
  const Bytes row(p.begin()+12,p.end());p.resize(12);p[6]=MAXOBS+1;
  for(int i=0;i<MAXOBS+1;++i) p.insert(p.end(),row.begin(),row.end());
  d.feed(sbf(4027,p));d.feed(end(102000));
  EXPECT_EQ(health.counts[GNSS_RX_CAPACITY_ROWS],1u);
  EXPECT_EQ(health.counts[GNSS_RX_CAPACITY_ATTEMPTS],0u);
  ASSERT_EQ(d.feed(meas(false,false,103000)),0);ASSERT_EQ(d.feed(end(103000)),1);
  EXPECT_EQ(d.raw->obs.n,1);
}

namespace {
Bytes diagnosticGps(int format,int prn,int id,int issue=0,bool parity=true) {
  Bytes sub(30);setbitu(sub.data(),43,3,id);
  if(id==1) setbitu(sub.data(),168,8,issue);
  if(id==2) setbitu(sub.data(),48,8,issue);
  if(id==3) setbitu(sub.data(),216,8,issue);
  if(format==STRFMT_OEM4) {
    Bytes p(44);put<uint32_t>(p,0,prn);put<uint32_t>(p,4,id);
    std::copy(sub.begin(),sub.end(),p.begin()+8);return oem(p,1330);
  }
  const bool sept=format==STRFMT_SEPT;
  Bytes p(sept?52:48);
  if(sept) {put<uint32_t>(p,0,100000);put<uint16_t>(p,4,2300);p[6]=prn;p[7]=parity;}
  else {p[1]=prn;p[4]=10;p[6]=2;}
  for(int i=0;i<10;++i) put<uint32_t>(p,(sept?12:8)+4*i,getbitu(sub.data(),24*i,24)<<6);
  return sept?sbf(4017,p):ubxMessage(2,0x13,p);
}
}
TEST(ReceiverDiagnostics, SatelliteNavigationWaitDuplicateInconsistencyAndRecovery) {
  for(int fmt:{STRFMT_UBX,STRFMT_SEPT,STRFMT_OEM4}) {
    SCOPED_TRACE(fmt);Decoder d(fmt);gnss_receiver_diagnostics h{};wire::ReceiverDiagnosticScope scope(h);
    const int prn=fmt==STRFMT_OEM4 ? 193 : 1;
    const int sat=satno(fmt==STRFMT_OEM4?SYS_QZS:SYS_GPS,prn);
    EXPECT_EQ(d.feed(diagnosticGps(fmt,prn,1)),0);
    EXPECT_EQ(h.navigation[sat][GNSS_NAV_GPS],2);EXPECT_EQ(h.missing[sat][GNSS_NAV_GPS],6);
    EXPECT_EQ(d.feed(diagnosticGps(fmt,prn+1,1)),0);
    EXPECT_EQ(d.feed(diagnosticGps(fmt,prn,3)),0); // still missing subframe2
    EXPECT_EQ(h.missing[sat][GNSS_NAV_GPS],2);
    EXPECT_EQ(d.feed(diagnosticGps(fmt,prn,2,7)),0);
    EXPECT_EQ(d.feed(diagnosticGps(fmt,prn,3)),0); // complete headers, mismatched IODE
    EXPECT_EQ(h.navigation[sat][GNSS_NAV_GPS],3);EXPECT_EQ(h.missing[sat][GNSS_NAV_GPS],0);
    EXPECT_EQ(h.counts[GNSS_RX_NAV_INCONSISTENT],1u);
    EXPECT_EQ(d.feed(diagnosticGps(fmt,prn,2)),0);
    ASSERT_EQ(d.feed(diagnosticGps(fmt,prn,3)),2);
    EXPECT_EQ(h.navigation[sat][GNSS_NAV_GPS],1);
    EXPECT_EQ(h.navigation[sat+1][GNSS_NAV_GPS],2); // independent satellite remains pending
    EXPECT_EQ(d.feed(diagnosticGps(fmt,prn,3)),0);
    EXPECT_EQ(h.counts[GNSS_RX_NAV_DUPLICATE],1u);
    EXPECT_EQ(h.counts[GNSS_RX_NAV_NO_OUTPUT],0u);
  }
}
TEST(ReceiverDiagnostics, SbfParityVersionLengthArrayAndReferenceAreDistinct) {
  Decoder d(STRFMT_SEPT);gnss_receiver_diagnostics h{};wire::ReceiverDiagnosticScope scope(h);
  EXPECT_EQ(d.feed(diagnosticGps(STRFMT_SEPT,1,1,0,false)),0);
  EXPECT_EQ(h.counts[GNSS_RX_NAV_QUALITY],1u);
  EXPECT_EQ(h.counts[GNSS_RX_NAV_WAIT],0u);
  Bytes nav(88);put<uint32_t>(nav,0,100000);put<uint16_t>(nav,4,2300);nav[6]=38;
  EXPECT_EQ(d.feed(sbf(4004|(2<<13),nav)),-1);
  EXPECT_EQ(h.counts[GNSS_RX_VERSION],0u);EXPECT_EQ(h.counts[GNSS_RX_LENGTH],1u);
  nav.resize(84);EXPECT_EQ(d.feed(sbf(4004,nav)),-1);
  EXPECT_EQ(h.counts[GNSS_RX_LENGTH],2u);EXPECT_EQ(h.counts[GNSS_RX_VERSION],0u);
  auto data=meas3();Bytes body(data.begin()+8,data.end());body[9]|=0x80;
  EXPECT_EQ(d.feed(sbf(4109,body)),-1);EXPECT_EQ(h.counts[GNSS_RX_ARRAY],1u);
  ASSERT_EQ(d.feed(meas()),0);ASSERT_EQ(d.feed(end()),1);
}
TEST(ReceiverDiagnostics, NavigationFamiliesKeepSeparatePendingState) {
  Decoder d(STRFMT_SEPT);gnss_receiver_diagnostics h{};wire::ReceiverDiagnosticScope scope(h);
  auto rawpage=[&](int id,int svid,unsigned bit,unsigned width,unsigned value,unsigned source=0) {
    Bytes p(52);put<uint32_t>(p,0,100000);put<uint16_t>(p,4,2300);p[6]=svid;p[7]=1;p[9]=source;
    Bytes page(40);setbitu(page.data(),bit,width,value);
    if(id==4023) setbitu(page.data(),114,1,1); // odd part of Galileo I/NAV
    if(id==4047 && bit==42) setbitu(page.data(),15,3,1);
    for(int i=0;i<10;++i) put<uint32_t>(p,12+i*4,getbitu(page.data(),i*32,32));
    return sbf(id,p);
  };
  EXPECT_EQ(d.feed(rawpage(4022,71,0,6,1,20)),0); // Galileo F/NAV page1
  const int gal=satno(SYS_GAL,1);
  EXPECT_EQ(h.navigation[gal][GNSS_NAV_FNAV],2);EXPECT_EQ(h.missing[gal][GNSS_NAV_FNAV],14);
  EXPECT_EQ(h.navigation[gal][GNSS_NAV_INAV],0);
  EXPECT_EQ(d.feed(rawpage(4023,71,2,6,1,17)),0);
  EXPECT_EQ(h.navigation[gal][GNSS_NAV_INAV],2);EXPECT_EQ(h.missing[gal][GNSS_NAV_INAV],62);
  EXPECT_EQ(d.feed(rawpage(4026,38,1,4,1)),0);
  EXPECT_EQ(h.missing[satno(SYS_GLO,1)][GNSS_NAV_GLO],30);
  EXPECT_EQ(d.feed(rawpage(4026,38,1,4,5)),0);
  EXPECT_EQ(h.missing[satno(SYS_GLO,1)][GNSS_NAV_GLO],14);
  EXPECT_EQ(d.feed(rawpage(4047,141,15,3,1)),0); // BDS D2 page number zero -> invalid field, no wait
  EXPECT_EQ(h.counts[GNSS_RX_FIELD],1u);
  EXPECT_EQ(d.feed(rawpage(4047,141,42,4,1)),0);
  EXPECT_EQ(h.missing[satno(SYS_CMP,1)][GNSS_NAV_BDS2],1020);
  EXPECT_EQ(d.feed(rawpage(4047,146,15,3,1)),0); // BDS C6 D1 subframe1
  EXPECT_EQ(h.navigation[satno(SYS_CMP,6)][GNSS_NAV_BDS1],2);
  EXPECT_EQ(d.feed(rawpage(4093,191,27,2,0)),0); // NavIC subframe1
  EXPECT_EQ(h.navigation[satno(SYS_IRN,1)][GNSS_NAV_IRN],2);
  EXPECT_EQ(h.missing[satno(SYS_IRN,1)][GNSS_NAV_IRN],2);
}
TEST(ReceiverDiagnostics, RtcmMsmSentinelsAndTrueZeroStayDistinct) {
  auto enc=std::make_unique<rtcm_t>();auto dec=std::make_unique<rtcm_t>();
  ASSERT_TRUE(init_rtcm(enc.get()));ASSERT_TRUE(init_rtcm(dec.get()));
  enc->time=dec->time=gpst2time(2300,100.);enc->staid=42;enc->obs.n=1;
  auto& o=enc->obs.data[0];o.time=enc->time;o.sat=satno(SYS_GPS,1);o.code[0]=CODE_L1C;
  o.P[0]=21000000.;o.L[0]=1e8;o.D[0]=-100;o.SNR[0]=45;
  ASSERT_TRUE(gen_rtcm3(enc.get(),1077,0,0));
  Bytes frame(enc->buff,enc->buff+enc->nbyte);
  auto feed=[&] {
    auto crc=rtk_crc24q(frame.data(),frame.size()-3);setbitu(frame.data(),(frame.size()-3)*8,24,crc);
    int result=0;for(auto b:frame) {int r=input_rtcm3(dec.get(),b);if(r)result=r;}return result;
  };
  // R2: one satellite/signal/cell -> full-frame DF405 at bit230, DF406 at250,
  // DF404 at295; DF397 at194, DF398 at206, DF399 at216.
  setbitu(frame.data(),230,20,1u<<19);setbitu(frame.data(),250,24,1u<<23);
  setbitu(frame.data(),295,15,1u<<14);
  ASSERT_EQ(feed(),1);ASSERT_EQ(dec->obs.n,1);
  EXPECT_TRUE(std::isnan(dec->obs.data[0].P[0]));EXPECT_TRUE(std::isnan(dec->obs.data[0].L[0]));
  EXPECT_TRUE(std::isnan(dec->obs.data[0].D[0]));
  setbitu(frame.data(),194,8,0);setbitu(frame.data(),206,10,0);setbitu(frame.data(),216,14,0);
  setbitu(frame.data(),230,20,0);setbitu(frame.data(),250,24,0);setbitu(frame.data(),295,15,0);
  ASSERT_EQ(feed(),1);EXPECT_DOUBLE_EQ(dec->obs.data[0].P[0],0.);EXPECT_DOUBLE_EQ(dec->obs.data[0].L[0],0.);
  EXPECT_FLOAT_EQ(dec->obs.data[0].D[0],0.f);
  setbitu(frame.data(),194,8,255);setbitu(frame.data(),216,14,1u<<13);
  ASSERT_EQ(feed(),1);EXPECT_TRUE(std::isnan(dec->obs.data[0].P[0]));
  EXPECT_TRUE(std::isnan(dec->obs.data[0].L[0]));EXPECT_TRUE(std::isnan(dec->obs.data[0].D[0]));
  free_rtcm(enc.get());free_rtcm(dec.get());
}

TEST(ReceiverDiagnostics, LegacyUbxSubframeWaitIsNotAnIgnoredPage) {
  Decoder d(STRFMT_UBX);gnss_receiver_diagnostics h{};wire::ReceiverDiagnosticScope scope(h);
  Bytes p(42);p[1]=1;put<uint32_t>(p,6,1u<<2);
  EXPECT_EQ(d.feed(ubxMessage(2,0x11,p)),0);
  EXPECT_EQ(h.navigation[1][GNSS_NAV_GPS],2);EXPECT_EQ(h.missing[1][GNSS_NAV_GPS],6);
  put<uint32_t>(p,6,6u<<2);EXPECT_EQ(d.feed(ubxMessage(2,0x11,p)),0);
  EXPECT_EQ(h.counts[GNSS_RX_FIELD],1u);EXPECT_EQ(h.counts[GNSS_RX_NAV_WAIT],1u);
}

TEST(RawObservations, RtcmLegacyMissingComponentsDoNotBecomeZeroOrStaleValues) {
  for(int type:{1002,1004,1010,1012}) {
    SCOPED_TRACE(type);
    auto dec=std::make_unique<rtcm_t>();ASSERT_TRUE(init_rtcm(dec.get()));
    dec->time=gpst2time(2300,100.);
    const bool glo=type>=1010,dual=type==1004||type==1012;
    for(int mode=0;mode<5;++mode) {
      SCOPED_TRACE(mode);
      Bytes frame(80);frame[0]=0xd3;int at=24;
      auto bits=[&](int value,int width){setbitu(frame.data(),at,width,static_cast<unsigned>(value));at+=width;};
      bits(type,12);bits(42,12);bits(100000+mode*1000+(glo?10782000:0),glo?27:30);
      bits(0,1);bits(1,5);bits(0,1);bits(0,3);bits(1,6);bits(0,1);
      if(glo) bits(0,5); // FCN=-7
      bits(mode==2?(glo?33554431:16777215):mode==3?0:5000000,glo?25:24);
      bits(mode==1?-524288:mode==3?0:-2000,20);bits(10,7);
      bits(mode==3?0:glo?35:70,glo?7:8);bits(mode==3?0:181,8);
      if(dual) {bits(0,2);bits(mode==1?-8192:mode==3?0:-100,14);
        bits(mode==1?-524288:mode==3?0:4000,20);bits(10,7);bits(mode==3?0:182,8);}
      const int bytes=(at+7)/8;frame.resize(bytes+3);setbitu(frame.data(),14,10,bytes-3);
      setbitu(frame.data(),bytes*8,24,rtk_crc24q(frame.data(),bytes));
      int result=0;for(auto byte:frame) {int r=input_rtcm3(dec.get(),byte);if(r)result=r;}
      ASSERT_EQ(result,1);ASSERT_EQ(dec->obs.n,1);
      for(int slot=0;slot<(dual?2:1);++slot) {
        const auto& o=dec->obs.data[0];EXPECT_TRUE(std::isnan(o.D[slot]));
        if(mode==2||(mode==1&&slot==1)) EXPECT_TRUE(std::isnan(o.P[slot]));
        else EXPECT_DOUBLE_EQ(o.P[slot],mode==3?0.:100000+70*299792.458-(slot?2.:0.));
        if(mode==1||mode==2) EXPECT_TRUE(std::isnan(o.L[slot]));
        else if(mode==3) EXPECT_DOUBLE_EQ(o.L[slot],0.);
        else EXPECT_TRUE(std::isfinite(o.L[slot]));
        if(mode==3) { EXPECT_TRUE(std::isnan(o.SNR[slot])); }
      }
    }
    free_rtcm(dec.get());
  }
}
