#include "gnss_ros_standardization/decoder_common.hpp"
#include "gnss_ros_standardization/ephemeris_store.hpp"
#include "gnss_ros_standardization/novatel_protocol.hpp"
#include "gnss_ros_standardization/sbf_protocol.hpp"
#include "gnss_ros_standardization/ubx_protocol.hpp"
#include <gtest/gtest.h>
using namespace gnss_ros_standardization;
namespace {
template <class T> void put(std::vector<uint8_t> &b, size_t offset, T v) {
  std::memcpy(b.data() + offset, &v, sizeof(v));
}
std::vector<uint8_t> pvt() {
  std::vector<uint8_t> b(92);
  put<uint32_t>(b, 0, 100000);
  b[20] = 3;
  b[21] = 1;
  put<uint32_t>(b, 40, 1000);
  put<uint32_t>(b, 44, 2000);
  return b;
}
} // namespace
TEST(ReceiverProtocol, BestposUsesEllipsoidHeightAndUsedSatellites) {
  std::vector<uint8_t> b(72);
  put<uint32_t>(b, 4, 16);
  put<double>(b, 8, 35);
  put<double>(b, 16, 139);
  put<double>(b, 24, 100);
  put<float>(b, 32, 30);
  put<uint32_t>(b, 36, 61);
  b[64] = 18;
  b[65] = 12;
  msg::GnssSolution s;
  ASSERT_TRUE(novatel::pvt::parseBESTPOS(b.data(), b.size(), s));
  EXPECT_DOUBLE_EQ(s.altitude, 130);
  EXPECT_EQ(s.num_sats, 12);
}
// Q-A15: u-blox "dead reckoning only" is a published DR solution, not NONE.
TEST(ReceiverProtocol, UbxDeadReckoningOnlyIsDr) {
  auto b = pvt();
  b[20] = 1;   // fixType: dead reckoning only
  msg::GnssSolution s;
  ASSERT_TRUE(ubx::pvt::parseNavPvt(b.data(), b.size(), s, true, false));
  EXPECT_EQ(s.status, msg::GnssSolution::STATUS_DR);
  b[21] = 0;   // gnssFixOK cleared: not a valid solution at all
  ASSERT_TRUE(ubx::pvt::parseNavPvt(b.data(), b.size(), s, true, false));
  EXPECT_EQ(s.status, msg::GnssSolution::STATUS_NONE);
}
TEST(ReceiverProtocol, DiagonalCovarianceSurvivesPvt) {
  auto b = pvt();
  msg::GnssSolution s;
  s.pos_enu_cov = {9, 0, 0, 0, 4, 0, 0, 0, 16};
  ASSERT_TRUE(ubx::pvt::parseNavPvt(b.data(), b.size(), s, true, false));
  EXPECT_DOUBLE_EQ(s.pos_enu_cov[0], 9);
  EXPECT_DOUBLE_EQ(s.pos_enu_cov[8], 16);
}
// Q-A8: an SBF epoch without a PVT (Mode 0 or a non-zero Error) is a timed
// epoch published as STATUS_NONE, not a dropped block. An invalid time is still
// rejected: such an epoch cannot be placed on the timeline.
TEST(ReceiverProtocol, SbfNoPvtIsAnEpochWithoutSolutionAndInvalidTimeIsRejected) {
  for (auto parse : {&sbf::pvt::parsePVTGeodetic, &sbf::pvt::parsePVTCartesian}) {
    std::vector<uint8_t> b(88);
    put<uint32_t>(b, 0, 100000);
    put<uint16_t>(b, 4, 2300);
    put<double>(b, 8, 0.6);   // valid as latitude [rad] and as ECEF x [m]
    b[6] = 1;
    msg::GnssSolution s;
    ASSERT_TRUE(parse(b.data(), b.size(), s));
    EXPECT_EQ(s.status, msg::GnssSolution::STATUS_SINGLE);
    for (uint8_t mode : {uint8_t(0), uint8_t(1)}) {
      b[6] = mode; b[7] = mode;   // Mode 0, or Error != 0
      s = msg::GnssSolution();
      ASSERT_TRUE(parse(b.data(), b.size(), s));
      EXPECT_EQ(s.status, msg::GnssSolution::STATUS_NONE);
      EXPECT_EQ(s.time_week, 2300u);
      EXPECT_DOUBLE_EQ(s.time_tow, 100.);
    }
    b[7] = 0;
    put<uint32_t>(b, 0, 604800000);
    EXPECT_FALSE(parse(b.data(), b.size(), s));
  }
}

// A-7: a Do-Not-Use velocity leaves the PVT position valid.
TEST(ReceiverProtocol, SbfVelocityDoNotUseKeepsThePosition) {
  std::vector<uint8_t> b(88);
  put<uint32_t>(b, 0, 100000);
  put<uint16_t>(b, 4, 2300);
  b[6] = 1;
  put<double>(b, 8, 0.6);
  put<double>(b, 16, 2.4);
  put<double>(b, 24, 50.);
  for (int offset : {36, 40, 44}) put<float>(b, offset, -2e10f);
  msg::GnssSolution s;
  ASSERT_TRUE(sbf::pvt::parsePVTGeodetic(b.data(), b.size(), s));
  EXPECT_EQ(s.status, msg::GnssSolution::STATUS_SINGLE);
  EXPECT_DOUBLE_EQ(s.altitude, 50.);
  EXPECT_TRUE(std::isnan(s.vel_enu.x));
  put<double>(b, 8, 6378137.);
  s = msg::GnssSolution();
  ASSERT_TRUE(sbf::pvt::parsePVTCartesian(b.data(), b.size(), s));
  EXPECT_DOUBLE_EQ(s.pos_ecef.x, 6378137.);
  EXPECT_TRUE(std::isnan(s.vel_ecef.x));
}
TEST(ReceiverProtocol, RawImuAxesAcrossFramesAreAssembledOnce) {
  ubx::EsfRawAssembler a;
  std::vector<uint8_t> b(12);
  for (int type : {16, 17, 18, 14, 13, 5}) {
    put<uint32_t>(b, 4, (uint32_t(type) << 24) | 1024);
    put<uint32_t>(b, 8, 1000);
    auto ready = a.push(b);
    if (type != 5)
      EXPECT_TRUE(ready.empty());
    else {
      ASSERT_EQ(ready.size(), 1u);
      EXPECT_TRUE(ready[0].complete());
    }
  }
  EXPECT_TRUE(a.push(b).empty());
}
TEST(ReceiverProtocol, EsfRawDataTypeUsesAllEightBits) {
  // U3 p73: unlike ESF-MEAS's six-bit type, ESF-RAW has an eight-bit type.
  // Unrecognized types must not alias one of the six supported IMU axes.
  std::vector<uint8_t> b(12);
  put<uint32_t>(b, 8, 1000);
  for (unsigned type = 0; type < 256; ++type) {
    put<uint32_t>(b, 4, (type << 24) | 1024u);
    const auto samples = ubx::esf_raw::parseEsfRawSamples(b);
    const bool recognized = type == 5 || type == 13 || type == 14 ||
                            type == 16 || type == 17 || type == 18;
    EXPECT_EQ(samples.size(), recognized ? 1u : 0u) << "wire type=" << type;
  }
}
TEST(ReceiverProtocol, NavClockSignedNanosecondsAndAccuracy) {
  // U1 p140: iTOW U4 ms, clkB/clkD I4 ns/ns/s, tAcc U4 ns.
  std::vector<uint8_t> payload(20);
  put<uint32_t>(payload,0,100000);
  put<int32_t>(payload,4,-2000000000);
  put<int32_t>(payload,8,-17);
  put<uint32_t>(payload,12,1000001);
  ubx::nav_clock::Result out;
  ASSERT_TRUE(ubx::nav_clock::parseNavClock(payload.data(),payload.size(),out));
  EXPECT_EQ(out.itow_ms,100000u);
  EXPECT_DOUBLE_EQ(out.clk_bias_s,-2.);
  EXPECT_NEAR(out.clk_drift_s_per_s,-17e-9,1e-22);
  EXPECT_NEAR(out.time_acc_s,.001000001,1e-18);
  EXPECT_FALSE(ubx::nav_clock::parseNavClock(payload.data(),19,out));
}
TEST(ReceiverProtocol, InvalidCovarianceCannotContaminateSolution) {
  std::vector<uint8_t> b(64);
  b[5] = 1;
  put<float>(b, 16, 1);
  put<float>(b, 28, 1);
  put<float>(b, 36, 1);
  msg::GnssSolution s;
  ASSERT_TRUE(ubx::nav_cov::parseNavCov(b.data(), b.size(), s));
  b[4] = 1;
  EXPECT_FALSE(ubx::nav_cov::parseNavCov(b.data(), b.size(), s));
  b[4] = 0;
  put<float>(b, 20, 2);
  EXPECT_FALSE(ubx::nav_cov::parseNavCov(b.data(), b.size(), s));
  put<float>(b, 20, std::numeric_limits<float>::quiet_NaN());
  EXPECT_FALSE(ubx::nav_cov::parseNavCov(b.data(), b.size(), s));
}
TEST(ReceiverProtocol, EphemerisDuplicatesRefinementsAndBoundedHistory) {
  gnss_utils::EphemerisStore store;
  eph_t e{};
  e.sat = 1;
  e.toe = gpst2time(2300, 100);
  e.toc = e.toe;
  e.ttr = e.toe;
  ASSERT_TRUE(store.ingestEph(e));
  EXPECT_FALSE(store.ingestEph(e));
  e.svh = 1;
  EXPECT_TRUE(store.ingestEph(e));
  EXPECT_FALSE(store.ingestEph(e));
  nav_t nav{};
  // 25 hours of broadcasts, 32 satellites, every 2 minutes, duplicate replay.
  for (int i = 0; i < 750; ++i)
    for (int sat = 1; sat <= 32; ++sat) {
      e.sat = sat;
      e.iode = i % 256;
      e.toe = gpst2time(2300, 100 + i * 120);
      e.toc = e.toe;
      e.ttr = e.toe;
      store.ingestEph(e);
      EXPECT_FALSE(store.ingestEph(e));
      EXPECT_LE(store.keplerCount(), 32u * 8);
    }
  store.applyToNav(nav);
  EXPECT_LE(nav.n, 256);
  store.applyToNav(nav);
  EXPECT_LE(nav.n, 256);
  std::free(nav.eph);
  std::free(nav.geph);
}
TEST(ReceiverProtocol, RedactsStreamCredentials) {
  EXPECT_EQ(
      decoder_common::redactStreamUri("ntrip://user:secret@host:2101/mount"),
      "ntrip://***@host:2101/mount");
  EXPECT_EQ(decoder_common::redactStreamUri("tcpcli://host:123"),
            "tcpcli://host:123");
}
TEST(ReceiverProtocol, CommandRepliesAreMatchedAndBinaryIsSkipped) {
  wire::CommandReply sbf(wire::Protocol::Sbf,
                         "sso, Stream1, COM1, PVTGeodetic, sec1");
  int status = 0;
  for (auto c : std::string("$R: another command\r\n"))
    status = sbf.feed(c);
  EXPECT_EQ(status, 0);
  for (auto c : std::string("$R: sso, Stream1, COM1, PVTGeodetic, sec1\r\n"))
    status = sbf.feed(c);
  EXPECT_EQ(status, 1);
  wire::CommandReply nov(wire::Protocol::Novatel, "LOG BESTPOSB ONTIME 1");
  std::vector<uint8_t> b(28 + 8 + 4);
  b[0] = 0xaa;
  b[1] = 0x44;
  b[2] = 0x12;
  b[3] = 28;
  b[8] = 8;
  const std::string fake = "\r\n<OK\r\n";
  std::copy(fake.begin(), fake.end(), b.begin() + 28);
  for (auto c : b)
    status = nov.feed(c);
  EXPECT_EQ(status, 0);
  for (auto c : std::string("\r\n<ERROR:Invalid Message ID\r\n"))
    status = nov.feed(c);
  EXPECT_EQ(status, -1);
}

TEST(ReceiverProtocol, UnterminatedPortPromptsDoNotHideReplies) {
  for (const auto protocol : {wire::Protocol::Sbf, wire::Protocol::Novatel}) {
    const bool sbf = protocol == wire::Protocol::Sbf;
    for (const auto& port : {"COM1", "USB2", "IP10", "ICOM1"}) {
      const std::string prompt = sbf ? std::string(port) + ">" : "[" + std::string(port) + "]";
      wire::CommandReply reply(protocol, "command");
      for (auto c : prompt) EXPECT_EQ(reply.feed(c), 0);
      int result = 0;
      for (auto c : std::string(sbf ? "$R: command\r\n" : "<OK\r\n")) result = reply.feed(c);
      EXPECT_EQ(result, 1);
    }
    wire::CommandReply echo(protocol, "command");
    for (auto c : std::string(sbf ? "echo>$R: command\r\n" : "[garbage]<OK\r\n"))
      EXPECT_EQ(echo.feed(c), 0);
  }
}

TEST(ReceiverProtocol, StructuralPromptsAndRepeatedPrompts) {
  for (const auto& text : {"IPS1>$R: cmd\r\n", "IPR2>$R: cmd\r\n",
       "BT10>$R: cmd\r\n", "OTG1>$R: cmd\r\n", "COM1>COM1>$R: cmd\r\n"}) {
    wire::CommandReply reply(wire::Protocol::Sbf,"cmd");
    int result=0; for (char c:std::string(text)) result=reply.feed(c);
    EXPECT_EQ(result,1) << text;
  }
  for (const auto& text : {"[XCOM1]<OK\r\n", "[AUX]<OK\r\n", "[COM1][SCOM1]<OK\r\n"}) {
    wire::CommandReply reply(wire::Protocol::Novatel,"cmd");
    int result=0; for (char c:std::string(text)) result=reply.feed(c);
    EXPECT_EQ(result,1) << text;
  }
  for (const auto& text : {"IPS1>$R: other\r\n", "STOP>$R: cmd\r\n"}) {
    wire::CommandReply reply(wire::Protocol::Sbf,"cmd");
    int result=0; for (char c:std::string(text)) result=reply.feed(c);
    EXPECT_EQ(result,0);
  }
  wire::CommandReply error(wire::Protocol::Novatel,"cmd");
  int result=0; for (char c:std::string("[COM1]<ERROR: rejected\r\n")) result=error.feed(c);
  EXPECT_EQ(result,-1);
}

TEST(ReceiverProtocol, PartialWritesPreserveEveryByteAndStopOnFailure) {
  const std::vector<uint8_t> input{1, 2, 3, 4, 5, 6, 7};
  std::vector<uint8_t> output;
  EXPECT_TRUE(decoder_common::writeAllWith(
      [&](const uint8_t *p, size_t n) {
        const auto count = std::min(n, size_t(2));
        output.insert(output.end(), p, p + count);
        return int(count);
      },
      input.data(), input.size()));
  EXPECT_EQ(input, output);
  EXPECT_FALSE(decoder_common::writeAllWith(
      [](const uint8_t *, size_t) { return 0; }, input.data(), input.size(),
      std::chrono::milliseconds(0)));
  EXPECT_FALSE(decoder_common::writeAllWith(
      [](const uint8_t *, size_t) { return -1; }, input.data(), input.size()));
}

TEST(ReceiverProtocol, GlonassUsesCanonicalGpstWithoutSecondLeapCorrection) {
  geph_t g{};
  g.sat = satno(SYS_GLO, 1);
  g.toe = gpst2time(2300, 604790);
  g.tof = gpst2time(2301, 5);
  auto m = gnss_utils::gephToMsg(g);
  EXPECT_EQ(m.week, 2300u);
  EXPECT_DOUBLE_EQ(m.toe, 604790);
  EXPECT_DOUBLE_EQ(m.tof, 5);
  const auto restored = gnss_utils::msgToGeph(m);
  EXPECT_DOUBLE_EQ(timediff(restored.toe, g.toe), 0);
  EXPECT_DOUBLE_EQ(timediff(restored.tof, g.tof), 0);
  m.pos.clear();
  EXPECT_EQ(gnss_utils::msgToGeph(m).sat, 0);
}

TEST(ReceiverProtocol, SbfCovarianceRejectsNonPsdAndPreservesZeroVariance) {
  using Parser = bool (*)(const uint8_t*, size_t, msg::GnssSolution&);
  for (Parser parse : {sbf::pvt::parsePosCovGeodetic, sbf::pvt::parsePosCovCartesian,
                       sbf::pvt::parseVelCovGeodetic, sbf::pvt::parseVelCovCartesian}) {
    std::vector<uint8_t> b(48); b[6] = 1;
    put<uint16_t>(b, 4, 2300);
    msg::GnssSolution s;
    ASSERT_TRUE(parse(b.data(), b.size(), s));
    for (int i : {8,12,16}) put<float>(b, i, 1);
    for (float cross : {-1.f, 0.f, 1.f}) {
      put<float>(b, 24, cross);
      EXPECT_TRUE(parse(b.data(), b.size(), s));
    }
    put<float>(b, 24, 2);
    EXPECT_FALSE(parse(b.data(), b.size(), s));
    put<float>(b, 24, .9); put<float>(b, 28, .9); put<float>(b, 36, -.9);
    EXPECT_FALSE(parse(b.data(), b.size(), s)); // determinant, not only 2x2 minors
    put<float>(b, 24, 0); put<float>(b, 28, 0); put<float>(b, 36, 0);
    b[6] = 0; EXPECT_FALSE(parse(b.data(), b.size(), s)); b[6] = 1;
    put<uint16_t>(b, 4, 65535); EXPECT_FALSE(parse(b.data(), b.size(), s));
  }
}
TEST(ReceiverProtocol, SbfMissingDopIsUnknown) {
  std::vector<uint8_t> b(24); put<uint16_t>(b, 4, 2300);
  for (int i : {8,10,12,14}) put<uint16_t>(b, i, 65535);
  msg::GnssSolution s;
  ASSERT_TRUE(sbf::pvt::parseDop(b.data(), b.size(), s));
  EXPECT_TRUE(std::isnan(s.pdop)); EXPECT_TRUE(std::isnan(s.gdop));
  put<uint16_t>(b, 8, 200); put<uint16_t>(b, 10, 100);
  ASSERT_TRUE(sbf::pvt::parseDop(b.data(), b.size(), s));
  EXPECT_FLOAT_EQ(s.pdop, 2); EXPECT_NEAR(s.gdop, std::sqrt(5), 1e-6);
}
TEST(ReceiverProtocol, UbxHighPrecisionVersionRangeAndOutputAtomicity) {
  std::vector<uint8_t> b(36);
  ubx::nav_hpposllh::Result result{};
  ASSERT_TRUE(ubx::nav_hpposllh::parseNavHpPosLlh(b.data(), b.size(), result));
  b[0] = 255;
  EXPECT_FALSE(ubx::nav_hpposllh::parseNavHpPosLlh(b.data(), b.size(), result));
  b[0] = 0; put<int32_t>(b, 12, 1000000000);
  EXPECT_FALSE(ubx::nav_hpposllh::parseNavHpPosLlh(b.data(), b.size(), result));
  EXPECT_DOUBLE_EQ(result.lat_deg, 0);
  put<int32_t>(b, 12, 900000000); b[25] = 0;
  EXPECT_TRUE(ubx::nav_hpposllh::parseNavHpPosLlh(b.data(), b.size(), result));
  b[25] = 1;
  EXPECT_FALSE(ubx::nav_hpposllh::parseNavHpPosLlh(b.data(), b.size(), result));
  double xyz[3]{}; b.assign(28, 0);
  ASSERT_TRUE(ubx::nav_hpposecef::parseNavHpPosEcef(b.data(), b.size(), xyz));
  b[0] = 1; EXPECT_FALSE(ubx::nav_hpposecef::parseNavHpPosEcef(b.data(), b.size(), xyz));
}
TEST(ReceiverProtocol, NovatelCovarianceStatusAndMissingFieldsAreIndependent) {
  std::vector<uint8_t> b(112);
  put<uint32_t>(b, 4, 16); put<uint32_t>(b, 48, 8);
  for (int offset : {32,36,40,76,80,84}) put<float>(b, offset, 2);
  msg::GnssSolution s;
  ASSERT_TRUE(novatel::pvt::parseBESTXYZ(b.data(), b.size(), s));
  EXPECT_DOUBLE_EQ(s.pos_cov_ecef[0], 4);
  EXPECT_DOUBLE_EQ(s.vel_cov_ecef[0], 4);
  put<uint32_t>(b, 44, 1);
  ASSERT_TRUE(novatel::pvt::parseBESTXYZ(b.data(), b.size(), s));
  EXPECT_DOUBLE_EQ(s.pos_cov_ecef[0], 4);
  EXPECT_TRUE(std::isnan(s.vel_cov_ecef[0]));
  put<float>(b, 32, std::numeric_limits<float>::quiet_NaN());
  EXPECT_FALSE(novatel::pvt::parseBESTXYZ(b.data(), b.size(), s));
  put<uint32_t>(b, 44, 0);
  ASSERT_TRUE(novatel::pvt::parseBESTXYZ(b.data(), b.size(), s));
  EXPECT_TRUE(std::isnan(s.pos_cov_ecef[0]));
  EXPECT_DOUBLE_EQ(s.vel_cov_ecef[0], 4);
}
TEST(ReceiverProtocol, NovatelDopRejectsInvalidValues) {
  std::vector<uint8_t> b(20);
  put<float>(b, 0, 3); put<float>(b, 4, 2); put<float>(b, 8, 1);
  msg::GnssSolution s;
  ASSERT_TRUE(novatel::pvt::parsePSRDOP(b.data(), b.size(), s));
  EXPECT_NEAR(s.vdop, std::sqrt(3), 1e-6);
  for (float bad : {-1.f, std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()}) {
    put<float>(b, 0, bad);
    EXPECT_FALSE(novatel::pvt::parsePSRDOP(b.data(), b.size(), s));
  }
}

#include "gnss_ros_standardization/receiver_session.hpp"
TEST(ReceiverSession, LossIsAnEdgeAndEpochRewindDoesNotMistakeWeekBoundary) {
  using namespace gnss_ros_standardization::wire;
  StreamSession session;
  EXPECT_FALSE(session.lost(1)); EXPECT_FALSE(session.lost(2));
  EXPECT_FALSE(session.lost(3)); EXPECT_TRUE(session.lost(1));
  EXPECT_FALSE(session.lost(0)); EXPECT_FALSE(session.lost(2)); EXPECT_TRUE(session.lost(-1));
  ReceiverTimeline time;
  EXPECT_FALSE(time.observe(2300.*604800+604799));
  EXPECT_FALSE(time.observe(2301.*604800));
  EXPECT_TRUE(time.observe(2301.*604800-.1));
  time.reset(); EXPECT_FALSE(time.observe(604799,1,true));
  EXPECT_FALSE(time.observe(0,1,true)); EXPECT_FALSE(time.observe(1,1,true));
  EXPECT_TRUE(time.observe(.9,1,true));
}

#include "gnss_ros_standardization/binary_framer.hpp"
// N2/Q3 intentionally replaces the former FINE-only/wall-clock anchor rule.
TEST(ReleaseN2, OemTimeMatchesRtklibPolicy) {
  wire::NovatelTimeQuality quality;
  wire::Frame frame;
  frame.week=2300; frame.tow_ms=100000;
  for (int status : {0, 60, 80, 100, 120, 130, 140, 160, 170, 180, 200}) {
    frame.time_status=status;
    EXPECT_TRUE(quality.observe(frame)) << status;
  }
  frame.time_status=20; EXPECT_FALSE(quality.observe(frame));
  frame.short_header=true; EXPECT_TRUE(quality.observe(frame));
  quality.reset(); EXPECT_TRUE(quality.observe(frame));
  frame.week=0; EXPECT_FALSE(quality.observe(frame));
  frame.week=2300; frame.tow_ms=604800000; EXPECT_FALSE(quality.observe(frame));
}

#ifndef _WIN32
#include <fcntl.h>
#include <unistd.h>
#include <cstdlib>
TEST(ReceiverSession, SerialHangupReopensTheSameDeviceWithoutTreatingIdleAsLoss) {
  struct Ports {
    int master=-1,slave=-1; stream_t stream{}; std::string directory,link;
    Ports() {
      char path[]="/tmp/gnss-session-XXXXXX";
      if (auto p=mkdtemp(path)) directory=p;
      link=directory+"/receiver";strinit(&stream);
    }
    bool create() {
      master=posix_openpt(O_RDWR|O_NOCTTY|O_NONBLOCK);
      if (master<0 || grantpt(master) || unlockpt(master)) return false;
      const char* path=ptsname(master); if (!path) return false;
      slave=open(path,O_RDWR|O_NOCTTY); if(slave<0) return false;
      unlink(link.c_str());return symlink(path,link.c_str())==0;
    }
    void unplug() {if(master>=0)close(master);if(slave>=0)close(slave);master=slave=-1;}
    ~Ports() {strclose(&stream);unplug();unlink(link.c_str());rmdir(directory.c_str());}
  } ports;
  ASSERT_FALSE(ports.directory.empty()); ASSERT_TRUE(ports.create());
  const std::string path=".."+ports.link+":115200";
  ASSERT_EQ(stropen(&ports.stream,STR_SERIAL,STR_MODE_R,path.c_str()),1);
  wire::StreamSession session; const auto start=wire::StreamSession::Clock::now();
  EXPECT_FALSE(session.poll(ports.stream,start));
  uint8_t buffer[16]; EXPECT_EQ(strread(&ports.stream,buffer,16),0);
  EXPECT_FALSE(session.poll(ports.stream,start));
  ports.unplug();
  EXPECT_EQ(strread(&ports.stream,buffer,16),0);
  EXPECT_TRUE(session.poll(ports.stream,start));
  ASSERT_TRUE(ports.create());
  EXPECT_FALSE(session.poll(ports.stream,start+std::chrono::seconds(2)));
  ASSERT_GE(strstat(&ports.stream,nullptr),2);
  ASSERT_EQ(write(ports.master,"new",3),3);
  int count=0;
  for(int i=0;i<100 && count==0;i++) {
    count=strread(&ports.stream,buffer,16);
    if (!count) usleep(1000);
  }
  ASSERT_EQ(count,3);EXPECT_EQ(std::string(reinterpret_cast<char*>(buffer),count),"new");
  EXPECT_FALSE(session.poll(ports.stream,start+std::chrono::seconds(2)));
}
#endif

TEST(ReceiverSession, WeekReferenceExpiresOnSteadyTimeAndCanBeReseeded) {
  using namespace std::chrono_literals;
  wire::FreshGnssReference ref;const auto start=wire::FreshGnssReference::Clock::now();
  EXPECT_EQ(ref.get(start).time,0);
  ref.update(gpst2time(2300,604799.),start);
  int week=0;
  EXPECT_TRUE(decoder_common::resolveGpsWeekForTow(ref.get(start+1s),0.,week));
  EXPECT_EQ(week,2301);
  EXPECT_EQ(ref.get(start+6s).time,0);EXPECT_EQ(ref.get(start-1s).time,0);
  ref.update(gpst2time(2301,1.),start+7s);
  EXPECT_NE(ref.get(start+7s).time,0);
  ref.reset();EXPECT_EQ(ref.get(start+7s).time,0);
}

TEST(ReceiverDiagnosticsFrontend, ParserFailuresKeepTheirSpecificReasonsAndRecovery) {
  gnss_receiver_diagnostics h{};wire::ReceiverDiagnosticScope scope(h);msg::GnssSolution out;
  auto u=pvt();ASSERT_FALSE(ubx::pvt::parseNavPvt(u.data(),1,out,true,false));
  put<uint32_t>(u,0,604800000);ASSERT_FALSE(ubx::pvt::parseNavPvt(u.data(),u.size(),out,true,false));
  put<uint32_t>(u,0,100000);put<int32_t>(u,28,910000000);
  ASSERT_FALSE(ubx::pvt::parseNavPvt(u.data(),u.size(),out,true,false));
  put<int32_t>(u,28,350000000);ASSERT_TRUE(ubx::pvt::parseNavPvt(u.data(),u.size(),out,true,false));
  std::vector<uint8_t> cov(64);cov[4]=1;
  ASSERT_FALSE(ubx::nav_cov::parseNavCov(cov.data(),cov.size(),out));
  EXPECT_EQ(h.counts[GNSS_RX_LENGTH],1u);EXPECT_EQ(h.counts[GNSS_RX_TIME],1u);
  EXPECT_EQ(h.counts[GNSS_RX_VALUE],1u);EXPECT_EQ(h.counts[GNSS_RX_VERSION],1u);
  std::vector<uint8_t> s(88);put<uint32_t>(s,0,100000);put<uint16_t>(s,4,2300);
  // No PVT is counted as a quality event but still yields an epoch (Q-A8).
  ASSERT_TRUE(sbf::pvt::parsePVTGeodetic(s.data(),s.size(),out));
  EXPECT_EQ(out.status,msg::GnssSolution::STATUS_NONE);
  s[6]=1;ASSERT_TRUE(sbf::pvt::parsePVTGeodetic(s.data(),s.size(),out));
  std::vector<uint8_t> v(44);put<uint32_t>(v,0,1);
  ASSERT_FALSE(novatel::pvt::parseBESTVEL(v.data(),v.size(),out));
  put<uint32_t>(v,0,0);ASSERT_TRUE(novatel::pvt::parseBESTVEL(v.data(),v.size(),out));
  EXPECT_EQ(h.counts[GNSS_RX_QUALITY],2u);
}

TEST(ReceiverProtocol, NovatelDelayedVelocityIsNotStampedAsCurrent) {
  std::vector<uint8_t> v(44), xyz(112);
  msg::GnssSolution out;
  put<uint32_t>(xyz, 4, 16); put<uint32_t>(xyz, 48, 8);
  for (float latency : {0.25f, -1.f, std::numeric_limits<float>::quiet_NaN()}) {
    put<float>(v, 8, latency); put<float>(xyz, 92, latency);
    EXPECT_FALSE(novatel::pvt::parseBESTVEL(v.data(), v.size(), out));
    ASSERT_TRUE(novatel::pvt::parseBESTXYZ(xyz.data(), xyz.size(), out));
    EXPECT_DOUBLE_EQ(out.pos_cov_ecef[0], 0);
    EXPECT_TRUE(std::isnan(out.vel_cov_ecef[0]));
  }
  put<float>(v, 8, 0); EXPECT_TRUE(novatel::pvt::parseBESTVEL(v.data(),v.size(),out));
  put<float>(xyz, 92, 0); EXPECT_TRUE(novatel::pvt::parseBESTXYZ(xyz.data(),xyz.size(),out));
  EXPECT_DOUBLE_EQ(out.vel_cov_ecef[0], 0);
}

#include "gnss_ros_standardization/measurement_validation.hpp"
TEST(ReleaseF1, ObservedRoverRatesAndGapsWithoutRateWindow) {
  for (double hz : {1., 5., 10., 20., 50., .5}) {
    gnss_utils::ObservedInterval grid;
    for (int i=0;i<=10;++i) grid.observe(i/hz);
    EXPECT_TRUE(grid.known()); EXPECT_NEAR(grid.seconds(),1./hz,1e-12);
    grid.observe(1000.); EXPECT_NEAR(grid.seconds(),1./hz,1e-12);
    grid.reset(); EXPECT_FALSE(grid.known());
  }
}
TEST(ReleaseF2, ImuCoverageScalesWithMeasuredSpacing) {
  for (double hz : {10.,50.,100.,400.}) {
    gnss_utils::ObservedInterval interval;
    interval.observe(std::numeric_limits<double>::quiet_NaN());
    for (int i=0;i<=10;++i) { interval.observe(i/hz);interval.observe(i/hz); }
    EXPECT_TRUE(interval.covers(1.01/hz));
    EXPECT_FALSE(interval.covers(3./hz));
    EXPECT_FALSE(interval.covers(-1./hz));
  }
}

TEST(ReleaseS1, P1_4_RedactsThroughLastAtInEntirePath) {
  // Supersedes the round-5 authority-only rule: RTKLIB uses strrchr(path, '@').
  for (const auto& item : std::vector<std::pair<std::string,std::string>>{
      {"ntrip://user:pa/ss@caster:2101/MNT", "ntrip://***@caster:2101/MNT"},
      {"ntrip://us@er:p@ss@caster:2101/MNT", "ntrip://***@caster:2101/MNT"},
      {"ntrip://user:pw@caster/M@NT", "ntrip://***@NT"},
      {"tcpcli://host/path@name", "tcpcli://***@name"},
      {"user:pa/ss@caster/MNT", "***@caster/MNT"},
      {"tcpcli://host:123/path", "tcpcli://host:123/path"},
      {"serial:///dev/ttyUSB0:115200", "serial:///dev/ttyUSB0:115200"},
      {"host/path", "host/path"}})
    EXPECT_EQ(decoder_common::redactStreamUri(item.first),item.second);
}

TEST(ReleaseV1, Oem7BestposTypesRetainUsablePositions) {
  using S=gnss_ros_standardization::msg::GnssSolution;
  for (int type:{1,2,16,51,53,67,72}) EXPECT_EQ(novatel::pvt::mapPosTypeToStatus(type),S::STATUS_SINGLE) << type;
  // PROPAGATED: carried forward by the receiver filter without new observations.
  EXPECT_EQ(novatel::pvt::mapPosTypeToStatus(19),S::STATUS_DR);
  for (int type:{17,54}) EXPECT_EQ(novatel::pvt::mapPosTypeToStatus(type),S::STATUS_DGPS) << type;
  for (int type:{18,52}) EXPECT_EQ(novatel::pvt::mapPosTypeToStatus(type),S::STATUS_SBAS) << type;
  for (int type:{32,33,34,55,71}) EXPECT_EQ(novatel::pvt::mapPosTypeToStatus(type),S::STATUS_FLOAT) << type;
  for (int type:{48,49,50,56,70}) EXPECT_EQ(novatel::pvt::mapPosTypeToStatus(type),S::STATUS_FIX) << type;
  for (int type:{68,69,73,74,77,78,79,80}) EXPECT_EQ(novatel::pvt::mapPosTypeToStatus(type),S::STATUS_PPP) << type;
  // Every value in the OEM7 range: named types above, reserved/no-position
  // values below. 33 and 51 retain older documented OEM7 interpretations.
  for (int type:{0,3,4,5,6,7,8,9,10,11,12,13,14,15,20,21,22,23,24,25,26,27,28,29,30,31,
                35,36,37,38,39,40,41,42,43,44,45,46,47,57,58,59,60,61,62,63,64,65,66,75,76,81,999})
    EXPECT_EQ(novatel::pvt::mapPosTypeToStatus(type),S::STATUS_NONE) << type;
}

TEST(ReleaseV2, InvalidMeasurementRateIsAnError) {
  for(int rate : {0,-1,201}) EXPECT_THROW(gnss_utils::validatePublishRate(rate,5,200,rclcpp::get_logger("test")),std::invalid_argument);
  int rate=100;EXPECT_TRUE(gnss_utils::validatePublishRate(rate,5,200,rclcpp::get_logger("test")));
  EXPECT_EQ(rate,100);
}

TEST(ReleaseF9, UnavailableVelocityIsNotStationary) {
  gnss_ros_standardization::msg::GnssSolution msg;
  gnss_utils::setUnknownVelocity(msg);
  EXPECT_TRUE(std::isnan(msg.vel_ecef.x));EXPECT_TRUE(std::isnan(msg.vel_enu.z));
  for(double v:msg.vel_cov_ecef) EXPECT_TRUE(std::isnan(v));
  for(double v:msg.vel_enu_cov) EXPECT_TRUE(std::isnan(v));
}

TEST(ReleaseV7, FixedPositionIsUsableWithoutClaimingRtkAndNoFixIsUnavailable) {
  using S=gnss_ros_standardization::msg::GnssSolution;
  EXPECT_EQ(sbf::pvt::mapModeToStatus(3), S::STATUS_SINGLE);
  S sol;
  for (auto status : {S::STATUS_SINGLE,S::STATUS_DGPS,S::STATUS_SBAS,S::STATUS_FLOAT,S::STATUS_FIX,S::STATUS_PPP}) {
    sol.status=status;EXPECT_TRUE(gnss_utils::hasPositionFix(sol));
  }
  sol.status=S::STATUS_NONE;EXPECT_FALSE(gnss_utils::hasPositionFix(sol));
  sol.vel_ecef.x=123;gnss_utils::setUnknownVelocity(sol);
  EXPECT_TRUE(std::isnan(sol.vel_ecef.x));EXPECT_TRUE(std::isnan(sol.vel_cov_ecef[0]));
}

TEST(ReleaseQ6, ReuseRatioFollowsObservedClocksInsteadOfConfiguredRates) {
  for (double rover_hz : {1.,5.,10.,20.}) {
    gnss_utils::ObservedInterval rover,base;
    for (int i=0;i<220;++i) {
      rover.observe(100.+i/rover_hz);
      base.observe(100.+std::floor(i/rover_hz));
    }
    ASSERT_TRUE(rover.known());ASSERT_TRUE(base.known());
    EXPECT_NEAR(base.seconds()/rover.seconds(),rover_hz,1e-9);
  }
}
