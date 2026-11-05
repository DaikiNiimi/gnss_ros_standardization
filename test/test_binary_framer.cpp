#include "gnss_ros_standardization/binary_framer.hpp"
#include <gtest/gtest.h>
using namespace gnss_ros_standardization::wire;
namespace {
std::vector<uint8_t> ubx(std::vector<uint8_t> payload = {6, 8}) {
  std::vector<uint8_t> b{0xB5, 0x62, 5, 1, uint8_t(payload.size()), 0};
  b.insert(b.end(), payload.begin(), payload.end());
  uint8_t a = 0, c = 0;
  for (size_t i = 2; i < b.size(); ++i) {
    a += b[i];
    c += a;
  }
  b.push_back(a);
  b.push_back(c);
  return b;
}
std::vector<uint8_t> sbf() {
  std::vector<uint8_t> b{0x24, 0x40, 0, 0, 0xA7, 0x0F, 12, 0, 1, 2, 3, 4};
  auto c = crc16(b.data() + 4, b.size() - 4);
  b[2] = c & 255;
  b[3] = c >> 8;
  return b;
}
std::vector<uint8_t> oem(bool short_header) {
  std::vector<uint8_t> b(short_header ? 12 : 28, 0);
  b[0] = 0xAA;
  b[1] = 0x44;
  b[2] = short_header ? 0x13 : 0x12;
  b[3] = short_header ? 40 : 28;
  b[4] = 0xB6;
  b[5] = 5;
  if (!short_header)
    b[8] = 40;
  auto w = short_header ? 6 : 14;
  b[w] = 0xFC;
  b[w + 1] = 8;
  b[w + 2] = 0xE8;
  b[w + 3] = 3;
  b.resize(b.size() + 40, 0);
  auto c = crc32(b.data(), b.size());
  for (int i = 0; i < 4; ++i)
    b.push_back(c >> (8 * i));
  return b;
}
} // namespace
TEST(BinaryFramer, FragmentationAndCoalescing) {
  Framer f(Protocol::Ubx);
  int count = 0;
  auto emit = [&](const Frame &x) {
    ++count;
    EXPECT_TRUE(matchesAck(x, 6, 8));
    EXPECT_FALSE(matchesAck(x, 6, 9));
  };
  for (int j = 0; j < 3; ++j)
    for (auto b : ubx())
      f.feed(b, emit);
  EXPECT_EQ(count, 3);
  EXPECT_EQ(f.buffered(), 0u);
}
TEST(BinaryFramer, BadChecksumThenGoodFrame) {
  for (auto protocol : {Protocol::Ubx, Protocol::Sbf, Protocol::Novatel}) {
    auto good = protocol == Protocol::Ubx   ? ubx()
                : protocol == Protocol::Sbf ? sbf()
                                            : oem(false);
    auto bad = good;
    bad.back() ^= 1;
    Framer f(protocol);
    int count = 0;
    auto emit = [&](const Frame &) { ++count; };
    for (auto b : bad)
      f.feed(b, emit);
    EXPECT_EQ(count, 0);
    for (auto b : good)
      f.feed(b, emit);
    EXPECT_EQ(count, 1);
    EXPECT_GT(f.rejected(), 0u);
  }
}
TEST(BinaryFramer, NovatelShortAndLongHeader) {
  Framer f(Protocol::Novatel);
  int count = 0;
  for (bool sh : {false, true})
    for (auto b : oem(sh))
      f.feed(b, [&](const Frame &x) {
        ++count;
        EXPECT_EQ(x.id, 1462);
        EXPECT_EQ(x.week, 2300);
        EXPECT_EQ(x.tow_ms, 1000u);
        EXPECT_EQ(x.payload.size(), 40u);
      });
  EXPECT_EQ(count, 2);
}
TEST(BinaryFramer, TruncatedLengthResynchronizesOnTimeout) {
  Framer f(Protocol::Ubx);
  int count = 0;
  auto emit = [&](const Frame &) { ++count; };
  for (auto b : std::vector<uint8_t>{0xB5, 0x62, 1, 7, 255, 255})
    f.feed(b, emit);
  for (auto b : ubx())
    f.feed(b, emit);
  EXPECT_EQ(count, 0);
  f.expire(emit);
  EXPECT_EQ(count, 1);
}
TEST(BinaryFramer, EmbeddedSyncDoesNotSplitValidPayload) {
  Framer f(Protocol::Ubx);
  int count = 0;
  auto nested = ubx();
  for (auto b : ubx(nested))
    f.feed(b, [&](const Frame &x) {
      ++count;
      EXPECT_EQ(x.payload, nested);
    });
  EXPECT_EQ(count, 1);
}
TEST(BinaryFramer, NoiseAndMalformedHeadersRemainBounded) {
  for (auto protocol : {Protocol::Ubx, Protocol::Sbf, Protocol::Novatel}) {
    Framer f(protocol);
    uint32_t x = 123;
    int count = 0;
    auto emit = [&](const Frame &) { ++count; };
    for (int i = 0; i < 200000; ++i) {
      x = x * 1664525 + 1013904223;
      f.feed(x >> 24, emit);
      if (i % 1000 == 0)
        f.expire(emit);
      EXPECT_LE(f.buffered(), 65800u);
    }
    EXPECT_EQ(count, 0);
  }
}

namespace {
std::vector<uint8_t> withText(Protocol protocol, const std::string &text) {
  std::vector<uint8_t> payload(text.begin(), text.end());
  if (protocol == Protocol::Ubx) return ubx(payload);
  if (protocol == Protocol::Sbf) {
    while (payload.size() % 4) payload.push_back(0);
    std::vector<uint8_t> b{0x24, 0x40, 0, 0, 0, 0,
                          uint8_t(payload.size() + 8), 0};
    b.insert(b.end(), payload.begin(), payload.end());
    auto c = crc16(b.data() + 4, b.size() - 4);
    b[2] = c; b[3] = c >> 8;
    return b;
  }
  auto b = oem(false);
  b.resize(28);
  b[8] = payload.size(); b[9] = payload.size() >> 8;
  b.insert(b.end(), payload.begin(), payload.end());
  auto c = crc32(b.data(), b.size());
  for (int i = 0; i < 4; ++i) b.push_back(c >> (8*i));
  return b;
}
}
TEST(BinaryFramer, MixedTextHasOneBoundaryOwner) {
  const std::string sentence = "$GPGGA,120000*00\r\n";
  for (auto protocol : {Protocol::Ubx, Protocol::Sbf, Protocol::Novatel}) {
    for (bool corrupt : {false, true}) {
      Framer f(protocol);
      int frames = 0;
      std::vector<std::string> lines;
      auto emit = [&](const Frame &x) { ++frames; EXPECT_FALSE(x.bytes.empty()); };
      auto text = [&](const std::string &x) { lines.push_back(x); };
      for (auto b : sentence) f.feed(b, emit, text);
      auto binary = withText(protocol, sentence + sentence);
      if (corrupt) binary.back() ^= 1;
      for (auto b : binary) f.feed(b, emit, text);
      for (auto b : sentence) f.feed(b, emit, text);
      EXPECT_EQ(frames, corrupt ? 0 : 1);
      ASSERT_EQ(lines.size(), 2u);
      EXPECT_EQ(lines[0], sentence);
      EXPECT_EQ(lines[1], sentence);
    }
  }
}
TEST(BinaryFramer, CommandEchoAndTruncatedBinaryCannotInjectText) {
  for (auto protocol : {Protocol::Ubx, Protocol::Sbf, Protocol::Novatel}) {
    Framer f(protocol);
    int lines = 0;
    auto emit = [](const Frame &) {};
    auto text = [&](const std::string &) { ++lines; };
    for (auto b : std::string("reply $GPGGA,120000*00\r\n")) f.feed(b, emit, text);
    EXPECT_EQ(lines, 0);
    auto binary = withText(protocol, "$GPGGA,120000*00\r\n");
    binary.pop_back();
    for (auto b : binary) f.feed(b, emit, text);
    f.expire(emit);
    EXPECT_EQ(lines, 0);
    for (auto b : std::string("$GPGGA,120001*00\r\n")) f.feed(b, emit, text);
    EXPECT_EQ(lines, 1);
  }
}

TEST(ReceiverTimeQuality, ReleaseN2Q3HeaderTimeDoesNotRequireFineTimeHistory) {
  NovatelTimeQuality quality;
  Frame frame; frame.week = 2300; frame.tow_ms = 100000;
  const auto now = NovatelTimeQuality::Clock::now();
  frame.short_header = true;
  EXPECT_TRUE(quality.observe(frame, now));
  frame.short_header = false;
  for (uint8_t status : {60,80,100,120,130,140,200,255}) {
    frame.time_status = status;
    EXPECT_TRUE(quality.observe(frame, now));
  }
  for (uint8_t status : {160,170,180}) {
    frame.short_header = false; frame.time_status = status;
    EXPECT_TRUE(quality.observe(frame, now));
    frame.short_header = true;
    EXPECT_TRUE(quality.observe(frame, now + std::chrono::seconds(1)));
    EXPECT_TRUE(quality.observe(frame, now + std::chrono::seconds(6)));
    frame.week = 2301; EXPECT_TRUE(quality.observe(frame, now)); frame.week = 2300;
  }
  frame.short_header = false; frame.time_status = 20;
  EXPECT_FALSE(quality.observe(frame, now));
  frame.short_header = true;
  EXPECT_TRUE(quality.observe(frame, now)); // no status field in short header
  frame.short_header = false; frame.time_status = 180;
  EXPECT_TRUE(quality.observe(frame, now));
  quality.reset(); frame.short_header = true;
  EXPECT_TRUE(quality.observe(frame, now));
  // Neither full nor short headers may publish an invalid GNSS epoch.
  for (bool short_header : {false, true}) {
    frame.short_header = short_header;
    frame.week = 0; EXPECT_FALSE(quality.observe(frame, now));
    frame.week = 2300; frame.tow_ms = 604800000;
    EXPECT_FALSE(quality.observe(frame, now));
    frame.tow_ms = 0; EXPECT_TRUE(quality.observe(frame, now));
  }
}

TEST(ReleaseL1, CorruptLongCandidateRecoversWithBoundedBuffer) {
  Framer f(Protocol::Ubx); int count=0;
  auto emit=[&](const Frame&){++count;};
  // Maximum-length corrupt candidate followed by a valid ACK. Expiration
  // scans the whole poisoned prefix, exercising amortized compaction.
  const std::vector<uint8_t> prefix{0xb5,0x62,0x01,0x01,0xff,0xff};
  for(auto c:prefix) f.feed(c,emit);
  for(int i=0;i<65000;++i) f.feed(uint8_t(0x7f),emit);
  for(auto c:ubx()) f.feed(c,emit);
  EXPECT_LE(f.buffered(),65543u);
  f.expire(emit);
  EXPECT_EQ(count,1); EXPECT_EQ(f.buffered(),0u);
}
