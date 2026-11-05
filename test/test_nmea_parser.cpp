#include "gnss_ros_standardization/gnss_utils.hpp"
#include <gtest/gtest.h>
using gnss_utils::NmeaParser;
using Sol = gnss_ros_standardization::msg::GnssSolution;
namespace {
std::string gga(std::string time = "000000", std::string lat = "0000.000",
                std::string lon = "00000.000", std::string altitude = "100.0") {
  return "$GPGGA," + time + "," + lat + ",N," + lon + ",E,1,12,1.0," + altitude + ",M,30.0,M,,";
}
std::string rmc(std::string time = "000000", std::string date = "040224", char status = 'A') {
  return "$GPRMC," + time + "," + status + ",0000.000,N,00000.000,E,1.0,0.0," + date + ",,,A";
}
Sol epoch(const std::vector<std::string>& sentences) {
  NmeaParser parser;
  Sol result;
  for (const auto& sentence : sentences) EXPECT_FALSE(parser.parseSentence(sentence, result));
  EXPECT_TRUE(parser.parseSentence(gga("000001"), result));
  return result;
}
}
TEST(NmeaParser, MidnightAndSentenceOrderHaveSameAbsoluteTime) {
  for (const auto& input : {std::vector<std::string>{gga(), rmc()},
                           std::vector<std::string>{rmc(), gga()}}) {
    auto s = epoch(input);
    EXPECT_EQ(s.time_week, 2300u);
    EXPECT_DOUBLE_EQ(s.time_tow, 18.0);
    EXPECT_EQ(s.status, Sol::STATUS_SINGLE);
    EXPECT_DOUBLE_EQ(s.latitude, 0);
    EXPECT_DOUBLE_EQ(s.longitude, 0);
    EXPECT_NEAR(s.pos_ecef.x, 6378137 + 130, 1e-6);
    EXPECT_NEAR(s.pos_ecef.y, 0, 1e-6);
    EXPECT_NEAR(s.vel_enu.y, .514444, 1e-6);
  }
}
// GGA quality 6 is an estimated (dead-reckoning) position: STATUS_DR, kept.
TEST(NmeaParser, GgaDeadReckoningIsDr) {
  NmeaParser parser;
  Sol s;
  const std::string dr = "$GPGGA,000000,0000.000,N,00000.000,E,6,00,,100.0,M,30.0,M,,";
  EXPECT_FALSE(parser.parseSentence(dr, s));
  EXPECT_TRUE(parser.parseSentence(gga("000001"), s));
  EXPECT_EQ(s.status, Sol::STATUS_DR);
  EXPECT_NEAR(s.pos_ecef.x, 6378137 + 130, 1e-6);
}
TEST(NmeaParser, NoDateIsUnknownAndExplicitReferenceIsSupported) {
  auto s = epoch({gga()});
  EXPECT_EQ(s.time_week, 0u);
  NmeaParser parser;
  EXPECT_FALSE(parser.setReferenceDate(2023, 2, 29));
  ASSERT_TRUE(parser.setReferenceDate(2024, 2, 4));
  EXPECT_FALSE(parser.parseSentence(gga(), s));
  EXPECT_TRUE(parser.parseSentence(gga("000001"), s));
  EXPECT_EQ(s.time_week, 2300u);
  EXPECT_DOUBLE_EQ(s.time_tow, 18);
}
TEST(NmeaParser, RmcAndZdaDatesUseCalendarAndGnssEra) {
  auto historical = epoch({rmc("000000", "010199"), gga()});
  uint32_t week; double tow;
  ASSERT_TRUE(gnss_utils::nmeaUtcToGpsTime(1999, 1, 1, 0, week, tow));
  EXPECT_EQ(historical.time_week, week);
  EXPECT_DOUBLE_EQ(historical.time_tow, tow);
  auto leap = epoch({"$GPZDA,000000,29,02,2024,00,00", gga()});
  ASSERT_TRUE(gnss_utils::nmeaUtcToGpsTime(2024, 2, 29, 0, week, tow));
  EXPECT_EQ(leap.time_week, week);
  EXPECT_DOUBLE_EQ(leap.time_tow, tow);
  auto invalid = epoch({rmc("000000", "290223"), gga()});
  EXPECT_EQ(invalid.time_week, 0u);
}
TEST(NmeaParser, DateRollsAcrossYearWithoutHostTimezone) {
  NmeaParser parser; Sol s;
  parser.parseSentence(rmc("235959", "311299"), s);
  parser.parseSentence(gga("235959"), s);
  ASSERT_TRUE(parser.parseSentence(gga(), s));
  auto before = gpst2time(s.time_week, s.time_tow);
  ASSERT_TRUE(parser.parseSentence(rmc("000000", "010100"), s));
  EXPECT_DOUBLE_EQ(timediff(gpst2time(s.time_week, s.time_tow), before), 1);
  // GGA-only next day remains anchored to the advanced date.
  parser.parseSentence(gga("000001"), s);
  ASSERT_TRUE(parser.parseSentence(gga("000002"), s));
  EXPECT_DOUBLE_EQ(timediff(gpst2time(s.time_week, s.time_tow), before), 2);
}
TEST(NmeaParser, MissingMalformedAndOutOfRangeCoordinatesCannotBecomeFix) {
  for (const std::string lat : {"", "bad", "3500.0junk", "3560.0", "9100.0", "-3500.0"}) {
    auto s = epoch({rmc(), gga("000000", lat)});
    EXPECT_EQ(s.status, Sol::STATUS_NONE) << lat;
    EXPECT_TRUE(std::isnan(s.latitude)) << lat;
    EXPECT_TRUE(std::isnan(s.pos_ecef.x)) << lat;
  }
  auto missing_height = epoch({rmc(), gga("000000", "0000.000", "00000.000", "")});
  EXPECT_TRUE(std::isnan(missing_height.altitude));
  EXPECT_TRUE(std::isnan(missing_height.pos_ecef.x));
}
TEST(NmeaParser, MissingCovarianceAndInvalidVelocityStayUnknown) {
  auto s = epoch({rmc("000000", "040224", 'V'), gga(), "$GPGST,000000,1,1,1,0,,,"});
  EXPECT_TRUE(std::isnan(s.vel_enu.x));
  EXPECT_TRUE(std::isnan(s.vel_ecef.x));
  EXPECT_TRUE(std::isnan(s.pos_enu_cov[0]));
  s = epoch({rmc(), gga(), "$GPGST,000000,1,1,1,0,1e300,1,1"});
  EXPECT_TRUE(std::isnan(s.pos_enu_cov[0]));
  s = epoch({rmc(), gga(), "$GPGST,000000,1,1,1,0,0,0,0"});
  EXPECT_DOUBLE_EQ(s.pos_enu_cov[0], 0);
}
TEST(NmeaParser, TruncatedChecksumAndInvalidTimeCannotUpdateEpoch) {
  for (const auto& bad : {gga()+"*", gga()+"*0", gga()+"*0Z", gga()+"*00junk",
                          gga(""), gga("240000"), gga("126000"), gga("1e100"), gga("nan")}) {
    NmeaParser parser; Sol s;
    EXPECT_FALSE(parser.parseSentence(bad, s));
    // If the invalid input created an epoch, this would prematurely flush it.
    EXPECT_FALSE(parser.parseSentence(gga("000001"), s)) << bad;
  }
}
TEST(NmeaParser, CalendarHelperRejectsInvalidDatesAndTimes) {
  uint32_t week; double tow;
  EXPECT_FALSE(gnss_utils::nmeaUtcToGpsTime(2023, 2, 29, 0, week, tow));
  EXPECT_FALSE(gnss_utils::nmeaUtcToGpsTime(2024, 4, 31, 0, week, tow));
  EXPECT_FALSE(gnss_utils::nmeaUtcToGpsTime(2024, 1, 1, 126000, week, tow));
  EXPECT_FALSE(gnss_utils::nmeaUtcToGpsTime(2024, 1, 1, INFINITY, week, tow));
  EXPECT_TRUE(gnss_utils::nmeaUtcToGpsTime(2024, 2, 29, 0, week, tow));
}

TEST(NmeaParser, ReceiverDateExpiresButExplicitReferenceSurvivesReset) {
  using namespace std::chrono_literals;
  const auto start=NmeaParser::Clock::now();
  NmeaParser parser; Sol result;
  parser.parseSentence(rmc(),result,start);
  parser.parseSentence(gga(),result,start);
  parser.parseSentence(gga("000001"),result,start+1s);
  EXPECT_EQ(result.time_week,2300u);
  parser.parseSentence(gga("000102"),result,start+62s);
  parser.parseSentence(gga("000103"),result,start+63s);
  EXPECT_EQ(result.time_week,0u);
  parser.reset();
  parser.parseSentence(gga(),result,start+64s);
  parser.parseSentence(gga("000001"),result,start+65s);
  EXPECT_EQ(result.time_week,0u);
  ASSERT_TRUE(parser.setReferenceDate(2024,2,4));
  parser.reset();
  parser.parseSentence(gga(),result,start+1000s);
  parser.parseSentence(gga("000001"),result,start+1001s);
  EXPECT_EQ(result.time_week,2300u);
  EXPECT_DOUBLE_EQ(result.time_tow,18.);
}
