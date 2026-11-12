// Unit tests of the rtk_bag_plot computations.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "decimate.hpp"
#include "encode.hpp"
#include "geometry.hpp"
#include "multipath.hpp"
#include "stats.hpp"

using namespace rtk_bag_plot;

namespace {
constexpr double kF1 = 1575.42e6, kF2 = 1227.60e6;

// Dual-frequency observation of a range with first-order ionosphere `iono` on
// L1, code multipath on L1 and carrier ambiguities.
void makeSamples(double range, double iono, double code_mp, double n1, double n2,
                 SignalSample& s1, SignalSample& s2) {
  const double gamma = (kF1 / kF2) * (kF1 / kF2);
  s1 = {range + iono + code_mp, (range - iono + n1) * kF1 / kSpeedOfLight, kF1};
  s2 = {range + gamma * iono, (range - gamma * iono + n2) * kF2 / kSpeedOfLight, kF2};
}
}  // namespace

TEST(BagPlotMultipath, IonosphereFromCarrierAndCodeMinusCarrier) {
  SignalSample s1, s2;
  makeSamples(2.1e7, 5.0, 0.7, 0.0, 0.0, s1, s2);
  double reference = 0.0, iono = 0.0;
  ASSERT_TRUE(referenceIonosphere({s1, s2}, reference, iono));
  EXPECT_EQ(reference, kF1);
  EXPECT_NEAR(iono, 5.0, 1e-6);
  EXPECT_NEAR(rawMultipath(s1, reference, iono), 0.7, 1e-6);
  EXPECT_NEAR(rawMultipath(s2, reference, iono), 0.0, 1e-6);
}

TEST(BagPlotMultipath, SingleFrequencyHasNoReference) {
  SignalSample s1{2.1e7, 1.1e8, kF1};
  double reference = 0.0, iono = 0.0;
  EXPECT_FALSE(referenceIonosphere({s1}, reference, iono));
  EXPECT_FALSE(referenceIonosphere({s1, {2.1e7, 1.1e8, kF1}}, reference, iono));
}

TEST(BagPlotMultipath, MissingCodeGivesNoValue) {
  SignalSample s{0.0, 1.0e8, kF1};
  EXPECT_EQ(rawMultipath(s, kF1, 1.0), 0.0);
}

TEST(BagPlotMultipath, ArcMeanRemoved) {
  std::vector<double> mp{100.0, 101.0, 102.0};
  removeArcMean(mp, {0, 0, 0});
  EXPECT_DOUBLE_EQ(mp[0], -1.0);
  EXPECT_DOUBLE_EQ(mp[1], 0.0);
  EXPECT_DOUBLE_EQ(mp[2], 1.0);
}

TEST(BagPlotMultipath, ArcRestartsOnSlip) {
  std::vector<double> mp{100.0, 102.0, 50.0, 54.0};
  removeArcMean(mp, {0, 0, 1, 0});
  EXPECT_DOUBLE_EQ(mp[0], -1.0);
  EXPECT_DOUBLE_EQ(mp[1], 1.0);
  EXPECT_DOUBLE_EQ(mp[2], -2.0);
  EXPECT_DOUBLE_EQ(mp[3], 2.0);
}

TEST(BagPlotMultipath, ArcRestartsOnJumpAboveFiveMetres) {
  std::vector<double> mp{10.0, 10.0, 20.0, 20.0};
  removeArcMean(mp, {0, 0, 0, 0});
  for (const double v : mp) EXPECT_NEAR(v, 0.0, 1e-12);
}

TEST(BagPlotMultipath, GeometryFreeJumpThreshold) {
  const double base = geometryFree(1.0e8, kF1, 7.8e7, kF2);
  const double one_cycle_l2 = geometryFree(1.0e8, kF1, 7.8e7 + 1.0, kF2);
  EXPECT_GT(std::fabs(one_cycle_l2 - base), kLgJumpThreshold);
}

TEST(BagPlotMultipath, SlantIonosphereUnits) {
  const double tecu = slantIonosphere(2.1e7, 2.1e7 + 1.0, kF1, kF2);
  EXPECT_NEAR(tecu, 9.52, 0.05);
  EXPECT_TRUE(std::isnan(slantIonosphere(0.0, 2.1e7, kF1, kF2)));
  EXPECT_TRUE(std::isnan(slantIonosphere(2.1e7, 2.1e7, kF1, kF1)));
}

TEST(BagPlotColors, SnrThresholdsMatchRtkplot) {
  const auto& marker = markerColors();
  EXPECT_EQ(snrColor(24.9), marker[7]);
  EXPECT_EQ(snrColor(25.0), marker[5]);
  EXPECT_EQ(snrColor(27.4), marker[5]);
  EXPECT_EQ(snrColor(47.6), marker[1]);
  EXPECT_EQ(snrColor(27.5), marker[5]);
  EXPECT_EQ(snrColor(32.5), marker[4]);
  EXPECT_EQ(snrColor(47.5), marker[1]);
}

TEST(BagPlotColors, SecondSolutionUsesRtkplotsSecondSet) {
  const auto& first = markerColors();
  const auto& second = secondMarkerColors();
  EXPECT_EQ(second[1], (Rgb{0x00, 0x40, 0x80}));
  EXPECT_EQ(second[2], (Rgb{0x80, 0x80, 0x00}));
  EXPECT_EQ(second[5], (Rgb{0xff, 0x80, 0x80}));
  for (const size_t q : {1U, 2U, 3U, 4U, 5U, 6U}) EXPECT_FALSE(first[q] == second[q]) << q;
}

TEST(BagPlotColors, MultipathAndIonosphereScales) {
  const auto& marker = markerColors();
  EXPECT_EQ(mpColor(1.0), marker[5]);
  EXPECT_EQ(mpColor(-1.0), marker[1]);
  EXPECT_EQ(mpColor(0.0), marker[3]);
  EXPECT_EQ(ionoColor(std::nan("")), marker[0]);
  EXPECT_EQ(ionoColor(-3.0), marker[1]);
  EXPECT_EQ(ionoColor(65.0), marker[7]);
}

TEST(BagPlotDecimate, KeepsExtremaEndsAndEvents) {
  std::vector<double> y(100000, 1.0);
  y[12345] = 99.0;
  y[54321] = -99.0;
  const DecimateChannel channel{[&](size_t i) { return y[i]; }, {}};
  const auto keep = selectRows(y.size(), 100, {channel}, [](size_t i) { return i == 777; });
  EXPECT_LT(keep.size(), 1000U);
  EXPECT_TRUE(std::is_sorted(keep.begin(), keep.end()));
  for (const uint32_t expected : {0U, 99999U, 12345U, 54321U, 777U})
    EXPECT_TRUE(std::binary_search(keep.begin(), keep.end(), expected)) << expected;
}

TEST(BagPlotDecimate, EventLimitBoundsReceiversThatFlagEveryEpoch) {
  const auto keep = selectRows(10000, 10, {}, [](size_t) { return true; }, 8);
  EXPECT_LE(keep.size(), 100U);
}

TEST(BagPlotDecimate, ChannelSkipsInvalidValues) {
  std::vector<int> y{0, 5, 0, 7, 0, 0};
  const DecimateChannel channel{[&](size_t i) { return double(y[i]); },
                                [&](size_t i) { return y[i] != 0; }};
  const auto keep = selectRows(y.size(), 1, {channel}, {});
  EXPECT_EQ(keep, (std::vector<uint32_t>{0U, 1U, 3U, 5U}));
}

TEST(BagPlotStats, MatchesRtkplotDefinition) {
  const auto s = computeAxisStats({1.0, 3.0, 5.0});
  EXPECT_EQ(s.n, 3U);
  EXPECT_DOUBLE_EQ(s.ave, 3.0);
  EXPECT_NEAR(s.std, 2.0, 1e-12);
  EXPECT_NEAR(s.rms, std::sqrt(35.0 / 3.0), 1e-12);
}

TEST(BagPlotStats, TableSeparatesQuality) {
  TimePos p;
  p.t = {0, 1, 2};
  p.x = {1, 2, 9};
  p.y = {0, 0, 0};
  p.z = {0, 0, 0};
  p.q = {1, 1, 2};
  const auto table = computeStatsTable(p);
  EXPECT_EQ(table[0][0].n, 3U);
  EXPECT_EQ(table[1][0].n, 2U);
  EXPECT_DOUBLE_EQ(table[1][0].ave, 1.5);
  EXPECT_DOUBLE_EQ(table[2][0].ave, 9.0);
}

TEST(BagPlotStats, DerivativeAtMidpointsAndStopsAtGaps) {
  TimePos p;
  p.t = {0, 1, 100};
  p.x = {0, 2, 3};
  p.y = {0, 0, 0};
  p.z = {0, 0, 0};
  p.q = {1, 2, 2};
  const auto v = timeDerivative(p);
  ASSERT_EQ(v.size(), 1U);
  EXPECT_DOUBLE_EQ(v.t[0], 0.5);
  EXPECT_DOUBLE_EQ(v.x[0], 2.0);
  EXPECT_EQ(v.q[0], 2);
}

TEST(BagPlotStats, DifferenceMatchesWithinTolerance) {
  TimePos a, b;
  a.t = {0.0, 1.0, 2.0};
  a.x = {1.0, 2.0, 3.0};
  a.y = a.z = {0.0, 0.0, 0.0};
  a.q = {1, 1, 1};
  b.t = {1.02, 2.5};
  b.x = {1.5, 7.0};
  b.y = b.z = {0.0, 0.0};
  b.q = {2, 2};
  const auto d = seriesDifference(a, b);
  ASSERT_EQ(d.size(), 1U);
  EXPECT_DOUBLE_EQ(d.t[0], 1.0);
  EXPECT_DOUBLE_EQ(d.x[0], 0.5);
  EXPECT_EQ(d.q[0], 2);
}

TEST(BagPlotEncode, Base64) {
  const std::string text = "Man";
  const auto* bytes = reinterpret_cast<const uint8_t*>(text.data());
  EXPECT_EQ(base64Encode(bytes, 3), "TWFu");
  EXPECT_EQ(base64Encode(bytes, 2), "TWE=");
  EXPECT_EQ(base64Encode(bytes, 1), "TQ==");
  EXPECT_EQ(base64Encode(bytes, 0), "");
}

TEST(BagPlotEncode, JsonStaysInsideScriptElement) {
  const std::string text = jsonString("</script>&\"\\\n");
  EXPECT_EQ(text.find('<'), std::string::npos);
  EXPECT_EQ(text.find('>'), std::string::npos);
  EXPECT_EQ(text.find('&'), std::string::npos);
  EXPECT_NE(text.find("\\\""), std::string::npos);
  EXPECT_EQ(jsonNumber(std::nan("")), "null");
  EXPECT_EQ(jsonNumber(1.5), "1.5");
}

TEST(BagPlotGeometry, MaskedDopUsesOnlySatellitesAboveMask) {
  const std::vector<double> azel{0.0,        1.2, 1.57, 0.5,  3.14, 0.5,
                                 4.71,       0.5, 0.8,  0.05, 5.5,  0.9};
  double all[4] = {0.0}, masked[4] = {0.0};
  EXPECT_EQ(maskedDop(azel, 0, all), 6);
  EXPECT_EQ(maskedDop(azel, 15, masked), 5);
  EXPECT_GT(masked[0], all[0]);
  double none[4] = {0.0};
  EXPECT_EQ(maskedDop({0.0, 0.1}, 45, none), 0);
  EXPECT_EQ(none[0], 0.0);
}

namespace {
RawRow makeRow(uint32_t epoch, uint32_t signal, double p, double l, uint8_t lli) {
  RawRow row;
  row.epoch = epoch;
  row.signal = signal;
  row.p = p;
  row.l = l;
  row.snr = 40.0F;
  row.lli = lli;
  return row;
}

SignalInfo makeSignal(int band, const char* code_str, uint8_t code) {
  SignalInfo info;
  info.sat = 1;
  info.sys = SYS_GPS;
  info.band = band;
  info.code = code;
  info.code_str = code_str;
  info.satid = "G01";
  info.system = "G";
  info.prn = 1;
  return info;
}
}  // namespace

TEST(BagPlotObservations, NormalizeSortsEpochsAndDropsDuplicates) {
  RawObservationTopic raw;
  raw.signals = {makeSignal(1, "2W", CODE_L2W), makeSignal(0, "1C", CODE_L1C)};
  raw.epoch_gpst = {1000.2, 1000.0, 1000.2, 1000.1};
  raw.rows = {makeRow(0, 0, 1.0, 1.0, 0), makeRow(1, 1, 2.0, 2.0, 0), makeRow(2, 0, 3.0, 3.0, 0),
              makeRow(3, 1, 4.0, 4.0, 0)};
  EXPECT_EQ(normalizeObservationTopic(raw), 1U);
  EXPECT_EQ(raw.epoch_gpst, (std::vector<double>{1000.0, 1000.1, 1000.2}));
  EXPECT_EQ(raw.signals[0].code_str, "1C");
  ASSERT_EQ(raw.rows.size(), 3U);
  EXPECT_DOUBLE_EQ(raw.rows[0].p, 2.0);
  EXPECT_DOUBLE_EQ(raw.rows[1].p, 4.0);
  EXPECT_DOUBLE_EQ(raw.rows[2].p, 1.0);
}

TEST(BagPlotObservations, EpochTimeSurvivesWeekRollover) {
  RawObservationTopic raw;
  raw.signals = {makeSignal(0, "1C", CODE_L1C)};
  raw.epoch_gpst = {2300.0 * 604800.0 + 604799.0, 2301.0 * 604800.0};
  raw.rows = {makeRow(0, 0, 2.1e7, 1.1e8, 0), makeRow(1, 0, 2.1e7, 1.1e8, 0)};
  normalizeObservationTopic(raw);
  const auto result = processObservationTopic(raw, nullptr, raw.epoch_gpst[0]);
  ASSERT_EQ(result.t.size(), 2U);
  EXPECT_DOUBLE_EQ(result.t[1], 1.0);
}

TEST(BagPlotObservations, FlagsAndMultipathPerSignal) {
  RawObservationTopic raw;
  raw.signals = {makeSignal(0, "1C", CODE_L1C), makeSignal(1, "2W", CODE_L2W)};
  raw.epoch_gpst = {1000.0, 1001.0, 1002.0};
  for (uint32_t e = 0; e < 3; ++e) {
    SignalSample s1, s2;
    makeSamples(2.1e7 + e * 100.0, 4.0, e == 1 ? 0.4 : 0.0, 0.0, 0.0, s1, s2);
    raw.rows.push_back(makeRow(e, 0, s1.p, s1.l, e == 1 ? 1 : 0));
    raw.rows.push_back(makeRow(e, 1, s2.p, s2.l, e == 2 ? 2 : 0));
  }
  normalizeObservationTopic(raw);
  const auto result = processObservationTopic(raw, nullptr, 1000.0);
  EXPECT_FALSE(result.has_geometry);
  ASSERT_EQ(result.signals.size(), 2U);
  const auto& l1 = result.signals[0];
  const auto& l2 = result.signals[1];
  EXPECT_EQ(l1.code, "1C");
  EXPECT_EQ(l1.flags[1] & kFlagSlip, kFlagSlip);
  EXPECT_EQ(l1.flags[0] & kFlagSlip, 0);
  EXPECT_EQ(l2.flags[2] & kFlagHalf, kFlagHalf);
  EXPECT_EQ(l1.flags[0] & (kFlagHasP | kFlagHasL), kFlagHasP | kFlagHasL);
  EXPECT_EQ(l1.snr[0], static_cast<uint8_t>(40.0 / kSnrScale));
  ASSERT_EQ(result.sats.size(), 1U);
  EXPECT_EQ(result.sats[0].id, "G01");
  // The slip at epoch 1 starts a new arc, so epoch 0 is alone in its arc.
  EXPECT_EQ(l1.mp[0], kMultipathInvalid);
}

TEST(BagPlotObservations, MissingCarrierStaysOutOfMultipath) {
  RawObservationTopic raw;
  raw.signals = {makeSignal(0, "1C", CODE_L1C), makeSignal(1, "2W", CODE_L2W)};
  raw.epoch_gpst = {1000.0};
  raw.rows = {makeRow(0, 0, 2.1e7, 0.0, 0), makeRow(0, 1, 2.1e7, 8.0e7, 0)};
  normalizeObservationTopic(raw);
  const auto result = processObservationTopic(raw, nullptr, 1000.0);
  EXPECT_EQ(result.signals[0].mp[0], kMultipathInvalid);
  EXPECT_EQ(result.signals[1].mp[0], kMultipathInvalid);
}
