#include <gtest/gtest.h>
#include "gnss_ros_standardization/gnss_utils.hpp"
#include <rclcpp/serialization.hpp>
#include "gnss_ros_standardization/decoder_common.hpp"
#include "gnss_ros_standardization/obs_converter.hpp"
#include "gnss_ros_standardization/rtklib_compat.hpp"

namespace {
using namespace gnss_utils;
TEST(ReleaseE6, ReceiverProjectionKeepsOriginAndCurrentVelocityFrame) {
  using gnss_ros_standardization::decoder_common::finalizeReceiverSolution;
  gnss_ros_standardization::msg::GnssSolution sol;
  bool initialized = false;
  double origin[3]{}, llh[3]{};
  sol.status = sol.STATUS_NONE;
  sol.pos_ecef.x = 6378137.;
  EXPECT_FALSE(finalizeReceiverSolution(sol, initialized, origin, llh));
  EXPECT_FALSE(initialized);
  EXPECT_TRUE(std::isnan(sol.pos_enu.x));
  EXPECT_TRUE(std::isnan(sol.pos_ecef.x));  // Q-A8: no solution, no position
  sol.status = sol.STATUS_SINGLE;
  sol.pos_ecef.x = 6378137.; sol.pos_ecef.y = sol.pos_ecef.z = 0.;
  sol.latitude = sol.longitude = sol.altitude = 0.;
  sol.vel_ecef.x = 1.; sol.vel_ecef.y = 2.; sol.vel_ecef.z = 3.;
  EXPECT_TRUE(finalizeReceiverSolution(sol, initialized, origin, llh));
  EXPECT_DOUBLE_EQ(sol.pos_enu.x, 0.);
  EXPECT_DOUBLE_EQ(sol.vel_enu.x, 2.);
  EXPECT_DOUBLE_EQ(sol.vel_enu.y, 3.);
  EXPECT_DOUBLE_EQ(sol.vel_enu.z, 1.);
  sol.longitude = 90.; sol.pos_ecef.x = 0.; sol.pos_ecef.y = 6378137.;
  EXPECT_FALSE(finalizeReceiverSolution(sol, initialized, origin, llh));
  EXPECT_DOUBLE_EQ(origin[0], 6378137.);
  EXPECT_DOUBLE_EQ(sol.pos_enu.x, 6378137.);
  EXPECT_NEAR(sol.vel_enu.x, -1., 1e-14);
  EXPECT_DOUBLE_EQ(sol.vel_enu.y, 3.);
  EXPECT_NEAR(sol.vel_enu.z, 2., 1e-14);
  sol.pos_ecef.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(finalizeReceiverSolution(sol, initialized, origin, llh));
  EXPECT_TRUE(std::isnan(sol.pos_enu.x));
  EXPECT_TRUE(std::isnan(sol.vel_ecef.x));
  EXPECT_DOUBLE_EQ(origin[0], 6378137.);
}
// Q-A8: a receiver epoch without a solution carries NaN position, whatever the
// receiver wrote. Q-A15: a dead-reckoned position is published but never
// becomes the ENU origin.
TEST(ReleaseE6, NoSolutionIsNaNAndDeadReckoningNeverDefinesTheOrigin) {
  using gnss_ros_standardization::decoder_common::finalizeReceiverSolution;
  using S = gnss_ros_standardization::msg::GnssSolution;
  S sol;
  bool initialized = false;
  double origin[3]{}, llh[3]{};
  sol.status = S::STATUS_NONE;
  sol.latitude = 35.; sol.longitude = 139.; sol.altitude = 10.;
  sol.pos_ecef.x = -3.9e6; sol.pos_ecef.y = 3.4e6; sol.pos_ecef.z = 3.6e6;
  sol.pos_cov_ecef.fill(1.);
  EXPECT_FALSE(finalizeReceiverSolution(sol, initialized, origin, llh));
  for (double v : {sol.latitude, sol.longitude, sol.altitude, sol.pos_ecef.x,
                   sol.pos_ecef.y, sol.pos_ecef.z, sol.pos_cov_ecef[0], sol.vel_ecef.x})
    EXPECT_TRUE(std::isnan(v));

  sol.status = S::STATUS_DR;
  sol.latitude = 0.; sol.longitude = 0.; sol.altitude = 0.;
  sol.pos_ecef.x = 6378137.; sol.pos_ecef.y = 0.; sol.pos_ecef.z = 0.;
  sol.vel_ecef.x = 1.; sol.vel_ecef.y = 2.; sol.vel_ecef.z = 3.;
  EXPECT_FALSE(finalizeReceiverSolution(sol, initialized, origin, llh));
  EXPECT_FALSE(initialized);
  EXPECT_DOUBLE_EQ(sol.pos_ecef.x, 6378137.);
  EXPECT_TRUE(std::isnan(sol.pos_enu.x));
  EXPECT_DOUBLE_EQ(sol.vel_enu.x, 2.);

  sol.status = S::STATUS_SINGLE;
  EXPECT_TRUE(finalizeReceiverSolution(sol, initialized, origin, llh));
  sol.status = S::STATUS_DR;
  sol.pos_ecef.z = 5.;
  EXPECT_FALSE(finalizeReceiverSolution(sol, initialized, origin, llh));
  EXPECT_NEAR(sol.pos_enu.z, 0., 1e-9);
  EXPECT_NEAR(sol.pos_enu.y, 5., 1e-9);
}

TEST(ReleaseF9, RtkEpochCannotReusePreviousVelocityCovarianceWhenSppIsSkipped) {
  prcopt_t options = prcopt_default;
  options.mode = PMODE_SINGLE;
  rtk_t backend{};
  rtkinit(&backend, &options);
  // This is the actual pinned backend's precise-position branch: it skips
  // pntpos and therefore produces no current Doppler velocity. Seed the
  // previous solution to expose the stale-covariance problem.
  backend.P[0] = 1e-6;
  backend.sol.rr[0] = 6378137.;
  backend.sol.rr[3] = 3.;
  backend.sol.qv[0] = 4.; backend.sol.qv[1] = 5.; backend.sol.qv[2] = 6.;
  obsd_t observation{};
  observation.sat = satno(SYS_GPS, 1); observation.rcv = 1;
  observation.time = gpst2time(2300, 100.);
  nav_t navigation{};
  EXPECT_EQ(rtkpos(&backend, &observation, 1, &navigation), 1);
  EXPECT_FLOAT_EQ(backend.sol.qv[0], 4.);  // reproduce the unchanged backend
  observation.time = timeadd(observation.time, 1.);
  EXPECT_EQ(rtkPositionEpoch(backend, &observation, 1, navigation), 1);
  for (float covariance : backend.sol.qv) EXPECT_FLOAT_EQ(covariance, 0.);
  EXPECT_DOUBLE_EQ(backend.P[0], 1e-6);
  EXPECT_DOUBLE_EQ(backend.sol.rr[0], 6378137.);
  EXPECT_DOUBLE_EQ(backend.sol.rr[3], 3.); // do not corrupt filter initialization
  rtkfree(&backend);
}
obsd_t sample() {
  obsd_t o{};
  o.sat = satno(SYS_GPS, 1); o.code[0] = CODE_L1C;
  o.P[0] = 21000000.125; o.L[0] = -110000000.25;
  o.D[0] = -1234.5f; o.SNR[0] = 45.25f; o.LLI[0] = 0x83;
  return o;
}
TEST(ObservationValidation, NormalValuesAndBackendLliRemainUnchanged) {
  const auto raw = sample(); const auto out = obsToMsg(raw, 0);
  EXPECT_EQ(out.system, "G"); EXPECT_EQ(out.prn, 1); EXPECT_EQ(out.code_str, "1C");
  EXPECT_EQ(out.p, raw.P[0]); EXPECT_EQ(out.l, raw.L[0]);
  EXPECT_EQ(out.d, raw.D[0]); EXPECT_EQ(out.snr, raw.SNR[0]); EXPECT_EQ(out.lli, raw.LLI[0]);
}
TEST(ObservationValidation, BeidouC63AndAllConstellationSatelliteIdsRoundTrip) {
  ASSERT_EQ(MAXPRNCMP,63);
  ASSERT_LE(MAXSAT,255); // obsd_t::sat is uint8_t
  EXPECT_EQ(satno(SYS_CMP,64),0);
  for(int satellite=1;satellite<=MAXSAT;++satellite) {
    int prn=0;const int system=satsys(satellite,&prn);
    ASSERT_NE(system,SYS_NONE);EXPECT_EQ(satno(system,prn),satellite);
    char id[16]{};satno2id(satellite,id);EXPECT_EQ(satid2no(id),satellite);
  }
  for(int prn:{1,50,51,59,60,62,63}) {
    auto raw=sample();raw.sat=satno(SYS_CMP,prn);raw.code[0]=CODE_L2I;
    const auto msg=obsToMsg(raw,0);
    ASSERT_EQ(msg.system,"C");EXPECT_EQ(msg.prn,prn);
    const auto restored=convertObs(msg,gpst2time(2300,100),1,FrequencyMask{});
    EXPECT_EQ(restored.sat,raw.sat);EXPECT_DOUBLE_EQ(restored.P[0],raw.P[0]);
    EXPECT_DOUBLE_EQ(restored.L[0],raw.L[0]);EXPECT_EQ(restored.LLI[0],raw.LLI[0]);
  }
}
TEST(ObservationValidation, AllLliBitsSurvivePublicationCdrAndEstimatorInput) {
  rclcpp::Serialization<gnss_ros_standardization::msg::GnssObservations> serializer;
  for (unsigned int bits = 0; bits <= 255; ++bits) {
    SCOPED_TRACE(bits);
    auto raw = sample(); raw.LLI[0] = static_cast<uint8_t>(bits);
    gnss_ros_standardization::msg::GnssObservations in, out;
    in.week = 2300; in.tow = 100.;
    gnss_ros_standardization::decoder_common::appendObservations(raw, in.observations);
    ASSERT_EQ(in.observations.size(), 1u);
    EXPECT_EQ(in.observations[0].lli, bits);
    rclcpp::SerializedMessage bytes;
    serializer.serialize_message(&in, &bytes);
    serializer.deserialize_message(&bytes, &out);
    ASSERT_EQ(out.observations.size(), 1u);
    EXPECT_EQ(out.observations[0].lli, bits);
    const auto epoch = buildObsEpoch(out, nullptr, FrequencyMask{});
    ASSERT_EQ(epoch.size(), 1u);
    EXPECT_EQ(epoch[0].LLI[0], bits);
    auto changed = out.observations[0]; changed.lli ^= LLI_HALFA;
    EXPECT_FALSE(sameObservation(changed, out.observations[0]));
  }
}
TEST(ObservationValidation, BadPDoesNotDiscardOtherComponents) {
  for (double bad : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), -1.}) {
    auto raw=sample(); raw.P[0]=bad;
    const auto out=obsToMsg(raw,0);
    EXPECT_TRUE(std::isnan(out.p)); EXPECT_EQ(out.l,raw.L[0]); EXPECT_EQ(out.d,raw.D[0]);
  }
}
TEST(ObservationValidation, GlonassFrequencyDefaultsEncodingCdrAndImport) {
  gnss_ros_standardization::msg::GnssObservation empty;
  EXPECT_EQ(empty.freq, 255);
  auto raw = sample(); raw.sat = satno(SYS_GLO, 1);
  for (int encoded : {0, 7, 13, 14, 255}) {
    raw.freq = encoded;
    const auto message = obsToMsg(raw, 0);
    EXPECT_EQ(message.freq, encoded <= 13 ? encoded : 255);
    gnss_ros_standardization::msg::GnssObservation restored;
    rclcpp::Serialization<gnss_ros_standardization::msg::GnssObservation> ser;
    rclcpp::SerializedMessage bytes;
    ser.serialize_message(&message, &bytes); ser.deserialize_message(&bytes, &restored);
    EXPECT_EQ(restored.freq, message.freq);
    EXPECT_EQ(convertObs(restored, {}, 1, FrequencyMask{}).freq, message.freq);
    restored.freq = message.freq == 7 ? 0 : 7;
    EXPECT_FALSE(sameObservation(message, restored));
  }
  raw = sample(); raw.freq = 7;
  EXPECT_EQ(obsToMsg(raw, 0).freq, 255); // Not GLONASS.
}
TEST(ObservationValidation, GlonassFrequencyUsesRecordThenTimedNavigationNeverUnagedCache) {
  auto raw = sample(); raw.sat = satno(SYS_GLO, 1); raw.freq = 255;
  nav_t nav{}; geph_t g{}; g.sat = raw.sat; g.frq = 6;
  nav.geph = &g; nav.ng = 1;
  EXPECT_EQ(obsToMsg(raw, 0, &nav).freq, 13);
  nav.glo_fcn[0] = 8;
  EXPECT_EQ(obsToMsg(raw, 0, &nav).freq, 13);
  raw.freq = 0;
  EXPECT_EQ(obsToMsg(raw, 0, &nav).freq, 0);
  raw.freq = 255; nav.glo_fcn[0] = 255; g.frq = -8;
  EXPECT_EQ(obsToMsg(raw, 0, &nav).freq, 255);
  nav.glo_fcn[0] = 8; nav.ng = 0;
  EXPECT_EQ(obsToMsg(raw, 0, &nav).freq, 255);
}
TEST(ObservationValidation, GlonassSignalMergeRejectsConflictingChannels) {
  auto raw = sample(); raw.sat = satno(SYS_GLO, 1); raw.freq = 7;
  gnss_ros_standardization::msg::GnssObservations in;
  in.week = 2300; in.tow = 100.;
  in.observations = {obsToMsg(raw, 0), obsToMsg(raw, 0)};
  in.observations[0].freq = 255;
  auto epoch = buildObsEpoch(in, nullptr, FrequencyMask{});
  ASSERT_EQ(epoch.size(), 1u); EXPECT_EQ(epoch[0].freq, 7);
  in.observations[0].freq = 0;
  EXPECT_TRUE(buildObsEpoch(in, nullptr, FrequencyMask{}).empty());
}
TEST(ObservationValidation, TimedGlonassNavigationMatchesFrequencyAndDoesNotMutateStore) {
  auto raw = sample(); raw.sat = satno(SYS_GLO, 1); raw.freq = 255;
  raw.time = gpst2time(2300, 10000);
  geph_t records[2]{};
  for (auto& g : records) g.sat = raw.sat;
  records[0].toe = timeadd(raw.time, -MAXDTOE_GLO - 1); records[0].frq = -7;
  records[1].toe = raw.time; records[1].frq = 6;
  nav_t nav{}; nav.geph = records; nav.ng = 2; nav.glo_fcn[0] = 1;
  // Reproduce the backend hazard: wavelength uses the first, stale record.
  EXPECT_DOUBLE_EQ(sat2freq(raw.sat, CODE_L1C, &nav), code2freq(SYS_GLO, CODE_L1C, -7));
  EXPECT_EQ(obsToMsg(raw, 0, &nav).freq, 13);
  GlonassNavigationView view(nav, raw.time);
  std::vector<obsd_t> obs{raw};
  ObservationRejectCounters rejected;
  view.filter(obs, rejected);
  ASSERT_EQ(obs.size(), 1u); EXPECT_EQ(obs[0].freq, 13); EXPECT_EQ(rejected.total(), 0u);
  EXPECT_DOUBLE_EQ(sat2freq(raw.sat, CODE_L1C, &view.get()), code2freq(SYS_GLO, CODE_L1C, 6));
  EXPECT_EQ(nav.geph, records); EXPECT_EQ(nav.ng, 2); EXPECT_EQ(nav.glo_fcn[0], 1);
  EXPECT_EQ(raw.freq, 255); // Original input is not rewritten.
  std::swap(records[0], records[1]);
  EXPECT_EQ(obsToMsg(raw, 0, &nav).freq, 13); // No array-order dependency.
}
TEST(ObservationValidation, GlonassChannelAgreementUnknownAndConflictAtBothReceivers) {
  auto raw = sample(); raw.sat = satno(SYS_GLO, 1); raw.time = gpst2time(2300, 100.);
  geph_t g{}; g.sat = raw.sat; g.toe = raw.time; g.frq = -7;
  nav_t nav{}; nav.geph = &g; nav.ng = 1;
  GlonassNavigationView view(nav, raw.time);
  for (int channel : {0, 255, 7, 13}) {
    std::vector<obsd_t> obs(2, raw);
    obs[0].rcv = 1; obs[0].freq = 0;
    obs[1].rcv = 2; obs[1].freq = channel;
    ObservationRejectCounters rejected;
    view.filter(obs, rejected);
    EXPECT_EQ(obs.size(), (channel == 0 || channel == 255) ? 2u : 1u);
    EXPECT_EQ(rejected.navigation_channel_conflict, (channel == 0 || channel == 255) ? 0u : 1u);
    for (auto& o : obs) EXPECT_EQ(o.freq, 0);
  }
  raw.freq = 13; // Collection preserves the direct value despite nav conflict.
  EXPECT_EQ(obsToMsg(raw, 0, &nav).freq, 13);
}
TEST(ObservationValidation, GlonassMissingStaleInvalidAndAmbiguousNavigation) {
  auto raw = sample(); raw.sat = satno(SYS_GLO, 1); raw.freq = 13;
  raw.time = gpst2time(2300, 100.);
  nav_t nav{}; nav.glo_fcn[0] = 14;
  geph_t records[2]{};
  records[0].sat = records[1].sat = raw.sat;
  records[0].toe = records[1].toe = raw.time;
  records[0].frq = 6; records[1].frq = -7;
  for (int mode = 0; mode < 4; ++mode) {
    nav.geph = records; nav.ng = mode == 0 ? 0 : mode == 3 ? 2 : 1;
    records[0].toe = mode == 1 ? timeadd(raw.time, -MAXDTOE_GLO - 1) : raw.time;
    records[0].frq = mode == 2 ? -8 : 6;
    GlonassNavigationView view(nav, raw.time);
    std::vector<obsd_t> obs{raw, sample()};
    ObservationRejectCounters rejected;
    view.filter(obs, rejected);
    ASSERT_EQ(obs.size(), 1u); EXPECT_EQ(satsys(obs[0].sat, nullptr), SYS_GPS);
    EXPECT_EQ(rejected.total(), 1u);
    EXPECT_EQ(rejected.invalid_navigation_channel, mode == 2 ? 1u : 0u);
    auto unknown = raw; unknown.freq = 255;
    EXPECT_EQ(obsToMsg(unknown, 0, &nav).freq, 255);
  }
}
TEST(ObservationValidation, GlonassSelectionAgeBoundaryAndReverseReplay) {
  const auto time = gpst2time(2301, 0.);
  geph_t records[2]{};
  records[0].sat = records[1].sat = satno(SYS_GLO, 1);
  records[0].toe = timeadd(time, -100); records[0].frq = -7;
  records[1].toe = timeadd(time, 100); records[1].frq = 6;
  nav_t nav{}; nav.geph = records; nav.ng = 2;
  EXPECT_EQ(selectGlonassEphemeris(nav, records[0].sat, time), nullptr);
  for (int index : {1, 0, 1}) {
    GlonassNavigationView view(nav, records[index].toe);
    ASSERT_EQ(view.get().ng, 1); EXPECT_EQ(view.get().geph[0].frq, records[index].frq);
  }
  nav.ng = 1;
  EXPECT_NE(selectGlonassEphemeris(nav, records[0].sat, timeadd(records[0].toe, MAXDTOE_GLO)), nullptr);
  EXPECT_EQ(selectGlonassEphemeris(nav, records[0].sat, timeadd(records[0].toe, MAXDTOE_GLO + .001)), nullptr);
}
TEST(ObservationValidation, GlonassBaseEpochCannotUseAnotherEpochsChannel) {
  geph_t records[2]{};
  records[0].sat = records[1].sat = satno(SYS_GLO, 1);
  records[0].toe = gpst2time(2300, 100); records[0].frq = -7;
  records[1].toe = gpst2time(2300, 102); records[1].frq = 6;
  nav_t nav{}; nav.geph = records; nav.ng = 2;
  GlonassNavigationView view(nav, records[0].toe);
  auto base = sample(); base.sat = records[1].sat; base.rcv = 2;
  base.time = records[1].toe; base.freq = 255;
  std::vector<obsd_t> obs{base}; ObservationRejectCounters rejected;
  view.filter(obs, rejected);
  EXPECT_TRUE(obs.empty()); EXPECT_EQ(rejected.navigation_channel_conflict, 1u);
}
TEST(ObservationValidation, ImportRejectsInvalidEpochBeforeBackendTimeConversion) {
  gnss_ros_standardization::msg::GnssObservations epoch;
  epoch.week = 2300; epoch.observations = {obsToMsg(sample(), 0)};
  for (double tow : {-1., 604800., std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    epoch.tow = tow; ObservationRejectCounters rejected;
    EXPECT_TRUE(buildObsEpoch(epoch, nullptr, FrequencyMask{}, &rejected).empty());
    EXPECT_EQ(rejected.invalid_epoch, 1u);
  }
  epoch.tow = 0; epoch.week = UINT32_MAX;
  EXPECT_TRUE(buildObsEpoch(epoch, nullptr, FrequencyMask{}).empty());
  epoch.week = 0; EXPECT_EQ(buildObsEpoch(epoch, nullptr, FrequencyMask{}).size(), 1u);
  auto base = epoch; base.tow = NAN;
  EXPECT_TRUE(buildObsEpoch(epoch, &base, FrequencyMask{}).empty());
}
TEST(ObservationValidation, ChannelTransitionMarksPrivatePhaseSlipForEachReceiver) {
  auto raw = sample(); raw.sat = satno(SYS_GLO, 1); raw.freq = 0; raw.LLI[0] = 0xc0;
  GlonassChannelTracker tracker;
  int previous_channel = 0;
  for (int channel : {0, 13, 13, 0}) {
    std::vector<obsd_t> obs(2, raw);
    obs[0].rcv = 1; obs[0].freq = channel; obs[1].rcv = 2;
    tracker.apply(obs);
    EXPECT_EQ(obs[1].LLI[0], 0xc0); // Base did not change.
    EXPECT_EQ(obs[0].LLI[0], channel != previous_channel ? 0xc1 : 0xc0);
    previous_channel = channel;
  }
  tracker.clear();
  std::vector<obsd_t> obs{raw}; obs[0].rcv = 1; obs[0].freq = 7;
  tracker.apply(obs);
  EXPECT_EQ(obs[0].LLI[0], 0xc0);
}
TEST(ObservationValidation, ChannelTransitionRemainsPendingAcrossMissingPhase) {
  auto raw = sample(); raw.sat = satno(SYS_GLO, 1); raw.freq = 0; raw.LLI[0] = 0x80;
  GlonassChannelTracker tracker;
  std::vector<obsd_t> obs{raw}; tracker.apply(obs);
  obs[0].freq = 13; obs[0].L[0] = 0; tracker.apply(obs);
  obs[0].L[0] = raw.L[0]; tracker.apply(obs);
  EXPECT_EQ(obs[0].LLI[0], 0x81);
}
TEST(ObservationValidation, NonFiniteComponentsBecomeNanIndependently) {
  auto raw=sample();raw.L[0]=NAN;raw.D[0]=INFINITY;raw.SNR[0]=-1;
  const auto out=obsToMsg(raw,0);
  EXPECT_EQ(out.p,raw.P[0]);EXPECT_TRUE(std::isnan(out.l));EXPECT_TRUE(std::isnan(out.d));EXPECT_TRUE(std::isnan(out.snr));
}
TEST(ObservationValidation, ExtraSlotsAndUnsupportedIdentity) {
  auto raw=sample();raw.code[NFREQ+NEXOBS-1]=CODE_L2W;raw.L[NFREQ+NEXOBS-1]=3;
  std::vector<gnss_ros_standardization::msg::GnssObservation> out;
  gnss_ros_standardization::decoder_common::appendObservations(raw,out);
  ASSERT_EQ(out.size(),2u);EXPECT_EQ(out.back().code_str,"2W");EXPECT_EQ(out.back().l,3);
  raw.code[0]=255;out.clear();
  gnss_ros_standardization::decoder_common::appendObservations(raw,out);
  EXPECT_EQ(out.size(),1u);EXPECT_EQ(obsToMsg(raw,0).sat,0);
  EXPECT_THROW(obsToMsg(raw,-1),std::out_of_range);
  EXPECT_THROW(obsToMsg(raw,NFREQ+NEXOBS),std::out_of_range);
  raw.sat=0;out.clear();gnss_ros_standardization::decoder_common::appendObservations(raw,out);
  EXPECT_TRUE(out.empty());
}
TEST(ObservationValidation, CodedZeroMeasurementsAndLliAreNotEmptySlots) {
  obsd_t raw{};
  raw.sat=satno(SYS_GPS,1);raw.code[0]=CODE_L1C;raw.LLI[0]=3;
  raw.code[NFREQ+NEXOBS-1]=CODE_L2W;raw.LLI[NFREQ+NEXOBS-1]=0x80;
  // Nonzero data without a code is still not an identified observation.
  raw.P[1]=21000000.;
  std::vector<gnss_ros_standardization::msg::GnssObservation> out;
  gnss_ros_standardization::decoder_common::appendObservations(raw,out);
  ASSERT_EQ(out.size(),2u);
  EXPECT_EQ(out[0].code_str,"1C");EXPECT_EQ(out[0].lli,3);
  EXPECT_EQ(out[1].code_str,"2W");EXPECT_EQ(out[1].lli,0x80);
  for (const auto& obs:out) {
    EXPECT_DOUBLE_EQ(obs.p,0.);EXPECT_DOUBLE_EQ(obs.l,0.);
    EXPECT_DOUBLE_EQ(obs.d,0.);EXPECT_DOUBLE_EQ(obs.snr,0.);
  }
}
TEST(ObservationValidation, ExistingSchemaCdrRoundtrip) {
  gnss_ros_standardization::msg::GnssObservations in,out;
  in.week=2300;in.tow=100.;in.observations.push_back(obsToMsg(sample(),0));
  rclcpp::Serialization<gnss_ros_standardization::msg::GnssObservations> serializer;
  rclcpp::SerializedMessage bytes;
  serializer.serialize_message(&in,&bytes);serializer.deserialize_message(&bytes,&out);
  EXPECT_EQ(in,out);
}

TEST(ObservationValidation, StableIdentitiesAndConflicts) {
  for (const auto& pair : {std::pair<int, int>{SYS_GPS, 1},
                          {SYS_GLO, 1},
                          {SYS_GAL, 1},
                          {SYS_QZS, 193},
                          {SYS_CMP, 1},
                          {SYS_IRN, 1},
                          {SYS_SBS, 120}}) {
    const int sat = satno(pair.first, pair.second);
    if (!sat)
      continue; // Optional constellations may be disabled at build time.
    EXPECT_EQ(resolveSatellite("", systemCode(pair.first), pair.second), sat);
    EXPECT_EQ(resolveSatellite(satId(sat), systemCode(pair.first), pair.second),
              sat);
  }
  EXPECT_EQ(resolveSatellite("", "J", 1), satno(SYS_QZS, 193));
  EXPECT_EQ(resolveSatellite("", "S", 20), satno(SYS_SBS, 120));
  EXPECT_EQ(resolveSatellite("G01", "", 1), satno(SYS_GPS, 1));
  EXPECT_EQ(resolveSatellite("G01", "", 2), 0);
  EXPECT_EQ(resolveSatellite("G01", "G", 2), 0);
  EXPECT_EQ(resolveSatellite("G01", "E", 1), 0);
  EXPECT_EQ(resolveSatellite("G01junk", "", 0), 0);
  EXPECT_EQ(resolveSatellite("G99999999999999999999", "", 0), 0);
  EXPECT_EQ(resolveSignal("1C", CODE_L2W), 0);
  EXPECT_EQ(resolveSignal("1Cjunk", 0), 0);
  EXPECT_EQ(resolveSignal("1C", 0), CODE_L1C);
}

TEST(ObservationValidation,
     ExistingEstimatorUsesStableIdentityAndRejectsBadComponents) {
  auto in = obsToMsg(sample(), 0);
  in.sat = 250;
  in.satid.clear();
  in.code = 0;
  in.p = NAN;
  in.d = INFINITY;
  auto out = convertObs(in, gpst2time(2300, 100), 1, FrequencyMask{});
  EXPECT_EQ(out.sat, satno(SYS_GPS, 1));
  EXPECT_EQ(out.P[0], 0);
  EXPECT_EQ(out.D[0], 0);
  EXPECT_EQ(out.L[0], in.l);
  in.satid = "G02";
  EXPECT_EQ(convertObs(in, {}, 1, FrequencyMask{}).sat, 0);
}
TEST(ObservationValidation, NanSurvivesCdrAndBecomesMissingOnlyAtBackendBoundary) {
  auto raw=sample();raw.P[0]=NAN;raw.D[0]=0;
  const auto in=obsToMsg(raw,0);
  gnss_ros_standardization::msg::GnssObservation out;
  rclcpp::Serialization<gnss_ros_standardization::msg::GnssObservation> serializer;
  rclcpp::SerializedMessage bytes;
  serializer.serialize_message(&in,&bytes);serializer.deserialize_message(&bytes,&out);
  EXPECT_TRUE(std::isnan(out.p));EXPECT_EQ(out.d,0);EXPECT_EQ(out.l,raw.L[0]);
  EXPECT_TRUE(sameObservation(in,out));
  const auto backend=convertObs(out,{},1,FrequencyMask{});
  EXPECT_EQ(backend.P[0],0);EXPECT_EQ(backend.L[0],raw.L[0]);EXPECT_EQ(backend.D[0],0);
  out.p=0;EXPECT_FALSE(sameObservation(in,out));
}

}

TEST(ObservationValidation, RosTimestampBoundsDoNotOverflowOrChangeHostMode) {
  using gnss_ros_standardization::decoder_common::TimeBaseLatch;
  const auto log=rclcpp::get_logger("timestamp_bounds"); rclcpp::Clock clock(RCL_ROS_TIME);
  TimeBaseLatch latch; const rclcpp::Time host(123,0,RCL_ROS_TIME);
  const auto huge=gpst2time(65534,100.);
  EXPECT_FALSE(isRosTimeRepresentable(huge));
  EXPECT_THROW(gpstToUtcRosTime(huge),std::out_of_range);
  EXPECT_FALSE(latch.stamp(true,true,huge,host,log,clock));
  ASSERT_TRUE(latch.stamp(false,true,huge,host,log,clock));
  EXPECT_EQ(latch.stamp(false,true,huge,host,log,clock)->nanoseconds(),host.nanoseconds());
  const gtime_t edge=utc2gpst(gtime_t{std::numeric_limits<int32_t>::max(),.5});
  EXPECT_TRUE(isRosTimeRepresentable(edge));
  EXPECT_EQ(gpstToUtcRosTime(edge).nanoseconds(),2147483647500000000LL);
  EXPECT_FALSE(isRosTimeRepresentable(timeadd(edge,1.)));
  gtime_t invalid=edge;invalid.sec=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(isRosTimeRepresentable(invalid));
  EXPECT_TRUE(latch.stamp(true,true,gpst2time(2300,100),host,log,clock));
}

TEST(ReleaseL2, UtcConversionRoundsFractionalNanosecondsAndCarries) {
  const gtime_t utc{1700000000,0.1234567896};
  const auto rounded=gnss_utils::gpstToUtcRosTime(utc2gpst(utc));
  EXPECT_EQ(rounded.nanoseconds(),1700000000123456790LL);
  const auto carry=gnss_utils::gpstToUtcRosTime(utc2gpst(gtime_t{1700000000,.9999999998}));
  EXPECT_EQ(carry.nanoseconds(),1700000001000000000LL);
}
