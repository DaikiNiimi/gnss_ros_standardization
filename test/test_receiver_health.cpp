#include <gtest/gtest.h>
#include <thread>
#include "gnss_ros_standardization/receiver_health.hpp"

namespace {
using Health=gnss_ros_standardization::wire::ReceiverHealth;
const auto start=Health::Clock::time_point{};
void contains(const std::string& actual,const std::string& expected) {
  EXPECT_NE(actual.find(expected),std::string::npos) << actual;
}
TEST(ReceiverHealth, NoInputAndValidFramesAreNotObservations) {
  Health h(start);h.configure(1);
  contains(h.describe(start),"state=no_input");
  h.input(100,start);h.frame(4027,1);h.backendResult(0);
  auto line=h.describe(start);
  contains(line,"state=observations_not_observed");
  contains(line,"bytes=100 integrity_frames=1");
  contains(line,"decoded_epochs=0 published_epochs=0");
  contains(line,"backend_filter_counts=not_observed");
  contains(line,"last_epoch=unavailable");
  h.rejectFrame();h.backendResult(-1);h.unsupportedNavigation();h.stationFiltered();
  line=h.describe(start);
  contains(line,"framing_rejected=1 payload_errors=1");
  contains(line,"unsupported_navigation=1 station_filtered=1");
  contains(h.describe(start+std::chrono::seconds(2)),"state=input_stale");
}
TEST(ReceiverHealth, RateEpochWeekRolloverAndSessionReset) {
  Health h(start);h.configure(1);h.expectObservations(5);h.input(100,start);
  h.backendResult(1);h.published(2300,604799.8,2,start);
  h.backendResult(1);h.published(2301,0,3,start);
  auto line=h.describe(start);
  contains(line,"state=observations_active");contains(line,"rate_matches_request=yes");
  contains(line,"decoded_epochs=2 published_epochs=2 published_signals=5");
  h.published(2301,1,1,start);
  contains(h.describe(start),"rate_matches_request=no");
  h.published(2301,.5,1,start); // regression must not retain an old good rate
  contains(h.describe(start),"rate_matches_request=not_checked");
  h.input(1,start+std::chrono::seconds(2));
  contains(h.describe(start+std::chrono::seconds(2)),"state=observations_stale");
  h.resetSession();line=h.describe(start+std::chrono::seconds(2));
  contains(line,"state=observations_not_observed");contains(line,"sessions_reset=1");
  contains(line,"last_epoch=unavailable");contains(line,"rate_matches_request=not_checked");
}
TEST(ReceiverHealth, EmptyEpochDoesNotClaimValidObservationsAndPeriodIsBounded) {
  Health h(start);h.input(5,start);h.published(2300,100,0,start);
  contains(h.describe(start),"state=observations_not_observed");
  contains(h.describe(start),"empty_epochs=1");
  for(double period:{0.,.1,61.,double(NAN),double(INFINITY)}) EXPECT_THROW(h.configure(period),std::invalid_argument);
  EXPECT_NO_THROW(h.configure(1));
  EXPECT_NO_THROW(h.configure(60));
}
}

TEST(ReceiverHealthDetails, ReasonsDoNotDependOnBackendReturnZero) {
  namespace wire=gnss_ros_standardization::wire;
  Health h(start); wire::Frame f; f.cls=2;f.id=0x15;f.payload.resize(16);
  f.payload[13]=2; // unsupported RAWX version, invalid week, zero valid TOW
  h.inspect(wire::Protocol::Ubx,f);h.backendResult(0);
  contains(h.describe(start),"unknown_versions=1");
  contains(h.describe(start),"time_unresolved_frames=1");
  f.cls=5;f.id=1;f.payload={6,1};h.inspect(wire::Protocol::Ubx,f); // valid ACK is known
  f.cls=0x99;h.inspect(wire::Protocol::Ubx,f);
  h.inspectRtcm(4095);h.inspectRtcm(1077);
  contains(h.describe(start),"unknown_ids=2");
  contains(h.describe(start),"version_unchecked_frames=2");
  f.id=4008;f.revision=0;f.payload.clear();h.inspect(wire::Protocol::Sbf,f);
  contains(h.describe(start),"known_without_decoder_frames=1");
  contains(h.describe(start),"decoded_epochs=0");
  h.assemblyPending(true);h.assemblyPending(true);h.timeWithheld();
  h.observationCapacity(95,96);h.observationCapacity(96,96);
  contains(h.describe(start),"assembly_pending=yes assembly_started=1");
  contains(h.describe(start),"capacity_at_limit_epochs=1");
  contains(h.describe(start),"capacity_unread_rows=not_observed");
  h.resetSession();contains(h.describe(start),"assembly_pending=no");
  h.published(2300,604799.125,1,start);
  contains(h.describe(start),"last_epoch=2300:604799.125");
}

TEST(ReceiverHealthDetails, FramingRejectsSeparateChecksumLengthAndTimeout) {
  namespace wire=gnss_ros_standardization::wire;
  wire::Framer f(wire::Protocol::Sbf);auto emit=[](const wire::Frame&){};
  for(auto b:std::vector<uint8_t>{'$','@',0,0,1,0,8,0}) f.feed(b,emit); // invalid CRC
  EXPECT_EQ(f.checksumRejected(),1u);
  for(auto b:std::vector<uint8_t>{'$','@',0,0,1,0,7,0}) f.feed(b,emit); // invalid length
  EXPECT_EQ(f.malformed(),1u);
  for(auto b:std::vector<uint8_t>{'$','@',0,0,1,0,20,0}) f.feed(b,emit);
  f.expire(emit);EXPECT_EQ(f.expired(),1u);
  EXPECT_EQ(f.rejected(),3u);
  f.reset();Health h(start);h.framing(f);
  contains(h.describe(start),"checksum_rejected=1 malformed_candidates=1 incomplete_expired=1");
}

TEST(ReceiverHealthDetails, ScopedCountersAreIsolatedNestedAndThreadLocal) {
  Health a(start),b(start);
  {
    auto outer=a.backendScope();
    GNSS_RX_EVENT(GNSS_RX_SATELLITE,2);
    {
      auto inner=b.backendScope();
      GNSS_RX_EVENT(GNSS_RX_SIGNAL,3);
    }
    std::thread other([&] {
      GNSS_RX_EVENT(GNSS_RX_SATELLITE,99); // no sink in this thread
      auto scope=b.backendScope();
      GNSS_RX_EVENT(GNSS_RX_SIGNAL,1);
    });other.join();
    GNSS_RX_EVENT(GNSS_RX_CAPACITY_ROWS,4);
  }
  GNSS_RX_EVENT(GNSS_RX_SATELLITE,99); // previous scope must have been restored
  contains(a.describe(start),"satellite_exclusions=2");
  contains(a.describe(start),"signal_mapping_exclusions=0");
  contains(a.describe(start),"capacity_unread_rows=4");
  contains(b.describe(start),"satellite_exclusions=0");
  contains(b.describe(start),"signal_mapping_exclusions=4");
  a.pvtPending(true);a.imuPending(true);a.resetSession();
  contains(a.describe(start),"pvt_pending=no pvt_incomplete_epochs=0 imu_pending=no");
  contains(a.describe(start),"satellite_exclusions=2"); // lifetime counts survive
}
TEST(ReceiverHealthDetails, SaturatesAndKeepsLookupFailureOutOfPayloadReasons) {
  gnss_receiver_diagnostics d{};
  {
    gnss_ros_standardization::wire::ReceiverDiagnosticScope scope(d);
    GNSS_RX_EVENT(GNSS_RX_SATELLITE,UINT64_MAX);
    GNSS_RX_EVENT(GNSS_RX_SATELLITE,2);
    EXPECT_EQ(gnss_receiver_negative("obsindex",10),-1);
    EXPECT_EQ(d.counts[GNSS_RX_VALIDATION],0u);
    EXPECT_EQ(gnss_receiver_negative("decode_rxmrawx",20),-1);
  }
  EXPECT_EQ(d.counts[GNSS_RX_SATELLITE],UINT64_MAX);
  EXPECT_EQ(d.counts[GNSS_RX_VALIDATION],1u);
  EXPECT_STREQ(d.functions[GNSS_RX_VALIDATION],"decode_rxmrawx");
  EXPECT_EQ(d.lines[GNSS_RX_VALIDATION],20u);
}
TEST(ReceiverHealthDetails, NavigationStateResetIsSeparateFromCumulativeEvents) {
  Health h(start);
  {
    auto scope=h.backendScope();
    gnss_receiver_navigation(1,GNSS_NAV_GPS,2,6);
    gnss_receiver_navigation(2,GNSS_NAV_GPS,3,0);
    gnss_receiver_navigation(1,GNSS_NAV_FNAV,2,14);
    GNSS_RX_EVENT(GNSS_RX_NAV_WAIT,2);
  }
  auto line=h.describe(start);
  contains(line,"navigation_pending=2 navigation_invalid=1");
  contains(line,"navigation_states=1:gps:wait:6,1:fnav:wait:14,2:gps:invalid:0");
  h.resetSession();line=h.describe(start);
  contains(line,"navigation_pending=not_observed navigation_invalid=not_observed");
  contains(line,"navigation_wait_events=2");
}

TEST(ReceiverHealthDetails, IncompletePvtRetirementIsRecordedOutsideAFrameScope) {
  Health h(start);h.pvtPending(true);h.pvtIncompleteDiscarded();h.pvtPending(false);
  contains(h.describe(start),"pvt_pending=no pvt_incomplete_epochs=1");
  h.resetSession();contains(h.describe(start),"pvt_incomplete_epochs=1");
}

TEST(ReceiverHealth, MsmAssemblyCountersSurviveSessionReset) {
  gnss_ros_standardization::wire::ReceiverHealth h;
  h.msmSplitDetected();h.msmTimeTieWithheld();h.msmSystemExcluded();
  h.msmIncompleteDiscarded(3);
  const auto check=[&] {
    EXPECT_NE(h.describe().find("msm_split_detected=1"),std::string::npos);
    EXPECT_NE(h.describe().find("msm_time_tie_withheld=1"),std::string::npos);
    EXPECT_NE(h.describe().find("msm_system_excluded=1"),std::string::npos);
    EXPECT_NE(h.describe().find("msm_incomplete_discarded=3"),std::string::npos);
  };
  check();h.resetSession();check();
  gnss_ros_standardization::wire::ReceiverHealth restarted;
  EXPECT_NE(restarted.describe().find("msm_split_detected=0"),std::string::npos);
}
