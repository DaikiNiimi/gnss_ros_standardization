#include <gtest/gtest.h>
#include <rclcpp/serialized_message.hpp>
#include <limits>
#include <set>
#include <rclcpp/serialization.hpp>
#include "gnss_ros_standardization/ephemeris_store.hpp"

namespace {
using namespace gnss_utils;
eph_t sample(int system = SYS_GPS) {
  eph_t e{};
  e.sat = satno(system, 1); e.iode = 7; e.iodc = 8;
  e.toe = e.toc = e.ttr = gpst2time(2300, 100);
  e.A = 26560000.; e.e = .01; e.i0 = .95; e.fit = 4;
  e.tgd[0] = 1e-9;
  return e;
}
geph_t glonass() {
  geph_t g{}; g.sat = satno(SYS_GLO, 1); g.frq = -7;
  g.toe = g.tof = gpst2time(2300, 100);
  g.pos[0] = 19000000.;
  return g;
}
TEST(EphemerisContract, ConsumerUsesGalileoFamilySlotsWhileRetiringEvictedEntries) {
  EphemerisStore store;nav_t nav{};
  auto f=sample(SYS_GAL);f.code=258;f.f0=1e-4;
  auto i=f;i.code=513;i.f0+=1e-9;
  store.ingestEph(f);store.applyToNav(nav,true);
  store.ingestEph(i);store.applyToNav(nav,true);
  ASSERT_EQ(nav.n,2);EXPECT_EQ(nav.eph[0].code,517);EXPECT_EQ(nav.eph[1].code,258);
  const auto t=timeadd(f.toe,100);
  double position[6],clock[2],variance;int health;
  ASSERT_TRUE(satpos(t,t,f.sat,EPHOPT_BRDC,&nav,position,clock,&variance,&health));
  const double expected_clock=clock[0];
  // Unrelated satellites/rehashes must not flip the equal-toe winner.
  for (int prn=1;prn<=32;++prn) {auto g=sample();g.sat=satno(SYS_GPS,prn);store.ingestEph(g);}
  store.applyToNav(nav,true);
  ASSERT_TRUE(satpos(t,t,f.sat,EPHOPT_BRDC,&nav,position,clock,&variance,&health));
  EXPECT_DOUBLE_EQ(clock[0],expected_clock);
  store.setHistoryPerSatellite(1);
  auto newer=i;newer.toe=newer.toc=newer.ttr=timeadd(i.toe,60);newer.iode++;
  store.ingestEph(newer);store.applyToNav(nav,true);
  int galileo=0;
  for(int j=0;j<nav.n;++j)if(nav.eph[j].sat==i.sat) {++galileo;EXPECT_EQ(nav.eph[j].iode,newer.iode);}
  EXPECT_EQ(galileo,1);
  store.clear();store.applyToNav(nav,true);EXPECT_EQ(nav.n,0);EXPECT_EQ(nav.ng,0);
  std::free(nav.eph);std::free(nav.geph);
}

TEST(EphemerisContract, BatchedNewNavigationIsIndependentOfUnrelatedSatellites) {
  for (int unrelated : {0, 1, 4, 12, 32, 63}) {
    EphemerisStore store; nav_t nav{};
    auto f=sample(SYS_GAL); f.code=258; f.f0=1e-4;
    auto i=f; i.code=513; i.f0+=1e-9;
    store.ingestEph(f);
    for (int prn=1;prn<=unrelated;++prn) {
      auto other=sample(SYS_CMP);other.sat=satno(SYS_CMP,prn);
      store.ingestEph(other);
    }
    store.ingestEph(i);
    store.applyToNav(nav,true);
    std::vector<int> galileo_codes;
    for(int j=0;j<nav.n;++j)if(nav.eph[j].sat==f.sat)galileo_codes.push_back(nav.eph[j].code);
    EXPECT_EQ(galileo_codes,(std::vector<int>{517,258})) << unrelated;
    // A fresh destination must use the same first-seen source order too.
    nav_t copy{};store.applyToNav(copy,true);
    ASSERT_EQ(copy.n,nav.n);
    for(int j=0;j<nav.n;++j) {
      EXPECT_EQ(copy.eph[j].sat,nav.eph[j].sat);
      EXPECT_EQ(copy.eph[j].code,nav.eph[j].code);
    }
    std::free(nav.eph);std::free(copy.eph);
  }
}

TEST(EphemerisContract, GlonassSolverOrderSurvivesBatchingAndRetirement) {
  EphemerisStore store;nav_t nav{};
  auto first=glonass();auto second=first;second.tof=timeadd(first.tof,30);
  store.ingestGeph(first);
  for(int prn=2;prn<=24;++prn) {auto g=first;g.sat=satno(SYS_GLO,prn);store.ingestGeph(g);}
  store.ingestGeph(second);store.applyToNav(nav,true);
  std::vector<double> transmitted;
  for(int j=0;j<nav.ng;++j)if(nav.geph[j].sat==first.sat)
    transmitted.push_back(timediff(nav.geph[j].tof,first.tof));
  EXPECT_EQ(transmitted,(std::vector<double>{0,30}));
  store.setHistoryPerSatellite(1);
  auto newer=second;newer.toe=newer.tof=timeadd(second.toe,60);++newer.iode;
  store.ingestGeph(newer);store.applyToNav(nav,true);
  int count=0;
  for(int j=0;j<nav.ng;++j)if(nav.geph[j].sat==first.sat){++count;EXPECT_EQ(nav.geph[j].iode,newer.iode);}
  EXPECT_EQ(count,1);
  store.clear();store.applyToNav(nav,true);EXPECT_EQ(nav.ng,0);
  std::free(nav.geph);
}

TEST(EphemerisContract, CanonicalSolverRefreshKeepsRawGalileoRecords) {
  EphemerisStore store;nav_t nav{};
  auto e=sample(SYS_GAL);e.code=513;e.f0=1e-4;
  store.ingestEph(e);e.code=516;e.f0=2e-4;e.ttr=timeadd(e.ttr,1);store.ingestEph(e);
  store.applyToNav(nav,true);ASSERT_EQ(nav.n,1);EXPECT_DOUBLE_EQ(nav.eph[0].f0,2e-4);
  // Replaying an older alias cannot replace a newer canonical refinement.
  e.code=513;e.f0=1e-4;e.ttr=timeadd(e.ttr,-1);EXPECT_FALSE(store.ingestEph(e));
  store.applyToNav(nav,true);ASSERT_EQ(nav.n,1);EXPECT_DOUBLE_EQ(nav.eph[0].f0,2e-4);
  EXPECT_EQ(store.buildSnapshot(rclcpp::Time(0)).gnss_ephemeris.size(),2u);
  std::free(nav.eph);
}

TEST(EphemerisContract, GalileoTwoGenerationsAreIndependentOfArrivalAndKeepRawHistory) {
  std::vector<eph_t> records;
  for (int generation=0;generation<4;++generation) for (int code : {513,258}) {
    auto e=sample(SYS_GAL);e.code=code;e.iode+=generation;e.iodc+=generation;
    e.toe=e.toc=e.ttr=timeadd(e.toe,60*generation);
    e.tgd[1]=code==513 ? 7e-10 : 0.;e.f0=code==513 ? 1e-4 : 2e-4;
    records.push_back(e);
  }
  std::vector<std::string> expected;
  for (int order=0;order<3;++order) {
    EphemerisStore store;store.setHistoryPerSatellite(0);nav_t nav{};
    if (order==1) std::reverse(records.begin(),records.end());
    if (order==2) std::rotate(records.begin(),records.begin()+3,records.end());
    for (const auto& e:records) {store.ingestEph(e);store.applyToNav(nav,true);}
    ASSERT_EQ(nav.n,4);
    EXPECT_EQ(nav.eph[0].code,517);EXPECT_EQ(nav.eph[1].code,258);
    EXPECT_EQ(nav.eph[2].code,517);EXPECT_EQ(nav.eph[3].code,258);
    EXPECT_EQ(nav.eph[0].iode,10);EXPECT_EQ(nav.eph[1].iode,10);
    EXPECT_EQ(nav.eph[2].iode,9);EXPECT_EQ(nav.eph[3].iode,9);
    std::vector<std::string> keys;
    for(int j=0;j<nav.n;++j)keys.push_back(navigationRecordKey(nav.eph[j]));
    if (!order) expected=keys;else EXPECT_EQ(keys,expected);
    EXPECT_EQ(store.buildSnapshot(rclcpp::Time(0)).gnss_ephemeris.size(),8u);
    store.applyToNav(nav,false);EXPECT_EQ(nav.n,8);
    std::free(nav.eph);
  }
}

TEST(EphemerisContract, OlderFractionalToeCannotReplaceSolverProjection) {
  EphemerisStore store;nav_t nav{};
  auto e=sample();e.toe.sec=.75;
  store.ingestEph(e);store.applyToNav(nav,true);
  e.toe.sec=.25;e.f0=1e-3;
  EXPECT_FALSE(store.ingestEph(e));store.applyToNav(nav,true);
  ASSERT_EQ(nav.n,1);EXPECT_DOUBLE_EQ(nav.eph[0].toe.sec,.75);EXPECT_DOUBLE_EQ(nav.eph[0].f0,0.);
  std::free(nav.eph);
}

TEST(EphemerisContract, IntegerFieldsRejectNarrowingBeforeConversion) {
  for (auto member : {&eph_t::iode, &eph_t::flag,
                      &eph_t::iodc, &eph_t::svh, &eph_t::code}) {
    const int maximum = (member == &eph_t::sva ||
                         member == &eph_t::flag) ? 255 : 65535;
    for (int good : {0, maximum}) {
      auto e = sample(); e.*member = good;
      EXPECT_EQ(ephemerisMessageError(e), nullptr);
      const auto roundtrip = msgToEph(ephToMsg(e));
      EXPECT_EQ(roundtrip.*member, good);
    }
    for (int bad : {-1, maximum + 1}) {
      auto e = sample(); e.*member = bad;
      ASSERT_NE(ephemerisMessageError(e), nullptr);
      EXPECT_THROW(ephToMsg(e), std::invalid_argument);
    }
  }
}
TEST(EphemerisContract, FractionalOrUnrepresentableFitIsNotTruncated) {
  for (double bad : {-1., 4.5, 256., std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    auto e = sample(); e.fit = bad;
    EXPECT_THROW(ephToMsg(e), std::invalid_argument);
  }
  for (double good : {0., 4., 255.}) {
    auto e = sample(); e.fit = good;
    EXPECT_DOUBLE_EQ(msgToEph(ephToMsg(e)).fit, good);
  }
}
TEST(EphemerisContract, BeidouIodeIsNeverGuessedFromIodc) {
  auto e = sample(SYS_CMP); e.iode = 240; e.iodc = 3;
  const auto m = ephToMsg(e);
  EXPECT_EQ(m.iode, 240); EXPECT_EQ(m.iodc, 3);
  auto restored = msgToEph(m);
  EXPECT_EQ(restored.iode, 240); EXPECT_EQ(restored.iodc, 3);
  auto id_only = m; id_only.system.clear();
  EXPECT_EQ(msgToEph(id_only).iode, 240);
}
TEST(EphemerisContract, AllSixDelaysSurviveExistingSchemaCdrAndBackend) {
  auto e = sample(SYS_CMP);
  for (int i = 0; i < 6; ++i) e.tgd[i] = (i + 1) * 1e-9;
  const auto m = ephToMsg(e);
  ASSERT_EQ(m.tgd.size(), 6u);
  rclcpp::Serialization<gnss_ros_standardization::msg::GnssEphemeris> serializer;
  rclcpp::SerializedMessage bytes;
  gnss_ros_standardization::msg::GnssEphemeris restored;
  serializer.serialize_message(&m, &bytes); serializer.deserialize_message(&bytes, &restored);
  const auto backend = msgToEph(restored);
  for (int i = 0; i < 6; ++i) EXPECT_DOUBLE_EQ(backend.tgd[i], e.tgd[i]);
  EXPECT_EQ(ephToMsg(sample()).tgd.size(), 2u);
  restored.tgd.resize(7, 0.);
  EXPECT_EQ(msgToEph(restored).sat, 0);
}
TEST(EphemerisContract, LegacyShortDelayArraysHaveZeroFilledTail) {
  auto m = ephToMsg(sample());
  for (size_t size = 0; size <= 6; ++size) {
    m.tgd.assign(size, 1e-9);
    const auto e = msgToEph(m);
    ASSERT_NE(e.sat, 0);
    for (size_t i = 0; i < 6; ++i) EXPECT_DOUBLE_EQ(e.tgd[i], i < size ? 1e-9 : 0.);
  }
}
TEST(EphemerisContract, OrbitRatesPreservedAndNonFiniteRejected) {
  for (auto member : {&eph_t::Adot, &eph_t::ndot}) {
    for (double value : {-1e-9, 0., 1e-9}) {
      auto e = sample(); e.*member = value;
      EXPECT_DOUBLE_EQ(msgToEph(ephToMsg(e)).*member, value);
    }
    for (double bad : {NAN, INFINITY, -INFINITY}) {
      auto e = sample(); e.*member = bad;
      EXPECT_THROW(ephToMsg(e), std::invalid_argument);
    }
  }
}
TEST(EphemerisContract, GlonassIntegerFieldsRejectNarrowing) {
  for (auto member : {&geph_t::iode, &geph_t::svh, &geph_t::age, &geph_t::frq}) {
    const int minimum = member == &geph_t::frq ? -128 : 0;
    const int maximum = member == &geph_t::frq ? 127 : 255;
    for (int good : {minimum, maximum}) {
      auto g = glonass(); g.*member = good;
      EXPECT_EQ(msgToGeph(gephToMsg(g)).*member, good);
    }
    for (int bad : {minimum - 1, maximum + 1}) {
      auto g = glonass(); g.*member = bad;
      EXPECT_THROW(gephToMsg(g), std::invalid_argument);
    }
  }
  auto wrong = glonass(); wrong.sat = satno(SYS_GPS, 1);
  EXPECT_THROW(gephToMsg(wrong), std::invalid_argument);
}
TEST(EphemerisContract, GlonassChannelSurvivesTopicCdrAndConsumerFrequencyLookup) {
  EphemerisStore publisher, consumer;
  rclcpp::Serialization<gnss_ros_standardization::msg::GnssEphemerides> serializer;
  const int sat = satno(SYS_GLO, 1);
  nav_t nav{};
  // Without navigation or a channel cache, FDMA frequency must remain unknown.
  EXPECT_DOUBLE_EQ(sat2freq(sat, CODE_L1C, &nav), 0.);
  EXPECT_DOUBLE_EQ(sat2freq(sat, CODE_L2C, &nav), 0.);
  for (int channel : {-7, 0, 6}) {
    SCOPED_TRACE(channel);
    auto g = glonass(); g.frq = channel;
    ASSERT_TRUE(publisher.ingestGeph(g));
    const auto sent = publisher.buildSnapshot(rclcpp::Time(0));
    gnss_ros_standardization::msg::GnssEphemerides received;
    rclcpp::SerializedMessage bytes;
    serializer.serialize_message(&sent, &bytes);
    serializer.deserialize_message(&bytes, &received);
    ASSERT_EQ(received.glonass_ephemeris.size(), 1u);
    const auto& message = received.glonass_ephemeris[0];
    EXPECT_EQ(message.frq, channel);
    ASSERT_TRUE(consumer.ingestGeph(msgToGeph(message)));
    consumer.applyToNav(nav, true);
    EXPECT_EQ(nav.ng, 1);
    // Independent expected FDMA frequencies (Hz), including valid channel zero.
    EXPECT_DOUBLE_EQ(sat2freq(sat, CODE_L1C, &nav), 1602e6 + channel * .5625e6);
    EXPECT_DOUBLE_EQ(sat2freq(sat, CODE_L2C, &nav), 1246e6 + channel * .4375e6);
    std::free(nav.geph); nav.geph = nullptr; nav.ng = nav.ngmax = 0;
  }
}
TEST(EphemerisContract, SnapshotOmitsOverflowWithoutCorruptingOtherRecords) {
  EphemerisStore store;
  auto good = sample(); auto gal = sample(SYS_GAL); gal.iode = 65536;
  ASSERT_TRUE(store.ingestEph(good)); ASSERT_TRUE(store.ingestEph(gal));
  ASSERT_TRUE(store.ingestGeph(glonass()));
  auto snapshot = store.buildSnapshot(rclcpp::Time(0));
  ASSERT_EQ(snapshot.gnss_ephemeris.size(), 1u);
  EXPECT_EQ(snapshot.gnss_ephemeris[0].system, "G");
  EXPECT_EQ(snapshot.glonass_ephemeris.size(), 1u);
  EXPECT_EQ(store.lastSnapshotRejected(), 1u);
  EXPECT_FALSE(store.ingestEph(gal));
  // The store's backend view keeps the original IOD; only ROS representation is limited.
  nav_t nav{}; store.applyToNav(nav);
  bool found = false;
  for (int i = 0; i < nav.n; ++i) if (nav.eph[i].sat == gal.sat) {
    found = true; EXPECT_EQ(nav.eph[i].iode, 65536);
  }
  EXPECT_TRUE(found); std::free(nav.eph); std::free(nav.geph);
  gal.iode = 255; ASSERT_TRUE(store.ingestEph(gal));
  snapshot = store.buildSnapshot(rclcpp::Time(1));
  EXPECT_EQ(snapshot.gnss_ephemeris.size(), 2u);
  EXPECT_EQ(store.lastSnapshotRejected(), 1u); // retained older overflow is still omitted
}
TEST(EphemerisContract, GalileoTenBitIodSurvivesCdrSnapshotAndBackend) {
  EphemerisStore store;
  rclcpp::Serialization<gnss_ros_standardization::msg::GnssEphemeris> serializer;
  for (int iod : {255, 256, 1023}) {
    auto e = sample(SYS_GAL); e.iode = iod; e.code = 517;
    const auto m = ephToMsg(e);
    EXPECT_EQ(m.iode, iod);
    rclcpp::SerializedMessage bytes;
    gnss_ros_standardization::msg::GnssEphemeris restored;
    serializer.serialize_message(&m, &bytes); serializer.deserialize_message(&bytes, &restored);
    EXPECT_EQ(restored.iode, iod);
    EXPECT_EQ(msgToEph(restored).iode, iod);
    ASSERT_TRUE(store.ingestEph(e));
  }
  const auto snapshot = store.buildSnapshot(rclcpp::Time(0));
  ASSERT_EQ(snapshot.gnss_ephemeris.size(), 3u);
  EXPECT_EQ(store.lastSnapshotRejected(), 0u);
  std::set<int> iods;
  for (const auto& e : snapshot.gnss_ephemeris) iods.insert(e.iode);
  EXPECT_EQ(iods, (std::set<int>{255, 256, 1023}));
}
TEST(EphemerisContract, ReferenceTimeMustSurviveTheNearestWeekContract) {
  auto e = sample(); e.toe = gpst2time(2300, 604790); e.toc = e.ttr = gpst2time(2301, 5);
  const auto restored = msgToEph(ephToMsg(e));
  EXPECT_DOUBLE_EQ(timediff(restored.toc, e.toc), 0);
  e.ttr = gpst2time(2302, 5);
  EXPECT_THROW(ephToMsg(e), std::invalid_argument);
  e = sample(); e.toe = gpst2time(-1, 0);
  EXPECT_THROW(ephToMsg(e), std::invalid_argument);
  e = sample(); e.toe.sec = NAN;
  EXPECT_THROW(ephToMsg(e), std::invalid_argument);
  auto g = glonass(); g.tof = gpst2time(2302, 100);
  EXPECT_THROW(gephToMsg(g), std::invalid_argument);
}
TEST(EphemerisContract, ExtendedDelayAndFieldUpdatesAreNotHidden) {
  EphemerisStore store; auto e = sample(SYS_CMP);
  ASSERT_TRUE(store.ingestEph(e)); e.tgd[5] = 2e-9;
  EXPECT_TRUE(store.ingestEph(e)); EXPECT_FALSE(store.ingestEph(e));
  const auto snapshot = store.buildSnapshot(rclcpp::Time(0));
  ASSERT_EQ(snapshot.gnss_ephemeris.size(), 1u);
  ASSERT_EQ(snapshot.gnss_ephemeris[0].tgd.size(), 6u);
  EXPECT_DOUBLE_EQ(snapshot.gnss_ephemeris[0].tgd[5], 2e-9);
  auto g = glonass(); ASSERT_TRUE(store.ingestGeph(g)); g.sva = 3;
  EXPECT_TRUE(store.ingestGeph(g)); EXPECT_FALSE(store.ingestGeph(g));
}

TEST(EphemerisContract, AllBackendFieldsSurviveCdrWithCanonicalWeek) {
  // Nonzero values in every scalar/array catch missing assignments, including
  // flags above bit 7 and signed backend metadata. week is normalized by design.
  for (int sys : {SYS_GPS, SYS_GAL, SYS_QZS, SYS_CMP, SYS_IRN}) {
    auto e = sample(sys);
    if (sys == SYS_QZS) e.sat = satno(sys, 193);
    ASSERT_NE(e.sat, 0);
    e.week = sys == SYS_CMP ? 944 : 2300;
    e.sva=2; e.svh=3; e.code=4; e.flag=5; e.toes=100.;
    e.toc=timeadd(e.toe,1.); e.ttr=timeadd(e.toe,2.);
    e.OMG0=.2; e.omg=.3; e.M0=.4; e.deln=1e-9; e.OMGd=-1e-8; e.idot=2e-9;
    e.crc=1.; e.crs=2.; e.cuc=3e-6; e.cus=4e-6; e.cic=5e-6; e.cis=6e-6;
    e.f0=1e-5; e.f1=-1e-12; e.f2=2e-20; e.Adot=-.125; e.ndot=1e-12;
    for (int j=0;j<6;++j) e.tgd[j]=(j+1)*1e-9;
    auto message=ephToMsg(e);
    rclcpp::Serialization<gnss_ros_standardization::msg::GnssEphemeris> serializer;
    rclcpp::SerializedMessage bytes;
    decltype(message) restored;
    serializer.serialize_message(&message,&bytes); serializer.deserialize_message(&bytes,&restored);
    EXPECT_EQ(navigationRecordKey(e),navigationRecordKey(msgToEph(restored))) << sys;
  }
  auto g=glonass(); g.iode=12; g.svh=15; g.age=3; g.flags=0x1ff; g.sva=-1;
  g.tof=timeadd(g.toe,30.); g.taun=1e-5; g.gamn=-1e-12; g.dtaun=2e-9;
  for (int j=0;j<3;++j) { g.pos[j]=19000000.+j; g.vel[j]=-1000.+j; g.acc[j]=1e-6*(j+1); }
  auto message=gephToMsg(g);
  rclcpp::Serialization<gnss_ros_standardization::msg::GlonassEphemeris> serializer;
  rclcpp::SerializedMessage bytes;
  decltype(message) restored;
  serializer.serialize_message(&message,&bytes); serializer.deserialize_message(&bytes,&restored);
  EXPECT_EQ(navigationRecordKey(g),navigationRecordKey(msgToGeph(restored)));
  for (auto member : {&geph_t::flags,&geph_t::sva}) {
    for (int value : {std::numeric_limits<int32_t>::min(),0,65535,std::numeric_limits<int32_t>::max()}) {
      g.*member=value;
      EXPECT_EQ(msgToGeph(gephToMsg(g)).*member,value);
    }
  }
}
TEST(EphemerisContract, NewFieldUpdatesReachSnapshotsAndPositioningStaysExplicit) {
  EphemerisStore store;
  auto e=sample(); ASSERT_TRUE(store.ingestEph(e));
  e.Adot=.125; ASSERT_TRUE(store.ingestEph(e)); EXPECT_FALSE(store.ingestEph(e));
  e.ndot=-1e-12; ASSERT_TRUE(store.ingestEph(e));
  auto g=glonass(); ASSERT_TRUE(store.ingestGeph(g));
  g.flags=0x1ff; ASSERT_TRUE(store.ingestGeph(g));
  g.sva=65535; ASSERT_TRUE(store.ingestGeph(g)); EXPECT_FALSE(store.ingestGeph(g));
  auto snapshot=store.buildSnapshot(rclcpp::Time(0));
  ASSERT_EQ(snapshot.gnss_ephemeris.size(),1u);
  ASSERT_EQ(snapshot.glonass_ephemeris.size(),1u);
  EXPECT_EQ(store.lastSnapshotRejected(),0u);
  EXPECT_DOUBLE_EQ(snapshot.gnss_ephemeris[0].adot,.125);
  EXPECT_DOUBLE_EQ(snapshot.gnss_ephemeris[0].ndot,-1e-12);
  EXPECT_EQ(snapshot.glonass_ephemeris[0].flags,0x1ff);
  EXPECT_EQ(snapshot.glonass_ephemeris[0].sva,65535);
  nav_t nav{}; store.applyToNav(nav);
  ASSERT_EQ(nav.n,1); EXPECT_DOUBLE_EQ(nav.eph[0].Adot,.125);
  store.applyToNav(nav,true); EXPECT_EQ(nav.n,0); EXPECT_EQ(nav.ng,1);
  auto legacy=sample(SYS_GAL); ASSERT_TRUE(store.ingestEph(legacy));
  store.applyToNav(nav,true); ASSERT_EQ(nav.n,1); EXPECT_EQ(nav.eph[0].sat,legacy.sat);
  std::free(nav.eph); std::free(nav.geph);
  // Default values remain compatible in newly generated messages only.
  EXPECT_TRUE(supportedByLegacyOrbitModel(sample()));
  e.ndot=NAN; EXPECT_FALSE(supportedByLegacyOrbitModel(e));
}
}  // namespace

TEST(EphemerisContract, SourceCodeBitsSurviveSnapshotAndCdr) {
  gnss_utils::EphemerisStore store;
  auto e = sample(SYS_GAL); e.code = 1;
  ASSERT_TRUE(store.ingestEph(e));
  const auto snapshot = store.buildSnapshot(rclcpp::Time(0));
  ASSERT_EQ(snapshot.gnss_ephemeris.size(), 1u);
  EXPECT_EQ(snapshot.gnss_ephemeris[0].code, 1);
  rclcpp::Serialization<gnss_ros_standardization::msg::GnssEphemerides> serializer;
  rclcpp::SerializedMessage bytes;
  gnss_ros_standardization::msg::GnssEphemerides restored;
  serializer.serialize_message(&snapshot, &bytes); serializer.deserialize_message(&bytes, &restored);
  EXPECT_EQ(snapshot, restored);
}

TEST(EphemerisContract, SignedAccuracyAndFiniteOrbitValidation) {
  for (int value : {-16, -1, 0, 255, INT32_MAX, INT32_MIN}) {
    auto e = sample(); e.sva = value;
    EXPECT_EQ(msgToEph(ephToMsg(e)).sva, value);
  }
  for (auto field : {&eph_t::A,&eph_t::e,&eph_t::OMGd,&eph_t::cis,&eph_t::f0}) {
    auto e = sample(); e.*field = NAN;
    EXPECT_THROW(ephToMsg(e), std::invalid_argument);
  }
  for (double value : {-1., 1., 2.}) { auto e=sample(); e.e=value; EXPECT_NE(ephemerisMessageError(e), nullptr); }
  auto e=sample(); e.A=0; EXPECT_NE(ephemerisMessageError(e), nullptr);
  auto g=glonass();g.vel[2]=NAN;EXPECT_THROW(gephToMsg(g),std::invalid_argument);
  auto m=ephToMsg(sample());m.toc=NAN;EXPECT_EQ(msgToEph(m).sat,0);
  m=ephToMsg(sample());m.ttr=INFINITY;EXPECT_EQ(msgToEph(m).sat,0);
  auto gm=gephToMsg(glonass());gm.tof=NAN;EXPECT_EQ(msgToGeph(gm).sat,0);
}

TEST(EphemerisContract, BeidouWeekBoundaryUsesBdtBeforeComputingWeek) {
  auto e=sample(SYS_CMP);e.toe=e.toc=e.ttr=gpst2time(2300,5);
  e.week=943;
  EXPECT_EQ(msgToEph(ephToMsg(e)).week,943);
}

TEST(RinexNavigationContract, RejectsSilentFieldLossAndPreservesQzssCodeZero) {
  auto e=sample();e.toes=100;
  auto check=[](const eph_t& value) { return gnss_utils::rinexNavigationError(ephToMsg(value),value,305); };
  EXPECT_EQ(check(e),nullptr);
  e.tgd[1]=1e-9;EXPECT_NE(check(e),nullptr);e.tgd[1]=0;
  e.toc.sec=.1;EXPECT_NE(check(e),nullptr);e.toc.sec=0;
  e.toes=101;EXPECT_NE(check(e),nullptr);e.toes=100;
  e.sva=-1;EXPECT_NE(check(e),nullptr);
  e=sample(SYS_GAL);e.toes=100;e.iodc=0;e.fit=0;
  EXPECT_EQ(check(e),nullptr);
  e.sva=255;EXPECT_EQ(check(e),nullptr);
  e.sva=126;EXPECT_NE(check(e),nullptr);e.sva=0;
  e.iodc=1;EXPECT_NE(check(e),nullptr);e.iodc=0;
  e.flag=1;EXPECT_NE(check(e),nullptr);
  e=sample();e.sat=satno(SYS_QZS,193);e.toes=100;e.code=0;
  EXPECT_EQ(check(e),nullptr);
  e.fit=6;EXPECT_NE(check(e),nullptr);
}
TEST(RinexNavigationContract, RejectsGlonassTimeAndStatusLoss) {
  auto g=glonass();g.toe=gpst2time(2300,18);g.iode=12;
  auto check=[](const geph_t& value) { return gnss_utils::rinexNavigationError(gephToMsg(value),value,305); };
  EXPECT_EQ(check(g),nullptr);
  g.iode=13;EXPECT_NE(check(g),nullptr);g.iode=12;
  g.toe=timeadd(g.toe,1);EXPECT_NE(check(g),nullptr);g.toe=timeadd(g.toe,-1);
  g.tof=timeadd(g.toe,50000);EXPECT_NE(check(g),nullptr);g.tof=g.toe;
  g.svh=16;EXPECT_NE(check(g),nullptr);g.svh=0;
}

TEST(EphemerisContract, IonosphereIsRetainedPublishedAndAppliedOnlyWhenReceived) {
  gnss_utils::EphemerisStore producer;
  nav_t nav{};
  EXPECT_FALSE(producer.ingestIon(nav));  // all zero = not received
  const double gps[8] = {1.1e-8, 2.2e-8, -1.2e-7, -1.2e-7, 1.2e5, 1.6e5, -1.3e5, -3.3e5};
  const double gal[4] = {200.25, 0.125, 0.0178, 0.0};
  std::copy(gps, gps + 8, nav.ion_gps);
  std::copy(gal, gal + 4, nav.ion_gal);
  EXPECT_TRUE(producer.ingestIon(nav));
  EXPECT_FALSE(producer.ingestIon(nav));  // unchanged: no new snapshot
  nav_t empty{};
  EXPECT_FALSE(producer.ingestIon(empty));  // zeros never erase a received set
  nav_t bad = nav;
  bad.ion_gps[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(producer.ingestIon(bad) == false);  // non-finite set ignored

  const auto msg = producer.buildSnapshot(rclcpp::Time(0, 0, RCL_SYSTEM_TIME));
  for (int i = 0; i < 8; ++i) EXPECT_EQ(msg.ion_gps[i], gps[i]);
  for (int i = 0; i < 4; ++i) EXPECT_EQ(msg.ion_gal[i], gal[i]);
  for (double v : msg.ion_qzs) EXPECT_EQ(v, 0.0);

  // The topic survives CDR and a consumer applies it to its nav_t.
  rclcpp::Serialization<gnss_ros_standardization::msg::GnssEphemerides> ser;
  rclcpp::SerializedMessage wire;
  ser.serialize_message(&msg, &wire);
  gnss_ros_standardization::msg::GnssEphemerides back;
  ser.deserialize_message(&wire, &back);
  gnss_utils::EphemerisStore consumer;
  EXPECT_TRUE(consumer.ingestIon(back));
  nav_t used{};
  used.ion_qzs[0] = 5.0;  // a set the snapshot does not carry stays untouched
  consumer.applyToNav(used, true);
  for (int i = 0; i < 8; ++i) EXPECT_EQ(used.ion_gps[i], gps[i]);
  for (int i = 0; i < 4; ++i) EXPECT_EQ(used.ion_gal[i], gal[i]);
  EXPECT_EQ(used.ion_qzs[0], 5.0);
  consumer.clear();
  nav_t cleared{};
  consumer.applyToNav(cleared);
  EXPECT_EQ(cleared.ion_gps[0], 0.0);
}
