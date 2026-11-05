#include <gtest/gtest.h>
#include "gnss_ros_standardization/rtklib_compat.hpp"

TEST(NavigationRecordKey, KeplerianReusedIodDoesNotHideUpdates) {
  eph_t original{};original.sat=satno(SYS_GPS,1);original.iode=7;original.iodc=7;
  original.toe=original.toc=original.ttr=gpst2time(2300,100);
  const auto key=gnss_utils::navigationRecordKey(original);
  EXPECT_EQ(key,gnss_utils::navigationRecordKey(original));
  auto changed=original;changed.toe=gpst2time(2301,100);
  EXPECT_NE(key,gnss_utils::navigationRecordKey(changed));
  changed=original;changed.svh=1;EXPECT_NE(key,gnss_utils::navigationRecordKey(changed));
  changed=original;changed.f0=1e-12;EXPECT_NE(key,gnss_utils::navigationRecordKey(changed));
  changed=original;changed.ttr.sec=.5;EXPECT_NE(key,gnss_utils::navigationRecordKey(changed));
  changed=original;changed.code=1;EXPECT_NE(key,gnss_utils::navigationRecordKey(changed));
  changed=original;changed.tgd[5]=1e-9;EXPECT_NE(key,gnss_utils::navigationRecordKey(changed));
  changed=original;changed.Adot=1.;EXPECT_NE(key,gnss_utils::navigationRecordKey(changed));
}
TEST(NavigationRecordKey, GlonassReusedIodDoesNotHideUpdates) {
  geph_t original{};original.sat=satno(SYS_GLO,1);original.iode=7;
  original.toe=original.tof=gpst2time(2300,100);
  const auto key=gnss_utils::navigationRecordKey(original);
  EXPECT_EQ(key,gnss_utils::navigationRecordKey(original));
  auto changed=original;changed.toe=gpst2time(2301,100);
  EXPECT_NE(key,gnss_utils::navigationRecordKey(changed));
  changed=original;changed.tof=gpst2time(2300,200);EXPECT_NE(key,gnss_utils::navigationRecordKey(changed));
  changed=original;changed.svh=1;EXPECT_NE(key,gnss_utils::navigationRecordKey(changed));
  changed=original;changed.pos[2]=1.;EXPECT_NE(key,gnss_utils::navigationRecordKey(changed));
  changed=original;changed.taun=1e-12;EXPECT_NE(key,gnss_utils::navigationRecordKey(changed));
  changed=original;changed.frq=-7;EXPECT_NE(key,gnss_utils::navigationRecordKey(changed));
}
