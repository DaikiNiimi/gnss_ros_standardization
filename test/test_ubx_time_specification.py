#!/usr/bin/env python3
"""U2 NAV-SOL/TIMEGPS/TIM-TM2 and U1 NAV-CLOCK -> ROS time/output policy.
Exact backend fractional time and malformed-packet state are separately checked
by UbxTimeSpecification in test_raw_observations.cpp. Tick duration here is an
explicit test setting, not a claim about undocumented receiver sTtag units.
"""
import struct
import unittest
import test_receiver_streams as wire
import test_protocol_specification as spec
from test_imu_specification import raw_esf


def rawx(tow):
    return spec.frame(struct.pack('<dHbBBB2x',tow,2300,18,1,1,1)+spec.row(1))


def auxiliary(mid):
    if mid==3331:
        return struct.pack('<BBHHH5I',0,0x4c,7,2300,2300,
                           101000,250000,101001,125000,100)
    p=bytearray(52 if mid==262 else 16)
    struct.pack_into('<IihBB',p,0,101000,-500000,2300,3 if mid==262 else 18,12 if mid==262 else 3)
    if mid==262:
        struct.pack_into('<3iI3iIHBBI',p,12,637813700,0,0,10,100,-200,300,10,100,0,12,0)
    else:struct.pack_into('<I',p,12,100)
    return p


class UbxTimeSpecification(unittest.TestCase):
    def test_time_auxiliary_no_synthetic_epochs_and_valid_recovery(self):
        for mid in (262,288,3331):
            p=auxiliary(mid)
            frames=[rawx(100.),wire.ubx(mid>>8,mid&255,p),
                    wire.ubx(mid>>8,mid&255,p[:-1]),wire.ubx(mid>>8,mid&255,b''),rawx(102.)]
            for mode in ('decoder','driver'):
                with self.subTest(mid=mid,mode=mode):
                    result=wire.run('spec_ubx_time_'+str(mid)+'_'+mode,'ubx_'+mode+'_node',frames,
                        ['-p','topics.observation:=/review/obs','-p','topics.ephemeris:=/review/nav',
                         '-p','use_gps_timestamp:=true'],chunk_delay=.05)
                    self.assertEqual([(m.week,m.tow) for m in result['obs']],[(2300,100.),(2300,102.)])
                    self.assertEqual(result['sol'],[])
                    self.assertEqual(result['raw'],[])
                    self.assertFalse(any(m.gnss_ephemeris or m.glonass_ephemeris for m in result['nav']))
                    for m in result['obs']:
                        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
                        self.assertAlmostEqual(stamp,315964800+2300*604800+m.tow-18,delta=1e-6)

    def test_clock_accuracy_gate_and_signed_bias_do_not_shift_gnss_epochs(self):
        for mode in ('decoder','driver'):
            with self.subTest(mode=mode):
                # tAcc just above1ms blocks the first free-running-IMU anchor;
                # exactly1ms permits it. clkB is not transport latency.
                bad=wire.ubx(1,34,struct.pack('<IiiII',100000,-2000000000,-1,1000001,1))
                good=wire.ubx(1,34,struct.pack('<IiiII',101000,2000000000,1,1000000,1))
                values=(1280,-2560,0,180*4096,-90*4096,0)
                frames=[bad+rawx(100.)+raw_esf(10000,values),
                        good+rawx(101.)+raw_esf(11000,values)]
                params=['-p','use_gps_timestamp:=true','-p','imu.sttag_tick_s:=0.001',
                        '-p','topics.observation:=/review/obs']
                if mode=='driver':params+=['-p','messages.esf_raw:=true','-p','messages.nav_clock:=true']
                result=wire.run('spec_ubx_clock_'+mode,'ubx_'+mode+'_node',frames,params,chunk_delay=.1)
                self.assertEqual(len(result['raw']),1)
                self.assertEqual([(m.week,m.tow) for m in result['obs']],[(2300,100.),(2300,101.)])
                for m in result['obs']:
                    stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
                    self.assertAlmostEqual(stamp,315964800+2300*604800+m.tow-18,delta=1e-6)


if __name__=='__main__':unittest.main(argv=[__file__]+wire.unittest_args)
