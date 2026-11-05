#!/usr/bin/env python3
"""G6 Tables4.5/4.6 descending transmitted bit numbers -> S1 GLORawCA.

Receiver parity checking is represented by CRCPassed; this test does not claim
to simulate channel coding. String5 provides the separately required UTC data.
"""
import math
import struct
import unittest
import test_receiver_streams as wire
PARAMS=['-p','topics.ephemeris:=/review/nav','-p','use_gps_timestamp:=true']

def strings(tk=20400,tb=23,slot=1,tau=-12345):
    frames={i:['0']*85 for i in range(1,6)}
    def put(sf,low,high,value,signed=False):
        width=high-low+1
        if signed:value=abs(value)|(int(value<0)<<(width-1))
        assert 0<=value<1<<width
        frames[sf][85-high:86-low]=format(value,f'0{width}b')
    for sf in frames:put(sf,81,84,sf)
    put(1,77,78,3);put(1,72,76,tk//3600);put(1,66,71,tk%3600//60);put(1,65,65,tk%60//30)
    put(2,78,80,5);put(2,77,77,1);put(2,70,76,tb)
    put(3,80,80,1);put(3,69,79,-123,True);put(3,66,67,2);put(3,65,65,1)
    put(4,59,80,tau,True);put(4,54,58,-7,True);put(4,49,53,7)
    put(4,34,34,1);put(4,30,33,6);put(4,16,26,1000);put(4,11,15,slot);put(4,9,10,2)
    for sf,pos,vel,acc in [(1,39000000,-1234567,7),(2,-25000000,2345678,-9),(3,42000000,-3456789,11)]:
        put(sf,9,35,pos,True);put(sf,41,64,vel,True);put(sf,36,40,acc,True)
    return {sf:int(''.join(bits)+'0'*11,2).to_bytes(12,'big') for sf,bits in frames.items()}

def envelopes(data,week=2300,tow=96006,channel=-7,passed=1):
    result=[]
    for sf in (5,1,2,3,4):
        if sf not in data:continue
        words=struct.unpack('>3I',data[sf])
        body=struct.pack('<IH6B3I',tow*1000,week,38,passed,0,8,channel+8,0,*words)
        result.append(wire.sbf(4026,body))
    return result

class GlonassRawSpecification(unittest.TestCase):
    def check(self,e,channel,week=2300,toe=96318.,tof=96018.,tau=-12345):
        self.assertEqual((e.prn,e.frq,e.week,e.toe,e.tof),(1,channel,week,toe,tof))
        self.assertEqual((e.svh,e.flags,e.sva,e.age),(9,382,6,7))
        for actual,raw in zip(e.pos,(39000000,-25000000,42000000)):self.assertEqual(actual,raw*2**-11*1000)
        for actual,raw in zip(e.vel,(-1234567,2345678,-3456789)):self.assertEqual(actual,raw*2**-20*1000)
        for actual,raw in zip(e.acc,(7,-9,11)):self.assertEqual(actual,raw*2**-30*1000)
        self.assertEqual((e.taun,e.dtaun),(tau*2**-30,-7*2**-30))
        self.assertTrue(math.isclose(e.gamn,-123*2**-40,rel_tol=2e-15,abs_tol=0.))

    def test_signed_fields_flags_channels_same_issue_update(self):
        for mode in ('decoder','driver'):
            frames=envelopes(strings())*2+envelopes(strings(tau=-12344),channel=6)
            result=wire.run('gloraw_values_'+mode,'sbf_'+mode+'_node',frames,PARAMS,chunk_delay=.05)
            snaps=[m for m in result['nav'] if m.glonass_ephemeris]
            self.assertEqual(len(snaps),2)
            for snap,ch,tau in zip(snaps,(-7,6),(-12345,-12344)):
                self.assertEqual(len(snap.glonass_ephemeris),1)
                self.check(snap.glonass_ephemeris[0],ch,tau=tau)
                self.assertEqual(snap.glonass_ephemeris[0].iode,23)

    def test_missing_string_parity_slot_and_recovery(self):
        for mode in ('decoder','driver'):
            for fault in ('missing','parity','slot'):
                data=strings(slot=2 if fault=='slot' else 1)
                if fault=='missing':del data[2]
                bad=envelopes(data,passed=0 if fault=='parity' else 1)
                result=wire.run('gloraw_'+fault+'_'+mode,'sbf_'+mode+'_node',bad+envelopes(strings()),PARAMS,chunk_delay=.05)
                snaps=[m for m in result['nav'] if m.glonass_ephemeris]
                self.assertEqual(len(snaps),1)
                self.check(snaps[0].glonass_ephemeris[0],-7)

    def test_utc_day_and_gps_week_rollover(self):
        for mode in ('decoder','driver'):
            result=wire.run('gloraw_rollover_'+mode,'sbf_'+mode+'_node',
                envelopes(strings(tk=10800,tb=12),tow=604790),PARAMS,chunk_delay=.05)
            snaps=[m for m in result['nav'] if m.glonass_ephemeris]
            self.assertEqual(len(snaps),1)
            self.check(snaps[0].glonass_ephemeris[0],-7,week=2301,toe=18.,tof=18.)

if __name__=='__main__':unittest.main(argv=[__file__]+wire.unittest_args)
