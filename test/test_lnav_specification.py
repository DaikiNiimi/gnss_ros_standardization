#!/usr/bin/env python3
"""G1 Figure20-1 sheets1-3, Tables20-I/III -> receiver envelopes -> ROS.

Fields below use one-based *transmitted* bit numbers (including parity gaps),
not RTKLIB's packed-bit offsets. U4/S1/N1 wrappers transport parity-checked
data; channel parity/FEC decoding is the receiver's responsibility.
G4 section4.1 reuses the LNAV orbit/clock layout, with a different fit mapping.
"""
import math
import struct
import unittest
import test_receiver_streams as wire
from test_ubx_legacy_specification import frame as raw_time

PARAMS=['-p','topics.ephemeris:=/review/nav','-p','use_gps_timestamp:=true']

# name: (subframe, transmitted bit runs, raw integer, scale, semicircles)
FIELDS={
 'tgd':(1,[(197,8)],-7,2**-31,False),
 'f2':(1,[(241,8)],-3,2**-55,False),
 'f1':(1,[(249,16)],19,2**-43,False),
 'f0':(1,[(271,22)],-12345,2**-31,False),
 'crs':(2,[(69,16)],-23,2**-5,False),
 'deln':(2,[(91,16)],17,2**-43,True),
 'm0':(2,[(107,8),(121,24)],-2345678,2**-31,True),
 'cuc':(2,[(151,16)],-11,2**-29,False),
 'e':(2,[(167,8),(181,24)],85899346,2**-33,False),
 'cus':(2,[(211,16)],13,2**-29,False),
 'sqrt_a':(2,[(227,8),(241,24)],2701918208,2**-19,False),
 'cic':(3,[(61,16)],-15,2**-29,False),
 'omg0':(3,[(77,8),(91,24)],3456789,2**-31,True),
 'cis':(3,[(121,16)],21,2**-29,False),
 'i0':(3,[(137,8),(151,24)],654321098,2**-31,True),
 'crc':(3,[(181,16)],31,2**-5,False),
 'omg':(3,[(197,8),(211,24)],-4567890,2**-31,True),
 'omgd':(3,[(241,24)],-7654,2**-43,True),
 'idot':(3,[(279,14)],-123,2**-43,True),
}

def subframes(issue=17,week=2300,toe=96000,tow=96006,health=5,clock=-12345,fit=0):
    bits=[['0']*300 for _ in range(3)]
    def put(sf,runs,value):
        width=sum(n for _,n in runs); encoded=format(value&((1<<width)-1),f'0{width}b')
        cursor=0
        for start,n in runs:
            assert all((i-1)%30<24 for i in range(start,start+n))
            bits[sf-1][start-1:start-1+n]=encoded[cursor:cursor+n];cursor+=n
    for sf in (1,2,3):
        put(sf,[(1,8)],0x8b);put(sf,[(31,17)],(tow+6*(sf-1))//6);put(sf,[(50,3)],sf)
    for runs,value in [([(61,10)],week%1024), ([(71,2)],2), ([(73,4)],3),
                       ([(77,6)],health), ([(83,2),(211,8)],issue),
                       ([(91,1)],1), ([(219,16)],toe//16)]:put(1,runs,value)
    put(2,[(61,8)],issue);put(2,[(271,16)],toe//16);put(2,[(287,1)],fit)
    put(3,[(271,8)],issue)
    expected={}
    for name,(sf,runs,value,scale,angle) in FIELDS.items():
        if name=='f0':value=clock
        put(sf,runs,value);expected[name]=value*scale*(math.pi if angle else 1)
    # The receiver has already parity-checked/uninverted each data word.
    data=[]
    for sf in bits:
        packed=''.join(''.join(sf[start:start+24]) for start in range(0,300,30))
        data.append(int(packed,2).to_bytes(30,'big'))
    return data,expected

def envelopes(path,data,system='G',week=2300,tow=96006,version=2,crc_passed=1):
    if path=='oem':
        return [wire.oem(41 if system=='G' else 1331,
                struct.pack('<III',1 if system=='G' else 193,week,tow)+b''.join(data))]
    frames=[]
    for sf in data:
        words=[int.from_bytes(sf[i:i+3],'big') for i in range(0,30,3)]
        if path=='oem_sub':
            # N1 p824: 30-byte subframe is padded to32 before U4 channel.
            subframe_id=(sf[5]>>2)&7
            frames.append(wire.oem(1330,struct.pack('<II',193,subframe_id)+sf+bytes(2)+struct.pack('<I',7)))
        elif path=='sfrb':frames.append(wire.ubx(2,0x11,bytes([0,1])+struct.pack('<10I',*words)))
        elif path=='sfrbx':
            frames.append(wire.ubx(2,0x13,bytes([0 if system=='G' else 5,1,0,0,10,0,version,0])+
                                   struct.pack('<10I',*(w<<6 for w in words))))
        elif path=='sbf':
            frames.append(wire.sbf(4017 if system=='G' else 4066,
                struct.pack('<IH6B',tow*1000,week,1 if system=='G' else 181,crc_passed,0,0 if system=='G' else 6,0,0)+
                struct.pack('<10I',*(w<<6 for w in words))))
        else:raise ValueError(path)
    return frames

def execute(label,path,mode,frames):
    prefix={'sfrb':'ubx','sfrbx':'ubx','sbf':'sbf','oem':'novatel','oem_sub':'novatel'}[path]
    if prefix=='ubx':frames=[raw_time(ms=96000000)]+frames
    return wire.run(label,prefix+'_'+mode+'_node',frames,PARAMS,chunk_delay=.08)

class LnavSpecification(unittest.TestCase):
    def check_fields(self,e,expected,system='G',week=2300,toe=96000,tow=96006,health=5,issue=17,fit=0):
        self.assertEqual((e.system,e.prn,e.satid),(system,1 if system=='G' else 193,system+'01'))
        self.assertEqual((e.week,e.toe,e.toes,e.toc,e.ttr),(week,toe,toe,toe,tow))
        self.assertEqual((e.iode,e.iodc,e.sva,e.svh,e.code,e.flag),(issue&255,issue,3,health,2,1))
        self.assertEqual(e.fit, (6 if fit else 4) if system=='G' else (4 if fit else 2))
        for name,value in expected.items():
            actual=e.a if name=='sqrt_a' else e.tgd[0] if name=='tgd' else getattr(e,name)
            if name=='sqrt_a':value=value**2
            self.assertTrue(math.isclose(actual,value,rel_tol=2e-14,abs_tol=1e-20),(name,actual,value))
        self.assertEqual((e.adot,e.ndot),(0.,0.))

    def test_gps_fields_duplicate_same_iod_health_clock_update_all_paths(self):
        first,expected=subframes();last,updated=subframes(health=0,clock=-12344)
        for path in ('sfrb','sfrbx','sbf','oem'):
            for mode in ('decoder','driver'):
                with self.subTest(path=path,mode=mode):
                    frames=envelopes(path,first)*2+envelopes(path,last)
                    result=execute('lnav_fields_'+path+'_'+mode,path,mode,frames)
                    snaps=[m for m in result['nav'] if m.gnss_ephemeris]
                    self.assertEqual(len(snaps),2)
                    self.assertEqual([len(m.gnss_ephemeris) for m in snaps],[1,1])
                    self.check_fields(snaps[0].gnss_ephemeris[0],expected)
                    self.check_fields(snaps[1].gnss_ephemeris[0],updated,health=0)

    def test_qzss_shared_lnav_layout_and_fit(self):
        data,expected=subframes(fit=1)
        for path in ('sfrbx','sbf','oem','oem_sub'):
            for mode in ('decoder','driver'):
                with self.subTest(path=path,mode=mode):
                    result=execute('qlnav_'+path+'_'+mode,path,mode,envelopes(path,data,'J'))
                    snaps=[m for m in result['nav'] if m.gnss_ephemeris]
                    self.assertEqual(len(snaps),1)
                    self.check_fields(snaps[0].gnss_ephemeris[0],expected,system='J',fit=1)

    def test_oem_qzss_exact_length_short_payload_and_recovery(self):
        data,expected=subframes()
        for mode in ('decoder','driver'):
            body=struct.pack('<III',193,2300,96000)+b''.join(data)
            self.assertEqual(len(body),102)
            frames=[wire.oem(1331,body[:-1]),wire.oem(1331,body),wire.oem(1331,body)]
            result=execute('qlnav_length_'+mode,'oem',mode,frames)
            snaps=[m for m in result['nav'] if m.gnss_ephemeris]
            self.assertEqual(len(snaps),1)
            self.check_fields(snaps[0].gnss_ephemeris[0],expected,system='J')

    def test_missing_mismatched_issue_then_recovery(self):
        first,expected=subframes();other,_=subframes(issue=18)
        for path in ('sfrb','sfrbx','sbf','oem'):
            for mode in ('decoder','driver'):
                with self.subTest(path=path,mode=mode):
                    if path=='oem':
                        frames=envelopes(path,[bytes(30),first[1],first[2]])
                    else:frames=envelopes(path,[first[2],first[0],first[2]])
                    frames+=envelopes(path,[first[0],other[1],first[2]])
                    frames+=envelopes(path,first)
                    result=execute('lnav_missing_'+path+'_'+mode,path,mode,frames)
                    snaps=[m for m in result['nav'] if m.gnss_ephemeris]
                    self.assertEqual(len(snaps),1)
                    self.check_fields(snaps[0].gnss_ephemeris[0],expected)

    def test_toe_next_week_and_bad_sbf_parity(self):
        data,expected=subframes(week=2300,toe=0,tow=604776,issue=511,fit=1)
        for path in ('sfrb','sfrbx','sbf','oem'):
            for mode in ('decoder','driver'):
                with self.subTest(path=path,mode=mode):
                    frames=envelopes(path,data,week=2300,tow=604776,crc_passed=0) if path=='sbf' else []
                    frames+=envelopes(path,data,week=2300,tow=604776)
                    result=execute('lnav_week_'+path+'_'+mode,path,mode,frames)
                    snaps=[m for m in result['nav'] if m.gnss_ephemeris]
                    self.assertEqual(len(snaps),1)
                    self.check_fields(snaps[0].gnss_ephemeris[0],expected,week=2301,toe=0,tow=604776,issue=511,fit=1)

if __name__=='__main__':unittest.main(argv=[__file__]+wire.unittest_args)
