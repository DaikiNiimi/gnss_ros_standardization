#!/usr/bin/env python3
"""Selected S1/N1/N3/N5 auxiliary layouts: core Topic isolation and recovery.
No claim that unrepresented almanac/SSR/SBAS/ION/UTC fields reach ROS.
GPSCNav2(4258) uses the local manufacturer sbfdef.h layout, not S1.
Native SbfAuxiliarySpecification separately checks bounds and correction indexes.
"""
import struct
import unittest
import test_receiver_streams as wire
import test_protocol_specification as spec

SBF_PAGES={4003:307,4018:277,4019:278,4020:283,4021:284,4030:308,4031:309,
    4036:303,4042:298,4067:290,4068:291,4119:320,4120:321,4121:322,
    4251:314,4252:316,4253:318,5892:295,5893:296,5894:297,5896:333,
    5902:423,5924:381,5925:328,5926:329,5927:330,5928:331,5929:332,
    5931:337,5932:338,5933:339}


def put(p,at,fmt,*values):struct.pack_into('<'+fmt,p,at,*values)
def sbf_payload(mid,revision=0):
    # Size excludes the eight-byte SBF header; padding belongs to wire helper.
    sizes={4003:53,4030:21,4031:29,4036:32,4042:164,4119:52,4120:40,
           4121:24,4251:152,4252:152,4253:144,4258:168,5892:52,5893:40,
           5894:29,5896:96,5902:(260,280,320,360,416)[revision],
           5924:24 if revision else 20,5925:7,5926:12,5927:20,5928:63,
           5929:60,5931:16,5932:52,5933:132}
    if mid in (4018,4019,4067,4068,4020,4021):
        svid=120 if mid in (4020,4021) else 181 if mid in (4067,4068) else 1
        # Documented envelope. CNAV payload type10 preamble; no orbit assembly
        # is claimed (compiled CNAV path has no navigation decoder).
        words=[0x8b000000 | (1<<18) | (10<<12)]+[0]*9
        if mid in (4020,4021):words=[0x53fc0000]+[0]*7
        source={4018:3,4019:4,4067:7,4068:26,4020:24,4021:25}[mid]
        return struct.pack('<IH6B',100000,2300,svid,1,0,source,0,0)+struct.pack('<'+'I'*len(words),*words)
    p=bytearray(sizes[mid]);put(p,0,'IH',100000,2300)
    if len(p)>6:p[6]=120 if mid>=5925 or mid==5896 else 71 if mid in (4003,4030,4031) else 141 if mid in (4119,4120,4121) else 1
    if mid in (5893,4120):put(p,8,'8f',2**-28,-2**-27,2**-26,-2**-25,65536.,131072.,-65536.,262144.)
    elif mid==4030:put(p,8,'3fB',100.,-.125,.25,0x15)
    elif mid in (5894,4031):put(p,8,'fdIBbBBb',-2**-40,2**-30,4096,2300%256,18,2301%256,7,19)
    elif mid==4121:put(p,8,'fdbBBb',-2**-40,2**-30,4,2301%256,6,5)
    elif mid==4036:p[6]=38;p[7]=7;put(p,8,'BBHfdff',8,1,100,2**-20,-2**-25,.125,-.25)
    elif mid in (5892,4003):
        put(p,8,'fI8f',.015625,4096,.125,-2**-25,5153.5,.25,-.375,.5,2**-35,-2**-20)
        p[48]=2300%256 if mid==5892 else 0
        p[49]=71 if mid==4003 else 1
    elif mid==4119:put(p,8,'I9fH',4096,5153.5,.015625,.125,-.25,.375,-2**-25,.25,-2**-20,2**-35,0)
    elif mid in (4042,4258):put(p,8,'HBBII2d',2300,0,2,100000,100000,26560000.,.125)
    elif mid in (4251,4252,4253):p[6]=1;p[7]=3;put(p,8,'I2d',100000,27906100.,.125)
    elif mid==5902:
        p[8:14]=b'MARKER';put(p,248,'3f',1.25,-2.5,3.75)
        if revision>=1:p[260:266]=b'GROUND'
        if revision>=4:put(p,360,'ddf',0.,0.,100.)
    elif mid==5924:p[6]=1;put(p,8,'fd',.125,.0625)
    elif mid==5926:p[7]=0;p[8]=2;p[9:11]=bytes((1,2))
    elif mid==5927:put(p,7,'5B',2,0,0,1,8);put(p,12,'BB2xf',1,5,-1.25)
    elif mid==5928:p[12:63]=bytes([5])*51
    elif mid==5929:p[8]=1;p[9:60]=bytes([3])*51
    elif mid==5931:p[7]=1;p[10]=2;p[11:13]=bytes((1,2))
    elif mid==5932:p[7]=1;p[8]=40;put(p,12,'4B8fI',1,1,0,17,1.,-2.,3.,.125,-.25,.375,2**-30,-2**-40,10000)
    elif mid==5933:
        p[9]=15;p[10]=8
        for i in range(15):put(p,12+8*i,'BB2xf',i+1,3,i+.25)
    if mid==5896:put(p,8,'HHI9d2f',17,2,10000,42164000.,0.,0.,0.,1.,-1.,0.,0.,0.,2**-25,-2**-40)
    return bytes(p)


def oem_payload(mid):
    if mid in (8,1347):return struct.pack('<8dII2dIIiiI',2**-28,-2**-27,2**-26,-2**-25,65536.,131072.,-65536.,262144.,2300,4096,2**-30,-2**-40,2301,7,18,19,0)
    if mid==1121:return struct.pack('<2diIIIIi2dII',2**-30,-2**-40,18,4096,2300,2301,7,19,2**-25,-2**-35,8192,2300)
    if mid==1127:return struct.pack('<3d5B',100.,-.125,.25,1,0,1,0,1)
    # N5 RAWWAASFRAME p418 / N1 RAWSBASFRAME p882,226bits+padding.
    return struct.pack('<III',1,120,63)+bytes.fromhex('53fc')+bytes(30)+struct.pack('<I',2)


class AuxiliarySpecification(unittest.TestCase):
    def test_sbf_documented_auxiliary_layouts_preserve_core_topics(self):
        profiles=[(mid,0) for mid in SBF_PAGES]+[(5902,r) for r in range(1,5)]+[(5924,1),(4258,0)]
        frames=[];expected=[]
        for i,(mid,rev) in enumerate(profiles):
            body=sbf_payload(mid,rev);tow=101000+1000*i
            # Full packet, truncated (at least4 bytes lost despite padding), badCRC.
            full=spec.sbf_spec(mid,body,rev);bad=bytearray(full);bad[2]^=1
            frames += [full+spec.sbf_spec(mid,body[:-4],rev)+bytes(bad),
                       spec.meas_spec(tow=tow)+spec.sbf_spec(5922,struct.pack('<IH2x',tow,2300))]
            expected.append(tow/1000)
        self.check('sbf',frames,expected)

    def test_oem_documented_auxiliary_layouts_preserve_core_topics(self):
        frames=[];expected=[]
        for i,mid in enumerate((8,1347,1121,1127,287,973)):
            p=oem_payload(mid);tow=101000+1000*i
            frames += [wire.oem(mid,p)+wire.oem(mid,p[:-1])+wire.oem(mid,p,bad=True),
                       spec.oem_spec([spec.range_row()],tow=tow)]
            expected.append(tow/1000)
        self.check('novatel',frames,expected)

    def check(self,family,frames,expected):
        for mode in ('decoder','driver'):
            with self.subTest(family=family,mode=mode):
                r=wire.run('aux_'+family+'_'+mode,family+'_'+mode+'_node',frames,
                    ['-p','topics.observation:=/review/obs','-p','topics.ephemeris:=/review/nav',
                     '-p','use_gps_timestamp:=true'],chunk_delay=.03)
                self.assertEqual([m.tow for m in r['obs']],expected)
                for m in r['obs']:
                    self.assertEqual(m.week,2300)
                    self.assertEqual(m.observations[0].satid,'G01')
                    self.assertAlmostEqual(m.observations[0].p,21000000.125,delta=1e-6)
                self.assertEqual(r['sol'],[]);self.assertEqual(r['raw'],[]);self.assertEqual(r['imu'],[])
                self.assertFalse(any(m.gnss_ephemeris or m.glonass_ephemeris for m in r['nav']))

if __name__=='__main__':unittest.main(argv=[__file__]+wire.unittest_args)
