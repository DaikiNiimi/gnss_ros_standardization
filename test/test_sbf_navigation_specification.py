#!/usr/bin/env python3
"""S1 decoded navigation tables -> packed fields -> both ROS paths.

GPSNav pp293-294, GLONav p301, GALNav pp304-306, BDSNav pp312-313,
QZSNav pp323-324. Fields are packed sequentially in the published table order.
Physical test values are chosen independently, using exact binary fractions.
"""
import math
import struct
import unittest
import test_receiver_streams as wire

PARAMS=['-p','topics.ephemeris:=/review/nav','-p','use_gps_timestamp:=true']
VALUES=dict(crs=-.5,deln=2**-35,m0=-.125,cuc=-2**-20,e=.015625,cus=2**-21,
            sqrt_a=5153.5,cic=-2**-22,omg0=.25,cis=2**-23,i0=.3125,
            crc=.75,omg=-.375,omgd=-2**-31,idot=2**-34,f2=-2**-55,f1=2**-43,f0=-2**-20)
ORBITS=[('crs','f'),('deln','f'),('m0','d'),('cuc','f'),('e','d'),('cus','f'),('sqrt_a','d'),
        ('toe','I'),('cic','f'),('omg0','d'),('cis','f'),('i0','d'),('crc','f'),('omg','d'),('omgd','f'),('idot','f')]
ANGLES={'deln','m0','omg0','i0','omg','omgd','idot'}

def pack(fields):return b''.join(struct.pack('<'+kind,value) for kind,value in fields)
def wrapped(mid,body):return wire.sbf(mid,body+bytes((-len(body))%4))

def kepler(mid,issue=17,clock=None,source=2):
    values=dict(VALUES,toe=96000)
    if clock is not None:values['f0']=clock
    svid={5891:1,4095:181,4081:141,4002:71}[mid]
    fields=[('I',96006000),('H',2300),('B',svid),('B',source if mid==4002 else 0)]
    if mid in (5891,4095):
        fields += [('H',252),('B',2),('B',3),('B',5),('B',1),('H',issue),
                   ('B',issue&255),('B',issue&255),('B',1),('B',0),('f',-2**-30),('I',96000)]
        fields += [('f',values[k]) for k in ('f2','f1','f0')]
        fields += [(kind,values[name]) for name,kind in ORBITS]
        fields += [('H',252),('H',252)]
    elif mid==4081:
        fields += [('H',944),('B',3),('B',1),('B',issue),('B',issue),('H',0),
                   ('f',-2**-30),('f',2**-29),('I',96000)]
        fields += [('f',values[k]) for k in ('f2','f1','f0')]
        fields += [(kind,values[name]) for name,kind in ORBITS]
        fields += [('H',944),('H',944)]
    else:
        fields += [('d',values[k]) for k in ('sqrt_a','m0','e','i0','omg','omg0')]
        fields += [('f',values[k]) for k in ('omgd','idot','deln','cuc','cus','crc','crs','cic','cis')]
        fields += [('I',96000),('I',96000),('f',values['f2']),('f',values['f1']),('d',values['f0']),
                   ('H',2300),('H',2300),('H',issue),('H',0x111|2|0x40|0x800),
                   ('B',0),('B',3),('B',4),('B',255),('f',-2**-30),('f',2**-29),('f',-2e10),('B',255)]
    return pack(fields),values

def glonass(revision,channel=-7,c=1):
    fields=[('I',96006000),('H',2300),('B',38),('B',channel+8)]
    fields += [('d',v) for v in (19100.,-12300.,21000.)]
    fields += [('f',v) for v in (-1.,2.,-.5,2**-20,-2**-21,2**-22,2**-30,-2**-20,2**-31)]
    fields += [('I',96000),('H',252),('B',45),('B',1),('B',7),('B',4),('H',180),
               ('B',1),('B',3),('B',1),('B',1),('H',100),('H',1000)]
    if revision:fields+=[('B',c)]
    body=pack(fields);return body+bytes((-len(body))%4)

class SbfNavigationSpecification(unittest.TestCase):
    def check_kepler(self,e,mid,values,issue,source):
        system={5891:'G',4095:'J',4081:'C',4002:'E'}[mid]
        self.assertEqual((e.system,e.satid,e.prn),(system,system+'01',193 if system=='J' else 1))
        toe=96014. if mid==4081 else 96000.
        self.assertEqual((e.week,e.toe,e.toes,e.toc,e.ttr),(2300,toe,96000.,toe,96006.))
        self.assertEqual(e.iode,issue)
        # GALNav has IODnav but no separate IODC; backend default remains0.
        self.assertEqual(e.iodc,0 if mid==4002 else issue)
        self.assertEqual(e.code,(517 if source==2 else 258) if mid==4002 else 0 if mid==4081 else 2)
        self.assertEqual(e.sva,(4 if source==2 else 3) if mid==4002 else 3)
        self.assertEqual(e.svh,161 if mid==4002 else 1 if mid==4081 else 5)
        self.assertEqual(e.fit,6 if mid==5891 else 4 if mid==4095 else 0)
        self.assertEqual(e.flag,1 if mid in (5891,4095) else 0)
        self.assertEqual(e.tgd[0],-2**-30)
        if mid in (4081,4002):self.assertEqual(e.tgd[1],2**-29)
        for name,value in values.items():
            if name=='toe':continue
            actual=e.a if name=='sqrt_a' else getattr(e,name)
            expected=value**2 if name=='sqrt_a' else value*math.pi if name in ANGLES else value
            self.assertTrue(math.isclose(actual,expected,rel_tol=2e-14,abs_tol=1e-20),(mid,name,actual,expected))
        self.assertEqual((e.adot,e.ndot),(0.,0.))

    def test_compatible_kepler_extensions_ignore_reserved_padding(self):
        for mid,source,issue in ((5891,0,17),(4095,0,17),(4081,0,17),(4002,2,1023)):
            body,values=kepler(mid,issue,source=source)
            body=bytearray(body)
            if mid!=4002:body[7]=0xa5  # Reserved in these three layouts
            frames=[wrapped(mid|(7<<13),bytes(body)+b'\xa5'*8)]
            for mode in ('decoder','driver'):
                r=wire.run(f'sbf_extension_{mid}_{mode}','sbf_'+mode+'_node',frames,PARAMS)
                nav=[m for m in r['nav'] if m.gnss_ephemeris]
                self.assertEqual(len(nav),1)
                self.check_kepler(nav[0].gnss_ephemeris[0],mid,values,issue,source)

    def test_kepler_fields_sources_high_iod_update_short_and_recovery(self):
        for mid,source,issue in [(5891,0,17),(4095,0,17),(4081,0,17),(4002,2,1023),(4002,16,700)]:
            first,values=kepler(mid,issue,source=source)
            last,changed=kepler(mid,issue,clock=-2**-19,source=source)
            frames=[wrapped(mid,first[:-4]),wrapped(mid,first),wrapped(mid,first),wrapped(mid,last)]
            for mode in ('decoder','driver'):
                with self.subTest(id=mid,source=source,mode=mode):
                    result=wire.run(f'sbf_nav_{mid}_{source}_{mode}','sbf_'+mode+'_node',frames,PARAMS,chunk_delay=.08)
                    snaps=[m for m in result['nav'] if m.gnss_ephemeris]
                    self.assertEqual(len(snaps),2)
                    for snap,expected in zip(snaps,(values,changed)):
                        self.assertEqual(len(snap.gnss_ephemeris),1)
                        self.check_kepler(snap.gnss_ephemeris[0],mid,expected,issue,source)

    def test_glonass_revisions_units_channels_flags_and_accuracy(self):
        # S1 4.1.6/4.1.12: later revisions retain existing fields and may
        # append fields. These are compatibility probes, not new FW claims.
        for rev in (0,1,2,7):
            for mode in ('decoder','driver'):
                frames=[wrapped(4004|(rev<<13),glonass(rev,channel)+
                        (bytes([0xa5])*7 if rev>1 else b'')) for channel in (-7,6)]
                result=wire.run(f'sbf_glonav_{rev}_{mode}','sbf_'+mode+'_node',frames,PARAMS,chunk_delay=.08)
                snaps=[m for m in result['nav'] if m.glonass_ephemeris]
                self.assertEqual(len(snaps),2)
                for snapshot,channel in zip(snaps,(-7,6)):
                    self.assertEqual(len(snapshot.glonass_ephemeris),1)
                    e=snapshot.glonass_ephemeris[0]
                    self.assertEqual((e.system,e.prn,e.week,e.frq,e.iode,e.age,e.sva),('R',1,2300,channel,12,7,6))
                    self.assertEqual((e.toe,e.tof),(96000.,96006.))
                    self.assertEqual(list(e.pos),[19100000.,-12300000.,21000000.])
                    self.assertEqual(list(e.vel),[-1000.,2000.,-500.])
                    self.assertEqual(list(e.acc),[1000*2**-20,-1000*2**-21,1000*2**-22])
                    self.assertEqual((e.gamn,e.taun,e.dtaun),(2**-30,-2**-20,2**-31))
                    self.assertEqual(e.flags,219) # M/P4/P2/P1/P, no P3 field in SBF
                    self.assertEqual(e.svh,15 if rev else 9)

    def test_qzss_identity_and_issue_consistency_before_recovery(self):
        body,expected=kepler(4095)
        wrong_sat=bytearray(body);wrong_sat[6]=1 # GPS SVID in a QZSS block
        wrong_issue=bytearray(body);wrong_issue[17]=18 # subframe3 IODE != subframe2
        struct.pack_into('<f',wrong_issue,36,-2**-19)
        for mode in ('decoder','driver'):
            for name,bad in [('satellite',wrong_sat),('issue',wrong_issue)]:
                with self.subTest(mode=mode,fault=name):
                    result=wire.run('sbf_qzss_invalid_'+name+'_'+mode,'sbf_'+mode+'_node',
                        [wrapped(4095,bad),wrapped(4095,body)],PARAMS,chunk_delay=.08)
                    snaps=[m for m in result['nav'] if m.gnss_ephemeris]
                    self.assertEqual(len(snaps),1)
                    self.assertEqual(len(snaps[0].gnss_ephemeris),1)
                    self.check_kepler(snaps[0].gnss_ephemeris[0],4095,expected,17,0)

if __name__=='__main__':unittest.main(argv=[__file__]+wire.unittest_args)
