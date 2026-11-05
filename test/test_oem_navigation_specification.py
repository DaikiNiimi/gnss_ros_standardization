#!/usr/bin/env python3
"""Selected decoded navigation tables: N1 pp520-521,623-625,741-743;
N3 pp434-435 (legacy 1122, not OEM7 GALINAV/FNAV replacements).
Fixtures follow published byte offsets; expected SI values are authored here.
Backend-specific loss of GLONASS flags/accuracy is explicitly not conformance.
"""
import math
import struct
import unittest
import test_receiver_streams as wire

PARAMS=['-p','topics.ephemeris:=/review/nav','-p','use_gps_timestamp:=true']
V=dict(sqrt_a=5153.5,e=.015625,omg=-.375,deln=2**-35,m0=-.125,omg0=.25,
       omgd=-2**-31,i0=.3125,idot=2**-34,cuc=-2**-20,cus=2**-21,crc=.75,
       crs=-.5,cic=-2**-22,cis=2**-23,f0=-2**-20,f1=2**-43,f2=-2**-55)

def put(body,offset,kind,value):struct.pack_into('<'+kind,body,offset,value)
def kepler(mid,clock=None,fnav=False):
    values=dict(V)
    if clock is not None:values['f0']=clock
    body=bytearray({1696:196,2123:204,1122:220}[mid])
    put(body,0,'I',1)
    if mid==1696:
        fixed={4:('I',944),8:('d',4.85),16:('I',1),20:('d',-2**-30),28:('d',2**-29),
               36:('I',21),40:('I',96000),68:('I',17),72:('I',96000)}
        offsets=dict(f0=44,f1=52,f2=60,sqrt_a=76,e=84,omg=92,deln=100,m0=108,
                     omg0=116,omgd=124,i0=132,idot=140,cuc=148,cus=156,crc=164,crs=172,cic=180,cis=188)
    elif mid==2123:
        fixed={4:('I',1276),32:('I',3),36:('I',96000),40:('d',-2**-30),56:('I',17),
               64:('I',1),68:('I',0),140:('I',96000)}
        offsets=dict(f0=8,f1=16,f2=24,deln=48,cuc=72,cus=80,cic=88,cis=96,crc=104,
                     crs=112,idot=120,m0=132,e=144,sqrt_a=152,omg0=160,omg=168,omgd=176,i0=184)
    else:
        fixed={4:('I',int(fnav)),8:('I',int(not fnav)),12:('B',1),13:('B',2),14:('B',3),
               15:('B',1),16:('B',0),17:('B',1),18:('B',4),20:('I',1023),24:('I',96000),
               148:('I',96000),176:('I',96000),204:('d',-2**-30),212:('d',2**-29)}
        offsets=dict(sqrt_a=28,deln=36,m0=44,e=52,omg=60,cuc=68,cus=76,crc=84,crs=92,
                     cic=100,cis=108,i0=116,idot=124,omg0=132,omgd=140)
        for i,k in enumerate(('f0','f1','f2')):
            put(body,152+8*i,'d',values[k] if fnav else 0.)
            put(body,180+8*i,'d',0. if fnav else values[k])
    for offset,(kind,value) in fixed.items():put(body,offset,kind,value)
    for name,offset in offsets.items():put(body,offset,'d',values[name])
    return bytes(body),values

def glo(channel=-7,health=3):
    body=bytearray(144)
    fields={0:('H',38),2:('H',channel+7),4:('B',1),6:('H',2300),8:('I',96000000),
            12:('I',10782),16:('H',1000),20:('I',17),24:('I',health),124:('I',20388),
            128:('I',3),132:('I',6),136:('I',7),140:('I',31)}
    for offset,(kind,value) in fields.items():put(body,offset,kind,value)
    for offset,values in [(28,(19100000.,-12300000.,21000000.)),(52,(-1000.,2000.,-500.)),
                          (76,(2**-20,-2**-21,2**-22)),(100,(-2**-20,2**-31,2**-30))]:
        for i,value in enumerate(values):put(body,offset+8*i,'d',value)
    return bytes(body)

class OemNavigationSpecification(unittest.TestCase):
    def test_long_header_extensions_preserve_navigation(self):
        body,values=kepler(1696)
        for mode in ('decoder','driver'):
            frames=[wire.oem(65534,b'\xaa\x44\x12'+bytes([0,0,0,0,0xff,0x7f])+bytes(20),short=True)]
            for size in (28,32,64,252):
                f=bytearray(wire.oem(1696,kepler(1696,clock=-size*2**-20)[0],tow=96006000))
                f[3]=size;f=f[:28]+bytes([0xa5])*(size-28)+f[28:-4]
                frames.append(bytes(f)+struct.pack('<I',wire.crc32(f)))
            result=wire.run('oem_header_'+mode,'novatel_'+mode+'_node',frames,PARAMS,chunk_delay=.08)
            nav=[m for m in result['nav'] if m.gnss_ephemeris]
            self.assertEqual(len(nav),4)
            for size,m in zip((28,32,64,252),nav):
                e=m.gnss_ephemeris[0]
                self.assertEqual((e.system,e.iode,e.week,e.toe),('C',17,2300,96014.))
                self.assertEqual(e.f0,-size*2**-20)

    def test_kepler_fields_high_iod_update_truncation_recovery(self):
        for mid,fnav in [(1696,False),(2123,False),(1122,False),(1122,True)]:
            first,values=kepler(mid,fnav=fnav)
            last,changed=kepler(mid,clock=-2**-19,fnav=fnav)
            frames=[wire.oem(mid,p,tow=96006000) for p in (first[:-1],first,first,last)]
            for mode in ('decoder','driver'):
                with self.subTest(id=mid,fnav=fnav,mode=mode):
                    result=wire.run(f'oem_nav_{mid}_{fnav}_{mode}','novatel_'+mode+'_node',frames,PARAMS+(['-p','messages.navic_ephem:=true'] if mode=='driver' else []),chunk_delay=.08)
                    snaps=[m for m in result['nav'] if m.gnss_ephemeris]
                    self.assertEqual(len(snaps),2)
                    for snap,expected in zip(snaps,(values,changed)):
                        self.assertEqual(len(snap.gnss_ephemeris),1)
                        e=snap.gnss_ephemeris[0];system={1696:'C',2123:'I',1122:'E'}[mid]
                        self.assertEqual((e.system,e.satid,e.prn),(system,system+'01',1))
                        toe=96014. if mid==1696 else 96000.
                        self.assertEqual((e.week,e.toe,e.toes,e.toc,e.ttr),(2300,toe,96000.,toe,96006.))
                        self.assertEqual((e.iode,e.iodc),(17,21) if mid==1696 else (17,17) if mid==2123 else (1023,1023))
                        self.assertEqual(e.sva,{1696:2,2123:3,1122:4}[mid])
                        self.assertEqual(e.svh,{1696:1,2123:2,1122:483}[mid])
                        self.assertEqual(e.code,258 if mid==1122 and fnav else 517 if mid==1122 else 0)
                        self.assertEqual(e.tgd[0],-2**-30)
                        self.assertEqual(e.tgd[1],0. if mid==2123 else 2**-29)
                        for name,value in expected.items():
                            actual=e.a if name=='sqrt_a' else getattr(e,name)
                            target=value*value if name=='sqrt_a' else value
                            self.assertTrue(math.isclose(actual,target,rel_tol=1e-14,abs_tol=1e-20),(mid,name,actual,target))

    def test_glonass_units_frequency_health_day_and_recovery(self):
        for mode in ('decoder','driver'):
            frames=[wire.oem(723,p,tow=96006000) for p in (glo()[:-1],glo(),glo(),glo(6,4))]
            result=wire.run('oem_glonav_'+mode,'novatel_'+mode+'_node',frames,PARAMS+(['-p','messages.navic_ephem:=true'] if mode=='driver' else []),chunk_delay=.08)
            snaps=[m for m in result['nav'] if m.glonass_ephemeris]
            self.assertEqual(len(snaps),2)
            for snapshot,channel,health in zip(snaps,(-7,6),(0,1)):
                e=snapshot.glonass_ephemeris[0]
                self.assertEqual((e.prn,e.frq,e.week,e.toe,e.tof,e.iode,e.age,e.svh),(1,channel,2300,96000.,96006.,17,7,health))
                self.assertEqual(list(e.pos),[19100000.,-12300000.,21000000.])
                self.assertEqual(list(e.vel),[-1000.,2000.,-500.])
                self.assertEqual(list(e.acc),[2**-20,-2**-21,2**-22])
                self.assertEqual((e.taun,e.dtaun,e.gamn),(-2**-20,2**-31,2**-30))
                # RTKLIB ignores wire Ft/flags. These zeros are backend defaults,
                # not the source's 6/31; keep the loss visible in the contract.
                self.assertEqual((e.sva,e.flags),(0,0))

    def test_ionutc_ionosphere_reaches_the_ephemeris_snapshot(self):
        ion=[2**-30,2**-27,-2**-24,-2**-24,2**11*59,2**14*10,-2**16*2,-2**16*5]
        body=bytearray(108)
        for i,value in enumerate(ion):put(body,8*i,'d',value)
        changed=list(ion);changed[0]=2**-29
        body2=bytearray(body);put(body2,0,'d',changed[0])
        for mode in ('decoder','driver'):
            with self.subTest(mode=mode):
                frames=[wire.oem(8,bytes(p),tow=96006000) for p in (body,body,body2)]
                result=wire.run('oem_ionutc_'+mode,'novatel_'+mode+'_node',frames,PARAMS,chunk_delay=.08)
                snaps=[list(m.ion_gps) for m in result['nav'] if any(m.ion_gps)]
                # One snapshot per change; a repeated identical set adds none.
                self.assertEqual(snaps[:2],[ion,changed])
                self.assertTrue(all(s==changed for s in snaps[1:]))
                self.assertTrue(all(not any(m.ion_qzs) for m in result['nav']))

if __name__=='__main__':unittest.main(argv=[__file__]+wire.unittest_args)
