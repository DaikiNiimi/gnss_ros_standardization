#!/usr/bin/env python3
"""Published single-signal mappings and SI values, independent of decoder tables.
S1 p260; U1 p20 (U5 selected B3/E6 additions); N1 pp832-834; F1 RINEX A2.
One signal per epoch avoids conflating mapping with six-slot arbitration tests.
Known backend selection limits remain: UBX E6/B3; OEM E6C/AltBOC/QZSS L6.
Their input must not create a stale observation. No new upstream signal support.
OEM preserves the supplied GLO FCN on L3 as well (no FDMA use implied).
"""
import math
import struct
import unittest
import test_receiver_streams as wire
import test_protocol_specification as spec

SBF=[('G',s,c) for s,c in enumerate(('1C','1W','2W','2L','5Q','1L'))]
SBF += [('J',6,'1C'),('J',7,'2L')]+[('R',s,c) for s,c in zip(range(8,13),('1C','1P','2P','2C','3Q'))]
SBF += [('C',13,'1P'),('C',14,'5P'),('I',15,'5A'),('E',17,'1C'),('E',19,'6C'),('E',20,'5Q'),('E',21,'7Q'),('E',22,'8Q'),('S',24,'1C'),('S',25,'5I'),('J',26,'5Q'),('J',27,'6L'),('C',28,'2I'),('C',29,'7I'),('C',30,'6I'),('J',32,'1L'),('J',33,'1Z'),('C',34,'7D'),('J',38,'1E'),('J',39,'5P')]
OEM={'G':[(0,'1C'),(5,'2P'),(9,'2W'),(14,'5Q'),(16,'1L'),(17,'2S')],
     'R':[(0,'1C'),(1,'2C'),(5,'2P'),(6,'3Q')],
     'E':[(2,'1C'),(6,'6B'),(7,'6C'),(12,'5Q'),(17,'7Q'),(20,'8Q')],
     'C':[(0,'2I'),(1,'7I'),(2,'6I'),(4,'2I'),(5,'7I'),(6,'6I'),(7,'1P'),(9,'5P'),(11,'7D')],
     'J':[(0,'1C'),(14,'5Q'),(16,'1L'),(17,'2S'),(27,'6L')],
     'S':[(0,'1C'),(6,'5I')],'I':[(0,'5A')]}
UBX={'G':[(0,'1C'),(3,'2L'),(4,'2S'),(6,'5I'),(7,'5Q')],
     'R':[(0,'1C'),(2,'2C')],'E':[(0,'1C'),(1,'1B'),(3,'5I'),(4,'5Q'),(5,'7I'),(6,'7Q'),(8,'6B'),(9,'6C'),(10,'6A')],
     'C':[(0,'2I'),(1,'2I'),(2,'7I'),(3,'7I'),(4,'6I'),(5,'1P'),(6,'1D'),(7,'5P'),(8,'5D'),(10,'6I')],
     'J':[(0,'1C'),(1,'1Z'),(4,'2S'),(5,'2L'),(8,'5I'),(9,'5Q')],'S':[(0,'1C')],'I':[(0,'5A')]}


def frequency(system,code):
    if system=='R':return {'1':1602e6-7*562500,'2':1246e6-7*437500,'3':1202.025e6}[code[0]]
    if system=='C' and code[0]=='2':return 1561.098e6
    if system=='C' and code[0]=='6':return 1268.52e6
    return {'1':1575.42e6,'2':1227.60e6,'5':1176.45e6,'6':1278.75e6,'7':1207.14e6,'8':1191.795e6}[code[0]]

def sbf_single(system,sig,tow,antenna=0):
    svid={'G':1,'R':38,'E':71,'C':141,'J':181,'S':120,'I':191}[system]
    info=4+(8 if sig in (8,9,10,11) else ((sig-32)<<3) if sig>=32 else 0)
    signal=31 if sig>=32 else sig
    mm=21000000125
    row=struct.pack('<BBBBIiHbBHBB',1,signal|(antenna<<5),svid,mm>>32,mm&0xffffffff,-1235000,
                    53191,-1,180 if sig in (1,2) else 140,10,info,0)
    return spec.sbf_spec(4027,struct.pack('<IH6B',tow,2300,1,20,12,0,0,0)+row,1)+spec.sbf_spec(5922,struct.pack('<IH2x',tow,2300))

class ObservationSignalSpecification(unittest.TestCase):
    def test_sbf_all_documented_compiled_signals_and_secondary_antenna_isolation(self):
        frames=[];expected=[]
        for i,(system,sig,code) in enumerate(SBF):
            tow=100000+i*1000
            frames += [sbf_single(system,sig,tow,1),sbf_single(system,sig,tow)]
            expected.append((system,code,tow/1000,21000000.125,21000000.125*frequency(system,code)/299792458-12.345))
        self.check('sbf',frames,expected,half=True)

    def test_ubx_documented_signals_including_b3_e6_and_navic(self):
        frames=[];expected=[]
        for system,rows in UBX.items():
            for sig,code in rows:
                t=100.+len(frames);gnss={'G':0,'S':1,'E':2,'C':3,'J':5,'R':6,'I':7}[system]
                frames.append(spec.frame(struct.pack('<dHbBBB2x',t,2300,18,1,1,1)+spec.row(120 if system=='S' else 1,gnss=gnss,signal=sig)))
                if not (system=='E' and sig in (8,9,10) or system=='C' and sig in (4,10)):
                    # Preserve documented repository/RTKLIB BDS GEO phase convention,
                    # not a U1 wire-value equivalence claim.
                    expected.append((system,code,t,21000000.125,-12344.75 if system=='C' else -12345.25))
        self.check('ubx',frames,expected)

    def test_oem_range_and_rangecmp_all_documented_compiled_signals(self):
        for compressed in (False,True):
            frames=[];expected=[]
            for system,rows in OEM.items():
                for sig,code in rows:
                    sys={'G':0,'R':1,'S':2,'E':3,'C':4,'J':5,'I':6}[system]
                    prn=38 if system=='R' else 193 if system=='J' else 120 if system=='S' else 1
                    flags=0x1c00|(sig<<21);t=100.+len(frames)
                    if compressed:
                        row=spec.rangecmp_row(prn,0,sys,flags)
                        phase=math.trunc((21200000.125*frequency(system,code)/299792458-12345.25)/8388608+.5)*8388608+12345.25
                    else:row=spec.range_row(prn,0,flags|(sys<<16));phase=110000000.25
                    frames.append(spec.oem_spec([row],mid=140 if compressed else 43,tow=round(t*1000)))
                    if not (system=='E' and sig in (7,20) or system=='J' and sig==27):
                        expected.append((system,code,t,21200000.125 if compressed else 21000000.125,phase))
            self.check('novatel',frames,expected)

    def check(self,family,frames,expected,half=False):
        for mode in ('decoder','driver'):
            with self.subTest(family=family,mode=mode):
                r=wire.run('signal_'+family+'_'+mode,family+'_'+mode+'_node',frames,spec.PARAMS,chunk_delay=.02)
                self.assertEqual(len(r['obs']),len(expected))
                for m,(system,code,t,p,l) in zip(r['obs'],expected):
                    with self.subTest(system=system,code=code,tow=t):
                        self.assertEqual((m.week,m.tow),(2300,t));self.assertEqual(len(m.observations),1)
                        o=m.observations[0];self.assertEqual((o.system,o.code_str),(system,code))
                        self.assertAlmostEqual(o.p,p,delta=1e-6);self.assertAlmostEqual(o.l,l,delta=1e-6)
                        self.assertEqual((o.d,o.snr),(-123.5,45.))
                        self.assertEqual(o.freq,0 if system=='R' and (code[0] in '12' or family=='novatel') else 255)
                        self.assertEqual(bool(o.lli&2),half or (family=='ubx' and system=='S'))

if __name__=='__main__':unittest.main(argv=[__file__]+wire.unittest_args)
