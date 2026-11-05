#!/usr/bin/env python3
"""G5 Figure11/Tables11-12 (pp16,19-20); S1 NAVICRaw p292.

Bits are serialized in table order. Receiver CRC/FEC status is supplied by SBF,
not independently channel-decoded. Week is the documented ten-bit count.
"""
import math
import struct
import unittest
import test_receiver_streams as wire
PARAMS=['-p','topics.ephemeris:=/review/nav','-p','use_gps_timestamp:=true']
VALUES=dict(f0=(22,-12345,2**-31),f1=(16,19,2**-43),f2=(8,-3,2**-55),
 tgd=(8,-7,2**-31),deln=(22,17,2**-41*math.pi),cuc=(15,-11,2**-28),cus=(15,13,2**-28),
 cic=(15,-15,2**-28),cis=(15,21,2**-28),crc=(15,31,2**-4),crs=(15,-23,2**-4),idot=(14,-123,2**-43*math.pi),
 m0=(32,-2345678,2**-31*math.pi),e=(32,85899346,2**-33),sqrt_a=(32,2701918208,2**-19),
 omg0=(32,3456789,2**-31*math.pi),omg=(32,-4567890,2**-31*math.pi),omgd=(22,-7654,2**-41*math.pi),i0=(32,654321098,2**-31*math.pi))

def subframes(clock=-12345,toe=96000,toc=96000,tow=96012,second_delta=12):
    payloads=[[(10,252),(22,clock),VALUES['f1'][:2],VALUES['f2'][:2],(4,3),(16,toc//16),VALUES['tgd'][:2],
               VALUES['deln'][:2],(8,17),(10,0),(1,1),(1,0)]+[VALUES[k][:2] for k in ('cuc','cus','cic','cis','crc','crs','idot')]+[(2,0)],
              [VALUES['m0'][:2],(16,toe//16)]+[VALUES[k][:2] for k in ('e','sqrt_a','omg0','omg','omgd','i0')]+[(2,0)]]
    result=[]
    for sf,body in enumerate(payloads):
        items=[(8,0),(17,(tow+sf*second_delta)//12),(1,0),(1,0),(2,sf),(1,0)]+body
        bits=''.join(format(v&((1<<n)-1),f'0{n}b') for n,v in items)
        assert len(bits)==262
        result.append(int(bits+'0'*58,2).to_bytes(40,'big'))
    return result

def envelope(frames,passed=1):
    return [wire.sbf(4093,struct.pack('<IH6B10I',96006000,2300,191,passed,0,15,0,0,*struct.unpack('>10I',frame))) for frame in frames]

class NavicRawSpecification(unittest.TestCase):
    def check(self,e,clock=-12345):
        self.assertEqual((e.system,e.prn,e.week,e.toe,e.toes,e.toc,e.ttr),('I',1,2300,96000.,96000.,96000.,96012.))
        self.assertEqual((e.iode,e.iodc,e.sva,e.svh),(17,17,3,2))
        for k,(_,raw,scale) in VALUES.items():
            expected=(clock if k=='f0' else raw)*scale
            actual=e.a if k=='sqrt_a' else e.tgd[0] if k=='tgd' else getattr(e,k)
            if k=='sqrt_a':expected*=expected
            self.assertTrue(math.isclose(actual,expected,rel_tol=2e-14,abs_tol=1e-20),(k,actual,expected))

    def test_fields_duplicate_same_issue_clock_update(self):
        for mode in ('decoder','driver'):
            result=wire.run('navic_raw_'+mode,'sbf_'+mode+'_node',envelope(subframes())*2+envelope(subframes(clock=-12344)),PARAMS,chunk_delay=.08)
            snaps=[m for m in result['nav'] if m.gnss_ephemeris]
            self.assertEqual(len(snaps),2)
            for snap,clock in zip(snaps,(-12345,-12344)):
                self.assertEqual(len(snap.gnss_ephemeris),1);self.check(snap.gnss_ephemeris[0],clock)

    def test_missing_mismatched_times_crc_and_recovery(self):
        for mode in ('decoder','driver'):
            for fault in ('missing','toc','sequence','crc'):
                data=subframes(toc=96016 if fault=='toc' else 96000,second_delta=24 if fault=='sequence' else 12)
                if fault=='missing':data=data[1:]
                result=wire.run('navic_'+fault+'_'+mode,'sbf_'+mode+'_node',
                    envelope(data,passed=0 if fault=='crc' else 1)+envelope(subframes()),PARAMS,chunk_delay=.08)
                snaps=[m for m in result['nav'] if m.gnss_ephemeris]
                self.assertEqual(len(snaps),1);self.check(snaps[0].gnss_ephemeris[0])

if __name__=='__main__':unittest.main(argv=[__file__]+wire.unittest_args)
