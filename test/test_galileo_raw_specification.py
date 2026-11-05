#!/usr/bin/env python3
"""G3 Tables28-31/65/68/73 F/NAV page order and scales; S1 p280 envelope.

Receiver CRC/Viterbi status is an input; FEC/channel decoding is not simulated.
"""
import math
import struct
import unittest
import test_receiver_streams as wire
PARAMS=['-p','topics.ephemeris:=/review/nav','-p','use_gps_timestamp:=true']
# Width, signed raw integer, scale; angles are converted from semicircles.
VALUES=dict(m0=(32,-2345678,2**-31*math.pi),omgd=(24,-7654,2**-43*math.pi),
 e=(32,85899346,2**-33),sqrt_a=(32,2701918208,2**-19),omg0=(32,3456789,2**-31*math.pi),
 idot=(14,-123,2**-43*math.pi),i0=(32,654321098,2**-31*math.pi),omg=(32,-4567890,2**-31*math.pi),
 deln=(16,17,2**-43*math.pi),cuc=(16,-11,2**-29),cus=(16,13,2**-29),crc=(16,31,2**-5),
 crs=(16,-23,2**-5),cic=(16,-15,2**-29),cis=(16,21,2**-29),f0=(31,-12345,2**-34),
 f1=(21,19,2**-46),f2=(6,-3,2**-59),tgd=(10,-7,2**-32))

def pages(issue=1023,clock=-12345,toe=96000,tow=96000,week=1276):
    fields={1:[(6,1),(6,1),(10,issue),(14,toe//60),(31,clock),VALUES['f1'][:2],VALUES['f2'][:2],
               (8,4),(11,0),(11,0),(14,0),(5,0),VALUES['tgd'][:2],(2,2),(12,week),(20,tow),(1,1),(26,0)],
            2:[(6,2),(10,issue)]+[VALUES[k][:2] for k in ('m0','omgd','e','sqrt_a','omg0','idot')]+[(12,week),(20,tow+10)],
            3:[(6,3),(10,issue)]+[VALUES[k][:2] for k in ('i0','omg','deln','cuc','cus','crc','crs')]+[(14,toe//60),(12,week),(20,tow+20),(8,0)],
            4:[(6,4),(10,issue),VALUES['cic'][:2],VALUES['cis'][:2]]}
    # Page4 GST-UTC/GST-GPS fields and TOW (Table31).
    fields[4]+=[(n,0) for n in (32,24,8,8,8,8,3,8,8,16,12,6)]+[(20,tow+30),(5,0)]
    result={}
    for page,items in fields.items():
        bits=''.join(format(v&((1<<n)-1),f'0{n}b') for n,v in items)
        assert len(bits)==214,(page,len(bits))
        result[page]=int(bits+'0'*42,2).to_bytes(32,'big')
    return result

def envelope(data,source=20,passed=1):
    return [wire.sbf(4022,struct.pack('<IH6B8I',96006000,2300,71,passed,0,source,0,0,*struct.unpack('>8I',data[p]))) for p in sorted(data)]

def inav_words(issue=1023,clock=-12345,toe=96000,tow=96000):
    # G3 Tables40-45; the 128-bit word is split112/16 by Table36.
    fields={1:[(6,1),(10,issue),(14,toe//60)]+[VALUES[k][:2] for k in ('m0','e','sqrt_a')]+[(2,0)],
            2:[(6,2),(10,issue)]+[VALUES[k][:2] for k in ('omg0','i0','omg','idot')]+[(2,0)],
            3:[(6,3),(10,issue)]+[VALUES[k][:2] for k in ('omgd','deln','cuc','cus','crc','crs')]+[(8,4)],
            4:[(6,4),(10,issue),(6,1),VALUES['cic'][:2],VALUES['cis'][:2],(14,toe//60),
               (31,clock),VALUES['f1'][:2],VALUES['f2'][:2],(2,0)],
            5:[(6,5),(11,0),(11,0),(14,0),(5,0),VALUES['tgd'][:2],(10,9),(2,2),(2,1),(1,1),(1,1),(12,1276),(20,tow),(23,0)],
            6:[(6,6)]+[(width,0) for width in (32,24,8,8,8,8,3,8)]+[(20,tow),(3,0)]}
    result={}
    for word,items in fields.items():
        bits=''.join(format(v&((1<<width)-1),f'0{width}b') for width,v in items)
        assert len(bits)==128
        result[word]=bits
    return result

def inav_envelope(words,source=17,alert=False,bad_part=False):
    result=[]
    # Backend also consumes separate UTC word6; transmit it before word5.
    for word in (6,1,2,3,4,5):
        if word not in words:continue
        bits=('0'+str(int(alert))+words[word][:112]+('0' if bad_part else '1')+
              str(int(alert))+words[word][112:]+'0'*124)
        assert len(bits)==256
        data=int(bits,2).to_bytes(32,'big')
        result.append(wire.sbf(4023,struct.pack('<IH6B8I',96006000,2300,71,1,0,source,0,0,*struct.unpack('>8I',data))))
    return result

class GalileoRawSpecification(unittest.TestCase):
    def check(self,e,clock=-12345,week=2300,toe=96000.,tow=96000.,inav_source=None):
        self.assertEqual((e.system,e.prn,e.week,e.toe,e.toc,e.toes,e.ttr),('E',1,week,toe,toe,toe,tow))
        code=258 if inav_source is None else 517 if inav_source==49 else 513 if inav_source==17 else 516
        self.assertEqual((e.iode,e.iodc,e.sva,e.svh,e.code),(1023,1023,4,40 if inav_source is None else 323,code))
        for k,(_,raw,scale) in VALUES.items():
            target=(clock if k=='f0' else raw)*scale
            actual=e.a if k=='sqrt_a' else e.tgd[0] if k=='tgd' else getattr(e,k)
            if k=='sqrt_a':target*=target
            self.assertTrue(math.isclose(actual,target,rel_tol=2e-14,abs_tol=1e-20),(k,actual,target))
        self.assertTrue(math.isclose(e.tgd[1],0. if inav_source is None else 9*2**-32,rel_tol=2e-15,abs_tol=0.))

    def test_fields_high_iod_sources_duplicate_update(self):
        for source in (20,22):
            for mode in ('decoder','driver'):
                frames=envelope(pages(),source)*2+envelope(pages(clock=-12344),source)
                result=wire.run(f'fnav_{source}_{mode}','sbf_'+mode+'_node',frames,PARAMS,chunk_delay=.05)
                snaps=[m for m in result['nav'] if m.gnss_ephemeris]
                self.assertEqual(len(snaps),2)
                for snap,clock in zip(snaps,(-12345,-12344)):
                    self.assertEqual(len(snap.gnss_ephemeris),1);self.check(snap.gnss_ephemeris[0],clock)

    def test_missing_mixed_iod_crc_source_recovery(self):
        for mode in ('decoder','driver'):
            for fault in ('missing','issue','crc','source'):
                data=pages()
                if fault=='missing':del data[2]
                if fault=='issue':data[2]=pages(issue=700)[2]
                frames=envelope(data,source=17 if fault=='source' else 20,passed=0 if fault=='crc' else 1)+envelope(pages())
                result=wire.run('fnav_'+fault+'_'+mode,'sbf_'+mode+'_node',frames,PARAMS,chunk_delay=.05)
                snaps=[m for m in result['nav'] if m.gnss_ephemeris]
                self.assertEqual(len(snaps),1);self.check(snaps[0].gnss_ephemeris[0])

    def test_toe_in_next_gst_week(self):
        for mode in ('decoder','driver'):
            result=wire.run('fnav_week_'+mode,'sbf_'+mode+'_node',envelope(pages(toe=0,tow=604740)),PARAMS,chunk_delay=.05)
            snaps=[m for m in result['nav'] if m.gnss_ephemeris]
            self.assertEqual(len(snaps),1);self.check(snaps[0].gnss_ephemeris[0],week=2301,toe=0.,tow=604740.)

    def test_inav_fields_sources_update_and_high_iod(self):
        for source in (17,21,22,49):
            for mode in ('decoder','driver'):
                frames=inav_envelope(inav_words(),source)*2+inav_envelope(inav_words(clock=-12344),source)
                result=wire.run(f'inav_{source}_{mode}','sbf_'+mode+'_node',frames,PARAMS,chunk_delay=.04)
                snaps=[m for m in result['nav'] if m.gnss_ephemeris];self.assertEqual(len(snaps),2)
                for snap,clock in zip(snaps,(-12345,-12344)):self.check(snap.gnss_ephemeris[0],clock,inav_source=source)

    def test_inav_missing_mixed_iod_alert_parts_and_recovery(self):
        for mode in ('decoder','driver'):
            for fault in ('missing','issue','alert','part'):
                data=inav_words()
                if fault=='missing':del data[2]
                if fault=='issue':data[2]=inav_words(issue=700)[2]
                frames=inav_envelope(data,alert=fault=='alert',bad_part=fault=='part')+inav_envelope(inav_words())
                result=wire.run('inav_'+fault+'_'+mode,'sbf_'+mode+'_node',frames,PARAMS,chunk_delay=.04)
                snaps=[m for m in result['nav'] if m.gnss_ephemeris];self.assertEqual(len(snaps),1)
                self.check(snaps[0].gnss_ephemeris[0],inav_source=17)

    def test_inav_week_boundary(self):
        for mode in ('decoder','driver'):
            result=wire.run('inav_week_'+mode,'sbf_'+mode+'_node',inav_envelope(inav_words(toe=0,tow=604740)),PARAMS,chunk_delay=.04)
            snaps=[m for m in result['nav'] if m.gnss_ephemeris];self.assertEqual(len(snaps),1)
            self.check(snaps[0].gnss_ephemeris[0],week=2301,toe=0.,tow=604740.,inav_source=17)

if __name__=='__main__':unittest.main(argv=[__file__]+wire.unittest_args)
