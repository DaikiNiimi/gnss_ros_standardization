#!/usr/bin/env python3
"""I03 payload rejection/recovery through real receiver nodes.

Meas3 fixtures follow the pinned backend layouts, not independent hardware data.
RTCM fixtures use its encoder; altered headers are re-CRC'd to test payloads.
"""
import math
import pathlib
import struct
import subprocess
import unittest
import test_receiver_streams as wire
from test_receiver_observations import measurement, marker, PARAMS


def meas3(tow=100000, delta=False, bad_version=False):
    p=bytearray(17 if delta else 22)
    struct.pack_into('<IH',p,0,tow,2300)
    struct.pack_into('<H',p,8,1);p[10]=0x20;p[11]=32 if bad_version else 0
    if delta:
        struct.pack_into('<I',p,13,2|(1<<2)|(8192<<4)|(8192<<18))
    else:
        p[12]=1;p[13]=1
        struct.pack_into('<II',p,14,1|(131072<<1)|(1<<20)|(21<<23),2000000125)
    return wire.sbf(4109,p+b'\0'*((-len(p))%4))


def extra(ident, values, tow=100000):
    payload=struct.pack('<IH2x',tow,2300)+values
    return wire.sbf(ident,payload+b'\0'*((-len(payload))%4))


def rtcm_payload(payload):
    head=bytes([0xd3,len(payload)>>8,len(payload)&255])+payload
    crc=0
    for byte in head:
        crc ^= byte<<16
        for _ in range(8):
            crc <<= 1
            if crc&0x1000000:crc ^= 0x1864cfb
    return head+(crc&0xffffff).to_bytes(3,'big')


def bits(frame, start, count, value):
    data=bytearray(frame)
    for i in range(count):
        offset=start+i;mask=1<<(7-offset%8)
        data[offset//8]=(data[offset//8]&~mask)|(((value>>(count-1-i))&1)<<(7-offset%8))
    return rtcm_payload(data[3:-3])


def msm_frames():
    path=pathlib.Path(wire.TEMP.name)/'boundaries.rtcm3'
    subprocess.run([wire.B+'make_rtcm_fixture',str(path)],check=True)
    data=path.read_bytes();frames=[]
    while data:
        length=6+((data[1]&3)<<8)+data[2];frames.append(data[:length]);data=data[length:]
    assert len(frames)==2
    return frames


RTCM_PARAMS=['-p','reference_week:=2300','-p','reference_tow:=100.0','-p','use_gps_timestamp:=true']


class DecoderBoundaries(unittest.TestCase):
    def test_meas3_reference_delta_extras_rejection_and_recovery(self):
        frames=[meas3()+extra(4110,b'\x0c')+extra(4111,b'\x0f\x7d\x00\x00')+marker(),
                meas3(100100,True)+marker(100100),
                meas3(101000,bad_version=True)+meas3(102000)]
        for mode in ['decoder','driver']:
            m=wire.run('meas3_'+mode,'sbf_'+mode+'_node',frames,PARAMS,chunk_delay=.02)
            self.assertEqual(len(m['obs']),3)
            for epoch,tow in zip(m['obs'],[100.,100.1,102.]):
                self.assertAlmostEqual(epoch.tow,tow,places=9)
            first,delta,last=[e.observations[0] for e in m['obs']]
            self.assertEqual((first.p,first.freq,first.snr),(21000000.125,255,45.25))
            self.assertAlmostEqual(first.d,2*1575.42e6/299792458.,places=5)
            self.assertEqual(delta.p,first.p)
            self.assertTrue(math.isnan(delta.d) and math.isnan(last.d))

    def test_meas3_eof_and_disconnect_complete_last_epoch(self):
        for mode in ['decoder','driver']:
            m=wire.run('meas3_disconnect_'+mode,'sbf_'+mode+'_node',meas3(),PARAMS,disconnect=True,chunk_delay=0)
            self.assertEqual([e.tow for e in m['obs']],[100.])
        m=wire.run('meas3_eof','sbf_decoder_node',meas3(),
                   PARAMS+[],file_input=True)
        self.assertIn('published_epochs=1',m['log'])

    def test_rtcm_incomplete_and_invalid_final_never_publish_partial_epoch(self):
        gps,gal=msm_frames()
        good=[bits(f,48,30,101000) for f in [gps,gal]]
        for label,ending in [('missing',b''),('short',rtcm_payload(gal[3:-11])),
                             ('unsupported',bits(gal,24,12,1091)),
                             ('bad_time',bits(gal,48,30,604800000))]:
            with self.subTest(ending=label):
                m=wire.run('msm_'+label,'rtcm_decoder_node',[gps+ending,b''.join(good)],RTCM_PARAMS,chunk_delay=.02)
                self.assertEqual([e.tow for e in m['obs']],[101.])
                self.assertEqual({o.system for o in m['obs'][0].observations},{'G','E'})

    def test_rtcm_station_lock_ignores_foreign_fragment_and_bad_first_payload(self):
        gps,gal=msm_frames()
        # A corrupt payload may have a valid CRC; it must not choose the station.
        first=bits(rtcm_payload(gal[3:-11]),36,12,7)
        foreign=bits(gal,36,12,43)
        m=wire.run('msm_station','rtcm_decoder_node',[first,gps,foreign,gal],RTCM_PARAMS+['-p','station_id:=42'],chunk_delay=.02)
        self.assertEqual([e.tow for e in m['obs']],[100.])
        self.assertEqual({o.system for o in m['obs'][0].observations},{'G','E'})
        m=wire.run('msm_bad_initial_station','rtcm_decoder_node',[first,gps,gal],RTCM_PARAMS,chunk_delay=.02)
        self.assertEqual([e.tow for e in m['obs']],[100.])
        self.assertEqual({o.system for o in m['obs'][0].observations},{'G','E'})
        m=wire.run('msm_explicit_station','rtcm_decoder_node',gps+gal+bits(gps,36,12,43)+foreign,
                   RTCM_PARAMS+['-p','station_id:=43'])
        self.assertEqual([e.tow for e in m['obs']],[100.])
        self.assertEqual({o.system for o in m['obs'][0].observations},{'G','E'})

    def test_rtcm_changed_issue_does_not_merge_fragment_sets(self):
        gps,gal=msm_frames()
        m=wire.run('msm_iod','rtcm_decoder_node',gps+bits(gal,79,3,1),RTCM_PARAMS)
        self.assertEqual([e.tow for e in m['obs']],[100.])
        self.assertEqual({o.system for o in m['obs'][0].observations},{'E'})

    def test_compressed_range_and_rawx_recover_after_invalid_counts(self):
        header=struct.pack('<dHbBBB2s',100.,2300,18,1,1,1,b'\0\0')
        row=struct.pack('<ddf4BH6B',21000000.125,110000000.,0.,0,1,0,0,1000,45,0,0,0,7,0)
        bad=bytearray(header+row);bad[11]=2
        cmp=bytearray(28);struct.pack_into('<II',cmp,0,1,(1<<10)|(1<<11)|(1<<12));cmp[15]=10;cmp[21]=1
        for protocol,data in [('ubx',wire.ubx(2,0x15,bad)+wire.ubx(2,0x15,header+row)),
                              ('novatel',wire.oem(140,struct.pack('<I',0xffffffff))+wire.oem(140,cmp))]:
            for mode in ['decoder','driver']:
                m=wire.run('count_'+protocol+mode,protocol+'_'+mode+'_node',data,PARAMS)
                self.assertEqual([e.tow for e in m['obs']],[100.])
                self.assertGreater(m['obs'][0].observations[0].p,0)
                self.assertEqual(m['obs'][0].observations[0].d,0.)


if __name__=='__main__':
    unittest.main(argv=[wire.sys.argv[0]]+wire.unittest_args,verbosity=2)
