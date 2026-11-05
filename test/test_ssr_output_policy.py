#!/usr/bin/env python3
"""R3 IGS SSR v1.00 Tables8-23; R4 L6-003 Table4.1.2-30.
Only core Topic isolation/recovery is promised: no SSR fields in fixed6 types.
"""
import json
import math
from pathlib import Path
import unittest
import test_receiver_streams as wire
from test_decoder_boundaries import rtcm_payload,RTCM_PARAMS
from test_rtcm_reference import msm

class Bits:
    def __init__(self):self.parts=[]
    def put(self,value,width):
        assert -(1<<(width-1))<=value<(1<<width)
        self.parts.append(format(value&((1<<width)-1),'0'+str(width)+'b'))
    def frame(self):
        s=''.join(self.parts);s+='0'*(-len(s)%8)
        return rtcm_payload(int(s,2).to_bytes(len(s)//8,'big'))

def igs(subtype):
    b=Bits();kind=subtype%20
    for v,w in [(4076,12),(1,3),(subtype,8),(100,20),(0,4),(0,1),(3,4),(12,16),(2,4)]:b.put(v,w)
    if kind in (1,3):b.put(0,1) # global CRS
    if kind==6:b.put(0,1);b.put(0,1) # dispersive and Melbourne-Wubbena consistency
    b.put(1,6);b.put(1,6) # one satellite, ID1 (system-specific numbering)
    if kind in (1,3):
        b.put(17,8)
        for v,w in [(100,22),(-50,20),(25,20),(-2,21),(3,19),(-4,19)]:b.put(v,w)
    if kind in (2,3):
        for v,w in [(-100,22),(50,21),(-25,27)]:b.put(v,w)
    if kind==4:b.put(100,22)
    if kind==5:
        for v,w in [(1,5),(0,5),(-100,14)]:b.put(v,w)
    if kind==6:
        for v,w in [(1,5),(100,9),(-2,8),(0,5),(1,1),(0,2),(3,4),(-123,20)]:b.put(v,w)
    if kind==7:b.put(5,6)
    return b.frame()

def compact_service(counter,size):
    b=Bits()
    for v,w in [(4073,12),(10,4),(0,1),(counter,3),(size,2)]:b.put(v,w)
    b.put(0,40*(size+1)) # payload meaning is outside the no-output contract
    return b.frame()

class SsrOutputPolicy(unittest.TestCase):
    def exercise_public_reference(self, malformed):
        table = json.loads(Path(__file__).with_name('rtcm_ssr_reference.json').read_text())
        self.assertEqual({c['id'] for c in table['cases']}, set(range(1240, 1264)))
        self.assertEqual(len(table['cases']), 72)
        frames = []
        for case in table['cases']:
            frame = bytes.fromhex(case['frame_hex'])
            self.assertEqual((frame[3] << 4) | (frame[4] >> 4), case['id'])
            if malformed:
                if case['variant'] != 'multiple':
                    continue
                # Keep the declared count but remove the final payload octet,
                # or keep only the ID. Recompute CRC so length checks are used.
                frames.extend([rtcm_payload(frame[3:-4]), rtcm_payload(frame[3:5]),
                               frame[:-1] + bytes([frame[-1] ^ 1])])
            else:
                frames.append(frame)
        chunks = []
        for i, frame in enumerate(frames):
            chunks.extend([frame, msm(7, tow_ms=100000 + i * 1000)])
        result = wire.run('ssr_reference_' + ('bad' if malformed else 'public'),
                          'rtcm_decoder_node', chunks,
                          RTCM_PARAMS + ['-p', 'topics.ephemeris:=/review/nav'],
                          chunk_delay=.015)
        self.assertEqual(len(result['obs']), len(frames))
        wavelength = 299792458. / 1575.42e6
        for i, epoch in enumerate(result['obs']):
            self.assertEqual((epoch.week, epoch.tow), (2300, 100. + i))
            self.assertEqual(len(epoch.observations), 1)
            obs = epoch.observations[0]
            self.assertEqual((obs.system, obs.prn, obs.code_str, obs.freq), ('G', 1, '1C', 255))
            self.assertAlmostEqual(obs.p, (70.5 - 512 * 2**-29) * 299792.458, delta=1e-6)
            self.assertAlmostEqual(obs.l, (70.5 + 4096 * 2**-31) * 299792.458 / wavelength, delta=1e-6)
            self.assertTrue(math.isclose(obs.d, 100.1234 / wavelength, rel_tol=1e-7, abs_tol=1e-5))
            self.assertEqual(obs.snr, 45.0625)
            self.assertEqual(obs.lli & 2, 2)
        self.assertFalse(result['sol'] or result['raw'] or result['imu'])
        self.assertFalse(any(e.gnss_ephemeris or e.glonass_ephemeris for e in result['nav']))

    def test_public_1240_1263_dialect_isolated_and_recovers(self):
        # QZSS count-width divergence is deliberately retained, so this is an
        # isolation/recovery assertion, never a hidden-SSR-decoding assertion.
        self.exercise_public_reference(False)

    def test_public_1240_1263_short_payload_and_bad_crc_recover(self):
        self.exercise_public_reference(True)

    def test_published_igs_subtypes_and_compact_service_recover(self):
        cases=[igs(base+kind) for base in (20,40,60,80,100,120) for kind in range(1,8)]
        cases += [compact_service(counter,size) for counter in (0,7) for size in range(4)]
        chunks=[]
        for i,f in enumerate(cases):
            chunks += [f+rtcm_payload(f[3:5])+f[:-1]+bytes([f[-1]^1]),msm(7,tow_ms=100000+1000*i)]
        r=wire.run('public_ssr','rtcm_decoder_node',chunks,
            RTCM_PARAMS+['-p','topics.ephemeris:=/review/nav'],chunk_delay=.015)
        self.assertEqual([e.tow for e in r['obs'] if e.observations],[100.+i for i in range(len(cases))])
        for e in r['obs']:
            self.assertEqual(e.week,2300)
            self.assertEqual(len(e.observations),1)
            self.assertEqual((e.observations[0].system,e.observations[0].code_str),('G','1C'))
            self.assertAlmostEqual(e.observations[0].p,(70.5-512*2**-29)*299792.458,delta=1e-6)
        self.assertFalse(r['sol'] or r['raw'] or r['imu'])
        self.assertFalse(any(e.gnss_ephemeris or e.glonass_ephemeris for e in r['nav']))

if __name__=='__main__':unittest.main(argv=[__file__]+wire.unittest_args)
