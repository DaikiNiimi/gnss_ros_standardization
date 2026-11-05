#!/usr/bin/env python3
"""U2 section10, U1 RXM-SFRBX v2; G3/G6/G7 satellite bits to ROS.

Uses independently authored ICD fields from the SBF specification cases, but
packs the UBX words independently. No production encoder or parity helper.
"""
import struct
import unittest
import test_receiver_streams as wire
import test_beidou_raw_specification as bds
import test_galileo_raw_specification as gal
import test_glonass_raw_specification as glo
from test_ubx_legacy_specification import frame as anchor


def crc24(bits):
    # G3 CRC-24Q: zero initial state, x^24+x^23+x^18+x^17+...+1.
    reg = 0
    for bit in bits:
        feedback = (reg >> 23) ^ int(bit)
        reg = (reg << 1) & 0xffffff
        if feedback:
            reg ^= 0x864cfb
    return reg


def frame(gnss, sv, signal, words, channel=0):
    return wire.ubx(2, 0x13, bytes([gnss, sv, signal, channel, len(words), 0, 2, 0]) +
                    struct.pack('<' + 'I' * len(words), *words))


def beidou(model, missing=False):
    data = bds.pages(model)
    if missing:
        del data[1]
    frames = []
    for p in sorted(data):
        bits = format(int.from_bytes(data[p], 'big'), '0320b')[:300]
        frames.append(frame(3, 6 if model == 1 else 1, 0,
                            [int(bits[i:i+30], 2) for i in range(0, 300, 30)]))
    return frames


def galileo(kind, fault=False):
    frames = []
    if kind == 'fnav':
        for p, data in gal.pages().items():
            if fault and p == 2:
                continue
            bits = format(int.from_bytes(data, 'big'), '0256b')[:214]
            bits += format(crc24(bits), '024b') + '0' * 18
            frames.append(frame(2, 1, 3, [int(bits[i:i+32], 2) for i in range(0, 256, 32)]))
    else:
        words = gal.inav_words()
        for p in (6, 1, 2, 3, 4, 5):
            even = '00' + words[p][:112]
            odd = '10' + words[p][112:] + '0' * 64
            crc = crc24(even + odd) ^ (1 if fault else 0)
            bits = even + '0' * 14 + odd + format(crc, '024b') + '0' * 22
            frames.append(frame(2, 1, 0, [int(bits[i:i+32], 2) for i in range(0, 256, 32)]))
    return frames


def glonass(fault=False):
    # G6 section4.7 checksum equations, numbered transmitted bits (LSB = 1).
    groups = [
        [9,10,12,13,15,17,19,20,22,24,26,28,30,32,34,35,37,39,41,43,
         45,47,49,51,53,55,57,59,61,63,65,66,68,70,72,74,76,78,80,82,84],
        [9,11,12,14,15,18,19,21,22,25,26,29,30,33,34,36,37,40,41,44,
         45,48,49,52,53,56,57,60,61,64,65,67,68,71,72,75,76,79,80,83,84],
        [i for a,b in [(10,12),(16,19),(23,26),(31,34),(38,41),(46,49),
                       (54,57),(62,65),(69,72),(77,80),(85,85)] for i in range(a,b+1)],
        [i for a,b in [(13,19),(27,34),(42,49),(58,65),(73,80)] for i in range(a,b+1)],
        list(range(20,35)) + list(range(50,66)) + list(range(81,86)),
        list(range(35,66)), list(range(66,86))]
    frames = []
    data = glo.strings()
    for p in (5, 1, 2, 3, 4):
        value = int.from_bytes(data[p], 'big') >> 11
        for i, group in enumerate(groups):
            value |= (sum((value >> (b-1)) & 1 for b in group) % 2) << i
        value |= (value.bit_count() % 2) << 7
        if fault:
            value ^= 3 << 20  # two-bit corruption, not correctable single bit
        packed = ((value << 11).to_bytes(12, 'big') + bytes(4))
        frames.append(frame(6, 1, 0, struct.unpack('>4I', packed), channel=0))
    return frames


class UbxNavigationSpecification(unittest.TestCase):
    def test_invalid_envelopes_cannot_seed_navigation_before_recovery(self):
        # Keep CRC valid: only the declared envelope is corrupt. A complete
        # invalid BDS ephemeris must not leak into the later valid GLO snapshot.
        invalid=[]
        for version,count_delta in ((0,0),(3,0),(255,0),(2,-1),(2,1)):
            for packet in beidou(1):
                p=bytearray(packet[6:-2]);p[6]=version;p[4]+=count_delta
                invalid.append(wire.ubx(2,0x13,p))
        for size in range(8):invalid.append(wire.ubx(2,0x13,bytes(size)))
        for mode in ('driver','decoder'):
            result=wire.run('ubx_envelope_'+mode,'ubx_'+mode+'_node',
                [anchor(ms=96006000),b''.join(invalid),*glonass()],glo.PARAMS,chunk_delay=.04)
            self.assertFalse(any(s.gnss_ephemeris for s in result['nav']))
            snaps=[s for s in result['nav'] if s.glonass_ephemeris]
            self.assertEqual(len(snaps),1)
            glo.GlonassRawSpecification.check(self,snaps[0].glonass_ephemeris[0],-7)

    def test_legacy_version1_reserved_header_layout(self):
        # U2 section32.18.7.1: protocol17 Time Sync, reserved bytes2/5/7.
        # No signal ID in this layout, so E5a F/NAV is not included.
        for mode in ('driver', 'decoder'):
            for kind, frames in [('bds',beidou(1)),('inav',galileo('inav')),
                                 ('glo',glonass())]:
                legacy=[]
                for packet in frames:
                    payload=bytearray(packet[6:-2])
                    # Reserved must be ignored, including values which mean
                    # F/NAV/B-CNAV in v2. U2 section32.18.7.1 has no sigId.
                    payload[2]=3 if kind=='inav' else 6
                    payload[5]=payload[7]=255
                    payload[6]=1
                    legacy.append(wire.ubx(2,0x13,payload))
                result=wire.run(f'ubx_v1_{mode}_{kind}','ubx_'+mode+'_node',
                    [anchor(ms=96006000)]+legacy,gal.PARAMS,chunk_delay=.04)
                snaps=[s for s in result['nav'] if s.gnss_ephemeris or s.glonass_ephemeris]
                self.assertEqual(len(snaps),1)
                if kind=='glo':glo.GlonassRawSpecification.check(self,snaps[0].glonass_ephemeris[0],-7)
                elif kind=='inav':gal.GalileoRawSpecification.check(self,snaps[0].gnss_ephemeris[0],inav_source=17)
                else:bds.BeidouRawSpecification.check(self,snaps[0].gnss_ephemeris[0],1)

    def test_bds_d1_d2_missing_then_complete(self):
        for mode in ('driver', 'decoder'):
            for model in (1, 2):
                result = wire.run(f'ubx_bds_{mode}_{model}', 'ubx_'+mode+'_node',
                    [anchor(ms=96006000)] + beidou(model, True) + beidou(model),
                    bds.PARAMS, chunk_delay=.04)
                snaps = [s for s in result['nav'] if s.gnss_ephemeris]
                self.assertEqual(len(snaps), 1)
                bds.BeidouRawSpecification.check(self, snaps[0].gnss_ephemeris[0], model)

    def test_galileo_inav_crc_fnav_missing_then_complete(self):
        for mode in ('driver', 'decoder'):
            for kind in ('inav', 'fnav'):
                result = wire.run(f'ubx_gal_{mode}_{kind}', 'ubx_'+mode+'_node',
                    [anchor(ms=96006000)] + galileo(kind, True) + galileo(kind),
                    gal.PARAMS, chunk_delay=.04)
                snaps = [s for s in result['nav'] if s.gnss_ephemeris]
                self.assertEqual(len(snaps), 1)
                gal.GalileoRawSpecification.check(self, snaps[0].gnss_ephemeris[0],
                                                  inav_source=17 if kind == 'inav' else None)

    def test_glonass_hamming_rejection_then_complete(self):
        for mode in ('driver', 'decoder'):
            result = wire.run('ubx_glo_'+mode, 'ubx_'+mode+'_node',
                [anchor(ms=96006000)] + glonass(True) + glonass(), glo.PARAMS, chunk_delay=.04)
            snaps = [s for s in result['nav'] if s.glonass_ephemeris]
            self.assertEqual(len(snaps), 1)
            glo.GlonassRawSpecification.check(self, snaps[0].glonass_ephemeris[0], -7)


if __name__ == '__main__':
    unittest.main(argv=[__file__] + wire.unittest_args)
