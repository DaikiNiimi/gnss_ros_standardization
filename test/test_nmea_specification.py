#!/usr/bin/env python3
"""U2 sections31.2.4/.11/.12/.14/.19, pp147-158/162: selected NMEA fields.

Receiver-manual evidence, not normative NMEA certification. WGS84 is an explicit
fixture assumption; talker identifiers do not establish a datum on real hardware.
"""
import math
import unittest
import test_receiver_streams as wire


def sentence(body, bad=False):
    checksum=0
    for ch in body.encode():checksum^=ch
    return f'${body}*{checksum ^ int(bad):02X}\r\n'.encode()


def epoch(second, invalid_velocity=False, zero_cov=False):
    t=f'0000{second:02d}.25'
    lines=[f'GNGGA,{t},0000.000,N,00000.000,E,1,12,1.25,93.25,M,30.0,M,,',
           f'GNRMC,{t},{"V" if invalid_velocity else "A"},0000.000,N,00000.000,E,2.0,90.0,040224,,,A,V',
           f'GNGST,{t},1.0,4.0,2.0,0.0,'+('0,0,0' if zero_cov else '2,3,4'),
           f'GNZDA,{t},04,02,2024,00,00']
    # Timestamp-less GSA is a repository freshness policy, not an epoch time.
    gsa=','.join(['GNGSA','A','3']+['01']+['']*11+['2.5','1.25','3.5'])
    return sentence(gsa)+b''.join(sentence(s) for s in lines)


class NmeaSpecification(unittest.TestCase):
    def test_units_axes_date_and_corrupt_sentence_recovery_all_receiver_paths(self):
        for family in ('ubx','sbf','novatel'):
            for mode in ('driver','decoder'):
                with self.subTest(family=family,mode=mode):
                    # Start the documented repository one-second source grace,
                    # then wait using the input harness's normal chunk pacing.
                    frames=[sentence('GNZDA,000000.25,04,02,2024,00,00'),b'',b'',b'']
                    for second in (1,2,3):
                        frames.append(sentence(f'GNGGA,0000{second:02d}.25,9000,N,00000,E,1,12,1,999,M,0,M,,',True)+epoch(second))
                    r=wire.run('nmea_spec_'+family+'_'+mode,family+'_'+mode+'_node',frames,
                               ['-p','use_gps_timestamp:=true','-p','local_origin.mode:=fixed']+
                               (['-p','messages.nav_pvt:=false'] if family=='ubx' and mode=='driver' else []),chunk_delay=.4)
                    self.assertEqual([m.time_tow for m in r['sol']],[19.25,20.25,21.25])
                    for m in r['sol']:
                        self.assertEqual((m.time_week,m.solution_source,m.status,m.num_sats),(2300,2,5,12))
                        self.assertEqual((m.latitude,m.longitude,m.altitude),(0.,0.,123.25))
                        self.assertAlmostEqual(m.pos_ecef.x,6378260.25,delta=1e-6)
                        self.assertAlmostEqual(m.vel_enu.x,2*1852/3600,delta=1e-6)
                        self.assertAlmostEqual(m.vel_enu.y,0.,delta=1e-12)
                        self.assertEqual(list(m.pos_enu_cov),[9.,0.,0.,0.,4.,0.,0.,0.,16.])
                        self.assertEqual((m.pdop,m.hdop,m.vdop),(2.5,1.25,3.5))
                        self.assertTrue(math.isnan(m.gdop))
                        self.assertTrue(all(math.isnan(v) for v in m.vel_enu_cov))
                        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
                        self.assertAlmostEqual(stamp,315964800+2300*604800+m.time_tow-18,delta=1e-6)

    def test_void_velocity_and_real_zero_uncertainty_are_distinct(self):
        for mode in ('driver','decoder'):
            r=wire.run('nmea_void_'+mode,'ubx_'+mode+'_node',
                [sentence('GNZDA,000000.25,04,02,2024,00,00'),b'',b'',b'',
                 epoch(1,True,True),epoch(2),epoch(3)],
                ['-p','use_gps_timestamp:=true','-p','local_origin.mode:=fixed']+
                (['-p','messages.nav_pvt:=false'] if mode=='driver' else []),chunk_delay=.4)
            self.assertEqual([m.time_tow for m in r['sol']],[19.25,20.25,21.25])
            self.assertTrue(math.isnan(r['sol'][0].vel_enu.x))
            self.assertEqual(list(r['sol'][0].pos_enu_cov),[0.]*9)
            self.assertAlmostEqual(r['sol'][1].vel_enu.x,2*1852/3600,delta=1e-6)


if __name__=='__main__':
    unittest.main(argv=[__file__]+wire.unittest_args)
