#!/usr/bin/env python3
"""U4 GPS.G6-SW-10018-F, section36.4 pp185-186: RXM-RAW.

The documented layout applies to u-blox6 raw-data variants FW6.00..7.03.
No payload-version byte exists. Reserved byte7 is not a version selector.
Only frame checksum and the ROS process harness are shared with other tests;
payloads and expectations are authored from the pinned U4 field table.
Satellite acceptance limits and non-finite/negative-value normalization are
repository policy, not a claim that U4 guarantees all tested PRNs in hardware.
"""
import math
import struct
import unittest
import test_receiver_streams as wire

PARAMS = ['-p', 'topics.observation:=/review/obs', '-p', 'use_gps_timestamp:=true']


def row(prn=1, quality=6, lli=0, p=21000000.125, l=-12345.25, d=-123.5, cn0=45):
    return struct.pack('<ddfBbbB', l, p, d, prn, quality, cn0, lli)


def frame(rows=None, ms=100500, week=2300, count=None, bad=False):
    if rows is None: rows = [row()]
    payload = struct.pack('<ihBB', ms, week, len(rows) if count is None else count, 0)
    return wire.ubx(2, 0x10, payload + b''.join(rows), bad=bad)


class LegacyRawSpecification(unittest.TestCase):
    def check_time(self, msg, week, tow):
        self.assertEqual(msg.week, week)
        self.assertAlmostEqual(msg.tow, tow, places=9)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.assertAlmostEqual(stamp, 315964800 + week * 604800 + tow - 18, delta=1e-6)

    def test_quality_thresholds_wire_lli_and_recovery(self):
        qualities = [-128, -1, 0, 3, 4, 5, 6, 127]
        rows = [row(i + 1, quality=q) for i, q in enumerate(qualities)]
        rows += [row(i + 9, lli=i) for i in range(8)]
        recovery = [row(i + 1, quality=6) for i in range(8)] + rows[8:]
        for mode in ('decoder', 'driver'):
            with self.subTest(mode=mode):
                result = wire.run('legacy_quality_' + mode, 'ubx_' + mode + '_node',
                                  [frame(rows), frame(recovery, ms=101500)], PARAMS, chunk_delay=.1)
                self.assertEqual(len(result['obs']), 2)
                for epoch, msg in enumerate(result['obs']):
                    self.check_time(msg, 2300, 100.5 + epoch)
                    self.assertEqual([o.prn for o in msg.observations], list(range(1, 17)))
                    for i, obs in enumerate(msg.observations):
                        with self.subTest(epoch=epoch, row=i):
                            q = qualities[i] if epoch == 0 and i < 8 else 6
                            self.assertEqual((obs.system, obs.code_str, obs.freq), ('G', '1C', 255))
                            if q < 4:
                                self.assertTrue(math.isnan(obs.p)); self.assertTrue(math.isnan(obs.d))
                            else:
                                self.assertEqual((obs.p, obs.d), (21000000.125, -123.5))
                            if q < 5: self.assertTrue(math.isnan(obs.l))
                            else: self.assertEqual(obs.l, -12345.25)
                            self.assertEqual(obs.snr, 45.)
                            # mesQI<6 says "likely" loss, not a replacement for
                            # the explicit RINEX LLI byte. Preserve that byte.
                            self.assertEqual(obs.lli, i - 8 if i >= 8 else 0)

    def test_signed_values_zero_nonfinite_and_satellite_identity(self):
        rows = [row(prn) for prn in (0, 1, 32, 33, 119, 120, 158, 159)]
        rows += [row(2, p=0., l=0., d=0., cn0=0),
                 row(3, p=-1., l=float('inf'), d=float('nan'), cn0=-128),
                 row(4, p=float('nan'), l=-.5, d=123.5, cn0=127)]
        for mode in ('decoder', 'driver'):
            with self.subTest(mode=mode):
                result = wire.run('legacy_values_' + mode, 'ubx_' + mode + '_node',
                                  [frame(rows), frame(ms=101500)], PARAMS, chunk_delay=.1)
                self.assertEqual(len(result['obs']), 2)
                msg = result['obs'][0]; self.check_time(msg, 2300, 100.5)
                values = {(o.system, o.prn): o for o in msg.observations}
                self.assertEqual(set(values), {('G', n) for n in (1, 2, 3, 4, 32)} | {('S', 120), ('S', 158)})
                self.assertEqual(len(msg.observations), len(values))
                for key in (('G', 1), ('G', 32), ('S', 120), ('S', 158)):
                    o = values[key]
                    self.assertEqual((o.p, o.l, o.d, o.snr, o.lli, o.freq),
                                     (21000000.125, -12345.25, -123.5, 45., 0, 255))
                o = values['G', 2]; self.assertEqual((o.p, o.l, o.d, o.snr), (0., 0., 0., 0.))
                o = values['G', 3]; self.assertTrue(all(math.isnan(v) for v in (o.p, o.l, o.d, o.snr)))
                o = values['G', 4]; self.assertTrue(math.isnan(o.p))
                self.assertEqual((o.l, o.d, o.snr), (-.5, 123.5, 127.))
                self.assertEqual(len(result['obs'][1].observations), 1)
                self.assertEqual(result['obs'][1].observations[0].p, 21000000.125)

    def test_signed_time_week_boundary_bad_counts_crc_and_recovery(self):
        frames = [frame(ms=604799999), frame(ms=0, week=2301)]
        frames += [frame(ms=1, week=w) for w in (0, -1, -32768)]
        frames += [frame(ms=t, week=2301) for t in (-1, 604800000, 2147483647)]
        frames += [frame(count=2, week=2301), frame(week=2301, bad=True),
                   wire.ubx(2, 0x10, bytes(7)), frame(ms=1, week=2301)]
        for mode in ('decoder', 'driver'):
            with self.subTest(mode=mode):
                result = wire.run('legacy_time_' + mode, 'ubx_' + mode + '_node',
                                  frames, PARAMS, chunk_delay=.03)
                self.assertEqual(len(result['obs']), 3)
                for msg, (week, tow) in zip(result['obs'], ((2300, 604799.999), (2301, 0.), (2301, .001))):
                    self.check_time(msg, week, tow)
                    self.assertEqual(len(msg.observations), 1)


if __name__ == '__main__': unittest.main(argv=[__file__] + wire.unittest_args)
