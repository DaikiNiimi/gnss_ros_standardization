#!/usr/bin/env python3
"""R2-derived MSM integer extrema and GPST/BDT/GLONASS time boundaries.

Reference: rtcm-rs d4d0269dc76b7e88b6620ca023b82889b6441662,
src/df/dfs.rs DF034/397..408/416/419/420 and msg1074..1137 layouts.
Authored wire integers and dimensional expectations never use RTKLIB encoding
or decoded results. These selected cases do not certify all RTCM revisions.
Week 2300 uses GPST-UTC=18 s; leap-second insertion itself is not tested.
"""
import math
import unittest

import test_receiver_streams as wire
from test_decoder_boundaries import rtcm_payload

WEEK_MS = 604800000
DAY_MS = 86400000
FAMILIES = [(1070, 'G01', 2), (1080, 'R01', 2), (1090, 'E01', 2),
            (1100, '120', 2), (1110, 'J01', 2), (1120, 'C01', 2), (1130, 'I01', 22)]


def wire_epoch(base, gpst_ms):
    if base == 1080:
        # GLONASS UTC(SU)+3 h: a separate 3-bit day and 27-bit TOD.
        day, tod = divmod((gpst_ms + 10800000 - 18000) % WEEK_MS, DAY_MS)
        return (day << 27) | tod
    return (gpst_ms - (14000 if base == 1120 else 0)) % WEEK_MS


def frame(variant, epoch, *, base=1070, signal=2, **overrides):
    extended = variant in (6, 7)
    with_rate = variant in (5, 7)
    pr_width, cp_width = (20, 24) if extended else (15, 22)
    values = dict(rough=70, modulo=512, rate=-100, channel=7, pr=-16,
                  cp=1024, lock=10, half=1, cnr=720 if extended else 45, fine_rate=-1234)
    assert set(overrides) <= set(values)
    values.update(overrides)
    fields = []

    def put(value, width, signed=False):
        low = -(1 << (width - 1)) if signed else 0
        high = (1 << (width - int(signed))) - 1
        assert low <= value <= high, (value, width, signed)
        fields.append(format(value & ((1 << width) - 1), f'0{width}b'))

    for value, width in [(base + variant, 12), (42, 12), (epoch, 30),
                         (0, 1), (1, 3), (0, 7), (0, 2), (0, 2), (0, 1), (0, 3),
                         (1 << 63, 64), (1 << (32 - signal), 32), (1, 1)]:
        put(value, width)
    put(values['rough'], 8)
    if with_rate:
        put(values['channel'] if base == 1080 else 0, 4)
    put(values['modulo'], 10)
    if with_rate:
        put(values['rate'], 14, True)
    for name, width, signed in [('pr', pr_width, True), ('cp', cp_width, True),
                                ('lock', 10 if extended else 4, False), ('half', 1, False),
                                ('cnr', 10 if extended else 6, False)]:
        put(values[name], width, signed)
    if with_rate:
        put(values['fine_rate'], 15, True)
    bits = ''.join(fields)
    bits += '0' * (-len(bits) % 8)
    return rtcm_payload(int(bits, 2).to_bytes(len(bits) // 8, 'big')), values


def parameters(tow):
    return ['-p', 'reference_week:=2300', '-p', f'reference_tow:={float(tow)}',
            '-p', 'use_gps_timestamp:=true']


class MsmBoundaries(unittest.TestCase):
    def check_stamp(self, msg, absolute_ms):
        week_offset, tow_ms = divmod(absolute_ms, WEEK_MS)
        self.assertEqual(msg.week, 2300 + week_offset)
        self.assertAlmostEqual(msg.tow, tow_ms / 1000., delta=1e-7)
        # Independent integer Unix epoch calculation, no backend time helper.
        expected_ns = (315964800 + 2300 * 604800 - 18) * 1000000000 + absolute_ms * 1000000
        actual_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        self.assertLessEqual(abs(actual_ns - expected_ns), 1000)

    def test_signed_extrema_independent_sentinels_and_recovery(self):
        for variant in (4, 5, 6, 7):
            extended = variant in (6, 7)
            pr_min = -(1 << (19 if extended else 14))
            cp_min = -(1 << (23 if extended else 21))
            cases = [
                dict(rough=254, modulo=1023, pr=-pr_min-1, cp=-cp_min-1,
                     rate=8191, fine_rate=16383, cnr=1023 if extended else 63,
                     lock=1023 if extended else 15),
                dict(pr=pr_min+1, cp=cp_min+1, rate=-8191, fine_rate=-16383, cnr=1, lock=0, half=0),
                dict(pr=pr_min), dict(cp=cp_min), dict(rate=-8192), dict(fine_rate=-16384),
                dict(rough=255),
                dict(rough=0, modulo=0, pr=0, cp=0, rate=0, fine_rate=0, cnr=0),
                {},
            ]
            fixtures = [frame(variant, 100000 + i * 1000, **case) for i, case in enumerate(cases)]
            result = wire.run(f'msm_extrema_{variant}', 'rtcm_decoder_node',
                              [f[0] for f in fixtures], parameters(100), chunk_delay=.03)
            self.assertEqual(len(result['obs']), len(cases))
            for i, (msg, (_, row)) in enumerate(zip(result['obs'], fixtures)):
                with self.subTest(variant=variant, case=i):
                    self.check_stamp(msg, 100000 + i * 1000)
                    self.assertEqual(len(msg.observations), 1)
                    obs = msg.observations[0]
                    self.assertEqual((obs.satid, obs.code_str, obs.freq), ('G01', '1C', 255))
                    rough = row['rough'] + row['modulo'] * 2**-10
                    expected = {
                        'p': math.nan if row['rough'] == 255 or row['pr'] == pr_min else
                             (rough + row['pr'] * 2**(-29 if extended else -24)) * 299792.458,
                        'l': math.nan if row['rough'] == 255 or row['cp'] == cp_min else
                             (rough + row['cp'] * 2**(-31 if extended else -29)) * 1575420.,
                        # Rate is independent of rough pseudorange validity.
                        'd': math.nan if variant in (4, 6) or row['rate'] == -8192 or row['fine_rate'] == -16384 else
                             -(row['rate'] + row['fine_rate'] * .0001) * 1575420000. / 299792458.,
                        'snr': row['cnr'] * (.0625 if extended else 1) if row['cnr'] else math.nan,
                    }
                    for name, value in expected.items():
                        actual = getattr(obs, name)
                        if math.isnan(value):
                            self.assertTrue(math.isnan(actual), (name, actual))
                        else:
                            # Backend D is float32 (ROS D is float64); P/L retain
                            # float64. Bounds fixed before execution.
                            tolerance = max(1e-5, abs(value) * 1e-7) if name == 'd' else 1e-6
                            self.assertAlmostEqual(actual, value, delta=tolerance, msg=name)
                    self.assertEqual(obs.lli & 2, row['half'] * 2)

    def test_gpst_week_rollover_all_constellations_and_invalid_epochs(self):
        times = [WEEK_MS - 1, WEEK_MS, WEEK_MS + 1, WEEK_MS + 1000]
        for base, satid, signal in FAMILIES:
            for variant in (4, 5, 6, 7):
                with self.subTest(base=base, variant=variant):
                    frames = []
                    expected = list(times)
                    # MSM4/6 cannot transmit an FCN: establish it in a prior MSM5 epoch.
                    if base == 1080 and variant in (4, 6):
                        frames.append(frame(5, wire_epoch(base, WEEK_MS - 1000), base=base)[0])
                        expected.insert(0, WEEK_MS - 1000)
                    frames.append(frame(variant, wire_epoch(base, times[0]), base=base, signal=signal)[0])
                    invalid = [(7 << 27), DAY_MS, (6 << 27) | ((1 << 27) - 1)] if base == 1080 else [WEEK_MS, (1 << 30) - 1]
                    frames.extend(frame(variant, epoch, base=base, signal=signal)[0] for epoch in invalid)
                    frames.extend(frame(variant, wire_epoch(base, t), base=base, signal=signal)[0] for t in times[1:])
                    result = wire.run(f'msm_week_{base}_{variant}', 'rtcm_decoder_node', frames,
                                      parameters(604798), chunk_delay=.03)
                    self.assertEqual(len(result['obs']), len(expected))
                    previous_ns = None
                    for msg, t in zip(result['obs'], expected):
                        self.check_stamp(msg, t)
                        self.assertEqual(len(msg.observations), 1)
                        obs = msg.observations[0]
                        self.assertEqual(obs.satid, satid)
                        self.assertTrue(math.isfinite(obs.p) and math.isfinite(obs.l))
                        stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
                        if previous_ns is not None:
                            self.assertGreater(stamp, previous_ns)
                        previous_ns = stamp

    def test_bdt_week_and_glonass_day_rollover(self):
        # BDT week starts at GPST Sunday 00:00:14. GLO midnight falls at
        # GPST 21:00:18; select the Saturday->Sunday GLO day-of-week rollover.
        for base, edge, signal in [(1120, 14000, 2), (1080, WEEK_MS - 10782000, 2)]:
            for variant in (4, 5, 6, 7):
                with self.subTest(base=base, variant=variant):
                    times = [edge - 1000, edge - 1, edge, edge + 1, edge + 1000]
                    frames = [frame(5 if base == 1080 and variant in (4, 6) and i == 0 else variant,
                                    wire_epoch(base, t), base=base, signal=signal)[0] for i, t in enumerate(times)]
                    result = wire.run(f'msm_system_day_{base}_{variant}', 'rtcm_decoder_node', frames,
                                      parameters((edge - 2000) / 1000), chunk_delay=.03)
                    self.assertEqual(len(result['obs']), len(times))
                    for msg, t in zip(result['obs'], times):
                        self.check_stamp(msg, t)
                        self.assertEqual(len(msg.observations), 1)
                        self.assertEqual(msg.observations[0].satid, 'R01' if base == 1080 else 'C01')


if __name__ == '__main__':
    unittest.main(argv=[__file__] + wire.unittest_args)
