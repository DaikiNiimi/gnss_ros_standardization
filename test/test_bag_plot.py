#!/usr/bin/env python3
"""rtk_bag_plot executable against synthetic bags; no receiver or external data."""
import argparse
import base64
import json
import os
import pathlib
import re
import struct
import subprocess
import sys
import tempfile
import unittest

import rosbag2_py
from rclpy.serialization import serialize_message

parser = argparse.ArgumentParser()
parser.add_argument('--bin-dir', required=True)
args, _ = parser.parse_known_args()
BIN = pathlib.Path(args.bin_dir).resolve()
os.environ['LD_LIBRARY_PATH'] = str(BIN) + os.pathsep + os.environ.get('LD_LIBRARY_PATH', '')
sys.path.insert(0, str(BIN / 'rosidl_generator_py'))
from gnss_ros_standardization.msg import GnssEphemerides, GnssObservation, GnssObservations, GnssSolution  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402

TOOL = BIN / 'rtk_bag_plot'
OBS_TYPE = 'gnss_ros_standardization/msg/GnssObservations'
SOL_TYPE = 'gnss_ros_standardization/msg/GnssSolution'
EPH_TYPE = 'gnss_ros_standardization/msg/GnssEphemerides'
IMU_TYPE = 'sensor_msgs/msg/Imu'
WEEK, TOW = 2300, 100000.0
FORMATS = {'f64': 'd', 'f32': 'f', 'u32': 'I', 'u16': 'H', 'i16': 'h', 'u8': 'B'}


def topic_metadata(**fields):
    try:
        return rosbag2_py.TopicMetadata(id=0, **fields)
    except TypeError:
        return rosbag2_py.TopicMetadata(**fields)


def write_bag(path, messages, compressed=False):
    """messages: (topic, type, message, seconds)."""
    if compressed:
        writer = rosbag2_py.SequentialCompressionWriter(
            rosbag2_py.CompressionOptions('zstd', rosbag2_py.CompressionMode.FILE))
    else:
        writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    seen = set()
    for topic, type_name, message, seconds in messages:
        if topic not in seen:
            writer.create_topic(topic_metadata(name=topic, type=type_name, serialization_format='cdr'))
            seen.add(topic)
        writer.write(topic, serialize_message(message), int(seconds * 1e9))
    del writer


def observation_message(tow, signals):
    message = GnssObservations()
    message.week = WEEK
    message.tow = tow
    for prn, code, snr, lli in signals:
        o = GnssObservation()
        o.system = 'G'
        o.prn = prn
        o.satid = f'G{prn:02d}'
        o.code_str = code
        o.p = 2.1e7
        o.l = 1.1e8
        o.d = 100.0
        o.snr = snr
        o.lli = lli
        o.freq = 255
        message.observations.append(o)
    return message


def solution_message(tow, status=GnssSolution.STATUS_FIX, north=0.0):
    message = GnssSolution()
    message.time_week = WEEK
    message.time_tow = tow
    message.status = status
    message.num_sats = 12
    message.ratio = 5.0
    message.latitude = 35.0 + north / 111000.0
    message.longitude = 137.0
    message.altitude = 50.0
    message.pos_ecef.x = float('nan')
    message.pos_ecef.y = float('nan')
    message.pos_ecef.z = float('nan')
    return message


def basic_messages():
    messages = []
    for i in range(5):
        tow = TOW + i
        signals = [(1, '1C', 40.0 + i, 1 if i == 2 else 0), (1, '2W', 30.0, 0), (5, '1C', 45.0, 0)]
        messages.append(('/gnss/observation', OBS_TYPE, observation_message(tow, signals), 1000.0 + i))
        messages.append(('/gnss/solution', SOL_TYPE, solution_message(tow, north=float(i)), 1000.0 + i))
        messages.append(('/other/solution', SOL_TYPE, solution_message(tow, GnssSolution.STATUS_FLOAT, north=float(i) + 1.0), 1000.0 + i))
    return messages


class Report:
    def __init__(self, path):
        html = pathlib.Path(path).read_text()
        self.html = html
        self.manifest = json.loads(re.search(r'id="manifest">(.*?)</script>', html, re.S).group(1))
        self.parts = {}
        for name, data in re.findall(r'data-column="([^"]+)" data-part="\d+">(.*?)</script>', html, re.S):
            self.parts[name] = self.parts.get(name, b'') + base64.b64decode(data)

    def column(self, name):
        meta = next(c for c in self.manifest['columns'] if c['name'] == name)
        return struct.unpack(f'<{meta["count"]}{FORMATS[meta["type"]]}', self.parts[name])


class BagPlot(unittest.TestCase):
    def run_tool(self, *arguments):
        return subprocess.run([str(TOOL), *map(str, arguments)], capture_output=True, text=True, timeout=120)

    def make_report(self, messages, compressed=False, extra=()):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        root = pathlib.Path(directory.name)
        bag = root / 'bag'
        write_bag(bag, messages, compressed)
        out = root / 'report.html'
        result = self.run_tool(bag, '--out', out, '--no-open', *extra)
        self.assertEqual(result.returncode, 0, result.stderr)
        return Report(out), root, bag

    def test_signal_rows_and_flags(self):
        report, _, _ = self.make_report(basic_messages())
        observation = report.manifest['observations'][0]
        self.assertEqual(observation['topic'], '/gnss/observation')
        self.assertEqual(report.column(observation['t']), (0.0, 1.0, 2.0, 3.0, 4.0))
        signals = {(observation['sats'][s['sat']]['id'], s['code']): s for s in observation['signals']}
        self.assertEqual(set(signals), {('G01', '1C'), ('G01', '2W'), ('G05', '1C')})
        s = signals[('G01', '1C')]
        self.assertEqual(s['band'], 0)
        offset, count = s['off'], s['n']
        self.assertEqual(count, 5)
        snr = report.column(observation['snr'])[offset:offset + count]
        self.assertEqual(snr, (160, 164, 168, 172, 176))
        flags = report.column(observation['flags'])[offset:offset + count]
        self.assertTrue(flags[2] & 1)
        self.assertFalse(flags[1] & 1)
        self.assertTrue(all(f & 4 and f & 8 and f & 16 for f in flags))

    def test_no_ephemeris_disables_geometry_but_keeps_observations(self):
        report, _, _ = self.make_report(basic_messages())
        observation = report.manifest['observations'][0]
        self.assertFalse(observation['geometry'])
        self.assertEqual(observation['dops'], [])
        self.assertTrue(any('ephemeris' in note for note in report.manifest['notes']))
        self.assertEqual(set(report.column(observation['el'])), {0xffff})

    def test_solutions_share_an_origin_and_are_compared(self):
        report, _, _ = self.make_report(basic_messages())
        solutions = report.manifest['solutions']
        self.assertEqual(len(solutions), 2)
        by_topic = {s['topic']: s for s in solutions}
        self.assertEqual(by_topic['/gnss/solution']['status_counts'][1], 5)
        self.assertEqual(by_topic['/other/solution']['status_counts'][2], 5)
        difference = report.manifest['differences'][0]
        north = report.column(difference['series']['y'])
        self.assertEqual(len(north), 5)
        self.assertTrue(all(abs(abs(v) - 1.0) < 0.05 for v in north), north)
        self.assertEqual(set(report.column(difference['series']['q'])), {2})

    def test_solution_only_bag(self):
        messages = [('/gnss/solution', SOL_TYPE, solution_message(TOW + i, north=float(i)), 1000.0 + i)
                    for i in range(3)]
        report, _, _ = self.make_report(messages)
        self.assertEqual(report.manifest['observations'], [])
        self.assertEqual(len(report.manifest['solutions']), 1)

    def test_imu_only_bag_uses_bag_time(self):
        messages = []
        for i in range(4):
            imu = Imu()
            imu.linear_acceleration.z = 9.8 + i
            messages.append(('/gnss/imu/data_raw', IMU_TYPE, imu, 2000.0 + i))
        report, _, _ = self.make_report(messages)
        self.assertEqual(report.manifest['time_basis'], 'bag')
        imu = report.manifest['imus'][0]
        self.assertEqual(report.column(imu['t']), (0.0, 1.0, 2.0, 3.0))
        self.assertAlmostEqual(report.column(imu['az_max'])[3], 12.8, places=4)

    def test_topic_overview_reports_gaps_and_order(self):
        messages = []
        for i in [0, 1, 2, 3, 3, 10, 11, 2]:
            signals = [(1, '1C', 40.0, 0)]
            messages.append(('/gnss/observation', OBS_TYPE, observation_message(TOW + i, signals), 1000.0 + len(messages)))
        report, _, _ = self.make_report(messages)
        topic = report.manifest['topics'][0]
        self.assertEqual(topic['count'], 8)
        self.assertEqual(topic['gap_count'], 1)
        self.assertAlmostEqual(topic['max_gap'], 7.0)
        self.assertEqual(topic['out_of_order'], 1)
        self.assertEqual(topic['duplicates'], 2)
        self.assertEqual(len(report.column(report.manifest['observations'][0]['t'])), 6)

    def test_event_driven_topics_have_no_gaps(self):
        messages = []
        for i, t in enumerate([0.0, 1.0, 2.0, 500.0]):
            messages.append(('/gnss/ephemeris', EPH_TYPE, GnssEphemerides(), 1000.0 + t))
        messages += basic_messages()
        report, _, _ = self.make_report(messages)
        topic = next(t for t in report.manifest['topics'] if t['name'] == '/gnss/ephemeris')
        self.assertEqual(topic['gap_count'], 0)

    def test_residuals_from_stat_file(self):
        with tempfile.TemporaryDirectory() as directory:
            stat = pathlib.Path(directory) / 'solution.stat'
            lines = []
            for i in range(3):
                lines.append(f'$SAT,{WEEK},{TOW + i:.3f},G01,1,120.0,45.0,0.5000,0.0100,1,42,1,0,10,0,0,0')
                lines.append(f'$SAT,{WEEK},{TOW + i:.3f},G05,1,200.0,30.0,0.0000,0.0000,0,38,1,0,10,0,0,0')
            stat.write_text('\n'.join(lines) + '\n')
            report, _, _ = self.make_report(basic_messages(), extra=('--stat', stat))
        residuals = report.manifest['residuals']
        self.assertEqual(residuals['total'], 6)
        self.assertEqual(sorted(residuals['sats']), ['G01', 'G05'])
        self.assertEqual(report.column(residuals['t'])[:2], (0.0, 0.0))
        self.assertAlmostEqual(report.column(residuals['resp'])[0], 0.5, places=4)
        self.assertAlmostEqual(report.column(residuals['el'])[0], 45.0, places=3)

    def test_compressed_bag_matches_plain_and_leaves_no_files(self):
        plain, _, _ = self.make_report(basic_messages())
        compressed, root, bag = self.make_report(basic_messages(), compressed=True)
        self.assertEqual(plain.parts, compressed.parts)
        self.assertEqual(sorted(p.name for p in bag.iterdir()),
                         sorted(['metadata.yaml', *[p.name for p in bag.iterdir() if p.name.endswith('.zstd')]]))
        self.assertTrue(any(p.name.endswith('.zstd') for p in bag.iterdir()))
        self.assertEqual(sorted(p.name for p in root.iterdir()), ['bag', 'report.html'])

    def test_comparison_bag_is_matched_by_gpst(self):
        with tempfile.TemporaryDirectory() as directory:
            other = pathlib.Path(directory) / 'fgo_run'
            # The other bag has its own recording time and even the same topic name.
            write_bag(other, [('/gnss/solution', SOL_TYPE, solution_message(TOW + i, north=float(i) + 3.0), 5000.0 + i)
                              for i in range(5)])
            report, _, _ = self.make_report(basic_messages(), extra=('--compare', other))
        by_topic = {s['topic']: i for i, s in enumerate(report.manifest['solutions'])}
        self.assertEqual(report.manifest['solutions'][by_topic['fgo_run:/gnss/solution']]['source'], 'compare')
        pair = next(d for d in report.manifest['differences']
                    if {d['a'], d['b']} == {by_topic['/gnss/solution'], by_topic['fgo_run:/gnss/solution']})
        north = report.column(pair['series']['y'])
        self.assertEqual(len(north), 5)
        self.assertTrue(all(abs(abs(v) - 3.0) < 0.05 for v in north), north)
        self.assertEqual(len(report.manifest['topics']), 3)

    def test_pos_file_is_compared_as_a_solution(self):
        with tempfile.TemporaryDirectory() as directory:
            pos = pathlib.Path(directory) / 'reference.pos'
            lines = ['%  GPST latitude(deg) longitude(deg) height(m) Q ns']
            for i in range(5):
                lat = 35.0 + (float(i) + 2.0) / 111000.0
                lines.append(f'{WEEK} {TOW + i:.3f} {lat:.9f} 137.000000000 50.0000 1 20')
            pos.write_text('\n'.join(lines) + '\n')
            report, _, _ = self.make_report(basic_messages(), extra=('--pos', pos))
        by_topic = {s['topic']: i for i, s in enumerate(report.manifest['solutions'])}
        self.assertEqual(report.manifest['solutions'][by_topic['pos:reference.pos']]['source'], 'pos')
        pair = next(d for d in report.manifest['differences']
                    if {d['a'], d['b']} == {by_topic['/gnss/solution'], by_topic['pos:reference.pos']})
        north = report.column(pair['series']['y'])
        self.assertEqual(len(north), 5)
        self.assertTrue(all(abs(abs(v) - 2.0) < 0.05 for v in north), north)

    def test_html_is_self_contained(self):
        report, _, _ = self.make_report(basic_messages())
        html = report.html
        script = html[html.rindex('<script>'):]
        for forbidden in ('http://', 'https://', 'fetch(', 'XMLHttpRequest', 'import('):
            self.assertNotIn(forbidden, script, forbidden)
        self.assertEqual(html.count('id="manifest"'), 1)
        self.assertNotIn('RTK_BAG_PLOT_DATA', html)
        for name, payload in re.findall(r'data-column="([^"]+)" data-part="\d+">(.*?)</script>', html, re.S):
            self.assertRegex(payload, r'^[A-Za-z0-9+/=]*$', name)

    def test_topic_names_are_html_safe(self):
        messages = [('/gnss/solution', SOL_TYPE, solution_message(TOW), 1000.0)]
        report, _, _ = self.make_report(messages)
        self.assertNotIn('</script><script', report.html.split('id="manifest"')[1].split('</script>')[0])

    def test_exit_codes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            self.assertEqual(self.run_tool().returncode, 2)
            self.assertEqual(self.run_tool('--unknown').returncode, 2)
            self.assertEqual(self.run_tool('--out').returncode, 2)
            self.assertEqual(self.run_tool('a', 'b').returncode, 2)
            self.assertEqual(self.run_tool(root / 'missing', '--no-open').returncode, 1)
            bag = root / 'bag'
            write_bag(bag, basic_messages())
            result = self.run_tool(bag, '--stat', root / 'missing.stat', '--no-open', '--out', root / 'r.html')
            self.assertEqual(result.returncode, 1)
            result = self.run_tool(bag, '--pos', root / 'missing.pos', '--no-open', '--out', root / 'r.html')
            self.assertEqual(result.returncode, 1)
            result = self.run_tool(bag, '--compare', root / 'missing', '--no-open', '--out', root / 'r.html')
            self.assertEqual(result.returncode, 1)
            self.assertFalse((root / 'r.html').exists())
            self.assertEqual(self.run_tool('--help').returncode, 0)
            version = self.run_tool('--version')
            self.assertEqual(version.returncode, 0)
            self.assertIn('rtk_bag_plot', version.stdout)

    def test_default_output_is_next_to_the_bag(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            write_bag(root / 'my_bag', basic_messages())
            result = self.run_tool(root / 'my_bag', '--no-open')
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue((root / 'my_bag.html').exists())
            self.assertEqual(pathlib.Path(result.stdout.strip()), root / 'my_bag.html')


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]])
