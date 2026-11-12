#!/usr/bin/env python3
"""rtcm_decoder_node publishes one GnssBaseStation record per station configuration.

The fixture switches from station 42 (1006 first) to station 77, whose 1005
arrives only after two of its epochs. Every epoch must still be published, and
each record must say from which epoch it applies and whether its position is
known yet.
"""
import argparse
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import unittest
import uuid

parser = argparse.ArgumentParser()
parser.add_argument('--bin-dir', required=True)
args, remaining = parser.parse_known_args()
BIN = Path(args.bin_dir).resolve()
os.environ.setdefault('ROS_DOMAIN_ID', str(100 + os.getpid() % 100))
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['LD_LIBRARY_PATH'] = str(BIN) + os.pathsep + os.environ.get('LD_LIBRARY_PATH', '')
sys.path.insert(0, str(BIN / 'rosidl_generator_py'))
import rclpy  # noqa: E402
from rclpy.context import Context  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from gnss_ros_standardization.msg import GnssBaseStation, GnssObservations  # noqa: E402

STATION_A = (-3961904.9, 3348993.8, 3698211.7)
STATION_B = (-3962904.9, 3347993.8, 3698711.7)


class BaseStationRecords(unittest.TestCase):
    def test_records_follow_station_changes_without_withholding_observations(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture = Path(directory) / 'stations.rtcm3'
            subprocess.run([str(BIN / 'make_rtcm_fixture'), str(fixture), 'stations'], check=True)
            case = uuid.uuid4().hex[:8]
            topics = {'obs': '/station_test_{}/observation'.format(case),
                      'station': '/station_test_{}/station'.format(case)}

            context = Context()
            rclpy.init(context=context)
            node = rclpy.create_node('station_test_' + case, context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            observations, records = [], []
            node.create_subscription(GnssObservations, topics['obs'], observations.append, 50)
            latched = QoSProfile(depth=8, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            node.create_subscription(GnssBaseStation, topics['station'], records.append, latched)

            server = socket.socket()
            server.bind(('127.0.0.1', 0))
            server.listen(1)
            server.settimeout(10)
            log = open(Path(directory) / 'node.log', 'w+')
            proc = subprocess.Popen(
                [str(BIN / 'rtcm_decoder_node'), '--ros-args',
                 '-p', 'stream_path:=tcpcli://127.0.0.1:{}'.format(server.getsockname()[1]),
                 '-p', 'topics.observation:=' + topics['obs'],
                 '-p', 'topics.station:=' + topics['station'],
                 '-p', 'reference_week:=2300', '-p', 'reference_tow:=100.0'],
                stdout=log, stderr=log, env=dict(os.environ, ROS_LOG_DIR=directory))
            try:
                conn = server.accept()[0]
                deadline = time.monotonic() + 10
                while not all(node.count_publishers(t) for t in topics.values()):
                    self.assertLess(time.monotonic(), deadline, 'decoder topics not discovered')
                    executor.spin_once(timeout_sec=0.05)
                end = time.monotonic() + 1.0  # let the decoder discover us too
                while time.monotonic() < end:
                    executor.spin_once(timeout_sec=0.05)
                conn.sendall(fixture.read_bytes())
                deadline = time.monotonic() + 10
                while len(observations) < 6 or len(records) < 4:
                    if time.monotonic() > deadline:
                        break
                    executor.spin_once(timeout_sec=0.05)
                end = time.monotonic() + 0.5  # a spurious republication would land here
                while time.monotonic() < end:
                    executor.spin_once(timeout_sec=0.05)

                # Every epoch is published, including those of 77 before its 1005.
                self.assertEqual([m.tow for m in observations], [100., 101., 102., 103., 104., 105.])

                summary = [(r.station_id, r.valid_from_week, r.valid_from_tow, r.position_valid)
                           for r in records]
                self.assertEqual(summary, [(42, 2300, 100., True), (77, 2300, 102., False),
                                           (77, 2300, 102., True), (77, 2300, 102., True)])
                a, pending, b, described = records
                def assert_position(record, expected):  # RTCM resolution 0.1 mm
                    for value, reference in zip((record.arp_ecef.x, record.arp_ecef.y,
                                                 record.arp_ecef.z), expected):
                        self.assertAlmostEqual(value, reference, delta=1e-4)
                assert_position(a, STATION_A)
                self.assertAlmostEqual(a.antenna_height, 1.5, places=4)
                self.assertTrue(math.isnan(pending.antenna_height))
                assert_position(b, STATION_B)
                self.assertTrue(math.isnan(b.antenna_height))  # 1005 carries no height
                self.assertEqual(b.antenna_descriptor, '')
                self.assertEqual((described.antenna_descriptor, described.antenna_setup_id,
                                  described.antenna_serial, described.receiver_type,
                                  described.receiver_firmware, described.receiver_serial),
                                 ('TRM59800.00     NONE', 3, 'A1', 'TRIMBLE NETR9', '5.45', 'R1'))
                self.assertTrue(all(math.isnan(v) for v in described.glonass_code_phase_bias))
            except Exception:
                log.flush()
                log.seek(0)
                print(log.read())
                raise
            finally:
                if proc.poll() is None:
                    proc.send_signal(signal.SIGINT)
                code = proc.wait(timeout=10)
                log.close()
                server.close()
                executor.shutdown()
                node.destroy_node()
                context.shutdown()
            self.assertEqual(code, 0)


if __name__ == '__main__':
    unittest.main(argv=[__file__] + remaining)
