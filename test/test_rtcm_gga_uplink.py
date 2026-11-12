#!/usr/bin/env python3
"""rtcm_decoder_node sends the rover position as NMEA GGA to an NTRIP caster.

A fake NTRIP v1 caster checks that nothing is sent before the first valid
position, that a correct GGA then arrives once per period, and that the
decoder's observation output and source-byte relay still work on the
read-write session.
"""
import argparse
import base64
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
from rcl_interfaces.srv import GetParameters  # noqa: E402
from gnss_ros_standardization.msg import GnssObservations, GnssSolution  # noqa: E402

GGA_PERIOD_S = 5.0


def listener():
    sock = socket.socket()
    sock.bind(('127.0.0.1', 0))
    sock.listen(1)
    sock.settimeout(10)
    return sock


def port(sock): return sock.getsockname()[1]


def nmea_fields(line):
    """Fields of a checksum-verified NMEA sentence (without '$' and '*hh')."""
    assert line.startswith('$') and line[-3] == '*', line
    body = line[1:-3]
    checksum = 0
    for byte in body.encode():
        checksum ^= byte
    assert int(line[-2:], 16) == checksum, 'bad checksum: ' + line
    return body.split(',')


def degrees(value, hemisphere, width):
    result = int(value[:width]) + float(value[width:]) / 60.0
    return -result if hemisphere in 'SW' else result


class LineReader:
    def __init__(self, conn):
        self.conn, self.buffer = conn, b''

    def line(self, timeout):
        """Next CRLF line, or None if none arrives within `timeout` seconds."""
        deadline = time.monotonic() + timeout
        while b'\r\n' not in self.buffer:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self.conn.settimeout(remaining)
            try:
                part = self.conn.recv(4096)
            except socket.timeout:
                return None
            if not part:
                raise AssertionError('caster connection closed by the decoder')
            self.buffer += part
        line, self.buffer = self.buffer.split(b'\r\n', 1)
        return line.decode()


class GgaUplink(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.env = dict(os.environ, ROS_LOG_DIR=self.directory.name)

    def tearDown(self):
        self.directory.cleanup()

    def start(self, command):
        self.log = open(Path(self.directory.name) / 'node.log', 'w+')
        self.addCleanup(self.log.close)
        return subprocess.Popen(command, stdout=self.log, stderr=self.log, env=self.env)

    def stop(self, proc):
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
        try:
            return proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
            raise

    def node_log(self):
        self.log.flush()
        self.log.seek(0)
        return self.log.read()

    def test_gga_after_first_position_then_periodic_with_relay_and_decoding(self):
        fixture = Path(self.directory.name) / 'msm.rtcm3'
        subprocess.run([str(BIN / 'make_rtcm_fixture'), str(fixture)], check=True)
        rtcm = fixture.read_bytes()
        case = uuid.uuid4().hex[:8]
        solution_topic = '/gga_test_{}/solution'.format(case)
        observation_topic = '/gga_test_{}/observation'.format(case)

        context = Context()
        rclpy.init(context=context)
        node = rclpy.create_node('gga_test_' + case, context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        publisher = node.create_publisher(GnssSolution, solution_topic, 10)
        observations = []
        node.create_subscription(GnssObservations, observation_topic, observations.append, 10)

        with listener() as caster, listener() as relay:
            proc = self.start([
                str(BIN / 'rtcm_decoder_node'), '--ros-args',
                '-p', 'stream_path:=ntrip://user:secret@127.0.0.1:{}/MOUNT'.format(port(caster)),
                '-p', 'gga.solution_topic:=' + solution_topic,
                '-p', 'topics.observation:=' + observation_topic,
                '-p', 'rtcm_output.destination:=tcpcli://127.0.0.1:{}'.format(port(relay)),
                '-p', 'reference_week:=2300', '-p', 'reference_tow:=100.0'])
            try:
                conn = caster.accept()[0]
                self.addCleanup(conn.close)
                reader = LineReader(conn)
                request = []
                while True:
                    line = reader.line(5)
                    self.assertIsNotNone(line, 'incomplete NTRIP request')
                    if not line:
                        break
                    request.append(line)
                self.assertTrue(request[0].startswith('GET /MOUNT'), request)
                # NTRIP v1: the status line alone; anything after it is stream data.
                conn.sendall(b'ICY 200 OK\r\n')

                # No position yet: the caster must not receive anything.
                self.assertIsNone(reader.line(2.0))

                # Wait for the decoder to subscribe, then provide a position.
                deadline = time.monotonic() + 10
                while publisher.get_subscription_count() == 0:
                    self.assertLess(time.monotonic(), deadline, 'decoder never subscribed')
                    executor.spin_once(timeout_sec=0.05)
                solution = GnssSolution()
                solution.status = GnssSolution.STATUS_SINGLE
                solution.time_week, solution.time_tow = 2300, 100.0
                solution.num_sats = 12
                solution.latitude, solution.longitude, solution.altitude = 35.6812345, 139.7671234, 40.0
                published = time.monotonic()
                publisher.publish(solution)

                first = reader.line(GGA_PERIOD_S + 2)
                self.assertIsNotNone(first, 'no GGA after the first position\n' + self.node_log())
                first_time = time.monotonic()
                self.assertLess(first_time - published, 2.0)
                fields = nmea_fields(first)
                self.assertTrue(fields[0].endswith('GGA'), first)
                self.assertAlmostEqual(degrees(fields[2], fields[3], 2), 35.6812345, places=6)
                self.assertAlmostEqual(degrees(fields[4], fields[5], 3), 139.7671234, places=6)
                self.assertEqual(fields[6], '1')  # single
                self.assertEqual(int(fields[7]), 12)
                self.assertAlmostEqual(float(fields[9]) + float(fields[11]), 40.0, places=2)

                second = reader.line(GGA_PERIOD_S + 2)
                self.assertIsNotNone(second, 'GGA not repeated')
                interval = time.monotonic() - first_time
                self.assertGreater(interval, GGA_PERIOD_S - 0.5)
                self.assertLess(interval, GGA_PERIOD_S + 1.0)
                nmea_fields(second)

                # Corrections still decode and relay byte-identically.
                conn.sendall(rtcm)
                with relay.accept()[0] as relayed:
                    relayed.settimeout(5)
                    received = b''
                    while len(received) < len(rtcm):
                        part = relayed.recv(len(rtcm) - len(received))
                        self.assertTrue(part, 'relay closed early')
                        received += part
                    self.assertEqual(received, rtcm)
                deadline = time.monotonic() + 5
                while not observations and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.05)
                self.assertEqual([(m.week, m.tow) for m in observations], [(2300, 100.0)])
            except Exception:
                print(self.node_log())
                raise
            finally:
                code = self.stop(proc)
                executor.shutdown()
                node.destroy_node()
                context.shutdown()
            self.assertEqual(code, 0)

    def test_credentials_stay_out_of_parameters_and_logs(self):
        case = uuid.uuid4().hex[:8]
        node_name = 'envtest_' + case
        self.env.update(GRS_TEST_USER='canaryuser', GRS_TEST_PASS='canarypass')
        with listener() as caster:
            proc = self.start([
                str(BIN / 'rtcm_decoder_node'), '--ros-args', '-r', '__node:=' + node_name,
                '-p', 'stream_path:=ntrip://${GRS_TEST_USER}:${GRS_TEST_PASS}@127.0.0.1:%d/M' % port(caster)])
            context = Context()
            rclpy.init(context=context)
            node = rclpy.create_node('probe_' + case, context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            try:
                conn = caster.accept()[0]
                self.addCleanup(conn.close)
                conn.settimeout(5)
                request = b''
                while b'\r\n\r\n' not in request:
                    request += conn.recv(4096)
                # The caster gets the real credentials...
                self.assertIn(base64.b64encode(b'canaryuser:canarypass'), request)
                # ...and the ROS parameter only the variable names.
                client = node.create_client(GetParameters, '/%s/get_parameters' % node_name)
                self.assertTrue(client.wait_for_service(timeout_sec=10))
                future = client.call_async(GetParameters.Request(names=['stream_path']))
                executor.spin_until_future_complete(future, timeout_sec=5)
                self.assertTrue(future.done())
                value = future.result().values[0].string_value
                self.assertIn('${GRS_TEST_PASS}', value)
                self.assertNotIn('canary', value)
            finally:
                code = self.stop(proc)
                executor.shutdown()
                node.destroy_node()
                context.shutdown()
            self.assertEqual(code, 0)
            self.assertNotIn('canary', self.node_log())

    def test_unset_credential_variable_is_a_startup_error(self):
        self.env.pop('GRS_TEST_UNSET', None)
        proc = self.start([str(BIN / 'rtcm_decoder_node'), '--ros-args',
                           '-p', 'stream_path:=ntrip://u:${GRS_TEST_UNSET}@127.0.0.1:1/M'])
        try:
            code = proc.wait(timeout=10)
        finally:
            self.stop(proc)
        self.assertNotEqual(code, 0)
        self.assertIn('GRS_TEST_UNSET', self.node_log())

    def test_gga_requires_ntrip_stream(self):
        fixture = Path(self.directory.name) / 'input.rtcm3'
        fixture.write_bytes(b'')
        proc = self.start([str(BIN / 'rtcm_decoder_node'), '--ros-args',
                           '-p', 'stream_path:=file://' + str(fixture),
                           '-p', 'gga.solution_topic:=/gnss/nmea_solution'])
        try:
            code = proc.wait(timeout=10)
        finally:
            self.stop(proc)
        self.assertNotEqual(code, 0)
        self.assertIn('gga.solution_topic requires an ntrip:// stream_path', self.node_log())


if __name__ == '__main__':
    unittest.main(argv=[__file__] + remaining)
