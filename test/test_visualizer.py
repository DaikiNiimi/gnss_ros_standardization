#!/usr/bin/env python3
"""gnss_visualizer: HTTP behaviour and the live stream, against the real node."""
import argparse
import json
import math
import os
import pathlib
import signal
import socket
import subprocess
import sys
import time
import unittest

parser = argparse.ArgumentParser()
parser.add_argument('--bin-dir', required=True)
args, _ = parser.parse_known_args()
BIN = pathlib.Path(args.bin_dir).resolve()
os.environ['LD_LIBRARY_PATH'] = str(BIN) + os.pathsep + os.environ.get('LD_LIBRARY_PATH', '')
sys.path.insert(0, str(BIN / 'rosidl_generator_py'))

import rclpy  # noqa: E402
import rclpy.qos  # noqa: E402
from gnss_ros_standardization.msg import GnssEphemerides, GnssEphemeris, GnssObservation, GnssObservations, GnssSolution  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402

PREFIX = f'/visualizer_test_{os.getpid()}'
OBS_TOPIC, SOL_TOPIC = f'{PREFIX}/observation', f'{PREFIX}/solution'
OTHER_SOL_TOPIC, EPH_TOPIC = f'{PREFIX}/other/solution', f'{PREFIX}/ephemeris'
IMU_TOPIC, OTHER_IMU_TOPIC = f'{PREFIX}/imu/data_raw', f'{PREFIX}/other/imu'


def reject_constant(name):
    raise ValueError(f'{name} is not valid JSON')


def free_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def solution(east, north, status=GnssSolution.STATUS_FIX, llh_valid=True):
    """A fix `east`/`north` metres from a fixed point. pos_enu is left at zero on purpose."""
    a, e2, lat0 = 6378137.0, 6.69437999014e-3, math.radians(35.0)
    w = 1.0 - e2 * math.sin(lat0) ** 2
    meridional, normal = a * (1.0 - e2) / w ** 1.5, a / math.sqrt(w)
    message = GnssSolution()
    message.status = status
    message.time_week = 2300
    message.time_tow = 100.0
    message.latitude = 35.0 + math.degrees(north / meridional) if llh_valid else math.nan
    message.longitude = 137.0 + math.degrees(east / (normal * math.cos(lat0))) if llh_valid else math.nan
    message.altitude = 50.0 if llh_valid else math.nan
    message.pos_ecef.x = message.pos_ecef.y = message.pos_ecef.z = math.nan
    return message


def observation(count=1):
    message = GnssObservations()
    message.week = 2300
    message.tow = 100.0
    for prn in range(1, count + 1):
        o = GnssObservation()
        o.sat = 250  # the backend number must not identify the satellite
        o.satid = f'G{prn:02d}'
        o.system = 'G'
        o.prn = prn
        o.code = 1
        o.code_str = '1C'
        o.snr = 45.0
        message.observations.append(o)
    return message


def ephemerides(count):
    """Satellites spread along one orbit plane, so some are above any horizon."""
    message = GnssEphemerides()
    for prn in range(1, count + 1):
        e = GnssEphemeris()
        e.system, e.prn, e.satid, e.week = 'G', prn, f'G{prn:02d}', 2300
        e.toe = e.toc = e.toes = 100.0
        e.ttr = 0.0
        e.a, e.e, e.i0, e.omg0 = 26559700.0, 0.001, 0.96, 2.4
        e.m0 = 2.0 * math.pi * prn / count
        e.tgd = [0.0]
        message.gnss_ephemeris.append(e)
    return message


def imu_sample(stamp_ns):
    message = Imu()
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    message.linear_acceleration.z = 9.80665
    return message


class Stream:
    """One Server-Sent Events connection."""

    def __init__(self, port):
        self.sock = socket.create_connection(('127.0.0.1', port), timeout=0.2)
        self.sock.sendall(f'GET /events HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\n\r\n'.encode())
        self.buffer = b''

    def frames(self):
        try:
            data = self.sock.recv(65536)
            if data:
                self.buffer += data
        except socket.timeout:
            pass
        out = []
        while b'\n\n' in self.buffer:
            frame, self.buffer = self.buffer.split(b'\n\n', 1)
            text = frame.decode()
            if text.startswith('HTTP/') or 'event: ' not in text:
                text = text[text.find('event: '):] if 'event: ' in text else ''
            if text:
                event = text.split('\n')[0][len('event: '):]
                out.append((event, json.loads(text.split('data: ', 1)[1], parse_constant=reject_constant)))
        return out

    def close(self):
        self.sock.close()


class Visualizer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.port = free_port()
        cls.process = subprocess.Popen(
            [str(BIN / 'gnss_visualizer'), '--ros-args', '-p', f'port:={cls.port}',
             '-p', f'topics.observation:={OBS_TOPIC}', '-p', f'topics.ephemeris:={EPH_TOPIC}',
             '-p', f'topics.solution:={SOL_TOPIC}', '-p', f'topics.imu:={IMU_TOPIC}'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        deadline = time.monotonic() + 20.0
        while True:
            try:
                socket.create_connection(('127.0.0.1', cls.port), timeout=0.5).close()
                break
            except OSError:
                if time.monotonic() > deadline or cls.process.poll() is not None:
                    raise RuntimeError('gnss_visualizer did not start') from None
                time.sleep(0.1)
        rclpy.init()
        cls.node = rclpy.create_node('visualizer_test_publisher')
        cls.obs_pub = cls.node.create_publisher(GnssObservations, OBS_TOPIC, 10)
        cls.sol_pub = cls.node.create_publisher(GnssSolution, SOL_TOPIC, 10)
        cls.other_sol_pub = cls.node.create_publisher(GnssSolution, OTHER_SOL_TOPIC, 10)
        cls.imu_pub = cls.node.create_publisher(Imu, IMU_TOPIC, 1000)
        cls.other_imu_pub = cls.node.create_publisher(Imu, OTHER_IMU_TOPIC, 1000)
        cls.eph_pub = cls.node.create_publisher(
            GnssEphemerides, EPH_TOPIC,
            rclpy.qos.QoSProfile(depth=256, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        cls.process.send_signal(signal.SIGINT)
        try:
            code = cls.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            cls.process.terminate()
            cls.process.wait(timeout=10)
            raise
        assert code == 0, code

    def request(self, path='/', host=None, method='GET'):
        with socket.create_connection(('127.0.0.1', self.port), timeout=3) as s:
            s.sendall(f'{method} {path} HTTP/1.1\r\nHost: {host or f"127.0.0.1:{self.port}"}\r\n\r\n'.encode())
            data = b''
            while True:
                chunk = s.recv(65536)
                if not chunk:
                    break
                data += chunk
        head, _, body = data.partition(b'\r\n\r\n')
        return head.decode(), body.decode()

    def wait_for(self, stream, publish, condition, timeout=15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            publish()
            for event, data in stream.frames():
                if condition(event, data):
                    return data
        self.fail('expected event did not arrive')

    def test_page_and_status_codes(self):
        head, body = self.request('/')
        self.assertIn('200 OK', head)
        self.assertIn('text/html', head)
        self.assertIn('EventSource("/events")', body)
        for external in ('http://', 'https://'):
            self.assertNotIn(external, body)
        self.assertIn('404', self.request('/missing')[0].split('\r\n')[0])
        self.assertIn('405', self.request(method='POST')[0].split('\r\n')[0])

    def test_loopback_server_refuses_foreign_hosts(self):
        self.assertIn('403', self.request(host='evil.example')[0].split('\r\n')[0])
        self.assertIn('200', self.request(host=f'localhost:{self.port}')[0].split('\r\n')[0])

    def test_stream_carries_points_and_satellites(self):
        stream = Stream(self.port)
        self.addCleanup(stream.close)
        first = self.wait_for(stream, lambda: None, lambda event, _: event == 'init')
        self.assertIn('tracks', first)
        base = self.wait_for(stream, lambda: self.sol_pub.publish(solution(0.0, 0.0)),
                             lambda event, data: event == 'update' and data['points'][0])['points'][0][-1]
        target = [None]

        def moved(event, data):
            if event != 'update':
                return False
            for point in data['points'][0]:
                if abs(point[0] - base[0]) > 1.0:
                    target[0] = point
                    return True
            return False

        update = self.wait_for(stream, lambda: self.sol_pub.publish(solution(12.5, -3.25)), moved)
        self.assertEqual(update['status']['status'], 'FIX')
        self.assertAlmostEqual(target[0][0] - base[0], 12.5, delta=0.01)
        self.assertAlmostEqual(target[0][1] - base[1], -3.25, delta=0.01)
        with_satellite = self.wait_for(stream, lambda: self.obs_pub.publish(observation()),
                                       lambda event, data: event == 'update' and data['sats']['list'])
        satellite = with_satellite['sats']['list'][0]
        self.assertEqual(satellite[0], 'G01')
        self.assertEqual(satellite[1], 'G')
        self.assertEqual(satellite[4], 45.0)

        late = Stream(self.port)
        self.addCleanup(late.close)
        history = self.wait_for(late, lambda: None, lambda event, _: event == 'init')
        self.assertIn(target[0], history['tracks'][0])

    def post_select(self, slot, topic, headers=None, port=None, address='127.0.0.1'):
        port = port or self.port
        host = f'{address}:{port}'
        extra = {'X-Gnss-Visualizer': 'select', 'Origin': f'http://{host}'} if headers is None else headers
        body = f'{slot}\n{topic}'
        head = f'POST /select HTTP/1.1\r\nHost: {host}\r\nContent-Length: {len(body)}\r\n'
        head += ''.join(f'{k}: {v}\r\n' for k, v in extra.items()) + '\r\n'
        with socket.create_connection((address, port), timeout=3) as s:
            s.sendall((head + body).encode())
            return s.recv(200).decode().split('\r\n')[0]

    def topics_message(self):
        """Topics are looked up on connecting, so reconnect until discovery has settled."""
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            stream = Stream(self.port)
            self.addCleanup(stream.close)
            message = self.wait_for(stream, lambda: None, lambda event, _: event == 'topics')
            if OTHER_SOL_TOPIC in message['solutions'] and OTHER_IMU_TOPIC in message['imus']:
                return message
            time.sleep(0.2)
        self.fail('topics were not discovered')

    def test_topics_are_listed_by_type_when_a_page_connects(self):
        message = self.topics_message()
        self.assertIn(OBS_TOPIC, message['observations'])
        self.assertIn(SOL_TOPIC, message['solutions'])
        self.assertNotIn(EPH_TOPIC, message['solutions'])
        self.assertTrue(message['can_select'])
        self.assertIn(IMU_TOPIC, message['imus'])
        self.assertNotIn(SOL_TOPIC, message['imus'])
        self.assertEqual(message['selected'], {'obs': OBS_TOPIC, 'sol': SOL_TOPIC, 'sol2': '', 'imu': IMU_TOPIC})

    def publish_imu(self, publisher, seconds):
        """Samples at 100 Hz, stamped from one moving time."""
        deadline = time.monotonic() + 10.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertGreater(publisher.get_subscription_count(), 0)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            publisher.publish(imu_sample(time.time_ns()))
            time.sleep(0.01)

    def test_imu_history_is_kept_without_a_page_and_analysed(self):
        self.addCleanup(lambda: self.post_select('imu', IMU_TOPIC))
        self.publish_imu(self.imu_pub, 1.5)
        stream = Stream(self.port)
        self.addCleanup(stream.close)
        bins = self.wait_for(stream, lambda: None, lambda event, _: event == 'imu_init')['bins']
        filled = [b for b in bins if b['n'] > 0]
        self.assertGreater(len(filled), 3)
        self.assertAlmostEqual(filled[-1]['a'][2][0], 9.80665, delta=0.001)
        self.assertEqual([b['seq'] for b in bins], sorted({b['seq'] for b in bins}))
        update = self.wait_for(stream, lambda: self.imu_pub.publish(imu_sample(time.time_ns())),
                               lambda event, data: event == 'update' and data['imu']['bin']
                               and data['imu']['bin']['n'] > 0)
        self.assertEqual(update['imu']['status']['up'], '+Z')
        self.assertIsNotNone(update['imu']['status']['rate'])

    def test_imu_selection_replaces_the_history(self):
        self.topics_message()
        self.addCleanup(lambda: self.post_select('imu', IMU_TOPIC))
        self.publish_imu(self.imu_pub, 0.6)
        stream = Stream(self.port)
        self.addCleanup(stream.close)
        self.wait_for(stream, lambda: None, lambda event, _: event == 'imu_init')
        self.assertIn('204', self.post_select('imu', OTHER_IMU_TOPIC))
        replaced = {}

        def changed(event, data):
            if event == 'selection' and data['imu'] == OTHER_IMU_TOPIC:
                replaced['selection'] = True
            if event == 'imu_init' and replaced.get('selection'):
                replaced['bins'] = data['bins']
                return True
            return False

        self.wait_for(stream, lambda: None, changed)
        self.assertFalse([b for b in replaced['bins'] if b['n'] > 0])
        self.publish_imu(self.other_imu_pub, 0.6)
        self.assertEqual(self.topics_message()['selected']['imu'], OTHER_IMU_TOPIC)
        self.assertIn('204', self.post_select('imu', 'not_a_topic'))
        time.sleep(0.5)
        self.assertEqual(self.topics_message()['selected']['imu'], OTHER_IMU_TOPIC)

    def test_selection_needs_a_page_of_this_server(self):
        self.topics_message()
        self.assertIn('403', self.post_select('sol2', OTHER_SOL_TOPIC, headers={}))
        self.assertIn('403', self.post_select('sol2', OTHER_SOL_TOPIC, headers={'X-Gnss-Visualizer': 'select',
                                                                               'Origin': 'http://evil.example'}))
        self.assertIn('403', self.post_select('sol2', OTHER_SOL_TOPIC, headers={'Origin': f'http://127.0.0.1:{self.port}'}))
        self.assertEqual(self.topics_message()['selected']['sol2'], '')

    def test_second_solution_is_tracked_separately(self):
        self.topics_message()
        stream = Stream(self.port)
        self.addCleanup(stream.close)
        self.addCleanup(lambda: self.post_select('sol2', ''))
        self.assertIn('204', self.post_select('sol2', OTHER_SOL_TOPIC))
        self.wait_for(stream, lambda: None, lambda event, data: event == 'selection' and data['sol2'] == OTHER_SOL_TOPIC)
        self.wait_for(stream, lambda: self.other_sol_pub.publish(solution(1.0, 1.0)),
                      lambda event, data: event == 'update' and data['points'][1])
        self.assertIn('204', self.post_select('sol2', 'not_a_topic'))
        time.sleep(0.5)
        self.assertEqual(self.topics_message()['selected']['sol2'], OTHER_SOL_TOPIC)

    def test_ephemerides_survive_selection_changes_and_page_reloads(self):
        self.topics_message()
        count = 12

        def has_angle(event, data):
            return event == 'update' and any(s[2] is not None for s in data['sats']['list'])

        def feed():
            self.obs_pub.publish(observation(count))
            self.sol_pub.publish(solution(0.0, 0.0))

        def feed_other():
            self.obs_pub.publish(observation(count))
            self.other_sol_pub.publish(solution(0.0, 0.0))

        first = Stream(self.port)
        self.addCleanup(first.close)
        self.eph_pub.publish(ephemerides(count))  # published once only
        self.wait_for(first, feed, has_angle)

        self.addCleanup(lambda: self.post_select('sol', SOL_TOPIC))
        self.assertIn('204', self.post_select('sol', OTHER_SOL_TOPIC))
        self.wait_for(first, feed_other, has_angle)

        reloaded = Stream(self.port)
        self.addCleanup(reloaded.close)
        self.wait_for(reloaded, feed_other, has_angle)

    def test_unusable_solutions_are_not_drawn(self):
        stream = Stream(self.port)
        self.addCleanup(stream.close)
        self.wait_for(stream, lambda: None, lambda event, _: event == 'init')

        def publish():
            self.sol_pub.publish(solution(math.nan, math.nan, llh_valid=False))
            self.sol_pub.publish(solution(7.0, 8.0, GnssSolution.STATUS_NONE))

        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            publish()
            for event, data in stream.frames():
                if event == 'update':
                    self.assertEqual(data['points'], [[], []])


class RemoteAccess(unittest.TestCase):
    def test_other_devices_can_view_but_not_select(self):
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect(('10.255.255.255', 1))
            address = probe.getsockname()[0]
        except OSError:
            self.skipTest('no non-loopback address')
        finally:
            probe.close()
        if address.startswith('127.'):
            self.skipTest('no non-loopback address')
        port = free_port()
        process = subprocess.Popen([str(BIN / 'gnss_visualizer'), '--ros-args', '-p', f'port:={port}', '-p', 'bind_address:=0.0.0.0',
                                    '-r', '__node:=remote_test'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.addCleanup(process.wait, 10)
        self.addCleanup(process.send_signal, signal.SIGINT)
        deadline = time.monotonic() + 20.0
        while True:
            try:
                sock = socket.create_connection((address, port), timeout=0.5)
                break
            except OSError:
                self.assertLess(time.monotonic(), deadline)
                time.sleep(0.1)
        self.addCleanup(sock.close)
        sock.settimeout(5)
        sock.sendall(f'GET /events HTTP/1.1\r\nHost: {address}:{port}\r\n\r\n'.encode())
        data = b''
        while b'event: topics' not in data or b'can_select' not in data:
            data += sock.recv(65536)
        self.assertIn(b'"can_select":false', data)
        body = 'sol2\n'
        request = (f'POST /select HTTP/1.1\r\nHost: {address}:{port}\r\nContent-Length: {len(body)}\r\n'
                   f'X-Gnss-Visualizer: select\r\nOrigin: http://{address}:{port}\r\n\r\n{body}')
        with socket.create_connection((address, port), timeout=3) as post:
            post.sendall(request.encode())
            self.assertIn('403', post.recv(200).decode().split('\r\n')[0])


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]])
