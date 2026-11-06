#!/usr/bin/env python3
"""Black-box intake/output contract of the installed gnss_fgo and gnss_imu_fgo.

Each case starts the node, feeds hand-made messages and checks the published
solutions, the debug trace (base selected per epoch) and the exit code:
  - malformed or out-of-range input is counted and dropped; the node lives on
    (invalid week/tow, duplicate/late rover, unsupported PRN, duplicate signal
    row, bad ephemeris, non-finite / reversed / no-data / bad-stamp IMU);
  - a rover clock step back of more than 5 s starts a new timeline;
  - an exact rover/base pair is decided at once; base(T) arriving after the
    IMU has passed T is still used (no IMU-crossing race);
  - with IMU stalled, an epoch is released once a rover epoch 0.2 s later has
    arrived (stream time), not held forever; after an executor stall the IMU
    already received covers the epoch (A5-3);
  - every epoch without a solution is STATUS_NONE with NaN fields, frame
    gnss_link; SIGINT ends with 0, a bad configuration with FATAL and 1.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from gnss_ros_standardization.msg import (GnssEphemerides, GnssEphemeris,
                                          GnssObservation, GnssObservations,
                                          GnssSolution)
from sensor_msgs.msg import Imu
from std_msgs.msg import String

WEEK = 2301
BASE = '[6378137.0,0.0,0.0]'


def spin_until(node, predicate, reason, timeout=10.0):
    until = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= until:
            raise AssertionError(reason)
        rclpy.spin_once(node, timeout_sec=0.01)


def spin_for(node, seconds):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=0.01)


def observation(tow, rows=(), week=WEEK, basis='gpst'):
    m = GnssObservations()
    m.week = week
    m.tow = float(tow)
    # GPST-scale header (legacy-bag convention); the IMU stamps below match.
    unix = week * 604800 + tow + 315964800 - (18 if basis == 'utc' else 0)
    m.header.stamp.sec = int(math.floor(unix))
    m.header.stamp.nanosec = int(round((unix - math.floor(unix)) * 1e9))
    for system, prn, code in rows:
        r = GnssObservation()
        r.system, r.prn, r.code_str = system, prn, code
        r.p, r.l, r.d, r.snr = 2.2e7, 1.1e8, -100.0, 45.0
        m.observations.append(r)
    return m


def imu(t_gpst, stamp_basis='gpst', finite=True, no_data=False, nanosec=None):
    m = Imu()
    unix = t_gpst + 315964800 - (18 if stamp_basis == 'utc' else 0)
    m.header.stamp.sec = int(math.floor(unix))
    m.header.stamp.nanosec = (int(round((unix - math.floor(unix)) * 1e9))
                              if nanosec is None else nanosec)
    m.linear_acceleration.z = 9.81 if finite else float('nan')
    if no_data:
        m.angular_velocity_covariance[0] = -1.0
    return m


def gpst(tow):
    return WEEK * 604800 + tow


class Case:
    def __init__(self, node, executable, imu_node, temp, name, extra=()):
        self.node = node
        self.imu_node = imu_node
        self.prefix = '/contract_{}_{}'.format(name, os.getpid())
        self.trace = Path(temp) / name
        self.rows = []
        qos = QoSProfile(depth=64)
        self.rover = node.create_publisher(GnssObservations, self.prefix + '/rover', qos)
        self.base = node.create_publisher(GnssObservations, self.prefix + '/base', qos)
        self.nav = node.create_publisher(
            GnssEphemerides, self.prefix + '/nav',
            QoSProfile(depth=256, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.end = node.create_publisher(String, self.prefix + '/end', qos)
        self.imu = node.create_publisher(Imu, self.prefix + '/imu', 1000) if imu_node else None
        self.sub = node.create_subscription(GnssSolution, self.prefix + '/solution',
                                            self.rows.append, 64)
        params = {'base_position.pos': BASE,
                  'topics.rover_observation': self.prefix + '/rover',
                  'topics.base_observation': self.prefix + '/base',
                  'topics.ephemeris': self.prefix + '/nav',
                  'topics.solution': self.prefix + '/solution',
                  'debug.end_topic': self.prefix + '/end',
                  'debug.trace_directory': str(self.trace)}
        if imu_node:
            params['topics.imu'] = self.prefix + '/imu'
        command = [executable, '--ros-args']
        for k, v in params.items():
            command += ['-p', '{}:={}'.format(k, v)]
        command += list(extra)
        self.log = open(Path(temp) / (name + '.log'), 'w')
        self.proc = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT)
        pubs = [self.rover, self.base, self.nav, self.end] + ([self.imu] if imu_node else [])
        spin_until(node, lambda: all(p.get_subscription_count() for p in pubs)
                   and node.count_publishers(self.prefix + '/solution'), 'discovery')
        # Writer-side matching can precede reader-side matching; data sent in
        # between is dropped by the reader. Give the node time to match back.
        spin_for(node, 1.0)

    def alive(self):
        assert self.proc.poll() is None, 'node exited with {}'.format(self.proc.returncode)

    def imu_until(self, t_from, t_to, step=0.01):
        n = int(round((t_to - t_from) / step))
        for i in range(n + 1):
            self.imu.publish(imu(gpst(t_from + i * step)))

    def wait_rows(self, n, reason):
        spin_until(self.node, lambda: len(self.rows) >= n, reason)

    def selected(self):
        out = {}
        for line in (self.trace / 'intake-trace.jsonl').read_text().splitlines():
            e = json.loads(line)
            if e['event'] == 'base_selected':
                out[e['rover_ms']] = e['base_ms']
        return out

    def finish(self, rows=None):
        self.end.publish(String(data='end'))
        if rows is not None:
            self.wait_rows(rows, 'outputs before the end marker were not all published')
        self.proc.wait(timeout=20)
        self.log.close()
        return self.proc.returncode

    def close(self):
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.proc.terminate()
                self.proc.wait(timeout=10)
        if not self.log.closed:
            self.log.close()
        for e in [self.rover, self.base, self.nav, self.end] + ([self.imu] if self.imu else []):
            self.node.destroy_publisher(e)
        self.node.destroy_subscription(self.sub)


def check_none(m):
    assert m.status == m.STATUS_NONE, m.status
    assert m.header.stamp.sec == int(m.time_week * 604800 + m.time_tow + 315964800), 'stamp not inherited'
    assert m.header.frame_id == 'gnss_link', m.header.frame_id
    assert m.num_sats == 0
    for v in [m.latitude, m.pos_ecef.x, m.pos_enu.x, m.vel_ecef.x, m.vel_enu.x,
              m.pos_cov_ecef[0], m.pos_enu_cov[4], m.vel_cov_ecef[8], m.gdop]:
        assert math.isnan(v), v


def malformed_input(node, exe, imu_node, temp):
    c = Case(node, exe, imu_node, temp, 'malformed_' + ('imu' if imu_node else 'gnss'))
    try:
        bad = GnssEphemerides()
        e = GnssEphemeris()
        e.system, e.prn, e.week, e.toe = 'G', 1, 0, 0.0   # week 0: invalid
        bad.gnss_ephemeris.append(e)
        e2 = GnssEphemeris()
        e2.system, e2.prn, e2.week, e2.toe = 'G', 40, WEEK, 0.0   # PRN out of range
        bad.gnss_ephemeris.append(e2)
        c.nav.publish(bad)
        if imu_node:
            c.imu.publish(imu(gpst(99.0), finite=False))
            c.imu.publish(imu(gpst(99.01), no_data=True))
            c.imu.publish(imu(gpst(99.02), nanosec=1000000001))
            c.imu_until(99.5, 105.0)
            c.imu.publish(imu(gpst(102.0)))            # does not advance
        c.base.publish(observation(105))               # only base: 5 s ahead of rover 100
        spin_for(node, 0.2)
        c.rover.publish(observation(5, week=0))        # invalid time
        c.rover.publish(observation(100, [('G', 33, '1C'), ('J', 183, '1C'),
                                          ('G', 1, '1C'), ('G', 1, '1C')]))
        c.wait_rows(1, 'malformed: rover 100 not published')
        c.rover.publish(observation(99.8))             # older than a decided epoch
        c.rover.publish(observation(104))
        c.rover.publish(observation(104))              # duplicate of a pending epoch
        c.rover.publish(observation(104.5))
        c.wait_rows(2, 'malformed: rover 104 not published')
        c.alive()
        # Clock step back by more than 5 s: new timeline, node keeps going.
        if imu_node:
            c.imu_until(49.5, 51.5)
        c.rover.publish(observation(50))
        c.rover.publish(observation(51))
        c.wait_rows(4, 'timeline reset: epochs not published')
        c.alive()
        assert c.finish(5) == 0
        assert [round(m.time_tow, 3) for m in c.rows] == [100.0, 104.0, 104.5, 50.0, 51.0], \
            [m.time_tow for m in c.rows]
        for m in c.rows:
            check_none(m)
        log = (Path(temp) / ('malformed_' + ('imu' if imu_node else 'gnss') + '.log')).read_text()
        for needle in ['new timeline', 'rover 8 (invalid time 1, duplicate 1, late 1',
                       'unsupported satellite 2, duplicate signal 1',
                       'invalid records 2']:
            assert needle in log, (needle, log)
        if imu_node:
            assert 'invalid 3, not advancing 1' in log, log
    finally:
        c.close()


def exact_pair_and_late_base(node, exe, temp):
    """IMU node: exact pair at once; base(T) after the IMU passed T is used."""
    c = Case(node, exe, True, temp, 'late_base')
    try:
        c.base.publish(observation(198))
        c.base.publish(observation(199))
        spin_for(node, 0.3)
        c.imu_until(198.5, 199.6)
        c.rover.publish(observation(199.4))   # aged base 199, cadence 1 s: at once
        c.wait_rows(1, 'cadence-proven aged pair not released')
        c.imu_until(199.61, 200.2)
        c.rover.publish(observation(200))     # IMU already past 200, base(200) not yet
        spin_for(node, 0.5)
        assert len(c.rows) == 1, 'epoch 200 released before its base could arrive'
        c.base.publish(observation(200))
        c.wait_rows(2, 'exact pair not released when base(T) arrived')
        assert c.finish(2) == 0
        sel = c.selected()
        assert sel[gpst(199.4) * 1000] == gpst(199) * 1000, sel
        assert sel[gpst(200) * 1000] == gpst(200) * 1000, sel
    finally:
        c.close()


def imu_stall(node, exe, temp):
    """IMU node: a stalled IMU delays an epoch by one later rover epoch only."""
    c = Case(node, exe, True, temp, 'imu_stall')
    try:
        c.imu_until(299.0, 299.9)             # IMU stops before 300
        c.base.publish(observation(300))
        c.rover.publish(observation(300))     # exact pair, IMU does not cover it
        spin_for(node, 0.5)
        assert len(c.rows) == 0, 'epoch released before IMU covered it or the wait ended'
        c.rover.publish(observation(300.2))   # 0.2 s later in stream time
        c.wait_rows(1, 'IMU stall held the epoch beyond the stream-time wait')
        assert round(c.rows[0].time_tow, 3) == 300.0
        assert c.finish(2) == 0
        log = (Path(temp) / 'imu_stall.log').read_text()
        assert 'epochs without IMU coverage 2' in log, log
    finally:
        c.close()


def executor_backlog(node, exe, temp):
    """IMU node (A5-3): after an executor stall the queued messages are taken
    one per subscription per cycle, so a later rover epoch is taken before the
    IMU that was published ahead of it. The IMU already received must cover
    the epoch: no epoch is solved without IMU coverage."""
    c = Case(node, exe, True, temp, 'executor_backlog')
    try:
        c.imu_until(299.0, 299.9)
        spin_for(node, 0.3)
        c.proc.send_signal(signal.SIGSTOP)    # executor (and DDS) stalled
        try:
            c.imu_until(299.91, 301.0)
            for tow in (300.0, 300.2, 300.4, 300.6, 300.8):
                c.base.publish(observation(tow))
                c.rover.publish(observation(tow))
            spin_for(node, 0.3)
        finally:
            c.proc.send_signal(signal.SIGCONT)
        c.wait_rows(4, 'epochs queued during the stall were not solved')
        assert c.finish(5) == 0
        log = (Path(temp) / 'executor_backlog.log').read_text()
        assert 'epochs without IMU coverage 0' in log, log
    finally:
        c.close()


def utc_headers(node, exe, temp):
    """IMU node: UTC header stamps are recognised from the rover; IMU is placed
    in GPS time with the leap seconds, so the epoch is covered at once."""
    c = Case(node, exe, True, temp, 'utc_headers')
    try:
        for i in range(201):
            c.imu.publish(imu(gpst(399.5 + i * 0.01), stamp_basis='utc'))
        c.base.publish(observation(400, basis='utc'))
        spin_for(node, 0.2)
        c.rover.publish(observation(400, basis='utc'))
        c.wait_rows(1, 'UTC-stamped IMU did not cover the epoch (time scale mixed up)')
        m = c.rows[0]
        assert m.header.stamp.sec == int(gpst(400)) + 315964800 - 18, m.header.stamp.sec
        assert c.finish(1) == 0
        log = (Path(temp) / 'utc_headers.log').read_text()
        assert 'Header stamps are UTC' in log and 'epochs without IMU coverage 0' in log, log
    finally:
        c.close()


def sigint_with_pending(node, exe, temp):
    """IMU node: an epoch still waiting for IMU at Ctrl-C is reported, not lost
    silently (rover count = solutions + drops + not solved at shutdown)."""
    c = Case(node, exe, True, temp, 'sigint_pending')
    try:
        c.imu_until(499.0, 499.9)
        c.base.publish(observation(500))
        c.rover.publish(observation(500))     # IMU never covers it, no later rover
        spin_for(node, 0.5)
        assert len(c.rows) == 0
        c.proc.send_signal(signal.SIGINT)
        c.proc.wait(timeout=10)
        assert c.proc.returncode == 0, c.proc.returncode
        c.log.close()
        log = (Path(temp) / 'sigint_pending.log').read_text()
        assert 'rover 1 (' in log and 'solutions 0' in log, log
        assert 'not solved at shutdown 1 (waiting 1, queued 0)' in log, log
    finally:
        c.close()


def base_first_after_loop(node, exe, imu_node, temp):
    """A loop seam where the base clock steps back first (base arrives before
    the rover of the new timeline): the base starts the new timeline instead of
    being dropped as late, and pairs with the rover epoch."""
    name = 'base_loop_' + ('imu' if imu_node else 'gnss')
    c = Case(node, exe, imu_node, temp, name)
    try:
        if imu_node:
            c.imu_until(299.5, 300.2)
        c.base.publish(observation(300))
        spin_for(node, 0.2)
        c.rover.publish(observation(300))
        c.wait_rows(1, 'first epoch not published')
        c.base.publish(observation(200))               # new timeline, base first
        spin_for(node, 0.2)
        if imu_node:
            c.imu_until(199.5, 200.2)
        c.rover.publish(observation(200))
        c.wait_rows(2, 'epoch of the new timeline not published')
        assert c.finish(2) == 0
        sel = c.selected()
        assert sel[gpst(200) * 1000] == gpst(200) * 1000, sel
        log = (Path(temp) / (name + '.log')).read_text()
        assert 'late 0)' in log and 'timeline resets 1' in log, log
    finally:
        c.close()


def sigint_and_startup(node, exe, imu_node, temp):
    c = Case(node, exe, imu_node, temp, 'sigint_' + ('imu' if imu_node else 'gnss'))
    try:
        c.proc.send_signal(signal.SIGINT)
        c.proc.wait(timeout=10)
        assert c.proc.returncode == 0, c.proc.returncode
    finally:
        c.close()
    env = dict(os.environ)
    for bad in [['-p', 'queue_depth:=0'], ['-p', 'max_age_s:=-1.0'],
                ['-p', 'base_position.pos:=[0.0,0.0,0.0]'],
                ['-p', 'algorithm_profile:=other'], ['-p', 'graph.lag_s:=1.0']]:
        command = [exe, '--ros-args', '-p', 'base_position.pos:=' + BASE] + bad
        p = subprocess.run(command, env=env, capture_output=True, text=True, timeout=20)
        assert p.returncode == 1 and 'FATAL' in p.stdout + p.stderr, (bad, p.returncode, p.stderr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gnss-executable', required=True)
    ap.add_argument('--imu-executable', required=True)
    a = ap.parse_args()
    with tempfile.TemporaryDirectory(prefix='fgo-node-contract-') as temp:
        os.environ['ROS_LOG_DIR'] = temp
        rclpy.init()
        node = Node('fgo_node_contract_probe')
        try:
            for exe, imu_node in [(a.gnss_executable, False), (a.imu_executable, True)]:
                malformed_input(node, exe, imu_node, temp)
                sigint_and_startup(node, exe, imu_node, temp)
                base_first_after_loop(node, exe, imu_node, temp)
            exact_pair_and_late_base(node, a.imu_executable, temp)
            imu_stall(node, a.imu_executable, temp)
            executor_backlog(node, a.imu_executable, temp)
            utc_headers(node, a.imu_executable, temp)
            sigint_with_pending(node, a.imu_executable, temp)
        finally:
            node.destroy_node()
            rclpy.shutdown()
    print('FGO node contract: malformed input, timeline reset, exact/late base, '
          'base-first loop seam, IMU stall, executor backlog, NONE fields, SIGINT (with a pending epoch) and startup errors passed')


if __name__ == '__main__':
    main()
