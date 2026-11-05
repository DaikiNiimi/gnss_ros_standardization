#!/usr/bin/env python3
"""Black-box GNSS MotionFactor admission, output, and startup contracts."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from gnss_ros_standardization.msg import GnssObservations, GnssSolution
from std_msgs.msg import String


def wait(node, predicate, reason, timeout=10):
    until = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= until:
            raise AssertionError(reason)
        rclpy.spin_once(node, timeout_sec=0.01)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--executable', required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='gnss-motion-contract-') as temp:
        env = dict(os.environ, ROS_LOG_DIR=temp, ROS_LOCALHOST_ONLY='1')
        common = [args.executable, '--ros-args', '-p', 'base_position.pos:=[6378137.0,0.0,0.0]']
        for parameter, message in [
            ('algorithm_profile:=unknown', 'unsupported algorithm profile'),
            ('graph.lag_s:=1.0', 'unsupported parameter'),
            ('time.output_stamp:=gpst', 'unsupported parameter'),  # removed: the stamp is the rover's
            ('queue_depth:=0', 'queue_depth'),
            ('max_age_s:=-1.0', 'base age'),
        ]:
            p = subprocess.run(common + ['-p', parameter], env=env,
                               capture_output=True, text=True, timeout=15)
            assert p.returncode != 0 and message in p.stderr + p.stdout, (parameter, p.stdout, p.stderr)
        rclpy.init()
        node = Node('motion_contract_probe')
        try:
            for mode in ['utc', 'gpst']:
                trace = Path(temp)/mode
                prefix = '/motion_contract/' + mode
                rover = node.create_publisher(GnssObservations, prefix+'/rover', 32)
                base = node.create_publisher(GnssObservations, prefix+'/base', 32)
                end = node.create_publisher(String, prefix+'/end', 32)
                rows = []
                sub = node.create_subscription(GnssSolution, prefix+'/solution', rows.append, 32)
                params = {'topics.rover_observation': prefix+'/rover',
                          'topics.base_observation': prefix+'/base', 'topics.solution': prefix+'/solution',
                          'debug.end_topic': prefix+'/end', 'debug.trace_directory': str(trace)}
                command = list(common)
                for name, value in params.items():
                    command += ['-p', name+':='+value]
                with (Path(temp)/(mode+'.log')).open('w') as log:
                    p = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT)
                    try:
                        wait(node, lambda: rover.get_subscription_count() and base.get_subscription_count()
                             and end.get_subscription_count() and node.count_publishers(prefix+'/solution'), 'discovery')
                        subscriptions = node.get_subscriber_names_and_types_by_node('gnss_fgo', '/')
                        assert not any(t == 'sensor_msgs/msg/Imu' for _, types in subscriptions for t in types)
                        def obs(tow):
                            # Rover header in this mode's time scale; the output inherits it.
                            m = GnssObservations(); m.week = 2301; m.tow = float(tow)
                            m.header.stamp.sec = int(2301*604800+tow+315964800-(18 if mode=='utc' else 0))
                            return m
                        # Future base cannot be used for rover 100. Its later use at
                        # 111 is causal; at 150 it exceeds the 30 s age limit.
                        base.publish(obs(110))
                        until = time.monotonic()+0.1
                        while time.monotonic() < until:
                            rclpy.spin_once(node, timeout_sec=0.01)
                        def check(m, tow):
                            assert m.status == m.STATUS_NONE and m.time_week == 2301 and m.time_tow == tow
                            assert m.header.stamp.sec == 2301*604800+tow+315964800-(18 if mode=='utc' else 0)
                            assert m.header.stamp.nanosec == 0 and m.header.frame_id == 'gnss_link'
                            assert all(math.isnan(v) for v in [m.pos_ecef.x,m.latitude,m.vel_ecef.x,m.pos_cov_ecef[0]])
                        def quiet(count, seconds=0.5):
                            until = time.monotonic()+seconds
                            while time.monotonic() < until:
                                rclpy.spin_once(node, timeout_sec=0.01)
                            assert len(rows) == count, (len(rows), count)
                        # Rover 100: its only base is 10 s ahead, so no base can
                        # serve it; it is decided at once (no IMU, next rover or EOF).
                        rover.publish(obs(100))
                        wait(node, lambda: len(rows) == 1, 'base-less rover was not released')
                        check(rows[-1], 100)
                        # Rover 111 has only the older base 110. With one base the
                        # cadence is unknown, so base(111) may still come: the epoch
                        # waits for the next rover epoch (stream time, no wall clock).
                        rover.publish(obs(111))
                        quiet(1)
                        rover.publish(obs(150))
                        wait(node, lambda: len(rows) == 2, 'aged pair not released by the next rover')
                        check(rows[-1], 111)
                        # Rover 150 (base 40 s old > max_age_s) is the newest; the
                        # end marker decides it.
                        end.publish(String(data='end'))
                        wait(node, lambda: len(rows) == 3, 'end marker did not release the newest rover')
                        check(rows[-1], 150)
                        p.wait(timeout=15)
                        assert p.returncode == 0
                        summary=json.loads((trace/'intake-summary.json').read_text())
                        assert summary == {'rover_count':3,'pending':0,'worker_exit':0}, summary
                        selected=[json.loads(l)['base_ms'] for l in (trace/'intake-trace.jsonl').read_text().splitlines()
                                  if json.loads(l)['event']=='base_selected']
                        assert selected == [-1,2301*604800000+110000,-1], selected
                        assert (trace/'solver-completed').read_text().strip() == '3'
                    finally:
                        if p.poll() is None:
                            p.terminate(); p.wait(timeout=10)
                node.destroy_subscription(sub)
                for pub in [rover,base,end]:node.destroy_publisher(pub)
        finally:
            node.destroy_node(); rclpy.shutdown()
    print('Motion ROS startup, no-IMU release, causal base/age, UTC/GPST, NONE and EOF passed')

if __name__ == '__main__':main()
