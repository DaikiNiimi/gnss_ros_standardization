#!/usr/bin/env python3
"""Q4: direction-explicit parameter names forward bytes through real nodes."""
import argparse
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import unittest

parser = argparse.ArgumentParser()
parser.add_argument('--bin-dir', required=True)
args, remaining = parser.parse_known_args()
BIN = Path(args.bin_dir).resolve()


def listener():
    sock = socket.socket()
    sock.bind(('127.0.0.1', 0))
    sock.listen(1)
    sock.settimeout(10)
    return sock


def uri(sock): return 'tcpcli://127.0.0.1:' + str(sock.getsockname()[1])


def receive(conn, size):
    conn.settimeout(5)
    result = b''
    while len(result) < size:
        part = conn.recv(size - len(result))
        if not part: raise AssertionError('stream closed before payload completion')
        result += part
    return result


class RelayNames(unittest.TestCase):
    def relay(self, executable, output=False):
        with listener() as receiver, listener() as corrections, tempfile.TemporaryDirectory() as directory:
            command = [str(BIN / executable), '--ros-args', '-p', 'stream_path:=' + uri(receiver)]
            if output:
                command += ['-p', 'rtcm_output.destination:=' + uri(corrections),
                            '-p', 'reference_week:=2300']
            else:
                command += ['-p', 'rtcm_input.listen:=' + uri(corrections)]
                if 'driver' in executable:
                    command += ['-p', 'rtcm_input.enable:=true', '-p', 'configure_on_startup:=false']
            env = dict(os.environ, ROS_LOG_DIR=directory, ROS_LOCALHOST_ONLY='1',
                       LD_LIBRARY_PATH=str(BIN) + os.pathsep + os.environ.get('LD_LIBRARY_PATH', ''))
            with open(Path(directory) / 'node.log', 'w+') as log:
                proc = subprocess.Popen(command, stdout=log, stderr=log, env=env)
                try:
                    with receiver.accept()[0] as main:
                        # Output TCP connects lazily on the first queued write.
                        payload = bytes(range(256)) * 4
                        if output: main.sendall(payload)
                        with corrections.accept()[0] as correction:
                            if not output: correction.sendall(payload)
                            outgoing = correction if output else main
                            self.assertEqual(receive(outgoing, len(payload)), payload)
                except Exception:
                    log.flush(); log.seek(0)
                    print(log.read())
                    raise
                finally:
                    if proc.poll() is None: proc.send_signal(signal.SIGINT)
                    try: code = proc.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        proc.kill(); proc.wait(); raise
                self.assertEqual(code, 0)

    def test_Q4_receiver_correction_input_all_six_nodes(self):
        for family in ['ubx', 'sbf', 'novatel']:
            for mode in ['driver', 'decoder']:
                with self.subTest(family=family, mode=mode):
                    self.relay(f'{family}_{mode}_node')

    def test_Q4_rtcm_source_output(self):
        self.relay('rtcm_decoder_node', output=True)

if __name__ == '__main__': unittest.main(argv=[__file__] + remaining)
