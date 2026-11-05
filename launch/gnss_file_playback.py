#!/usr/bin/env python3
"""Play a raw receiver log into a decoder once everything that reads it is up.

pipeline.launch.py starts a decoder whose source is file:// listening on
tcpsvr://127.0.0.1:<port> instead, and runs this script next to it. A decoder
reading the file itself starts at once, while a recorder subscribes only after
it has discovered the decoder's topics, so the start of every file was lost.
Here nothing is sent until each listed topic has a publisher and at least the
given number of subscribers; then the file goes over the loopback socket as
fast as the decoder reads it, which is what file:// did.

usage: gnss_file_playback.py <file> <port> [<topic>=<subscribers> ...]
"""
import os
import socket
import sys
import time

import rclpy
from rclpy.node import Node

CHUNK = 65536


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    path, port = sys.argv[1], int(sys.argv[2])
    wants = {}
    for arg in sys.argv[3:]:
        topic, _, count = arg.rpartition('=')
        wants[topic] = int(count)

    rclpy.init(args=[sys.argv[0]])
    node = Node('gnss_file_playback_{}'.format(port))
    log = node.get_logger()
    try:
        size = os.path.getsize(path)
    except OSError as exc:
        log.fatal('cannot read {}: {}'.format(path, exc))
        sys.exit(1)

    # Event-driven on the ROS graph: the poll period only sets how soon the
    # start is noticed, never whether data is lost.
    waiting = None
    while rclpy.ok():
        missing = sorted(t for t, n in wants.items()
                         if node.count_publishers(t) < 1 or node.count_subscribers(t) < n)
        if not missing:
            break
        if missing != waiting:
            log.info('{}: waiting for {}'.format(os.path.basename(path), ', '.join(
                '{} ({} of {} subscriber(s))'.format(t, node.count_subscribers(t), wants[t])
                for t in missing)))
            waiting = missing
        time.sleep(0.05)

    sock = None
    while rclpy.ok() and sock is None:
        try:
            sock = socket.create_connection(('127.0.0.1', port))
        except OSError:
            time.sleep(0.05)
    if sock is None:
        return
    log.info('playing {} ({} bytes) into 127.0.0.1:{}'.format(path, size, port))
    sent = 0
    with sock, open(path, 'rb') as f:
        while True:
            chunk = f.read(CHUNK)
            if not chunk:
                break
            sock.sendall(chunk)
            sent += len(chunk)
        sock.shutdown(socket.SHUT_WR)
    log.info('{}: end of file, {} bytes played'.format(os.path.basename(path), sent))
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
