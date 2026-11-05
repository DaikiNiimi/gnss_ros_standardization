#!/usr/bin/env python3
"""Run `ros2 bag record` so that one Ctrl-C reaches it exactly once.

A terminal Ctrl-C sends SIGINT to the whole foreground process group, and
`ros2 launch` then forwards its own SIGINT to every process it started. rosbag2
finalizes the bag on the first SIGINT but exits non-zero on the second, which
launch reports as "process has died" although the bag is complete.

The recorder therefore runs in its own process group. This wrapper stays in the
launch process group, forwards the first SIGINT (or SIGHUP, e.g. a closed
terminal) once, passes launch's SIGTERM escalation on unchanged, and exits with
the recorder's exit code. If the wrapper itself is killed, the kernel delivers
SIGINT to the recorder (PR_SET_PDEATHSIG), so the bag is still finalized.

usage: gnss_record.py ros2 bag record <arguments...>
"""
import ctypes
import os
import signal
import subprocess
import sys

PR_SET_PDEATHSIG = 1


def own_process_group():
    os.setpgrp()
    try:
        ctypes.CDLL(None, use_errno=True).prctl(PR_SET_PDEATHSIG, signal.SIGINT)
    except (AttributeError, OSError):
        pass  # not Linux: the forwarding below still applies


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    child = subprocess.Popen(sys.argv[1:], preexec_fn=own_process_group)
    stopping = [False]

    def interrupt(signum, frame):
        if not stopping[0]:
            stopping[0] = True
            child.send_signal(signal.SIGINT)

    def terminate(signum, frame):
        stopping[0] = True
        child.send_signal(signal.SIGTERM)

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGHUP, interrupt)
    signal.signal(signal.SIGTERM, terminate)
    code = child.wait()
    sys.exit(128 - code if code < 0 else code)


if __name__ == '__main__':
    main()
