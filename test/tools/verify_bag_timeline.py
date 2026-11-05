#!/usr/bin/env python3
"""Check that an IMU topic carries a usable TIMELINE.

This answers a different question from verify_bag_imu.py, which checks the
body-frame convention. Both can pass while the other fails, so they stay
separate scripts.

What is checked, and why each matters to a fusion node:

  n_backwards   stamps that do not advance. Must be 0. The tightly-coupled FGO
                drops the entire IMU factor for an epoch whose interval is not
                strictly increasing (gnss_imu_fgo.cpp, `imu_interval_valid`),
                and its imu_buffer_ stops being sorted, which imuWatermark()
                and the epoch drain both assume. The EKF pushes the sample
                into its buffer before checking dt, so it corrupts the buffer
                the OOSM replay reads back.

  std(dt)       spacing stability. Stamping from host arrival time quantizes
                the timeline to the poll period (10 ms) and the USB burst
                structure, so this sits in the millisecond range. Stamping
                from the receiver's own sample counter should put it at the
                counter resolution instead.

  median(dt)    a wrong tick constant shows up here. u-blox does not document
                the sTtag unit (Scale and Unit are both blank in the ZED-F9R
                Interface Description), so the value in the code is an
                assumption worth measuring against --expect-rate.

  n_samples     the fix must not drop or duplicate samples.

Usage:
  verify_bag_timeline.py --bag <bag> [--topic /gnss/imu/data_raw]
                         [--expect-rate 100] [--expect-samples N]
"""
import argparse
import glob
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu


def resolve_uri(bag):
    """Accept a bag directory or a bare .mcap file.

    A recording stopped with SIGINT can be left without metadata.yaml, which
    makes the directory unopenable while the .mcap beside it is perfectly
    readable. Falling back to the file keeps a half-finished capture usable.
    """
    if os.path.isdir(bag) and not os.path.exists(os.path.join(bag, "metadata.yaml")):
        files = sorted(glob.glob(os.path.join(bag, "*.mcap")))
        if files:
            print(f"note: {bag} has no metadata.yaml; reading {files[0]}")
            return files[0]
    return bag


def read_stamps(bag, topic):
    """-> (header_stamp_s, bag_receive_s) in message order."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=resolve_uri(bag), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    names = {t.name for t in reader.get_all_topics_and_types()}
    if topic not in names:
        raise SystemExit(f"bag has no topic {topic}; present: {sorted(names)}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    header, recv = [], []
    while reader.has_next():
        _, data, t_ns = reader.read_next()
        m = deserialize_message(data, Imu)
        header.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        recv.append(t_ns * 1e-9)
    if not header:
        raise SystemExit(f"topic {topic} carries no messages")
    return np.array(header), np.array(recv)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--topic", default="/gnss/imu/data_raw")
    ap.add_argument("--expect-rate", type=float, default=None,
                    help="nominal IMU rate [Hz]; enables the median-dt check")
    ap.add_argument("--expect-samples", type=int, default=None,
                    help="sample count the producer was fed")
    ap.add_argument("--max-std-dt-ms", type=float, default=None,
                    help="fail if the dt standard deviation exceeds this [ms]")
    args = ap.parse_args()

    stamp, recv = read_stamps(args.bag, args.topic)
    dt = np.diff(stamp)
    n_back = int(np.sum(dt <= 0.0))

    print(f"topic          : {args.topic}")
    print(f"n_samples      : {len(stamp)}")
    print(f"span (header)  : {stamp[-1] - stamp[0]:.3f} s")
    print(f"span (receive) : {recv[-1] - recv[0]:.3f} s")
    print(f"n_backwards    : {n_back}")
    if len(dt):
        print(f"dt median      : {np.median(dt)*1e3:.6f} ms")
        print(f"dt mean        : {np.mean(dt)*1e3:.6f} ms")
        print(f"dt std         : {np.std(dt)*1e3:.6f} ms")
        print(f"dt min / max   : {np.min(dt)*1e3:.6f} / {np.max(dt)*1e3:.6f} ms")
        # Where the mass sits, so a bimodal (burst) distribution is visible.
        qs = np.percentile(dt, [1, 5, 50, 95, 99]) * 1e3
        print("dt pct (1/5/50/95/99) : " +
              " / ".join(f"{q:.4f}" for q in qs) + " ms")

    ok = True
    if n_back:
        print(f"FAIL: {n_back} stamps do not advance "
              f"({100.0*n_back/max(len(dt),1):.2f} % of intervals)")
        ok = False
    if args.expect_samples is not None and len(stamp) != args.expect_samples:
        print(f"FAIL: expected {args.expect_samples} samples, got {len(stamp)}")
        ok = False
    if args.expect_rate and len(dt):
        want = 1.0 / args.expect_rate
        got = float(np.median(dt))
        if abs(got - want) > 0.05 * want:
            print(f"FAIL: median dt {got*1e3:.4f} ms is not within 5 % of the "
                  f"expected {want*1e3:.4f} ms (wrong tick constant?)")
            ok = False
    if args.max_std_dt_ms is not None and len(dt):
        got = float(np.std(dt)) * 1e3
        if got > args.max_std_dt_ms:
            print(f"FAIL: dt std {got:.6f} ms exceeds {args.max_std_dt_ms} ms")
            ok = False

    print("PASS" if ok else "FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
