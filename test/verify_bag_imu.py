#!/usr/bin/env python3
"""Check that the IMU in the bag is in the body frame the FGO node assumes.

The companion scripts verify_bag_obs.py / verify_bag_eph.py answer "did the
converter preserve the GNSS data?". This answers the question that sits one
level above the IMU: "is the body-frame convention right at all?" - which
`--imu-frame` cannot answer for itself, because every mode produces a
well-formed bag and a node that runs to completion.

The tightly-coupled node integrates in a z-up ENU navigation frame, so the bag
must carry an FLU body: x forward, y left, z up, accelerometer reporting
specific force (+g on z at rest). Two checks establish that against a reference
trajectory:

    1. accel x  vs  d|v|/dt                 -> slope +1 means x is forward
    2. gyro  z  vs  d(atan2(vn, ve))/dt     -> slope +1 means z is up

x forward and z up plus right-handedness force y left, so the two suffice.

Both regressors are built from the reference VELOCITY alone, so neither depends
on any Euler-angle convention. That matters: the reference's roll/pitch/heading
carry sign conventions of their own (this dataset's pitch is nose-up positive,
the opposite of the middle angle of gtsam's ENU/FLU RzRyRx), so a check written
against those columns can be defeated by a convention disagreement. A check
written against velocity cannot.

Do NOT substitute "the node's initialized roll/pitch/heading look right". A
frame conversion applied to already-converted data can negate accel x and gyro
y/z while leaving accel y alone: roll then still agrees with the reference to a
fraction of a degree while pitch and heading are silently wrong and the forward
accelerometer runs backwards.

Usage:
  verify_bag_imu.py --bag <bag> --reference <reference.csv>
                    [--topic /gnss/imu/data_raw] [--min-speed 3.0]
"""
import argparse
import csv
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

# Slope must land within this of +1 to pass. The regressors are differentiated
# reference velocities, so they are noisy; the failure this guards against is a
# SIGN error, which lands near -1 and is nowhere near the band.
SLOPE_TOL = 0.45
MIN_CORR = 0.5
# GPS week seconds, for mapping bag stamps (GPST as unix-like) onto tow.
WEEK_S = 604800.0


def read_reference(path):
    """-> tow-sorted arrays (tow, course_enu_deg, speed)."""
    tow, course, speed = [], [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            ve = float(row["East Velocity (m/s)"])
            vn = float(row["North Velocity (m/s)"])
            tow.append(float(row["GPS TOW (s)"]))
            # ENU yaw: CCW from East, the same sense as gtsam's rpy().z() in a
            # z-up navigation frame.
            course.append(math.degrees(math.atan2(vn, ve)))
            speed.append(math.hypot(ve, vn))
    order = np.argsort(tow)
    return (np.array(tow)[order], np.array(course)[order], np.array(speed)[order])


def read_bag_imu(bag, topic):
    """-> (stamp_s, accel Nx3, gyro Nx3) in message order."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    names = {t.name for t in reader.get_all_topics_and_types()}
    if topic not in names:
        raise SystemExit(f"bag has no topic {topic}; present: {sorted(names)}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    stamp, acc, gyr = [], [], []
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, Imu)
        stamp.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        acc.append((m.linear_acceleration.x, m.linear_acceleration.y,
                    m.linear_acceleration.z))
        gyr.append((m.angular_velocity.x, m.angular_velocity.y,
                    m.angular_velocity.z))
    if not stamp:
        raise SystemExit(f"topic {topic} carries no messages")
    return np.array(stamp), np.array(acc), np.array(gyr)


def align_to_tow(stamp, ref_tow):
    """Bag stamps are GPST seconds on an arbitrary epoch; recover the offset.

    Rounding to the week does not work on its own (the writer's epoch need not
    be the GPS epoch), so take the offset that puts the IMU start on the
    reference start, then require the spans to agree.
    """
    offset = round((stamp[0] - ref_tow[0]) / WEEK_S) * WEEK_S
    tow = stamp - offset
    shift = tow[0] - ref_tow[0]
    tow -= round(shift)
    if abs(tow[0] - ref_tow[0]) > 1.0 or abs(tow[-1] - ref_tow[-1]) > 5.0:
        raise SystemExit(
            f"IMU span {tow[0]:.1f}..{tow[-1]:.1f} does not match the reference "
            f"{ref_tow[0]:.1f}..{ref_tow[-1]:.1f}; wrong reference for this bag?")
    return tow


def regress(y, x):
    """Least-squares slope of y on x, with the correlation."""
    y, x = np.asarray(y), np.asarray(x)
    if len(x) < 30:
        raise SystemExit(f"only {len(x)} usable epochs; cannot judge the frame")
    slope = float(np.linalg.lstsq(
        np.vstack([x, np.ones_like(x)]).T, y, rcond=None)[0][0])
    return slope, float(np.corrcoef(x, y)[0, 1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--reference", required=True)
    ap.add_argument("--topic", default="/gnss/imu/data_raw")
    ap.add_argument("--min-speed", type=float, default=3.0,
                    help="course is meaningless at a standstill [m/s]")
    args = ap.parse_args()

    ref_tow, course, speed = read_reference(args.reference)
    stamp, acc, gyr = read_bag_imu(args.bag, args.topic)
    tow = align_to_tow(stamp, ref_tow)

    dt = np.median(np.diff(ref_tow))
    ax, dv, gz, dcourse = [], [], [], []
    for i in range(1, len(ref_tow)):
        if abs((ref_tow[i] - ref_tow[i - 1]) - dt) > 1e-6:
            continue  # gap: the difference would not be a rate
        window = (tow >= ref_tow[i - 1]) & (tow < ref_tow[i])
        if window.sum() < 2:
            continue
        ax.append(acc[window, 0].mean())
        dv.append((speed[i] - speed[i - 1]) / dt)
        if speed[i] < args.min_speed or speed[i - 1] < args.min_speed:
            continue
        # Wrapped difference only - never unwrap a series with gaps in it.
        rate = ((course[i] - course[i - 1] + 180.0) % 360.0 - 180.0) / dt
        if abs(rate) > 60.0:
            continue  # reference glitch, not a real turn
        gz.append(math.degrees(gyr[window, 2].mean()))
        dcourse.append(rate)

    checks = [
        ("accel x vs d|v|/dt      (+1 => x forward)", *regress(ax, dv)),
        ("gyro  z vs d(course)/dt (+1 => z up)     ", *regress(gz, dcourse)),
    ]
    ok = True
    print(f"IMU body-frame check: {args.bag}")
    print(f"  samples {len(stamp)}, {1.0 / np.median(np.diff(stamp)):.0f} Hz, "
          f"accel z mean {acc[:, 2].mean():+.3f} m/s^2")
    for label, slope, corr in checks:
        good = abs(slope - 1.0) <= SLOPE_TOL and corr >= MIN_CORR
        ok &= good
        print(f"  [{'ok ' if good else 'BAD'}] {label} slope={slope:+.3f} "
              f"corr={corr:+.3f}")
    if not ok:
        print("\nThe bag's body frame is NOT the FLU z-up frame the FGO node "
              "integrates in.\nRebuild it with a different --imu-frame "
              "(raw | frd2flu | frd2flu_grav). A negative\nslope is a sign "
              "flip on that axis; the GNSS-only nodes are unaffected, so this "
              "shows\nup only as the tightly-coupled node losing to them.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
