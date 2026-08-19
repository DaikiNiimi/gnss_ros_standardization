#!/usr/bin/env python3
"""Check that the ground-truth topics written by `rinex_to_rosbag --reference`
say what reference.csv says - in the conventions ROS consumers assume.

Position and velocity are transcriptions and are checked as such. Attitude is
not: reference.csv reports Heading as degrees CLOCKWISE FROM NORTH, while
nav_msgs/Odometry carries an ENU orientation whose yaw is CCW FROM EAST. The
converter applies yaw_enu = 90 - heading, and a sign or offset error there
produces a bag that looks entirely well-formed - correct message types, correct
rates, plausible angles - while every attitude comparison drawn from it is
wrong.

So the yaw is verified against a quantity that carries NO Euler convention at
all: reference.csv's own East/North velocity. The course over ground,
atan2(VN, VE), is already ENU yaw by definition, and for a wheeled vehicle
driving forward the body heading equals it to within side-slip. Regressing the
bag's quaternion yaw on that course must give slope +1 and intercept 0; a
flipped sign lands at -1 and a North/East mix-up shows up in the intercept.

This is the same discipline verify_bag_imu.py enforces for the IMU body frame,
and for the same reason: that bug survived for months because the check being
performed (compare initialized roll/pitch/heading to the reference) could not
detect it. A check written against velocity cannot be defeated by an angle
convention disagreement, because it does not use any.

Roll and pitch are compared directly - they have no 90-degree offset to get
wrong, and the reference is near-level for most of a road course, so this
catches a swapped axis rather than a sign convention.

Usage:
  verify_bag_reference.py --bag <bag> --reference <reference.csv>
                          [--topic /gnss/reference/solution] [--min-speed 3.0]
"""
import argparse
import csv
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

from gnss_ros_standardization.msg import GnssSolution

# The course regressor is a wheeled-vehicle assumption plus differentiated
# reference velocity, so it is noisy; the failure guarded against is a SIGN or
# 90-degree error, which lands nowhere near +1.
SLOPE_TOL = 0.15
MIN_CORR = 0.9
MAX_YAW_BIAS_DEG = 8.0
# Transcription tolerances: these are copies, not derived quantities.
MAX_POS_ERR_M = 1e-3
MAX_VEL_ERR_MPS = 1e-6
MAX_RP_ERR_DEG = 0.5
WEEK_S = 604800.0


def read_reference(path):
    """-> dict tow -> row of reference quantities."""
    out = {}
    with open(path) as f:
        for row in csv.DictReader(f):
            tow = round(float(row["GPS TOW (s)"]), 3)
            ve = float(row["East Velocity (m/s)"])
            vn = float(row["North Velocity (m/s)"])
            out[tow] = dict(
                week=int(row["GPS Week"]),
                ecef=(float(row["ECEF X (m)"]), float(row["ECEF Y (m)"]),
                      float(row["ECEF Z (m)"])),
                vel=(ve, vn, float(row["Up Velocity (m/s)"])),
                roll=float(row["Roll (deg)"]),
                pitch=float(row["Pitch (deg)"]),
                heading=float(row["Heading (deg)"]),
                # ENU yaw by definition: CCW from East.
                course=math.degrees(math.atan2(vn, ve)),
                speed=math.hypot(ve, vn),
            )
    if not out:
        raise SystemExit(f"{path} produced no rows")
    return out


def read_bag(bag, topic):
    """-> (solutions by tow, odometry in message order)."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    names = {t.name for t in reader.get_all_topics_and_types()}
    odom_topic = topic + "_odom"
    for want in (topic, odom_topic):
        if want not in names:
            raise SystemExit(
                f"bag has no topic {want}; present: {sorted(names)}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic, odom_topic]))
    sols, odoms = {}, []
    n_sol_msgs = 0
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name == topic:
            m = deserialize_message(data, GnssSolution)
            sols[round(m.time_tow, 3)] = m
            n_sol_msgs += 1
        else:
            m = deserialize_message(data, Odometry)
            odoms.append(m)
    if not sols:
        raise SystemExit(f"topic {topic} carries no messages")
    if len(sols) != n_sol_msgs:
        raise SystemExit(
            f"{n_sol_msgs} solution msgs collapse to {len(sols)} distinct TOWs "
            "- duplicate epochs would break the epoch pairing below")
    if len(odoms) != len(sols):
        raise SystemExit(
            f"{len(sols)} solution msgs but {len(odoms)} odometry msgs - the "
            "two reference topics must be written in lockstep")
    return sols, odoms


def rpy_from_quaternion(q):
    """-> (roll, pitch, yaw) degrees, ENU/FLU, matching gtsam's RzRyRx."""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--reference", required=True)
    ap.add_argument("--topic", default="/gnss/reference/solution")
    ap.add_argument("--min-speed", type=float, default=3.0,
                    help="course is meaningless at a standstill [m/s]")
    args = ap.parse_args()

    ref = read_reference(args.reference)
    sols, odoms = read_bag(args.bag, args.topic)

    matched = sorted(set(ref) & set(sols))
    if len(matched) < 30:
        raise SystemExit(
            f"only {len(matched)} epochs matched by TOW between the bag and "
            f"{args.reference}; wrong reference for this bag?")
    if len(matched) < 0.99 * len(ref):
        raise SystemExit(
            f"only {len(matched)} of {len(ref)} reference rows reached the bag")

    failures = []

    # 1. Position / velocity are transcriptions.
    pos_err = vel_err = 0.0
    for tow in matched:
        r, s = ref[tow], sols[tow]
        pos_err = max(pos_err, math.dist(
            r["ecef"], (s.pos_ecef.x, s.pos_ecef.y, s.pos_ecef.z)))
        vel_err = max(vel_err, math.dist(
            r["vel"], (s.vel_enu.x, s.vel_enu.y, s.vel_enu.z)))
        if s.time_week != r["week"]:
            failures.append(f"tow {tow}: week {s.time_week} != {r['week']}")
            break
    print(f"position  max |bag - reference| : {pos_err:.6f} m "
          f"(limit {MAX_POS_ERR_M})")
    print(f"velocity  max |bag - reference| : {vel_err:.9f} m/s "
          f"(limit {MAX_VEL_ERR_MPS})")
    if pos_err > MAX_POS_ERR_M:
        failures.append(f"position transcription error {pos_err:.4f} m")
    if vel_err > MAX_VEL_ERR_MPS:
        failures.append(f"velocity transcription error {vel_err:.9f} m/s")

    # 2. Attitude. The converter writes one Odometry immediately after each
    #    GnssSolution, and `sols` is insertion-ordered by bag order, so zipping
    #    its keys against the odometry list pairs the same epochs.
    by_tow = dict(zip(list(sols.keys()), odoms))

    yaw_bag, course_ref, rp_err = [], [], 0.0
    for tow in matched:
        r = ref[tow]
        roll, pitch, yaw = rpy_from_quaternion(by_tow[tow].pose.pose.orientation)
        rp_err = max(rp_err, abs(wrap180(roll - r["roll"])),
                     abs(wrap180(pitch - r["pitch"])))
        if r["speed"] >= args.min_speed:
            yaw_bag.append(yaw)
            course_ref.append(r["course"])

    print(f"roll/pitch max |bag - reference|: {rp_err:.4f} deg "
          f"(limit {MAX_RP_ERR_DEG})")
    if rp_err > MAX_RP_ERR_DEG:
        failures.append(f"roll/pitch error {rp_err:.3f} deg")

    if len(yaw_bag) < 30:
        failures.append(
            f"only {len(yaw_bag)} epochs above {args.min_speed} m/s; the yaw "
            "convention cannot be judged")
    else:
        # Unwrap the two series INDEPENDENTLY, then fit yaw = slope*course + b.
        #
        # Independence is the whole point: deriving the bag series from the
        # reference one (y = c + wrap180(y-c)) removes the +-180 seam but also
        # forces slope ~ +1 by construction, so the slope test would pass on a
        # sign-flipped bag. Unwrapping each on its own is valid here because
        # consecutive epochs are 0.2 s apart and a road vehicle cannot turn
        # 180 deg between them.
        c = np.unwrap(np.radians(np.array(course_ref)))
        y = np.unwrap(np.radians(np.array(yaw_bag)))
        slope = float(np.linalg.lstsq(
            np.vstack([c, np.ones_like(c)]).T, y, rcond=None)[0][0])
        corr = float(np.corrcoef(c, y)[0, 1])
        # wrap180 already returns degrees.
        bias = float(np.median(
            [wrap180(a - b) for a, b in zip(yaw_bag, course_ref)]))
        print(f"yaw vs course-over-ground     : slope {slope:+.4f} "
              f"(want +1.00 +-{SLOPE_TOL}), corr {corr:+.4f}, "
              f"median bias {bias:+.3f} deg (limit {MAX_YAW_BIAS_DEG})")
        if abs(slope - 1.0) > SLOPE_TOL:
            failures.append(
                f"yaw slope {slope:+.3f} - the heading convention is wrong "
                "(expected yaw_enu = 90 - heading)")
        if corr < MIN_CORR:
            failures.append(f"yaw correlation {corr:+.3f} below {MIN_CORR}")
        if abs(bias) > MAX_YAW_BIAS_DEG:
            failures.append(
                f"yaw bias {bias:+.2f} deg - a constant offset in the heading "
                "convention")

    if failures:
        print("\nFAIL:")
        for f in failures:
            print(f"  - {f}")
        return 1
    print(f"\nOK: {len(matched)} epochs; reference topics match "
          f"{args.reference} in position, velocity and attitude.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
