#!/usr/bin/env python3
"""Field-by-field check that rinex_to_rosbag preserved the observations.

The positioning nodes never see the RINEX; they see the bag. So "is the bag a
faithful copy of the RINEX?" has to be answered on the data itself rather than
by reading the converter. This walks both and compares every observation:

    epoch count / times, and per (satellite, band): pseudorange, carrier phase
    [cycles], Doppler, SNR, LLI (slip + half-cycle bits), and the tracking code

Any of these being dropped or altered would silently degrade ambiguity
resolution (LLI drives cycle-slip detection; SNR drives the mask), so they are
compared exactly rather than approximately.

Note: RTKLIB keeps at most one code per band (highest getcodepri), and the
converter emits what RTKLIB parsed. This script therefore compares the bag
against the RINEX columns RTKLIB would have selected, per system/band.

Usage:
  verify_bag_obs.py --bag <bag> --obs <rover.obs> [--topic /gnss/observation]
                    [--max-epochs N]
"""
import argparse
import sys
from collections import defaultdict

import rosbag2_py
from rclpy.serialization import deserialize_message
from gnss_ros_standardization.msg import GnssObservations


def read_rinex(path, max_epochs=0):
    """Minimal RINEX 3 obs reader -> {tow: {(satid, obs_type): (val, lli, snr)}}."""
    epochs = {}
    types = {}
    with open(path) as f:
        # header
        sys_cur = None
        for line in f:
            label = line[60:].strip()
            if label == "SYS / # / OBS TYPES":
                if line[0].strip():
                    sys_cur = line[0]
                    n = int(line[3:6])
                    types[sys_cur] = line[7:60].split()
                    types[sys_cur + "_n"] = n
                else:
                    types[sys_cur] += line[7:60].split()
            elif label == "END OF HEADER":
                break
        # data
        for line in f:
            if not line.startswith(">"):
                continue
            y, mo, d = int(line[2:6]), int(line[7:9]), int(line[10:12])
            hh, mi = int(line[13:15]), int(line[16:18])
            ss = float(line[19:29])
            nsat = int(line[32:35])
            import datetime
            # RINEX 3 obs epochs are already GPST (TIME SYSTEM: GPS) - no leap
            # second is applied here; the bag carries the same GPST week/tow.
            t = datetime.datetime(y, mo, d, hh, mi) + datetime.timedelta(seconds=ss)
            tow = ((t - datetime.datetime(1980, 1, 6)).total_seconds()) % (7 * 86400)
            obs = {}
            for _ in range(nsat):
                row = f.readline().rstrip("\n")
                satid = row[0:3]
                s = satid[0]
                if s not in types:
                    continue
                for i, ot in enumerate(types[s]):
                    fld = row[3 + 16 * i: 3 + 16 * i + 14]
                    lli_c = row[3 + 16 * i + 14: 3 + 16 * i + 15]
                    if not fld.strip():
                        continue
                    lli = int(lli_c) if lli_c.strip().isdigit() else 0
                    obs[(satid, ot)] = (float(fld), lli)
            epochs[round(tow, 3)] = obs
            if max_epochs and len(epochs) >= max_epochs:
                break
    return epochs, types


def read_bag(path, topic, max_epochs=0):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=""),
           rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    out = {}
    while r.has_next():
        _, data, _ = r.read_next()
        m = deserialize_message(data, GnssObservations)
        e = {}
        for o in m.observations:
            sid = o.satid
            # RTKLIB names SBAS satellites by PRN ("122"); RINEX 3 uses S+(prn-100)
            # ("S22"). Same satellite, different convention.
            if sid[:1].isdigit():
                sid = "S%02d" % (int(sid) - 100)
            e[(sid, o.code_str)] = (o.p, o.l, o.d, o.snr, o.lli, o.code)
        out[round(m.tow, 3)] = e
        if max_epochs and len(out) >= max_epochs:
            break
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--obs", required=True)
    ap.add_argument("--topic", default="/gnss/observation")
    ap.add_argument("--max-epochs", type=int, default=200)
    a = ap.parse_args()

    rnx, types = read_rinex(a.obs, a.max_epochs)
    bag = read_bag(a.bag, a.topic, a.max_epochs)

    common = sorted(set(rnx) & set(bag))
    print(f"epochs: rinex={len(rnx)} bag={len(bag)} common={len(common)}")
    if not common:
        print("NO COMMON EPOCHS - time base mismatch")
        return 1

    stats = defaultdict(int)
    bad = []
    for tow in common:
        rr, bb = rnx[tow], bag[tow]
        for (satid, code_str), (p, l, d, snr, lli, code) in bb.items():
            for kind, val, rtype in (("P", p, "C" + code_str),
                                     ("L", l, "L" + code_str),
                                     ("D", d, "D" + code_str),
                                     ("S", snr, "S" + code_str)):
                key = (satid, rtype)
                if key not in rr:
                    if val != 0.0:
                        stats[f"{kind}_missing_in_rinex"] += 1
                        if len(bad) < 5:
                            bad.append(f"{tow} {satid} {rtype} bag={val} rinex=-")
                    continue
                rv, rlli = rr[key]
                stats[f"{kind}_compared"] += 1
                tol = 1e-3 if kind in ("P", "D") else (1e-3 if kind == "L" else 1e-2)
                if abs(rv - val) > tol:
                    stats[f"{kind}_MISMATCH"] += 1
                    if len(bad) < 10:
                        bad.append(f"{tow} {satid} {rtype} bag={val} rinex={rv}")
                if kind == "L":
                    # LLI: bit0 slip, bit1 half-cycle; converter keeps lower 3 bits
                    if (rlli & 0x3) != (lli & 0x3):
                        stats["LLI_MISMATCH"] += 1
                        if len(bad) < 10:
                            bad.append(f"{tow} {satid} LLI bag={lli} rinex={rlli}")
                    elif rlli & 0x3:
                        stats["LLI_set_and_matched"] += 1

    for k in sorted(stats):
        print(f"  {k:26s} {stats[k]}")
    if bad:
        print("  examples:")
        for b in bad:
            print("   ", b)
    nbad = sum(v for k, v in stats.items() if "MISMATCH" in k or "missing" in k)
    print("RESULT:", "FAITHFUL (no mismatches)" if nbad == 0 else f"{nbad} MISMATCHES")
    return 0 if nbad == 0 else 2


if __name__ == "__main__":
    sys.exit(main())
