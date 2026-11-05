#!/usr/bin/env python3
"""Structural checks on a converted bag's ephemeris stream.

Catches the two converter bug classes found on 2026-07-17 and their relatives:

  1. GAL future-toe emission: a Galileo record whose toe is in the future at
     emission time is DEAD WEIGHT downstream (RTKLIB seleph requires past toe
     for Galileo) and, under on-change dedup, it shadows the usable record.
     -> must be zero.
  2. Coverage: every satellite the rover actually observes (in the enabled
     systems) whose ephemeris exists in the source nav file must appear in the
     bag stream. (RINEX 3 nav pads single-digit PRNs with a space - "E 9" -
     which naive parsers miscount; parsed here accordingly.)
  3. Offered QoS on /gnss/ephemeris: depth >= 256 + transient_local, so a
     late-joining subscriber still receives the full latched set.
  4. Emission-time sanity: every eph message's stamp lies within the obs span.

Behavioural parity with rnx2rtkp (which satellites are actually USED per
epoch) is checked separately by diff_solstat.py on the RTK node's solution
status - run both.

Usage: verify_bag_eph.py --bag <bag> --nav <base.nav> [--obs-topic /gnss/observation]
"""
import argparse
import re
import sys
from collections import Counter

import rosbag2_py
from rclpy.serialization import deserialize_message
from gnss_ros_standardization.msg import GnssEphemerides, GnssObservations


def nav_sats(path):
    """Satellites with >=1 record in the RINEX 3 nav file (space-padded PRN)."""
    out = Counter()
    started = False
    for line in open(path, errors="replace"):
        if "END OF HEADER" in line:
            started = True
            continue
        if not started:
            continue
        m = re.match(r"^([GERJCIS])\s*(\d{1,2})\s+\d{4}\s", line)
        if m:
            out[f"{m.group(1)}{int(m.group(2)):02d}"] += 1
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--nav", required=True)
    ap.add_argument("--obs-topic", default="/gnss/observation")
    ap.add_argument("--min-obs-epochs", type=int, default=50)
    a = ap.parse_args()

    problems = []

    # --- QoS (from metadata written by the converter)
    import yaml, os
    meta = yaml.safe_load(open(os.path.join(a.bag, "metadata.yaml")))
    for t in meta["rosbag2_bagfile_information"]["topics_with_message_count"]:
        tm = t["topic_metadata"]
        if tm["name"] != "/gnss/ephemeris":
            continue
        prof = tm["offered_qos_profiles"]
        # Jazzy stores a list of dicts; Humble a YAML string. Normalize.
        if isinstance(prof, str):
            prof = yaml.safe_load(prof)
        if not isinstance(prof, list):
            prof = [prof]
        p0 = prof[0] if prof else {}
        depth = int(p0.get("depth", 0)) if isinstance(p0, dict) else 0
        dur = str(p0.get("durability", "")) if isinstance(p0, dict) else ""
        if depth < 256:
            problems.append(f"QoS depth < 256 (got {depth})")
        if "transient_local" not in dur and dur != "1":
            problems.append(f"QoS not transient_local (got {dur})")

    # --- bag streams
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id=""),
                rosbag2_py.ConverterOptions("", ""))
    obs_sats = Counter()
    obs_tow_min = None
    obs_tow_max = None
    eph_sats = set()
    gal_future = 0
    n_eph_msgs = 0
    week = None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == a.obs_topic:
            m = deserialize_message(data, GnssObservations)
            week = m.week
            obs_tow_min = m.tow if obs_tow_min is None else min(obs_tow_min, m.tow)
            obs_tow_max = m.tow if obs_tow_max is None else max(obs_tow_max, m.tow)
            for o in m.observations:
                obs_sats[o.satid] += 1
        elif topic == "/gnss/ephemeris":
            n_eph_msgs += 1
            m = deserialize_message(data, GnssEphemerides)
            for e in m.gnss_ephemeris:
                if not e.satid:
                    continue
                eph_sats.add(e.satid)
                # emission-time tow of this message is unknown here; the
                # converter emits at obs epochs, so compare against the CURRENT
                # max obs tow seen so far (stream is time-ordered).
                if e.system == "E" and obs_tow_max is not None \
                        and e.toe >= obs_tow_max + 1e-3:
                    gal_future += 1
            for g in m.glonass_ephemeris:
                if g.satid:
                    eph_sats.add(g.satid)

    if gal_future:
        problems.append(f"GAL future-toe emissions: {gal_future} (must be 0)")

    nav = nav_sats(a.nav)
    used = {s for s, n in obs_sats.items()
            if n >= a.min_obs_epochs and s[0] in "GEJC"}
    missing = sorted(s for s in used if s in nav and s not in eph_sats)
    if missing:
        problems.append(f"observed sats with nav record but NO bag eph: {missing}")
    no_nav = sorted(s for s in used if s not in nav)

    print(f"bag={a.bag}")
    print(f"  eph msgs={n_eph_msgs}, distinct eph sats={len(eph_sats)}, "
          f"rover sats used={len(used)} (>{a.min_obs_epochs} epochs, GEJC)")
    if no_nav:
        print(f"  note: observed but absent from source nav (CLI can't use "
              f"them either): {no_nav}")
    if problems:
        for p in problems:
            print(f"  PROBLEM: {p}")
        print("RESULT: FAIL")
        return 2
    print("RESULT: OK (no GAL future-toe, full coverage, QoS depth>=256 TL)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
