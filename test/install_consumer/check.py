#!/usr/bin/env python3
"""Check installed Python types and exchange current messages with external C++."""
import argparse
import importlib
import json
import math
from pathlib import Path
import signal
import subprocess
import time
import uuid

import rclpy
from rclpy.serialization import serialize_message
from gnss_ros_standardization.msg import (
    GlonassEphemeris, GnssEphemerides, GnssEphemeris, GnssObservation,
    GnssObservations, GnssSolution,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", required=True)
    parser.add_argument("--prefix", required=True)
    args = parser.parse_args()
    prefix = Path(args.prefix).resolve()
    # E1: binary installations must carry the statically linked backend's notice.
    share = prefix / "share/gnss_ros_standardization"
    assert (share / "LICENSE").is_file()
    notices = (share / "LICENSE").read_text()
    assert "Copyright (c) 2007-2020, T. Takasu" in notices
    assert "Redistributions in binary form must" in notices
    assert "modified copies" in notices
    classes = (GnssObservation, GnssObservations, GnssEphemeris,
               GlonassEphemeris, GnssEphemerides, GnssSolution)
    package = importlib.import_module("gnss_ros_standardization.msg")
    for removed in ("NavigationMetadata", "ReceiverFrame"):
        assert not hasattr(package, removed), removed
    assert "metadata" not in GnssEphemeris.get_fields_and_field_types()
    assert "metadata" not in GlonassEphemeris.get_fields_and_field_types()
    modules = {}
    for cls in classes:
        path = Path(importlib.import_module(cls.__module__).__file__).resolve()
        assert path.is_relative_to(prefix), (cls.__name__, path, prefix)
        modules[cls.__name__] = str(path)
        serialize_message(cls())  # Load installed native type support too.
    assert GnssObservation().freq == 255

    obs = GnssObservations(week=2400, tow=123456.0)
    obs.header.frame_id = "installed_receiver"
    obs.observations = [GnssObservation(
        system="R", prn=1, satid="R01", code=1, code_str="1C",
        freq=13, p=21000000.25, l=float("nan"), d=0.0, snr=42.5, lli=0xc3)]
    nav = GnssEphemerides()
    nav.gnss_ephemeris = [GnssEphemeris(
        system="E", prn=1, satid="E01", week=2400, iode=1023,
        sva=-2, adot=0.25, ndot=1e-12, tgd=[1e-9] * 6)]
    nav.glonass_ephemeris = [GlonassEphemeris(
        system="R", prn=1, satid="R01", week=2400, frq=6,
        flags=0x101, sva=-1, pos=[1.0, 2.0, 3.0],
        vel=[4.0, 5.0, 6.0], acc=[7.0, 8.0, 9.0])]
    expected = {"observations": obs, "ephemerides": nav}
    received = set()
    namespace = "/install_check_" + uuid.uuid4().hex[:10]
    rclpy.init()
    node = rclpy.create_node("installed_python_consumer", namespace=namespace)
    publishers, subscriptions = [], []

    def echo(name, message):
        # Serialized comparison also checks every nested field and preserves NaN.
        assert serialize_message(message) == serialize_message(expected[name]), name
        received.add(name)

    for name, message in expected.items():
        publishers.append((node.create_publisher(type(message), "input/" + name, 10), message))
        subscriptions.append(node.create_subscription(
            type(message), "echo/" + name,
            lambda message, name=name: echo(name, message), 10))

    def probe(message):
        assert message.week == 2400 and message.tow == 123456.0
        assert len(message.observations) == 1
        value = message.observations[0]
        assert value.system == "R" and value.prn == 1 and value.satid == "R01"
        assert value.freq == 13 and value.lli == 0xc3
        assert value.p == 21000000.25 and math.isnan(value.l)
        assert value.d == 0.0 and value.snr == 42.5
        received.add("cpp_probe")

    subscriptions.append(node.create_subscription(GnssObservations, "cpp/probe", probe, 10))
    child = subprocess.Popen([args.executable, "--ros-args", "-r", "__ns:=" + namespace])
    try:
        deadline = time.monotonic() + 30.0
        while received != {*expected, "cpp_probe"} and time.monotonic() < deadline:
            assert child.poll() is None, f"C++ consumer exited: {child.returncode}"
            for publisher, message in publishers:
                publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.1)
        assert received == {*expected, "cpp_probe"}, f"missing exchanges: {received}"
        loaded = sorted({line.split()[-1] for line in Path("/proc/self/maps").read_text().splitlines()
                         if "/" in line and ("libgnss_" in line or
                                             "_gnss_ros_standardization_s" in line)})
        assert loaded and all(Path(path).resolve().is_relative_to(prefix) for path in loaded), loaded
        hashes = {
            name: [str(getattr(info, "topic_type_hash", "unavailable on this ROS distro")) for info in
                   node.get_publishers_info_by_topic(namespace + "/echo/" + name)]
            for name in expected
        }
        print(json.dumps({"result": "passed", "exchanges": sorted(received),
                          "installed_modules": modules, "loaded_type_support": loaded,
                          "topic_type_hashes": hashes}, indent=2))
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
