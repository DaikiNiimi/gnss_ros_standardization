#!/usr/bin/env python3
"""Every parameter a shipped file or the launch passes to a node is declared by it.

ROS 2 ignores an override for a parameter the node never declares, without a
warning, so a misspelt or renamed key would silently leave the node on its code
default. This test reads the declarations from the node sources and checks
  - every key of every shipped config YAML (config/*.yaml, config/ppc/*.yaml),
  - every parameter the launch files build for a receiver, estimator or the
    visualizer.
"""
import pathlib
import re
import sys
import unittest
from unittest.mock import patch

import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'launch'))
import gnss_pipeline_common as pipeline  # noqa: E402

SOURCES = {
    'ubx_decoder_node': ['src/decoders/ubx_decoder_node.cpp'],
    'sbf_decoder_node': ['src/decoders/sbf_decoder_node.cpp'],
    'novatel_decoder_node': ['src/decoders/novatel_decoder_node.cpp'],
    'rtcm_decoder_node': ['src/decoders/rtcm_decoder_node.cpp'],
    'ubx_driver_node': ['src/drivers/ubx_driver_node.cpp'],
    'sbf_driver_node': ['src/drivers/sbf_driver_node.cpp'],
    'novatel_driver_node': ['src/drivers/novatel_driver_node.cpp'],
    'single_point_positioning': ['src/positioning/spp/single_point_positioning.cpp'],
    'real_time_kinematic': ['src/positioning/rtk/real_time_kinematic.cpp'],
    'gnss_imu_kalman_filter': ['src/positioning/ekf/gnss_imu_kalman_filter.cpp'],
    'gnss_fgo': ['src/positioning/fgo/ros_node.hpp'],
    'gnss_imu_fgo': ['src/positioning/fgo/ros_node.hpp'],
    'gnss_visualizer': ['src/utils/gnss_visualizer.cpp'],
}
# Declared by a shared header for every receiver node (NMEA date handling).
SHARED = ['include/gnss_ros_standardization/decoder_common.hpp']
# declare_parameter and the node-local helpers that wrap it
# (RTK declare_safe, EKF load_vec3, FGO vectorParameter).
DECLARE = re.compile(
    r'\b(?:declare_parameter|declare_safe|load_vec3|vectorParameter)\s*(?:<[^;()]*>)?\s*\(\s*"([^"]+)"')
# Shared helpers that declare parameters for every node that calls them.
HELPERS = {'declareLocalOrigin': 'src/utils/gnss_utils.cpp'}
FGO_TOPIC = re.compile(r'\btopic\(\s*"([a-z_]+)"')
FGO_SIGNAL = re.compile(r'^signals\.(rover|base)\.[GEJC]\.[PLDS]$')
# Declared by every rclcpp node.
ROS = {'use_sim_time'}


def declared(executable):
    names = set(ROS)
    files = SOURCES[executable] + (SHARED if executable.endswith(('_decoder_node', '_driver_node')) else [])
    for rel in files:
        text = (ROOT / rel).read_text()
        names.update(DECLARE.findall(text))
        for helper, where in HELPERS.items():
            if helper + '(*this)' in text:
                body = (ROOT / where).read_text().split('LocalOrigin ' + helper + '(', 1)[1].split('\n}\n', 1)[0]
                names.update(DECLARE.findall(body))
        if rel.endswith('fgo/ros_node.hpp'):
            names.update('topics.' + t for t in FGO_TOPIC.findall(text))
    return names


def undeclared(executable, keys):
    known = declared(executable)
    return sorted(k for k in keys if k not in known and not (
        executable in ('gnss_fgo', 'gnss_imu_fgo') and FGO_SIGNAL.match(k)))


def flatten(mapping, prefix=''):
    for key, value in mapping.items():
        if isinstance(value, dict):
            yield from flatten(value, prefix + key + '.')
        else:
            yield prefix + key


class ParameterNames(unittest.TestCase):
    def test_sources_declare_parameters(self):
        for executable in SOURCES:
            with self.subTest(executable=executable):
                self.assertGreater(len(declared(executable) - ROS), 5)

    def test_shipped_yaml_keys_are_declared(self):
        files = sorted((ROOT / 'config').glob('*.yaml')) + sorted((ROOT / 'config' / 'ppc').glob('*.yaml'))
        checked = 0
        for path in files:
            data = yaml.safe_load(path.read_text())
            for node, body in data.items():
                if not isinstance(body, dict) or 'ros__parameters' not in body:
                    continue  # site files are launch settings, not node parameters
                executable = node.split('/')[-1]
                checked += 1
                with self.subTest(file=path.name, node=node):
                    self.assertIn(executable, SOURCES)
                    self.assertEqual(undeclared(executable, flatten(body['ros__parameters'])), [])
        self.assertGreaterEqual(checked, 14)

    def test_launch_parameters_are_declared(self):
        built = []

        def node(**kw):
            built.append(kw)
            return kw

        with patch.object(pipeline, 'Node', side_effect=node), \
                patch.object(pipeline, 'executable_available', return_value=True), \
                patch.object(pipeline, 'load_params_dict', return_value={}), \
                patch.object(pipeline, 'playback_source', side_effect=lambda s, m, t: s):
            for receiver, spec in pipeline.RECEIVERS.items():
                for mode in ('decoder', 'driver'):
                    if spec[mode] is None:
                        continue
                    for role in ('rover', 'base'):
                        relay = {'relay_forward': 'tcpcli://localhost:5556'} if receiver == 'rtcm' \
                            else {'relay_listen': 'tcpsvr://:5556', 'novatel_relay_output': 'serial:///dev/x:115200'}
                        pipeline.receiver_node(role, receiver, mode, 'serial:///dev/x:115200',
                                               use_imu=True, use_gps_timestamp=True,
                                               reference_time=(2300, 0.0) if receiver == 'rtcm' else None,
                                               **relay)
            for kind, spec in pipeline.ESTIMATORS.items():
                deployment = {'params_file': '', 'base_position': [35.0, 137.0, 50.0],
                              'base_position_type': 'llh', 'lever_arm': [0.0, 0.0, 0.0]}
                pipeline.estimator_node(kind, deployment, gnss_solution_topic='/gnss/solution')
            pipeline.visualizer_node()
        executables = set()
        for kw in built:
            executable = kw['executable']
            executables.add(executable)
            keys = [k for p in kw['parameters'] if isinstance(p, dict) for k in p]
            with self.subTest(executable=executable, name=kw.get('name')):
                self.assertIn(executable, SOURCES)
                self.assertEqual(undeclared(executable, keys), [])
        self.assertEqual(executables, set(SOURCES) - {'single_point_positioning'})


if __name__ == '__main__':
    unittest.main()
