#!/usr/bin/env python3
"""Release contracts: packaging and public docs, source defaults and build patches,
and startup rejection of invalid configuration before any sensor data."""
import argparse
import ast
import os
import pathlib
import re
import resource
import subprocess
import tempfile
import types
import unittest
import xml.etree.ElementTree as ET
import yaml

parser=argparse.ArgumentParser();parser.add_argument('--bin-dir',required=True)
args,remaining=parser.parse_known_args()
ROOT=pathlib.Path(__file__).resolve().parents[1]
BIN=pathlib.Path(args.bin_dir).resolve()
resource.setrlimit(resource.RLIMIT_CORE,(0,0))

PUBLIC = [ROOT / name for name in (
    'README.md', 'docs/TOPIC_DATA_CONTRACT.md', 'docs/PARAMETER_NAMES.md',
    'docs/INSTALLED_API.md', 'msg/README.md', 'test/README.md')]

PUBLIC += list((ROOT / 'src/positioning').rglob('README.md'))

PUBLIC += list((ROOT / 'src').glob('*/README.md')) + [ROOT / 'launch/README.md']

def flatten(values, prefix=''):
    result = {}
    for key, value in values.items():
        name = prefix + key
        if isinstance(value, dict):
            result.update(flatten(value, name + '.'))
        else:
            result[name] = value
    return result

class ReleasePublication(unittest.TestCase):
    def test_E1_license_text_and_install_rule(self):
        notice = (ROOT / 'LICENSE').read_text()
        self.assertIn((ROOT / 'third_party/RTKLIB/license.txt').read_text(), notice)
        self.assertIn('modified copies', notice)
        cmake = (ROOT / 'CMakeLists.txt').read_text()
        self.assertRegex(cmake, r'install\(FILES LICENSE\s+DESTINATION share/\$\{PROJECT_NAME\}\)')
        licenses = [x.text for x in ET.parse(ROOT / 'package.xml').getroot().findall('license')]
        self.assertEqual(set(licenses), {'MIT', 'BSD-2-Clause'})

    def test_E2_metadata_factory_supports_both_constructor_signatures(self):
        tree = ast.parse((ROOT / 'test/test_converter_roundtrip.py').read_text())
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'topic_metadata')
        module = ast.Module(body=[function], type_ignores=[])
        for supports_id in [False, True]:
            calls = []
            def constructor(**kw):
                calls.append(dict(kw))
                if not supports_id and 'id' in kw:
                    raise TypeError('Humble signature has no id')
                return kw
            namespace = {'rosbag2_py': types.SimpleNamespace(TopicMetadata=constructor)}
            exec(compile(module, 'metadata_factory', 'exec'), namespace)
            value = namespace['topic_metadata'](name='/obs', type='T', serialization_format='cdr')
            self.assertEqual(value['name'], '/obs')
            self.assertEqual(len(calls), 1 if supports_id else 2)
            self.assertEqual('id' in value, supports_id)
        self.assertNotIn('Humble%20%7C', (ROOT / 'README.md').read_text())

    def test_E3_E4_contact_and_pinned_dependency(self):
        package = ET.parse(ROOT / 'package.xml').getroot()
        self.assertNotIn('MALIB', package.find('description').text)
        self.assertIn(package.find('maintainer').attrib['email'], (ROOT / 'README.md').read_text())
        ci = (ROOT / '.github/workflows/ci.yml').read_text()
        self.assertIn('cmake/build_fgo_dependency.sh', ci)
        helper=(ROOT/'cmake/build_fgo_dependency.sh').read_text()
        pin=re.search(r'[0-9a-f]{40}',helper).group(0)
        self.assertIn(pin, (ROOT / 'README.md').read_text())
        self.assertRegex(helper, r'[0-9a-f]{64}')
        self.assertNotIn('git checkout develop', (ROOT / 'README.md').read_text())

    def test_E5_no_dead_build_targets(self):
        cmake = (ROOT / 'CMakeLists.txt').read_text()
        for removed in ['BUILD_SYNTHETIC_SCENARIO_TOOL', 'ros2_rinex_writer.cpp']:
            self.assertNotIn(removed, cmake)
        package = (ROOT / 'package.xml').read_text()
        self.assertNotIn('ament_lint_', package)

    def test_Q4_C2_selected_defaults_match_code_yaml_and_docs(self):
        cases = [('single_point_positioning', 'tcp_port', 8001),
                 ('single_point_positioning', 'masks.elevation_deg', 15.0),
                 ('real_time_kinematic', 'tcp_port', 8002),
                 ('real_time_kinematic', 'topics.rover_observation', '/gnss/observation'),
                 ('gnss_fgo', 'max_age_s', 30.0)]
        for node, key, expected in cases:
            folder = {'gnss_fgo': 'fgo', 'single_point_positioning': 'spp', 'real_time_kinematic': 'rtk'}[node]
            source = ROOT / 'src/positioning' / folder / (node + '.cpp')
            doc = source.with_name('README.md').read_text()
            if node == 'gnss_fgo':
                source = ROOT / 'src/positioning/fgo/ros_node.hpp'
            params = flatten(yaml.safe_load((ROOT / 'config' / (node + '.yaml')).read_text())[node]['ros__parameters'])
            self.assertEqual(params[key], expected, key)
            match = re.search(r'(?:declare_parameter|declare_safe|load_vec3)(?:<[^;]+?>)?\("' + re.escape(key) + r'",\s*("[^"]*"|[^,)]+)', source.read_text())
            self.assertIsNotNone(match, key)
            self.assertEqual(yaml.safe_load(match[1].removeprefix('std::string(')), expected, key)
            self.assertRegex(doc, r'\| `' + re.escape(key) + r'` \| `?' + re.escape(str(expected)))
        header = (ROOT / 'include/gnss_ros_standardization/gnss_imu_kalman_filter.hpp').read_text()
        self.assertIn('topic_solution  = "/gnss/fusion/ekf_solution"', header)

    def test_D1_D2_D3_public_docs_are_portable_and_links_exist(self):
        private = ['GENERALIZATION_PLAN.md', 'RELIABILITY.md', 'COMMIT_PLAN.md',
                   'REVIEW_PLAN.md', 'AUDIT_EVIDENCE.md', 'REVIEW_FIXES_20261025.md',
                   'FINAL_ACCEPTANCE_20261027.md', 'docs/history/']
        for path in PUBLIC:
            text = path.read_text()
            self.assertNotRegex(text, r'[ぁ-んァ-ン一-龯]', str(path))
            self.assertNotRegex(text, r'/home/[^/\s]+/', str(path))
            for target in re.findall(r'\]\(([^)#]+)(?:#[^)]*)?\)', text):
                if '://' in target or target.startswith('mailto:'): continue
                self.assertFalse(any(x in target for x in private), (path, target))
                self.assertTrue((path.parent / target).exists(), (path, target))
        ignore = (ROOT / '.gitignore').read_text()
        for name in private[:5]: self.assertIn('/' + name, ignore)

    def test_D4_D5_D6_D8_current_contract(self):
        contract = (ROOT / 'docs/TOPIC_DATA_CONTRACT.md').read_text()
        self.assertIn('multiple-message bit 0', contract)
        self.assertIn('Incomplete epochs are never published', contract)
        self.assertIn('MSM4–7', (ROOT / 'src/decoders/README.md').read_text())
        self.assertIn('NaN', (ROOT / 'msg/GnssObservation.msg').read_text())
        self.assertIn('Week zero is invalid', (ROOT / 'msg/GnssObservations.msg').read_text())
        for path in PUBLIC:
            if path.parent.name in ('spp', 'rtk'):  # the nodes with the NMEA TCP server
                self.assertIn('all network interfaces', path.read_text())
                self.assertIn('tcp_port: 0', path.read_text())
        for name in ['src/positioning/README.md', 'src/drivers/README.md']:
            text = (ROOT / name).read_text()
            self.assertNotIn('time.reset_backjump_s', text)
            self.assertNotIn('observation.assembly_timeout_s', text)

class ReleaseSourceContracts(unittest.TestCase):
    def test_release_W5_rinex_patch_rejects_ambiguous_context(self):
        original=(ROOT/'third_party/RTKLIB/src/rinex.c').read_text()
        for ambiguous in [False,True]:
            with self.subTest(ambiguous=ambiguous),tempfile.TemporaryDirectory() as d:
                root=pathlib.Path(d);(root/'src').mkdir()
                (root/'src/rinex.c').write_text(original+('\n/* sigind_t *index, obsd_t *obs) */\n' if ambiguous else ''))
                script=root/'patch.cmake';script.write_text(f'set(RTKLIB_ROOT "{root}")\nset(CMAKE_CURRENT_BINARY_DIR "{root}")\ninclude("{ROOT}/cmake/rtklib_rinex.cmake")\n')
                result=subprocess.run(['cmake','-P',str(script)],capture_output=True,text=True)
                self.assertEqual(result.returncode==0,not ambiguous,result.stdout+result.stderr)
                if ambiguous:self.assertIn('expected 1 matches, found 2',result.stderr)

    def test_release_K1_csv_defaults_are_off(self):
        cfg=yaml.safe_load((ROOT/'config/gnss_imu_kalman_filter.yaml').read_text())['gnss_imu_kalman_filter']['ros__parameters']
        for key in ['sensors_log_enabled','state_log_enabled']:
            self.assertIs(cfg['csv'][key],False)
        text=(ROOT/'include/gnss_ros_standardization/gnss_imu_kalman_filter.hpp').read_text()
        self.assertRegex(text,r'state_log_enabled\s*=\s*false')
        self.assertRegex(text,r'sensors_log_enabled\s*=\s*false')

    def test_release_L3_U1_V8_driver_defaults_and_disclosure(self):
        ubx=(ROOT/'src/drivers/ubx_driver_node.cpp').read_text()
        self.assertRegex(ubx,r'enable_esf_raw\s*\{false\}')

    def test_release_L7_removed_unused_framer_fields(self):
        for family in ['ubx','sbf','novatel']:
            text=(ROOT/f'src/decoders/{family}_decoder_node.cpp').read_text()
            self.assertNotRegex(text,r'\b(?:ubx|sbf|oem4)_(?:state|len|pos|hdr_rem|hdr_pos|body_pos)_\b')

    def test_release_F3_F4_F6_F8_F14_F17_docs(self):
        fgo=(ROOT/'src/positioning/fgo/README.md').read_text()
        self.assertNotIn('0 disables the bound',fgo)
        self.assertNotIn('official GNSS',fgo)
        ekf=(ROOT/'src/positioning/ekf/README.md').read_text()
        self.assertIn('output_reference_frame=imu',ekf);self.assertNotIn('IMU body (FRD)',ekf)
        for name in ['gnss_fgo','gnss_imu_fgo']:
            source=(ROOT/f'src/positioning/fgo/{name}.cpp').read_text()
            self.assertNotIn('Cap scope:',source);self.assertNotIn('official GNSS factors',source)
        self.assertNotIn('non-zero off-diagonal in',(ROOT/'include/gnss_ros_standardization/ubx_protocol.hpp').read_text())

    def test_release_L5_X4_converter_help_and_tables(self):
        source=(ROOT/'src/converter/rinex_to_rosbag.cpp').read_text()
        self.assertNotIn('test/ppc_eval/verify_bag_reference.py',source)
        self.assertTrue((ROOT/'test/tools/verify_bag_reference.py').exists())
        docs=(ROOT/'src/converter/README.md').read_text()
        for flag in ['--topic-obs','--topic-nav','--topic-pos','--timestamp-scale','--imu','--imu-frame','--reference','--topic-reference']:
            self.assertRegex(docs,r'`'+re.escape(flag)+r'[` ]')

    def test_release_F11_navigation_does_not_share_observation_loss_callback(self):
        # Navigation losses are counted separately from observation losses.
        text=(ROOT/'src/positioning/fgo/ros_node.hpp').read_text()
        text=re.sub(r'//[^\n]*','',text)  # comments may themselves contain ';'
        def call(name):  # the whole create_subscription(...) call, lambdas included
            m=re.search(name+r' = create_subscription<[^>]+>\((.*?)_options\);',text,re.S)
            self.assertIsNotNone(m,name);return m[0]
        self.assertTrue(call('nav_sub_').endswith('nav_options);'))
        for subscription in ['rover_sub_','base_sub_']:
            self.assertTrue(call(subscription).endswith('obs_options);'),subscription)
        self.assertIn('nav_options.event_callbacks.message_lost_callback',text)

    def test_release_W4_spp_failure_warnings_are_throttled(self):
        text=(ROOT/'src/positioning/spp/single_point_positioning.cpp').read_text()
        for message in ['Not enough satellites observed.', 'SPP failed:']:
            self.assertRegex(text,r'RCLCPP_WARN_THROTTLE\(get_logger\(\), \*get_clock\(\), 5000,\s*"'+re.escape(message))

class ReleaseStartup(unittest.TestCase):
    def reject(self,node,parameters,expected):
        with tempfile.TemporaryDirectory() as directory:
            command=[str(BIN/node),'--ros-args']
            for value in parameters:command+=['-p',value]
            env=dict(os.environ,ROS_LOG_DIR=directory,ROS_DOMAIN_ID=str(80+os.getpid()%20),
                     ROS_LOCALHOST_ONLY='1',LD_LIBRARY_PATH=str(BIN)+os.pathsep+os.environ.get('LD_LIBRARY_PATH',''))
            run=subprocess.run(command,env=env,capture_output=True,text=True,timeout=15)
            self.assertNotEqual(run.returncode,0,run.stdout+run.stderr)
            self.assertIn(expected,run.stdout+run.stderr)

    def test_local_origin_rejects_invalid_or_ignored_settings(self):
        nodes=['ubx_decoder_node','sbf_decoder_node','novatel_decoder_node','ubx_driver_node',
               'sbf_driver_node','novatel_driver_node','single_point_positioning',
               'real_time_kinematic','gnss_imu_kalman_filter','gnss_fgo','gnss_imu_fgo']
        cases=[(['local_origin.mode:=manual'],'local_origin.mode must be auto or fixed'),
               (['local_origin.postype:=enu'],'local_origin.postype must be llh or ecef'),
               (['local_origin.pos:=[35.0,139.0,10.0]'],'local_origin.mode is auto'),
               (['local_origin.mode:=fixed','local_origin.pos:=[91.0,0.0,0.0]'],'latitude must be within'),
               (['local_origin.mode:=fixed','local_origin.postype:=ecef'],'centre of the Earth')]
        checked=0
        for node in nodes:
            if not (BIN/node).exists():continue
            extra=['ant2.pos:=[35.0,139.0,10.0]'] if node=='real_time_kinematic' else []
            if 'fgo' in node:extra=['base_position.postype:=llh','base_position.pos:=[35.0,139.0,10.0]']
            if node.endswith('_driver_node'):extra+=['configure_on_startup:=false']
            for params,expected in cases:
                with self.subTest(node=node,expected=expected):self.reject(node,extra+params,expected)
            checked+=1
        self.assertGreaterEqual(checked,9)

    def test_ubx_driver_rejects_unknown_generation_and_dynamic_model(self):
        if not (BIN/'ubx_driver_node').exists(): self.skipTest('ubx_driver_node not built')
        self.reject('ubx_driver_node',['configure_on_startup:=false','generation:=G8'],'generation must be G9 or G10')
        self.reject('ubx_driver_node',['configure_on_startup:=false','dynamic_model:=car'],'dynamic_model must be one of')

    def test_release_C1_base_required(self):
        self.reject('real_time_kinematic',[], 'ant2.pos')
        self.reject('real_time_kinematic',['ant2.postype:=rtcm'],'ant2.postype')
        for node in ['gnss_fgo','gnss_imu_fgo']:
            if not (BIN/node).exists():continue  # optional executable; core CI also checks RTK
            self.reject(node,[], 'surveyed terrestrial coordinate')

    def test_release_F4_F12_F19_K2_W3_invalid_modes(self):
        cases=[('single_point_positioning',['excluded_satellites:=[C99]'],'excluded_satellites'),
               ('real_time_kinematic',['ant2.pos:=[35.0,139.0,10.0]','excluded_satellites:=[C99]'],'excluded_satellites')]
        for node in ['gnss_fgo','gnss_imu_fgo']:
            if not (BIN/node).exists():continue
            base=['base_position.postype:=llh','base_position.pos:=[35.0,139.0,10.0]']
            for key in ['base_position.postype','fixed_origin.postype']:
                cases.append((node,base+[key+':=typo'],'position type' if key == 'base_position.postype' else key))
        if (BIN/'gnss_imu_fgo').exists():
            for key in ['local_origin.mode','local_origin.postype','time.gnss_epoch_source']:
                cases.append(('gnss_imu_fgo',['base_position.postype:=llh','base_position.pos:=[35.0,139.0,10.0]',key+':=typo'],key))
            cases.append(('gnss_imu_fgo',['base_position.postype:=llh','base_position.pos:=[35.0,139.0,10.0]','imu_orientation:=[0.0,0.0]'],'imu_orientation'))
        if (BIN/'gnss_imu_kalman_filter').exists():
            for key in ['local_origin.mode','gnss.update_mode','wheel.topic_type','wheel.mode']:
                cases.append(('gnss_imu_kalman_filter',[key+':=typo'],key))
        for node,params,expected in cases:
            with self.subTest(node=node,key=expected):self.reject(node,params,expected)

    def test_release_V2_invalid_requested_rates(self):
        for node,key in [('ubx_driver_node','rate_hz'),('sbf_driver_node','rate_hz'),('novatel_driver_node','rate_hz')]:
            for rate in [0,-1,10000]:
                with self.subTest(node=node,rate=rate):self.reject(node,[f'{key}:={rate}'],'rate')

    def test_release_C2_K3_V4_documented_defaults(self):
        cases=[('rtk/real_time_kinematic','tcp_port','8002'),
               ('rtk/real_time_kinematic','topics.rover_observation','/gnss/observation'),
               ('spp/single_point_positioning','tcp_port','8001'),
               ('fgo/gnss_fgo','max_age_s','30.')]
        for name,key,value in cases:
            source=ROOT/'src/positioning'/(name+'.cpp')
            doc=(source.parent/'README.md').read_text()
            if name.startswith('fgo/'):  # both FGO mains declare through the shared node
                source=source.parent/'ros_node.hpp'
            text=source.read_text()
            declarations=re.findall(r'(?:declare_parameter|declare_safe)(?:<[^>]+>)?\("'+re.escape(key)+r'",\s*([^;]+);',text)
            self.assertEqual(len(declarations),1,key)
            self.assertIn(value,declarations[0]);self.assertRegex(doc,r'\| `'+re.escape(key)+r'` \| `?'+re.escape(value))
        imu=(ROOT/'src/positioning/fgo/README.md').read_text()
        self.assertIn('`<topics.solution>_odom`',imu)
        ekf=(ROOT/'src/positioning/ekf/README.md').read_text()
        self.assertIn('/gnss/fusion/ekf_solution',ekf);self.assertIn('<topics.solution>_odom',ekf)
        nov=(ROOT/'src/drivers/novatel_driver_node.cpp').read_text()
        self.assertRegex(nov,r'receiver_port\s*=\s*"THISPORT"|receiver_port\s*\{"THISPORT"\}')

    def test_release_Q1_Q6_removed_public_parameters(self):
        retired=['diagnostics.period_s','ephemeris.history_per_satellite','time.reset_backjump_s',
                 'solution.assembly_timeout_s','observation.assembly_timeout_s','measurement.base_reuse_factor']
        for directory in ['src','config']:
            for path in (ROOT/directory).rglob('*'):
                if path.suffix not in ['.cpp','.yaml']:continue
                text=path.read_text()
                for name in retired:
                    self.assertNotIn('"'+name+'"',text,str(path))
                    self.assertNotIn(name+':',text,str(path))


if __name__=='__main__':unittest.main(argv=[__file__]+remaining)
