#!/usr/bin/env python3
import pathlib
import importlib.util
import tempfile
import sys
import unittest
from unittest.mock import patch
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'launch'))
import gnss_pipeline_common as pipeline
from launch import LaunchContext
from launch_ros.utilities import normalize_parameters, evaluate_parameters
spec=importlib.util.spec_from_file_location("pipeline_launch", pathlib.Path(pipeline.__file__).with_name("pipeline.launch.py"))
launch_file=importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_file)

class LaunchSettings(unittest.TestCase):
    def test_native_fgo_requires_sensor_time(self):
        settings=self.resolve(pipeline='gnss_imu_fgo',use_gps_timestamp='false')
        with self.assertRaisesRegex(pipeline.PipelineError,'requires sensor timestamps'):
            launch_file.build_gnss_imu(settings)

    def test_P1_4_whole_path_credentials_and_startup_summary(self):
        cases = [
            ('ntrip://user:pa/ss@caster:2101/MNT', 'ntrip://***@caster:2101/MNT'),
            ('ntrip://us@er:p@ss@caster:2101/MNT', 'ntrip://***@caster:2101/MNT'),
            ('ntrip://user:pw@caster/M@NT', 'ntrip://***@NT'),
            ('tcpcli://host/path@name', 'tcpcli://***@name'),
            ('user:pa/ss@caster/MNT', '***@caster/MNT'),
            ('tcpcli://host:123/path', 'tcpcli://host:123/path'),
            ('serial:///dev/ttyUSB0:115200', 'serial:///dev/ttyUSB0:115200'),
            ('host/path', 'host/path')]
        for uri, expected in cases:
            with self.subTest(uri=uri):
                self.assertEqual(pipeline._mask_uri(uri), expected)
                values = {'pipeline': 'gnss_fgo', 'rover_source': uri,
                          'rover_receiver': 'ubx', 'rover_mode': 'decoder'}
                settings = pipeline.Settings(values, {}, '')
                with patch('launch.actions.LogInfo', side_effect=lambda **kw: kw):
                    message = pipeline.summary_actions(settings)[0]['msg']
                self.assertIn(expected, message)
                if expected != uri:
                    self.assertNotIn(uri, message)

    def test_release_Q4_directional_rtcm_parameters(self):
        with patch.object(pipeline, 'Node', side_effect=lambda **kw: kw):
            for family in ['ubx', 'sbf', 'novatel']:
                for mode in ['driver', 'decoder']:
                    node = pipeline.receiver_node('rover', family, mode,
                        'serial:///dev/example:115200', relay_listen='tcpsvr://:5556',
                        novatel_relay_output='serial:///dev/correction:115200')
                    params = node['parameters'][-1]
                    self.assertEqual(params['rtcm_input.listen'], 'tcpsvr://:5556')
                    self.assertNotIn('rtcm_output.destination', params)
                    if mode == 'driver': self.assertTrue(params['rtcm_input.enable'])
                    if family == 'novatel':
                        self.assertEqual(params['rtcm_input.receiver_port'], 'THISPORT')
                        self.assertEqual(params['rtcm_input.receiver_stream'], 'serial:///dev/correction:115200')
            node = pipeline.receiver_node('base', 'rtcm', 'decoder',
                'tcpcli://localhost:9000', relay_forward='tcpcli://localhost:5556')
            self.assertEqual(node['parameters'][-1]['rtcm_output.destination'], 'tcpcli://localhost:5556')
            self.assertNotIn('rtcm_input.listen', node['parameters'][-1])


    def test_release_W2_record_estimator_odometry(self):
        for kind in ['ekf','gnss_imu_fgo']:
            self.assertIn('/custom/solution_odom',pipeline.record_topics(estimator_topic='/custom/solution',estimator_kind=kind))
        self.assertNotIn('/custom/solution_odom',pipeline.record_topics(estimator_topic='/custom/solution',estimator_kind='rtk'))

    def test_fallback_gps_is_forwarded_to_both_sources(self):
        values={'pipeline':'gnss_imu_fgo','rover_receiver':'ubx','rover_mode':'decoder','rover_source':'file://rover.ubx',
                'base_receiver':'rtcm','base_source':'file://base.rtcm3','use_gps_timestamp':True}
        settings=pipeline.Settings(values,{'use_gps_timestamp':'default'},'')
        with patch.object(pipeline,'Node',side_effect=lambda **kw:kw):
            nodes,has_base=pipeline.source_nodes(settings)
        self.assertTrue(has_base)
        for node in nodes:self.assertIs(node['parameters'][-1]['use_gps_timestamp'],True)
    def test_explicit_false_and_unspecified_keep_precedence(self):
        for value,origin in [(False,'command line'),(False,'site file'),(None,'unset')]:
            values={'pipeline':'gnss_imu_fgo','rover_receiver':'ubx','rover_mode':'decoder','rover_source':'file://rover.ubx',
                    'use_gps_timestamp':value}
            settings=pipeline.Settings(values,{'use_gps_timestamp':origin},'')
            with patch.object(pipeline,'Node',side_effect=lambda **kw:kw):
                nodes,_=pipeline.source_nodes(settings)
            params=nodes[0]['parameters'][-1]
            if value is None:self.assertNotIn('use_gps_timestamp',params)
            else:self.assertIs(params['use_gps_timestamp'],False)

    def resolve(self, site=None, **overrides):
        context=LaunchContext()
        context.launch_configurations.update({name:'' for name in pipeline.ARGUMENT_NAMES})
        context.launch_configurations.update(overrides)
        with patch.object(pipeline,'load_site',return_value=(site or {},'test-site')):
            return pipeline.resolve_settings(context)

    def test_cli_site_fallback_precedence_and_invalid_booleans(self):
        site={'imu':{'enabled':True,'use_gps_timestamp':True}}
        settings=self.resolve(site,pipeline='gnss_imu_fgo',use_gps_timestamp='false')
        self.assertFalse(settings['use_gps_timestamp']);self.assertTrue(settings['use_imu'])
        self.assertEqual(settings.source('use_gps_timestamp'),'command line')
        for value in ['flase','maybe',2]:
            with self.subTest(value=value),self.assertRaises(pipeline.PipelineError):
                self.resolve({'imu':{'enabled':value}})

    def test_nonfinite_triplets_and_relay_port_are_rejected(self):
        for value in ['nan,1,2','1,inf,2',[0,float('-inf'),0]]:
            with self.subTest(value=value),self.assertRaises(pipeline.PipelineError):
                pipeline.parse_triplet(value,'lever_arm')
        for value in ['-1','65536','3.5','nan']:
            with self.subTest(value=value),self.assertRaises(pipeline.PipelineError):
                self.resolve(relay_port=value)
        for value in ['0','65535']:
            self.assertEqual(self.resolve(relay_port=value)['relay_port'],int(value))

    def test_base_position_range_and_explicit_zero_llh(self):
        for pos in [[float('nan'),1,2],[91.,0.,0.],[0.,181.,0.]]:
            with self.subTest(pos=pos),self.assertRaises(pipeline.PipelineError):
                pipeline.validate_base_position(pos,'llh','test')
        pipeline.validate_base_position([0.,0.,0.],'llh','command line',allow_zero_llh=True)
        with self.assertRaises(pipeline.PipelineError):
            pipeline.validate_base_position([0.,0.,0.],'ecef','command line',allow_zero_llh=True)

    def test_parameter_yaml_selects_node_and_namespace(self):
        with tempfile.TemporaryDirectory() as directory:
            path=pathlib.Path(directory)/'nodes.yaml'
            path.write_text("""other_node:
  ros__parameters: {rate: 99}
'/**':
  ros__parameters: {use_gps_timestamp: false, rate: 1}
ubx_driver_node:
  ros__parameters: {rate: 5, messages: {esf_raw: false}}
east:
  ubx_driver_node:
    ros__parameters: {rate: 10, messages.esf_raw: true}
""")
            with patch.object(pipeline,'Node',side_effect=lambda **kw:kw):
                east=pipeline.receiver_node('rover','ubx','driver','file://east',params_file=str(path),name='east')
                west=pipeline.receiver_node('rover','ubx','driver','file://west',params_file=str(path),name='west',use_gps_timestamp=True)
            def effective(node):
                merged={}
                for block in evaluate_parameters(LaunchContext(),normalize_parameters(node['parameters'])):
                    merged.update(block)
                return merged
            first,second=effective(east),effective(west)
            self.assertEqual(first['rate'],10);self.assertEqual(second['rate'],5)
            self.assertTrue(first['messages.esf_raw']);self.assertFalse(second['messages.esf_raw'])
            self.assertFalse(first['use_gps_timestamp']);self.assertTrue(second['use_gps_timestamp'])
            self.assertEqual(first['topics.observation'],'/east/gnss/observation')
            self.assertEqual(second['topics.observation'],'/west/gnss/observation')
            self.assertEqual(east['namespace'],'east')
            path.write_text('other_node:\n  ros__parameters: {rate: 99}\n')
            with self.assertRaises(pipeline.PipelineError):
                pipeline.load_params_dict(str(path),'ubx_driver_node','east')

    def test_flat_estimator_yaml_matches_validation_and_node_values(self):
        with tempfile.TemporaryDirectory() as directory:
            path=pathlib.Path(directory)/'rtk.yaml'
            path.write_text("""wrong_node:
  ros__parameters: {ant2.pos: [0., 0., 0.]}
real_time_kinematic:
  ros__parameters: {ant2.postype: llh, ant2.pos: [35.0, 139.0, 100.0]}
""")
            settings=self.resolve(pipeline='rtk',params_file=str(path))
            deployment=pipeline.resolve_estimator_deployment('rtk',settings)
            self.assertEqual(deployment['base_position_value'],[35.,139.,100.])
            with patch.object(pipeline,'Node',side_effect=lambda **kw:kw),patch.object(pipeline,'executable_available',return_value=True):
                node=pipeline.estimator_node('rtk',deployment)
            self.assertEqual(node['parameters'][0]['ant2.pos'],[35.,139.,100.])

    def test_onboard_base_receives_timestamp_policy(self):
        for value in ['true','false']:
            settings=self.resolve(pipeline='onboard_rtk',rover_source='file://rover.ubx',
                base_source='file://base.rtcm3',use_gps_timestamp=value)
            with patch.object(pipeline,'Node',side_effect=lambda **kw:kw),patch.object(launch_file,'TimerAction',side_effect=lambda **kw:kw):
                actions,_,_,_=launch_file.build_onboard_rtk(settings)
            rover=actions[0];base=actions[-1]['actions'][0]
            for node in [rover,base]:
                self.assertEqual(node['parameters'][-1]['use_gps_timestamp'],value=='true')

    def test_multi_rover_recording_uses_distinct_topics_and_no_duplicate_base_nav(self):
        settings=self.resolve({'rovers':[{'name':'east','receiver':'ubx','source':'file://a'},
                                          {'name':'west','receiver':'sbf','source':'file://b','imu':True}],
                               'base':{'source':'file://base'},'imu':{'use_gps_timestamp':True}})
        with patch.object(pipeline,'Node',side_effect=lambda **kw:kw):
            nodes,has_base=pipeline.source_nodes(settings)
        self.assertTrue(has_base)
        for node in nodes:self.assertTrue(node['parameters'][-1]['use_gps_timestamp'])
        topics=pipeline.record_topics(with_base=True,rovers=settings.rovers)
        self.assertEqual(len(topics),len(set(topics)))
        self.assertIn('/east/gnss/observation',topics);self.assertIn('/west/gnss/imu/data_raw',topics)
        self.assertIn('/base/gnss/ephemeris',topics)
        for prefix in ['/east','/west','/base']:
            self.assertNotIn(prefix+'/gnss/receiver_frames',topics)
        self.assertNotIn('receiver_frame_topic',nodes[0]['parameters'][-1])
        with self.assertRaises(pipeline.PipelineError):
            pipeline._resolve_rovers([{'name':'east\n','receiver':'ubx','source':'file://a'}],'test',False)

    # A-22: a historical RTCM base log needs its GPS week (MSM carries none).
    def test_historical_rtcm_base_receives_reference_week(self):
        settings=self.resolve({'base':{'source':'file://base.rtcm3','reference_week':1709,
                                        'reference_tow':345600}},
                              pipeline='rtk',rover_source='file://rover.ubx')
        with patch.object(pipeline,'Node',side_effect=lambda **kw:kw):
            nodes,_=pipeline.source_nodes(settings)
        base=nodes[-1]['parameters'][-1]
        self.assertEqual((base['reference_week'],base['reference_tow']),(1709,345600.0))
        settings=self.resolve(pipeline='rtk',rover_source='file://rover.ubx',base_source='file://b')
        with patch.object(pipeline,'Node',side_effect=lambda **kw:kw):
            nodes,_=pipeline.source_nodes(settings)
        self.assertNotIn('reference_week',nodes[-1]['parameters'][-1])
        for site,cli in [({'base':{'receiver':'ubx','source':'file://b','reference_week':1709}},{}),
                         ({'base':{'source':'file://b','reference_tow':10}},{}),
                         ({},{'base_source':'file://b','base_reference_tow':'604800'}),
                         ({},{'base_source':'file://b','base_reference_week':'-1'})]:
            with self.subTest(site=site,cli=cli),self.assertRaises(pipeline.PipelineError),\
                 patch.object(pipeline,'Node',side_effect=lambda **kw:kw):
                pipeline.source_nodes(self.resolve(site,pipeline='rtk',rover_source='file://r',**cli))

    # A-21: a file:// decoder listens on loopback and its file is played back
    # only once every consumer of its topics has subscribed.
    def test_file_sources_play_back_after_their_consumers(self):
        root=pathlib.Path(__file__).resolve().parents[1]
        settings=self.resolve(pipeline='rtk',rover_source='file://rover.ubx',base_source='file://base.rtcm3',
                              base_position='35,139,100',record='true')
        with patch.object(pipeline,'config',side_effect=lambda name:str(root/'config'/name)),\
             patch.object(pipeline,'executable_available',return_value=True),\
             patch.object(pipeline,'Node',side_effect=lambda **kw:kw),\
             patch.object(pipeline,'ExecuteProcess',side_effect=lambda **kw:kw):
            pipeline.take_playbacks()
            actions,_,has_base,output=launch_file.BUILDERS['rtk'](settings)
            plays=pipeline.playback_actions(pipeline.take_playbacks(),
                                            pipeline.stream_consumers(settings,has_base,output))
        nodes=[a for a in actions if isinstance(a,dict) and 'executable' in a]
        rover=next(n for n in nodes if n['executable']=='ubx_decoder_node')
        self.assertRegex(rover['parameters'][-1]['stream_path'],r'^tcpsvr://127\.0\.0\.1:\d+$')
        self.assertEqual(len(plays),2)
        waits={tuple(p['cmd'][4:]) for p in plays}
        # rtk and the recorder both read the observations and rover ephemerides;
        # topics nobody reads (here the IMU) are not waited for.
        self.assertEqual(waits,{('/gnss/ephemeris=2','/gnss/nmea_solution=1','/gnss/observation=2'),
                                ('/base/gnss/ephemeris=1','/base/gnss/observation=2')})
        for p in plays: self.assertTrue(p['cmd'][2].endswith('rover.ubx') or p['cmd'][2].endswith('base.rtcm3'))
        # RTKLIB file options keep RTKLIB's own file stream
        self.assertEqual(pipeline.playback_source('file://x.ubx::T','decoder',{}),'file://x.ubx::T')
        self.assertEqual(pipeline.playback_source('file://x.ubx','driver',{}),'file://x.ubx')

    def test_all_pipeline_builders_compose_with_existing_topics(self):
        root=pathlib.Path(__file__).resolve().parents[1]
        for kind in pipeline.PIPELINES:
            settings=self.resolve(pipeline=kind,rover_source='file://rover.ubx',
                base_source='file://base.rtcm3',base_position='35,139,100')
            with self.subTest(kind=kind),patch.object(pipeline,'config',side_effect=lambda name:str(root/'config'/name)),patch.object(pipeline,'executable_available',return_value=True),patch.object(pipeline,'Node',side_effect=lambda **kw:kw),patch.object(launch_file,'TimerAction',side_effect=lambda **kw:kw):
                actions,_,_,_=launch_file.BUILDERS[kind](settings)
                self.assertTrue(actions)

    def test_terminal_ctrl_c_reaches_the_recorder_once(self):
        # A terminal Ctrl-C signals the whole process group and launch forwards a
        # second SIGINT; rosbag2 exits non-zero on the second one. The stand-in
        # recorder below fails the same way. Through gnss_record.py it must see
        # one SIGINT and exit 0; the recorder action must use the wrapper.
        import os, signal, subprocess, time
        recorder = ('import signal,sys,time\n'
                    'n=[0]\n'
                    'def h(s,f): n[0]+=1\n'
                    'signal.signal(signal.SIGINT,h)\n'
                    'print("ready",flush=True)\n'
                    'while n[0]==0: time.sleep(.01)\n'
                    'time.sleep(.5)\n'
                    'sys.exit(0 if n[0]==1 else 1)\n')
        wrapper = pathlib.Path(pipeline.__file__).with_name('gnss_record.py')
        def ctrl_c(command):
            p = subprocess.Popen(command, start_new_session=True, stdout=subprocess.PIPE, text=True)
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:  # the stand-in reports its handler
                line = p.stdout.readline()
                if line.strip() == 'ready': break
            os.killpg(p.pid, signal.SIGINT)   # terminal: whole foreground group
            time.sleep(.05)
            p.send_signal(signal.SIGINT)      # launch: forwarded to its child
            return p.wait(timeout=10)
        self.assertEqual(ctrl_c([sys.executable, '-c', recorder]), 1)
        self.assertEqual(ctrl_c([sys.executable, str(wrapper), sys.executable, '-c', recorder]), 0)
        action = pipeline.record_action(['/gnss/observation'], 'bag')
        from launch.utilities import perform_substitutions
        cmd = [perform_substitutions(LaunchContext(), part) for part in action.cmd]
        self.assertEqual(cmd[1:5], [str(wrapper), 'ros2', 'bag', 'record'])

if __name__=='__main__':unittest.main()
