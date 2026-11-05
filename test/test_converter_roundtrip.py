#!/usr/bin/env python3
"""Real converter executables + CDR bags; no receiver or external data needed."""
import argparse
import copy
import math
import os
import pathlib
import sqlite3
import sys
import subprocess
import tempfile
import unittest
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message

parser = argparse.ArgumentParser()
parser.add_argument('--bin-dir', required=True)
args, _ = parser.parse_known_args()
# Child executables must use this build's libraries, even if an older workspace
# install was sourced for Python message imports.
os.environ['LD_LIBRARY_PATH']=str(pathlib.Path(args.bin_dir).resolve())+os.pathsep+os.environ.get('LD_LIBRARY_PATH','')
sys.path.insert(0, str(pathlib.Path(args.bin_dir).resolve()/"rosidl_generator_py"))
from gnss_ros_standardization.msg import GnssSolution, GnssObservations, GnssObservation, GnssEphemerides, GnssEphemeris, GlonassEphemeris
assert GnssEphemeris.get_fields_and_field_types()["iode"] == "uint16", "stale message generation"

def topic_metadata(**fields):
    """Humble has no id argument; Jazzy accepts it."""
    try:
        return rosbag2_py.TopicMetadata(id=0, **fields)
    except TypeError:
        return rosbag2_py.TopicMetadata(**fields)

class ConverterRoundtrip(unittest.TestCase):
    def test_release_X1_physical_pitch_includes_body_motion_not_only_grade(self):
        import numpy as np
        from reference_attitude_check import check_attitude
        t=np.arange(0.,120.,.1);speed=10.+np.sin(t*.07)
        course=t*.08;grade=.01*np.sin(t*.09)
        v=np.c_[speed*np.cos(course),speed*np.sin(course),speed*np.tan(grade)]
        pitch=-grade-.04+.035*np.sin(t*.4);roll=.03*np.sin(t*.2)
        rpy=np.c_[roll,pitch,course]
        acc=np.c_[np.gradient(speed,t)-9.81*np.sin(pitch),
                  speed*np.gradient(course,t)+9.81*np.cos(pitch)*np.sin(roll),
                  np.gradient(v[:,2],t)+9.81*np.cos(pitch)*np.cos(roll)]
        result=check_attitude(t,v,rpy,t,acc)
        self.assertGreater(result['pitch']['correlation'],.99)
        self.assertLess(result['pitch_climb_diagnostic']['correlation'],.9)
        rpy[:,1]*=-1
        with self.assertRaises(AssertionError):check_attitude(t,v,rpy,t,acc)

    def test_release_X2_X3_X5_transaction_covariance_and_bad_cdr(self):
        import numpy as np
        with tempfile.TemporaryDirectory() as directory:
            root=pathlib.Path(directory); pos=root/'input.pos'
            pos.write_text('% GPST latitude(deg) longitude(deg) height(m) Q ns sdn sde sdu sdne sdeu sdun age ratio\n'
                           '2300 100.000 0 0 100 1 12 1 2 3 0 0 0 0 4\n')
            bag=root/'bag'
            self.converter('pos_to_rosbag','--pos',pos,'--out',bag,'--storage','sqlite3')
            reader=rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
            _,data,_=reader.read_next(); sol=deserialize_message(data,GnssSolution);del reader
            # At lat/lon=0, Up=X, East=Y, North=Z: independent axis permutation.
            np.testing.assert_allclose(np.array(sol.pos_enu_cov).reshape(3,3),np.diag([4,1,9]))
            np.testing.assert_allclose(np.array(sol.pos_cov_ecef).reshape(3,3),np.diag([9,4,1]))
            with sqlite3.connect(next(bag.glob('*.db3'))) as db:
                db.execute('UPDATE messages SET data=?',(b'bad-cdr',))
            out=root/'kept.pos';out.write_text('existing output must survive\n')
            result=self.converter('rosbag_to_pos','--bag',bag,'--out',out,success=False)
            self.assertIn('Rejected 1 message',result.stderr)
            self.assertEqual(out.read_text(),'existing output must survive\n')
            self.assertEqual(list(root.glob('*.tmp-*')),[])
            # Existing bags must never be replaced on failed or repeated conversion.
            before={str(p.relative_to(bag)):p.read_bytes() for p in bag.rglob('*') if p.is_file()}
            self.converter('pos_to_rosbag','--pos',pos,'--out',bag,success=False)
            self.assertEqual(before,{str(p.relative_to(bag)):p.read_bytes() for p in bag.rglob('*') if p.is_file()})
            # Failure after staging, with an unavailable storage plugin, cleans only its own output.
            failed=root/'failed'
            self.converter('pos_to_rosbag','--pos',pos,'--out',failed,'--storage','does_not_exist',success=False)
            self.assertFalse(failed.exists());self.assertEqual(list(root.glob('*.tmp-*')),[])
            for header in ["% GPST latitude(dms) longitude(dms) height(m)", '% pos format : lat/lon/height geodetic/unused /geodetic']:
                pos.write_text(header+'\n2300 100 0 0 100 1 12\n')
                self.converter('pos_to_rosbag','--pos',pos,'--out',failed,success=False)
                self.assertFalse(failed.exists())

    def test_release_T2_file_tools_ignore_unwritable_ros_log_dir(self):
        previous=os.environ.get('ROS_LOG_DIR')
        os.environ['ROS_LOG_DIR']='/proc/gnss-unwritable-log'
        try:
            self.test_release_N2_invalid_time_does_not_enter_rinex_span()
            self.test_reference_odometry_position_and_child_frame_velocity()
            self.test_pos_bag_pos_scales_storage_and_failures()
        finally:
            if previous is None:os.environ.pop('ROS_LOG_DIR',None)
            else:os.environ['ROS_LOG_DIR']=previous

    def test_release_L6_X4_unknown_pos_arguments_fail(self):
        for executable in ['pos_to_rosbag','rosbag_to_pos']:
            result=self.converter(executable,'--timestamp_scale','gpst',success=False)
            self.assertIn('unknown argument',result.stderr)

    def test_release_N2_invalid_time_does_not_enter_rinex_span(self):
        with tempfile.TemporaryDirectory() as directory:
            root=pathlib.Path(directory);bag=root/'input'
            writer=rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
            metadata=dict(name='/gnss/observation',type='gnss_ros_standardization/msg/GnssObservations',serialization_format='cdr')
            try: topic=rosbag2_py.TopicMetadata(id=0,**metadata)
            except TypeError: topic=rosbag2_py.TopicMetadata(**metadata)
            writer.create_topic(topic)
            epochs=[(0,0.),(2300,100.),(2300,110.),(0,0.),(2300,float('nan')),(2300,604800.)]
            for i,(week,tow) in enumerate(epochs):
                m=GnssObservations(week=week,tow=tow,observations=[GnssObservation(
                    satid='G01',system='G',prn=1,code_str='1C',code=1,p=21000000.,l=110000000.,d=0.,snr=40.)])
                writer.write('/gnss/observation',serialize_message(m),1707004882000000000+i)
            del writer
            result=self.converter('rosbag_to_rinex','--bag',bag,'--obs',root/'out.obs')
            self.assertIn('skipped 4 observation epochs',result.stderr)
            output=(root/'out.obs').read_text()
            self.assertNotIn('1980',output)
            self.assertEqual(sum(l.startswith('>') for l in output.splitlines()),2)
            self.assertTrue(all('2024' in l for l in output.splitlines() if 'TIME OF' in l))

    def test_observation_epoch_merge_order_and_transaction(self):
        def epoch(tow,sat='G01',p=21000000.):
            return GnssObservations(week=2300,tow=float(tow),observations=[GnssObservation(
                satid=sat,system='G',prn=int(sat[1:]),code_str='1C',code=1,
                p=p,l=float('nan'),d=0.,snr=40.)])
        cases=[('merge',[epoch(100),epoch(100,'G02'),epoch(100)],True),
               ('late',[epoch(100),epoch(104),epoch(95)],False),
               ('conflict',[epoch(100),epoch(100,p=21000001.)],False)]
        first=epoch(100);second=epoch(100,'G02')
        first.observations[0].satid='';second.observations[0].satid=''
        cases += [('merge_fallback',[first,second],True),('alias',[first,epoch(100)],True)]
        for label,messages,ok in cases:
            with self.subTest(case=label), tempfile.TemporaryDirectory() as directory:
                root=pathlib.Path(directory);bag=root/'input'
                writer=rosbag2_py.SequentialWriter()
                writer.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
                writer.create_topic(topic_metadata(name='/gnss/observation',
                    type='gnss_ros_standardization/msg/GnssObservations',serialization_format='cdr'))
                for i,msg in enumerate(messages):
                    writer.write('/gnss/observation',serialize_message(msg),1707004882000000000+i)
                del writer
                out=root/'out.obs'
                self.converter('rosbag_to_rinex','--bag',bag,'--obs',out,success=ok)
                if ok:
                    lines=out.read_text().splitlines()
                    self.assertEqual(sum(line.startswith('>') for line in lines),1)
                    self.assertEqual(sum(line.startswith(('G01','G02')) for line in lines),
                                     len({o.prn for m in messages for o in m.observations}))
                else:
                    self.assertFalse(out.exists())
                    out.write_text('previous output')
                    self.converter('rosbag_to_rinex','--bag',bag,'--obs',out,success=False)
                    self.assertEqual(out.read_text(),'previous output')

    def test_cli_topics_partial_nav_and_empty_input_failures(self):
        with tempfile.TemporaryDirectory(prefix='gnss_cli_') as directory:
            root=pathlib.Path(directory)
            e=GnssEphemeris(system='G',prn=1,satid='G01',week=2300,
                toe=100.,toc=100.,ttr=100.,toes=100.,a=26560000.,e=.01,iode=7)
            bag=self.write_navigation_bag(root,GnssEphemerides(gnss_ephemeris=[e]))
            for flag,value in [('--topic-obs','/missing'),('--topic-obs','/gnss/ephemeris'),
                               ('--unknown','value'),('--rnx-version','nan'),('--rnx-version','3.04junk')]:
                self.converter('rosbag_to_rinex','--bag',bag,'--obs',root/'bad.obs',flag,value,success=False)
                self.assertFalse((root/'bad.obs').exists())
            self.converter('rosbag_to_rinex','--bag',bag,'--nav',root/'bad.nav','--topic-nav','/missing',success=False)
            self.assertFalse((root/'bad.nav').exists())
            for suffix in ['obs','nav']:
                output=root/('good.'+suffix);output.write_text('previous output');output.chmod(0o600)
            self.converter('rosbag_to_rinex','--bag',bag,'--obs',root/'good.obs','--nav',root/'good.nav')
            for suffix in ['obs','nav']:self.assertEqual((root/('good.'+suffix)).stat().st_mode & 0o777,0o600)
            # Explicit nav-only export is valid; unrelated observation topic is not required.
            self.converter('rosbag_to_rinex','--bag',bag,'--nav',root/'navonly.nav','--topic-obs','/absent')
            for extra in [('--unknown','value'),('--nav',),('--max-dtoe-glo','nan'),
                          ('--max-dtoe-glo','-1'),('--max-dtoe-glo','1junk'),
                          ('--eph-mode','on-change'),('--max-dtoe-gnss','7200'),
                          ('--nav',root/'good.nav','--nav',root/'missing.nav')]:
                self.converter('rinex_to_rosbag','--obs',root/'good.obs','--out',root/'badbag',*extra,success=False)
                self.assertFalse((root/'badbag').exists())
            # Zero retains the existing exact-toe-only GLONASS channel lookup.
            self.converter('rinex_to_rosbag','--obs',root/'good.obs',
                           '--out',root/'zero-age','--max-dtoe-glo','0')
            (root/'empty.obs').write_text('')
            self.converter('rinex_to_rosbag','--obs',root/'empty.obs','--out',root/'badbag',success=False)
            self.converter('rinex_to_rosbag','--nav',root/'good.nav','--out',root/'badbag',success=False)
            # A declared empty observation stream is intentionally exportable as a header.
            with sqlite3.connect(next(bag.glob('*.db3'))) as db:db.execute('DELETE FROM messages')
            self.converter('rosbag_to_rinex','--bag',bag,'--obs',root/'empty-output.obs')
            self.assertIn('END OF HEADER',(root/'empty-output.obs').read_text())

    def test_decoder_navigation_exports_without_synthetic_iodc_defaults(self):
        import test_receiver_streams as wire
        from test_rtcm_navigation_reference import navigation
        frames=[navigation(mid)[0] for mid in [1019,1045,1046,1042,1020]]
        captured=wire.run('rinex_decoder_nav','rtcm_decoder_node',frames,
            ['-p','reference_week:=2300','-p','reference_tow:=96000.0',
             '-p','topics.ephemeris:=/review/nav'],chunk_delay=.05)
        batch=next(n for n in reversed(captured['nav']) if n.glonass_ephemeris)
        self.assertTrue(any(e.system=='E' and e.iodc==e.iode and e.iode>0 for e in batch.gnss_ephemeris))
        with tempfile.TemporaryDirectory(prefix='gnss_decoder_nav_') as directory:
            root=pathlib.Path(directory)
            bag=self.write_navigation_bag(root,batch)
            self.converter('rosbag_to_rinex','--bag',bag,'--nav',root/'out.nav')
            self.assertEqual(sum(line.startswith(('G01 ','E01 ','C01 ','R01 '))
                                 for line in (root/'out.nav').read_text().splitlines()),5)

    def test_receiver_bds_tags_and_independent_galileo_iodc(self):
        for system,flag,iodc,ok in [('C',1,7,True),('C',2,7,True),('C',3,7,False),
                                    ('E',0,7,True),('E',0,8,False)]:
            with self.subTest(system=system,flag=flag,iodc=iodc), tempfile.TemporaryDirectory() as directory:
                root=pathlib.Path(directory)
                e=GnssEphemeris(system=system,prn=1,satid=system+'01',week=2300,
                    toe=100.,toc=100.,ttr=100.,toes=86. if system=='C' else 100.,
                    a=26560000.,e=.01,iode=7,iodc=iodc,flag=flag)
                bag=self.write_navigation_bag(root,GnssEphemerides(gnss_ephemeris=[e]))
                self.converter('rosbag_to_rinex','--bag',bag,'--nav',root/'out.nav',success=ok)

    def test_all_signal_slots_and_valid_zero_are_written(self):
        with tempfile.TemporaryDirectory(prefix='gnss_export_signals_') as directory:
            root=pathlib.Path(directory);bag=root/'input'
            writer=rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
            writer.create_topic(topic_metadata(name='/gnss/observation',
                type='gnss_ros_standardization/msg/GnssObservations',serialization_format='cdr'))
            expected={('E01',s) for s in ['1C','1X','5Q','5X','7Q','7X','8X']}|{('R01','3X')}
            epoch=GnssObservations(week=2300,tow=100.,observations=[GnssObservation(
                system=sat[0],satid=sat,prn=1,code_str=code,p=21000000.,l=0.,d=0.,snr=45.,lli=3)
                for sat,code in sorted(expected)])
            writer.write('/gnss/observation',serialize_message(epoch),1707004882000000000)
            del writer
            self.converter('rosbag_to_rinex','--bag',bag,'--obs',root/'out.obs')
            types={};last=None;values={}
            for line in (root/'out.obs').read_text().splitlines():
                if line[60:].strip()=='SYS / # / OBS TYPES':
                    if line[0]!=' ':last=line[0];types[last]=[]
                    types[last]+=line[7:60].split()
                elif line[:3] in ['E01','R01']:
                    for index,kind in enumerate(types[line[0]]):
                        field=line[3+16*index:19+16*index]
                        values[(line[:3],kind)]=(float(field[:14]),field[14])
            for sat,code in expected:
                self.assertEqual(values[(sat,'D'+code)][0],0.)
                self.assertEqual(values[(sat,'L'+code)],(0.,'3'))
            self.assertEqual(len(values),len(expected)*4)

    def test_release_X1_nose_up_pitch_has_upward_forward_axis(self):
        self.test_reference_odometry_position_and_child_frame_velocity(pitch_deg=5.)

    def test_reference_odometry_position_and_child_frame_velocity(self, pitch_deg=0.):
        from nav_msgs.msg import Odometry
        with tempfile.TemporaryDirectory(prefix='gnss_reference_') as directory:
            root=pathlib.Path(directory)
            # Independent small RINEX input; reference geometry at the equator
            # makes ECEF +Y exactly ENU east. Heading north means body +Y west.
            def header(value,label):return value.ljust(60)+label+'\n'
            obs=(header('     3.04           OBSERVATION DATA    M','RINEX VERSION / TYPE')+
                 header('G    4 C1C L1C D1C S1C','SYS / # / OBS TYPES')+
                 header('','END OF HEADER')+'> 2024 02 04 00 01 40.0000000  0  1\n'+
                 'G01'+''.join(f'{v:14.3f}  ' for v in [21000000.,110000000.,-12.,45.])+'\n')
            (root/'input.obs').write_text(obs)
            (root/'reference.csv').write_text('tow,week,lat,lon,h,x,y,z,roll,pitch,heading,e,n,u\n'
                f'100,2300,0,0,0,6378137,0,0,0,{pitch_deg},0,1,2,3\n'
                f'101,2300,0,0,0,6378137,10,0,0,{pitch_deg},0,1,2,3\n')
            self.converter('rinex_to_rosbag','--obs',root/'input.obs','--reference',root/'reference.csv',
                           '--out',root/'bag','--storage','sqlite3')
            reader=rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(root/'bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
            actual=[]
            while reader.has_next():
                topic,data,_=reader.read_next()
                if topic.endswith('_odom'):actual.append(deserialize_message(data,Odometry))
            self.assertEqual(len(actual),2)
            for i,m in enumerate(actual):
                self.assertEqual((m.header.frame_id,m.child_frame_id),('map','base_link'))
                self.assertAlmostEqual(m.pose.pose.position.x,i*10.)
                self.assertAlmostEqual(m.pose.pose.position.y,0.)
                self.assertAlmostEqual(m.pose.pose.position.z,0.)
                if pitch_deg:
                    q=m.pose.pose.orientation
                    forward_z=2*(q.x*q.z-q.w*q.y)
                    self.assertGreater(forward_z,0.)
                    self.assertAlmostEqual(forward_z,math.sin(math.radians(pitch_deg)))
                    continue
                self.assertAlmostEqual(m.twist.twist.linear.x,2.)
                self.assertAlmostEqual(m.twist.twist.linear.y,-1.)
                self.assertAlmostEqual(m.twist.twist.linear.z,3.)
                self.assertAlmostEqual(m.pose.pose.orientation.z,math.sqrt(.5))

    def test_glonass_observation_channels_bag_rinex_bag_without_navigation(self):
        with tempfile.TemporaryDirectory(prefix='gnss_fcn_') as directory:
            root=pathlib.Path(directory);bag=root/'input'
            writer=rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
            kwargs=dict(name='/gnss/observation',type='gnss_ros_standardization/msg/GnssObservations',serialization_format='cdr')
            try:meta=rosbag2_py.TopicMetadata(id=0,**kwargs)
            except TypeError:meta=rosbag2_py.TopicMetadata(**kwargs)
            writer.create_topic(meta)
            expected={1:0,2:7,3:13,4:255}
            epoch=GnssObservations(week=2300,tow=100.,observations=[
                GnssObservation(system='R',prn=prn,satid=f'R{prn:02}',freq=freq,
                                code=1,code_str='1C',p=21000000.,l=110000000.,d=-12.,snr=45.)
                for prn,freq in expected.items()])
            writer.write('/gnss/observation',serialize_message(epoch),1707004882000000000)
            del writer
            self.converter('rosbag_to_rinex','--bag',bag,'--obs',root/'out.obs','--nav',root/'out.nav')
            self.converter('rinex_to_rosbag','--obs',root/'out.obs','--out',root/'roundtrip','--storage','sqlite3')
            reader=rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(root/'roundtrip'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
            actual={}
            while reader.has_next():
                topic,data,_=reader.read_next()
                if topic=='/gnss/observation':
                    actual.update({o.prn:o.freq for o in deserialize_message(data,GnssObservations).observations})
            self.assertEqual(actual,expected)
            # Separate files must retain their own header, even for the same SV.
            source=(root/'out.obs').read_text()
            self.assertIn(' R01 -7',source)
            (root/'other.obs').write_text(source.replace(' R01 -7',' R01  6'))
            self.converter('rinex_to_rosbag','--obs','first='+str(root/'out.obs'),
                '--obs','second='+str(root/'other.obs'),'--out',root/'multiple','--storage','sqlite3')
            separate=rosbag2_py.SequentialReader()
            separate.open(rosbag2_py.StorageOptions(uri=str(root/'multiple'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
            channels={}
            while separate.has_next():
                topic,data,_=separate.read_next()
                if topic.endswith('/observation'):
                    channels[topic]=next(o.freq for o in deserialize_message(data,GnssObservations).observations if o.prn==1)
            self.assertEqual(channels,{'/first/observation':0,'/second/observation':13})
            result=self.converter('rosbag_to_rinex','--bag',bag,'--obs',root/'old.obs',
                '--nav',root/'old.nav','--rnx-version','3.01',success=False)
            self.assertIn('frequency channels require',result.stderr)
            # A single RINEX header cannot express competing FCNs for one slot.
            epoch.observations.append(GnssObservation(system='R',prn=1,satid='R01',freq=7,
                code=1,code_str='1C',p=21000000.,l=110000000.,d=-12.,snr=45.))
            with sqlite3.connect(next(bag.glob('*.db3'))) as db:
                db.execute('UPDATE messages SET data=?',(serialize_message(epoch),))
            result=self.converter('rosbag_to_rinex','--bag',bag,'--obs',root/'conflict.obs','--nav',root/'conflict.nav',success=False)
            self.assertIn('Conflicting GLONASS frequency channels',result.stderr)

    def test_navigation_records_reach_the_bag_at_transmission_time(self):
        # The bag carries every NAV record (the nodes select): default at its
        # ttr (earlier ones 1 s before the first epoch, later than the last
        # epoch omitted); --eph-mode all: every record 1 s before the first
        # epoch. The initial batch must precede the first observation strictly
        # in the bag (A5-2: equal stamps on two topics play back in no order).
        def eph(iode,ttr,toe=200.):
            return GnssEphemeris(system='G',prn=1,satid='G01',week=2300,toe=toe,toc=toe,
                ttr=float(ttr),toes=toe,a=26560000.,e=.01,iode=iode,iodc=iode)
        with tempfile.TemporaryDirectory(prefix='gnss_nav_stream_') as directory:
            root=pathlib.Path(directory);source=root/'source'
            writer=rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(source),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
            for topic,typ in [('/gnss/observation','GnssObservations'),('/gnss/ephemeris','GnssEphemerides')]:
                writer.create_topic(topic_metadata(name=topic,type='gnss_ros_standardization/msg/'+typ,serialization_format='cdr'))
            records=[eph(7,50.),eph(8,250.),eph(9,300.),eph(10,500.,toe=600.)]
            writer.write('/gnss/ephemeris',serialize_message(GnssEphemerides(gnss_ephemeris=records)),1707004882000000000)
            for i,tow in enumerate([100.,200.,300.,400.]):
                writer.write('/gnss/observation',serialize_message(GnssObservations(week=2300,tow=tow,
                    observations=[GnssObservation(system='G',prn=1,satid='G01',code=1,code_str='1C',
                    p=21000000.,l=110000000.,d=-12.,snr=45.)])),1707004882000000000+i)
            del writer
            self.converter('rosbag_to_rinex','--bag',source,'--obs',root/'in.obs','--nav',root/'in.nav')
            def stream(*mode):
                out=root/('bag'+'_'.join(mode))
                self.converter('rinex_to_rosbag','--obs',root/'in.obs','--nav',root/'in.nav',
                               '--out',out,'--timestamp-scale','gpst',*mode)
                reader=rosbag2_py.SequentialReader()
                reader.open(rosbag2_py.StorageOptions(uri=str(out),storage_id=''),rosbag2_py.ConverterOptions('',''))
                rows=[];first_nav=first_obs=None
                while reader.has_next():
                    topic,data,stamp=reader.read_next()
                    if topic=='/gnss/observation' and first_obs is None:first_obs=stamp
                    if topic=='/gnss/ephemeris':
                        first_nav=stamp if first_nav is None else min(first_nav,stamp)
                        m=deserialize_message(data,GnssEphemerides)
                        rows.append((round(m.header.stamp.sec+m.header.stamp.nanosec*1e-9-315964800-2300*604800,3),
                                     sorted(e.iode for e in m.gnss_ephemeris)))
                self.assertEqual(first_obs-first_nav,1000000000)
                return sorted(rows)
            self.assertEqual(stream(),[(99.,[7]),(250.,[8]),(300.,[9])])
            self.assertEqual(stream('--eph-mode','transmission'),[(99.,[7]),(250.,[8]),(300.,[9])])
            self.assertEqual(stream('--eph-mode','all'),[(99.,[7,8,9,10])])
            # A record the message cannot carry (QZSS-style "unhealthy, no
            # orbit": zero orbit lines) is omitted with a warning, not fatal.
            lines=(root/'in.nav').read_text().splitlines(keepends=True)
            head=next(i for i,l in enumerate(lines) if 'END OF HEADER' in l)+1
            block=lines[head:head+8]
            empty=[block[0]]+[l[:4]+' 0.000000000000E+00'*4+'\n' for l in block[1:5]]+block[5:]
            (root/'in.nav').write_text(''.join(lines+empty))
            result=self.converter('rinex_to_rosbag','--obs',root/'in.obs','--nav',root/'in.nav',
                                  '--out',root/'with_empty','--timestamp-scale','gpst')
            self.assertIn('NAV record without an orbit omitted (G01)',result.stderr)
            self.assertIn('1 NAV records without an orbit omitted',result.stderr)

    def test_ionosphere_parameters_bag_rinex_header_bag(self):
        # Broadcast ionosphere sets: bag -> RINEX NAV header (IONOSPHERIC CORR,
        # 4 significant digits as RINEX writes them) -> bag on every message.
        gps=[1.583e-08,2.235e-08,-1.192e-07,-1.192e-07,120800.,163800.,-131100.,-327700.]
        gal=[200.3,0.1289,0.01776,0.]
        record=GnssEphemeris(system='G',prn=1,satid='G01',week=2300,toe=200.,toc=200.,
            ttr=50.,toes=200.,a=26560000.,e=.01,iode=7,iodc=7)
        with tempfile.TemporaryDirectory(prefix='gnss_ion_') as directory:
            root=pathlib.Path(directory)
            source=self.write_navigation_bag(root,GnssEphemerides(gnss_ephemeris=[record],ion_gps=gps,ion_gal=gal))
            self.converter('rosbag_to_rinex','--bag',source,'--obs',root/'in.obs','--nav',root/'in.nav')
            header=(root/'in.nav').read_text().split('END OF HEADER')[0]
            for label in ['GPSA','GPSB','GAL ']:
                self.assertTrue(any(l.startswith(label) and 'IONOSPHERIC CORR' in l for l in header.splitlines()),label)
            self.assertNotIn('QZSA',header)
            out=root/'back'
            self.converter('rinex_to_rosbag','--obs',root/'in.obs','--nav',root/'in.nav','--out',out)
            reader=rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(out),storage_id=''),rosbag2_py.ConverterOptions('',''))
            seen=0
            while reader.has_next():
                topic,data,_=reader.read_next()
                if topic!='/gnss/ephemeris':continue
                m=deserialize_message(data,GnssEphemerides);seen+=1
                for got,want in [(m.ion_gps,gps),(m.ion_gal[:3],gal[:3])]:
                    for a,b in zip(got,want):self.assertTrue(math.isclose(a,b,rel_tol=1e-3),(a,b))
                self.assertEqual(list(m.ion_qzs),[0.]*8)
            self.assertGreater(seen,0)

    def write_navigation_bag(self, root, batch):
        bag=root/'input'
        writer=rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
        for topic,typ in [('/gnss/observation','GnssObservations'),('/gnss/ephemeris','GnssEphemerides')]:
            kwargs=dict(name=topic,type='gnss_ros_standardization/msg/'+typ,serialization_format='cdr')
            try:meta=rosbag2_py.TopicMetadata(id=0,**kwargs)
            except TypeError:meta=rosbag2_py.TopicMetadata(**kwargs)
            writer.create_topic(meta)
        writer.write('/gnss/ephemeris',serialize_message(batch),1707004882000000000)
        observations=[GnssObservation(system=sys,prn=1,satid=sys+'01',code=1,code_str='1C',
                         p=21000000.,l=110000000.,d=-12.,snr=45.) for sys in ['G','R']]
        writer.write('/gnss/observation',serialize_message(GnssObservations(
            week=2300,tow=100.,observations=observations)),1707004882000000000)
        del writer
        return bag

    def converter(self, executable, *values, success=True):
        result=subprocess.run([str(pathlib.Path(args.bin_dir)/executable),*map(str,values)],
                              capture_output=True,text=True,timeout=30)
        self.assertEqual(result.returncode==0,success,result.stdout+result.stderr)
        return result

    def test_orbit_rates_survive_bag_and_rinex_export_rejects_loss(self):
        for field in ['adot','ndot']:
            with self.subTest(field=field), tempfile.TemporaryDirectory(prefix='gnss_orbit_rates_') as directory:
                root=pathlib.Path(directory)
                e=GnssEphemeris(system='G',prn=1,satid='G01',week=2300,
                    toe=100.,toc=100.,ttr=100.,toes=100.,a=26560000.,e=.01,iode=7)
                setattr(e,field,-.125 if field=='adot' else 1e-12)
                batch=GnssEphemerides(gnss_ephemeris=[e])
                bag=self.write_navigation_bag(root,batch)
                reader=rosbag2_py.SequentialReader()
                reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
                topic,data,_=reader.read_next()
                self.assertEqual(topic,'/gnss/ephemeris')
                self.assertEqual(getattr(deserialize_message(data,GnssEphemerides).gnss_ephemeris[0],field),getattr(e,field))
                for version in ['3.04','3.05']:
                    result=self.converter('rosbag_to_rinex','--bag',bag,'--obs',root/'out.obs',
                        '--nav',root/'out.nav','--rnx-version',version,success=False)
                    self.assertIn('cannot preserve Adot/ndot',result.stderr)

    def test_navigation_export_rejects_other_silent_losses(self):
        cases=[('tgd', [1e-9,2e-9], 'extra tgd'), ('toc',100.25,'fractional'),
               ('toes',101.,'inconsistent'), ('sva',-3,'accuracy index')]
        for field,value,reason in cases:
            with self.subTest(field=field), tempfile.TemporaryDirectory(prefix='gnss_nav_loss_') as directory:
                root=pathlib.Path(directory)
                e=GnssEphemeris(system='G',prn=1,satid='G01',week=2300,
                    toe=100.,toc=100.,ttr=100.,toes=100.,a=26560000.,e=.01,iode=7)
                setattr(e,field,value)
                bag=self.write_navigation_bag(root,GnssEphemerides(gnss_ephemeris=[e]))
                result=self.converter('rosbag_to_rinex','--bag',bag,'--obs',root/'out.obs',
                                      '--nav',root/'out.nav',success=False)
                self.assertIn(reason,result.stderr)
                self.assertFalse((root/'out.obs').exists())
                self.assertFalse((root/'out.nav').exists())
                for suffix in ['obs','nav']:(root/('out.'+suffix)).write_text('previous output')
                self.converter('rosbag_to_rinex','--bag',bag,'--obs',root/'out.obs',
                               '--nav',root/'out.nav',success=False)
                for suffix in ['obs','nav']:self.assertEqual((root/('out.'+suffix)).read_text(),'previous output')
                self.assertFalse(list(root.glob('*.tmp-*')))

    def test_glonass_metadata_rinex305_roundtrip_and_older_rejection(self):
        with tempfile.TemporaryDirectory(prefix='gnss_glo_metadata_') as directory:
            root=pathlib.Path(directory)
            g=GlonassEphemeris(system='R',prn=1,satid='R01',week=2300,
                toe=18.,tof=100.,iode=12,frq=-7,svh=0,flags=511,sva=7,dtaun=2e-9,
                pos=[19000000.,1000000.,15000000.],vel=[1.,2.,3.],acc=[0.,0.,0.])
            bag=self.write_navigation_bag(root,GnssEphemerides(glonass_ephemeris=[g]))
            result=self.converter('rosbag_to_rinex','--bag',bag,'--obs',root/'old.obs',
                                  '--nav',root/'old.nav','--rnx-version','3.04',success=False)
            self.assertIn('--rnx-version 3.05',result.stderr)
            self.converter('rosbag_to_rinex','--bag',bag,'--obs',root/'out.obs',
                           '--nav',root/'out.nav')
            self.converter('rinex_to_rosbag','--obs',root/'out.obs','--nav',root/'out.nav',
                           '--out',root/'roundtrip','--storage','sqlite3')
            reader=rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(root/'roundtrip'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
            values=[]
            while reader.has_next():
                topic,data,_=reader.read_next()
                if topic=='/gnss/ephemeris':values+=list(deserialize_message(data,GnssEphemerides).glonass_ephemeris)
            self.assertTrue(values)
            self.assertEqual((values[0].flags,values[0].sva),(511,7))
            self.assertAlmostEqual(values[0].dtaun,2e-9,places=18)

    def test_observation_navigation_rinex_roundtrip(self):
        with tempfile.TemporaryDirectory(prefix='gnss_rinex_') as directory:
            root=pathlib.Path(directory); bag=root/'input'
            writer=rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
            for topic,typ in [('/gnss/observation','GnssObservations'),('/gnss/ephemeris','GnssEphemerides')]:
                kwargs=dict(name=topic,type='gnss_ros_standardization/msg/'+typ,serialization_format='cdr')
                try:meta=rosbag2_py.TopicMetadata(id=0,**kwargs)
                except TypeError:meta=rosbag2_py.TopicMetadata(**kwargs)
                writer.create_topic(meta)
            nav=GnssEphemerides();e=GnssEphemeris(system='G',prn=1,satid='G01',week=2300,
                toe=100.,toc=100.,ttr=100.,toes=100.,
                a=26560000.,e=0.01,i0=0.95,omg0=1.,omg=0.5,m0=0.1,iode=7,iodc=7,tgd=[1e-9,0.],fit=4)
            nav.gnss_ephemeris=[e]
            writer.write('/gnss/ephemeris',serialize_message(nav),1707004882000000000)
            for tow in (100.,101.):
                epoch=GnssObservations(week=2300,tow=tow)
                epoch.observations=[GnssObservation(sat=1,code=1,p=21000000.125,l=110000000.25,
                    d=-1234.5,snr=45.,lli=1,system='G',prn=1,satid='G01',code_str='1C')]
                writer.write('/gnss/observation',serialize_message(epoch),int(1707004782+tow)*10**9)
            del writer
            def run(exe,*values,success=True):
                result=subprocess.run([str(pathlib.Path(args.bin_dir)/exe),*map(str,values)],capture_output=True,text=True,timeout=30)
                self.assertEqual(result.returncode == 0,success,result.stdout+result.stderr)
                return result
            run('rosbag_to_rinex','--bag',bag,'--obs',root/'output.obs','--nav',root/'output.nav')
            run('rinex_to_rosbag','--obs',root/'output.obs','--nav',root/'output.nav','--out',root/'roundtrip','--storage','sqlite3')
            reader=rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(root/'roundtrip'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
            observations=[];ephemerides=[]
            while reader.has_next():
                topic,data,stamp=reader.read_next()
                if topic=='/gnss/observation':
                    epoch=deserialize_message(data,GnssObservations);observations.append(epoch)
                    self.assertEqual(stamp,int(1707004782+epoch.tow)*10**9)
                elif topic=='/gnss/ephemeris':ephemerides+=list(deserialize_message(data,GnssEphemerides).gnss_ephemeris)
            self.assertEqual([o.tow for o in observations],[100.,101.])
            for epoch in observations:
                self.assertEqual(epoch.week,2300);self.assertEqual(len(epoch.observations),1)
                obs=epoch.observations[0];self.assertEqual((obs.sat,obs.code,obs.lli),(1,1,1))
                for field,expected in [('p',21000000.125),('l',110000000.25),('d',-1234.5),('snr',45.)]:
                    self.assertAlmostEqual(getattr(obs,field),expected,places=3)
            self.assertTrue(ephemerides)
            self.assertEqual(ephemerides[0].week,2300);self.assertEqual(ephemerides[0].toe,100.);self.assertEqual(ephemerides[0].iode,7);self.assertAlmostEqual(ephemerides[0].a,26560000.,places=3)


            # An input NAV record exceeding the fixed ROS field width must fail
            # explicitly, not wrap IODE or silently omit it from a successful bag.
            lines=(root/'output.nav').read_text().splitlines(keepends=True)
            first=next(i for i,line in enumerate(lines) if line.startswith('G01 '))
            lines[first+1]=lines[first+1][:4]+f'{65536.:19.12E}'.replace('E','D')+lines[first+1][23:]
            (root/'overflow.nav').write_text(''.join(lines))
            result=run('rinex_to_rosbag','--obs',root/'output.obs','--nav',root/'overflow.nav',
                       '--out',root/'overflow_bag','--storage','sqlite3',success=False)
            self.assertIn('iode outside uint16',result.stderr)

            # Exercise the RINEX writer itself, including failure on final flush.
            run('rosbag_to_rinex','--bag',bag,'--obs','/dev/full','--nav',root/'full.nav',success=False)
            run('rosbag_to_rinex','--bag',bag,'--obs',root/'full.obs','--nav','/dev/full',success=False)
            with sqlite3.connect(next(bag.glob('*.db3'))) as db:
                db.execute("UPDATE messages SET data=? WHERE topic_id=(SELECT id FROM topics WHERE name='/gnss/observation')",(b'bad-cdr',))
            result=run('rosbag_to_rinex','--bag',bag,'--obs',root/'broken.obs','--nav',root/'broken.nav',success=False)
            self.assertIn('Cannot deserialize',result.stderr)

    def test_galileo_ten_bit_iod_bag_rinex_bag(self):
        with tempfile.TemporaryDirectory(prefix='gnss_galileo_iod_') as directory:
            root=pathlib.Path(directory);bag=root/'input'
            writer=rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
            for topic,typ in [('/gnss/observation','GnssObservations'),('/gnss/ephemeris','GnssEphemerides')]:
                kwargs=dict(name=topic,type='gnss_ros_standardization/msg/'+typ,serialization_format='cdr')
                try:meta=rosbag2_py.TopicMetadata(id=0,**kwargs)
                except TypeError:meta=rosbag2_py.TopicMetadata(**kwargs)
                writer.create_topic(meta)
            for index,iod in enumerate([255,256,1023]):
                tow=120.+index*120.
                e=GnssEphemeris(system='E',prn=1,satid='E01',week=2300,code=517,
                    toe=tow,toc=tow,ttr=tow,toes=tow,a=26560000.,e=.01,i0=.95,
                    iode=iod,tgd=[1e-9,2e-9])
                # Existing Galileo selection requires toe strictly before the observation.
                obs_tow=tow+1.
                stamp=int(1707004782+obs_tow)*10**9
                writer.write('/gnss/ephemeris',serialize_message(GnssEphemerides(gnss_ephemeris=[e])),stamp)
                observation=GnssObservation(system='E',prn=1,satid='E01',code=1,code_str='1C',
                                            p=21000000.,l=110000000.,d=-12.,snr=45.)
                writer.write('/gnss/observation',serialize_message(GnssObservations(
                    week=2300,tow=obs_tow,observations=[observation])),stamp)
            del writer
            for executable,values in [
                ('rosbag_to_rinex',['--bag',bag,'--obs',root/'out.obs','--nav',root/'out.nav']),
                ('rinex_to_rosbag',['--obs',root/'out.obs','--nav',root/'out.nav',
                                   '--out',root/'roundtrip','--storage','sqlite3'])]:
                result=subprocess.run([str(pathlib.Path(args.bin_dir)/executable),*map(str,values)],
                                      capture_output=True,text=True,timeout=30)
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            reader=rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(root/'roundtrip'),storage_id='sqlite3'),
                        rosbag2_py.ConverterOptions('',''))
            pairs=set()
            while reader.has_next():
                topic,data,_=reader.read_next()
                if topic=='/gnss/ephemeris':
                    for e in deserialize_message(data,GnssEphemerides).gnss_ephemeris:
                        self.assertEqual((e.system,e.prn),('E',1))
                        pairs.add((e.toe,e.iode))
            self.assertEqual(pairs,{(120.,255),(240.,256),(360.,1023)})

    def test_nan_observations_preserve_other_components_through_rinex(self):
        with tempfile.TemporaryDirectory(prefix='gnss_nan_bag_') as directory:
            root=pathlib.Path(directory);bag=root/'input'
            writer=rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
            kwargs=dict(name='/gnss/observation',type='gnss_ros_standardization/msg/GnssObservations',serialization_format='cdr')
            try:meta=rosbag2_py.TopicMetadata(id=0,**kwargs)
            except TypeError:meta=rosbag2_py.TopicMetadata(**kwargs)
            writer.create_topic(meta)
            epoch=GnssObservations(week=2300,tow=100.,observations=[
                GnssObservation(system='G',prn=1,satid='G01',code=1,code_str='1C',
                    p=21000000.,l=110000000.,d=-12.,snr=45.),
                GnssObservation(system='G',prn=2,satid='G02',code=1,code_str='1C',
                    p=math.nan,l=120000000.25,d=-1234.5,snr=math.nan),
                GnssObservation(system='G',prn=3,satid='G03',code=1,code_str='1C',
                    p=math.nan,l=math.nan,d=-100.5,snr=math.nan),
                GnssObservation(system='G',prn=4,satid='G04',code=1,code_str='1C',
                    p=math.nan,l=math.nan,d=math.nan,snr=38.5),
                GnssObservation(system='G',prn=5,satid='G05',code=1,code_str='1C',
                    p=21000000.,l=math.nan,d=0.,snr=40.),
                GnssObservation(system='G',prn=6,satid='G06',code=1,code_str='1C',
                    p=21000000.,l=110000000.,d=-10.,snr=40.,lli=5)])
            writer.write('/gnss/observation',serialize_message(epoch),1707004882000000000)
            del writer
            for executable,values in [
                ('rosbag_to_rinex',['--bag',bag,'--obs',root/'out.obs']),
                ('rinex_to_rosbag',['--obs',root/'out.obs','--out',root/'roundtrip','--storage','sqlite3'])]:
                result=subprocess.run([str(pathlib.Path(args.bin_dir)/executable),*map(str,values)],capture_output=True,text=True,timeout=30)
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            text=(root/'out.obs').read_text()
            self.assertNotRegex(text.lower(),r'\b(?:nan|[+-]?inf)\b')
            row=next(line for line in text.splitlines() if line.startswith('G02'))
            self.assertEqual(row[3:19].strip(),'')  # C1C is missing, never numeric NaN or zero.
            reader=rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(root/'roundtrip'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
            values=[]
            while reader.has_next():
                topic,data,_=reader.read_next()
                if topic=='/gnss/observation':values+=list(deserialize_message(data,GnssObservations).observations)
            obs=next(o for o in values if o.satid=='G02')
            self.assertTrue(math.isnan(obs.p))
            self.assertTrue(math.isnan(obs.snr))
            self.assertAlmostEqual(obs.l,120000000.25,places=3)
            self.assertAlmostEqual(obs.d,-1234.5,places=3)
            doppler=next(o for o in values if o.satid=='G03')
            self.assertTrue(math.isnan(doppler.p));self.assertTrue(math.isnan(doppler.l))
            self.assertEqual(doppler.d,-100.5)
            cn0=next(o for o in values if o.satid=='G04')
            self.assertTrue(all(math.isnan(v) for v in (cn0.p,cn0.l,cn0.d)))
            self.assertEqual(cn0.snr,38.5)
            zero=next(o for o in values if o.satid=='G05')
            self.assertEqual(zero.d,0.)
            self.assertTrue(math.isnan(zero.l))
            self.assertEqual(next(o for o in values if o.satid=='G06').lli,5)

    def test_navigation_dedup_preserves_time_and_content_changes(self):
        with tempfile.TemporaryDirectory(prefix='gnss_nav_dedup_') as directory:
            root=pathlib.Path(directory);bag=root/'input'
            writer=rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
            for topic,typ in [('/gnss/observation','GnssObservations'),('/gnss/ephemeris','GnssEphemerides')]:
                kwargs=dict(name=topic,type='gnss_ros_standardization/msg/'+typ,serialization_format='cdr')
                try:meta=rosbag2_py.TopicMetadata(id=0,**kwargs)
                except TypeError:meta=rosbag2_py.TopicMetadata(**kwargs)
                writer.create_topic(meta)
            base=GnssEphemeris(system='G',prn=1,satid='G01',week=2300,
                toe=100.,toc=100.,ttr=100.,toes=100.,a=26560000.,e=.01,i0=.95,
                iode=7,iodc=7,tgd=[1e-9,0.],fit=4)
            gnss=[base]
            for updates in [dict(toe=200.,toc=200.,toes=200.),dict(week=2301),dict(svh=1),dict(f0=1e-6)]:
                item=copy.deepcopy(base)
                for key,value in updates.items():setattr(item,key,value)
                gnss.append(item)
            glo_base=GlonassEphemeris(system='R',prn=1,satid='R01',week=2300,
                toe=18.,tof=100.,iode=12,frq=-7,pos=[19000000.,1000000.,15000000.],
                vel=[1.,2.,3.],acc=[0.,0.,0.])
            glo=[glo_base]
            for updates in [dict(toe=918.,tof=1000.,iode=13),dict(week=2301),dict(svh=1),dict(taun=1e-6),dict(flags=128),dict(sva=3)]:
                item=copy.deepcopy(glo_base)
                for key,value in updates.items():setattr(item,key,value)
                glo.append(item)
            retransmit=copy.deepcopy(base);retransmit.ttr+=10.
            glo_retransmit=copy.deepcopy(glo_base);glo_retransmit.tof+=10.
            noise=copy.deepcopy(retransmit);noise.e=math.nextafter(base.e, math.inf)
            glo_noise=copy.deepcopy(glo_retransmit)
            glo_noise.pos[0]=math.nextafter(glo_base.pos[0], math.inf)
            nav=GnssEphemerides(gnss_ephemeris=gnss+[copy.deepcopy(base),retransmit,noise],
                               glonass_ephemeris=glo+[copy.deepcopy(glo_base),glo_retransmit,glo_noise])
            # Identical records both within a snapshot and in its retransmission.
            for stamp in [1707004882000000000,1707004883000000000]:
                writer.write('/gnss/ephemeris',serialize_message(nav),stamp)
            epoch=GnssObservations(week=2300,tow=100.,observations=[GnssObservation(
                system='G',prn=1,code_str='1C',code=1,p=21000000.125)])
            writer.write('/gnss/observation',serialize_message(epoch),1707004882000000000)
            del writer
            result=subprocess.run([str(pathlib.Path(args.bin_dir)/'rosbag_to_rinex'),
                '--bag',str(bag),'--obs',str(root/'out.obs'),'--nav',str(root/'out.nav')],
                capture_output=True,text=True,timeout=30)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            lines=(root/'out.nav').read_text().splitlines()
            gps=[i for i,line in enumerate(lines) if line.startswith('G01 ')]
            glonass=[i for i,line in enumerate(lines) if line.startswith('R01 ')]
            self.assertEqual(len(gps),5)
            self.assertEqual(len(glonass),7)
            # Each retained record has distinct output content, not five copies.
            self.assertEqual(len({tuple(lines[i:i+8]) for i in gps}),5)
            self.assertEqual(len({tuple(lines[i:i+5]) for i in glonass}),7)
            self.converter('rosbag_to_rinex','--bag',bag,'--nav',root/'all.nav','--nav-all-transmissions')
            all_lines=(root/'all.nav').read_text().splitlines()
            self.assertEqual(sum(line.startswith('G01 ') for line in all_lines),7)
            self.assertEqual(sum(line.startswith('R01 ') for line in all_lines),9)


    def test_stable_qzss_sbas_identity_without_satid(self):
        with tempfile.TemporaryDirectory(prefix='gnss_identity_') as directory:
            root=pathlib.Path(directory);bag=root/'input'
            writer=rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
            kwargs=dict(name='/gnss/observation',type='gnss_ros_standardization/msg/GnssObservations',serialization_format='cdr')
            try: meta=rosbag2_py.TopicMetadata(id=0,**kwargs)
            except TypeError: meta=rosbag2_py.TopicMetadata(**kwargs)
            writer.create_topic(meta)
            # The backend satellite number is deliberately wrong. Stable fields
            # carry the identity, including the actual QZSS and SBAS PRNs.
            epoch=GnssObservations(week=2300,tow=100.)
            epoch.observations=[GnssObservation(sat=250,system=system,prn=prn,code=1,
                code_str='1C',p=21000000.125,l=110000000.25,d=-1234.5,snr=45.)
                for system,prn in [('J',193),('S',120)]]
            writer.write('/gnss/observation',serialize_message(epoch),1707004882000000000)
            del writer
            for executable,values in [
                ('rosbag_to_rinex',['--bag',bag,'--obs',root/'output.obs','--nav',root/'output.nav']),
                ('rinex_to_rosbag',['--obs',root/'output.obs','--out',root/'roundtrip','--storage','sqlite3'])]:
                result=subprocess.run([str(pathlib.Path(args.bin_dir)/executable),*map(str,values)],capture_output=True,text=True,timeout=30)
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            reader=rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(root/'roundtrip'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
            observations=[]
            while reader.has_next():
                topic,data,_=reader.read_next()
                if topic=='/gnss/observation': observations+=list(deserialize_message(data,GnssObservations).observations)
            self.assertEqual({(o.system,o.prn,o.satid,o.code_str) for o in observations},
                             {('J',193,'J01','1C'),('S',120,'120','1C')})
            for obs in observations:
                self.assertAlmostEqual(obs.p,21000000.125,places=3)
                self.assertAlmostEqual(obs.l,110000000.25,places=3)
                self.assertAlmostEqual(obs.d,-1234.5,places=3)

    def test_pos_bag_pos_scales_storage_and_failures(self):
        with tempfile.TemporaryDirectory(prefix='gnss_converter_') as directory:
            root = pathlib.Path(directory)
            pos = root / 'input.pos'
            pos.write_text('% GPST latitude(deg) longitude(deg) height(m) Q ns sdn sde sdu sdne sdeu sdun age ratio\n'
                           '2300 100.000 35.0 139.0 130.0 1 12 1 2 3 0 0 0 0 4\n'
                           '2300 100.250 35.0 139.0 131.0 2 13 1 2 3 0 0 0 0 4\n')
            def run(exe, *values, success=True):
                result = subprocess.run([str(pathlib.Path(args.bin_dir)/exe), *map(str, values)],
                                        capture_output=True, text=True, timeout=30)
                self.assertEqual(result.returncode == 0, success, result.stdout + result.stderr)
                return result
            for storage in ('sqlite3', 'mcap'):
                for scale, leap in [('utc',18),('gpst',0)]:
                    bag = root / (storage+'_'+scale)
                    run('pos_to_rosbag','--pos',pos,'--out',bag,'--storage',storage,'--timestamp-scale',scale)
                    self.assertIn('gpst_legacy' if scale=='gpst' else 'utc', (bag/'gnss_time_basis.yaml').read_text())
                    reader = rosbag2_py.SequentialReader()
                    reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id=storage),rosbag2_py.ConverterOptions('',''))
                    rows=[]
                    while reader.has_next():
                        _,data,stamp=reader.read_next()
                        sol=deserialize_message(data,GnssSolution)
                        expected=(315964800+2300*604800+int(sol.time_tow)-leap)*10**9 + round((sol.time_tow-int(sol.time_tow))*1e9)
                        self.assertEqual(stamp,expected)
                        self.assertEqual(sol.header.stamp.sec*10**9+sol.header.stamp.nanosec,expected)
                        rows.append(sol)
                    del reader
                    self.assertEqual(len(rows),2)
                    self.assertEqual([s.altitude for s in rows],[130.,131.])
                    output = root/(storage+'_'+scale+'.pos')
                    run('rosbag_to_pos','--bag',bag,'--out',output)
                    self.assertEqual(len([l for l in output.read_text().splitlines() if l and not l.startswith('%')]),2)
                    run('rosbag_to_pos','--bag',bag,'--out','/dev/full',success=False)
                    if storage=='sqlite3' and scale=='utc':
                        database=next(bag.glob('*.db3'))
                        with sqlite3.connect(database) as db:
                            db.execute("UPDATE messages SET data=? WHERE id=(SELECT MIN(id) FROM messages)",(b'bad-cdr',))
                        result=run('rosbag_to_pos','--bag',bag,'--out',root/'partial.pos',success=False)
                        self.assertIn('Conversion failed',result.stderr)
            run('pos_to_rosbag','--pos',pos,'--timestamp-scale','invalid',success=False)
            run('rosbag_to_rinex','--bag',root/'absent',success=False)

if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(ConverterRoundtrip))
    raise SystemExit(0 if result.wasSuccessful() else 1)
