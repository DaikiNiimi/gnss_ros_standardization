#!/usr/bin/env python3
import os, socket, struct, subprocess, time, signal, json, math, sys, uuid
os.environ.setdefault('ROS_DOMAIN_ID',str(100 + os.getpid() % 100))
os.environ['ROS_LOCALHOST_ONLY']='1'
import tempfile, pathlib, argparse, unittest, resource
resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
TEMP=tempfile.TemporaryDirectory(prefix='gnss_protocol_')
os.environ['ROS_LOG_DIR']=TEMP.name+'/roslog'
ap=argparse.ArgumentParser();ap.add_argument('--bin-dir',required=True);args,unittest_args=ap.parse_known_args()
# Child executables must use this build's libraries, even if an older workspace
# install was sourced for Python message imports.
os.environ['LD_LIBRARY_PATH']=str(pathlib.Path(args.bin_dir).resolve())+os.pathsep+os.environ.get('LD_LIBRARY_PATH','')
sys.path.insert(0, str(pathlib.Path(args.bin_dir).resolve()/"rosidl_generator_py"))
import rclpy
from sensor_msgs.msg import Imu
from rcl_interfaces.srv import GetParameters
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from gnss_ros_standardization.msg import GnssSolution, GnssObservations, GnssEphemerides
B=args.bin_dir+'/'
MIN_DRAIN_SECONDS=1.1
def ubx(c,i,p,bad=False):
    b=bytes([c,i])+struct.pack('<H',len(p))+p
    a=z=0
    for v in b: a=(a+v)&255; z=(z+a)&255
    return b'\xb5\x62'+b+bytes([a,z^(1 if bad else 0)])
def crc32(b):
    c=0
    for x in b:
        c^=x
        for _ in range(8): c=(c>>1)^(0xedb88320 if c&1 else 0)
    return c
def oem(mid,p,short=False,bad=False,tow=100000,status=160):
    if short: h=struct.pack('<3sBHHI',b'\xaa\x44\x13',len(p),mid,2300,tow)
    else:
        h=bytearray(28); h[:3]=b'\xaa\x44\x12'; h[3]=28
        struct.pack_into('<H',h,4,mid); struct.pack_into('<H',h,8,len(p))
        h[13]=status; struct.pack_into('<HI',h,14,2300,tow)
    b=bytes(h)+p
    return b+struct.pack('<I',crc32(b)^(1 if bad else 0))
def pvt(tow=100000):
    p=bytearray(92); struct.pack_into('<I',p,0,tow); p[11]=7; p[20]=3; p[21]=1; p[23]=12
    struct.pack_into('<iii',p,24,1390000000,350000000,100000)
    struct.pack_into('<II',p,40,1000,2000); struct.pack_into('<I',p,68,3000)
    return p
def nmea(t):
    lines=[f'GPRMC,{t},A,3500.000,N,13900.000,E,1.0,0.0,040224,,,A',f'GPGGA,{t},3500.000,N,13900.000,E,1,12,1.0,100.0,M,30.0,M,,']
    out=b''
    for s in lines:
        c=0
        for x in s.encode(): c^=x
        out+=f'${s}*{c:02X}\r\n'.encode()
    return out
def run(label,exe,data,extra=None,*,file_input=False,disconnect=False,chunk_delay=.4):
    # Each fake receiver is a separate ROS participant. Give its subscriber the
    # same lifetime, so discovery state cannot leak across dozens of processes.
    context=Context()
    rclpy.init(context=context)
    node=rclpy.create_node('receiver_test_'+uuid.uuid4().hex[:8],context=context)
    executor=SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        return _run(node,executor,label,exe,data,extra,file_input=file_input,
                    disconnect=disconnect,chunk_delay=chunk_delay)
    finally:
        executor.shutdown()
        node.destroy_node()
        context.shutdown()

def _run(node,executor,label,exe,data,extra,*,file_input,disconnect,chunk_delay):
    messages={k:[] for k in ('sol','raw','imu','obs','nav')}
    case_id=uuid.uuid4().hex
    topic_prefix='/review/case_'+case_id+'/'
    receiver_name=exe+'_'+case_id[:8]
    subs=[]
    subscriptions={}
    for key,typ in [('sol',GnssSolution),('raw',Imu),('imu',Imu),('obs',GnssObservations),('nav',GnssEphemerides)]:
        subscriptions[key]=node.create_subscription(typ,topic_prefix+key,lambda m,k=key:messages[k].append(m),100)
        subs.append(subscriptions[key])
    server=socket.socket(); server.bind(('127.0.0.1',0)); server.listen(); server.settimeout(5)
    source=f'tcpcli://127.0.0.1:{server.getsockname()[1]}'
    if file_input:
        path=pathlib.Path(TEMP.name)/(label+'.sbf')
        path.write_bytes(b''.join(data) if isinstance(data,list) else data)
        source='file://'+str(path)
    args=[B+exe,'--ros-args','-p',f'stream_path:={source}','-p','topics.solution:=/review/sol','-p','topics.imu_raw:=/review/raw']
    if exe.startswith('rtcm'):
        args=[B+exe,'--ros-args','-p',f'stream_path:={source}','-p','topics.observation:=/review/obs']
    if not exe.startswith(('sbf','rtcm')): args+=['-p','topics.imu:=/review/imu']
    if 'driver' in exe:
        args+=['-p','configure_on_startup:=false']
        if exe.startswith('ubx'): args+=['-p','messages.nav_pvt:=true']
    if extra: args+=extra
    args=[arg.replace('/review/',topic_prefix) for arg in args]
    args+=['-r','__node:='+receiver_name]
    with open(TEMP.name+'/'+label+'.log','w') as log:
        proc=subprocess.Popen(args,stdout=log,stderr=log)
        client=None
        conn=None
        try:
            conn=None if file_input else server.accept()[0]
            if file_input:
                end=time.monotonic()+1.0
                while time.monotonic()<end: executor.spin_once(timeout_sec=.05)
            else:
                # Sending a finite fixture after an arbitrary sleep can lose
                # every sample before DDS discovery completes. Match this node
                # (not a previous case's publisher) before supplying TCP bytes.
                required=set()
                for key,param in [('obs','topics.observation'),('nav','topics.ephemeris'),
                                  ('sol','topics.solution'),('raw','topics.imu_raw'),('imu','topics.imu')]:
                    if param+':='+topic_prefix+key in args: required.add(key)
                end=time.monotonic()+10.0
                while True:
                    ready=all(
                        subscriptions[key].get_publisher_count()>0 and
                        any(info.node_name==receiver_name for info in
                            node.get_publishers_info_by_topic(topic_prefix+key))
                        for key in required)
                    if ready: break
                    if proc.poll() is not None or time.monotonic()>=end:
                        raise AssertionError(f'{label}: DDS publishers not matched: {sorted(required)}\n'
                                             +pathlib.Path(log.name).read_text())
                    executor.spin_once(timeout_sec=.05)
                # A local match alone is not proof that the other participant
                # has discovered us. Complete a real ROS request/reply with this
                # uniquely named node before sending the finite input fixture.
                client=node.create_client(GetParameters,'/'+receiver_name+'/get_parameters')
                if not client.wait_for_service(timeout_sec=5.0):
                    raise AssertionError(f'{label}: parameter service unavailable')
                future=client.call_async(GetParameters.Request(names=['stream_path']))
                executor.spin_until_future_complete(future,timeout_sec=5.0)
                if not future.done() or future.result().values[0].string_value!=source:
                    raise AssertionError(f'{label}: receiver ROS handshake failed: '
                                         f'done={future.done()}, result={future.result() if future.done() else None}, expected={source}')
                # Local reader matching can precede discovery at the remote
                # writer. Retain the settling interval after (not instead of)
                # matching. Per-case topics prevent stale previous endpoints
                # from satisfying the readiness check.
                end=time.monotonic()+1.0
                while time.monotonic()<end: executor.spin_once(timeout_sec=.05)
            chunks=[] if file_input else data if isinstance(data,list) else [data]
            for chunk in chunks:
                if callable(chunk):
                    chunk(node,executor)
                    continue
                if chunk is None:
                    # End one TCP session; let RTKLIB reconnect without restarting
                    # the ROS node or its subscriptions. Its default retry is 10 s.
                    conn.shutdown(socket.SHUT_RDWR)
                    conn.close(); conn=None
                    server.setblocking(False)
                    deadline=time.monotonic()+20.0
                    while conn is None:
                        try: conn=server.accept()[0]
                        except BlockingIOError:
                            if proc.poll() is not None or time.monotonic()>=deadline:
                                raise AssertionError(label+': receiver did not reconnect')
                            executor.spin_once(timeout_sec=.02)
                    server.settimeout(5)
                    continue
                conn.sendall(chunk)
                end=time.monotonic()+chunk_delay
                while time.monotonic()<end: executor.spin_once(timeout_sec=.01)
            if disconnect and conn is not None:
                conn.close();conn=None
            # Q1: diagnostics use the production five-second cadence. File
            # EOF tests inspect counters; delivery before discovery is not assumed.
            drain=max(MIN_DRAIN_SECONDS,5.1 if file_input else 1.1)
            end=time.monotonic()+drain
            while time.monotonic()<end: executor.spin_once(timeout_sec=.05)
            if conn is not None: conn.close()
        except (TimeoutError, ConnectionError) as exc:
            raise AssertionError(f'{label}: receiver connection failed\n'+pathlib.Path(log.name).read_text()) from exc
        finally:
            proc.send_signal(signal.SIGINT)
            try: proc.wait(timeout=4)
            except subprocess.TimeoutExpired: proc.kill(); proc.wait()
            if conn is not None: conn.close()
            server.close()
            artifact_dir=os.environ.get('GNSS_TEST_ARTIFACT_DIR')
            if artifact_dir:
                path=pathlib.Path(artifact_dir);path.mkdir(parents=True,exist_ok=True)
                (path/(label+'_'+case_id+'.log')).write_text(pathlib.Path(log.name).read_text())
            if client is not None: node.destroy_client(client)
            for s in subs: node.destroy_subscription(s)
    result={'case':label,'counts':{k:len(v) for k,v in messages.items()}}
    if messages['sol']:
        s=messages['sol'][-1]; result['sol']={'altitude':s.altitude,'week':s.time_week,'tow':s.time_tow,'num_sats':s.num_sats,'status':s.status,'pos_cov':list(s.pos_enu_cov),'velocity':[s.vel_enu.x,s.vel_enu.y,s.vel_enu.z]}
    if messages['raw']:
        s=messages['raw'][-1]; result['raw']={'acc':[s.linear_acceleration.x,s.linear_acceleration.y,s.linear_acceleration.z],'gyro':[s.angular_velocity.x,s.angular_velocity.y,s.angular_velocity.z],'acc_cov0':s.linear_acceleration_covariance[0],'gyro_cov0':s.angular_velocity_covariance[0]}
    print(json.dumps(result),flush=True)
    if proc.returncode != 0:
        raise AssertionError(pathlib.Path(TEMP.name, label+'.log').read_text())
    messages['log']=pathlib.Path(log.name).read_text()
    return messages

def sbf(mid,p,bad=False):
    h=struct.pack('<HH',mid,len(p)+8); crc=0
    for b in h+p:
        crc^=b<<8
        for _ in range(8): crc=((crc<<1)^(0x1021 if crc&0x8000 else 0))&0xffff
    return b'$@'+struct.pack('<H',crc^(1 if bad else 0))+h+p

class ReceiverStreams(unittest.TestCase):
    def test_release_V7_fixed_position_is_retained_on_both_paths(self):
        for family in ['sbf','novatel']:
            for mode in ['driver','decoder']:
                if family=='sbf':
                    body=bytearray(88);struct.pack_into('<IHBB3d',body,0,100000,2300,3,0,.6,2.4,100.);body[66]=12
                    frame=sbf(4007,body)
                else:
                    body=bytearray(72);struct.pack_into('<II3dfI3f',body,0,0,1,35.,139.,100.,30.,61,1.,2.,3.);body[65]=12
                    frame=oem(42,body)
                # Drivers deliberately lock their solution source at startup;
                # enable the binary output exercised by this fixture.
                params=[]
                if mode=='driver':
                    key='pvt_geodetic' if family=='sbf' else 'bestpos'
                    params=['-p','messages.'+key+':=true']
                r=run('V7_fixed_'+family+'_'+mode,family+'_'+mode+'_node',frame,params)
                self.assertEqual(len(r['sol']),1)
                sol=r['sol'][0];self.assertEqual(sol.status,GnssSolution.STATUS_SINGLE)
                self.assertTrue(all(math.isfinite(v) for v in [sol.pos_enu.x,sol.pos_enu.y,sol.pos_enu.z]))

    def test_release_V5_large_read_burst_preserves_final_observation(self):
        from test_receiver_sessions import oem_range
        from test_receiver_observations import measurement,marker
        for family in ['sbf','novatel']:
            prefix=(sbf(8190,bytes(512)) if family=='sbf' else oem(65534,bytes(512)))*130
            final=measurement()+marker() if family=='sbf' else oem_range()
            r=run('V5_burst_'+family,family+'_driver_node',prefix+final,
                  ['-p','topics.observation:=/review/obs'])
            self.assertEqual(len(r['obs']),1)
            self.assertEqual((r['obs'][0].week,r['obs'][0].tow),(2300,100.))

    def test_integrity_in_all_six_nodes(self):
        best=bytearray(72);struct.pack_into('<II3dfI3f',best,0,0,16,35.,139.,100.,30.,61,1.,2.,3.);best[65]=12
        geo=bytearray(88);struct.pack_into('<IHBB3d',geo,0,100000,2300,1,0,.6,2.4,100.);geo[66]=12
        for name,frame in [('ubx',ubx(1,7,pvt(),True)),('novatel',oem(42,best,bad=True)),('sbf',sbf(4007,geo,True))]:
            for mode in ['decoder','driver']:
                with self.subTest(receiver=name,mode=mode):
                    m=run(name+mode+'_bad',name+'_'+mode+'_node',frame)
                    self.assertEqual(len(m['sol']),0)

    def test_binary_payload_never_becomes_nmea(self):
        # A valid sentence remains payload even for unknown binary message IDs
        # and even when its enclosing frame fails its integrity check.
        payload=b''.join(nmea(f'00000{i}.00') for i in range(5))
        for name in ['ubx','sbf','novatel']:
            for mode in ['decoder','driver']:
                frames=[]
                for bad in [False,True]:
                    if name=='ubx': frame=ubx(0x99,0x99,payload,bad)
                    elif name=='sbf': frame=sbf(7000,payload+b'\0'*((-len(payload))%4),bad)
                    else: frame=oem(65000,payload,bad=bad)
                    frames.append(frame)
                extra=['-p','messages.nav_pvt:=false'] if name=='ubx' and mode=='driver' else []
                with self.subTest(receiver=name,mode=mode):
                    m=run('embedded_'+name+mode,name+'_'+mode+'_node',frames,extra)
                    self.assertEqual(len(m['sol']),0)
                    m=run('outside_'+name+mode,name+'_'+mode+'_node',
                          frames+[nmea(f'00001{i}.00') for i in range(4)],extra+['-p','local_origin.mode:=fixed'])
                    self.assertGreater(len(m['sol']),0)
                    self.assertAlmostEqual(m['sol'][-1].pos_enu_org_ecef.x,6378137.,places=5)
                    self.assertAlmostEqual(m['sol'][-1].pos_enu_org_ecef.y,0.,places=5)

    def test_rawx_values_survive_mixed_demux(self):
        header=struct.pack('<dHbBBB2s',100.,2300,18,1,1,1,b'\0\0')
        observation=struct.pack('<ddf4BH6B',21000000.125,110000000.,-1234.,0,1,0,0,10000,45,0,0,0,3,0)
        frame=ubx(2,0x15,header+observation)
        for mode in ['decoder','driver']:
            m=run('rawx_demux_'+mode,'ubx_'+mode+'_node',
                  [ubx(2,0x15,header+observation,bad=True)+nmea('000000.00')+frame[:3],frame[3:]],
                  ['-p','topics.observation:=/review/obs','-p','use_gps_timestamp:=true'])
            self.assertEqual(len(m['obs']),1)
            epoch=m['obs'][0];self.assertEqual((epoch.week,epoch.tow),(2300,100.))
            self.assertEqual(len(epoch.observations),1)
            obs=epoch.observations[0]
            self.assertEqual((obs.system,obs.prn,obs.code_str),('G',1,'1C'))
            self.assertAlmostEqual(obs.p,21000000.125,places=6)
            self.assertAlmostEqual(obs.l,110000000.,places=6)
            self.assertAlmostEqual(obs.d,-1234.,places=6)

    def test_rawx_invalid_component_preserves_other_measurements(self):
        header=struct.pack('<dHbBBB2s',100.,2300,18,1,1,1,b'\0\0')
        observation=struct.pack('<ddf4BH6B',math.nan,110000000.,-1234.,0,1,0,0,10000,45,0,0,0,3,0)
        for mode in ['decoder','driver']:
            m=run('rawx_validity_'+mode,'ubx_'+mode+'_node',ubx(2,0x15,header+observation),
                  ['-p','topics.observation:=/review/obs','-p','use_gps_timestamp:=true'])
            self.assertEqual(len(m['obs']),1)
            epoch=m['obs'][0]; obs=epoch.observations[0]
            self.assertEqual((epoch.week,epoch.tow),(2300,100.))
            self.assertTrue(math.isnan(obs.p))
            self.assertEqual((obs.l,obs.d),(110000000.,-1234.))
            self.assertEqual((obs.system,obs.prn,obs.code_str),('G',1,'1C'))

    def test_novatel_range_component_validity_both_paths(self):
        frames=[]
        for index,pseudorange in enumerate([21000000.125, math.nan]):
            row=struct.pack('<HHdfdffffI',1,0,pseudorange,.25,-110000000.,.125,-1234.,45.,10.,(1<<10)|(1<<11)|(1<<12))
            frames.append(oem(43,struct.pack('<I',1)+row,tow=100000+index*1000))
        for mode in ['decoder','driver']:
            m=run('range_validity_'+mode,'novatel_'+mode+'_node',frames,
                  ['-p','topics.observation:=/review/obs','-p','use_gps_timestamp:=true'])
            self.assertEqual(len(m['obs']),2)
            for index,epoch in enumerate(m['obs']):
                self.assertEqual((epoch.week,epoch.tow),(2300,100.+index))
                obs=epoch.observations[0]
                self.assertEqual((obs.system,obs.prn,obs.code_str),('G',1,'1C'))
                self.assertEqual((obs.l,obs.d,obs.snr),(110000000.,-1234.,45.))
                if index==0: self.assertEqual(obs.p,21000000.125)
                else: self.assertTrue(math.isnan(obs.p))

    def test_sbf_measepoch_validity_and_end_marker_both_paths(self):
        # A missing carrier and zero Doppler must not discard valid code/CN0.
        row=bytearray(20);row[2]=1;row[3]=4
        struct.pack_into('<Ii',row,4,3820130941,0)
        row[14]=128;row[15]=140;struct.pack_into('<H',row,16,65535)
        body=struct.pack('<IH6B',100000,2300,1,20,12,0,0,0)+row
        frames=[sbf(4027,body),sbf(5922,struct.pack('<IH2x',100000,2300))]
        for mode in ['decoder','driver']:
            m=run('measepoch_validity_'+mode,'sbf_'+mode+'_node',frames,
                  ['-p','topics.observation:=/review/obs','-p','use_gps_timestamp:=true'])
            self.assertEqual(len(m['obs']),1)
            epoch=m['obs'][0]; self.assertEqual((epoch.week,epoch.tow),(2300,100.))
            obs=epoch.observations[0]
            self.assertEqual((obs.system,obs.prn,obs.code_str),('G',1,'1C'))
            self.assertAlmostEqual(obs.p,21000000.125,places=6)
            self.assertTrue(math.isnan(obs.l))
            self.assertEqual((obs.d,obs.snr),(0.,45.))

    def test_rawimusx_both_paths(self):
        frames=[oem(1462,struct.pack('<BBHdI6i',0,8,2300,t,0,1,2,3,4,5,6),short=True) for t in [100.,100.01]]
        for mode in ['decoder','driver']:
            m=run('rawimu_'+mode,'novatel_'+mode+'_node',frames)
            self.assertEqual(len(m['raw']),1)
            self.assertAlmostEqual(m['raw'][0].linear_acceleration.x,3*(2**-14)/.01,places=4)

    def test_release_N2_observation_time_policy(self):
        from test_receiver_sessions import oem_range
        for mode in ['decoder','driver']:
            for gps in [False,True]:
                for middle,expected in [(130,[100.,101.,102.]),(20,[100.,102.])]:
                    frames=[]
                    for i,status in enumerate([180,middle,180]):
                        frame=bytearray(oem_range(tow=(100+i)*1000))
                        frame[13]=status
                        frames.append(bytes(frame[:-4])+struct.pack('<I',crc32(frame[:-4])))
                    m=run(f'N2_{mode}_{gps}_{middle}','novatel_'+mode+'_node',frames,
                        ['-p','topics.observation:=/review/obs','-p',f'use_gps_timestamp:={str(gps).lower()}'],chunk_delay=1.5)
                    self.assertEqual([o.tow for o in m['obs']],expected)
                    self.assertTrue(all(o.week==2300 for o in m['obs']))
                    if middle==20:self.assertIn('time_withheld_outputs=1',m['log'])

    def test_novatel_time_quality_and_recovery(self):
        b=bytearray(72);struct.pack_into('<II3dfI3f',b,0,0,16,35.,139.,100.,30.,61,1.,2.,3.);b[65]=12
        for mode in ['decoder','driver']:
            extra=['-p','messages.bestpos:=true'] if mode=='driver' else []
            gps=extra+['-p','use_gps_timestamp:=true']
            with self.subTest(mode=mode):
                m=run('unknown_time_'+mode,'novatel_'+mode+'_node',oem(42,b,status=20),gps)
                self.assertEqual(len(m['sol']),0)
                m=run('host_unknown_'+mode,'novatel_'+mode+'_node',oem(42,b,status=20),extra)
                # N2/Q3: unknown receiver time is withheld in host mode too.
                self.assertEqual(len(m['sol']),0)
                m=run('recover_time_'+mode,'novatel_'+mode+'_node',
                      [oem(42,b,status=20),oem(42,b,tow=101000,status=180)],gps)
                self.assertEqual(len(m['sol']),1)
                self.assertEqual(m['sol'][0].time_week,2300)
                self.assertEqual(m['sol'][0].time_tow,101.)

    def test_bestpos_geometry(self):
        b=bytearray(72);struct.pack_into('<II3dfI3f',b,0,0,16,35.,139.,100.,30.,61,1.,2.,3.);b[64]=18;b[65]=12
        m=run('bestpos','novatel_decoder_node',oem(42,b))
        self.assertEqual(len(m['sol']),1);self.assertEqual(m['sol'][0].altitude,130.);self.assertEqual(m['sol'][0].num_sats,12)

    def test_partial_imu_is_withheld(self):
        m=run('partial','ubx_decoder_node',ubx(16,3,b'\0'*4+struct.pack('<II',(16<<24)|1024,1000)))
        self.assertEqual(len(m['raw']),0)

    def test_dop_does_not_select_binary_solution(self):
        dop=ubx(1,4,struct.pack('<I7H',100000,*([100]*7)))
        m=run('dop_nmea','ubx_decoder_node',[dop]+[nmea(f'00000{i}.00') for i in range(5)])
        self.assertGreater(len(m['sol']),0)

    def test_esf_ins_uses_measurement_epoch(self):
        rawx=ubx(2,0x15,struct.pack('<dHbBBB2s',100.,2300,18,0,1,1,b'\0\0'))
        ins=bytearray(36);struct.pack_into('<I',ins,0,0x3f00);struct.pack_into('<I',ins,8,100000)
        m=run('esf_time','ubx_decoder_node',rawx+ubx(1,7,pvt())+ubx(16,0x15,ins),['-p','use_gps_timestamp:=true'])
        self.assertEqual(len(m['sol']),1);self.assertEqual(len(m['imu']),1)
        self.assertEqual(m['sol'][0].header.stamp,m['imu'][0].header.stamp)

    def test_rtcm_msm_fragments_historical_time_and_duplicates(self):
        path=pathlib.Path(TEMP.name)/'msm.rtcm3'
        subprocess.run([B+'make_rtcm_fixture',str(path)],check=True)
        data=path.read_bytes()
        # Split in a frame header, not only at a message boundary; replay the
        # complete epoch to exercise duplicate suppression across reads.
        m=run('rtcm_msm','rtcm_decoder_node',[data[:2],data[2:],data],
              ['-p','reference_week:=2300','-p','reference_tow:=100.0','-p','use_gps_timestamp:=true'])
        self.assertEqual(len(m['obs']),1)
        epoch=m['obs'][0];self.assertEqual((epoch.week,epoch.tow),(2300,100.))
        self.assertEqual(epoch.header.stamp.sec,1707004882)
        self.assertEqual({o.system for o in epoch.observations},{'G','E'})
        for observation in epoch.observations:
            self.assertAlmostEqual(observation.p,21000000.125,places=2)
            self.assertAlmostEqual(observation.d,-1234.5,places=2)

    def test_rtcm_galileo_ten_bit_iod_and_observations(self):
        path=pathlib.Path(TEMP.name)/'navigation.rtcm3'
        subprocess.run([B+'make_rtcm_fixture',str(path),'navigation'],check=True)
        m=run('rtcm_nav','rtcm_decoder_node',path.read_bytes(),
              ['-p','reference_week:=2300','-p','reference_tow:=100.0',
               '-p','topics.ephemeris:=/review/nav'])
        self.assertEqual(len(m['obs']),1)
        self.assertEqual({o.system for o in m['obs'][0].observations},{'G','E'})
        records=[e for snapshot in m['nav'] for e in snapshot.gnss_ephemeris]
        self.assertTrue(records)
        self.assertEqual({e.iode for e in records if e.system=='E'},{255,256,1023})
        self.assertEqual({e.iode for e in records if e.system=='G'},{7})
        glonass=[g for snapshot in m['nav'] for g in snapshot.glonass_ephemeris]
        self.assertTrue(glonass)
        self.assertEqual((glonass[-1].flags,glonass[-1].sva,glonass[-1].svh),(511,7,8))
        self.assertEqual(glonass[-1].frq,-7)
        self.assertNotIn('Ephemeris omitted',pathlib.Path(TEMP.name,'rtcm_nav.log').read_text())

    def test_covariance_order_on_both_paths(self):
        cov=bytearray(64);struct.pack_into('<I',cov,0,100000);cov[5]=1
        struct.pack_into('<6f',cov,16,4.,0.,0.,9.,0.,16.)
        for mode in ['decoder','driver']:
            for covariance_first in [False,True]:
                frames=[ubx(1,7,pvt()),ubx(1,0x36,cov)]
                if covariance_first:frames.reverse()
                m=run('cov_'+mode+str(covariance_first),'ubx_'+mode+'_node',b''.join(frames),
                      ['-p','messages.nav_cov:=true'] if mode=='driver' else [])
                self.assertEqual(len(m['sol']),1)
                self.assertEqual(list(m['sol'][0].pos_enu_cov),[9.,0.,0.,0.,4.,0.,0.,0.,16.])

    def test_nmea_explicit_reference_date(self):
        gga=[nmea(f'00000{i}.00').split(b'\r\n')[1]+b'\r\n' for i in range(4)]
        m=run('gga_no_date','ubx_decoder_node',gga,['-p','use_gps_timestamp:=true'])
        self.assertEqual(len(m['sol']),0)
        m=run('gga_reference','ubx_decoder_node',gga,
              ['-p','use_gps_timestamp:=true','-p','nmea.reference_date:=[2024,2,4]'])
        self.assertGreater(len(m['sol']),0)
        for s in m['sol']:
            self.assertEqual(s.time_week,2300)
            self.assertGreaterEqual(s.time_tow,18.)
            self.assertLess(s.time_tow,22.)

    def test_week_zero_tow(self):
        rawx=ubx(2,0x15,struct.pack('<dHbBBB2s',604799.9,2300,18,0,1,1,b'\0\0'))
        m=run('rollover','ubx_decoder_node',rawx+ubx(1,7,pvt(0)),['-p','use_gps_timestamp:=true'])
        self.assertEqual(len(m['sol']),1);self.assertEqual(m['sol'][0].time_week,2301);self.assertEqual(m['sol'][0].time_tow,0.)

if __name__=='__main__':
    unittest.main(argv=[sys.argv[0]]+unittest_args,verbosity=2)
