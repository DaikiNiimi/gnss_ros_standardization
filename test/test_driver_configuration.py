#!/usr/bin/env python3
"""Fake receiver command channel; exercises actual driver startup and failures."""
import argparse
import os
import pathlib
import resource
import tty
import select
import signal
import socket
import struct
import subprocess
import tempfile
import time
import unittest

parser=argparse.ArgumentParser();parser.add_argument('--bin-dir',required=True)
args,_=parser.parse_known_args()
# Child executables must use this build's libraries, even if an older workspace
# install was sourced for Python message imports.
os.environ['LD_LIBRARY_PATH']=str(pathlib.Path(args.bin_dir).resolve())+os.pathsep+os.environ.get('LD_LIBRARY_PATH','')
resource.setrlimit(resource.RLIMIT_CORE,(0,0))
os.environ.setdefault('ROS_DOMAIN_ID',str(110+os.getpid()%100))
os.environ['ROS_LOCALHOST_ONLY']='1'

def ubx(cls,mid,payload):
    packet=bytes([cls,mid])+struct.pack('<H',len(payload))+payload
    a=b=0
    for c in packet:a=(a+c)&255;b=(b+a)&255
    return b'\xb5\x62'+packet+bytes([a,b])

SBF_ESCAPE=b'S'*10  # Septentrio escape to command mode
def stream_table(kind,streams,count=10):
    # Fake getSBFOutput/getNMEAOutput reply lines; streams maps N to a port or (port, list, interval).
    lines=b''
    for name in ['Stream%d'%n for n in range(1,count+1)]+(['Res%d'%n for n in range(1,5)] if kind=='SBF' else []):
        entry=(streams or {}).get(int(name[6:]) if name.startswith('Stream') else None,'none')
        port,items,interval=(entry,'none','off') if entry=='none' else (entry,'MeasEpoch+MeasExtra','msec100') if isinstance(entry,str) else entry
        lines+=('  %sOutput, %s, %s, %s, %s\r\n'%(kind,name,port,items,interval)).encode()
    return lines

VALSET_PROTVER='33.40'  # e.g. ZED-F9R HPS 1.40: CFG-VALSET
LEGACY_PROTVER='18.00'  # e.g. NEO-M8: legacy UBX-CFG only

class PtyConnection:
    def __init__(self,fd):self.fd=fd
    def fileno(self):return self.fd
    def recv(self,n):return os.read(self.fd,n)
    def sendall(self,data):
        while data:data=data[os.write(self.fd,data):]
    def close(self):os.close(self.fd)

class DriverConfiguration(unittest.TestCase):
    def test_release_V6_optional_output_nak_and_legacy_gsv(self):
        text=self.receiver('ubx','valset_nak')
        self.assertNotIn('configuration is incomplete',text)
        self.receiver('ubx','ack',ubx_protver=LEGACY_PROTVER)
        gsv=[p for p in self.last_packets if p[2:4]==bytes([6,1]) and p[6:8]==bytes([240,3])]
        # U2 UBX-CFG-MSG: 2-byte poll, then the 8-byte per-port form: GSV off on
        # the connected port (USB, index 3) and on every other port.
        self.assertEqual([g[6:-2] for g in gsv],[bytes([240,3]),bytes([240,3,0,0,0,0,0,0])])

    def test_release_V3_nmea_follows_requested_rate(self):
        for family in ['sbf','novatel']:
            self.receiver(family,'ack',['-p','rate_hz:=5','-p',('messages.nmea_gpgga:=true' if family=='novatel' else 'messages.nmea_gga:=true')])
            commands='\n'.join(self.last_commands)
            if family=='sbf':
                self.assertTrue(any('GGA' in c and 'msec200' in c for c in self.last_commands),commands)
            else:
                self.assertTrue(any('GPGGA' in c and 'ONTIME 0.2' in c for c in self.last_commands),commands)

    def test_release_N3_no_redundant_unlog(self):
        # Fake rejects per-log UNLOG just as a receiver with no active log does.
        self.receiver('novatel','ack')

    def receiver(self,family,behavior,extra=None,serial=False,output=None,reconnect_output=None,
                 gnss_poll_payload=None,port_prompt=None,ubx_protver=VALSET_PROTVER,ubx_port=3,
                 sbf_streams=None,nmea_streams=None,ubx_signals=None):
        # The prompt names the receiver port of this connection (SBF default receiver_port).
        port_prompt=port_prompt or ('USB1' if family=='sbf' else 'COM1')
        post_ack=b'';escaped=False;escapes=0
        if behavior=='ack_stream':
            from test_receiver_observations import measurement,marker
            post_ack=measurement()+marker()
        if behavior=='stale_buffer':
            # Output the receiver buffered before the session, then a fresh epoch after the prompt.
            from test_receiver_observations import measurement,marker
            stale=[measurement(50000)+marker(50000),measurement(60000)+marker(60000)]
            fresh=measurement(100000)+marker(100000)
        with tempfile.TemporaryDirectory(prefix='gnss_commands_') as directory:
            server=socket.socket();server.bind(('127.0.0.1',0));server.listen();server.settimeout(5)
            slave=None
            source=f'tcpcli://127.0.0.1:{server.getsockname()[1]}'
            if serial:
                master,slave=os.openpty();tty.setraw(slave)
                source='serial://'+os.ttyname(slave)+':115200:8:n:1:off'
            command=[str(pathlib.Path(args.bin_dir)/(family+'_driver_node')),'--ros-args',
                     '-p','stream_path:='+source,'--log-level',family+'_driver_node:=debug']
            if extra:command+=extra
            path=pathlib.Path(directory)/'node.log';env=dict(os.environ,ROS_LOG_DIR=directory)
            with path.open('w') as log:
                process=subprocess.Popen(command,stdout=log,stderr=log,env=env)
                accepted=0;commands=[];packets=[]
                try:
                    connection=PtyConnection(master) if serial else server.accept()[0]
                    if behavior=='stale_buffer':connection.sendall(stale[0])
                    buffer=b'';end=time.monotonic()+30
                    while process.poll() is None and time.monotonic()<end:
                        if 'Driver initialized' in path.read_text() or 'Driver started' in path.read_text():break
                        if not select.select([connection],[],[],.05)[0]:continue
                        data=connection.recv(4096)
                        if not data:break
                        buffer+=data
                        if family=='ubx':
                            while len(buffer)>=6:
                                self.assertEqual(buffer[:2],b'\xb5\x62')
                                n=struct.unpack_from('<H',buffer,4)[0]+8
                                if len(buffer)<n:break
                                packet,buffer=buffer[:n],buffer[n:];cls,mid=packet[2:4]
                                packets.append(packet)
                                if behavior=='timeout':continue
                                if cls==10:
                                    # U2 UBX-MON-VER: swVersion[30], hwVersion[10], 30-byte extensions.
                                    extension=b'' if ubx_protver is None else ('PROTVER='+ubx_protver).encode().ljust(30,b'\0')
                                    response=ubx(cls,mid,b'\0'*40+extension)
                                    if behavior=='truncated':response=response[:4]
                                elif cls==6 and mid==0 and len(packet)==8:
                                    # U2 UBX-CFG-PRT poll without payload: the port it arrives on.
                                    response=(ubx(5,0,bytes([6,0])) if ubx_port is None else
                                              ubx(6,0,bytes([ubx_port])+b'\0'*19)+ubx(5,1,bytes([6,0])))
                                elif cls==6 and mid==1 and len(packet)==10:
                                    # U2 UBX-CFG-MSG poll: rates on I2C, UART1, UART2, USB, SPI, reserved.
                                    response=ubx(6,1,packet[6:8]+bytes([0,1,0,1,0,0]))+ubx(5,1,bytes([6,1]))
                                elif cls==6 and mid==0x8b:
                                    # CFG-VALGET (RAM): one value per key; a key the receiver lacks NAKs the
                                    # whole poll. ubx_signals maps key->value (None: every key reads 0).
                                    keys=[struct.unpack_from('<I',packet,10+4*i)[0] for i in range((len(packet)-12)//4)]
                                    state={k:0 for k in keys} if ubx_signals is None else ubx_signals
                                    if any(k not in state for k in keys):response=ubx(5,0,bytes([6,0x8b]))
                                    else:response=ubx(6,0x8b,bytes([1,0,0,0])+b''.join(struct.pack('<IB',k,state[k]) for k in keys))+ubx(5,1,bytes([6,0x8b]))
                                elif cls==6 and mid==0x3e and len(packet)==8:
                                    # U2 pp208-209: enable plus GPS L1CA signal bit.
                                    # An enabled system must select at least one signal.
                                    default=bytes([0,32,32,1,0,8,16,0])+struct.pack('<I',0x10001)
                                    response=ubx(cls,mid,default if gnss_poll_payload is None else gnss_poll_payload)
                                elif mid==4:continue  # CFG-RST has no ACK
                                else:
                                    accepted+=1
                                    reject=behavior=='nak' or (behavior=='optional_nak' and mid==0x24) or (behavior=='valset_nak' and mid==0x8a and any(struct.pack('<I',0x20910114+port) in packet for port in (1,2,3)))
                                    # An F9P has no ESF: enabling ESF-RAW output on USB is rejected.
                                    reject=reject or (behavior=='esf_raw_nak' and mid==0x8a and struct.pack('<IB',0x2091029f+3,1) in packet)
                                    response=ubx(5,0 if reject else 1,bytes([cls,mid]))
                                    if behavior=='wrong_ack':response=ubx(5,1,bytes([cls,mid^0xff]))
                                connection.sendall(response)
                        else:
                            if family=='sbf' and SBF_ESCAPE in buffer:
                                # Escape to command mode; the prompt has no line terminator.
                                buffer=buffer.replace(SBF_ESCAPE,b'',1);escaped=True;escapes+=1
                                if behavior=='false_prompt' and escapes==1:
                                    # Buffered binary that is not a valid SBF block, with a prompt-like sequence.
                                    connection.sendall(b'\x00\x13\xffAB3>\x00\x81'+port_prompt.encode()+b'>')
                                elif behavior=='stale_buffer':
                                    connection.sendall(stale[1]+port_prompt.encode()+b'>'+fresh)
                                elif behavior not in ('timeout','escape_silent','readonly_silent'):
                                    connection.sendall(port_prompt.encode()+b'>')
                            while b'\n' in buffer:
                                line,buffer=buffer.split(b'\n',1);line=line.strip()
                                if not line:continue
                                accepted+=1;commands.append(line.decode())
                                if behavior=='timeout':continue
                                # Input mode 'auto' after RTCM input: commands are ignored until the escape.
                                if behavior=='escape_required' and not escaped:continue
                                if behavior=='readonly_silent':continue
                                if family=='sbf':response=b'$R? Invalid command\r\n' if behavior=='nak' else b'$R: '+line+b'\r\n'+port_prompt.encode()+b'>'
                                else:response=b'<ERROR:Invalid command\r\n' if behavior=='nak' else b'<OK\r\n['+port_prompt.encode()+b']'
                                for query,kind,streams in [(b'getSBFOutput','SBF',sbf_streams),(b'getNMEAOutput','NMEA',nmea_streams)]:
                                    if family=='sbf' and line==query and behavior!='nak':
                                        # One line per stream after the ACK; 'unreadable' = ACK without a table.
                                        table=b'' if streams=='unreadable' else stream_table(kind,streams)
                                        response=b'$R: '+line+b'\r\n'+table+port_prompt.encode()+b'>'
                                if family=='sbf' and line.startswith(b'exeSBFOnce') and behavior=='once_nak':
                                    response=b'$R? '+line+b': Invalid command!\r\n'+port_prompt.encode()+b'>'
                                if family=='novatel' and behavior=='readonly_nak' and b'GALEPHEMERISB' in line:
                                    response=b'<ERROR:Message is not available\r\n['+port_prompt.encode()+b']'
                                if family=='novatel' and line.startswith(b'UNLOG '):
                                    response=b'<ERROR:Requested log does not exist\r\n['+port_prompt.encode()+b']'
                                if behavior=='ack_split':
                                    reply,prompt=response.rsplit(b'\r\n',1)
                                    connection.sendall(reply+b'\r\n')
                                    time.sleep(.02)
                                    connection.sendall(prompt)
                                else:connection.sendall(response)
                                if post_ack and b'MeasEpoch' in line:
                                    time.sleep(.05)
                                    connection.sendall(post_ack)
                                    post_ack=b''
                    if output is not None:
                        for frame in output:
                            connection.sendall(frame);time.sleep(.05)
                        time.sleep(5.1) # Q1: production diagnostic interval
                    if reconnect_output is not None:
                        connection.close();server.settimeout(15)
                        connection,_=server.accept()
                        # Reconnecting transports do not replay startup commands.
                        self.assertFalse(select.select([connection],[],[],.3)[0])
                        connection.sendall(reconnect_output);time.sleep(5.1)
                    if behavior in ('ack_stream','stale_buffer'):
                        time.sleep(.3)  # allow the poll timer to publish after startup
                    connection.close()
                finally:
                    if process.poll() is None:process.send_signal(signal.SIGINT)
                    try:process.wait(timeout=5)
                    except subprocess.TimeoutExpired:process.kill();process.wait()
                    server.close()
                    if slave is not None:os.close(slave)
            text=path.read_text()
            artifact=os.environ.get('GNSS_TEST_ARTIFACT_DIR')
            if artifact:
                target=pathlib.Path(artifact);target.mkdir(parents=True,exist_ok=True)
                (target/(family+'_'+behavior+'_'+str(time.monotonic_ns())+'.log')).write_text(text)
            if behavior in ('ack','ack_split','ack_stream','optional_nak','valset_nak','esf_raw_nak','escape_required','escape_silent',
                            'stale_buffer','once_nak','readonly','readonly_silent','readonly_nak','false_prompt'):
                self.assertEqual(process.returncode,0,text)
                self.assertIn('Driver initialized',text)
                if not behavior.startswith('readonly'):self.assertGreater(accepted,0)
                if family=='sbf' and not behavior.startswith('readonly'):
                    self.assertTrue(any('MeasEpoch' in c and 'EndOfMeas' in c for c in commands),commands)
            else:
                self.assertEqual(process.returncode,1,text)
                self.assertNotIn('Receiver configuration complete',text)
            self.last_commands=commands;self.last_packets=packets
            return text

    def test_ack_and_actual_observation_output_are_separate(self):
        from test_receiver_sessions import rawx,oem_range
        from test_receiver_observations import measurement,marker
        import test_receiver_streams as wire
        for family in ['ubx','sbf','novatel']:
            parameters=['-p','rate_hz:=5']
            make={'ubx':lambda t:rawx(t),'sbf':lambda t:measurement(round(t*1000))+marker(round(t*1000)),
                  'novatel':lambda t:oem_range(round(t*1000))}[family]
            unknown={'ubx':ubx(0xff,0xff,b''),'sbf':wire.sbf(8190,b'\0'*8),
                     'novatel':wire.oem(65534,b'')}[family]
            for name,frames,expected in [
                ('absent',[],'state=no_input'),
                ('other',[unknown],'state=observations_not_observed'),
                ('wrong_rate',[make(100),make(101)],'rate_matches_request=no'),
                ('correct_rate',[make(100),make(100.2)],'rate_matches_request=yes')]:
                with self.subTest(family=family,case=name):
                    text=self.receiver(family,'ack',parameters,output=frames)
                    self.assertIn(expected,text)
                    if name=='absent':self.assertIn('No receiver input',text)
                    if 'rate' in name:self.assertIn('published_epochs=2',text)

    def test_reconnect_keeps_startup_only_configuration_and_resumes_data(self):
        from test_receiver_sessions import rawx,oem_range
        from test_receiver_observations import measurement,marker
        for family,frame in [('ubx',rawx()),('sbf',measurement()+marker()),('novatel',oem_range())]:
            with self.subTest(family=family):
                text=self.receiver(family,'ack',[],reconnect_output=frame)
                self.assertIn('Configuration is startup-only',text)
                self.assertIn('last_epoch=2300:100',text)
                self.assertIn('published_epochs=1',text)

    def test_command_port_prompt_matrix(self):
        for family,ports in [('sbf',['COM1','USB1','IP10','IPS1','IPR1','BT10']),
                             ('novatel',['COM1','USB1','ICOM1','XCOM1','AUX'])]:
            for port in ports:
                with self.subTest(family=family,port=port):
                    extra=['-p','receiver_port:='+port] if family=='sbf' else None
                    self.receiver(family,'ack_split',extra,port_prompt=port)

    def test_split_unterminated_prompts(self):
        for family in ['sbf','novatel']:
            for serial in [False,True]:
                with self.subTest(family=family,serial=serial):
                    self.receiver(family,'ack_split',serial=serial)

    def test_serial_pty_configuration(self):
        for family in ['ubx','sbf','novatel']:
            with self.subTest(family=family):self.receiver(family,'ack',serial=True)

    def test_sbf_binary_immediately_after_configuration_is_not_discarded(self):
        # Reuse an existing frame fixture; this verifies the startup transport
        # path, not independent bit-level specification conformance.
        text=self.receiver('sbf','ack_stream')
        self.assertIn('Published obs: week=2300 tow=100.000 n=1',text)

    def test_port_and_requested_intervals(self):
        self.receiver('ubx','ack',['-p','rate_hz:=10'],ubx_protver=LEGACY_PROTVER)
        self.assertTrue(any(p[2:4]==b'\x06\x08' and struct.unpack_from('<H',p,6)[0]==100 for p in self.last_packets))
        text=self.receiver('sbf','ack',['-p','receiver_port:=COM2','-p','rate_hz:=20'],port_prompt='COM2')
        self.assertIn('COM2',text);self.assertIn('msec50',text)
        self.receiver('novatel','ack',['-p','receiver_port:=COM2','-p','rate_hz:=5'])
        self.assertTrue(any('LOG COM2 RANGECMPB ONTIME 0.2' in c for c in self.last_commands),self.last_commands)

    # Desired CFG-SIGNAL RAM values for the navsys parameters used below.
    SIGNALS={0x1031001f:1,0x10310001:1,0x10310003:1,0x10310004:1,0x10310020:0,0x10310005:0,
             0x10310021:1,0x10310007:1,0x10310009:1,0x1031000a:1,
             0x10310022:1,0x1031000f:1,0x10310028:1,0x1031000d:1,0x1031000e:1,
             0x10310024:1,0x10310012:1,0x10310014:0,0x10310015:1,0x10310017:1,
             0x10310025:1,0x10310018:1,0x1031001a:1,0x10310026:0,0x1031001d:0}
    NAVSYS=['-p','navsys.gps:=true','-p','navsys.glo:=true','-p','navsys.gal:=true','-p','navsys.bds:=true',
            '-p','navsys.qzs:=true','-p','navsys.irn:=false','-p','navsys.sbs:=false']

    def signal_writes(self):
        writes=[]
        for p in self.last_packets:
            if p[2:4]!=bytes([6,0x8a]):continue
            key=struct.unpack_from('<I',p,10)[0]
            if key>>16==0x1031:writes.append((key,p[14]))
        return writes,[p for p in self.last_packets if p[2:4]==bytes([6,4])]

    def test_ubx_gnss_restart_only_when_the_signal_configuration_changes(self):
        text=self.receiver('ubx','ack',self.NAVSYS,ubx_signals=dict(self.SIGNALS))
        self.assertEqual(self.signal_writes(),([],[]));self.assertIn('GNSS restart not needed',text)
        changed=dict(self.SIGNALS);changed[0x10310003]=0  # GPS L2C off in the receiver
        self.receiver('ubx','ack',self.NAVSYS,ubx_signals=changed)
        writes,resets=self.signal_writes()
        self.assertEqual(writes,[(0x10310003,1)]);self.assertEqual(len(resets),1)
        # F9P: no L5/B1C/B2a/NavIC keys. Unsupported keys requested off are not
        # reported; per-band keys are best effort.
        f9p={k:v for k,v in self.SIGNALS.items() if k not in (0x10310004,0x10310009,0x1031000f,0x10310028,0x10310017,0x10310026,0x1031001d)}
        text=self.receiver('ubx','ack',self.NAVSYS,ubx_signals=f9p)
        self.assertEqual(self.signal_writes(),([],[]))
        self.assertNotIn('Receiver configuration is incomplete',text)
        text=self.receiver('ubx','ack',self.NAVSYS+['-p','navsys.irn:=true'],ubx_signals=f9p)
        self.assertIn('navsys.irn (CFG-SIGNAL-NAVIC_ENA)',text)
        # Legacy CFG-GNSS: the polled block already has the requested enable bit.
        gps_only=bytes([0,32,32,1])+struct.pack('<4BI',0,8,16,0,0x10001)
        self.receiver('ubx','ack',['-p','navsys.gps:=true'],gnss_poll_payload=gps_only,ubx_protver=LEGACY_PROTVER)
        self.assertEqual([p[6:-2] for p in self.last_packets if p[2:4]==bytes([6,0x3e])],[b''])
        self.assertEqual(self.signal_writes()[1],[])

    def test_ubx_rejected_request_is_named(self):
        text=self.receiver('ubx','esf_raw_nak',['-p','messages.esf_raw:=true'])
        self.assertIn('Receiver configuration is incomplete; receiver rejected: messages.esf_raw (UBX-ESF-RAW output)',text)

    def test_optional_ubx_rejection_reports_incomplete_configuration(self):
        text=self.receiver('ubx','optional_nak',ubx_protver=LEGACY_PROTVER)
        self.assertIn('Receiver configuration is incomplete',text)

    def test_sbf_escapes_to_command_mode_before_configuration(self):
        for serial in (False,True):
            with self.subTest(serial=serial):
                text=self.receiver('sbf','escape_required',serial=serial)
                self.assertIn('Connected to receiver port USB1',text)
        # The port name is compared case-insensitively.
        self.receiver('sbf','ack',['-p','receiver_port:=usb1'])

    def test_sbf_prompt_of_another_port_stops_configuration(self):
        text=self.receiver('sbf','wrong_port',port_prompt='USB2')
        self.assertIn('USB2',text);self.assertIn('receiver_port is USB1',text)
        self.assertIn('receiver_port must be the receiver port this stream is connected to',text)
        self.assertEqual(self.last_commands,[])

    def test_sbf_prompt_like_binary_is_confirmed_by_a_second_escape(self):
        text=self.receiver('sbf','false_prompt')
        self.assertIn('Connected to receiver port USB1',text)
        self.assertNotIn('AB3',text)

    def test_sbf_without_prompt_still_sends_commands(self):
        self.receiver('sbf','escape_silent')
        text=self.receiver('sbf','timeout')
        self.assertIn('SBF stream configuration could not be read',text)

    def test_sbf_imu_uses_its_own_stream_at_the_imu_rate(self):
        self.receiver('sbf','ack',['-p','messages.ext_sensor_meas:=true','-p','rate_hz:=10'])
        self.assertIn('sso, Stream2, USB1, ExtSensorMeas, OnChange',self.last_commands)
        stream1=[c for c in self.last_commands if c.startswith('sso, Stream1,')]
        self.assertEqual(len(stream1),1,self.last_commands)
        self.assertNotIn('ExtSensorMeas',stream1[0]);self.assertIn('msec100',stream1[0])

    def sbf_sets(self):
        return [c for c in self.last_commands if c.startswith(('sso,','sno,'))]

    def test_sbf_streams_of_other_ports_are_never_written(self):
        # A disk log on Stream1 with a long block list (lines of several hundred characters).
        log=('DSK1','+'.join(['MeasEpoch','MeasExtra','GEORawL1','GPSRawCA','GLORawCA','GALRawFNAV',
                               'GALRawINAV','BDSRaw','QZSRawL1CA','NAVICRaw','PVTGeodetic','ReceiverStatus']*3),'msec100')
        self.receiver('sbf','ack',sbf_streams={1:log,4:('COM2','PVTGeodetic','sec1')})
        main=[c for c in self.last_commands if c.startswith('sso, Stream2, USB1, MeasEpoch')]
        self.assertEqual(len(main),1,self.last_commands)
        self.assertFalse([c for c in self.sbf_sets() if c.startswith(('sso, Stream1,','sso, Stream4,'))],self.last_commands)
        text=self.receiver('sbf','ack',['-p','messages.ext_sensor_meas:=true'],sbf_streams={1:log})
        self.assertIn('sso, Stream3, USB1, ExtSensorMeas, OnChange',self.last_commands)
        self.assertTrue(any(c.startswith('sso, Stream2, USB1, MeasEpoch') for c in self.last_commands))
        self.assertFalse([c for c in self.sbf_sets() if c.startswith('sso, Stream1,')])
        self.assertIn('SBF output: Stream2 (GNSS), Stream3 (IMU) on USB1',text)

    def test_sbf_own_leftover_streams_are_switched_off(self):
        self.receiver('sbf','ack',sbf_streams={1:'USB1',2:('USB1','ExtSensorMeas','OnChange')})
        self.assertTrue(any(c.startswith('sso, Stream1, USB1, MeasEpoch') for c in self.last_commands))
        self.assertIn('sso, Stream2, none, none, off',self.last_commands)
        self.receiver('sbf','ack',['-p','messages.ext_sensor_meas:=true'],
                      sbf_streams={1:'DSK1',2:'USB1',3:('USB1','ExtSensorMeas','OnChange')})
        self.assertTrue(any(c.startswith('sso, Stream2, USB1, MeasEpoch') for c in self.last_commands))
        self.assertIn('sso, Stream3, USB1, ExtSensorMeas, OnChange',self.last_commands)
        self.assertFalse([c for c in self.sbf_sets() if 'none, none, off' in c or c.startswith('sso, Stream1,')])

    def test_sbf_without_free_stream_or_readable_table_sets_nothing(self):
        text=self.receiver('sbf','ack_fail',sbf_streams={n:'COM%d'%n for n in range(1,11)})
        self.assertIn('No free SBF stream for USB1',text);self.assertIn('Stream1: COM1',text)
        self.assertEqual(self.sbf_sets(),[])
        text=self.receiver('sbf','ack_fail',sbf_streams='unreadable')
        self.assertIn('SBF stream configuration could not be read',text)
        self.assertEqual(self.sbf_sets(),[])

    def test_sbf_nmea_streams_follow_the_same_rule(self):
        self.receiver('sbf','ack',['-p','messages.nmea_rmc:=false','-p','messages.nmea_gsa:=false',
                                   '-p','messages.nmea_gst:=false'],nmea_streams={1:('COM2','RMC','sec1')})
        self.assertIn('sno, Stream2, USB1, GGA, sec1',self.last_commands)
        self.assertFalse([c for c in self.sbf_sets() if c.startswith('sno, Stream1,')])
        off=['-p','messages.nmea_gga:=false','-p','messages.nmea_rmc:=false','-p','messages.nmea_gsa:=false',
             '-p','messages.nmea_gst:=false']
        self.receiver('sbf','ack',off,nmea_streams={1:('COM2','RMC','sec1'),2:('USB1','GGA','sec1')})
        self.assertEqual([c for c in self.sbf_sets() if c.startswith('sno,')],['sno, Stream2, none, none, off'])
        text=self.receiver('sbf','ack',off,nmea_streams='unreadable')
        self.assertIn('NMEA stream configuration could not be read',text)
        self.assertFalse([c for c in self.sbf_sets() if c.startswith('sno,')])
        text=self.receiver('sbf','ack_fail',nmea_streams='unreadable')
        self.assertIn('NMEA stream configuration could not be read',text)
        self.assertEqual(self.sbf_sets(),[])

    def test_sbf_buffered_output_before_the_prompt_is_discarded(self):
        text=self.receiver('sbf','stale_buffer')
        self.assertIn('Published obs: week=2300 tow=100.000',text)
        self.assertNotIn('tow=50.000',text);self.assertNotIn('tow=60.000',text)

    def test_sbf_requests_stored_navigation_once(self):
        nav='GPSNav+GLONav+GALNav+BDSNav+QZSNav+GPSIon+GALIon+BDSIon'
        for extra,expected in [([],'exeSBFOnce, USB1, '+nav),
                               (['-p','messages.glo_nav:=false','-p','messages.bds_nav:=false'],
                                'exeSBFOnce, USB1, GPSNav+GALNav+QZSNav+GPSIon+GALIon'),
                               (['-p','messages.gps_nav:=false','-p','messages.glo_nav:=false','-p','messages.gal_nav:=false',
                                 '-p','messages.bds_nav:=false','-p','messages.qzs_nav:=false'],None)]:
            with self.subTest(expected=expected):
                self.receiver('sbf','ack',extra)
                once=[c for c in self.last_commands if c.startswith('exeSBFOnce')]
                self.assertEqual(once,[expected] if expected else [])
                if expected:self.assertEqual(self.last_commands[-1],expected)
        text=self.receiver('sbf','once_nak')
        self.assertIn('Stored navigation data not requested',text)
        self.assertIn('[WARN]',text)

    def test_sbf_unconfigured_driver_only_requests_stored_navigation(self):
        # configure_on_startup false: no settings change; the request goes to the prompt's port.
        readonly=['-p','configure_on_startup:=false']
        text=self.receiver('sbf','readonly',readonly,port_prompt='USB2')
        self.assertEqual(self.last_commands,['exeSBFOnce, USB2, GPSNav+GLONav+GALNav+BDSNav+QZSNav+GPSIon+GALIon+BDSIon'])
        self.assertNotIn('receiver_port is',text)
        from test_receiver_observations import measurement,marker
        text=self.receiver('sbf','readonly_silent',readonly,output=[measurement()+marker()])
        self.assertEqual(self.last_commands,[])
        self.assertIn('No command prompt received; stored navigation data not requested',text)
        self.assertIn('Published obs: week=2300 tow=100.000',text)

    def test_novatel_unconfigured_driver_only_requests_stored_navigation(self):
        readonly=['-p','configure_on_startup:=false']
        logs=['RAWEPHEMB','GLOEPHEMERISB','GALEPHEMERISB','BDSEPHEMERISB','QZSSRAWEPHEMB','IONUTCB','GALIONOB','QZSSIONUTCB']
        self.receiver('novatel','readonly',readonly)
        self.assertEqual(self.last_commands,['LOG THISPORT '+log+' ONCE' for log in logs])
        self.receiver('novatel','readonly',readonly+['-p','messages.glo_ephem:=false','-p','messages.navic_ephem:=true',
                                                      '-p','messages.ionutc:=false','-p','receiver_port:=COM2'])
        self.assertEqual(self.last_commands,['LOG COM2 '+log+' ONCE' for log in
                                             ['RAWEPHEMB','GALEPHEMERISB','BDSEPHEMERISB','QZSSRAWEPHEMB','NAVICEPHEMERISB']])
        text=self.receiver('novatel','readonly_nak',readonly)
        self.assertEqual(self.last_commands,['LOG THISPORT '+log+' ONCE' for log in logs])
        self.assertIn('Stored navigation request rejected: LOG THISPORT GALEPHEMERISB ONCE',text)

    def test_ascii_ack_and_rejection(self):
        for family in ['sbf','novatel']:
            for behavior in ['ack','nak','timeout']:
                with self.subTest(receiver=family,behavior=behavior):self.receiver(family,behavior)
    def test_ubx_poll_and_ack_integrity(self):
        for behavior in ['ack','truncated','wrong_ack','timeout']:
            with self.subTest(behavior=behavior):self.receiver('ubx',behavior)

if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(DriverConfiguration))
    raise SystemExit(0 if result.wasSuccessful() else 1)
