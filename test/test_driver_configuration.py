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
        self.receiver('ubx','ack',['-p','generation:=G9'])
        gsv=[p for p in self.last_packets if p[2:4]==bytes([6,1]) and p[6:8]==bytes([240,3])]
        self.assertEqual(len(gsv),1)
        self.assertEqual(gsv[0][8:14],bytes(6))

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
                 gnss_poll_payload=None,port_prompt="COM1"):
        post_ack=b''
        if behavior=='ack_stream':
            from test_receiver_observations import measurement,marker
            post_ack=measurement()+marker()
        with tempfile.TemporaryDirectory(prefix='gnss_commands_') as directory:
            server=socket.socket();server.bind(('127.0.0.1',0));server.listen();server.settimeout(5)
            slave=None
            source=f'tcpcli://127.0.0.1:{server.getsockname()[1]}'
            if serial:
                master,slave=os.openpty();tty.setraw(slave)
                source='serial://'+os.ttyname(slave)+':115200:8:n:1:off'
            command=[str(pathlib.Path(args.bin_dir)/(family+'_driver_node')),'--ros-args',
                     '-p','stream_path:='+source]
            if family=='ubx':command+=['-p','generation:=G10']
            if extra:command+=extra
            path=pathlib.Path(directory)/'node.log';env=dict(os.environ,ROS_LOG_DIR=directory)
            with path.open('w') as log:
                process=subprocess.Popen(command,stdout=log,stderr=log,env=env)
                accepted=0;commands=[];packets=[]
                try:
                    connection=PtyConnection(master) if serial else server.accept()[0]
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
                                    response=ubx(cls,mid,b'\0'*40)
                                    if behavior=='truncated':response=response[:4]
                                elif cls==6 and mid==0x3e and len(packet)==8:
                                    # U2 pp208-209: enable plus GPS L1CA signal bit.
                                    # An enabled system must select at least one signal.
                                    default=bytes([0,32,32,1,0,8,16,0])+struct.pack('<I',0x10001)
                                    response=ubx(cls,mid,default if gnss_poll_payload is None else gnss_poll_payload)
                                elif mid==4:continue  # CFG-RST has no ACK
                                else:
                                    accepted+=1
                                    reject=behavior=='nak' or (behavior=='optional_nak' and mid==0x24) or (behavior=='valset_nak' and mid==0x8a and any(struct.pack('<I',0x20910114+port) in packet for port in (1,2,3)))
                                    response=ubx(5,0 if reject else 1,bytes([cls,mid]))
                                    if behavior=='wrong_ack':response=ubx(5,1,bytes([cls,mid^0xff]))
                                connection.sendall(response)
                        else:
                            while b'\n' in buffer:
                                line,buffer=buffer.split(b'\n',1);line=line.strip()
                                if not line:continue
                                accepted+=1;commands.append(line.decode())
                                if behavior=='timeout':continue
                                if family=='sbf':response=b'$R? Invalid command\r\n' if behavior=='nak' else b'$R: '+line+b'\r\n'+port_prompt.encode()+b'>'
                                else:response=b'<ERROR:Invalid command\r\n' if behavior=='nak' else b'<OK\r\n['+port_prompt.encode()+b']'
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
                    if behavior=='ack_stream':
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
            if behavior in ('ack','ack_split','ack_stream','optional_nak','valset_nak'):
                self.assertEqual(process.returncode,0,text)
                self.assertIn('Driver initialized',text)
                self.assertGreater(accepted,0)
                if family=='sbf':
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
                    self.receiver(family,'ack_split',port_prompt=port)

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

    def test_generation_port_and_requested_intervals(self):
        self.receiver('ubx','ack',['-p','generation:=G9','-p','rate_hz:=10'])
        self.assertTrue(any(p[2:4]==b'\x06\x08' and struct.unpack_from('<H',p,6)[0]==100 for p in self.last_packets))
        text=self.receiver('sbf','ack',['-p','receiver_port:=COM2','-p','rate_hz:=20'])
        self.assertIn('COM2',text);self.assertIn('msec50',text)
        self.receiver('novatel','ack',['-p','receiver_port:=COM2','-p','rate_hz:=5'])
        self.assertTrue(any('LOG COM2 RANGECMPB ONTIME 0.2' in c for c in self.last_commands),self.last_commands)

    def test_optional_ubx_rejection_reports_incomplete_configuration(self):
        text=self.receiver('ubx','optional_nak',['-p','generation:=G9'])
        self.assertIn('Receiver configuration is incomplete',text)

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
