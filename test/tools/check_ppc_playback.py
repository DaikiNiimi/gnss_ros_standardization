#!/usr/bin/env python3
"""Play each full PPC bag into a raw subscriber and a real rosbag2 recorder.

Compare per-topic counts and ordered payload hashes (zero pad to a 4-byte
CDR boundary: Fast DDS may add up to three trailing padding bytes). Re-recorded
receive timestamps are intentionally different; message header timestamps are
part of the payload and must remain byte-identical. Does not test hardware.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
os.environ.setdefault('ROS_DOMAIN_ID',str(210+os.getpid()%20))
os.environ['ROS_LOCALHOST_ONLY']='1'
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rosidl_runtime_py.utilities import get_message
from rosbag2_interfaces.srv import Resume
import rosbag2_py


def add(stats,topic,data):
    data=data+b'\0'*((-len(data))%4)
    if topic not in stats:stats[topic]=[0,hashlib.sha256()]
    stats[topic][0]+=1;stats[topic][1].update(len(data).to_bytes(8,'little'));stats[topic][1].update(data)

def finish(stats):return {topic:{'count':v[0],'sha256':v[1].hexdigest()} for topic,v in stats.items()}

def inspect(bag):
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id=''),rosbag2_py.ConverterOptions('',''))
    types={t.name:t.type for t in reader.get_all_topics_and_types()};stats={}
    while reader.has_next():
        topic,data,_=reader.read_next();add(stats,topic,data)
    return types,finish(stats)

def stop(process):
    if process.poll() is None:process.send_signal(signal.SIGINT)
    try:process.wait(timeout=15)
    except subprocess.TimeoutExpired:process.kill();process.wait()

def check(bag,output,rate):
    output.mkdir(parents=True,exist_ok=True);types,expected=inspect(bag)
    node=rclpy.create_node('ppc_acceptance');stats={};subscriptions=[];player=recorder=None
    name='ppc_recorder';player_name='ppc_player'
    qos_file=output/'qos.yaml'
    qos_file.write_text(''.join(f'{topic}:\n  reliability: reliable\n  durability: '+
        ('transient_local' if typ=='gnss_ros_standardization/msg/GnssEphemerides' else 'volatile')+'\n  history: keep_last\n  depth: 10000\n' for topic,typ in types.items()))
    for topic,typ in types.items():
        qos=QoSProfile(depth=10000,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL if typ=='gnss_ros_standardization/msg/GnssEphemerides' else DurabilityPolicy.VOLATILE)
        subscriptions.append(node.create_subscription(get_message(typ),topic,lambda data,t=topic:add(stats,t,data),qos,raw=True))
    try:
        with (output/'player.log').open('w') as plog,(output/'recorder.log').open('w') as rlog:
            player=subprocess.Popen(['ros2','bag','play',str(bag),'--rate',str(rate),'--start-paused',
                '--disable-keyboard-controls','--wait-for-all-acked','10000','--read-ahead-queue-size','10000',
                '--qos-profile-overrides-path',str(qos_file),'--remap','__node:='+player_name],stdout=plog,stderr=plog)
            recorder=subprocess.Popen(['ros2','bag','record','--node-name',name,'--storage','mcap',
                '--output',str(output/'recorded'),'--disable-keyboard-controls','--qos-profile-overrides-path',str(qos_file),
                '--topics',*types],stdout=rlog,stderr=rlog)
            resume=node.create_client(Resume,'/'+player_name+'/resume')
            deadline=time.monotonic()+30
            while not (resume.service_is_ready() and all(
                any(info.node_name==name for info in node.get_subscriptions_info_by_topic(t)) for t in types)):
                if player.poll() is not None or recorder.poll() is not None or time.monotonic()>deadline:
                    raise RuntimeError('discovery/startup failed; inspect '+str(output))
                rclpy.spin_once(node,timeout_sec=.05)
            until=time.monotonic()+2
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
            future=resume.call_async(Resume.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=10)
            assert future.done() and future.exception() is None
            started=time.monotonic();deadline=started+600
            while player.poll() is None:
                if time.monotonic()>deadline:raise TimeoutError('playback exceeded 10 minutes')
                rclpy.spin_once(node,timeout_sec=.01)
            assert player.returncode==0,player.returncode
            until=time.monotonic()+2
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
            stop(recorder);assert recorder.returncode==0,recorder.returncode
            actual=finish(stats);_,recorded=inspect(output/'recorded')
            result={'source':str(bag),'rate':rate,'elapsed_s':time.monotonic()-started,
                    'expected':expected,'subscriber':actual,'recorded':recorded,
                    'passed':actual==expected and recorded==expected,
                    'timestamp_scope':'message bytes including headers exact after CDR tail alignment; recorder storage receive timestamps differ',
                    'cdr_tail_padding':'normalized to 4-byte alignment; no message fields removed'}
            (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
            assert result['passed'],str(output/'result.json')
            return {'run':bag.name,'passed':True,'rate':rate,'messages':sum(v['count'] for v in expected.values())}
    finally:
        for process in [player,recorder]:
            if process is not None:stop(process)
        node.destroy_node()

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--candidates',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True);ap.add_argument('--rate',type=float,default=50.)
    ap.add_argument('--single',action='store_true',help='treat --candidates as one bag, for synthetic launch acceptance')
    args=ap.parse_args();rclpy.init();results=[]
    try:
        bags=[args.candidates] if args.single else sorted(
            p for p in args.candidates.iterdir() if p.is_dir() and (p/'metadata.yaml').is_file())
        for bag in bags:
            record=check(bag,args.output/bag.name,args.rate);results.append(record);print(json.dumps(record),flush=True)
        assert len(results)==(1 if args.single else 6),'unexpected number of runs'
        (args.output/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
    finally:rclpy.shutdown()
if __name__=='__main__':main()
