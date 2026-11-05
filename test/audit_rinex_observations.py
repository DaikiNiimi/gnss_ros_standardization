#!/usr/bin/env python3
"""Audit every RINEX 3 signal before RTKLIB selection against a ROS bag.

Independent input parser: RINEX 3.04 tables A2/A3.
No band priority selection, epoch cap, RTKLIB helper or rounded-TOW-only key.
Missing values (blank or zero per A3) may appear as zero or NaN in the current
backend contract. This does not assert preservation of receiver-only metadata.
"""
import argparse
from collections import Counter
from datetime import datetime
from decimal import Decimal
import hashlib
import json
import math
from pathlib import Path

GPS_EPOCH=datetime(1980,1,6)
WEEK_NS=604800_000_000_000

def read_header(stream):
    header=[];types={};counts={};channels={};system=None;time_system=None
    for line in stream:
        header.append(line.rstrip('\n'));label=line[60:].strip()
        if label=='RINEX VERSION / TYPE':
            if not 3 <= float(line[:9]) < 4 or line[20]!='O':raise ValueError('requires RINEX 3 observations')
        elif label=='SYS / # / OBS TYPES':
            if line[0]!=' ':system=line[0];counts[system]=int(line[3:6]);types[system]=[]
            if system is None:raise ValueError('orphan observation header continuation')
            types[system]+=line[7:60].split()
        elif label=='TIME OF FIRST OBS':time_system=line[48:51].strip()
        elif label=='GLONASS SLOT / FRQ #':
            tokens=line[4:60].split()
            for i in range(0,len(tokens),2):channels[tokens[i]]=int(tokens[i+1])+7
        elif label=='END OF HEADER':break
    else:raise ValueError('missing END OF HEADER')
    if time_system!='GPS':raise ValueError('audit currently requires explicit GPS time system')
    if any(len(types[s])!=n for s,n in counts.items()):raise ValueError('incomplete observation type header')
    return {'lines':header,'types':types,'glo_channels':channels,'time_system':time_system}

def epochs(stream,header):
    previous=None
    for line in stream:
        if not line.strip():continue
        if not line.startswith('>'):raise ValueError('missing epoch marker')
        values=line[1:].split();y,mo,d,h,mi=map(int,values[:5]);sec=Decimal(values[5])
        flag,n=int(values[6]),int(values[7])
        if flag not in (0,1):raise ValueError(f'event flag {flag} needs an explicit audit policy')
        ns=int((datetime(y,mo,d,h,mi)-GPS_EPOCH).total_seconds())*10**9+int(sec*10**9)
        if previous is not None and ns<=previous:raise ValueError('non-increasing/duplicate source epoch')
        previous=ns;signals={}
        for _ in range(n):
            row=stream.readline().rstrip('\r\n');sat=row[:3]
            if not sat or sat[0] not in header['types']:raise ValueError('unknown satellite row '+sat)
            # RINEX 3 uses one variable-length record per satellite (A3).
            for i,kind in enumerate(header['types'][sat[0]]):
                field=row[3+16*i:3+16*(i+1)].ljust(16)
                number=field[:14].strip();value=float(number.replace('D','E')) if number else 0.
                if not math.isfinite(value):raise ValueError('nonfinite RINEX value')
                lli=int(field[14]) if field[14].strip() else 0
                if not number and not (kind[0]=='L' and lli):continue
                if kind[0] not in 'CLDS':raise ValueError('unrepresented observation type '+kind)
                signal=signals.setdefault((sat,kind[1:]),{})
                if number:signal[kind[0]]=value
                if kind[0]=='L':signal['lli']=lli
        yield ns,signals

def bag_epochs(path,topic):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from gnss_ros_standardization.msg import GnssObservations
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path),storage_id=''),rosbag2_py.ConverterOptions('',''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    previous=None
    while reader.has_next():
        _,data,stamp=reader.read_next();message=deserialize_message(data,GnssObservations)
        ns=message.week*WEEK_NS+round(message.tow*10**9)
        # floating representation of TOW at ns precision may differ by <1 ns.
        if previous is not None and ns<=previous:raise ValueError('non-increasing/duplicate bag epoch')
        previous=ns;signals={}
        for o in message.observations:
            sat=o.satid
            if sat[:1].isdigit():sat=f'S{int(sat)-100:02d}'
            key=(sat,o.code_str)
            if key in signals:raise ValueError('duplicate bag signal '+str(key))
            signals[key]={'C':o.p,'L':o.l,'D':o.d,'S':o.snr,'lli':o.lli,'freq':o.freq}
        yield ns,signals,stamp,message.header.stamp.sec*10**9+message.header.stamp.nanosec

def compare(source,actual,header,timestamp_scale,require_nan_missing=False):
    counts=Counter();examples=[];missing=Counter();max_error=Counter()
    def issue(kind,epoch,key,expected=None,got=None):
        counts[kind]+=1
        if isinstance(got,float) and not math.isfinite(got):got=str(got)
        if len(examples)<30:examples.append({'kind':kind,'gpst_ns':epoch,'signal':key,'expected':expected,'actual':got})
    current=next(actual,None)
    for ns,expected in source:
        counts['source_epochs']+=1;counts['source_signals']+=len(expected)
        while current is not None and current[0]<ns-1:
            issue('extra_epoch',current[0],None);current=next(actual,None)
        if current is None or abs(current[0]-ns)>1:
            issue('missing_epoch',ns,None);continue
        counts['matched_epochs']+=1;got=current[1];counts['bag_signals']+=len(got)
        if timestamp_scale=='gpst':
            stamp=315964800_000_000_000+ns
            if abs(current[2]-stamp)>1000:issue('bag_timestamp',ns,None,stamp,current[2])
            if abs(current[3]-stamp)>1000:issue('header_timestamp',ns,None,stamp,current[3])
        for key in expected.keys()-got.keys():
            issue('missing_signal',ns,key);missing['/'.join(key)]+=1
        for key in got.keys()-expected.keys():issue('extra_signal',ns,key)
        for key in expected.keys()&got.keys():
            e,a=expected[key],got[key]
            for kind in 'CLDS':
                target=e.get(kind,0.);value=a[kind]
                if require_nan_missing and kind not in e:
                    if not math.isnan(value):issue('blank_not_nan_'+kind,ns,key,None,value)
                    continue
                if target==0:
                    if require_nan_missing:
                        if value!=0:issue('explicit_zero_'+kind,ns,key,0,value)
                        continue
                    if value!=0 and not math.isnan(value):issue('invented_'+kind,ns,key,target,value)
                    continue
                # P/L are doubles; D and S pass through backend float32.
                tolerance=max(1e-6,abs(target)*6e-8) if kind in 'DS' else 1e-6
                if not math.isfinite(value) or abs(value-target)>tolerance:issue('value_'+kind,ns,key,target,value)
                if math.isfinite(value):max_error[kind]=max(max_error[kind],abs(value-target))
            if e.get('lli',0)!=a['lli']:issue('lli',ns,key,e.get('lli',0),a['lli'])
            channel=header['glo_channels'].get(key[0],255)
            if key[0][0]=='R' and channel!=255 and a['freq']!=channel:issue('freq',ns,key,channel,a['freq'])
            if key[0][0]!='R' and a['freq']!=255:issue('non_glonass_freq',ns,key,255,a['freq'])
        current=next(actual,None)
    while current is not None:issue('extra_epoch',current[0],None);current=next(actual,None)
    benign={'source_epochs','source_signals','matched_epochs','bag_signals'}
    return {'passed':not any(v for k,v in counts.items() if k not in benign),
            'require_nan_missing':require_nan_missing,
            'counts':dict(counts),'missing_by_signal':dict(missing),'max_abs_error':dict(max_error),'examples':examples}

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--obs',type=Path,required=True);ap.add_argument('--bag',type=Path,required=True)
    ap.add_argument('--topic',required=True);ap.add_argument('--report',type=Path,required=True)
    ap.add_argument('--timestamp-scale',choices=['gpst','unchecked'],default='gpst');args=ap.parse_args()
    with args.obs.open() as stream:
        header=read_header(stream)
        result=compare(epochs(stream,header),iter(bag_epochs(args.bag,args.topic)),header,args.timestamp_scale)
    with args.obs.open('rb') as raw:source_hash=hashlib.file_digest(raw,'sha256').hexdigest()
    result.update(source=str(args.obs),source_sha256=source_hash,
                  bag=str(args.bag),topic=args.topic,header=header,timestamp_scale=args.timestamp_scale)
    args.report.parent.mkdir(parents=True,exist_ok=True)
    args.report.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'passed':result['passed'],'counts':result['counts'],'report':str(args.report)}))
    return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
