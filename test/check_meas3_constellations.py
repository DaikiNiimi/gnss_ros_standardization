#!/usr/bin/env python3
"""Optional Meas3 manufacturer comparison for all seven constellations.

Byte layouts come from the pinned RxTools reference. This is deliberately not
classified as an independent published bit specification. No vendor code is
redistributed. Tests master short/long, both delta forms, extras, all master LTI
encodings, GLONASS channels and LLI transitions on both ROS nodes.
"""
import argparse
import csv
import io
import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess
import tempfile
import test_receiver_streams as wire
from test_receiver_observations import marker,PARAMS
from test_decoder_boundaries import extra

# Adapter channel->PRN is RxTools' internal numbering (sviddef.h), not SBF SVID.
# S1 signal-tracking table identifies Galileo E1 as E1-C (manufacturer's
# SIG_GALE1BC label is not a promise of a combined B+C RINEX observable).
SYSTEMS=[('G',1,0,'1C'),('R',222,8,'1C'),('E',64,17,'1C'),('C',159,28,'2I'),
         ('S',120,24,'1C'),('J',104,6,'1C'),('I',256,15,'5A')]

def ranges(tow,kind,lti=1,channel=0,reserved=0):
    p=bytearray(struct.pack('<IH2xHBB',tow,2300,127,0x20,reserved))
    for sys in range(7):
        if kind.startswith('delta'):p+=b'\0' # reuse constellation header
        else:
            p+=b'\x01\x01' # one-byte satellite mask, satellite1, master signal0
            if sys==1:p+=bytes([channel+8])
            if sys==3:p+=struct.pack('<H',1) # BDS GEO/IGSO long-range base
        if kind=='short':p+=struct.pack('<II',1|(131072<<1)|(lti<<20)|(21<<23),2000000125)
        elif kind=='long':p+=struct.pack('<IIH',(4<<2)|(2097152<<6)|(lti<<28),21000000125&0xffffffff,35|(1<<6))
        elif kind=='delta_short':p+=struct.pack('<I',2|(1<<2)|(8192<<4)|(8192<<18))
        elif kind=='delta_long':p+=bytes([14|(8<<4)])+struct.pack('<I',(4<<13)|(32768<<16))
        else:raise ValueError(kind)
    return wire.sbf(4109,p+bytes((-len(p))%4))

def slave_ranges(tow,kind,lti=1,channel=0):
    # Manufacturer-reference layouts; no claim of independent specification.
    p=bytearray(struct.pack('<IH2xHBB',tow,2300,127,0x20,0))
    for sys in range(7):
        if kind.startswith('delta'):p+=b'\0'
        else:
            p+=b'\x01\x01'
            if sys==1:p+=bytes([channel+8])
            if sys==3:p+=struct.pack('<H',1)
        have_slave=sys!=6  # NavIC L1/tentative S are outside this profile.
        if kind=='short':
            p+=struct.pack('<II',1|(131072<<1)|(lti<<20)|(21<<23)|(int(have_slave)<<28),2000000125)
        elif kind=='long':
            p+=struct.pack('<IIH',(4<<2)|(2097152<<6)|(lti<<28),21000000125&0xffffffff,35|((3 if have_slave else 1)<<6))
        elif kind=='delta_short':p+=struct.pack('<I',2|(1<<2)|(8192<<4)|(8192<<18))
        elif kind=='delta_long':p+=bytes([14|(8<<4)])+struct.pack('<I',(4<<13)|(32768<<16))
        if have_slave:
            if kind=='short':p+=struct.pack('<IB',1|(32768<<1)|(10000<<17),lti|(21<<3))
            elif kind=='long':p+=struct.pack('<IHB',(2097152<<2)|(lti<<24)|(4<<28),1234,35)
            else:p+=struct.pack('<HB',2|(2048<<2)|(2<<14),128)
    return wire.sbf(4109,p+bytes((-len(p))%4))

def fixtures():
    for kind,ltis in [('short',[1,2,3,4,5,6,7,0,1]),('long',list(range(16)))]:
        frames=[]
        for i,lti in enumerate(ltis):
            tow=100000+2000*i;channel=i%14-7
            frames.append(ranges(tow,kind,lti,channel,i%32)+extra(4110,b'\xcc'*4,tow)+
                          extra(4111,b'\x0f\x7d\0\0'*7,tow)+marker(tow))
            for dt,delta in [(100,'delta_short'),(200,'delta_long')]:
                frames.append(ranges(tow+dt,delta)+marker(tow+dt))
        yield kind,frames
    for kind,ltis in [('short',range(8)),('long',range(16))]:
        frames=[]
        for i,lti in enumerate(ltis):
            tow=100000+2000*i
            for dt,layout in [(0,kind),(100,'delta_short'),(200,'delta_long')]:
                frames.append(slave_ranges(tow+dt,layout,lti,i%14-7)+marker(tow+dt))
        yield 'slave_'+kind,frames

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--reference',type=Path,required=True);ap.add_argument('--report',type=Path,required=True)
    ap.add_argument('--fixture-output',type=Path,help='Generate reference expectations without invoking the ROS receiver')
    args,_=ap.parse_known_args();reports=[];golden={}
    identities={(svid,sig):(system,120 if system=='S' else 193 if system=='J' else 1,code)
                for system,svid,sig,code in SYSTEMS}
    for system,svid,sig,code in [('G',1,3,'2L'),('R',222,11,'2C'),('E',64,20,'5Q'),
                                  ('C',159,29,'7I'),('S',120,25,'5I'),('J',104,7,'2L')]:
        identities[svid,sig]=(system,120 if system=='S' else 193 if system=='J' else 1,code)
    with tempfile.TemporaryDirectory(prefix='meas3_all_systems_') as tmp:
        for label,frames in fixtures():
            path=Path(tmp)/'input.sbf';path.write_bytes(b''.join(frames))
            reference=subprocess.check_output([str(args.reference.resolve()),str(path),'--lock-time'],text=True)
            expected={};previous={}
            for row in csv.reader(io.StringIO(reference)):
                assert len(row)==13,row
                week,tow,svid,ant,sig,fcn=map(int,row[:6]);assert ant==0
                system,prn,code=identities[svid,sig]
                key=(week,tow,system,prn,code);assert key not in expected,key
                flags,locks,timer=map(int,row[10:]);identity=(system,prn,code)
                lli=(1 if identity in previous and timer<previous[identity] else 0)|(2 if flags&4 else 0)
                previous[identity]=timer
                expected[key]=(list(map(float,row[6:10])),lli,fcn-1 if system=='R' else 255)
            assert len(expected)==len(frames)*(13 if label.startswith('slave_') else 7),(label,len(expected),len(frames))
            if args.fixture_output:
                golden[label]=[dict(key=key,values=values,lli=lli,freq=freq)
                               for key,(values,lli,freq) in expected.items()]
                continue
            for mode in ('decoder','driver'):
                result=wire.run('meas3_systems_'+label+'_'+mode,'sbf_'+mode+'_node',frames,PARAMS,chunk_delay=.04)
                validate(result,expected,label+'_'+mode)
                reports.append(dict(layout=label,node=mode,epochs=len(result['obs']),signals=len(expected),passed=True))
    if args.fixture_output:
        refroot=Path('/opt/Septentrio/RxTools/sbf2asc')
        hashes={name:hashlib.sha256((refroot/name).read_bytes()).hexdigest()
                for name in ('sbfread_meas.c','sbfdef.h','measepoch.h','sbfsigtypes.h','sviddef.h')}
        args.fixture_output.write_text(json.dumps(dict(reference='RxTools26.0.0/sbf2asc3.1.12',
            scope='Manufacturer decoder comparison; independent code origin not established',source_sha256=hashes,cases=golden),indent=2)+'\n')
        print('Reference-only fixtures generated: '+str(sum(map(len,golden.values()))));return
    args.report.write_text(json.dumps(dict(scope='manufacturer comparison, not independent bit specification',cases=reports),indent=2)+'\n')
    print(json.dumps(reports))

def validate(result,expected,label):
    actual={(e.week,round(e.tow*1000),o.system,o.prn,o.code_str):o for e in result['obs'] for o in e.observations}
    assert len(actual)==sum(len(e.observations) for e in result['obs'])
    assert actual.keys()==expected.keys(),(label,set(expected)-set(actual),set(actual)-set(expected))
    for key,(values,lli,freq) in expected.items():
        obs=actual[key]
        for name,value,tol in zip(('p','l','d','snr'),values,(1e-6,1e-6,1e-5,1e-6)):
            got=getattr(obs,name)
            assert math.isnan(got) if value==-2e10 else math.isclose(got,value,rel_tol=1e-7 if name=='d' else 0,abs_tol=tol),(label,key,name,got,value)
        assert obs.freq==freq,(key,'frequency',obs.freq,freq)
        assert obs.lli==lli,(key,'LLI',obs.lli,lli)

if __name__=='__main__':main()
