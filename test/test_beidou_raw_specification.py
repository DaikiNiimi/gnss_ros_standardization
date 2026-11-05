#!/usr/bin/env python3
"""G7 Figures5-8..10,5-14-1..10; Tables5-7/5-10; S1 BDSRaw p285.

One-based transmitted bit runs include parity gaps. Channel BCH decoding has
already happened in the receiver. Backend IODE=toc/720 modulo240 is intentional.
"""
import math
import struct
import unittest
import test_receiver_streams as wire
PARAMS=['-p','topics.ephemeris:=/review/nav','-p','use_gps_timestamp:=true']
RAW=dict(f0=-12345,f1=19,f2=-3,tgd1=-7,tgd2=9,deln=17,cuc=-11,m0=-2345678,e=85899346,
 cus=13,crc=31,crs=-23,sqrt_a=2701918208,i0=654321098,cic=-15,omgd=-7654,cis=21,idot=-123,omg0=3456789,omg=-4567890)
SCALE=dict(f0=2**-33,f1=2**-50,f2=2**-66,tgd1=1e-10,tgd2=1e-10,
 deln=2**-43*math.pi,cuc=2**-31,m0=2**-31*math.pi,e=2**-33,cus=2**-31,crc=2**-6,crs=2**-6,
 sqrt_a=2**-19,i0=2**-31*math.pi,cic=2**-31,omgd=2**-43*math.pi,cis=2**-31,idot=2**-43*math.pi,
 omg0=2**-31*math.pi,omg=2**-31*math.pi)
D1=dict(f0=[(1,226,7),(1,241,17)],f1=[(1,258,5),(1,271,17)],f2=[(1,215,11)],
 tgd1=[(1,99,10)],tgd2=[(1,109,4),(1,121,6)],deln=[(2,43,10),(2,61,6)],
 cuc=[(2,67,16),(2,91,2)],m0=[(2,93,20),(2,121,12)],e=[(2,133,10),(2,151,22)],
 cus=[(2,181,18)],crc=[(2,199,4),(2,211,14)],crs=[(2,225,8),(2,241,10)],
 sqrt_a=[(2,251,12),(2,271,20)],i0=[(3,66,17),(3,91,15)],cic=[(3,106,7),(3,121,11)],
 omgd=[(3,132,11),(3,151,13)],cis=[(3,164,9),(3,181,9)],idot=[(3,190,13),(3,211,1)],
 omg0=[(3,212,21),(3,241,11)],omg=[(3,252,11),(3,271,21)])
D2=dict(f0=[(3,101,12),(3,121,12)],f1=[(3,133,4),(4,47,6),(4,61,12)],f2=[(4,73,10),(4,91,1)],
 tgd1=[(1,103,10)],tgd2=[(1,121,10)],deln=[(4,97,16)],cuc=[(4,121,14),(5,47,4)],
 m0=[(5,51,2),(5,61,22),(5,91,8)],cus=[(5,99,14),(5,121,4)],e=[(5,125,10),(6,47,6),(6,61,16)],
 sqrt_a=[(6,77,6),(6,91,22),(6,121,4)],cic=[(6,125,10),(7,47,6),(7,61,2)],cis=[(7,63,18)],
 i0=[(7,106,7),(7,121,14),(8,47,6),(8,61,5)],crc=[(8,66,17),(8,91,1)],crs=[(8,92,18)],
 omgd=[(8,110,3),(8,121,16),(9,47,5)],omg0=[(9,52,1),(9,61,22),(9,91,9)],
 omg=[(9,100,13),(9,121,14),(10,47,5)],idot=[(10,52,1),(10,61,13)])

def pages(model,toe=96000,toc=None,sow=96000,clock=-12345,bad_sequence=False):
    data={p:['0']*300 for p in (range(1,4) if model==1 else range(1,11))}
    def put(runs,value):
        width=sum(n for _,_,n in runs);bits=format(value&((1<<width)-1),f'0{width}b');cursor=0
        for page,start,n in runs:data[page][start-1:start-1+n]=bits[cursor:cursor+n];cursor+=n
    for p in data:
        put([(p,1,11)],0x712);put([(p,16,3)],p if model==1 else 1)
        time=sow+(p-1)*(6 if model==1 else 3)
        if bad_sequence and p==3:time+=6
        put([(p,19,8),(p,31,12)],time)
        if model==2:put([(p,43,4)],p)
    for name,runs in (D1 if model==1 else D2).items():put(runs,clock if name=='f0' else RAW[name])
    toc=toe if toc is None else toc
    if model==1:
        fields=[([(1,43,1)],1),([(1,44,5)],21),([(1,49,4)],3),([(1,61,13)],944),
                ([(1,74,9),(1,91,8)],toc//8),([(1,288,5)],17),([(2,291,2),(3,43,10),(3,61,5)],toe//8)]
    else:
        fields=[([(1,47,1)],1),([(1,48,5)],21),([(1,61,4)],3),([(1,65,13)],944),
                ([(1,78,5),(1,91,12)],toc//8),([(4,92,5)],17),([(7,81,2),(7,91,15)],toe//8)]
    for runs,value in fields:put(runs,value)
    return {p:int(''.join(bits)+'0'*20,2).to_bytes(40,'big') for p,bits in data.items()}

def envelope(data,model):
    return [wire.sbf(4047,struct.pack('<IH6B10I',96014000,2300,146 if model==1 else 141,1,0,28,0,0,*struct.unpack('>10I',data[p]))) for p in sorted(data)]

class BeidouRawSpecification(unittest.TestCase):
    def check(self,e,model,toe=96000,sow=96000,week=2300,clock=-12345):
        gpst_toe=(toe+14)%604800
        self.assertEqual((e.system,e.prn,e.week,e.toe,e.toc,e.toes,e.ttr),
                         ('C',6 if model==1 else 1,week,gpst_toe,gpst_toe,float(toe),float(sow+14)))
        self.assertEqual((e.iode,e.iodc,e.sva,e.svh,e.flag),(int(toe//720)%240,21,3,1,model))
        for name,value in RAW.items():
            expected=(clock if name=='f0' else value)*SCALE[name]
            actual=e.a if name=='sqrt_a' else e.tgd[int(name[-1])-1] if name.startswith('tgd') else getattr(e,name)
            if name=='sqrt_a':expected*=expected
            self.assertTrue(math.isclose(actual,expected,rel_tol=2e-14,abs_tol=1e-20),(name,actual,expected))

    def test_d1_d2_signed_fields_same_issue_update(self):
        for model in (1,2):
            for mode in ('decoder','driver'):
                result=wire.run(f'bdsraw_{model}_{mode}','sbf_'+mode+'_node',
                    envelope(pages(model),model)*2+envelope(pages(model,clock=-12344),model),PARAMS,chunk_delay=.04)
                snaps=[m for m in result['nav'] if m.gnss_ephemeris];self.assertEqual(len(snaps),2)
                for snap,clock in zip(snaps,(-12345,-12344)):self.check(snap.gnss_ephemeris[0],model,clock=clock)

    def test_d1_d2_nearest_week_both_directions(self):
        for model in (1,2):
            for mode in ('decoder','driver'):
                for toe,sow,week in [(0,604740,2301),(604760,60,2299)]:
                    with self.subTest(model=model,mode=mode,toe=toe):
                        result=wire.run(f'bdsraw_week_{model}_{mode}_{toe}','sbf_'+mode+'_node',envelope(pages(model,toe=toe,sow=sow),model),PARAMS,chunk_delay=.04)
                        snaps=[m for m in result['nav'] if m.gnss_ephemeris];self.assertEqual(len(snaps),1)
                        self.check(snaps[0].gnss_ephemeris[0],model,toe=toe,sow=sow,week=week)

    def test_d1_d2_missing_inconsistent_time_and_recovery(self):
        for model in (1,2):
            for mode in ('decoder','driver'):
                for fault in ('missing','toc','sequence'):
                    data=pages(model,toc=96008 if fault=='toc' else None,bad_sequence=fault=='sequence')
                    if fault=='missing':del data[1]
                    result=wire.run(f'bdsraw_bad_{model}_{mode}_{fault}','sbf_'+mode+'_node',envelope(data,model)+envelope(pages(model),model),PARAMS,chunk_delay=.04)
                    snaps=[m for m in result['nav'] if m.gnss_ephemeris];self.assertEqual(len(snaps),1);self.check(snaps[0].gnss_ephemeris[0],model)

if __name__=='__main__':unittest.main(argv=[__file__]+wire.unittest_args)
