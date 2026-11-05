#!/usr/bin/env python3
"""R2-derived multi-cell layouts and ROS assembly policy tests.

Pinned rtcm-rs d4d0269dc76b7e88b6620ca023b82889b6441662:
msg/msm46_sat.rs, msm57_sat.rs, msg107[4-7].rs and the msm_data_seg_frag!
macro (satellite-major cell mask; field-major satellite/signal arrays).
These authored fixtures never call an RTKLIB encoder. Assembly expectations
are the repository's documented policy, not normative RTCM certification.
"""
import math
import re
import time
import unittest
from pathlib import Path
import test_receiver_streams as wire
wire.MIN_DRAIN_SECONDS=5.1
from test_decoder_boundaries import RTCM_PARAMS, rtcm_payload
from test_rtcm_reference import SIGNALS, radio_frequency


def grid(variant, satellites=(1, 3, 8), signals=(2, 8), active=None,
         tow_ms=100000, base=1070, station=42, iod=1, sync=0):
    """Return independently packed frame and chosen integer measurement rows."""
    satellites=sorted(satellites);signals=sorted(signals)
    active=set((sat,sig) for sat in satellites for sig in signals) if active is None else set(active)
    assert active<=set((sat,sig) for sat in satellites for sig in signals)
    cells=[(sat,sig) for sat in satellites for sig in signals if (sat,sig) in active]
    fields=[]
    def put(value,width):
        assert -(1<<(width-1))<=value<(1<<width),(value,width)
        fields.append(format(value&((1<<width)-1),f'0{width}b'))
    put(base+variant,12);put(station,12)
    put(tow_ms+10782000 if base==1080 else tow_ms-14000 if base==1120 else tow_ms,30)
    put(sync,1);put(iod,3);put(0,7);put(0,2);put(0,2);put(0,1);put(0,3)
    put(sum(1<<(64-sat) for sat in satellites),64)
    put(sum(1<<(32-sig) for sig in signals),32)
    for sat in satellites:
        for sig in signals:put(int((sat,sig) in active),1)
    satrows={sat:dict(rough=60+i,modulo=100+i*7,rate=-100+i,channel=-7+i%14)
             for i,sat in enumerate(satellites)}
    ext=variant in (6,7);rate=variant in (5,7)
    rows={cell:dict(pr=(-1 if j%2 else 1)*(j+1)*16,cp=(-1 if j%3 else 1)*(j+1)*128,
                    lock=10+j%3,half=j%2,cnr=(40+j%20)*16+1 if ext else 40+j%20,
                    fine_rate=(-1 if j%2 else 1)*(j+1)*13)
          for j,cell in enumerate(cells)}
    for sat in satellites:put(satrows[sat]['rough'],8)
    if rate:
        for sat in satellites:put(satrows[sat]['channel']+7 if base==1080 else 0,4)
    for sat in satellites:put(satrows[sat]['modulo'],10)
    if rate:
        for sat in satellites:put(satrows[sat]['rate'],14)
    for name,width in [('pr',20 if ext else 15),('cp',24 if ext else 22),
                       ('lock',10 if ext else 4),('half',1),('cnr',10 if ext else 6)]+([('fine_rate',15)] if rate else []):
        for cell in cells:put(rows[cell][name],width)
    bits=''.join(fields);bits+='0'*((-len(bits))%8)
    return rtcm_payload(int(bits,2).to_bytes(len(bits)//8,'big')),satrows,rows


def legacy(message, tow_ms, sync):
    """1004/1012 with one satellite, like test_rtcm_reference.legacy, plus DF005 sync."""
    glo = message in (1010, 1012); dual = message in (1004, 1012); fields = []
    def put(v, n): fields.append(format(v & ((1 << n) - 1), f'0{n}b'))
    put(message, 12); put(42, 12); put(tow_ms + 10782000 if glo else tow_ms, 27 if glo else 30)
    put(sync, 1); put(1, 5); put(0, 1); put(0, 3)
    put(1, 6); put(0, 1)
    if glo: put(7, 5)
    put(5000000, 25 if glo else 24); put(-2000, 20); put(10, 7); put(35 if glo else 70, 7 if glo else 8)
    put(181, 8)
    if dual:
        put(0, 2); put(-100, 14); put(4000, 20); put(10, 7); put(182, 8)
    bits = ''.join(fields); bits += '0' * ((-len(bits)) % 8)
    return rtcm_payload(int(bits, 2).to_bytes(len(bits) // 8, 'big'))


class MsmStates(unittest.TestCase):
    def test_release_D7_file_eof_discards_only_unfinished_chain(self):
        frames = []
        for tow in [100000, 101000, 102000]:
            frames += [grid(7, (1,), (2,), base=1070, tow_ms=tow, sync=1)[0],
                       grid(7, (1,), (2,), base=1080, tow_ms=tow, sync=int(tow == 102000))[0]]
        result = wire.run('D7_file_eof', 'rtcm_decoder_node', b''.join(frames),
                          RTCM_PARAMS, file_input=True)
        self.assertIn('Input stream opened: file://', result['log'])
        # A small file can finish before DDS discovery: counters are the oracle.
        published = [int(x) for x in re.findall(r'published_epochs=(\d+)', result['log'])]
        discarded = [int(x) for x in re.findall(r'msm_incomplete_discarded=(\d+)', result['log'])]
        self.assertTrue(published, result['log'])
        self.assertTrue(discarded, result['log'])
        self.assertEqual(max(published), 2, result['log'])
        self.assertEqual(max(discarded), 2, result['log'])


    def test_release_R4_legacy_split_is_diagnosed(self):
        label='R4_legacy_split'
        result=wire.run(label,'rtcm_decoder_node',legacy(1004,100000,0)+legacy(1012,100000,0),RTCM_PARAMS)
        self.assertEqual(len(result['obs']),1)
        self.assertEqual({o.system for o in result['obs'][0].observations},{'G'})
        self.assertRegex(result['log'],r'msm_split_detected=1\b')
        self.assertIn('DF393',result['log'])

    def test_release_R5_rejected_final_and_overflow_recover_at_next_epoch(self):
        for overflow in [False,True]:
            g=grid(7,(1,),(2,),tow_ms=100000,sync=1)[0]
            final=grid(7,(1,),(2,),base=1090,tow_ms=100000,sync=0)[0]
            bad=rtcm_payload(final[3:-7])
            prefix=(g*257+final) if overflow else (g+bad)
            following=grid(7,(1,),(2,),tow_ms=101000,sync=1)[0]+grid(7,(1,),(2,),base=1090,tow_ms=101000,sync=0)[0]
            result=wire.run('R5_recovery_'+str(overflow),'rtcm_decoder_node',[prefix,following],RTCM_PARAMS)
            self.assertEqual([(m.tow,{o.system for o in m.observations}) for m in result['obs']],[(101.,{'G','E'})])

    def test_release_R4_mixed_legacy_msm(self):
        for order in ['legacy_first','msm_first']:
            frames=[]
            for t in range(100,104):
                g=grid(7,(1,),(2,),base=1070,tow_ms=t*1000,sync=1)[0]
                e=grid(7,(1,),(2,),base=1090,tow_ms=t*1000,sync=0 if order=='legacy_first' else 1)[0]
                frames.append(legacy(1004,t*1000,1)+legacy(1012,t*1000,1)+e if order=='legacy_first'
                              else g+e+legacy(1012,t*1000,0))
            result=wire.run('R4_'+order,'rtcm_decoder_node',frames,RTCM_PARAMS,chunk_delay=.25)
            self.assertEqual([m.tow for m in result['obs']],[100.,101.,102.,103.])
            self.assertTrue(all({o.system for o in m.observations}=={'G','R','E'} for m in result['obs']))

    def test_release_R4_single_frequency_legacy_can_close_mixed_chain(self):
        # 1002/1010 do not set RTKLIB obsflag themselves; both can still be
        # the final DF005=0 frame after a sync=1 MSM/legacy fragment.
        for final in [1002,1010]:
            frames=[]
            for t in range(100,103):
                e=grid(7,(1,),(2,),base=1090,tow_ms=t*1000,sync=1)[0]
                other=1010 if final==1002 else 1002
                frames.append(e+legacy(other,t*1000,1)+legacy(final,t*1000,0))
            result=wire.run('R4_single_final_'+str(final),'rtcm_decoder_node',frames,RTCM_PARAMS)
            self.assertEqual([m.tow for m in result['obs']],[100.,101.,102.])
            self.assertTrue(all({o.system for o in m.observations}=={'G','R','E'} for m in result['obs']))

    def test_release_R5_rejected_or_unsupported_middle_preserves_chain(self):
        for kind in ['truncated','unsupported','overflow']:
            frames=[]
            for t in range(100,104):
                g=grid(7,(1,),(2,),base=1070,tow_ms=t*1000,sync=1)[0]
                e=grid(7,(1,),(2,),base=1090,tow_ms=t*1000,sync=0)[0]
                r=grid(7,(1,),(2,),base=1080,tow_ms=t*1000,sync=1)[0]
                if kind=='truncated':bad=rtcm_payload(r[3:-7])
                elif kind=='unsupported':
                    body=bytearray(legacy(1002,t*1000,1)[3:-3]);body[0]=1001>>4;body[1]=((1001&15)<<4)|(body[1]&15)
                    bad=rtcm_payload(bytes(body))
                else:bad=g*256
                frames.append(g+bad+e)
            result=wire.run('R5_'+kind,'rtcm_decoder_node',frames,RTCM_PARAMS,chunk_delay=.25)
            self.assertEqual(len(result['obs']),0 if kind=='overflow' else 4)
            for epoch in result['obs']:self.assertEqual({o.system for o in epoch.observations},{'G','E'})

    def test_real_gmsd_chain_matches_individually_completed_frames(self):
        source=Path(__file__).resolve().parents[1]/'third_party/RTKLIB/test/data/rcvraw/GMSD7_20121014.rtcm3'
        data=source.read_bytes()[:262144]
        frames=[];at=0;last_complete=0
        while at+6<=len(data):
            if data[at]!=0xd3:
                at+=1;continue
            length=((data[at+1]&3)<<8)+data[at+2]+6
            if at+length>len(data):break
            frame=data[at:at+length];at+=length
            self.assertEqual(frame,rtcm_payload(frame[3:-3]),'fixture CRC')
            frames.append(frame)
            mid=(frame[3]<<4)|(frame[4]>>4)
            if 1074<=mid<=1137 and mid%10 in (4,5,6,7) and not (frame[9]&2):
                last_complete=len(frames)
        frames=frames[:last_complete]
        separate=[]
        for frame in frames:
            mid=(frame[3]<<4)|(frame[4]>>4)
            if 1074<=mid<=1137 and mid%10 in (4,5,6,7):
                payload=bytearray(frame[3:-3]);payload[6]&=~2
                frame=rtcm_payload(payload)
            if mid//10 != 112: separate.append(frame)
        params=['-p','reference_week:=1710','-p','reference_tow:=0.0',
                '-p','topics.ephemeris:=/review/nav',]
        # A8 does not merge independent completions. Decode each constellation
        # in its own stream to retain the independent single-frame value oracle.
        streams=[('chain',frames)]
        for lane,system in [(107,'G'),(108,'R'),(111,'J')]:
            streams.append((system,[f for f in separate if not 107<=((f[3]<<4)|(f[4]>>4))//10<=113
                                    or ((f[3]<<4)|(f[4]>>4))//10==lane]))
        outputs=[]
        for label,stream in streams:
            payload=b''.join(stream)
            chunks=[payload[i:i+1024] for i in range(0,len(payload),1024)]
            result=wire.run('gmsd_'+label,'rtcm_decoder_node',chunks,params,chunk_delay=.01)
            self.assertEqual(max(len(m.gnss_ephemeris) for m in result['nav']),15)
            self.assertEqual(max(len(m.glonass_ephemeris) for m in result['nav']),16)
            self.assertEqual(len(result['obs']),257)
            labels=[(m.week,m.tow) for m in result['obs']]
            self.assertTrue(all(a<b for a,b in zip(labels,labels[1:])))
            expected_systems={'G','R','J'} if label=='chain' else {label}
            self.assertTrue(all({o.system for o in m.observations}==expected_systems for m in result['obs']))
            if label=='chain':
                log=Path(wire.TEMP.name,'gmsd_chain.log').read_text()
                self.assertEqual(max(map(int,re.findall(r'msm_chain_time_mismatch=(\d+)',log))),257)
            import os
            if os.environ.get('GNSS_TEST_ARTIFACT_DIR'):
                Path(os.environ['GNSS_TEST_ARTIFACT_DIR'], 'gmsd_'+label+'_epochs.tsv').write_text(
                    ''.join(f'{m.week} {m.tow:.3f} {"".join(sorted({o.system for o in m.observations}))}\n' for m in result['obs']))
            rows={}
            for m in result['obs']:
                for o in m.observations:
                    values=(o.p,o.l,o.d,o.snr,o.lli,o.freq)
                    rows[(m.week,m.tow,o.satid,o.code_str)]=tuple(
                        None if isinstance(v,float) and math.isnan(v) else v for v in values)
            outputs.append(rows)
        independent={key:value for rows in outputs[1:] for key,value in rows.items()}
        self.assertEqual(set(outputs[0]),set(independent))
        for key in outputs[0]:
            self.assertEqual(outputs[0][key],independent[key],str(key))
        self.assertEqual({k[2][0] for k in outputs[0]},{'G','R','J'})
        self.assertGreater(len(outputs[0]),0)

    def test_mixed_epoch_chain_rejects_inconsistent_constellation_time(self):
        # Do not relabel the source's inconsistent BDS observations, or let
        # their future timestamps hide valid G/R/J epochs in consumers.
        fixtures = [grid(7, (1,), (2,), base=base, tow_ms=ms, sync=sync)
                    for base, ms, sync in [(1070,100000,1), (1080,100000,1),
                                           (1110,100000,1), (1120,114000,0),
                                           (1070,101000,1), (1120,115000,0)]]
        result = wire.run('msm_mixed_times', 'rtcm_decoder_node',
                          [f[0] for f in fixtures], RTCM_PARAMS, chunk_delay=.03)
        self.assertEqual([m.tow for m in result['obs']], [100.])
        self.assertEqual([{o.system for o in m.observations} for m in result['obs']],
                         [{'G','R','J'}])
        for msg, indices in zip(result['obs'], [(0,1,2),(4,)]):
            for i in indices:
                family, base, system = [('gps',1070,'G'),('glo',1080,'R'),
                                        ('qzss',1110,'J'),('bds',1120,'C'),
                                        ('gps',1070,'G'),('bds',1120,'C')][i]
                part=type(msg)();part.week=msg.week;part.tow=msg.tow
                part.observations=[o for o in msg.observations if o.system==system]
                self.check_epoch(part,7,family,base,msg.tow,fixtures[i])

    def test_separate_completions_and_late_epoch_are_dropped(self):
        gps=grid(7,(1,),(2,),tow_ms=100000)[0]
        gal=grid(7,(1,),(2,),tow_ms=100000,base=1090)[0]
        late=grid(7,(1,),(2,),tow_ms=99000,base=1110)[0]
        def idle(node,executor):
            end=time.monotonic()+.4
            while time.monotonic()<end: executor.spin_once(timeout_sec=.02)
        result=wire.run('msm_merge_late','rtcm_decoder_node',[gps+gal,idle,late],
                        RTCM_PARAMS+[],chunk_delay=.01)
        self.assertEqual([m.tow for m in result['obs']],[100.])
        self.assertEqual({o.system for o in result['obs'][0].observations},{'G'})
        log=Path(wire.TEMP.name,'msm_merge_late.log').read_text()
        self.assertEqual(max(map(int,re.findall(r'observation_not_monotonic=(\d+)',log))),2)
        self.assertEqual(max(map(int,re.findall(r'msm_split_detected=(\d+)',log))),1)

    def test_chain_mismatch_and_merge_conflict_counters(self):
        gps=grid(7,(1,),(2,),tow_ms=100000,sync=1)[0]
        bds=grid(7,(1,),(2,),tow_ms=114000,base=1120)[0]
        from test_rtcm_navigation_reference import navigation
        result=wire.run('msm_time_mismatch','rtcm_decoder_node',[gps+bds+navigation(1019)[0]],
                        RTCM_PARAMS+[
                                     '-p','topics.ephemeris:=/review/nav'],chunk_delay=.01)
        # A tied chain cannot advance the navigation date anchor.
        self.assertEqual(result['nav'][-1].gnss_ephemeris[0].ttr,100.)
        self.assertEqual(result['obs'],[])
        log=Path(wire.TEMP.name,'msm_time_mismatch.log').read_text()
        self.assertEqual(max(map(int,re.findall(r'msm_time_tie_withheld=(\d+)',log))),1)
        first=grid(7,(1,),(2,),tow_ms=100000,sync=1)[0]
        # Same satellite now uses rough range from the second position.
        second=grid(7,(1,2),(2,),tow_ms=100000,sync=1)[0]
        payload=bytearray(first[3:-3]);payload[-1]^=0x10;payload[6]&=~2
        conflict=rtcm_payload(payload)
        result=wire.run('msm_merge_conflict','rtcm_decoder_node',[first+second+conflict],
                        RTCM_PARAMS+[],chunk_delay=.01)
        self.assertEqual(len(result['obs']),1)
        self.assertEqual(len(result['obs'][0].observations),2)
        log=Path(wire.TEMP.name,'msm_merge_conflict.log').read_text()
        self.assertGreater(max(map(int,re.findall(r'observation_merge_conflict=(\d+)',log))),0)

    def test_majority_is_independent_of_fragment_order_and_preserves_new_epoch(self):
        def f(base,t,sync):return grid(7,(1,),(2,),base=base,tow_ms=t*1000,sync=sync)[0]
        for first in (True,False):
            chunks=[]
            for t in range(100,103):
                chunks.append(f(1120,t+14,1)+f(1070,t,1)+f(1080,t,0) if first
                              else f(1070,t,1)+f(1080,t,1)+f(1120,t+14,0))
            label='majority_'+str(first)
            result=wire.run(label,'rtcm_decoder_node',chunks,RTCM_PARAMS+[],chunk_delay=.3)
            self.assertEqual([(m.tow,{o.system for o in m.observations}) for m in result['obs']],
                             [(float(t),{'G','R'}) for t in range(100,103)])
            log=Path(wire.TEMP.name,label+'.log').read_text()
            self.assertEqual(max(map(int,re.findall(r'msm_chain_time_mismatch=(\d+)',log))),3)
        result=wire.run('unfinished_new','rtcm_decoder_node',
            [f(1080,100,1), f(1070,101,1)+f(1080,101,1)+f(1090,101,0)],RTCM_PARAMS,chunk_delay=.03)
        self.assertEqual([(m.tow,{o.system for o in m.observations}) for m in result['obs']],[(101.,{'G','R','E'})])

    def test_tie_warning_exclusion_final_sync_and_recovery(self):
        def f(base,t,sync):return grid(7,(1,),(2,),base=base,tow_ms=t*1000,sync=sync)[0]
        params=RTCM_PARAMS+[]
        chunks=[f(1070,t,1)+f(1120,t+14,0) for t in range(100,105)]
        result=wire.run('time_tie','rtcm_decoder_node',chunks,params,chunk_delay=.25)
        self.assertEqual(result['obs'],[])
        log=Path(wire.TEMP.name,'time_tie.log').read_text()
        self.assertEqual(max(map(int,re.findall(r'msm_time_tie_withheld=(\d+)',log))),5)
        self.assertIn('+14.000',log);self.assertIn('observation.excluded_systems',log)
        result=wire.run('time_excluded','rtcm_decoder_node',chunks,
            params+['-p','observation.excluded_systems:=["C"]'],chunk_delay=.25)
        self.assertEqual([m.tow for m in result['obs']],list(map(float,range(100,105))))
        self.assertTrue(all({o.system for o in m.observations}=={'G'} for m in result['obs']))
        log=Path(wire.TEMP.name,'time_excluded.log').read_text()
        self.assertEqual(max(map(int,re.findall(r'msm_system_excluded=(\d+)',log))),5)
        self.assertEqual(max(map(int,re.findall(r'msm_chain_time_mismatch=(\d+)',log))),0)
        result=wire.run('tie_recovery','rtcm_decoder_node',chunks[:1]+[f(1070,105,1)+f(1120,105,0)],params,chunk_delay=.03)
        self.assertEqual([m.tow for m in result['obs']],[105.])

    def test_standard_completion_latency_including_first_and_constellation_changes(self):
        def f(base,t,sync):return grid(7,(1,),(2,),base=base,tow_ms=t*1000,sync=sync)[0]
        sent={};chunks=[]
        for t in range(100,108):
            chunks.append(lambda n,e,t=t:sent.__setitem__(t,time.time()))
            chunks.append(f(1070,t,1)+f(1080,t,1 if t in (100,101,102,107) else 0)
                          +(f(1110,t,0) if t in (100,101,102,107) else b''))
        result=wire.run('standard_latency','rtcm_decoder_node',chunks,
            RTCM_PARAMS+['-p','use_gps_timestamp:=false'],chunk_delay=.05)
        self.assertEqual([m.tow for m in result['obs']],list(map(float,range(100,108))))
        for m in result['obs']:
            self.assertEqual({o.system for o in m.observations},
                             {'G','R','J'} if m.tow in (100,101,102,107) else {'G','R'})
        delays=[m.header.stamp.sec+m.header.stamp.nanosec*1e-9-sent[m.tow] for m in result['obs']]
        print('standard completion delays including first',delays)
        self.assertLessEqual(max(delays),.020)

    def test_split_stream_first_completion_only_in_one_and_two_writes(self):
        def f(base,t):return grid(7,(1,),(2,),base=base,tow_ms=t*1000)[0]
        for single_write in [True,False]:
            chunks=[]
            for t in range(100,106):
                chunks.extend([f(1070,t)+f(1080,t)] if single_write else [f(1070,t),f(1080,t)])
            label='split_drop_'+str(single_write)
            result=wire.run(label,'rtcm_decoder_node',chunks,
                RTCM_PARAMS+[],chunk_delay=.03)
            self.assertEqual([m.tow for m in result['obs']],list(map(float,range(100,106))))
            self.assertTrue(all({o.system for o in m.observations}=={'G'} for m in result['obs']))
            log=Path(wire.TEMP.name,label+'.log').read_text()
            self.assertEqual(max(map(int,re.findall(r'msm_split_detected=(\d+)',log))),6)
            self.assertEqual(max(map(int,re.findall(r'observation_not_monotonic=(\d+)',log))),6)
            self.assertIn('RTCM 10403 DF393',log)

    def test_release_R4_legacy_completion_is_immediate(self):
        from test_rtcm_reference import legacy
        sent={};chunks=[]
        for i,ident in enumerate([1002,1004,1010,1012]):
            tow=100+i
            chunks.append(lambda n,e,t=tow:sent.__setitem__(t,time.time()))
            chunks.append(legacy(ident,tow*1000))
        result=wire.run('legacy_latency','rtcm_decoder_node',chunks,
            RTCM_PARAMS+['-p','use_gps_timestamp:=false'],chunk_delay=.05)
        self.assertEqual([m.tow for m in result['obs']],[100.,101.,102.,103.])
        delays=[m.header.stamp.sec+m.header.stamp.nanosec*1e-9-sent[m.tow] for m in result['obs']]
        print('legacy completion delays',delays)
        self.assertLessEqual(max(delays),.020)

    def test_completion_with_continuous_navigation(self):
        from test_rtcm_navigation_reference import navigation
        nav=navigation(1019)[0]
        def f(base,t,sync):return grid(7,(1,),(2,),base=base,tow_ms=t*1000,sync=sync)[0]
        sent={};chunks=[]
        for t in range(100,106):
            chunks.append(f(1070,t,1))
            chunks.extend([nav]*4)
            chunks.append(lambda n,e,t=t:sent.__setitem__(t,time.time()))
            chunks.append(f(1080,t,0))
        result=wire.run('continuous_nav_latency','rtcm_decoder_node',chunks,
            RTCM_PARAMS+['-p','use_gps_timestamp:=false'],chunk_delay=.03)
        self.assertEqual([m.tow for m in result['obs']],list(map(float,range(100,106))))
        self.assertTrue(all({o.system for o in m.observations}=={'G','R'} for m in result['obs']))
        delays=[m.header.stamp.sec+m.header.stamp.nanosec*1e-9-sent[m.tow] for m in result['obs']]
        print('continuous NAV completion delays',delays)
        self.assertLessEqual(max(delays),.020)

    def test_late_incomplete_fragment_does_not_poison_next_chain(self):
        def f(base,t,sync):
            return grid(7,(1,),(2,),base=base,tow_ms=t*1000,sync=sync)[0]
        chunks=[f(1070,t,1)+f(1080,t,0) for t in range(100,104)]
        # The late R fragment never completes. Discard it before it can
        # vote against G@105; only actual duplicate completions count as split.
        chunks += [f(1070,104,0),f(1080,104,1),
                   f(1070,105,1)+f(1080,105,0)]
        result=wire.run('late_incomplete','rtcm_decoder_node',chunks,
            RTCM_PARAMS+[],chunk_delay=.03)
        self.assertEqual([m.tow for m in result['obs']],list(map(float,range(100,106))))
        self.assertEqual({o.system for o in result['obs'][-1].observations},{'G','R'})
        log=Path(wire.TEMP.name,'late_incomplete.log').read_text()
        self.assertEqual(max(map(int,re.findall(r'msm_split_detected=(\d+)',log))),0)
        self.assertEqual(max(map(int,re.findall(r'observation_not_monotonic=(\d+)',log))),1)
        self.assertEqual(max(map(int,re.findall(r'msm_time_tie_withheld=(\d+)',log))),0)

    def test_idle_incomplete_chain_is_not_released_and_excluded_navigation_remains(self):
        from test_rtcm_navigation_reference import navigation
        def f(base,t,sync=0):return grid(7,(1,),(2,),base=base,tow_ms=t*1000,sync=sync)[0]
        def idle(node,executor):
            deadline=time.monotonic()+.35
            while time.monotonic()<deadline:executor.spin_once(timeout_sec=.01)
        result=wire.run('idle_incomplete','rtcm_decoder_node',[f(1070,100,1),idle,f(1080,100)],
            RTCM_PARAMS,chunk_delay=.03)
        self.assertEqual([m.tow for m in result['obs']],[100.])
        self.assertEqual({o.system for o in result['obs'][0].observations},{'G','R'})
        result=wire.run('excluded_nav','rtcm_decoder_node',[navigation(1019)[0]+f(1070,100)],
            RTCM_PARAMS+['-p','observation.excluded_systems:=["G"]','-p','topics.ephemeris:=/review/nav'],chunk_delay=.03)
        self.assertEqual(result['obs'],[])
        self.assertTrue(any(m.gnss_ephemeris for m in result['nav']))

    def test_invalid_exclusion_fails_startup(self):
        import subprocess
        for assignment in ['observation.excluded_systems:=["X"]',
                           'observation.excluded_systems:=["GPS"]']:
            result=subprocess.run([wire.B+'rtcm_decoder_node','--ros-args','-p',
                'stream_path:=tcpcli://127.0.0.1:1','-p',assignment],capture_output=True,text=True,timeout=5)
            self.assertNotEqual(result.returncode,0,assignment)
            self.assertIn(assignment.split(':=')[0],result.stderr)

    def check_epoch(self,msg,variant,family,base,tow,fixture):
        _,satrows,rows=fixture
        system={'gps':'G','glo':'R','gal':'E','sbas':'S','qzss':'J','bds':'C','navic':'I'}[family]
        expected={(str(119+sat) if system=='S' else f'{system}{sat:02}',SIGNALS[family][sig]):(sat,row)
                  for (sat,sig),row in rows.items()}
        self.assertEqual((msg.week,msg.tow),(2300,tow))
        actual={(o.satid,o.code_str):o for o in msg.observations}
        self.assertEqual(len(actual),len(msg.observations),'duplicate satellite/signal')
        self.assertEqual(set(actual),set(expected))
        for key,(sat,row) in expected.items():
            o=actual[key];r=satrows[sat];f=radio_frequency(system,key[1],r['channel'])
            rough=r['rough']+r['modulo']*2**-10
            self.assertAlmostEqual(o.p,(rough+row['pr']*2**(-29 if variant in (6,7) else -24))*299792.458,places=7)
            self.assertAlmostEqual(o.l,(rough+row['cp']*2**(-31 if variant in (6,7) else -29))*f*.001,places=6)
            if variant in (5,7):self.assertAlmostEqual(o.d,-(r['rate']+row['fine_rate']*.0001)*f/299792458.,places=3)
            else:self.assertTrue(math.isnan(o.d))
            self.assertEqual(o.snr,row['cnr']*(.0625 if variant in (6,7) else 1))
            self.assertEqual(o.lli&2,row['half']*2)
            self.assertEqual(o.freq,r['channel']+7 if system=='R' else 255)

    def test_sparse_and_dense_cells_all_constellations(self):
        families=[('gps',1070,(2,8)),('glo',1080,(2,8)),('gal',1090,(2,14)),
                  ('sbas',1100,(2,22)),('qzss',1110,(2,22)),('bds',1120,(2,8)),('navic',1130,(8,22))]
        for variant in (5,7,4,6):
            with self.subTest(variant=variant):
                fixtures=[];cases=[]
                for family,base,signals in families:
                    for active in ({(1,signals[1]),(3,signals[0]),(8,signals[1])},None):
                        ms=100000+len(fixtures)*1000
                        fixture=grid(variant,signals=signals,active=active,tow_ms=ms,base=base)
                        # GLONASS MSM4/6 have no channel field: seed known channels
                        # using a complete MSM5 epoch, then advance time.
                        if family=='glo' and variant in (4,6):
                            seed=grid(5,signals=signals,tow_ms=ms,base=base)
                            fixtures.append(seed);cases.append((5,family,base,ms*.001))
                            ms+=1000;fixture=grid(variant,signals=signals,active=active,tow_ms=ms,base=base)
                        fixtures.append(fixture);cases.append((variant,family,base,ms*.001))
                result=wire.run('msm_grid_'+str(variant),'rtcm_decoder_node',[x[0] for x in fixtures],RTCM_PARAMS,chunk_delay=.03)
                self.assertEqual(len(result['obs']),len(cases))
                for msg,case,fixture in zip(result['obs'],cases,fixtures):self.check_epoch(msg,*case,fixture)

    def test_empty_masks_capacity_and_recovery(self):
        for variant in (4,5,6,7):
            with self.subTest(variant=variant):
                empty=[grid(variant,(),(),tow_ms=100000),grid(variant,active=set(),tow_ms=101000),
                       grid(variant,(1,),(),tow_ms=102000),grid(variant,(),(2,),tow_ms=103000)]
                limit=grid(variant,range(1,33),tow_ms=104000)
                # The backend accepts at most 64 satellite x signal mask entries.
                # This is an implementation limit, not a normative RTCM limit.
                overflow=grid(variant,range(1,34),tow_ms=105000)
                recovery=grid(variant,(2,),(2,),tow_ms=106000)
                result=wire.run('msm_capacity_'+str(variant),'rtcm_decoder_node',
                    [x[0] for x in empty+[limit,overflow,recovery]],RTCM_PARAMS,chunk_delay=.03)
                # R2/A2 now explicitly withholds epochs with no accepted
                # observations, including declared satellites without signals.
                # Retain all empty-mask inputs and verify subsequent recovery.
                self.assertEqual([m.tow for m in result['obs']],[104.,106.])
                for msg,fixture,tow in zip(result['obs'],[limit,recovery],[104.,106.]):
                    self.check_epoch(msg,variant,'gps',1070,tow,fixture)

    def test_fragment_station_issue_corruption_and_recovery(self):
        for variant in (4,5,6,7):
            with self.subTest(variant=variant):
                def fragment(ms,base=1070,**kw):return grid(variant,(1,),(2,),tow_ms=ms,base=base,**kw)[0]
                corrupt=bytearray(fragment(105000,1090));corrupt[-1]^=1
                short=fragment(103000,1090)
                frames=[fragment(100000,sync=1),fragment(100000,1090),
                        fragment(101000,sync=1),fragment(101000,1090,station=43),fragment(101000,1090),
                        fragment(102000,sync=1,iod=1),fragment(102000,1090,iod=2),
                        fragment(103000,sync=1),rtcm_payload(short[3:-8]),fragment(104000),
                        fragment(105000,sync=1),bytes(corrupt),fragment(105000,1090)]
                # Split headers, payloads and CRC across TCP writes.
                chunks=[part for frame in frames for part in (frame[:2],frame[2:7],frame[7:-1],frame[-1:])]
                # This case requires a fixed station so a foreign fragment cannot
                # interrupt its GPS/Galileo assembly. Automatic VRS station
                # changes are covered separately by test_receiver_sessions.
                result=wire.run('msm_state_'+str(variant),'rtcm_decoder_node',chunks,
                                RTCM_PARAMS+['-p','station_id:=42'],chunk_delay=.005)
                self.assertEqual([m.tow for m in result['obs']],[100.,101.,102.,104.,105.])
                for msg,systems in zip(result['obs'],[{'G','E'},{'G','E'},{'E'},{'G'},{'G','E'}]):
                    self.assertEqual({o.system for o in msg.observations},systems)
                    self.assertEqual(len(msg.observations),len(systems))
                    for system in systems:
                        # Apply the numeric oracle to each constellation's part.
                        part=type(msg)();part.week=msg.week;part.tow=msg.tow
                        part.observations=[o for o in msg.observations if o.system==system]
                        base=1070 if system=='G' else 1090;family='gps' if system=='G' else 'gal'
                        fixture=grid(variant,(1,),(2,),tow_ms=round(msg.tow*1000),base=base)
                        self.check_epoch(part,variant,family,base,msg.tow,fixture)

if __name__=='__main__':unittest.main(argv=[__file__]+wire.unittest_args)
