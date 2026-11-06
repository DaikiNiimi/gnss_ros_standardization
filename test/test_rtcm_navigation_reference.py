#!/usr/bin/env python3
"""R2 field definitions -> independently packed navigation -> ROS numeric fields.

The table records the pinned independent source hashes. Integer test values and
ROS expectations are authored here; neither RTKLIB encoder nor observed output
supplies them. This is limited source-reference conformance, not normative RTCM.
"""
import json
import math
from pathlib import Path
import unittest
import test_receiver_streams as wire
from test_decoder_boundaries import rtcm_payload,RTCM_PARAMS

TABLE=json.loads(Path(__file__).with_name('rtcm_navigation_reference.json').read_text())


def navigation(mid,issue=0,raw_fields=None):
    physical={};bits=[format(mid,'012b')]
    for index,row in enumerate(TABLE['messages'][str(mid)]):
        name,n,kind=row['name'],row['width'],row['storage']
        scale,bias=row['scale'],row['bias']
        value=0
        if name.endswith('satellite_id'):value=1
        elif name.endswith('week_number'):value=1276 if mid in (1045,1046) else 944 if mid==1042 else 252
        elif name=='sqrt_a_sqrt_m':value=round(5153.5/scale)
        elif name=='eccentricity':value=round(.01/scale)
        elif name in ('toe_s','toc_s'):value=round(96000/scale)
        elif name in ('iode','iodec','aode','iodnav','iodc','aodc'):value=(700 if name=='iodnav' else 17)+issue
        elif name in ('ura_index','sisa_e1_e5a_index','sisa_e1_e5b_index'):value=3
        elif name=='code_on_l2_ind':value=2
        elif kind.startswith(('I','SM')):value=(-1 if index%2 else 1)*min(13,(1<<(n-1))-1)
        # Explicit GLONASS state and flags, independently chosen from DF definitions.
        if mid==1020:
            fixed={'glo_satellite_freq_chan_number':0,'glo_alm_health_flag':1,'glo_alm_health_avail_flag':1,
                   'p1_ind':2,'p2_flag':1,'p3_flag':1,'glo_m_p4_flag':1,'glo_m_p_ind':3,'glo_m_m_ind':1,
                   'tk_h':3,'tk_min':2,'tk_s':1,'tb_min':12+issue,'glo_eph_health_flag':1,
                   'glo_m_3str_ln_flag':1,'glo_m_ft_ind':6,'en_d':7}
            if name in fixed:value=fixed[name]
        if raw_fields and name in raw_fields:value=raw_fields[name]
        assert -(1<<(n-1))<=value<(1<<n),(name,value,n)
        physical[name]=value*scale+bias
        encoded=((1<<(n-1))|(-value)) if value<0 and kind.startswith('SM') else value&((1<<n)-1)
        bits.append(format(encoded,f'0{n}b'))
    payload=''.join(bits);payload+='0'*((-len(payload))%8)
    return rtcm_payload(int(payload,2).to_bytes(len(payload)//8,'big')),physical


class NavigationReference(unittest.TestCase):
    def test_real_historical_log_auto_date_matches_explicit_reference(self):
        from rosidl_runtime_py.convert import message_to_ordereddict
        def values_of(m):  # field values with NaN comparable; CDR padding is not data
            def norm(v):
                if isinstance(v,float) and math.isnan(v):return 'nan'
                if isinstance(v,dict):return {k:norm(x) for k,x in v.items()}
                if isinstance(v,(list,tuple)):return [norm(x) for x in v]
                return v
            return norm(message_to_ordereddict(m))
        path=Path(__file__).resolve().parents[1]/'third_party/RTKLIB/test/data/rcvraw/testglo.rtcm3'
        data=path.read_bytes();chunks=[data[i:i+1024] for i in range(0,len(data),1024)]
        outputs=[]
        for label,extra in [('auto',[]),('explicit',['-p','reference_week:=1562',
                                                    '-p','reference_tow:=515145.0'])]:
            result=wire.run('real_historical_'+label,'rtcm_decoder_node',chunks,
                           ['-p','topics.ephemeris:=/review/nav']+extra,chunk_delay=.02)
            self.assertEqual(len(result['obs']),186)
            self.assertEqual({m.week for m in result['obs']},{1562})
            largest=max(result['nav'],key=lambda m:len(m.gnss_ephemeris)+len(m.glonass_ephemeris))
            self.assertEqual((len(largest.gnss_ephemeris),len(largest.glonass_ephemeris)),(15,18))
            # Only host receipt stamps differ across runs; compare every other
            # serialized observation/navigation field, including NaN payloads.
            values=result['obs']+[largest]
            for m in values:m.header.stamp.sec=m.header.stamp.nanosec=0
            outputs.append([values_of(m) for m in values])
        self.assertEqual(outputs[0],outputs[1])

    def test_glonass_absolute_date_recovers_earlier_gps_navigation(self):
        gps,_=navigation(1019,raw_fields={'gps_week_number':538,
                                        'toe_s':32200,'toc_s':32200})
        glo,_=navigation(1020,raw_fields={'glo_m_nt_d':719,'glo_m_n4_year':4,
            'additional_data_flag':1,'tk_h':2,'tk_min':5,'tk_s':1,'tb_min':9})
        result=wire.run('glo_absolute_date','rtcm_decoder_node',[gps,glo],
                        ['-p','topics.ephemeris:=/review/nav'],chunk_delay=.03)
        snapshot=result['nav'][-1]
        self.assertEqual(len(snapshot.gnss_ephemeris),1)
        self.assertEqual(len(snapshot.glonass_ephemeris),1)
        e=snapshot.gnss_ephemeris[0];g=snapshot.glonass_ephemeris[0]
        self.assertEqual((e.week,e.toe,e.toc,e.ttr),(1562,515200.,515200.,515145.))
        self.assertEqual((g.week,g.toe,g.tof),(1562,515715.,515145.))

    def test_gps_toe_and_toc_across_broadcast_week_boundary(self):
        frame,_=navigation(1019,raw_fields={'gps_week_number':2300%1024,
                                          'toe_s':0,'toc_s':37799})
        result=wire.run('gps_toe_toc_week','rtcm_decoder_node',[frame],
            ['-p','reference_week:=2300','-p','reference_tow:=604790.0',
             '-p','topics.ephemeris:=/review/nav'],chunk_delay=.03)
        nav=[e for m in result['nav'] for e in m.gnss_ephemeris]
        self.assertEqual(len(nav),1)
        self.assertEqual((nav[0].week,nav[0].toe,nav[0].toc,nav[0].ttr),
                         (2301,0.,604784.,604790.))

    def test_gps_rollover_uses_stream_reference_not_host_date(self):
        for week in (1100,1563,2300):
            frame,_=navigation(1019,raw_fields={'gps_week_number':week%1024})
            result=wire.run('gps_reference_'+str(week),'rtcm_decoder_node',[frame],
                ['-p',f'reference_week:={week}','-p','reference_tow:=96000.0',
                 '-p','topics.ephemeris:=/review/nav'],chunk_delay=.03)
            nav=[e for m in result['nav'] for e in m.gnss_ephemeris]
            self.assertEqual(len(nav),1)
            self.assertEqual((nav[0].week,nav[0].toe,nav[0].toc),(week,96000.,96000.))

    def test_absolute_date_announcement_seeds_historical_stream(self):
        # R2 msg1013: station(12), MJD(16), UTC SOD(17), count(5), leap(8).
        week=1100;gps_tow=96000;leaps=13
        utc=week*604800+gps_tow-leaps
        mjd=44244+utc//86400;sod=utc%86400
        values=[(1013,12),(42,12),(mjd,16),(sod,17),(0,5),(leaps,8)]
        bits=''.join(format(v,f'0{n}b') for v,n in values);bits+='0'*((-len(bits))%8)
        announcement=rtcm_payload(int(bits,2).to_bytes(len(bits)//8,'big'))
        nav,_=navigation(1019,raw_fields={'gps_week_number':week%1024})
        result=wire.run('gps_absolute_reference','rtcm_decoder_node',[announcement,nav],
                        ['-p','topics.ephemeris:=/review/nav'],chunk_delay=.03)
        records=[e for m in result['nav'] for e in m.gnss_ephemeris]
        self.assertEqual(len(records),1)
        self.assertEqual((records[0].week,records[0].toe,records[0].ttr),(week,96000.,96000.))
        # The operator's explicit reference takes priority over an announcement
        # from a different recording. A malformed announcement cannot seed time.
        current,_=navigation(1019)
        result=wire.run('gps_reference_priority','rtcm_decoder_node',
                        [announcement,current],RTCM_PARAMS+['-p','topics.ephemeris:=/review/nav'],chunk_delay=.03)
        records=[e for m in result['nav'] for e in m.gnss_ephemeris]
        self.assertEqual({e.week for e in records},{2300})

    def check_number(self,actual,expected,label):
        self.assertTrue(math.isclose(actual,expected,rel_tol=2e-14,abs_tol=1e-18),(label,actual,expected))

    def test_navigation_week_and_glonass_day_boundaries(self):
        for mid in (1019,1041,1042,1044,1045,1046):
            rows=TABLE['messages'][str(mid)]
            week_field=next(r['name'] for r in rows if r['name'].endswith('week_number'))
            time_scale=next(r['scale'] for r in rows if r['name']=='toe_s')
            encoded_week=1276 if mid in (1045,1046) else 944 if mid==1042 else 252
            last=int(604780/time_scale)
            first,unused=navigation(mid,0,{week_field:encoded_week,'toe_s':last,'toc_s':last})
            second,unused=navigation(mid,1,{week_field:encoded_week+1,'toe_s':0,'toc_s':0})
            params=['-p','reference_week:=2300','-p','reference_tow:=604790.0',
                    '-p','topics.ephemeris:=/review/nav','-p','use_gps_timestamp:=true']
            result=wire.run('navigation_week_'+str(mid),'rtcm_decoder_node',[first,second],params,chunk_delay=.05)
            snapshots=[m for m in result['nav'] if m.gnss_ephemeris]
            self.assertEqual(len(snapshots),2)
            for index,(week,tow) in enumerate([(2300,last*time_scale+(14 if mid==1042 else 0)),
                                                (2301,14 if mid==1042 else 0)]):
                matches=[e for e in snapshots[index].gnss_ephemeris if e.week==week and e.toe==tow]
                self.assertEqual(len(matches),1,(mid,index,[(e.week,e.toe) for e in snapshots[index].gnss_ephemeris]))
                e=matches[0]
                self.assertEqual(e.toc,tow)
                self.assertEqual(e.toes,last*time_scale if index==0 else 0)
                self.assertEqual(e.ttr,604790.)
        # GLONASS Saturday 23:45 and Sunday 00:00 -> GPST 2300, 593118/594018.
        a,_=navigation(1020,0,{'tb_min':95,'tk_h':23,'tk_min':59,'tk_s':1})
        b,_=navigation(1020,1,{'tb_min':0,'tk_h':0,'tk_min':0,'tk_s':0})
        result=wire.run('navigation_glonass_day','rtcm_decoder_node',[a,b],
                        ['-p','reference_week:=2300','-p','reference_tow:=593018.0',
                         '-p','topics.ephemeris:=/review/nav'],chunk_delay=.05)
        snapshots=[m for m in result['nav'] if m.glonass_ephemeris]
        self.assertEqual(len(snapshots),2)
        for index,(iod,toe,tof) in enumerate([(95,593118.,593988.),(0,594018.,594018.)]):
            matches=[e for e in snapshots[index].glonass_ephemeris if e.iode==iod]
            self.assertEqual(len(matches),1)
            self.assertEqual((matches[0].week,matches[0].toe,matches[0].tof),(2300,toe,tof))

    def test_seven_navigation_types_fields_and_update(self):
        systems={1019:'G',1020:'R',1041:'I',1042:'C',1044:'J',1045:'E',1046:'E'}
        for mid,system in systems.items():
            with self.subTest(message=mid):
                first,_=navigation(mid);last,expected=navigation(mid,1)
                # Truncated, valid, duplicate, new issue: only the two valid updates publish.
                frames=[rtcm_payload(first[3:-7]),first,first,last]
                result=wire.run('rtcm_navigation_'+str(mid),'rtcm_decoder_node',frames,
                    RTCM_PARAMS+['-p','topics.ephemeris:=/review/nav'],chunk_delay=.05)
                # Snapshots retain history, with no extra snapshot for the
                # duplicate. The initial empty snapshot may precede discovery.
                sizes=[len(m.gnss_ephemeris)+len(m.glonass_ephemeris) for m in result['nav']]
                self.assertEqual([n for n in sizes if n],[1,2])
                snapshot=result['nav'][-1]
                if mid==1020:
                    self.assertEqual(len(snapshot.glonass_ephemeris),2)
                    e=next(e for e in snapshot.glonass_ephemeris if e.iode==13)
                    self.assertEqual((e.system,e.satid,e.week,e.frq,e.iode,e.age,e.sva),('R','R01',2300,-7,13,7,6))
                    self.assertEqual(e.flags,251);self.assertEqual(e.svh,15)
                    self.assertEqual(e.toe,918.);self.assertEqual(e.tof,168.)
                    for output,suffix in [('pos','_km'),('vel','_first_deriv_km_s'),('acc','_second_deriv_km_s2')]:
                        for i,axis in enumerate('xyz'):self.check_number(getattr(e,output)[i],expected[axis+'n'+suffix]*1000,output+axis)
                    for output,source in [('gamn','gamma_n'),('taun','tau_n_s'),('dtaun','glo_m_delta_tau_n_s')]:
                        self.check_number(getattr(e,output),expected[source],output)
                    continue
                self.assertEqual(len(snapshot.gnss_ephemeris),2)
                e=next(e for e in snapshot.gnss_ephemeris if e.iodc==(701 if system=='E' else 18))
                self.assertEqual((e.system,e.satid,e.week),(system,system+'01',2300))
                # Upstream f542de90425010781d0b52040492e00b9f30684f deliberately
                # uses an SSR-matching IODE for BDS. AODE is not exposed by this
                # path; preserving RTKLIB semantics is the accepted ROS contract.
                self.assertEqual(e.iode,int(expected['toc_s']/720)%240 if system=='C' else 701 if system=='E' else 18)
                self.assertEqual(e.iodc,701 if system=='E' else 18)
                self.assertEqual(e.prn,193 if system=='J' else 1)
                self.assertEqual(e.sva,3);self.assertEqual(e.svh,0)
                self.assertEqual(e.code,{1019:2,1044:2,1045:258,1046:517}.get(mid,0))
                # 1044 carries no L2 P-data flag; RTKLIB's QZSS mapping
                # explicitly supplies 1. This is a backend semantic default,
                # not an independently decoded wire field.
                self.assertEqual(e.flag,1 if mid==1044 else 0)
                self.assertEqual(e.fit,4 if mid==1019 else 2 if mid==1044 else 0)
                self.assertEqual(e.ttr,100.)
                self.assertEqual(e.toe,96014. if system=='C' else 96000.)
                self.assertEqual(e.toc,e.toe);self.assertEqual(e.toes,96000.)
                self.check_number(e.a,expected['sqrt_a_sqrt_m']**2,'a')
                self.check_number(e.e,expected['eccentricity'],'e')
                for out,src in [('i0','i0_sc'),('omg0','omega0_sc'),('omg','omega_sc'),('m0','m0_sc'),
                                ('deln','delta_n_sc_s'),('omgd','omegadot_sc_s'),('idot','idot_sc_s')]:
                    self.check_number(getattr(e,out),expected[src]*math.pi,out)
                for name in ('crc','crs','cuc','cus','cic','cis'):
                    self.check_number(getattr(e,name),expected[name+('_m' if name in ('crc','crs') else '_rad')],name)
                for out,src in [('f0','a0_s' if system=='C' else 'af0_s'),
                                ('f1','a1_s_s' if system=='C' else 'af1_s_s'),
                                ('f2','a2_s_s2' if system=='C' else 'af2_s_s2')]:
                    self.check_number(getattr(e,out),expected[src],out)
                delay=['tgd1_s','tgd2_s'] if system=='C' else ['bgd_e1_e5a_s']+(['bgd_e1_e5b_s'] if mid==1046 else []) if system=='E' else ['tgd_s']
                for i,name in enumerate(delay):self.check_number(e.tgd[i],expected[name],name)
                self.assertEqual((e.adot,e.ndot),(0.,0.))

if __name__=='__main__':unittest.main(argv=[__file__]+wire.unittest_args)
