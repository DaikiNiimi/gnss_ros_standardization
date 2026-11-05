#!/usr/bin/env python3
"""I04 session and replay regressions through installed ROS interfaces."""
import unittest
import struct
import math
import test_receiver_streams as wire
from test_decoder_boundaries import meas3, msm_frames, bits, RTCM_PARAMS
from test_receiver_observations import PARAMS, marker, measurement

def rawx(tow=100., week=2300, lock=10000, clock_reset=False):
    header=struct.pack('<dHbBBB2s',tow,week,18,1,3 if clock_reset else 1,1,b'\0\0')
    row=struct.pack('<ddf4BH6B',21000000.125,110000000.,0.,0,1,0,0,lock,45,0,0,0,7,0)
    return wire.ubx(2,0x15,header+row)

def oem_range(tow=100000,week=2300,lock=10.):
    row=struct.pack('<HHdfdffffI',1,0,21000000.125,.25,-110000000.,.125,0.,45.,lock,(1<<10)|(1<<11)|(1<<12))
    frame=bytearray(wire.oem(43,struct.pack('<I',1)+row,tow=tow))
    struct.pack_into('<H',frame,14,week)
    return bytes(frame[:-4])+struct.pack('<I',wire.crc32(frame[:-4]))

def sbf_epoch(tow=100000,week=2300):
    frame=measurement(tow)
    p=bytearray(frame[8:]);struct.pack_into('<H',p,4,week)
    return wire.sbf(4027,p)

def clock_step(seconds):
    def send(node,executor):
        from rosgraph_msgs.msg import Clock
        pub=node.create_publisher(Clock,'/clock',10)
        try:
            deadline=wire.time.monotonic()+5
            while pub.get_subscription_count()==0:
                if wire.time.monotonic()>deadline:raise AssertionError('clock subscriber absent')
                executor.spin_once(timeout_sec=.02)
            msg=Clock();msg.clock.sec=seconds;pub.publish(msg)
            deadline=wire.time.monotonic()+.2
            while wire.time.monotonic()<deadline:executor.spin_once(timeout_sec=.02)
        finally:node.destroy_publisher(pub)
    return send

class ReceiverSessions(unittest.TestCase):
    def test_meas3_reference_does_not_cross_reconnect(self):
        for mode in ['decoder','driver']:
            m=wire.run('session_meas3_'+mode,'sbf_'+mode+'_node',
                [meas3(), None, meas3(100100,True)+marker(100100),meas3(102000)],PARAMS,chunk_delay=.05)
            self.assertEqual([e.tow for e in m['obs']],[100.,102.])

    def test_meas3_reference_does_not_cross_rewind(self):
        for mode in ['decoder','driver']:
            m=wire.run('rewind_meas3_'+mode,'sbf_'+mode+'_node',
                [meas3()+marker(),meas3(100200,True)+marker(100200),
                 meas3(100100,True)+marker(100100),meas3(102000)],PARAMS,chunk_delay=.05)
            self.assertEqual(len(m['obs']),3)
            for e,tow in zip(m['obs'],[100.,100.2,102.]):self.assertAlmostEqual(e.tow,tow,places=9)

    def test_rtcm_station_change_after_reconnect(self):
        gps,gal=msm_frames()
        m=wire.run('session_rtcm','rtcm_decoder_node',
            [gps+gal,None,bits(gps,36,12,43)+bits(gal,36,12,43)],RTCM_PARAMS,chunk_delay=.05)
        self.assertEqual([e.tow for e in m['obs']],[100.,100.])

    def test_rtcm_default_station_changes_do_not_mix_fragments(self):
        gps,gal=msm_frames()
        other_gps=bits(bits(gps,36,12,43),48,30,101000)
        other_gal=bits(bits(gal,36,12,43),48,30,101000)
        m=wire.run('station_change_rtcm','rtcm_decoder_node',
            [gps+gal,bits(gps,48,30,101000),other_gal,
             bits(other_gps,48,30,102000)+bits(other_gal,48,30,102000)],RTCM_PARAMS,chunk_delay=.05)
        self.assertEqual([e.tow for e in m['obs']],[100.,101.,102.])
        self.assertEqual({o.system for o in m['obs'][1].observations},{'E'})

    def test_rtcm_constellation_offsets_do_not_reset_navigation(self):
        from test_rtcm_navigation_reference import navigation
        gps,gal=msm_frames()
        nav,_=navigation(1019)
        frames=[nav]
        for tow in range(100,104):
            frames.append(bits(gps,48,30,(tow+14)*1000)+bits(gal,48,30,tow*1000))
        m=wire.run('lane_times_rtcm','rtcm_decoder_node',frames,
            RTCM_PARAMS+['-p','topics.ephemeris:=/review/nav'],chunk_delay=.05)
        populated=[n for n in m['nav'] if n.gnss_ephemeris]
        self.assertTrue(populated)
        first=next(i for i,n in enumerate(m['nav']) if n.gnss_ephemeris)
        self.assertTrue(all(n.gnss_ephemeris for n in m['nav'][first:]))

    def test_raw_receiver_lock_history_is_per_connection(self):
        for protocol,frames in [('ubx',[rawx(),None,rawx(lock=1)]),
                                ('novatel',[oem_range(),None,oem_range(lock=.001)])]:
            for mode in ['decoder','driver']:
                m=wire.run('session_lock_'+protocol+mode,protocol+'_'+mode+'_node',frames,PARAMS,chunk_delay=.05)
                self.assertEqual([e.tow for e in m['obs']],[100.,100.])
                self.assertEqual(m['obs'][-1].observations[0].lli & 1,0)

    def test_rawx_clock_reset_discards_previous_lock_history(self):
        for mode in ['decoder','driver']:
            m=wire.run('clock_reset_'+mode,'ubx_'+mode+'_node',
                [rawx(),rawx(tow=101.,lock=1,clock_reset=True),rawx(tow=102.,lock=1001)],PARAMS,chunk_delay=.05)
            self.assertEqual([e.tow for e in m['obs']],[100.,101.,102.])
            self.assertTrue(all(e.observations[0].lli & 1==0 for e in m['obs']))

    def test_out_of_ros_range_time_is_withheld_and_recovers(self):
        for protocol,frames in [('ubx',[rawx(week=65534),rawx()]),
                                ('sbf',[sbf_epoch(week=65534),sbf_epoch()+marker()]),
                                ('novatel',[oem_range(week=65534),oem_range()])]:
            for mode in ['decoder','driver']:
                m=wire.run('time_range_'+protocol+mode,protocol+'_'+mode+'_node',frames,PARAMS,chunk_delay=.05)
                self.assertEqual([(e.week,e.tow) for e in m['obs']],[(2300,100.)])

    def test_rtcm_rewind_keeps_all_new_fragments_and_fixed_station(self):
        gps,gal=msm_frames()
        later=bits(gps,48,30,101000)+bits(gal,48,30,101000)
        m=wire.run('rewind_rtcm','rtcm_decoder_node',[later,gps+gal],RTCM_PARAMS,chunk_delay=.05)
        self.assertEqual([e.tow for e in m['obs']],[101.,100.])
        self.assertTrue(all({o.system for o in e.observations}=={'G','E'} for e in m['obs']))
        m=wire.run('session_fixed_rtcm','rtcm_decoder_node',
            [gps+gal,None,bits(gps,36,12,43)+bits(gal,36,12,43)+gps+gal],
            RTCM_PARAMS+['-p','station_id:=42'],chunk_delay=.05)
        self.assertEqual([e.tow for e in m['obs']],[100.,100.])

    def test_gnss_epochs_survive_ros_clock_steps(self):
        for protocol,make in [('ubx',lambda t:rawx(tow=t)),
                              ('sbf',lambda t:sbf_epoch(tow=int(t*1000))+marker(int(t*1000))),
                              ('novatel',lambda t:oem_range(tow=int(t*1000)))]:
            for mode in ['decoder','driver']:
                m=wire.run('ros_clock_'+protocol+mode,protocol+'_'+mode+'_node',
                    [clock_step(500),make(100),clock_step(100),make(101),clock_step(15000),make(102)],
                    PARAMS+['-p','use_sim_time:=true'],chunk_delay=.05)
                self.assertEqual([e.tow for e in m['obs']],[100.,101.,102.])
                stamps=[e.header.stamp.sec+e.header.stamp.nanosec*1e-9 for e in m['obs']]
                self.assertEqual([t-stamps[0] for t in stamps],[0.,1.,2.])

    def test_nmea_source_grace_uses_steady_clock_when_ros_time_paused(self):
        for protocol in ['ubx','sbf','novatel']:
            m=wire.run('paused_grace_'+protocol,protocol+'_decoder_node',
                [wire.nmea('000000'),wire.nmea('000001')+wire.nmea('000002')],
                ['-p','use_sim_time:=true','-p','use_gps_timestamp:=true'],chunk_delay=1.2)
            self.assertGreaterEqual(len(m['sol']),1)
            self.assertTrue(all(e.time_week==2300 for e in m['sol']))

    def test_decoder_source_selection_restarts_after_reconnect(self):
        best=bytearray(72);struct.pack_into('<II3dfI3f',best,0,0,16,35.,139.,100.,30.,61,1.,2.,3.);best[65]=12
        geo=bytearray(88);struct.pack_into('<IHBB3d',geo,0,100000,2300,1,0,.6,2.4,100.);geo[66]=12
        for protocol,binary in [('ubx',wire.ubx(1,7,wire.pvt())),
                                ('sbf',wire.sbf(4007,geo)),('novatel',wire.oem(42,best))]:
            m=wire.run('session_source_'+protocol,protocol+'_decoder_node',
                [binary,None,wire.nmea('000000'),wire.nmea('000001')+wire.nmea('000002')],chunk_delay=1.2)
            self.assertGreaterEqual(len(m['sol']),2)
            self.assertTrue(any(18.<=e.time_tow<=20. for e in m['sol']))

    def test_rtcm_incomplete_epoch_does_not_cross_reconnect(self):
        gps,gal=msm_frames()
        m=wire.run('session_partial_rtcm','rtcm_decoder_node',[gps,None,gal],RTCM_PARAMS,chunk_delay=.05)
        self.assertEqual([e.tow for e in m['obs']],[100.])
        self.assertEqual({o.system for o in m['obs'][0].observations},{'E'})

    def test_ubx_weekless_pvt_requires_fresh_time_reference(self):
        def wait_for_expiry(node,executor):
            deadline=wire.time.monotonic()+5.2
            while wire.time.monotonic()<deadline:executor.spin_once(timeout_sec=.02)
        # Empty RAWX/RAW frames still carry a backend-validated full epoch.
        # Exercise populated RAWX too, and ensure all references expire alike.
        references=[('rawx',rawx),
            ('empty_rawx',lambda t:wire.ubx(2,0x15,struct.pack('<dHbBBB2s',t,2300,18,0,1,1,b'\0\0'))),
            ('empty_raw',lambda t:wire.ubx(2,0x10,struct.pack('<IHBB',int(t*1000),2300,0,0)))]
        for mode in ['decoder','driver']:
            for name,reference in references:
                with self.subTest(mode=mode,reference=name):
                    m=wire.run('week_reference_'+mode+'_'+name,'ubx_'+mode+'_node',
                        [reference(100.)+wire.ubx(1,7,wire.pvt(100000)),wait_for_expiry,
                         wire.ubx(1,7,wire.pvt(101000)),reference(102.)+wire.ubx(1,7,wire.pvt(102000))],
                        PARAMS,chunk_delay=.15)
                    self.assertEqual([e.time_tow for e in m['sol']],[100.,102.])

if __name__=='__main__':
    unittest.main(argv=[wire.sys.argv[0]]+wire.unittest_args,verbosity=2)
