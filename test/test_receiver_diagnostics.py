#!/usr/bin/env python3
"""Controlled input -> real node diagnostics, without changing accepted data."""
import pathlib
import re
import struct
import unittest
import test_receiver_streams as wire
wire.MIN_DRAIN_SECONDS=5.1
from test_protocol_specification import frame,payload,row,meas_spec,sbf_spec,oem_spec,range_row

PARAMS=['-p','topics.observation:=/review/obs','-p','use_gps_timestamp:=true']


def health(label):
    text=pathlib.Path(wire.TEMP.name,label+'.log').read_text()
    lines=[dict(re.findall(r'(\w+)=([^\s]+)',line)) for line in text.splitlines() if 'Receiver health:' in line]
    if not lines:raise AssertionError(text)
    stamps=[float(re.search(r'\[([0-9]+\.[0-9]+)\]',line)[1])
            for line in text.splitlines() if 'Receiver health:' in line]
    if any(b-a<4.95 for a,b in zip(stamps,stamps[1:])):
        raise AssertionError('diagnostics exceeded the internal 5 second interval: '+str(stamps))
    return lines[-1]


class ReceiverDiagnostics(unittest.TestCase):
    def test_rawx_data0_reserved_is_not_an_unknown_version(self):
        p=bytearray(payload(0));p[13]=255
        for mode in ('driver','decoder'):
            label='rawx_legacy_diagnostic_'+mode
            r=wire.run(label,'ubx_'+mode+'_node',frame(p),PARAMS+['-p','rawx_layout:=data0'])
            self.assertEqual(len(r['obs']),1)
            h=health(label)
            self.assertEqual(h['unknown_versions'],'0')
            self.assertEqual(h['payload_errors'],'0')

    def test_covariance_without_pvt_reports_retirement_all_six_paths(self):
        import time
        for family in ('ubx','sbf','novatel'):
            for mode in ('driver','decoder'):
                with self.subTest(family=family,mode=mode):
                    label='diagnostic_covariance_only_'+family+'_'+mode
                    if family=='ubx':
                        p=bytearray(64);struct.pack_into('<I',p,0,100000);p[5]=1
                        data=wire.ubx(1,0x36,p)
                    elif family=='sbf':
                        p=bytearray(48);struct.pack_into('<IH',p,0,100000,2300);p[6]=1
                        data=wire.sbf(5906,p)
                    else:
                        p=bytearray(112);struct.pack_into('<I',p,4,16)
                        data=wire.oem(241,p)
                    def wait_retired(node,executor):
                        deadline=time.monotonic()+1.2
                        while time.monotonic()<deadline:executor.spin_once(timeout_sec=.05)
                    params=[]
                    if mode=='driver':
                        # Drivers explicitly opt in to these supplementary blocks.
                        message={'ubx':'nav_cov','sbf':'pos_cov_geodetic','novatel':'bestxyz'}[family]
                        params+=['-p','messages.'+message+':=true']
                    result=wire.run(label,family+'_'+mode+'_node',[data,wait_retired],params)
                    self.assertEqual(len(result['sol']),0)
                    h=health(label)
                    self.assertEqual(h['pvt_incomplete_epochs'],'1')
                    self.assertEqual(h['pvt_pending'],'no')
                    self.assertEqual(h['frontend_rejected_events'],'0')

    def test_backend_exclusion_counts_survive_both_ros_paths(self):
        for family in ('ubx','sbf','novatel'):
            for mode in ('driver','decoder'):
                with self.subTest(family=family,mode=mode):
                    if family=='ubx':
                        rows=[row(64),row(1,signal=255),row(1)]
                        data=frame(struct.pack('<dHbBBB2x',100.5,2300,18,3,1,1)+b''.join(rows))
                    elif family=='novatel':
                        data=oem_spec([range_row(prn=64),range_row(flags=0x1c00|(31<<21)),range_row()])
                    else:
                        good=meas_spec();p=bytearray(good[8:]);p[14]=0
                        invalid_sat=sbf_spec(4027,p,revision=1)
                        p=bytearray(good[8:]);p[13]=31;p[30]=0
                        data=invalid_sat+sbf_spec(4027,p,revision=1)+good+sbf_spec(5922,struct.pack('<IH2x',100500,2300))
                    label='diagnostic_filters_'+family+'_'+mode
                    result=wire.run(label,family+'_'+mode+'_node',data,PARAMS)
                    self.assertEqual(len(result['obs']),1)
                    h=health(label)
                    self.assertEqual(h['satellite_exclusions'],'1')
                    self.assertEqual(h['signal_mapping_exclusions'],'1')
                    self.assertEqual(h['payload_errors'],'0')
                    self.assertEqual(h['capacity_unread_rows'],'0')
                    self.assertIn(':',h['satellite_exclusions_last'])

    def test_ubx_navigation_pvt_and_imu_waits_then_recover(self):
        import time
        for mode in ('driver','decoder'):
            label='diagnostic_pending_ubx_'+mode
            nav=bytearray(48);nav[1]=1;nav[4]=10;nav[6]=2
            struct.pack_into('<I',nav,12,1<<8) # GPS subframe1, stored before ephemeris completes
            def axes(types):
                return wire.ubx(16,3,bytes(4)+b''.join(struct.pack('<II',(kind<<24)|1,100) for kind in types))
            def wait_pending(node,executor):
                deadline=time.monotonic()+5.1
                while time.monotonic()<deadline:executor.spin_once(timeout_sec=.05)
                h=health(label)
                self.assertEqual(h['pvt_pending'],'no')
                self.assertEqual(h['imu_pending'],'yes')
                self.assertEqual(h['navigation_wait_events'],'1')
                self.assertEqual(h['navigation_pending'],'1')
                self.assertEqual(h['navigation_states'],'1:gps:wait:6')
            def wait_recovered(node,executor):
                # Q1: the default 50 ms PVT flush has elapsed; allow publication.
                deadline=time.monotonic()+2.1
                while time.monotonic()<deadline:executor.spin_once(timeout_sec=.05)
            data=[wire.ubx(1,7,b'\x00')+wire.ubx(2,0x13,nav)+wire.ubx(1,7,wire.pvt())+axes([14,13,5]),
                  wait_pending,axes([16,17,18]),wait_recovered]
            result=wire.run(label,'ubx_'+mode+'_node',data,
                [],chunk_delay=.05)
            self.assertEqual(len(result['raw']),1)
            self.assertEqual(len(result['sol']),1)
            h=health(label)
            self.assertEqual(h['imu_pending'],'no')
            self.assertEqual(h['pvt_pending'],'no')
            self.assertEqual(h['imu_wait_events'],'1')
            self.assertEqual(h['frontend_rejected_events'],'1')
            self.assertEqual(h['length_events'],'1')
            self.assertEqual(h['navigation_inconsistent_events'],'0')

    def test_controlled_frame_reasons_and_recovery_all_receiver_paths(self):
        for family in ('ubx','sbf','novatel'):
            for mode in ('driver','decoder'):
                with self.subTest(family=family,mode=mode):
                    if family=='ubx':
                        good=frame(payload());unknown=wire.ubx(0x99,0x55,b'')
                        bad_time=frame(payload(week=0));bad_version=bytearray(payload());bad_version[13]=2
                        prefix=[unknown,frame(bad_version),bad_time]
                        expected_version=1
                    elif family=='sbf':
                        good=meas_spec()+sbf_spec(5922,struct.pack('<IH2x',100500,2300))
                        unknown=sbf_spec(8191,struct.pack('<IH2x',100500,2300))
                        bad_time=sbf_spec(4027,struct.pack('<IH6B',0xffffffff,65535,0,20,12,0,0,0))
                        prefix=[unknown,bad_time];expected_version=0
                    else:
                        good=oem_spec([range_row()]);unknown=oem_spec([],mid=65535)
                        prefix=[unknown,oem_spec([range_row()],status=20)];expected_version=0
                    # Damage only the trailing CRC byte; subsequent frame must recover.
                    # For SBF, its CRC lives at offset2 instead of the tail.
                    corrupt=bytearray(good if family!='sbf' else meas_spec())
                    corrupt[2 if family=='sbf' else -1]^=1
                    label='diagnostics_'+family+'_'+mode
                    result=wire.run(label,family+'_'+mode+'_node',[*prefix,bytes(corrupt),good],PARAMS,chunk_delay=.1)
                    self.assertEqual(len(result['obs']),1)
                    last=health(label)
                    self.assertEqual(last['unknown_ids'],'1')
                    self.assertEqual(last['checksum_rejected'],'1')
                    self.assertEqual(last['unknown_versions'],str(expected_version))
                    self.assertGreaterEqual(int(last['time_unresolved_frames']),1)
                    self.assertEqual(last['published_epochs'],'1')
                    self.assertEqual(last['last_epoch'],'2300:100.5')
                    self.assertEqual(last['backend_filter_counts'],'instrumented')
                    self.assertEqual(last['capacity_unread_rows'],'0')
                    self.assertEqual(last['assembly_pending'],'no')

    def test_sbf_assembly_pending_then_timeout_publishes_once(self):
        for mode in ('driver','decoder'):
            label='diagnostics_assembly_'+mode
            def observe_pending(node,executor):
                import time
                deadline=time.monotonic()+5.1
                while time.monotonic()<deadline:executor.spin_once(timeout_sec=.05)
                self.assertEqual(health(label)['assembly_pending'],'no')
            result=wire.run(label,'sbf_'+mode+'_node',[meas_spec(),observe_pending],
                PARAMS+[],chunk_delay=.05)
            self.assertEqual(len(result['obs']),1)
            self.assertEqual(health(label)['assembly_started'],'1')


class RtcmDiagnostics(unittest.TestCase):
    def test_msm_satellite_and_signal_exclusions_are_active_cells(self):
        from test_decoder_boundaries import bits,msm_frames,RTCM_PARAMS
        gps,gal=msm_frames()
        # Satellite mask starts at97 in a full frame; keep one active cell,
        # moving GPS PRN1 to out-of-range PRN64. Signal mask starts at161.
        bad_sat=bits(gps,97,64,1)
        bad_signal=bits(gps,161,32,1<<31) # reserved signal ID1
        label='diagnostic_filters_rtcm'
        result=wire.run(label,'rtcm_decoder_node',[bad_sat,bad_signal,gps,gal],
            RTCM_PARAMS+[],chunk_delay=.05)
        self.assertEqual(len(result['obs']),1)
        self.assertEqual(len(result['obs'][0].observations),2)
        h=health(label)
        self.assertEqual(h['satellite_exclusions'],'1')
        self.assertEqual(h['signal_mapping_exclusions'],'1')
        self.assertEqual(h['payload_errors'],'0')

    def test_unknown_crc_and_pending_fragment_are_distinct(self):
        from test_decoder_boundaries import rtcm_payload,msm_frames,RTCM_PARAMS
        import time
        valid=msm_frames();unknown=rtcm_payload(bytes.fromhex('fff000'))
        corrupt=bytearray(valid[0]);corrupt[-1]^=1
        label='diagnostics_rtcm'
        def pending(node,executor):
            deadline=time.monotonic()+5.1
            while time.monotonic()<deadline:executor.spin_once(timeout_sec=.05)
            self.assertEqual(health(label)['assembly_pending'],'yes')
        result=wire.run(label,'rtcm_decoder_node',[unknown,bytes(corrupt),valid[0],pending,valid[1]],
            RTCM_PARAMS+[],chunk_delay=.05)
        self.assertEqual(len(result['obs']),1)
        h=health(label)
        self.assertEqual(h['unknown_ids'],'1');self.assertEqual(h['checksum_rejected'],'1')
        self.assertEqual(h['assembly_pending'],'no');self.assertEqual(h['published_epochs'],'1')


if __name__=='__main__':unittest.main(argv=[__file__]+wire.unittest_args)
