#!/usr/bin/env python3
"""Focused observation validity/completion tests using real ROS executables."""
import math
import pathlib
import struct
import subprocess
import unittest
import test_receiver_streams as wire

PARAMS=['-p','topics.observation:=/review/obs','-p','use_gps_timestamp:=true']

def measurement(tow=100000,missing=False):
    row=bytearray(20);row[2]=1;row[3]=4
    struct.pack_into('<Ii',row,4,3820130941,-2147483648 if missing else 0)
    row[14]=128;row[15]=255 if missing else 140;struct.pack_into('<H',row,16,65535)
    return wire.sbf(4027,struct.pack('<IH6B',tow,2300,1,20,12,0,0,0)+row)

def marker(tow=100000):
    return wire.sbf(5922,struct.pack('<IH2x',tow,2300))

class ReceiverObservations(unittest.TestCase):
    def test_bdsnav_extended_svid_both_paths(self):
        # mosaic-X5 FW4.15 reference 4.2.6, 4081 rev0; SVID table 4.1.9.
        frames=[];prns=[1,32,33,40,41,50,51,59,60,62,63]
        for prn in prns:
            p=bytearray(132);struct.pack_into('<IH',p,0,100000,2300)
            p[6]=prn+(140 if prn<=40 else 182);struct.pack_into('<H',p,8,944)
            p[10]=2;p[12]=7;p[13]=8
            struct.pack_into('<I',p,24,100);struct.pack_into('<d',p,60,.01)
            struct.pack_into('<dI',p,72,5153.5,100);struct.pack_into('<HH',p,128,944,944)
            frames.append(wire.sbf(4081,p))
        for mode in ['decoder','driver']:
            m=wire.run('bdsnav_svid_'+mode,'sbf_'+mode+'_node',b''.join(frames),
                       ['-p','topics.ephemeris:=/review/nav'])
            records={e.prn:e for batch in m['nav'] for e in batch.gnss_ephemeris}
            self.assertEqual(set(records),set(prns))
            for prn,e in records.items():
                self.assertEqual((e.system,e.satid,e.week,e.toe,e.iode,e.iodc),('C',f'C{prn:02d}',2300,114.,8,7))
                self.assertEqual(e.a,5153.5**2)

    def test_rtcm_glonass_observation_channels_without_navigation(self):
        path=pathlib.Path(wire.TEMP.name)/'glonass.rtcm3'
        subprocess.run([wire.B+'make_rtcm_fixture',str(path),'glonass'],check=True)
        m=wire.run('rtcm_fcn','rtcm_decoder_node',path.read_bytes(),
                   ['-p','reference_week:=2300','-p','reference_tow:=100.0'])
        self.assertEqual([o.freq for e in m['obs'] for o in e.observations],[0,7,13])
        self.assertFalse(any(batch.glonass_ephemeris for batch in m['nav']))

    def test_glonass_observation_channels_without_navigation(self):
        for protocol in ['ubx','sbf','novatel']:
            frames=[]
            for index,encoded in enumerate([0,7,13]):
                tow=100.+index
                if protocol=='ubx':
                    header=struct.pack('<dHbBBB2s',tow,2300,18,1,1,1,b'\0\0')
                    row=struct.pack('<ddf4BH6B',21000000.125,-110000000.,0.,6,1,0,encoded,10000,45,0,0,0,7,0)
                    frames.append(wire.ubx(2,0x15,header+row))
                elif protocol=='sbf':
                    row=bytearray(20);row[1]=8;row[2]=38;row[3]=4
                    struct.pack_into('<I',row,4,3820130941)
                    row[15]=140;row[18]=(encoded+1)<<3
                    frames.append(wire.sbf(4027,struct.pack('<IH6B',int(tow*1000),2300,1,20,12,0,0,0)+row))
                    frames.append(marker(int(tow*1000)))
                else:
                    flags=(1<<16)|(1<<12)|(1<<11)|(1<<10)
                    row=struct.pack('<HHdfdffffI',38,encoded,21000000.125,.25,-110000000.,.125,-1234.,45.,10.,flags)
                    frames.append(wire.oem(43,struct.pack('<I',1)+row,tow=int(tow*1000)))
            for mode in ['decoder','driver']:
                m=wire.run('fcn_'+protocol+'_'+mode,protocol+'_'+mode+'_node',frames,PARAMS)
                self.assertEqual(len(m['obs']),3)
                self.assertEqual([e.observations[0].freq for e in m['obs']],[0,7,13])
                self.assertTrue(all(e.observations[0].system=='R' for e in m['obs']))
                self.assertFalse(any(batch.glonass_ephemeris for batch in m['nav']))

    def test_sbf_cnav_rejection_and_glonass_fields_both_paths(self):
        gps=bytearray(164)
        struct.pack_into('<IHBBH',gps,0,100000,2300,1,0,2300)
        struct.pack_into('<II',gps,12,400000,300)
        struct.pack_into('<ddff',gps,20,26560000.,-.125,1e-9,-1e-12)
        struct.pack_into('<d',gps,52,.01)
        struct.pack_into('<I',gps,120,300)
        glo=bytearray(88)
        struct.pack_into('<IHBB',glo,0,100000,2300,38,1)
        struct.pack_into('<3d',glo,8,19000.,1000.,15000.)
        struct.pack_into('<IH',glo,68,100,2300 % 1024)
        struct.pack_into('<H',glo,78,180)
        struct.pack_into('<H',glo,86,1234)
        gps[11]=253  # signed URA_ED = -3
        struct.pack_into('<bBBB',gps,124,-2,1,2,2300%256)
        struct.pack_into('<4f',gps,148,1e-9,-2e10,3e-9,4e-9)
        updated=bytearray(gps);struct.pack_into('<d',updated,136,1e-5)
        frames=[wire.sbf(4042,gps),wire.sbf(4004,glo),wire.sbf(4042,updated),
                wire.sbf(4042|(1<<13),gps)]
        for mode in ['decoder','driver']:
            m=wire.run('nav_fields_'+mode,'sbf_'+mode+'_node',frames,
                       ['-p','topics.ephemeris:=/review/nav'])
            gps_records=[e for batch in m['nav'] for e in batch.gnss_ephemeris if e.system=='G']
            glo_records=[g for batch in m['nav'] for g in batch.glonass_ephemeris]
            self.assertFalse(gps_records)  # CNAV has no model discriminator in ROS.
            self.assertTrue(glo_records)
            self.assertEqual(glo_records[-1].sva,-1)
            self.assertEqual(glo_records[-1].frq,-7)
            self.assertEqual(glo_records[-1].flags,0)

    def test_beidou_cnav_is_rejected_without_blocking_observations(self):
        frames=[]
        for ident in [4251,4252,4253]:
            data=bytearray(144 if ident==4253 else 152)
            struct.pack_into('<IHBBI',data,0,100000,2300,40,3,300)
            struct.pack_into('<d',data,12,26560000.)
            struct.pack_into('<d',data,44,.01)
            struct.pack_into('<I',data,108,300)
            frames.append(wire.sbf(ident,data))
        # A supported observation immediately following rejected nav still publishes.
        frames += [measurement(),marker()]
        for mode in ['decoder','driver']:
            m=wire.run('bds_models_'+mode,'sbf_'+mode+'_node',frames,
                       PARAMS+['-p','topics.ephemeris:=/review/nav'])
            self.assertFalse(any(batch.gnss_ephemeris for batch in m['nav']))
            self.assertEqual([e.tow for e in m['obs']],[100.])

    def test_rawx_flagged_missing_and_valid_zero_both_paths(self):
        frames=[]
        for index,flags in enumerate([7,6,5,4,15]):
            header=struct.pack('<dHbBBB2s',100.+index,2300,18,1,1,1,b'\0\0')
            row=struct.pack('<ddf4BH6B',21000000.125,-110000000.,0.,0,1,0,0,10000,45,0,0,0,flags,0)
            frames.append(wire.ubx(2,0x15,header+row))
        for mode in ['decoder','driver']:
            m=wire.run('flags_'+mode,'ubx_'+mode+'_node',frames,PARAMS)
            self.assertEqual(len(m['obs']),5)
            for epoch,flags in zip(m['obs'],[7,6,5,4,15]):
                obs=epoch.observations[0]
                self.assertEqual(math.isnan(obs.p),not bool(flags&1))
                self.assertEqual(math.isnan(obs.l),not bool(flags&2))
                self.assertEqual((obs.d,obs.snr),(0.,45.))
                self.assertEqual(obs.lli & 0x40,0x40 if flags & 8 else 0)
                if flags&1:self.assertEqual(obs.p,21000000.125)
                if flags&2:self.assertEqual(obs.l,-110000000.)

    def test_range_code_only_tracking_both_paths(self):
        frames=[]
        for index,flags in enumerate([(1<<12),0,(1<<12)|(1<<10)]):
            row=struct.pack('<HHdfdffffI',1,0,21000000.125,.25,-110000000.,.125,-1234.,45.,10.,flags|(1<<11))
            frames.append(wire.oem(43,struct.pack('<I',1)+row,tow=100000+1000*index))
        for mode in ['decoder','driver']:
            m=wire.run('range_locks_'+mode,'novatel_'+mode+'_node',frames,PARAMS)
            self.assertEqual(len(m['obs']),3)
            first,missing,full=[e.observations[0] for e in m['obs']]
            self.assertEqual((first.p,first.d),(21000000.125,-1234.))
            self.assertTrue(math.isnan(first.l))
            self.assertTrue(all(math.isnan(v) for v in [missing.p,missing.l,missing.d]))
            self.assertEqual((full.p,full.l,full.d),(21000000.125,110000000.,-1234.))

    def test_sbf_final_epoch_idle_disconnect_and_late_marker(self):
        for mode in ['decoder','driver']:
            for disconnect in [False,True]:
                m=wire.run('final_'+mode+str(disconnect),'sbf_'+mode+'_node',measurement(missing=True),PARAMS,
                           disconnect=disconnect,chunk_delay=0)
                self.assertEqual(len(m['obs']),1)
                epoch=m['obs'][0];self.assertEqual((epoch.week,epoch.tow),(2300,100.))
                self.assertEqual(epoch.header.stamp.sec,1707004882)
                obs=epoch.observations[0];self.assertAlmostEqual(obs.p,21000000.125,places=6)
                self.assertTrue(all(math.isnan(v) for v in [obs.l,obs.d,obs.snr]))
            m=wire.run('late_marker_'+mode,'sbf_'+mode+'_node',
                       [measurement(),marker(),measurement(101000),marker(101000)],PARAMS)
            self.assertEqual([e.tow for e in m['obs']],[100.,101.])

    def test_sbf_file_eof_and_truncated_tail(self):
        for tail in [b'',measurement(101000)[:17]]:
            m=wire.run('eof_'+str(len(tail)),'sbf_decoder_node',measurement()+tail,
                       PARAMS+[],file_input=True)
            self.assertIn('published_epochs=1',m['log'])

    def test_sbf_boundary_reverse_time_and_fragmented_frame(self):
        for mode in ['decoder','driver']:
            # Gap below assembly timeout: explicit next-epoch boundary flushes,
            # including a backwards replay; the last epoch then drains on idle.
            data=measurement(101000)+measurement(100000)
            m=wire.run('backwards_'+mode,'sbf_'+mode+'_node',[data[:13],data[13:]],PARAMS,chunk_delay=.02)
            self.assertEqual([e.tow for e in m['obs']],[101.,100.])

if __name__=='__main__':
    unittest.main(argv=[wire.sys.argv[0]]+wire.unittest_args,verbosity=2)
