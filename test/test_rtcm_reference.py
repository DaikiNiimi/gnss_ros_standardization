#!/usr/bin/env python3
"""Independent-layout cases: rtcm-rs v0.11.0 definitions -> real ROS decoder.

Source: d4d0269dc76b7e88b6620ca023b82889b6441662, legacy 1002/1004/1010/1012,
seven-constellation MSM4..7, msm46_sat.rs/msm57_sat.rs/msm_mappings.rs
and src/df/dfs.rs. Covers all 73 defined signal mappings as single cells.
Values below are chosen independently of RTKLIB; neither its encoder nor its
output is used as the expected value. This is NOT normative RTCM conformance
or a run of the Rust encoder/decoder. No Rust/runtime/network dependency.
"""
import math
import unittest
import test_receiver_streams as wire
from test_decoder_boundaries import rtcm_payload,RTCM_PARAMS


def msm(variant,tow_ms=100000,missing=False,zero=False,rough_missing=False,
        base=1070,signal=2,satellite=1,channel=0):
    fields=[]
    def put(value,width):
        assert -(1<<(width-1))<=value<(1<<width)
        fields.append(format(value&((1<<width)-1),f'0{width}b'))
    put(base+variant,12);put(42,12)
    epoch=tow_ms+10782000 if base==1080 else tow_ms-14000 if base==1120 else tow_ms
    put(epoch,30)
    put(0,1);put(1,3);put(0,7);put(0,2);put(0,2);put(0,1);put(0,3)
    put(1<<(64-satellite),64);put(1<<(32-signal),32);put(1,1)
    put(255 if rough_missing else 0 if zero else 70,8)
    if variant in (5,7):put(channel+7 if base==1080 else 0,4)
    put(0 if zero else 512,10)
    if variant in (5,7):put(-8192 if rough_missing else 0 if zero else -100,14)
    extended=variant in (6,7)
    pr_bits=20 if extended else 15
    cp_bits=24 if extended else 22
    put(-(1<<(pr_bits-1)) if missing else 0 if zero else -512 if extended else -16,pr_bits)
    put(-(1<<(cp_bits-1)) if missing else 0 if zero else 4096 if extended else 1024,cp_bits)
    put(10,10 if extended else 4);put(1,1)
    put(0 if zero else 721 if extended else 45,10 if extended else 6)
    if variant in (5,7):put(-16384 if missing else 0 if zero else -1234,15)
    bits=''.join(fields);bits+='0'*((-len(bits))%8)
    return rtcm_payload(int(bits,2).to_bytes(len(bits)//8,'big'))


SIGNALS = {'gps': {2: '1C', 3: '1P', 4: '1W', 8: '2C', 9: '2P', 10: '2W', 15: '2S', 16: '2L', 17: '2X', 22: '5I', 23: '5Q', 24: '5X', 30: '1S', 31: '1L', 32: '1X'}, 'glo': {2: '1C', 3: '1P', 8: '2C', 9: '2P'}, 'gal': {2: '1C', 3: '1A', 4: '1B', 5: '1X', 6: '1Z', 8: '6C', 9: '6A', 10: '6B', 11: '6X', 12: '6Z', 14: '7I', 15: '7Q', 16: '7X', 18: '8I', 19: '8Q', 20: '8X', 22: '5I', 23: '5Q', 24: '5X'}, 'sbas': {2: '1C', 22: '5I', 23: '5Q', 24: '5X'}, 'qzss': {2: '1C', 9: '6S', 10: '6L', 11: '6X', 15: '2S', 16: '2L', 17: '2X', 22: '5I', 23: '5Q', 24: '5X', 30: '1S', 31: '1L', 32: '1X'}, 'bds': {2: '2I', 3: '2Q', 4: '2X', 8: '6I', 9: '6Q', 10: '6X', 14: '7I', 15: '7Q', 16: '7X', 22: '5D', 23: '5P', 24: '5X', 25: '7D', 30: '1D', 31: '1P', 32: '1X'}, 'navic': {8: '9A', 22: '5A'}}


def radio_frequency(system,code,channel=0):
    band=code[0]
    if system=='R':return (1602.+channel*.5625 if band=='1' else 1246.+channel*.4375)*1e6
    if system=='C' and band in ('2','6'):return {'2':1561.098,'6':1268.52}[band]*1e6
    return {'1':1575.42,'2':1227.60,'5':1176.45,'6':1278.75,
            '7':1207.14,'8':1191.795,'9':2492.028}[band]*1e6


def legacy(message,tow_ms=100000,mode='normal',channel=0):
    """R2 msg1002/1004/1010/1012; DF009..020 and DF034..050."""
    glo=message in (1010,1012);dual=message in (1004,1012);fields=[]
    def put(v,n):fields.append(format(v&((1<<n)-1),f'0{n}b'))
    put(message,12);put(42,12);put(tow_ms+10782000 if glo else tow_ms,27 if glo else 30)
    put(0,1);put(1,5);put(0,1);put(0,3)
    put(1,6);put(0,1)
    if glo:put(channel+7,5)
    width=25 if glo else 24
    put((1<<width)-1 if mode=='missing_range' else 0 if mode=='zero' else 5000000,width)
    put(-524288 if mode=='missing_phase' else 0 if mode=='zero' else -2000,20)
    put(10,7);put(0 if mode=='zero' else 35 if glo else 70,7 if glo else 8)
    put(0 if mode=='zero' else 181,8)
    if dual:
        put(0,2);put(-8192 if mode=='missing_phase' else 0 if mode=='zero' else -100,14)
        put(-524288 if mode=='missing_phase' else 0 if mode=='zero' else 4000,20)
        put(10,7);put(0 if mode=='zero' else 182,8)
    bits=''.join(fields);bits+='0'*((-len(bits))%8)
    return rtcm_payload(int(bits,2).to_bytes(len(bits)//8,'big'))


class RtcmIndependentLayout(unittest.TestCase):
    def test_all_reference_signal_mappings_msm4_to_7(self):
        families=[('gps',1070,'G01'),('glo',1080,'R01'),('gal',1090,'E01'),
                  ('sbas',1100,'120'),('qzss',1110,'J01'),('bds',1120,'C01'),('navic',1130,'I01')]
        for family,base,satid in families:
            with self.subTest(family=family):
                cases=[(v,sig,code) for v in (5,7,4,6) for sig,code in SIGNALS[family].items()]
                frames=[msm(v,100000+i*1000,base=base,signal=sig,channel=-7)
                        for i,(v,sig,_) in enumerate(cases)]
                result=wire.run('rtcm_signals_'+family,'rtcm_decoder_node',frames,RTCM_PARAMS,chunk_delay=.025)
                self.assertEqual(len(result['obs']),len(cases))
                for i,(msg,(v,sig,code)) in enumerate(zip(result['obs'],cases)):
                    with self.subTest(family=family,variant=v,signal=sig):
                        self.assertEqual((msg.week,msg.tow),(2300,100.+i))
                        self.assertEqual(len(msg.observations),1)
                        o=msg.observations[0];f=radio_frequency('S' if family=='sbas' else satid[0],code,-7)
                        self.assertEqual((o.satid,o.code_str,o.freq),(satid,code,0 if family=='glo' else 255))
                        self.assertAlmostEqual(o.p,(70.5-2**-20)*299792.458,places=7)
                        self.assertAlmostEqual(o.l,(70.5+2**-19)*f*.001,places=6)
                        if v in (5,7):self.assertAlmostEqual(o.d,100.1234*f/299792458.,places=3)
                        else:self.assertTrue(math.isnan(o.d))
                        self.assertEqual(o.snr,45.0625 if v in (6,7) else 45.)
                        self.assertEqual(o.lli&2,2)

    def test_legacy_independent_component_missing_zero_recovery(self):
        modes=['normal','missing_phase','missing_range','zero','normal']
        for message in (1002,1004,1010,1012):
            with self.subTest(message=message):
                frames=[legacy(message,100000+i*1000,mode,channel=-7) for i,mode in enumerate(modes)]
                result=wire.run('rtcm_legacy_'+str(message),'rtcm_decoder_node',frames,RTCM_PARAMS,chunk_delay=.025)
                self.assertEqual(len(result['obs']),len(modes))
                glo=message>=1010;dual=message in (1004,1012)
                for i,(msg,mode) in enumerate(zip(result['obs'],modes)):
                    self.assertEqual((msg.week,msg.tow),(2300,100.+i))
                    obs={o.code_str:o for o in msg.observations}
                    self.assertEqual(set(obs),{'1C','2C' if glo else '2X'} if dual else {'1C'})
                    for code,o in obs.items():
                        self.assertEqual((o.satid,o.freq),('R01',0) if glo else ('G01',255))
                        f=radio_frequency('R' if glo else 'G',code,-7)
                        first=code=='1C';base=100000+70*299792.458
                        p=base if first else base-2
                        phase=base-1 if first else base+2
                        if mode=='missing_range' or (mode=='missing_phase' and not first):self.assertTrue(math.isnan(o.p),(message,mode,code,'P',o.p))
                        else:self.assertAlmostEqual(o.p,0 if mode=='zero' else p,places=7)
                        if mode in ('missing_range','missing_phase'):self.assertTrue(math.isnan(o.l),(message,mode,code,'L',o.l))
                        else:self.assertAlmostEqual(o.l,0 if mode=='zero' else phase*f/299792458.,places=6)
                        self.assertTrue(math.isnan(o.d),(message,mode,code,'D',o.d))
                        if mode=='zero':self.assertTrue(math.isnan(o.snr))
                        else:self.assertEqual(o.snr,45.25 if first else 45.5)

    def test_gps_msm4_to_7_signed_scale_missing_halfcycle_and_recovery(self):
        c=299792458.0;frequency=1575420000.0;wavelength=c/frequency
        for variant in (4,5,6,7):
            with self.subTest(variant=variant):
                result=wire.run(f'rtcm_reference_{variant}','rtcm_decoder_node',
                    [msm(variant),msm(variant,101000,True),msm(variant,102000,zero=True),
                     msm(variant,103000,rough_missing=True),msm(variant,104000)],RTCM_PARAMS)
                self.assertEqual(len(result['obs']),5)
                for i,msg in enumerate(result['obs']):
                    self.assertEqual((msg.week,msg.tow),(2300,100.+i))
                    self.assertEqual(len(msg.observations),1)
                    obs=msg.observations[0]
                    self.assertEqual((obs.satid,obs.code_str,obs.freq),('G01','1C',255))
                    self.assertEqual(obs.lli&2,2)
                    if i==2:
                        self.assertTrue(math.isnan(obs.snr)) # DF403/408: zero is the invalid sentinel
                        self.assertEqual((obs.p,obs.l),(0.,0.))
                        if variant in (5,7):self.assertEqual(obs.d,0.)
                        else:self.assertTrue(math.isnan(obs.d))
                        continue
                    self.assertEqual(obs.snr,45.0625 if variant in (6,7) else 45.)
                    if i in (1,3):
                        self.assertTrue(math.isnan(obs.p));self.assertTrue(math.isnan(obs.l))
                        self.assertTrue(math.isnan(obs.d))
                    else:
                        self.assertAlmostEqual(obs.p,(70.5-2**-20)*c*.001,places=7)
                        self.assertAlmostEqual(obs.l,(70.5+2**-19)*frequency*.001,places=6)
                        if variant in (5,7):self.assertAlmostEqual(obs.d,100.1234/wavelength,places=3)
                        else:self.assertTrue(math.isnan(obs.d))

if __name__=='__main__':unittest.main(argv=[__file__]+wire.unittest_args)
