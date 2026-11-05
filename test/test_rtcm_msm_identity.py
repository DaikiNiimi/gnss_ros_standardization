#!/usr/bin/env python3
"""R2 mask/identity cases plus explicit six-slot backend capacity policy.

Wire layout and signal codes use pinned R2. Satellite acceptance limits and
overflow selection are repository policy, not a normative RTCM claim.
"""
import unittest
import test_receiver_streams as wire
from test_decoder_boundaries import RTCM_PARAMS
from test_rtcm_reference import SIGNALS
import test_rtcm_msm_states as layout

grid = layout.grid


class MsmIdentity(unittest.TestCase):
    def check(self, msg, variant, family, base, fixture, kept):
        encoded, satellites, rows = fixture
        filtered = {key: value for key, value in rows.items() if key in kept}
        layout.MsmStates.check_epoch(self, msg, variant, family, base, msg.tow,
                              (encoded, satellites, filtered))

    def test_satellite_limit_unknown_signal_and_recovery(self):
        families = [('gps',1070,32), ('glo',1080,27), ('gal',1090,36),
                    ('sbas',1100,39), ('qzss',1110,10), ('bds',1120,63), ('navic',1130,14)]
        for variant in (5,7,4,6):
            fixtures=[]; expected=[]
            for family,base,limit in families:
                known=next(iter(SIGNALS[family]))
                unknown=next(i for i in range(1,33) if i not in SIGNALS[family])
                # Unknown signal and out-of-range satellite surround a valid
                # cell; their data must not shift the valid cell's array index.
                for sats,sigs in [((1,limit,limit+1),(unknown,known)),
                                   ((1,),(unknown,)), ((1,),(known,))]:
                    ms=100000+len(fixtures)*1000
                    if family=='glo' and variant in (4,6):
                        seed=grid(5,sats,(known,),tow_ms=ms,base=base)
                        fixtures.append(seed)
                        expected.append((5,family,base,{key for key in seed[2] if key[0]<=limit}))
                        ms+=1000
                    f=grid(variant,sats,sigs,tow_ms=ms,base=base)
                    fixtures.append(f)
                    expected.append((variant,family,base,{key for key in f[2] if key[0]<=limit and key[1]==known}))
            result=wire.run(f'msm_identity_{variant}','rtcm_decoder_node',
                            [f[0] for f in fixtures],RTCM_PARAMS,chunk_delay=.03)
            # R2/A2: no accepted observations means no observation publication.
            # Unknown-only frames previously produced an empty message. Keep
            # their original timestamps in the input and verify the following
            # supported frame recovers without a relabelled/empty output.
            accepted=[(i,args,f) for i,(args,f) in enumerate(zip(expected,fixtures)) if args[3]]
            self.assertEqual(len(result['obs']),len(accepted))
            for msg,(i,args,f) in zip(result['obs'],accepted):
                with self.subTest(variant=variant,index=i):
                    self.assertEqual(msg.tow,100.+i)
                    self.check(msg,*args[:3],f,args[3])

    def test_six_slots_same_band_and_overflow_policy(self):
        # Default GPS priorities are L1 C>P>W, L2 C>P>W, L5 I>Q.
        # NFREQ=3/NEXOBS=3: all six below fit; in seven-cell input the
        # final extended L5Q is omitted. Never label capacity as lossless.
        for variant in (4,5,6,7):
            fixtures=[grid(variant,(1,),signals,tow_ms=100000+i*1000)
                      for i,signals in enumerate([(2,3,8,9,22,23),
                                                  (2,3,4,8,9,22,23),
                                                  (2,3,8,9,22,23), (23,)])]
            result=wire.run(f'msm_slots_{variant}','rtcm_decoder_node',
                            [f[0] for f in fixtures],RTCM_PARAMS,chunk_delay=.03)
            self.assertEqual(len(result['obs']),4)
            for i,(msg,f) in enumerate(zip(result['obs'],fixtures)):
                with self.subTest(variant=variant,index=i):
                    kept=set(f[2]) - ({(1,23)} if i==1 else set())
                    self.assertEqual(msg.tow,100.+i)
                    self.check(msg,variant,'gps',1070,f,kept)


if __name__=='__main__':unittest.main(argv=[__file__]+wire.unittest_args)
