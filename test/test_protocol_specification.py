#!/usr/bin/env python3
"""Independent byte-layout cases from pinned UBX, SBF and OEM references.

U1: F9 HPG1.50 UBXDOC-963802114-12815 R01, 3.17.6 pp195-197,
    1.5.4 table4 p20 (v1 fields and signal IDs).
U2: M8 UBX-13003221 R28, 32.18.4.1 pp409-412 (protocol17 DATA0).

Fixture bytes/expected measurements below use these field tables, not a backend
encoder or decoded output. Only the ROS process/transport harness is shared.
UBX scope includes the table4 GPS/Galileo/QZSS/BeiDou data/pilot signals within the
six-slot contract and GLONASS L1 OF boundaries; not every firmware/RAWX signal.
LLI is checked against this repository's documented RTKLIB bit mapping.
"""
import math
import struct
import unittest
import test_receiver_streams as wire

PARAMS=['-p','topics.observation:=/review/obs','-p','use_gps_timestamp:=true']

# U1 table4 UBX IDs -> RINEX3.04 observation codes (F1 A2).
SIGNALS=[('G01',0,1,3,'2L'),('G01',0,1,4,'2S'),('G01',0,1,6,'5I'),('G01',0,1,7,'5Q')]
SIGNALS += [('E01',2,1,s,c) for s,c in [(0,'1C'),(1,'1B'),(3,'5I'),(4,'5Q'),(5,'7I'),(6,'7Q')]]
SIGNALS += [('J01',5,1,s,c) for s,c in [(0,'1C'),(1,'1Z'),(4,'2S'),(5,'2L'),(8,'5I'),(9,'5Q')]]
SIGNALS += [('C20',3,20,s,c) for s,c in [(0,'2I'),(2,'7I'),(5,'1P'),(6,'1D'),(7,'5P'),(8,'5D')]]
EXPECTED={(sid,code):(21000001.+i*.125,987654.5+i*.25,123.25+i*.5)
          for i,(sid,_,_,_,code) in enumerate(SIGNALS)}


def frame(payload):
    # UBX frame checksum covers class, ID, little-endian length and payload.
    data=b'\x02\x15'+len(payload).to_bytes(2,'little')+payload
    first=second=0
    for value in data:
        first=(first+value)%256;second=(second+first)%256
    return b'\xb5\x62'+data+bytes([first,second])


def row(prn,*,gnss=0,signal=0,channel=0,p=21000000.125,l=-12345.25,
        d=-123.5,cn0=45,status=7,phase_std=0):
    # Offsets relative to the 32-byte measurement record: R8/R8/R4 at 0/8/16;
    # gnss/sv/sig/freq at20..23, locktime at24, quality/status at26..31.
    return struct.pack('<ddfBBBBHBBBBBB',p,l,d,gnss,prn,signal,channel,1000,
                       cn0,0,phase_std,0,status,0)


def payload(version=1,*,week=2300,tow=100.5,reverse=False):
    rows=[row(1),row(2,p=0.,d=0.,cn0=0),row(3,phase_std=15),
          row(4,status=6),row(5,status=5),row(6,status=3),row(7,status=15),
          row(1,gnss=6,channel=0),row(2,gnss=6,channel=13)]
    if version==1:
        rows += [row(prn,gnss=gnss,signal=signal,p=EXPECTED[sid,code][0],
                     l=EXPECTED[sid,code][1],d=EXPECTED[sid,code][2])
                 for sid,gnss,prn,signal,code in SIGNALS]
    if reverse:rows.reverse()
    header=struct.pack('<dHbBBB2x',tow,week,18,len(rows),1,version)
    return header+b''.join(rows)


class UbxSpecification(unittest.TestCase):
    def test_explicit_legacy_layouts_ignore_reserved_bytes(self):
        for layout,version in [('data0',255),('data1_pre27',1)]:
            p=bytearray(payload(0));p[13]=version
            for offset in range(16,len(p),32):p[offset+22]=253
            for mode in ('driver','decoder'):
                result=wire.run('rawx_'+layout+'_'+mode,'ubx_'+mode+'_node',frame(p),
                                PARAMS+['-p','rawx_layout:='+layout])
                self.assertEqual(len(result['obs']),1)
                self.check_epoch(result['obs'][0],0)

    def check_epoch(self,epoch,version,week=2300,tow=100.5,first=True):
        self.assertEqual((epoch.week,epoch.tow),(week,tow))
        stamp=epoch.header.stamp.sec+epoch.header.stamp.nanosec*1e-9
        # GPS epoch Unix seconds + full GPS weeks/TOW - 18s for these 2024 dates.
        self.assertAlmostEqual(stamp,315964800+week*604800+tow-18,places=6)
        observations={(o.satid,o.code_str):o for o in epoch.observations}
        expected={(f'G{i:02d}','1C') for i in range(1,8)}|{('R01','1C'),('R02','1C')}
        if version==1:expected.update(EXPECTED)
        self.assertEqual(set(observations),expected)
        self.assertEqual(len(epoch.observations),len(expected),'no duplicate signals')
        for key,o in observations.items():
            with self.subTest(signal=key):
                sid,code=key
                self.assertEqual(o.system,sid[0]);self.assertEqual(o.prn,193 if sid=='J01' else int(sid[1:]))
                self.assertEqual(o.freq,{'R01':0,'R02':13}.get(sid,255))
                if key in EXPECTED:
                    self.assertEqual((o.p,o.l,o.d),EXPECTED[key]);self.assertEqual(o.snr,45.)
                    continue
                if sid=='G04':self.assertTrue(math.isnan(o.p))
                else:self.assertEqual(o.p,0. if sid=='G02' else 21000000.125)
                if sid in ('G03','G05'):self.assertTrue(math.isnan(o.l))
                else:self.assertEqual(o.l,-12345.25)
                self.assertEqual(o.d,0. if sid=='G02' else -123.5)
                self.assertEqual(o.snr,0. if sid=='G02' else 45.)
                self.assertEqual(o.lli & 2,2 if sid=='G06' else 0)
                if sid=='G07':self.assertEqual(o.lli,65 if first else 64) # adjusted + transition slip

    def test_data0_protocol17_and_data1_layout(self):
        # Independent fixed-byte anchors for time, week and signed Doppler.
        self.assertEqual(payload()[:10],bytes.fromhex('0000000000205940fc08'))
        self.assertEqual(row(1)[16:20],bytes.fromhex('0000f7c2'))
        for version in (0,1):
            for mode in ('driver','decoder'):
                with self.subTest(version=version,node=mode):
                    result=wire.run(f'spec_rawx_{version}_{mode}','ubx_'+mode+'_node',frame(payload(version)),PARAMS)
                    self.assertEqual(len(result['obs']),1)
                    self.check_epoch(result['obs'][0],version)

    def test_rejection_recovery_and_week_boundary(self):
        unsupported=bytearray(payload());unsupported[13]=2
        wrong_count=bytearray(payload());wrong_count[11]+=1
        bad_crc=bytearray(frame(payload()));bad_crc[-1]^=1
        invalid=[frame(unsupported),frame(wrong_count),bytes(bad_crc),
                 frame(payload(tow=float('nan'))),frame(payload(tow=604800.)),frame(payload(week=0))]
        valid=[frame(payload()),frame(payload(tow=604799.75)),frame(payload(week=2301,tow=.25))]
        for mode in ('driver','decoder'):
            with self.subTest(node=mode):
                result=wire.run('spec_rawx_recover_'+mode,'ubx_'+mode+'_node',
                                [b''.join(invalid),*valid],PARAMS,chunk_delay=.1)
                self.assertEqual([(m.week,m.tow) for m in result['obs']],
                                 [(2300,100.5),(2300,604799.75),(2301,.25)])
                # Half-subtract slip is a transition, so verify full fields on
                # the first accepted epoch; subsequent epochs verify time/sets.
                self.check_epoch(result['obs'][0],1)
                self.check_epoch(result['obs'][1],1,tow=604799.75,first=False)
                self.check_epoch(result['obs'][2],1,week=2301,tow=.25,first=False)

    def test_same_band_signals_survive_reordering_and_reconnect(self):
        for mode in ('driver','decoder'):
            with self.subTest(node=mode):
                result=wire.run('spec_rawx_signals_'+mode,'ubx_'+mode+'_node',
                    [frame(payload()),frame(payload(tow=100.75,reverse=True)),None,
                     frame(payload(tow=101.,reverse=True))],PARAMS,chunk_delay=.1)
                self.assertEqual(len(result['obs']),3)
                self.check_epoch(result['obs'][0],1)
                self.check_epoch(result['obs'][1],1,tow=100.75,first=False)
                self.check_epoch(result['obs'][2],1,tow=101.,first=True)



# Independent S1 §4.2.1 pp263-267, §4.1.9/10: 4027 rev0/1, 5922.
# Slave signed offsets intentionally negative; master contains a fractional mm.
def sbf_spec(block,payload,revision=0):
    payload+=b'\0'*(-len(payload)%4)
    data=struct.pack('<HH',block|(revision<<13),len(payload)+8)+payload
    crc=0
    for value in data:
        crc^=value<<8
        for _ in range(8):crc=((crc<<1)^(0x1021 if crc&0x8000 else 0))&65535
    return b'$@'+struct.pack('<H',crc)+data


def meas_spec(tow=100500,revision=1,missing=False):
    pseudorange_mm=21000000125
    # Carrier offset -12.345 cycles. Doppler -123.5 Hz. C/N0 45 dBHz.
    master=struct.pack('<BBBBI i H b B H BB',0,0,1,pseudorange_mm>>32,
        pseudorange_mm&0xffffffff,-2147483648 if missing else -1235000,
        0 if missing else 53191,-128 if missing else -1,255 if missing else 140,
        65535 if missing else 10,4,1)
    # GPS L2P: PR delta -1.25m, carrier offset -2.5cycles,
    # Doppler delta -0.5Hz. 3-bit/5-bit signed MSBs both -1.
    slave=struct.pack('<BBBBbBHHH',2,10,180,255,-1,0,64286,63036,60536)
    header=struct.pack('<IH6B',tow,2300,1,20,12,0,0,0)
    return sbf_spec(4027,header+master+slave,revision)


# N1 OEM7 v6A pp39-42 header, pp830-835 RANGE and tracking flags.
# N3 OEM6 Rev12 RANGE uses the same FCN+7 convention (not nav.glo_fcn FCN+8).
def oem_spec(rows,*,tow=100500,week=2300,status=160,mid=43):
    payload=struct.pack('<I',len(rows))+b''.join(rows)
    header=bytearray.fromhex('aa44121c')+bytearray(24)
    struct.pack_into('<H',header,4,mid);struct.pack_into('<H',header,8,len(payload))
    header[13]=status;struct.pack_into('<HI',header,14,week,tow)
    data=bytes(header)+payload;crc=0
    for value in data:
        crc^=value
        for _ in range(8):crc=(crc>>1)^(0xedb88320 if crc&1 else 0)
    return data+struct.pack('<I',crc)


def range_row(prn=1,channel=0,flags=0x1c00,lock=10.,p=21000000.125,d=-123.5):
    return struct.pack('<HHdfdffffI',prn,channel,p,.25,-110000000.25,.125,d,45.,lock,flags)


class SbfSpecification(unittest.TestCase):
    def test_compatible_subblock_extensions_preserve_observations(self):
        frames=[]
        for rev in (0,1,7):
            tow=100500+rev*1000
            p=bytearray(meas_spec(tow)[8:]);p[7]=24;p[8]=16;p[11]=0xa5
            p=p[:12]+p[12:32]+bytes([0xa5])*4+p[32:44]+bytes([0x5a])*4
            frames.extend([sbf_spec(4027,p,rev),sbf_spec(5922,struct.pack('<IH2x',tow,2300))])
        for mode in ('driver','decoder'):
            result=wire.run('sbf_extension_'+mode,'sbf_'+mode+'_node',frames,PARAMS,chunk_delay=.05)
            self.assertEqual([e.tow for e in result['obs']],[100.5,101.5,107.5])
            for epoch in result['obs']:
                rows={o.code_str:o for o in epoch.observations}
                self.assertEqual(set(rows),{'1C','2W'})
                self.assertAlmostEqual(rows['1C'].p,21000000.125,places=6)
                self.assertAlmostEqual(rows['2W'].p,20999998.875,places=6)
                self.assertEqual(rows['1C'].d,-123.5)

    def test_measepoch_master_slave_revisions_and_missing(self):
        for mode in ('driver','decoder'):
            with self.subTest(node=mode):
                frames=[]
                for rev,tow,missing in [(0,100500,False),(1,101500,False),(1,102500,True)]:
                    frames += [meas_spec(tow,rev,missing),sbf_spec(5922,struct.pack('<IH2x',tow,2300))]
                result=wire.run('spec_meas_'+mode,'sbf_'+mode+'_node',frames,PARAMS,chunk_delay=.05)
                self.assertEqual([m.tow for m in result['obs']],[100.5,101.5,102.5])
                for i,epoch in enumerate(result['obs']):
                    signals={o.code_str:o for o in epoch.observations}
                    self.assertEqual(set(signals),{'1C','2W'})
                    for code,o in signals.items():
                        self.assertEqual((o.satid,o.freq),('G01',255))
                    main=signals['1C'];slave=signals['2W']
                    self.assertAlmostEqual(main.p,21000000.125,places=6)
                    self.assertAlmostEqual(slave.p,20999998.875,places=6)
                    if i<2:
                        self.assertAlmostEqual(main.l,21000000.125*1575420000/299792458-12.345,places=6)
                        self.assertEqual((main.d,main.snr),( -123.5,45.))
                    else:
                        self.assertTrue(math.isnan(main.l));self.assertTrue(math.isnan(main.d));self.assertTrue(math.isnan(main.snr))
                    self.assertEqual(main.lli&2,2)
                    self.assertAlmostEqual(slave.l,20999998.875*1227600000/299792458-2.5,places=6)
                    if i<2:self.assertAlmostEqual(slave.d,-123.5*120/154-.5,places=5)
                    else:self.assertTrue(math.isnan(slave.d))
                    self.assertEqual(slave.snr,45.)


class OemSpecification(unittest.TestCase):
    def test_range_glonass_frequency_boundaries(self):
        for mode in ('driver','decoder'):
            with self.subTest(node=mode):
                frames=[oem_spec([range_row(38,c,0x11c00)],tow=100500+1000*i)
                        for i,c in enumerate([0,7,13,14,65535])]
                result=wire.run('spec_range_fcn_'+mode,'novatel_'+mode+'_node',frames,PARAMS,chunk_delay=.05)
                self.assertEqual(len(result['obs']),5)
                for i,(epoch,freq) in enumerate(zip(result['obs'],[0,7,13,255,255])):
                    self.assertEqual((epoch.week,epoch.tow),(2300,100.5+i))
                    self.assertEqual(len(epoch.observations),1)
                    o=epoch.observations[0]
                    self.assertEqual((o.satid,o.code_str,o.freq),('R01','1C',freq))
                    self.assertEqual((o.p,o.l,o.d,o.snr),(21000000.125,110000000.25,-123.5,45.))

    def test_range_locks_halfcycle_slip_time_and_recovery(self):
        for mode in ('driver','decoder'):
            with self.subTest(node=mode):
                frames=[oem_spec([range_row(flags=0x10001c00)]),
                        oem_spec([range_row(flags=0x1400,lock=0.,p=0.,d=0.)],tow=101500),
                        oem_spec([range_row(flags=0x1800)],tow=102500),
                        oem_spec([range_row(flags=0x800)],tow=103500),
                        oem_spec([range_row()],status=20,tow=104500),
                        oem_spec([range_row()],tow=105500)]
                result=wire.run('spec_range_locks_'+mode,'novatel_'+mode+'_node',frames,PARAMS,chunk_delay=.05)
                self.assertEqual([m.tow for m in result['obs']],[100.5,101.5,102.5,103.5,105.5])
                a,b,c,d,e=[m.observations[0] for m in result['obs']]
                self.assertEqual(a.lli&0x40,0x40)
                self.assertEqual(b.lli&3,3);self.assertEqual((b.p,b.d),(0.,0.))
                self.assertTrue(math.isnan(c.l));self.assertEqual(c.p,21000000.125)
                self.assertTrue(all(math.isnan(x) for x in (d.p,d.l,d.d)))
                self.assertEqual((e.p,e.l,e.d),(21000000.125,110000000.25,-123.5))



# N1 table186 pp839-841 (N3 table at p593): little-endian bit-packed record.
def rangecmp_row(prn=1,channel=0,system=0,flags=0x1c00,d=-123.5,cn0=45):
    word=flags|(system<<16)
    word|=(int(d*256)&((1<<28)-1))<<32
    word|=int(21200000.125*128)<<60
    word|=(int(-12345.25*256)&0xffffffff)<<96
    word|=prn<<136
    word|=320<<144  # ten seconds lock time
    word|=(cn0-20)<<165
    word|=channel<<170
    return word.to_bytes(24,'little')


class OemCompressedSpecification(unittest.TestCase):
    def test_rangecmp_own_frequency_and_scaled_fields_without_navigation(self):
        for mode in ('driver','decoder'):
            with self.subTest(node=mode):
                frames=[]
                for i,channel in enumerate([0,7,13,14,63]):
                    rows=[rangecmp_row(d=0.,cn0=20),rangecmp_row(38,channel,1,cn0=51)]
                    frames.append(oem_spec(rows,mid=140,tow=100500+i*1000))
                result=wire.run('spec_rangecmp_'+mode,'novatel_'+mode+'_node',frames,PARAMS,chunk_delay=.05)
                self.assertEqual(len(result['obs']),5)
                for i,(epoch,channel) in enumerate(zip(result['obs'],[0,7,13,14,63])):
                    self.assertEqual((epoch.week,epoch.tow),(2300,100.5+i))
                    by_id={o.satid:o for o in epoch.observations}
                    self.assertEqual(set(by_id),{'G01','R01'})
                    for sid,o in by_id.items():
                        self.assertEqual((o.code_str,o.p),('1C',21200000.125))
                        self.assertEqual((o.d,o.snr),(0.,20.) if sid=='G01' else (-123.5,51.))
                        self.assertEqual(o.freq,channel if sid=='R01' and channel<=13 else 255)
                        if sid=='R01' and channel>13:self.assertTrue(math.isnan(o.l));continue
                        frequency=1575420000 if sid=='G01' else 1602000000+(channel-7)*562500
                        # N1 rounded ambiguity integer; L is negative of OEM ADR.
                        wraps=math.trunc((21200000.125*frequency/299792458-12345.25)/8388608+.5)
                        self.assertEqual(o.l,wraps*8388608+12345.25)
                        self.assertEqual(o.lli&2,0)


if __name__=='__main__':unittest.main(argv=[__file__]+wire.unittest_args)
