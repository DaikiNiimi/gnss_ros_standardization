#!/usr/bin/env python3
"""U2 pp278-279 / U3 pp71-72 and S2 pp331-333 independent IMU cases.

Payload values and SI expectations follow the source tables, not production
offsets/scales. TCP/CRC helpers alone are shared with the node test harness.
"""
import math
import struct
import unittest
import test_receiver_streams as wire
import test_protocol_specification as spec


def esf(ms, version=1, validity=63):
    return wire.ubx(16, 21, struct.pack('<III6i', version | validity << 8, 0, ms,
                                      180000, -90000, 0, 125, -250, 0))


def sensor(kind, source=0, model=2, info=0, values=None):
    if values is None:
        values = (1.25, -2.5, 0.) if kind == 0 else (180., -90., 0.)
    return struct.pack('<BBBB3d', source, model, kind, info, *values)


def sbf(ms, rows, week=2300):
    return spec.sbf_spec(4050, struct.pack('<IHBB', ms, week, len(rows), 28) + b''.join(rows))


def raw_esf(tick, values, types=(16,17,18,14,13,5)):
    # U2 pp117-118/281-282, U3 p73: high EIGHT bits are dataType;
    # lower24 bits are signed data, same type/units as ESF-MEAS.
    return wire.ubx(16,3,bytes(4)+b''.join(struct.pack('<II',
        typ<<24 | (value & 0xffffff),tick) for typ,value in zip(types,values)))


class ImuSpecification(unittest.TestCase):
    def test_oem_rate_decrease_and_increase_recover(self):
        for mid,topic in ((1462,'raw'),(812,'imu')):
            def sample(t):
                if mid==1462:
                    return wire.oem(mid,struct.pack('<BBHdI6i',0,8,2300,t,0,0,0,16384,0,0,0),short=True)
                return wire.oem(mid,struct.pack('<I7d',2300,t,0.,0.,0.,1.,0.,0.),tow=round(t*1000))
            # New slower cadence must not remain blocked by historical minimum dt.
            times=[100.,100.125,100.375,100.625,100.75]
            frames=[wire.oem(43,struct.pack('<I',0))]+[sample(t) for t in times]
            for mode in ('decoder','driver'):
                r=wire.run(f'oem_rate_{mid}_{mode}','novatel_'+mode+'_node',frames,
                    ['-p','use_gps_timestamp:=true'],chunk_delay=.02)
                self.assertEqual(len(r[topic]),3)
                self.assertEqual([m.linear_acceleration.x for m in r[topic]],[8.,4.,8.])

    def test_oem_rawimusx_documented_scales_axes_and_quality(self):
        # N1 Tables271/302 pp1042-1044/1197-1198. Independent SI constants;
        # the synthetic 8Hz cadence tests conversion, not an IMU operating mode.
        scales={i:(2**-27*.3048,2**-33) for i in (5,11,20,27,28)}
        scales.update({12:(2**-26*.3048,2**-33),8:(2**-14,2**-19),26:(2e-8,1e-9)})
        scales.update({i:(2**-29,2**-33) for i in (58,68,69)})
        scales.update({i:(2**-22,2**-21*math.pi/180) for i in (32,56)})
        for i,rate in ((41,125),(62,200)):
            scales[i]=(.200/65536*9.80665/1000/rate,.008/65536*math.pi/180/rate)
        scales[61]=(.400/65536*9.80665/1000/200,.0151515/65536*math.pi/180/200)
        def sample(t,typ=8,info=0):
            return wire.oem(1462,struct.pack('<BBHdI6i',info,typ,2300,t,0,
                0,-2147483648,2147483647,-1,1024,-2048),short=True)
        frames=[wire.oem(43,struct.pack('<I',0)),sample(100.)]; expected=[]
        for n,(typ,scale) in enumerate(scales.items(),1):
            t=100+n*.125
            frames += [sample(t,typ,1),sample(t,typ,2),sample(t,255),sample(t,typ)]
            expected.append((t,scale))
        for mode in ('decoder','driver'):
            with self.subTest(mode=mode):
                result=wire.run('spec_oem_rawimu_'+mode,'novatel_'+mode+'_node',frames,
                    ['-p','use_gps_timestamp:=true'],chunk_delay=.01)
                self.assertEqual(len(result['raw']),len(expected))
                for m,(t,(acc,gyro)) in zip(result['raw'],expected):
                    for axis,value in zip('xyz',(2147483647,2147483648,0)):
                        self.assertAlmostEqual(getattr(m.linear_acceleration,axis),value*acc*8,delta=1e-7)
                    for axis,value in zip('xyz',(-2048,-1024,-1)):
                        self.assertAlmostEqual(getattr(m.angular_velocity,axis),value*gyro*8,delta=1e-12)
                    self.assertEqual(m.orientation_covariance[0],-1.)
                    self.assertAlmostEqual(m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                        315964800+2300*604800+t-18,delta=1e-6)

    def test_oem_corrimudata_increments_axes_nonfinite_and_recovery(self):
        # N1 pp1102-1103: increments in the configured output frame, x/y/z.
        # Body is 60 bytes (last double starts at52), despite CRC offset typo56.
        def sample(t,values=(.125,-.25,0.,1.25,-2.5,0.)):
            return wire.oem(812,struct.pack('<I7d',2300,t,*values))
        frames=[sample(100.),sample(100.125)]
        for j in range(6):
            v=[.125,-.25,0.,1.25,-2.5,0.];v[j]=float('nan')
            frames.append(sample(100.25,v))
        frames += [wire.oem(812,bytes(59)),sample(100.25)]
        for mode in ('decoder','driver'):
            with self.subTest(mode=mode):
                result=wire.run('spec_oem_corrimu_'+mode,'novatel_'+mode+'_node',frames,
                    ['-p','use_gps_timestamp:=true'],chunk_delay=.02)
                self.assertEqual(len(result['imu']),2)
                for m in result['imu']:
                    self.assertEqual([getattr(m.angular_velocity,a) for a in 'xyz'],[1.,-2.,0.])
                    self.assertEqual([getattr(m.linear_acceleration,a) for a in 'xyz'],[10.,-20.,0.])
                    self.assertEqual(m.orientation_covariance[0],-1.)

    def test_esf_raw_signed_limits_zero_and_fragmented_axes(self):
        cases=[(1280,-2560,0,180*4096,-90*4096,0),
               (-8388608,8388607,-1,8388607,-8388608,1),(0,)*6]
        frames=[raw_esf(10000,cases[0]),raw_esf(10001,cases[1])]
        frames += [raw_esf(10002,cases[2][:3],(16,17,18)),
                   raw_esf(10002,cases[2][3:],(14,13,5)),raw_esf(10002,cases[2])]
        for mode in ('decoder','driver'):
            with self.subTest(mode=mode):
                params=['-p','imu.sttag_tick_s:=0.001']
                if mode=='driver':params+=['-p','messages.esf_raw:=true']
                result=wire.run('spec_esf_raw_limits_'+mode,'ubx_'+mode+'_node',frames,params,chunk_delay=.05)
                self.assertEqual(len(result['raw']),3)
                stamps=[]
                for m,values in zip(result['raw'],cases):
                    for axis,value in zip('xyz',values[:3]):
                        self.assertEqual(getattr(m.linear_acceleration,axis),value/1024.)
                    for axis,value in zip('xyz',values[3:]):
                        self.assertAlmostEqual(getattr(m.angular_velocity,axis),value/4096.*math.pi/180.,delta=1e-12)
                    self.assertEqual(m.orientation_covariance[0],-1.)
                    stamps.append(m.header.stamp.sec*10**9+m.header.stamp.nanosec)
                self.assertTrue(all(a<b for a,b in zip(stamps,stamps[1:])))
                # Tick duration was explicitly configured: receiver/host alignment
                # is not an independent GNSS synchronization or default tick test.

    def test_esf_raw_full_type_byte_unknown_types_and_recovery(self):
        values=(1280,-2560,0,180*4096,-90*4096,0)
        frames=[raw_esf(10000+i,values,tuple(t+extra for t in (16,17,18,14,13,5)))
                for i,extra in enumerate((64,128,192))]
        frames += [wire.ubx(16,3,bytes(3)),raw_esf(10003,values)]
        for mode in ('decoder','driver'):
            with self.subTest(mode=mode):
                params=['-p','imu.sttag_tick_s:=0.001']
                if mode=='driver':params+=['-p','messages.esf_raw:=true']
                result=wire.run('spec_esf_raw_types_'+mode,'ubx_'+mode+'_node',frames,params,chunk_delay=.05)
                self.assertEqual(len(result['raw']),1)
                self.assertEqual(result['raw'][0].linear_acceleration.x,1.25)
                self.assertAlmostEqual(result['raw'][0].angular_velocity.x,math.pi,delta=1e-12)

    def check_si(self, msg, tow, week=2300):
        for key, expected in zip('xyz', (1.25, -2.5, 0.)):
            self.assertAlmostEqual(getattr(msg.linear_acceleration, key), expected, delta=1e-12)
        for key, expected in zip('xyz', (math.pi, -math.pi / 2, 0.)):
            self.assertAlmostEqual(getattr(msg.angular_velocity, key), expected, delta=1e-12)
        self.assertEqual(msg.orientation_covariance[0], -1.)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.assertAlmostEqual(stamp, 315964800 + week * 604800 + tow - 18, delta=1e-6)

    def test_esf_ins_documented_version_validity_and_recovery(self):
        anchor = spec.frame(struct.pack('<dHbBBB2x', 100., 2300, 18, 1, 1, 1) + spec.row(1))
        frames = [anchor + esf(100000), esf(101000, version=2)]
        frames += [esf(102000 + i * 1000, validity=63 ^ (1 << i)) for i in range(6)]
        frames += [esf(108000), esf(109000, version=0)]
        for mode in ('decoder', 'driver'):
            with self.subTest(mode=mode):
                args = ['-p', 'use_gps_timestamp:=true']
                if mode == 'driver': args += ['-p', 'messages.esf_ins:=true']
                result = wire.run('spec_esf_' + mode, 'ubx_' + mode + '_node', frames, args, chunk_delay=.1)
                self.assertEqual(len(result['imu']), 3)
                for msg, tow in zip(result['imu'], (100., 108., 109.)):
                    self.check_si(msg, tow)
                # Version 0 retains existing compatibility; only version 1 is
                # documented by U2/U3. Unknown version 2 must remain rejected.

    def test_sbf_units_and_quality_recovery(self):
        frames = [sbf(100000, [sensor(0), sensor(1)])]
        for i, info in enumerate((2, 4, 6)):
            frames += [sbf(101000 + i * 1000, [sensor(0, info=info), sensor(1)])]
        frames += [sbf(104000, [sensor(0, values=(-2e10, 0., 0.)), sensor(1)]),
                   sbf(105000, [sensor(0, info=1), sensor(1, info=1)])]
        for mode in ('decoder', 'driver'):
            with self.subTest(mode=mode):
                args = ['-p', 'use_gps_timestamp:=true']
                if mode == 'driver': args += ['-p', 'messages.ext_sensor_meas:=true']
                result = wire.run('spec_ext_quality_' + mode, 'sbf_' + mode + '_node', frames, args, chunk_delay=.1)
                self.assertEqual(len(result['raw']), 2)
                for msg, tow in zip(result['raw'], (100., 105.)):
                    self.check_si(msg, tow)

    def test_sbf_never_combines_different_sensor_or_week(self):
        frames = [sbf(100000, [sensor(0)]), sbf(100000, [sensor(1, source=1)]),
                  sbf(101000, [sensor(0)]), sbf(101000, [sensor(1, model=5)]),
                  sbf(102000, [sensor(0)]), sbf(102000, [sensor(1)], week=2301),
                  sbf(103000, [sensor(0)], week=2301),
                  sbf(103000, [sensor(1)], week=2301)]
        for mode in ('decoder', 'driver'):
            with self.subTest(mode=mode):
                args = ['-p', 'use_gps_timestamp:=true']
                if mode == 'driver': args += ['-p', 'messages.ext_sensor_meas:=true']
                result = wire.run('spec_ext_identity_' + mode, 'sbf_' + mode + '_node', frames, args, chunk_delay=.1)
                self.assertEqual(len(result['raw']), 1)
                self.check_si(result['raw'][0], 103., 2301)


if __name__ == '__main__': unittest.main(argv=[__file__] + wire.unittest_args)
