#!/usr/bin/env python3
"""Selected PVT layouts (U1 pp150-153, S1 pp346-349, N1 pp529/543) and authored
supplements (U1 pp140-144/148-149/168, S1 pp342-358, N1 pp545-546/796-797).

Independent authored payloads at the equator/Greenwich make ECEF and ENU
expectations analytic; values are binary fractions and analytic equator rotations.
Frame/CRC helpers are shared; production offsets and coordinate conversion
functions never determine the oracle. This is selected-layout coverage, not
all firmware/revision certification.
"""
import math
import struct
import unittest
import test_receiver_streams as wire
import test_protocol_specification as spec


def payload(family, ms, invalid=False):
    if family=='ubx':
        p=bytearray(92)
        struct.pack_into('<I',p,0,ms);p[11]=7;p[20]=3;p[21]=1;p[23]=12
        struct.pack_into('<iii',p,24,0,1000000000 if invalid else 0,123250)
        struct.pack_into('<IIiii',p,40,200,300,1000,-2000,-3000)
        struct.pack_into('<I',p,68,300)
        anchor=spec.frame(struct.pack('<dHbBBB2x',ms*.001,2300,18,1,1,1)+spec.row(1))
        return anchor+wire.ubx(1,7,p)
    if family=='sbf':
        p=bytearray(88)
        struct.pack_into('<IHBB3d',p,0,ms,2300,1,0,2. if invalid else 0.,0.,123.25)
        struct.pack_into('<fff',p,36,1.,-2.,3.);p[66]=12
        return spec.sbf_spec(4007,p,2)
    p=bytearray(72)
    struct.pack_into('<II3dfI3f',p,0,0,16,100. if invalid else 0.,0.,93.25,30.,61,.1,.2,.3)
    p[65]=12
    velocity=struct.pack('<IIffdddf',0,8,0.,0.,math.sqrt(5),360+math.degrees(math.atan2(-2,1)),3.,0.)
    return wire.oem(42,p,tow=ms)+wire.oem(99,velocity,tow=ms)


class PvtSpecification(unittest.TestCase):
    def test_position_height_velocity_time_and_invalid_latitude_both_paths(self):
        for family in ('ubx','sbf','novatel'):
            for mode in ('decoder','driver'):
                with self.subTest(family=family,mode=mode):
                    params=['-p','use_gps_timestamp:=true','-p','local_origin.mode:=fixed']
                    if mode=='driver' and family=='sbf':params+=['-p','messages.pvt_geodetic:=true']
                    if mode=='driver' and family=='novatel':params+=['-p','messages.bestpos:=true','-p','messages.bestvel:=true']
                    result=wire.run('pvt_spec_'+family+'_'+mode,family+'_'+mode+'_node',
                                    [payload(family,100500),payload(family,101500,True),payload(family,102500)],
                                    params,chunk_delay=.15)
                    self.assertEqual([m.time_tow for m in result['sol']],[100.5,102.5])
                    for msg in result['sol']:
                        self.assertEqual((msg.time_week,msg.status,msg.solution_source,msg.num_sats),(2300,5,1,12))
                        self.assertEqual((msg.latitude,msg.longitude,msg.altitude),(0.,0.,123.25))
                        for point,values in [(msg.pos_ecef,(6378137.+123.25,0.,0.)),
                                             (msg.vel_enu,(-2.,1.,3.)),(msg.vel_ecef,(3.,-2.,1.))]:
                            for key,value in zip('xyz',values):
                                self.assertAlmostEqual(getattr(point,key),value,delta=1e-6)
                        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                        self.assertAlmostEqual(stamp,315964800+2300*604800+msg.time_tow-18,delta=1e-6)
                        if family=='novatel':
                            for index,value in zip((0,4,8),(.04,.01,.09)):
                                self.assertAlmostEqual(msg.pos_enu_cov[index],value,delta=1e-7)


class LocalOriginSpecification(unittest.TestCase):
    """local_origin.* is one schema for every receiver node: auto, fixed llh, fixed ecef."""
    def test_auto_fixed_llh_and_fixed_ecef_origin(self):
        fix=(6378137.+123.25,0.,0.)  # payload(): lat 0, lon 0, h 123.25
        cases=[([],fix,(0.,0.,0.)),
               (['-p','local_origin.mode:=fixed'],(6378137.,0.,0.),(0.,0.,123.25)),
               (['-p','local_origin.mode:=fixed','-p','local_origin.postype:=ecef',
                 '-p','local_origin.pos:=[6378160.25,0.0,0.0]'],(6378160.25,0.,0.),(0.,0.,100.))]
        for family in ('ubx','sbf','novatel'):
            for mode in ('decoder','driver'):
                for index,(origin_params,origin,enu) in enumerate(cases):
                    with self.subTest(family=family,mode=mode,case=index):
                        params=['-p','use_gps_timestamp:=true']+origin_params
                        if mode=='driver' and family=='sbf':params+=['-p','messages.pvt_geodetic:=true']
                        if mode=='driver' and family=='novatel':params+=['-p','messages.bestpos:=true','-p','messages.bestvel:=true']
                        result=wire.run('origin_spec_'+family+'_'+mode+str(index),family+'_'+mode+'_node',
                                        [payload(family,100500),payload(family,102500)],params,chunk_delay=.15)
                        self.assertEqual(len(result['sol']),2)
                        for msg in result['sol']:
                            for point,values in [(msg.pos_enu_org_ecef,origin),(msg.pos_enu,enu)]:
                                for key,value in zip('xyz',values):
                                    self.assertAlmostEqual(getattr(point,key),value,delta=1e-6)


def ubx_cov(ms, version=0, valid=1):
    return struct.pack('<IBBB9x12f', ms, version, valid, valid,
                       4., .5, -.25, 9., .75, 16.,
                       1., .125, -.0625, 2.25, .1875, 4.)


def sbf_cov(mid, ms, invalid=False):
    # TOW/WNc/Mode/Error followed by diagonals, clock and cross terms.
    values=(4., 9., 16., 25., .5, -.25, 0., .75, 0., 0.)
    if invalid: values=(-2e10,)+values[1:]
    return spec.sbf_spec(mid, struct.pack('<IHBB10f', ms, 2300, 1, 0, *values))


def dop(family, ms):
    if family=='ubx':
        return wire.ubx(1, 4, struct.pack('<I7H', ms, 500, 300, 400, 240, 180, 100, 100))
    if family=='sbf':
        return spec.sbf_spec(4001, struct.pack('<IHBB4H2f', ms, 2300, 12, 0,
                                              300, 400, 180, 240, 10., 20.))
    # Full PSRDOP, including cutoff, count and PRNs (unused in ROS).
    return wire.oem(174, struct.pack('<6fI3I', 5., 3., 1.8, 4., 4., 5., 3, 1, 2, 3), tow=ms)


def cartesian(family, ms):
    if family=='sbf':
        p=bytearray(88)
        struct.pack_into('<IHBB3df3f', p, 0, ms, 2300, 1, 0,
                         6378260.25, 0., 0., 30., 3., -2., 1.)
        p[66]=12
        return spec.sbf_spec(4006, p, 2)
    p=bytearray(112)
    struct.pack_into('<II3d3fII3d3f', p, 0, 0, 16, 6378260.25, 0., 0.,
                     2., 3., 4., 0, 8, 3., -2., 1., .5, .75, 1.)
    p[104:108]=bytes([15, 12, 10, 8])
    return wire.oem(241, p, tow=ms)


class PvtSupplementSpecification(unittest.TestCase):
    def assert_array(self, actual, expected, tolerance=1e-7):
        self.assertEqual(len(actual), len(expected))
        for a,b in zip(actual,expected): self.assertAlmostEqual(a,b,delta=tolerance)

    def params(self, family, mode, names):
        p=['-p','use_gps_timestamp:=true','-p','local_origin.mode:=fixed']
        if mode=='driver':
            for name in names: p+=['-p','messages.'+name+':=true']
        return p

    def assert_dop(self, solutions):
        self.assertTrue(math.isnan(solutions[0].pdop))  # cadence unknown
        self.assert_array([getattr(solutions[1],f) for f in ('gdop','pdop','hdop','vdop')],
                          [5.,3.,1.8,2.4], 5e-7)

    def test_ubx_ecef_covariance_dop_and_invalid_covariance_recovery(self):
        for mode in ('decoder','driver'):
            with self.subTest(mode=mode):
                data=[]
                for index,ms in enumerate((100500,101500,102500,103500)):
                    cov=ubx_cov(ms, version=1 if index==2 else 0)
                    extra=wire.ubx(1,1,struct.pack('<IiiiI',ms,637826025,0,0,20))
                    extra+=wire.ubx(1,17,struct.pack('<IiiiI',ms,300,-200,100,30))
                    data.append(dop('ubx',ms)+payload('ubx',ms)+wire.ubx(1,54,cov)+extra)
                result=wire.run('pvt_supp_ubx_'+mode,'ubx_'+mode+'_node',data,
                    self.params('ubx',mode,['nav_cov','nav_dop','nav_posecef','nav_velecef']),chunk_delay=.15)
                self.assertEqual([m.time_tow for m in result['sol']],[100.5,101.5,102.5,103.5])
                self.assert_dop(result['sol'])
                for index,m in enumerate(result['sol']):
                    self.assert_array([getattr(m.pos_ecef,k) for k in 'xyz'],[6378260.25,0,0])
                    self.assert_array([getattr(m.vel_ecef,k) for k in 'xyz'],[3,-2,1])
                    if index==2:
                        # Repository fallback divides the combined hAcc^2 equally
                        # between E/N; this is a policy, not a covariance in U1.
                        self.assertAlmostEqual(m.pos_enu_cov[0]+m.pos_enu_cov[4],.04,delta=1e-7)
                        self.assertEqual(m.pos_enu_cov[1],0.)
                    else:
                        self.assert_array(m.pos_enu_cov,[9,.5,-.75,.5,4,.25,-.75,.25,16])
                        self.assert_array(m.pos_cov_ecef,[16,-.75,.25,-.75,9,.5,.25,.5,4])
                        self.assert_array(m.vel_enu_cov,[2.25,.125,-.1875,.125,1,.0625,-.1875,.0625,4])

    def test_ubx_high_precision_signed_components_validity_and_recovery(self):
        for mode in ('decoder','driver'):
            for mid,name in ((19,'nav_hpposecef'),(20,'nav_hpposllh')):
                with self.subTest(mode=mode,mid=mid):
                    data=[]
                    for index,ms in enumerate((100500,101500,102500)):
                        if mid==19:
                            p=struct.pack('<B3xI3i3bBI',0,ms,637826025,0,0,99,-99,9,index==1,2000)
                        else:
                            p=struct.pack('<B2xBI4i4b2I',0,index==1,ms,0,0,123250,93250,-99,99,-9,9,2000,3000)
                        data.append(payload('ubx',ms)+wire.ubx(1,mid,p))
                    result=wire.run('pvt_hp_'+name+'_'+mode,'ubx_'+mode+'_node',data,
                                   self.params('ubx',mode,[name]),chunk_delay=.15)
                    self.assertEqual([m.time_tow for m in result['sol']],[100.5,101.5,102.5])
                    for i,m in enumerate(result['sol']):
                        if i==1:
                            self.assertEqual((m.latitude,m.longitude,m.altitude),(0.,0.,123.25))
                            self.assert_array([getattr(m.pos_ecef,k) for k in 'xyz'],[6378260.25,0.,0.])
                        elif mid==19:
                            self.assert_array([getattr(m.pos_ecef,k) for k in 'xyz'],[6378260.2599,-.0099,.0009])
                        else:
                            self.assertAlmostEqual(m.latitude,99e-9,delta=1e-15)
                            self.assertAlmostEqual(m.longitude,-99e-9,delta=1e-15)
                            self.assertAlmostEqual(m.altitude,123.2491,delta=1e-10)

    def test_sbf_cartesian_geodetic_covariance_dop_dnu_and_recovery(self):
        for mode in ('decoder','driver'):
            for cart in (False,True):
                with self.subTest(mode=mode,cartesian=cart):
                    data=[]
                    for index,ms in enumerate((100500,101500,102500,103500)):
                        pos=cartesian('sbf',ms) if cart else payload('sbf',ms)
                        data.append(dop('sbf',ms)+pos+sbf_cov(5905 if cart else 5906,ms,index==2)+
                                    sbf_cov(5907 if cart else 5908,ms,index==2))
                    names=['dop','pvt_cartesian' if cart else 'pvt_geodetic',
                           'pos_cov_cartesian' if cart else 'pos_cov_geodetic',
                           'vel_cov_cartesian' if cart else 'vel_cov_geodetic']
                    result=wire.run('pvt_supp_sbf_'+mode+str(cart),'sbf_'+mode+'_node',data,
                                   self.params('sbf',mode,names),chunk_delay=.15)
                    self.assertEqual([m.time_tow for m in result['sol']],[100.5,101.5,102.5,103.5])
                    self.assert_dop(result['sol'])
                    for index,m in enumerate(result['sol']):
                        self.assert_array([getattr(m.pos_ecef,k) for k in 'xyz'],[6378260.25,0.,0.])
                        self.assert_array([getattr(m.vel_ecef,k) for k in 'xyz'],[3.,-2.,1.])
                        self.assertEqual((m.status,m.num_sats),(5,12))
                        cov=m.pos_cov_ecef if cart else m.pos_enu_cov
                        vel=m.vel_cov_ecef if cart else m.vel_enu_cov
                        expected=[4,.5,-.25,.5,9,.75,-.25,.75,16] if cart else [9,.5,.75,.5,4,-.25,.75,-.25,16]
                        if index==2: expected=[0.]*9  # no covariance from rejected DNU block
                        self.assert_array(cov,expected)
                        self.assert_array(vel,expected)

    def test_oem_delayed_velocity_and_bestxyz_only(self):
        for mode in ('decoder','driver'):
            frames=[cartesian('novatel',99000)]
            for i,latency in enumerate((.25,float('nan'),0.)):
                ms=100000+i*1000
                pair=payload('novatel',ms)
                pos=pair[:104]  # 28-byte header +72-byte BESTPOS +CRC
                vel=bytearray(pair[104+28:-4]);struct.pack_into('<f',vel,8,latency)
                xyz=bytearray(cartesian('novatel',ms)[28:-4]);struct.pack_into('<f',xyz,92,latency)
                frames.append(pos+wire.oem(99,vel,tow=ms)+wire.oem(241,xyz,tow=ms))
            r=wire.run('oem_latency_'+mode,'novatel_'+mode+'_node',frames,
                self.params('novatel',mode,['bestpos','bestvel','bestxyz']),chunk_delay=.15)
            self.assertEqual([m.time_tow for m in r['sol']],[100.,101.,102.])
            for i,m in enumerate(r['sol']):
                self.assertEqual(m.pos_cov_ecef[0],4.)
                if i<2:
                    self.assertTrue(math.isnan(m.vel_enu.x))
                    self.assertTrue(math.isnan(m.vel_cov_ecef[0]))
                else:
                    self.assertAlmostEqual(m.vel_enu.x,-2.)
                    self.assertEqual(m.vel_cov_ecef[0],.25)

    def test_oem_bestxyz_covariance_dop_and_short_payload_recovery(self):
        for mode in ('decoder','driver'):
            with self.subTest(mode=mode):
                data=[]
                for index,ms in enumerate((100500,101500,102500,103500)):
                    xyz=wire.oem(241,bytes(20),tow=ms) if index==2 else cartesian('novatel',ms)
                    data.append(dop('novatel',ms)+payload('novatel',ms)+xyz)
                result=wire.run('pvt_supp_oem_'+mode,'novatel_'+mode+'_node',data,
                    self.params('novatel',mode,['bestpos','bestvel','bestxyz','psrdop']),chunk_delay=.15)
                self.assertEqual([m.time_tow for m in result['sol']],[100.5,101.5,102.5,103.5])
                self.assert_dop(result['sol'])
                for index,m in enumerate(result['sol']):
                    self.assert_array([getattr(m.pos_ecef,k) for k in 'xyz'],[6378260.25,0.,0.])
                    self.assert_array([getattr(m.vel_ecef,k) for k in 'xyz'],[3.,-2.,1.])
                    if index==2:
                        self.assertAlmostEqual(m.pos_cov_ecef[0],.09,delta=1e-7)
                    else:
                        self.assert_array(m.pos_cov_ecef,[4,0,0,0,9,0,0,0,16])
                        self.assert_array(m.pos_enu_cov,[9,0,0,0,16,0,0,0,4])
                        self.assert_array(m.vel_cov_ecef,[.25,0,0,0,.5625,0,0,0,1])


if __name__=='__main__':unittest.main(argv=[__file__]+wire.unittest_args)
