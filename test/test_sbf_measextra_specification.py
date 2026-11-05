#!/usr/bin/env python3
"""S1 mosaic-X5 FW4.15.0 pp268-269: MeasExtra revisions/count/CN0.

No RTKLIB encoder or decoder supplies fixture bytes or expected values.
Variance, correction and lock metadata without ROS fields are not claimed.
"""
import math
import struct
import unittest
import test_receiver_streams as wire
import test_protocol_specification as spec


def extra(tow, revision=3, count=1, fine=7, short=False, extension=0):
    length = [12,14,16,16][min(revision,3)]+extension
    rows=[]
    for index in range(count):
        row=bytearray(length)
        row[0]=0 if index==count-1 else 1+index%255
        row[1]=0  # GPS L1CA, main antenna
        struct.pack_into('<hhHHH',row,2,-123,456,25,16,10)
        if revision>=1:row[12]=3;row[13]=255
        if revision>=3:row[15]=fine
        rows.append(bytes(row))
    body=struct.pack('<IHBBf',tow,2300,count%256,length,2.)+b''.join(rows)
    if short:body=body[:-length]
    return spec.sbf_spec(4000,body,revision)


class MeasExtraSpecification(unittest.TestCase):
    def test_compatible_revision_and_subblock_extension(self):
        frames=[]
        for i,rev in enumerate((0,1,2,3,7)):
            ms=100500+i*1000
            frames += [spec.meas_spec(ms),extra(ms,rev,extension=4),
                       spec.sbf_spec(5922,struct.pack('<IH2x',ms,2300))]
        for mode in ('decoder','driver'):
            result=wire.run('measextra_extension_'+mode,'sbf_'+mode+'_node',frames,spec.PARAMS,chunk_delay=.02)
            self.assertEqual(len(result['obs']),5)
            for i,m in enumerate(result['obs']):
                main=next(o for o in m.observations if o.code_str=='1C')
                self.assertEqual(main.snr,45.+(7*.03125 if i>=3 else 0.))

    def test_revisions_count_wrap_missing_and_epoch_isolation(self):
        cases=[(0,1,False,False),(1,1,False,False),(2,1,False,False),
               (3,1,False,False),(3,255,False,False),(3,256,False,False),
               (3,257,False,False),(3,1,True,False),(3,2,False,True)]
        for mode in ('decoder','driver'):
            frames=[]
            for i,(rev,count,missing,short) in enumerate(cases):
                tow=100500+i*1000
                frames += [spec.meas_spec(tow,missing=missing),extra(tow,rev,count,short=short),
                           spec.sbf_spec(5922,struct.pack('<IH2x',tow,2300))]
            # Different epoch cannot refine the preceding measurement.
            frames += [spec.meas_spec(109500),extra(110500),
                       spec.sbf_spec(5922,struct.pack('<IH2x',109500,2300)),
                       spec.meas_spec(111500),extra(111500),
                       spec.sbf_spec(5922,struct.pack('<IH2x',111500,2300))]
            result=wire.run('measextra_spec_'+mode,'sbf_'+mode+'_node',frames,spec.PARAMS,chunk_delay=.01)
            self.assertEqual([m.tow for m in result['obs']],[100.5+i for i in range(len(cases))]+[109.5,111.5])
            for i,msg in enumerate(result['obs']):
                with self.subTest(mode=mode,case=i):
                    rows={o.code_str:o for o in msg.observations}
                    self.assertEqual(set(rows),{'1C','2W'})
                    main=rows['1C']
                    if i==7:
                        self.assertTrue(math.isnan(main.snr))
                        continue
                    expected=45.+(7*.03125 if i in (3,4,5,6,10) else 0.)
                    self.assertEqual(main.snr,expected)
                    self.assertAlmostEqual(main.p,21000000.125,places=6)
                    self.assertAlmostEqual(main.l,21000000.125*1575420000/299792458-12.345,places=6)
                    self.assertEqual(main.d,-123.5)
                    self.assertEqual(rows['2W'].snr,45.)


if __name__=='__main__':unittest.main(argv=[__file__]+wire.unittest_args)
