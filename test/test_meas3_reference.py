#!/usr/bin/env python3
"""Run frozen manufacturer-derived expectations without RxTools dependency."""
import json
from pathlib import Path
import unittest
import test_receiver_streams as wire
from check_meas3_constellations import fixtures,validate,PARAMS

TABLE=json.loads(Path(__file__).with_name('meas3_reference.json').read_text())

class Meas3Reference(unittest.TestCase):
    def test_seven_constellations_master_delta_extras_lli_and_channels(self):
        for label,frames in fixtures():
            rows=TABLE['cases'][label]
            expected={tuple(r['key']):(r['values'],r['lli'],r['freq']) for r in rows}
            for mode in ('decoder','driver'):
                with self.subTest(layout=label,node=mode):
                    result=wire.run('meas3_reference_'+label+'_'+mode,'sbf_'+mode+'_node',frames,PARAMS,chunk_delay=.04)
                    validate(result,expected,label+'_'+mode)

if __name__=='__main__':unittest.main(argv=[__file__]+wire.unittest_args)
