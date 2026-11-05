#!/usr/bin/env python3
"""Rebuild the diagnostic-only ID catalog from compiled dispatch + ROS frontend IDs.

--write updates the checked-in header. Without it, stale IDs fail the check.
Frontend entries include ACK/config replies; recognizing an ID does not promise
that its payload version or fields are supported. No acceptance filter uses it.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[1]
FRONTEND={'UBX':[0x0107,0x0104,0x0122,0x0136,0x0101,0x0111,0x0113,0x0114,0x1003,0x1015,
                 0x0500,0x0501,0x0a04,0x063e,0x0604,0x0608,0x0624,0x0601,0x068a],
          'SBF':[4007,4006,5906,5905,5908,5907,4001,4050],
          'OEM4':[42,99,174,241,1462,812], 'RTCM3':[]}

def render(inventory):
    lines=['// Source dispatch inventory + ROS frontend IDs. Diagnostic only, never an input gate.',
           '// Regeneration/check: test/check_diagnostic_catalog.py','#pragma once',
           '#include <algorithm>','#include <iterator>', 'namespace gnss_ros_standardization::wire {']
    for family,extra in FRONTEND.items():
        ids=sorted(set(extra+[r['id'] for r in inventory['backend_dispatch'] if r['family']==family]))
        lines += [f'inline bool known{family}Id(unsigned id) {{',
                  '  static constexpr unsigned ids[] = {'+', '.join(map(str,ids))+'};',
                  '  return std::binary_search(std::begin(ids),std::end(ids),id);','}']
    for family,extra in FRONTEND.items():
        ids=sorted({r['id'] for r in inventory['backend_dispatch']
                    if r['family']==family and r['backend_status']=='no_op'}-set(extra))
        lines += [f'inline bool noDecoder{family}Id(unsigned id) {{']
        if ids:
            lines += ['  static constexpr unsigned ids[] = {'+', '.join(map(str,ids))+'};',
                      '  return std::binary_search(std::begin(ids),std::end(ids),id);']
        else:lines += ['  (void)id; return false;']
        lines += ['}']
    return '\n'.join(lines+['}'])+'\n'

if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--build',required=True);ap.add_argument('--write',action='store_true')
    args=ap.parse_args()
    with tempfile.TemporaryDirectory() as tmp:
        inventory=Path(tmp)/'dispatch.json'
        subprocess.run([sys.executable,str(ROOT/'test/inventory_protocol_dispatch.py'),args.build,str(inventory),'--catalog-features'],check=True)
        expected=render(json.loads(inventory.read_text()))
    header=ROOT/'include/gnss_ros_standardization/protocol_id_catalog.hpp'
    if args.write:header.write_text(expected)
    elif header.read_text()!=expected:raise SystemExit('diagnostic catalog is stale; regenerate with --write')
    print('Diagnostic ID catalog matches source dispatch with all constellation features and frontend list.')
