#!/usr/bin/env python3
"""Verify the compiled diagnostic catalog and reject accidental source drift."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'cmake'))
from receiver_diagnostic_reasons import apply

ap=argparse.ArgumentParser(description=__doc__)
ap.add_argument('--build',required=True)
args=ap.parse_args()
manifest=json.loads((ROOT/'cmake/receiver_diagnostic_reasons.json').read_text())
count=0
for name,entry in manifest.items():
    source=(Path(args.build)/'rtklib_patched'/f'{name}_diagnostic_base.c').read_text()
    output=apply(name,source)
    actual=(Path(args.build)/'rtklib_patched'/f'{name}.c').read_text()
    assert output==actual, name
    assert 'GNSS_RX_NAV_NO_OUTPUT' not in output, name
    assert all(s['reason']!='UNRESOLVED' for s in entry['sites']),name
    assert len({s['line'] for s in entry['sites']})==len(entry['sites']),name
    try:
        apply(name,source+'\n/* dependency drift */\n')
    except ValueError:
        pass
    else:
        raise AssertionError(f'{name}: accepted unreviewed source')
    count+=len(entry['sites'])
print(f'{count} diagnostic sites match the compiled source; all four source-drift guards reject changes.')
