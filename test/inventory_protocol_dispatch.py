#!/usr/bin/env python3
"""Inventory compiled receiver dispatch, without treating handlers as conformance.

Uses a configured build's patched sources and generated feature definitions.
The report deliberately retains no-op branches and frontend-rejected navigation.
Payload versions, proprietary subtypes and firmware applicability need separate
review: a top-level ID count is never a specification coverage percentage.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess


def body(text,name):
    match=re.search(r'\b'+re.escape(name)+r'\s*\([^;{}]*\)\s*\{',text)
    if not match:raise ValueError('missing function '+name)
    start=match.end();depth=1
    for end in range(start,len(text)):
        if text[end]=='{':depth+=1
        elif text[end]=='}':depth-=1
        if not depth:return text[start:end]
    raise ValueError('unterminated function '+name)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('build',type=Path);ap.add_argument('output',type=Path)
    ap.add_argument('--catalog-features',action='store_true',
                    help='include known IDs even when constellation support is disabled in this build')
    args=ap.parse_args();build=args.build.resolve()
    manifest=build/'protocol_compile_definitions.txt'
    if manifest.exists():
        definitions=['-D'+line for line in manifest.read_text().splitlines() if line]
    else:  # compatibility with older configured Makefiles builds
        flags=(build/'CMakeFiles/rtkcore_lib.dir/flags.make').read_text()
        definitions=shlex.split(re.search(r'^C_DEFINES = (.*)$',flags,re.M)[1])
    if args.catalog_features:
        definitions=sorted(set(definitions)|{'-D'+flag for flag in
            ['ENAGLO','ENAGAL','ENACMP','ENAQZS','ENASBS','ENAIRN']})
    includes=['-I'+str(build/'rtklib_source/src'),
              '-I'+str(Path(__file__).resolve().parents[1]/'include')]
    rows=[];sources={}
    raw_navigation=build/'rtklib_patched/rcvraw.c'
    if raw_navigation.exists():sources['rcvraw.c']=hashlib.sha256(raw_navigation.read_bytes()).hexdigest()
    for family,filename,function in [('UBX','ublox.c','decode_ubx'),('SBF','septentrio.c','decode_sbf'),
                                     ('OEM4','novatel.c','decode_oem4'),('RTCM3','rtcm3.c','decode_rtcm3')]:
        path=build/'rtklib_patched'/filename;source=path.read_text()
        sources[filename]=hashlib.sha256(path.read_bytes()).hexdigest()
        names={}
        for name,value in re.findall(r'^#define\s+(ID_\w+)\s+(0x[0-9a-fA-F]+|\d+)\b',source,re.M):
            names.setdefault(int(value,0),[]).append(name)
        compiled=subprocess.check_output(['cc','-E','-P',*definitions,*includes,str(path)],text=True)
        dispatch=body(compiled,function)
        if family=='SBF':
            # Earlier switches validate payload bounds; only the final switch
            # dispatches a block. Counting both duplicated eight message IDs.
            switches=list(re.finditer(r'\bswitch\s*\(type\)\s*\{',dispatch))
            if not switches:raise ValueError('missing SBF dispatch switch')
            dispatch=body('selected() '+dispatch[switches[-1].end()-1:],'selected')
        cases=list(re.finditer(r'\bcase\s+(0x[0-9a-fA-F]+|\d+)\s*:',dispatch));pending=[]
        if len({int(c[1],0) for c in cases})!=len(cases):
            raise ValueError('duplicate dispatch IDs in '+family)
        for index,case in enumerate(cases):
            pending.append(int(case[1],0))
            segment=dispatch[case.end():cases[index+1].start() if index+1<len(cases) else len(dispatch)]
            actions=re.findall(r'(?:return|ret\s*=)\s*([a-zA-Z_]\w*\s*\([^;]+\)|0)\s*;',segment)
            if not actions:continue
            # A conditional early return (e.g. EndOfMeas without pending data)
            # does not make a later decoder/flush call a no-op branch.
            action=next((a for a in actions if a!='0'),'0')
            for identifier in pending:
                row={'family':family,'id':identifier,'names':names.get(identifier,[]),
                     'backend_action':' '.join(action.split()),
                     'backend_status':'no_op' if action=='0' else 'handler_present',
                     'spec_conformance':'not_established_by_inventory',
                     'version_coverage':'requires_message_specific_review'}
                if family=='UBX':row['class_id_hex']=f'{identifier>>8:02X}-{identifier&255:02X}'
                if family=='SBF' and identifier in [4042,4258,4251,4252,4253,4254]:
                    row['ros_frontend']='rejected_before_backend'
                if family=='SBF' and identifier==4004:
                    row['ros_frontend']='rev0/96 bytes or rev1/100 bytes only (including header)'
                rows.append(row)
            pending=[]
        if pending:raise ValueError('unresolved cases '+str(pending))
    counts={f:{'ids':sum(r['family']==f for r in rows),
               'direct_no_ops':sum(r['family']==f and r['backend_status']=='no_op' for r in rows)}
            for f in ['UBX','SBF','OEM4','RTCM3']}
    result={'scope':__doc__,'defines':definitions,'patched_source_sha256':sources,
            'counts':counts,'backend_dispatch':rows,
            'limitations':['ROS frontend PVT/IMU/configuration and NMEA are not part of this inventory',
                           'handler_present can include stubs or output without a ROS topic',
                           'MSM/SSR and raw navigation subtypes are not distinct top-level IDs',
                           'RTCM normative conformance deferred by user; inventory is source evidence only']}
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(counts))


if __name__=='__main__':main()
