#!/usr/bin/env python3
"""Apply reviewed diagnostic reasons to the pinned, already patched backend.

The manifest records source context and its hash, not heuristic classifications.
A backend/functional patch change must be reviewed before this gate is updated.
Return values, buffer writes and publication decisions remain unchanged.
"""
import hashlib
import json
from pathlib import Path
import re
import sys

NEGATIVE = 'gnss_receiver_negative(__func__,__LINE__)'
ZERO = '(GNSS_RX_EVENT(GNSS_RX_NAV_NO_OUTPUT,1),0)'
FAMILIES = {
    'decode_eph': 'GPS', 'decode_nav': 'GPS', 'decode_gpsrawcanav': 'GPS',
    'decode_qzssrawsubframeb': 'GPS', 'decode_inav': 'INAV',
    'decode_galrawinav': 'INAV', 'decode_fnav': 'FNAV', 'decode_galrawfnav': 'FNAV',
    'decode_cnav': 'BDS', 'decode_cmpraw': 'BDS', 'decode_gnav': 'GLO',
    'decode_glorawcanav': 'GLO', 'decode_navicraw': 'IRN',
}

def family_expression(family):
    if family == 'BDS':
        return '(prn>=6 && prn<=58 ? GNSS_NAV_BDS1 : GNSS_NAV_BDS2)'
    return 'GNSS_NAV_' + family


def apply(name, source):
    manifest = json.loads(Path(__file__).with_suffix('.json').read_text())[name]
    if hashlib.sha256(source.encode()).hexdigest() != manifest['sha256']:
        raise ValueError(f'{name}: diagnostic source changed; review reason manifest before rebuilding')
    lines = source.splitlines(keepends=True)
    for site in manifest['sites']:
        i, reason, target = site['line']-1, site['reason'], site['target']
        if target not in lines[i]:
            raise ValueError(f'{name}:{i+1}: missing reviewed diagnostic site')
        if reason in ('RECORDED', 'PROPAGATED'):
            replacement = '0' if target == ZERO else '-1' if reason == 'PROPAGATED' else target
        elif reason=='WAIT_LEGACY_GPS':
            replacement='(id>=1 && id<=2 ? gnss_nav_wait(raw,sat,GNSS_NAV_GPS,0) : (GNSS_RX_EVENT(id<1 || id>5 ? GNSS_RX_FIELD : GNSS_RX_NAV_PAGE_IGNORED,1),0))'
        elif reason.startswith(('WAIT_', 'FAIL_')):
            family = reason.split('_', 1)[1]
            replacement = f'gnss_nav_wait(raw,sat,GNSS_NAV_{family},{int(reason.startswith("FAIL_"))})'
            # Almanac pages are not new ephemeris fragments.
            if reason == 'WAIT_BDS1':
                replacement = f'(id<=2 ? {replacement} : (GNSS_RX_EVENT(GNSS_RX_NAV_PAGE_IGNORED,1),0))'
            if reason == 'WAIT_GLO':
                replacement = f'(m<={5 if name=="septentrio" else 3} ? {replacement} : (GNSS_RX_EVENT(GNSS_RX_NAV_PAGE_IGNORED,1),0))'
            if reason == 'WAIT_FNAV':
                replacement = f'(type<4 ? {replacement} : (GNSS_RX_EVENT(GNSS_RX_NAV_PAGE_IGNORED,1),0))'
        else:
            kind = reason if 'GNSS_RX_' in reason else 'GNSS_RX_' + reason
            result = '0' if target == ZERO else NEGATIVE
            if reason == 'NAV_DUPLICATE' and site['function'] in FAMILIES:
                family = family_expression(FAMILIES[site['function']])
                result = f'gnss_receiver_navigation(sat,{family},1,0),0'
            replacement = f'(GNSS_RX_EVENT({kind},1),{result})'
            if target == 'goto invalid_meas3;':
                replacement = f'{{ GNSS_RX_EVENT({kind},1); goto invalid_meas3; }}'
        lines[i] = lines[i].replace(target, replacement)
    source = ''.join(lines)
    # Correct the original length-labelled compound checks without double
    # reporting a revision failure as a length failure.
    for name_ in ('decode_gpscnav', 'decode_glonav', 'decode_cmpcnav2'):
        if name != 'septentrio': break
        pattern = r'(static int '+name_+r'\([^\n]*\n.*?)(\n}\n)'
        match = re.search(pattern,source,re.S)
        if not match: raise ValueError(name_)
        body = match[1].replace('GNSS_RX_EVENT(GNSS_RX_LENGTH,1); ', '',1)
        source=source[:match.start(1)]+body+source[match.end(1):]
    if name != 'rtcm3':
        source = source.replace('#include "gnss_ros_standardization/receiver_diagnostics.h"',
            '#include "gnss_ros_standardization/receiver_navigation_diagnostics.h"')
        # Ready/duplicate clears only the corresponding assembly family. Full
        # decoded ephemerides never fabricate an observed raw-page assembly.
        for func, family in FAMILIES.items():
            pattern = r'(static int '+func+r'\([^\n]*\n\{?.*?)(\n}\n)'
            match = re.search(pattern,source,re.S)
            if not match: continue
            body=match[1]
            ready=f'gnss_receiver_navigation(sat,{family_expression(family)},1,0)'
            body=body.replace('return 2;',f'return ({ready},2);')
            body=body.replace('(GNSS_RX_EVENT(GNSS_RX_NAV_WAIT,1),0)',
                f'gnss_nav_wait(raw,sat,{family_expression(family)},0)')
            if func=='decode_navicraw':
                body=body.replace('return ret;', 'return (ret ? ret : (GNSS_RX_EVENT(GNSS_RX_NAV_AUXILIARY,1),0));')
                body=body.replace('memcpy(raw->subfrm[sat-1]+id*37, buff, 37);',
                    'gnss_receiver_irn_fragment(sat,id);\n    memcpy(raw->subfrm[sat-1]+id*37, buff, 37);')
            source=source[:match.start(1)]+body+source[match.end(1):]
    # Other zero returns with a documented reason, not a generic no-output.
    if name=='septentrio':
        for func in ('gnss_meas3_extra','decode_measepochextra'):
            pattern=r'(static int '+func+r'\([^\n]*\n.*?)(\n}\n)'
            match=re.search(pattern,source,re.S)
            if not match: raise ValueError(func)
            body=match[1]
            body=body.replace('if (strstr(raw->opt,"-NO_MEAS3")) return 0;',
                'if (strstr(raw->opt,"-NO_MEAS3")) return (GNSS_RX_EVENT(GNSS_RX_SELECTION,1),0);')
            body=body.replace('if ((U2(raw->buff+14)&7)!=ant) return 0;',
                'if ((U2(raw->buff+14)&7)!=ant) return (GNSS_RX_EVENT(GNSS_RX_SELECTION,1),0);')
            body=body.replace('timediff(raw->time,raw->obuf.data[0].time)!=0) return 0;',
                'timediff(raw->time,raw->obuf.data[0].time)!=0) return (GNSS_RX_EVENT(GNSS_RX_REFERENCE,1),0);')
            source=source[:match.start(1)]+body+source[match.end(1):]
        source=source.replace('return (svid==62) ? 0 : -1;',
            'return (GNSS_RX_EVENT(GNSS_RX_NAV_SATELLITE,1), (svid==62) ? 0 : -1);')
    if name=='rtcm3':
        for typ in (1019,1020,1041,1042,1044,1045,1046):
            pattern=r'(static int decode_type'+str(typ)+r'\([^\n]*\n.*?)(\n}\n)'
            match=re.search(pattern,source,re.S)
            if not match: raise ValueError(typ)
            body=match[1]
            # These functions' only zero returns are the explicit Galileo
            # selection option and unchanged ephemeris checks.
            body=re.sub(r'(if \(strstr\(rtcm->opt,[^\n]*)(return 0;)',
                r'\1return (GNSS_RX_EVENT(GNSS_RX_SELECTION,1),0);',body)
            body=re.sub(r'(if \(strstr\(rtcm->opt,[^\n]*\n\s*)return 0;',
                r'\1return (GNSS_RX_EVENT(GNSS_RX_SELECTION,1),0);',body)
            body=body.replace('return 0;','return (GNSS_RX_EVENT(GNSS_RX_NAV_DUPLICATE,1),0);')
            source=source[:match.start(1)]+body+source[match.end(1):]
    return source

if __name__ == '__main__':
    name, path = sys.argv[1:]
    p=Path(path)
    p.write_text(apply(name,p.read_text()))
