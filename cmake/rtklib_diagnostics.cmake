# Observational hooks only, after all functional receiver patches. Exact match
# guards fail configuration when the pinned backend changes. No ABI additions.
if(receiver STREQUAL "rtcm3e")
  return()
endif()
gnss_receiver_replace(1 "#include \"rtklib.h\""
  "#include \"rtklib.h\"\n#include \"gnss_ros_standardization/receiver_diagnostics.h\"")

# Restrict zero-return hooks to named navigation functions, never byte-stream
# input or ordinary auxiliary/configuration dispatch. No-output is deliberately
# different from assembly-wait (it can also be duplicate/alert/parity failure).
function(gnss_navigation_zero name)
  string(FIND "${_receiver_source}" "static int ${name}(" start)
  if(start LESS 0)
    message(FATAL_ERROR "Missing diagnostic navigation function: ${receiver}/${name}")
  endif()
  string(SUBSTRING "${_receiver_source}" ${start} -1 tail)
  string(FIND "${tail}" "\n}\n" end)
  if(end LESS 0)
    message(FATAL_ERROR "Missing diagnostic function end: ${receiver}/${name}")
  endif()
  math(EXPR end "${end}+3")
  string(SUBSTRING "${tail}" 0 ${end} body)
  string(REPLACE "return 0;" "return (GNSS_RX_EVENT(GNSS_RX_NAV_NO_OUTPUT,1),0);" patched "${body}")
  string(REPLACE "${body}" "${patched}" result "${_receiver_source}")
  set(_receiver_source "${result}" PARENT_SCOPE)
endfunction()

if(receiver STREQUAL "ublox")
  foreach(label "ubx rxmsfrbx nav unsupported" "ubx rxmsfrbx Galileo E6 CNAV unsupported:"
                "ubx rxmsfrbx BDS B-CNAV1 unsupported:" "ubx rxmsfrbx BDS B-CNAV2 unsupported:")
    string(REGEX MATCH "trace\\(3, ?\"${label}" prefix "${_receiver_source}")
    if(prefix STREQUAL "")
      message(FATAL_ERROR "Missing unsupported navigation hook: ${label}")
    endif()
    gnss_receiver_replace(1 "${prefix}" "GNSS_RX_EVENT(GNSS_RX_NAV_UNSUPPORTED,1); ${prefix}")
  endforeach()
  gnss_receiver_replace(1 "if (tow >= 604800000.0) return -1;"
    "if (tow >= 604800000.0) return GNSS_RX_FAIL(GNSS_RX_TIME);")
  gnss_receiver_replace(1 "        trace(2,\"ubx rxmrawx unsupported version or invalid tow"
    "        GNSS_RX_EVENT(ver>1 ? GNSS_RX_VERSION : GNSS_RX_TIME,1);\n        trace(2,\"ubx rxmrawx unsupported version or invalid tow")
  gnss_receiver_replace(1 "if (type!=5) return 0;"
    "if (type!=5) return (GNSS_RX_EVENT(GNSS_RX_NAV_WAIT,1),0);")
  gnss_receiver_replace(1 "if (type != 4) return 0;"
    "if (type != 4) return (GNSS_RX_EVENT(GNSS_RX_NAV_WAIT,1),0);")
  gnss_receiver_replace(1 "    return 0;\n}\n/* decode Galileo I/NAV"
    "    return (GNSS_RX_EVENT(GNSS_RX_NAV_WAIT,1),0);\n}\n/* decode Galileo I/NAV")
  foreach(name decode_eph decode_ionutc decode_nav decode_inav decode_fnav decode_cnav
               decode_gnav decode_snav decode_rxmsfrbx decode_trksfrbx decode_rxmsfrb)
    gnss_navigation_zero(${name})
  endforeach()
  foreach(label "ubx rxmraw sat number error:" "ubx rxmrawx: system error"
                "ubx rxmrawx sat number error:" "ubx trkmeas: system error"
                "ubx trkmeas sat number error:" "ubx trkd5: system error"
                "ubx trkd5 sat number error:")
    # Only observation-row rejection traces; raw navigation has other units.
      gnss_receiver_replace(1 "trace(2,\"${label}" "GNSS_RX_EVENT(GNSS_RX_SATELLITE,1); trace(2,\"${label}")
  endforeach()
  gnss_receiver_replace(1 "continue; /* suppress warning for unknown glo satellite */"
    "GNSS_RX_EVENT(GNSS_RX_SATELLITE,1); continue; /* suppress warning for unknown glo satellite */")
  gnss_receiver_replace(1 "        if ((idx=sig_idx(sys,code))<0) {"
    "        if ((idx=sig_idx(sys,code))<0) {\n            GNSS_RX_EVENT(code==CODE_NONE ? GNSS_RX_SIGNAL : GNSS_RX_SLOT,1);")
  # Tail rows were not inspected. Count wire rows, never claim valid satellites.
  gnss_receiver_replace(1 "for (i=0,p+=8;i<nsat&&i<MAXOBS;i++,p+=24) {"
    "if (nsat>MAXOBS) GNSS_RX_EVENT(GNSS_RX_CAPACITY_ROWS,nsat-MAXOBS);\n    for (i=0,p+=8;i<nsat&&i<MAXOBS;i++,p+=24) {")
  gnss_receiver_replace(1 "    raw->obs.n=n;\n    return 1;\n}\n/* decode UBX-NAV-SOL"
    "    raw->obs.n=n;\n    if (i<nmeas && n>=MAXOBS) GNSS_RX_EVENT(GNSS_RX_CAPACITY_ROWS,nmeas-i);\n    return 1;\n}\n/* decode UBX-NAV-SOL")
  gnss_receiver_replace(1 "    raw->obs.n=n;\n    return 1;\n}\n/* decode UBX-TRKD5"
    "    raw->obs.n=n;\n    if (i<nch && n>=MAXOBS) GNSS_RX_EVENT(GNSS_RX_CAPACITY_ROWS,nch-i);\n    return 1;\n}\n/* decode UBX-TRKD5")
  gnss_receiver_replace(1 "    raw->obs.n=n;\n    return 1;\n}\n/* UTC 8-bit week"
    "    raw->obs.n=n;\n    if (n>=MAXOBS && p-raw->buff+len<=raw->len-2) GNSS_RX_EVENT(GNSS_RX_CAPACITY_ROWS,(raw->len-2-(p-raw->buff))/len);\n    return 1;\n}\n/* UTC 8-bit week")
elseif(receiver STREQUAL "novatel")
  gnss_receiver_replace(1 "if (tow >= 604800.0) return -1;"
    "if (tow >= 604800.0) return GNSS_RX_FAIL(GNSS_RX_TIME);")
  foreach(name decode_rawephemb decode_rawwaasframeb decode_gloephemerisb
               decode_qzssrawephemb decode_qzssrawsubframeb decode_galephemerisb
               decode_bdsephemerisb decode_navicephemerisb)
    gnss_navigation_zero(${name})
  endforeach()
  gnss_receiver_replace(1 "if (obs->n>=MAXOBS) return -1;"
    "if (obs->n>=MAXOBS) { GNSS_RX_EVENT(GNSS_RX_CAPACITY_ATTEMPTS,1); return -1; }")
  foreach(label "oem4 unknown system:" "oem4 signal type error:")
    if(label STREQUAL "oem4 unknown system:")
      set(kind GNSS_RX_SATELLITE)
    else()
      set(kind GNSS_RX_SIGNAL)
    endif()
    gnss_receiver_replace(1 "trace(2,\"${label}" "GNSS_RX_EVENT(${kind},1); trace(2,\"${label}")
  endforeach()
  gnss_receiver_replace(2 "if ((idx=checkpri(raw->opt,sys,code,idx))<0) continue;"
    "if ((idx=checkpri(raw->opt,sys,code,idx))<0) { GNSS_RX_EVENT(GNSS_RX_SLOT,1); continue; }")
  gnss_receiver_replace(2 "if (sys==SYS_GLO&&!parity) continue;"
    "if (sys==SYS_GLO&&!parity) { GNSS_RX_EVENT(GNSS_RX_QUALITY,1); continue; }")
  foreach(label "oem4 rangecmpb satellite number error:" "oem4 rangeb satellite number error:")
    gnss_receiver_replace(1 "trace(3,\"${label}" "GNSS_RX_EVENT(GNSS_RX_SATELLITE,1); trace(3,\"${label}")
  endforeach()
elseif(receiver STREQUAL "septentrio")
  gnss_receiver_replace(1 "if (ant_sel != antennaIdx) return ret;"
    "if (ant_sel != antennaIdx) { GNSS_RX_EVENT(GNSS_RX_SELECTION,1); return ret; }")
  gnss_receiver_replace(2 "        if (ant != ant_sel) {"
    "        if (ant != ant_sel) {\n            GNSS_RX_EVENT(GNSS_RX_SELECTION,1);")
  gnss_receiver_replace(2 "if (strstr(raw->opt, \"-NO_MEAS2\")) return ret;"
    "if (strstr(raw->opt, \"-NO_MEAS2\")) { GNSS_RX_EVENT(GNSS_RX_SELECTION,1); return ret; }")
  gnss_receiver_replace(1 "if (strstr(raw->opt, \"-NO_MEAS3\")) return ret;"
    "if (strstr(raw->opt, \"-NO_MEAS3\")) { GNSS_RX_EVENT(GNSS_RX_SELECTION,1); return ret; }")
  gnss_receiver_replace(1 "fabs(timediff(raw->time,sbf->meas3_reference_time)-(TOW%refEpochInterval)*.001)>1e-6)) {\n        goto invalid_meas3;"
    "fabs(timediff(raw->time,sbf->meas3_reference_time)-(TOW%refEpochInterval)*.001)>1e-6)) {\n        GNSS_RX_EVENT(GNSS_RX_REFERENCE,1); goto invalid_meas3;")
  gnss_receiver_replace(2 "if (TOW%refEpochInterval==0 || !sbf->meas3_reference_sat[navsys][svid]) goto invalid_meas3;"
    "if (TOW%refEpochInterval==0 || !sbf->meas3_reference_sat[navsys][svid]) { GNSS_RX_EVENT(GNSS_RX_REFERENCE,1); goto invalid_meas3; }")
  gnss_receiver_replace(1 "if (sig<0 || sig>=MEAS3_SIG_MAX) return -1;"
    "if (sig<0 || sig>=MEAS3_SIG_MAX) { GNSS_RX_EVENT(GNSS_RX_SIGNAL,1); return -1; }")
  gnss_receiver_replace(1 "    trace(4, \"SBF, decode_gpsrawcanav: sat=%2d\\n\", sat);\n    return 0;"
    "    trace(4, \"SBF, decode_gpsrawcanav: sat=%2d\\n\", sat);\n    return (GNSS_RX_EVENT(GNSS_RX_NAV_WAIT,1),0);")
  foreach(name decode_gpsionutc decode_gpsrawcanav decode_gpsrawcnav decode_glorawcanav
               decode_galrawfnav decode_galrawinav decode_georaw decode_cmpraw decode_navicraw
               decode_gpsnav decode_gpsalm decode_gpscnav decode_glonav decode_galnav
               decode_cmpnav decode_cmpcnav2 decode_qzssnav decode_geonav)
    gnss_navigation_zero(${name})
  endforeach()
  gnss_receiver_replace(1 "        sat = svid2sat(svid);\n        if (!sat) {"
    "        sat = svid2sat(svid);\n        if (!sat) {\n            GNSS_RX_EVENT(GNSS_RX_SATELLITE,1);")
  gnss_receiver_replace(1 "if (sig<0 || sig>SBF_MAXSIG || sig_tbl[sig][0]!=sys) return -1;"
    "if (sig<0 || sig>SBF_MAXSIG || sig_tbl[sig][0]!=sys) { GNSS_RX_EVENT(GNSS_RX_SIGNAL,1); return -1; }")
  # These are mapping attempts (including Meas3 reference lookups), not a
  # fabricated count of lost physical measurements.
  gnss_receiver_replace(1 "    return sigPriority(sys, idx, opt, code);"
    "    int selected=sigPriority(sys, idx, opt, code);\n    if (selected<0) GNSS_RX_EVENT(GNSS_RX_SLOT,1);\n    return selected;")
  gnss_receiver_replace(1 "    return sigPriority(Meas3_NavSys[sbf_navsys], idx, opt, code);"
    "    int selected=sigPriority(Meas3_NavSys[sbf_navsys], idx, opt, code);\n    if (selected<0) GNSS_RX_EVENT(*code==CODE_NONE ? GNSS_RX_SIGNAL : GNSS_RX_SLOT,1);\n    return selected;")
  gnss_receiver_replace(1 "                satNo = satno(Meas3_NavSys[navsys], Meas3_SVIDBase[navsys]+svid);"
    "                satNo = satno(Meas3_NavSys[navsys], Meas3_SVIDBase[navsys]+svid);\n                if (!satNo) GNSS_RX_EVENT(GNSS_RX_SATELLITE,1);")
  gnss_receiver_replace(1 "if (n>=MAXOBS) goto invalid_meas3;"
    "if (n>=MAXOBS) { GNSS_RX_EVENT(GNSS_RX_CAPACITY_ATTEMPTS,1); goto invalid_meas3; }")
  gnss_receiver_replace(1 "    raw->obuf.n = n;\n\n    return ret;\n}\n\nstatic int decode_measepochextra"
    "    if (i<n1 && n>=MAXOBS) GNSS_RX_EVENT(GNSS_RX_CAPACITY_ROWS,n1-i);\n    raw->obuf.n = n;\n\n    return ret;\n}\n\nstatic int decode_measepochextra")
elseif(receiver STREQUAL "rtcm3")
  gnss_receiver_replace(1 "if (dow>6 || tod>=86400.0) return -1;"
    "if (dow>6 || tod>=86400.0) return GNSS_RX_FAIL(GNSS_RX_TIME);")
  gnss_receiver_replace(2 "if (tow>=604800.0) return -1;"
    "if (tow>=604800.0) return GNSS_RX_FAIL(GNSS_RX_TIME);")
  foreach(type 1002 1004 1010 1012)
    gnss_receiver_replace(1 "trace(2,\"rtcm3 ${type} satellite number error:"
      "GNSS_RX_EVENT(GNSS_RX_SATELLITE,1); trace(2,\"rtcm3 ${type} satellite number error:")
  endforeach()
  gnss_receiver_replace(1 "if (i>=MAXOBS) return -1; /* overflow */"
    "if (i>=MAXOBS) { GNSS_RX_EVENT(GNSS_RX_CAPACITY_ATTEMPTS,1); return -1; } /* overflow */")
  # MSM cell counts: only active cells rejected at the final store gate.
  gnss_receiver_replace(1 "            if (sat&&index>=0&&idx[k]>=0) {"
    "            if (!sat) GNSS_RX_EVENT(GNSS_RX_SATELLITE,1);\n            else if (index>=0 && idx[k]<0) GNSS_RX_EVENT(code[k]==CODE_NONE ? GNSS_RX_SIGNAL : GNSS_RX_SLOT,1);\n            if (sat&&index>=0&&idx[k]>=0) {")
endif()

# A trace immediately describing a length failure is a verified reason; retain
# the existing trace and return value. Remaining checks report their exact site
# as validation_events rather than guessing a reason from return -1.
string(REGEX REPLACE "(trace\\(2,[^;]*length error[^;]*;)([ \n\r]*return -1;)"
  "GNSS_RX_EVENT(GNSS_RX_LENGTH,1); \\1\\2" _receiver_source "${_receiver_source}")

# All negative-return sites remain discoverable even when a protocol-specific
# reason is not known. Helper lookup negatives are excluded in the C sink.
# This is an event count, not an additional count of rejected frames.
string(REPLACE "return -1;" "return gnss_receiver_negative(__func__,__LINE__);" _receiver_source "${_receiver_source}")
string(REPLACE "return ret ? ret : -1;" "return (GNSS_RX_EVENT(GNSS_RX_VALIDATION,1), ret ? ret : -1);" _receiver_source "${_receiver_source}")

# Final detailed reason catalog is checked against the entire intermediate
# source. Do not silently apply line-based reasons to a changed dependency.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(_reason_script "${CMAKE_CURRENT_LIST_DIR}/receiver_diagnostic_reasons.py")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${_reason_script}" "${CMAKE_CURRENT_LIST_DIR}/receiver_diagnostic_reasons.json")
set(_reason_input "${CMAKE_CURRENT_BINARY_DIR}/rtklib_patched/${receiver}_diagnostic_input.c")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/rtklib_patched/${receiver}_diagnostic_base.c" "${_receiver_source}")
file(WRITE "${_reason_input}" "${_receiver_source}")
execute_process(COMMAND "${Python3_EXECUTABLE}" "${_reason_script}" "${receiver}" "${_reason_input}"
  RESULT_VARIABLE _reason_result ERROR_VARIABLE _reason_error)
if(NOT _reason_result EQUAL 0)
  message(FATAL_ERROR "Diagnostic classification failed: ${_reason_error}")
endif()
file(READ "${_reason_input}" _receiver_source)
