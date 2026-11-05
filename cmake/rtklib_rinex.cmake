# Keep phase/Doppler/CN0-only RINEX signals identifiable. The pinned RTKLIB
# sets obsd_t.code only in the pseudorange branch. Do not edit the submodule:
# compile a narrowly patched build-tree copy and fail if its context changes.
set(_rinex_input "${RTKLIB_ROOT}/src/rinex.c")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_rinex_input}")
file(READ "${_rinex_input}" _rinex_source)
set(_rinex_marker "        switch (ind->type[i]) {")
string(FIND "${_rinex_source}" "${_rinex_marker}" _rinex_offset)
if(_rinex_offset LESS 0)
  message(FATAL_ERROR "RTKLIB RINEX signal-identity patch no longer matches; review the dependency update")
endif()
string(LENGTH "${_rinex_marker}" _rinex_marker_size)
math(EXPR _rinex_after "${_rinex_offset} + ${_rinex_marker_size}")
string(SUBSTRING "${_rinex_source}" ${_rinex_after} -1 _rinex_tail)
string(FIND "${_rinex_tail}" "${_rinex_marker}" _rinex_duplicate)
if(NOT _rinex_duplicate EQUAL -1)
  message(FATAL_ERROR "RTKLIB RINEX signal-identity patch has ambiguous context")
endif()
set(_rinex_replacement [=[        /* Preserve signal identity when P is missing. A present P retains
           its original priority; the case-0 branch below still assigns it. */
        if (!obs->code[p[i]]) obs->code[p[i]]=ind->code[i];
        switch (ind->type[i]) {]=])
string(REPLACE "${_rinex_marker}" "${_rinex_replacement}" _rinex_patched "${_rinex_source}")
# Opt-in for the ROS converter only: preserve blank fields as NaN before the
# backend collapses them to numeric zero. The default RTKLIB solver/API keeps
# its zero-sentinel convention. Explicit numeric zeros stay numeric: RINEX
# permits zero to mean missing, so their intent cannot be recovered generally.
function(gnss_rinex_replace before after)
  set(expected 1)
  if(ARGC GREATER 2)
    set(expected ${ARGV2})
  endif()
  string(FIND "${_rinex_patched}" "${before}" offset)
  string(REPLACE "${before}" "" removed "${_rinex_patched}")
  string(LENGTH "${_rinex_patched}" original_size)
  string(LENGTH "${removed}" removed_size)
  string(LENGTH "${before}" pattern_size)
  math(EXPR count "(${original_size} - ${removed_size}) / ${pattern_size}")
  if(NOT count EQUAL expected)
    message(FATAL_ERROR "RTKLIB RINEX presence patch expected ${expected} matches, found ${count}: ${before}")
  endif()
  string(REPLACE "${before}" "${after}" result "${_rinex_patched}")
  set(_rinex_patched "${result}" PARENT_SCOPE)
endfunction()
gnss_rinex_replace("sigind_t *index, obsd_t *obs)"
                  "sigind_t *index, obsd_t *obs, int ros_fields)")
gnss_rinex_replace("decode_obsdata(fp,buff,ver,mask,index,data+n)"
                  "decode_obsdata(fp,buff,ver,mask,index,data+n,strstr(opt,\"-ROSOBS\")!=NULL)")
gnss_rinex_replace("    uint8_t lli[MAXOBSTYPE]={0};"
                  "    uint8_t lli[MAXOBSTYPE]={0}, present[MAXOBSTYPE]={0};")
gnss_rinex_replace("lli[i]=(uint8_t)str2num(buff,j+14,1)&3;"
                  "lli[i]=(uint8_t)str2num(buff,j+14,1)&(ros_fields ? 7 : 3);")
gnss_rinex_replace("            val[i]=str2num(buff,j,14)+ind->shift[i];" [=[            for (int column=j; column<j+14 && column<(int)strlen(buff); ++column) {
                if (buff[column]!=' ' && buff[column]!='\r' && buff[column]!='\n') present[i]=1;
            }
            val[i]=str2num(buff,j,14)+((!ros_fields || present[i]) ? ind->shift[i] : 0.0);]=])
gnss_rinex_replace("        obs->P[i]=obs->L[i]=0.0;"
                  "        obs->P[i]=obs->L[i]=ros_fields ? NAN : 0.0;")
gnss_rinex_replace("        obs->D[i]=obs->SNR[i]=0.0;"
                  "        obs->D[i]=obs->SNR[i]=ros_fields ? NAN : 0.0;")
foreach(array k l r)
  foreach(index 0 1)
    gnss_rinex_replace("val[${array}[${index}]]==0.0"
      "(ros_fields ? !present[${array}[${index}]] : val[${array}[${index}]]==0.0)" 2)
    gnss_rinex_replace("val[${array}[${index}]]!=0.0"
      "(ros_fields ? present[${array}[${index}]] : val[${array}[${index}]]!=0.0)")
  endforeach()
endforeach()
gnss_rinex_replace("        if (p[i]<0||(val[i]==0.0&&lli[i]==0)) continue;"
 [=[        if (p[i]<0||((ros_fields ? !present[i] : val[i]==0.0)&&lli[i]==0)) continue;
        if (ros_fields && !present[i]) val[i]=NAN;]=])
set(_rinex_output "${CMAKE_CURRENT_BINARY_DIR}/rtklib_patched/rinex.c")
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/rtklib_patched")
set(_rinex_previous "")
if(EXISTS "${_rinex_output}")
  file(READ "${_rinex_output}" _rinex_previous)
endif()
if(NOT _rinex_previous STREQUAL _rinex_patched)
  file(WRITE "${_rinex_output}" "${_rinex_patched}")
endif()
list(REMOVE_ITEM _core_sources "${_rinex_input}")
list(APPEND _core_sources "${_rinex_output}")
