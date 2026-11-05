# All translation units and installed consumers must see the same MAXSAT ABI.
# Merely adding -I for a patched header is insufficient: vendor C files include
# "rtklib.h" from their own directory first. Stage the source without modifying
# the submodule, tracking inputs and avoiding writes to unchanged copies.
set(RTKLIB_VENDOR_ROOT "${RTKLIB_ROOT}")
set(RTKLIB_ROOT "${CMAKE_CURRENT_BINARY_DIR}/rtklib_source")
file(GLOB_RECURSE _rtklib_inputs CONFIGURE_DEPENDS
  RELATIVE "${RTKLIB_VENDOR_ROOT}" "${RTKLIB_VENDOR_ROOT}/src/*.c" "${RTKLIB_VENDOR_ROOT}/src/*.h")
foreach(_input IN LISTS _rtklib_inputs)
  if(NOT _input STREQUAL "src/rtklib.h")
    configure_file("${RTKLIB_VENDOR_ROOT}/${_input}" "${RTKLIB_ROOT}/${_input}" COPYONLY)
  endif()
endforeach()
set(_rtklib_header_input "${RTKLIB_VENDOR_ROOT}/src/rtklib.h")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_rtklib_header_input}")
file(READ "${_rtklib_header_input}" _rtklib_header)
set(_prn_before "#define MAXPRNCMP   50                  /* max satellite sat number of BeiDou */")
string(REGEX MATCHALL "#define MAXPRNCMP   50[^\n]*" _prn_matches "${_rtklib_header}")
list(LENGTH _prn_matches _prn_count)
if(NOT _prn_count EQUAL 1 OR NOT _prn_matches STREQUAL _prn_before)
  message(FATAL_ERROR "RTKLIB BeiDou PRN patch no longer matches; review dependency update")
endif()
string(REPLACE "${_prn_before}"
  "#define MAXPRNCMP   63                  /* BeiDou C01..C63; package ABI */"
  _rtklib_header "${_rtklib_header}")
set(_rtklib_header_output "${RTKLIB_ROOT}/src/rtklib.h")
set(_previous_header "")
if(EXISTS "${_rtklib_header_output}")
  file(READ "${_rtklib_header_output}" _previous_header)
endif()
if(NOT _previous_header STREQUAL _rtklib_header)
  file(WRITE "${_rtklib_header_output}" "${_rtklib_header}")
endif()
