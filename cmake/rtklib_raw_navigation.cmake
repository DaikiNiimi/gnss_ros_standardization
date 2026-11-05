# G7 BDS-SIS-ICD-B1I-3.0 sections5.2.4.3/.4: WN/SOW locate transmission.
# Resolve toe/toc to the closest BDT week; the pinned D1/D2 branches reverse
# both week adjustments. Keep the SSR IODE=toc/720 policy unchanged.
set(receiver rcvraw)
set(input "${RTKLIB_ROOT}/src/rcvraw.c")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${input}")
file(READ "${input}" _receiver_source)
gnss_receiver_replace(2 [=[    if      (eph_bds.toes>sow1+302400.0) eph_bds.week++;
    else if (eph_bds.toes<sow1-302400.0) eph_bds.week--;]=]
[=[    if      (eph_bds.toes>sow1+302400.0) eph_bds.week--;
    else if (eph_bds.toes<sow1-302400.0) eph_bds.week++;]=])
set(output "${CMAKE_CURRENT_BINARY_DIR}/rtklib_patched/rcvraw.c")
set(previous "")
if(EXISTS "${output}")
  file(READ "${output}" previous)
endif()
if(NOT previous STREQUAL _receiver_source)
  file(WRITE "${output}" "${_receiver_source}")
endif()
list(REMOVE_ITEM _core_sources "${input}")
list(APPEND _core_sources "${output}")
