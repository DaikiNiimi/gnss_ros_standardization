cmake_minimum_required(VERSION 3.16)
project(GNSS_ROS_Standardization LANGUAGES C CXX)

# ==== Basics ==============================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==== ROS / ament =========================================================
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(Threads REQUIRED)

# ==== Options (core feature flags, MALIB互換) =============================
option(ENABLE_GLO "Enable GLONASS" ON)
option(ENABLE_GAL "Enable Galileo" ON)
option(ENABLE_BDS "Enable BeiDou (Compass)" ON)
option(ENABLE_QZS "Enable QZSS" ON)
option(ENABLE_SBS "Enable SBAS" ON)
set(NEXOBS "3" CACHE STRING "Extra observation channels (NEXOBS)")

option(BUILD_RTCM_DECODER_NODE "Build rtcm_decoder_node" ON)
option(BUILD_RINEX_WRITER_TOOL "Build ros2_rinex_writer (db3->RINEX)" ON)

# ==== Messages ============================================================
set(_msg_dir "${CMAKE_CURRENT_SOURCE_DIR}/msg")
set(_msgs
  "${_msg_dir}/GnssObservation.msg"
  "${_msg_dir}/GnssObservations.msg"
  "${_msg_dir}/GnssEphemeris.msg"
  "${_msg_dir}/GlonassEphemeris.msg"
  "${_msg_dir}/GnssEphemerides.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${_msgs}
  DEPENDENCIES std_msgs builtin_interfaces
)

# ==== MALIB core only =====================================================
set(MALIB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/MALIB")
if(NOT EXISTS "${MALIB_ROOT}/src/rtklib.h")
  message(FATAL_ERROR "MALIB not found. Expected: ${MALIB_ROOT}/src/rtklib.h")
endif()

# コアソース収集（app/util/test を除外）
file(GLOB _core_main "${MALIB_ROOT}/src/*.c")
file(GLOB _core_rcv  "${MALIB_ROOT}/src/rcv/*.c")
set(_core_sources ${_core_main} ${_core_rcv})

# 不要アプリを保険で除外
foreach(_maybe_app
    convbin.c rnx2rtkp.c str2str.c rtknavi.c rtkrcv.c rtkplot.c rtkgetopt.c rtksvr.c
    convkml.c ephemeris.c)
  if(EXISTS "${MALIB_ROOT}/src/${_maybe_app}")
    list(REMOVE_ITEM _core_sources "${MALIB_ROOT}/src/${_maybe_app}")
  endif()
endforeach()

list(LENGTH _core_sources _core_src_count)
if(_core_src_count EQUAL 0)
  message(FATAL_ERROR "MALIB core sources list is empty. Check MALIB tree.")
endif()

add_library(rtkcore_lib STATIC ${_core_sources})
set_target_properties(rtkcore_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(rtkcore_lib PUBLIC "${MALIB_ROOT}/src")

# 構成フラグ（MALIB/RTKLIB 共通系）
add_library(rtkcore_config INTERFACE)
target_compile_definitions(rtkcore_config INTERFACE
  $<$<BOOL:${ENABLE_GLO}>:ENAGLO>
  $<$<BOOL:${ENABLE_GAL}>:ENAGAL>
  $<$<BOOL:${ENABLE_BDS}>:ENACMP>
  $<$<BOOL:${ENABLE_QZS}>:ENAQZS>
  $<$<BOOL:${ENABLE_SBS}>:ENASBS>
  NEXOBS=${NEXOBS}
)
target_include_directories(rtkcore_config INTERFACE "${MALIB_ROOT}/src")
target_link_libraries(rtkcore_lib PUBLIC rtkcore_config)
if(UNIX AND NOT APPLE)
  target_link_libraries(rtkcore_lib PRIVATE m)
endif()

# 追加のヘッダパス（あれば）
if(EXISTS "${MALIB_ROOT}/include")
  include_directories("${MALIB_ROOT}/include")
endif()

# ==== Executables =========================================================
set(_rosidl_deps
  ${PROJECT_NAME}__rosidl_generator_c
  ${PROJECT_NAME}__rosidl_generator_cpp
  ${PROJECT_NAME}__rosidl_typesupport_c
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# rtcm_decoder_node (src/rtcm_decoder_node.cpp)
if(BUILD_RTCM_DECODER_NODE)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/rtcm_decoder_node.cpp")
    add_executable(rtcm_decoder_node src/rtcm_decoder_node.cpp)
    ament_target_dependencies(rtcm_decoder_node rclcpp)
    target_link_libraries(rtcm_decoder_node rtkcore_lib Threads::Threads)
    add_dependencies(rtcm_decoder_node ${_rosidl_deps})
    install(TARGETS rtcm_decoder_node DESTINATION lib/${PROJECT_NAME})
  else()
    message(WARNING "BUILD_RTCM_DECODER_NODE=ON but src/rtcm_decoder_node.cpp not found.")
  endif()
endif()

# ros2_rinex_writer
# 優先: src/converter/ros2_rinex_writer.cpp
# 後方互換: src/converter/rtcm_decoder_node.cpp を誤名で置いている場合に拾う
if(BUILD_RINEX_WRITER_TOOL)
  set(_writer_src "")
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/converter/ros2_rinex_writer.cpp")
    set(_writer_src "src/converter/ros2_rinex_writer.cpp")
  elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/converter/rtcm_decoder_node.cpp")
    message(WARNING "Using src/converter/rtcm_decoder_node.cpp as ros2_rinex_writer (please rename to ros2_rinex_writer.cpp).")
    set(_writer_src "src/converter/rtcm_decoder_node.cpp")
  endif()

  if(_writer_src)
    add_executable(ros2_rinex_writer ${_writer_src})
    ament_target_dependencies(ros2_rinex_writer rclcpp rcutils rosbag2_cpp rosbag2_storage)
    target_include_directories(ros2_rinex_writer PRIVATE "${MALIB_ROOT}/src")
    target_link_libraries(ros2_rinex_writer rtkcore_lib rtkcore_config)
    add_dependencies(ros2_rinex_writer ${_rosidl_deps})
    install(TARGETS ros2_rinex_writer DESTINATION lib/${PROJECT_NAME})
  else()
    message(WARNING "BUILD_RINEX_WRITER_TOOL=ON but no writer source found in src/converter/.")
  endif()
endif()

# ==== Export & Install =====================================================
ament_export_dependencies(rosidl_default_runtime)
ament_package()

