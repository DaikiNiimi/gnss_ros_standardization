cmake_minimum_required(VERSION 3.16)
project(gnss_ros_standardization VERSION 1.0.0 LANGUAGES C CXX)
# NOTE: Keep VERSION in sync with package.xml. Used by tools' --version flag.

# Basics
# A plain `colcon build` passes no build type, which compiles without
# optimization; the FGO nodes then cannot keep up with the receiver rate.
get_property(_multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(NOT _multi_config AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  message(STATUS "No CMAKE_BUILD_TYPE given; defaulting to Release")
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(Threads REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cv_bridge QUIET)
find_package(image_transport QUIET)
find_package(OpenCV QUIET)

# Options
option(ENABLE_GLO "Enable GLONASS" ON)
option(ENABLE_GAL "Enable Galileo" ON)
option(ENABLE_BDS "Enable BeiDou (Compass)" ON)
option(ENABLE_QZS "Enable QZSS" ON)
option(ENABLE_SBS "Enable SBAS" ON)
option(ENABLE_IRN "Enable IRNSS" ON)
set(NEXOBS "3" CACHE STRING "Extra observation channels (NEXOBS)")
option(BUILD_RTCM_DECODER_NODE "Build rtcm_decoder_node" ON)
option(BUILD_RINEX_WRITER_TOOL "Build rosbag_to_rinex (ROS bag to RINEX)" ON)

# Messages
set(_msgs
  msg/GnssObservation.msg
  msg/GnssObservations.msg
  msg/GnssEphemeris.msg
  msg/GlonassEphemeris.msg
  msg/GnssEphemerides.msg
  msg/GnssSolution.msg
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${_msgs}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# RTKLIB core
set(RTKLIB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/RTKLIB")
if(NOT EXISTS "${RTKLIB_ROOT}/src/rtklib.h")
  message(FATAL_ERROR "RTKLIB not found: ${RTKLIB_ROOT}/src/rtklib.h")
endif()
include(cmake/rtklib_source.cmake)
file(GLOB _core_main "${RTKLIB_ROOT}/src/*.c")
file(GLOB _core_rcv "${RTKLIB_ROOT}/src/rcv/*.c")
set(_core_sources ${_core_main} ${_core_rcv})

set(_apps_to_exclude
  convbin.c rnx2rtkp.c str2str.c rtknavi.c rtkrcv.c rtkplot.c rtkgetopt.c rtksvr.c
  convkml.c
)
foreach(_maybe_app IN LISTS _apps_to_exclude)
  if(EXISTS "${RTKLIB_ROOT}/src/${_maybe_app}")
    list(REMOVE_ITEM _core_sources "${RTKLIB_ROOT}/src/${_maybe_app}")
  endif()
endforeach()

# Exclude receiver files that reference nav_t::leaps (removed in this RTKLIB version)
# and are not used by this package (ComNav / Tersus receivers).
list(REMOVE_ITEM _core_sources
  "${RTKLIB_ROOT}/src/rcv/comnav.c"
  "${RTKLIB_ROOT}/src/rcv/tersus.c"
)

list(LENGTH _core_sources _core_src_count)
if(_core_src_count EQUAL 0)
  message(FATAL_ERROR "RTKLIB core sources list is empty")
endif()

include(cmake/rtklib_rinex.cmake)
include(cmake/rtklib_receivers.cmake)
add_library(rtkcore_lib STATIC ${_core_sources} src/utils/receiver_diagnostics.c)
target_include_directories(rtkcore_lib PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
set_target_properties(rtkcore_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  target_compile_options(rtkcore_lib PRIVATE "-w")
endif()
target_include_directories(rtkcore_lib PUBLIC
  $<BUILD_INTERFACE:${RTKLIB_ROOT}/src>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}/rtklib>
)

add_library(rtkcore_config INTERFACE)
target_compile_definitions(rtkcore_config INTERFACE
  $<$<BOOL:${ENABLE_GLO}>:ENAGLO>
  $<$<BOOL:${ENABLE_GAL}>:ENAGAL>
  $<$<BOOL:${ENABLE_BDS}>:ENACMP>
  $<$<BOOL:${ENABLE_QZS}>:ENAQZS>
  $<$<BOOL:${ENABLE_SBS}>:ENASBS>
  $<$<BOOL:${ENABLE_IRN}>:ENAIRN>
  NEXOBS=${NEXOBS}
)
target_include_directories(rtkcore_config INTERFACE
  $<BUILD_INTERFACE:${RTKLIB_ROOT}/src>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}/rtklib>
)
target_link_libraries(rtkcore_lib PUBLIC rtkcore_config Threads::Threads)
# Generator-independent preprocessing evidence (also works with Ninja).
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/protocol_compile_definitions.txt"
  CONTENT "$<JOIN:$<TARGET_PROPERTY:rtkcore_lib,COMPILE_DEFINITIONS>,\n>\n")
if(UNIX AND NOT APPLE)
  target_link_libraries(rtkcore_lib PRIVATE m)
endif()

# Generated headers include paths
set(_gen_include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

# rosidl build order
set(_rosidl_deps
  ${PROJECT_NAME}__rosidl_generator_c
  ${PROJECT_NAME}__rosidl_generator_cpp
  ${PROJECT_NAME}__rosidl_typesupport_c
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

include_directories(include)

# gnss_utils
set(_gnss_utils_sources src/utils/gnss_utils.cpp)
list(APPEND _gnss_utils_sources src/utils/gnss_preprocessor.cpp)
add_library(gnss_utils ${_gnss_utils_sources})
target_include_directories(gnss_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(gnss_utils PRIVATE "${RTKLIB_ROOT}/src")
# We need rtkcore_lib for rtklib types used in gnss_utils
target_link_libraries(gnss_utils rtkcore_lib rtkcore_config Eigen3::Eigen ${cpp_typesupport_target})
target_compile_features(gnss_utils PUBLIC cxx_std_17)
ament_target_dependencies(gnss_utils rclcpp std_msgs geometry_msgs sensor_msgs
  nav_msgs builtin_interfaces rosbag2_storage)

install(TARGETS gnss_utils rtkcore_lib rtkcore_config
  EXPORT export_${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES "${RTKLIB_ROOT}/src/rtklib.h"
  DESTINATION include/${PROJECT_NAME}/rtklib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

# Pipeline launch files (see launch/README.md). gnss_pipeline_common.py rides
# along as a plain module: the launch files put this directory on sys.path, so
# it resolves identically from the source tree and from the install tree.
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# rtcm_decoder_node
if(BUILD_RTCM_DECODER_NODE)
  add_executable(rtcm_decoder_node src/decoders/rtcm_decoder_node.cpp)
  target_include_directories(rtcm_decoder_node PUBLIC ${_gen_include})
  ament_target_dependencies(rtcm_decoder_node rclcpp std_msgs builtin_interfaces)
  target_link_libraries(rtcm_decoder_node gnss_utils Threads::Threads)
  add_dependencies(rtcm_decoder_node ${_rosidl_deps})
  install(TARGETS rtcm_decoder_node DESTINATION lib/${PROJECT_NAME})
endif()

# ubx_decoder_node
option(BUILD_UBX_DECODER_NODE "Build ubx_decoder_node" ON)
if(BUILD_UBX_DECODER_NODE)
  add_executable(ubx_decoder_node src/decoders/ubx_decoder_node.cpp)
  target_include_directories(ubx_decoder_node PUBLIC ${_gen_include})
  ament_target_dependencies(ubx_decoder_node rclcpp std_msgs geometry_msgs sensor_msgs builtin_interfaces)
  target_link_libraries(ubx_decoder_node gnss_utils Threads::Threads)
  add_dependencies(ubx_decoder_node ${_rosidl_deps})
  install(TARGETS ubx_decoder_node DESTINATION lib/${PROJECT_NAME})
endif()

# ubx_driver_node (u-blox driver with configuration)
option(BUILD_UBX_DRIVER_NODE "Build ubx_driver_node (u-blox driver)" ON)
if(BUILD_UBX_DRIVER_NODE)
  add_executable(ubx_driver_node src/drivers/ubx_driver_node.cpp)
  target_include_directories(ubx_driver_node PUBLIC ${_gen_include})
  ament_target_dependencies(ubx_driver_node rclcpp std_msgs geometry_msgs sensor_msgs builtin_interfaces)
  target_link_libraries(ubx_driver_node gnss_utils Threads::Threads)
  add_dependencies(ubx_driver_node ${_rosidl_deps})
  install(TARGETS ubx_driver_node DESTINATION lib/${PROJECT_NAME})
endif()

# sbf_decoder_node (Septentrio)
option(BUILD_SBF_DECODER_NODE "Build sbf_decoder_node (Septentrio)" ON)
if(BUILD_SBF_DECODER_NODE)
  add_executable(sbf_decoder_node src/decoders/sbf_decoder_node.cpp)
  target_include_directories(sbf_decoder_node PUBLIC ${_gen_include})
  ament_target_dependencies(sbf_decoder_node rclcpp std_msgs geometry_msgs sensor_msgs builtin_interfaces)
  target_link_libraries(sbf_decoder_node gnss_utils Threads::Threads)
  add_dependencies(sbf_decoder_node ${_rosidl_deps})
  install(TARGETS sbf_decoder_node DESTINATION lib/${PROJECT_NAME})
endif()

# sbf_driver_node (Septentrio driver with configuration)
option(BUILD_SBF_DRIVER_NODE "Build sbf_driver_node (Septentrio driver)" ON)
if(BUILD_SBF_DRIVER_NODE)
  add_executable(sbf_driver_node src/drivers/sbf_driver_node.cpp)
  target_include_directories(sbf_driver_node PUBLIC ${_gen_include})
  ament_target_dependencies(sbf_driver_node rclcpp std_msgs geometry_msgs sensor_msgs builtin_interfaces)
  target_link_libraries(sbf_driver_node gnss_utils Threads::Threads)
  add_dependencies(sbf_driver_node ${_rosidl_deps})
  install(TARGETS sbf_driver_node DESTINATION lib/${PROJECT_NAME})
endif()

# novatel_decoder_node (NovAtel OEM3/OEM4/OEM7)
option(BUILD_NOVATEL_DECODER_NODE "Build novatel_decoder_node (NovAtel)" ON)
if(BUILD_NOVATEL_DECODER_NODE)
  add_executable(novatel_decoder_node src/decoders/novatel_decoder_node.cpp)
  target_include_directories(novatel_decoder_node PUBLIC ${_gen_include})
  ament_target_dependencies(novatel_decoder_node rclcpp std_msgs geometry_msgs sensor_msgs builtin_interfaces)
  target_link_libraries(novatel_decoder_node gnss_utils Threads::Threads)
  add_dependencies(novatel_decoder_node ${_rosidl_deps})
  install(TARGETS novatel_decoder_node DESTINATION lib/${PROJECT_NAME})
endif()

# novatel_driver_node (NovAtel driver with configuration)
option(BUILD_NOVATEL_DRIVER_NODE "Build novatel_driver_node (NovAtel driver)" ON)
if(BUILD_NOVATEL_DRIVER_NODE)
  add_executable(novatel_driver_node src/drivers/novatel_driver_node.cpp)
  target_include_directories(novatel_driver_node PUBLIC ${_gen_include})
  ament_target_dependencies(novatel_driver_node rclcpp std_msgs geometry_msgs sensor_msgs builtin_interfaces)
  target_link_libraries(novatel_driver_node gnss_utils Threads::Threads)
  add_dependencies(novatel_driver_node ${_rosidl_deps})
  install(TARGETS novatel_driver_node DESTINATION lib/${PROJECT_NAME})
endif()

# rosbag_to_rinex
if(BUILD_RINEX_WRITER_TOOL)
  add_executable(rosbag_to_rinex src/converter/rosbag_to_rinex.cpp)
  target_include_directories(rosbag_to_rinex PUBLIC ${_gen_include})
  target_compile_definitions(rosbag_to_rinex PRIVATE PACKAGE_VERSION="${PROJECT_VERSION}")
  ament_target_dependencies(rosbag_to_rinex rclcpp rcutils rosbag2_cpp rosbag2_storage std_msgs builtin_interfaces)
  target_include_directories(rosbag_to_rinex PRIVATE "${RTKLIB_ROOT}/src")
  target_link_libraries(rosbag_to_rinex gnss_utils)
  add_dependencies(rosbag_to_rinex ${_rosidl_deps})
  install(TARGETS rosbag_to_rinex DESTINATION lib/${PROJECT_NAME})
endif()

# rinex_to_rosbag
add_executable(rinex_to_rosbag src/converter/rinex_to_rosbag.cpp)
target_include_directories(rinex_to_rosbag PUBLIC ${_gen_include})
ament_target_dependencies(rinex_to_rosbag rclcpp rosbag2_cpp rosbag2_storage std_msgs sensor_msgs nav_msgs builtin_interfaces)
target_link_libraries(rinex_to_rosbag gnss_utils)
add_dependencies(rinex_to_rosbag ${_rosidl_deps})
install(TARGETS rinex_to_rosbag DESTINATION lib/${PROJECT_NAME})

# rosbag_to_pos (GnssSolution -> RTKLIB .pos)
add_executable(rosbag_to_pos src/converter/rosbag_to_pos.cpp)
target_include_directories(rosbag_to_pos PUBLIC ${_gen_include})
target_compile_definitions(rosbag_to_pos PRIVATE PACKAGE_VERSION="${PROJECT_VERSION}")
target_include_directories(rosbag_to_pos PRIVATE "${RTKLIB_ROOT}/src")
ament_target_dependencies(rosbag_to_pos rclcpp rosbag2_cpp rosbag2_storage std_msgs geometry_msgs builtin_interfaces)
target_link_libraries(rosbag_to_pos gnss_utils)
add_dependencies(rosbag_to_pos ${_rosidl_deps})
install(TARGETS rosbag_to_pos DESTINATION lib/${PROJECT_NAME})

# pos_to_rosbag (RTKLIB .pos -> GnssSolution bag)
add_executable(pos_to_rosbag src/converter/pos_to_rosbag.cpp)
target_include_directories(pos_to_rosbag PUBLIC ${_gen_include})
target_include_directories(pos_to_rosbag PRIVATE "${RTKLIB_ROOT}/src")
ament_target_dependencies(pos_to_rosbag rclcpp rosbag2_cpp rosbag2_storage std_msgs geometry_msgs builtin_interfaces)
target_link_libraries(pos_to_rosbag gnss_utils)
add_dependencies(pos_to_rosbag ${_rosidl_deps})
install(TARGETS pos_to_rosbag DESTINATION lib/${PROJECT_NAME})

# single point positioning
option(BUILD_SPP_NODE "Build single_point_positioning (RTKLIB pntpos)" ON)

if(BUILD_SPP_NODE)
  add_executable(single_point_positioning src/positioning/spp/single_point_positioning.cpp)
  target_include_directories(single_point_positioning PUBLIC ${_gen_include})
  ament_target_dependencies(single_point_positioning rclcpp std_msgs builtin_interfaces)
  target_link_libraries(single_point_positioning
    gnss_utils
    Threads::Threads)
  add_dependencies(single_point_positioning ${_rosidl_deps})
  install(TARGETS single_point_positioning DESTINATION lib/${PROJECT_NAME})
endif()
# real time kinematic
option(BUILD_RTK_NODE "Build real_time_kinematic (RTKLIB rtkpos)" ON)

if(BUILD_RTK_NODE)
  add_executable(real_time_kinematic src/positioning/rtk/real_time_kinematic.cpp)
  target_include_directories(real_time_kinematic PUBLIC ${_gen_include})
  ament_target_dependencies(real_time_kinematic rclcpp std_msgs builtin_interfaces)
  target_link_libraries(real_time_kinematic
    gnss_utils
    Threads::Threads)
  add_dependencies(real_time_kinematic ${_rosidl_deps})
  install(TARGETS real_time_kinematic DESTINATION lib/${PROJECT_NAME})
endif()

# gnss_imu_kalman_filter (GNSS/IMU/WheelSpeed Kalman Filter)
option(BUILD_EKF_NODE "Build gnss_imu_kalman_filter (GNSS/IMU Kalman Filter)" ON)
if(BUILD_EKF_NODE)
  add_executable(gnss_imu_kalman_filter src/positioning/ekf/gnss_imu_kalman_filter.cpp)
  target_include_directories(gnss_imu_kalman_filter PUBLIC ${_gen_include})
  ament_target_dependencies(gnss_imu_kalman_filter rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs builtin_interfaces)
  target_link_libraries(gnss_imu_kalman_filter gnss_utils Eigen3::Eigen Threads::Threads)
  add_dependencies(gnss_imu_kalman_filter ${_rosidl_deps})
  install(TARGETS gnss_imu_kalman_filter DESTINATION lib/${PROJECT_NAME})
endif()

# gnss_visualizer
if(cv_bridge_FOUND AND image_transport_FOUND AND OpenCV_FOUND)
  add_executable(gnss_visualizer src/utils/gnss_visualizer.cpp)
  target_include_directories(gnss_visualizer PUBLIC ${_gen_include})
  ament_target_dependencies(gnss_visualizer rclcpp std_msgs sensor_msgs cv_bridge image_transport builtin_interfaces)
  target_link_libraries(gnss_visualizer gnss_utils ${OpenCV_LIBRARIES})
  add_dependencies(gnss_visualizer ${_rosidl_deps})
  install(TARGETS gnss_visualizer DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "Skipping gnss_visualizer: cv_bridge, image_transport, or OpenCV not found.")
endif()

# FGO uses repository-defined GNSS factors and the GTSAM nonlinear factor API.
option(BUILD_FGO_NODES "Build gnss_fgo and gnss_imu_fgo (requires the documented GTSAM)" OFF)
if(BUILD_FGO_NODES)
  find_package(GTSAM REQUIRED)
  include(CheckCXXSourceCompiles)
  set(CMAKE_REQUIRED_LIBRARIES gtsam)
  check_cxx_source_compiles("
    #include <gtsam/nonlinear/NoiseModelFactorN.h>
    #include <gtsam/nonlinear/NonlinearFactor.h>
    int main() {
      gtsam::OptionalMatrixVecType jacobians = nullptr;
      using Factor = gtsam::NoiseModelFactorN<double>;
      return jacobians != nullptr || sizeof(Factor*) == 0;
    }" GNSS_GTSAM_FACTOR_API_SUPPORTED)
  unset(CMAKE_REQUIRED_LIBRARIES)
  if(NOT GNSS_GTSAM_FACTOR_API_SUPPORTED)
    message(FATAL_ERROR "GTSAM must provide NoiseModelFactorN and OptionalMatrixVecType; see the pinned FGO build instructions.")
  endif()

  add_executable(gnss_fgo src/positioning/fgo/gnss_fgo.cpp)
  target_include_directories(gnss_fgo PUBLIC ${_gen_include})
  ament_target_dependencies(gnss_fgo rclcpp std_msgs builtin_interfaces sensor_msgs)
  target_link_libraries(gnss_fgo fgo_engine gnss_utils ${cpp_typesupport_target} Threads::Threads)
  add_dependencies(gnss_fgo ${_rosidl_deps})
  install(TARGETS gnss_fgo DESTINATION lib/${PROJECT_NAME})

  set(CMAKE_REQUIRED_LIBRARIES gtsam)
  check_cxx_source_compiles("
    #include <gtsam/navigation/CombinedImuFactor.h>
    int main() {
      auto p = gtsam::PreintegrationCombinedParams::MakeSharedU(9.81);
      p->setImuFactorErrorMode(gtsam::ImuFactorErrorMode::Logmap);
    }" GNSS_GTSAM_LOGMAP_IMU_SUPPORTED)
  unset(CMAKE_REQUIRED_LIBRARIES)
  if(NOT GNSS_GTSAM_LOGMAP_IMU_SUPPORTED)
    message(FATAL_ERROR "The FGO nodes need a GTSAM with ImuFactorErrorMode::Logmap (borglab/gtsam develop after 2026-09; see cmake/build_fgo_dependency.sh).")
  endif()
  add_library(fgo_engine STATIC
    src/positioning/fgo/engine.cpp
    src/positioning/fgo/preprocessor.cpp)
  target_link_libraries(fgo_engine PUBLIC gtsam)
  target_include_directories(fgo_engine PUBLIC src/positioning/fgo)
  add_executable(gnss_imu_fgo src/positioning/fgo/gnss_imu_fgo.cpp)
  target_include_directories(gnss_imu_fgo PRIVATE ${_gen_include})
  ament_target_dependencies(gnss_imu_fgo rclcpp std_msgs sensor_msgs nav_msgs builtin_interfaces)
  target_link_libraries(gnss_imu_fgo fgo_engine gnss_utils ${cpp_typesupport_target} Threads::Threads)
  add_dependencies(gnss_imu_fgo ${_rosidl_deps})
  install(TARGETS gnss_imu_fgo DESTINATION lib/${PROJECT_NAME})
  add_executable(test_fgo_ros_adapter EXCLUDE_FROM_ALL test/fgo/test_ros_adapter.cpp)
  target_include_directories(test_fgo_ros_adapter PRIVATE ${_gen_include})
  ament_target_dependencies(test_fgo_ros_adapter sensor_msgs builtin_interfaces)
  target_link_libraries(test_fgo_ros_adapter fgo_engine gnss_utils ${cpp_typesupport_target})
  add_dependencies(test_fgo_ros_adapter ${_rosidl_deps})

  # Embed GTSAM's library directory in the installed FGO binaries so they run
  # without LD_LIBRARY_PATH even when GTSAM lives in a non-system prefix
  # (e.g. $HOME/gtsam-install). CMake excludes dependencies inside the source or
  # build tree from INSTALL_RPATH_USE_LINK_PATH; include GTSAM explicitly too.
  # `--disable-new-dtags` emits DT_RPATH instead of DT_RUNPATH: DT_RPATH is also
  # searched for GTSAM's own transitive deps (libmetis-gtsam / libcephes-gtsam,
  # which libgtsam.so carries no runpath for), whereas DT_RUNPATH would not
  # cascade to them and the node would fail to load without LD_LIBRARY_PATH.
  foreach(_fgo_exe gnss_fgo gnss_imu_fgo)
    if(TARGET ${_fgo_exe})
      set_target_properties(${_fgo_exe} PROPERTIES
        INSTALL_RPATH_USE_LINK_PATH ON
        INSTALL_RPATH "$<TARGET_FILE_DIR:gtsam>")
      target_link_options(${_fgo_exe} PRIVATE "-Wl,--disable-new-dtags")
    endif()
  endforeach()
endif()

# Tests: missing test sources are configuration errors.
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_executable(make_rtcm_fixture test/make_rtcm_fixture.cpp)
  target_link_libraries(make_rtcm_fixture rtkcore_lib rtkcore_config)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_test(test_rtcm_relay_names
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_rtcm_relay_names.py"
      --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
    ENV "PYTHONDONTWRITEBYTECODE=1" "ROS_LOCALHOST_ONLY=1" TIMEOUT 120)
  ament_add_test(test_release
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_release.py"
            --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
    ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 240)
  ament_add_test(test_launch_settings
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_launch_settings.py"
    ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 30)
  ament_add_test(test_parameter_names
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_parameter_names.py"
    ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 30)
  ament_add_test(test_diagnostic_reasons
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/check_diagnostic_reasons.py"
      --build "${CMAKE_CURRENT_BINARY_DIR}"
    ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 30)
  ament_add_test(test_diagnostic_catalog
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/check_diagnostic_catalog.py"
      --build "${CMAKE_CURRENT_BINARY_DIR}"
    ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 30)
  ament_add_test(test_rinex_audit
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_rinex_audit.py"
    ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 30)
  if(BUILD_UBX_DECODER_NODE AND BUILD_UBX_DRIVER_NODE AND BUILD_SBF_DECODER_NODE
      AND BUILD_SBF_DRIVER_NODE AND BUILD_NOVATEL_DECODER_NODE AND BUILD_NOVATEL_DRIVER_NODE AND BUILD_RTCM_DECODER_NODE)
    ament_add_test(test_beidou_raw_specification
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_beidou_raw_specification.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 180)
    ament_add_test(test_navic_raw_specification
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_navic_raw_specification.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 120)
    ament_add_test(test_galileo_raw_specification
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_galileo_raw_specification.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 180)
    ament_add_test(test_glonass_raw_specification
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_glonass_raw_specification.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 120)
    ament_add_test(test_lnav_specification
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_lnav_specification.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 240)
    ament_add_test(test_sbf_navigation_specification
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_sbf_navigation_specification.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 180)
    ament_add_test(test_oem_navigation_specification
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_oem_navigation_specification.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 120)
    ament_add_test(test_meas3_reference
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_meas3_reference.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 120)
    ament_add_test(test_rtcm_auxiliary_reference
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_rtcm_auxiliary_reference.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 120)
    ament_add_test(test_ubx_legacy_specification
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_ubx_legacy_specification.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 120)
    ament_add_test(test_imu_specification
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_imu_specification.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 120)
    ament_add_test(test_pvt_specification
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_pvt_specification.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 240)
    ament_add_test(test_ubx_configuration_specification
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_ubx_configuration_specification.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 120)
    ament_add_test(test_ubx_time_specification
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_ubx_time_specification.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 120)
    ament_add_test(test_auxiliary_specification
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_auxiliary_specification.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 120)
    ament_add_test(test_observation_signal_specification
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_observation_signal_specification.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 120)
    ament_add_test(test_ssr_output_policy
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_ssr_output_policy.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 120)
    ament_add_test(test_ubx_navigation_specification
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_ubx_navigation_specification.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 120)
    ament_add_test(test_nmea_specification
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_nmea_specification.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 120)
    ament_add_test(test_rtcm_msm_identity
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_rtcm_msm_identity.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 120)
    ament_add_test(test_sbf_measextra_specification
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_sbf_measextra_specification.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 120)
    ament_add_test(test_rtcm_msm_boundaries
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_rtcm_msm_boundaries.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 240)
    ament_add_test(test_rtcm_msm_states
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_rtcm_msm_states.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 600)
    ament_add_test(test_rtcm_navigation_reference
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_rtcm_navigation_reference.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 120)
    ament_add_test(test_rtcm_reference
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_rtcm_reference.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 120)
    ament_add_test(test_receiver_diagnostics
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_receiver_diagnostics.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 600)
    ament_add_test(test_protocol_specification
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_protocol_specification.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 180)
    ament_add_test(test_driver_configuration
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_driver_configuration.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 600)
    ament_add_test(test_receiver_streams
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_receiver_streams.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 600)
    ament_add_test(test_receiver_observations
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_receiver_observations.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 180)
  endif()
  if(TARGET sbf_driver_node AND TARGET sbf_decoder_node
      AND TARGET ubx_driver_node AND TARGET ubx_decoder_node
      AND TARGET novatel_driver_node AND TARGET novatel_decoder_node AND TARGET rtcm_decoder_node)
    ament_add_test(test_receiver_sessions
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_receiver_sessions.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 420)
    ament_add_test(test_decoder_boundaries
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_decoder_boundaries.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 180)
  endif()
  if(TARGET pos_to_rosbag AND TARGET rosbag_to_pos AND TARGET rosbag_to_rinex AND TARGET rinex_to_rosbag)
    ament_add_test(test_converter_roundtrip
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_converter_roundtrip.py"
        --bin-dir "${CMAKE_CURRENT_BINARY_DIR}"
      ENV "PYTHONDONTWRITEBYTECODE=1" TIMEOUT 180)
  endif()

  ament_add_gtest(test_navigation_record_key test/test_navigation_record_key.cpp)
  target_link_libraries(test_navigation_record_key rtkcore_lib rtkcore_config)

  ament_add_gtest(test_tcp_nmea_server test/test_tcp_nmea_server.cpp)
  target_include_directories(test_tcp_nmea_server PRIVATE include)
  ament_target_dependencies(test_tcp_nmea_server rclcpp)
  set_tests_properties(test_tcp_nmea_server PROPERTIES TIMEOUT 20)

  ament_add_gtest(test_receiver_health test/test_receiver_health.cpp)
  target_include_directories(test_receiver_health PRIVATE include)
  ament_target_dependencies(test_receiver_health rclcpp)
  target_link_libraries(test_receiver_health rtkcore_lib)

  ament_add_gtest(test_stream_transport test/test_stream_transport.cpp)
  target_include_directories(test_stream_transport PUBLIC ${_gen_include})
  target_link_libraries(test_stream_transport gnss_utils)
  ament_target_dependencies(test_stream_transport rclcpp std_msgs builtin_interfaces)
  add_dependencies(test_stream_transport ${_rosidl_deps})
  set_tests_properties(test_stream_transport PROPERTIES TIMEOUT 30)

  ament_add_gtest(test_ephemeris_contract test/test_ephemeris_contract.cpp)
  target_include_directories(test_ephemeris_contract PUBLIC ${_gen_include})
  target_link_libraries(test_ephemeris_contract gnss_utils)
  ament_target_dependencies(test_ephemeris_contract rclcpp std_msgs builtin_interfaces)
  add_dependencies(test_ephemeris_contract ${_rosidl_deps})

  ament_add_gtest(test_observation_validation test/test_observation_validation.cpp)
  target_include_directories(test_observation_validation PUBLIC ${_gen_include})
  target_link_libraries(test_observation_validation gnss_utils)
  ament_target_dependencies(test_observation_validation rclcpp std_msgs builtin_interfaces)
  add_dependencies(test_observation_validation ${_rosidl_deps})

  ament_add_gtest(test_raw_observations test/test_raw_observations.cpp)
  target_include_directories(test_raw_observations PUBLIC ${_gen_include})
  target_link_libraries(test_raw_observations gnss_utils)
  ament_target_dependencies(test_raw_observations rclcpp std_msgs builtin_interfaces)
  add_dependencies(test_raw_observations ${_rosidl_deps})

  ament_add_gtest(test_binary_framer test/test_binary_framer.cpp)
  ament_add_gtest(test_nmea_parser test/test_nmea_parser.cpp)
  target_include_directories(test_nmea_parser PUBLIC ${_gen_include})
  target_link_libraries(test_nmea_parser gnss_utils)
  ament_target_dependencies(test_nmea_parser rclcpp std_msgs builtin_interfaces)
  add_dependencies(test_nmea_parser ${_rosidl_deps})
  ament_add_gtest(test_receiver_protocol test/test_receiver_protocol.cpp)
  target_include_directories(test_receiver_protocol PUBLIC ${_gen_include})
  target_link_libraries(test_receiver_protocol gnss_utils)
  ament_target_dependencies(test_receiver_protocol rclcpp std_msgs builtin_interfaces)
  add_dependencies(test_receiver_protocol ${_rosidl_deps})


  # Core matcher/preprocessor regressions are MANDATORY (not EXISTS-guarded):
  # silently omitting a missing source file previously left only the FGO math
  # test registered, hiding losslessness regressions.
  if(TARGET gnss_visualizer)
    ament_add_gtest(test_visualizer_release test/test_visualizer_release.cpp)
    target_include_directories(test_visualizer_release PUBLIC ${_gen_include})
    ament_target_dependencies(test_visualizer_release rclcpp std_msgs sensor_msgs cv_bridge image_transport builtin_interfaces)
    target_link_libraries(test_visualizer_release gnss_utils ${OpenCV_LIBRARIES})
    add_dependencies(test_visualizer_release ${_rosidl_deps})
  endif()

  ament_add_gtest(test_epoch_matcher test/test_epoch_matcher.cpp)
  target_include_directories(test_epoch_matcher PUBLIC ${_gen_include})
  target_link_libraries(test_epoch_matcher gnss_utils)
  ament_target_dependencies(test_epoch_matcher rclcpp std_msgs builtin_interfaces)
  add_dependencies(test_epoch_matcher ${_rosidl_deps})

  # SensorTickClock is header-only and needs neither RTKLIB nor the generated
  # messages, so this links nothing. Mandatory for the same reason as above.
  ament_add_gtest(test_ins_tick_clock test/test_ins_tick_clock.cpp)
  ament_target_dependencies(test_ins_tick_clock rclcpp)

  ament_add_gtest(test_gnss_preprocessor test/test_gnss_preprocessor.cpp)
  target_include_directories(test_gnss_preprocessor PUBLIC ${_gen_include})
  target_link_libraries(test_gnss_preprocessor gnss_utils)
  ament_target_dependencies(test_gnss_preprocessor rclcpp std_msgs builtin_interfaces)
  add_dependencies(test_gnss_preprocessor ${_rosidl_deps})

  # End-to-end verification of the GNSS/IMU/wheel-speed EKF: the real node is
  # compiled in (main() guarded out) and driven through its ROS interfaces.
  if(BUILD_EKF_NODE)
    ament_add_gtest(test_gnss_imu_ekf
      test/test_gnss_imu_ekf.cpp
      src/positioning/ekf/gnss_imu_kalman_filter.cpp)
    if(TARGET test_gnss_imu_ekf)
      target_compile_definitions(test_gnss_imu_ekf PRIVATE GNSS_IMU_KF_NO_MAIN)
      target_include_directories(test_gnss_imu_ekf PUBLIC ${_gen_include}
        "${CMAKE_CURRENT_SOURCE_DIR}/test")
      target_link_libraries(test_gnss_imu_ekf gnss_utils Eigen3::Eigen Threads::Threads)
      ament_target_dependencies(test_gnss_imu_ekf
        rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs builtin_interfaces)
      add_dependencies(test_gnss_imu_ekf ${_rosidl_deps})
    endif()
  endif()

  # The SPP/RTK nodes compiled in (main() guarded out), driven over topics.
  foreach(_node_test rtk spp)
    if(_node_test STREQUAL "rtk")
      set(_node_dir rtk)
      set(_node_enabled ${BUILD_RTK_NODE})
    else()
      set(_node_dir spp)
      set(_node_enabled ${BUILD_SPP_NODE})
    endif()
    if(_node_enabled)
      ament_add_gtest(test_${_node_test}_node test/test_${_node_test}_node.cpp)
      target_include_directories(test_${_node_test}_node PUBLIC ${_gen_include}
        "${CMAKE_CURRENT_SOURCE_DIR}/src/positioning/${_node_dir}")
      target_link_libraries(test_${_node_test}_node gnss_utils Threads::Threads)
      ament_target_dependencies(test_${_node_test}_node rclcpp std_msgs builtin_interfaces)
      add_dependencies(test_${_node_test}_node ${_rosidl_deps})
    endif()
  endforeach()

  # End-to-end FGO math tests; need the opt-in FGO nodes (BUILD_FGO_NODES).
  if(BUILD_FGO_NODES)
    # The black-box test must use messages generated by this build, without
    # requiring an older package installation in the caller's environment.
    add_dependencies(gnss_fgo ${PROJECT_NAME}__py)
    ament_add_test(test_fgo_motion_ros
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/fgo/test_motion_ros.py"
        --executable "$<TARGET_FILE:gnss_fgo>"
      TIMEOUT 120
      APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py"
      APPEND_LIBRARY_DIRS "${CMAKE_CURRENT_BINARY_DIR}")
    add_dependencies(gnss_imu_fgo ${PROJECT_NAME}__py)
    ament_add_test(test_fgo_node_contract
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/fgo/test_node_contract.py"
        --gnss-executable "$<TARGET_FILE:gnss_fgo>"
        --imu-executable "$<TARGET_FILE:gnss_imu_fgo>"
      TIMEOUT 300
      APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py"
      APPEND_LIBRARY_DIRS "${CMAKE_CURRENT_BINARY_DIR}")
    ament_add_test(test_fgo_base_oracle
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/fgo/test_base_oracle.py"
      TIMEOUT 30)
    foreach(_fgo_test primitives transactions motion)
      add_executable(test_fgo_${_fgo_test} test/fgo/test_${_fgo_test}.cpp)
      target_link_libraries(test_fgo_${_fgo_test} fgo_engine)
      target_compile_definitions(test_fgo_${_fgo_test} PRIVATE
        FGO_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fgo")
    endforeach()
    set_target_properties(test_fgo_ros_adapter PROPERTIES EXCLUDE_FROM_ALL FALSE)
    foreach(_fgo_test primitives transactions motion ros_adapter)
      target_link_options(test_fgo_${_fgo_test} PRIVATE "-Wl,--disable-new-dtags")
      ament_add_test(test_fgo_${_fgo_test}
        GENERATE_RESULT_FOR_RETURN_CODE_ZERO
        COMMAND "$<TARGET_FILE:test_fgo_${_fgo_test}>"
        TIMEOUT 30)
    endforeach()
  endif()
endif()

# Export
ament_export_targets(export_${PROJECT_NAME}_core HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime rclcpp std_msgs geometry_msgs
  sensor_msgs nav_msgs builtin_interfaces rosbag2_storage Eigen3 Threads)
ament_package()
