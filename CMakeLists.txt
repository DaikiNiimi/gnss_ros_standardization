cmake_minimum_required(VERSION 3.16)
project(gnss_ros_standardization VERSION 1.0.0 LANGUAGES C CXX)
# NOTE: Keep VERSION in sync with package.xml. Used by tools' --version flag.

# Basics
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
option(BUILD_RINEX_WRITER_TOOL "Build ros2_rinex_writer (db3->RINEX)" ON)

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
file(GLOB _core_main "${RTKLIB_ROOT}/src/*.c")
if(EXISTS "${RTKLIB_ROOT}/src/rcv")
  file(GLOB _core_rcv  "${RTKLIB_ROOT}/src/rcv/*.c")
endif()
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

add_library(rtkcore_lib STATIC ${_core_sources})
set_target_properties(rtkcore_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  target_compile_options(rtkcore_lib PRIVATE "-w")
endif()
target_include_directories(rtkcore_lib PUBLIC "${RTKLIB_ROOT}/src")

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
target_include_directories(rtkcore_config INTERFACE "${RTKLIB_ROOT}/src")
target_link_libraries(rtkcore_lib PUBLIC rtkcore_config)
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
# GnssPreprocessor (optimizer-ready DD preprocessing) ships with the FGO
# examples; build it only when present.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/utils/gnss_preprocessor.cpp")
  list(APPEND _gnss_utils_sources src/utils/gnss_preprocessor.cpp)
endif()
add_library(gnss_utils ${_gnss_utils_sources})
target_include_directories(gnss_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(gnss_utils PRIVATE "${RTKLIB_ROOT}/src")
# We need rtkcore_lib for rtklib types used in gnss_utils
target_link_libraries(gnss_utils rtkcore_lib rtkcore_config Eigen3::Eigen ${cpp_typesupport_target})
ament_target_dependencies(gnss_utils rclcpp std_msgs builtin_interfaces)

install(TARGETS gnss_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# rtcm_decoder_node
if(BUILD_RTCM_DECODER_NODE AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/decoders/rtcm_decoder_node.cpp")
  add_executable(rtcm_decoder_node src/decoders/rtcm_decoder_node.cpp)
  target_include_directories(rtcm_decoder_node PUBLIC ${_gen_include})
  ament_target_dependencies(rtcm_decoder_node rclcpp std_msgs builtin_interfaces)
  target_link_libraries(rtcm_decoder_node gnss_utils Threads::Threads)
  add_dependencies(rtcm_decoder_node ${_rosidl_deps})
  install(TARGETS rtcm_decoder_node DESTINATION lib/${PROJECT_NAME})
elseif(BUILD_RTCM_DECODER_NODE)
  message(WARNING "BUILD_RTCM_DECODER_NODE=ON but src/decoders/rtcm_decoder_node.cpp not found.")
endif()

# ubx_decoder_node
option(BUILD_UBX_DECODER_NODE "Build ubx_decoder_node" ON)
if(BUILD_UBX_DECODER_NODE AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/decoders/ubx_decoder_node.cpp")
  add_executable(ubx_decoder_node src/decoders/ubx_decoder_node.cpp)
  target_include_directories(ubx_decoder_node PUBLIC ${_gen_include})
  ament_target_dependencies(ubx_decoder_node rclcpp std_msgs geometry_msgs sensor_msgs builtin_interfaces)
  target_link_libraries(ubx_decoder_node gnss_utils Threads::Threads)
  add_dependencies(ubx_decoder_node ${_rosidl_deps})
  install(TARGETS ubx_decoder_node DESTINATION lib/${PROJECT_NAME})
elseif(BUILD_UBX_DECODER_NODE)
  message(WARNING "BUILD_UBX_DECODER_NODE=ON but src/decoders/ubx_decoder_node.cpp not found.")
endif()

# ubx_driver_node (u-blox driver with configuration)
option(BUILD_UBX_DRIVER_NODE "Build ubx_driver_node (u-blox driver)" ON)
if(BUILD_UBX_DRIVER_NODE AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/ubx_driver_node.cpp")
  add_executable(ubx_driver_node src/drivers/ubx_driver_node.cpp)
  target_include_directories(ubx_driver_node PUBLIC ${_gen_include})
  ament_target_dependencies(ubx_driver_node rclcpp std_msgs geometry_msgs sensor_msgs builtin_interfaces)
  target_link_libraries(ubx_driver_node gnss_utils Threads::Threads)
  add_dependencies(ubx_driver_node ${_rosidl_deps})
  install(TARGETS ubx_driver_node DESTINATION lib/${PROJECT_NAME})
elseif(BUILD_UBX_DRIVER_NODE)
  message(WARNING "BUILD_UBX_DRIVER_NODE=ON but src/drivers/ubx_driver_node.cpp not found.")
endif()

# sbf_decoder_node (Septentrio)
option(BUILD_SBF_DECODER_NODE "Build sbf_decoder_node (Septentrio)" ON)
if(BUILD_SBF_DECODER_NODE AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/decoders/sbf_decoder_node.cpp")
  add_executable(sbf_decoder_node src/decoders/sbf_decoder_node.cpp)
  target_include_directories(sbf_decoder_node PUBLIC ${_gen_include})
  ament_target_dependencies(sbf_decoder_node rclcpp std_msgs geometry_msgs sensor_msgs builtin_interfaces)
  target_link_libraries(sbf_decoder_node gnss_utils Threads::Threads)
  add_dependencies(sbf_decoder_node ${_rosidl_deps})
  install(TARGETS sbf_decoder_node DESTINATION lib/${PROJECT_NAME})
elseif(BUILD_SBF_DECODER_NODE)
  message(WARNING "BUILD_SBF_DECODER_NODE=ON but src/decoders/sbf_decoder_node.cpp not found.")
endif()

# sbf_driver_node (Septentrio driver with configuration)
option(BUILD_SBF_DRIVER_NODE "Build sbf_driver_node (Septentrio driver)" ON)
if(BUILD_SBF_DRIVER_NODE AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/sbf_driver_node.cpp")
  add_executable(sbf_driver_node src/drivers/sbf_driver_node.cpp)
  target_include_directories(sbf_driver_node PUBLIC ${_gen_include})
  ament_target_dependencies(sbf_driver_node rclcpp std_msgs geometry_msgs sensor_msgs builtin_interfaces)
  target_link_libraries(sbf_driver_node gnss_utils Threads::Threads)
  add_dependencies(sbf_driver_node ${_rosidl_deps})
  install(TARGETS sbf_driver_node DESTINATION lib/${PROJECT_NAME})
elseif(BUILD_SBF_DRIVER_NODE)
  message(WARNING "BUILD_SBF_DRIVER_NODE=ON but src/drivers/sbf_driver_node.cpp not found.")
endif()

# novatel_decoder_node (NovAtel OEM3/OEM4/OEM7)
option(BUILD_NOVATEL_DECODER_NODE "Build novatel_decoder_node (NovAtel)" ON)
if(BUILD_NOVATEL_DECODER_NODE AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/decoders/novatel_decoder_node.cpp")
  add_executable(novatel_decoder_node src/decoders/novatel_decoder_node.cpp)
  target_include_directories(novatel_decoder_node PUBLIC ${_gen_include})
  ament_target_dependencies(novatel_decoder_node rclcpp std_msgs geometry_msgs sensor_msgs builtin_interfaces)
  target_link_libraries(novatel_decoder_node gnss_utils Threads::Threads)
  add_dependencies(novatel_decoder_node ${_rosidl_deps})
  install(TARGETS novatel_decoder_node DESTINATION lib/${PROJECT_NAME})
elseif(BUILD_NOVATEL_DECODER_NODE)
  message(WARNING "BUILD_NOVATEL_DECODER_NODE=ON but src/decoders/novatel_decoder_node.cpp not found.")
endif()

# novatel_driver_node (NovAtel driver with configuration)
option(BUILD_NOVATEL_DRIVER_NODE "Build novatel_driver_node (NovAtel driver)" ON)
if(BUILD_NOVATEL_DRIVER_NODE AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/novatel_driver_node.cpp")
  add_executable(novatel_driver_node src/drivers/novatel_driver_node.cpp)
  target_include_directories(novatel_driver_node PUBLIC ${_gen_include})
  ament_target_dependencies(novatel_driver_node rclcpp std_msgs geometry_msgs sensor_msgs builtin_interfaces)
  target_link_libraries(novatel_driver_node gnss_utils Threads::Threads)
  add_dependencies(novatel_driver_node ${_rosidl_deps})
  install(TARGETS novatel_driver_node DESTINATION lib/${PROJECT_NAME})
elseif(BUILD_NOVATEL_DRIVER_NODE)
  message(WARNING "BUILD_NOVATEL_DRIVER_NODE=ON but src/drivers/novatel_driver_node.cpp not found.")
endif()

# rosbag_to_rinex
if(BUILD_RINEX_WRITER_TOOL)
  set(_writer_src "")
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/converter/rosbag_to_rinex.cpp")
    set(_writer_src src/converter/rosbag_to_rinex.cpp)
  elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/converter/ros2_rinex_writer.cpp")
     # fallback or legacy
    set(_writer_src src/converter/ros2_rinex_writer.cpp)
  endif()

  if(_writer_src)
    add_executable(rosbag_to_rinex ${_writer_src})
    target_include_directories(rosbag_to_rinex PUBLIC ${_gen_include})
    target_compile_definitions(rosbag_to_rinex PRIVATE PACKAGE_VERSION="${PROJECT_VERSION}")
    ament_target_dependencies(rosbag_to_rinex rclcpp rcutils rosbag2_cpp rosbag2_storage std_msgs builtin_interfaces)
    target_include_directories(rosbag_to_rinex PRIVATE "${RTKLIB_ROOT}/src")
    target_link_libraries(rosbag_to_rinex gnss_utils)
    add_dependencies(rosbag_to_rinex ${_rosidl_deps})
    install(TARGETS rosbag_to_rinex DESTINATION lib/${PROJECT_NAME})
  else()
    message(WARNING "BUILD_RINEX_WRITER_TOOL=ON but no writer source found in src/converter/.")
  endif()
endif()

# rinex_to_rosbag
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/converter/rinex_to_rosbag.cpp")
  add_executable(rinex_to_rosbag src/converter/rinex_to_rosbag.cpp)
  target_include_directories(rinex_to_rosbag PUBLIC ${_gen_include})
  ament_target_dependencies(rinex_to_rosbag rclcpp rosbag2_cpp rosbag2_storage std_msgs sensor_msgs nav_msgs builtin_interfaces)
  target_link_libraries(rinex_to_rosbag gnss_utils)
  add_dependencies(rinex_to_rosbag ${_rosidl_deps})
  install(TARGETS rinex_to_rosbag DESTINATION lib/${PROJECT_NAME})
endif()

# rosbag_to_pos (GnssSolution -> RTKLIB .pos)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/converter/rosbag_to_pos.cpp")
  add_executable(rosbag_to_pos src/converter/rosbag_to_pos.cpp)
  target_include_directories(rosbag_to_pos PUBLIC ${_gen_include})
  target_compile_definitions(rosbag_to_pos PRIVATE PACKAGE_VERSION="${PROJECT_VERSION}")
  target_include_directories(rosbag_to_pos PRIVATE "${RTKLIB_ROOT}/src")
  ament_target_dependencies(rosbag_to_pos rclcpp rosbag2_cpp rosbag2_storage std_msgs geometry_msgs builtin_interfaces)
  target_link_libraries(rosbag_to_pos gnss_utils)
  add_dependencies(rosbag_to_pos ${_rosidl_deps})
  install(TARGETS rosbag_to_pos DESTINATION lib/${PROJECT_NAME})
endif()

# pos_to_rosbag (RTKLIB .pos -> GnssSolution bag)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/converter/pos_to_rosbag.cpp")
  add_executable(pos_to_rosbag src/converter/pos_to_rosbag.cpp)
  target_include_directories(pos_to_rosbag PUBLIC ${_gen_include})
  target_include_directories(pos_to_rosbag PRIVATE "${RTKLIB_ROOT}/src")
  ament_target_dependencies(pos_to_rosbag rclcpp rosbag2_cpp rosbag2_storage std_msgs geometry_msgs builtin_interfaces)
  target_link_libraries(pos_to_rosbag gnss_utils)
  add_dependencies(pos_to_rosbag ${_rosidl_deps})
  install(TARGETS pos_to_rosbag DESTINATION lib/${PROJECT_NAME})
endif()

# synthetic_scenario_to_rosbag (multi-GNSS + IMU simulator -> bag, for
# validating the examples against a known ground truth). Reuses the generator
# headers under test/. GTSAM-independent.
option(BUILD_SYNTHETIC_SCENARIO_TOOL "Build synthetic_scenario_to_rosbag" ON)
if(BUILD_SYNTHETIC_SCENARIO_TOOL AND
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/converter/synthetic_scenario_to_rosbag.cpp")
  add_executable(synthetic_scenario_to_rosbag src/converter/synthetic_scenario_to_rosbag.cpp)
  target_include_directories(synthetic_scenario_to_rosbag PUBLIC ${_gen_include})
  target_include_directories(synthetic_scenario_to_rosbag PRIVATE
    "${RTKLIB_ROOT}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/test")
  ament_target_dependencies(synthetic_scenario_to_rosbag
    rclcpp rosbag2_cpp rosbag2_storage std_msgs sensor_msgs geometry_msgs builtin_interfaces)
  target_link_libraries(synthetic_scenario_to_rosbag gnss_utils Eigen3::Eigen)
  add_dependencies(synthetic_scenario_to_rosbag ${_rosidl_deps})
  install(TARGETS synthetic_scenario_to_rosbag DESTINATION lib/${PROJECT_NAME})
endif()

# single point positioning
option(BUILD_SPP_EXAMPLE "Build single point positioning example (pntpos)" ON)

if(BUILD_SPP_EXAMPLE AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples/single_point_positioning/single_point_positioning.cpp")
  add_executable(single_point_positioning examples/single_point_positioning/single_point_positioning.cpp)
  target_include_directories(single_point_positioning PUBLIC ${_gen_include})
  ament_target_dependencies(single_point_positioning rclcpp std_msgs builtin_interfaces)
  target_link_libraries(single_point_positioning
    gnss_utils
    Threads::Threads)
  add_dependencies(single_point_positioning ${_rosidl_deps})
  install(TARGETS single_point_positioning DESTINATION lib/${PROJECT_NAME})
elseif(BUILD_SPP_EXAMPLE)
  message(WARNING "BUILD_SPP_EXAMPLE=ON but examples/single_point_positioning/single_point_positioning.cpp was not found.")
endif()
# real time kinematic
option(BUILD_RTK_EXAMPLE "Build real time kinematic example (rtkpos)" ON)

if(BUILD_RTK_EXAMPLE AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples/real_time_kinematic/real_time_kinematic.cpp")
  add_executable(real_time_kinematic examples/real_time_kinematic/real_time_kinematic.cpp)
  target_include_directories(real_time_kinematic PUBLIC ${_gen_include})
  ament_target_dependencies(real_time_kinematic rclcpp std_msgs builtin_interfaces)
  target_link_libraries(real_time_kinematic
    gnss_utils
    Threads::Threads)
  add_dependencies(real_time_kinematic ${_rosidl_deps})
  install(TARGETS real_time_kinematic DESTINATION lib/${PROJECT_NAME})
elseif(BUILD_RTK_EXAMPLE)
  message(WARNING "BUILD_RTK_EXAMPLE=ON but examples/real_time_kinematic/real_time_kinematic.cpp not found.")
endif()

# gnss_imu_kalman_filter (GNSS/IMU/WheelSpeed Kalman Filter)
option(BUILD_GNSS_IMU_KALMAN_FILTER "Build gnss_imu_kalman_filter (GNSS/IMU Kalman Filter)" ON)
if(BUILD_GNSS_IMU_KALMAN_FILTER AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples/gnss_imu_kalman_filter/gnss_imu_kalman_filter.cpp")
  add_executable(gnss_imu_kalman_filter examples/gnss_imu_kalman_filter/gnss_imu_kalman_filter.cpp)
  target_include_directories(gnss_imu_kalman_filter PUBLIC ${_gen_include})
  ament_target_dependencies(gnss_imu_kalman_filter rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs builtin_interfaces)
  target_link_libraries(gnss_imu_kalman_filter gnss_utils Eigen3::Eigen Threads::Threads)
  add_dependencies(gnss_imu_kalman_filter ${_rosidl_deps})
  install(TARGETS gnss_imu_kalman_filter DESTINATION lib/${PROJECT_NAME})
elseif(BUILD_GNSS_IMU_KALMAN_FILTER)
  message(WARNING "BUILD_GNSS_IMU_KALMAN_FILTER=ON but examples/gnss_imu_kalman_filter/gnss_imu_kalman_filter.cpp not found.")
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

# Tightly-coupled FGO examples (opt-in: they need GTSAM with the GNSS DD
# factors, which are only on GTSAM's develop branch as of June 2026 - see
# examples/tightly_coupled_fgo/README.md for build instructions).
option(BUILD_GTSAM_FGO_EXAMPLES "Build tightly-coupled FGO examples (requires GTSAM develop)" OFF)
if(BUILD_GTSAM_FGO_EXAMPLES)
  find_package(GTSAM REQUIRED)
  get_target_property(_gtsam_incs gtsam INTERFACE_INCLUDE_DIRECTORIES)
  find_path(GTSAM_GNSS_FACTOR_DIR gtsam/navigation/PseudorangeFactor.h
            PATHS ${_gtsam_incs} NO_DEFAULT_PATH)
  if(NOT GTSAM_GNSS_FACTOR_DIR)
    message(FATAL_ERROR
      "GTSAM was found but lacks the GNSS DD factors "
      "(gtsam/navigation/PseudorangeFactor.h). Build GTSAM from the develop "
      "branch - see examples/tightly_coupled_fgo/README.md.")
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples/tightly_coupled_fgo/gnss_fgo.cpp")
    add_executable(gnss_fgo examples/tightly_coupled_fgo/gnss_fgo.cpp)
    target_include_directories(gnss_fgo PUBLIC ${_gen_include})
    ament_target_dependencies(gnss_fgo rclcpp std_msgs builtin_interfaces)
    target_link_libraries(gnss_fgo gnss_utils gtsam Threads::Threads)
    add_dependencies(gnss_fgo ${_rosidl_deps})
    install(TARGETS gnss_fgo DESTINATION lib/${PROJECT_NAME})
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples/tightly_coupled_fgo/gnss_imu_fgo.cpp")
    add_executable(gnss_imu_fgo examples/tightly_coupled_fgo/gnss_imu_fgo.cpp)
    target_include_directories(gnss_imu_fgo PUBLIC ${_gen_include})
    ament_target_dependencies(gnss_imu_fgo rclcpp std_msgs sensor_msgs nav_msgs builtin_interfaces)
    target_link_libraries(gnss_imu_fgo gnss_utils gtsam Threads::Threads)
    add_dependencies(gnss_imu_fgo ${_rosidl_deps})
    install(TARGETS gnss_imu_fgo DESTINATION lib/${PROJECT_NAME})
  endif()

  # Embed GTSAM's library directory in the installed FGO binaries so they run
  # without LD_LIBRARY_PATH even when GTSAM lives in a non-system prefix
  # (e.g. $HOME/gtsam-install). INSTALL_RPATH_USE_LINK_PATH picks up the absolute
  # dir of the linked gtsam target, so this works wherever GTSAM was installed.
  # `--disable-new-dtags` emits DT_RPATH instead of DT_RUNPATH: DT_RPATH is also
  # searched for GTSAM's own transitive deps (libmetis-gtsam / libcephes-gtsam,
  # which libgtsam.so carries no runpath for), whereas DT_RUNPATH would not
  # cascade to them and the node would fail to load without LD_LIBRARY_PATH.
  foreach(_fgo_exe gnss_fgo gnss_imu_fgo)
    if(TARGET ${_fgo_exe})
      set_target_properties(${_fgo_exe} PROPERTIES INSTALL_RPATH_USE_LINK_PATH ON)
      target_link_options(${_fgo_exe} PRIVATE "-Wl,--disable-new-dtags")
    endif()
  endforeach()
endif()

# Tests (run locally with `colcon test`; CI wiring is handled separately)
#
# The whole block is guarded on the test/ DIRECTORY, not on the individual
# files, so a packaging step that strips tests still configures; but once test/
# is present, a MISSING FILE INSIDE IT is a hard error (see the comment on the
# two core tests below). Those are different situations and only one of them is
# an accident.
if(BUILD_TESTING AND NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test")
  message(STATUS
    "test/ is not present - no unit tests will be built or run. It ships with "
    "the package, so this means either a stripped-down copy or a working tree "
    "where test/ has gone missing.")
endif()
if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test")
  find_package(ament_cmake_gtest REQUIRED)

  # Core matcher/preprocessor regressions are MANDATORY (not EXISTS-guarded):
  # silently omitting a missing source file previously left only the FGO math
  # test registered, hiding losslessness regressions.
  ament_add_gtest(test_epoch_matcher test/test_epoch_matcher.cpp)
  target_include_directories(test_epoch_matcher PUBLIC ${_gen_include})
  target_link_libraries(test_epoch_matcher gnss_utils)
  ament_target_dependencies(test_epoch_matcher rclcpp std_msgs builtin_interfaces)
  add_dependencies(test_epoch_matcher ${_rosidl_deps})

  ament_add_gtest(test_gnss_preprocessor test/test_gnss_preprocessor.cpp)
  target_include_directories(test_gnss_preprocessor PUBLIC ${_gen_include})
  target_link_libraries(test_gnss_preprocessor gnss_utils)
  ament_target_dependencies(test_gnss_preprocessor rclcpp std_msgs builtin_interfaces)
  add_dependencies(test_gnss_preprocessor ${_rosidl_deps})

  # End-to-end verification of the GNSS/IMU/wheel-speed EKF: the real node is
  # compiled in (main() guarded out) and driven through its ROS interfaces.
  if(BUILD_GNSS_IMU_KALMAN_FILTER AND
     EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_gnss_imu_ekf.cpp" AND
     EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/examples/gnss_imu_kalman_filter/gnss_imu_kalman_filter.cpp")
    ament_add_gtest(test_gnss_imu_ekf
      test/test_gnss_imu_ekf.cpp
      examples/gnss_imu_kalman_filter/gnss_imu_kalman_filter.cpp)
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

  # End-to-end FGO math tests; need the opt-in GTSAM examples.
  if(BUILD_GTSAM_FGO_EXAMPLES)
    foreach(_fgo_test test_gnss_fgo test_gnss_imu_fgo)
      if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/${_fgo_test}.cpp")
        ament_add_gtest(${_fgo_test} test/${_fgo_test}.cpp)
      endif()
      if(TARGET ${_fgo_test})
        target_include_directories(${_fgo_test} PUBLIC ${_gen_include}
          "${CMAKE_CURRENT_SOURCE_DIR}/examples/tightly_coupled_fgo")
        target_link_libraries(${_fgo_test} gnss_utils gtsam)
        ament_target_dependencies(${_fgo_test} rclcpp std_msgs builtin_interfaces)
        add_dependencies(${_fgo_test} ${_rosidl_deps})
        set_target_properties(${_fgo_test} PROPERTIES INSTALL_RPATH_USE_LINK_PATH ON)
        target_link_options(${_fgo_test} PRIVATE "-Wl,--disable-new-dtags")
      endif()
    endforeach()
  endif()
endif()

# Export
ament_export_dependencies(rosidl_default_runtime std_msgs builtin_interfaces)
ament_package()
