cmake_minimum_required(VERSION 3.16)
project(gnss_ros_standardization LANGUAGES C CXX)

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

# MALIB core
set(MALIB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/MALIB")
if(NOT EXISTS "${MALIB_ROOT}/src/rtklib.h")
  message(FATAL_ERROR "MALIB not found: ${MALIB_ROOT}/src/rtklib.h")
endif()
file(GLOB _core_main "${MALIB_ROOT}/src/*.c")
if(EXISTS "${MALIB_ROOT}/src/rcv")
  file(GLOB _core_rcv  "${MALIB_ROOT}/src/rcv/*.c")
endif()
set(_core_sources ${_core_main} ${_core_rcv})

set(_apps_to_exclude
  convbin.c rnx2rtkp.c str2str.c rtknavi.c rtkrcv.c rtkplot.c rtkgetopt.c rtksvr.c
  convkml.c
)
foreach(_maybe_app IN LISTS _apps_to_exclude)
  if(EXISTS "${MALIB_ROOT}/src/${_maybe_app}")
    list(REMOVE_ITEM _core_sources "${MALIB_ROOT}/src/${_maybe_app}")
  endif()
endforeach()

list(LENGTH _core_sources _core_src_count)
if(_core_src_count EQUAL 0)
  message(FATAL_ERROR "MALIB core sources list is empty")
endif()

add_library(rtkcore_lib STATIC ${_core_sources})
set_target_properties(rtkcore_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  target_compile_options(rtkcore_lib PRIVATE "-w")
endif()
target_include_directories(rtkcore_lib PUBLIC "${MALIB_ROOT}/src")

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
target_include_directories(rtkcore_config INTERFACE "${MALIB_ROOT}/src")
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
add_library(gnss_utils src/utils/gnss_utils.cpp)
target_include_directories(gnss_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(gnss_utils PRIVATE "${MALIB_ROOT}/src")
# We need rtkcore_lib for rtklib types used in gnss_utils
target_link_libraries(gnss_utils rtkcore_lib rtkcore_config ${cpp_typesupport_target})
ament_target_dependencies(gnss_utils rclcpp std_msgs builtin_interfaces)

install(TARGETS gnss_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
    ament_target_dependencies(rosbag_to_rinex rclcpp rcutils rosbag2_cpp rosbag2_storage std_msgs builtin_interfaces)
    target_include_directories(rosbag_to_rinex PRIVATE "${MALIB_ROOT}/src")
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
  ament_target_dependencies(rinex_to_rosbag rclcpp rosbag2_cpp rosbag2_storage std_msgs builtin_interfaces)
  target_link_libraries(rinex_to_rosbag gnss_utils)
  add_dependencies(rinex_to_rosbag ${_rosidl_deps})
  install(TARGETS rinex_to_rosbag DESTINATION lib/${PROJECT_NAME})
endif()

# rosbag_to_pos (GnssSolution -> RTKLIB .pos)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/converter/rosbag_to_pos.cpp")
  add_executable(rosbag_to_pos src/converter/rosbag_to_pos.cpp)
  target_include_directories(rosbag_to_pos PUBLIC ${_gen_include})
  target_include_directories(rosbag_to_pos PRIVATE "${MALIB_ROOT}/src")
  ament_target_dependencies(rosbag_to_pos rclcpp rosbag2_cpp rosbag2_storage std_msgs geometry_msgs builtin_interfaces)
  target_link_libraries(rosbag_to_pos gnss_utils)
  add_dependencies(rosbag_to_pos ${_rosidl_deps})
  install(TARGETS rosbag_to_pos DESTINATION lib/${PROJECT_NAME})
endif()

# pos_to_rosbag (RTKLIB .pos -> GnssSolution bag)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/converter/pos_to_rosbag.cpp")
  add_executable(pos_to_rosbag src/converter/pos_to_rosbag.cpp)
  target_include_directories(pos_to_rosbag PUBLIC ${_gen_include})
  target_include_directories(pos_to_rosbag PRIVATE "${MALIB_ROOT}/src")
  ament_target_dependencies(pos_to_rosbag rclcpp rosbag2_cpp rosbag2_storage std_msgs geometry_msgs builtin_interfaces)
  target_link_libraries(pos_to_rosbag gnss_utils)
  add_dependencies(pos_to_rosbag ${_rosidl_deps})
  install(TARGETS pos_to_rosbag DESTINATION lib/${PROJECT_NAME})
endif()

# single point positioning
option(BUILD_SPP_EXAMPLE "Build single point positioning example (pntpos)" ON)

if(BUILD_SPP_EXAMPLE AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/positioning/single_point_positioning.cpp")
  add_executable(single_point_positioning src/positioning/single_point_positioning.cpp)
  target_include_directories(single_point_positioning PUBLIC ${_gen_include})
  ament_target_dependencies(single_point_positioning rclcpp std_msgs builtin_interfaces)
  target_link_libraries(single_point_positioning
    gnss_utils
    Threads::Threads)
  add_dependencies(single_point_positioning ${_rosidl_deps})
  install(TARGETS single_point_positioning DESTINATION lib/${PROJECT_NAME})
elseif(BUILD_SPP_EXAMPLE)
  message(WARNING "BUILD_SPP_EXAMPLE=ON but src/positioning/single_point_positioning.cpp was not found.")
endif()
# real time kinematic
option(BUILD_RTK_EXAMPLE "Build real time kinematic example (rtkpos)" ON)

if(BUILD_RTK_EXAMPLE AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/positioning/real_time_kinematic.cpp")
  add_executable(real_time_kinematic src/positioning/real_time_kinematic.cpp)
  target_include_directories(real_time_kinematic PUBLIC ${_gen_include})
  ament_target_dependencies(real_time_kinematic rclcpp std_msgs builtin_interfaces)
  target_link_libraries(real_time_kinematic
    gnss_utils
    Threads::Threads)
  add_dependencies(real_time_kinematic ${_rosidl_deps})
  install(TARGETS real_time_kinematic DESTINATION lib/${PROJECT_NAME})
elseif(BUILD_RTK_EXAMPLE)
  message(WARNING "BUILD_RTK_EXAMPLE=ON but src/positioning/real_time_kinematic.cpp not found.")
endif()

# ekf_node (GNSS/IMU/WheelSpeed EKF)
option(BUILD_EKF_NODE "Build ekf_node (GNSS/IMU EKF)" ON)
if(BUILD_EKF_NODE AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/positioning/ekf_node.cpp")
  add_executable(ekf_node src/positioning/ekf_node.cpp)
  target_include_directories(ekf_node PUBLIC ${_gen_include})
  ament_target_dependencies(ekf_node rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs builtin_interfaces)
  target_link_libraries(ekf_node gnss_utils Eigen3::Eigen Threads::Threads)
  add_dependencies(ekf_node ${_rosidl_deps})
  install(TARGETS ekf_node DESTINATION lib/${PROJECT_NAME})
elseif(BUILD_EKF_NODE)
  message(WARNING "BUILD_EKF_NODE=ON but src/positioning/ekf_node.cpp not found.")
endif()

# gnss_visualizer
if(cv_bridge_FOUND AND image_transport_FOUND AND OpenCV_FOUND)
  add_executable(gnss_visualizer src/utils/gnss_visualizer.cpp)
  target_include_directories(gnss_visualizer PUBLIC ${_gen_include})
  ament_target_dependencies(gnss_visualizer rclcpp std_msgs sensor_msgs cv_bridge image_transport builtin_interfaces)
  target_link_libraries(gnss_visualizer gnss_utils ${OpenCV_LIBRARIES})
  add_dependencies(gnss_visualizer ${_rosidl_deps})
  install(TARGETS gnss_visualizer DESTINATION lib/${PROJECT_NAME})

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/utils/simple_gnss_visualizer.cpp")
    add_executable(simple_gnss_visualizer src/utils/simple_gnss_visualizer.cpp)
    target_include_directories(simple_gnss_visualizer PUBLIC ${_gen_include})
    ament_target_dependencies(simple_gnss_visualizer rclcpp std_msgs sensor_msgs cv_bridge image_transport builtin_interfaces)
    target_link_libraries(simple_gnss_visualizer gnss_utils ${OpenCV_LIBRARIES})
    add_dependencies(simple_gnss_visualizer ${_rosidl_deps})
    install(TARGETS simple_gnss_visualizer DESTINATION lib/${PROJECT_NAME})
  endif()
else()
  message(STATUS "Skipping gnss_visualizer: cv_bridge, image_transport, or OpenCV not found.")
endif()

# Export
ament_export_dependencies(rosidl_default_runtime std_msgs builtin_interfaces)
ament_package()

