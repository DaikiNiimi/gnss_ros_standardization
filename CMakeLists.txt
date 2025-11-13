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
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(Threads REQUIRED)

# Options
option(ENABLE_GLO "Enable GLONASS" ON)
option(ENABLE_GAL "Enable Galileo" ON)
option(ENABLE_BDS "Enable BeiDou (Compass)" ON)
option(ENABLE_QZS "Enable QZSS" ON)
option(ENABLE_SBS "Enable SBAS" ON)
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
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${_msgs}
  DEPENDENCIES std_msgs builtin_interfaces
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
target_include_directories(rtkcore_lib PUBLIC "${MALIB_ROOT}/src")

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

# rtcm_decoder_node
if(BUILD_RTCM_DECODER_NODE AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/rtcm_decoder_node.cpp")
  add_executable(rtcm_decoder_node src/rtcm_decoder_node.cpp)
  target_include_directories(rtcm_decoder_node PUBLIC ${_gen_include})
  ament_target_dependencies(rtcm_decoder_node rclcpp std_msgs builtin_interfaces)
  target_link_libraries(rtcm_decoder_node rtkcore_lib Threads::Threads ${cpp_typesupport_target})
  add_dependencies(rtcm_decoder_node ${_rosidl_deps})
  install(TARGETS rtcm_decoder_node DESTINATION lib/${PROJECT_NAME})
elseif(BUILD_RTCM_DECODER_NODE)
  message(WARNING "BUILD_RTCM_DECODER_NODE=ON but src/rtcm_decoder_node.cpp not found.")
endif()

# ros2_rinex_writer
if(BUILD_RINEX_WRITER_TOOL)
  set(_writer_src "")
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/converter/ros2_rinex_writer.cpp")
    set(_writer_src src/converter/ros2_rinex_writer.cpp)
  elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/converter/rtcm_decoder_node.cpp")
    message(WARNING "Using src/converter/rtcm_decoder_node.cpp as ros2_rinex_writer (rename to ros2_rinex_writer.cpp).")
    set(_writer_src src/converter/rtcm_decoder_node.cpp)
  endif()

  if(_writer_src)
    add_executable(ros2_rinex_writer ${_writer_src})
    target_include_directories(ros2_rinex_writer PUBLIC ${_gen_include})
    ament_target_dependencies(ros2_rinex_writer rclcpp rcutils rosbag2_cpp rosbag2_storage std_msgs builtin_interfaces)
    target_include_directories(ros2_rinex_writer PRIVATE "${MALIB_ROOT}/src")
    target_link_libraries(ros2_rinex_writer rtkcore_lib rtkcore_config ${cpp_typesupport_target})
    add_dependencies(ros2_rinex_writer ${_rosidl_deps})
    install(TARGETS ros2_rinex_writer DESTINATION lib/${PROJECT_NAME})
  else()
    message(WARNING "BUILD_RINEX_WRITER_TOOL=ON but no writer source found in src/converter/.")
  endif()
endif()

# single point positioning
option(BUILD_SPP_EXAMPLE "Build single point positioning example (pntpos)" ON)

if(BUILD_SPP_EXAMPLE AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/positioning/single_point_positioning.cpp")
  add_executable(single_point_positioning src/positioning/single_point_positioning.cpp)
  target_include_directories(single_point_positioning PUBLIC ${_gen_include})
  ament_target_dependencies(single_point_positioning rclcpp std_msgs builtin_interfaces)
  target_link_libraries(single_point_positioning
    rtkcore_lib
    rtkcore_config
    ${cpp_typesupport_target}
    Threads::Threads)
  add_dependencies(single_point_positioning ${_rosidl_deps})
  install(TARGETS single_point_positioning DESTINATION lib/${PROJECT_NAME})
elseif(BUILD_SPP_EXAMPLE)
  message(WARNING "BUILD_SPP_EXAMPLE=ON ですが src/positioning/single_point_positioning.cpp が見つかりません。")
endif()

# Export
ament_export_dependencies(rosidl_default_runtime std_msgs builtin_interfaces)
ament_package()
