cmake_minimum_required(VERSION 3.16)
project(gnss_install_consumer LANGUAGES C CXX)

# Deliberately no source-tree include paths, RTKLIB definitions or explicit
# dependency discovery: the installed package must provide its public contract.
find_package(gnss_ros_standardization REQUIRED)
add_executable(install_consumer consumer.cpp)
target_link_libraries(install_consumer gnss_ros_standardization::gnss_utils)
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/installed_core.txt" CONTENT
  "$<TARGET_PROPERTY:gnss_ros_standardization::gnss_utils,TYPE>\n$<TARGET_FILE:gnss_ros_standardization::gnss_utils>\n")
