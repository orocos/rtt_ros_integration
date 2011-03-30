cmake_minimum_required(VERSION 2.4.6)

project(rtt_ros_integration)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
rosbuild_find_ros_package(rtt )
find_package(Orocos-RTT HINTS ${rtt_PACKAGE_PATH}/install )

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

#common commands for building c++ executables and libraries
orocos_plugin(${PROJECT_NAME} src/ros_plugin.cpp src/ros_publish_activity.cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

orocos_typekit( rtt-ros-primitives-typekit
${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_primitives_typekit_plugin.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_time_type.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_uint8_type.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_int8_type.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_uint16_type.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_int16_type.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_uint32_type.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_int32_type.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_uint64_type.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_int64_type.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_float32_type.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_float64_type.cpp
${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_string_type.cpp )
  
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

orocos_generate_package()