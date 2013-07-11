cmake_minimum_required(VERSION 2.4.6)

project(rtt_rosnode)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE MinSizeRel)

rosbuild_init()
rosbuild_find_ros_package(rtt )
rosbuild_find_ros_package( ocl )
set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/../install )
find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})
list(APPEND CMAKE_MODULE_PATH ${ocl_PACKAGE_PATH}/config)
find_package(RTTPlugin REQUIRED rtt-marshalling)
find_package(roslib)
find_package(roscpp)

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

#common commands for building c++ executables and libraries
orocos_plugin(${PROJECT_NAME} src/ros_plugin.cpp src/ros_publish_activity.cpp src/rtt_ros_service.cpp)

# build ROS param as a separate service
orocos_service(rtt_rosparam_service src/rtt_rosparam_service.cpp)
target_link_libraries(rtt_rosparam_service ${RTT_PLUGIN_rtt-marshalling_LIBRARIES})

# Required for Groovy or later (RTT no longer links with roslib, but with rospack).
if (roscpp_FOUND)
  target_link_libraries(rtt_rosnode ${roslib_LIBRARIES} ${roscpp_LIBRARIES})
  target_link_libraries(rtt_rosparam_service ${roslib_LIBRARIES} ${roscpp_LIBRARIES})
endif(roscpp_FOUND)

orocos_typekit( rtt-ros-primitives-typekit ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_primitives_typekit_plugin.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_time_type.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_uint8_type.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_int8_type.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_uint16_type.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_int16_type.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_uint32_type.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_int32_type.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_uint64_type.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_int64_type.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_float32_type.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_float64_type.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_string_type.cpp )

orocos_generate_package()
