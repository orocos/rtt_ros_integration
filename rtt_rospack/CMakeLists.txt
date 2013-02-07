cmake_minimum_required(VERSION 2.4.6)

project(rtt_rospack)

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
find_package( roslib )

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

#build ROS pack as a separate service
orocos_plugin(rtt_rospack src/rtt_rospack_service.cpp)

if (roslib_FOUND)
  # Required for Groovy or later (RTT no longer links with roslib, but with rospack).
  target_link_libraries(rtt_rospack ${roslib_LIBRARIES})
endif()

orocos_generate_package()
