cmake_minimum_required(VERSION 2.8.3)
project(rtt_rosparam)

find_package(catkin REQUIRED COMPONENTS roscpp rtt_rospack)

find_package(OROCOS-RTT REQUIRED rtt-marshalling)

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${catkin_INCLUDE_DIRS})

catkin_package(
  DEPENDS rtt ocl
  CATKIN_DEPENDS roscpp)

# build ROS param as a separate service
orocos_service(rtt_rosparam src/rtt_rosparam_service.cpp)
target_link_libraries(rtt_rosparam ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY} ${catkin_LIBRARIES})

orocos_generate_package()

