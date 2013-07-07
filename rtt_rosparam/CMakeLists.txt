cmake_minimum_required(VERSION 2.8.3)
project(rtt_rosparam)

find_package(catkin REQUIRED COMPONENTS roscpp rtt_ros)
find_package(OROCOS-RTT REQUIRED rtt-marshalling)

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

catkin_package(
  DEPENDS rtt ocl
  CATKIN_DEPENDS roscpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

# build ROS param as a separate service
catkin_orocos_service(rtt_rosparam src/rtt_rosparam_service.cpp)
target_link_libraries(rtt_rosparam 
  ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY} 
  ${catkin_LIBRARIES})

# Generate install targets and pkg-config files
orocos_generate_package()

