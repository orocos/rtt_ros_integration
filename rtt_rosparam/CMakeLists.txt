cmake_minimum_required(VERSION 2.8.3)
project(rtt_rosparam)

find_package(catkin REQUIRED COMPONENTS roscpp rtt_ros)

# Note: find_package(Eigen3) is not available in earlier Ubuntu versions (< trusty)
include(FindPkgConfig)
pkg_search_module(EIGEN3 REQUIRED eigen3)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS})

# build ROS param as a separate service
orocos_service(rtt_rosparam src/rtt_rosparam_service.cpp)
target_link_libraries(rtt_rosparam 
#  ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY} 
  ${catkin_LIBRARIES})

# Generate install targets and pkg-config files
orocos_install_headers(include/rtt_rosparam/rosparam.h)
orocos_generate_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  DEPENDS roscpp eigen3
)
