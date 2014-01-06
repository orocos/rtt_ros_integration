cmake_minimum_required(VERSION 2.8.3)
project(rtt_kdl_conversions)

find_package(catkin REQUIRED COMPONENTS kdl_conversions)

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

include_directories(${catkin_INCLUDE_DIRS})

orocos_typekit(kdlconversions kdl_conversions-types.cpp)
target_link_libraries(kdlconversions ${catkin_LIBRARIES})

orocos_generate_package(
  DEPENDS kdl_conversions
)

