cmake_minimum_required(VERSION 2.8.3)
project(rtt_std_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm)

include_directories(include/orocos)

ros_generate_rtt_typekit(std_msgs)

if(NOT DEFINED CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_BUILD_TYPE MinSizeRel)
endif()
orocos_plugin(rtt-ros-primitives-transport src/ros_primitives_transport_plugin.cpp)

install(
  DIRECTORY include/orocos/std_msgs
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/orocos
)

orocos_generate_package(
  DEPENDS std_msgs
  DEPENDS_TARGETS rtt_roscomm
)
