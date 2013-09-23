cmake_minimum_required(VERSION 2.8.3)
project(rtt_actionlib)

find_package(catkin REQUIRED COMPONENTS
  roscpp rtt_ros actionlib rtt_roscomm)
find_package(OROCOS-RTT REQUIRED rtt-marshalling)

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

catkin_package(
  INCLUDE_DIRS include 
  DEPENDS rtt ocl
  CATKIN_DEPENDS roscpp rtt_roscomm actionlib)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS})

# ROS Message / Topic Support
orocos_service(rtt_actionlib_service src/rtt_actionlib_service.cpp)
target_link_libraries(rtt_actionlib_service ${catkin_LIBRARIES})

orocos_generate_package()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

