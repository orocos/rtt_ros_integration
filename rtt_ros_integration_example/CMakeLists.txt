cmake_minimum_required(VERSION 2.8.3)

project(rtt_ros_integration_example)

find_package(catkin REQUIRED COMPONENTS rtt_ros std_msgs)

find_package(OROCOS-RTT REQUIRED)

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

orocos_component(${PROJECT_NAME} src/HelloRobot.cpp)

catkin_package(
  DEPENDS rtt 
  CATKIN_DEPENDS rtt_ros std_msgs
)

orocos_generate_package()

install(FILES 
  example.launch 
  example.ops 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


