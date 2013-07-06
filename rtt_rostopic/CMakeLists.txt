cmake_minimum_required(VERSION 2.8.3)
project(rtt_rostopic)

find_package(catkin REQUIRED COMPONENTS roscpp rtt_rospack rtt_ros)
find_package(OROCOS-RTT REQUIRED rtt-marshalling)

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

orocos_use_package(rtt_ros)

catkin_package(
  INCLUDE_DIRS include 
  DEPENDS rtt ocl
  CATKIN_DEPENDS roscpp  rtt_rospack 
  CFG_EXTRAS GenerateRTTtypekit.cmake)

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

orocos_service(rtt_rostopic
  src/rtt_ros_service.cpp
  src/ros_publish_activity.cpp)
target_link_libraries(rtt_rostopic ${catkin_LIBRARIES})

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
target_link_libraries(rtt-ros-primitives-typekit ${catkin_LIBRARIES})

# Generate install targets for headers
orocos_install_headers(
  include/rtt_rostopic/rtt_rostopic.h
  include/rtt_rostopic/ros_msg_transporter.hpp
  include/rtt_rostopic/ros_publish_activity.hpp)

# Generate install targets and pkg-config files
orocos_generate_package()

install(PROGRAMS 
  scripts/create_boost_headers.py
  scripts/create_rtt_msgs
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY scripts/rtt_msgs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES cmake/GenerateRTTtypekit.cmake DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)
install(DIRECTORY cmake/rtt_msg_pkg_template DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

install(FILES 
  test/test.launch 
  test/test.ops 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test)
