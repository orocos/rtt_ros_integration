cmake_minimum_required(VERSION 2.8.3)
project(rtt_rosnode)

find_package(catkin REQUIRED COMPONENTS roscpp rtt_rospack)
find_package(OROCOS-RTT REQUIRED rtt-marshalling)

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

catkin_package(
  INCLUDE_DIRS include 
  DEPENDS rtt ocl
  CATKIN_DEPENDS roscpp  rtt_rospack 
  CFG_EXTRAS GenerateRTTtypekit.cmake)

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

orocos_plugin(rtt_rosnode 
  src/ros_plugin.cpp 
  src/ros_publish_activity.cpp
  src/rtt_ros_service.cpp)
target_link_libraries(rtt_rosnode ${catkin_LIBRARIES})

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

# Generate install targets and pkg-config files
orocos_generate_package()

install(PROGRAMS 
  scripts/create_boost_headers.py
  scripts/create_rtt_msgs
  scripts/rtt-upgrade-2.5
  # Wrapper scripts
  scripts/deployer
  scripts/orocreate-pkg
  scripts/orogen
  scripts/orogen-unregister
  scripts/rttlua
  scripts/rttlua-tlsf
  scripts/rttscript
  scripts/typegen
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY scripts/rtt_msgs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES cmake/GenerateRTTtypekit.cmake DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)
install(DIRECTORY cmake/pkg_template DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)
