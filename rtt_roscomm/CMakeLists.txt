cmake_minimum_required(VERSION 2.8.3)
project(rtt_roscomm)

find_package(catkin REQUIRED COMPONENTS roscpp rtt_rospack )
find_package(OROCOS-RTT REQUIRED rtt-marshalling)

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

catkin_package(
  INCLUDE_DIRS include 
  DEPENDS rtt ocl
  CATKIN_DEPENDS roscpp  rtt_rospack 
  CFG_EXTRAS GenerateRTTROSCommPackage.cmake)

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

# ROS Message / Topic Support
orocos_service(rtt_rostopic
  src/rtt_rostopic_service.cpp
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

# ROS Service Support
orocos_service(rtt_rosservice_registry
  src/rtt_rosservice_registry_service.cpp)

orocos_service(rtt_rosservice
  src/rtt_rosservice_service.cpp)
target_link_libraries(rtt_rosservice ${catkin_LIBRARIES})

# Generate install targets and pkg-config files
orocos_generate_package()



install(PROGRAMS 
  scripts/create_rtt_pkg
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Install cmake macros
install(PROGRAMS cmake/create_boost_header.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

# Install template files to both install and develspace
install(DIRECTORY rtt_roscomm_pkg_template DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
file(COPY rtt_roscomm_pkg_template DESTINATION "${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}")

# Install tests
install(FILES 
  test/test.launch 
  test/test.ops 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test)
