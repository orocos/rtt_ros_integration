cmake_minimum_required(VERSION 2.8.3)
project(rtt_ros)

find_package(catkin REQUIRED COMPONENTS rostime rospack roslib)
find_package(LibXml2 REQUIRED)
find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

catkin_package(
  CFG_EXTRAS rtt_ros-extras.cmake
)

include_directories(include ${catkin_INCLUDE_DIRS} ${LIBXML2_INCLUDE_DIR})

add_definitions(-DRTT_COMPONENT)

orocos_library(rtt_ros src/rtt_ros.cpp)
target_link_libraries(rtt_ros
  ${catkin_LIBRARIES} 
  ${LIBXML2_LIBRARIES}
  )

orocos_service(rtt_ros_service src/rtt_ros_service.cpp)
target_link_libraries(rtt_ros_service
  rtt_ros
  ${catkin_LIBRARIES} 
  )

# ROS Primitives typekit
set(CMAKE_BUILD_TYPE MinSizeRel)
orocos_typekit(rtt-ros-primitives-typekit
  src/orocos/types/ros_primitives_typekit_plugin.cpp
  src/orocos/types/ros_time_type.cpp
  src/orocos/types/ros_uint8_type.cpp
  src/orocos/types/ros_int8_type.cpp
  src/orocos/types/ros_uint16_type.cpp
  src/orocos/types/ros_int16_type.cpp
  src/orocos/types/ros_uint32_type.cpp
  src/orocos/types/ros_int32_type.cpp
  src/orocos/types/ros_uint64_type.cpp
  src/orocos/types/ros_int64_type.cpp
  src/orocos/types/ros_float32_type.cpp
  src/orocos/types/ros_float64_type.cpp
  src/orocos/types/ros_string_type.cpp)
target_link_libraries(rtt-ros-primitives-typekit ${catkin_LIBRARIES})

orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rostime
)

install(DIRECTORY scripts/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

orocos_install_headers(
  DIRECTORY include/${PROJECT_NAME}/
)
