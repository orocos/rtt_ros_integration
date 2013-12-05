cmake_minimum_required(VERSION 2.8.3)
project(rtt_roscomm)

find_package(catkin REQUIRED COMPONENTS roscpp rtt_ros)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rtt_ros
  CFG_EXTRAS GenerateRTTROSCommPackage.cmake
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

# ROS Message / Topic Support
orocos_service(rtt_rostopic
  src/rtt_rostopic_service.cpp
  src/ros_publish_activity.cpp)
target_link_libraries(rtt_rostopic ${catkin_LIBRARIES})

# ROS Service Support
orocos_service(rtt_rosservice_registry
  src/rtt_rosservice_registry_service.cpp)

orocos_service(rtt_rosservice
  src/rtt_rosservice_service.cpp)
target_link_libraries(rtt_rosservice rtt_rosservice_registry ${catkin_LIBRARIES})

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

# Generate install targets and pkg-config files
orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS roscpp
)

install(PROGRAMS 
  scripts/create_rtt_pkg
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

orocos_install_headers(
   DIRECTORY include/rtt_rostopic include/rtt_rosservice
   INSTALL ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/orocos
)

# Install cmake macros
file(COPY cmake/create_boost_header.py DESTINATION "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake")
install(PROGRAMS cmake/create_boost_header.py DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

# Install template files to both install and develspace
file(COPY src/templates DESTINATION "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/src")
install(DIRECTORY src/templates DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src)

# This one will only be used by create_rtt_pkg with `rospack find rtt_roscomm` => no devel-space installation
install(DIRECTORY rtt_roscomm_pkg_template DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
