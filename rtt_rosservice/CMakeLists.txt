cmake_minimum_required(VERSION 2.8.3)
project(rtt_rosservice)

catkin_package(
  INCLUDE_DIRS include 
  DEPENDS rtt ocl
  CATKIN_DEPENDS roscpp rtt_rospack)

find_package(catkin REQUIRED COMPONENTS roscpp rtt_rospack )
find_package(OROCOS-RTT REQUIRED rtt-marshalling)

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)


include_directories(
  include
  ${catkin_INCLUDE_DIRS})

orocos_service(rtt_rosservice
  src/rtt_rosservice.cpp
  src/ros_publish_activity.cpp)
target_link_libraries(rtt_rostopic ${catkin_LIBRARIES})

# Generate install targets for headers
#orocos_install_headers(
  #include/rtt_rostopic/rtt_rosservice.h
  #include/rtt_rostopic/ros_publish_activity.hpp)

# Generate install targets and pkg-config files
orocos_generate_package()

#install(DIRECTORY include/${PROJECT_NAME}/
   #DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   #FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
#)

# Install cmake macros
#install(FILES cmake/GenerateRTTtypekit.cmake cmake/create_boost_header.py
  #DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

# Install template files to both install and develspace
#install(DIRECTORY rtt_msg_pkg_template DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)
#file(COPY rtt_msg_pkg_template DESTINATION "${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake")

# Install tests
#install(FILES 
  #test/test.launch 
  #test/test.ops 
  #DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test)

