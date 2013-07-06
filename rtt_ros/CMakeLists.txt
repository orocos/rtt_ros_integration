cmake_minimum_required(VERSION 2.8.3)
project(rtt_ros)

find_package(catkin REQUIRED COMPONENTS roscpp rtt_rospack)
find_package(OROCOS-RTT REQUIRED rtt-marshalling)

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

catkin_package(
  DEPENDS rtt ocl
  CATKIN_DEPENDS roscpp  rtt_rospack)

include_directories(${catkin_INCLUDE_DIRS})

# Create orocos rosnode plugin
orocos_plugin(rtt_ros src/ros_plugin.cpp)
target_link_libraries(rtt_ros ${catkin_LIBRARIES})

# Generate install targets and pkg-config files
orocos_generate_package()

install(PROGRAMS 
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

install(FILES 
  test/test.launch 
  test/test.ops 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test)
