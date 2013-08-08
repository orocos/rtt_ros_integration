cmake_minimum_required(VERSION 2.8.3)
project(rtt_ros)

find_package(catkin REQUIRED COMPONENTS roscpp rospack)

catkin_package(
  DEPENDS rtt ocl
  CATKIN_DEPENDS roscpp)

catkin_add_env_hooks(10.orocos SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(OROCOS-RTT REQUIRED rtt-marshalling)
find_package(LibXml2 REQUIRED)

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

include_directories(${catkin_INCLUDE_DIRS} ${LIBXML2_INCLUDE_DIR})

add_definitions(-DRTT_COMPONENT)
orocos_service(rtt_ros src/rtt_ros_service.cpp)
target_link_libraries(rtt_ros ${catkin_LIBRARIES} ${LIBXML2_LIBRARIES})

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
