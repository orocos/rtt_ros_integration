cmake_minimum_required(VERSION 2.8.3)
project(rtt_rosservice)

catkin_package(
  INCLUDE_DIRS include 
  DEPENDS rtt ocl
  CATKIN_DEPENDS roscpp rtt_rospack
  CFG_EXTRAS GenerateServiceProxies.cmake)

find_package(catkin REQUIRED COMPONENTS roscpp rtt_rospack )
find_package(OROCOS-RTT REQUIRED rtt-marshalling)

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)


include_directories(
  include
  ${catkin_INCLUDE_DIRS})

orocos_service(rtt_rosservice_registry
  src/rtt_rosservice_registry.cpp)

orocos_service(rtt_rosservice
  src/rtt_rosservice.cpp)
target_link_libraries(rtt_rosservice ${catkin_LIBRARIES})

# Generate install targets and pkg-config files
orocos_generate_package()

# Install template files to both install and develspace
install(DIRECTORY rtt_srv_pkg_template DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)
file(COPY rtt_srv_pkg_template DESTINATION "${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/cmake")
