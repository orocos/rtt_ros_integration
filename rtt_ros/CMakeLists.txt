cmake_minimum_required(VERSION 2.8.3)
project(rtt_ros)

find_package(catkin REQUIRED COMPONENTS rospack rostime)

find_package(OROCOS-RTT REQUIRED 
  COMPONENTS rtt-marshalling rtt-scripting)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

catkin_package(
  INCLUDE_DIRS include
  CFG_EXTRAS rtt_ros-extras.cmake
)

find_package(LibXml2 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${LIBXML2_INCLUDE_DIR})

add_definitions(-DRTT_COMPONENT)
orocos_service(rtt_ros src/rtt_ros_service.cpp)
target_link_libraries(rtt_ros ${catkin_LIBRARIES} ${LIBXML2_LIBRARIES})

orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rostime
)

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

orocos_install_headers(
  DIRECTORY include/${PROJECT_NAME}
)
