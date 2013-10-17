cmake_minimum_required(VERSION 2.8.3)
project(rtt_ros)

find_package(catkin REQUIRED COMPONENTS roscpp rospack)

find_package(OROCOS-RTT REQUIRED 
  COMPONENTS rtt-marshalling rtt-scripting)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  CFG_EXTRAS rtt_ros-extras.cmake)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(LibXml2 REQUIRED)

# This adds orocos environment variables to the catkin setup.sh
catkin_add_env_hooks(10.orocos SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

include_directories(include ${catkin_INCLUDE_DIRS} ${LIBXML2_INCLUDE_DIR})

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

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

