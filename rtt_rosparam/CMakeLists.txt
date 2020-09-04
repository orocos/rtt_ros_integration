cmake_minimum_required(VERSION 2.8.3)
project(rtt_rosparam)

find_package(catkin REQUIRED COMPONENTS roscpp rtt_ros)

# Note: find_package(Eigen3) is not available in earlier Ubuntu versions (< trusty)
include(FindPkgConfig)
pkg_search_module(EIGEN3 REQUIRED eigen3)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS})

# build ROS param as a separate service
orocos_service(rtt_rosparam src/rtt_rosparam_service.cpp)
target_link_libraries(rtt_rosparam 
#  ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY} 
  ${catkin_LIBRARIES})

  # Add information coverage when needed
  if(CATKIN_ENABLE_TESTING)
    option(USE_CODE_COVERAGE "Enables information coverage for tests" OFF)
    if (USE_CODE_COVERAGE)
      find_package(cmake_common_scripts)
      # add_rostest_gtest calls target_link_libraries() in plain mode
      # target_code_coverage() calls target_link_libraries() in keyword mode
      # So policy CMP0023 needs to be deactivated to avoid the warning  
      if(POLICY CMP0023)
        #cmake_policy(SET CMP0023 OLD)
      endif()
      message(STATUS "Added information coverage in package: ${CMAKE_PROJECT_NAME}")
      add_code_coverage()
    endif()
  endif()
# if(CATKIN_ENABLE_TESTING AND USE_CODE_COVERAGE)
#   message(WARNING "Added specific target for coverage: ${CMAKE_PROJECT_NAME}::rtt_rosparam")
#   target_code_coverage(rtt_rosparam AUTO)
# endif()

# Generate install targets and pkg-config files
orocos_install_headers(include/rtt_rosparam/rosparam.h)
orocos_generate_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  DEPENDS roscpp eigen3
)
