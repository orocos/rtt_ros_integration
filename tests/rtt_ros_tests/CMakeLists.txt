cmake_minimum_required(VERSION 2.8.3)
project(rtt_ros_tests)

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

find_package(catkin REQUIRED
  COMPONENTS rtt_ros)

catkin_package()

if(CATKIN_ENABLE_TESTING)

  catkin_add_gtest(load_test test/load.cpp)
  target_link_libraries(load_test ${USE_OROCOS_LIBRARIES})

endif()



