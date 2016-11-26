cmake_minimum_required(VERSION 2.8.3)
project(rtt_rosparam_tests)

find_package(catkin REQUIRED COMPONENTS rtt_ros)

catkin_package()

if(CATKIN_ENABLE_TESTING)

  orocos_use_package(rtt_rosparam)
  orocos_use_package(rtt_geometry_msgs)

  include_directories(
    ${catkin_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
    ${OROCOS-RTT_INCLUDE_DIRS}
  )

  find_package(rostest REQUIRED)
  add_rostest_gtest(param_tests test/param_tests.test test/param_tests.cpp)
  target_link_libraries(param_tests
    ${catkin_LIBRARIES}
    ${USE_OROCOS_LIBRARIES}
    ${OROCOS-RTT_LIBRARIES})

endif()
