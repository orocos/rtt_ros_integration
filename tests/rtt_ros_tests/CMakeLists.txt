cmake_minimum_required(VERSION 2.8.3)
project(rtt_ros_tests)

find_package(catkin REQUIRED
  COMPONENTS rtt_ros)

catkin_package()

if(CATKIN_ENABLE_TESTING)

  catkin_add_gtest(rtt_ros_api_tests test/api_tests.cpp)
  target_link_libraries(rtt_ros_api_tests
    ${USE_OROCOS_LIBRARIES}
    ${OROCOS-RTT_LIBRARIES}
    ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY})

endif()

orocos_generate_package()
