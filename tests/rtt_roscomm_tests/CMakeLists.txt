cmake_minimum_required(VERSION 2.8.3)
project(rtt_roscomm_tests)

find_package(catkin REQUIRED COMPONENTS rtt_ros)

catkin_package()

if(CATKIN_ENABLE_TESTING)

  orocos_use_package(ocl-logging)
  orocos_use_package(ocl-deployment)
  orocos_use_package(rtt_std_msgs)

  include_directories(${USE_OROCOS_INCLUDE_DIRS})

  catkin_add_gtest(api_tests test/api_tests.cpp)
  target_link_libraries(api_tests
    ${catkin_LIBRARIES}
    ${USE_OROCOS_LIBRARIES}
    ${OROCOS-RTT_LIBRARIES}
    ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} )

  find_package(rostest REQUIRED)
  add_rostest_gtest(transport_tests test/transport_tests.test test/transport_tests.cpp)
  target_link_libraries(transport_tests
    ${catkin_LIBRARIES}
    ${USE_OROCOS_LIBRARIES}
    ${OROCOS-RTT_LIBRARIES})

  add_custom_target(test_create_rtt_msgs ${CMAKE_CURRENT_SOURCE_DIR}/test/create_rtt_msgs/test_create_rtt_msgs.sh)
  add_dependencies(_run_tests_${PROJECT_NAME} test_create_rtt_msgs)

endif()
