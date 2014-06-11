cmake_minimum_required(VERSION 2.8.3)
project(rtt_dynamic_reconfigure_tests)

find_package(catkin REQUIRED COMPONENTS rtt_ros dynamic_reconfigure)

if(CATKIN_ENABLE_TESTING)

  generate_dynamic_reconfigure_options(
    cfg/Test.cfg
  )

  orocos_plugin(rtt_dynamic_reconfigure_tests_service test/service.cpp)
  add_dependencies(rtt_dynamic_reconfigure_tests_service ${PROJECT_NAME}_gencfg)

  catkin_add_gtest(rtt_dynamic_reconfigure_tests test/rtt_dynamic_reconfigure_tests.cpp)
  add_dependencies(rtt_dynamic_reconfigure_tests rtt_dynamic_reconfigure_tests_service)
  target_link_libraries(rtt_dynamic_reconfigure_tests
    ${catkin_LIBRARIES}
    ${USE_OROCOS_LIBRARIES}
    ${OROCOS-RTT_LIBRARIES}
    ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} )

  orocos_generate_package()

endif()
