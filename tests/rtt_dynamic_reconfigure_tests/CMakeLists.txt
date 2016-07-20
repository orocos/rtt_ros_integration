cmake_minimum_required(VERSION 2.8.3)
project(rtt_dynamic_reconfigure_tests)

find_package(catkin REQUIRED COMPONENTS rtt_dynamic_reconfigure)

generate_dynamic_reconfigure_options(
  cfg/Test.cfg
)

catkin_package()

if(CATKIN_ENABLE_TESTING)

  include_directories(${USE_OROCOS_INCLUDE_DIRS})

  orocos_plugin(rtt_dynamic_reconfigure_tests_service test/service.cpp)
  add_dependencies(rtt_dynamic_reconfigure_tests_service ${PROJECT_NAME}_gencfg)

  orocos_component(rtt_dynamic_reconfigure_tests_component test/test_component.cpp)

  catkin_add_gtest(rtt_dynamic_reconfigure_tests test/rtt_dynamic_reconfigure_tests.cpp)
  add_dependencies(rtt_dynamic_reconfigure_tests rtt_dynamic_reconfigure_tests_service)
  orocos_configure_executable(rtt_dynamic_reconfigure_tests)
  target_link_libraries(rtt_dynamic_reconfigure_tests rtt_dynamic_reconfigure_tests_component)

endif()
