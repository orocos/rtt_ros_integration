cmake_minimum_required(VERSION 2.8.3)
project(rtt_dynamic_reconfigure_tests)

find_package(catkin REQUIRED COMPONENTS rtt_dynamic_reconfigure)

generate_dynamic_reconfigure_options(
  cfg/Test.cfg
)

orocos_use_package(rtt_dynamic_reconfigure)

orocos_plugin(rtt_dynamic_reconfigure_tests_service service.cpp)
add_dependencies(rtt_dynamic_reconfigure_tests_service ${PROJECT_NAME}_gencfg)
