cmake_minimum_required(VERSION 2.8.3)
project(rtt_dynamic_reconfigure_tests)

find_package(catkin REQUIRED COMPONENTS rtt_dynamic_reconfigure)

generate_dynamic_reconfigure_options(
  cfg/Test.cfg
)

catkin_package()

if(CATKIN_ENABLE_TESTING)

  include_directories(${USE_OROCOS_INCLUDE_DIRS})

  option(USE_CODE_COVERAGE "Enables information coverage for tests" OFF)
  if (USE_CODE_COVERAGE)
    find_package(cmake_common_scripts REQUIRED)
    # add_rostest_gtest calls target_link_libraries() in plain mode
    # target_code_coverage() calls target_link_libraries() in keyword mode
    # So policy CMP0023 needs to be deactivated to avoid the warning  
    if(POLICY CMP0023)
      cmake_policy(SET CMP0023 OLD)
    endif()
    set(COVERAGE_EXCLUDE /usr/* /opt/* ${CMAKE_CURRENT_LIST_DIR}/test/* /*/gtest/* /*/rtt/*)
    # add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE})
  endif()
  
  orocos_plugin(rtt_dynamic_reconfigure_tests_service test/service.cpp)
  add_dependencies(rtt_dynamic_reconfigure_tests_service ${PROJECT_NAME}_gencfg)

  orocos_component(rtt_dynamic_reconfigure_tests_component test/test_component.cpp)

  catkin_add_gtest(rtt_dynamic_reconfigure_tests test/rtt_dynamic_reconfigure_tests.cpp)
  add_dependencies(rtt_dynamic_reconfigure_tests rtt_dynamic_reconfigure_tests_service)
  
  orocos_configure_executable(rtt_dynamic_reconfigure_tests)
  target_link_libraries(rtt_dynamic_reconfigure_tests rtt_dynamic_reconfigure_tests_component)

  if (USE_CODE_COVERAGE)
    message(STATUS "Information coverage is used for package ${CMAKE_PROJECT_NAME}: rtt_dynamic_reconfigure_tests_service")
    target_code_coverage(rtt_dynamic_reconfigure_tests_service AUTO EXTERNAL EXCLUDE ${COVERAGE_EXCLUDE})
    message(STATUS "Information coverage is used for package ${CMAKE_PROJECT_NAME}: rtt_dynamic_reconfigure_tests")
    target_code_coverage(rtt_dynamic_reconfigure_tests AUTO EXTERNAL EXCLUDE ${COVERAGE_EXCLUDE})
  endif()

endif()
