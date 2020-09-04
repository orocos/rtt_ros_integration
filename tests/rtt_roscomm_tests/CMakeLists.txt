cmake_minimum_required(VERSION 2.8.3)
project(rtt_roscomm_tests)

find_package(catkin REQUIRED COMPONENTS rtt_ros)

catkin_package()

if(CATKIN_ENABLE_TESTING)

  orocos_use_package(ocl-logging)
  orocos_use_package(ocl-deployment)
  orocos_use_package(rtt_std_msgs)

  option(USE_CODE_COVERAGE "Enables information coverage for tests" OFF)
  if (USE_CODE_COVERAGE)
    find_package(cmake_common_scripts REQUIRED)
    # add_rostest_gtest calls target_link_libraries() in plain mode
    # target_code_coverage() calls target_link_libraries() in keyword mode
    # So policy CMP0023 needs to be deactivated to avoid the warning  
    if(POLICY CMP0023)
      cmake_policy(SET CMP0023 OLD)
    endif()
    # set(COVERAGE_EXCLUDE /usr/* /opt/* ${CMAKE_CURRENT_LIST_DIR}/test/* /*/gtest/* /*/rtt/*)
    set(COVERAGE_EXCLUDE /usr/* /opt/* /*/gtest/* /*/rtt/*)
  endif()

  include_directories(${USE_OROCOS_INCLUDE_DIRS})

  catkin_add_gtest(api_tests test/api_tests.cpp)
  target_link_libraries(api_tests PRIVATE
    ${catkin_LIBRARIES}
    ${USE_OROCOS_LIBRARIES}
    ${OROCOS-RTT_LIBRARIES}
    ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} )

  if (USE_CODE_COVERAGE)
    message(STATUS "Information coverage is used for package ${CMAKE_PROJECT_NAME}: api_tests")
    target_code_coverage(api_tests AUTO EXTERNAL EXCLUDE ${COVERAGE_EXCLUDE})
  endif()

  find_package(rostest REQUIRED)
  add_rostest_gtest(transport_tests test/transport_tests.test test/transport_tests.cpp)
  target_link_libraries(transport_tests PRIVATE
    ${catkin_LIBRARIES}
    ${USE_OROCOS_LIBRARIES}
    ${OROCOS-RTT_LIBRARIES}
  )

  if (USE_CODE_COVERAGE)
    message(STATUS "Information coverage is used for package ${CMAKE_PROJECT_NAME}: transport_tests")
    target_code_coverage(transport_tests AUTO EXTERNAL EXCLUDE ${COVERAGE_EXCLUDE})
  endif()

  add_custom_target(test_create_rtt_msgs ${CMAKE_CURRENT_SOURCE_DIR}/test/create_rtt_msgs/test_create_rtt_msgs.sh)
  add_dependencies(_run_tests_${PROJECT_NAME} test_create_rtt_msgs)

endif()
