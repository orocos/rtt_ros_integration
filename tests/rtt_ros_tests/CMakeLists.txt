cmake_minimum_required(VERSION 2.8.3)
project(rtt_ros_tests)

find_package(catkin REQUIRED COMPONENTS rtt_ros)

catkin_package()

if(CATKIN_ENABLE_TESTING)

  orocos_use_package(ocl-logging)
  orocos_use_package(ocl-deployment)

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

  catkin_add_gtest(rtt_ros_api_tests test/api_tests.cpp)
  target_link_libraries(rtt_ros_api_tests PRIVATE
    ${catkin_LIBRARIES}
    ${USE_OROCOS_LIBRARIES}
    ${OROCOS-RTT_LIBRARIES}
    ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY})

  if (USE_CODE_COVERAGE)
    message(STATUS "Information coverage is used for package ${CMAKE_PROJECT_NAME}: rtt_ros_api_tests")
    # In the next test, nothing from rtt_ros is actually linked: import() loads dynamically
    # That is why, if excluded ${CMAKE_CURRENT_LIST_DIR}/test/* no result exists
    find_package(catkin REQUIRED COMPONENTS rtt_ros)
    # Next line is to force linking the executable to the library that otherwise
    # is loaded dynamically. (example: devel/lib/librtt_ros-gnulinux.so)
    target_link_libraries(rtt_ros_api_tests ${${catkin_LIBRARIES}})
    # The effect is not as expected: the rtt_ros library doesn't produce
    # coverage information.
    target_code_coverage(rtt_ros_api_tests AUTO EXTERNAL EXCLUDE ${COVERAGE_EXCLUDE})
  endif()
endif()
