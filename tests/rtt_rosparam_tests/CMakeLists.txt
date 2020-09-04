cmake_minimum_required(VERSION 2.8.3)
project(rtt_rosparam_tests)

find_package(catkin REQUIRED COMPONENTS rtt_ros)

catkin_package()

if(CATKIN_ENABLE_TESTING)

  orocos_use_package(rtt_rosparam)
  orocos_use_package(rtt_geometry_msgs)
  
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
  endif()
  
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

  if (USE_CODE_COVERAGE)
    # find_package(catkin REQUIRED COMPONENTS rtt_rosparam)
    message(STATUS "Information coverage is used for package ${CMAKE_PROJECT_NAME}")
    # target_link_libraries(param_tests ${catkin_LIBRARIES})
    target_code_coverage(param_tests AUTO EXTERNAL EXCLUDE ${COVERAGE_EXCLUDE})
  endif()
  

endif()

# catkin build -v -DUSE_CODE_COVERAGE=ON --force-cmake --no-deps rtt_rosparam_tests --make-args ccov-clean
# catkin build -v -DUSE_CODE_COVERAGE=ON --force-cmake --no-deps rtt_rosparam_tests --make-args ccov-preprocessing
# catkin build -v -DUSE_CODE_COVERAGE=ON --force-cmake --no-deps rtt_rosparam_tests --make-args ccov-param_tests
# catkin build -v -DUSE_CODE_COVERAGE=ON --force-cmake --no-deps rtt_rosparam_tests --make-args ccov-capture-param_tests
# catkin build -v -DUSE_CODE_COVERAGE=ON --force-cmake --no-deps rtt_rosparam_tests --make-args ccov-run-param_tests

# If used AUTO there is:
# catkin build --no-deps rtt_rosparam_tests --catkin-make-args ccov

