cmake_minimum_required(VERSION 2.8.3)
project(rtt_rospack_tests)

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

find_package(catkin REQUIRED
  COMPONENTS rtt_rospack roslib)

catkin_package()

if(CATKIN_ENABLE_TESTING)

  catkin_add_gtest(rtt_rospack_api_tests test/api_tests.cpp)
  target_link_libraries(rtt_rospack_api_tests
    ${catkin_LIBRARIES}
    ${USE_OROCOS_LIBRARIES}
    ${OROCOS-RTT_LIBRARIES}
    ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY})

endif()

orocos_generate_package()
