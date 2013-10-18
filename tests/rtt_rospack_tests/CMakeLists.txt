cmake_minimum_required(VERSION 2.8.3)
project(rtt_rospack_tests)

find_package(catkin REQUIRED
  COMPONENTS rtt_ros rtt_rospack roslib)

orocos_use_package(ocl-logging)
orocos_use_package(ocl-deployment)

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
