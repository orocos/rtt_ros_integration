cmake_minimum_required(VERSION 2.8.3)
project(rtt_roscomm_tests)

find_package(catkin REQUIRED
  COMPONENTS rtt_ros rtt_roscomm rtt_std_msgs rostest)

catkin_package()

if(CATKIN_ENABLE_TESTING)

  catkin_add_gtest(rtt_roscomm_api_tests test/api_tests.cpp)
  target_link_libraries(rtt_roscomm_api_tests 
    ${USE_OROCOS_LIBRARIES}
    ${OROCOS-RTT_LIBRARIES}
    ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} )

  #add_rostest(test/connpolicy/connpolicy.test)
endif()

orocos_generate_package()
