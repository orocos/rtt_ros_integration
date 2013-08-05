cmake_minimum_required(VERSION 2.8.3)
project(rtt_std_srvs)

find_package(OROCOS-RTT 2.0.0 COMPONENTS rtt-scripting rtt-marshalling)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

find_package(catkin REQUIRED COMPONENTS rtt_rosservice std_srvs)

catkin_package(
  CATKIN_DEPENDS rtt_rosservice std_srvs
)

include_directories(${catkin_INCLUDE_DIRS})

orocos_service(rtt_std_srvs_ros_service_proxies src/rtt_ros_service_proxies.cpp)
target_link_libraries(rtt_std_srvs_ros_service_proxies ${catkin_LIBRARIES})

orocos_generate_package()
