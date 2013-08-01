cmake_minimum_required(VERSION 2.8.3)
project(rtt_std_srvs)

find_package(catkin REQUIRED COMPONENTS rtt_rostopic std_srvs)

catkin_package(
    DEPENDS rtt_rostopic std_msgs
    CATKIN_DEPENDS rtt_rostopic std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

orocos_service(rtt_std_srvs_ros_service_proxies src/rtt_ros_service_proxies.cpp)
