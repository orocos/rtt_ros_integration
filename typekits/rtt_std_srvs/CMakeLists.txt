cmake_minimum_required(VERSION 2.8.3)
project(rtt_std_srvs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm)

include_directories(include)

ros_generate_rtt_service_proxies(std_srvs
  EXTRA_INCLUDES
    rtt_std_srvs/Empty.h
    rtt_std_srvs/SetBool.h
    rtt_std_srvs/Trigger.h
)

orocos_generate_package(
  DEPENDS rosgraph_msgs
  DEPENDS_TARGETS rtt_roscomm rtt_std_msgs
)
