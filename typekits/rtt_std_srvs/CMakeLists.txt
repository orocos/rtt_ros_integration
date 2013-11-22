cmake_minimum_required(VERSION 2.8.3)
project(rtt_std_srvs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm std_srvs)

catkin_package(CATKIN_DEPENDS rtt_roscomm std_srvs)

ros_generate_rtt_service_proxies(std_srvs)

orocos_generate_package(
  DEPENDS rosgraph_msgs
  DEPENDS_TARGETS rtt_roscomm rtt_std_msgs
)
