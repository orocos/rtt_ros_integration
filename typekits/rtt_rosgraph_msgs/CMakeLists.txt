cmake_minimum_required(VERSION 2.8.3)
project(rtt_rosgraph_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm)

ros_generate_rtt_typekit(rosgraph_msgs)

orocos_generate_package(
  DEPENDS rosgraph_msgs
  DEPENDS_TARGETS rtt_roscomm rtt_std_msgs
)
