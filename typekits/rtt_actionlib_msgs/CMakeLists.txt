cmake_minimum_required(VERSION 2.8.3)
project(rtt_actionlib_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm)

ros_generate_rtt_typekit(actionlib_msgs)
ros_generate_rtt_service_proxies(actionlib_msgs)

orocos_generate_package(
  DEPENDS actionlib_msgs
  DEPENDS_TARGETS rtt_roscomm rtt_std_msgs
)
