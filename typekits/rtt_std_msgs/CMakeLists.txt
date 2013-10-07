cmake_minimum_required(VERSION 2.8.3)
project(rtt_std_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm std_msgs)

ros_generate_rtt_typekit(std_msgs)

catkin_package(
  CATKIN_DEPENDS rtt_roscomm std_msgs
)

orocos_generate_package()

