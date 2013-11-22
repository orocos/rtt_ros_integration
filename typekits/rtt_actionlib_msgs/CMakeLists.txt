cmake_minimum_required(VERSION 2.8.3)
project(rtt_actionlib_msgs)

find_package(catkin REQUIRED COMPONENTS 
  rtt_roscomm 
  actionlib_msgs
  rtt_std_msgs)

ros_generate_rtt_typekit(actionlib_msgs)
ros_generate_rtt_service_proxies(actionlib_msgs)

catkin_package(
  CATKIN_DEPENDS 
  rtt_roscomm 
  actionlib_msgs
  rtt_std_msgs)

orocos_generate_package(
  DEPENDS actionlib_msgs
  DEPENDS_TARGETS rtt_roscomm rtt_std_msgs
)
