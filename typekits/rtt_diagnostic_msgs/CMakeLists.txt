cmake_minimum_required(VERSION 2.8.3)
project(rtt_diagnostic_msgs)

find_package(catkin REQUIRED COMPONENTS 
  rtt_roscomm 
  diagnostic_msgs
  rtt_std_msgs)

ros_generate_rtt_typekit(diagnostic_msgs)
ros_generate_rtt_service_proxies(diagnostic_msgs)

catkin_package(
  CATKIN_DEPENDS 
  rtt_roscomm 
  diagnostic_msgs
  rtt_std_msgs)

orocos_generate_package()
