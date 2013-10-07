cmake_minimum_required(VERSION 2.8.3)
project(rtt_geometry_msgs)

find_package(catkin REQUIRED COMPONENTS 
  rtt_roscomm 
  geometry_msgs
  rtt_std_msgs)

ros_generate_rtt_typekit(geometry_msgs)
ros_generate_rtt_service_proxies(geometry_msgs)

catkin_package(
  DEPENDS ocl rtt
  CATKIN_DEPENDS 
  rtt_roscomm 
  geometry_msgs
  rtt_std_msgs)

orocos_generate_package()
