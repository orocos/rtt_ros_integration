cmake_minimum_required(VERSION 2.8.3)
project(rtt_nav_msgs)

find_package(catkin REQUIRED COMPONENTS 
  rtt_roscomm 
  nav_msgs
  rtt_std_msgs  rtt_geometry_msgs  rtt_actionlib_msgs)

ros_generate_rtt_typekit(nav_msgs)
ros_generate_rtt_service_proxies(nav_msgs)

catkin_package(
  CATKIN_DEPENDS 
  rtt_roscomm 
  nav_msgs
  rtt_std_msgs  rtt_geometry_msgs  rtt_actionlib_msgs)

orocos_generate_package()
