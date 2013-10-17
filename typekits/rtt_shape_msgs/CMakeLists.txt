cmake_minimum_required(VERSION 2.8.3)
project(rtt_shape_msgs)

find_package(catkin REQUIRED COMPONENTS 
  rtt_roscomm 
  shape_msgs
  rtt_std_msgs  rtt_geometry_msgs)

ros_generate_rtt_typekit(shape_msgs)
ros_generate_rtt_service_proxies(shape_msgs)

catkin_package(
  CATKIN_DEPENDS 
  rtt_roscomm 
  shape_msgs
  rtt_std_msgs  rtt_geometry_msgs)

orocos_generate_package()
