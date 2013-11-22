cmake_minimum_required(VERSION 2.8.3)
project(rtt_sensor_msgs)

find_package(catkin REQUIRED COMPONENTS 
  rtt_roscomm 
  sensor_msgs
  rtt_std_msgs  rtt_geometry_msgs)

ros_generate_rtt_typekit(sensor_msgs)
ros_generate_rtt_service_proxies(sensor_msgs)

catkin_package(
  CATKIN_DEPENDS 
  rtt_roscomm 
  sensor_msgs
  rtt_std_msgs  rtt_geometry_msgs)

orocos_generate_package(
  DEPENDS sensor_msgs
  DEPENDS_TARGETS rtt_roscomm rtt_std_msgs rtt_geometry_msgs
)
