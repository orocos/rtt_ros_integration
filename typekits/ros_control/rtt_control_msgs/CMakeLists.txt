cmake_minimum_required(VERSION 2.8.3)
project(rtt_control_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm control_msgs)

ros_generate_rtt_typekit(control_msgs)
ros_generate_rtt_service_proxies(control_msgs)

orocos_generate_package(
  DEPENDS control_msgs 
  DEPENDS_TARGETS rtt_roscomm rtt_std_msgs rtt_geometry_msgs rtt_trajectory_msgs rtt_actionlib_msgs
)
