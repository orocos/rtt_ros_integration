cmake_minimum_required(VERSION 2.8.3)
project(rtt_trajectory_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm)

ros_generate_rtt_typekit(trajectory_msgs)
ros_generate_rtt_service_proxies(trajectory_msgs)

orocos_generate_package(
  DEPENDS trajectory_msgs
  DEPENDS_TARGETS rtt_roscomm rtt_std_msgs rtt_geometry_msgs
)
