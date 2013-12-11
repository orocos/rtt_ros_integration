cmake_minimum_required(VERSION 2.8.3)
project(rtt_geometry_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm)

ros_generate_rtt_typekit(geometry_msgs)
ros_generate_rtt_service_proxies(geometry_msgs)

orocos_generate_package(
  DEPENDS geometry_msgs
  DEPENDS_TARGETS rtt_roscomm rtt_std_msgs
)
