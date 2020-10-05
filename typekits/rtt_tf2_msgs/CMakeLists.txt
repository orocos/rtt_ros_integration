cmake_minimum_required(VERSION 2.8.3)
project(rtt_tf2_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm)

ros_generate_rtt_typekit(tf2_msgs)
ros_generate_rtt_service_proxies(tf2_msgs)

orocos_generate_package(
  DEPENDS tf2_msgs
  DEPENDS_TARGETS rtt_roscomm rtt_std_msgs rtt_actionlib_msgs rtt_geometry_msgs
)
