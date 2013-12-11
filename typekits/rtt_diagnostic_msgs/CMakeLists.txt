cmake_minimum_required(VERSION 2.8.3)
project(rtt_diagnostic_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm)

ros_generate_rtt_typekit(diagnostic_msgs)
ros_generate_rtt_service_proxies(diagnostic_msgs)

orocos_generate_package(
  DEPENDS diagnostic_msgs
  DEPENDS_TARGETS rtt_roscomm rtt_std_msgs
)
