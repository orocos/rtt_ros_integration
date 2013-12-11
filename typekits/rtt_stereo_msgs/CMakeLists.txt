cmake_minimum_required(VERSION 2.8.3)
project(rtt_stereo_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm)

ros_generate_rtt_typekit(stereo_msgs)
ros_generate_rtt_service_proxies(stereo_msgs)

orocos_generate_package(
  DEPENDS stereo_msgs
  DEPENDS_TARGETS rtt_roscomm rtt_std_msgs rtt_geometry_msgs rtt_sensor_msgs
)
