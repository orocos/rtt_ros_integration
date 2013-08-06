cmake_minimum_required(VERSION 2.8.3)
project(rtt_@pkgname@)

find_package(catkin REQUIRED COMPONENTS 
  rtt_roscomm 
  rtt_rostopic 
  rtt_rosservice 
  @pkgname@
@catkin_deplist@)

catkin_package(
  DEPENDS ocl rtt
  CATKIN_DEPENDS 
  rtt_roscomm 
  rtt_rostopic 
  rtt_rosservice 
  @pkgname@
@catkin_deplist@)

include_directories(${catkin_INCLUDE_DIRS})
ros_generate_rtt_typekit(@pkgname@)
ros_generate_rtt_service_proxies(@pkgname@)

orocos_generate_package()
