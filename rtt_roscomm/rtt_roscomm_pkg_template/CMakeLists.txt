cmake_minimum_required(VERSION 2.8.3)
project(rtt_@pkgname@)

find_package(catkin REQUIRED COMPONENTS 
  rtt_roscomm 
  @pkgname@
 @rtt_packages@)

ros_generate_rtt_typekit(@pkgname@)
ros_generate_rtt_service_proxies(@pkgname@)

catkin_package(
#  DEPENDS ocl rtt
  CATKIN_DEPENDS 
  rtt_roscomm 
  @pkgname@
 @rtt_packages@)

orocos_generate_package(
  DEPENDS @pkgname@
  DEPENDS_TARGETS rtt_roscomm @rtt_packages@
)
