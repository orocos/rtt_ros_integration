cmake_minimum_required(VERSION 2.8.3)
project(rtt_rospack)

find_package(catkin REQUIRED COMPONENTS roslib rtt_ros)

#build ROS pack as a separate service
orocos_plugin(rtt_rospack src/rtt_rospack_service.cpp)
target_link_libraries(rtt_rospack ${roslib_LIBRARIES})

orocos_generate_package(
  DEPENDS roslib
)
