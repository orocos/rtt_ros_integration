cmake_minimum_required(VERSION 2.8.3)
project(rtt_rosdeployment)

find_package(catkin REQUIRED COMPONENTS roscpp rtt_ros rtt_ros_msgs) 

include_directories(${catkin_INCLUDE_DIRS})

orocos_use_package(ocl-deployment)

orocos_plugin(rtt_rosdeployment src/rtt_rosdeployment_service.cpp)
target_link_libraries(rtt_rosdeployment ${catkin_LIBRARIES})

orocos_generate_package(
  DEPENDS rtt_ros_msgs
)
