cmake_minimum_required(VERSION 2.8.3)
project(rtt_rosnode)

find_package(catkin REQUIRED COMPONENTS roscpp rtt_ros)

include_directories(${catkin_INCLUDE_DIRS})

# Create orocos rosnode plugin
orocos_plugin(rtt_rosnode src/ros_plugin.cpp)
target_link_libraries(rtt_rosnode ${catkin_LIBRARIES})

# Generate install targets and pkg-config files
orocos_generate_package(
  DEPENDS roscpp
)
