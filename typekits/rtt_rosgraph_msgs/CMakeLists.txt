cmake_minimum_required(VERSION 2.8.3)
project(rtt_rosgraph_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_rostopic rosgraph_msgs)

catkin_package(
    DEPENDS ocl rtt
    CATKIN_DEPENDS rtt_rostopic rosgraph_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
ros_generate_rtt_typekit(rosgraph_msgs)

