cmake_minimum_required(VERSION 2.8.3)
project(rtt_rosgraph_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm rosgraph_msgs)

ros_generate_rtt_typekit(rosgraph_msgs)

catkin_package(
    DEPENDS ocl rtt
    CATKIN_DEPENDS rtt_roscomm rosgraph_msgs
)

orocos_generate_package()
