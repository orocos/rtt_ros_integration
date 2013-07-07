cmake_minimum_required(VERSION 2.8.3)
project(rtt_std_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_rostopic std_msgs)

catkin_package(
    DEPENDS rtt_rostopic std_msgs
    CATKIN_DEPENDS rtt_rostopic std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
ros_generate_rtt_typekit(std_msgs)


