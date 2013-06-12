# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(rtt_rosgraph_msgs)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS rtt_rosnode rosgraph_msgs)

include_directories(include)
ros_generate_rtt_typekit(rosgraph_msgs)

catkin_package(
    DEPENDS rtt_rosnode rosgraph_msgs
    CATKIN_DEPENDS rtt_rosnode rosgraph_msgs
    INCLUDE_DIRS include
)
