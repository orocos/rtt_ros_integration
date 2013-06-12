# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(rtt_std_msgs)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS rtt_rosnode std_msgs)

# CATKIN_MIGRATION
# use include(module) after finding the path
# rosbuild_include(rtt_rosnode GenerateRTTtypekit)

include_directories(include)
ros_generate_rtt_typekit(std_msgs)

# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS rtt_rosnode std_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include
    LIBRARIES # TODO
)
