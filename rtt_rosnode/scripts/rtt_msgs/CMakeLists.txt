cmake_minimum_required(VERSION 2.6.4)

catkin_project(@pkgname@)
set(CMAKE_BUILD_TYPE MinSizeRel)

find_package(rtt_rosnode)

rosbuild_include(rtt_rosnode GenerateRTTtypekit)

find_package(roscpp)

ros_generate_rtt_typekit(@pkgname@)


