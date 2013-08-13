cmake_minimum_required(VERSION 2.8.3)
project(rtt_kdl_conversions)

find_package(catkin REQUIRED COMPONENTS rtt tf_conversions)

catkin_package(
    DEPENDS rtt 
    CATKIN_DEPENDS tf_conversions
)

find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})#

orocos_typekit(kdlconversions kdl_conversions-types.cpp)
orocos_generate_package()

