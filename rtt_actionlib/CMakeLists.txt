cmake_minimum_required(VERSION 2.8.3)
project(rtt_actionlib)

find_package(catkin REQUIRED COMPONENTS
  roscpp rtt_ros actionlib rtt_roscomm)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS})

# ROS Message / Topic Support
orocos_service(rtt_actionlib_service src/rtt_actionlib_service.cpp)
target_link_libraries(rtt_actionlib_service ${catkin_LIBRARIES})

orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS roscpp actionlib
  DEPENDS_TARGETS rtt_ros rtt_roscomm
)

orocos_install_headers(
  DIRECTORY include/${PROJECT_NAME}/
)
