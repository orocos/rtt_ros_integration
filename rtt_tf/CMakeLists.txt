cmake_minimum_required(VERSION 2.8.3)
project(rtt_tf)

find_package(catkin REQUIRED COMPONENTS tf rtt_ros rtt_roscomm tf2)

ros_generate_rtt_typekit(tf)

include_directories(${catkin_INCLUDE_DIRS})

orocos_component(rtt_tf rtt_tf-component.cpp)
target_link_libraries(rtt_tf ${catkin_LIBRARIES})
orocos_install_headers( rtt_tf-component.hpp )
orocos_install_headers(include/rtt_tf/tf_interface.h)

orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS tf
  DEPENDS_TARGETS rtt_roscomm rtt_geometry_msgs
)

# Tests
if(CATKIN_ENABLE_TESTING)
  orocos_use_package(ocl-taskbrowser REQUIRED)
  orocos_use_package(ocl-deployment REQUIRED)

  add_definitions(-DRTT_COMPONENT)
  orocos_executable(broadcaster_test tests/broadcaster_test.cpp)
  target_link_libraries(broadcaster_test ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
  orocos_executable(lookup_test tests/lookup_test.cpp)
  target_link_libraries(lookup_test ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
endif()
