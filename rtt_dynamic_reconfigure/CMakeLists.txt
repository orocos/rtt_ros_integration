cmake_minimum_required(VERSION 2.8.3)
project(rtt_dynamic_reconfigure)

find_package(catkin REQUIRED COMPONENTS rtt_ros roscpp dynamic_reconfigure)

include_directories(
  include/orocos
  ${catkin_INCLUDE_DIRS}
)

orocos_library(rtt_dynamic_reconfigure src/auto_config.cpp)
target_link_libraries(rtt_dynamic_reconfigure ${catkin_LIBRARIES})

orocos_plugin(rtt_dynamic_reconfigure_service src/reconfigure_service.cpp)
target_link_libraries(rtt_dynamic_reconfigure_service rtt_dynamic_reconfigure)

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

orocos_generate_package(
  DEPENDS roscpp actionlib dynamic_reconfigure
  DEPENDS_TARGETS rtt_ros
)
