cmake_minimum_required(VERSION 2.4.6)

project(rtt_rosnode)

find_package(OROCOS-RTT REQUIRED rtt-marshalling)
find_package(catkin REQUIRED COMPONENTS roscpp)
# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${catkin_INCLUDE_DIRS})

set(SKIP_PKG_CONFIG_GENERATION TRUE)
catkin_package(
  INCLUDE_DIRS include src
  CATKIN_DEPENDS roscpp rtt 
  CFG_EXTRAS GenerateRTTtypekit.cmake
  SKIP_PKG_CONFIG_GENERATION )


#common commands for building c++ executables and libraries
orocos_plugin(rtt_rosnode src/ros_plugin.cpp src/ros_publish_activity.cpp src/rtt_ros_service.cpp)
target_link_libraries(rtt_rosnode ${catkin_LIBRARIES})

# build ROS param as a separate service
orocos_service(rtt_rosparam_service src/rtt_rosparam_service.cpp)
target_link_libraries(rtt_rosparam_service ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY} ${catkin_LIBRARIES})

orocos_typekit( rtt-ros-primitives-typekit ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_primitives_typekit_plugin.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_time_type.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_uint8_type.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_int8_type.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_uint16_type.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_int16_type.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_uint32_type.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_int32_type.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_uint64_type.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_int64_type.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_float32_type.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_float64_type.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_string_type.cpp )
target_link_libraries(rtt-ros-primitives-typekit ${catkin_LIBRARIES})

catkin_python_setup()

install(DIRECTORY include/ src/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(PROGRAMS scripts/create_boost_headers.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY scripts/rtt_msgs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/create_rtt_msgs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES cmake/GenerateRTTtypekit.cmake DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)



orocos_generate_package()
