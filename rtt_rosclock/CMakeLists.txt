cmake_minimum_required(VERSION 2.8.3)
project(rtt_rosclock)

find_package(catkin REQUIRED COMPONENTS roscpp rtt_ros rospack rostime cmake_modules rosgraph_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

orocos_library(rtt_rosclock
  src/rtt_rosclock.cpp
  src/rtt_rosclock_sim_clock_thread.cpp
  src/rtt_rosclock_sim_clock_activity.cpp
  src/rtt_rosclock_sim_clock_activity_manager.cpp)
target_link_libraries(rtt_rosclock ${catkin_LIBRARIES})

orocos_service(sim_clock_activity_service
  src/rtt_rosclock_sim_clock_activity_service.cpp)
target_link_libraries(sim_clock_activity_service
  rtt_rosclock ${catkin_LIBRARIES})

# Link against Xenomai POSIX skin to get
if(OROCOS_TARGET STREQUAL "xenomai")
  set(CMAKE_MODULE_PATH  ${CMAKE_CURRENT_SOURCE_DIR}/config)
  find_package(Xenomai REQUIRED)
  if(NOT XENOMAI_POSIX_FOUND)
      message(FATAL_ERROR "Xenomai Posix skin is required for rtt_rosclock")
  endif()
  include_directories(${Xenomai_POSIX_INCLUDE_DIRS})
  set_target_properties(rtt_rosclock PROPERTIES 
    LINK_FLAGS ${XENOMAI_POSIX_LDFLAGS}
    COMPILE_FLAGS ${XENOMAI_POSIX_CFLAGS} __XENO__
  )
  target_link_libraries(rtt_rosclock ${XENOMAI_POSIX_LIBRARIES})
endif()

orocos_service(rtt_rosclock_service src/rtt_rosclock_service.cpp)
target_link_libraries(rtt_rosclock_service rtt_rosclock ${catkin_LIBRARIES} )


orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rostime
)

orocos_install_headers(
  DIRECTORY include/${PROJECT_NAME}/
)
