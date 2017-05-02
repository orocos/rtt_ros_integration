# 
# Generate RTT typekits and plugins for using ROS .msg messages and .srv
# services
#

cmake_minimum_required(VERSION 2.8.3)

@[if DEVELSPACE]@
  set(rtt_roscomm_TEMPLATES_DIR @(PROJECT_SOURCE_DIR)/src/templates)
  set(rtt_roscomm_SCRIPTS_DIR @(PROJECT_SOURCE_DIR)/scripts)
@[else]@
  set(rtt_roscomm_TEMPLATES_DIR ${rtt_roscomm_DIR}/../src/templates)
  set(rtt_roscomm_SCRIPTS_DIR @(CMAKE_INSTALL_PREFIX)/@(CATKIN_PACKAGE_BIN_DESTINATION))
@[end if]@

macro(rtt_roscomm_destinations)
  if(ORO_USE_ROSBUILD)
    #message(STATUS "[ros_generate_rtt_typekit] Generating ROS typekit for ${PROJECT_NAME} with ROSBuild destinations.")
    set(rtt_roscomm_GENERATED_HEADERS_OUTPUT_DIRECTORY    "${PROJECT_SOURCE_DIR}/include")
    set(rtt_roscomm_GENERATED_HEADERS_INSTALL_DESTINATION)
  elseif(ORO_USE_CATKIN)
    #message(STATUS "[ros_generate_rtt_typekit] Generating ROS typekit for ${PROJECT_NAME} with Catkin destinations.")
    catkin_destinations()
    set(rtt_roscomm_GENERATED_HEADERS_OUTPUT_DIRECTORY    "${CATKIN_DEVEL_PREFIX}/include")
    set(rtt_roscomm_GENERATED_HEADERS_INSTALL_DESTINATION "${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  else()
    #message(STATUS "[ros_generate_rtt_typekit] Generating ROS typekit for ${PROJECT_NAME} with normal CMake destinations.")
    set(rtt_roscomm_GENERATED_HEADERS_OUTPUT_DIRECTORY    "${PROJECT_BINARY_DIR}/include")
    set(rtt_roscomm_GENERATED_HEADERS_INSTALL_DESTINATION "${CMAKE_INSTALL_PREFIX}/include")
  endif()

  if(DEFINED ENV{VERBOSE_CONFIG})
    message(STATUS "[ros_generate_rtt_typekit]   Generating headers in: ${rtt_roscomm_GENERATED_HEADERS_OUTPUT_DIRECTORY}")
    message(STATUS "[ros_generate_rtt_typekit]   Installing headers to: ${rtt_roscomm_GENERATED_HEADERS_INSTALL_DESTINATION}")
  endif()
endmacro()

macro(rtt_roscomm_debug)
  if(DEFINED ENV{VERBOSE_CONFIG})
    message(STATUS "[ros_generate_rtt_typekit]     catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")
  endif()
endmacro()

macro(ros_generate_rtt_typekit package)
  set(_package ${package})
  add_subdirectory(${rtt_roscomm_TEMPLATES_DIR}/typekit ${package}_typekit)
endmacro(ros_generate_rtt_typekit)

macro(ros_generate_rtt_service_proxies package)
  set(_package ${package})
  add_subdirectory(${rtt_roscomm_TEMPLATES_DIR}/service ${package}_service_proxies)
endmacro(ros_generate_rtt_service_proxies)
