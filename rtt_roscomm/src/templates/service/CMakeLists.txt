find_package(OROCOS-RTT 2.0.0 QUIET COMPONENTS rtt-scripting rtt-marshalling)
if (NOT OROCOS-RTT_FOUND)
  message(FATAL_ERROR "\n   RTT not found. Is the version correct? Use the CMAKE_PREFIX_PATH cmake or environment variable to point to the installation directory of RTT.")
else()
  include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
  #add_definitions( -DRTT_COMPONENT )
endif()

include(AddFileDependencies)

# Configure source and destination paths of generated files
rtt_roscomm_destinations()

# Check if we're generating code for services in this package
if("${_package}" STREQUAL "${PROJECT_NAME}")
  set(${_package}_FOUND True)
else()
  find_package(${_package} QUIET)
endif()

find_package(genmsg REQUIRED)

# Get all .srv files
if(${_package}_FOUND)
  # Use catkin-based genmsg to find srv files
  if(genmsg_VERSION VERSION_GREATER 0.4.19)
    set(SRV_FILES)
    # TODO: genmsg API is unstable at this level
    foreach(FILE ${${_package}_SERVICE_FILES})
      if(IS_ABSOLUTE "${FILE}")
        list(APPEND SRV_FILES ${FILE})
      else()
        list(APPEND SRV_FILES ${${_package}_DIR}/../${FILE})
      endif()
    endforeach()
  else()
    message(SEND_ERROR "genmsg version must be 0.4.19 or greater")
  endif()
elseif(ROSBUILD_init_called)
  # try to find rosbuild-style message package
  rosbuild_find_ros_package(${_package})
  if(DEFINED ${_package}_PACKAGE_PATH)
    set(${_package}_FOUND TRUE)
    set(${_package}_INCLUDE_DIRS "${${_package}_PACKAGE_PATH}/include")
    file(GLOB SRV_FILES "${${_package}_PACKAGE_PATH}/srv/*.srv")
    set(${_package}_EXPORTED_TARGETS)
  endif()
else()
  message(SEND_ERROR "Package ${_package} not found. Will not generate RTT service proxy.")
  return()
endif()


#Return if nothing to do:
if ( "${SRV_FILES}" STREQUAL "" )
  message(STATUS "ros_generate_rtt_service_proxies: Could not find any .srv files in the ${_package} package.")
  return()
endif()

# Get the ros package name
set(ROSPACKAGE ${_package})

set(ROS_SRV_HEADERS)
set(ROS_SRV_FACTORIES)
foreach( FILE ${SRV_FILES} )
  # Extract the service name
  string(REGEX REPLACE ".+/\(.+\).srv" "\\1" ROS_SRV_NAME ${FILE})

  # Define the service typenames
  set(ROS_SRV_TYPE "${ROSPACKAGE}::${ROS_SRV_NAME}")
  set(ROS_SRV_TYPENAME "${ROSPACKAGE}/${ROS_SRV_NAME}")

  # rtt_rosservice_proxies.cpp.in
  set(ROS_SRV_HEADERS "${ROS_SRV_HEADERS}#include <${ROS_SRV_TYPENAME}.h>\n")
  set(ROS_SRV_FACTORIES "${ROS_SRV_FACTORIES}  success = success && register_service_factory(new ROSServiceProxyFactory<${ROS_SRV_TYPE}>(\"${ROS_SRV_TYPENAME}\"));\n")

endforeach()

# Service proxy factories
configure_file(
  rtt_rosservice_proxies.cpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/rtt_rosservice_proxies.cpp @ONLY )

add_file_dependencies( ${CMAKE_CURRENT_BINARY_DIR}/rtt_rosservice_proxies.cpp ${SRV_FILES})

include_directories(
  ${CATKIN_DEVEL_PREFIX}/include
  ${USE_OROCOS_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

# Targets
set(CMAKE_BUILD_TYPE MinSizeRel)
orocos_service(         rtt_${ROSPACKAGE}_rosservice_proxies ${CMAKE_CURRENT_BINARY_DIR}/rtt_rosservice_proxies.cpp)
target_link_libraries(  rtt_${ROSPACKAGE}_rosservice_proxies ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
if(DEFINED ${_package}_EXPORTED_TARGETS)
  add_dependencies(       rtt_${ROSPACKAGE}_rosservice_proxies ${${_package}_EXPORTED_TARGETS})
endif()
add_file_dependencies(  ${CMAKE_CURRENT_BINARY_DIR}/rtt_rosservice_proxies.cpp "${CMAKE_CURRENT_LIST_FILE}")

get_directory_property(_additional_make_clean_files ADDITIONAL_MAKE_CLEAN_FILES)
list(APPEND _additional_make_clean_files "${CMAKE_CURRENT_BINARY_DIR}/rtt_rosservice_proxies.cpp")
set_directory_properties(PROPERTIES
  ADDITIONAL_MAKE_CLEAN_FILES "${_additional_make_clean_files}")

# Export variables to the PARENT_SCOPE
set(OROCOS_DEFINED_PLUGINS ${OROCOS_DEFINED_PLUGINS} PARENT_SCOPE)
set(${PROJECT_NAME}_EXPORTED_TARGETS ${${PROJECT_NAME}_EXPORTED_TARGETS} PARENT_SCOPE)
set(${PROJECT_NAME}_EXPORTED_INCLUDE_DIRS ${${PROJECT_NAME}_EXPORTED_INCLUDE_DIRS} PARENT_SCOPE)
