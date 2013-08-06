
find_package(OROCOS-RTT 2.0.0 COMPONENTS rtt-scripting rtt-marshalling)

if (NOT OROCOS-RTT_FOUND)
  message(FATAL_ERROR "\n   RTT not found. Is the version correct? Use the CMAKE_PREFIX_PATH cmake or environment variable to point to the installation directory of RTT.")
else()
  include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
  add_definitions( -DRTT_COMPONENT )
endif()

set(ROS_BUILD_TYPE MinSizeRel)
set(CMAKE_BUILD_TYPE MinSizeRel)

function(ros_generate_rtt_service_proxies package)

  find_package(catkin REQUIRED COMPONENTS genmsg rtt_rosservice roscpp ${package})

  orocos_use_package(rtt_rosservice)

  # Get all .msg files
  if(genmsg_VERSION VERSION_GREATER 0.4.19)
    set(SRVS ${${package}_SERVICE_FILES})
  endif()
  
  #Return if nothing to do:
  if ( "${SRVS}" STREQUAL "" )
    message(SEND_ERROR "ros_generate_rtt_service_proxies: Could not find any .srv files in the ${package} package.")
    return()
  endif()

  set(ROS_SRV_HEADERS "")
  set(ROS_SRV_FACTORIES "")
  set(ROSPACKAGE ${package})
  foreach( FILE ${SRVS} )
    string(REGEX REPLACE ".+/\(.+\).srv" "\\1" ROS_SRV_NAME ${FILE})
    
    set(ROS_SRV_TYPE "${ROSPACKAGE}::${ROS_SRV_NAME}")
    set(ROS_SRV_TYPENAME "${ROSPACKAGE}/${ROS_SRV_NAME}")

    set(ROS_SRV_HEADERS "${ROS_SRV_HEADERS}#include <${ROS_SRV_TYPENAME}.h>\n")
    set(ROS_SRV_FACTORIES "${ROS_SRV_PROXY_FACTORIES}  success = success && register_service_factory(new ROSServiceProxyFactory<${ROS_SRV_TYPE}>(\"${ROS_SRV_TYPENAME}\"));\n")
  endforeach()
  
  # TypeInfo object:
  set(_template_proxies_src_dir "${rtt_rosservice_DIR}/rtt_srv_pkg_template/src")
  set(_template_proxies_dst_dir "${CATKIN_DEVEL_PREFIX}/src")

  configure_file( 
    ${_template_proxies_src_dir}/rtt_ros_service_proxies.cpp.in 
    ${_template_proxies_dst_dir}/rtt_ros_service_proxies.cpp @@ONLY )
  
  include_directories(${catkin_INCLUDE_DIRS})
  orocos_service(rtt_${ROSPACKAGE}_ros_service_proxies ${_template_proxies_dst_dir}/rtt_ros_service_proxies.cpp)
  target_link_libraries(rtt_${ROSPACKAGE}_ros_service_proxies ${catkin_LIBRARIES})

  set_directory_properties(PROPERTIES 
    ADDITIONAL_MAKE_CLEAN_FILES "${_template_proxies_dst_dir}/rtt_ros_service_proxies.cpp")

  orocos_generate_package()
  
endfunction(ros_generate_rtt_service_proxies)

