macro(rosbuild_get_msgs_external package msgs)
  rosbuild_find_ros_package(${package})
  
  file(GLOB _msg_files RELATIVE "${${package}_PACKAGE_PATH}/msg" "${${package}_PACKAGE_PATH}/msg/*.msg")
  set(${msgs} ${_ROSBUILD_GENERATED_MSG_FILES})
  # Loop over each .msg file, establishing a rule to compile it
  foreach(_msg ${_msg_files})
    # Make sure we didn't get a bogus match (e.g., .#Foo.msg, which Emacs
    # might create as a temporary file).  the file()
    # command doesn't take a regular expression, unfortunately.
    if(${_msg} MATCHES "^[^\\.].*\\.msg$")
      list(APPEND ${msgs} ${_msg})
    endif(${_msg} MATCHES "^[^\\.].*\\.msg$")
  endforeach(_msg)
endmacro(rosbuild_get_msgs_external)


macro(ros_generate_rtt_typekit package)

rosbuild_find_ros_package(rtt_ros_integration)
  
if(NOT EXISTS ${PROJECT_SOURCE_DIR}/include/${package}/boost)
   execute_process(COMMAND ${rtt_ros_integration_PACKAGE_PATH}/scripts/create_boost_headers.py ${package}
     WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
endif()
#endif()

#Get all .msg files
rosbuild_get_msgs_external(${package} MSGS)

foreach( FILE ${MSGS} )
  string(REGEX REPLACE "\(.+\).msg" "\\1" ROSMSGNAME ${FILE})

  set(ROSMSGTYPE "${package}::${ROSMSGNAME}")
  set(ROSMSGBOOSTHEADER "${package}/boost/${ROSMSGNAME}.h")

  configure_file( ${rtt_ros_integration_PACKAGE_PATH}/src/ros_msg_typekit_plugin.cpp.in 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${ROSMSGNAME}_typekit_plugin.cpp @ONLY )
  
  configure_file( ${rtt_ros_integration_PACKAGE_PATH}/src/ros_msg_transport_plugin.cpp.in 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${ROSMSGNAME}_transport_plugin.cpp @ONLY )
  
  orocos_typekit( rtt-ros-${ROSMSGNAME}-typekit ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${ROSMSGNAME}_typekit_plugin.cpp )
  orocos_typekit( rtt-ros-${ROSMSGNAME}-transport ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${ROSMSGNAME}_transport_plugin.cpp )

#  set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${ROSMSGNAME}_typekit_plugin.cpp)
#  set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${ROSMSGNAME}_transport_plugin.cpp)


endforeach( FILE ${MSGS} )

endmacro(ros_generate_rtt_typekit)

