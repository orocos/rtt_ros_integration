
rosbuild_find_ros_package( rtt )
set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/install )
find_package(Orocos-RTT 2.0.0 COMPONENTS rtt-scripting rtt-marshalling ${RTT_HINTS})
if (NOT OROCOS-RTT_FOUND)
  message(FATAL_ERROR "\n   RTT not found. Is the version correct? Use the CMAKE_PREFIX_PATH cmake or environment variable to point to the installation directory of RTT.")
else()
  include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
  add_definitions( -DRTT_COMPONENT )
endif()
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE MinSizeRel)
endif(NOT CMAKE_BUILD_TYPE)


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
  rosbuild_find_ros_package(rtt )
  find_package(Orocos-RTT HINTS ${rtt_PACKAGE_PATH}/install )
  
  # Defines the orocos_* cmake macros. See that file for additional
  # documentation.
  include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
  
  if(NOT EXISTS ${PROJECT_SOURCE_DIR}/include/${package}/boost)
    execute_process(COMMAND ${rtt_ros_integration_PACKAGE_PATH}/scripts/create_boost_headers.py ${package}
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
  endif()

  #Get all .msg files
  rosbuild_get_msgs_external(${package} MSGS)
  
  foreach( FILE ${MSGS} )
    string(REGEX REPLACE "\(.+\).msg" "\\1" ROSMSGNAME ${FILE})
    
    set(ROSMSGTYPE "${package}::${ROSMSGNAME}")
    set(ROSMSGTYPENAME "/${package}/${ROSMSGNAME}")
    set(ROSMSGBOOSTHEADER "${package}/boost/${ROSMSGNAME}.h")
    set(ROSMSGBOOSTHEADERS "${ROSMSGBOOSTHEADERS}#include <${package}/${ROSMSGNAME}.h>\n")
    set(ROSMSGTYPES       "${ROSMSGTYPES}        rtt_ros_addType_${ROSMSGNAME}(); // factory function for adding TypeInfo.\n")
    set(ROSMSGTYPEDECL "${ROSMSGTYPEDECL}        void rtt_ros_addType_${ROSMSGNAME}();\n")
    set(ROSMSGTYPELINE "        void rtt_ros_addType_${ROSMSGNAME}() { RTT::types::Types()->addType( new types::StructTypeInfo<${ROSMSGTYPE}>(\"${ROSMSGTYPENAME}\") ); RTT::types::Types()->addType( new types::SequenceTypeInfo<std::vector<${ROSMSGTYPE}> >(\"${ROSMSGTYPENAME}[]\") ); }\n")
    set(ROSMSGTRANSPORTS "${ROSMSGTRANSPORTS}         if(name == \"${ROSMSGTYPENAME}\")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<${ROSMSGTYPE}>());
")
    
    
    configure_file( ${rtt_ros_integration_PACKAGE_PATH}/src/ros_msg_typekit_plugin.cpp.in 
      ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${ROSMSGNAME}_typekit_plugin.cpp @ONLY )
    
    configure_file( ${rtt_ros_integration_PACKAGE_PATH}/src/ros_msg_transport_plugin.cpp.in 
      ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${ROSMSGNAME}_transport_plugin.cpp @ONLY )
    
    list(APPEND ROSMSG_TYPEKIT_PLUGINS ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${ROSMSGNAME}_typekit_plugin.cpp )
    list(APPEND ROSMSG_TRANSPORT_PLUGIN ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${ROSMSGNAME}_transport_plugin.cpp )
    
  endforeach( FILE ${MSGS} )
  
  set(ROSPACKAGE ${package})
  configure_file( ${rtt_ros_integration_PACKAGE_PATH}/src/ros_msg_typekit_package.cpp.in 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${package}_typekit.cpp @ONLY )
  
  configure_file( ${rtt_ros_integration_PACKAGE_PATH}/src/ros_msg_transport_package.cpp.in 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${package}_transport.cpp @ONLY )
  
  orocos_typekit( rtt-ros-${package}-typekit ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${package}_typekit.cpp ${ROSMSG_TYPEKIT_PLUGINS})
  orocos_typekit( rtt-ros-${package}-transport ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${package}_transport.cpp )
  
  set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${ROSMSG_TYPEKIT_PLUGINS};${ROSMSG_TRANSPORT_PLUGIN};${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${package}_typekit.cpp;${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${package}_transport.cpp")
  
endmacro(ros_generate_rtt_typekit)

