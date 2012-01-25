
rosbuild_find_ros_package( rtt )
set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/../install )
find_package(OROCOS-RTT 2.0.0 COMPONENTS rtt-scripting rtt-marshalling ${RTT_HINTS})
if (NOT OROCOS-RTT_FOUND)
  message(FATAL_ERROR "\n   RTT not found. Is the version correct? Use the CMAKE_PREFIX_PATH cmake or environment variable to point to the installation directory of RTT.")
else()
  include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
  add_definitions( -DRTT_COMPONENT )
endif()
set(ROS_BUILD_TYPE MinSizeRel)
set(CMAKE_BUILD_TYPE MinSizeRel)
include(AddFileDependencies)

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
      list(APPEND ${msgs} "${${package}_PACKAGE_PATH}/msg/${_msg}")
    endif(${_msg} MATCHES "^[^\\.].*\\.msg$")
  endforeach(_msg)
endmacro(rosbuild_get_msgs_external)


function(ros_generate_rtt_typekit package)

  rosbuild_find_ros_package(rtt_rosnode)

  #Get all .msg files
  rosbuild_get_msgs_external(${package} MSGS )
  
  #Return if nothing to do:
  if ( "${MSGS}" STREQUAL "" )
    message("ros_generate_rtt_typekit: could not find any .msg files in the ${package} package.")
    return()
  endif()

  set(ROSPACKAGE ${package})
  foreach( FILE ${MSGS} )
    string(REGEX REPLACE ".+/msg/\(.+\).msg" "\\1" ROSMSGNAME ${FILE})
    
    set(ROSMSGTYPE "${package}::${ROSMSGNAME}")
    set(ROSMSGTYPENAME "/${package}/${ROSMSGNAME}")
    set(ROSMSGCTYPENAME "/${package}/c${ROSMSGNAME}")
    set(ROSMSGBOOSTHEADER "${package}/boost/${ROSMSGNAME}.h")
    set(ROSMSGBOOSTHEADERS "${ROSMSGBOOSTHEADERS}#include <${package}/${ROSMSGNAME}.h>\n")
    set(ROSMSGTYPES       "${ROSMSGTYPES}        rtt_ros_addType_${package}_${ROSMSGNAME}(); // factory function for adding TypeInfo.\n")
    set(ROSMSGTYPEDECL "${ROSMSGTYPEDECL}        void rtt_ros_addType_${package}_${ROSMSGNAME}();\n")
    set(ROSMSGTYPELINE "
        void rtt_ros_addType_${package}_${ROSMSGNAME}() {
             // Only the .msg type is sent over ports. The msg[] (variable size) and  cmsg[] (fixed size) exist only as members of larger messages
             RTT::types::Types()->addType( new types::StructTypeInfo<${ROSMSGTYPE}>(\"${ROSMSGTYPENAME}\") );
             RTT::types::Types()->addType( new types::PrimitiveSequenceTypeInfo<std::vector<${ROSMSGTYPE}> >(\"${ROSMSGTYPENAME}[]\") );
             RTT::types::Types()->addType( new types::CArrayTypeInfo<RTT::types::carray<${ROSMSGTYPE}> >(\"${ROSMSGCTYPENAME}[]\") );
        }\n")
    set(ROSMSGTRANSPORTS "${ROSMSGTRANSPORTS}         if(name == \"${ROSMSGTYPENAME}\")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<${ROSMSGTYPE}>());
")
    set(ROSMSGTYPESHEADERS "${ROSMSGTYPESHEADERS}#include \"${ROSMSGNAME}.h\"\n")

    # Necessary for create_boost_headers.py command below
    list(APPEND ROSMSGS_GENERATED_BOOST_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/${ROSMSGBOOSTHEADER}")
    
    # TypeInfo object:
    configure_file( ${rtt_rosnode_PACKAGE_PATH}/src/ros_msg_typekit_plugin.cpp.in 
      ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${ROSMSGNAME}_typekit_plugin.cpp @ONLY )
    
    # Transport for ROS:
    configure_file( ${rtt_rosnode_PACKAGE_PATH}/src/ros_msg_transport_plugin.cpp.in 
      ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${ROSMSGNAME}_transport_plugin.cpp @ONLY )
    
    # Types.hpp helper for extern templates:
    configure_file( ${rtt_rosnode_PACKAGE_PATH}/src/msg_Types.hpp.in 
      ${CMAKE_CURRENT_SOURCE_DIR}/include/${package}/typekit/${ROSMSGNAME}.h @ONLY )
    
    list(APPEND ROSMSG_TYPEKIT_PLUGINS ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${ROSMSGNAME}_typekit_plugin.cpp )
    list(APPEND ROSMSG_TRANSPORT_PLUGIN ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${ROSMSGNAME}_transport_plugin.cpp )
    
    add_file_dependencies( ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${package}_typekit.cpp ${FILE})
  endforeach( FILE ${MSGS} )
  
  add_custom_command(OUTPUT ${ROSMSGS_GENERATED_BOOST_HEADERS} COMMAND ${rtt_rosnode_PACKAGE_PATH}/scripts/create_boost_headers.py ${package}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} DEPENDS ${MSGS} VERBATIM)
  #set_source_files_properties(${ROSMSGS_GENERATED_BOOST_HEADERS} PROPERTIES GENERATED TRUE)

  configure_file( ${rtt_rosnode_PACKAGE_PATH}/src/ros_msg_typekit_package.cpp.in 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${package}_typekit.cpp @ONLY )
  
  configure_file( ${rtt_rosnode_PACKAGE_PATH}/src/ros_msg_transport_package.cpp.in 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${package}_transport.cpp @ONLY )
  
  # Both are equivalent:
  configure_file( ${rtt_rosnode_PACKAGE_PATH}/src/Types.hpp.in 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${package}/typekit/Types.hpp @ONLY )
  configure_file( ${rtt_rosnode_PACKAGE_PATH}/src/Types.h.in 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${package}/typekit/Types.h @ONLY )
  
  orocos_typekit( rtt-${package}-typekit ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${package}_typekit.cpp ${ROSMSG_TYPEKIT_PLUGINS})
  orocos_typekit( rtt-${package}-ros-transport ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${package}_transport.cpp )
  add_file_dependencies( ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${package}_typekit.cpp "${CMAKE_CURRENT_LIST_FILE}" ${ROSMSGS_GENERATED_BOOST_HEADERS} )
  add_file_dependencies( ${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${package}_transport.cpp "${CMAKE_CURRENT_LIST_FILE}" ${ROSMSGS_GENERATED_BOOST_HEADERS} )
  if (CMAKE_COMPILER_IS_GNUCXX)
    #
    # This fails on Ubuntu Lucid with gcc Ubuntu 4.4.3-4ubuntu5 and ld/binutils 2.20.1-system.20100303 and ld/binutils-gold 2.20.1-system.20100303) 1.9
    # This works on Ubuntu Maverick with gcc Ubuntu/Linaro 4.4.4-14ubuntu5 and ld/binutils 2.20.51-system.20100908
    # Main suspect is the compiler, but this has not been proven. We could enable this flag from gcc 4.5 on to be on the safe side.
    #
    # The failure is that a DataSource<T> of one .so can not be dynamic_cast'ed to a DataSource<T> of another .so
    #
    #set_target_properties( rtt-${package}-typekit PROPERTIES COMPILE_FLAGS "-fvisibility=hidden" )
    #set_target_properties( rtt-${package}-ros-transport PROPERTIES COMPILE_FLAGS "-fvisibility=hidden" )
  endif()

  set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${ROSMSG_TYPEKIT_PLUGINS};${ROSMSG_TRANSPORT_PLUGIN};${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${package}_typekit.cpp;${CMAKE_CURRENT_SOURCE_DIR}/src/orocos/types/ros_${package}_transport.cpp;${CMAKE_CURRENT_SOURCE_DIR}/include/${package}/boost")

  orocos_generate_package()
  
endfunction(ros_generate_rtt_typekit)

