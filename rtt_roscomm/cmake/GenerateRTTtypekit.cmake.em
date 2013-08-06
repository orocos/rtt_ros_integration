
find_package(OROCOS-RTT 2.0.0 COMPONENTS rtt-scripting rtt-marshalling)

if (NOT OROCOS-RTT_FOUND)
  message(FATAL_ERROR "\n   RTT not found. Is the version correct? Use the CMAKE_PREFIX_PATH cmake or environment variable to point to the installation directory of RTT.")
else()
  include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
  add_definitions( -DRTT_COMPONENT )
endif()

set(ROS_BUILD_TYPE MinSizeRel)
set(CMAKE_BUILD_TYPE MinSizeRel)
include(AddFileDependencies)

macro(get_ros_msgs_external package msgs)

  # Find package with catkin
  # TODO: Possibly move this find_package call up into the caller scope?
  #find_package(catkin REQUIRED COMPONENTS ${package})

  # Make sure this package has a msg-paths cmake file
  set(_msg_paths_file "${${package}_DIR}/${package}-msg-paths.cmake")
  if(NOT EXISTS ${_msg_paths_file})
    message(SEND_ERROR "get_ros_msgs_external: Could not find ${package}-msg-paths.cmake file (should be at ${_msg_paths_file}).")
    return()
  endif()

  # Include the cmake file with the msg file paths
  include(${_msg_paths_file})

  # TODO: Set this to be empty, since _ROSBUILD_GENERATED_MSG_FILES is no longer used?
  set(${msgs} ${_ROSBUILD_GENERATED_MSG_FILES})

  foreach(_msg_dir ${${package}_MSG_INCLUDE_DIRS}) 
    file(GLOB _msg_files RELATIVE "${_msg_dir}" "${_msg_dir}/*.msg")
    # Loop over each .msg file, establishing a rule to compile it
    foreach(_msg ${_msg_files})
      # Make sure we didn't get a bogus match (e.g., .#Foo.msg, which Emacs
      # might create as a temporary file).  the file()
      # command doesn't take a regular expression, unfortunately.
      if(${_msg} MATCHES "^[^\\.].*\\.msg$")
        list(APPEND ${msgs} "${_msg_dir}/${_msg}")
      endif(${_msg} MATCHES "^[^\\.].*\\.msg$")
    endforeach(_msg)
  endforeach(_msg_dir)
endmacro(get_ros_msgs_external)


function(ros_generate_rtt_typekit package)

  find_package(catkin REQUIRED COMPONENTS genmsg rtt_roscomm roscpp ${package})

  orocos_use_package(rtt_roscomm)

  # Get all .msg files
  if(genmsg_VERSION VERSION_GREATER 0.4.19)
    set(MSGS ${${package}_MESSAGE_FILES})
  else()
    message("Using old get_ros_msgs_external")
    get_ros_msgs_external(${package} MSGS )
  endif()
  
  #Return if nothing to do:
  if ( "${MSGS}" STREQUAL "" )
    message(SEND_ERROR "ros_generate_rtt_typekit: Could not find any .msg files in the ${package} package.")
    return()
  endif()

  @[if DEVELSPACE]@
  set(CREATE_BOOST_HEADER_EXE_PATH @(CMAKE_CURRENT_SOURCE_DIR)/cmake)
  @[else]@
    set(CREATE_BOOST_HEADER_EXE_PATH "")
  @[end if]@


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
    set(_ROSMSG_GENERATED_BOOST_HEADER  "${CATKIN_DEVEL_PREFIX}/include/${ROSMSGBOOSTHEADER}")
    list(APPEND ROSMSGS_GENERATED_BOOST_HEADERS ${_ROSMSG_GENERATED_BOOST_HEADER})

    add_custom_command(
      OUTPUT ${_ROSMSG_GENERATED_BOOST_HEADER} 
      COMMAND ${CREATE_BOOST_HEADER_EXE_PATH}/create_boost_header.py ${package} "${package}/${ROSMSGNAME}" ${FILE} ${_ROSMSG_GENERATED_BOOST_HEADER} 
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} 
      DEPENDS ${FILE} 
      VERBATIM)
    # TODO: add custom target
    # add_dependencies(${_ROSMSG_GENERATED_BOOST_HEADER} @(CMAKE_CURRENT_SOURCE_DIR)/cmake/create_boost_header.py)

    #set_source_files_properties(${ROSMSGS_GENERATED_BOOST_HEADERS} PROPERTIES GENERATED TRUE)
    
    # TypeInfo object:
    set(_template_types_src_dir "${rtt_roscomm_DIR}/rtt_roscomm_pkg_template/src/orocos/types")
    set(_template_typekit_src_dir "${rtt_roscomm_DIR}/rtt_roscomm_pkg_template/include/PKG_NAME/typekit")

    set(_template_types_dst_dir "${CATKIN_DEVEL_PREFIX}/src/orocos/types")
    set(_template_typekit_dst_dir "${CATKIN_DEVEL_PREFIX}/include/${package}/typekit")

    include_directories("${CATKIN_DEVEL_PREFIX}/include")

    configure_file( 
      ${_template_types_src_dir}/ros_msg_typekit_plugin.cpp.in 
      ${_template_types_dst_dir}/ros_${ROSMSGNAME}_typekit_plugin.cpp @@ONLY )
    
    # Transport for ROS:
    configure_file( 
      ${_template_types_src_dir}/ros_msg_transport_plugin.cpp.in 
      ${_template_types_dst_dir}/ros_${ROSMSGNAME}_transport_plugin.cpp @@ONLY )
    
    # Types.hpp helper for extern templates:
    configure_file( 
      ${_template_typekit_src_dir}/msg_Types.hpp.in 
      ${_template_typekit_dst_dir}/${ROSMSGNAME}.h @@ONLY )
    
    list(APPEND ROSMSG_TYPEKIT_PLUGINS ${_template_types_dst_dir}/ros_${ROSMSGNAME}_typekit_plugin.cpp )
    list(APPEND ROSMSG_TRANSPORT_PLUGIN ${_template_types_dst_dir}/ros_${ROSMSGNAME}_transport_plugin.cpp )
    
    add_file_dependencies( ${_template_types_dst_dir}/ros_${package}_typekit.cpp ${FILE})
  endforeach( FILE ${MSGS} )
  
  configure_file( 
    ${_template_types_src_dir}/ros_msg_typekit_package.cpp.in 
    ${_template_types_dst_dir}/ros_${package}_typekit.cpp @@ONLY )
  
  configure_file( 
    ${_template_types_src_dir}/ros_msg_transport_package.cpp.in 
    ${_template_types_dst_dir}/ros_${package}_transport.cpp @@ONLY )
  
  # Both are equivalent:
  configure_file( 
    ${_template_typekit_src_dir}/Types.hpp.in 
    ${_template_types_dst_dir}/Types.hpp @@ONLY )
  configure_file(
    ${_template_typekit_src_dir}/Types.h.in 
    ${_template_typekit_dst_dir}/Types.h @@ONLY )
  
  orocos_typekit( rtt-${package}-typekit ${_template_types_dst_dir}/ros_${package}_typekit.cpp ${ROSMSG_TYPEKIT_PLUGINS})
  orocos_typekit( rtt-${package}-ros-transport ${_template_types_dst_dir}/ros_${package}_transport.cpp )
  add_file_dependencies( ${_template_types_dst_dir}/ros_${package}_typekit.cpp "${CMAKE_CURRENT_LIST_FILE}" ${ROSMSGS_GENERATED_BOOST_HEADERS} )
  add_file_dependencies( ${_template_types_dst_dir}/ros_${package}_transport.cpp "${CMAKE_CURRENT_LIST_FILE}" ${ROSMSGS_GENERATED_BOOST_HEADERS} )

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

  set_directory_properties(PROPERTIES 
    ADDITIONAL_MAKE_CLEAN_FILES "${ROSMSG_TYPEKIT_PLUGINS};${ROSMSG_TRANSPORT_PLUGIN};${_template_types_dst_dir}/ros_${package}_typekit.cpp;${_template_types_dst_dir}/ros_${package}_transport.cpp;${CATKIN_DEVEL_PREFIX}/include/${package}/boost")

  orocos_generate_package()
  
endfunction(ros_generate_rtt_typekit)

