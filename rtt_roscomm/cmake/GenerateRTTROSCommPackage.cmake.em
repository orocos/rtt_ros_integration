cmake_minimum_required(VERSION 2.8.3)

set(ROS_BUILD_TYPE MinSizeRel)
set(CMAKE_BUILD_TYPE MinSizeRel)

find_package(OROCOS-RTT 2.0.0 COMPONENTS rtt-scripting rtt-marshalling)

if (NOT OROCOS-RTT_FOUND)
  message(FATAL_ERROR "\n   RTT not found. Is the version correct? Use the CMAKE_PREFIX_PATH cmake or environment variable to point to the installation directory of RTT.")
else()
  include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
  add_definitions( -DRTT_COMPONENT )
endif()

include(AddFileDependencies)

macro(ros_generate_rtt_typekit package)
  # Check if we're generating code for messages in this package
  if(NOT package STREQUAL PROJECT_NAME)
    find_package(${package})
  endif()

  find_package(genmsg)
  find_package(rtt_roscomm)

  # Get all .msg files
  if(genmsg_VERSION VERSION_GREATER 0.4.19)
    set(MSGS ${${package}_MESSAGE_FILES})
  else()
    message(SEND_ERROR "genmsg version must be 0.4.19 or greater to generate RTT typekits for ROS messages")
  endif()
  
  if( NOT "${MSGS}" STREQUAL "" )

    # Set the boost header generation script path
    @[if DEVELSPACE]@
      set(CREATE_BOOST_HEADER_EXE_PATH @(CMAKE_CURRENT_SOURCE_DIR)/cmake/create_boost_header.py)
    @[else]@
      set(CREATE_BOOST_HEADER_EXE_PATH @(CMAKE_INSTALL_PREFIX)/@(CATKIN_PACKAGE_SHARE_DESTINATION)/cmake/create_boost_header.py)
    @[end if]@

    # Store the ros package name
    set(ROSPACKAGE ${package})

    # Generate code for each message type
    foreach( FILE ${MSGS} )
      # Get just the message name
      string(REGEX REPLACE ".+/\(.+\).msg" "\\1" ROSMSGNAME ${FILE})
      
      # Define the typenames for this message
      set(ROSMSGTYPE         "${package}::${ROSMSGNAME}")
      set(ROSMSGTYPENAME     "/${package}/${ROSMSGNAME}")
      set(ROSMSGCTYPENAME    "/${package}/c${ROSMSGNAME}")

      # msg_Types.hpp.in, ros_msg_typekit_plugin.cpp.in, ros_msg_typekit_package.cpp.in
      set(ROSMSGBOOSTHEADER  "${package}/boost/${ROSMSGNAME}.h")
      # ros_msg_typekit_plugin.cpp.in, ros_msg_typekit_package.cpp.in
      set(ROSMSGBOOSTHEADERS "${ROSMSGBOOSTHEADERS}#include <${package}/${ROSMSGNAME}.h>\n")
      # Types.hpp.in, ros_msg_typekit_package.cpp.in
      set(ROSMSGTYPES        "${ROSMSGTYPES}        rtt_ros_addType_${package}_${ROSMSGNAME}(); // factory function for adding TypeInfo.\n")
      # ros_msg_typekit_package.cpp.in
      set(ROSMSGTYPEDECL     "${ROSMSGTYPEDECL}        void rtt_ros_addType_${package}_${ROSMSGNAME}();\n")
      # ros_msg_typekit_plugin.cpp.in
      set(ROSMSGTYPELINE "
          void rtt_ros_addType_${package}_${ROSMSGNAME}() {
               // Only the .msg type is sent over ports. The msg[] (variable size) and  cmsg[] (fixed size) exist only as members of larger messages
               RTT::types::Types()->addType( new types::StructTypeInfo<${ROSMSGTYPE}>(\"${ROSMSGTYPENAME}\") );
               RTT::types::Types()->addType( new types::PrimitiveSequenceTypeInfo<std::vector<${ROSMSGTYPE}> >(\"${ROSMSGTYPENAME}[]\") );
               RTT::types::Types()->addType( new types::CArrayTypeInfo<RTT::types::carray<${ROSMSGTYPE}> >(\"${ROSMSGCTYPENAME}[]\") );
          }\n")
      # ros_msg_transport_package.cpp.in
      set(ROSMSGTRANSPORTS   "${ROSMSGTRANSPORTS}         if(name == \"${ROSMSGTYPENAME}\") { return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<${ROSMSGTYPE}>()); } else\n")
      # Types.hpp.in
      set(ROSMSGTYPESHEADERS "${ROSMSGTYPESHEADERS}#include \"${ROSMSGNAME}.h\"\n")

      # Necessary for create_boost_header.py command below
      set(_ROSMSG_GENERATED_BOOST_HEADER  "${CATKIN_DEVEL_PREFIX}/include/${ROSMSGBOOSTHEADER}")
      list(APPEND ROSMSGS_GENERATED_BOOST_HEADERS ${_ROSMSG_GENERATED_BOOST_HEADER})

      add_custom_command(
        OUTPUT ${_ROSMSG_GENERATED_BOOST_HEADER} 
        COMMAND ${CREATE_BOOST_HEADER_EXE_PATH} ${package} "${package}/${ROSMSGNAME}" ${FILE} ${_ROSMSG_GENERATED_BOOST_HEADER} 
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} 
        DEPENDS ${FILE} ${${package}_EXPORTED_TARGETS} ${CREATE_BOOST_HEADER_EXE_PATH}
        VERBATIM)

      #set_source_files_properties(${ROSMSGS_GENERATED_BOOST_HEADERS} PROPERTIES GENERATED TRUE)
      
      # TypeInfo object
      set(_template_types_src_dir "${rtt_roscomm_DIR}/rtt_roscomm_pkg_template/src/orocos/types")
      set(_template_types_dst_dir                 "${CMAKE_CURRENT_BINARY_DIR}/src/orocos/types")

      set(_template_typekit_src_dir "${rtt_roscomm_DIR}/rtt_roscomm_pkg_template/include/PKG_NAME/typekit")
      set(_template_typekit_dst_dir                    "${CATKIN_DEVEL_PREFIX}/include/${package}/typekit")

      configure_file( 
        ${_template_types_src_dir}/ros_msg_typekit_plugin.cpp.in 
        ${_template_types_dst_dir}/ros_${ROSMSGNAME}_typekit_plugin.cpp @@ONLY )
      
      # Transport for ROS
      configure_file( 
        ${_template_types_src_dir}/ros_msg_transport_plugin.cpp.in 
        ${_template_types_dst_dir}/ros_${ROSMSGNAME}_transport_plugin.cpp @@ONLY )
      
      # Types.hpp helper for extern templates
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
    
    # Both are equivalent
    configure_file( 
      ${_template_typekit_src_dir}/Types.hpp.in 
      ${_template_typekit_dst_dir}/Types.hpp @@ONLY )
    configure_file(
      ${_template_typekit_src_dir}/Types.h.in 
      ${_template_typekit_dst_dir}/Types.h @@ONLY )
    
    # Targets
    include_directories(${CATKIN_DEVEL_PREFIX}/include ${catkin_INCLUDE_DIRS})

    orocos_typekit(         rtt-${package}-typekit ${_template_types_dst_dir}/ros_${package}_typekit.cpp ${ROSMSG_TYPEKIT_PLUGINS})
    orocos_typekit(         rtt-${package}-ros-transport ${_template_types_dst_dir}/ros_${package}_transport.cpp )
    target_link_libraries(  rtt-${package}-typekit ${catkin_LIBRARIES})
    target_link_libraries(  rtt-${package}-ros-transport ${catkin_LIBRARIES})
    add_dependencies(       rtt-${package}-typekit ${${package}_EXPORTED_TARGETS})
    add_dependencies(       rtt-${package}-ros-transport ${${package}_EXPORTED_TARGETS})
    add_file_dependencies(  ${_template_types_dst_dir}/ros_${package}_typekit.cpp "${CMAKE_CURRENT_LIST_FILE}" ${ROSMSGS_GENERATED_BOOST_HEADERS} )
    add_file_dependencies(  ${_template_types_dst_dir}/ros_${package}_transport.cpp "${CMAKE_CURRENT_LIST_FILE}" ${ROSMSGS_GENERATED_BOOST_HEADERS} )

    set_directory_properties(PROPERTIES 
      ADDITIONAL_MAKE_CLEAN_FILES "${ROSMSG_TYPEKIT_PLUGINS};${ROSMSG_TRANSPORT_PLUGIN};${_template_types_dst_dir}/ros_${package}_typekit.cpp;${_template_types_dst_dir}/ros_${package}_transport.cpp;${CATKIN_DEVEL_PREFIX}/include/${package}/boost")

    # Install generated header files (dependent packages might need them)
    install(FILES ${ROSMSGS_GENERATED_BOOST_HEADERS} DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${package}/boost/)
    install(DIRECTORY "${CATKIN_DEVEL_PREFIX}/include/${package}/typekit/" DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${package}/typekit)

  else()
    # Return if nothing to do
    message(WARNING "ros_generate_rtt_typekit: Could not find any .msg files in the ${package} package.")
  endif()

endmacro(ros_generate_rtt_typekit)


macro(ros_generate_rtt_service_proxies package)
  # Check if we're generating code for services in this package
  if(NOT package STREQUAL PROJECT_NAME)
    find_package(${package})
  endif()

  find_package(genmsg)
  find_package(rtt_roscomm)

  # Get all .msg files
  if(genmsg_VERSION VERSION_GREATER 0.4.19)
    set(SRVS ${${package}_SERVICE_FILES})
  else()
    message(SEND_ERROR "genmsg version must be 0.4.19 or greater")
  endif()
  
  if ( NOT "${SRVS}" STREQUAL "" )

    # Get the ros package name
    set(ROSPACKAGE ${package})

    foreach( FILE ${SRVS} )
      # Extract the service name
      string(REGEX REPLACE ".+/\(.+\).srv" "\\1" ROS_SRV_NAME ${FILE})
      
      # Define the service typenames
      set(ROS_SRV_TYPE "${ROSPACKAGE}::${ROS_SRV_NAME}")
      set(ROS_SRV_TYPENAME "${ROSPACKAGE}/${ROS_SRV_NAME}")

      # rtt_ros_service_proxies.cpp.in
      set(ROS_SRV_HEADERS "${ROS_SRV_HEADERS}#include <${ROS_SRV_TYPENAME}.h>\n")
      set(ROS_SRV_FACTORIES "${ROS_SRV_PROXY_FACTORIES}  success = success && register_service_factory(new ROSServiceProxyFactory<${ROS_SRV_TYPE}>(\"${ROS_SRV_TYPENAME}\"));\n")

    endforeach()
    
    # Service proxy factories
    set(_template_proxies_src_dir "${rtt_roscomm_DIR}/rtt_roscomm_pkg_template/src")
    set(_template_proxies_dst_dir "${CMAKE_CURRENT_BINARY_DIR}/src")

    configure_file( 
      ${_template_proxies_src_dir}/rtt_ros_service_proxies.cpp.in 
      ${_template_proxies_dst_dir}/rtt_ros_service_proxies.cpp @@ONLY )

    add_file_dependencies( ${_template_proxies_dst_dir}/rtt_ros_service_proxies.cpp ${SRVS})
    
    # Targets
    include_directories(${CATKIN_DEVEL_PREFIX}/include ${catkin_INCLUDE_DIRS})

    orocos_service(         rtt_${ROSPACKAGE}_ros_service_proxies ${_template_proxies_dst_dir}/rtt_ros_service_proxies.cpp)
    target_link_libraries(  rtt_${ROSPACKAGE}_ros_service_proxies ${catkin_LIBRARIES})
    add_dependencies(       rtt_${ROSPACKAGE}_ros_service_proxies ${${package}_EXPORTED_TARGETS})
    add_file_dependencies(  ${_template_proxies_dst_dir}/rtt_ros_service_proxies.cpp "${CMAKE_CURRENT_LIST_FILE}")

    set_directory_properties(PROPERTIES 
      ADDITIONAL_MAKE_CLEAN_FILES "${_template_proxies_dst_dir}/rtt_ros_service_proxies.cpp")

  else()
    #Return if nothing to do:
    message(WARNING "ros_generate_rtt_service_proxies: Could not find any .srv files in the ${package} package.")
  endif()
  
endmacro(ros_generate_rtt_service_proxies)



function(rtt_roscomm_generate_package package)
  ros_generate_rtt_typekit(${package})
  ros_generate_rtt_service_proxies(${package})
  orocos_generate_package()
endfunction(rtt_roscomm_generate_package)
