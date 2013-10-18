# 
# Generate RTT typekits and plugins for using ROS .msg messages and .srv
# services
#

cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_BUILD_TYPE MinSizeRel)

include(AddFileDependencies)

macro(rtt_roscomm_destinations)
  if(ORO_USE_ROSBUILD)
    message(STATUS "[ros_generate_rtt_typekit] Generating ROS typekit for ${PROJECT_NAME} with ROSBuild destinations.")
    set(rtt_roscomm_GENERATED_HEADERS_OUTPUT_DIRECTORY    "${PROJECT_SOURCE_DIR}/include")
    set(rtt_roscomm_GENERATED_SOURCES_OUTPUT_DIRECTORY    "${CMAKE_CURRENT_BINARY_DIR}/src")
    set(rtt_roscomm_GENERATED_HEADERS_INSTALL_DESTINATION)
  elseif(ORO_USE_CATKIN)
    message(STATUS "[ros_generate_rtt_typekit] Generating ROS typekit for ${PROJECT_NAME} with Catkin destinations.")
    catkin_destinations()
    set(rtt_roscomm_GENERATED_HEADERS_OUTPUT_DIRECTORY    "${CATKIN_DEVEL_PREFIX}/include")
    set(rtt_roscomm_GENERATED_SOURCES_OUTPUT_DIRECTORY    "${CMAKE_CURRENT_BINARY_DIR}/src")
    set(rtt_roscomm_GENERATED_HEADERS_INSTALL_DESTINATION "${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  else()
    message(STATUS "[ros_generate_rtt_typekit] Generating ROS typekit for ${PROJECT_NAME} with normal CMake destinations.")
    set(rtt_roscomm_GENERATED_HEADERS_OUTPUT_DIRECTORY    "${PROJECT_BINARY_DIR}/include")
    set(rtt_roscomm_GENERATED_SOURCES_OUTPUT_DIRECTORY    "${PROJECT_BINARY_DIR}/src")
    set(rtt_roscomm_GENERATED_HEADERS_INSTALL_DESTINATION "${CMAKE_INSTALL_PREFIX}/include")
  endif()

  if(DEFINED ENV{VERBOSE_CONFIG})
    message(STATUS "[ros_generate_rtt_typekit]   Generating headers in: ${rtt_roscomm_GENERATED_HEADERS_OUTPUT_DIRECTORY}")
    message(STATUS "[ros_generate_rtt_typekit]   Generating sources in: ${rtt_roscomm_GENERATED_SOURCES_OUTPUT_DIRECTORY}")
    message(STATUS "[ros_generate_rtt_typekit]   Installing headers to: ${rtt_roscomm_GENERATED_HEADERS_INSTALL_DESTINATION}")
  endif()
endmacro()

macro(rtt_roscomm_debug)
  if(DEFINED ENV{VERBOSE_CONFIG})
    message(STATUS "[ros_generate_rtt_typekit]     catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")
  endif()
endmacro()

macro(ros_generate_rtt_typekit package)
  find_package(OROCOS-RTT 2.0.0 COMPONENTS rtt-scripting rtt-marshalling)
  if (NOT OROCOS-RTT_FOUND)
    message(FATAL_ERROR "\n   RTT not found. Is the version correct? Use the CMAKE_PREFIX_PATH cmake or environment variable to point to the installation directory of RTT.")
  else()
    include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
    add_definitions( -DRTT_COMPONENT )
  endif()

  # Configure source and destination paths of generated files
  rtt_roscomm_destinations()
  set(_template_types_src_dir "${rtt_roscomm_DIR}/../rtt_roscomm_pkg_template/src/orocos/types")
  set(_template_types_dst_dir "${rtt_roscomm_GENERATED_SOURCES_OUTPUT_DIRECTORY}/orocos/types")

  set(_template_typekit_src_dir "${rtt_roscomm_DIR}/../rtt_roscomm_pkg_template/include/PKG_NAME/typekit")
  set(_template_typekit_dst_dir "${rtt_roscomm_GENERATED_HEADERS_OUTPUT_DIRECTORY}/orocos/${package}/typekit")

  # Check if we're generating code for messages in this package
  if("${package}" STREQUAL "${PROJECT_NAME}")
    set(${package}_FOUND True)
  else()
    find_package(${package} QUIET)
  endif()

  find_package(genmsg REQUIRED)

  # Get all .msg files
  if(${package}_FOUND)
    # Use catkin-based genmsg to find msg files
    if(genmsg_VERSION VERSION_GREATER 0.4.19)
      set(MSG_FILES)
      # TODO: genmsg API is unstable at this level
      foreach(FILE ${${package}_MESSAGE_FILES})
        if(IS_ABSOLUTE "${FILE}")
          list(APPEND MSG_FILES ${FILE})
        else()
          list(APPEND MSG_FILES ${${package}_DIR}/../${FILE})
        endif()
      endforeach()
    else()
      message(SEND_ERROR "genmsg version must be 0.4.19 or greater to generate RTT typekits for ROS messages")
    endif()
  elseif(ROSBUILD_init_called)
    # try to find rosbuild-style message package
    rosbuild_find_ros_package(${package})
    if(DEFINED ${package}_PACKAGE_PATH)
      set(${package}_FOUND TRUE)
      set(${package}_INCLUDE_DIRS "${${package}_PACKAGE_PATH}/include")
      file(GLOB MSG_FILES "${${package}_PACKAGE_PATH}/msg/*.msg")
      set(${package}_EXPORTED_TARGETS)
    endif()
  endif()

  # message package not found
  if(NOT ${package}_FOUND)
    message(SEND_ERROR "Package ${package} not found. Will not generate RTT typekit.")
    set(MSG_FILES)
  endif()

  if( NOT "${MSG_FILES}" STREQUAL "" )

    # Set the boost header generation script path
    @[if DEVELSPACE]@
      set(CREATE_BOOST_HEADER_EXE_PATH @(CMAKE_CURRENT_SOURCE_DIR)/cmake/create_boost_header.py)
    @[else]@
      set(CREATE_BOOST_HEADER_EXE_PATH @(CMAKE_INSTALL_PREFIX)/@(CATKIN_PACKAGE_SHARE_DESTINATION)/cmake/create_boost_header.py)
    @[end if]@

    # Store the ros package name
    set(ROSPACKAGE ${package})

    # Generate code for each message type
    foreach( FILE ${MSG_FILES} )

      # Get just the message name
      string(REGEX REPLACE ".+/\(.+\).msg" "\\1" ROSMSGNAME ${FILE})
      
      # Define the typenames for this message
      set(ROSMSGTYPE         "${package}::${ROSMSGNAME}")
      set(ROSMSGTYPENAME     "/${package}/${ROSMSGNAME}")
      set(ROSMSGCTYPENAME    "/${package}/c${ROSMSGNAME}")

      # msg_Types.hpp.in, ros_msg_typekit_plugin.cpp.in, ros_msg_typekit_package.cpp.in
      set(ROSMSGBOOSTHEADER  "${package}/boost/${ROSMSGNAME}.h")
      # ros_msg_typekit_plugin.cpp.in, ros_msg_typekit_package.cpp.in
      set(ROSMSGBOOSTHEADERS "${ROSMSGBOOSTHEADERS}#include <orocos/${ROSMSGBOOSTHEADER}>\n")
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
      set(_ROSMSG_GENERATED_BOOST_HEADER  "${rtt_roscomm_GENERATED_HEADERS_OUTPUT_DIRECTORY}/orocos/${ROSMSGBOOSTHEADER}")
      list(APPEND ROSMSGS_GENERATED_BOOST_HEADERS ${_ROSMSG_GENERATED_BOOST_HEADER})

      add_custom_command(
        OUTPUT ${_ROSMSG_GENERATED_BOOST_HEADER} 
        COMMAND ${CREATE_BOOST_HEADER_EXE_PATH} ${package} "${package}/${ROSMSGNAME}" ${FILE} ${_ROSMSG_GENERATED_BOOST_HEADER} 
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} 
        DEPENDS ${FILE} ${${package}_EXPORTED_TARGETS} ${CREATE_BOOST_HEADER_EXE_PATH}
        VERBATIM)

      #set_source_files_properties(${ROSMSGS_GENERATED_BOOST_HEADERS} PROPERTIES GENERATED TRUE)
      
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
    endforeach( FILE ${MSG_FILES} )
    
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
    
    include_directories(
      ${rtt_roscomm_GENERATED_HEADERS_OUTPUT_DIRECTORY} 
      ${rtt_roscomm_GENERATED_HEADERS_OUTPUT_DIRECTORY}/orocos
      ${rtt_roscomm_GENERATED_HEADERS_INSTALL_DESTINATION}/orocos
      ${USE_OROCOS_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS})

    # Targets
    orocos_typekit(         rtt-${package}-typekit ${_template_types_dst_dir}/ros_${package}_typekit.cpp ${ROSMSG_TYPEKIT_PLUGINS})
    orocos_typekit(         rtt-${package}-ros-transport ${_template_types_dst_dir}/ros_${package}_transport.cpp )
    target_link_libraries(  rtt-${package}-typekit ${catkin_LIBRARIES})
    target_link_libraries(  rtt-${package}-ros-transport ${catkin_LIBRARIES})

    # Add an explicit dependency between the typekits and message files
    # TODO: Add deps for all msg dependencies
    if(DEFINED ${package}_EXPORTED_TARGETS)
      if(NOT ${package} STREQUAL ${PROJECT_NAME})
        add_dependencies(       rtt-${package}-typekit ${${package}_EXPORTED_TARGETS})
        add_dependencies(       rtt-${package}-ros-transport ${${package}_EXPORTED_TARGETS})
      endif()
    endif()

    # Add the typekit libraries to the dependecies exported by this project
#    LIST(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS "rtt-${package}-typekit")        # <-- This is already done in orocos_typekit().
#    LIST(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS "rtt-${package}-ros-transport")  # <-- This is already done in orocos_typekit().
    LIST(APPEND ${PROJECT_NAME}_EXPORTED_INCLUDE_DIRS "${rtt_roscomm_GENERATED_HEADERS_OUTPUT_DIRECTORY}/orocos" ${${package}_INCLUDE_DIRS})

    add_file_dependencies(  ${_template_types_dst_dir}/ros_${package}_typekit.cpp "${CMAKE_CURRENT_LIST_FILE}" ${ROSMSGS_GENERATED_BOOST_HEADERS} )
    add_file_dependencies(  ${_template_types_dst_dir}/ros_${package}_transport.cpp "${CMAKE_CURRENT_LIST_FILE}" ${ROSMSGS_GENERATED_BOOST_HEADERS} )

    get_directory_property(_additional_make_clean_files ADDITIONAL_MAKE_CLEAN_FILES)
    list(APPEND _additional_make_clean_files "${ROSMSG_TYPEKIT_PLUGINS};${ROSMSG_TRANSPORT_PLUGIN};${_template_types_dst_dir}/ros_${package}_typekit.cpp;${_template_types_dst_dir}/ros_${package}_transport.cpp;${rtt_roscomm_GENERATED_HEADERS_OUTPUT_DIRECTORY}/orocos/${package}")
    set_directory_properties(PROPERTIES 
      ADDITIONAL_MAKE_CLEAN_FILES "${_additional_make_clean_files}")

    # Install generated header files (dependent packages might need them)
    if(DEFINED rtt_roscomm_GENERATED_HEADERS_INSTALL_DESTINATION)
      # install(FILES ${ROSMSGS_GENERATED_BOOST_HEADERS} DESTINATION ${rtt_roscomm_GENERATED_HEADERS_INSTALL_DESTINATION}/${package}/boost/)
      # install(DIRECTORY "${rtt_roscomm_GENERATED_HEADERS_OUTPUT_DIRECTORY}/orocos/${package}/typekit" DESTINATION ${rtt_roscomm_GENERATED_HEADERS_INSTALL_DESTINATION}/orocos/${package})
      install(
        DIRECTORY "${rtt_roscomm_GENERATED_HEADERS_OUTPUT_DIRECTORY}/orocos/${package}" 
        DESTINATION "${rtt_roscomm_GENERATED_HEADERS_INSTALL_DESTINATION}/orocos")
    endif()

    list(APPEND RTT_ROSCOMM_GENERATED_TARGETS 
      rtt-${package}-typekit
      rtt-${package}-ros-transport
      )

  else()
    # Return if nothing to do
    message(STATUS "ros_generate_rtt_typekit: Could not find any .msg files in the ${package} package.")
  endif()

endmacro(ros_generate_rtt_typekit)


macro(ros_generate_rtt_service_proxies package)
  find_package(OROCOS-RTT 2.0.0 COMPONENTS rtt-scripting rtt-marshalling)
  if (NOT OROCOS-RTT_FOUND)
    message(FATAL_ERROR "\n   RTT not found. Is the version correct? Use the CMAKE_PREFIX_PATH cmake or environment variable to point to the installation directory of RTT.")
  else()
    include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
    add_definitions( -DRTT_COMPONENT )
  endif()

  # Configure source and destination paths of generated files
  rtt_roscomm_destinations()
  set(_template_proxies_src_dir "${rtt_roscomm_DIR}/../rtt_roscomm_pkg_template/src")
  set(_template_proxies_dst_dir "${rtt_roscomm_GENERATED_SOURCES_OUTPUT_DIRECTORY}/orocos")

  # Check if we're generating code for services in this package
  if(NOT package STREQUAL PROJECT_NAME)
    find_package(${package} QUIET)
  endif()

  find_package(genmsg REQUIRED)

  # Get all .srv files
  if(${package}_FOUND)
    # Use catkin-based genmsg to find srv files
    if(genmsg_VERSION VERSION_GREATER 0.4.19)
      set(SRV_FILES)
      # TODO: genmsg API is unstable at this level
      foreach(FILE ${${package}_SERVICE_FILES})
        if(IS_ABSOLUTE "${FILE}")
          list(APPEND SRV_FILES ${FILE})
        else()
          list(APPEND SRV_FILES ${${package}_DIR}/../${FILE})
        endif()
      endforeach()
    else()
      message(SEND_ERROR "genmsg version must be 0.4.19 or greater")
    endif()
  elseif(ROSBUILD_init_called)
    # try to find rosbuild-style message package
    rosbuild_find_ros_package(${package})
    if(DEFINED ${package}_PACKAGE_PATH)
      set(${package}_FOUND TRUE)
      set(${package}_INCLUDE_DIRS "${${package}_PACKAGE_PATH}/include")
      file(GLOB SRV_FILES "${${package}_PACKAGE_PATH}/srv/*.srv")
      set(${package}_EXPORTED_TARGETS)
    endif()
  else()
    message(SEND_ERROR "Package ${package} not found. Will not generate RTT service proxy.")
    set(SRV_FILES)
  endif()

  
  if ( NOT "${SRV_FILES}" STREQUAL "" )

    # Get the ros package name
    set(ROSPACKAGE ${package})

    foreach( FILE ${SRV_FILES} )
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
    configure_file( 
      ${_template_proxies_src_dir}/rtt_ros_service_proxies.cpp.in 
      ${_template_proxies_dst_dir}/rtt_ros_service_proxies.cpp @@ONLY )

    add_file_dependencies( ${_template_proxies_dst_dir}/rtt_ros_service_proxies.cpp ${SRV_FILES})
    
    include_directories(
      ${CATKIN_DEVEL_PREFIX}/include 
      ${USE_OROCOS_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS})

    # Targets
    orocos_service(         rtt_${ROSPACKAGE}_ros_service_proxies ${_template_proxies_dst_dir}/rtt_ros_service_proxies.cpp)
    target_link_libraries(  rtt_${ROSPACKAGE}_ros_service_proxies ${catkin_LIBRARIES})
    if(DEFINED ${package}_EXPORTED_TARGETS)
      add_dependencies(       rtt_${ROSPACKAGE}_ros_service_proxies ${${package}_EXPORTED_TARGETS})
    endif()
    add_file_dependencies(  ${_template_proxies_dst_dir}/rtt_ros_service_proxies.cpp "${CMAKE_CURRENT_LIST_FILE}")

    get_directory_property(_additional_make_clean_files ADDITIONAL_MAKE_CLEAN_FILES)
    list(APPEND _additional_make_clean_files "${_template_proxies_dst_dir}/rtt_ros_service_proxies.cpp")
    set_directory_properties(PROPERTIES 
      ADDITIONAL_MAKE_CLEAN_FILES "${_additional_make_clean_files}")

  else()
    #Return if nothing to do:
    message(STATUS "ros_generate_rtt_service_proxies: Could not find any .srv files in the ${package} package.")
  endif()
  
endmacro(ros_generate_rtt_service_proxies)
