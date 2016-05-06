find_package(OROCOS-RTT 2.0.0  COMPONENTS rtt-scripting rtt-marshalling rtt-transport-mqueue rtt-transport-corba)
if (NOT OROCOS-RTT_FOUND)
  message(FATAL_ERROR "\n   RTT not found. Is the version correct? Use the CMAKE_PREFIX_PATH cmake or environment variable to point to the installation directory of RTT.")
else()
  include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
  #add_definitions( -DRTT_COMPONENT )
endif()

include(AddFileDependencies)

if (OROCOS-RTT_MQUEUE_FOUND)
  message(STATUS "Building MQueue transport for ROS messages with ${OROCOS-RTT_MQUEUE_LIBRARIES}")
else()
  message(WARNING "Not building MQueue transport for ROS messages")
endif()

if ( OROCOS-RTT_CORBA_FOUND )
  message(STATUS "Building CORBA transport for ROS messages with ${OROCOS-RTT_CORBA_LIBRARIES}")
else( OROCOS-RTT_CORBA_FOUND )
  message(WARNING "Not building CORBA transport for ROS messages")
endif( OROCOS-RTT_CORBA_FOUND )

# Configure source and destination paths of generated files
rtt_roscomm_destinations()
set(_template_typekit_dst_dir "${rtt_roscomm_GENERATED_HEADERS_OUTPUT_DIRECTORY}/orocos/${_package}/typekit")

# Check if we're generating code for messages in this package
if("${_package}" STREQUAL "${PROJECT_NAME}")
  set(${_package}_FOUND True)
else()
  find_package(${_package} QUIET)
endif()

find_package(genmsg REQUIRED)

# Get all .msg files
if(${_package}_FOUND AND NOT ROSBUILD_init_called)
  # Use catkin-based genmsg to find msg files
  if(genmsg_VERSION VERSION_GREATER 0.4.19)
    set(MSG_FILES)
    # TODO: genmsg API is unstable at this level
    foreach(FILE ${${_package}_MESSAGE_FILES})
      if(IS_ABSOLUTE "${FILE}")
        list(APPEND MSG_FILES ${FILE})
      else()
        list(APPEND MSG_FILES ${${_package}_DIR}/../${FILE})
      endif()
    endforeach()
  else()
    message(SEND_ERROR "genmsg version must be 0.4.19 or greater to generate RTT typekits for ROS messages")
  endif()
elseif(ROSBUILD_init_called)
  # try to find rosbuild-style message package
  rosbuild_find_ros_package(${_package})
  if(DEFINED ${_package}_PACKAGE_PATH)
    set(${_package}_FOUND TRUE)
    set(${_package}_INCLUDE_DIRS "${${_package}_PACKAGE_PATH}/include")
    file(GLOB MSG_FILES "${${_package}_PACKAGE_PATH}/msg/*.msg")
    set(${_package}_EXPORTED_TARGETS)
  endif()
endif()

# message package not found
if(NOT ${_package}_FOUND)
  message(SEND_ERROR "Package ${_package} not found. Will not generate RTT typekit.")
  return()
endif()

# Return if nothing to do
if( "${MSG_FILES}" STREQUAL "" )
  message(STATUS "ros_generate_rtt_typekit: Could not find any .msg files in the ${_package} package.")
  return()
endif()

# Set the boost header generation script path
set(CREATE_BOOST_HEADER_EXE_PATH ${rtt_roscomm_DIR}/create_boost_header.py)

# Store the ros package name
set(ROSPACKAGE ${_package})

# Generate code for each message type
foreach( FILE ${MSG_FILES} )
  # Get just the message name
  string(REGEX REPLACE ".+/\(.+\).msg" "\\1" ROSMSGNAME ${FILE})

  # Define the typenames for this message
  set(ROSMSGTYPE         "${_package}::${ROSMSGNAME}")
  set(ROSMSGTYPENAME     "/${_package}/${ROSMSGNAME}")
  set(ROSMSGCTYPENAME    "/${_package}/c${ROSMSGNAME}")

  # msg_Types.hpp.in, ros_msg_typekit_plugin.cpp.in, ros_msg_typekit_package.cpp.in
  set(ROSMSGBOOSTHEADER  "${_package}/boost/${ROSMSGNAME}.h")
  # ros_msg_typekit_plugin.cpp.in, ros_msg_typekit_package.cpp.in
  set(ROSMSGBOOSTHEADERS "${ROSMSGBOOSTHEADERS}#include <orocos/${ROSMSGBOOSTHEADER}>\n")
  # ros_corba_conversion.hpp.in
  set(ROSMSGCORBABOOSTHEADER "#include <orocos/${ROSMSGBOOSTHEADER}>\n")
  # ros_msg_transport_corba_package.cpp.in
  set(ROSMSGCORBAHEADERS "${ROSMSGCORBAHEADERS}#include \"ros_${ROSMSGNAME}_corba_conversion.hpp\"\n")
  # Types.hpp.in, ros_msg_typekit_package.cpp.in
  set(ROSMSGTYPES        "${ROSMSGTYPES}        rtt_ros_addType_${_package}_${ROSMSGNAME}(); // factory function for adding TypeInfo.\n")
  # ros_msg_typekit_package.cpp.in
  set(ROSMSGTYPEDECL     "${ROSMSGTYPEDECL}        void rtt_ros_addType_${_package}_${ROSMSGNAME}();\n")
  # ros_msg_typekit_plugin.cpp.in
  set(ROSMSGTYPELINE "
      void rtt_ros_addType_${_package}_${ROSMSGNAME}() {
           // Only the .msg type is sent over ports. The msg[] (variable size) and  cmsg[] (fixed size) exist only as members of larger messages
           RTT::types::Types()->addType( new types::StructTypeInfo<${ROSMSGTYPE}>(\"${ROSMSGTYPENAME}\") );
           RTT::types::Types()->addType( new types::PrimitiveSequenceTypeInfo<std::vector<${ROSMSGTYPE}> >(\"${ROSMSGTYPENAME}[]\") );
           RTT::types::Types()->addType( new types::CArrayTypeInfo<RTT::types::carray<${ROSMSGTYPE}> >(\"${ROSMSGCTYPENAME}[]\") );
      }\n")
  # ros_msg_transport_package.cpp.in
  set(ROSMSGTRANSPORTS   "${ROSMSGTRANSPORTS}       if(name == \"${ROSMSGTYPENAME}\") { return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<${ROSMSGTYPE}>());} else\n")
  # ros_msg_mqueue_transport_package.cpp.in
  set(MQMSGTRANSPORTS   "${MQMSGTRANSPORTS}         if(name == \"${ROSMSGTYPENAME}\") { return ti->addProtocol(ORO_MQUEUE_PROTOCOL_ID, new RTT::mqueue::MQTemplateProtocol<${ROSMSGTYPE}>());} else\n")
  # ros_corba_transport_package.cpp.in
  set(CORBAMSGTRANSPORTS   "${CORBAMSGTRANSPORTS}         if(name == \"${ROSMSGTYPENAME}\") { return ti->addProtocol(ORO_CORBA_PROTOCOL_ID, new RTT::corba::CorbaTemplateProtocol<${ROSMSGTYPE}>());} else\n")
  # Types.hpp.in
  set(ROSMSGTYPESHEADERS "${ROSMSGTYPESHEADERS}#include \"${ROSMSGNAME}.h\"\n")

  # Necessary for create_boost_header.py command below
  set(_ROSMSG_GENERATED_BOOST_HEADER  "${rtt_roscomm_GENERATED_HEADERS_OUTPUT_DIRECTORY}/orocos/${ROSMSGBOOSTHEADER}")
  list(APPEND ROSMSGS_GENERATED_BOOST_HEADERS ${_ROSMSG_GENERATED_BOOST_HEADER})

  add_custom_command(
    OUTPUT ${_ROSMSG_GENERATED_BOOST_HEADER}
    COMMAND ${CREATE_BOOST_HEADER_EXE_PATH} ${_package} "${_package}/${ROSMSGNAME}" ${FILE} ${_ROSMSG_GENERATED_BOOST_HEADER}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    DEPENDS ${FILE} ${${_package}_EXPORTED_TARGETS} ${CREATE_BOOST_HEADER_EXE_PATH}
    VERBATIM)

  #set_source_files_properties(${ROSMSGS_GENERATED_BOOST_HEADERS} PROPERTIES GENERATED TRUE)

  configure_file(
    ros_msg_typekit_plugin.cpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/ros_${ROSMSGNAME}_typekit_plugin.cpp @ONLY )

  # Transport for ROS
  configure_file(
    ros_msg_transport_plugin.cpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/ros_${ROSMSGNAME}_transport_plugin.cpp @ONLY )

  # Transport for MQ-ROS
  configure_file(
    ros_msg_transport_mqueue_plugin.cpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/ros_${ROSMSGNAME}_transport_mqueue_plugin.cpp @ONLY )

  # Transport for CORBA
  configure_file(
    ros_corba_conversion.hpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/ros_${ROSMSGNAME}_corba_conversion.hpp @ONLY )

  # Types.hpp helper for extern templates
  configure_file(
    msg_Types.hpp.in
    ${_template_typekit_dst_dir}/${ROSMSGNAME}.h @ONLY )

  list(APPEND ROSMSG_TYPEKIT_PLUGINS ${CMAKE_CURRENT_BINARY_DIR}/ros_${ROSMSGNAME}_typekit_plugin.cpp )
  list(APPEND ROSMSG_TRANSPORT_PLUGIN ${CMAKE_CURRENT_BINARY_DIR}/ros_${ROSMSGNAME}_transport_plugin.cpp )
  list(APPEND ROSMSG_TRANSPORT_MQUEUE_PLUGIN ${CMAKE_CURRENT_BINARY_DIR}/ros_${ROSMSGNAME}_transport_mqueue_plugin.cpp )
  list(APPEND ROSMSG_TRANSPORT_CORBA_PLUGIN ${CMAKE_CURRENT_BINARY_DIR}/ros_${ROSMSGNAME}_corba_conversion.hpp )

  add_file_dependencies( ${CMAKE_CURRENT_BINARY_DIR}/ros_${_package}_typekit.cpp ${FILE})
endforeach( FILE ${MSG_FILES} )

configure_file(
  ros_msg_typekit_package.cpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/ros_${_package}_typekit.cpp @ONLY )

configure_file(
  ros_msg_transport_package.cpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/ros_${_package}_transport.cpp @ONLY )

configure_file(
  ros_msg_transport_mqueue_package.cpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/ros_${_package}_transport_mqueue.cpp @ONLY )

configure_file(
  ros_msg_transport_corba_package.cpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/ros_${_package}_transport_corba.cpp @ONLY )

# Both are equivalent
configure_file(
  Types.hpp.in
  ${_template_typekit_dst_dir}/Types.hpp @ONLY )
configure_file(
  Types.h.in
  ${_template_typekit_dst_dir}/Types.h @ONLY )

include_directories(
  ${rtt_roscomm_GENERATED_HEADERS_OUTPUT_DIRECTORY}
  ${rtt_roscomm_GENERATED_HEADERS_OUTPUT_DIRECTORY}/orocos
  ${rtt_roscomm_GENERATED_HEADERS_INSTALL_DESTINATION}/orocos
  ${USE_OROCOS_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

if ( OROCOS-RTT_CORBA_FOUND )
  list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/config)
  set(ENABLE_CORBA TRUE)
  find_package(Corba REQUIRED)
  FILE( GLOB IDLS [^.]*.idl )
  set(CORBA_GEN_CPP "") 
  set(CORBA_GEN_HPP "")
  ORO_ADD_CORBA_SERVERS(CORBA_GEN_CPP HPP ${IDLS} )
  ADD_CUSTOM_TARGET( idl-ros-${_package} DEPENDS ${CORBA_GEN_CPP} ${CORBA_GEN_HPP} COMMENT "Generating sources from .idl files")
  MESSAGE(STATUS "ROS ${_package} Corba HPPS: ${CORBA_GEN_CPP} ${CORBA_GEN_HPP}")
  INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_BINARY_DIR} )
endif()


# Targets
if(NOT DEFINED CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_BUILD_TYPE MinSizeRel)
endif()
orocos_typekit(         rtt-${_package}-typekit ${CMAKE_CURRENT_BINARY_DIR}/ros_${_package}_typekit.cpp ${ROSMSG_TYPEKIT_PLUGINS})
orocos_typekit(         rtt-${_package}-ros-transport ${CMAKE_CURRENT_BINARY_DIR}/ros_${_package}_transport.cpp )

# Build mqueue typekit
if (OROCOS-RTT_MQUEUE_FOUND)
  orocos_typekit(         rtt-${_package}-ros-transport-mqueue ${CMAKE_CURRENT_BINARY_DIR}/ros_${_package}_transport_mqueue.cpp )
  target_link_libraries(  rtt-${_package}-ros-transport-mqueue ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${OROCOS-RTT_MQUEUE_LIBRARIES})
endif()

# Build corba typekit
if (OROCOS-RTT_CORBA_FOUND)
  orocos_typekit(         rtt-${_package}-ros-transport-corba ${CMAKE_CURRENT_BINARY_DIR}/ros_${_package}_transport_corba.cpp ${CORBA_GEN_CPP})
  target_link_libraries(  rtt-${_package}-ros-transport-corba ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${OROCOS-RTT_CORBA_LIBRARIES})
endif()

target_link_libraries(  rtt-${_package}-typekit ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(  rtt-${_package}-ros-transport ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

# Add an explicit dependency between the typekits and message files
# TODO: Add deps for all msg dependencies
if(${_package}_EXPORTED_TARGETS)
  if(NOT ${_package} STREQUAL ${PROJECT_NAME})
    add_dependencies(rtt-${_package}-typekit ${${_package}_EXPORTED_TARGETS})
    add_dependencies(rtt-${_package}-ros-transport ${${_package}_EXPORTED_TARGETS})
    if (OROCOS-RTT_MQUEUE_FOUND)
      add_dependencies(rtt-${_package}-ros-transport-mqueue ${${_package}_EXPORTED_TARGETS})
    endif()
    if (OROCOS-RTT_CORBA_FOUND)
      add_dependencies(rtt-${_package}-ros-transport-corba ${${_package}_EXPORTED_TARGETS})
    endif()
  endif()
endif()

# Add the typekit libraries to the dependecies exported by this project
#    LIST(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS "rtt-${_package}-typekit")        # <-- This is already done in orocos_typekit().
#    LIST(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS "rtt-${_package}-ros-transport")  # <-- This is already done in orocos_typekit().
LIST(APPEND ${PROJECT_NAME}_EXPORTED_INCLUDE_DIRS "${rtt_roscomm_GENERATED_HEADERS_OUTPUT_DIRECTORY}/orocos" ${${_package}_INCLUDE_DIRS})

add_file_dependencies(  ${CMAKE_CURRENT_BINARY_DIR}/ros_${_package}_typekit.cpp "${CMAKE_CURRENT_LIST_FILE}" ${ROSMSGS_GENERATED_BOOST_HEADERS} )
add_file_dependencies(  ${CMAKE_CURRENT_BINARY_DIR}/ros_${_package}_transport.cpp "${CMAKE_CURRENT_LIST_FILE}" ${ROSMSGS_GENERATED_BOOST_HEADERS} )

if (OROCOS-RTT_MQUEUE_FOUND)
  add_file_dependencies(  ${CMAKE_CURRENT_BINARY_DIR}/ros_${_package}_transport_mqueue.cpp "${CMAKE_CURRENT_LIST_FILE}" ${ROSMSGS_GENERATED_BOOST_HEADERS} )
endif()

if (OROCOS-RTT_CORBA_FOUND)
  add_file_dependencies(  ${CMAKE_CURRENT_BINARY_DIR}/ros_${_package}_transport_corba.cpp "${CMAKE_CURRENT_LIST_FILE}" ${ROSMSGS_GENERATED_BOOST_HEADERS} )
endif()

get_directory_property(_additional_make_clean_files ADDITIONAL_MAKE_CLEAN_FILES)
list(APPEND _additional_make_clean_files "${ROSMSG_TYPEKIT_PLUGINS};${ROSMSG_TRANSPORT_PLUGIN};${ROSMSG_TRANSPORT_CORBA_PLUGIN};${ROSMSG_TRANSPORT_MQUEUE_PLUGIN};${CMAKE_CURRENT_BINARY_DIR}/ros_${_package}_typekit.cpp;${CMAKE_CURRENT_BINARY_DIR}/ros_${_package}_transport.cpp;${rtt_roscomm_GENERATED_HEADERS_OUTPUT_DIRECTORY}/orocos/${_package}")
set_directory_properties(PROPERTIES
  ADDITIONAL_MAKE_CLEAN_FILES "${_additional_make_clean_files}")

# Install generated header files (dependent packages might need them)
if(DEFINED rtt_roscomm_GENERATED_HEADERS_INSTALL_DESTINATION)
  # install(FILES ${ROSMSGS_GENERATED_BOOST_HEADERS} DESTINATION ${rtt_roscomm_GENERATED_HEADERS_INSTALL_DESTINATION}/${_package}/boost/)
  # install(DIRECTORY "${rtt_roscomm_GENERATED_HEADERS_OUTPUT_DIRECTORY}/orocos/${_package}/typekit" DESTINATION ${rtt_roscomm_GENERATED_HEADERS_INSTALL_DESTINATION}/orocos/${_package})
  install(
    DIRECTORY "${rtt_roscomm_GENERATED_HEADERS_OUTPUT_DIRECTORY}/orocos/${_package}"
    DESTINATION "${rtt_roscomm_GENERATED_HEADERS_INSTALL_DESTINATION}/orocos")
endif()

#list(APPEND RTT_ROSCOMM_GENERATED_TARGETS
#  rtt-${_package}-typekit
#  rtt-${_package}-ros-transport
#  )

# Export variables to the PARENT_SCOPE
set(OROCOS_DEFINED_TYPES ${OROCOS_DEFINED_TYPES} PARENT_SCOPE)
set(${PROJECT_NAME}_EXPORTED_TARGETS ${${PROJECT_NAME}_EXPORTED_TARGETS} PARENT_SCOPE)
set(${PROJECT_NAME}_EXPORTED_INCLUDE_DIRS ${${PROJECT_NAME}_EXPORTED_INCLUDE_DIRS} PARENT_SCOPE)
