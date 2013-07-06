
# Macro for setting the proper output directory for a given type of OROCOS target
macro( catkin_orocos_devel_output LIB_TARGET_NAME TYPE)
  set_target_properties(${LIB_TARGET_NAME}
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib${OROCOS_SUFFIX}/${PROJECT_NAME}/${TYPE})
endmacro( catkin_orocos_devel_output )

macro( catkin_orocos_plugin LIB_TARGET_NAME)
  orocos_plugin(${LIB_TARGET_NAME} ${ARGN})
  catkin_orocos_devel_output(${LIB_TARGET_NAME} plugins)
endmacro( catkin_orocos_plugin )

macro( catkin_orocos_typekit LIB_TARGET_NAME)
  orocos_typekit(${LIB_TARGET_NAME} ${ARGN})
  catkin_orocos_devel_output(${LIB_TARGET_NAME} types)
endmacro( catkin_orocos_typekit )

macro( catkin_orocos_service LIB_TARGET_NAME)
  orocos_service(${LIB_TARGET_NAME} ${ARGN})
  catkin_orocos_devel_output(${LIB_TARGET_NAME} plugins)
endmacro( catkin_orocos_service )
