################################################################################
#
# CMake script for finding An Orocos RTT Plugin in case you require it during build time.
# If the optional RTT_COMPONENT_PATH environment variable exists, header files and
# libraries will be searched in the RTT_COMPONENT_PATH/include and RTT_COMPONENT_PATH/lib/orocos/plugins
# directories, respectively. Otherwise the default CMake search process will be
# used. Use the RTT_HINTS variable to hint the location of the orocos-rtt installation directory.
#
# Usage: find_package( RTTPlugin COMPONENTS rtt-scripting )
#
# This script creates the following variables:
#  RTT_PLUGIN_<COMPONENT>_FOUND: Boolean that indicates if the plugin was found
#  RTT_PLUGIN_<COMPONENT>_INCLUDE_DIRS: Path to the plugin header files (equal to OROCOS_RTT_INCLUDE_DIRS)
#  RTT_PLUGIN_<COMPONENT>_LIBRARIES: Plugin library for the current OROCOS_TARGET cmake or environment variable.
#  RTT_PLUGIN_<COMPONENT>_<OROCOS_TARGET>_LIBRARIES: Plugin library for the current OROCOS_TARGET cmake or environment variable (same as above).
#
################################################################################

include(LibFindMacros)

find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})

FOREACH(COMPONENT ${RTTPlugin_FIND_COMPONENTS})
    # We search for both 'given name' and 'given name + -target'
    set(PLUGIN_NAME   ${COMPONENT} ${COMPONENT}-${OROCOS_TARGET} )
    #STRING(TOUPPER ${COMPONENT} COMPONENT)

    set(RTT_PLUGIN_${COMPONENT}_INCLUDE_DIR ${OROCOS-RTT_INCLUDE_DIRS} )
    set(RTT_PLUGIN_${COMPONENT}_${OROCOS_TARGET}_INCLUDE_DIR ${OROCOS-RTT_INCLUDE_DIRS} )
    # Find plugins
    if(OROCOS-RTT_PLUGIN_PATH)
        # Use location specified by environment variable
        find_library(RTT_PLUGIN_${COMPONENT}_LIBRARY        NAMES ${PLUGIN_NAME} PATHS ${OROCOS-RTT_PLUGIN_PATH} PATH_SUFFIXES ${OROCOS_TARGET}  NO_DEFAULT_PATH)
        find_library(RTT_PLUGIN_${COMPONENT}D_LIBRARY       NAMES ${PLUGIN_NAME}${CMAKE_DEBUG_POSTFIX} PATHS ${OROCOS-RTT_PLUGIN_PATH} PATH_SUFFIXES ${OROCOS_TARGET}  NO_DEFAULT_PATH)
        find_library(RTT_PLUGIN_${COMPONENT}_${OROCOS_TARGET}_LIBRARY        NAMES ${PLUGIN_NAME} PATHS ${OROCOS-RTT_PLUGIN_PATH} PATH_SUFFIXES ${OROCOS_TARGET}  NO_DEFAULT_PATH)
        find_library(RTT_PLUGIN_${COMPONENT}_${OROCOS_TARGET}D_LIBRARY       NAMES ${PLUGIN_NAME}${CMAKE_DEBUG_POSTFIX} PATHS ${OROCOS-RTT_PLUGIN_PATH} PATH_SUFFIXES ${OROCOS_TARGET}  NO_DEFAULT_PATH)
    else()
        # Use default CMake search process
        find_library(RTT_PLUGIN_${COMPONENT}_LIBRARY        NAMES ${PLUGIN_NAME} PATHS ${OROCOS-RTT_PLUGIN_PATH} PATH_SUFFIXES ${OROCOS_TARGET})
        find_library(RTT_PLUGIN_${COMPONENT}D_LIBRARY       NAMES ${PLUGIN_NAME}${CMAKE_DEBUG_POSTFIX} PATHS ${OROCOS-RTT_PLUGIN_PATH} PATH_SUFFIXES ${OROCOS_TARGET})
        find_library(RTT_PLUGIN_${COMPONENT}_${OROCOS_TARGET}_LIBRARY        NAMES ${PLUGIN_NAME} PATHS ${OROCOS-RTT_PLUGIN_PATH} PATH_SUFFIXES ${OROCOS_TARGET})
        find_library(RTT_PLUGIN_${COMPONENT}_${OROCOS_TARGET}D_LIBRARY       NAMES ${PLUGIN_NAME}${CMAKE_DEBUG_POSTFIX} PATHS ${OROCOS-RTT_PLUGIN_PATH} PATH_SUFFIXES ${OROCOS_TARGET})
    endif()

    # Set the include dir variables and the libraries and let libfind_process do the rest.
    # NOTE: Singular variables for this library, plural for libraries this this lib depends on.
    set(RTT_PLUGIN_${COMPONENT}_PROCESS_INCLUDES RTT_PLUGIN_${COMPONENT}_INCLUDE_DIR )
    if ( RTT_PLUGIN_${COMPONENT}_${OROCOS_TARGET}D_LIBRARY )
      list(APPEND RTT_PLUGIN_${COMPONENT}_${OROCOS_TARGET}_PROCESS_LIBS   RTT_PLUGIN_${COMPONENT}_${OROCOS_TARGET}D_LIBRARY)
    endif()
    if ( RTT_PLUGIN_${COMPONENT}_${OROCOS_TARGET}_LIBRARY )
      list(APPEND RTT_PLUGIN_${COMPONENT}_${OROCOS_TARGET}_PROCESS_LIBS   RTT_PLUGIN_${COMPONENT}_${OROCOS_TARGET}_LIBRARY)
    endif()

    # Forward FIND_REQUIRED
    if(RTTPlugin_FIND_REQUIRED)
        set(RTT_PLUGIN_${COMPONENT}_${OROCOS_TARGET}_FIND_REQUIRED TRUE)
    endif()

    # Forward FIND_QUIET
    if(RTTPlugin_FIND_QUIET)
        set(RTT_PLUGIN_${COMPONENT}_${OROCOS_TARGET}_FIND_QUIET TRUE)
    endif()
    
    libfind_process( RTT_PLUGIN_${COMPONENT}_${OROCOS_TARGET} )

    # Since libfind_process does not deal correctly with "optimized" and "debug" keywords,
    # we have to manualy add them thereafter
    if(RTT_PLUGIN_${COMPONENT}_${OROCOS_TARGET}_LIBRARY AND RTT_PLUGIN_${COMPONENT}_${OROCOS_TARGET}D_LIBRARY)
        set(RTT_PLUGIN_${COMPONENT}_LIBRARIES
            debug ${RTT_PLUGIN_${COMPONENT}D_LIBRARY}
            optimized ${RTT_PLUGIN_${COMPONENT}_LIBRARY})
        set(RTT_PLUGIN_${COMPONENT}_${OROCOS_TARGET}_LIBRARIES
            debug ${RTT_PLUGIN_${COMPONENT}_${OROCOS_TARGET}D_LIBRARY}
            optimized ${RTT_PLUGIN_${COMPONENT}_${OROCOS_TARGET}_LIBRARY})
    else()
        set(RTT_PLUGIN_${COMPONENT}_LIBRARIES "") 
        set(RTT_PLUGIN_${COMPONENT}_${OROCOS_TARGET}_LIBRARIES "")
    endif()

    if ( NOT "RTT_PLUGIN_${COMPONENT}_LIBRARIES" STREQUAL "RTT_PLUGIN_${COMPONENT}_${OROCOS_TARGET}_LIBRARIES" )
      # Avoid worst case and disable the generic case:
      message("[FindRTTPlugin] Forcing RTT_PLUGIN_${COMPONENT}_LIBRARIES to ${OROCOS_TARGET} since your are switching OROCOS_TARGETs")
      set( RTT_PLUGIN_${COMPONENT}_LIBRARIES ${RTT_PLUGIN_${COMPONENT}_${OROCOS_TARGET}_LIBRARIES})
    endif()
ENDFOREACH(COMPONENT)
