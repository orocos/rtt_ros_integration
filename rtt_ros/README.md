RTT ROS
=======

This package contains tools for building and running Orocos components
integrated into a ROS architecture.

### CMake Macros

The following are simple wrappers to the respective Orocos CMake macros which
sets the proper target output directory so that it can be found in a catkin
workspace.

 * `catkin_orocos_plugin( LIB_TARGET_NAME )`
 * `catkin_orocos_component( LIB_TARGET_NAME )`
 * `catkin_orocos_typekit( LIB_TARGET_NAME )`
 * `catkin_orocos_service( LIB_TARGET_NAME )`

### Launch Files

 * **deployer.launch** Launch the orocos deployer
   * **LOG_LEVEL** (default "info") The Orocos log level
   * **DEPLOYER_ARGS** (default "") Additional arguments passed to the deployer
   * **OROCOS_TARGET** (default `$OROCOS_TARGET`)
   * **RTT_COMPONENT_PATH** (default `$LD_LIBRARY_PATH`)

### Wrapper Scripts

 * deployer
 * orocreate-pkg
 * orogen
 * orogen-unregister
 * rttlua
 * rttlua-tlsf
 * rttscript
 * typegen
