RTT ROS
=======

This package contains tools for building and running Orocos components
integrated into a ROS architecture.

### CMake Macros

The following are simple wrappers to the respective Orocos CMake macros which
sets the proper target output directory so that it can be found in a catkin
workspace.

 * `orocos_plugin( LIB_TARGET_NAME )`
 * `orocos_component( LIB_TARGET_NAME )`
 * `orocos_typekit( LIB_TARGET_NAME )`
 * `orocos_service( LIB_TARGET_NAME )`

### Launch Files

 * **[deployer.launch](launch/deployer.launch)** Launch the orocos deployer
   * **LOG_LEVEL** (default "info") The Orocos log level
   * **DEPLOYER_ARGS** (default "") Additional arguments passed to the deployer
   * **OROCOS_TARGET** (default `$OROCOS_TARGET`)
   * **RTT_COMPONENT_PATH** (default `$LD_LIBRARY_PATH`)

#### Launchfile Examples

For example, the deployer can be launched by including the launchfile in
another launchfile like the following:

```xml
<launch>
  <include file="$(find rtt_ros)/launch/deployer.launch">
    <arg name="DEPLOYER_ARGS" value="-s $(find rtt_ros_integration_example)/example.ops"/>
    <arg name="LOG_LEVEL" value="debug"/>
    <arg name="DEBUG" value="false"/>
  </include>
</launch>
```

### Wrapper Scripts

 * deployer
 * orocreate-pkg
 * orogen
 * orogen-unregister
 * rttlua
 * rttlua-tlsf
 * rttscript
 * typegen

### Environment Hooks

This package also provides catkin env-hooks which will get executed when a user
sources the catkin-generated setup.sh scripts. These hooks declare Orocos
environment variables needed for setting the system configuration and finding
Orocos resources.
