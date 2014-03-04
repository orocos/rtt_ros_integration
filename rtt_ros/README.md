RTT ROS
=======

This package contains tools for building and running Orocos components
integrated into a ROS architecture.

### RTT Services

#### ROS Package Import Chaining Service

The "ros" RTT service can be used to import all of the RTT plugins built in a
given ROS package along with plugins on which they depend. These dependencies
are specified in package.xml files in the `<export>` section.

For example, all RTT plugins in the ROS package "my\_package\_name" along with
its dependencies can be imported by running the following Orocos .ops script
(or calling the appropriate operation on the "ros" global service):

```python
import("rtt_ros")
ros.import("my_pkg_name")
```

When doing the above, first the `rtt_ros` package is imported using the normal
mechanism. This then loads the `ros` service, which provides a ROS import
function, `ros.import()`, which will parse ROS package metadata and import the
Orocos plugins from the named package _and_ all packages listed in
`<rtt_ros><plugin_depend>PKG_NAME</plugin_depend></rtt_ros>` tags in the
`<export>` section of the package.xml files.

Catkin introduces several new dependency types for buildtime, development, and
other purposes. In order to keep the RTT plugin depdencencies clear, and avoid
trying to load orocos components from _every_ package dependency, we use the
extensible ROS metadata `<export>` tag.

For example, if loading the plugins from package `pkg_one` should necessitate
first loading plugins from packages `rtt_roscomm` and `pkg_two`, then
`pkg_one/package.xml` should have the following:

```xml
<package>
  <name>pkg_one</name>
  <!-- ... -->
  <export>
    <rtt_ros>
      <plugin_depend>rtt_roscomm</plugin_depend>
      <plugin_depend>pkg_two</plugin_depend>
    </rtt_ros>
  </export>
</package>
```

### Launch Files

 * **[deployer.launch](launch/deployer.launch)** Launch the orocos deployer
   * **LOG_LEVEL** (default "info") The Orocos log level
   * **DEPLOYER_ARGS** (default "") Additional arguments passed to the deployer
   * **OROCOS_TARGET** (default `$OROCOS_TARGET`)
   * **RTT_COMPONENT_PATH** (default `$LD_LIBRARY_PATH`)
   * **DEBUG** (default `false`) Launch the deployer in GDB.

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

