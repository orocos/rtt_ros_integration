^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Orocos RTT/ROS Integration Changelog
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.7.0
=====

Orocos Version: 2.7.x
ROS Distribution: Hydro

The Orocos RTT/ROS Integration has been **heavily refactored** and extended in the
2.7 release. Part of this is due to the new buildsystem standard in the ROS
community, but also it is due to long-desired enhancements to the integration.

### Reorganization

Prior to the 2.7 release, the packages which interfaced core RTT functionality
with core ROS functionality was split across four "stacks":

 1. **rtt_ros_integration**: RTT plugins for communicating with ROS
 2. **rtt_ros_comm**: RTT Typekits for rosgraph_msgs and std_msgs
 3. **rtt_common_msgs**: RTT Typekits for common_msgs
 4. TODO: **rtt_geometry**: RTT typekits for KDL datatypes and RTT plugins for TF

#### Typekit Packages Merged into rtt_ros_integration

In the new release, the packages from these four repositories have been
reorganized. The two repositories which only contained typekits (rtt_ros_comm
and rtt_common_msgs) were moved into rtt_ros_integration under a directory
called "typekits". 

#### Refactor of rtt_ros_integration

The rtt_rosnode package from the old rtt_ros_integration stack contained the
following tools:

* Scripts to create RTT/ROS packags
* CMake build and code-generation scripts to create RTT typekits from ROS
messages
* An RTT plugin called "ros_integration" which instantiated a ROS node and
  provides several services:
  * An RTT global service called "ros" for constructing ROS topic connection policies:
  
    ```cpp 
    // Create a ROS topic connection policy
    var ConnPolicy cp = ros.topic("topic_name");
    // Create a buffered ROS topic connection policy
    var ConnPolicy cp_buffered = ros.topicBuffer("topic_name",10);
    ```

  * An RTT activity for publishing messages to ROS topics
  * An RTT component service called "rosparam" for loading and saving RTT
    component properties as ROS parameter server parameters:

    ```cpp
    loadService("ComponentName","rosparam");
    // Store all properties of this component to the ROS param server
    ComponentName.rosparam.storeProperties();
    // Refresh all properties of this component from the ROS param server
    ComponentName.rosparam.refreshProperties();
    // Store a specific property on the ROS param server
    var bool private_ns = false;
    var bool relative_ns = falsle;
    ComponentName.rosparam.storeProperty("prop_name", private_ns, relative_ns)
    // Refresh a specific property of this component from the ROS param server
    var bool private_ns = false;
    var bool relative_ns = falsle;
    ComponentName.rosparam.refreshProperty("prop_name", private_ns, relative_ns)
    ```
 
Orocos used to include optional built-in ROS support, wherein each Orocos
package could also be treated like a rosbuild package. This former design was
desirable due to the challenges in incorporating ROS package management
standards with non-ROS libraries. Orocos now takes advantage of
[Catkin](http://www.ros.org/wiki/catkin) in order to provide simple integration
with a catkin workspace while removing the dependency on ROS.

With rosbuild, all orocos components were built within a given package's own
build directory, and all generated typekit code was also placed in a given
package's include and src directories. Now, all such files are built in the
Catkin develspace.

As of now, the new interfaces have only been developed to support catkin-based
ROS packages, but adding backwards-compatible rosbuild support shouldn't be too hard.

There are a few organizational changes that have been made in
`rtt_ros_integration` compared to the ROS Groovy Galapagos release:

 * "rtt_rosnode" has been split into three packages: 
   1. "rtt_rosnode": Contains a plugin for creating a ROS node in an RTT program
   2. "rtt_roscomm": Contains msg primitive typekit and the transport plugin for using ROS topics
   3. "rtt_rosparam": Contains a plugin for synchronizing a component's properties with ROS parameters

 * A new package, "rtt_ros" has been added. This package contains convenience 
   launchfiles and wrapper scripts for running Orocos programs (typegen, 
   deployer, etc). This package also contains a plugin for importing ROS 
   packages and their dependencies, instead of overloading the use of the 
   deployer's "import()" function. 

There are also several API changes related to importing plugins from ROS 
packages and creating ROS topic connections.

2.7.0 (forthcoming)
-------------------
* Major reorganization (see additional notes in CHANGELOG.rst)
* Moved typekits from rtt_ros_comm and rtt_common_msgs into rtt_ros_integration
* Added Catkin buildsystem support
* Added RTT interfaces for ROS Service Calls (serving and calling)
* Added RTT interfaces for ROS Actionlib (action serving)
* Re-wrote RTT rosparam service to handle parameter arrays more cleanly
* 

2.6
===

Orocos Version: 2.6.x
ROS Distribution: Fuerte, Groovy

2.6.0.3 (2013-02-16)
--------------------
* Minor bugfixes

2.6.0.2 (2013-02-16)
--------------------
* Minor bugfixes

2.6.0-1 (2012-12-21)
--------------------
* Adding support for ROS Groovy 

2.6.0-0 (2012-11-22)
--------------------
* The RTT interface to rospack has been moved to a separate package called
  "rtt_rospack"
* rtt_rosnode: create_boost_headers.py is now compatible with ROS Fuerte

0.5
===

Orocos Version: 2.5.x
ROS Distribution: Electric

As of the ROS Electric release, the orocos_toolchain_ros stack is split up in 5
different stacks:

* orocos_toolchain: containing the bare orocos packages
* rtt_ros_integration (now a stack!): containing all orocos-ros integration code
* rtt_geometry: containing integration code for working with orocos and tf
* rtt_ros_comm: RTT typekits for the ros_comm messages
* rtt_common_msgs: RTT typekits for the common_msgs messages


0.5.0.7 (2011-11-08)
--------------------
* Minor bugfixes

0.5.0.6 (2011-10-21)
--------------------
* Minor bugfixes

0.5.0.5 (2011-10-04)
--------------------
* Remove rosdep.yaml file, it now lives in orocos_toolchain
* Minor bugfixes

0.5.0.4 (2011-10-04)
--------------------
* Minor bugfixes

0.5.0.3 (2011-09-29)
--------------------
* Added primitive typekits for ROS Time and Duration
* Minor bugfixes

0.5.0.2 (2011-09-29)
--------------------
* rtt_tf has been moved to the rtt_geometry stack

0.5.0.1 (2011-09-25)
--------------------
* Adding support for ROS Electric 
* Adding support for Orocos 2.5.x

0.4
===

Orocos Version: 2.4.x
ROS Distribution:  Diamondback

The stack contains all of the Orocos Toolchain v2.4.x integrated in the ROS
build system. The orocos_toolchain_ros stack contains utilmm, utilrb, typelib
and orogen, to automatically create ros packages for the automatic typekit
generation for C++ classes.

On top of the Orocos Toolchain v2.4.x this stack contains:

* rtt_ros_integration: This package contains the following:
  * The ros-plugin: this RTT plugin allows Orocos/RTT components to contact the
    ROS master
  * CMake macro's to automatically create Orocos/RTT typekits and transport
    plugins from .msg files
* rtt_ros_integration_std_msgs: This package shows how the CMake macro's have to
  be used, it creates the Orocos/RTT typekits and transport plugins for all
  roslib and std_msgs messages
* rtt_ros_integration_example: This package shows how the rtt_ros_integration
  should be used from an Orocos/RTT user/developer point of view. It contains a
  HelloRobot component which can be contacted using rostopic echo

0.4.0 (2011-06-27) 
------------------
* Initial Stack Release

0.0
===

Orocos Version: 2.4.x
ROS Distribution:  C-Turtle

0.0.0 (2010-09-10) 
------------------
* Initial Version
