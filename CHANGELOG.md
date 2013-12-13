# Orocos RTT/ROS Integration Changelog

## Version 2.7 (ROS Hydro Distribution)

The Orocos RTT/ROS Integration has been heavily refactored and extended in the
2.7 release. Part of this is due to the new buildsystem standard in the ROS
community, but also it is due to long-desired enhancements to the integration.

### Reorganization

Prior to the 2.7 release, the packages which interfaced core RTT functionality
with core ROS functionality was split across three "stacks":

 1. rtt_ros_integration: RTT plugins for communicating with ROS
 2. rtt_ros_comm: RTT Typekits for rosgraph_msgs and std_msgs
 3. rtt_common_msgs: RTT Typekits for common_msgs
 4. TODO: rtt_geometry: RTT typekits for KDL datatypes and RTT plugins for TF

rtt_ros_integration contained two packages: rtt_rosnode and rtt_rospack.

#### Typekit Packages Merged into rtt_ros_integration

In the new release, the packages from these four repositories have been
reorganized. The two repositories which only contained typekits (rtt_ros_comm
and rtt_common_msgs) were moved into rtt_ros_integration under a directory
called "typekits". 

#### Refactor of rtt_rosnode

The rtt_rosnode package contained the following tools:

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

## Version 2.6 (ROS Groovy Distribution)

