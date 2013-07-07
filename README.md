Orocos RTT / ROS Integration Packages
=====================================

## Introduction

This repository contains ROS packages necessary for building OROCOS libraries,
plugins, and components which communicate with the ROS messaging system and the
ROS parameter server.

## Packages

The packages in this repository provide:

 * **rtt\_rostopic** ROS message typekit generation.
 * **rtt\_ros** Plugin for ROS node instantiation inside an Orocos program, and common
   roslaunch files.
 * **rtt\_rosparam** Plugin for synchronizing ROS parameters with Orocos
   component properties
 * **rtt\_rospacak** Plugin for locating ROS resources.

See each package's README.md file for more information.

## Building Orocos From Source

The [Orocos Toolchain](http://www.orocos.org/orocos/toolchain) can be built from
source in a catkin workspace using `catkin_build_isolated` since Orocos packages
now contain `package.xml` files. 

```shell
mkdir -p ~/ws/underlay_isolated/src/orocos
mkdir -p ~/ws/underlay/src
cd ~/ws/underlay_isolated/src/orocos
git clone --recursive git://gitorious.org/orocos-toolchain/orocos_toolchain.git
cd ~/ws/underlay_isolated
catkin_make_isolated --install
```

## History

Orocos used to include optional built-in ROS support, wherein each Orocos
package could also be treated like a rosbuild package. This former design was
desirable due to the challenges in incorporating ROS package management
standards with non-ROS libraries. Orocos now takes advantage of
[catkin](http://www.ros.org/wiki/catkin) in order to provide simple integration
with a catkin workspace while removing the dependency on ROS.

With rosbuild, all orocos components were built within a given package's own
build directory, and all generated typekit code was also placed in a given
package's include and src directories. Now, all such files are built in the
catkin develspace.
