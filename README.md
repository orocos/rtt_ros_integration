Orocos RTT / ROS Integration Packages
=====================================

## Introduction

This repository contains ROS packages necessary for building OROCOS libraries,
plugins, and components which communicate with the ROS messaging system and the
ROS parameter server.

## Packages

The packages in this repository provide:

 * **rtt\_ros** ROS package import plugin as well as wrapper scripts and
   launchfiles for using Orocos with ROS.
 * **rtt\_rosnode** Plugin for ROS node instantiation inside an Orocos program.
 * **rtt\_rosparam** Plugin for synchronizing ROS parameters with Orocos
   component properties.
 * **rtt\_rostopic** ROS message typekit generation and Orocos plugin for
   publishing and subscribing to ROS topics.
 * **rtt\_rospack** Plugin for locating ROS resources.
 * **rtt\_ros\_integration\_example** Example use of some of the features of
   this repository.
 * **rtt\_ros\_integration** Catkin
   [metapackage](http://ros.org/wiki/catkin/package.xml#Metapackages) for this
   repository.

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

## Using ROS-Based Orocos Plugins

Orocos plugins (components, typekits, plugins, etc.) are now built into the
catkin develspace lib directory.  Specificallly, they are built under
`devel/lib/$OROCOS_TARGET/PKG_NAME/`. These directories should be on the default
Oroco search path as long as `devel/lib` is in the `$RTT_COMPONENT_PATH` (which
happens automatically when using the launchfiles in **rtt\_ros**).

In order to import Orocos plugins built in a ROS package, there are two
options:

  1. _Import single package_:

    ```python
    import("my_pkg_name")
    ```

  2. _Import package and it's rtt plugin dependencies_: 

    ```python
    import("rtt_ros")
    ros.import("my_pkg_name")
    ```

In **Option 1**, a single ROS package with orocos plugins can still be imported
with the standard deployer `import()` function. However, Orocos RTT 2.7 no
longer parses ROS package metadata in order to import all of a plugin's
dependencies, so only the named package will be imported.

In **Option 2**, first the `rtt_ros` package is imported using the normal
mechanism. This then loads the `ros` service, which provides a ROS import
function which will parse ROS package metadata and import the Orocos plugins
from the named package _and_ all packages listed in `<rtt_plugin_depend>` tags
in the `<export>` section of the package.xml files.

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
