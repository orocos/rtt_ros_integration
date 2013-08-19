Orocos RTT / ROS Integration Packages
=====================================

## Introduction

This repository contains ROS packages necessary for building OROCOS libraries,
plugins, and components which communicate with the ROS messaging system and the
ROS parameter server.

## Packages

The packages in this repository provide:

 * [**rtt\_ros**](rtt_ros) ROS package import plugin as well as wrapper scripts
   and launchfiles for using Orocos with ROS.
 * [**rtt\_rosnode**](rtt_rosnode) Plugin for ROS node instantiation inside an
   Orocos program.
 * [**rtt\_rosparam**](rtt_rosparam) Plugin for synchronizing ROS parameters
   with Orocos component properties.
 * [**rtt\_roscomm**](rtt_roscomm) ROS message typekit generation and Orocos
   plugin for publishing and subscribing to ROS topics as well as calling and
   responding to ROS services.
 * [**rtt\_rospack**](rtt_rospack) Plugin for locating ROS resources.
 * [**rtt\_actionlib**](rtt_actionlib) RTT-Enabled
   [actionlib](http://ros.org/wiki/actionlib) action server and client for
   providing actions from ROS-integrated RTT components.
 * [**rtt\_ros\_integration**](rtt_ros_integration) Catkin
   [metapackage](http://ros.org/wiki/catkin/package.xml#Metapackages) for this
   repository.

See each package's README.md file for more information.

## Usage

For numerous examples of usage, see the
[**rtt\_ros\_examples**](http://github.com/jhu-lcsr/rtt_ros_examples)
stack.

### Building Orocos From Source

The [Orocos Toolchain](http://www.orocos.org/orocos/toolchain) can be built from
source in a Catkin workspace using `catkin_build_isolated` since Orocos packages
now contain Catkin `package.xml` files. 

```shell
mkdir -p ~/ws/underlay_isolated/src/orocos
mkdir -p ~/ws/underlay/src
cd ~/ws/underlay_isolated/src/orocos
git clone --recursive git://gitorious.org/orocos-toolchain/orocos_toolchain.git
cd ~/ws/underlay_isolated
catkin_make_isolated --install
```

### Using ROS-Based Orocos Plugins

Orocos plugins (components, typekits, plugins, etc.) are now built into the
Catkin develspace lib directory.  Specificallly, they are built under
`devel/lib/$OROCOS_TARGET/PKG_NAME/`. These directories should be on the default
Oroco search path as long as `devel/lib` is in the `$RTT_COMPONENT_PATH` (which
happens automatically when using the launchfiles in **rtt\_ros**).

In order to import Orocos plugins built in a ROS package, there are two
options:

  1. _Import a **single** package_:

    ```python
    import("my_pkg_name")
    ```

  2. _Import package **and** it's rtt plugin dependencies_: 

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

### Bulding ROS-Based Orocos Plugins

Orocos plugins are built normally, with Orocos CMake macros. See
[rtt_ros_integration_example](rtt_ros_integration_example/CMakeLists.txt) for 
an example.

### Running Orocos Programs

The `rtt_ros` package provides several launchfiles and wrapper scripts for
making it easier to Orocos programs in a ROS environment. See
[rtt_ros](rtt_ros) for more information.

### Connecting Orocos Ports to ROS Topics

The `rtt_roscomm` package provides a typekit for the the ROS Message 
primitives, as well as a plugin which manages construction of ROS publishers
and subscribers. See [rtt_roscomm](rtt_roscomm) for more 
information. 

### Connecting Orocos Operations to ROS Services

The `rtt_rsocomm` package provides RTT services for binding an Orocos RTT
operation or operation caller to the ROS service server or client,
respectively. See [rtt_roscomm](rtt_roscomm) for more information.

### Running an Actionlib Action Server in an Orocos Component

In progress...
(See current development in [rtt_actionlib](rtt_actionlib))
(See current implementation [here](https://github.com/RCPRG-ros-pkg/orocos_tools))

## History

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
