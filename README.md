Orocos RTT / ROS Integration Packages
=====================================

## Introduction

This repository contains ROS packages necessary for building OROCOS libraries,
plugins, and components which communicate with the ROS messaging system and the
ROS parameter server.

## Changelog

See the metapackage [rtt_ros_integration/CHANGELOG.rst](CHANGELOG.rst) for a
comprehensive changelog.

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
  [actionlib](http://ros.org/wiki/actionlib) action server for providing
  actions from ROS-integrated RTT components.
* [**rtt\_ros\_integration**](rtt_ros_integration) Catkin
  [metapackage](http://ros.org/wiki/catkin/package.xml#Metapackages) for this
  repository.

***See each package's README.md file for more information.***

## Usage

For numerous examples of usage, see the
[**rtt\_ros\_examples**](http://github.com/jhu-lcsr/rtt_ros_examples)
stack.

### Building Orocos From Source

The [Orocos Toolchain](http://www.orocos.org/orocos/toolchain) can be built from
source in a Catkin workspace using `catkin_build_isolated` since Orocos packages
now contain Catkin `package.xml` files. 

First, create an isolated underlay for building plain CMake-based packages like
Orocos:
```shell
export OROCOS_TARGET=gnulinux
mkdir -p ~/ws/underlay_isolated/src/orocos
cd ~/ws/underlay_isolated
git clone --recursive git@github.com:orocos-toolchain/orocos_toolchain.git src/orocos/orocos_toolchain
catkin_make_isolated --install
source install/setup.sh
```

Then, in the same shell, create an underlay for building Catkin-based packages:
```shell
mkdir -p ~/ws/underlay/src
cd ~/ws/underlay
git clone git@github.com:orocos/rtt_ros_integration.git src/rtt_ros_integration
catkin_make
source devel/setup.sh
```

At this point you can create Catkin or rosbuild packages which use the
rtt\_ros\_integration tools.

### Building ROS-Based Orocos Components

While the ROS community has standardized on the rosbuild (ROS Hydro and earlier)
and Catkin (ROS Groovy and later) buildsystems, Orocos has its own
CMake/PkgConfig-based package description system which uses
[Autoproj](http://rock-robotics.org/stable/documentation/autoproj/) manifest.xml
files similar to rosbuild manifest.xml files. 

This is primarily because Orocos builds its libraries with respect to a given
`$OROCOS_TARGET` (gnulinux/xenomai/macosx/etc) so that you can build multiple
versions of the same library in place without having to rebuild everything
whenever you change targets.

So in order to build Orocos components in a rosbuild or Catkin package, you need
to first include the RTT use-file:

```cmake
find_package(OROCOS-RTT REQUIRED ...)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
```

When this file is included, it both defines and executes several macros.
Specifically, it parses the package.xml or manifest.xml of the including
package, and executes `orocos_find_package(pkg-name)` on all build dependencies.
This populates several variables includeing, but not limited to
`${OROCOS_USE_INCLUDE_DIRS}` and  `${OROCOS_USE_LIBRARIES}` which are used by
Orocos target- and package-definition macros like `orocos_executable()`,
`orocos_library()` and `orocos_generate_package()`.

Also, while the `orocos_find_package()` macro can be used to find both
Orocos-based packages and normal pkg-config-based packages, you should only use
it for Orocos-based packages. You should use the normal CMake and Catkin
mechanisms for all non-Orocos dependencies. As long as the names of orocos
packages are listed as `<build_depend>` dependencies in your package.xml files,
their build flags will automatically be made available when buildinf your
package. Listing them in the package.xml file will also enforce proper build
ordering.

To build components, libraries, typekits, and other Orocos plugins, use the
standard `orocos_*()` CMake macros. Then to make these available to other
packages at build-time (through `orocos_find_package()`), declare an Orocos
package at the end of your CMakeLists.txt file:

```cmake
orocos_generate_package(DEPENDS some-other-oro-pkg)
```

See the Orocos RTT documentation (or [cheat
sheet](http://www.orocos.org/stable/documentation/rtt/v2.x/doc-xml/rtt_cheat_sheet.pdf))
for more info on these macros.

**NOTE:** You still need to call `find_package(catkin ...)` and
`catkin_package(...)` for non-orocos dependencies and targets, but you shuold
use the `orocos_*()` CMake macros for Orocos-based code.

### Dynamically Loading ROS-Based Orocos Plugins

Orocos plugins (components, typekits, plugins, etc.) are now built into the
Catkin develspace lib directory.  Specificallly, they are built under
`devel/lib/orocos/$OROCOS_TARGET/PKG_NAME/`. These directories should be on the 
default Orocos search path as long as `devel/lib/orocos` is in the `$RTT_COMPONENT_PATH`
(which happens automatically through the env-hooks supplied by **rtt\_ros**).

In order to import Orocos plugins built in a ROS package and all of that plugin's
dependencies, no matter where it is, you can use the `ros.import()` service:

```python
import("rtt_ros")
ros.import("my_pkg_name")
```

In this example, first the `rtt_ros` package is imported using the normal
mechanism. This loads the `ros` service, which provides a ROS import
function, `ros.import()`, which will parse ROS package metadata and import the
Orocos plugins from the named package _and_ all packages listed in
`<rtt_ros><plugin_depend>PKG_NAME</plugin_depend></rtt_ros>` tags in the
`<export>` section of the package.xml files.

A single ROS package with orocos plugins can still be imported
with the standard deployer `import()` function. However, this will only work
if the named package is built in the same workspace as `rtt_ros` or a 
workspace which `rtt_ros` extends. Additionally, Orocos RTT 2.7 no
longer parses ROS package metadata in order to import all of a plugin's
dependencies, so only the named package will be imported.

For more information on specifying RTT plugin dependencies in ROS packages, see
the README in the [rtt_ros](rtt_ros) package.

### Bulding ROS-Based Orocos Plugins

Orocos plugins are built normally, with Orocos CMake macros. See
[rtt_actionlib](rtt_actionlib/CMakeLists.txt) for 
an example of an Orocos RTT service plugin.

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

The `rtt_roscomm` package provides RTT services for binding an Orocos RTT
operation or operation caller to the ROS service server or client,
respectively. See [rtt_roscomm](rtt_roscomm) for more information.

### Running an Actionlib Action Server in an Orocos Component

The `rtt_actionlib` package provides a C++ API and an RTT service for
implementing [actionlib](http://www.ros.org/wiki/actionlib) actions with Orocos
RTT components. See [rtt_actionlib](rtt_actionlib) for more information.


## Future Work

The following packages are in the planning stages, please contact the
maintainers if you're interested in using or contributing to them:

* [**rtt\_rostime**](rtt_rostime) Plugin for basing RTT time off of ROS sim
  time.
* [**rtt\_rosops**](rtt_rosops) Plugin for executing Orocos Ops script via ROS
  service call.
* [**rtt\_dynamic_reconfigure**](rtt_dynamic_reconfigure) Plugin for running
  a [dynamic\_reconfigure](http://ros.org/wiki/dynamic_reconfigure) server from
  an RTT component.

