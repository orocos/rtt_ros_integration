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
* [**rtt\_rosclock**](rtt_rosclock) Realtime-Safe NTP clock measurement and ROS
  `Time` structure construction as well as a simulation-clock-based periodic
  RTT activity.
* [**rtt\_rosnode**](rtt_rosnode) Plugin for ROS node instantiation inside an
  Orocos program.
* [**rtt\_rosparam**](rtt_rosparam) Plugin for synchronizing ROS parameters
  with Orocos component properties.
* [**rtt\_roscomm**](rtt_roscomm) ROS message typekit generation and Orocos
  plugin for publishing and subscribing to ROS topics as well as calling and
  responding to ROS services.
* [**rtt\_rospack**](rtt_rospack) Plugin for locating ROS resources.
* [**rtt\_tf**](rtt_tf) RTT-Plugin which uses [tf](http://ros.org/wiki/tf) to
  allow RTT components to lookup and publish transforms.
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

### Installing Orocos From Binary Packages

The Orocos toolchain and the rtt_ros_integration packages are available as 
binary packages hosted by the Open Source Robotics Foundation (OSRF) and can be
installed on supported operating systems.

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
git clone --recursive git://gitorious.org/orocos-toolchain/orocos_toolchain.git -b toolchain-2.7 src/orocos/orocos_toolchain
catkin_make_isolated --install
source install_isolated/setup.sh
```

Then, in the same shell, create an underlay for building Catkin-based packages:
```shell
mkdir -p ~/ws/underlay/src
cd ~/ws/underlay
git clone https://github.com/orocos/rtt_ros_integration.git src/rtt_ros_integration
catkin_make
source devel/setup.sh
```

At this point you can create Catkin or rosbuild packages which use the
rtt\_ros\_integration tools.

### Creating an Orocos-ROS Package

The Orocos and ROS communities have both standardized on using CMake for building source code, and have also both developed independent CMake macros for assisting the process. The ROS community has developed [catkin](http://github.com/ros/catkin) and the Orocos toolchain uses Orocos-specific macros. These macros are used for exporting (declaring) and retreiving inter-package dependencies. It's a good, conflict-preventing, practice to decide when a package should be characterized by Catkin-based or Orocos-based macros.

Any package that builds orocos targets (plugins, components, executables, etc) _needs_to call the `orocos_generate_package()` macro so that the appropriate platform-specific pkg-config .pc files are generated. These packages can _depend_ on ROS libraries, but they should avoid _exporting_ headers, libraries, or other resources via the `catkin_package()` macro.

A simple Orocos-ROS package looks like the following:

```
my_orocos_pkg
├── README.md
├── CMakeLists.txt
├── package.xml
├── include
│   └── my_orocos_pkg
└── src
```

Where the CMakeLists.txt has the following directives:

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(my_orocos_pkg)

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  # ADDITIONAL ROS PACKAGES
  )

include_directories(${catkin_INCLUDE_DIRS})

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file

include_directories(${USE_OROCOS_INCLUDE_DIRS})

### Orocos Targets ###

# orocos_component(my_component src/my_component.cpp)
# target_link_libraries(my_component ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

# orocos_library(my_library src/my_library.cpp)
# target_link_libraries(my_library ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

# orocos_service(my_service src/my_service.cpp)
# target_link_libraries(my_service ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

# orocos_plugin(my_plugin src/my_plugin.cpp)
# target_link_libraries(my_plugin ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

# orocos_typekit(my_typekit src/my_typekit.cpp)
# target_link_libraries(my_typekit ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Package Exports and Install Targets ###

# Generate install targets for header files

orocos_install_headers(DIRECTORY include/${PROJECT_NAME})

# Export package information (replaces catkin_package() macro) 
orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros
)
```

The package.xml file is a normal Catkin package.xml file, with some additional export flags for ROS plugin auto-loading:

```xml
<package>
  <name>my_orocos_package</name>
  <version>0.1.0</version>
  <license>BSD</license>
  <maintainer email="name@domain.com">Firstname Lastname</maintainer>
  <description>
    Package description.
  </description>

  <buildtool_depend>catkin</buildtool_depend>

  <!-- Build deps are queried automatically with orocos_use_package() -->
  <build_depend>rtt</build_depend>
  <build_depend>ocl</build_depend>
  <build_depend>rtt_ros</build_depend>

  <run_depend>rtt</run_depend>
  <run_depend>ocl</run_depend>
  <run_depend>rtt_ros</run_depend>

  <!-- ROS Msg Typekits and Srv Proxies -->
  <build_depend>rtt_sensor_msgs</build_depend>
  <run_depend>rtt_sensor_msgs</run_depend>

  <export>
    <rtt_ros>
      <!-- Plugin deps are loaded automatically by the rtt_ros import service -->
      <plugin_depend>rtt_sensor_msgs</plugin_depend>
    </rtt_ros>
  </export>
</package>
```

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
to first include the RTT CMake macros. This is done automatically when you find
the `rtt_ros` package:

```cmake
find_package(catkin REQUIRED COMPONENTS rtt_ros)
```

If you need other RTT libraries like the CORBA transport etc, you can use the
`use_orocos()` macro provided by the `rtt_ros` package:

```cmake
use_orocos(rtt-transport-corba)
```

The above is equivalent to calling the following:

```cmake
find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-transport-corba)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )
```

When this file is included, it both defines and executes several macros.
Specifically, it parses the package.xml or manifest.xml of the including
package, and executes `orocos_use_package(pkg-name)` on all build dependencies.
This populates several variables including, but not limited to
`${OROCOS_USE_INCLUDE_DIRS}` and  `${OROCOS_USE_LIBRARIES}` which are used by
Orocos target- and package-definition macros like `orocos_executable()`,
`orocos_library()` and `orocos_generate_package()`.

Also, while the `orocos_use_package()` macro can be used to find both
Orocos-based packages and normal pkg-config-based packages, you should only use
it for Orocos-based packages. You should use the normal CMake and Catkin
mechanisms for all non-Orocos dependencies. As long as the names of _orocos_
packages are listed as `<build_depend>` dependencies in your package.xml file,
their build flags will automatically be made available when building your
package. _Do not_ use `find_package(catkin COMPONENTS)` to find orocos packages,
since catkin doesn't properly handle the orocos-target-specific packages. Listing 
them in the package.xml file will also enforce proper build ordering.

To build components, libraries, typekits, and other Orocos plugins, use the
standard `orocos_*()` CMake macros. Then to make these available to other
packages at build-time (through `orocos_use_package()`), declare an Orocos
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

* [**rtt\_rosops**](rtt_rosops) Plugin for executing Orocos Ops script via ROS
  service call.
* [**rtt\_dynamic_reconfigure**](rtt_dynamic_reconfigure) Plugin for running
  a [dynamic\_reconfigure](http://ros.org/wiki/dynamic_reconfigure) server from
  an RTT component.

