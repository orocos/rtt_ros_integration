^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Orocos RTT/ROS Integration Changelog
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.9.0 (2017-05-02)
------------------
* Added individual changelogs and bumped versions to 2.9.0
* Contributors: Johannes Meyer

2.8.5 (2017-03-28)
------------------

2.8.4 (2016-11-26)
------------------

2.8.3 (2016-07-20)
------------------
* Please check the changelogs of the individual packages, e.g.:

  - `rtt_roscomm <../rtt_roscomm/CHANGELOG.rst>`_
  - `rtt_rosclock <../rtt_rosclock/CHANGELOG.rst>`_
  - `rtt_rosparam <../rtt_rosparam/CHANGELOG.rst>`_
  - `rtt_dynamic_reconfigure <../rtt_dynamic_reconfigure/CHANGELOG.rst>`_
  - ...

2.8.2 (2015-06-12)
-----------

2.8.1 (2015-03-16)
------------------

2.7.x
=====

:Orocos Version: 2.7.x
:ROS Distro: Hydro

The Orocos RTT/ROS Integration has been **heavily refactored** and extended in the
2.7 release. Part of this is due to the new Catkin buildsystem standard in the ROS
community, but also it is due to long-desired enhancements to the integration 
packages.

Prior to the 2.7 release, the packages which interfaced core RTT functionality
with core ROS functionality was split across four "stacks":

1. rtt_ros_integration: RTT plugins for communicating with ROS
2. rtt_ros_comm: RTT Typekits for rosgraph_msgs and std_msgs
3. rtt_common_msgs: RTT Typekits for common_msgs
4. **TODO:** rtt_geometry: RTT typekits for KDL datatypes and RTT plugins for TF

In the 2.7.0 release, the packages from these four repositories have been
reorganized. The two repositories which only contained typekits (rtt_ros_comm
and rtt_common_msgs) were moved into rtt_ros_integration under a directory
called "typekits". 

The rtt_rosnode package from the old rtt_ros_integration stack has also been 
refactored. In previous releases, it contained the following tools:

* Scripts to create RTT/ROS packags
* CMake build and code-generation scripts to create RTT typekits from ROS messages
* An RTT plugin called "ros_integration" which instantiated a ROS node and provides several services
  
  - An RTT global service called "ros" for constructing ROS topic connection policies.
  - An RTT activity for publishing messages to ROS topics
  - An RTT component service called "rosparam" for loading and saving RTT component properties as ROS parameter server parameters.
 
In this release, the "rtt_rosnode" has been split into four packages:

* "rtt_ros": Core ROS system integration

  - Convenience launchfiles and wrapper scripts for running Orocos programs (typegen, deployer, etc) 
  - An RTT global service for importing ROS packages and their dependencies
  
* "rtt_rosnode": Contains a plugin for creating a ROS node in an RTT program
* "rtt_roscomm": Contains ROS communication plugins including:
  
  - ROS msg primitives typekit
  - An RTT transport plugin for communicating via ROS topics and RTT ports
  - An RTT plugin for serving and calling ROS services via RTT operations
  - A tempalte and package generator for integrating new ROS msg and srv types
    
* "rtt_rosparam": Contains a plugin for synchronizing a component's properties with ROS parameters

There are also several API changes related to importing plugins from ROS 
packages, creating ROS topic connections, and synchronizing RTT properties
with ROS parameter server parameters.
 
See each package's individual CHANGELOG.rst for more details.


2.7.0 (2013-12-15)
------------------
* Major reorganization (see additional notes in CHANGELOG.rst)
* Moved typekits from rtt_ros_comm and rtt_common_msgs into rtt_ros_integration
* Added Catkin buildsystem support
* Added RTT interfaces for ROS Service Calls (serving and calling)
* Added RTT interfaces for ROS Actionlib (action serving)
* Re-wrote RTT rosparam service to handle parameter arrays more cleanly
* Added ROS service interface to interact with the Orocos deployer (rtt_rosdeployment)
* Added RTT interface to dynamic_reconfigure (rtt_dynamic_reconfigure)

2.6.x
=====

:Orocos Version: 2.6.x
:ROS Distro: Fuerte, Groovy, Hydro

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

0.5.x
=====

:Orocos Version: 2.5.x
:ROS Distro: Electric

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

0.4.x
=====

:Orocos Version: 2.4.x
:ROS Distro: Diamondback

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
* Initial stack release

0.0.x
=====

:Orocos Version: 2.4.x
:ROS Distro: C-Turtle

0.0.0 (2010-09-10) 
------------------
* Initial development version
