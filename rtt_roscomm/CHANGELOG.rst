^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rtt_roscomm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.8.6 (2017-11-15)
------------------
* rtt_roscomm: also set ${PROJECT_NAME}_EXPORTED_LIBRARIES in parent scope
  Related to https://github.com/orocos-toolchain/rtt/pull/244.
* Contributors: Johannes Meyer

2.8.5 (2017-03-28)
------------------
* Merge pull request `#85 <https://github.com/orocos/rtt_ros_integration/issues/85>`_ from meyerj/ros-primitives-transport-indigo-devel
  Added a ROS transport plugin for primitive types (indigo-devel)
* rtt_roscomm: fix caller engine in RosServiceServerProxyBase to make sure that OwnThread operations are executed in the owner's thread
* rtt_roscomm: added topicLatched() method to rtt_rostopic service
* rtt_roscomm: only set CMAKE_BUILD_TYPE to MinSizeRel if either not set or if it was Release before
  This enables debugging of ROS typekits.
* Contributors: Johannes Meyer

2.8.4 (2016-11-26)
------------------
* Merge pull request `#79 <https://github.com/orocos/rtt_ros_integration/issues/79>`_ from meyerj/added-rtt-rosservice-operations
  rtt_roscomm: added operations disconnect() and disconnectAll() to the rosservice service
* Merge branch 'B`#59 <https://github.com/orocos/rtt_ros_integration/issues/59>`__cleaning_registered_services' of https://github.com/ubi-agni/rtt_ros_integration into indigo-devel
* rtt_roscomm: include exported headers and link typekit and transport plugin to exported libraries
* rtt_roscomm: export build dependency roscpp
* Contributors: Johannes Meyer, Guillaume Walck

2.8.3 (2016-07-20)
------------------
* rtt_roscomm: set minimum ROS subscriber queue_size to 1
* rtt_roscomm: fixed destruction of RosSubChannelElement<T> and ROS subscriber shutdown (fix `#61 <https://github.com/orocos/rtt_ros_integration/issues/61>`_)
* Contributors: Johannes Meyer

2.8.2 (2015-06-12)
------------------
* see `rtt_ros_integratoin/CHANGELOG.rst <../rtt_ros_integration/CHANGELOG.rst>`_
