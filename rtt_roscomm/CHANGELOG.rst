^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rtt_roscomm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.9.0 (2017-05-02)
------------------
* rtt_roscomm: find templates and create_boost_header.py script directly in the source-space
* rtt_roscomm: fixed missing package headers include directories for service proxies (fix `#87 <https://github.com/orocos/rtt_ros_integration/issues/87>`_)
* rtt_roscomm: remove using namespace directive from rtt_rostopic_ros_msg_transporter.hpp header
* Added deprecation warning for header rtt_roscomm/rtt_rostopic.h and updated some include directives within rtt_ros_integration
* rtt_roscomm: remove using namespace directive from rtt_rostopic_ros_msg_transporter.hpp header
* rtt_roscomm: renamed header rtt_rostopic.h to rostopic.h and changed namespace for the ROSService service requester for consistency
* rtt_roscomm: added new operations to the documentation in README.md
* rtt_roscomm: get rid of custom IDL
* rtt_roscomm: use @ROSMSGTYPE@ variable in ros_msg_corba_conversion.hpp.in to allow reuse for custom types
* rtt_roscomm: do not include boost header from Types.hpp
* rtt_roscomm: avoid unnecessary copy during conversion of ROS types to CORBA sequence and catch StreamOverrunException
* rtt_roscomm: do not generate unused source files for per-message typekit
* rtt_roscomm: avoid mismatched-tags warning in clang by removing the extern template declaration and instantiation for RTT::internal::DataSourceTypeInfo<T>
* rtt_roscomm: introduced cmake options ENABLE_MQ and ENABLE_CORBA and disable additional transport plugins by default
* Added individual changelogs and bumped versions to 2.9.0
* Also add a virtual destructor to the base class of the ROS Service Proxy
* Added an explicit destructor to shutdown services servers, and cleanup the registered proxies
* Added CORBA and mqueue transport for ROS typekits
* rtt_roscomm: added support for updated dataflow semantics (RTT version >= 2.8.99)
* Contributors: Antoine Hoarau, Guillaume Walck, Johannes Meyer

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
