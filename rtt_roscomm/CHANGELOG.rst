^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rtt_roscomm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.10.0 (2021-01-04)
-------------------
* Merge remote-tracking branch 'origin/toolchain-2.9' into toolchain-2.10
* Merge pull request `#154 <https://github.com/orocos/rtt_ros_integration/issues/154>`_ from orocos/feature/add-licenses
  add LICENSE files to rtt_ros_integration packages
* update e-mail of Orocos Developers in package.xml files
* licenses: update text and make it match to current BSD-3
  New update on the files with licenses headers.
* fix licenses format in source files
  This patch applies a general homogeneous format for the licenses
  accross the packages.
* homogenize licenses to BSD
  The patch reorganizes all the licesnes to match to BSD and a
  single LICENSE file is placed in the root of the repository.
* Merge remote-tracking branch 'origin/toolchain-2.9' into toolchain-2.10
* Merge pull request `#140 <https://github.com/orocos/rtt_ros_integration/issues/140>`_ from orocos/fix/create-boost-headers-with-python3
  rtt_roscomm: update create_boost_header.py for Python3 compatibility
* rtt_roscomm: update create_boost_header.py for Python3 compatibility
* Merge pull request `#134 <https://github.com/orocos/rtt_ros_integration/issues/134>`_ from orocos/fix-rtt_roscomm-rosservice-namespace
  rtt_roscomm: add rtt_roscomm namespace for rosservice plugins and helper classes
* Merge branch 'toolchain-2.9' into fix-rtt_roscomm-rosservice-namespace
* Merge pull request `#132 <https://github.com/orocos/rtt_ros_integration/issues/132>`_ from orocos/fix/rtt_roscomm-parallel-builds
  rtt_roscomm: replace add_file_dependencies() macro call by a custom target
* Merge pull request `#133 <https://github.com/orocos/rtt_ros_integration/issues/133>`_ from orocos/fix-rtt_std_srvs-for-standard-operations-with-clang
  rtt_roscomm: fixed compilation of service plugins with Clang
* rtt_roscomm: add rtt_roscomm namespace for rosservice plugins and helper classes
* rtt_roscomm: fixed compilation of service plugins with Clang (regression from https://github.com/orocos/rtt_ros_integration/pull/123)
  Clang seems to handle SFINAE differently than gcc and compilation of generated service proxy plugins failed with errors like:
  In file included from rtt_diagnostic_msgs/diagnostic_msgs_service_proxies/rtt_rosservice_proxies.cpp:7:
  rtt_roscomm/rtt_rosservice_proxy.h:99:30: error: implicit instantiation of undefined template 'ROSServiceServerOperationCallerWrapper<diagnostic_msgs::AddDiagnostics, 1>'
* rtt_roscomm: replace add_file_dependencies() macro call by a custom target
  Fixes build issues with parallel builds. If multiple targets depend on the same generated header file,
  the file gets generated more than once, too (with CMake 3.5 in Ubuntu xenial).
  Each invocation of the compiler might therefore see an inconsistent state of the generated header file
  if another process associated with another target is about to overwrite the same file. The solution is
  to replace the file-level dependencies by an explicit rtt-package-generate_boost_headers custom target
  that all library targets depend on.
  The dependencies on the current CMake list file (CMAKE_CURRENT_LIST_FILE) are not required because if
  the file would change, cmake needs to run again and is normally executed automatically. If not, the
  file dependency does not help either because the contents of the generated source files did not change.
  Changes in upstream message or service definitions (.msg or .srv files) are already covered by the
  declared dependencies of the generated boost headers or the implicit target dependency from including
  the generated service headers.
* Merge tag '2.9.2' into toolchain-2.9
* Merge pull request `#124 <https://github.com/orocos/rtt_ros_integration/issues/124>`_ from orocos/rtt_std_srvs-for-standard-operations
  Add wrappers for common operation signatures to rtt_std_srvs (follow-up)
* rtt_roscomm: move public methods of ROSServiceServerOperationCaller<ROS_SERVICE_T> back to the public section
* Merge pull request `#123 <https://github.com/orocos/rtt_ros_integration/issues/123>`_ from orocos/rtt_std_srvs-for-standard-operations
  Add wrappers for common operation signatures to rtt_std_srvs
* fix the build correctly
* fix build for pre-C++11 compatibility
* rtt_roscomm/rtt_std_srvs: add wrappers for common operation signatures to rtt_std_srvs (fix `#101 <https://github.com/orocos/rtt_ros_integration/issues/101>`_)
  With this patch it is possible to create ROS service servers for some common operation signatures out of the box,
  without the need to write custom wrappers or to add extra operations with the ROS specific callback signature.
  Supported service types and associated signatures:
  std_srvs/Empty:
  - bool empty()                     // The service call fails if empty() returns false!
  // Use std_srvs/Trigger if the result should be returned as the response.
  - void empty()
  std_srvs/SetBool:
  - bool setBool(bool, std::string &message_out)
  - bool setBool(bool)               // response.message will be empty
  - std::string setBool(bool)        // response.success = true
  - void setBool(bool)               // response.success = true and response.message will be empty
  std_srvs/Trigger:
  - bool trigger(std::string &message_out)
  - bool trigger()                   // response.message will be empty
  - std::string trigger()            // response.success = true
  The approach can be easily extended to other ROS service types.
* Merge pull request `#111 <https://github.com/orocos/rtt_ros_integration/issues/111>`_ from orocos/fix-110
  Declare loadROSService() methods as static to fix name clashes
* Declare loadROSService() methods as static to fix name clashes (fix `#110 <https://github.com/orocos/rtt_ros_integration/issues/110>`_)
* Merge pull request `#109 <https://github.com/orocos/rtt_ros_integration/issues/109>`_ from orocos/fix/rtt_roscomm-python-interpreter
  rtt_roscomm: fix hard-coded path to python interpreter in shebang of create_boost_header.py
* rtt_roscomm: fix hard-coded path to python interpreter in shebang of create_boost_header.py
* Merge pull request `#106 <https://github.com/orocos/rtt_ros_integration/issues/106>`_ from ahoarau/patch-2 into indigo-devel
  add topicLatched to scripting
* add topicLatched to scripting
* Merge pull request `#106 <https://github.com/orocos/rtt_ros_integration/issues/106>`_ from ahoarau/patch-2
  add topicLatched to scripting
* add topicLatched to scripting
* Merge pull request `#103 <https://github.com/orocos/rtt_ros_integration/issues/103>`_ from orocos/rtt_roscomm-feature-connection-introspection
  Add RTT connection introspection support for ROS publisher and subscriber streams
* rtt_roscomm: fixed retrieval of fully-qualified topic in RubPubChannelElement/RosSubChannelElement
  ROS graph names cannot be simply concatenated, e.g. if the topic name is already fully qualified
  (starts with a /). Another broken case were private publishers or subscribers, where the topic
  passed by the user starts with a tilde (~) and the constructor uses the private node handle.
  Luckily ros::Publisher and ros::Subscriber already provide a getTopic() method which returns the
  fully-qualified topic in all these cases.
* rtt_roscomm: adding introspection ability
* Contributors: Antoine Hoarau, Francisco Almeida, Hamal Marino, Johannes Meyer, Sergio Portoles, Sergio Portoles Diez, SergioPD

2.9.2 (2019-05-15)
------------------
* Merge pull request `#111 <https://github.com/orocos/rtt_ros_integration/issues/111>`_ from orocos/fix-110 into 2.9.2
  * Declare loadROSService() methods as static to fix name clashes (fix `#110 <https://github.com/orocos/rtt_ros_integration/issues/110>`_)
* Merge pull request `#109 <https://github.com/orocos/rtt_ros_integration/issues/109>`_ from orocos/fix/rtt_roscomm-python-interpreter into 2.9.2
  * rtt_roscomm: fix hard-coded path to python interpreter in shebang of create_boost_header.py
* Merge pull request `#106 <https://github.com/orocos/rtt_ros_integration/issues/106>`_ from ahoarau/patch-2 into 2.9.2
  * add topicLatched to scripting
* Contributors: Antoine Hoarau, Johannes Meyer

2.9.1 (2017-11-16)
------------------
* Merge with version 2.8.6

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
