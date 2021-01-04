^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rtt_roscomm_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.10.0 (2021-01-04)
-------------------
* Merge remote-tracking branch 'origin/toolchain-2.9' into toolchain-2.10
* Merge pull request `#154 <https://github.com/orocos/rtt_ros_integration/issues/154>`_ from orocos/feature/add-licenses
  add LICENSE files to rtt_ros_integration packages
* update e-mail of Orocos Developers in package.xml files
* Merge remote-tracking branch 'origin/toolchain-2.9' into toolchain-2.10
* Merge pull request `#142 <https://github.com/orocos/rtt_ros_integration/issues/142>`_ from orocos/test-callback-signature-in-rtt_roscomm_tests-transport_tests
  rtt_roscomm_tests: check exact callback signature in transport_tests.cpp
* rtt_roscomm_tests: check exact callback signature in transport_tests.cpp
* Merge tag '2.9.2' into toolchain-2.9
* Merge pull request `#123 <https://github.com/orocos/rtt_ros_integration/issues/123>`_ from orocos/rtt_std_srvs-for-standard-operations
  Add wrappers for common operation signatures to rtt_std_srvs
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
* Contributors: Francisco Almeida, Johannes Meyer, Sergio Portoles Diez, SergioPD

2.9.2 (2019-05-15)
------------------

2.9.1 (2017-11-16)
------------------

2.9.0 (2017-05-02)
------------------
* rtt_std_msgs: added a VectorMultiArrayAdapter class and added type transporter for arrays (std_msgs/Float64MultiArray)
* rtt_std_msgs: added a transport plugin for ROS primitive types
* rtt_roscomm_tests: fixed create_rtt_msgs test
* rtt_roscomm: renamed header rtt_rostopic.h to rostopic.h and changed namespace for the ROSService service requester for consistency
* rtt_roscomm_tests: added a test for the rosservice plugin
* rtt_roscomm_tests: check advertised/subscribed topics and publisher/subscriber destruction
* Added individual changelogs and bumped versions to 2.9.0
* tests: split off out-of-band ROS transport test using rostest and integrated create_rtt_msgs test with catkin
* tests: add USE_OROCOS_INCLUDE_DIRS explicitly and remove orocos_generate_package() calls for tests
* rtt_roscomm_tests: added an out-of-band test using the ROS transport
* Contributors: Johannes Meyer

2.8.6 (2017-11-15)
------------------

2.8.5 (2017-03-28)
------------------
* Merge pull request `#85 <https://github.com/orocos/rtt_ros_integration/issues/85>`_ from meyerj/ros-primitives-transport-indigo-devel
  Added a ROS transport plugin for primitive types (indigo-devel)
* rtt_roscomm_tests: added test dependency rosbash required for the rosrun command
* Merge pull request `#57 <https://github.com/orocos/rtt_ros_integration/issues/57>`_ from meyerj/improved-tests
  Refactored test packages and improved rtt_roscomm_tests
* Contributors: Johannes Meyer

2.8.4 (2016-11-26)
------------------

2.8.3 (2016-07-20)
------------------
* tests: removed all run dependencies or replaced them by test dependencies
* Contributors: Johannes Meyer

2.8.2 (2015-06-12)
------------------
* see `rtt_ros_integratoin/CHANGELOG.rst <../rtt_ros_integration/CHANGELOG.rst>`_
