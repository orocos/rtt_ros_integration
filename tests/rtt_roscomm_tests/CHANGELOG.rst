^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rtt_roscomm_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
