^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rtt_roscomm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Also add a virtual destructor to the base class of the ROS Service Proxy
* Added an explicit destructor to shutdown services servers, and cleanup the registered proxies
* added topicLatched() method to rtt_rostopic service
* Corba / Mqueue support for ROS message typekits
* added support for updated dataflow semantics (RTT version >= 2.8.99)
* only set CMAKE_BUILD_TYPE to MinSizeRel if either not set or if it was Release before
* Contributors: Antoine Hoarau, Guillaume Walck, Johannes Meyer

2.8.3 (2016-07-20)
------------------
* rtt_roscomm: set minimum ROS subscriber queue_size to 1
* rtt_roscomm: fixed destruction of RosSubChannelElement<T> and ROS subscriber shutdown (fix `#61 <https://github.com/orocos/rtt_ros_integration/issues/61>`_)
* Contributors: Johannes Meyer

2.8.2 (2015-06-12)
------------------
* see `rtt_ros_integratoin/CHANGELOG.rst <../rtt_ros_integration/CHANGELOG.rst>`_
