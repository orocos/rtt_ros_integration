^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rtt_std_srvs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.10.0 (2021-01-04)
-------------------
* Merge remote-tracking branch 'origin/toolchain-2.9' into toolchain-2.10
* Merge pull request `#154 <https://github.com/orocos/rtt_ros_integration/issues/154>`_ from orocos/feature/add-licenses
  add LICENSE files to rtt_ros_integration packages
* update e-mail of Orocos Developers in package.xml files
* homogenize licenses to BSD
  The patch reorganizes all the licesnes to match to BSD and a
  single LICENSE file is placed in the root of the repository.
* add LICENSE files to rtt_ros_integration packages
* Merge pull request `#134 <https://github.com/orocos/rtt_ros_integration/issues/134>`_ from orocos/fix-rtt_roscomm-rosservice-namespace
  rtt_roscomm: add rtt_roscomm namespace for rosservice plugins and helper classes
* rtt_roscomm: add rtt_roscomm namespace for rosservice plugins and helper classes
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
* Added individual changelogs and bumped versions to 2.9.0
* Contributors: Johannes Meyer

2.8.6 (2017-11-15)
------------------

2.8.5 (2017-03-28)
------------------

2.8.4 (2016-11-26)
------------------

2.8.3 (2016-07-20)
------------------

2.8.2 (2015-06-12)
------------------
* see `rtt_ros_integratoin/CHANGELOG.rst <../rtt_ros_integration/CHANGELOG.rst>`_
