^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rtt_rosparam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.10.0 (2021-01-04)
-------------------
* Merge remote-tracking branch 'origin/toolchain-2.9' into toolchain-2.10
* Merge pull request `#157 <https://github.com/orocos/rtt_ros_integration/issues/157>`_ from orocos/feature/rosparam-unitests
  rtt_rosparam_tests: add unit tests for the new DataSource feature
* rtt_rosparam: remove Operations to add eigen related DS
  This patch removes the request to add addRosParamProperty<Type>
  for Eigen:: types, which are not supported as ROS parameters.
* Merge pull request `#154 <https://github.com/orocos/rtt_ros_integration/issues/154>`_ from orocos/feature/add-licenses
  add LICENSE files to rtt_ros_integration packages
* Merge pull request `#148 <https://github.com/orocos/rtt_ros_integration/issues/148>`_ from orocos/feature/rosparam-add-datasource
  rtt_rosparam: add a new DataSource linked to ROS parameter
* rtt_rosparam: ammend PR comments clean includes
* update e-mail of Orocos Developers in package.xml files
* rtt_rosparam: implement updated() and other PR comments
  The patch moves the part of the set( new_value) implementation
  to updated() such that when using ref_t set() the user can call
  updated() and make the ROS parameter actually be updated with
  the new value.
* rtt_rosparam: remove tag data from header
* rtt_rosparam: fix license into BSD
  This patch modifies the license of the new feature to match
  the BSD-3clause of the project.
* homogenize licenses to BSD
  The patch reorganizes all the licesnes to match to BSD and a
  single LICENSE file is placed in the root of the repository.
* rtt_rosparam: tackle comments of PR `#148 <https://github.com/orocos/rtt_ros_integration/issues/148>`_
  This patch applies changes according to the comments of the PR `#148 <https://github.com/orocos/rtt_ros_integration/issues/148>`_.
* add LICENSE files to rtt_ros_integration packages
* rtt_rosparam: add documentation related to new DataSource
* rtt_rosparam: add OperationCallers to the interface
* rtt_rosparam: add support for ResolutionPolicy and referred types
* rtt_rosparam: add a new RTT::DataSource that links to ROS parameters
  The new RTT::DataSource keeps a reference name to a ROS parameter
  so that .get() reads in the parameter and .set() sets out the
  parameter value assigned to the data source.
  The ROSParamService provides a new operation to add properties
  based on ROS parameters. (for now only for Boolean types.)
* Merge tag '2.9.2' into toolchain-2.9
* Contributors: Johannes Meyer, Sergio Portoles, Sergio Portoles Diez, SergioPD

2.9.2 (2019-05-15)
------------------

2.9.1 (2017-11-16)
------------------

2.9.0 (2017-05-02)
------------------
* rtt_rosparam: removed cmake_modules dependency and fixed export of eigen3 dependency
* rtt_rosparam: fixed retrieval of Eigen::VectorXf properties from the parameter server
* rtt_rosparam: added missing operations and operation callers
* rtt_rosparam: moved ResolutionPolicy enum to the service requester header
* rtt_rosparam: enabled Eigen-based operation callers in rosparam.h and added build_export_depend
* sync service with new functions
* add vectorXd/Xf methods
* add Component private/relative/absolute methods
* add template instead of macros
* add missing comma
* only ops supported types
* add ros param getter and setters operations
* add Component private/Relative resolution
* Added individual changelogs and bumped versions to 2.9.0
* cast enum to int to avoid warning
* Contributors: Antoine Hoarau, Johannes Meyer

2.8.6 (2017-11-15)
------------------

2.8.5 (2017-03-28)
------------------

2.8.4 (2016-11-26)
------------------

2.8.3 (2016-07-20)
------------------
* cast enum to int to avoid warning
* Contributors: Antoine Hoarau

2.8.2 (2015-06-12)
------------------
* see `rtt_ros_integratoin/CHANGELOG.rst <../rtt_ros_integration/CHANGELOG.rst>`_
