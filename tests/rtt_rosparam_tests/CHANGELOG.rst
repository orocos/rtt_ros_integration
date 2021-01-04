^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rtt_rosparam_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.10.0 (2021-01-04)
-------------------
* Merge remote-tracking branch 'origin/toolchain-2.9' into toolchain-2.10
* Merge pull request `#157 <https://github.com/orocos/rtt_ros_integration/issues/157>`_ from orocos/feature/rosparam-unitests
  rtt_rosparam_tests: add unit tests for the new DataSource feature
* rtt_rosparam_tests: tackle comments PR`#157 <https://github.com/orocos/rtt_ros_integration/issues/157>`_ nit
* rtt_rosparam_tests: remove direct access parameters from test
  This patch removes the dependency on the features introduced in
  PR `#147 <https://github.com/orocos/rtt_ros_integration/issues/147>`_ on the unit tests for rtt_rosparam, since the feature
  was closed.
  The unit tests are then focused on the new DataSource.
* rtt_rosparam_tests: add tests for the new two features
  The features referred in this patch are the direcct access
  to ros parameters and the new DataSource.
* Merge pull request `#154 <https://github.com/orocos/rtt_ros_integration/issues/154>`_ from orocos/feature/add-licenses
  add LICENSE files to rtt_ros_integration packages
* update e-mail of Orocos Developers in package.xml files
* homogenize licenses to BSD
  The patch reorganizes all the licesnes to match to BSD and a
  single LICENSE file is placed in the root of the repository.
* add LICENSE files to rtt_ros_integration packages
* Merge pull request `#149 <https://github.com/orocos/rtt_ros_integration/issues/149>`_ from orocos/feature/rosparam-tests-optional-eigen
  rtt_rosparam_tests: make Eigen optional according to rosparam.h
* rtt_rosparam_tests: make Eigen optional according to rosparam.h
  This patch watches over the assumption that the eigen support is
  always enabled, to make it depend on the definition:
  RTT_ROSPARAM_EIGEN_SUPPORT
* Merge tag '2.9.2' into toolchain-2.9
* Contributors: Johannes Meyer, Sergio Portoles Diez, SergioPD

2.9.2 (2019-05-15)
------------------

2.9.1 (2017-11-16)
------------------

2.9.0 (2017-05-02)
------------------
* rtt_rosparam_tests: fixed compilation with C++11
* rtt_rosparam_tests: added new test package for rtt_rosparam
* Contributors: Johannes Meyer
