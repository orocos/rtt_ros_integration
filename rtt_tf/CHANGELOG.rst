^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rtt_tf
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge pull request `#151 <https://github.com/orocos/rtt_ros_integration/issues/151>`_ from orocos/feature/rtt_tf2-transform-updates
  rtt_tf: add transform updates on Orocos ports
* rtt_tf: tackle comments of PR `#151 <https://github.com/orocos/rtt_ros_integration/issues/151>`_
* rtt_tf: fix to the actual license of rtt_tf from source file
* add LICENSE files to rtt_ros_integration packages
* rtt_tf: remove C++11 dependent code
  To pass the compilation in current Travis configuration.
* rtt_tf: fix transformation from TF to geometry msgs
  This patch fixes a memory bug with the transfromations from
  geometry messages to TF.
  The tests implemented work properly.
* rtt_tf: track transformations and write result in port
  This patch adds an operation to set a tracking item corresponging
  to a transformation between to TF frames and it publishes the
  corresponding geometry_msgs stamped transformation in a port
  generated for this purpose during the updateHook().
* Merge branch 'toolchain-2.9' into fix-rtt_roscomm-rosservice-namespace
* Merge pull request `#136 <https://github.com/orocos/rtt_ros_integration/issues/136>`_ from Hugal31/fix/rtt_tf_static
  Fix RTT_TF static stream initialization
* Correctly configure rtt_tf tf_static streams
* Merge pull request `#121 <https://github.com/orocos/rtt_ros_integration/issues/121>`_ from orocos/rtt-migrate-tf2
  Migrate rtt_tf to tf2
* Merge tag '2.9.2' into toolchain-2.9
* rtt_tf: add output port for static transform and broadcastStaticTransform(s) operations
* rtt_tf: reduce code in RTT_TF::internalUpdate and differentiate static transform case; remove dead code
* rtt_tf: handle configureHook() failure
* rtt_tf: add canTransformAtTime operation, remove code duplication and other minor cleanups
* rtt_tf: add dependencies on tf2, rtt_tf2_msgs to rtt_tf
* remove unnecessary import statements from README. The rtt_ros plugin loads a component that allows importing ROS packages following the dependency tree.
* Update README by referring to tf2.
* update rtt_tf tests to load any useful plugins automatically
* InputPort port_tf_static_in is connected to /tf_static topic
* refactor component to be implemented based on tf2::BufferCore
* Contributors: Francisco Almeida, Hugo Laloge, Johannes Meyer, Sergio Portoles, Sergio Portoles Diez, SergioPD

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
