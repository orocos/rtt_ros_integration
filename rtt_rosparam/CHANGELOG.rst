^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rtt_rosparam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
