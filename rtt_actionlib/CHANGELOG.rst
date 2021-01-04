^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rtt_actionlib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.10.0 (2021-01-04)
-------------------
* Merge remote-tracking branch 'origin/toolchain-2.9' into toolchain-2.10
* Merge pull request `#154 <https://github.com/orocos/rtt_ros_integration/issues/154>`_ from orocos/feature/add-licenses
  add LICENSE files to rtt_ros_integration packages
* update e-mail of Orocos Developers in package.xml files
* Merge remote-tracking branch 'origin/toolchain-2.9' into toolchain-2.10
* Merge pull request `#137 <https://github.com/orocos/rtt_ros_integration/issues/137>`_ from Hugal31/fix/fix-simple-action-server
  Fix rtt_actionlib simple action server
* Remove option to not call RTTActionServer::initialize in RTTSimpleActionServer::start
  This is because RTTActionServer::start will call RTTActionServer::initialize anyway.
* Add a shutdown method to RTTActionServer to stop its timer
* Merge tag '2.9.2' into toolchain-2.9
* Merge pull request `#100 <https://github.com/orocos/rtt_ros_integration/issues/100>`_ from disRecord/feat/simple-action-server
  Simple action server
* Remove unnecessary namespace and typos fix.
* Fix header (remove "orocos/" from header path).
* Fix typo in isPreempting() method name.
* Remove nullptr for pre C++11 compatibility and fix macro name.
* Fix typos in RTTSimleActionServer example.
* Active goal is not canceled automaticaly anymore.
* Add documentating for RTTSimpleActionServer.
* Add RTTSimpleActionServer.
* Contributors: Hugo Laloge, Johannes Meyer, Oleg Goncharov, Sergio Portoles Diez, SergioPD, disRecord

2.9.2 (2019-05-15)
------------------

2.9.1 (2017-11-16)
------------------

2.9.0 (2017-05-02)
------------------
* Added deprecation warning for header rtt_roscomm/rtt_rostopic.h and updated some include directives within rtt_ros_integration
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
