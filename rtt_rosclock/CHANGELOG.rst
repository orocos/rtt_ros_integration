^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rtt_rosclock
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.10.0 (2021-01-04)
-------------------
* Merge remote-tracking branch 'origin/toolchain-2.9' into toolchain-2.10
* Merge pull request `#154 <https://github.com/orocos/rtt_ros_integration/issues/154>`_ from orocos/feature/add-licenses
  add LICENSE files to rtt_ros_integration packages
* update e-mail of Orocos Developers in package.xml files
* licenses: update text and make it match to current BSD-3
  New update on the files with licenses headers.
* homogenize licenses to BSD
  The patch reorganizes all the licesnes to match to BSD and a
  single LICENSE file is placed in the root of the repository.
* add LICENSE files to rtt_ros_integration packages
* Merge pull request `#138 <https://github.com/orocos/rtt_ros_integration/issues/138>`_ from Hugal31/fix/rtt_rosclock_no_rosmaster
  Do not use ROS connection with rtt_rosclock::SimClockThread if time source is manual
* Do not use ROS connection with rtt_rosclock::SimClockThread if not needed
  rtt_rosclock::SimClockThread now instantiate a ros::NodeHandle only if the time
  source is the /clock topic, and not the manual time source.
* Merge tag '2.9.2' into toolchain-2.9
* Merge pull request `#112 <https://github.com/orocos/rtt_ros_integration/issues/112>`_ from honeybee-robotics-forks/fix-rtt-rosclock-thread-segfault
  rtt_rosclock: fixing isSelf segfault when using simclock with ownthread operation caller
* rtt_rosclock: fixing isSelf segfault when using simclock with ownthread operation caller
* Contributors: Hugo Laloge, Johannes Meyer, Jonathan Bohren, Sergio Portoles Diez, SergioPD

2.9.2 (2019-05-15)
------------------
* Merge pull request `#112 <https://github.com/orocos/rtt_ros_integration/issues/112>`_ from honeybee-robotics-forks/fix-rtt-rosclock-thread-segfault into 2.9.2
  * rtt_rosclock: fixing isSelf segfault when using simclock with ownthread operation caller
* Contributors: Johannes Meyer, Jonathan Bohren

2.9.1 (2017-11-16)
------------------
* Merge pull request `#93 <https://github.com/orocos/rtt_ros_integration/issues/93>`_ from ahoarau/xenomai3-support
  Xenomai 3 support
* Contributors: Antoine Hoarau

2.9.0 (2017-05-02)
------------------
* fix xenomai rtt_now() fix PR`#41 <https://github.com/orocos/rtt_ros_integration/issues/41>`_
* Added individual changelogs and bumped versions to 2.9.0
* rtt_rosclock: adapted SimClockActivity to the updated ActivityInterface with the master-update-hook-vs-callback-queue patch (`orocos-toolchain/rtt#91 <https://github.com/orocos-toolchain/rtt/issues/91>`_)
  Signed-off-by: Johannes Meyer <johannes@intermodalics.eu>
* Contributors: Antoine Hoarau, Johannes Meyer

2.8.6 (2017-11-15)
------------------

2.8.5 (2017-03-28)
------------------

2.8.4 (2016-11-26)
------------------

2.8.3 (2016-07-20)
------------------
* fix xenomai rtt_now() fix PR`#41 <https://github.com/orocos/rtt_ros_integration/issues/41>`_
* Contributors: Antoine Hoarau

2.8.2 (2015-06-12)
------------------
* see `rtt_ros_integratoin/CHANGELOG.rst <../rtt_ros_integration/CHANGELOG.rst>`_
