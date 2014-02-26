RTT ROSClock
============

This package contains helpful functions for measuring time according to
different sources on a realtime operating system. 

When running Orocos components in a ROS network, it's important to keep
time synchronized between different machines. If you use the standard
`RTT::TimeService` calls to query time, this might be **tens of seconds**
different from the NTP-corrected time and any timestamped messages published
to ROS will be dramatically delayed.

To avoid these problems while staying realtime-safe (and portable), you can 
use the `rtt_rosclock::host_rt_now()` function to get a ROS time structure
that uses the `CLOCK_HOST_REALTIME` realtime time source on a Xenomai system. 
On a gnu/linux system, this call will just return the result of the standard
`ros::Time::now()` function.

### RTT Services

#### Clock Service

The "rosclock" RTT service also provides a service "time" which provides the
following operations:

 * `ros::Time ros.clock.rtt_now(void)` Get a ROS time structure from the RTT clock source.
 * `ros::Time ros.clock.ros_now(void)` Get a ROS time structure from the ROS clock.
 * `ros::Time ros.clock.host_rt_now(void)` Get a ROS time structure from the NTP-adjusted realtime clock (`CLOCK_HOST_REALTIME`).
 * `RTT::Seconds ros.clock.host_rt_offset_from_rtt(void)` Get the differences from the RTT clock source to the NTP-adjusted realtime clock.

#### C++ API

The calls are also available via the `rtt_rosclock/rtt_rosclock.h` header:

 * `ros::Time rtt_rosclock::rtt_now(void)`
 * `ros::Time rtt_rosclock::ros_now(void)`
 * `ros::Time rtt_rosclock::host_rt_now(void)`
 * `RTT::Seconds rtt_rosclock::host_rt_offset_from_rtt(void)`

