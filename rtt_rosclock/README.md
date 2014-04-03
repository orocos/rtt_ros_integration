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
use the `rtt_rosclock::host_now()` function to get a ROS time structure
that uses the `CLOCK_HOST_REALTIME` realtime time source on a Xenomai system. 
On a gnu/linux system, this call will just return the result of the standard
`ros::Time::now()` function.

### Use host_now() for broadcasting ROS Header Stamps

The `rtt_rosclock::host_now()` function is the time source that should always be used
with ROS header timestamps because it is the time that you want to use to
broadcast ROS messages to other machines or processes. It is assumed that
if the `/clock` topic is active, then any ROS messages broadcasted are
based on that time source.

When compiled against Xenomai and not running in simulation mode, this function
will return the NTP-synchronized clock time via the `CLOCK_HOST_REALTIME` clock
source. *Note that this is only supported under Xenomai 2.6 and above.*

When **not** compiled against Xenomai and not running in simulation mode, it is
a pass-through to `ros::Time::now()`.

When running in simulation mode, this will always use the simulation clock,
which is based off of the ROS `/clock` topic. It is a pass-through to
`rtt_rosclock::rtt_now()`.

### RTT Services

#### Clock Service

The "rosclock" RTT service also provides a sub-service of the global "ros"
service called "ros.clock" which provides the following operations:

 * `ros::Time ros.clock.host_now(void)` Get a ROS time structure from the NTP-adjusted realtime clock or the sim clock.
 * `ros::Time ros.clock.host_wall_now(void)` Get a ROS time structure from the NTP-adjusted realtime clock or the wall clock.
 * `ros::Time ros.clock.rtt_now(void)` Get a ROS time structure from the RTT clock or the sim clock.
 * `ros::Time ros.clock.rtt_wall_now(void)` Get a ROS time structure from the RTT wall clock.
 * `RTT::Seconds ros.clock.host_offset_from_rtt(void)` Get the differences from the RTT wall clock source to the NTP-adjusted realtime clock source.

#### C++ API

The calls are also available via the `rtt_rosclock/rtt_rosclock.h` header:

 * `ros::Time rtt_rosclock::host_now(void)`
 * `ros::Time rtt_rosclock::host_wall_now(void)`
 * `ros::Time rtt_rosclock::rtt_now(void)`
 * `ros::Time rtt_rosclock::rtt_wall_now(void)`
 * `RTT::Seconds rtt_rosclock::host_offset_from_rtt(void)`

### Simulation Clock Activity

This package also provides an RTT `SimClockActivity` which will periodically execute
a task subject to the progression of a simulated clock source. This clock source can
either be the ROS `/clock` topic if it exists and the `/use_sim_time` parameter is
set to `true`, or it can use a manual time update from some other in-process source.

The simulation clock activity is implemented with three classes:

 * `SimClockActivity` The actual activity which executes a given task.
 * `SimClockActivityManager` A singleton class which is responsible for
   coordinating execution of all `SimClockActivity` periodically (subject to
   their minimum desired periods).
 * `SimClockThread` A singleton thread which is responsible for the acutal
   updates to the `RTT::os::TimeService` and optinally subscribing to ROS
   messages on the `/clock` topic.

By default, `SimClockThread` is not running, but when it is started, it will
override the normal RTT `TimeService` and update it based on the update rate of
whatever `SimClockThread` is using as a clock source.

#### Usage

The `SimClockActivity` is used similarly to the `RTT::PeriodicActivity`. Once
the `rtt_rosclock` service plugin has been loaded, you can set a number of
tasks' activities to SimClockActivities, set the clock source, and then enable
the `SimClockThread`.

```cpp

import("rtt_ros");
ros.import("rtt_rosclock");

// ... create components ...

my_component_1.setPeriod(0.01);
my_component_2.setPeriod(1.0);

// Set simulation clock activities
ros.clock.setSimClockActivity(my_component_1);
ros.clock.setSimClockActivity(my_component_2);

// Set the SimClockThread to use the ROS /clock topic for time
ros.clock.useROSClockTopic();

// Start simulation clock thread and override RTT clock
ros.clock.enableSim();

```
