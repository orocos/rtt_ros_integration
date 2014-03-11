
#include <time.h>
#include <rtt/RTT.hpp>

#include <rtt_rosclock/rtt_rosclock.h>

//! Get the current time according to RTT
const ros::Time rtt_rosclock::rtt_now() {
  //return ros::Time(((double)RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks()))*1E-9);
  return ros::Time(RTT::nsecs_to_Seconds(RTT::os::TimeService::Instance()->getNSecs()));
}

//! Get the current time according to ROS
const ros::Time rtt_rosclock::ros_now() {
  return ros::Time::now();
}

//! Get the current time according to CLOCK_HOST_REALTIME
const ros::Time rtt_rosclock::host_rt_now() {
#ifdef __XENO__
  // Use Xenomai 2.6 feature to get the NTP-synched real-time clock
  timespec ts = {0,0};
  int ret = clock_gettime(CLOCK_HOST_REALTIME, &ts);
  if(ret) {
    RTT::log(RTT::Error) << "Could not query CLOCK_HOST_REALTIME (" << CLOCK_HOST_REALTIME <<"): "<< errno << RTT::endlog();
  }

  return ros::Time(ts.tv_sec, ts.tv_nsec);
#else 
  ros::WallTime now(ros::WallTime::now());
  return ros::Time(now.sec, now.nsec);
#endif
}

//! Get the difference in seconds between RTT and CLOCK_HOST_REALTIME
const RTT::Seconds rtt_rosclock::host_rt_offset_from_rtt() {
  return (rtt_rosclock::host_rt_now() - rtt_rosclock::rtt_now()).toSec();
}
