
#include <time.h>
#include <rtt/RTT.hpp>

#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt_rosclock/rtt_rosclock_sim_clock_thread.h>

//! Get the current time according to RTT
const ros::Time rtt_rosclock::rtt_now() 
{
  //return ros::Time(((double)RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks()))*1E-9);
  return ros::Time(RTT::nsecs_to_Seconds(RTT::os::TimeService::Instance()->getNSecs()));
}

//! Get the current time according to ROS
const ros::Time rtt_rosclock::ros_now() 
{
  return ros::Time::now();
}

//! Get the current time according to CLOCK_HOST_REALTIME
const ros::Time rtt_rosclock::host_rt_now() 
{
  if(SimClockThread::GetInstance() && SimClockThread::GetInstance()->simTimeEnabled()) {
    return rtt_now();   
  } else {
    #ifdef __XENO__
    // Use Xenomai 2.6 feature to get the NTP-synched real-time clock
    timespec ts = {0,0};
    int ret = clock_gettime(CLOCK_HOST_REALTIME, &ts);
    if(ret) {
      RTT::log(RTT::Error) << "Could not query CLOCK_HOST_REALTIME (" << CLOCK_HOST_REALTIME <<"): "<< errno << RTT::endlog();
    }

    return ros::Time(ts.tv_sec, ts.tv_nsec);
    #else 
    return ros::Time::now();
    #endif
  }
}

//! Get the difference in seconds between RTT and CLOCK_HOST_REALTIME
const RTT::Seconds rtt_rosclock::host_rt_offset_from_rtt() 
{
  return (rtt_rosclock::host_rt_now() - rtt_rosclock::rtt_now()).toSec();
}

//! Use ROS /clock topic for time measurement
void rtt_rosclock::use_ros_clock_topic()
{
  SimClockThread::Instance()->useROSClockTopic();
}

//! Use manual clock updates
void rtt_rosclock::use_manual_clock()
{
  SimClockThread::Instance()->useManualClock();
}

//! Set a TaskContext to use a periodic simulation clock activity
const bool rtt_rosclock::set_sim_clock_activity(RTT::TaskContext *t)
{
  if (!t) return false;
  return t->setActivity(new SimClockActivity(t->getPeriod()));
}

//! Use a simulated clock source
const bool rtt_rosclock::enable_sim()
{
  return SimClockThread::Instance()->start();
}

//! Do't use a simulated clock source
const bool rtt_rosclock::disable_sim() 
{
  return SimClockThread::Instance()->stop();
}

//! Update the simulated time
void rtt_rosclock::update_sim_clock(const RTT::os::Seconds now)
{
  SimClockThread::Instance()->updateClock(now);
}


