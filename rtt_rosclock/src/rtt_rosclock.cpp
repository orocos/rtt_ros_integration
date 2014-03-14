
#include <time.h>
#include <rtt/RTT.hpp>

#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt_rosclock/rtt_rosclock_sim_clock_activity.h>
#include <rtt_rosclock/rtt_rosclock_sim_clock_thread.h>

namespace rtt_rosclock {
  boost::shared_ptr<rtt_rosclock::SimClockThread> sim_clock_thread;
}

const ros::Time rtt_rosclock::rtt_now() 
{
  //return ros::Time(((double)RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks()))*1E-9);
  uint64_t nsec64 = RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks()); //RTT::os::TimeService::Instance()->getNSecs();
  uint32_t sec = nsec64 / 1E9;
  uint32_t nsec = (uint32_t)(nsec64 - (sec*1E9));
  return ros::Time(sec, nsec);
}

const ros::Time rtt_rosclock::ros_now() 
{
  return ros::Time::now();
}

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
    ros::WallTime now(ros::WallTime::now());
    return ros::Time(now.sec, now.nsec);
    #endif
  }
}

const RTT::Seconds rtt_rosclock::host_rt_offset_from_rtt() 
{
  return (rtt_rosclock::host_rt_now() - rtt_rosclock::rtt_now()).toSec();
}

void rtt_rosclock::use_ros_clock_topic()
{
  sim_clock_thread = SimClockThread::Instance();
  SimClockThread::Instance()->useROSClockTopic();
}

void rtt_rosclock::use_manual_clock()
{
  sim_clock_thread = SimClockThread::Instance();
  SimClockThread::Instance()->useManualClock();
}

const bool rtt_rosclock::set_sim_clock_activity(RTT::TaskContext *t)
{
  sim_clock_thread = SimClockThread::Instance();
  if (!t) return false;
  return t->setActivity(new SimClockActivity(t->getPeriod()));
}

const bool rtt_rosclock::enable_sim()
{
  sim_clock_thread = SimClockThread::Instance();
  return SimClockThread::Instance()->start();
}

const bool rtt_rosclock::disable_sim() 
{
  sim_clock_thread = SimClockThread::Instance();
  return SimClockThread::Instance()->stop();
}

void rtt_rosclock::update_sim_clock(const ros::Time new_time)
{
  sim_clock_thread = SimClockThread::Instance();
  SimClockThread::Instance()->updateClock(new_time);
}
