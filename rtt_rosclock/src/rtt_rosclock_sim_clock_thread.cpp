
#include <rtt_rosclock/rtt_Rosclock_sim_clock_activity.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <rtt/plugin/Plugin.hpp>

#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/subscribe_options.h>

using namespace rtt_rosclock;

boost::weak_ptr<SimClockThread> SimClockThread::singleton;

boost::shared_ptr<SimClockThread> SimClockThread::GetInstance()
{
  return singleton.lock();
}

boost::shared_ptr<SimClockThread> SimClockThread::Instance()
{
  // Create a new singleton, if necessary
  boost::shared_ptr<SimClockThread> shared = GetInstance();
  if(singleton.expired()) {
    shared.reset(new SimClockThread());
    singleton = shared;
  }

  return shared;
}


SimClockThread::SimClockThread(const std::string& name, RTT::TaskContext* owner) 
  : RTT::Service(name, owner)
  , RTT::os::Thread(ORO_SCHED_OTHER, RTT::os::LowestPriority, 0.0, 0, name)
  , time_service_(RTT::os::TimeService::Instance())
{
}

bool simTimeEnabled()
{
  return use_sim_time_;
}

void SimClockThread::clockMsgCallback(const rosgraph_msgs::ClockConstPtr& clock)
{
  // Get the simulation time
  using namespace RTT::os;
  TimeService::Seconds clock_secs =
    (TimeService::Seconds)clock->clock.sec +
    ((TimeService::Seconds)clock->clock.nsec)*1E-9;

  // Update the RTT clock
  updateClock(clock_secs);
}

void SimClockThread::updateClock(RTT::os::TimeService::Seconds clock_secs)
{
  // Update the RTT time to match the gazebo time
  using namespace RTT::os;
  TimeService::ticks rtt_ticks = time_service_->getTicks();
  TimeService::Seconds rtt_secs = RTT::nsecs_to_Seconds(TimeService::ticks2nsecs(rtt_ticks));

  // Compute the time update
  TimeService::Seconds dt = clock_secs - rtt_secs;

  // Check if time went backwards
  if(dt < 0) {
    RTT::log(RTT::Warning) << "Time went backwards by " << dt << " seconds!" << RTT::endlog();
  }

  // Update the RTT clock
  time_service_->secondsChange(dt);

  // trigger all SimClockActivities
  boost::shared_ptr<SimClockActivityManager> manager = SimClockActivityManager::GetInstance();
  if (manager) {
    manager->setSimulationPeriod(dt);
    manager->update();
  }

  // set sim time flag
  use_sim_time_ = true;
}

bool SimClockThread::initialize()
{
  // Get /use_sim_time parameter from ROS
  ros::param::get("/use_sim_time", use_sim_time);

  if(!use_sim_time_) {
    RTT::log(RTT::Info) << "Did not enable ROS simulation clock because the ROS parameter '/use_sim_time' is not set to true." << RTT::endlog();
    break_loop_ = true;
    return false;
  }

  RTT::log(RTT::Info) << "Switching to simulated time based on ROS /clock topic..." << RTT::endlog();

  // We have to set the Logger reference time to zero in order to get correct logging timestamps.
  // RTT::Logger::Instance()->setReferenceTime(0);
  //
  // Unfortunately this method is not available, therefore shutdown and restart logging.
  // This workaround is not exact.

  // Shutdown the RTT Logger
  RTT::Logger::Instance()->shutdown();

  // Disable the RTT system clock so Gazebo can manipulate time and reset it to 0
  time_service->enableSystemClock(false);
  time_service->secondsChange(-time_service_->secondsSince(0));
  // assert(time_service_->getTicks() == 0);

  // Restart the RTT Logger with reference time 0
  RTT::Logger::Instance()->startup();
  // assert(RTT::Logger::Instance()->getReferenceTime() == 0)

  // Subscribe the /clock topic (simulation time, e.g. published by Gazebo)
  ros::SubscribeOptions ops = ros::SubscribeOptions::create<rosgraph_msgs::Clock>(
      "/clock", 1, boost::bind(&SimClockThread::clockMsgCallback, this, _1),
      ros::VoidConstPtr(), &callback_queue_);
  clock_subscriber_ = nh_.subscribe(ops);

  break_loop_ = false;

  return true;
}

void SimClockThread::loop()
{
  static const ros::WallDuration timeout(0.1);

  while(!break_loop_) {
    callback_queue_.callAvailable(timeout);
  }
}

bool SimClockThread::breakLoop()
{
  break_loop_ = true;
  return true;
}

void SimClockThread::finalize()
{
  RTT::log(RTT::Info) << "Disablign simulated time..." << RTT::endlog();

  // Shutdown the subscriber so no more clock message events will be handled
  clock_subscriber_.shutdown();

  // Unset flag
  use_sim_time_ = false;

  // Shutdown the RTT Logger
  RTT::Logger::Instance()->shutdown();

  // Re-enable system clock
  time_service->enableSystemClock(true);
  
  // Restart the RTT Logger with reference walltime
  RTT::Logger::Instance()->startup();
}

