/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Johannes Meyer, TU Darmstadt
 *  Copyright (c) 2013, Intermodalics BVBA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of TU Darmstadt and Intermodalics BVBA
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Copyright (c) 2014, Jonathan Bohren, The Johns Hopkins University
 *  - Generalized for multiple time sources
 *  - Integrated with rtt_rosclock package
 *********************************************************************/

#include <rtt_rosclock/rtt_rosclock_sim_clock_thread.h>
#include <rtt_rosclock/rtt_rosclock_sim_clock_activity.h>
#include <rtt_rosclock/rtt_rosclock_sim_clock_activity_manager.h>

#include <rtt/TaskContext.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <rtt/plugin/Plugin.hpp>

#include <ros/time.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/subscribe_options.h>

#include <rtt_rosclock/rtt_rosclock.h>

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

SimClockThread::SimClockThread() 
  : RTT::os::Thread(ORO_SCHED_OTHER, RTT::os::LowestPriority, 0.0, 0, "rtt_rosclock_SimClockThread")
  , time_service_(RTT::os::TimeService::Instance())
  , clock_source_(SIM_CLOCK_SOURCE_MANUAL)
  , process_callbacks_(false)
{
}

SimClockThread::~SimClockThread()
{
  this->stop();
}

bool SimClockThread::setClockSource(SimClockSource clock_source) 
{
  // Don't allow changing the source while running
  if(this->isActive()) {
    RTT::log(RTT::Error) << "The SimClockThread clock source cannot be changed while the thread is running." << RTT::endlog();
    return false;
  }

  // Set the clock source 
  clock_source_ = clock_source;

  return true;
}

bool SimClockThread::useROSClockTopic() 
{
  return this->setClockSource(SIM_CLOCK_SOURCE_ROS_CLOCK_TOPIC);
}

bool SimClockThread::useManualClock() 
{
  return this->setClockSource(SIM_CLOCK_SOURCE_MANUAL);
}

bool SimClockThread::simTimeEnabled() const 
{ 
  return this->isActive();
}

void SimClockThread::clockMsgCallback(const rosgraph_msgs::ClockConstPtr& clock)
{
  // Update the RTT clock
  updateClockInternal(ros::Time(clock->clock.sec, clock->clock.nsec));
}

bool SimClockThread::updateClock(const ros::Time new_time)
{
  if(clock_source_ != SIM_CLOCK_SOURCE_MANUAL) {
    RTT::log(RTT::Error) << "Cannot update simulation clock manually unless the clock source is set to MANUAL_CLOCK." << RTT::endlog();
    return false;
  }

  return this->updateClockInternal(new_time);
}

bool SimClockThread::updateClockInternal(const ros::Time new_time)
{
  // Make sure the system time isn't being used
  if(time_service_->systemClockEnabled()) {
    time_service_->enableSystemClock(false);
  }

  // Check if time restarted
  if(new_time.isZero()) {
    
    RTT::log(RTT::Warning) << "Time has reset to 0! Re-setting time service..." << RTT::endlog();

    // Re-set the time service and don't update the activities
    this->resetTimeService();

  } else {
    // Update the RTT time to match the sim time
    using namespace RTT::os;
    //TimeService::ticks rtt_ticks = time_service_->getTicks();
    //TimeService::Seconds rtt_secs = RTT::nsecs_to_Seconds(TimeService::ticks2nsecs(rtt_ticks));

    // Compute the time update
    TimeService::Seconds dt = (new_time - rtt_rosclock::rtt_now()).toSec();

    // Check if time went backwards
    if(dt < 0) {
      RTT::log(RTT::Warning) << "Time went backwards by " << dt << " seconds! (" << rtt_rosclock::rtt_now() << " --> " << new_time <<")" << RTT::endlog();
    }

    // Update the RTT clock
    time_service_->secondsChange(dt);

    // Trigger all SimClockActivities
    boost::shared_ptr<SimClockActivityManager> manager = SimClockActivityManager::GetInstance();
    if (manager) {
      // Update the simulation period
      manager->setSimulationPeriod(dt);
      // Update all the SimClockActivities
      manager->update();
    }
  }

  return true;
}

void SimClockThread::resetTimeService()
{
  // We have to set the Logger reference time to zero in order to get correct logging timestamps.
  // RTT::Logger::Instance()->setReferenceTime(0);
  //
  // Unfortunately this method is not available, therefore shutdown and restart logging.
  // This workaround is not exact.

  // Shutdown the RTT Logger
  RTT::Logger::Instance()->shutdown();

  // Disable the RTT system clock so Gazebo can manipulate time and reset it to 0
  time_service_->enableSystemClock(false);
  assert(time_service_->systemClockEnabled() == false);

  time_service_->ticksChange(-time_service_->ticksSince(0));
  assert(time_service_->getTicks() == 0);

  // Restart the RTT Logger with reference time 0
  RTT::Logger::Instance()->startup();
  assert(RTT::Logger::Instance()->getReferenceTime() == 0);
}

bool SimClockThread::initialize()
{

  RTT::log(RTT::Debug) << "[rtt_rosclock] Attempting to enable global simulation clock source..." << RTT::endlog();

  switch(clock_source_) 
  {
    case SIM_CLOCK_SOURCE_ROS_CLOCK_TOPIC:
      {
        // Get /use_sim_time parameter from ROS
        bool use_sim_time = false;
        ros::param::get("/use_sim_time", use_sim_time);

        if(!use_sim_time) {
          RTT::log(RTT::Error) << "[rtt_rosclock] Did not enable ROS simulation clock because the ROS parameter '/use_sim_time' is not set to true." << RTT::endlog();
          process_callbacks_ = false;
          return false;
        }

        RTT::log(RTT::Debug) << "[rtt_rosclock] Switching to simulated time based on ROS /clock topic..." << RTT::endlog();

        // Reset the timeservice and logger
        this->resetTimeService();

        // Subscribe the /clock topic (simulation time, e.g. published by Gazebo)
        ros::SubscribeOptions ops = ros::SubscribeOptions::create<rosgraph_msgs::Clock>(
            "/clock", 1, boost::bind(&SimClockThread::clockMsgCallback, this, _1),
            ros::VoidConstPtr(), &callback_queue_);
        clock_subscriber_ = nh_.subscribe(ops);

        // The loop needs to run in order to call the callback queue
        process_callbacks_ = true;
      }
      break;

    case SIM_CLOCK_SOURCE_MANUAL:
      {
        RTT::log(RTT::Debug) << "[rtt_rosclock] Switching to simulated time based on a manual clock source..." << RTT::endlog();

        // Reset the timeservice and logger
        this->resetTimeService();

        // We're not processing the callback queue, so we won't loop.
        process_callbacks_ = false;
      }
      break;

    default:
      {
        RTT::log(RTT::Error) << "Unknown simulation clock source for SimClockThread!" << RTT::endlog();
        return false;
      }
  };

  return true;
}

void SimClockThread::loop()
{
  static const ros::WallDuration timeout(0.1);

  // Service callbacks while 
  while(process_callbacks_) {
    callback_queue_.callAvailable(timeout);
  }
}

bool SimClockThread::breakLoop()
{
  process_callbacks_ = false;
  return true;
}

void SimClockThread::finalize()
{
  RTT::log(RTT::Info) << "Disabling simulated time..." << RTT::endlog();

  // Shutdown the subscriber so no more clock message events will be handled
  clock_subscriber_.shutdown();

  // Shutdown the RTT Logger
  RTT::Logger::Instance()->shutdown();

  // Re-enable system clock
  time_service_->enableSystemClock(true);
  
  // Restart the RTT Logger with reference walltime
  RTT::Logger::Instance()->startup();
}

