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

#include <rtt_rosclock/rtt_rosclock_sim_clock_activity.h>
#include <rtt_rosclock/rtt_rosclock_sim_clock_activity_manager.h>

#include <rtt/base/RunnableInterface.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Logger.hpp>

#include <boost/weak_ptr.hpp>

using namespace rtt_rosclock;

boost::weak_ptr<SimClockActivityManager> SimClockActivityManager::singleton;

boost::shared_ptr<SimClockActivityManager> SimClockActivityManager::GetInstance()
{
  return singleton.lock();
}

boost::shared_ptr<SimClockActivityManager> SimClockActivityManager::Instance()
{
  // Create a new instance, if necessary
  boost::shared_ptr<SimClockActivityManager> shared = GetInstance();
  if(singleton.expired()) {
    shared.reset(new SimClockActivityManager());
    singleton = shared;
  }

  return shared;
}

SimClockActivityManager::SimClockActivityManager() 
  : simulation_period_(0.0) 
{ 
}

SimClockActivityManager::~SimClockActivityManager()
{
}

RTT::Seconds SimClockActivityManager::getSimulationPeriod() const
{
  return simulation_period_;
}

void SimClockActivityManager::setSimulationPeriod(RTT::Seconds s)
{
  simulation_period_ = s;
}

void SimClockActivityManager::update()
{
  RTT::os::MutexLock lock(modify_activities_mutex_);
  RTT::os::TimeService::ticks now = RTT::os::TimeService::Instance()->getTicks();

  // Iterate through all activities
  for(std::list<SimClockActivity *>::const_iterator it = activities_.begin(); it != activities_.end(); it++)
  {
    // Update this activitiy at its desired minimum period
    SimClockActivity *activity = *it;
    if (RTT::os::TimeService::ticks2nsecs(now - activity->getLastExecutionTicks()) * 1e-9 >= activity->getPeriod())
    {
      activity->execute();
    }
  }
}

void SimClockActivityManager::add(SimClockActivity *activity)
{
  RTT::os::MutexLock lock(modify_activities_mutex_);
  std::list<SimClockActivity *>::iterator it = std::find(activities_.begin(), activities_.end(), activity);
  if (it == activities_.end()) {
    activities_.push_back(activity);
  }
}

void SimClockActivityManager::remove(SimClockActivity *activity)
{
  RTT::os::MutexLock lock(modify_activities_mutex_);
  std::list<SimClockActivity *>::iterator it = std::find(activities_.begin(), activities_.end(), activity);
  if (it != activities_.end()) {
    activities_.erase(it);
  }
}
