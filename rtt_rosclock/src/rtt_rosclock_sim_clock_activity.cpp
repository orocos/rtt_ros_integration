
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

#include <rtt/base/RunnableInterface.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Logger.hpp>

#include <boost/weak_ptr.hpp>

using namespace RTT::base;
using namespace rtt_rosclock;

boost::weak_ptr<SimClockActivityManager> SimClockActivityManager::sinstance;

boost::shared_ptr<SimClockActivityManager> SimClockActivityManager::GetInstance()
{
  return sinstance.lock();
}

boost::shared_ptr<SimClockActivityManager> SimClockActivityManager::Instance()
{
  // Create a new instance, if necessary
  boost::shared_ptr<SimClockActivityManager> shared = GetInstance();
  if(sinstance.expired()) {
    shared.reset(new SimClockActivityManager());
    sinstance = shared;
  }

  return shared;
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
  RTT::os::MutexLock lock(mutex_);
  RTT::os::TimeService::ticks now = RTT::os::TimeService::Instance()->getTicks();

  for(std::list<SimClockActivity *>::const_iterator it = activities_.begin(); it != activities_.end(); it++)
  {
    SimClockActivity *activity = *it;
    if (RTT::os::TimeService::ticks2nsecs(now - activity->getLastExecutionTicks()) * 1e-9 >= activity->getPeriod())
    {
      activity->execute();
    }
  }
}

void SimClockActivityManager::add(SimClockActivity *activity)
{
  RTT::os::MutexLock lock(mutex_);
  std::list<SimClockActivity *>::iterator it = std::find(activities_.begin(), activities_.end(), activity);
  if (it == activities_.end()) {
    activities_.push_back(activity);
  }
}

void SimClockActivityManager::remove(SimClockActivity *activity)
{
  RTT::os::MutexLock lock(mutex_);
  std::list<SimClockActivity *>::iterator it = std::find(activities_.begin(), activities_.end(), activity);
  if (it != activities_.end()) {
    activities_.erase(it);
  }
}

SimClockActivity::SimClockActivity(RunnableInterface* run, const std::string& name)
: ActivityInterface(run), name_(name), running_(false), active_(false), manager_(SimClockActivityManager::Instance())
{
  manager_->add(this);
}

SimClockActivity::SimClockActivity(RTT::Seconds period, RunnableInterface* run, const std::string& name)
: ActivityInterface(run), name_(name), period_(period), running_(false), active_(false), manager_(SimClockActivityManager::Instance())
{
  manager_->add(this);
}

SimClockActivity::~SimClockActivity()
{
  stop();
  manager_->remove(this);
}

RTT::Seconds SimClockActivity::getPeriod() const
{
  if (period_ > 0.0)
    return period_;
  else
    return manager_->getSimulationPeriod();
}

bool SimClockActivity::setPeriod(RTT::Seconds s)
{
  period_ = s;
  return true;
}

unsigned SimClockActivity::getCpuAffinity() const
{
  return ~0;
}

bool SimClockActivity::setCpuAffinity(unsigned cpu)
{
  return false;
}

RTT::os::ThreadInterface* SimClockActivity::thread()
{
  return 0;
}

bool SimClockActivity::initialize()
{
  return true;
}

void SimClockActivity::step()
{
}

void SimClockActivity::loop()
{
  this->step();
}

bool SimClockActivity::breakLoop()
{
  return false;
}


void SimClockActivity::finalize()
{
}

bool SimClockActivity::start()
{
  if ( active_ == true )
  {
    RTT::log(RTT::Error) << "Unable to start slave as it is already started" << RTT::endlog();
    return false;
  }

  active_ = true;
  last_ = 0;

  if ( runner ? runner->initialize() : this->initialize() ) {
    running_ = true;
  } else {
    active_ = false;
  }

  return active_;
}

bool SimClockActivity::stop()
{
  if ( !active_ )
    return false;

  running_ = false;
  if (runner)
    runner->finalize();
  else
    this->finalize();
  active_ = false;
  return true;
}

bool SimClockActivity::isRunning() const
{
  return running_;
}

bool SimClockActivity::isPeriodic() const
{
  return true;
}

bool SimClockActivity::isActive() const
{
  return active_;
}

bool SimClockActivity::trigger()
{
  return false;
}

bool SimClockActivity::execute()
{
  if (!running_) return false;
  if (runner) runner->step(); else this->step();
  last_ = RTT::os::TimeService::Instance()->getTicks();
  return true;
}

RTT::os::TimeService::ticks SimClockActivity::getLastExecutionTicks() const
{
  return last_;
}
