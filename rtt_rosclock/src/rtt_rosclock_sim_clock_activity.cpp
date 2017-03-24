
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

using namespace RTT::base;
using namespace rtt_rosclock;


SimClockActivity::SimClockActivity(RunnableInterface* run, const std::string& name)
: ActivityInterface(run)
  , name_(name)
  , period_(0.0)
  , running_(false)
  , active_(false)
  , manager_(SimClockActivityManager::Instance())
{
  manager_->add(this);
}

SimClockActivity::SimClockActivity(RTT::Seconds period, RunnableInterface* run, const std::string& name)
: ActivityInterface(run)
  , name_(name)
  , period_(period)
  , running_(false)
  , active_(false)
  , manager_(SimClockActivityManager::Instance())
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

bool SimClockActivity::isPeriodic() const
{
  return true;
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

#if defined(RTT_VERSION_GTE)
#if RTT_VERSION_GTE(2,9,0)
void SimClockActivity::work(RunnableInterface::WorkReason)
{
}
#endif
#endif

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

bool SimClockActivity::isActive() const
{
  return active_;
}

bool SimClockActivity::trigger()
{
  return false;
}

bool SimClockActivity::timeout()
{
  return false;
}

bool SimClockActivity::execute()
{
  if (!running_) return false;
  if (runner) {
      runner->step();
#if defined(RTT_VERSION_GTE)
#if RTT_VERSION_GTE(2,9,0)
      runner->work(RunnableInterface::TimeOut);
#endif
#endif
  } else {
      this->step();
#if defined(RTT_VERSION_GTE)
#if RTT_VERSION_GTE(2,9,0)
      this->work(RunnableInterface::TimeOut);
#endif
#endif
  }
  last_ = RTT::os::TimeService::Instance()->getTicks();
  return true;
}

RTT::os::TimeService::ticks SimClockActivity::getLastExecutionTicks() const
{
  return last_;
}
