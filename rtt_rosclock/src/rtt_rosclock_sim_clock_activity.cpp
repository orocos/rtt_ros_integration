
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

#include <rtt_gazebo_activity/gazebo_activity.hpp>

#include <rtt/base/RunnableInterface.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Logger.hpp>

#include <boost/weak_ptr.hpp>

namespace rtt_gazebo_activity {

using namespace RTT::base;

boost::weak_ptr<GazeboActivityManager> GazeboActivityManager::sinstance;

boost::shared_ptr<GazeboActivityManager> GazeboActivityManager::GetInstance()
{
    return sinstance.lock();
}

boost::shared_ptr<GazeboActivityManager> GazeboActivityManager::Instance()
{
    // Create a new instance, if necessary
    boost::shared_ptr<GazeboActivityManager> shared = GetInstance();
    if(sinstance.expired()) {
        shared.reset(new GazeboActivityManager());
        sinstance = shared;
    }

    return shared;
}

GazeboActivityManager::~GazeboActivityManager()
{
}

RTT::Seconds GazeboActivityManager::getSimulationPeriod() const
{
    return simulation_period_;
}

void GazeboActivityManager::setSimulationPeriod(RTT::Seconds s)
{
    simulation_period_ = s;
}

void GazeboActivityManager::update()
{
    RTT::os::MutexLock lock(mutex_);
    RTT::os::TimeService::ticks now = RTT::os::TimeService::Instance()->getTicks();

    for(std::list<GazeboActivity *>::const_iterator it = activities_.begin(); it != activities_.end(); it++)
    {
        GazeboActivity *activity = *it;
        if (RTT::os::TimeService::ticks2nsecs(now - activity->getLastExecutionTicks()) * 1e-9 >= activity->getPeriod())
        {
            activity->execute();
        }
    }
}

void GazeboActivityManager::add(GazeboActivity *activity)
{
    RTT::os::MutexLock lock(mutex_);
    std::list<GazeboActivity *>::iterator it = std::find(activities_.begin(), activities_.end(), activity);
    if (it == activities_.end()) {
        activities_.push_back(activity);
    }
}

void GazeboActivityManager::remove(GazeboActivity *activity)
{
    RTT::os::MutexLock lock(mutex_);
    std::list<GazeboActivity *>::iterator it = std::find(activities_.begin(), activities_.end(), activity);
    if (it != activities_.end()) {
        activities_.erase(it);
    }
}

GazeboActivity::GazeboActivity(RunnableInterface* run, const std::string& name)
    : ActivityInterface(run), name_(name), running_(false), active_(false), manager_(GazeboActivityManager::Instance())
{
    manager_->add(this);
}

GazeboActivity::GazeboActivity(RTT::Seconds period, RunnableInterface* run, const std::string& name)
    : ActivityInterface(run), name_(name), period_(period), running_(false), active_(false), manager_(GazeboActivityManager::Instance())
{
    manager_->add(this);
}

GazeboActivity::~GazeboActivity()
{
    stop();
    manager_->remove(this);
}

RTT::Seconds GazeboActivity::getPeriod() const
{
    if (period_ > 0.0)
        return period_;
    else
        return manager_->getSimulationPeriod();
}

bool GazeboActivity::setPeriod(RTT::Seconds s)
{
    period_ = s;
    return true;
}

unsigned GazeboActivity::getCpuAffinity() const
{
    return ~0;
}

bool GazeboActivity::setCpuAffinity(unsigned cpu)
{
    return false;
}

RTT::os::ThreadInterface* GazeboActivity::thread()
{
    return 0;
}

bool GazeboActivity::initialize()
{
    return true;
}

void GazeboActivity::step()
{
}

void GazeboActivity::loop()
{
    this->step();
}

bool GazeboActivity::breakLoop()
{
    return false;
}


void GazeboActivity::finalize()
{
}

bool GazeboActivity::start()
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

bool GazeboActivity::stop()
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

bool GazeboActivity::isRunning() const
{
    return running_;
}

bool GazeboActivity::isPeriodic() const
{
    return true;
}

bool GazeboActivity::isActive() const
{
    return active_;
}

bool GazeboActivity::trigger()
{
    return false;
}

bool GazeboActivity::execute()
{
    if (!running_) return false;
    if (runner) runner->step(); else this->step();
    last_ = RTT::os::TimeService::Instance()->getTicks();
    return true;
}

RTT::os::TimeService::ticks GazeboActivity::getLastExecutionTicks() const
{
    return last_;
}

} // namespace rtt_gazebo_activity
