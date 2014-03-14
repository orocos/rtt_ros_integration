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

#ifndef __RTT_ROSCLOCK_RTT_ROSCLOCK_SIM_CLOCK_ACTIVITY_MANAGER_H
#define __RTT_ROSCLOCK_RTT_ROSCLOCK_SIM_CLOCK_ACTIVITY_MANAGER_H

#include <rtt/base/ActivityInterface.hpp>

#include <rtt/os/TimeService.hpp>
#include <rtt/os/Mutex.hpp>

#include <list>

namespace rtt_rosclock {

  class SimClockActivity;

  /**
   * \brief A centralized list of all TaskContexts using SimClockActivity
   *
   * The SimClockActivityManager is used to synchronously update all TaskContexts
   * using a SimClockActivity. This is the primary interface to executing a set of
   * periodic tasks in simulation. 
   *
   */
  class SimClockActivityManager 
  {
  public:
    //! Get an instance of the singleton
    static boost::shared_ptr<SimClockActivityManager> Instance();

    //! Get an instance of the singleton if it exists, null pointer otherwise
    static boost::shared_ptr<SimClockActivityManager> GetInstance();

    ~SimClockActivityManager();

    RTT::Seconds getSimulationPeriod() const;
    void setSimulationPeriod(RTT::Seconds s);

    //! Execute all activities modulo their desired periods
    void update();

  protected:
    //! The SimClockActivityManager is a singleton and is constructed by calling Instance()
    SimClockActivityManager();
    //! The SimClockActivityManager is a singleton and is constructed by calling Instance()
    SimClockActivityManager(SimClockActivityManager const&);
    void operator=(SimClockActivityManager const&);

  private:
    //! Only SimClockActivities can add or remove themselves
    friend class SimClockActivity;

    //! Add an activity to the manager
    void add(SimClockActivity *activity);
    //! Remove an activity from the manager
    void remove(SimClockActivity *activity);

  private:

    //! SimClockActivityManager singleton
    static boost::weak_ptr<SimClockActivityManager> singleton;

    //! Mutex used to exclude adding and removing activities from the update to all activities
    RTT::os::Mutex modify_activities_mutex_;

    //! All existing SimClockActivities
    std::list<SimClockActivity *> activities_;

    //! The desired/expected simulation period
    RTT::Seconds simulation_period_;
  };

}

#endif // ifndef __RTT_ROSCLOCK_RTT_ROSCLOCK_SIM_CLOCK_ACTIVITY_MANAGER_H
