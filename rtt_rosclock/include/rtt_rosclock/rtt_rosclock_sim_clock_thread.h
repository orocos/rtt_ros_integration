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

#ifndef __RTT_ROSCLOCK_SIM_CLOCK_THREAD_H
#define __RTT_ROSCLOCK_SIM_CLOCK_THREAD_H

#include <rtt/Service.hpp>
#include <rtt/os/Thread.hpp>
#include <rtt/os/TimeService.hpp>

#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/callback_queue.h>

#include <rosgraph_msgs/Clock.h>

namespace rtt_rosclock {

  /** 
   * This activity subscribes to the ROS /clock topic and overrides the RTT
   * TimeService if the `/use_sim_time` ROS parameter is set.
   */
  class SimClockThread : public RTT::os::Thread 
  {
  public:
    //! Get an instance to the singleton SimClockThread or create one
    static boost::shared_ptr<SimClockThread> Instance();
    //! Get an instance to the singleton SimClockThread or NULL
    static boost::shared_ptr<SimClockThread> GetInstance();

    virtual ~SimClockThread();

    //! Simulation clock sources
    enum SimClockSource {
      SIM_CLOCK_SOURCE_MANUAL = 0,
      SIM_CLOCK_SOURCE_ROS_CLOCK_TOPIC = 1
    };

    //! Set the simulation clock source by ID (see ClockSource enum)
    bool setClockSource(SimClockSource clock_source);
    //! Set the clock source to use the ROS /clock topic
    bool useROSClockTopic();
    //! Set the clock source to use a manual source, i.e. call `updateClock()` manually
    bool useManualClock();

    //! Check if simulation time is enabled
    bool simTimeEnabled() const;

    /**
     * Update the RTT clock and SimClockActivities with a new time
     *
     * This can be called internally from clockMsgCallback or externally
     * by another library (e.g. Gazebo) for in-process triggering. This
     * can be called manually via the ros.clock global service's updateSimClock()
     * operation.
     */
    bool updateClock(const ros::Time new_time);

  protected:

    //! Constructor is protected, use Instance() to create and get a singleton
    SimClockThread();
    SimClockThread(SimClockThread const&);
    void operator=(SimClockThread const&);

    //! SimClockThread singleton
    static boost::weak_ptr<SimClockThread> singleton;

    //! Re-set the RTT::os::TimeService to zero and restart logging
    void resetTimeService();

    //! Update the RTT clock and SimClockActivities with a new time (see updateClock() for manually updating)
    bool updateClockInternal(const ros::Time new_time);

    // RTT::os::Thread interface
    virtual bool initialize();
    virtual void loop();
    virtual bool breakLoop();
    virtual void finalize();

    //! Convenient pointer to RTT time service
    RTT::os::TimeService *time_service_;

    //! Current clock source
    SimClockSource clock_source_;
    
    //! Keep running the thread loop if this is set to true
    bool process_callbacks_;

    //! ROS NodeHandle for communication
    ros::NodeHandle nh_;
    //! ROS /clock topic subscriber
    ros::Subscriber clock_subscriber_;
    //! Custom callback queue used in this thread
    ros::CallbackQueue callback_queue_;
    //! ROS message callback for /clock topic
    void clockMsgCallback(const rosgraph_msgs::ClockConstPtr& clock);
  };

}

#endif // ifndef __RTT_ROSCLOCK_RTT_ROSCLOCK_ACTIVITY_H



