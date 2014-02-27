#ifndef __RTT_ROSCLOCK_SIM_CLOCK_THREAD_H
#define __RTT_ROSCLOCK_SIM_CLOCK_THREAD_H

#include <rtt/Service.hpp>
#include <rtt/os/Thread.hpp>
#include <rtt/os/TimeService.hpp>

#include <ros/subscriber.h>
#include <ros/callback_queue.h>

#include <rosgraph_msgs/Clock.h>

namespace rtt_rosclock {

  /** 
   * This activity subscribes to the ROS /clock topic and overrides the RTT
   * TimeService if the `/use_sim_time` ROS parameter is set.
   */
  class SimClockThread : public RTT::os::Thread {
  public:
    //! Get an instance to the singleton SimClockThread or create one
    static boost::shared_ptr<SimClockThread> Instance();
    //! Get an instance to the singleton SimClockThread or NULL
    static boost::shared_ptr<SimClockThread> GetInstance();

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
    void updateClock(RTT::os::TimeService::Seconds clock_secs);

  protected:

    //! SimClockThread singleton
    static boost::weak_ptr<SimClockThread> singleton;

    //! Re-set the RTT::os::TimeService to zero and restart logging
    void resetTimeService();

    //! Update the RTT clock and SimClockActivities with a new time (see updateClock() for manually updating)
    void updateClockInternal(RTT::os::TimeService::Seconds clock_secs);

    // RTT::os::Thread interface
    virtual bool initialize();
    virtual void loop();
    virtual bool breakLoop();
    virtual void finalize();

    //! Convenient pointer to RTT time service
    RTT::os::TimeService *time_service_;

    //! Current clock source
    ClockSource clock_source_;
    
    //! Keep running the thread loop if this is set to true
    bool process_callbacks_;

    //! ROS NodeHandle for communication
    ros::NodeHandle nh_;
    //! ROS /clock topic subscriber
    ros::Subscriber clock_subscriber_;
    //! Custom callback queue used in this thread
    ros::CallbackQueue callback_queue_;
    //! ROS message callback for /clock topic
    void clockMsgCallback(const rosgraph_msgs::ClockConstPtr& clock)
  };

}

#endif // ifndef __RTT_ROSCLOCK_RTT_ROSCLOCK_ACTIVITY_H



