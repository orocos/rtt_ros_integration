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

    //! Check if simulation time is enabled
    bool simTimeEnabled() { 
      return use_sim_time_;
    }

    /**
     * Update the RTT clock and SimClockActivities with a new time
     *
     * This can be called internally from clockMsgCallback or externally
     * by another library (e.g. Gazebo) for in-process triggering.
     */
    void SimClockThread::updateClock(RTT::os::TimeService::Seconds clock_secs);

  protected:

    //! RTT::os::Thread interface
    virtual bool initialize();
    virtual void loop();
    virtual bool breakLoop();
    virtual void finalize();

  private:
    //! SimClockThread singleton
    static boost::weak_ptr<SimClockThread> singleton;

    //! Convenient pointer to RTT time service
    RTT::os::TimeService *time_service_;

    //! Thread stuff
    bool break_loop_;

    //! Flag designating that simulation time should be used
    bool use_sim_time_;

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



