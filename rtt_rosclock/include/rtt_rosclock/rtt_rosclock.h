#ifndef __RTT_ROSCLOCK_RTT_ROSCLOCK_H
#define __RTT_ROSCLOCK_RTT_ROSCLOCK_H

#include <rtt/RTT.hpp>
#include <rtt/os/TimeService.hpp>
#include <ros/time.h>

namespace rtt_rosclock {

  //! Get the current time according to RTT
  const ros::Time rtt_now();

  //! Get the current time according to ROS
  const ros::Time ros_now();

  //! Get the current time according to CLOCK_HOST_REALTIME
  const ros::Time host_rt_now();

  //! Get the difference in seconds between RTT and CLOCK_HOST_REALTIME
  const RTT::Seconds host_rt_offset_from_rtt();

  //! Set a TaskContext to use a periodic simulation clock activity
  const bool set_sim_clock_activity(RTT::TaskContext *t);

  //! Use ROS /clock topic for time measurement
  void use_ros_clock_topic();

  //! Use manual clock updates
  void use_manual_clock();

  //! Use a simulated clock source
  const bool enable_sim();
  
  //! Do't use a simulated clock source
  const bool disable_sim();

  //! Update the current simulation time and trigger all simulated TaskContexts
  void update_sim_clock(const ros::Time new_time);
}

#endif // ifndef __RTT_ROSCLOCK_RTT_ROSCLOCK_H
