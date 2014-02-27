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

  //! Set a TaskContext's activity to use the simulation clock periodically
  const bool set_sim_clock_activity(RTT::TaskContext *t);

  //! Use ROS /clock topic for time measurement
  const bool enable_sim();
  
  //! Don't use ROS /clock topic for time measurement
  const bool disable_sim();

  //! Update the current simulation time
  void update_sim_time(const RTT::os::Seconds now)
}

#endif // ifndef __RTT_ROSCLOCK_RTT_ROSCLOCK_H
