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

  //! Use ROS /clock topic for time measurement
  const bool enable_sim();
  
  //! Don't use ROS /clock topic for time measurement
  const bool disable_sim();
}

#endif // ifndef __RTT_ROSCLOCK_RTT_ROSCLOCK_H
