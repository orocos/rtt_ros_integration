#ifndef __RTT_ROS_CLOCK_H
#define __RTT_ROS_CLOCK_H

#include <rtt/os/TimeService.hpp>
#include <ros/time.h>
#include <time.h>

namespace rtt_rosclock {
  //! Get the current time according to RTT
  const ros::Time rtt_now();

  //! Get the current time according to ROS
  const ros::Time ros_now();

  //! Get the current time according to CLOCK_HOST_REALTIME
  const ros::Time host_rt_now();

  //! Get the difference in seconds between RTT and CLOCK_HOST_REALTIME
  const RTT::Seconds host_rt_offset_from_rtt();
}

#endif // ifndef __RTT_ROS_CLOCK_H
