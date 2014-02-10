#ifndef __RTT_ROS_TIME_H
#define __RTT_ROS_TIME_H

#include <rtt/os/TimeService.hpp>
#include <ros/time.h>

namespace rtt_ros {
  namespace time {
    //! Get a ros time structure using
    static const ros::Time now() {
      //return ros::Time(((double)RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks()))*1E-9);
      return ros::Time(RTT::nsecs_to_Seconds(RTT::os::TimeService::Instance()->getNSecs()));
    }
  }
}

#endif // ifndef __RTT_ROS_TIME_H
