#ifndef __RTT_ROS_RTT_ROS_H
#define __RTT_ROS_RTT_ROS_H

#include <string>

namespace rtt_ros {

  //! Import a ROS package and all of its rtt_ros/plugin_depend dependencies
  bool import(const std::string& package);

}

#endif // __RTT_ROS_RTT_ROS_H
