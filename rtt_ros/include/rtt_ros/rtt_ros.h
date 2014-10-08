#ifndef __RTT_ROS_RTT_ROS_H
#define __RTT_ROS_RTT_ROS_H

#include <string>

namespace rtt_ros {

  //! Global ros namespace
  static std::string active_ns;

  //! Import a ROS package and all of its rtt_ros/plugin_depend dependencies
  bool import(const std::string& package);

  //! Set the full ROS namespace
  std::string setNS(const std::string& ns);

  //! Resolve the namespace based on the active namespace
  std::string resolveName(const std::string& name);
  
  //! Reset the full ROS namespace
  std::string resetNS();
  
  //! Get the full ROS namespace
  std::string getNS();
  
  //! Append to the ROS namespace
  std::string pushNS(const std::string& ns);
  
  //! Remove the last token from the ROS namespace
  std::string popNS();
}

#endif // __RTT_ROS_RTT_ROS_H
