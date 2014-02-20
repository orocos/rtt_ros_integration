#ifndef __RTT_ROS_ROS_H
#define __RTT_ROS_ROS_H

#include <rtt/RTT.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <rtt_rostopic/rtt_rostopic.h> 

namespace rtt_ros {

  class ROS : public RTT::ServiceRequester
  {
  public:
    ROS() :
      RTT::ServiceRequester("ros",NULL),
      import("import")
    {
      this->addOperationCaller(import);

      RTT::log(RTT::Warning) << "Ignore the following warnings about callers not being set." << RTT::endlog();
      this->connectTo(RTT::internal::GlobalService::Instance()->provides("ros"));
    }

    RTT::OperationCaller<bool(const std::string &)> import;
  };
}

#endif // ifndef __RTT_ROS_ROS_H

