#ifndef __RTT_ROSSERVICE_ROSSERVICE_H
#define __RTT_ROSSERVICE_ROSSERVICE_H

#include <rtt/RTT.hpp>
#include <rtt/Property.hpp>

namespace rtt_rosservice {

  class ROSService : public RTT::ServiceRequester
  {
  public:
    ROSService(RTT::TaskContext *owner) :
      RTT::ServiceRequester("rosservice",owner),
      connect("connect")
    {
      this->addOperationCaller(connect);
    }

    RTT::OperationCaller<bool(const std::string &, const std::string &, const std::string &)> connect;
  };
}

#endif // ifndef __RTT_ROSSERVICE_ROSSERVICE_H

