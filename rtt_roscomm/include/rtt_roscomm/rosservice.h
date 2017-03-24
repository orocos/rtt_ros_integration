#ifndef __RTT_ROSSERVICE_ROSSERVICE_H
#define __RTT_ROSSERVICE_ROSSERVICE_H

#include <rtt/RTT.hpp>
#include <rtt/Property.hpp>

namespace rtt_roscomm {

  class ROSService : public RTT::ServiceRequester
  {
  public:
    ROSService(RTT::TaskContext *owner) :
      RTT::ServiceRequester("rosservice",owner),
      connect("connect"),
      disconnect("disconnect"),
      disconnectAll("disconnectAll")
    {
      this->addOperationCaller(connect);
      this->addOperationCaller(disconnect);
      this->addOperationCaller(disconnectAll);
    }

    RTT::OperationCaller<bool(const std::string &, const std::string &, const std::string &)> connect;
    RTT::OperationCaller<bool(const std::string &)> disconnect;
    RTT::OperationCaller<void()> disconnectAll;
  };
}

namespace rtt_rosservice {
    using rtt_roscomm::ROSService; // deprecated
}

#endif // ifndef __RTT_ROSSERVICE_ROSSERVICE_H

