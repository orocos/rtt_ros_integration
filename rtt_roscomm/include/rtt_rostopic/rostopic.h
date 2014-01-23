#ifndef __RTT_ROSTOPIC_ROSTOPIC_H
#define __RTT_ROSTOPIC_ROSTOPIC_H

#include <rtt/RTT.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <rtt_rostopic/rtt_rostopic.h> 

namespace rtt_rostopic {

  class ROSTopic : public RTT::ServiceRequester
  {
  public:
    ROSTopic() :
      RTT::ServiceRequester("rostopic",NULL),
      connection("connection"),
      bufferedConnection("bufferedConnection"),
      unbufferedConnection("unbufferedConnection"),
      protocol_id(ORO_ROS_PROTOCOL_ID)
    {
      this->addOperationCaller(connection);
      this->addOperationCaller(bufferedConnection);
      this->addOperationCaller(unbufferedConnection);

      this->connectTo(RTT::internal::GlobalService::Instance()->provides("rostopic"));
    }

    RTT::OperationCaller<RTT::ConnPolicy(const std::string &)> connection;
    RTT::OperationCaller<RTT::ConnPolicy(const std::string &, int)> bufferedConnection;
    RTT::OperationCaller<RTT::ConnPolicy(const std::string &)> unbufferedConnection;

    int protocol_id;
  };
}

#endif // ifndef __RTT_ROSTOPIC_ROSTOPIC_H

