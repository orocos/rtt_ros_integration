#ifndef __RTT_ROSTOPIC_ROSTOPIC_H
#define __RTT_ROSTOPIC_ROSTOPIC_H

#include <rtt/RTT.hpp>
#include <rtt/Property.hpp>

namespace rtt_rostopic {

  class ROSTopic : public RTT::ServiceRequester
  {
  public:
    ROSTopic(RTT::TaskContext *owner) :
      RTT::ServiceRequester("rostopic",owner),
      connection("connection"),
      bufferedConnection("bufferedConnection"),
      unbufferedConnection("unbufferedConnection")
    {
      this->addOperationCaller(connection);
      this->addOperationCaller(bufferedConnection);
      this->addOperationCaller(unbufferedConnection);
    }

    RTT::OperationCaller<RTT::ConnPolicy(const std::string &)> connection;
    RTT::OperationCaller<RTT::ConnPolicy(const std::string &, int)> bufferedConnection;
    RTT::OperationCaller<RTT::ConnPolicy(const std::string &)> unbufferedConnection;
  };
}

#endif // ifndef __RTT_ROSTOPIC_ROSTOPIC_H

