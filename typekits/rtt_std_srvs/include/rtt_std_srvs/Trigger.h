#ifndef RTT_STD_SRVS_TRIGGER_H
#define RTT_STD_SRVS_TRIGGER_H

#include <rtt_roscomm/rtt_rosservice_proxy.h>
#include <std_srvs/Trigger.h>

// Specialized implementations of ROSServiceServerOperationCaller for std_srvs/Trigger.
//
// Accepted signatures:
//  - bool trigger(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&)
//  - bool trigger(std::string &message_out)
//  - bool trigger() // response.message will be empty
//  - std::string trigger() // response.success = true
//

namespace rtt_roscomm {

template<> struct ROSServiceServerOperationCallerWrapper<std_srvs::Trigger,1> {
  typedef bool Signature(std::string &message_out);
  typedef RTT::OperationCaller<Signature> ProxyOperationCallerType;
  template <typename Callable> static bool call(Callable& call, std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
    response.success = call(response.message);
    return true;
  }
};

template<> struct ROSServiceServerOperationCallerWrapper<std_srvs::Trigger,2> {
  typedef bool Signature();
  typedef RTT::OperationCaller<Signature> ProxyOperationCallerType;
  template <typename Callable> static bool call(Callable& call, std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
    response.success = call();
    return true;
  }
};

template<> struct ROSServiceServerOperationCallerWrapper<std_srvs::Trigger,3> {
  typedef std::string Signature();
  typedef RTT::OperationCaller<Signature> ProxyOperationCallerType;
  template <typename Callable> static bool call(Callable& call, std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
    response.message = call();
    response.success = true;
    return true;
  }
};

}  // namespace rtt_roscomm

#endif  // RTT_STD_SRVS_TRIGGER_H
