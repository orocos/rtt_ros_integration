#ifndef RTT_STD_SRVS_SETBOOL_H
#define RTT_STD_SRVS_SETBOOL_H

#include <rtt_roscomm/rtt_rosservice_proxy.h>
#include <std_srvs/SetBool.h>

// Specialized implementation of ROSServiceServerOperationCaller for std_srvs/SetBool.
//
// Accepted signatures:
//  - bool setBool(std_srvs::SetBool::Request&, std_srvs::SetBool::Response&)
//  - bool setBool(bool, std::string &message_out)
//  - bool setBool(bool) // response.message will be empty
//  - std::string setBool(bool) // response.success = true
//  - void setBool(bool) // response.success = true and response.message will be empty
//

namespace rtt_roscomm {

template<> struct ROSServiceServerOperationCallerWrapper<std_srvs::SetBool,1> {
  typedef bool Signature(bool, std::string&);
  typedef RTT::OperationCaller<Signature> ProxyOperationCallerType;
  template <typename Callable> static bool call(Callable& call, std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response) {
    response.success = call(request.data, response.message);
    return true;
  }
};

template<> struct ROSServiceServerOperationCallerWrapper<std_srvs::SetBool,2> {
  typedef bool Signature(bool);
  typedef RTT::OperationCaller<Signature> ProxyOperationCallerType;
  template <typename Callable> static bool call(Callable& call, std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response) {
    response.success = call(request.data);
    return true;
  }
};

template<> struct ROSServiceServerOperationCallerWrapper<std_srvs::SetBool,3> {
  typedef std::string Signature(bool);
  typedef RTT::OperationCaller<Signature> ProxyOperationCallerType;
  template <typename Callable> static bool call(Callable& call, std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response) {
    response.message = call(request.data);
    response.success = true;
    return true;
  }
};

template<> struct ROSServiceServerOperationCallerWrapper<std_srvs::SetBool,4> {
  typedef void Signature(bool);
  typedef RTT::OperationCaller<Signature> ProxyOperationCallerType;
  template <typename Callable> static bool call(Callable& call, std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response) {
    call(request.data);
    response.success = true;
    return true;
  }
};

}  // namespace rtt_roscomm

#endif  // RTT_STD_SRVS_SETBOOL_H
