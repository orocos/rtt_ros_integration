#ifndef RTT_STD_SRVS_EMPTY_H
#define RTT_STD_SRVS_EMPTY_H

#include <rtt_roscomm/rtt_rosservice_proxy.h>
#include <std_srvs/Empty.h>

// Specialized implementations of ROSServiceServerOperationCaller for std_srvs/Empty.
//
// Accepted signatures:
//  - bool empty(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
//  - bool empty()
//  - void empty()
//

namespace rtt_roscomm {

template<> struct ROSServiceServerOperationCallerWrapper<std_srvs::Empty,1> {
  typedef bool Signature();
  typedef RTT::OperationCaller<Signature> ProxyOperationCallerType;
  template <typename Callable> static bool call(Callable& call, std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    return call();
  }
};

template<> struct ROSServiceServerOperationCallerWrapper<std_srvs::Empty,2> {
  typedef void Signature();
  typedef RTT::OperationCaller<Signature> ProxyOperationCallerType;
  template <typename Callable> static bool call(Callable& call, std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    call();
    return true;
  }
};

}  // namespace rtt_roscomm

#endif  // RTT_STD_SRVS_EMPTY_H
