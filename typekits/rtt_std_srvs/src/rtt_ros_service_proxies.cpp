
#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/internal/GlobalService.hpp>

#include <rtt_rosservice/ros_service_proxy.h> 

////////////////////////////////////////////////////////////////////////////////
#include <std_srvs/Empty.h>
////////////////////////////////////////////////////////////////////////////////

void registerROSServiceProxies(){
  // Get the ros service service
  RRTT::Service::shared_ptr rss(RTT::internal::GlobalService::Instance()->getService("rosservice"));
  RTT::OperationCaller<void(ROSServiceProxyBase*)> register_service = rss->getOperation("registerServiceType");

  //////////////////////////////////////////////////////////////////////////////
  /** Proxy for "std_srvs/Empty" **/
  ROSServiceProxyFactoryBase* EmptyProxy = new ROSServiceProxyFactory<std_srvs::Empty, std_srvs::Empty::Request, std_srvs::Empty::Response>("std_srvs","Empty");
  register_service(EmptyProxy);
  //////////////////////////////////////////////////////////////////////////////
}

using namespace RTT;
extern "C" {
  bool loadRTTPlugin(RTT::TaskContext* c){
    registerROSServiceProxies();
    return true;
  }
  std::string getRTTPluginName (){
    return "rtt_std_srvs_ros_service_proxies";
  }
  std::string getRTTTargetName (){
    return OROCOS_TARGET_NAME;
  }
}
