
#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/internal/GlobalService.hpp>

#include <rtt_rosservice/ros_service_proxy.h> 

////////////////////////////////////////////////////////////////////////////////
#include <std_srvs/Empty.h>
////////////////////////////////////////////////////////////////////////////////

bool registerROSServiceProxies() {
  // Get the ros service registry service
  if(!RTT::internal::GlobalService::Instance()->hasService("rosservice_registry")) {
    return false;
  }

  // Get the factory registration operation
  RTT::OperationCaller<bool(ROSServiceProxyFactoryBase*)> register_service_factory = 
    RTT::internal::GlobalService::Instance()->getService("rosservice_registry")->getOperation("registerServiceFactory");

  // Make sure the registration operation is ready
  if(!register_service_factory.ready()) {
    return false;
  }

  // True at the end if all factories have been registered
  bool success = true;

  //////////////////////////////////////////////////////////////////////////////
  success = success && register_service_factory(new ROSServiceProxyFactory<std_srvs::Empty>("std_srvs/Empty"));
  //////////////////////////////////////////////////////////////////////////////

  return success;
}

extern "C" {
  bool loadRTTPlugin(RTT::TaskContext* c) { return registerROSServiceProxies(); }
  std::string getRTTPluginName () { return "rtt_std_srvs_ros_service_proxies"; }
  std::string getRTTTargetName () { return OROCOS_TARGET_NAME; }
}
