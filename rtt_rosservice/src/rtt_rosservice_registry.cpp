
#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/internal/GlobalService.hpp>

#include <rtt_rosservice/ros_service_proxy.h> 

class ROSServiceRegistryService : public RTT::Service 
{
public:
  /**
   * Instantiates this service.
   * @param owner The owner or null in case of global.
   */
  ROSServiceRegistryService(RTT::TaskContext* owner) 
    : Service("rosservice_registry", owner)
  {
    this->doc("Global RTT Service for registering ROS service types.");
    this->addOperation("registerServiceFactory", &ROSServiceRegistryService::registerServiceFactory, this, RTT::ClientThread);
    this->addOperation("hasServiceFactory", &ROSServiceRegistryService::hasServiceFactory, this, RTT::ClientThread);
    this->addOperation("getServiceFactory", &ROSServiceRegistryService::getServiceFactory, this, RTT::ClientThread);
  }

  /** \brief Register a ROS service proxy factory
   *
   * This enables the ROSServiceRegistryService to construct ROS service clients and
   * servers from a string name.
   */
  void registerServiceFactory(ROSServiceProxyFactoryBase* factory) 
  {
    RTT::os::MutexLock lock(factory_lock_);
    // Store the factory
    factories_[factory->getType()] = factory;
  }

  bool hasServiceFactory(const std::string &service_type)
  {
    RTT::os::MutexLock lock(factory_lock_);
    return factories_.find(service_type) != factories_.end();
  }

  ROSServiceProxyFactoryBase* getServiceFactory(const std::string &service_type) 
  {
    RTT::os::MutexLock lock(factory_lock_);
    if(factories_.find(service_type) != factories_.end()) {
      return factories_[service_type];
    }

    RTT::log(RTT::Error)<<"Service type \""<<service_type<<"\" has not been registerd with the rosserivce_registry service."<<RTT::endlog();

    return NULL;
  }

  //! ROS service proxy factories
  static std::map<std::string, ROSServiceProxyFactoryBase*> factories_;
  static RTT::os::Mutex factory_lock_;
};

std::map<std::string, ROSServiceProxyFactoryBase*> ROSServiceRegistryService::factories_;
RTT::os::Mutex ROSServiceRegistryService::factory_lock_;

void loadROSServiceRegistryService()
{
  RTT::Service::shared_ptr rsrs(new ROSServiceRegistryService(NULL));
  RTT::internal::GlobalService::Instance()->addService(rsrs);
}

using namespace RTT;
extern "C" {
  bool loadRTTPlugin(RTT::TaskContext* c){
    loadROSServiceRegistryService();
    return true;
  }
  std::string getRTTPluginName (){
    return "rosservice_registry";
  }
  std::string getRTTTargetName (){
    return OROCOS_TARGET_NAME;
  }
}
