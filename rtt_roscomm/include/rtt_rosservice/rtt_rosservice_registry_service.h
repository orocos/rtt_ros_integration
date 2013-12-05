#ifndef __RTT_ROSSERVICE_RTT_ROSSERVICE_REGISTRY_SERVICE_H
#define __RTT_ROSSERVICE_RTT_ROSSERVICE_REGISTRY_SERVICE_H

#include <rtt/RTT.hpp>
#include <boost/shared_ptr.hpp>

class ROSServiceRegistryService;
class ROSServiceProxyFactoryBase;
typedef boost::shared_ptr<ROSServiceRegistryService> ROSServiceRegistryServicePtr;

class ROSServiceRegistryService : public RTT::Service
{
public:
  static ROSServiceRegistryServicePtr Instance();
  static void Release();

  /** \brief Register a ROS service proxy factory
   *
   * This enables the ROSServiceRegistryService to construct ROS service clients and
   * servers from a string name.
   */
  bool registerServiceFactory(ROSServiceProxyFactoryBase* factory);

  bool hasServiceFactory(const std::string &service_type);

  ROSServiceProxyFactoryBase* getServiceFactory(const std::string &service_type);

private:
  /**
   * Instantiates this service.
   * @param owner The owner or null in case of global.
   */
  ROSServiceRegistryService(RTT::TaskContext* owner);

  //! ROS service proxy factories
  static std::map<std::string, boost::shared_ptr<ROSServiceProxyFactoryBase> > factories_;
  static RTT::os::MutexRecursive factory_lock_;

  //! The singleton instance
  static ROSServiceRegistryServicePtr s_instance_;
};

#endif // __RTT_ROSSERVICE_RTT_ROSSERVICE_REGISTRY_SERVICE_H
