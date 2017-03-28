#ifndef __RTT_ROSCOMM_RTT_ROSSERVICE_PROXY_H
#define __RTT_ROSCOMM_RTT_ROSSERVICE_PROXY_H

#include <ros/ros.h>

#include <rtt/RTT.hpp>
#include <rtt/internal/GlobalEngine.hpp>
#include <rtt/plugin/ServicePlugin.hpp>

//! Abstract ROS Service Proxy
class ROSServiceProxyBase
{
public:
  ROSServiceProxyBase(const std::string &service_name) : service_name_(service_name) { }
  virtual ~ROSServiceProxyBase() { }
  //! Get the name of the ROS service
  const std::string& getServiceName() const { return service_name_; }
private:
  //! ROS Service name (fully qualified)
  std::string service_name_;
};


//! Abstract ROS Service Server Proxy
class ROSServiceServerProxyBase : public ROSServiceProxyBase
{ 
public:
  ROSServiceServerProxyBase(const std::string &service_name) :
    ROSServiceProxyBase(service_name),
    proxy_operation_caller_()
  { }
  
  //! Connect an RTT Operation to this ROS service server
  bool connect(RTT::TaskContext *owner, RTT::OperationInterfacePart* operation) {
    // Link the caller with the operation
    return proxy_operation_caller_->setImplementation(
        operation->getLocalOperation(),
        RTT::internal::GlobalEngine::Instance());
  }

protected:
  //! The underlying ROS service server
  ros::ServiceServer server_;
  //! The underlying RTT operation caller
  boost::shared_ptr<RTT::base::OperationCallerBaseInvoker> proxy_operation_caller_;
};

template<class ROS_SERVICE_T>
class ROSServiceServerProxy : public ROSServiceServerProxyBase 
{
public:
  //! Operation caller for a ROS service server proxy
  typedef RTT::OperationCaller<bool(typename ROS_SERVICE_T::Request&, typename ROS_SERVICE_T::Response&)> ProxyOperationCallerType;

  /** \brief Construct a ROS service server and associate it with an Orocos
   * task's required interface and operation caller.
   */
  ROSServiceServerProxy(const std::string &service_name) :
    ROSServiceServerProxyBase(service_name)
  {
    // Construct operation caller
    proxy_operation_caller_.reset(new ProxyOperationCallerType("ROS_SERVICE_SERVER_PROXY"));

    // Construct the ROS service server
    ros::NodeHandle nh;
    server_ = nh.advertiseService(
        service_name, 
        &ROSServiceServerProxy<ROS_SERVICE_T>::ros_service_callback, 
        this);
  }

  ~ROSServiceServerProxy()
  {
    // Clean-up advertized ROS services
    server_.shutdown();     
  }

private:
  
  //! The callback called by the ROS service server when this service is invoked
  bool ros_service_callback(typename ROS_SERVICE_T::Request& request, typename ROS_SERVICE_T::Response& response) {
    // Downcast the proxy operation caller
    ProxyOperationCallerType &proxy_operation_caller = *dynamic_cast<ProxyOperationCallerType*>(proxy_operation_caller_.get());
    // Check if the operation caller is ready, and then call it
    return proxy_operation_caller_->ready() && proxy_operation_caller(request, response);
  }
};


//! Abstract ROS Service Client Proxy
class ROSServiceClientProxyBase : public ROSServiceProxyBase
{
public:
  ROSServiceClientProxyBase(const std::string &service_name) : 
    ROSServiceProxyBase(service_name),
    proxy_operation_()
  { }

  //! Connect an operation caller with this proxy
  bool connect(RTT::TaskContext *owner, RTT::base::OperationCallerBaseInvoker* operation_caller) {
    return proxy_operation_.get() != NULL &&
      operation_caller->setImplementation(
          proxy_operation_->getImplementation(),
          owner->engine());
  }

protected:
  //! The underlying ROS service client
  ros::ServiceClient client_;
  //! The underlying RTT operation 
  boost::shared_ptr<RTT::base::OperationBase> proxy_operation_;
};

template<class ROS_SERVICE_T>
class ROSServiceClientProxy : public ROSServiceClientProxyBase 
{
public:

  //! The proxy RTT operation type for this ROS service
  typedef RTT::Operation<bool(typename ROS_SERVICE_T::Request&, typename ROS_SERVICE_T::Response&)> ProxyOperationType;

  ROSServiceClientProxy(const std::string &service_name) :
    ROSServiceClientProxyBase(service_name)
  {
    // Construct a new 
    proxy_operation_.reset(new ProxyOperationType("ROS_SERVICE_CLIENT_PROXY"));

    // Construct the underlying service client
    ros::NodeHandle nh;
    client_ = nh.serviceClient<ROS_SERVICE_T>(service_name);

    // Link the operation with the service client
    dynamic_cast<ProxyOperationType*>(proxy_operation_.get())->calls(
        &ROSServiceClientProxy<ROS_SERVICE_T>::orocos_operation_callback,
        this,
        RTT::ClientThread);
  }

private:
  
  //! The callback for the RTT operation
  bool orocos_operation_callback(typename ROS_SERVICE_T::Request& request, typename ROS_SERVICE_T::Response& response) {
    // Make sure the ROS service client exists and then call it (blocking)
    return client_.exists() && client_.isValid() && client_.call(request, response);
  }
};


//! Abstract factory for ROS Service Proxy Factories
class ROSServiceProxyFactoryBase 
{
public:

  ROSServiceProxyFactoryBase(const std::string &service_type) : service_type_(service_type) { }

  //! Get the ROS service type
  const std::string& getType() { return service_type_; }

  //! Get a proxy to a ROS service client
  virtual ROSServiceClientProxyBase* create_client_proxy(const std::string &service_name) = 0;
  //! Get a proxy to a ROS service server
  virtual ROSServiceServerProxyBase* create_server_proxy(const std::string &service_name) = 0;

private:
  std::string service_type_;
};

template<class ROS_SERVICE_T>
class ROSServiceProxyFactory : public ROSServiceProxyFactoryBase 
{
public:

  ROSServiceProxyFactory(const std::string &service_type) : ROSServiceProxyFactoryBase(service_type) { }

  virtual ROSServiceClientProxyBase* create_client_proxy(const std::string &service_name) {
    return new ROSServiceClientProxy<ROS_SERVICE_T>(service_name);
  }

  virtual ROSServiceServerProxyBase* create_server_proxy( const std::string &service_name) {
    return new ROSServiceServerProxy<ROS_SERVICE_T>(service_name);
  }
};

#endif // ifndef __RTT_ROSCOMM_RTT_ROSSERVICE_PROXY_H
