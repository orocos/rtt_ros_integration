#ifndef __RTT_ROSSERVICE_ROS_SERVICE_PROXY_H
#define __RTT_ROSSERVICE_ROS_SERVICE_PROXY_H

#include <boost/range/iterator_range.hpp>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>

#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/internal/GlobalService.hpp>

//! Abstract ROS Service Proxy
class ROSServiceProxyBase
{
public:
  ROSServiceProxyBase(const std::string &service_name) : service_name_(service_name) { }
  //! Get the name of the ROS service
  const std::string& getServiceName() const { return service_name_; }
private:
  //! ROS Service name (fully qualified)
  std::string service_name_;
};


//! Abstract ROS Service Proxy Server
class ROSServiceServerProxyBase : public ROSServiceProxyBase
{ 
public:
  ROSServiceServerProxyBase(const std::string &full_service_name) : ROSServiceProxyBase(full_service_name) { }
private:
  //! The underlying ROS service server
  ros::ServiceServer server_;
};

template<class ROS_SERVICE_T, class ROS_SERVICE_REQ_T, class ROS_SERVICE_RESP_T>
class ROSServiceServerProxy : public ROSServiceServerProxyBase 
{
public:
  //! Operation caller for a ROS service server proxy
  typedef RTT::OperationCaller<bool(ROS_SERVICE_REQ_T&, ROS_SERVICE_RESP_T&)> ProxyOperationCallerType;

  /** \brief Construct a ROS service server and associate it with an Orocos
   * task's required interface and operation caller.
   */
  ROSServiceServerProxy(RTT::ServiceRequester* requester, const std::string &full_service_name) 
    : ROSServiceServerProxyBase(full_service_name)
  {
    // Add required interfaces corresponding to the namespace tokens
    RTT::ServiceRequester *required = requester;
    for(std::vector<std::string>::iterator it = service_ns_tokens_.begin();
        it != service_ns_tokens_.end();
        ++it)
    {
      required = required->requires(*it);
    }

    // Create the operation caller with the name of the ROS service
    operation_caller_.reset(new ProxyOperationCallerType(service_name_));

    // Add the operation caller to the required interface
    required->addOperationCaller(*operation_caller_.get());

    // Construct the ROS service server
    ros::NodeHandle nh;
    server_ = nh.advertiseService(
        full_service_name_, 
        &ROSServiceServerProxy<ROS_SERVICE_T, ROS_SERVICE_REQ_T, ROS_SERVICE_RESP_T>::ros_service_callback, 
        this);
  }

private:

  //! The Orocos operation caller which gets called 
  boost::shared_ptr<ProxyOperationCallerType> operation_caller_;
  
  //! The callback called by the ROS service server when this service is invoked
  bool ros_service_callback(ROS_SERVICE_REQ_T& request, ROS_SERVICE_RESP_T& response) {
    return operation_caller_.ready() && operation_caller_(request, response);
  }
};


//! Abstract ROS Service Proxy Client
class ROSServiceClientProxyBase : public ROSServiceProxyBase
{
public:
  ROSServiceClientProxyBase(const std::string &service_name) : ROSServiceProxyBase(service_name) { }
private:
  //! The underlying ROS service client
  ros::ServiceClient client_;
};

template<class ROS_SERVICE_T, class ROS_SERVICE_REQ_T, class ROS_SERVICE_RESP_T>
class ROSServiceClientProxy : public ROSServiceClientProxyBase 
{
public:

  ROSServiceClientProxy(const std::string &service_name) 
    : ROSServiceClientProxyBase(service_name)
      proxy_("ROS_SERVICE_CLIENT_PROXY")
  {
    // Construct the underlying service client
    ros::NodeHandle nh;
    client_ = nh.serviceClient<ROS_SERVICE_T>(service_name);

    // Link the operation with the service client
    proxy_.calls(
        &ROSServiceClientProxy<ROS_SERVICE_T,ROS_SERVICE_REQ_T,ROS_SERVICE_RESP_T>::orocos_operation_callback,
        this,
        RTT::ClientThread);
  }

  //! Connect an operation caller with this proxy
  bool connect(RTT::TaskContext *owner, RTT::base::OperationCallerBaseInvoker* operation_caller) {
    // Link the caller with the operation
    return operaiton_caller->setImplementation(
        proxy_.getImplementation(),
        owner->engine())
  }

private:

  //! The callback called by an Orocos operation caller
  bool orocos_operation_callback(ROS_SERVICE_REQ_T& request, ROS_SERVICE_RESP_T& response) {
    return client_.exists() && client_.isValid() && client_.call(request, response);
  }

  //! The RTT operation
  RTT::Operation<bool(ROS_SERVICE_REQ_T&, ROS_SERVICE_RESP_T&)> proxy_;
};



//! Abstract factory for ROS Service Proxy Factories
class ROSServiceProxyFactoryBase 
{
public:
  const std::string& getType() { return service_type_; }

  virtual ROSServiceClientProxyBase* create_client_proxy(RTT::base::OperationCallerBaseInvoker* operation_caller, const std::string &service_name) = 0;
  virtual ROSServiceServerProxyBase* create_server_proxy(RTT::TaskContext* task, const std::string &service_name) = 0;

private:
  ROSServiceProxyFactoryBase(const std::string &service_type) :
    service_type_(service_type)
  { }

  std::string service_type_;
};

template<class ROS_SERVICE_T, class ROS_SERVICE_REQ_T, class ROS_SERVICE_RESP_T>
class ROSServiceProxyFactory : public ROSServiceProxyFactoryBase 
{
  public:
    ROSServiceProxyFactory(const std::string &service_type)
      : ROSServiceProxyFactoryBase(service_type)
    { }

  virtual ROSServiceClientProxyBase*
    create_client_proxy(
        const std::string &service_name) 
    {
      return new ROSServiceClientProxy<ROS_SERVICE_T, ROS_SERVICE_REQ_T, ROS_SERVICE_RESP_T>(operation_caller, service_name);
    }

    virtual ROSServiceServerProxyBase* create_server_proxy(
        RTT::TaskContext* task,
        const std::string &service_name) 
    {
      return new ROSServiceServerProxy<ROS_SERVICE_T, ROS_SERVICE_REQ_T, ROS_SERVICE_RESP_T>(task, service_name);
    }
};

#endif // ifndef __RTT_ROSSERVICE_ROS_SERVICE_PROXY_H
