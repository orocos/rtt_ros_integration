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

  ROSServiceProxyBase(const std::string &full_service_name) 
    : full_service_name_(full_service_name)
  {
    // Split the full service name into (service_ns, service_name)
    boost::iterator_range<std::string::const_iterator> service_ns_end = boost::algorithm::find_last(full_service_name_, "/");
    service_ns_.assign(full_service_name.begin(), service_ns_end.begin());
    service_name_.assign(service_ns_end.begin()+1, full_service_name.end());

    // Get the service namespace tokens
    boost::split(service_ns_tokens_, service_ns_, boost::is_any_of("/"));
    
    // Store the operation name
    operation_name_tokens_.assign(service_ns_tokens_.begin(), service_ns_tokens_.end());
    operation_name_tokens_.push_back(service_name_);
    operation_name_ = boost::algorithm::join(operation_name_tokens_, ".");
  }

  //! Get the name of the Orocos operation
  const std::string& getOperationName() const {
    return operation_name_;
  }

private:

  //! ROS Service name (fully qualified)
  std::string full_service_name_;
  std::string service_ns_;
  std::string service_name_;
  std::vector<std::string> service_ns_tokens_;

  std::string operation_name_;
  std::vector<std::string> operation_name_tokens_;
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
  ROSServiceClientProxyBase(const std::string &full_service_name) : ROSServiceProxyBase(full_service_name) { }
private:
  //! The underlying ROS service client
  ros::ServiceClient client_;
};

template<class ROS_SERVICE_T, class ROS_SERVICE_REQ_T, class ROS_SERVICE_RESP_T>
class ROSServiceClientProxy : public ROSServiceClientProxyBase 
{
public:

  ROSServiceClientProxy(RTT::Service* provider, const std::string &full_service_name) 
    : ROSServiceClientProxyBase(full_service_name) 
  {
    // Construct the underlying service client
    ros::NodeHandle nh;
    client_ = nh.serviceClient<ROS_SERVICE_T>(full_service_name_);

    // Add provided interfaces corresponding to the namespace tokens
    RTT::Service *provided = provider;
    for(std::vector<std::string>::iterator it = service_ns_tokens_.begin();
        it != service_ns_tokens_.end();
        ++it)
    {
      provided = provided->provides(*it);
    }

    // Add the Orocos Operation to the provided interface
    provided->addOperation(service_name_,
        &ROSServiceClientProxy<ROS_SERVICE_T,ROS_SERVICE_REQ_T,ROS_SERVICE_RESP_T>::orocos_operation_callback,
        this,
        RTT::ClientThread);
  }

private:

  //! The callback called by an Orocos operation caller
  bool orocos_operation_callback(ROS_SERVICE_REQ_T& request, ROS_SERVICE_RESP_T& response)
  {
    return client_.exists() && client_.isValid() && client_.call(request, response);
  }
};



//! Abstract factory for ROS Service Proxy Factories
class ROSServiceProxyFactoryBase 
{
public:
  const std::string& getPackage() { return package_name_; }
  const std::string& getType() { return service_type_; }

  virtual ROSServiceClientProxyBase* create_client_proxy(RTT::TaskContext* task, const std::string &service_name) = 0;
  virtual ROSServiceServerProxyBase* create_server_proxy(RTT::TaskContext* task, const std::string &service_name) = 0;

private:
  ROSServiceProxyFactoryBase(const std::string &package_name, const std::string &service_type) :
    package_name_(package_name),
    service_type_(service_type)
  { }

  std::string package_name_;
  std::string service_type_;
};

template<class ROS_SERVICE_T, class ROS_SERVICE_REQ_T, class ROS_SERVICE_RESP_T>
class ROSServiceProxyFactory : public ROSServiceProxyFactoryBase 
{
  public:
    ROSServiceProxyFactory(const std::string &package_name, const std::string &service_type)
      : ROSServiceProxyFactoryBase(package_name, service_type)
    { }
    virtual ROSServiceClientProxyBase* create_client_proxy(
        RTT::TaskContext* task,
        const std::string &service_name) 
    {
      return new ROSServiceClientProxy<ROS_SERVICE_T, ROS_SERVICE_REQ_T, ROS_SERVICE_RESP_T>(task, service_name);
    }
    virtual ROSServiceServerProxyBase* create_server_proxy(
        RTT::TaskContext* task,
        const std::string &service_name) 
    {
      return new ROSServiceServerProxy<ROS_SERVICE_T, ROS_SERVICE_REQ_T, ROS_SERVICE_RESP_T>(task, service_name);
    }
};

#endif // ifndef __RTT_ROSSERVICE_ROS_SERVICE_PROXY_H
