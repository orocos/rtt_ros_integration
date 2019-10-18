#ifndef __RTT_ROSCOMM_RTT_ROSSERVICE_PROXY_H
#define __RTT_ROSCOMM_RTT_ROSSERVICE_PROXY_H

#include <ros/ros.h>

#include <rtt/RTT.hpp>
#include <rtt/internal/GlobalEngine.hpp>
#include <rtt/plugin/ServicePlugin.hpp>

#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_void.hpp>

namespace rtt_roscomm {

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
    ROSServiceProxyBase(service_name)
  { }

  //! Connect an RTT Operation to this ROS service server
  virtual bool connect(RTT::TaskContext *owner, RTT::OperationInterfacePart* operation) = 0;

protected:
  //! The underlying ROS service server
  ros::ServiceServer server_;
};

template<class ROS_SERVICE_T>
class ROSServiceServerOperationCallerBase {
public:
  typedef boost::shared_ptr<ROSServiceServerOperationCallerBase<ROS_SERVICE_T> > Ptr;
  virtual ~ROSServiceServerOperationCallerBase() {}
  virtual bool call(typename ROS_SERVICE_T::Request& request, typename ROS_SERVICE_T::Response& response) const = 0;
};

template<class ROS_SERVICE_T, int variant = 0>
struct ROSServiceServerOperationCallerWrapper {
  typedef void ProxyOperationCallerType;
};

// Default implementation of an OperationCaller that fowards ROS service calls to Orocos operations
// that have the default bool(Request&, Response&) signature. You can add more variants of this class
// to add support for custom operation types.
//
// See package std_srvs for an example.
//
template<class ROS_SERVICE_T>
struct ROSServiceServerOperationCallerWrapper<ROS_SERVICE_T,0> {
  typedef bool Signature(typename ROS_SERVICE_T::Request&, typename ROS_SERVICE_T::Response&);
  typedef RTT::OperationCaller<Signature> ProxyOperationCallerType;
  template <typename Callable> static bool call(Callable& call, typename ROS_SERVICE_T::Request& request, typename ROS_SERVICE_T::Response& response) {
    return call(request, response);
  }
};

template<class ROS_SERVICE_T, int variant = 0>
class ROSServiceServerOperationCaller : public ROSServiceServerOperationCallerBase<ROS_SERVICE_T> {
public:
  typedef typename ROSServiceServerOperationCallerBase<ROS_SERVICE_T>::Ptr Ptr;

  //! The wrapper type for this variant
  typedef ROSServiceServerOperationCallerWrapper<ROS_SERVICE_T, variant> Wrapper;

  //! Default operation caller for a ROS service server proxy
  typedef typename Wrapper::ProxyOperationCallerType ProxyOperationCallerType;
  typedef boost::shared_ptr<ProxyOperationCallerType> ProxyOperationCallerTypePtr;

  static Ptr connect(RTT::OperationInterfacePart* operation);

  virtual bool call(typename ROS_SERVICE_T::Request& request, typename ROS_SERVICE_T::Response& response) const {
    // Check if the operation caller is ready, and then call it.
    if (!proxy_operation_caller_->ready()) return false;
    return Wrapper::call(*proxy_operation_caller_, request, response);
  }

private:
  ROSServiceServerOperationCaller(const boost::shared_ptr<ProxyOperationCallerType>& impl)
      : proxy_operation_caller_(impl) {}

  ProxyOperationCallerTypePtr proxy_operation_caller_;
};

namespace {

template<class ROS_SERVICE_T, int variant, typename Enabled = void>
struct ROSServiceServerOperationCallerWrapperNextVariant {
  typedef typename ROSServiceServerOperationCallerBase<ROS_SERVICE_T>::Ptr Ptr;
  static Ptr connect(RTT::OperationInterfacePart*) { return Ptr(); }
};

template<class ROS_SERVICE_T, int variant>
struct ROSServiceServerOperationCallerWrapperNextVariant<ROS_SERVICE_T, variant,
    typename boost::disable_if<boost::is_void<typename ROSServiceServerOperationCallerWrapper<ROS_SERVICE_T, variant + 1>::ProxyOperationCallerType> >::type> {
  typedef typename ROSServiceServerOperationCallerBase<ROS_SERVICE_T>::Ptr Ptr;
  static Ptr connect(RTT::OperationInterfacePart* operation) {
    return ROSServiceServerOperationCaller<ROS_SERVICE_T, variant + 1>::connect(operation);
  }
};

}

template<class ROS_SERVICE_T, int variant>
typename ROSServiceServerOperationCaller<ROS_SERVICE_T, variant>::Ptr
ROSServiceServerOperationCaller<ROS_SERVICE_T, variant>::connect(RTT::OperationInterfacePart* operation) {
  ProxyOperationCallerTypePtr proxy_operation_caller
      = boost::make_shared<ProxyOperationCallerType>(operation->getLocalOperation(), RTT::internal::GlobalEngine::Instance());
  if (proxy_operation_caller->ready()) {
    return Ptr(new ROSServiceServerOperationCaller<ROS_SERVICE_T, variant>(proxy_operation_caller));
  }
  return ROSServiceServerOperationCallerWrapperNextVariant<ROS_SERVICE_T, variant>::connect(operation);
}

template<class ROS_SERVICE_T>
class ROSServiceServerProxy : public ROSServiceServerProxyBase
{
public:
  /** \brief Construct a ROS service server and associate it with an Orocos
   * task's required interface and operation caller.
   */
  ROSServiceServerProxy(const std::string &service_name) :
    ROSServiceServerProxyBase(service_name)
  {
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

  virtual bool connect(RTT::TaskContext *, RTT::OperationInterfacePart* operation)
  {
    impl_ = ROSServiceServerOperationCaller<ROS_SERVICE_T>::connect(operation);
    return (impl_.get() != 0);
  }

private:
  //! The callback called by the ROS service server when this service is invoked
  bool ros_service_callback(typename ROS_SERVICE_T::Request& request, typename ROS_SERVICE_T::Response& response) {
    if (!impl_) return false;
    return impl_->call(request, response);
  }

  boost::shared_ptr<ROSServiceServerOperationCallerBase<ROS_SERVICE_T> > impl_;
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

}  // namespace rtt_roscomm

#endif // ifndef __RTT_ROSCOMM_RTT_ROSSERVICE_PROXY_H
