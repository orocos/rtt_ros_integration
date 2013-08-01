
#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <rtt_rostopic/rtt_rostopic.h> 

////////////////////////////////////////////////////////////////////////////////
#include <std_srvs/Empty.h>
////////////////////////////////////////////////////////////////////////////////


class ROSServiceProxyBase
{
public:

  ROSServiceProxyBase(const std::string &full_service_name) 
    : full_service_name_(full_service_name)
  {
    // Split the full service name into (service_ns, service_name)
    boost::iterator_range<std::string::const_iterator> service_ns_end = boost::algorithm::find_last(full_service_name_, "/");
    service_ns_(full_service_name.begin(), service_ns_end.begin());
    service_name_(service_ns_end.begin()+1, full_service_name.end());

    // Get the service namespace tokens
    boost::split(service_ns_tokens_, service_ns_, boost::is_any_of("/"));
    
    // Store the operation name
    std::vector<std::string> operation_name_tokens(service_ns_tokens_);
    operation_name_tokens.push_front(task->getName());
    operation_name_tokens.push_back(service_name_);
    operation_name_ = boost::algorithm::join(operation_name_tokens, ".");
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
  
  //! Orocos operation name
  std::string operation_name_;
}

class ROSServiceServerProxyBase : public ROSServiceProxyBase
{ 
public:
  ROSServiceServerProxyBase(const std::string &full_service_name)
    : ROSServiceProxyBase(full_service_name);
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
  ROSServiceClientProxy(RTT::TaskContext* task, const std::string &full_service_name) 
    : ROSServiceServerProxyBase(full_service_name)
  {
    // Add required interfaces corresponding to the namespace tokens
    RTT::ServiceRequester *required = task->requires();
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

class ROSServiceClientProxyBase : public ROSServiceProxyBase
{
private:
  //! The underlying ROS service client
  ros::ServiceClient client_;
};

template<class ROS_SERVICE_T, class ROS_SERVICE_REQ_T, class ROS_SERVICE_RESP_T>
class ROSServiceClientProxy : public ROSServiceClientProxyBase 
{
public:

  ROSServiceClientProxy(RTT::TaskContext* task, const std::string &service_name) {
    ros::NodeHandle nh;
    
  }

private:
  RTT::Operation<bool(ROS_SERVICE_REQ_T&, ROS_SERVICE_RESP_T&)> operation_;
};


class ROSServiceProxyFactoryBase {
public:
    virtual bool create_client_proxy(RTT::TaskContext* task, const std::string &service_name) = 0;
    virtual bool create_server_proxy(RTT::TaskContext* task, const std::string &service_name) = 0;
};

template<class ROS_SERVICE_T, class ROS_SERVICE_REQ_T, class ROS_SERVICE_RESP_T>
class ROSServiceProxyFactory : public ROSServiceProxyFactoryBase {
  public:
    virtual ROSServiceClientProxyBase* create_client_proxy(RTT::TaskContext* task, const std::string &service_name) {
      return new ROSServiceClientProxy<ROS_SERVICE_T, ROS_SERVICE_REQ_T, ROS_SERVICE_RESP_T>(task, service_name);
    }
    virtual ROSServiceClientProxyBase* create_server_proxy(RTT::TaskContext* task, const std::string &service_name) {
      return new ROSServiceServerProxy<ROS_SERVICE_T, ROS_SERVICE_REQ_T, ROS_SERVICE_RESP_T>(task, service_name);
    }
};


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
