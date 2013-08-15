
#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/internal/GlobalService.hpp>

#include <rtt_rosservice/ros_service_proxy.h> 

using namespace RTT;
using namespace std;

/**
 * The globally loadable ROS service.
 */
class ActionlibService : public RTT::Service 
{
public:
  /**
   * Instantiates this service.
   * @param owner The owner or null in case of global.
   */
  ActionlibService(TaskContext* owner) 
    : Service("actionlib", owner)
  {
    if(owner) {
      this->doc("RTT Service for connecting the operations of to ROS actionlib actions.");
    }

    this->addOperation("connect", &ActionlibService::connect, this)
      .doc( "Connects an RTT operation or operation caller to an associated ROS service server or client.")
      .arg( "operation_name", "The RTT operation name (like \"some_provided_service.another.operation\").")
      .arg( "service_name", "The ROS service name (like \"/my_robot/ns/some_service\").")
      .arg( "service_type", "The ROS service type (like \"std_srvs/Empty\").");

    // Get the global ros service registry
    //rosservice_registry_ = RTT::internal::GlobalService::Instance()->getService("rosservice_registry");
    //has_service_factory = rosservice_registry_->getOperation("hasServiceFactory");
    //get_service_factory = rosservice_registry_->getOperation("getServiceFactory");
  }

  //! Get an RTT service from a string identifier
  RTT::Service::shared_ptr get_owner_service(const std::string rtt_uri)
  {
    // Split up the service uri
    std::vector<std::string> rtt_uri_tokens;
    boost::split(rtt_uri_tokens, rtt_uri, boost::is_any_of("."));

    // Make sure the uri has at least one token
    if(rtt_uri_tokens.size() < 1) {
      return NULL;
    }

    // Iterate through the tokens except for the last one (the operation name)
    RTT::Service::shared_ptr provided = this->getOwner()->provides();
    for(std::vector<std::string>::iterator it = rtt_uri_tokens.begin();
        it != rtt_uri_tokens.end();
        ++it)
    {
      if(provided->hasService(*it)) {
        provided = provided->provides(*it);
      } else {
        return NULL;
      }
    }

    // Get the operation 
    return provided;
  }

  /** \brief Connect an RTT operation or operation caller to a ROS service
   * server or service client.
   */
  bool connect(
    const std::string &rtt_service_name,
    const std::string &ros_action_ns)
  {
    // Check if the operation is required by the owner
    RTT::Service::shared_ptr action_interface_service = this->get_owner_service(rtt_service_name);
    
    // Make sure the factory for this service type exists
    if(!rtt_actionlib::has_action_server_ports(action_interface_service)) {
      return false; 
    }

    return this->connect(action_interface_service, ros_action_ns);
  }

  bool connect(
      RTT::Service::shared_ptr action_interface_service,
      const std::string &ros_action_ns)
  {
    // Make sure the service is good and owned by this service's owner
    if(action_interface_service.get() == NULL || action_interface_service->getOwner() != this->getOwner()) {
      return false;
    }

    if(operation_caller) {
      // Check if the client proxy already exists
      if(client_proxies_.find(ros_service_name) == client_proxies_.end()) {
        // Create a new client proxy
        client_proxies_[ros_service_name] =
          get_service_factory(ros_service_type)->create_client_proxy(ros_service_name);
      }

      // Associate an RTT operation caller with a ROS service client
      return client_proxies_[ros_service_name]->connect(this->getOwner(), operation_caller);
    }
    
    // Check if the operation is provided by the owner
    RTT::OperationInterfacePart*
      operation = this->get_owner_operation(rtt_operation_name);
    
    if(operation) {
      // Check if the server proxy already exists
      if(server_proxies_.find(ros_service_name) == server_proxies_.end()) {
        // Create a new server proxy
        server_proxies_[ros_service_name] =
          get_service_factory(ros_service_type)->create_server_proxy(ros_service_name);
      }

      // Associate an RTT operation with a ROS service server 
      return server_proxies_[ros_service_name]->connect(this->getOwner(), operation);
    }

    return false;
  }

  RTT::Service::shared_ptr rosservice_registry_;
  RTT::OperationCaller<bool(const std::string&)> has_service_factory;
  RTT::OperationCaller<ROSServiceProxyFactoryBase*(const std::string&)> get_service_factory;

  std::map<std::string, ROSServiceServerProxyBase*> server_proxies_;
  std::map<std::string, ROSServiceClientProxyBase*> client_proxies_;
};


ORO_SERVICE_NAMED_PLUGIN(ROSServiceService, "rosservice")

