#include <boost/algorithm/string.hpp>

#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>

#include <rtt_roscomm/rtt_rosservice_registry_service.h>
#include <rtt_roscomm/rtt_rosservice_proxy.h>

using namespace RTT;
using namespace std;

/**
 * The globally loadable ROS service.
 */
class ROSServiceService : public RTT::Service 
{
public:
  /**
   * Instantiates this service.
   * @param owner The owner or null in case of global.
   */
  ROSServiceService(TaskContext* owner) 
    : Service("rosservice", owner)
  {
    if(owner) {
      this->doc("RTT Service for connecting the operations of "+owner->getName()+" to ROS service clients and servers.");
    }

    this->addOperation("connect", &ROSServiceService::connect, this)
      .doc( "Connects an RTT operation or operation caller to an associated ROS service server or client.")
      .arg( "operation_name", "The RTT operation name (like \"some_provided_service.another.operation\").")
      .arg( "service_name", "The ROS service name (like \"/my_robot/ns/some_service\").")
      .arg( "service_type", "The ROS service type (like \"std_srvs/Empty\").");
    this->addOperation("disconnect", &ROSServiceService::disconnect, this)
      .doc( "Disconnects an RTT operation or operation caller from an associated ROS service server or client.")
      .arg( "service_name", "The ROS service name (like \"/my_robot/ns/some_service\").");
    this->addOperation("disconnectAll", &ROSServiceService::disconnectAll, this)
      .doc( "Disconnects all RTT operations and operation callers from associated ROS service servers or clients.");

    // Get the global ros service registry
    rosservice_registry_ = ROSServiceRegistryService::Instance();
    has_service_factory = rosservice_registry_->getOperation("hasServiceFactory");
    get_service_factory = rosservice_registry_->getOperation("getServiceFactory");
  }

  ~ROSServiceService()
  {
    disconnectAll();
  }

  //! Get an RTT operation caller from a string identifier
  RTT::base::OperationCallerBaseInvoker* get_owner_operation_caller(const std::string rtt_uri)
  {
    // Split up the service uri
    std::vector<std::string> rtt_uri_tokens;
    boost::split(rtt_uri_tokens, rtt_uri, boost::is_any_of("."));

    // Make sure the uri has at least one token
    if(rtt_uri_tokens.size() < 1) {
      return NULL;
    }

    // Iterate through the tokens except for the last one (the operation name)
    boost::shared_ptr<RTT::ServiceRequester> required = this->getOwner()->requires();
    for(std::vector<std::string>::iterator it = rtt_uri_tokens.begin();
        it+1 != rtt_uri_tokens.end();
        ++it)
    {
      if(required->requiresService(*it)) {
        required = required->requires(*it);
      } else {
        return NULL;
      }
    }

    // Get the operation caller
    return required->getOperationCaller(rtt_uri_tokens.back());
  }

  //! Get an RTT operation from a string identifier
  RTT::OperationInterfacePart* get_owner_operation(const std::string rtt_uri)
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
        it+1 != rtt_uri_tokens.end();
        ++it)
    {
      if(provided->hasService(*it)) {
        provided = provided->provides(*it);
      } else {
        return NULL;
      }
    }

    // Get the operation 
    return provided->getOperation(rtt_uri_tokens.back());
  }

  /** \brief Connect an RTT operation or operation caller to a ROS service
   * server or service client.
   */
  bool connect(
    const std::string &rtt_operation_name,
    const std::string &ros_service_name,
    const std::string &ros_service_type)
  {
    // Make sure the factory for this service type exists
    if(!this->has_service_factory(ros_service_type)) {
      RTT::log(RTT::Error) << "Unknown service type '" << ros_service_type << "'" << RTT::endlog();
      return false;
    }

    // Check if the operation is required by the owner
    RTT::base::OperationCallerBaseInvoker* 
      operation_caller = this->get_owner_operation_caller(rtt_operation_name);

    if(operation_caller) {
      // Check if the client proxy already exists
      if(client_proxies_.find(ros_service_name) == client_proxies_.end()) {
        // Create a new client proxy
        client_proxies_[ros_service_name] =
          get_service_factory(ros_service_type)->create_client_proxy(ros_service_name);
      }

      // Associate an RTT operation caller with a ROS service client
      if (!client_proxies_[ros_service_name]->connect(this->getOwner(), operation_caller)) {
        RTT::log(RTT::Error) << "Could not connect OperationCaller '" << rtt_operation_name << "' to ROS service client '" << ros_service_name << "'" << RTT::endlog();
        return false;
      }
      return true;
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
      if (!server_proxies_[ros_service_name]->connect(this->getOwner(), operation)) {
        RTT::log(RTT::Error) << "Could not connect Operation '" << rtt_operation_name << "' to ROS service server '" << ros_service_name << "'" << RTT::endlog();
        return false;
      }
      return true;
    }

    RTT::log(RTT::Error) << "No such Operation or OperationCaller '" << rtt_operation_name << "' in '" << getOwner()->getName() << "'" << RTT::endlog();
    return false;
  }

  bool disconnect(const std::string &ros_service_name)
  {
    bool found = false;

    // Cleanup ROS service or client named ros_service_name
    std::map<std::string, ROSServiceServerProxyBase*>::iterator iter_s
        = server_proxies_.find(ros_service_name);
    if (iter_s != server_proxies_.end()) {
      delete iter_s->second;
      server_proxies_.erase(iter_s);
      found = true;
    }

    std::map<std::string, ROSServiceClientProxyBase*>::iterator iter_c
        = client_proxies_.find(ros_service_name);
    if (iter_c != client_proxies_.end())
    {
      delete iter_c->second;
      client_proxies_.erase(iter_c);
      found = true;
    }

    return found;
  }

  void disconnectAll()
  {
    // Cleanup registered ROS services and clients
    std::map<std::string, ROSServiceServerProxyBase*>::iterator iter_s;
    for(iter_s = server_proxies_.begin(); iter_s != server_proxies_.end(); iter_s = server_proxies_.begin())
    {
      delete iter_s->second;
      server_proxies_.erase(iter_s);
    }

    std::map<std::string, ROSServiceClientProxyBase*>::iterator iter_c;
    for(iter_c = client_proxies_.begin(); iter_c != client_proxies_.end(); iter_c = client_proxies_.begin())
    {
      delete iter_c->second;
      client_proxies_.erase(iter_c);
    }
  }

  RTT::Service::shared_ptr rosservice_registry_;
  RTT::OperationCaller<bool(const std::string&)> has_service_factory;
  RTT::OperationCaller<ROSServiceProxyFactoryBase*(const std::string&)> get_service_factory;

  std::map<std::string, ROSServiceServerProxyBase*> server_proxies_;
  std::map<std::string, ROSServiceClientProxyBase*> client_proxies_;
};

ORO_SERVICE_NAMED_PLUGIN(ROSServiceService, "rosservice")
