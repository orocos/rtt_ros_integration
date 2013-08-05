
#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/internal/GlobalService.hpp>

#include <rtt_rosservice/ros_service_proxy.h> 

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
      this->addOperation("connect", &ROSServiceService::connect, this)
        .doc( "Connects an RTT operation or operation caller to an associated ROS service server or client.")
        .arg( "operation_name", "The RTT operation name (like \"some_provided_service.another.operation\").")
        .arg( "service_name", "The ROS service name (like \"/my_robot/ns/some_service\").")
        .arg( "service_type", "The ROS service type (like \"std_srvs/Empty\").");
    } else {
      this->doc("Global RTT Service for registering ROS service types.");
      this->addOperation("registerServiceType", &ROSServiceService::registerServiceType, this);
    }
  }

  /** \brief Register a ROS service proxy factory
   *
   * This enables the ROSServiceService to construct ROS service clients and
   * servers from a string name.
   */
  void registerServiceType(ROSServiceProxyFactoryBase* factory) 
  {
    os::MutexLock lock(factory_lock_);
    // Store the factory
    factories_[factory->getType()] = factory;
  }

  /** \brief Add an Orocos operation to this service which calls a ROS service
   * client.
   *
   * This allows another component to call this operation in order to invoke a
   * ROS service call.
   */

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
    RTT::ServiceRequester *required = this->getOwner()->requires();
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
    RTT::Service::shared_ptr *provided = this->getOwner()->provides();
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

  bool connectOperation(
    const std::string &rtt_operation_name,
    const std::string &ros_service_name,
    const std::string &ros_service_type)
  {
    // Get the fectory lock
    os::MutexLock lock(factory_lock_);

    // Make sure the factory for this service type exists
    if(factories_.find(ros_service_type) == factories_.end()) {
      return false; 
    }

    // Check if the operation is required by the owner
    RTT::base::OperationCallerBaseInvoker* operation_caller = 
      this->get_owner_operation_caller(rtt_operation_name);

    if(operation_caller) {
      ROSServiceClientProxyBase* client_proxy = NULL;

      // Check if the client proxy already exists
      if(client_proxies_.find(ros_service_name) != client_proxies_.end()) {
        // Get the existing client proxy
        client_proxy = client_proxies_[ros_service_name];
      } else {
        // Create a new client proxy
        client_proxy = factories_[ros_service_type]->create_client_proxy(ros_service_name);
        
        // Store the client proxy
        client_proxies_[ros_service_name] = client_proxy;
      }

      // Associate an RTT operation caller with a ROS service client
      return client_proxy.connect(this, operation_caller);
    }
    
    // Check if the operation is provided by the owner
    RTT::OperationInterfacePart* operation = 
      this->get_owner_operation(rtt_operation_name);
    
    if(operation) {
      ROSServiceServerProxyBase* server_proxy = NULL;

      // Check if the server proxy already exists
      if(server_proxies_.find(ros_service_name) != server_proxies_.end()) {
        // Get the existing server proxy
        server_proxy = server_proxies_[ros_service_name];
      } else {
        // Create a new server proxy
        server_proxy = factories_[ros_service_type]->create_server_proxy(ros_service_name);
        
        // Store the server proxy
        server_proxies_[ros_service_name] = server_proxy;
      }

      // Associate an RTT operation caller with a ROS service client
      return client_proxy.connect(this, operation);

      return true;
    }

    return false;
  }

  /** \brief Add an Orocos operation caller to this service which is called by
   * a ROS service server.
   *
   * This allows another component to be called by this service when a ROS
   * service call is invoked.
   */
  const std::string server(
      const std::string &ros_service_name
      const std::string &ros_service_type)
  {
    // Check if the server proxy already exists
    if(server_proxies_.find(ros_service_name) != server_proxies_.end()) {
      return server_proxies_[ros_service_name]->getOperationName();
    }
        
    // Split the typename
    std::vector<std::string> type_tokens;
    boost::split(type_tokens, ros_service_type, boost::is_any_of("/"));
    if(type_tokens.size() != 2) { return "ERROR"; }

    // Have the factory create a ROS service server add the operation 
    ROSServiceServerProxyBase* server_proxy =
      factories_[type_tokens[0]][type_tokens[1]]->create_server_proxy(
          this->requires("servers"), ros_service_name);
    // Store the server proxy
    server_proxies_[ros_service_name] = server_proxy;

    return this->getName() + ".servers." + server_proxy->getOperationName();
  }

  //! ROS service proxy factories
  std::map<std::string, std::map<std::string, ROSServiceProxyFactoryBase*> > factories_;
  RTT::os::Mutex factory_lock_;

  std::map<std::string, ROSServiceServerProxyBase*> server_proxies_;
  std::map<std::string, ROSServiceClientProxyBase*> client_proxies_;


};

void loadROSServiceService()
{
  RTT::Service::shared_ptr rss(new ROSServiceService(0));
  RTT::internal::GlobalService::Instance()->addService(rss);
}

using namespace RTT;
extern "C" {
  bool loadRTTPlugin(RTT::TaskContext* c){
    loadROSServiceService();
    return true;
  }
  std::string getRTTPluginName (){
    return "rosservice";
  }
  std::string getRTTTargetName (){
    return OROCOS_TARGET_NAME;
  }
}
