
#include <boost/algorithm/string.hpp>

#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/internal/GlobalService.hpp>

#include <rtt_roscomm/rostopic.h>

#include <rtt_actionlib/rtt_actionlib.h>

using namespace RTT;
using namespace std;

/**
 * This is an RTT service which automatically connects 
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
    this->doc("RTT Service for connecting RTT ports to ROS actionlib actions.");

    this->addOperation( "connect", 
        (bool (ActionlibService::*)(const std::string&))&ActionlibService::connect, this)
      .doc( "Connects a set of RTT data ports (goal,cancel,status,result,feedback) to a ROS actionlib action server or client.")
      .arg( "action_ns", "The ROS action namespace (like \"/some/action\").");

    this->addOperation( "connectSub", 
        (bool (ActionlibService::*)(const std::string&, const std::string&))&ActionlibService::connect, this)
      .doc( "Connects a set of RTT data ports (goal,cancel,status,result,feedback) defined on a sub-service to a ROS actionlib action server or client.")
      .arg( "service_name", "The RTT service name (like \"some_provided_service.another\") under which the ports are defined.")
      .arg( "action_ns", "The ROS action namespace (like \"/some/action\").");

  }

  //! Get an RTT service from a string identifier
  RTT::Service::shared_ptr get_owner_service(const std::string rtt_uri)
  {
    // Split up the service uri
    std::vector<std::string> rtt_uri_tokens;
    boost::split(rtt_uri_tokens, rtt_uri, boost::is_any_of("."));

    // Make sure the uri has at least one token
    if(rtt_uri_tokens.size() < 1) {
      return RTT::Service::shared_ptr();
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
        return RTT::Service::shared_ptr();
      }
    }

    // Get the operation 
    return provided;
  }

  /** \brief 
   */
  bool connect(
    const std::string &ros_action_ns)
  {
    return this->connect(this->getOwner()->provides(), ros_action_ns);
  }

  /** \brief Connect 
   */
  bool connect(
    const std::string &rtt_service_name,
    const std::string &ros_action_ns)
  {
    // Check if the operation is provided by the owner
    RTT::Service::shared_ptr rtt_action_service = this->get_owner_service(rtt_service_name);

    // Return false if the operation isn't provided
    if(rtt_action_service.get() == NULL) {
      return false;
    }
    
    return this->connect(rtt_action_service, ros_action_ns);
  }

  /** \brief Connect an RTT operation or operation caller to a ROS service
   * server or service client.
   */
  bool connect(
      RTT::Service::shared_ptr rtt_action_service,
      const std::string &ros_action_ns)
  {
    // Make sure the service is good and owned by this service's owner
    if(rtt_action_service.get() == NULL || rtt_action_service->getOwner() != this->getOwner()) {
      return false;
    }

    // Construct the action bridge
    rtt_actionlib::ActionBridge action_bridge;

    // Get existing ports from the service
    if(!action_bridge.setPortsFromService(rtt_action_service)) {
      return false;
    }

    // Try to connect the topics
    if(!action_bridge.createStream(ros_action_ns)) {
      return false; 
    }

    return true;
  }
};


ORO_SERVICE_NAMED_PLUGIN(ActionlibService, "actionlib")

