
#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <rtt_rostopic/rtt_rostopic.h> 

using namespace RTT;
using namespace std;

/**
 * The globally loadable ROS service.
 */
class ROSServiceService : public RTT::Service {
public:
  /**
   * Instantiates this service.
   * @param owner The owner or null in case of global.
   */
  ROSTopicService(TaskContext* owner) 
    : Service("rosservice", owner)
  {
    this->doc("Main RTT Service for connecting RTT operations to ROS service clients and servers.");

    this->provides("client");
    this->provides("server");

    // ROS Topic-based Operations
    this->addOperation("client", &ROSServiceService::client, this).doc(
        "Creates a ConnPolicy for subscribing to or publishing a topic. No buffering is done, only the last message is kept.").arg(
            "name", "The ros topic name");
    this->addOperation("server", &ROSTopicService::server, this).doc(
        "Creates a ConnPolicy for subscribing to or publishing a topic with a fixed-length message buffer.").arg(
            "name", "The ros topic name");

  }


  //! Get an operation caller
  const std::string client(const std::string &ros_service_name) {
    // Convert 
    // create operationcaller
    // return string name of operationcaller
    return "";
  }

  //! Get an operation 
  const std::string server(const std::string &ros_service_name) {
    // create operation
    // return string name of  operation


    return "";
  }



};

void loadROSServiceService(){
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
