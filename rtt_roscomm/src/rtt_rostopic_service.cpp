#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <rtt_rostopic/rtt_rostopic.h> 

using namespace RTT;
using namespace std;

/**
 * The globally loadable ROS service.
 */
class ROSTopicService : public RTT::Service {
public:
  int protocol_id;
  /**
   * Instantiates this service.
   * @param owner The owner or null in case of global.
   */
  ROSTopicService(TaskContext* owner) 
    : Service("rostopic", owner),
    protocol_id(ORO_ROS_PROTOCOL_ID)
  {
    this->doc("Main RTT Service for connecting RTT ports to ROS message topics. See also the 'rosparam' service which can be added to a component and the 'rospack' global service for finding ros packages.");

    // ROS Package-importing

    // ROS Topic-based Operations
    this->addOperation("connection", &ROSTopicService::topic, this).doc(
        "Creates a ConnPolicy for subscribing to or publishing a topic. No buffering is done, only the last message is kept.").arg(
            "name", "The ros topic name");
    this->addOperation("bufferedConnection", &ROSTopicService::topicBuffer, this).doc(
        "Creates a ConnPolicy for subscribing to or publishing a topic with a fixed-length message buffer.").arg(
            "name", "The ros topic name").arg(
            "size","The size of the buffer.");
    this->addOperation("unbuffered", &ROSTopicService::topicUnbuffered, this).doc(
        "Creates a ConnPolicy for unbuffered publishing a topic. This may not be real-time safe!").arg(
            "name", "The ros topic name");
    this->addConstant("protocol_id", protocol_id );

  }

  /**
   * Returns a ConnPolicy object for streaming to or from 
   * the given ROS topic. No buffering is done.
   */
  ConnPolicy topic(const std::string& name) {
    ConnPolicy cp = ConnPolicy::data();
    cp.transport = ORO_ROS_PROTOCOL_ID;
    cp.name_id = name;
    return cp;
  }

  /**
   * Returns a ConnPolicy object for streaming to or from 
   * the given ROS topic. Also specifies the buffer size of
   * the connection to be created.
   */
  ConnPolicy topicBuffer(const std::string& name, int size) {
    ConnPolicy cp = ConnPolicy::buffer(size);
    cp.transport = ORO_ROS_PROTOCOL_ID;
    cp.name_id = name;
    return cp;
  }

  /**
   * Returns a ConnPolicy object for streaming to or from
   * the given ROS topic. Use this only for unbuffered
   * publishing, where the publish() method is called
   * in the thread of the writing TaskContext.
   */
  ConnPolicy topicUnbuffered(const std::string& name) {
    ConnPolicy cp = ConnPolicy();
    cp.type = ConnPolicy::UNBUFFERED;
    cp.transport = ORO_ROS_PROTOCOL_ID;
    cp.name_id = name;
    return cp;
  }};

void loadROSTopicService(){
  RTT::Service::shared_ptr rts(new ROSTopicService(0));
  RTT::internal::GlobalService::Instance()->addService(rts);
}

using namespace RTT;
extern "C" {
  bool loadRTTPlugin(RTT::TaskContext* c){
    loadROSTopicService();
    return true;
  }
  std::string getRTTPluginName (){
    return "rostopic";
  }
  std::string getRTTTargetName (){
    return OROCOS_TARGET_NAME;
  }
}
