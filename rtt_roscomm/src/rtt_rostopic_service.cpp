#include <rtt/RTT.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <rtt_rostopic/rtt_rostopic.h> 

using namespace RTT;
using namespace std;

/**
 * The globally loadable ROS service.
 */
class ROSTopicService {
public:
  static const int protocol_id;

  /**
   * Returns a ConnPolicy object for streaming to or from 
   * the given ROS topic. No buffering is done.
   */
  static ConnPolicy topic(const std::string& name) {
    ConnPolicy cp = ConnPolicy::data();
    cp.transport = protocol_id;
    cp.name_id = name;
    return cp;
  }

  /**
   * Returns a ConnPolicy object for streaming to or from 
   * the given ROS topic. Also specifies the buffer size of
   * the connection to be created.
   */
  static ConnPolicy topicBuffer(const std::string& name, int size) {
    ConnPolicy cp = ConnPolicy::buffer(size);
    cp.transport = protocol_id;
    cp.name_id = name;
    return cp;
  }

  /**
   * Returns a ConnPolicy object for streaming to or from
   * the given ROS topic. Use this only for unbuffered
   * publishing, where the publish() method is called
   * in the thread of the writing TaskContext.
   */
  static ConnPolicy topicUnbuffered(const std::string& name) {
    ConnPolicy cp = ConnPolicy();
    cp.type = ConnPolicy::UNBUFFERED;
    cp.transport = protocol_id;
    cp.name_id = name;
    return cp;
  }};

const int ROSTopicService::protocol_id = ORO_ROS_PROTOCOL_ID;

void loadROSTopicService(){
  RTT::Service::shared_ptr ros = RTT::internal::GlobalService::Instance()->provides("ros");

  // ROS Topic-based Operations
  ros->addOperation("topic", &ROSTopicService::topic).doc(
      "Creates a ConnPolicy for subscribing to or publishing a topic. No buffering is done, only the last message is kept.").arg(
          "name", "The ros topic name");
  ros->addOperation("topicBuffer", &ROSTopicService::topicBuffer).doc(
      "Creates a ConnPolicy for subscribing to or publishing a topic with a fixed-length message buffer.").arg(
          "name", "The ros topic name").arg(
          "size","The size of the buffer.");
  ros->addOperation("topicUnbuffered", &ROSTopicService::topicUnbuffered).doc(
      "Creates a ConnPolicy for unbuffered publishing a topic. This may not be real-time safe!").arg(
          "name", "The ros topic name");
  ros->addConstant("protocol_id", ROSTopicService::protocol_id);
}

using namespace RTT;
extern "C" {
  bool loadRTTPlugin(RTT::TaskContext* c){
    if (c != 0) return false;
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
