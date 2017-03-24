#include <rtt/RTT.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <rtt_roscomm/rostopic.h>

using namespace RTT;
using namespace std;

void loadROSTopicService(){
  RTT::Service::shared_ptr ros = RTT::internal::GlobalService::Instance()->provides("ros");
  RTT::Service::shared_ptr roscomm = ros->provides("comm");

  // New topic construction operators
  roscomm->addConstant("protocol_id", rtt_roscomm::protocol_id);

  roscomm->addOperation("topic", &rtt_roscomm::topic).doc(
      "Creates a ConnPolicy for subscribing to or publishing a topic. No buffering is done, only the last message is kept.").arg(
          "name", "The ros topic name");

  roscomm->addOperation("topicBuffer", &rtt_roscomm::topicBuffer).doc(
      "Creates a ConnPolicy for subscribing to or publishing a topic with a fixed-length message buffer.").arg(
          "name", "The ros topic name").arg(
          "size","The size of the buffer.");

  roscomm->addOperation("topicUnbuffered", &rtt_roscomm::topicUnbuffered).doc(
      "Creates a ConnPolicy for unbuffered publishing a topic. This may not be real-time safe!").arg(
          "name", "The ros topic name");

  // Backwards-compatibility
  ros->addConstant("protocol_id", rtt_roscomm::protocol_id);

  ros->addOperation("topic", &rtt_roscomm::topic).doc(
      "Creates a ConnPolicy for subscribing to or publishing a topic. No buffering is done, only the last message is kept.").arg(
          "name", "The ros topic name");
  ros->addOperation("topicBuffer", &rtt_roscomm::topicBuffer).doc(
      "Creates a ConnPolicy for subscribing to or publishing a topic with a fixed-length message buffer.").arg(
          "name", "The ros topic name").arg(
          "size","The size of the buffer.");
  ros->addOperation("topicUnbuffered", &rtt_roscomm::topicUnbuffered).doc(
      "Creates a ConnPolicy for unbuffered publishing a topic. This may not be real-time safe!").arg(
          "name", "The ros topic name");
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
