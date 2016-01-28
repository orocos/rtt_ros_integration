
#include <rtt_roscomm/rtt_rostopic.h>

RTT::ConnPolicy rtt_roscomm::topic(const std::string& name) {
  RTT::ConnPolicy cp = RTT::ConnPolicy::data();
  cp.transport = protocol_id;
  cp.name_id = name;
  cp.init=false;
  return cp;
}

RTT::ConnPolicy rtt_roscomm::topicLatched(const std::string& name) {
  RTT::ConnPolicy cp = RTT::ConnPolicy::data();
  cp.transport = protocol_id;
  cp.name_id = name;
  cp.init = true;
  cp.pull = false;
  return cp;
}

/**
 * Returns a ConnPolicy object for streaming to or from 
 * the given ROS topic. Also specifies the buffer size of
 * the connection to be created.
 */
RTT::ConnPolicy rtt_roscomm::topicBuffer(const std::string& name, int size) {
  RTT::ConnPolicy cp = RTT::ConnPolicy::buffer(size);
  cp.transport = protocol_id;
  cp.name_id = name;
  cp.init=false;
  return cp;
}

/**
 * Returns a ConnPolicy object for streaming to or from
 * the given ROS topic. Use this only for unbuffered
 * publishing, where the publish() method is called
 * in the thread of the writing TaskContext.
 */
RTT::ConnPolicy rtt_roscomm::topicUnbuffered(const std::string& name) {
  RTT::ConnPolicy cp = RTT::ConnPolicy();
  cp.type = RTT::ConnPolicy::UNBUFFERED;
  cp.transport = protocol_id;
  cp.name_id = name;
  cp.init=false;
  return cp;
}

