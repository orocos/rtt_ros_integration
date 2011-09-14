#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <RosLib.hpp> // todo: put RosLib.hpp in include dir of rtt_rosnode package

using namespace RTT;
using namespace std;

/**
 * The globally loadable ROS service.
 */
class ROSService : public RTT::Service {
public:
    int protocol_id;
    /**
     * Instantiates this service.
     * @param owner The owner or null in case of global.
     */
    ROSService(TaskContext* owner) 
        : Service("ros", owner),
          protocol_id(ORO_ROS_PROTOCOL_ID)
    {
        this->doc("The main ROS helper service for RTT. See also the 'rosparam' service which can be added to a component and the 'rospack' global service for finding ros packages.");
        // stream("Simulation.ctrl", ros.topic("/cmd_vel") )
        this->addOperation("topic", &ROSService::topic, this).doc("Creates a ConnPolicy for subscribing to or publishing a topic. No buffering is done, only the last sample is kept.").arg("name", "The ros topic name");
        this->addOperation("topicBuffer", &ROSService::topicBuffer, this).doc("Creates a ConnPolicy for subscribing to or publishing a topic.").arg("name", "The ros topic name").arg("size","The size of the buffer.");
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
};

void loadROSService(){
  RTT::Service::shared_ptr rts(new ROSService(0));
  RTT::internal::GlobalService::Instance()->addService(rts);
}
