#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>

#include <rtt_rosbuild_tests/typekit/String.h>
#include <geometry_msgs/typekit/Pose.h>

namespace rtt_rosbuild_tests {

class Component : public RTT::TaskContext
{
public:
    Component(const std::string& name)
        : RTT::TaskContext(name)
    {
        this->addPort("string", port_string);
        this->addPort("pose", port_pose);
    }

    void updateHook()
    {
        rtt_rosbuild_tests::String string;
        string.data = "Hello from " ROS_PACKAGE_NAME "!";
        port_string.write(string);
    }

private:
    RTT::OutputPort<rtt_rosbuild_tests::String> port_string;
    RTT::InputPort<geometry_msgs::Pose> port_pose;
};

} // namespace rtt_rosbuild_tests

ORO_CREATE_COMPONENT(rtt_rosbuild_tests::Component)
