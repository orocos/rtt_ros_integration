// required for ROS_STATIC_ASSERT(), not included in <std_msgs/builtin_string.h>
#include <ros/assert.h>

#include <std_msgs/builtin_bool.h>
#include <std_msgs/builtin_double.h>
#include <std_msgs/builtin_float.h>
#include <std_msgs/builtin_int8.h>
#include <std_msgs/builtin_int16.h>
#include <std_msgs/builtin_int32.h>
#include <std_msgs/builtin_int64.h>
#include <std_msgs/builtin_string.h>
#include <std_msgs/builtin_uint8.h>
#include <std_msgs/builtin_uint16.h>
#include <std_msgs/builtin_uint32.h>
#include <std_msgs/builtin_uint64.h>

#include <std_msgs/Duration.h>
#include <std_msgs/Time.h>
#include <ros/time.h>

#include <std_msgs/vector_multi_array_adapter.h>

#include <rtt_roscomm/rtt_rostopic_ros_msg_transporter.hpp>
#include <rtt_roscomm/rostopic.h>
#include <rtt/types/TransportPlugin.hpp>
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/rt_string.hpp>

// There are no message_traits for ros::Time and ros::Duration, so we define it here.
STD_MSGS_DEFINE_BUILTIN_TRAITS(::ros::Duration, Duration, 0x3e286caf4241d664ULL, 0xe55f3ad380e2ae46ULL)
STD_MSGS_DEFINE_BUILTIN_TRAITS(::ros::Time, Time, 0xcd7166c74c552c31ULL, 0x1fbcc2fe5a7bc289ULL)

// Adapt std::vector<double> to std_msgs/Float64MultiArray
namespace rtt_roscomm {

  template <class ContainerAllocator>
  struct RosMessageAdapter<std::vector<double, ContainerAllocator> >
  {
    typedef std::vector<double, ContainerAllocator> OrocosType;
    typedef std_msgs::VectorMultiArrayAdapter<double, ContainerAllocator> RosType;
    static RosType toRos(const OrocosType &t) { return RosType(t); }
    static const OrocosType &fromRos(const RosType &t) { return *t; }
  };

} // namespace rtt_roscomm


namespace rtt_std_msgs {
  using namespace RTT;
  using rtt_roscomm::RosMsgTransporter;

  struct ROSPrimitivesPlugin
    : public types::TransportPlugin
  {
    bool registerTransport(std::string name, types::TypeInfo* ti)
    {
      if (name == "array") { return ti->addProtocol(ORO_ROS_PROTOCOL_ID, new RosMsgTransporter<std::vector<double> >());} else
      if (name == "bool") { return ti->addProtocol(ORO_ROS_PROTOCOL_ID, new RosMsgTransporter<bool>());} else
      if (name == "duration") { return ti->addProtocol(ORO_ROS_PROTOCOL_ID, new RosMsgTransporter<ros::Duration>());} else
      if (name == "float32") { return ti->addProtocol(ORO_ROS_PROTOCOL_ID, new RosMsgTransporter<float>());} else
      if (name == "float64") { return ti->addProtocol(ORO_ROS_PROTOCOL_ID, new RosMsgTransporter<double>());} else
      if (name == "int8") { return ti->addProtocol(ORO_ROS_PROTOCOL_ID, new RosMsgTransporter<int8_t>());} else
      if (name == "int16") { return ti->addProtocol(ORO_ROS_PROTOCOL_ID, new RosMsgTransporter<int16_t>());} else
      if (name == "int32") { return ti->addProtocol(ORO_ROS_PROTOCOL_ID, new RosMsgTransporter<int32_t>());} else
      if (name == "int64") { return ti->addProtocol(ORO_ROS_PROTOCOL_ID, new RosMsgTransporter<int64_t>());} else
      if (name == "string") { return ti->addProtocol(ORO_ROS_PROTOCOL_ID, new RosMsgTransporter<std::string>());} else
      if (name == "rt_string") { return ti->addProtocol(ORO_ROS_PROTOCOL_ID, new RosMsgTransporter<RTT::rt_string>());} else
      if (name == "time") { return ti->addProtocol(ORO_ROS_PROTOCOL_ID, new RosMsgTransporter<ros::Time>());} else
      if (name == "uint8") { return ti->addProtocol(ORO_ROS_PROTOCOL_ID, new RosMsgTransporter<uint8_t>());} else
      if (name == "uint16") { return ti->addProtocol(ORO_ROS_PROTOCOL_ID, new RosMsgTransporter<uint16_t>());} else
      if (name == "uint32") { return ti->addProtocol(ORO_ROS_PROTOCOL_ID, new RosMsgTransporter<uint32_t>());} else
      if (name == "uint64") { return ti->addProtocol(ORO_ROS_PROTOCOL_ID, new RosMsgTransporter<uint64_t>());} else
      { }
      return false;
    }

    std::string getTransportName() const {
        return "ros";
    }

    std::string getTypekitName() const {
        return "ros-primitives";
    }
    std::string getName() const {
        return "rtt-ros-primitives-transport";
    }

  };
}

ORO_TYPEKIT_PLUGIN( rtt_std_msgs::ROSPrimitivesPlugin )
