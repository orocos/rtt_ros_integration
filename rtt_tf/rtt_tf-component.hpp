#ifndef OROCOS_RTT_TF_COMPONENT_HPP
#define OROCOS_RTT_TF_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <tf2_msgs/TFMessage.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>

#include <boost/shared_ptr.hpp>

namespace rtt_tf
{
  // Inherit from TaskContext and Transformer, the second is required in order to use tf_prefix
  class RTT_TF: public RTT::TaskContext, protected tf2::BufferCore
  {
    typedef boost::shared_ptr<RTT::OutputPort<geometry_msgs::TransformStamped> > OutputPortGeometryTransfromStampedPtr;
    typedef boost::shared_ptr<tf2_ros::TransformListener> TransformListenerPtr;
    typedef boost::shared_ptr<tf2::BufferCore> BufferCorePtr;

    static const int DEFAULT_BUFFER_SIZE = 100;

    double prop_cache_time;
    double prop_buffer_size;
    std::string prop_tf_prefix;

    RTT::InputPort<tf2_msgs::TFMessage> port_tf_in;
    RTT::InputPort<tf2_msgs::TFMessage> port_tf_static_in;
    RTT::OutputPort<tf2_msgs::TFMessage> port_tf_out;
    RTT::OutputPort<tf2_msgs::TFMessage> port_tf_static_out;
    std::map<std::pair<std::string, std::string>, OutputPortGeometryTransfromStampedPtr> ports_trackers;
    BufferCorePtr buffer_core;
    TransformListenerPtr transform_listener;

    void internalUpdate(
        tf2_msgs::TFMessage& msg,
        RTT::InputPort<tf2_msgs::TFMessage>& port,
        bool is_static);

    ros::Time getLatestCommonTime(
        const std::string& target,
        const std::string& source) const;

    bool canTransform(
        const std::string& target,
        const std::string& source) const;

    bool canTransformAtTime(
        const std::string& target,
        const std::string& source,
        const ros::Time& common_time) const;

    geometry_msgs::TransformStamped lookupTransform(
        const std::string& target,
        const std::string& source) const;

    geometry_msgs::TransformStamped lookupTransformAtTime(
        const std::string& target,
        const std::string& source,
        const ros::Time& common_time) const;

    void broadcastTransform(
        const geometry_msgs::TransformStamped &tform);

    void broadcastTransforms(
        const std::vector<geometry_msgs::TransformStamped> &tforms);

    void broadcastStaticTransform(
        const geometry_msgs::TransformStamped &tform);

    void broadcastStaticTransforms(
        const std::vector<geometry_msgs::TransformStamped> &tforms);

    void addTFOperations(RTT::Service::shared_ptr service);

    bool subscribeTransfrom(
        const std::string& target,
        const std::string& source);

    void listTrackers();

  public:
    RTT_TF(std::string const& name);

    bool configureHook();

    bool startHook();

    void updateHook();

    void stopHook();

    void cleanupHook();
  };
}//namespace rtt_tf
#endif
