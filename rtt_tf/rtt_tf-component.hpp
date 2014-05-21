#ifndef OROCOS_RTT_TF_COMPONENT_HPP
#define OROCOS_RTT_TF_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <tf/tf.h>
#include <tf/tfMessage.h>

namespace rtt_tf
{
  // Inherit from TaskContext and Transformer, the second is required in order to use tf_prefix
  class RTT_TF: public RTT::TaskContext, protected tf::Transformer
  {
    static const int DEFAULT_BUFFER_SIZE = 100;

    boost::shared_ptr<tf::Transformer> m_transformer;
    double prop_cache_time;
    double prop_buffer_size;
    std::string prop_tf_prefix;

    RTT::InputPort<tf::tfMessage> port_tf_in;
    RTT::OutputPort<tf::tfMessage> port_tf_out;

    bool canTransform(
        const std::string& parent,
        const std::string& child);

    geometry_msgs::TransformStamped lookupTransform(
        const std::string& parent,
        const std::string& child);

    geometry_msgs::TransformStamped lookupTransformAtTime(
        const std::string& parent,
        const std::string& child,
        const ros::Time& common_time);

    void broadcastTransform(
        const geometry_msgs::TransformStamped &tform);

    void broadcastTransforms(
        const std::vector<geometry_msgs::TransformStamped> &tforms);

    void addTFOperations(RTT::Service::shared_ptr service);

  public:
    RTT_TF(std::string const& name);

    bool configureHook();

    bool startHook(){return true;};

    void updateHook();

    void stopHook(){};

    void cleanupHook(){};
  };
}//namespace rtt_tf
#endif
