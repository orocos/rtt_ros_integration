#ifndef OROCOS_RTT_TF_COMPONENT_HPP
#define OROCOS_RTT_TF_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <tf/tf.h>
#include <tf/tfMessage.h>

namespace rtt_tf
{
  class RTT_TF: public RTT::TaskContext
  {
    static const int DEFAULT_BUFFER_SIZE = 100;

    boost::shared_ptr<tf::Transformer> m_transformer;
    bool prop_interpolating;
    double prop_cache_time;
    double prop_buffer_size;

    RTT::InputPort<tf::tfMessage> port_tf_in;
    RTT::OutputPort<tf::tfMessage> port_tf_out;

    geometry_msgs::TransformStamped lookupTransform(
        const std::string& parent,
        const std::string& child);

    void broadcastTransform(
        const geometry_msgs::TransformStamped &tform);

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
