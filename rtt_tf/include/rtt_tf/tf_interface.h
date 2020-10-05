#ifndef __RTT_TF_TF_INTERFACE_H
#define __RTT_TF_TF_INTERFACE_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <rtt/RTT.hpp>
#include <geometry_msgs/TransformStamped.h>

namespace rtt_tf {

  //! Class for using the TF component from C++
  class TFInterface {
  public:
    //! Add interfaces to a given taskcontext
    TFInterface(RTT::TaskContext *owner) :
      lookupTransform("lookupTransform"),
      lookupTransformAtTime("lookupTransformAtTime"),
      broadcastTransform("broadcastTransform"),
      broadcastTransforms("broadcastTransforms"),
      broadcastStaticTransform("broadcastStaticTransform"),
      broadcastStaticTransforms("broadcastStaticTransforms"),
      canTransform("canTransform"),
      canTransformAtTime("canTransformAtTime")
    {
      owner->requires("tf")->addOperationCaller(lookupTransform);
      owner->requires("tf")->addOperationCaller(lookupTransformAtTime);
      owner->requires("tf")->addOperationCaller(broadcastTransform);
      owner->requires("tf")->addOperationCaller(broadcastTransforms);
      owner->requires("tf")->addOperationCaller(broadcastStaticTransform);
      owner->requires("tf")->addOperationCaller(broadcastStaticTransforms);
      owner->requires("tf")->addOperationCaller(canTransform);
      owner->requires("tf")->addOperationCaller(canTransformAtTime);
    }

    //! Check if the operations are ready
    bool ready() {
      return 
        lookupTransform.ready() &&
        lookupTransformAtTime.ready() &&
        broadcastTransform.ready() &&
        broadcastTransforms.ready() &&
        broadcastStaticTransform.ready() &&
        broadcastStaticTransforms.ready() &&
        canTransform.ready() &&
        canTransformAtTime.ready();
    }

    RTT::OperationCaller<geometry_msgs::TransformStamped(const std::string&, const std::string&)> lookupTransform;
    RTT::OperationCaller<geometry_msgs::TransformStamped(const std::string&, const std::string&, const ros::Time&)> lookupTransformAtTime;
    RTT::OperationCaller<void(const geometry_msgs::TransformStamped&)> broadcastTransform;
    RTT::OperationCaller<void(const std::vector<geometry_msgs::TransformStamped>&)> broadcastTransforms;
    RTT::OperationCaller<void(const geometry_msgs::TransformStamped&)> broadcastStaticTransform;
    RTT::OperationCaller<void(const std::vector<geometry_msgs::TransformStamped>&)> broadcastStaticTransforms;
    RTT::OperationCaller<bool(const std::string&, const std::string&)> canTransform;
    RTT::OperationCaller<bool(const std::string&, const std::string&, const ros::Time&)> canTransformAtTime;
  };
}

#endif // ifndef __RTT_TF_TF_INTERFACE_H
