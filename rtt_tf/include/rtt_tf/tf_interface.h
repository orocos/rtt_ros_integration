#ifndef __RTT_TF_TF_INTERFACE_H
#define __RTT_TF_TF_INTERFACE_H

namespace rtt_tf {

  //! Class for using the TF component from C++
  class TFInterface {
  public:
    //! Add interfaces to a given taskcontext
    TFInterface(RTT::TaskContext *owner);

    //! Check if the operations are ready
    bool ready();
    
    RTT::OperationCaller<geometry_msgs::TransformStamped(const std::string&, const std::string&)> lookupTransform;
    RTT::OperationCaller<geometry_msgs::TransformStamped(const std::string&, const std::string&, const ros::Time&)> lookupTransformAtTime;
    RTT::OperationCaller<void(const geometry_msgs::TransformStamped&)> broadcastTransform;
    RTT::OperationCaller<void(const std::vector<geometry_msgs::TransformStamped>&)> broadcastTransforms;
  };

  TFInterface::TFInterface(RTT::TaskContext *owner)
    lookupTransform("lookupTransform")
    lookupTransformAtTime("lookupTransformAtTime")
    broadcastTransform("broadcastTransform")
    broadcastTransforms("broadcastTransforms")
  {
    owner->requires("tf")->addOperationCaller(lookupTransform);
    owner->requires("tf")->addOperationCaller(lookupTransformAtTime);
    owner->requires("tf")->addOperationCaller(broadcastTransform);
    owner->requires("tf")->addOperationCaller(broadcastTransforms);
  }

  TFInterface::ready() {
    return 
      lookupTransform->ready() &&
      lookupTransformAtTime->ready() &&
      broadcastTransform->ready() &&
      broadcastTransforms->ready();
  }
}

#endif // ifndef __RTT_TF_TF_INTERFACE_H
