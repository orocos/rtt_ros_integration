

#include <ros/ros.h>
#include <ros/time.h>
#include <rtt/os/main.h>
#include <rtt/RTT.hpp>
#include <ocl/DeploymentComponent.hpp>
#include <ocl/TaskBrowser.hpp>
#include "../rtt_tf-component.hpp"
#include <geometry_msgs/TransformStamped.h>

class LookupComponent : public RTT::TaskContext {
public:
  RTT::OperationCaller
    <geometry_msgs::TransformStamped(const std::string &, const std::string &, const ros::Time &)> lookup_;
  RTT::OperationCaller
    <geometry_msgs::TransformStamped(const std::string &, const std::string &)> lookup_now_;
  RTT::OperationCaller
    <bool(const std::string &, const std::string &)> subscribe_transform_;
  geometry_msgs::TransformStamped tform_, tform_now_;

  LookupComponent(const std::string &name) :
    RTT::TaskContext(name, RTT::TaskContext::PreOperational)
    ,lookup_("lookupTransformAtTime")
    ,lookup_now_("lookupTransform")
    ,subscribe_transform_("subscribeTransform")
  {
    this->addProperty("tform",tform_);
    this->addProperty("tform_now",tform_now_);
    this->requires("tf")->addOperationCaller(lookup_);
    this->requires("tf")->addOperationCaller(lookup_now_);
    this->requires("tf")->addOperationCaller(subscribe_transform_);
  }
  virtual ~LookupComponent()  { }
  bool configureHook() { 
    subscribe_transform_("rel_rtt_tf_test","rtt_tf_test");
    return true;
  }
  bool startHook() {
    return true;
  }
  void updateHook() {
    try{
      tform_ = lookup_("/world","/rtt_tf_test",ros::Time::now()-ros::Duration(0.1));
      tform_now_ = lookup_now_("/world","rel_rtt_tf_test");
    } catch(...) {
      
    }
  }
  void stopHook() { }
};

ORO_CREATE_COMPONENT(LookupComponent);

int ORO_main(int argc, char** argv) {

  LookupComponent lookup("lookup");

  {
    // Create deployer
    OCL::DeploymentComponent deployer;

    // initialize ROS and load typekits
    deployer.import("rtt_rosnode");
    deployer.import("rtt_std_msgs");
    deployer.import("rtt_geometry_msgs");
    deployer.import("rtt_tf2_msgs");

    // import and load our component
    deployer.import("rtt_tf");
    deployer.loadComponent("tf","rtt_tf::RTT_TF");

    RTT::TaskContext *tf_component = deployer.myGetPeer("tf");

    // Connect components and deployer
    deployer.connectPeers(&lookup);
    deployer.setActivity("lookup",0.02,0,ORO_SCHED_OTHER);
    deployer.setActivity("tf",0.1,0,ORO_SCHED_OTHER);

    // Connect services between the tf component and the broadcaster
    lookup.connectServices(tf_component);

    // Configure and start the components
    tf_component->configure();
    tf_component->start();

    lookup.configure();
    lookup.start();
    
    // Create orocos taskbrowser for easy interaction
    OCL::TaskBrowser task_browser(&deployer);
    task_browser.loop();

    lookup.stop();
  }


  return 0;
}

