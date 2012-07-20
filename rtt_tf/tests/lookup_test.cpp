

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
    <geometry_msgs::TransformStamped(const std::string &, const std::string &)> lookup_;
  geometry_msgs::TransformStamped tform_;

  LookupComponent(const std::string &name) :
    RTT::TaskContext(name, RTT::TaskContext::PreOperational)
    ,lookup_("lookupTransform")
  {
    this->addProperty("tform",tform_);
    this->requires("tf")->addOperationCaller(lookup_);
  }
  virtual ~LookupComponent()  { }
  bool configureHook() { 
    return true;
  }
  bool startHook() {
    return true;
  }
  void updateHook() {
    try{
      tform_ = lookup_("/world","/rtt_tf_test");
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

    deployer.import("rtt_tf");
    deployer.loadComponent("tf","rtt_tf::RTT_TF");

    RTT::TaskContext *tf_component = deployer.myGetPeer("tf");

    // Connect components and deployer
    deployer.connectPeers(&lookup);
    deployer.setActivity("lookup",0.02,0,ORO_SCHED_OTHER);

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

