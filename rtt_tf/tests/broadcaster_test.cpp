
#include <ros/ros.h>
#include <ros/time.h>
#include <rtt/os/main.h>
#include <rtt/RTT.hpp>
#include <ocl/DeploymentComponent.hpp>
#include <ocl/TaskBrowser.hpp>
#include "../rtt_tf-component.hpp"
#include <geometry_msgs/TransformStamped.h>

class BroadcasterComponent : public RTT::TaskContext {
public:
  RTT::OperationCaller
    <void(const geometry_msgs::TransformStamped &)> broadcaster_;
  geometry_msgs::TransformStamped tform_;

  BroadcasterComponent(const std::string &name) :
    RTT::TaskContext(name, RTT::TaskContext::PreOperational)
    ,broadcaster_("broadcastTransform")
  {
    this->requires("tf")->addOperationCaller(broadcaster_);
  }
  virtual ~BroadcasterComponent()  { }
  bool configureHook() { 
    return true;
  }
  bool startHook() {
    tform_.transform.translation.x = 0;
    tform_.transform.translation.y = 0;
    tform_.transform.translation.z = 0;
    tform_.transform.rotation.x = 0;
    tform_.transform.rotation.y = 0;
    tform_.transform.rotation.z = 0;
    tform_.transform.rotation.w = 1;

    return true;
  }
  void updateHook() {
    RTT::os::TimeService::Seconds t = RTT::os::TimeService::Instance()->secondsSince(0);
    tform_.header.frame_id = "/world";
    tform_.header.stamp = ros::Time(((double)RTT::os::TimeService::Instance()->getNSecs())*1E-9);
    tform_.child_frame_id = "/rtt_tf_test";
    tform_.transform.translation.x = cos(t);
    tform_.transform.translation.y = sin(t);
    broadcaster_(tform_);
    tform_.header.frame_id = "/world";
    tform_.header.stamp = ros::Time(((double)RTT::os::TimeService::Instance()->getNSecs())*1E-9);
    tform_.child_frame_id = "rel_rtt_tf_test";
    tform_.transform.translation.x = cos(t);
    tform_.transform.translation.y = sin(t);
    broadcaster_(tform_);
  }
  void stopHook() { }
};

ORO_CREATE_COMPONENT(BroadcasterComponent);

int ORO_main(int argc, char** argv) {

  BroadcasterComponent bcaster("bcaster");

  {
    // Create deployer
    OCL::DeploymentComponent deployer;

    deployer.import("rtt_tf");
    deployer.loadComponent("tf","rtt_tf::RTT_TF");

    RTT::TaskContext *tf_component = deployer.myGetPeer("tf");

    // Connect components and deployer
    deployer.connectPeers(&bcaster);
    deployer.setActivity("bcaster",0.02,0,ORO_SCHED_OTHER);

    // Connect services between the tf component and the broadcaster
    bcaster.connectServices(tf_component);

    // Configure and start the components
    tf_component->configure();
    tf_component->start();

    bcaster.configure();
    bcaster.start();
    
    // Create orocos taskbrowser for easy interaction
    OCL::TaskBrowser task_browser(&deployer);
    task_browser.loop();
  }


  bcaster.stop();

  return 0;
}

