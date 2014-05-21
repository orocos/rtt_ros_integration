
#include <rtt/RTT.hpp>
#include <rtt/plugin/Plugin.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/internal/GlobalService.hpp>

#include <rtt_ros_msgs/RunScript.h>
#include <rtt_ros_msgs/GetPeerList.h>

#include <ocl/DeploymentComponent.hpp>

#include <ros/ros.h>

using namespace RTT;
using namespace std;

class ROSDeploymentService : public RTT::Service 
{
public:
  ROSDeploymentService(OCL::DeploymentComponent* deployer);

private:
  OCL::DeploymentComponent *deployer_;

  ros::NodeHandle nh_;

  ros::ServiceServer run_script_service_;
  ros::ServiceServer get_peer_list_service_;

  bool run_script_cb(
      rtt_ros_msgs::RunScript::Request& request,
      rtt_ros_msgs::RunScript::Response& response);

  bool get_peer_list_cb(
      rtt_ros_msgs::GetPeerList::Request& request,
      rtt_ros_msgs::GetPeerList::Response& response);
};


ROSDeploymentService::ROSDeploymentService(OCL::DeploymentComponent* deployer) :
  Service("rosdeployment", static_cast<RTT::TaskContext*>(deployer)),
  deployer_(deployer),
  nh_("~"+deployer->getName())
{
  if(deployer_) {
    // Create services
    run_script_service_ = nh_.advertiseService("run_script",&ROSDeploymentService::run_script_cb,this);
    get_peer_list_service_ = nh_.advertiseService("get_peer_list",&ROSDeploymentService::get_peer_list_cb,this);
  } else {
    RTT::log(RTT::Error) << "Attempted to load the rosdeployment service on a TaskContext which is not an OCL::DeploymentComponent. No ROS services will be advertised." << RTT::endlog();
  }
}

bool ROSDeploymentService::run_script_cb(
    rtt_ros_msgs::RunScript::Request& request,
    rtt_ros_msgs::RunScript::Response& response)
{
  response.success = deployer_->runScript(request.file_path);
  return true;
}

bool ROSDeploymentService::get_peer_list_cb(
    rtt_ros_msgs::GetPeerList::Request& request,
    rtt_ros_msgs::GetPeerList::Response& response)
{
  response.peers = deployer_->getPeerList();
  return true;
}

bool loadROSDeploymentService(RTT::TaskContext *tc) {
  OCL::DeploymentComponent *deployer = dynamic_cast<OCL::DeploymentComponent*>(tc);

  if(!deployer) {
    RTT::log(RTT::Error) << "The rosdeployment service must be loaded on a valid OCL::DeploymentComponent" <<RTT::endlog();
    return false; 
  }

  deployer->import("rtt_rosnode");

  if(!ros::isInitialized()) {
    RTT::log(RTT::Error) << "The rtt_rosdeployment plugin cannot be used without the rtt_rosnode plugin. Please load rtt_rosnode." << RTT::endlog();

    return false;
  }

  RTT::Service::shared_ptr sp( new ROSDeploymentService( deployer ) ); 
  return tc->provides()->addService( sp ); 
}

extern "C" {
  RTT_EXPORT bool loadRTTPlugin(RTT::TaskContext* tc);  
  bool loadRTTPlugin(RTT::TaskContext* tc) {    
    if(tc == 0) return true;
    return loadROSDeploymentService(tc);
  } 
  RTT_EXPORT RTT::Service::shared_ptr createService();  
  RTT::Service::shared_ptr createService() {    
    RTT::Service::shared_ptr sp; 
    return sp; 
  } 
  RTT_EXPORT std::string getRTTPluginName(); 
  std::string getRTTPluginName() { 
    return "rosdeployment"; 
  } 
  RTT_EXPORT std::string getRTTTargetName(); 
  std::string getRTTTargetName() { 
    return OROCOS_TARGET_NAME; 
  } 
}

