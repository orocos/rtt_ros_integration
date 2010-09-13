#include <rtt/plugin/Plugin.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Activity.hpp>
#include <rtt/Logger.hpp>
#include <ros/ros.h>

using namespace RTT;
extern "C" {
  bool loadRTTPlugin(RTT::TaskContext* c){
    log(Info)<<"Initializing ROS node"<<endlog();
    if(!ros::isInitialized()){
      int argc=0;
      char* argv[0];
      ros::init(argc,argv,"rtt",ros::init_options::AnonymousName);
      if(ros::master::check())
          ros::start();
      else{
          log(Error)<<"No ros::master available"<<endlog();
          ros::shutdown();
          return false;
      }
    }
    static ros::AsyncSpinner spinner(1); // Use 1 threads
    spinner.start();
    log(Info)<<"ROS node spinner started"<<endlog();
    return true;
  }
  std::string getRTTPluginName (){
    return "ros_integration";
  }
  std::string getRTTTargetName (){
    return OROCOS_TARGET_NAME;
  }
}

