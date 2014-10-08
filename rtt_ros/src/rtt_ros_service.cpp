#include <cstdlib>
#include <list>
#include <queue>
#include <sstream>
#include <set>

#include <boost/filesystem.hpp>
#include <boost/version.hpp>

#include <libxml/parser.h>
#include <libxml/xpath.h>
#include <libxml/xpathInternals.h>
#include <libxml/tree.h>

#include <rtt/RTT.hpp>
#include <rtt/internal/GlobalService.hpp>

#include <rtt/deployment/ComponentLoader.hpp>
#include <rtt/Logger.hpp>

#include <ros/names.h>
#include <ros/this_node.h>
#include <rospack/rospack.h>

#include <rtt_ros/rtt_ros.h>

void loadROSService(){
  RTT::Service::shared_ptr ros_service = RTT::internal::GlobalService::Instance()->provides("ros");

  ros_service->doc("RTT service for loading RTT plugins ");

  // ROS Package-importing
  ros_service->addOperation("import", &rtt_ros::import).doc(
      "Imports the Orocos plugins from a given ROS package (if found) along with the plugins of all of the package's run or exec dependencies as listed in the package.xml.").arg(
          "package", "The ROS package name.");

  // Namespace manipulation
  ros_service->addOperation("resolveName", &rtt_ros::resolveName).doc(
      "Resolves a ros name based on the active namespace.");

  ros_service->addOperation("resetNS", &rtt_ros::resetNS).doc(
      "Resets the global namespace to the original namespace");

  ros_service->addOperation("setNS", &rtt_ros::setNS).doc(
      "Set the global namespace for all namespaced ROS operations").
    arg("ns", "The new namespace.");

  ros_service->addOperation("getNS", &rtt_ros::getNS).doc(
      "Get the global namespace for all namespaced ROS operations");

  ros_service->addOperation("pushNS", &rtt_ros::pushNS).doc(
      "Append to the global namespace for all namespaced ROS operations").
    arg("ns", "The new sub-namespace.");

  ros_service->addOperation("popNS", &rtt_ros::popNS).doc(
      "Remove the last namespace extension which was pushed.");
}

using namespace RTT;
extern "C" {
  bool loadRTTPlugin(RTT::TaskContext* c){
    if (c != 0) return false;
    loadROSService();
    return true;
  }
  std::string getRTTPluginName (){
    return "ros";
  }
  std::string getRTTTargetName (){
    return OROCOS_TARGET_NAME;
  }
}
