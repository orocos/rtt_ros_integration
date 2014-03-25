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

#include <rospack/rospack.h>

#include <rtt_ros/rtt_ros.h>

void loadROSService(){
  RTT::Service::shared_ptr ros = RTT::internal::GlobalService::Instance()->provides("ros");

  ros->doc("RTT service for loading RTT plugins ");

  // ROS Package-importing
  ros->addOperation("import", &rtt_ros::import).doc(
      "Imports the Orocos plugins from a given ROS package (if found) along with the plugins of all of the package's run or exec dependencies as listed in the package.xml.").arg(
          "package", "The ROS package name.");
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
