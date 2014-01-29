#include <cstdlib>
#include <list>
#include <queue>
#include <sstream>

#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/internal/GlobalService.hpp>

#include <rtt/deployment/ComponentLoader.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/os/StartStopManager.hpp>
#include <rtt/plugin/PluginLoader.hpp>
#include <rtt/types/TypekitRepository.hpp>

#include <rospack/rospack.h>

#include <rtt_rosclock/rtt_rosclock.h>

using namespace RTT;
using namespace std;

/**
 * The globally loadable ROS service.
 */
class ROSClockService : public RTT::Service {
public:
  /**
   * Instantiates this service.
   * @param owner The owner or null in case of global.
   */
  ROSClockService(TaskContext* owner) 
    : Service("rosclock", owner)
  {
    this->doc("RTT service for querying RTT vs Host time information.");

    this->addOperation("rtt_now", &rtt_rosclock::rtt_now).doc(
        "Get a ros::Time structure based on the RTT time source.");
    this->addOperation("ros_now", &rtt_rosclock::ros_now).doc(
        "Get a ros::Time structure based on the ROS time.");
    this->addOperation("host_rt_now", &rtt_rosclock::host_rt_now).doc(
        "Get a ros::Time structure based on the NTP-corrected RT time. This is equivalent to the CLOCK_HOST_REALTIME clock source.");

    this->addOperation("host_rt_offset_from_rtt", &rtt_rosclock::host_rt_offset_from_rtt).doc(
        "Get the difference between the Orocos clock and the ROS clock in seconds (host_time - rtt_time).");
  }
};

void loadROSClockService(){
  RTT::Service::shared_ptr rcs(new ROSClockService(0));
  RTT::internal::GlobalService::Instance()->addService(rcs);
}

using namespace RTT;
extern "C" {
  bool loadRTTPlugin(RTT::TaskContext* c){
    loadROSClockService();
    return true;
  }
  std::string getRTTPluginName (){
    return "rosclock";
  }
  std::string getRTTTargetName (){
    return OROCOS_TARGET_NAME;
  }
}
