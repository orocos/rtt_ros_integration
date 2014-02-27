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

void loadROSClockService(){
  RTT::Service::shared_ptr rosclock = RTT::internal::GlobalService::Instance()->provides("ros")->provides("clock");

  rosclock->doc("RTT service for realtime and non-realtime clock measurement.");

  // Create sim time thread
  sim_clock_thread = rtt_rosclock::SimClockThread::Instance();

  // Getting current time 
  rosclock->addOperation("rtt_now", &rtt_rosclock::rtt_now).doc(
      "Get a ros::Time structure based on the RTT time source.");
  rosclock->addOperation("ros_now", &rtt_rosclock::ros_now).doc(
      "Get a ros::Time structure based on the ROS time.");
  rosclock->addOperation("host_rt_now", &rtt_rosclock::host_rt_now).doc(
      "Get a ros::Time structure based on the NTP-corrected RT time. This is equivalent to the CLOCK_HOST_REALTIME clock source.");

  // Getting time offset
  rosclock->addOperation("host_rt_offset_from_rtt", &rtt_rosclock::host_rt_offset_from_rtt).doc(
      "Get the difference between the Orocos clock and the ROS clock in seconds (host_time - rtt_time).");

  // Enabling/Disabling simulation clock
  rosclock->addOperation("enable_sim", &rtt_rosclock::enable_sim).doc(
      "Enable simulation time based on the ROS /clock topic if the /use_sim_time parameter is set. This will override RTT::os::TimeService");
  rosclock->addOperation("disable_sim", &rtt_rosclock::disble_sim).doc(
      "Disable simulation time based on the ROS /clock topic.");
  rosclock->addOperation("setSimClockActivity", &rtt_rosclock::set_sim_clock_activity).doc(
      "Set a TaskContext's activity to a periodic activity driven by the ROS /clock topic.");
}

using namespace RTT;
extern "C" {
  bool loadRTTPlugin(RTT::TaskContext* c){
    if (c != 0) return false;
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
