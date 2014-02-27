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
#include <rtt_rosclock/rtt_rosclock_sim_clock_thread.h>

using namespace RTT;
using namespace std;

namespace {
  boost::shared_ptr<rtt_rosclock::SimClockThread> sim_clock_thread;
}

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

  // Setting the source for the simulation clock
  rosclock->addOperation("useROSClockTopic", &rtt_rosclock::use_ros_clock_topic).doc(
      "Use the ROS /clock topic source for updating simulation time.");
  rosclock->addOperation("useManualClock", &rtt_rosclock::use_manual_clock).doc(
      "Use a manual source for simulation time by calling updateSimClock.");

  rosclock->addOperation("setSimClockActivity", &rtt_rosclock::set_sim_clock_activity).doc(
      "Set a TaskContext's activity to a periodic activity driven by the simulated clock.").arg(
          "task","A TaskContext which should be run periodically according to the simulation time.");

  // Enabling/Disabling simulation clock
  rosclock->addOperation("enableSimClock", &rtt_rosclock::enable_sim).doc(
      "Enable simulation time based on the ROS /clock topic if the /use_sim_time parameter is set. This will override RTT::os::TimeService");
  rosclock->addOperation("disableSimClock", &rtt_rosclock::disable_sim).doc(
      "Disable simulation time based on the ROS /clock topic.");

  rosclock->addOperation("updateSimClock", &rtt_rosclock::update_sim_clock).doc(
      "Update the current simulation time and update all SimClockActivities as per their respective frequencies.").arg(
          "time","Current simulated time in seconds.");
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
