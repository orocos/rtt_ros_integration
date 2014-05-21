#include <rtt/RTT.hpp>
#include <rtt/internal/GlobalService.hpp>

#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt_rosclock/rtt_rosclock_sim_clock_thread.h>

#include <rtt/os/StartStopManager.hpp>

namespace {
  boost::shared_ptr<rtt_rosclock::SimClockThread> sim_clock_thread;
}

void unloadROSClockService() {
  sim_clock_thread.reset();
}

void loadROSClockService(){
  RTT::Service::shared_ptr rosclock = RTT::internal::GlobalService::Instance()->provides("ros")->provides("clock");

  rosclock->doc("RTT service for realtime and non-realtime clock measurement.");

  // Create sim time thread
  sim_clock_thread = rtt_rosclock::SimClockThread::Instance();
  RTT::os::StartStopManager::Instance()->stopFunction(&unloadROSClockService);

  // Getting current time 
  rosclock->addOperation("host_now", &rtt_rosclock::host_now).doc(
      "Get a ros::Time structure based on the NTP-corrected RT time or the ROS simulation time.");
  rosclock->addOperation("host_wall_now", &rtt_rosclock::host_now).doc(
      "Get a ros::Time structure based on the NTP-corrected RT time or the ROS wall time.");
  rosclock->addOperation("rtt_now", &rtt_rosclock::rtt_now).doc(
      "Get a ros::Time structure based on the RTT time source.");
  rosclock->addOperation("rtt_wall_now", &rtt_rosclock::rtt_wall_now).doc(
      "Get a ros::Time structure based on the RTT wall clock time.");

  // Getting time offset
  rosclock->addOperation("host_offset_from_rtt", &rtt_rosclock::host_offset_from_rtt).doc(
      "Get the difference between the Orocos wall clock and the NTP-corrected wall clock in seconds (host_wall - rtt_wall).");

  // Setting the source for the simulation clock
  rosclock->addOperation("useROSClockTopic", &rtt_rosclock::use_ros_clock_topic).doc(
      "Use the ROS /clock topic source for updating simulation time.");
  rosclock->addOperation("useManualClock", &rtt_rosclock::use_manual_clock).doc(
      "Use a manual source for simulation time by calling updateSimClock.");

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
