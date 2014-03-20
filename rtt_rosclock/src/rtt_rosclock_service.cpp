#include <rtt/RTT.hpp>
#include <rtt/internal/GlobalService.hpp>

#include <rtt_rosclock/rtt_rosclock.h>

void loadROSClockService(){
  RTT::Service::shared_ptr rosclock = RTT::internal::GlobalService::Instance()->provides("ros")->provides("clock");

  rosclock->doc("RTT service for realtime and non-realtime clock measurement.");

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
