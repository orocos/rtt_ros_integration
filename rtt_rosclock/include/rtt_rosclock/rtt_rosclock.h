#ifndef __RTT_ROSCLOCK_RTT_ROSCLOCK_H
#define __RTT_ROSCLOCK_RTT_ROSCLOCK_H

#include <rtt/RTT.hpp>
#include <rtt/os/TimeService.hpp>
#include <ros/time.h>

namespace rtt_rosclock {

  /** \brief Get the current time according to CLOCK_HOST_REALTIME or the
   * simulation time.
   *
   * This is the time source that should always be used with ROS header
   * timestamps because it is the time that you want to use to broadcast ROS
   * messages to other machines or processes. 
   *
   * When compiled against Xenomai and not running in simulation mode,
   * this function will return the NTP-synchronized clock time via the
   * CLOCK_HOST_REALTIME clock source. Note that this is only supported under
   * Xenomai 2.6 and above.
   *
   * When not compiled against Xenomai and not running in simulation mode, it
   * is a pass-through to ros::Time::now().
   *
   * When running in simulation mode, this will always use the simulation
   * clock, which is based off of the ROS /clock topic. It is a pass-through to
   * rtt_now().
   */
  const ros::Time host_now();

  /** \brief Get the current time according to CLOCK_HOST_REALTIME or the
   * wall time.
   */
  const ros::Time host_wall_now();

  /** \brief Get the current time according to RTT
   *
   * If the simulation clock is enabled, this will return the simulated time.
   */
  const ros::Time rtt_now();
  
  /** \brief Get the current wall time according to RTT
   *
   * Even if the simualtion clock is enabled, this will still return the wall
   * clock time.
   */
  const ros::Time rtt_wall_now();

  //! Get the difference in seconds between rtt_wall_now() and host_wall_now()
  const RTT::Seconds host_offset_from_rtt();

  //! Set a TaskContext to use a periodic simulation clock activity
  const bool set_sim_clock_activity(RTT::TaskContext *t);

  //! Use ROS /clock topic for time measurement
  void use_ros_clock_topic();

  //! Use manual clock updates
  void use_manual_clock();

  //! Use a simulated clock source
  const bool enable_sim();
  
  //! Do't use a simulated clock source
  const bool disable_sim();

  //! Update the current simulation time and trigger all simulated TaskContexts
  void update_sim_clock(const ros::Time new_time);
}

#endif // ifndef __RTT_ROSCLOCK_RTT_ROSCLOCK_H
