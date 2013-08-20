#ifndef __RTT_ACTION_SERVER_H
#define __RTT_ACTION_SERVER_H

#include <rtt/RTT.hpp>
#include <rtt/os/Timer.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/internal/GlobalService.hpp>

#include <actionlib/action_definition.h>
#include <actionlib/server/action_server_base.h>

#include <rtt_actionlib/rtt_actionlib.h>

#include <rtt_ros/time.h>

namespace rtt_actionlib {

  // Forward declaration
  template<class ActionSpec>
  class RTTActionServer;

  //! Timer to trigger status publications
  template<class ActionSpec>
  class RTTActionServerStatusTimer : public RTT::os::Timer 
  {
  public:
    RTTActionServerStatusTimer(RTTActionServer<ActionSpec> &server) :
      RTT::os::Timer(1,ORO_SCHED_OTHER),
      server_(server) 
    { }

    //! Publish the action server status
    virtual void timeout(RTT::os::Timer::TimerId timer_id) {
      server_.publishStatus();
    }
  private:
    //! A reference to the owning action server
    RTTActionServer<ActionSpec> &server_;
  };

  //! Orocos RTT-Based Action Server
  template<class ActionSpec>
  class RTTActionServer : public actionlib::ActionServerBase<ActionSpec>
  {
  public:
    // Generates typedefs that make our lives easier
    ACTION_DEFINITION(ActionSpec);

    typedef actionlib::ServerGoalHandle<ActionSpec> GoalHandle;

    //! Constructor
    RTTActionServer(const double status_period = 0.200);
    virtual ~RTTActionServer();

    //! Add actionlib ports to a given rtt service
    bool addPorts(RTT::Service::shared_ptr service);

    //! Check if the server is ready to be started
    bool ready();

    //! \brief Set up status publishing timers
    virtual void initialize();

    //! Publishes a result for a given goal
    virtual void publishResult(const actionlib_msgs::GoalStatus& status, const Result& result);

    //! Publishes feedback for a given goal
    virtual void publishFeedback(const actionlib_msgs::GoalStatus& status, const Feedback& feedback);

    //! Explicitly publish status
    virtual void publishStatus();
    
  private:

    /* \brief Wrapper for action server base goalCallback
     * This function is called when messages arrive on the goal RTT port. It
     * then reads the port and calls ActionServerBase::goalCallback, which, in
     * turn, calls the user supplied goal callback.
     */
    void goalCallback(RTT::base::PortInterface* port);

    /* \brief Wrapper for action server base cancelCallback 
     * This function is called when messages arrive on the goal RTT port. It
     * then reads the port and calls ActionServerBase::cancelCallback, which, in
     * turn, calls the user supplied cancel callback.
     */
    void cancelCallback(RTT::base::PortInterface* port);

    //! Period (Hz) at which the status should be published
    double status_period_;

    //! Action bridge container for RTT ports corresponding to the action interface
    rtt_actionlib::ActionBridge action_bridge_;

    //! RTT Timer for periodic status updates
    RTTActionServerStatusTimer<ActionSpec> status_timer_;
  };

  template <class ActionSpec>
    RTTActionServer<ActionSpec>::RTTActionServer(const double status_period) :
      actionlib::ActionServerBase<ActionSpec>(boost::function<void (GoalHandle)>(), boost::function<void (GoalHandle)>(), false),
      status_period_(status_period),
      status_timer_(*this)
  {

  }

  template <class ActionSpec>
    RTTActionServer<ActionSpec>::~RTTActionServer()
    {

    }

  template <class ActionSpec>
    void RTTActionServer<ActionSpec>::initialize()
    {
      if(this->ready()) {
        // Start status publish timer 
        if(!status_timer_.startTimer(0,status_period_)) {
          RTT::log(RTT::Error) << "Failed to initialize RTT Action Server: could not start status publisher timer." << RTT::endlog();
        }
      } else {
        RTT::log(RTT::Error) << "Failed to initialize RTT Action Server: ports not ready." << RTT::endlog();
      }
    }

  template <class ActionSpec>
    bool RTTActionServer<ActionSpec>::ready() 
    {
      return action_bridge_.allConnected();
    }

  template <class ActionSpec>
    bool RTTActionServer<ActionSpec>::addPorts(
        RTT::Service::shared_ptr service)
    {
      // Try to get existing ports from service
      if(!action_bridge_.setPortsFromService(service)) {
        // Create the ports
        if(!action_bridge_.createServerPorts<ActionSpec>()) {
          return false;
        }
      }

      // Add the ports to the service
      service->addEventPort(
          action_bridge_.goalInput<ActionSpec>(), 
          boost::bind(&RTTActionServer<ActionSpec>::goalCallback, this, _1))
        .doc("Actionlib goal port. Do not read from this port directly.");

      service->addEventPort(
          action_bridge_.cancelInput(), 
          boost::bind(&RTTActionServer<ActionSpec>::cancelCallback, this, _1))
        .doc("Actionlib cancel port. Do not read from this port directly.");

      service->addPort(action_bridge_.resultOutput<ActionSpec>())
        .doc("Actionlib result port. Do not write to this port directly.");

      service->addPort(action_bridge_.statusOutput())
        .doc("Actionlib result port. Do not write to this port directly.");

      service->addPort(action_bridge_.feedbackOutput<ActionSpec>())
        .doc("Actionlib result port. Do not write to this port directly.");

      return true;
    }

  template <class ActionSpec>
    void RTTActionServer<ActionSpec>::goalCallback(RTT::base::PortInterface* port)
    {
      ActionGoal goal;
      // Read the goal from the RTT port
      if(action_bridge_.goalInput<ActionSpec>().read(goal) == RTT::NewData) {
        // The action server base class expects a shared pointer
        actionlib::ActionServerBase<ActionSpec>::goalCallback(
            boost::make_shared<const ActionGoal>(goal));
      }
    }

  template <class ActionSpec>
    void RTTActionServer<ActionSpec>::cancelCallback(RTT::base::PortInterface* port)
    {
      actionlib_msgs::GoalID goal_id;
      // Read the goal id from the RTT port
      if(action_bridge_.cancelInput().read(goal_id) == RTT::NewData) {
        // The action server base class expects a shared pointer
        actionlib::ActionServerBase<ActionSpec>::cancelCallback(
            boost::make_shared<const actionlib_msgs::GoalID>(goal_id));
      }
    }

  template <class ActionSpec>
    void RTTActionServer<ActionSpec>::publishResult(
        const actionlib_msgs::GoalStatus& status,
        const Result& result)
    {
      ACTION_DEFINITION(ActionSpec);

      boost::recursive_mutex::scoped_lock lock(this->lock_);

      // Create the action result container
      ActionResult action_result;
      action_result.header.stamp = rtt_ros::time::now();
      action_result.status = status;
      action_result.result = result;

      //ROS_DEBUG_NAMED("actionlib", "Publishing result for goal with id: %s and stamp: %.2f", status.goal_id.id.c_str(), status.goal_id.stamp.toSec());

      // Write the result to the RTT data port
      action_bridge_.resultOutput<ActionSpec>().write(action_result);

      this->publishStatus();
    }

  template <class ActionSpec>
    void RTTActionServer<ActionSpec>::publishFeedback(
        const actionlib_msgs::GoalStatus& status,
        const Feedback& feedback)
    {
      ACTION_DEFINITION(ActionSpec);

      boost::recursive_mutex::scoped_lock lock(this->lock_);

      // Create the action result container
      ActionFeedback action_feedback;
      action_feedback.header.stamp = rtt_ros::time::now();
      action_feedback.status = status;
      action_feedback.feedback = feedback;

      //ROS_DEBUG_NAMED("actionlib", "Publishing result for goal with id: %s and stamp: %.2f", status.goal_id.id.c_str(), status.goal_id.stamp.toSec());

      // Write the feedback to the RTT data port
      action_bridge_.feedbackOutput<ActionSpec>().write(action_feedback);
    }

  template <class ActionSpec>
    void RTTActionServer<ActionSpec>::publishStatus()
    {
      boost::recursive_mutex::scoped_lock lock(this->lock_);

      // Build a status array
      actionlib_msgs::GoalStatusArray status_array;

      status_array.header.stamp = rtt_ros::time::now();

      status_array.status_list.resize(this->status_list_.size());

      unsigned int i = 0;
      for(typename std::list<actionlib::StatusTracker<ActionSpec> >::iterator it = this->status_list_.begin();
          it != this->status_list_.end();
          ++i)
      {
        status_array.status_list[i] = (*it).status_;

        // Check if the item is due for deletion from the status list
        if((*it).handle_destruction_time_ != ros::Time() &&
           (*it).handle_destruction_time_ + this->status_list_timeout_ < rtt_ros::time::now()){
          it = this->status_list_.erase(it);
        } else {
          ++it;
        }
      }

      // Publish the status
      action_bridge_.statusOutput().write(status_array);
    }

}



#endif // ifndef __RTT_ACTION_SERVER_H
