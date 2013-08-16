
#include <rtt_actionlib/rtt_action_server.h>

using namespace rtt_actionlib;

static const ros::Time ros_rtt_now() {
  return ros::Time(((double)RTT::os::TimeService::Instance()->getNSecs())*1E-9);
}

template <class ActionSpec>
RTTActionServer<ActionSpec>::RTTActionServer()
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
    //TODO: create timer
  }
}

template <class ActionSpec>
bool ready() {
  return action_bridge_.allConnected();
}


template <class ActionSpec>
void RTTActionServer<ActionSpec>::goalCallback()
{
  ActionGoal goal;
  // Read the goal from the RTT port
  if(action_bridge_.goal()->read(goal) == RTT::NewData) {
    // The action server base class expects a shared pointer
    actionlib::ActionServerBase<ActionSpec>::goalCallback(
        boost::make_shared<const ActionGoal>(goal));
  }
}

template <class ActionSpec>
void RTTActionServer<ActionSpec>::cancelCallback()
{
  actionlib_msgs::GoalID goal_id;
  // Read the goal id from the RTT port
  if(action_bridge_.cancel()->read(goal_id) == RTT::NewData) {
    // The action server base class expects a shared pointer
    actionlib::ActionServerBase<ActionSpec>::cancelCallback(
        boost::make_shared<const actionlib_msgs::GoalID>(goal_id));
  }
}

template <class ActionSpec>
bool RTTActionServer<ActionSpec>::addPorts(
    RTT::Service::shared_ptr service)
{
  // Try to get existing ports from service
  if(!action_bridge_.setPortsFromService(service)) {
    // Create the ports
    if(!action_bridge_.createServerPorts()) {
      return false;
    }
  }

  // Temporaries for callbacks
  RTT::DataFlowInterface::SlotFunction 
    goal_cb = boost::bind(&RTTActionServer<ActionSpec>::goalCallback, this, _1),
    cancel_cb = boost::bind(&RTTActionServer<ActionSpec>::cancelCallback, this, _1);

  // Add the ports to the service
  bool success = true;

  success &= service->addEventPort(action_bridge_.goal(), goal_cb);
  success &= service->addEventPort(action_bridge_.cancel(), cancel_cb);
  success &= service->addPort(action_bridge_.result());
  success &= service->addPort(action_bridge_.status());
  success &= service->addPort(action_bridge_.feedback());

  return success;
}

template <class ActionSpec>
void RTTActionServer<ActionSpec>::publishResult(
    const actionlib_msgs::GoalStatus& status,
    const Result& result)
{
  boost::recursive_mutex::scoped_lock lock(lock_);

  // Create the action result container
  ActionResult action_result;
  action_result->header.stamp = ros_rtt_now();
  action_result->status = status;
  action_result->result = result;

  //ROS_DEBUG_NAMED("actionlib", "Publishing result for goal with id: %s and stamp: %.2f", status.goal_id.id.c_str(), status.goal_id.stamp.toSec());

  // Write the result to the RTT data port
  action_bridge_.result()->write(action_result);

  this->publishStatus();
}

template <class ActionSpec>
void RTTActionServer<ActionSpec>::publishFeedback(
    const actionlib_msgs::GoalStatus& status,
    const Feedback& feedback)
{
  boost::recursive_mutex::scoped_lock lock(lock_);

  // Create the action result container
  ActionFeedback action_feedback;
  action_feedback->header.stamp = ros_rtt_now();
  action_feedback->status = status;
  action_feedback->feedback = feedback;

  //ROS_DEBUG_NAMED("actionlib", "Publishing result for goal with id: %s and stamp: %.2f", status.goal_id.id.c_str(), status.goal_id.stamp.toSec());

  // Write the feedback to the RTT data port
  action_bridge_.feedback()->write(action_feedback);
}

template <class ActionSpec>
void RTTActionServer<ActionSpec>::publishStatus()
{
  boost::recursive_mutex::scoped_lock lock(lock_);

  // Get the current time from RTT
  ros::Time now = ros_rtt_now();

  // Update the tracked statuses
  for(typename std::list<actionlib::StatusTracker<ActionSpec> >::iterator it = status_list_.begin();
      it != status_list_.end();)
  {
    // Check if the item is due for deletion from the status list
    if(it->handle_destruction_time_ != ros::Time() &&
       it->handle_destruction_time_ + status_list_timeout_ < now)
    {
      // Update the iterator after deletion
      it = status_list_.erase(it);
    } else {
      // Increment the iterator
      ++it;
    }
  }

  // Build a status array
  actionlib_msgs::GoalStatusArray status_array;
  status_array.header.stamp = now;
  status_array.status_list.assign(status_list_.begin(),status_list_.end());

  // Publish the status
  action_bridge_.status().write(status_array);
}
