
#include <rtt_actionlib/rtt_action_server.h>

using namespace rtt_actionlib;

// Get a ros time 
static const ros::Time ros_rtt_now() 
{
  return ros::Time(((double)RTT::os::TimeService::Instance()->getNSecs())*1E-9);
}

template <class ActionSpec>
RTTActionServer<ActionSpec>::RTTActionServer(const double status_period) :
  status_period_(status_period)
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

  // Temporaries for callbacks
  RTT::DataFlowInterface::SlotFunction 
    goal_cb = boost::bind(&RTTActionServer<ActionSpec>::goalCallback, this, _1),
    cancel_cb = boost::bind(&RTTActionServer<ActionSpec>::cancelCallback, this, _1);

  // Add the ports to the service
  bool success = true;

  success &= service->addEventPort(action_bridge_.goalInput<ActionSpec>(), goal_cb);
  success &= service->addEventPort(action_bridge_.cancelInput(), cancel_cb);
  success &= service->addPort(action_bridge_.resultOutput<ActionSpec>());
  success &= service->addPort(action_bridge_.statusOutput());
  success &= service->addPort(action_bridge_.feedbackOutput<ActionSpec>());

  return success;
}

template <class ActionSpec>
void RTTActionServer<ActionSpec>::goalCallback()
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
void RTTActionServer<ActionSpec>::cancelCallback()
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
  boost::recursive_mutex::scoped_lock lock(this->lock_);

  // Create the action result container
  ActionResult action_result;
  action_result->header.stamp = ros_rtt_now();
  action_result->status = status;
  action_result->result = result;

  //ROS_DEBUG_NAMED("actionlib", "Publishing result for goal with id: %s and stamp: %.2f", status.goal_id.id.c_str(), status.goal_id.stamp.toSec());

  // Write the result to the RTT data port
  action_bridge_.resultOutput<ActionSpec>()->write(action_result);

  this->publishStatus();
}

template <class ActionSpec>
void RTTActionServer<ActionSpec>::publishFeedback(
    const actionlib_msgs::GoalStatus& status,
    const Feedback& feedback)
{
  boost::recursive_mutex::scoped_lock lock(this->lock_);

  // Create the action result container
  ActionFeedback action_feedback;
  action_feedback->header.stamp = ros_rtt_now();
  action_feedback->status = status;
  action_feedback->feedback = feedback;

  //ROS_DEBUG_NAMED("actionlib", "Publishing result for goal with id: %s and stamp: %.2f", status.goal_id.id.c_str(), status.goal_id.stamp.toSec());

  // Write the feedback to the RTT data port
  action_bridge_.feedbackOutput<ActionSpec>()->write(action_feedback);
}

template <class ActionSpec>
void RTTActionServer<ActionSpec>::publishStatus()
{
  boost::recursive_mutex::scoped_lock lock(this->lock_);

  // Get the current time from RTT
  ros::Time now = ros_rtt_now();

  // Update the tracked statuses
  for(typename std::list<actionlib::StatusTracker<ActionSpec> >::iterator it = this->status_list_.begin();
      it != this->status_list_.end();)
  {
    // Check if the item is due for deletion from the status list
    if(it->handle_destruction_time_ != ros::Time() &&
       it->handle_destruction_time_ + this->status_list_timeout_ < now)
    {
      // Update the iterator after deletion
      it = this->status_list_.erase(it);
    } else {
      // Increment the iterator
      ++it;
    }
  }

  // Build a status array message
  actionlib_msgs::GoalStatusArray status_array;
  status_array.header.stamp = now;
  status_array.status_list.assign(this->status_list_.begin(),this->status_list_.end());

  // Publish the status
  action_bridge_.statusOutput().write(status_array);
}
