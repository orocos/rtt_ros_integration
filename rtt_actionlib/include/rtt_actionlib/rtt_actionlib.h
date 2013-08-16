#ifndef __RTT_ACTIONLIB_H
#define __RTT_ACTIONLIB_H

#include <actionlib/action_definition.h>

#include <rtt_rostopic/rtt_rostopic.h>

namespace rtt_actionlib {

  //! Reqiured and optional ports for a valid Actionlib interface
  class ActionBridge
  {
  public:

    //! Constructor
    ActionBridge() : 
      goal_(NULL), cancel_(NULL), status_(NULL), result_(NULL), feedback_(NULL)
    {
    }

    //! Add actionlib ports to a given rtt service
    template<class ActionSpec>
    bool createServerPorts(bool provide_cancel, bool provide_feedback)
    {
      // Generates typedefs that make our lives easier
      ACTION_DEFINITION(ActionSpec);

      // Make sure it isn't valid or connected
      if(this->isValid() || this->isConnected()) {
        return false;
      }

      // Construct server ports
      goal_ = new RTT::InputPort<ActionGoal>("goal", exec_thread_type);
      cancel = provide_cancel ? new RTT::InputPort<actionlib_msgs::GoalID>("cancel", exec_thread_type) : NULL;
      result = new RTT::OutputPort<ActionResult>("result");
      status = new RTT::OutputPort<actionlib_msgs::GoalStatusArray>("status");
      feedback = provide_feedback ? new RTT::OutputPort<ActionFeedback>("feedback") : NULL;

      return true;
    }

    template<class ActionSpec>
    bool createClientPorts(
        RTT::Service::shared_ptr service,
        const RTT::ExecutionThread exec_thread_type,
        bool require_cencel, bool require_feedback)
    {
      // Generates typedefs that make our lives easier
      ACTION_DEFINITION(ActionSpec);

      // Make sure it isn't valid or connected
      if(this->isValid() || this->isConnected()) {
        return false;
      }

      // Construct server ports
      goal_ = new RTT::OutputPort<ActionGoal>("goal");
      cancel = require_cancel ? new RTT::OutputPort<actionlib_msgs::GoalID>("cancel") : NULL;
      result = new RTT::InputPort<ActionResult>("result");
      status = new RTT::InputPort<actionlib_msgs::GoalStatusArray>("status");
      feedback = require_feedback ? new RTT::InputPort<ActionFeedback>("feedback") : NULL;

      // Add all the ports to this service
      bool success = true;

      success &= service->addPort(goal_);
      success &= require_cancel ? service->addPort(cancel) : true;
      success &= service->addPort(result);
      success &= service->addPort(status);
      success &= require_feedback ? service->addPort(feedback) : true;

      return success;
    }

    //! Get the goal port
    RTT::base::PortInteface* goal() { return goal_; }
    //! Get the cancel port
    RTT::base::PortInteface* cancel() { return cancel_; }
    //! Get the status port
    RTT::base::PortInteface* status() { return status_; }
    //! Get the result port
    RTT::base::PortInteface* result() { return result_; }
    //! Get the feedback port
    RTT::base::PortInteface* feedback() { return feedback_; }

    //! Store the RTT ports manually
    bool seetPorts(
        RTT::base::PortInteface* goal,
        RTT::base::PortInteface* cancel,
        RTT::base::PortInteface* status,
        RTT::base::PortInteface* result,
        RTT::base::PortInteface* feedback) 
    {
      // Store the ports
      goal_ = goal;
      cancel_ = cancel;
      status_ = status;
      result_ = result;
      feedbacl_ = feedbacl;

      return this->isValid();
    }

    //! Store the required RTT ports from a given RTT service
    bool setPortsFromService(RTT::Service::shared_ptr service)
    {
      // Make sure service isn't null
      if(service.get() == NULL) { return false; }

      // Get ports
      goal_ = service->getPort("goal");
      cancel_ = service->getPort("cancel");
      status_ = service->getPort("status");
      result_ = service->getPort("result");
      feedback_ = service->getPort("feedback");

      // Return true if required ports are non-null
      return this->isValid();
    }
    
    //! True if all required ports are not null
    bool isValid() const 
    {
      return goal_ && status_ && result_;
    }

    //! True if valid, goal/cancel are inputs, and result/status/feedback are outputs
    bool is_server() const
    {
      // Make sure the bridge is valid
      if(!this->is_valid()) { return false; }

      RTT::base::InputPortInterface goal_in, cancel_in;
      RTT::base::OutputPortInterface status_out, result_out, feedback_out;

      // Get port directions
      goal_in = dynamic_cast<RTT::base::InputPortInterface*>(goal_);
      cancel_in = dynamic_cast<RTT::base::InputPortInterface*>(cancel_);

      status_out = dynamic_cast<RTT::base::OutputPortInterface*>(status_);
      result_out = dynamic_cast<RTT::base::OutputPortInterface*>(result_);
      feedback_out = dynamic_cast<RTT::base::OutputPortInterface*>(feedback_);

      // Check port existence / directions
      bool valid = true;

      // Make sure optional ports are either the right direction or non-existent
      valid &= cancel_in != NULL || cancel_ == NULL;
      valid &= feedback_out != NULL || feedback_ == NULL;

      return valid;
    }

    //! True if valid, goal/cancel are outputs, and result/status/feedback are inputs 
    bool is_client() const
    {
      // Make sure the bridge is valid
      if(!this->is_valid()) { return false; }

      // Declare actual port directions
      RTT::base::InputPortInterface status_in, result_in, feedback_in;
      RTT::base::OutputPortInterface goal_out, cancel_out;

      // Get port directions
      goal_out = dynamic_cast<RTT::base::OuptutPortInterface*>(goal_);
      cancel_out = dynamic_cast<RTT::base::OuptutPortInterface*>(cancel_);

      status_in = dynamic_cast<RTT::base::InputPortInterface*>(status_);
      result_in = dynamic_cast<RTT::base::InputPortInterface*>(result_);
      feedback_in = dynamic_cast<RTT::base::InputPortInterface*>(feedback_);

      // Check port existence / directions
      bool valid = true;

      // Make sure optional ports are either the right direction or non-existent
      valid &= cancel_out != NULL || cancel_ == NULL;
      valid &= feedback_in != NULL || feedback_ == NULL;

      return valid;
    }

    //! Create a stream from this RTT actionlib port interface to the appropriate ROS topic interface
    bool createStream(const std::string action_ns, RTT::ConnPolicy cp_template = RTT::ConnPolicy::data()) 
    {
      // Make sure the bridge is valid
      if(!this->is_valid()) { return false; }

      // Construct connection policies for each port
      RTT::ConnPolicy 
        cp_goal = cp_template,
        cp_cancel = cp_template,
        cp_status = cp_template,
        cp_result = cp_template,
        cp_feedback = cp_template;
    
      // Set the ros connection policies
      cp_goal.transport = ORO_ROS_PROTOCOL_ID;
      cp_cancel.transport = ORO_ROS_PROTOCOL_ID;
      cp_status.transport = ORO_ROS_PROTOCOL_ID;
      cp_result.transport = ORO_ROS_PROTOCOL_ID;
      cp_feedback.transport = ORO_ROS_PROTOCOL_ID;

      // Construct the actionlib topic names
      cp_goal.name_id = action_ns+"/goal";
      cp_cancel.name_id = action_ns+"/cancel";
      cp_status.name_id = action_ns+"/status";
      cp_result.name_id = action_ns+"/result";
      cp_feedback.name_id = action_ns+"/feedback";

      // Connect each stream
      bool valid = true;

      valid &= goal_->createStream(cp_goal);
      valid &= cancel_ ? cancel_->createStream(cp_cancel):true;
      valid &= status_->createStream(cp_status);
      valid &= result_->createStream(cp_result);
      valid &= feedback_ ? feedback_->createStream(cp_goal):true;

      return valid;
    }

    //! True if all existing ports are connected
    bool isConnected() const 
    {
      // Make sure the bridge is valid
      if(!this->is_valid()) { return false; }

      bool valid = true;

      valid &= goal_->isConnected();
      valid &= cancel_ ? cancel_->createStream():true;
      valid &= status_->isConnected();
      valid &= result_->isConnected();
      valid &= feedback_ ? feedback_->isConnected():true;

      return valid;
    }

    // RTT Ports
    RTT::base::PortInteface* goal_;
    RTT::base::PortInteface* cancel_;
    RTT::base::PortInteface* status_;
    RTT::base::PortInteface* result_;
    RTT::base::PortInteface* feedback_;
  };

}

#endif // ifndef __RTT_ACTIONLIB_H
