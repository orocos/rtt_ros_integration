#ifndef __RTT_ACTIONLIB_H
#define __RTT_ACTIONLIB_H

#include <rtt/RTT.hpp>

#include <actionlib/action_definition.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <rtt_roscomm/rostopic.h>

namespace rtt_actionlib {

  /* \brief Actionlib RTT bridge used to create and access actionlib RTT data ports
   */
  class ActionBridge
  {
  public:

    //! Constructor
    ActionBridge() : 
      owns_port_pointers_(false), goal_(NULL), cancel_(NULL), status_(NULL), result_(NULL), feedback_(NULL)
    { }

    ~ActionBridge()
    {
      if(owns_port_pointers_) {
        if(goal_) delete goal_;
        if(cancel_) delete cancel_;
        if(status_) delete status_;
        if(result_) delete result_;
        if(feedback_) delete feedback_;
      }
    }

    //! Add actionlib ports to a given rtt service
    template<class ActionSpec>
    bool createServerPorts()
    {
      // Generates typedefs that make our lives easier
      ACTION_DEFINITION(ActionSpec);

      // Make sure it isn't valid or connected
      if(this->isValid() || this->anyConnected()) {
        return false;
      }

      // Construct server ports
      goal_     = new RTT::InputPort<ActionGoal>("_action_goal");
      cancel_   = new RTT::InputPort<actionlib_msgs::GoalID>("_action_cancel");
      result_   = new RTT::OutputPort<ActionResult>("_action_result");
      status_   = new RTT::OutputPort<actionlib_msgs::GoalStatusArray>("_action_status");
      feedback_ = new RTT::OutputPort<ActionFeedback>("_action_feedback");

      // Set the ownership flag
      owns_port_pointers_ = true;

      return this->isValid();
    }

    template<class ActionSpec>
    bool createClientPorts()
    {
      // Generates typedefs that make our lives easier
      ACTION_DEFINITION(ActionSpec);

      // Make sure it isn't valid or connected
      if(this->isValid() || this->anyConnected()) {
        return false;
      }

      // Construct server ports
      goal_     = new RTT::OutputPort<ActionGoal>("_action_goal");
      cancel_   = new RTT::OutputPort<actionlib_msgs::GoalID>("_action_cancel");
      result_   = new RTT::InputPort<ActionResult>("_action_result");
      status_   = new RTT::InputPort<actionlib_msgs::GoalStatusArray>("_action_status");
      feedback_ = new RTT::InputPort<ActionFeedback>("_action_feedback");

      // Set the ownership flag
      owns_port_pointers_ = true;

      return this->isValid();
    }

    //! Get the goal input port
    template<class ActionSpec>
    RTT::InputPort<typename ActionSpec::_action_goal_type>& goalInput() { return *dynamic_cast<RTT::InputPort<typename ActionSpec::_action_goal_type>*>(goal_); }
    //! Get the cancel input port
    RTT::InputPort<actionlib_msgs::GoalID>& cancelInput() { return *dynamic_cast<RTT::InputPort<actionlib_msgs::GoalID>*>(cancel_); }
    //! Get the status input port
    RTT::InputPort<actionlib_msgs::GoalStatusArray>& statusInput() { return *dynamic_cast<RTT::InputPort<actionlib_msgs::GoalStatusArray>*>(status_); }
    //! Get the result input port
    template<class ActionSpec>
    RTT::InputPort<typename ActionSpec::_action_result_type>& resultInput() { return *dynamic_cast<RTT::InputPort<typename ActionSpec::_action_result_type>*>(result_); }
    //! Get the feedback input port
    template<class ActionSpec>
    RTT::InputPort<typename ActionSpec::_action_feedback_type>& feedbackInput() { return *dynamic_cast<RTT::InputPort<typename ActionSpec::_action_feedback_type>*>(feedback_); }
    
    //! Get the goal output port
    template<class ActionSpec>
    RTT::OutputPort<typename ActionSpec::_action_goal_type>& goalOutput() { return *dynamic_cast<RTT::OutputPort<typename ActionSpec::_action_goal_type>*>(goal_); }
    //! Get the cancel output port
    RTT::OutputPort<actionlib_msgs::GoalID>& cancelOutput() { return *dynamic_cast<RTT::OutputPort<actionlib_msgs::GoalID>*>(cancel_); }
    //! Get the status output port
    RTT::OutputPort<actionlib_msgs::GoalStatusArray>& statusOutput() { return *dynamic_cast<RTT::OutputPort<actionlib_msgs::GoalStatusArray>*>(status_); }
    //! Get the result output port
    template<class ActionSpec>
    RTT::OutputPort<typename ActionSpec::_action_result_type>& resultOutput() { return *dynamic_cast<RTT::OutputPort<typename ActionSpec::_action_result_type>*>(result_); }
    //! Get the feedback output port
    template<class ActionSpec>
    RTT::OutputPort<typename ActionSpec::_action_feedback_type>& feedbackOutput() { return *dynamic_cast<RTT::OutputPort<typename ActionSpec::_action_feedback_type>*>(feedback_); }

    //! Store the RTT ports manually
    bool setPorts(
        RTT::base::PortInterface* goal,
        RTT::base::PortInterface* cancel,
        RTT::base::PortInterface* status,
        RTT::base::PortInterface* result,
        RTT::base::PortInterface* feedback) 
    {
      // Store the ports
      goal_ = goal;
      cancel_ = cancel;
      status_ = status;
      result_ = result;
      feedback_ = feedback;

      // Set the ownership flag
      owns_port_pointers_ = false;

      return this->isValid();
    }

    //! Store the required RTT ports from a given RTT service
    bool setPortsFromService(RTT::Service::shared_ptr service)
    {
      // Make sure service isn't null
      if(service.get() == NULL) { return false; }

      // Get ports
      goal_ = service->getPort("_action_goal");
      cancel_ = service->getPort("_action_cancel");
      status_ = service->getPort("_action_status");
      result_ = service->getPort("_action_result");
      feedback_ = service->getPort("_action_feedback");

      // Set the ownership flag
      owns_port_pointers_ = false;

      // Return true if required ports are non-null
      return this->isValid();
    }
    
    //! True if all required ports are not null
    bool isValid() const 
    {
      return goal_ && cancel_ && status_ && result_ && feedback_;
    }

    //! True if valid, goal/cancel are inputs, and result/status/feedback are outputs
    bool isServer() const
    {
      // Make sure the bridge is valid
      if(!this->isValid()) { return false; }

      RTT::base::InputPortInterface *goal_in, *cancel_in;
      RTT::base::OutputPortInterface *status_out, *result_out, *feedback_out;

      // Get port directions
      goal_in = dynamic_cast<RTT::base::InputPortInterface*>(goal_);
      cancel_in = dynamic_cast<RTT::base::InputPortInterface*>(cancel_);

      status_out = dynamic_cast<RTT::base::OutputPortInterface*>(status_);
      result_out = dynamic_cast<RTT::base::OutputPortInterface*>(result_);
      feedback_out = dynamic_cast<RTT::base::OutputPortInterface*>(feedback_);

      return goal_in && cancel_in && status_out && result_out && feedback_out;
    }

    //! True if valid, goal/cancel are outputs, and result/status/feedback are inputs 
    bool isClient() const
    {
      // Make sure the bridge is valid
      if(!this->isValid()) { return false; }

      // Declare actual port directions
      RTT::base::InputPortInterface *status_in, *result_in, *feedback_in;
      RTT::base::OutputPortInterface *goal_out, *cancel_out;

      // Get port directions
      goal_out = dynamic_cast<RTT::base::OutputPortInterface*>(goal_);
      cancel_out = dynamic_cast<RTT::base::OutputPortInterface*>(cancel_);

      status_in = dynamic_cast<RTT::base::InputPortInterface*>(status_);
      result_in = dynamic_cast<RTT::base::InputPortInterface*>(result_);
      feedback_in = dynamic_cast<RTT::base::InputPortInterface*>(feedback_);

      return goal_out && cancel_out && status_in && result_in && feedback_in;
    }

    //! Create a stream from this RTT actionlib port interface to the appropriate ROS topic interface
    bool createStream(const std::string action_ns, RTT::ConnPolicy cp_template = RTT::ConnPolicy::data()) 
    {
      // Make sure the bridge is valid
      if(!this->isValid()) { return false; }

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
      valid &= cancel_->createStream(cp_cancel);
      valid &= status_->createStream(cp_status);
      valid &= result_->createStream(cp_result);
      valid &= feedback_->createStream(cp_feedback);

      return valid;
    }

    //! True if all existing ports are connected
    bool allConnected() const 
    {
      // Make sure the bridge is valid
      if(!this->isValid()) { return false; }

      bool valid = true;

      valid &= goal_->connected();
      valid &= cancel_->connected();
      valid &= status_->connected();
      valid &= result_->connected();
      valid &= feedback_->connected();

      return valid;
    }

    bool anyConnected() const 
    {
      // Make sure the bridge is valid
      if(!this->isValid()) { return false; }

      bool valid = false;

      valid |= goal_->connected();
      valid |= cancel_->connected();
      valid |= status_->connected();
      valid |= result_->connected();
      valid |= feedback_->connected();

      return valid;
    }

  private:

    // Ownership of pointers
    bool owns_port_pointers_;

    // RTT Ports
    RTT::base::PortInterface* goal_;
    RTT::base::PortInterface* cancel_;
    RTT::base::PortInterface* status_;
    RTT::base::PortInterface* result_;
    RTT::base::PortInterface* feedback_;

  };

}

#endif // ifndef __RTT_ACTIONLIB_H
