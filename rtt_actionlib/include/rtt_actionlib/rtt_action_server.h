#ifndef __RTT_ACTION_SERVER_H
#define __RTT_ACTION_SERVER_H

#include <actionlib/action_definition.h>
#include <actionlib/server/action_server_base.h>

namespace rtt_actionlib {

  template<class ActionSpec>
  class RTTActionServer : public actionlib::ActionServerBase<ActionSpec>
  {
  public:
    // Generates typedefs that make our lives easier
    ACTION_DEFINITION(ActionSpec);

    RTTActionServer();

    //! Add actionlib ports to a given rtt service
    bool addPorts(
        RTT::Service::shared_ptr service,
        const RTT::ExecutionThread exec_thread_type);

  private:

    //! \brief  Initialize all RTT/ROS connections and setup timers
    virtual void initialize();

    /* \brief  Publishes a result for a given goal
     * \param status The status of the goal with which the result is associated
     * \param result The result to publish
     */
    virtual void publishResult(const actionlib_msgs::GoalStatus& status, const Result& result);

    /* \brief  Publishes feedback for a given goal
     * \param status The status of the goal with which the feedback is associated
     * \param feedback The feedback to publish
     */
    virtual void publishFeedback(const actionlib_msgs::GoalStatus& status, const Feedback& feedback);

    //! Explicitly publish status
    virtual void publishStatus();
    
  private:

    rtt_actionlib::ActionBridge action_bridge_;
  };

}



#endif // ifndef __RTT_ACTION_SERVER_H
