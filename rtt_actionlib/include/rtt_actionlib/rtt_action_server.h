#ifndef __RTT_ACTION_SERVER_H
#define __RTT_ACTION_SERVER_H

#include <rtt/RTT.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/internal/GlobalService.hpp>

#include <actionlib/action_definition.h>
#include <actionlib/server/action_server_base.h>

#include <rtt_actionlib/rtt_actionlib.h>

namespace rtt_actionlib {

#if 0
  template<class ActionSpec>
  class RTTActionServerStatusTimer : public RTT::os::Timer 
  {
    public:
      void setServer() {}
      virtual void timeout(RTT::os::Timer::TimerID timer_id) {
        
      }
  };
#endif

  template<class ActionSpec>
  class RTTActionServer : public actionlib::ActionServerBase<ActionSpec>
  {
  public:
    // Generates typedefs that make our lives easier
    ACTION_DEFINITION(ActionSpec);

    //! Constructor
    RTTActionServer(const double status_period = 0.200);
    virtual ~RTTActionServer();

    //! Add actionlib ports to a given rtt service
    bool addPorts(RTT::Service::shared_ptr service);

    //! Check if the server is ready to be started
    bool ready();

  private:

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
    void goalCallback();

    /* \brief Wrapper for action server base cancelCallback 
     * This function is called when messages arrive on the goal RTT port. It
     * then reads the port and calls ActionServerBase::cancelCallback, which, in
     * turn, calls the user supplied cancel callback.
     */
    void cancelCallback();

    //! Period (Hz) at which the status should be published
    double status_period_;

    //! Action bridge container for RTT ports corresponding to the action interface
    rtt_actionlib::ActionBridge action_bridge_;
  };

}



#endif // ifndef __RTT_ACTION_SERVER_H
