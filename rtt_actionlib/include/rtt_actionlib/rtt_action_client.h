#ifndef __RTT_ACTION_CLIENT_H
#define __RTT_ACTION_CLIENT_H

#include <actionlib/action_definition.h>
#include <actionlib/client/client_helpers.h>
#include <rtt/Service.hpp>
#include <rtt_actionlib/rtt_actionlib.h>

namespace rtt_actionlib
{

//! Orocos RTT-Based Action Client
template <class ActionSpec>
class RTTActionClient
{
public:
    // Generates typedefs that make our lives easier
    ACTION_DEFINITION(ActionSpec);

    typedef actionlib::ClientGoalHandle<ActionSpec> GoalHandle;
    typedef boost::function<void(GoalHandle)> TransitionCallback;
    typedef boost::function<void(GoalHandle, const FeedbackConstPtr &)> FeedbackCallback;
    typedef boost::function<void(const ActionGoalConstPtr)> SendGoalFunc;

public:
    RTTActionClient();
    ~RTTActionClient() = default;

    //! Add actionlib ports to a given rtt service
    bool addPorts(RTT::Service::shared_ptr service,
                  const bool create_topics = false,
                  const std::string &topic_namespace = "");

    //! Check if the client is ready to be started
    bool ready() const;

    /**
     * \brief Sends a goal to the ActionServer, and also registers callbacks
     * \param transition_cb Callback that gets called on every client state transition
     * \param feedback_cb Callback that gets called whenever feedback for this goal is received
     */
    GoalHandle sendGoal(const Goal &goal,
                        TransitionCallback transition_cb = TransitionCallback(),
                        FeedbackCallback feedback_cb = FeedbackCallback());

    /**
     * \brief Cancel all goals currently running on the action server
     *
     * This preempts all goals running on the action server at the point that
     * this message is serviced by the ActionServer.
     */
    void cancelAllGoals();

    /**
     * \brief Cancel all goals that were stamped at and before the specified time
     * \param time All goals stamped at or before `time` will be canceled
     */
    void cancelGoalsAtAndBeforeTime(const ros::Time &time);

private:
    void sendGoalFunc(const ActionGoalConstPtr &action_goal);
    void sendCancelFunc(const actionlib_msgs::GoalID &cancel_msg);

    void resultCallback(RTT::base::PortInterface *);
    void statusCallback(RTT::base::PortInterface *);
    void feedbackCallback(RTT::base::PortInterface *);

private:
    boost::shared_ptr<actionlib::DestructionGuard> guard_;
    ///! ROS Actionlib GoalManager
    actionlib::GoalManager<ActionSpec> manager_;

    //! Action bridge container for RTT ports corresponding to the action interface
    rtt_actionlib::ActionBridge action_bridge_;
};

template <class ActionSpec>
RTTActionClient<ActionSpec>::RTTActionClient() : guard_ {new actionlib::DestructionGuard {}}
                                               , manager_ {guard_}
{
    manager_.registerSendGoalFunc(boost::bind(&RTTActionClient<ActionSpec>::sendGoalFunc, this, _1));
    manager_.registerCancelFunc(boost::bind(&RTTActionClient<ActionSpec>::sendCancelFunc, this, _1));
}

template <class ActionSpec>
bool RTTActionClient<ActionSpec>::addPorts(RTT::Service::shared_ptr service,
                                           const bool create_topics,
                                           const string &topic_namespace)
{
    // Try to get existing ports from service
    if (!action_bridge_.setPortsFromService(service))
    {
        // Create the ports
        if (!action_bridge_.createClientPorts<ActionSpec>())
        {
            return false;
        }
    }

    // Add the ports to the service
    service->addPort(action_bridge_.goalOutput<ActionSpec>())
            .doc("Actionlib goal port. Do not write to this port directly.");

    service->addPort(action_bridge_.cancelOutput()).doc("Actionlib cancel port. Do not write to this port directly.");

    service->addEventPort(action_bridge_.resultInput<ActionSpec>(),
                          boost::bind(&RTTActionClient<ActionSpec>::resultCallback, this, _1))
            .doc("Actionlib result port. Do not read from this port directly.");

    service->addEventPort(action_bridge_.statusInput(),
                          boost::bind(&RTTActionClient<ActionSpec>::statusCallback, this, _1))
            .doc("Actionlib status port. Do not read from this port directly.");

    service->addEventPort(action_bridge_.feedbackInput<ActionSpec>(),
                          boost::bind(&RTTActionClient<ActionSpec>::feedbackCallback, this, _1))
            .doc("Actionlib feedback port. Do not read from this port directly.");

    // Create ROS topics
    if (create_topics)
    {
        action_bridge_.goalOutput<ActionSpec>().createStream(rtt_roscomm::topic(topic_namespace + "goal"));
        action_bridge_.cancelOutput().createStream(rtt_roscomm::topic(topic_namespace + "cancel"));
        action_bridge_.resultInput<ActionSpec>().createStream(rtt_roscomm::topic(topic_namespace + "result"));
        action_bridge_.statusInput().createStream(rtt_roscomm::topic(topic_namespace + "status"));
        action_bridge_.feedbackInput<ActionSpec>().createStream(rtt_roscomm::topic(topic_namespace + "feedback"));
    }

    return true;
}

template <class ActionSpec>
bool RTTActionClient<ActionSpec>::ready() const
{
    return action_bridge_.allConnected();
}

template <class ActionSpec>
void RTTActionClient<ActionSpec>::sendGoalFunc(const RTTActionClient::ActionGoalConstPtr &action_goal)
{
    action_bridge_.goalOutput<ActionSpec>().write(*action_goal);
}

template <class ActionSpec>
void RTTActionClient<ActionSpec>::sendCancelFunc(const actionlib_msgs::GoalID &cancel_msg)
{
    action_bridge_.cancelOutput().write(cancel_msg);
}

template <class ActionSpec>
void RTTActionClient<ActionSpec>::resultCallback(RTT::base::PortInterface *)
{
    auto actionResult = boost::make_shared<ActionResult>();

    if (action_bridge_.resultInput<ActionSpec>().read(*actionResult) == RTT::NewData)
    {
        manager_.updateResults(std::move(actionResult));
    }
}

template <class ActionSpec>
void RTTActionClient<ActionSpec>::statusCallback(RTT::base::PortInterface *)
{
    auto goalStatus = boost::make_shared<actionlib_msgs::GoalStatusArray>();

    if (action_bridge_.statusInput().read(*goalStatus) == RTT::NewData)
    {
        manager_.updateStatuses(std::move(goalStatus));
    }
}

template <class ActionSpec>
void RTTActionClient<ActionSpec>::feedbackCallback(RTT::base::PortInterface *)
{
    auto actionFeedback = boost::make_shared<ActionFeedback>();

    if (action_bridge_.feedbackInput<ActionSpec>().read(*actionFeedback) == RTT::NewData)
    {
        manager_.updateFeedbacks(std::move(actionFeedback));
    }
}

template <class ActionSpec>
typename RTTActionClient<ActionSpec>::GoalHandle
RTTActionClient<ActionSpec>::sendGoal(const Goal &goal,
                                      RTTActionClient::TransitionCallback transition_cb,
                                      RTTActionClient::FeedbackCallback feedback_cb)
{
    return manager_.initGoal(goal, transition_cb, feedback_cb);
}

template <class ActionSpec>
void RTTActionClient<ActionSpec>::cancelAllGoals()
{
    actionlib_msgs::GoalID cancel_msg;
    // CancelAll policy encoded by stamp=0, id=0
    cancel_msg.stamp = ros::Time(0, 0);
    cancel_msg.id = "";
    action_bridge_.cancelOutput().write(cancel_msg);
}

template <class ActionSpec>
void RTTActionClient<ActionSpec>::cancelGoalsAtAndBeforeTime(const ros::Time &time)
{
    actionlib_msgs::GoalID cancel_msg;
    cancel_msg.stamp = time;
    cancel_msg.id = "";
    action_bridge_.cancelOutput().write(cancel_msg);
}

} // namespace rtt_actionlib.

#endif // __RTT_ACTION_CLIENT_H
