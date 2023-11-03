#ifndef RTT_SIMPLE_ACTION_CLIENT_HPP
#define RTT_SIMPLE_ACTION_CLIENT_HPP

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <actionlib/client/simple_goal_state.h>
#include <rtt/Logger.hpp>

#include "rtt_action_client.h"

namespace rtt_actionlib
{

template <class ActionSpec>
class RTTSimpleActionClient
{
public:
    // Generates typedefs that make our lives easier
    ACTION_DEFINITION(ActionSpec);
    typedef actionlib::ClientGoalHandle<ActionSpec> GoalHandle;
    typedef boost::function<void(const actionlib::SimpleClientGoalState &state, const Result &result)>
            SimpleDoneCallback;
    typedef boost::function<void()> SimpleActiveCallback;
    typedef boost::function<void(const Feedback &feedback)> SimpleFeedbackCallback;

public:
    explicit RTTSimpleActionClient(boost::shared_ptr<RTT::Service> owner_service);
    ~RTTSimpleActionClient() = default;

    /**
     * Check all connections.
     */
    bool ready() const { return action_client_.ready(); }

    /**
     * \brief Sends a goal to the ActionServer. Discard old goal if exists.
     */
    void sendGoal(const Goal &goal);

    /**
     * \brief Get the state information for this goal
     *
     * Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST.
     * \return The goal's state. Returns LOST if this SimpleActionClient isn't tracking a goal.
     */
    actionlib::SimpleClientGoalState getState() const;

    /**
     * \brief Cancel the goal that we are currently pursuing
     */
    void cancelGoal();

    /**
     * @brief Setup active hook.
     * @param resultHook A function, being called if current goal become active.
     **/
    void setActiveHook(SimpleActiveCallback _activeHook) { activeHook_ = std::move(_activeHook); }

    /**
     * @brief Setup result hook.
     * @param resultHook A function, being called if the goal is done.
     **/
    void setDoneHook(SimpleDoneCallback _doneHook) { doneHook_ = std::move(_doneHook); }

    /**
     * @brief Setup feedback hook.
     * @param feedbackHook A function, being called when active goal has a feedback.
     **/
    void setFeedbackHook(SimpleFeedbackCallback _feedbackHook) { feedbackHook_ = std::move(_feedbackHook); }

private:
    void handleFeedback(GoalHandle gh, const FeedbackConstPtr &feedback);
    void handleTransition(GoalHandle gh);

private:
    RTTActionClient<ActionSpec> action_client_;

    GoalHandle gh_;
    actionlib::SimpleGoalState cur_simple_state_;

    SimpleActiveCallback activeHook_;
    SimpleDoneCallback doneHook_;
    SimpleFeedbackCallback feedbackHook_;
};

template <class ActionSpec>
RTTSimpleActionClient<ActionSpec>::RTTSimpleActionClient(boost::shared_ptr<RTT::Service> owner_service)
        : cur_simple_state_ {actionlib::SimpleGoalState::PENDING}
{
    if (!owner_service)
        throw std::invalid_argument("RTTSimpleActionClient: owner pointer must be valid.");
    action_client_.addPorts(owner_service);
}

template <class ActionSpec>
void RTTSimpleActionClient<ActionSpec>::sendGoal(const Goal &goal)
{
    gh_.reset();
    cur_simple_state_ = actionlib::SimpleGoalState::PENDING;
    gh_ = action_client_.sendGoal(goal,
                                  boost::bind(&RTTSimpleActionClient<ActionSpec>::handleTransition, this, _1),
                                  boost::bind(&RTTSimpleActionClient<ActionSpec>::handleFeedback, this, _1, _2));
}

template <class ActionSpec>
void RTTSimpleActionClient<ActionSpec>::cancelGoal()
{
    if (gh_.isExpired())
    {
        RTT::Logger::In in {"RTTSimpleActionClient"};
        RTT::log(RTT::Error)
                << "Trying to cancelGoal() when no goal is running. You are incorrectly using SimpleActionClient"
                << RTT::endlog();
    }

    gh_.cancel();
}

template <class ActionSpec>
actionlib::SimpleClientGoalState RTTSimpleActionClient<ActionSpec>::getState() const
{
    // Code copied from actionlib/client/simple_action_client.h
    RTT::Logger::In in {"RTTSimpleActionClient"};

    if (gh_.isExpired())
    {
        RTT::log(RTT::Error)
                << "Trying to getState() when no goal is running. You are incorrectly using SimpleActionClient";
        return actionlib::SimpleClientGoalState(actionlib::SimpleClientGoalState::LOST);
    }

    actionlib::CommState comm_state_ = gh_.getCommState();

    switch (comm_state_.state_)
    {
        case actionlib::CommState::WAITING_FOR_GOAL_ACK:
        case actionlib::CommState::PENDING:
        case actionlib::CommState::RECALLING:
            return actionlib::SimpleClientGoalState(actionlib::SimpleClientGoalState::PENDING);
        case actionlib::CommState::ACTIVE:
        case actionlib::CommState::PREEMPTING:
            return actionlib::SimpleClientGoalState(actionlib::SimpleClientGoalState::ACTIVE);
        case actionlib::CommState::DONE:
        {
            switch (gh_.getTerminalState().state_)
            {
                case actionlib::TerminalState::RECALLED:
                    return actionlib::SimpleClientGoalState(actionlib::SimpleClientGoalState::RECALLED,
                                                            gh_.getTerminalState().text_);
                case actionlib::TerminalState::REJECTED:
                    return actionlib::SimpleClientGoalState(actionlib::SimpleClientGoalState::REJECTED,
                                                            gh_.getTerminalState().text_);
                case actionlib::TerminalState::PREEMPTED:
                    return actionlib::SimpleClientGoalState(actionlib::SimpleClientGoalState::PREEMPTED,
                                                            gh_.getTerminalState().text_);
                case actionlib::TerminalState::ABORTED:
                    return actionlib::SimpleClientGoalState(actionlib::SimpleClientGoalState::ABORTED,
                                                            gh_.getTerminalState().text_);
                case actionlib::TerminalState::SUCCEEDED:
                    return actionlib::SimpleClientGoalState(actionlib::SimpleClientGoalState::SUCCEEDED,
                                                            gh_.getTerminalState().text_);
                case actionlib::TerminalState::LOST:
                    return actionlib::SimpleClientGoalState(actionlib::SimpleClientGoalState::LOST,
                                                            gh_.getTerminalState().text_);
                default:
                    RTT::log(RTT::Error) << "Unknown terminal state [" << gh_.getTerminalState().state_
                                         << "]. This is a bug in RTTSimpleActionClient" << RTT::endlog();
                    return actionlib::SimpleClientGoalState(actionlib::SimpleClientGoalState::LOST,
                                                            gh_.getTerminalState().text_);
            }
        }
        case actionlib::CommState::WAITING_FOR_RESULT:
        case actionlib::CommState::WAITING_FOR_CANCEL_ACK:
        {
            switch (cur_simple_state_.state_)
            {
                case actionlib::SimpleGoalState::PENDING:
                    return actionlib::SimpleClientGoalState(actionlib::SimpleClientGoalState::PENDING);
                case actionlib::SimpleGoalState::ACTIVE:
                    return actionlib::SimpleClientGoalState(actionlib::SimpleClientGoalState::ACTIVE);
                case actionlib::SimpleGoalState::DONE:
                    RTT::log(RTT::Error) << "In WAITING_FOR_RESULT or WAITING_FOR_CANCEL_ACK, yet we are in "
                                            "SimpleGoalState DONE. This is a bug in RTTSimpleActionClient"
                                         << RTT::endlog();
                    return actionlib::SimpleClientGoalState(actionlib::SimpleClientGoalState::LOST);
                default:
                    RTT::log(RTT::Error) << "\"Got a SimpleGoalState of [" << cur_simple_state_.state_
                                         << "]. This is a bug in SimpleActionClient\"" << RTT::endlog();
            }
        }
        default: break;
    }
    RTT::log(RTT::Error) << "Error trying to interpret CommState - " << comm_state_.state_ << RTT::endlog();
    return actionlib::SimpleClientGoalState(actionlib::SimpleClientGoalState::LOST);
}

template <class ActionSpec>
void RTTSimpleActionClient<ActionSpec>::handleFeedback(RTTSimpleActionClient::GoalHandle gh,
                                                       const RTTSimpleActionClient::FeedbackConstPtr &feedback)
{
    if (gh_ != gh)
    {
        RTT::Logger::In in {"RTTSimpleActionClient"};
        RTT::log(RTT::Error) << "Got a callback on a goalHandle that we're not tracking. \
               This is an internal rtt_action bug. \
               This could also be a GoalID collision"
                             << RTT::endlog();
    }

    if (feedbackHook_)
        feedbackHook_(*feedback);
}

template <class ActionSpec>
void RTTSimpleActionClient<ActionSpec>::handleTransition(RTTSimpleActionClient::GoalHandle gh)
{
    // Code copied from actionlib/client/simple_action_client.h

    RTT::Logger::In in {"RTTSimpleActionClient"};
    actionlib::CommState comm_state_ = gh.getCommState();
    switch (comm_state_.state_)
    {
        case actionlib::CommState::WAITING_FOR_GOAL_ACK:
            RTT::log(RTT::Error) << "BUG: Shouldn't ever get a transition callback for WAITING_FOR_GOAL_ACK"
                                 << RTT::endlog();
            break;
        case actionlib::CommState::PENDING:
            if (cur_simple_state_ != actionlib::SimpleGoalState::PENDING)
                RTT::log(RTT::Error) << "BUG: Got a transition to CommState [" << comm_state_.toString()
                                     << "] when our in SimpleGoalState [" << cur_simple_state_.toString() << "]"
                                     << RTT::endlog();
            break;
        case actionlib::CommState::ACTIVE:
            switch (cur_simple_state_.state_)
            {
                case actionlib::SimpleGoalState::PENDING:
                    cur_simple_state_ = actionlib::SimpleGoalState::ACTIVE;
                    if (activeHook_)
                        activeHook_();
                    break;
                case actionlib::SimpleGoalState::ACTIVE: break;
                case actionlib::SimpleGoalState::DONE:
                    RTT::log(RTT::Error) << "BUG: Got a transition to CommState [" << comm_state_.toString()
                                         << "] when our in SimpleGoalState [" << cur_simple_state_.toString() << "]"
                                         << RTT::endlog();
                    break;
            }
            break;
        case actionlib::CommState::WAITING_FOR_RESULT: break;
        case actionlib::CommState::WAITING_FOR_CANCEL_ACK: break;
        case actionlib::CommState::RECALLING:
            RTT::log(RTT::Error) << "BUG: Got a transition to CommState [" << comm_state_.toString()
                                 << "] when our in SimpleGoalState [" << cur_simple_state_.toString() << "]"
                                 << RTT::endlog();
            break;
        case actionlib::CommState::PREEMPTING:
            switch (cur_simple_state_.state_)
            {
                case actionlib::SimpleGoalState::PENDING:
                    cur_simple_state_ = actionlib::SimpleGoalState::ACTIVE;
                    if (activeHook_)
                        activeHook_();
                    break;
                case actionlib::SimpleGoalState::ACTIVE: break;
                case actionlib::SimpleGoalState::DONE:
                    RTT::log(RTT::Error) << "BUG: Got a transition to CommState [" << comm_state_.toString()
                                         << "] when our in SimpleGoalState [" << cur_simple_state_.toString() << "]"
                                         << RTT::endlog();
                    break;
            }
            break;
        case actionlib::CommState::DONE:
            switch (cur_simple_state_.state_)
            {
                case actionlib::SimpleGoalState::PENDING:
                case actionlib::SimpleGoalState::ACTIVE:
                    cur_simple_state_ = actionlib::SimpleGoalState::DONE;

                    if (doneHook_)
                        doneHook_(getState(), *gh.getResult());
                    break;
                case actionlib::SimpleGoalState::DONE:
                    RTT::log(RTT::Error) << "BUG: Got a second transition to DONE" << RTT::endlog();
                    break;
            }
            break;
        default:
            RTT::log(RTT::Error) << "Unknown CommState received [%u]" << comm_state_.state_ << RTT::endlog();
            break;
    }
}

} // namespace rtt_actionlib.

#endif // RTT_SIMPLE_ACTION_CLIENT_HPP
