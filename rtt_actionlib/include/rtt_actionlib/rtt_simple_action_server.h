#ifndef  RTT_SIMPLE_ACTION_SERVER_HPP
#define  RTT_SIMPLE_ACTION_SERVER_HPP

#include <stdexcept>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <actionlib/server/action_server.h>
#include <actionlib/action_definition.h>
#include <orocos/rtt_actionlib/rtt_action_server.h>

namespace rtt_actionlib {

template <class ActionSpec> class RTTSimpleActionServer 
{
	protected:
		// GoalHandle
		typedef actionlib::ServerGoalHandle<ActionSpec> GoalHandle;
		// Goal, Feedback, Result typedefs
		ACTION_DEFINITION(ActionSpec);	

	protected:
		rtt_actionlib::RTTActionServer<ActionSpec> action_server;
		GoalHandle goal_active;
		GoalHandle goal_pending;

		// callbacks
		boost::function<void(const Goal&)> newGoalHook;
		boost::function<void()> cancelGoalHook;

	protected:

		void goalCallback(GoalHandle gh);
		void cancelCallback(GoalHandle gh);

	public:

		/**
		 * Create RTTSimpleActionServer in given Service.
		 *
		 * Create ports, register callbacks.
		 * @param owner_service Pointer to owner service.
		 */
		RTTSimpleActionServer(boost::shared_ptr<RTT::Service> owner_service) 
		{	
			if (!owner_service) throw std::invalid_argument("RTTSimpleActionServer: owner pointer must be valid.");
			action_server.addPorts(owner_service);
			action_server.registerGoalCallback(boost::bind(&RTTSimpleActionServer<ActionSpec>::goalCallback, this, _1));
			action_server.registerCancelCallback(boost::bind(&RTTSimpleActionServer<ActionSpec>::cancelCallback, this, _1));
		}

		/**
		 * Check all connections.
		 */
		bool ready() { 
			return action_server.ready(); 
		}


		/**
		* Start action server in given TskContext.
		* @param publish_feedback if true publish feedback periodically
		*/
		bool start(bool publish_feedback = false);

		/**
		* Stop action server.
		*/
		void shutdown() {
			action_server.shutdown();
		}

		/**
		* Check, if active goal is present.
		*/
		bool isActive() const {
			return goal_active.isValid() && goal_active.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE;
		}

		/**
		* Check, if active goal is being preempted.
		*/
		bool isPreempting() const {
			return goal_active.isValid() && goal_active.getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTING;
		}

		/**
		* Check, if pending goal is present.
		*/
		bool isPending() const {
			return goal_pending.isValid() && goal_pending.getGoalStatus().status == actionlib_msgs::GoalStatus::PENDING;
		}

		/** 
		* Return pointer to active goal.
		*/
		boost::shared_ptr<const Goal> getActiveGoal() const {
			if (isActive()) return goal_active.getGoal();
			else return boost::shared_ptr<const Goal>(); // return NULL
			// else nullptr;
		}

		/**
		 * Return pointer to pending goal.
		 **/
		boost::shared_ptr<const Goal> getPendingGoal() const {
			if (isPending()) return goal_pending.getGoal();
			else return boost::shared_ptr<const Goal>(); // return NULL
			// else nullptr;
		}

		/**
		 * @brief Accept pending goal.
		 * Accept pending goal, preempt the active goal with result if present. Do nothing if there is no pending goal.
		 * @param result Result of the preemted active goal.
		 * @return true if pending goal is accepted.
		 **/
		bool acceptPending(const Result& result, const std::string& msg = "");

		/**
		 * @brief Reject pending goal.
		 * Reject pending goal with result, do nothing if there is no pending goal.
		 * @param result Result of the rejected pending goal.
		 * @return true if pending goal is rejected..
		 **/
		bool rejectPending(const Result& result, const std::string& msg = "");

		/**
		 * @brief Publish feedback on active goal.
		 * Publish feedback on active goal, do nothing if there is no active goal.
		 * @param feedback action feedback
		 * @return true if active goal is present.
		 **/
		bool publishFeedback(const Feedback& feedpack);

		/**
		 * @brief Abort active goal.
		 * Abort active goal with given result, do nothing if there is no active goal.
		 * @param result Result of the aborted active goal.
		 * @return true if active goal is aborted.
		 **/
		bool abortActive(const Result& result, const std::string& msg = "");

		/**
		 * @brief Cancel active goal.
		 * Cancel active goal with given result, do nothing if there is no active goal.
		 * @param result Result of the aborted active goal.
		 * @return true if goal is been canceled.
		 **/
		bool cancelActive(const Result& result, const std::string& msg = "");

		/**
		 * @brief Succeed active goal.
		 * Succeed active goal with given result, do nothing if there is no active goal.
		 * @param result Result of the succeed active goal.
		 * @return true if active goal is succeed.
		 **/
		bool succeedActive(const Result& result, const std::string& msg = "");

		/**
		 * @brief Setup goal hook.
		 * @param newGoalHook A function, being called if new goal arrives.
		 **/
		void setGoalHook(boost::function< void(const Goal&)> _newGoalHook) {
			newGoalHook = _newGoalHook;
		}

		/**
		 * @brief Setup cancel hook.
		 * @param cancelGoalHook A function, being called when active goal is canceled.
		 **/
		void setCancelHook(boost::function< void()> _cancelGoalHook) {
			cancelGoalHook = _cancelGoalHook;
		}
};

template <class ActionSpec> bool RTTSimpleActionServer<ActionSpec>::start(bool publish_feedback) 
{
	if (ready()) {
		action_server.start();
		if (publish_feedback) action_server.initialize();
		return true;
	}
	else return false;
}

template <class ActionSpec> bool RTTSimpleActionServer<ActionSpec>::acceptPending(const Result& result, const std::string& msg)
{
	if (isPending()) {
		if (goal_active.isValid()) {
			switch (goal_active.getGoalStatus().status) {
				case actionlib_msgs::GoalStatus::ACTIVE:
				case actionlib_msgs::GoalStatus::PREEMPTING:
					goal_active.setAborted(result, msg);
					break;
			}
		}
		goal_pending.setAccepted();
		goal_active = goal_pending;
		return isActive();
	}
	else {
		// if pending is in RECALLING cancelHook perform necessary actions
		return false;
	}
}

template <class ActionSpec> bool RTTSimpleActionServer<ActionSpec>::rejectPending(const Result& result, const std::string& msg) 
{
	if (!goal_pending.isValid())  return false;
	switch (goal_pending.getGoalStatus().status) {
		case actionlib_msgs::GoalStatus::PENDING:
		case actionlib_msgs::GoalStatus::RECALLING:
			goal_pending.setRejected(result, msg);
			return true;
	}
	return false;
}

template <class ActionSpec> bool RTTSimpleActionServer<ActionSpec>::publishFeedback(const Feedback& feedpack)
{
	if (!isActive()) return false;
	goal_active.publishFeedback(feedpack);
	return true;
}

template <class ActionSpec> bool RTTSimpleActionServer<ActionSpec>::abortActive(const Result& result, const std::string& msg)
{
	if (!goal_active.isValid())  return false;
	switch (goal_active.getGoalStatus().status) {
		case actionlib_msgs::GoalStatus::ACTIVE:
		case actionlib_msgs::GoalStatus::PREEMPTING:
			goal_active.setAborted(result, msg);
			return true;
	}
	return false;
}

template <class ActionSpec> bool RTTSimpleActionServer<ActionSpec>::cancelActive(const Result& result, const std::string& msg)
{
	if (!goal_active.isValid()) return false;
	switch (goal_active.getGoalStatus().status) {
		case actionlib_msgs::GoalStatus::ACTIVE:
		case actionlib_msgs::GoalStatus::PREEMPTING:
			goal_active.setCanceled(result, msg);
			return true;
	}
	return true;
}

template <class ActionSpec> bool RTTSimpleActionServer<ActionSpec>::succeedActive(const Result& result, const std::string& msg)
{
	if (!goal_active.isValid()) return false;
	switch (goal_active.getGoalStatus().status) {
		case actionlib_msgs::GoalStatus::ACTIVE:
		case actionlib_msgs::GoalStatus::PREEMPTING:
			goal_active.setSucceeded(result, msg);
			return true;
	}
	return false;
}

template <class ActionSpec> void RTTSimpleActionServer<ActionSpec>::goalCallback(GoalHandle gh) 
{
	if (goal_pending.isValid()) {
		switch (goal_pending.getGoalStatus().status) {
			case actionlib_msgs::GoalStatus::PENDING:
			case actionlib_msgs::GoalStatus::RECALLING:
				goal_pending.setRejected(Result(), "Pending goal is replaced by new received goal.");
				break;
		}
	}
	goal_pending = gh;
	if (isPending() && newGoalHook) {
		newGoalHook(*goal_pending.getGoal());
	}
}

template <class ActionSpec> void RTTSimpleActionServer<ActionSpec>::cancelCallback(GoalHandle gh) {
	if (!gh.isValid()) return;
	switch (gh.getGoalStatus().status) {
		case actionlib_msgs::GoalStatus::ACTIVE:
		case actionlib_msgs::GoalStatus::PREEMPTING:
		case actionlib_msgs::GoalStatus::PENDING:
		case actionlib_msgs::GoalStatus::RECALLING:
			if (goal_active == gh) {
				if (cancelGoalHook) cancelGoalHook();
				// cancelActive(Result(), "Active goal is canceled by client request.");
			}
			else if (goal_pending == gh) {
				gh.setCanceled(Result(), "Pending goal is canceled by client request.");
			}
			else {
				gh.setCanceled(Result(), "Unknown goal.");
			}
			break;
	}
}

} // namespace rtt_actionlib

#endif  /*RTT_SIMPLE_ACTION_SERVER_HPP*/
