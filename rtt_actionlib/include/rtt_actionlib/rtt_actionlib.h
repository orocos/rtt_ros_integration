#ifndef __RTT_ACTIONLIB_H
#define __RTT_ACTIONLIB_H

namespace rtt_actionlib {
  bool has_action_server_ports(RTT::Service::shared_ptr service) {
    // Make sure service isn't null
    if(service.get() == NULL) {
      return false;
    }

    RTT::base::PortInterface goal, cancel, status, result, feedback;
    RTT::base::InputPortInterface goal_in, cancel_in;
    RTT::base::OutputPortInterface status_out, result_out, feedback_out;

    // Get ports
    goal = service->getPort("goal");
    cancel = service->getPort("cancel");
    status = service->getPort("status");
    result = service->getPort("result");
    feedback = service->getPort("feedback");

    // Get port directions
    goal_in = dynamic_cast<RTT::base::InputPortInterface*>(goal);
    cancel_in = dynamic_cast<RTT::base::InputPortInterface*>(cancel);

    status_out = dynamic_cast<RTT::base::OutputPortInterface*>(status);
    result_out = dynamic_cast<RTT::base::OutputPortInterface*>(result);
    feedback_out = dynamic_cast<RTT::base::OutputPortInterface*>(feedback);

    // Check port existence / directions
    bool valid = true;

    valid &= goal_in != NULL;
    valid &= cancel_in != NULL || cancel == NULL;
    valid &= status_out != NULL;
    valid &= result_out != NULL;
    valid &= feedback_out != NULL || feedback == NULL;

    return valid;
  }
}

#endif // ifndef __RTT_ACTIONLIB_H
