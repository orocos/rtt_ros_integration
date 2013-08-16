
#include <rtt_actionlib/rtt_action_server.h>

using namespace rtt_actionlib;

template <class ActionSpec>
RTTActionServer<ActionSpec>::RTTActionServer(
    RTT::Service::shared_ptr service,
    const RTT::ExecutionThread exec_thread_type)
{
  
}

template <class ActionSpec>
RTTActionServer<ActionSpec>::initialize()
{
  
}

template <class ActionSpec>
bool RTTActionServer<ActionSpec>::addPorts(
    RTT::Service::shared_ptr service,
    const RTT::ExecutionThread exec_thread_type)
{
  bool provide_cancel = true;
  bool provide_feedback = true;

  // Create the ports
  if(!action_bridge_.createServerPorts(service, exec_thread_type, provide_cancel, provide_feedback)) {
    return false;
  }

  // Add the ports to the service
  bool success = true;

  success &= service->addEventPort(action_bridge_.goal(), boost::bind(&RTTActionServer<ActionSpec>::goalCallback, this, _1));
  success &= provide_cancel ? service->addPort(action_bridge_.cancel(), boost::bind(&RTTActionServer<ActionSpec>::cancelCallback, this, _1)) : true;
  success &= service->addPort(action_bridge_.result());
  success &= service->addPort(action_bridge_.status());
  success &= provide_feedback ? service->addPort(action_bridge_.feedback()) : true;

  return success;
}
