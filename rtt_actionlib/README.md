RTT Actionlib
=============

Actionlib action servers provide the following interfaces, and actionlib action
clients reciprocate:
* Inputs:
  * Goal topic
  * Cancel topic
* Outputs: 
  * Status topic
  * Feedback topic
  * Result topic (per goal)

For an Orocos RTT component to provide an actionlib interface, it needs to
provide the topics described above. In order to expose actionlib interfaces
to components running in real-time threads, we should use the rtt data ports
to connect to these topics. In order to make it easy to connect these ports,
this package provides a C++ API for constructing the appropriate RTT data ports
and an RTT service for connecting those ports to ROS topics if necessary.

Contents
--------

This package provides a C++ API and RTT service for constructing and connecting
several RTT ports to a ROS actionlib action namespace.

### Providng Actions from Orocos

In the standard ROS actionlib server, subscribing and publishing ROS messages on
the actionlib topics are handled automatiaclly by the ActionServer class, and
users are meant to delegate to an instance of this class. ActionServer then
calls short-running "goal" and "cancel" callbacks when it receives the
appropriate messages. It is in these user-supplied callbacks that goal
handlesare set to be accepted/rejected or preempted, respectively. 

The user sets the status of these handles by calling member functions
`setSucceeded()`, `setAborted()` etc. on the handles. These calls then call the
parent ActionServer's `publishResult()`,`publishStatus()` etc. member functions.
The "normal" ActionServer implements these functions with ROS publishers and
subscribers, but for RTT, we want these to use data ports.

To this end, we implement an RTTActionServer C++ class which inherits from
ActionServerBase and implements the "publish" functions with RTT data ports and
binds the goal and cancel callbacks to RTT event ports.

### Calling Actions from Orocos

TBD

Usage
-----

First the appropriate RTT ports need to be created in a given service (or
subservice) of a TaskContext. These can be easily creted by delegating to an
RTTActionServer. The RTTActionServer will create the necessary RTT ports and
bind them to the user-supplied callbacks. For example, to add an actionlib
interface to a given compnent, you could do something similar to the following:

```cpp
class SomeComponent : public RTT::TaskContext {
private:
  
  // Convenience typedefs
  typedef actionlib::ServerGoalHandle<some_msgs::SomeAction> GoalHandle;

  // RTT action server
  rtt_actionlib::RTTActionServer<some_msgs::SomeAction> rtt_action_server_;

public:

  // Component constructor
  SomeComponent(std::string name) :
    TaskContext(name, RTT::PreOperational)
  { 
    // Add action server ports to this task's root service
    rtt_action_server_.addPorts(this->provides());
    
    // Bind action server goal and cancel callbacks (see below)
    rtt_action_server_.registerGoalCallback(boost::bind(&SomeComponent::goalCallback, this, _1));
    rtt_action_server_.registerCancelCallback(boost::bind(&SomeComponent::cancelCallback, this, _1));
  }

  // RTT start hook
  bool startHook() {
    // Start action server
    rtt_action_server_.start();
    return true;
  }

  // RTT update hook
  void updateHook() {
    // Pursue goal here
  }
  
  // Called by rtt_action_server_ when a new goal is received
  void goalCallback(GoalHandle gh) {
    // Accept/reject goal requests here
  }

  // Called by rtt_action_server_ when a goal is cancelled / preempted
  void cancelCallback(GoalHandle gh) {
    // Handle preemption here
  }
};
```

Second, the ports need to be connected to ROS topics. This can either be done
in C++ with hard-coded action topic names, or with the "actionlib" RTT service
similarly to the following:

```python
## Imports
import("rtt_ros");
ros.import("rtt_actionlib");

## Load some application-specific component
loadComponent("some_component_name","some_component_package::SomeComponent")

## Load the actionlib service
loadService("some_component_name","actionlib")

## Connect an actionlib interface to the task's root service
## This requires that the following ports exist with directions (server/client):
##  some_component_name._action_goal (in/out)
##  some_component_name._action_cancel (in/out)
##  some_component_name._action_status (out/in)
##  some_component_name._action_result (out/in)
##  some_component_name._action_feedback (out/in)
## These ports can be created by the RTTActionServer or RTTActionClient.

some_component_name.actionlib.connect("/some/ros/namespace/my_action")

## Alternatively, connect an actionlib interface to a provided sub-service:

some_component_name.actionlib.connectSub(
  "some_prov.ided_service",
  "/some/ros/namespace/my_other_action")

```

Future Work
-----------

* Add operation to actionlib service which connects goal/cancel callbacks to
  given RTT operations so that any RTT component with operations with the right
  types can be bound to an actionlib service
* Add action client support.
* Add a "simple" mode which implememnts a policy like SimpleActionServer





