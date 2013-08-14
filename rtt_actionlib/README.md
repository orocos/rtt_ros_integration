RTT Actionlib
=============

***NOTE: This is a preliminary design document and rtt\_actionlib is currently
unimplemented.***

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
subservice) of a TaskContext. These are creted by delegating to an
RTTActionServer. The RTTActionServer will create the necessary RTT ports and
bind them to the user-supplied callbacks.

```cpp
class SomeComponent : public RTT::TaskContext {
private:
  
  // Convenience typedefs
  typedef rtt_actionlib::RTTActionServer<some_msgs::SomeAction> RTTActionServerType;
  typedef actionlib::ServerGoalHandle<some_msgs::SomeAction> GoalHandle;

  // RTT action server
  boost::shared_ptr<RTTActionServerType> rtt_action_server_;

public:

  // Component constructor
  SomeComponent(std::string name) : TaskContext(name, RTT::PreOperational)
  { 
    // Initialize RTT action server (creates data ports)
    rtt_action_server_.reset(new RTTActionServerType(this->provides(), RTT::OwnThread));
    
    // Bind action server goal and cancel callbacks
    rtt_action_server_->registerGoalCallback(boost::bind(&SomeComponent::goalCallback, this, _1));
    rtt_action_server_->registerCancelCallback(boost::bind(&SomeComponent::cancelCAllback, this, _1));
  }

  // RTT configure hook
  bool configureHook() {
    return true;
  }

  // RTT start hook
  bool startHook() {
    // Start action server
    rtt_action_server_.start();
    return true;
  }

  // RTT update hook
  void updateHook() {
    // Pursue goal...

    // EXAMPLE //
    if(current_gh_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE) {
      current_gh_.setSucceeded();
    }
    // EXAMPLE //
  }
  
  // Accept/reject goal requests here
  // NOTE: Since we created the RTT action server with `RTT::OwnThread`, calls to
  //       this member function will be serialized with updateHook()
  void goalCallback(GoalHandle gh) {
    // EXAMPLE //
    // Always preempt the current goal and accept the new one
    if(current_gh_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE) {
      current_gh_.setCanceled();
    }
    gh.setAccepted();
    current_gh_ = gh;
    // EXAMPLE //
  }

  // Handle preemption here
  // NOTE: Since we created the RTT action server with `RTT::OwnThread`, calls to
  //       this member function will be serialized with updateHook()
  void cancelCallback(GoalHandle gh) {
    // EXAMPLE //
    if(current_gh_ == gh && current_gh_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE) {
      current_gh_.setCanceled();
    }
    // EXAMPLE //
  }
};
```

Second, the ports need to be connected to ROS topics. This can be done easily
with the "actionlib" service as shown below in Orocos .ops script:

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
##  some_component_name.goal (in/out)
##  some_component_name.cancel (in/out)
##  some_component_name.status (out/in)
##  some_component_name.result (out/in)
##  some_component_name.feedback (out/in)
## These ports can be created by the RTTActionServer or RTTActionClient.

some_component_name.actionlib.connect("/some/ros/namespace/my_action")

## Alternatively, connect an actionlib interface to a provided sub-service:

some_component_name.actionlib.connect(
  "some_prov.ided_service",
  "/some/ros/namespace/my_other_action")

```

Alternatively, each data port could be streamed to the appropriate action
namespace manually.

Use Cases
---------

### Sending a Reference Trajectory to a Controller 




