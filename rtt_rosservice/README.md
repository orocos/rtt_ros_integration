rtt\_rosservice
===============

***Nomenclature Warning:*** *This package provides an Orocos Plugin which
supports connecting "ROS Services" to "Orocos Operations" by providing
a global "Orocos Service". A "ROS Service" is a remote procedure call
that hapens over ROS and an "Orocos Service" is a grouping of properties,
functions, and data ports. As such, this package provides an "Orocos
Service" called "RosServiceService".* 


This Orocos Service Plugin creates a global service which provides the
following operations:
* **connect operation to ROS service** `rosservice.connectOperation(RTT_OPERATION_NAME, ROS_SERVICE_NAME, ROS_SERVICE_TYPE)`

It also provides a C++ API for connecting Oroocs operations to ROS services.

## Usage

To connect an Orocos operation to a ROS service via .ops script from within an
Orocos DeploymentComponent: 

```cpp
// Imports
import("rtt_rosservice")
import("rtt_std_srvs")

// Load some application-specific component
loadComponent("some_component_name","some_component_package::SomeComponent")
// Load the rosservice RTT service for this components
loadService("some_component_name","rosservice")

// Expose a provided operation of this component as a ROS service
some_component_name.rosservice.connect(
  "some_provided_service.some_operation",
  "/some/ros/namespace/empty", "std_srvs/Empty")

// Expose a ROS service to this component
some_component_name.rosservice.connect(
  "some_Required_service.some_operation_caller",
  "/some/ros/namespace/empty", "std_srvs/Empty")
```

## Design

The `rosservice` RTT service contains a list of ROS service clients and servers
which are associated with RTT operations and operationcallers, respectively.
The `rosservice.connectOperation` operation, inspects whether the first argument
is an operation or operationcaller. If it is an *RTT operation*, it will
instantiate a *ROS service server* wrapped in an *RTT operationCaller* to call
the operation. If it is an *RTT operationCaller*, it will instantiate a *ROS
service client* wrapped in an *RTT operation* to be called by the operation
caller. 

The provided and required services on which the wrapper operations and
operationCallers are created are private to the ROS service service. 

**NOTE** global services can't _require_ services / operations

**NOTE** you can't resolve an RTT uri unless the component is a peer of roservice

