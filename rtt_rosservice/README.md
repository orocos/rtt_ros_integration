rtt\_rosservice
===============

***Nomenclature Warning:***  *A "ROS Service" is a remote procedure call
that hapens over the ROS communication protocols and an "Orocos/RTT Service" is
a grouping of properties, functions, and data ports. "ROS Services" satisfy a
similar role to "Orocos/RTT Operations".* 

## Contents 

This package provides an Orocos Plugin which supports connecting "ROS Services"
to "Orocos Operations" by providing typekit generation and a few "Orocos
Service Plugins".

### Plugins

This package provides both a global RTT service and a task-scoped service. The
global service, `rosservice_registry` is used to register factories for
creating proxies to ROS service clients and servers. The task-scoped service is
ued to bind RTT operations and operation callers to ROS services. In general,
users will only use the task-scoped RTT service, similarly to how the
`rtt_rosparam` service is used.

The task-scoped RTT service `rosservice` provides the following operations:
* `rosservice.connect(RTT_OPERATION_NAME, ROS_SERVICE_NAME, ROS_SERVICE_TYPE)`
  * Connect an RTT operation to ROS service.
  * `RTT_OPERATION_NAME`: The task-scoped operation/operation caller name, with provided/required services separated by dots (like `foo.bar.baz.op`)
  * `ROS_SERVICE_NAME`: The name of the service client/server in the ROS graph (like `/some/ros/ns/my_service`)
  * `ROS_SERVICE_TYPE`: The full typename of the service (like `std_srvs/Empty`)

The global RTT service `rosservice_registry` provides the following operations:
* `rosservice_registry.registerServiceFactory(FACTORY)`: Register a ROS service factory
* `rosservice_registry.hasServiceFactory(TYPENAME)`: Check if ROS service type has been registered
* `rosservice_registry.geServiceFactory(TYPENAME)`: Get a ROS service client/server factory

### Code Generation

This package also provides facilities for generating typekits for ROS service
types defined in `.srv` files as well as generating plugins which register ROS
service types with the `rosservice_registry` service.

## Usage

### Connecting RTT Operations to ROS Services

To connect an Orocos operation to a ROS service via .ops script from within an
Orocos DeploymentComponent: 

```python
## Imports
import("rtt_rosservice")
import("rtt_std_srvs")

## Load some application-specific component
loadComponent("some_component_name","some_component_package::SomeComponent")
// Load the rosservice RTT service for this components
loadService("some_component_name","rosservice")

## Expose a provided operation of this component as a ROS service
some_component_name.rosservice.connect(
  "some_provided_service.some_operation",
  "/some/ros/namespace/empty", "std_srvs/Empty")

## Expose a ROS service to this component
some_component_name.rosservice.connect(
  "some_Required_service.some_operation_caller",
  "/some/ros/namespace/empty", "std_srvs/Empty")
```

### Making a ROS Service Type Available



## Design

The `rosservice_registry` RTT service contains a list of ROS service clients
and servers which are associated with RTT operations and operationcallers,
respectively.  The `rosservice.connectOperation` operation, inspects whether
the first argument is an operation or operationcaller. If it is an **RTT
operation**, it will instantiate a **ROS service server** wrapped in an **RTT
operationCaller** to call the operation. If it is an **RTT operationCaller**,
it will instantiate a **ROS service client** wrapped in an **RTT operation** to
be called by the operation caller. 

The provided and required services on which the wrapper operations and
operationCallers are created are private to the ROS service service. 

