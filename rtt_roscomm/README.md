RTT ROS Communications
======================

***Nomenclature Warning:***  *A "ROS Service" is a remote procedure call
that hapens over the ROS communication protocols and an "Orocos/RTT Service" is
a grouping of properties, functions, and data ports. "ROS Services" satisfy a
similar role to "Orocos/RTT Operations".* 

Contents 
--------

This package serves several purposes. It provides:
* An Orocos RTT Service for publishing and subscribing to ROS topics
* Orocos RTT Services for calling and serving ROS services
* Orocos typekits for ROS message primitive types
* A template for generating wrapper packages for ROS .msg and .srv files
  * typekits for .msg files
  * transport plugin for .msg files
  * ros service proxy factories for .srv files

### Plugins

#### ROS Topics

This package provides a global RTT service for creating real-time-safe
connections between ROS topics and Orocos RTT data ports.

This package provides two Orocos connection policies: buffered and 
unbufferd connections to ROS topics. Publishing and subscribing are done
with the same command, and the topic type is inferred from the Orocos port
type. Connection policies are created with these operations:

* `rostopic.connection(TOPIC_NAME)`: Creates a connection with a buffer length
  of 1.
* `rostopic.connectionBuffered(TOPIC_NAME, BUFFER_LENGTH)`: Creates a
  connection with a user-supplied buffer length.

#### ROS Services

This package provides both a global RTT service and a task-scoped service for
facilitating communication with ROS services. The global service,
`rosservice_registry` is used to register factories for creating proxies to ROS
service clients and servers. The task-scoped service is ued to bind RTT
operations and operation callers to ROS services. In general, users will only
use the task-scoped RTT service, similarly to how the `rtt_rosparam` service is
used.

The task-scoped RTT service `rosservice` provides the following operations:
* `rosservice.connect(RTT_OPERATION_NAME, ROS_SERVICE_NAME, ROS_SERVICE_TYPE)`
  * Connect an RTT operation to ROS service. Note that this is the same
    function whether the RTT operation is an operation or an operation caller.
  * `RTT_OPERATION_NAME`: The task-scoped operation/operation caller name, with
    provided/required services separated by dots (like `foo.bar.baz.op`)
  * `ROS_SERVICE_NAME`: The name of the service client/server in the ROS graph
    (like `/some/ros/ns/my_service`)
  * `ROS_SERVICE_TYPE`: The full typename of the service (like
    `std_srvs/Empty`)

The global RTT service `rosservice_registry` provides the following operations:
* `rosservice_registry.registerServiceFactory(FACTORY)`: Register a ROS service
  factory
* `rosservice_registry.hasServiceFactory(TYPENAME)`: Check if ROS service type
  has been registered
* `rosservice_registry.geServiceFactory(TYPENAME)`: Get a ROS service
  client/server factory

### Code Generation

This package also provides facilities for generating typekits for ROS service
types defined in `.srv` files as well as generating plugins which register ROS
service types with the `rosservice_registry` service.


Usage
-----

### Connecting an Orocos Port to a ROS Topic

```python
## Imports
import("rtt_roscomm")
# Publish
stream("my_component.my_output", rostopic.connection("my_ros_output"))
# Subscribe
stream("my_component.my_input", rostopic.connection("my_ros_input"))
```

### Connecting RTT Operations to ROS Services

To connect an Orocos operation to a ROS service via .ops script from within an
Orocos DeploymentComponent: 

```python
## Imports
import("rtt_roscomm")
import("rtt_std_srvs")

## Load some application-specific component
loadComponent("some_component_name","some_component_package::SomeComponent")
## Load the rosservice RTT service for this components
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


### Making a Package's ROS .msg and .srv Types Available

Generally, you can crete a catkin package simply with the `create_rtt_srvs`
script by running:

```shell
rosrun rtt_roscomm create_rtt_msgs my_srvs
```

All this does is create a package with the following CMakeLists.txt and
corresponding package.xml:

```cmake
project(rtt_std_srvs)
find_package(catkin REQUIRED COMPONENTS genmsg rtt_roscomm @pkgname@)

# Generate typekits for ros .msg files
ros_generate_rtt_typekit(@pkgname@)
# Generate the plugin which makes the services in std_srvs available
ros_generate_rtt_service_proxies(@pkgname@)

# Call catkin_package after the above to export the proper targets
catkin_package(CATKIN_DEPENDS genmsg rtt_roscomm @pkgname@)

```

The `ros_generate_rtt_service_proxies()` cmake function will generate an RTT
plugin which registers factories for all of the services in the named package
when the plugin is loaded.


Design
------

### ROS Services

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


Todo
----

* Implement typekit generation (similar to rtt\_rostopic) so that services can
  be called from the taskbrowser.
* Automatically detect the type of ROS service from the service name or the
  operation signature.
