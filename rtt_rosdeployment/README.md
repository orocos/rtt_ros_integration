rtt_rosdeployment
-----------------

This package provides an easy way to interact with an Orocos deployment
component over ROS. This uses ROS srv types in the
[rtt_ros_msgs](../rtt_ros_msgs) package.

## Supported Operations (C++ : ROS Service Name)

* `DeploymentCompnent::runScript`:`run_script`
* `DeploymentCompnent::getPeerList`:`get_peer_list`

## Usage

You can advertise the services from a deployment component like the following:

```cpp
import("rtt_rosdeployment");
this.loadservice("rosdeployment");
```
