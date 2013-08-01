
rtt\_rosservice
===============

***Nomenclature Warning:*** *This package provides an Orocos Plugin which
supports connecting "ROS Services" to "Orocos Operations" by providing
a global "Orocos Service". A "ROS Service" is a remote procedure call
that hapens over ROS and an "Orocos Service" is a grouping of properties,
functions, and data ports. As such, this package provides an "Orocos
Service" called "RosServiceService".* 

This Orocos Service Plugin provides the following operations:
* **service client** `rosservice.client(ROS_SERVICE_NAME, ROS_SERVICE_TYPE)`
* **service server** `rosservice.server(ROS_SERVICE_NAME, ROS_SERVICE_TYPE)`

To connect an Orocos operation to a ROS service via ops script from within an
Orocos DeploymentComponent: 

```cpp
import("rtt_rosservice")
import("rtt_ros")
ros.import("rtt_std_srvs")
connectServices("my_comp.some.rtt.service.empty",rosservice.client("/some/ros/namespace/empty","std_srvs/Empty"))
```




