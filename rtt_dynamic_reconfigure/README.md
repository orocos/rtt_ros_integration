rtt_dynamic_reconfigure
=======================

This package provides a way to manipulate the properties of an Orocos RTT
component via the ROS dynamic_reconfigure interface.

Dynamic reconfigure uses a combination of ROS topics, services, and the 
parameter server to enable quick reconfiguration of parameters over the
network.

## Usage

```cpp

import("rtt_ros");
ros.import('rtt_dynamic_reconfigure');

loadService("my_component","dynamic_reconfigure")
```
