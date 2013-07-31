
rtt\_rosservice
===============

To connect an Orocos operation to a ROS service via ops script from within an
Orocos DeploymentComponent: 
```cpp
import("rtt_rosservice")
connectServices("my_comp.some.rtt.service.add_two_ints",rosservice.client("/some/ros/namespace/add_two_ints"))
```




