rtt_rospack
-----------

This package provides an easy way to access rospack from within Orocos RTT in
order to resolve the path to a ROS package on the current system. 

## Usage

You can resolve the path to a ROS package on your system in the following way
using rtt_rospack:

```cpp
import("rtt_ros");
ros.import("rtt_rospack");

var string the_path = rospack.find("actionlib");
```
