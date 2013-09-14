RTT TF
======

This package provides an Orocos RTT component for utilizing the TF rigid body
transform library from within Orocos. This component provides a "tf" service
with operations for requesting and broadcasting sing and batch transforms.

### Usage

In general, a given process only needs a single TF component. This component
provides the following operations both on its bare interface and also on the
provided "tf" RTT service.

* "lookupTransform" Lookup the most recent transform from source to target.
  * "target", Target frame
  * "source", Source frame

* "lookupTransformAtTime" Lookup the most recent transform from source to target at a specific time.
  * "target", Target frame
  * "source", Source frame
  * "common\_time", `ros::Time` The common time at which the transform should be computed

* "broadcastTransform" Broadcast a stamped transform immediately.
  * "transform", `geometry\_msgs::TransformStamped`

* "broadcastTransforms" Broadcast a stamped transform immediately.
  * "transforms", `std::vector<geometry_msgs::TransformStamped>`

To use these operations, you need to instantiate an `rtt_tf::RTT_TF` component,
and then connect it with whichever components you intend to have use it:

```cpp
import("rtt_ros");

// Import and load the TF component
ros.import("rtt_tf");
loadComponent("tf","rtt_tf::RTT_TF");

// Connect operations the tf component's provided "tf" service to another
// component's required "tf" service (easily created with the C++ API shown in the
// next section)
loadComponent("my_component","MyComponent");
connectServices("my_component","tf");
```

### C++ API

The [rtt\_tf/tf\_interface.h](include/rtt_tf/tf_interface.h) header includes the
`rtt_tf::TFInterface` class which adds a required service to a given RTT
component. This can be used for more easily connecting with the TF component.

It can be used to create the RTT service requester "tf" like so:

```cpp
#include <rtt_tf/tf_interface.h>
//...
class MyComponent : public RTT::TaskContext {
  private:
    rtt_tf::TFInterface tf_;
  public:
    MyComponent(std::string &name) : 
      RTT::TaskContext(name),
      // Add the services
      tf_(this)
    { /* ... */ }

    bool startHook() {
      // Make sure tf is ready & connected
      return tf_.ready();
    }

    void updateHook() {
      // Lookup a transform
      geometry_msgs::TransformStamped tform = tf_.lookupTransform("/world","/my_frame");
      // ...
    }
};

```
