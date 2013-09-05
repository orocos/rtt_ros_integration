RTT ROSParam
============

This package provides an RTT service and service-requester for associating RTT
component properties with ROS parameters.

## Design

This RTT service enables both setting/getting individual RTT properties of a
given RTT component to/from the ROS parameter server, as well as setting/getting
all properties of a component simultaneously. The operations map directly onto
operations in the roscpp library, except they only store the retrieved values in
RTT properties of the given component.

## Usage

### Service Interface

#### Constants (name resolution policies):

* ***RELATIVE***   Relative resolution:  "name" -> "name"
* ***ABSOLUTE***   Absolute resolution:  "name" -> "/name"
* ***PRIVATE***    Private resolution:   "name" -> "~name"
* ***COMPONENT***  Component resolution: "name" -> "~COMPONENT\_NAME/name"

#### Operations (getting/setting params)

* ***getAll()*** Attempt to get all properties from ros parameters in the
  **COMPONENT** namespace.
* ***setAll()*** Set ROS parameters in the **COMPONENT** namespace from all
  properties of this component.

* ***get(name,policy)*** Attempt to get the property named **name** from the
  namespace specified by **policy**.
* ***getRelative(name)***
* ***getAbsolute(name)***
* ***getPrivate(name)***
* ***getComponentPrivate(name)***

* ***set(name,policy)*** Attempt to set the property named **name** from the
  namespace specified by **policy**.
* ***setRelative(name)***
* ***setAbsolute(name)***
* ***setPrivate(name)***
* ***setComponentPrivate(name)***

### C++ Interface

If your component is designed to be used with ROS parameters, you can also
easily access the rosparam service from C++ using an RTT ServiceRequester.

For example, a simple component which gets the "robot\_description" ROS param
from the global namespace ("/robot\_description") and the "publish\_period" ROS
param from the node's private namespace ("~publish\_period") would look something
like the following:

```cpp
// Include the service requester interface
#include <rtt_rosparam/ROSParam.h>


class MyComponent : public RTT::TaskContext {
  private:
    std::string robot_description_;
    double publish_period_;

  public:
    MyComponent::MyComponent(std::string &name) : RTT::TaskContext(name) {
      // Add some properties
      this->addProperty("robot_description",robot_description_);
      this->addProperty("publish_period",publish_period_);
    }

    // ... 

    bool configureHook() {
      bool all_params_found = true;

      // Get the rosparam service requester
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam = this->getProvider<rtt_rosparam::ROSParam>("rosparam");

      // Get the parameters
      if(rosparam) {
        // Get the ROS parameter "/robot_description"
        bool result = rosparam->getAbsolute("robot_description"); // 2.

        // Get the ROS parameter "~publish_period"
        bool result = rosparam->getPrivate("publish_priod"); // 2.
      }
    }

    // ...
}
```

See the API documentation for more information.
