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

Using this service requires that the RTT properties of a given component match
the names of their associated ROS parameters, modulo the namespace resolution
policy (see below).

## Usage

### Service Interface

#### Constants (name resolution policies):

* ***RELATIVE***   Relative resolution:  "name" -> "name"
* ***ABSOLUTE***   Absolute resolution:  "name" -> "/name"
* ***PRIVATE***    Private resolution:   "name" -> "~name"
* ***COMPONENT***  Component resolution: "name" -> "~COMPONENT\_NAME/name"

#### Operations (getting/setting params)

##### Operations for getting all properties
* **getAll()** or **getAllComponentPrivate()** Attempt to get all properties of this component (and its sub-services)
  from the ROS parameter server in the **COMPONENT** namespace.
* **getAllRelative()** Attempt to get all properties of this component (and its sub-services)
  from the ROS parameter server in the relative namespace.
* **getAllAbsolute()** Attempt to get all properties of this component (and its sub-services)
  from the ROS parameter server in the absolute namespace.
* **getAllPrivate()** Attempt to get all properties of this component (and its sub-services)
  from the ROS parameter server in the node's private namespace.

##### Operations for setting all properties
* **setAll()** or **setAllComponentPrivate()** Stores all properties of this component (and its sub-services)
  on the ROS parameter server from the similarly-named property in the **COMPONENT**'s private namespace.
* **setAllRelative()** Stores all properties of this component (and its sub-services)
  on the ROS parameter server from the similarly-named property in the relative namespace.
* **setAllAbsolute()** Stores all properties of this component (and its sub-services)
  on the ROS parameter server from the similarly-named property in the absolute namespace.
* **setAllPrivate()** Stores all properties of this component (and its sub-services)
  on the ROS parameter server from the similarly-named property in the node's private namespace.


##### Operations for getting single properties

* **get(name,policy)** Attempt to get the property named **name** (or populates the properties of a named RTT sub-service)
  from the ROS parameter namespace specified by **policy**.
* **getRelative(name)**
* **getAbsolute(name)**
* **getPrivate(name)**
* **getComponentPrivate(name)**

##### Operations for setting single properties

* **set(name,policy)** Attempt to set the property named **name** (or stores the properties of a named RTT sub-service)
  in the ROS parameter namespace specified by **policy**.
* **setRelative(name)**
* **setAbsolute(name)**
* **setPrivate(name)**
* **setComponentPrivate(name)**

### Scripting Interface

Components which were never meant to be used with ROS parameters can have their
properties set from the ROS parameter server through the scripting interface:

```
// Import packages
import("rtt_ros")
ros.import("rtt_rosparam")

// Create your components ...
laodService("my_component","rosparam")

// Try to get all parameters from the component namespace "~my_component/..."
my_component.rosparam.getAll()

// Try to get the "robot_description" property from the absolute namespace "/robot_description"
my_component.rosparam.getAbsolute("robot_description")
// Alternatively:
my_component.rosparam.get("robot_description",my_component.rosparam.ABSOLUTE)
```

### C++ Interface

If your component is designed to be used with ROS parameters, you can also
easily access the rosparam service from C++ using an RTT ServiceRequester.

For example, a simple component which gets the "robot\_description" ROS param
from the global namespace ("/robot\_description") and the "publish\_period" ROS
param from the node's private namespace ("~publish\_period") would look something
like the following:

```cpp
// Include the service requester interface
#include <rtt_rosparam/rosparam.h>


class MyComponent : public RTT::TaskContext {
  private:
    // Storage for RTT properties
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
      boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
          this->getProvider<rtt_rosparam::ROSParam>("rosparam");

      // Get the parameters
      if(rosparam) {
        // Get the ROS parameter "/robot_description"
        bool result = rosparam->getAbsolute("robot_description");

        // Get the ROS parameter "~publish_period"
        bool result = rosparam->getPrivate("publish_priod");
      }
    }

    // ...
}
```

See the API documentation for more information.
