rtt_dynamic_reconfigure
=======================

This package provides a way to manipulate the properties of an Orocos RTT
component via the ROS [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure) interface.

Dynamic reconfigure uses a combination of ROS topics, services, and the 
parameter server to enable quick reconfiguration of parameters over the
network.

## Usage

#### With an automatically generated config

The easiest way to use rtt_dynamic_reconfigure is to use the special `AutoConfig` config type. `AutoConfig` automatically
generates a dynamic_reconfigure config description from the set of properties of the `TaskContext` it is loaded into. No
new properties will be created.

rtt_dynamic_reconfigure already comes with a precompiled service plugin for the `AutoConfig` type.
Simply add a dynamic_reconfigure
server to any component by loading the `reconfigure` service:

```cpp
import("rtt_dynamic_reconfigure");
loadService("my_component", "reconfigure")
my_component.reconfigure.advertise("/my_component")
```
The disadvantage of this method is that the minimum and maximum values requrired for dynamic_reconfigure are unknown. The current values
of the properties are advertised as default values. However, you can overwrite the automatically choosen minimum/maximum values before
calling the `advertise()` operation:

```cpp
import("rtt_dynamic_reconfigure");
loadService("my_component", "reconfigure")
my_component.reconfigure.min.int_property = 0
my_component.reconfigure.max.int_property = 100
my_component.reconfigure.advertise("~/my_component")
```

#### Using a custom config file

The rtt_dynamic_reconfigure package comes with a templated `rtt_dynamic_reconfigure::Server<ConfigType>`
class that implements the core functionality of a dynamic_reconfigure server. Usually the `ConfigType`
class header is generated automatically from a `*.cfg` file by dynamic_reconfigure's `generate_dynamic_reconfigure_options()`
cmake macro. This config file describes all available parameters and their minimum, maximum and default values.

In order to use rtt_dynamic_reconfigure with a custom config, you first need to create the `.cfg` file as explained in
[this tutorial](http://wiki.ros.org/dynamic_reconfigure/Tutorials/HowToWriteYourFirstCfgFile). Afterwards you can add a
rtt_dynamic_reconfigure service plugin to your package:

```cmake
project(my_package)
find_package(dynamic_reconfigure)
generate_dynamic_reconfigure_options(cfg/MyPackage.cfg)

orocos_plugin(my_package_reconfigure_service src/reconfigure_service.cpp)
add_dependencies(rtt_dynamic_reconfigure_tests_service ${PROJECT_NAME}_gencfg)
```

The `reconfigure_service.cpp` source file instantiates the `rtt_dynamic_reconfigure::Server<ConfigType>` for the new config type `MyPackageConfig`, implements the `rtt_dynamic_reconfigure::Updater<ConfigType>` class that explains how to fill the config from a
PropertyBag and vice-versa and registers the server as a service plugin:

```cpp
#include <rtt_dynamic_reconfigure/server.h>
#include <my_package/MyPackageConfig.h>         // <-- This header is created by generate_dynamic_reconfigure_options(cfg/MyPackage.cfg)

using namespace my_package;

namespace rtt_dynamic_reconfigure {

template <>
struct Updater<MyPackageConfig> {
  static bool propertiesFromConfig(MyPackageConfig &config, uint32_t level, RTT::PropertyBag &bag) {
    setProperty<int>("int_param", bag, config.int_param);
    setProperty<double>("double_param", bag, config.double_param);
    setProperty<std::string>("str_param", bag, config.str_param);
    setProperty<bool>("bool_param", bag, config.bool_param);
    return true;
  }
  static bool configFromProperties(MyPackageConfig &config, const RTT::PropertyBag &bag) {
    getProperty<int>("int_param", bag, config.int_param);
    getProperty<double>("double_param", bag, config.double_param);
    getProperty<std::string>("str_param", bag, config.str_param);
    getProperty<bool>("bool_param", bag, config.bool_param);
    return true;
  }
};

} // namespace rtt_dynamic_reconfigure

RTT_DYNAMIC_RECONFIGURE_SERVICE_PLUGIN(MyPackageConfig, "my_package_reconfigure")
```

The `rtt_dynamic_reconfigure::setProperty<T>(...)` and `rtt_dynamic_reconfigure::getProperty<T>(...)` helper functions can be used in the `Updater` implementation. Properties that do not exist yet in the owner's TaskContext will be created automatically.

Alternatively, the TaskContext in which the service is loaded can inherit and implement the `Updater<MyPackageConfig>` class directly. In this case you do not need to provide a specialized version of it.

*Note:* The `Updater<ConfigType>::propertiesFromConfig(...)` implementation should create properties that are
references to the respective fields in the `ConfigType` struct. This is the case if the properties
are added with `bag->addProperty(const std::string &name, T &attr)` or with the `rtt_dynamic_reconfigure::setProperty<T>(...)`
helper function.

Once the service plugin is compiled you can load it in any component. The properties with the given names

```cpp
import("rtt_ros");
ros.import("my_package");
loadService("my_component", "my_package_reconfigure")
my_component.reconfigure.advertise("~/my_component")
```

#### Overriding the update operation

Normally rtt_dynamic_reconfigure updates all properties of the TaskContext with the standard `RTT::updateProperties()` call
running in the owner's thread. Properties cannot be updated while the `updateHook()` is executed. For the case
you want more control over the property updates, you can add a `bool updateProperties(const RTT::PropertyBag &source, uint32_t level)` operation with a custom implementation to the owner component. If this operation exists, it is used instead of the default implementation. The `source` bag is the bag filled in a previous `Updater<ConfigType>::propertiesFromConfig(...)` call.

#### Adding a property update notification callback

Sometimes it is required that the component is notified whenever properties have been updated by rtt_dynamic_reconfigure. If a `void notifyPropertiesUpdated(uint32_t level)` operation exists, it is called after every parameter update from a ROS service call.

## Service API

#### reconfigure.advertise(string ns)

Advertise the dynamic_reconfigure topics and service server in the namespace `ns`.

#### reconfigure.updated()

Notifies the rtt_dynamic_reconfigure server that some property values have been updated and need to be republished to
update the user interface.

#### reconfigure.refresh()

Refreshs the config description with their updated minimum, maximum and default values.
For the `AutoConfig` config type also rediscovers newly added properties and removes deleted ones.

#### reconfigure.min, reconfigure.max, reconfigure.dflt

These `PropertyBag`s mirror the properties of the dynamic_reconfigure config and hold the minimum, maximum and default values.
Call `reconfigure.refresh()` after every update to republish the updated config description.

