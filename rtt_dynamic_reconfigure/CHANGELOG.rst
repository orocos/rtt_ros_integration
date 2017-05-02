^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rtt_dynamic_reconfigure
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.9.0 (2017-05-02)
------------------
* rtt_dynamic_reconfigure: create a partially filled PropertyBag from a Config message that only has a subset of fields
* rtt_dynamic_reconfigure: support both, non-const and const updateProperties callback signatures
  This is an improved version of https://github.com/orocos/rtt_ros_integration/pull/81 that is backwards-compatible
  to existing components that provide the const-variant of the callback.
* rtt_dynamic_reconfigure: allow the update callback to change the values in the propertybag
  This fixes a thread-safety issue with the previous commit b603585e9f74b3a553347301b44c73b0249856a1.
  But the user-defined update callback has to update the referenced bag manually in case it modified
  some property values.
* rtt_dynamic_reconfigure: publish updated property values in setConfigCallback() instead of desired once
  Rebuild new_config ConfigType from owner's PropertyBag before serializing and publishing dynamic_reconfigure msg from PropertyBag.
  The user might have modified the property values in one of the callbacks.
* rtt_dynamic_reconfigure: fixed potential deadlock in refresh() for RTT prior to 2.9
  We do not know for sure which thread is calling this method/operation, but we can check if the current
  thread is the same as the thread that will process the update/notify operation. If yes, we clone the
  underlying OperationCaller implementation and set the caller to the processing engine. In this case
  RTT <2.9 should always call the operation directly as if it would be a ClientThread operation:
  https://github.com/orocos-toolchain/rtt/blob/toolchain-2.8/rtt/base/OperationCallerInterface.hpp#L79
  RTT 2.9 and above already checks the caller thread internally and therefore does not require this hack.
* Added individual changelogs and bumped versions to 2.9.0
* Contributors: Johannes Meyer, Viktor Kunovski

2.8.5 (2017-03-28)
------------------
* Merge pull request `#86 <https://github.com/orocos/rtt_ros_integration/issues/86>`_ from orocos/rtt_dynamic_reconfigure-check-updated-properties
  rtt_dynamic_reconfigure: report updated property values in the service response (indigo-devel)
* Contributors: Johannes Meyer

2.8.4 (2016-11-26)
------------------

2.8.3 (2016-07-20)
------------------
* rtt_dynamic_reconfigure: fixed potential deadlock in refresh() for RTT prior to 2.9
* rtt_dynamic_reconfigure: set owner of default updateCallback implementation
* Contributors: Johannes Meyer

2.8.2 (2015-06-12)
------------------
* rtt_dynamic_reconfigure: added support for Property composition and decomposition and fixed reconfiguration of nested properties
* rtt_dynamic_reconfigure: fixed AutoConfig maximum value for type unsigned int
* rtt_dynamic_reconfigure: fixed property updating from config in AutoConfig
* Contributors: Johannes Meyer

2.8.1 (2015-03-16)
------------------
* see `rtt_ros_integratoin/CHANGELOG.rst <../rtt_ros_integration/CHANGELOG.rst>`_
