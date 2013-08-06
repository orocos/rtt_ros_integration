RTT Rostopic
============

This package serves three purposes. It provides:
 * An Orocos typekit for ROS message primitive types
 * A template for generating ROS message typekit packages
 * An Orocos RTT Service for publishing and subscribing to ROS topics
 
## Usage

### Connecting an Orocos Port to a ROS Topic

This package provides two Orocos connection policies: buffered and 
unbufferd connections to ROS topics. Publishing and subscribing are done
with the same command, and the topic type is inferred from the Orocos port
type. Connection policies are created with these operations:

 * **unbuffered** `rostopic.connect(TOPIC_NAME)`
 * **buffered** `rostopic.connectBuffered(TOPIC_NAME, BUFFER_LENGTH)`
 
### Example

```python
# Publish
stream("my_component.my_output", rostopic.connect("my_ros_output"))
# Subscribe
stream("my_component.my_input", rostopic.connect("my_ros_input"))
```
