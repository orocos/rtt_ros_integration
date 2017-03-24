/** Copyright (c) 2013, Jonathan Bohren, all rights reserved.
 * This software is released under the BSD 3-clause license, for the details of
 * this license, please see LICENSE.txt at the root of this repository.
 */

#include <rtt/os/startstop.h>
#include <rtt/Logger.hpp>
#include <rtt/deployment/ComponentLoader.hpp>

#include <rtt_roscomm/rostopic.h>
#include <rtt_roscomm/rosservice.h>
#include <std_msgs/typekit/String.h>
#include <std_srvs/Empty.h>

#include <ros/names.h>
#include <ros/service_manager.h>
#include <ros/this_node.h>

#include <boost/weak_ptr.hpp>

#include <gtest/gtest.h>

#include <algorithm>

TEST(TransportTest, OutOfBandTest)
{
  ros::V_string advertised_topics, subscribed_topics;

  // Import plugins
  ASSERT_TRUE(RTT::ComponentLoader::Instance()->import("rtt_ros", "" ));
  ASSERT_TRUE(RTT::ComponentLoader::Instance()->import("rtt_rosnode", "" ));
  ASSERT_TRUE(RTT::ComponentLoader::Instance()->import("rtt_roscomm", "" ));
  ASSERT_TRUE(RTT::ComponentLoader::Instance()->import("rtt_std_msgs", "" ));

  RTT::OutputPort<std_msgs::String> out("out");
  RTT::InputPort<std::string> in("in");

  // Create an out-of-band connection with ROS transport (a publisher/subscriber pair)
  // NOTE: The rtt-ros-primitives-transport installs a transport for the std::string type
  // which is compatible to std_msgs/String.
  std::string topic = ros::names::resolve("~talker");
//  EXPECT_TRUE(out.connectTo(&in, rtt_roscomm::topicLatched(topic)));
  EXPECT_TRUE(out.createStream(rtt_roscomm::topicLatched(topic)));
  EXPECT_TRUE(in.createStream(rtt_roscomm::topic(topic)));

  // Check that the publisher and subscriber have been successfully registered:
  ros::this_node::getAdvertisedTopics(advertised_topics);
  EXPECT_TRUE(std::find(advertised_topics.begin(), advertised_topics.end(),
                        topic) != advertised_topics.end());
  ros::this_node::getSubscribedTopics(subscribed_topics);
  EXPECT_TRUE(std::find(subscribed_topics.begin(), subscribed_topics.end(),
                        topic) != subscribed_topics.end());

  // publish and latch one sample
  std_msgs::String sample;
  sample.data = "Hello world!";
  out.write(sample);
  usleep(1000000);

  // read sample through input port
  std::string received;
  EXPECT_EQ(RTT::NewData, in.read(received) );
  EXPECT_EQ(sample.data, received);

  // Close connection
  out.disconnect();
  EXPECT_FALSE(out.connected());
  in.disconnect();
  EXPECT_FALSE(in.connected());

  // Check that the publisher and subscriber have been destroyed:
  advertised_topics.clear();
  subscribed_topics.clear();
  ros::this_node::getAdvertisedTopics(advertised_topics);
  EXPECT_FALSE(std::find(advertised_topics.begin(), advertised_topics.end(),
                        topic) != advertised_topics.end());
  ros::this_node::getSubscribedTopics(subscribed_topics);
  EXPECT_FALSE(std::find(subscribed_topics.begin(), subscribed_topics.end(),
                        topic) != subscribed_topics.end());
}

TEST(TransportTest, VectorTest)
{
  // Import plugins
  ASSERT_TRUE(RTT::ComponentLoader::Instance()->import("rtt_ros", "" ));
  ASSERT_TRUE(RTT::ComponentLoader::Instance()->import("rtt_rosnode", "" ));
  ASSERT_TRUE(RTT::ComponentLoader::Instance()->import("rtt_roscomm", "" ));
  ASSERT_TRUE(RTT::ComponentLoader::Instance()->import("rtt_std_msgs", "" ));

  RTT::OutputPort<std::vector<double> > out("out");
  RTT::InputPort<std::vector<double> > in("in");

  // Create an out-of-band connection with ROS transport (a publisher/subscriber pair)
  std::string topic = ros::names::resolve("~array_talker");
  EXPECT_TRUE(out.connectTo(&in, rtt_roscomm::topicLatched(topic)));
//  EXPECT_TRUE(out.createStream(rtt_roscomm::topicLatched(topic)));
//  EXPECT_TRUE(in.createStream(rtt_roscomm::topic(topic)));

  // publish and latch one sample
  static const double sample_array[] = { 1., 2., 3., 4., 5. };
  std::vector<double> sample(sample_array, sample_array + sizeof(sample_array)/sizeof(sample_array[0]));
  out.write(sample);
  usleep(1000000);

  // read sample through input port
  std::vector<double> received;
  EXPECT_EQ(RTT::NewData, in.read(received) );
  EXPECT_EQ(sample.size(), received.size());
  if (sample.size() == received.size()) {
    EXPECT_TRUE(std::equal(received.begin(), received.end(), sample.begin()));
  }

  // Close connection
  out.disconnect();
  in.disconnect();
  EXPECT_FALSE(out.connected());
  EXPECT_FALSE(in.connected());
}

static int callback_called = 0;
bool callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ++callback_called;
  return true;
}

TEST(TransportTest, ServiceServerTest)
{
  std::string service = ros::names::resolve("~empty");

  // Import plugins
  ASSERT_TRUE(RTT::ComponentLoader::Instance()->import("rtt_rosnode", "" ));
  ASSERT_TRUE(RTT::ComponentLoader::Instance()->import("rtt_roscomm", "" ));
  ASSERT_TRUE(RTT::ComponentLoader::Instance()->import("rtt_std_srvs", "" ));

  // Create a TaskContext
  RTT::TaskContext *tc = new RTT::TaskContext("TaskContext");
  tc->addOperation("empty", &callback);

  // Load the rosservice service
  boost::weak_ptr<rtt_rosservice::ROSService> rosservice;
  rosservice = tc->getProvider<rtt_rosservice::ROSService>("rosservice");
  ASSERT_FALSE(rosservice.expired());

  // Create a service server
  EXPECT_TRUE(rosservice.lock()->connect("empty", service, "std_srvs/Empty"));

  // Check that the service server has been successfully registered:
  EXPECT_TRUE(ros::ServiceManager::instance()->lookupServicePublication(service).get());

  // Create a service client
  RTT::OperationCaller<bool(std_srvs::Empty::Request&, std_srvs::Empty::Response&)> service_caller("empty");
  tc->requires()->addOperationCaller(service_caller);
  EXPECT_TRUE(rosservice.lock()->connect("empty", service, "std_srvs/Empty"));
  EXPECT_TRUE(service_caller.ready());

  // Call the service
  EXPECT_EQ(0, callback_called);
  std_srvs::Empty empty;
  EXPECT_TRUE(service_caller(empty.request, empty.response));
  EXPECT_EQ(1, callback_called);

  // Disconnect the service
  EXPECT_TRUE(rosservice.lock()->disconnect(service));

  // Check that the service server has been destroyed
  EXPECT_FALSE(ros::ServiceManager::instance()->lookupServicePublication(service));

  // Destroy the TaskContext
  delete tc;
  EXPECT_TRUE(rosservice.expired());

  // Check that the service server has been destroyed (again)
  EXPECT_FALSE(ros::ServiceManager::instance()->lookupServicePublication(service));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  // Initialize Orocos
  __os_init(argc, argv);

  RTT::Logger::log().setStdStream(std::cerr);
  RTT::Logger::log().mayLogStdOut(true);
  RTT::Logger::log().setLogLevel(RTT::Logger::Debug);

  return RUN_ALL_TESTS();
}

