/** Copyright (c) 2013, Jonathan Bohren, all rights reserved.
 * This software is released under the BSD 3-clause license, for the details of
 * this license, please see LICENSE.txt at the root of this repository.
 */

#include <rtt/os/startstop.h>
#include <rtt/Logger.hpp>
#include <rtt/deployment/ComponentLoader.hpp>

#include <rtt_roscomm/rtt_rostopic.h>
#include <std_msgs/typekit/String.h>

#include <ros/names.h>
#include <ros/this_node.h>

#include <gtest/gtest.h>

#include <algorithm>

TEST(TransportTest, OutOfBandTest)
{
  ros::V_string advertised_topics, subscribed_topics;

  // Import plugins
  EXPECT_TRUE(RTT::ComponentLoader::Instance()->import("rtt_std_msgs", "" ));
  EXPECT_TRUE(RTT::ComponentLoader::Instance()->import("rtt_rosnode", "" ));

  RTT::OutputPort<std_msgs::String> out("out");
  RTT::InputPort<std_msgs::String> in("in");

  // Create an out-of-band connection with ROS transport (a publisher/subscriber pair)
  std::string topic = ros::names::resolve("~talker");
  EXPECT_TRUE(out.connectTo(&in, rtt_roscomm::topicLatched(topic)));

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
  sample.data.clear();
  EXPECT_EQ(RTT::NewData, in.read(sample) );
  EXPECT_EQ("Hello world!", sample.data);

  // Close connection
  out.disconnect();
  EXPECT_FALSE(out.connected());
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

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  // Initialize Orocos
  __os_init(argc, argv);

  RTT::Logger::log().setStdStream(std::cerr);
  RTT::Logger::log().mayLogStdOut(true);
  RTT::Logger::log().setLogLevel(RTT::Logger::Debug);

  return RUN_ALL_TESTS();
}

