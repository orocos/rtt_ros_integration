/** Copyright (c) 2013, Jonathan Bohren, all rights reserved.
 * This software is released under the BSD 3-clause license, for the details of
 * this license, please see LICENSE.txt at the root of this repository.
 */

#include <rtt/os/startstop.h>
#include <rtt/Logger.hpp>
#include <rtt/deployment/ComponentLoader.hpp>

#include <rtt_roscomm/rtt_rostopic.h>
#include <std_msgs/typekit/String.h>

#include <gtest/gtest.h>

TEST(TransportTest, OutOfBandTest)
{
  // Import plugins
  EXPECT_TRUE(RTT::ComponentLoader::Instance()->import("rtt_std_msgs", "" ));
  EXPECT_TRUE(RTT::ComponentLoader::Instance()->import("rtt_rosnode", "" ));

  RTT::OutputPort<std_msgs::String> out("out");
  RTT::InputPort<std_msgs::String> in("in");

  // Create an out-of-band connection with ROS transport (a publisher/subscriber pair)
  EXPECT_TRUE(out.connectTo(&in, rtt_roscomm::topicLatched("~talker")));

  std_msgs::String sample;
  sample.data = "Hello world!";
  out.write(sample);

  usleep(1000000);

  sample.data.clear();
  EXPECT_EQ(RTT::NewData, in.read(sample) );
  EXPECT_EQ("Hello world!", sample.data);
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

