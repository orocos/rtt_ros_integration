/** Copyright (c) 2013, Jonathan Bohren, all rights reserved. 
 * This software is released under the BSD 3-clause license, for the details of
 * this license, please see LICENSE.txt at the root of this repository. 
 */

#include <string>
#include <vector>
#include <iterator>

#include <rtt/os/startstop.h>

#include <ocl/DeploymentComponent.hpp>
#include <ocl/TaskBrowser.hpp>
#include <ocl/LoggingService.hpp>
#include <rtt/Logger.hpp>
#include <rtt/deployment/ComponentLoader.hpp>
#include <rtt/scripting/Scripting.hpp>

#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

#include <gtest/gtest.h>

boost::shared_ptr<OCL::DeploymentComponent> deployer;
boost::shared_ptr<RTT::Scripting> scripting_service;

TEST(BasicTest, Import) 
{
  // Import rtt_ros plugin
  RTT::ComponentLoader::Instance()->import("rtt_ros", "" );
}

TEST(BasicTest, ImportChaining) 
{
  // Import rtt_ros plugin
  EXPECT_TRUE(scripting_service->eval("ros.import(\"rtt_ros_tests\")"));
  EXPECT_TRUE(scripting_service->eval("rospack.find(\"rtt_ros\")"));
}


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  // Initialize Orocos
  __os_init(argc, argv);

  deployer = boost::make_shared<OCL::DeploymentComponent>();
  scripting_service = deployer->getProvider<RTT::Scripting>("scripting");

  RTT::Logger::log().setStdStream(std::cerr);
  RTT::Logger::log().mayLogStdOut(true);
  RTT::Logger::log().setLogLevel(RTT::Logger::Debug);
  
  return RUN_ALL_TESTS();
}
