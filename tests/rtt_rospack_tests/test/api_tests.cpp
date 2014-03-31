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
#include <rtt/internal/GlobalService.hpp>

#include <ros/package.h>

#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

#include <gtest/gtest.h>

boost::shared_ptr<OCL::DeploymentComponent> deployer;
boost::shared_ptr<RTT::Scripting> scripting_service;
RTT::Service::shared_ptr global_service;

TEST(BasicTest, Import) 
{
  // Import rtt_ros plugin
  RTT::ComponentLoader::Instance()->import("rtt_ros", "" );
}

TEST(BasicTest, PackageFinding) 
{
  // Import rtt_ros plugin
  EXPECT_TRUE(scripting_service->eval("ros.import(\"rtt_rospack_tests\")"));

  RTT::OperationCaller<std::string(const std::string&)> rpf = global_service->provides("ros")->getOperation("find");
  std::string roscpp_dir = ros::package::getPath("roscpp");
  EXPECT_EQ(roscpp_dir,rpf("roscpp"));

  std::string rtt_rospack_tests_dir = ros::package::getPath("rtt_rospack_tests");
  EXPECT_EQ(rtt_rospack_tests_dir,rpf("rtt_rospack_tests"));
}


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  // Initialize Orocos
  __os_init(argc, argv);

  deployer = boost::make_shared<OCL::DeploymentComponent>();
  scripting_service = deployer->getProvider<RTT::Scripting>("scripting");
  global_service = RTT::internal::GlobalService::Instance();

  RTT::Logger::log().setStdStream(std::cerr);
  RTT::Logger::log().mayLogStdOut(true);
  RTT::Logger::log().setLogLevel(RTT::Logger::Debug);
  
  return RUN_ALL_TESTS();
}
