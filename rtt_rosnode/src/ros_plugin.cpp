/*
 * (C) 2010, Ruben Smits, ruben.smits@mech.kuleuven.be
 * Department of Mechanical Engineering,
 * Katholieke Universiteit Leuven, Belgium.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <rtt/plugin/Plugin.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Activity.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <rtt/Logger.hpp>
#include <rtt/os/startstop.h>
#include <ros/ros.h>

using namespace RTT;

static void loadROSService()
{
  RTT::Service::shared_ptr ros = RTT::internal::GlobalService::Instance()->provides("ros");

  ros->addOperation("getNodeName", &ros::this_node::getName)
	  .doc("Return full name of ROS node.");
  ros->addOperation("getNamespace", &ros::this_node::getNamespace)
	  .doc("Return ROS node namespace.");
}

extern "C" {
  bool loadRTTPlugin(RTT::TaskContext* c){

    // Initialize ROS if necessary
    if (!ros::isInitialized()) {
      log(Info) << "Initializing ROS node in Orocos plugin..." << endlog();

      int argc = __os_main_argc();
      char ** argv = __os_main_argv();

      // copy the argv array of C strings into a std::vector<char *>
      // Rationale: ros::init(int &argc, char **argv) removes some of the
      // command line arguments and rearranges the remaining ones in the argv
      // vector.
      // See https://github.com/orocos/rtt_ros_integration/issues/54
      std::vector<char *> argvv(argv, argv + argc);
      assert(argvv.size() == argc);
      ros::init(argc, argvv.data(), "rtt", ros::init_options::AnonymousName);
      argvv.resize(argc);

      if (ros::master::check()) {
        ros::start();
      } else {
        log(Warning) << "'roscore' is not running: no ROS functions will be available." << endlog();
        ros::shutdown();
        return true;
      }

	  // Register new operations in global ros Service
	  loadROSService();
    }

    // Defaults the number of threads to the number of CPUs available on the machine
    int thread_count = 0;
    // get number of spinners from parameter server, if available
    ros::param::get("~spinner_threads", thread_count);

    // Create an asynchronous spinner to handle the default callback queue
    static ros::AsyncSpinner spinner(thread_count); // Use thread_count threads

    // TODO: Check spinner.canStart() to suppress errors / warnings once it's incorporated into ROS
    spinner.start();
    log(Info) << "ROS node spinner started (" << thread_count << " " << (thread_count > 1 ? "threads" : "thread") << ")." << endlog();

    return true;
  }

  std::string getRTTPluginName () {
    return "rosnode";
  }

  std::string getRTTTargetName () {
    return OROCOS_TARGET_NAME;
  }
}
