/*
 * (C) 2010, Ruben Smits, ruben.smits@mech.kuleuven.be
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

#include <rtt_roscomm/rtt_rostopic_ros_publish_activity.hpp>

namespace rtt_roscomm {

    using namespace RTT;

    RosPublishActivity::weak_ptr RosPublishActivity::ros_pub_act;

    RosPublishActivity::RosPublishActivity( const std::string& name)
      : Activity(ORO_SCHED_OTHER, RTT::os::LowestPriority, 0.0, 0, name)
    {
      Logger::In in("RosPublishActivity");
      log(Debug)<<"Creating RosPublishActivity"<<endlog();
    }

    void RosPublishActivity::loop(){
      os::MutexLock lock(publishers_lock);
      for(iterator it = publishers.begin(); it != publishers.end(); ++it) {
        (*it)->publish();
      }
    }

    RosPublishActivity::shared_ptr RosPublishActivity::Instance() {
      shared_ptr ret = ros_pub_act.lock();
      if ( !ret ) {
        ret.reset(new RosPublishActivity("RosPublishActivity"));
        ros_pub_act = ret;
        ret->start();
      }
      return ret;
    }

    void RosPublishActivity::addPublisher(RosPublisher* pub) {
      os::MutexLock lock(publishers_lock);
      publishers.insert(pub);
    }

    void RosPublishActivity::removePublisher(RosPublisher* pub) {
      os::MutexLock lock(publishers_lock);
      publishers.erase(pub);
    }

    RosPublishActivity::~RosPublishActivity() {
      Logger::In in("RosPublishActivity");
      log(Info) << "RosPublishActivity cleans up: no more work."<<endlog();
      stop();
    }

}
