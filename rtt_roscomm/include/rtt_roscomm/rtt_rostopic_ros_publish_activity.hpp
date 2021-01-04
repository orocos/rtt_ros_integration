/*
 * (C) 2010 Ruben Smits, ruben.smits@mech.kuleuven.be
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

#ifndef __RTT_ROSCOMM_ROS_MSG_ACTIVITY_HPP_
#define __RTT_ROSCOMM_ROS_MSG_ACTIVITY_HPP_

#include <rtt/Activity.hpp>
#include <rtt/os/Mutex.hpp>
#include <rtt/os/MutexLock.hpp>
#include <boost/shared_ptr.hpp>
#include <rtt/Logger.hpp>

#include <ros/ros.h>

#include <set>

namespace rtt_roscomm{

  /**
   * The interface a channel element must implement in
   * order to publish data to a ROS topic.
   */
  struct RosPublisher
  {
  public:
    /**
     * Publish all data in the channel to a ROS topic.
     */
    virtual void publish()=0;
  };


  /**
   * A process wide thread that handles all publishing of
   * ROS topics of the current process.
   * There is no strong reason why only one publisher should
   * exist, in later implementations, one publisher thread per
   * channel may exist as well. See the usage recommendations
   * for Instance()
   */
  class RosPublishActivity : public RTT::Activity
  {
  public:
    typedef boost::shared_ptr<RosPublishActivity> shared_ptr;
  private:
    typedef boost::weak_ptr<RosPublishActivity> weak_ptr;
    //! This pointer may not be refcounted since it would prevent cleanup.
    static weak_ptr ros_pub_act;

    //! A set keeping track of all publishers in the current
    //! process. It must be guarded by the mutex since
    //! insertion/removal happens concurrently.
    typedef std::set< RosPublisher* > Publishers;
    typedef Publishers::iterator iterator;
    Publishers publishers;
    RTT::os::Mutex publishers_lock;

    RosPublishActivity( const std::string& name);

    void loop();

  public:
    /**
     * Returns the single instance of the RosPublisher. This function
     * is not thread-safe when it creates the RosPublisher object.
     * Therefor, it is advised to cache the object which Intance() returns
     * such that, in the unlikely event that two publishers exist,
     * you consistently keep using the same instance, which is fine then.
     */
    static shared_ptr Instance();

    void addPublisher(RosPublisher* pub);
    void removePublisher(RosPublisher* pub);

    ~RosPublishActivity();

  };//class
}//namespace
#endif
