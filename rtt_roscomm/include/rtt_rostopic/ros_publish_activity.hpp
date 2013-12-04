/***************************************************************************
  tag: Ruben Smits  Tue Nov 16 09:18:49 CET 2010  ros_publish_activity.hpp

                        ros_publish_activity.hpp -  description
                           -------------------
    begin                : Tue November 16 2010
    copyright            : (C) 2010 Ruben Smits
    email                : first.last@mech.kuleuven.be

 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/


#ifndef _ROS_MSG_ACTIVITY_HPP_
#define _ROS_MSG_ACTIVITY_HPP_

#include <rtt/Activity.hpp>
#include <rtt/os/Mutex.hpp>
#include <rtt/os/MutexLock.hpp>
#include <boost/shared_ptr.hpp>
#include <rtt/Logger.hpp>

#include <ros/ros.h>

#include <set>

namespace ros_integration{

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
  class RosPublishActivity : public RTT::Activity {
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

