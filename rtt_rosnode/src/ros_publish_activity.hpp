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
#include <rtt/internal/MWSRQueue.hpp>
#include <boost/shared_ptr.hpp>
#include <rtt/Logger.hpp>

#include <ros/ros.h>

namespace ros_integration{
  using namespace RTT;
  
  struct RosPublisher
  {
  public:
    virtual void publish()=0;
  };
  
  
  class RosPublishActivity : public RTT::Activity{
  public:
    typedef boost::shared_ptr<RosPublishActivity> shared_ptr;
  private:
    static shared_ptr ros_pub_act;
    
    internal::MWSRQueue<RosPublisher*> pending_queue;

    RosPublishActivity( const std::string& name)
      : Activity(0, name),pending_queue(128)
    {
      Logger::In in("RosPublishActivity");
      log(Debug)<<"Creating RosPublishActivity"<<endlog();
    }
    
    void loop(){
        //Logger::In in("RosPublishActivity");
        //log(Debug)<<"execute"<<endlog();
        RosPublisher* chan;
        while(pending_queue.dequeue(chan))
            chan->publish();
    }
    
  public:
      
      static RosPublishActivity::shared_ptr Instance() {
          if ( !ros_pub_act ) {
              ros_pub_act.reset(new RosPublishActivity("RosPublishActivity"));
              ros_pub_act->start();
          }
          return ros_pub_act;
      }
      
      bool requestPublish(RosPublisher* chan){
          //Logger::In in("RosPublishActivity");
          //log(Debug)<<"Requesting publish"<<endlog();
          bool retval = pending_queue.enqueue(chan);
          return retval&&this->trigger();
      }
      ~RosPublishActivity() {
          Logger::In in("RosPublishActivity");
          log(Info) << "RosPublishActivity cleans up: no more work."<<endlog();
          stop();
      }
      
  };//class
}//namespace
#endif

