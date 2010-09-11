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
    typedef boost::shared_ptr<RosPublishActivity> shared_ptr;
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

