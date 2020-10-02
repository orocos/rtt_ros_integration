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

#ifndef __RTT_ROSCOMM_ROS_MSG_TRANSPORTER_HPP_
#define __RTT_ROSCOMM_ROS_MSG_TRANSPORTER_HPP_

#include <rtt/rtt-config.h>
#include <rtt/types/TypeTransporter.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/internal/ConnFactory.hpp>
#include <ros/ros.h>

#include <rtt_roscomm/rtt_rostopic_ros_publish_activity.hpp>

#ifndef RTT_VERSION_GTE
  #define RTT_VERSION_GTE(major,minor,patch) \
      ((RTT_VERSION_MAJOR > major) || (RTT_VERSION_MAJOR == major && \
       (RTT_VERSION_MINOR > minor) || (RTT_VERSION_MINOR == minor && \
       (RTT_VERSION_PATCH >= patch))))
#endif

namespace rtt_roscomm {

  /**
   * An adapter that allows to overwrite the corresponding ROS type for a
   * given Orocos type by specialization.
   */
  template <typename T>
  struct RosMessageAdapter
  {
    typedef T OrocosType;
    typedef T RosType;
    static const RosType &toRos(const OrocosType &t) { return t; }
    static const OrocosType &fromRos(const RosType &t) { return t; }
  };

  /**
   * A ChannelElement implementation to publish data over a ros topic
   */
  template<typename T>
  class RosPubChannelElement: public RTT::base::ChannelElement<T>, public RosPublisher
  {
    typedef RosMessageAdapter<T> adapter;
    typedef typename adapter::RosType RosType;

    char hostname[1024];
    std::string topicname;
    ros::NodeHandle ros_node;
    ros::NodeHandle ros_node_private;
    ros::Publisher ros_pub;
      //! We must cache the RosPublishActivity object.
    RosPublishActivity::shared_ptr act;

    typename RTT::base::ChannelElement<T>::value_t sample;

  public:

    /**
     * Contructor of to create ROS publisher ChannelElement, it will
     * create a topic from the name given by the policy.name_id, if
     * this is empty a default is created as hostname/componentname/portname/pid
     *
     * @param port port for which we will create a the ROS publisher
     * @param policy connection policy containing the topic name and buffer size
     *
     * @return ChannelElement that will publish data to topics
     */
    RosPubChannelElement(RTT::base::PortInterface* port, const RTT::ConnPolicy& policy) :
      ros_node(),
      ros_node_private("~")
    {
      if ( policy.name_id.empty() ){
        std::stringstream namestr;
        gethostname(hostname, sizeof(hostname));

        if (port->getInterface() && port->getInterface()->getOwner()) {
          namestr << hostname<<'/' << port->getInterface()->getOwner()->getName()
            << '/' << port->getName() << '/'<<this << '/' << getpid();
        } else {
          namestr << hostname<<'/' << port->getName() << '/'<<this << '/' << getpid();
        }
        policy.name_id = namestr.str();
      }
      topicname=policy.name_id;
      RTT::Logger::In in(topicname);

      if (port->getInterface() && port->getInterface()->getOwner()) {
        RTT::log(RTT::Debug)<<"Creating ROS publisher for port "<<port->getInterface()->getOwner()->getName()<<"."<<port->getName()<<" on topic "<<policy.name_id<<RTT::endlog();
      } else {
        RTT::log(RTT::Debug)<<"Creating ROS publisher for port "<<port->getName()<<" on topic "<<policy.name_id<<RTT::endlog();
      }

      // Handle private names
      if(topicname.length() > 1 && topicname.at(0) == '~') {
        ros_pub = ros_node_private.advertise<RosType>(policy.name_id.substr(1), policy.size > 0 ? policy.size : 1, policy.init); // minimum 1
      } else {
        ros_pub = ros_node.advertise<RosType>(policy.name_id, policy.size > 0 ? policy.size : 1, policy.init); // minimum 1
      }
      act = RosPublishActivity::Instance();
      act->addPublisher( this );
    }

    ~RosPubChannelElement() {
      RTT::Logger::In in(topicname);
//      RTT::log(RTT::Debug) << "Destroying RosPubChannelElement" << RTT::endlog();
      act->removePublisher( this );
    }

    /**
     * Function to see if the ChannelElement is ready to receive inputs
     *
     * @return always true in our case
     */
    virtual bool inputReady() {
      return true;
    }

    virtual bool isRemoteElement() const {
        return true;
    }

    virtual std::string getElementName() const {
        return "RosPubChannelElement";
    }

    virtual std::string getRemoteURI() const {
        return ros_pub.getTopic();
    }

    /**
     * Create a data sample, this could be used to allocate the necessary memory
     *
     * @param sample
     *
     * @return always true/WriteSuccess
     */
#if RTT_VERSION_GTE(2,8,99)
    virtual RTT::WriteStatus data_sample(typename RTT::base::ChannelElement<T>::param_t sample)
    {
      this->sample = sample;
      return RTT::WriteSuccess;
    }
#else
    virtual bool data_sample(typename RTT::base::ChannelElement<T>::param_t sample)
    {
      this->sample = sample;
      return true;
    }
#endif

    /**
     * signal from the port that new data is availabe to publish
     *
     * @return true if publishing succeeded
     */
    bool signal(){
      //RTT::Logger::In in(topicname);
      //RTT::log(RTT::Debug) << "Requesting publish" << RTT::endlog();
      return act->trigger();
    }

    void publish(){
      // this read should always succeed since signal() means 'data available in a data element'.
      typename RTT::base::ChannelElement<T>::shared_ptr input = this->getInput();
      while( input && (input->read(sample,false) == RTT::NewData) )
        write(sample);
    }

#if RTT_VERSION_GTE(2,8,99)
    RTT::WriteStatus write(typename RTT::base::ChannelElement<T>::param_t sample)
#else
    bool write(typename RTT::base::ChannelElement<T>::param_t sample)
#endif
    {
      ros_pub.publish(adapter::toRos(sample));
#if RTT_VERSION_GTE(2,8,99)
      return RTT::WriteSuccess;
#else
      return true;
#endif
    }

  };

  /**
   * A ChannelElement implementation to subscribe to data over a ros topic
   */
  template<typename T>
  class RosSubChannelElement: public RTT::base::ChannelElement<T>
  {
    typedef RosMessageAdapter<T> adapter;
    typedef typename adapter::RosType RosType;

    std::string topicname;
    ros::NodeHandle ros_node;
    ros::NodeHandle ros_node_private;
    ros::Subscriber ros_sub;

  public:
    /**
     * Contructor of to create ROS subscriber ChannelElement, it will
     * subscribe to a topic with the name given by the policy.name_id
     *
     * @param port port for which we will create a the ROS publisher
     * @param policy connection policy containing the topic name and buffer size
     *
     * @return ChannelElement that will publish data to topics
     */
    RosSubChannelElement(RTT::base::PortInterface* port, const RTT::ConnPolicy& policy) :
      ros_node(),
      ros_node_private("~")
    {
      topicname=policy.name_id;
      RTT::Logger::In in(topicname);
      if (port->getInterface() && port->getInterface()->getOwner()) {
        RTT::log(RTT::Debug)<<"Creating ROS subscriber for port "<<port->getInterface()->getOwner()->getName()<<"."<<port->getName()<<" on topic "<<policy.name_id<<RTT::endlog();
      } else {
        RTT::log(RTT::Debug)<<"Creating ROS subscriber for port "<<port->getName()<<" on topic "<<policy.name_id<<RTT::endlog();
      }
      if(topicname.length() > 1 && topicname.at(0) == '~') {
        ros_sub = ros_node_private.subscribe(policy.name_id.substr(1), policy.size > 0 ? policy.size : 1, &RosSubChannelElement::newData, this); // minimum queue_size 1
      } else {
        ros_sub = ros_node.subscribe(policy.name_id, policy.size > 0 ? policy.size : 1, &RosSubChannelElement::newData, this); // minimum queue_size 1
      }
    }

    ~RosSubChannelElement() {
      RTT::Logger::In in(topicname);
//      RTT::log(RTT::Debug)<<"Destroying RosSubChannelElement"<<RTT::endlog();
    }

    virtual bool inputReady() {
      return true;
    }

    virtual bool isRemoteElement() const {
        return true;
    }

    virtual std::string getElementName() const {
        return "RosSubChannelElement";
    }

    virtual std::string getRemoteURI() const {
        return ros_sub.getTopic();
    }

    /**
     * Callback function for the ROS subscriber, it will trigger the ChannelElement's signal function
     *
     * @param msg The received message
     */
    void newData(const RosType& msg){
      typename RTT::base::ChannelElement<T>::shared_ptr output = this->getOutput();
      if (output)
          output->write(adapter::fromRos(msg));
    }
  };

  template <class T>
  class RosMsgTransporter : public RTT::types::TypeTransporter
  {
    virtual RTT::base::ChannelElementBase::shared_ptr createStream (RTT::base::PortInterface *port, const RTT::ConnPolicy &policy, bool is_sender) const{
      RTT::base::ChannelElementBase::shared_ptr channel;

      // Pull semantics are not supported by the ROS message transport.
      if (policy.pull) {
          RTT::log(RTT::Error) << "Pull connections are not supported by the ROS message transport." << RTT::endlog();
          return RTT::base::ChannelElementBase::shared_ptr();
      }

      // Check if this node is initialized
      if (!ros::ok()) {
          RTT::log(RTT::Error) << "Cannot create ROS message transport because the node is not initialized or already shutting down. Did you import package rtt_rosnode before?" << RTT::endlog();
          return RTT::base::ChannelElementBase::shared_ptr();
      }

      if (is_sender){
        channel = new RosPubChannelElement<T>(port, policy);

        if (policy.type == RTT::ConnPolicy::UNBUFFERED){
          RTT::log(RTT::Debug) << "Creating unbuffered publisher connection for port " << port->getName() << ". This may not be real-time safe!" << RTT::endlog();
          return channel;
        }

        RTT::base::ChannelElementBase::shared_ptr buf = RTT::internal::ConnFactory::buildDataStorage<T>(policy);
        if (!buf) return RTT::base::ChannelElementBase::shared_ptr();
#if RTT_VERSION_GTE(2,8,99)
        buf->connectTo(channel);
#else
        buf->setOutput(channel);
#endif
        return buf;

      } else {
        channel = new RosSubChannelElement<T>(port, policy);

#if !RTT_VERSION_GTE(2,8,99)
        RTT::base::ChannelElementBase::shared_ptr buf = RTT::internal::ConnFactory::buildDataStorage<T>(policy);
        if (!buf) return RTT::base::ChannelElementBase::shared_ptr();
        channel->setOutput(buf);
#endif
      }

      return channel;
    }
  };
}
#endif
