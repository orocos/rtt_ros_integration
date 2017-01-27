/***************************************************************************
  tag: Ruben Smits  Tue Nov 16 09:18:49 CET 2010  ros_msg_transporter.hpp

                        ros_msg_transporter.hpp -  description
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


// Copyright  (C)  2010  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


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
