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

#include <rtt/types/TypeTransporter.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/internal/ConnFactory.hpp>
#include <ros/ros.h>

#include <rtt_roscomm/rtt_rostopic_ros_publish_activity.hpp>

namespace rtt_roscomm {

  using namespace RTT;
  /**
   * A ChannelElement implementation to publish data over a ros topic
   */
  template<typename T>
  class RosPubChannelElement: public base::ChannelElement<T>,public RosPublisher
  {
    char hostname[1024];
    std::string topicname;
    ros::NodeHandle ros_node;
    ros::NodeHandle ros_node_private;
    ros::Publisher ros_pub;
      //! We must cache the RosPublishActivity object.
    RosPublishActivity::shared_ptr act;

    typename base::ChannelElement<T>::value_t sample;

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
    RosPubChannelElement(base::PortInterface* port,const ConnPolicy& policy):
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
      Logger::In in(topicname);

      if (port->getInterface() && port->getInterface()->getOwner()) {
        log(Debug)<<"Creating ROS publisher for port "<<port->getInterface()->getOwner()->getName()<<"."<<port->getName()<<" on topic "<<policy.name_id<<endlog();
      } else {
        log(Debug)<<"Creating ROS publisher for port "<<port->getName()<<" on topic "<<policy.name_id<<endlog();
      }

      // Handle private names
      if(topicname.length() > 1 && topicname.at(0) == '~') {
        ros_pub = ros_node_private.advertise<T>(policy.name_id.substr(1), policy.size > 0 ? policy.size : 1, policy.init); // minimum 1
      } else {
        ros_pub = ros_node.advertise<T>(policy.name_id, policy.size > 0 ? policy.size : 1, policy.init); // minimum 1
      }
      act = RosPublishActivity::Instance();
      act->addPublisher( this );
    }

    ~RosPubChannelElement() {
      Logger::In in(topicname);
      log(Debug)<<"Destroying RosPubChannelElement"<<endlog();
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
     * @return always true
     */
    virtual bool data_sample(typename base::ChannelElement<T>::param_t sample)
    {
      this->sample = sample;
      return true;
    }

    /** 
     * signal from the port that new data is availabe to publish
     * 
     * @return true if publishing succeeded
     */
    bool signal(){
      //Logger::In in(topicname);
      //log(Debug)<<"Requesting publish"<<endlog();
      return act->trigger();
    }
    
    void publish(){
      // this read should always succeed since signal() means 'data available in a data element'.
      typename base::ChannelElement<T>::shared_ptr input = this->getInput();
      while( input && (input->read(sample,false) == NewData) )
        write(sample);
    }

    bool write(typename base::ChannelElement<T>::param_t sample)
    {
      ros_pub.publish(sample);
      return true;
    }
    
  };

  /**
   * A ChannelElement implementation to subscribe to data over a ros topic
   */
  template<typename T>
  class RosSubChannelElement: public base::ChannelElement<T>
  {
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
    RosSubChannelElement(base::PortInterface* port, const ConnPolicy& policy) :
      ros_node(),
      ros_node_private("~")
    {
      topicname=policy.name_id;
      Logger::In in(topicname);
      if (port->getInterface() && port->getInterface()->getOwner()) {
        log(Debug)<<"Creating ROS subscriber for port "<<port->getInterface()->getOwner()->getName()<<"."<<port->getName()<<" on topic "<<policy.name_id<<endlog();
      } else {
        log(Debug)<<"Creating ROS subscriber for port "<<port->getName()<<" on topic "<<policy.name_id<<endlog();
      }
      if(topicname.length() > 1 && topicname.at(0) == '~') {
        ros_sub = ros_node_private.subscribe(policy.name_id.substr(1), policy.size > 0 ? policy.size : 1, &RosSubChannelElement::newData, this); // minimum queue_size 1
      } else {
        ros_sub = ros_node.subscribe(policy.name_id, policy.size > 0 ? policy.size : 1, &RosSubChannelElement::newData, this); // minimum queue_size 1
      }
    }

    ~RosSubChannelElement() {
      Logger::In in(topicname);
      log(Debug)<<"Destroying RosSubChannelElement"<<endlog();
    }

    virtual bool inputReady() {
      return true;
    }
    /** 
     * Callback function for the ROS subscriber, it will trigger the ChannelElement's signal function
     * 
     * @param msg The received message
     */
    void newData(const T& msg){
      typename base::ChannelElement<T>::shared_ptr output = this->getOutput();
      if (output)
          output->write(msg);
    }
  };

  template <class T>
  class RosMsgTransporter : public RTT::types::TypeTransporter
  {
    virtual base::ChannelElementBase::shared_ptr createStream (base::PortInterface *port, const ConnPolicy &policy, bool is_sender) const{
      base::ChannelElementBase::shared_ptr buf = internal::ConnFactory::buildDataStorage<T>(policy);
      base::ChannelElementBase::shared_ptr tmp;
      if(is_sender){
        tmp = base::ChannelElementBase::shared_ptr(new RosPubChannelElement<T>(port,policy));
        if (policy.type == RTT::ConnPolicy::UNBUFFERED){
          log(Debug) << "Creating unbuffered publisher connection for port " << port->getName() << ". This may not be real-time safe!" << endlog();
          return tmp;
        }
        if (!buf) return base::ChannelElementBase::shared_ptr();
        buf->setOutput(tmp);
        return buf;
      }
      else {
        if (!buf) return base::ChannelElementBase::shared_ptr();
        tmp = new RosSubChannelElement<T>(port,policy);
        tmp->setOutput(buf);
        return tmp;
      }
    }
  };
} 
#endif
