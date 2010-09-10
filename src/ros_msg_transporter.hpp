#ifndef _ROS_MSG_TRANSPORTER_HPP_
#define _ROS_MSG_TRANSPORTER_HPP_

#include <rtt/types/TypeTransporter.hpp>
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/internal/ConnFactory.hpp>
#include <ros/ros.h>


#include "ros_publish_activity.hpp"

namespace ros_integration {

  using namespace RTT;
  /**
   * A ChannelElement implementation to publish data over a ros topic
   * 
   */
  template<typename T>
  class RosPubChannelElement: public base::ChannelElement<T>,public RosPublisher
  {
    char hostname[1024];
    std::string topicname;
    ros::NodeHandle ros_node;
    ros::Publisher ros_pub;
    RosPublishActivity::shared_ptr act;

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
    RosPubChannelElement(base::PortInterface* port,const ConnPolicy& policy)
    {
      if ( policy.name_id.empty() ){
	std::stringstream namestr;
	gethostname(hostname, sizeof(hostname));
	
	namestr << hostname<<'/' << port->getInterface()->getOwner()->getName()
		<< '/' << port->getName() << '/'<<this << '/' << getpid();
	policy.name_id = namestr.str();
      }
      topicname=policy.name_id;
      Logger::In in(topicname);
      log(Debug)<<"Creating ROS publisher for port "<<port->getInterface()->getOwner()->getName()<<"."<<port->getName()<<" on topic "<<policy.name_id<<endlog();

      ros_pub = ros_node.advertise<T>(policy.name_id, policy.size);
      act = RosPublishActivity::Instance();
    }

    ~RosPubChannelElement() {
      Logger::In in(topicname);
      log(Debug)<<"Destroying RosPubChannelElement"<<endlog();
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
     * Create a data sample, this could be used to allocate the necessary memory, it is not needed in our case
     * 
     * @param sample 
     * 
     * @return always true
     */
    virtual bool data_sample(typename base::ChannelElement<T>::param_t sample)
    {
      return true;
    }

    /** 
     * signal from the port that new data is availabe to publish
     * 
     * @return true if publishing succeeded
     */
    bool signal(){
      Logger::In in(topicname);
      log(Debug)<<"Requesting publish"<<endlog();
      return act->requestPublish(this);
    }
    
    void publish(){
      typename base::ChannelElement<T>::value_t sample; // XXX: real-time !
      // this read should always succeed since signal() means 'data available in a data element'.
      base::ChannelElement<T>* input = dynamic_cast< base::ChannelElement<T>* >(this->input);
      if( input->read(sample) == NewData )
	ros_pub.publish(sample);
    }
    
  };

  template<typename T>
  class RosSubChannelElement: public base::ChannelElement<T>
  {
    ros::NodeHandle ros_node;
    ros::Subscriber ros_sub;
    bool newdata,init;
    T m_msg;
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
    RosSubChannelElement(base::PortInterface* port, const ConnPolicy& policy):
    newdata(false),init(false)
    {
      log(Debug)<<"Creating ROS subscriber for port "<<port->getInterface()->getOwner()->getName()<<"."<<port->getName()<<" on topic "<<policy.name_id<<endlog();
      ros_sub=ros_node.subscribe(policy.name_id,policy.size,&RosSubChannelElement::newData,this);
      this->ref();
    }

    ~RosSubChannelElement() {
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
      m_msg = msg;
      newdata=true;
      init=true;
      this->signal();
    }

    /** 
     * function that the port will use to get the received data
     * 
     * @param sample object to put the received data into
     * 
     * @return FlowStatus for the port
     */
    FlowStatus read(typename base::ChannelElement<T>::reference_t sample)
    {
      if(!init)
	return NoData;
      sample=m_msg;
      
      if(newdata){
	newdata=false;
	return NewData;
      }
      else
	return OldData;
    }
  };
    
  template <class T>
  class RosMsgTransporter : public RTT::types::TypeTransporter{
    virtual base::ChannelElementBase * createStream (base::PortInterface *port, const ConnPolicy &policy, bool is_sender) const{
      if(is_sender){
	base::ChannelElementBase* buf = internal::ConnFactory::buildDataStorage<T>(policy);
	base::ChannelElementBase::shared_ptr tmp = base::ChannelElementBase::shared_ptr(new RosPubChannelElement<T>(port,policy));
	buf->setOutput(tmp);
	return buf;
      }
      else
	return new RosSubChannelElement<T>(port,policy);
      
    }
  };
} 
#endif
