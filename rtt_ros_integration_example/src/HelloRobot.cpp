/***************************************************************************
  tag: Ruben Smits  Tue Nov 16 09:21:07 CET 2010  HelloRobot.cpp

                        HelloRobot.cpp -  description
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


#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <rtt/Component.hpp>
#include <std_srvs/Empty.h>
#include <ros/ros.h>

using namespace RTT;

class HelloRobot : public RTT::TaskContext{
private:  
  InputPort<std_msgs::Float64> inport;
  OutputPort<std_msgs::Float64> outport;

  InputPort<std_msgs::String> sinport;
  OutputPort<std_msgs::String> soutport;
  
  std::string prop_answer;
  double prop_counter_step;
  double prop_service_call_counter;

  double counter;

  OperationCaller<bool(std_srvs::Empty::Request&, std_srvs::Empty::Response&)> updated;

public:
  HelloRobot(const std::string& name):
    TaskContext(name),
    inport("float_in"),outport("float_out"),
    sinport("string_in"),soutport("string_out","Hello Robot"),
    prop_answer("Hello Robot"),prop_counter_step(0.01),prop_service_call_counter(0.0),
    updated("updated")
  {
    this->addEventPort(inport).doc("Receiving a message here will wake up this component.");
    this->addPort(outport).doc("Sends out 'answer'.");
    this->addEventPort(sinport).doc("Receiving a message here will wake up this component.");
    this->addPort(soutport).doc("Sends out a counter value based on 'counter_step'.");
    
    this->addProperty("answer",prop_answer).doc("The text being sent out on 'string_out'.");
    this->addProperty("counter_step",prop_counter_step).doc("The increment for each new sample on 'float_out'");
    this->addProperty("service_call_counter",prop_service_call_counter).doc("The number of times the incrememt operation has been called.");

    this->provides()->addOperation("increment",&HelloRobot::increment,this,RTT::OwnThread);
    this->requires()->addOperationCaller(updated);
    
    counter=0.0;
  }
  ~HelloRobot(){}
private:
  void updateHook(){
    std_msgs::Float64 fdata;
    std_msgs::String sdata;
    if(NewData==inport.read(fdata)){
      log(Info)<<"Float in: "<<fdata<<endlog();
      counter=fdata.data;
    }
    if(NewData==sinport.read(sdata))
      log(Info)<<"String in: "<<sdata<<endlog();
    counter+=prop_counter_step;
    fdata.data=counter;
    outport.write(fdata);
    sdata.data=prop_answer;
    soutport.write(sdata);

    if(updated.ready()) {
      std_srvs::Empty::Request req;
      std_srvs::Empty::Response res;
      updated(req,res);
    }
  }

  bool increment(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp) {
    prop_service_call_counter++;
    return true;
  }
};
ORO_CREATE_COMPONENT(HelloRobot)
