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

using namespace RTT;

class HelloRobot : public RTT::TaskContext{
private:  
  InputPort<std_msgs::Float64> inport;
  OutputPort<std_msgs::Float64> outport;

  InputPort<std_msgs::String> sinport;
  OutputPort<std_msgs::String> soutport;
  
  std::string prop_answer;
  double prop_counter_step;

  double counter;
  
public:
  HelloRobot(const std::string& name):
    TaskContext(name),
    inport("float_in"),outport("float_out"),
    sinport("string_in"),soutport("string_out","Hello Robot"),
    prop_answer("Hello Robot"),prop_counter_step(0.01)
  {
    this->addEventPort(inport).doc("Receiving a message here will wake up this component.");
    this->addPort(outport).doc("Sends out 'answer'.");
    this->addEventPort(sinport).doc("Receiving a message here will wake up this component.");
    this->addPort(soutport).doc("Sends out a counter value based on 'counter_step'.");
    
    this->addProperty("answer",prop_answer).doc("The text being sent out on 'string_out'.");
    this->addProperty("counter_step",prop_counter_step).doc("The increment for each new sample on 'float_out'");
    
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
  }
};
ORO_CREATE_COMPONENT(HelloRobot)
